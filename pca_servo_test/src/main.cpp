// ESP32-S3 firmware: PCA9685 servo calibration + tripod walk.
//
// Hardware
//   I2C SDA=GPIO6, SCL=GPIO5
//   PCA9685 0x40 (UI servos 1–16) and 0x41 A0 bridged (UI 17–32)
//   18 MG996R-class joints; map in servo_map.json / JOINTS[] below
//
// Calibration (NVS namespace "servo_cal")
//   Hip:  ref = mount DEFAULT (model 0°). No min/max stops.
//   Thigh/shin: ref = folded MAX. MIN/MID derived from span and dir.
//   dir = +1 means higher PWM moves toward model MIN.
//   Thigh/shin spans are locked to the shortest available max→min.
//   Snapshot: calibration/servo_cal.json (compiled into calibration_backup.h).
//
// Joint signs: pca_servo_test/JOINT_CONVENTION.md
// Gait: src/walk.cpp (same IK as hexapod_description/view_hexapod.html)
// Web:  /  calibration   /walk  3D preview + deploy

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include "wifi_secrets.h"
#include "walk.h"
#include "gait_preview_html.h"
#include "calibration_backup.h"

static const int SDA_PIN = 6;  // ESP32-S3 I2C data
static const int SCL_PIN = 5;  // ESP32-S3 I2C clock

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
WebServer server(80);
Preferences prefs;

// PCA9685 tick window used for MG996R-class (~0.5–2.5 ms at 50 Hz).
static const uint16_t PULSE_LO = 80;
static const uint16_t PULSE_HI = 520;
static const uint16_t PULSE_DEFAULT = 300;
static const uint8_t SERVO_COUNT = 32;
static const float TICKS_PER_DEG = 400.0f / 180.0f;

// ui is the 1-based number on the 32-channel calibrator page.
// board 0 = ui 1–16 (PCA 0x40 ch 0–15); board 1 = ui 17–32 (PCA 0x41 ch 0–15).
struct JointMap {
  const char *leg;
  const char *joint;
  uint8_t ui;
};
static const JointMap JOINTS[] = {
  {"FL", "hip", 12}, {"FL", "thigh", 8}, {"FL", "shin", 22},
  {"ML", "hip", 11}, {"ML", "thigh", 7}, {"ML", "shin", 21},
  {"RL", "hip", 10}, {"RL", "thigh", 5}, {"RL", "shin", 17},
  {"FR", "hip", 23}, {"FR", "thigh", 19}, {"FR", "shin", 13},
  {"MR", "hip", 29}, {"MR", "thigh", 28}, {"MR", "shin", 9},
  {"RR", "hip", 26}, {"RR", "thigh", 18}, {"RR", "shin", 1},
};
static const uint8_t JOINT_COUNT = sizeof(JOINTS) / sizeof(JOINTS[0]);

static bool isHip(uint8_t ji) { return !strcmp(JOINTS[ji].joint, "hip"); }
static bool isThigh(uint8_t ji) { return !strcmp(JOINTS[ji].joint, "thigh"); }
static bool isShin(uint8_t ji) { return !strcmp(JOINTS[ji].joint, "shin"); }

// Thigh/shin: full travel from MAX stop. Hip: ±45° about mount default.
static float jointHalfOrFullDeg(uint8_t ji) {
  if (isHip(ji)) return 45.0f;
  if (isThigh(ji)) return 102.5f;
  if (isShin(ji)) return 122.4f;
  return 90.0f;
}

static bool pca0Ok = false;
static bool pca1Ok = false;
static uint16_t lastPulse[SERVO_COUNT];

// calRef: hip = mount DEFAULT; thigh/shin = mechanical MAX.
// dir: +1 = PWM increases toward model MIN.
// lockedSpan*: 0 = use degree formula; else shared PWM ticks for that joint type.
static uint16_t calRef[JOINT_COUNT];
static int8_t calDir[JOINT_COUNT];
static uint16_t lockedSpanThigh = 0;
static uint16_t lockedSpanShin = 0;

static uint8_t uiToGlobal(uint8_t ui) { return ui - 1; }
static bool boardOk(uint8_t globalCh) {
  return (globalCh < 16) ? pca0Ok : pca1Ok;
}

static uint16_t idealSpan(uint8_t ji) {
  return (uint16_t)lroundf(jointHalfOrFullDeg(ji) * TICKS_PER_DEG);
}

// Room from this joint's MAX toward MIN before hitting pulse rails.
static uint16_t availableSpan(uint8_t ji) {
  if (!calRef[ji] || isHip(ji)) return 0;
  int32_t room = (calDir[ji] > 0)
                     ? ((int32_t)PULSE_HI - (int32_t)calRef[ji])
                     : ((int32_t)calRef[ji] - (int32_t)PULSE_LO);
  if (room < 0) room = 0;
  uint16_t ideal = idealSpan(ji);
  return (uint16_t)((uint32_t)room < ideal ? room : ideal);
}

static uint16_t pulseDelta(uint8_t ji) {
  if (isThigh(ji) && lockedSpanThigh) return lockedSpanThigh;
  if (isShin(ji) && lockedSpanShin) return lockedSpanShin;
  return idealSpan(ji);
}

static uint16_t clampPulse(int32_t p) {
  return (uint16_t)constrain(p, (int32_t)PULSE_LO, (int32_t)PULSE_HI);
}

static uint16_t derivedMin(uint8_t ji) {
  if (!calRef[ji]) return 0;
  return clampPulse((int32_t)calRef[ji] + (int32_t)calDir[ji] * (int32_t)pulseDelta(ji));
}

static uint16_t derivedMax(uint8_t ji) {
  if (!calRef[ji]) return 0;
  if (isHip(ji)) {
    return clampPulse((int32_t)calRef[ji] - (int32_t)calDir[ji] * (int32_t)pulseDelta(ji));
  }
  return calRef[ji];
}

static uint16_t derivedMid(uint8_t ji) {
  if (!calRef[ji]) return 0;
  if (isHip(ji)) return calRef[ji];
  return (uint16_t)((derivedMax(ji) + derivedMin(ji)) / 2);
}

// Shortest available max→min among calibrated joints of a type; 0 if none ready.
static uint16_t shortestAvailable(bool thigh) {
  uint16_t best = 0xFFFF;
  uint8_t n = 0;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (thigh ? !isThigh(i) : !isShin(i)) continue;
    if (!calRef[i]) continue;
    uint16_t a = availableSpan(i);
    n++;
    if (a < best) best = a;
  }
  if (n == 0 || best == 0xFFFF) return 0;
  return best;
}

static void saveLockedSpans() {
  prefs.begin("servo_cal", false);
  prefs.putUShort("spanTh", lockedSpanThigh);
  prefs.putUShort("spanSh", lockedSpanShin);
  prefs.end();
}

// Recompute and store locked spans from current calibration (all 6 of a type if present).
static String lockShortestSpans() {
  uint16_t th = shortestAvailable(true);
  uint16_t sh = shortestAvailable(false);
  uint8_t nTh = 0, nSh = 0;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (isThigh(i) && calRef[i]) nTh++;
    if (isShin(i) && calRef[i]) nSh++;
  }
  String msg;
  if (nTh == 6 && th > 0) {
    lockedSpanThigh = th;
    msg += "thigh span locked=" + String(th);
  } else {
    msg += "thigh not locked (" + String(nTh) + "/6)";
  }
  msg += " · ";
  if (nSh == 6 && sh > 0) {
    lockedSpanShin = sh;
    msg += "shin span locked=" + String(sh);
  } else {
    msg += "shin not locked (" + String(nSh) + "/6)";
  }
  saveLockedSpans();
  return msg;
}

static void setServoPulse(uint8_t globalCh, uint16_t pulse) {
  if (globalCh >= SERVO_COUNT || !boardOk(globalCh)) return;
  pulse = constrain(pulse, PULSE_LO, PULSE_HI);
  if (globalCh < 16) pwm0.setPWM(globalCh, 0, pulse);
  else pwm1.setPWM(globalCh - 16, 0, pulse);
  lastPulse[globalCh] = pulse;
  Serial.printf("ch%u pulse=%u\n", globalCh, pulse);
}

static void loadCal() {
  prefs.begin("servo_cal", true);
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    char k[12];
    snprintf(k, sizeof(k), "ref%u", i);
    uint16_t r = prefs.getUShort(k, 0);
    if (!r) {
      if (!isHip(i)) {
        snprintf(k, sizeof(k), "max%u", i);
        r = prefs.getUShort(k, 0);
      }
    }
    calRef[i] = r;
    snprintf(k, sizeof(k), "dir%u", i);
    int8_t d = (int8_t)prefs.getChar(k, -1);
    calDir[i] = (d == 1) ? 1 : -1;
  }
  lockedSpanThigh = prefs.getUShort("spanTh", 0);
  lockedSpanShin = prefs.getUShort("spanSh", 0);
  prefs.end();
}

static uint8_t calibratedCount() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (calRef[i]) n++;
  return n;
}

// Copy the committed snapshot into RAM (does not write NVS by itself).
static void applyBackupToRam() {
  for (uint8_t i = 0; i < JOINT_COUNT && i < CAL_BACKUP_COUNT; i++) {
    calRef[i] = CAL_BACKUP_REF[i];
    calDir[i] = (CAL_BACKUP_DIR[i] >= 0) ? 1 : -1;
  }
  lockedSpanThigh = CAL_BACKUP_SPAN_THIGH;
  lockedSpanShin = CAL_BACKUP_SPAN_SHIN;
}

static void persistAllCal() {
  prefs.begin("servo_cal", false);
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    char k[12];
    snprintf(k, sizeof(k), "ref%u", i);
    prefs.putUShort(k, calRef[i]);
    snprintf(k, sizeof(k), "dir%u", i);
    prefs.putChar(k, (char)calDir[i]);
  }
  prefs.putUShort("spanTh", lockedSpanThigh);
  prefs.putUShort("spanSh", lockedSpanShin);
  prefs.end();
}

// Use the JSON snapshot if this chip has never been calibrated.
static void restoreBackupIfEmpty() {
  if (calibratedCount() > 0) return;
  applyBackupToRam();
  persistAllCal();
  Serial.println("NVS empty — restored calibration/servo_cal.json snapshot");
}

static void saveRef(uint8_t ji, uint16_t pulse) {
  calRef[ji] = pulse;
  prefs.begin("servo_cal", false);
  char k[12];
  snprintf(k, sizeof(k), "ref%u", ji);
  prefs.putUShort(k, pulse);
  prefs.end();
}

static void saveDir(uint8_t ji, int8_t dir) {
  calDir[ji] = (dir >= 0) ? 1 : -1;
  prefs.begin("servo_cal", false);
  char k[12];
  snprintf(k, sizeof(k), "dir%u", ji);
  prefs.putChar(k, (char)calDir[ji]);
  prefs.end();
}

static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static String calJson() {
  String s = "{\"ticks_per_deg\":";
  s += String(TICKS_PER_DEG, 3);
  s += ",\"locked_span_thigh\":";
  s += lockedSpanThigh;
  s += ",\"locked_span_shin\":";
  s += lockedSpanShin;
  s += ",\"joints\":[";
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (i) s += ',';
    uint8_t ch = uiToGlobal(JOINTS[i].ui);
    s += "{\"i\":";
    s += i;
    s += ",\"leg\":\"";
    s += JOINTS[i].leg;
    s += "\",\"joint\":\"";
    s += JOINTS[i].joint;
    s += "\",\"mode\":\"";
    s += isHip(i) ? "default" : "max";
    s += "\",\"ui\":";
    s += JOINTS[i].ui;
    s += ",\"ch\":";
    s += ch;
    s += ",\"range_deg\":";
    s += String(isHip(i) ? 90.0f : jointHalfOrFullDeg(i), 1);
    s += ",\"span\":";
    s += pulseDelta(i);
    s += ",\"available\":";
    s += availableSpan(i);
    s += ",\"dir\":";
    s += calDir[i];
    s += ",\"pulse\":";
    s += lastPulse[ch];
    s += ",\"ref\":";
    s += calRef[i];
    s += ",\"max\":";
    s += derivedMax(i);
    s += ",\"min\":";
    s += derivedMin(i);
    s += ",\"mid\":";
    s += derivedMid(i);
    s += '}';
  }
  s += "]}";
  return s;
}

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Servo calibration</title>
  <style>
    body { font-family: system-ui, sans-serif; max-width: 640px; margin: 16px auto; padding: 0 12px; }
    h1 { font-size: 1.15rem; margin: 0 0 6px; }
    h2 { font-size: 1rem; margin: 16px 0 6px; border-bottom: 1px solid #ddd; padding-bottom: 4px; }
    #info { font-size: 0.88rem; color: #333; margin-bottom: 8px; }
    .howto { font-size: 0.85rem; background: #f4f6fa; padding: 10px 12px; border-radius: 6px; line-height: 1.45; }
    .all { width: 100%; padding: 9px; margin: 4px 0; font-size: 0.9rem; }
    .row { border-bottom: 1px solid #e6e6e6; padding: 10px 0; }
    .label { font-weight: 650; margin-bottom: 4px; }
    .meta { font-weight: 400; opacity: 0.6; font-size: 0.82rem; }
    .cal { font-size: 0.8rem; margin: 2px 0 6px; font-variant-numeric: tabular-nums; line-height: 1.35; }
    .cal .ok { color: #1a7f37; }
    .cal .miss { color: #b54708; }
    .controls, .nudge, .save { display: flex; gap: 5px; align-items: center; flex-wrap: wrap; margin-top: 4px; }
    .controls { flex-wrap: nowrap; }
    input[type=range] { flex: 1; min-width: 120px; }
    .val { min-width: 3.2rem; text-align: right; font-variant-numeric: tabular-nums; }
    button { padding: 5px 8px; font-size: 0.85rem; }
    button.pri { background: #1f6feb; color: #fff; border: 1px solid #1f6feb; border-radius: 4px; }
    button.sec { background: #fff; border: 1px solid #bbb; border-radius: 4px; }
    button.dir { min-width: 4.6rem; font-weight: 650; }
    button.dir.on { background: #111; color: #fff; border-color: #111; }
    #status { min-height: 1.2em; margin-top: 12px; white-space: pre-wrap; font-size: 0.88rem; }
  </style>
</head>
<body>
  <h1>Servo calibration</h1>
  <div class="howto">
    <b>Model standard</b> (same on every leg; mirrors = Direction only):<br/>
    · <b>Hip</b> = save mount default only (no min/max). Side buttons mark which way is <b>+CCW</b> from above.<br/>
    · <b>Thigh +</b> = knee rises · MAX=folded hard stop · MIN=straight/flat<br/>
    · <b>Shin +</b> = foot folds under · MAX=folded hard stop · MIN=straight<br/>
    Thigh/shin: tap <b>MIN</b> on the slider side toward model MIN after saving MAX.
  </div>
  <p id="info">Loading…</p>
  <p><a href="/walk"><b>Open walking controls →</b></a></p>
  <button class="all" onclick="lockSpans()">Lock thighs/shins to shortest max→min span</button>
  <button class="all" onclick="gotoGroup('thigh','max')">All thighs → MAX</button>
  <button class="all" onclick="gotoGroup('thigh','mid')">All thighs → MID</button>
  <button class="all" onclick="gotoGroup('thigh','min')">All thighs → MIN</button>
  <button class="all" onclick="gotoGroup('shin','max')">All shins → MAX</button>
  <button class="all" onclick="gotoGroup('shin','mid')">All shins → MID</button>
  <button class="all" onclick="gotoGroup('shin','min')">All shins → MIN</button>
  <button class="all" onclick="gotoAll('mid')">All calibrated → MID / DEFAULT</button>
  <button class="all" onclick="exportCal()">Download calibration JSON</button>
  <button class="all" onclick="restoreCal()">Restore snapshot from firmware backup</button>
  <div id="list"></div>
  <p id="status"></p>
  <script>
    const JOINTS = [
      { leg:'FL', joint:'hip',   ui:12 },
      { leg:'FL', joint:'thigh', ui:8 },
      { leg:'FL', joint:'shin',  ui:22 },
      { leg:'ML', joint:'hip',   ui:11 },
      { leg:'ML', joint:'thigh', ui:7 },
      { leg:'ML', joint:'shin',  ui:21 },
      { leg:'RL', joint:'hip',   ui:10 },
      { leg:'RL', joint:'thigh', ui:5 },
      { leg:'RL', joint:'shin',  ui:17 },
      { leg:'FR', joint:'hip',   ui:23 },
      { leg:'FR', joint:'thigh', ui:19 },
      { leg:'FR', joint:'shin',  ui:13 },
      { leg:'MR', joint:'hip',   ui:29 },
      { leg:'MR', joint:'thigh', ui:28 },
      { leg:'MR', joint:'shin',  ui:9 },
      { leg:'RR', joint:'hip',   ui:26 },
      { leg:'RR', joint:'thigh', ui:18 },
      { leg:'RR', joint:'shin',  ui:1 },
    ];
    const PULSE_LO = 80, PULSE_HI = 520, PULSE_DEFAULT = 300;
    const list = document.getElementById('list');
    const status = document.getElementById('status');
    const info = document.getElementById('info');
    const timers = {};
    const rows = [];
    const isHip = (j) => j.joint === 'hip';

    function boardLabel(ui) { return ui <= 16 ? '0x40' : '0x41'; }
    function localCh(ui) { return (ui - 1) % 16; }
    function constrain(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

    let lastLeg = '';
    JOINTS.forEach((j, i) => {
      if (j.leg !== lastLeg) {
        lastLeg = j.leg;
        const h = document.createElement('h2');
        h.textContent = j.leg;
        list.appendChild(h);
      }
      const ch = j.ui - 1;
      const hip = isHip(j);
      const saveLabel = hip ? 'Save DEFAULT' : 'Save MAX';
      const hint = hip
        ? ' · +model=CCW from above · Save DEFAULT only (no min/max)'
        : (j.joint === 'thigh'
          ? ' · +model=knee up · Save at folded MAX · MIN = open/flat'
          : ' · +model=fold under · Save at folded MAX · MIN = straight');
      // Hip: side = which way is +CCW. Thigh/shin: side = which way is model MIN.
      // calDir +1 => higher PWM toward model MIN (−hip / open thigh-shin).
      const leftDir = hip ? 1 : -1;
      const rightDir = hip ? -1 : 1;
      const leftLbl = hip ? '← +CCW' : 'MIN ←';
      const rightLbl = hip ? '+CCW →' : '→ MIN';
      const row = document.createElement('div');
      row.className = 'row';
      row.innerHTML =
        '<div class="label">' + j.leg + ' ' + j.joint +
        ' <span class="meta">ui#' + j.ui + ' · ' + boardLabel(j.ui) + ' ch' + localCh(j.ui) +
        hint + '</span></div>' +
        '<div class="cal" data-cal></div>' +
        '<div class="nudge">' +
        '<button class="sec" data-n="-10">-10</button>' +
        '<button class="sec" data-n="-1">-1</button>' +
        '<button class="sec" data-n="1">+1</button>' +
        '<button class="sec" data-n="10">+10</button>' +
        '</div>' +
        '<div class="controls">' +
        '<button class="sec dir" data-dir="' + leftDir + '">' + leftLbl + '</button>' +
        '<input type="range" min="' + PULSE_LO + '" max="' + PULSE_HI + '" value="' + PULSE_DEFAULT + '" step="1"/>' +
        '<span class="val">' + PULSE_DEFAULT + '</span>' +
        '<button class="sec dir" data-dir="' + rightDir + '">' + rightLbl + '</button>' +
        '</div>' +
        '<div class="save">' +
        '<button class="pri" data-save="1">' + saveLabel + '</button>' +
        (hip
          ? '<button class="sec" data-goto="mid">Go DEFAULT</button>'
          : '<button class="sec" data-goto="max">Go MAX</button>' +
            '<button class="sec" data-goto="mid">Go MID</button>' +
            '<button class="sec" data-goto="min">Go MIN</button>') +
        '</div>';
      const range = row.querySelector('input');
      const val = row.querySelector('.val');
      const calEl = row.querySelector('[data-cal]');
      const dirBtns = row.querySelectorAll('[data-dir]');
      const setPulseUI = (p) => { range.value = p; val.textContent = p; };
      const setDirUI = (d) => {
        dirBtns.forEach(b => b.classList.toggle('on', +b.dataset.dir === d));
      };
      dirBtns.forEach(btn => {
        btn.onclick = () => setDir(i, +btn.dataset.dir, setDirUI);
      });
      row.querySelectorAll('[data-n]').forEach(btn => {
        btn.onclick = () => {
          const p = constrain(+range.value + (+btn.dataset.n), PULSE_LO, PULSE_HI);
          setPulseUI(p);
          movePulse(ch, p);
        };
      });
      range.oninput = () => {
        val.textContent = range.value;
        const p = +range.value;
        clearTimeout(timers[ch]);
        timers[ch] = setTimeout(() => movePulse(ch, p), 60);
      };
      row.querySelector('[data-save]').onclick = () => saveRef(i, +range.value);
      row.querySelectorAll('[data-goto]').forEach(btn => {
        btn.onclick = () => gotoCal(i, btn.dataset.goto, setPulseUI);
      });
      list.appendChild(row);
      rows.push({ i, ch, j, hip, range, val, calEl, setPulseUI, setDirUI });
    });

    function fmtCal(c) {
      if (c.mode === 'default') {
        const side = c.dir > 0 ? '+CCW on left (←)' : '+CCW on right (→)';
        if (!c.ref) {
          return '<span class="miss">default=unset</span> · ' + side + ' · now=' + c.pulse;
        }
        return '<span class="ok">default=' + c.ref + '</span> · ' + side + ' · now=' + c.pulse;
      }
      const dir = c.dir > 0 ? 'MIN on right (→ higher pulse)' : 'MIN on left (← lower pulse)';
      if (!c.ref) {
        return '<span class="miss">max=unset</span> · ' + dir + ' · now=' + c.pulse;
      }
      return '<span class="ok">max=' + c.max + '</span>' +
        ' → <span class="ok">mid=' + c.mid + '</span>' +
        ' → <span class="ok">min=' + c.min + '</span>' +
        ' · span=' + c.span + ' (avail ' + c.available + ')' +
        ' · ' + dir + ' · now=' + c.pulse;
    }

    async function refresh() {
      try {
        info.textContent = await (await fetch('/api/status')).text();
        const data = await (await fetch('/api/cal')).json();
        const lockNote = ' · locked thigh span=' + (data.locked_span_thigh || 'off') +
          ' · shin span=' + (data.locked_span_shin || 'off');
        info.textContent += lockNote;
        data.joints.forEach((c, idx) => {
          const r = rows[idx];
          if (!r) return;
          r.calEl.innerHTML = fmtCal(c);
          r.setDirUI(c.dir);
          if (c.pulse) r.setPulseUI(c.pulse);
        });
      } catch (e) {
        info.textContent = 'Cannot reach ESP';
      }
    }

    async function movePulse(ch, pulse) {
      try {
        const r = await fetch('/api/pulse?ch=' + ch + '&pulse=' + pulse);
        status.textContent = r.ok ? ('ch' + ch + ' → ' + pulse) : await r.text();
      } catch (e) { status.textContent = 'Request failed'; }
    }

    async function setDir(i, dir, setDirUI) {
      try {
        status.textContent = await (await fetch('/api/cal/dir?i=' + i + '&dir=' + dir)).text();
        setDirUI(dir);
        await refresh();
      } catch (e) { status.textContent = 'Dir failed'; }
    }

    async function saveRef(i, pulse) {
      try {
        status.textContent = await (await fetch('/api/cal/save?i=' + i + '&pulse=' + pulse)).text();
        await refresh();
      } catch (e) { status.textContent = 'Save failed'; }
    }

    async function gotoCal(i, which, setPulseUI) {
      try {
        const t = await (await fetch('/api/cal/goto?i=' + i + '&which=' + which)).text();
        status.textContent = t;
        const m = t.match(/pulse=(\d+)/);
        if (m) setPulseUI(+m[1]);
        await refresh();
      } catch (e) { status.textContent = 'Goto failed'; }
    }

    async function gotoAll(which) {
      try {
        status.textContent = await (await fetch('/api/cal/all?which=' + which)).text();
        await refresh();
      } catch (e) { status.textContent = 'Failed'; }
    }

    async function gotoGroup(joint, which) {
      try {
        status.textContent = await (await fetch('/api/cal/group?joint=' + joint + '&which=' + which)).text();
        await refresh();
      } catch (e) { status.textContent = 'Failed'; }
    }

    async function lockSpans() {
      try {
        status.textContent = await (await fetch('/api/cal/lock')).text();
        await refresh();
      } catch (e) { status.textContent = 'Lock failed'; }
    }

    async function restoreCal() {
      if (!confirm('Overwrite NVS with the committed servo_cal.json snapshot?')) return;
      try {
        status.textContent = await (await fetch('/api/cal/restore')).text();
        await refresh();
      } catch (e) { status.textContent = 'Restore failed'; }
    }

    async function exportCal() {
      try {
        const t = await (await fetch('/api/cal')).text();
        status.textContent = t;
        const blob = new Blob([t], { type: 'application/json' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'servo_cal.json';
        a.click();
        URL.revokeObjectURL(a.href);
      } catch (e) { status.textContent = 'Export failed'; }
    }

    refresh();
    setInterval(refresh, 8000);
  </script>
</body>
</html>
)HTML";

static void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

static void handleStatus() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (calRef[i]) n++;
  String msg;
  msg += pca0Ok ? "PCA0 0x40 OK. " : "PCA0 0x40 MISSING. ";
  msg += pca1Ok ? "PCA1 0x41 OK. " : "PCA1 0x41 MISSING. ";
  msg += "IP ";
  msg += WiFi.localIP().toString();
  msg += " | calibrated ";
  msg += n;
  msg += "/18 (hips=default, others=max)";
  server.send(200, "text/plain", msg);
}

static void handleCalGet() { server.send(200, "application/json", calJson()); }

static void handlePulse() {
  if (!server.hasArg("ch") || !server.hasArg("pulse")) {
    server.send(400, "text/plain", "missing ch or pulse");
    return;
  }
  int ch = server.arg("ch").toInt();
  int pulse = server.arg("pulse").toInt();
  if (ch < 0 || ch >= SERVO_COUNT) {
    server.send(400, "text/plain", "ch out of range");
    return;
  }
  if (!boardOk((uint8_t)ch)) {
    server.send(500, "text/plain", "pca missing");
    return;
  }
  setServoPulse((uint8_t)ch, (uint16_t)pulse);
  server.send(200, "text/plain", "ok");
}

static void handleCalSave() {
  if (!server.hasArg("i")) {
    server.send(400, "text/plain", "missing i");
    return;
  }
  int i = server.arg("i").toInt();
  if (i < 0 || i >= JOINT_COUNT) {
    server.send(400, "text/plain", "i out of range");
    return;
  }
  uint8_t ch = uiToGlobal(JOINTS[i].ui);
  uint16_t pulse = server.hasArg("pulse")
                       ? (uint16_t)constrain(server.arg("pulse").toInt(), (int)PULSE_LO, (int)PULSE_HI)
                       : lastPulse[ch];
  saveRef((uint8_t)i, pulse);
  setServoPulse(ch, pulse);
  String msg = String(JOINTS[i].leg) + " " + JOINTS[i].joint + " ";
  if (isHip((uint8_t)i)) {
    msg += "DEFAULT=" + String(pulse) + " (no min/max)";
  } else {
    msg += "MAX=" + String(pulse);
    msg += " · ";
    msg += lockShortestSpans();
    msg += " → mid=" + String(derivedMid((uint8_t)i)) +
           " min=" + String(derivedMin((uint8_t)i));
  }
  Serial.println(msg);
  server.send(200, "text/plain", msg);
}

static void handleCalDir() {
  if (!server.hasArg("i") || !server.hasArg("dir")) {
    server.send(400, "text/plain", "missing i or dir");
    return;
  }
  int i = server.arg("i").toInt();
  if (i < 0 || i >= JOINT_COUNT) {
    server.send(400, "text/plain", "i out of range");
    return;
  }
  saveDir((uint8_t)i, server.arg("dir").toInt() >= 0 ? 1 : -1);
  String msg = String(JOINTS[i].leg) + " " + JOINTS[i].joint +
               " dir=" + String(calDir[i] > 0 ? "+pulse toward −model" : "+pulse toward +model");
  if (calRef[i] && !isHip((uint8_t)i)) {
    msg += " · ";
    msg += lockShortestSpans();
    msg += " → max=" + String(derivedMax((uint8_t)i)) +
           " mid=" + String(derivedMid((uint8_t)i)) +
           " min=" + String(derivedMin((uint8_t)i));
  } else if (calRef[i] && isHip((uint8_t)i)) {
    msg += " · default=" + String(calRef[i]) +
           (calDir[i] > 0 ? " · +CCW ←" : " · +CCW →");
  }
  server.send(200, "text/plain", msg);
}

static bool gotoWhich(uint8_t i, const String &which, String &msg) {
  if (!calRef[i]) {
    msg = String(JOINTS[i].leg) + " " + JOINTS[i].joint + " not calibrated";
    return false;
  }
  if (isHip(i) && (which == "max" || which == "min")) {
    msg = String(JOINTS[i].leg) + " hip has default only (no min/max)";
    return false;
  }
  uint16_t pulse = 0;
  if (which == "max") pulse = derivedMax(i);
  else if (which == "min") pulse = derivedMin(i);
  else if (which == "mid") pulse = derivedMid(i);
  else {
    msg = "which must be max|min|mid";
    return false;
  }
  setServoPulse(uiToGlobal(JOINTS[i].ui), pulse);
  msg = String(JOINTS[i].leg) + " " + JOINTS[i].joint + " -> " + which +
        " pulse=" + String(pulse);
  return true;
}

static void handleCalGoto() {
  if (!server.hasArg("i") || !server.hasArg("which")) {
    server.send(400, "text/plain", "missing i or which");
    return;
  }
  int i = server.arg("i").toInt();
  if (i < 0 || i >= JOINT_COUNT) {
    server.send(400, "text/plain", "i out of range");
    return;
  }
  String which = server.arg("which");
  which.toLowerCase();
  String msg;
  if (!gotoWhich((uint8_t)i, which, msg)) {
    server.send(400, "text/plain", msg);
    return;
  }
  server.send(200, "text/plain", msg);
}

static void handleCalAll() {
  String which = server.hasArg("which") ? server.arg("which") : "mid";
  which.toLowerCase();
  uint8_t ok = 0, skip = 0;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    String msg;
    if (gotoWhich(i, which, msg)) ok++;
    else skip++;
  }
  server.send(200, "text/plain",
              "moved " + String(ok) + " to " + which + ", skipped " + String(skip));
}

static void handleCalGroup() {
  if (!server.hasArg("joint") || !server.hasArg("which")) {
    server.send(400, "text/plain", "missing joint or which");
    return;
  }
  String joint = server.arg("joint");
  joint.toLowerCase();
  String which = server.arg("which");
  which.toLowerCase();
  if (joint != "thigh" && joint != "shin") {
    server.send(400, "text/plain", "joint must be thigh|shin");
    return;
  }
  uint8_t ok = 0, skip = 0;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (joint == "thigh" && !isThigh(i)) continue;
    if (joint == "shin" && !isShin(i)) continue;
    String msg;
    if (gotoWhich(i, which, msg)) ok++;
    else skip++;
  }
  server.send(200, "text/plain",
              "moved " + String(ok) + " " + joint + "s to " + which +
              ", skipped " + String(skip));
}

static void handleCalLock() {
  server.send(200, "text/plain", lockShortestSpans());
}

static void handleCalRestore() {
  applyBackupToRam();
  persistAllCal();
  server.send(200, "text/plain",
              "restored snapshot: " + String(calibratedCount()) +
              "/18  thighSpan=" + String(lockedSpanThigh) +
              " shinSpan=" + String(lockedSpanShin));
}

// Model angle (deg) → PWM using calibration.
// Hip: θ=0 at default; +θ = CCW. Thigh/shin: θ=0 straight, θ=θmax at folded MAX.
static uint16_t modelToPulse(uint8_t ji, float angleDeg) {
  if (!calRef[ji]) return PULSE_DEFAULT;
  if (isHip(ji)) {
    int32_t p = (int32_t)calRef[ji] -
                (int32_t)calDir[ji] * (int32_t)lroundf(angleDeg * TICKS_PER_DEG);
    return clampPulse(p);
  }
  float tmax = jointHalfOrFullDeg(ji);
  float u = angleDeg / tmax;
  if (u < 0) u = 0;
  if (u > 1) u = 1;
  int32_t p = (int32_t)calRef[ji] +
              (int32_t)calDir[ji] * (int32_t)lroundf((1.f - u) * (float)pulseDelta(ji));
  return clampPulse(p);
}

static int jointIndex(uint8_t leg, const char *joint) {
  // leg 0..5 = FL ML RL FR MR RR; joints packed 3 per leg in JOINTS[]
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t ji = leg * 3 + i;
    if (!strcmp(JOINTS[ji].joint, joint)) return ji;
  }
  return -1;
}

static void applyWalkPose() {
  const bool freezeHips = walkGetParams().freezeHips;
  for (uint8_t leg = 0; leg < 6; leg++) {
    LegAngles a = walkLegAngles(leg);
    int hi = jointIndex(leg, "hip");
    int th = jointIndex(leg, "thigh");
    int sh = jointIndex(leg, "shin");
    if (hi >= 0) {
      // freeze: hold mount default (0°); else follow gait coxa
      float hipDeg = freezeHips ? 0.f : (a.coxa * RAD_TO_DEG);
      setServoPulse(uiToGlobal(JOINTS[hi].ui), modelToPulse(hi, hipDeg));
    }
    if (th >= 0) setServoPulse(uiToGlobal(JOINTS[th].ui), modelToPulse(th, a.femur * RAD_TO_DEG));
    if (sh >= 0) setServoPulse(uiToGlobal(JOINTS[sh].ui), modelToPulse(sh, a.knee * RAD_TO_DEG));
  }
}

static void handleWalkPage() { server.send_P(200, "text/html", WALK_HTML); }

static void handleWalkGet() {
  WalkParams p = walkGetParams();
  String j = "{";
  j += "\"enabled\":" + String(walkEnabled() ? "true" : "false");
  j += ",\"freq\":" + String(p.freq, 3);
  j += ",\"stride_mm\":" + String(p.stride * 1000.f, 1);
  j += ",\"lift_mm\":" + String(p.lift * 1000.f, 1);
  j += ",\"height_mm\":" + String(p.height * 1000.f, 1);
  j += ",\"radius_mm\":" + String(p.radius * 1000.f, 1);
  j += ",\"splay_deg\":" + String(p.splay * RAD_TO_DEG, 2);
  j += ",\"crab_deg\":" + String(p.crab * RAD_TO_DEG, 1);
  j += ",\"turn_dps\":" + String(p.turn * RAD_TO_DEG, 1);
  j += ",\"freezeHips\":" + String(p.freezeHips ? "true" : "false");
  j += ",\"phase\":" + String(walkPhase(), 3);
  j += "}";
  server.send(200, "application/json", j);
}

static void handleWalkParams() {
  WalkParams p = walkGetParams();
  if (server.hasArg("freq")) p.freq = constrain(server.arg("freq").toFloat(), 0.1f, 2.0f);
  if (server.hasArg("stride")) p.stride = constrain(server.arg("stride").toFloat(), 0, 110) / 1000.f;
  if (server.hasArg("lift")) p.lift = constrain(server.arg("lift").toFloat(), 5, 60) / 1000.f;
  if (server.hasArg("height")) p.height = constrain(server.arg("height").toFloat(), 65, 115) / 1000.f;
  if (server.hasArg("radius")) p.radius = constrain(server.arg("radius").toFloat(), 110, 178) / 1000.f;
  if (server.hasArg("splay")) p.splay = constrain(server.arg("splay").toFloat(), 0, 35) * DEG_TO_RAD;
  if (server.hasArg("crab")) p.crab = constrain(server.arg("crab").toFloat(), -90, 90) * DEG_TO_RAD;
  if (server.hasArg("turn")) p.turn = constrain(server.arg("turn").toFloat(), -45, 45) * DEG_TO_RAD;
  if (server.hasArg("freezeHips")) p.freezeHips = server.arg("freezeHips") != "0";
  walkSetParams(p);
  String msg = "freq=" + String(p.freq, 2) + " stride=" + String(p.stride * 1000, 0) +
               "mm lift=" + String(p.lift * 1000, 0) + "mm h=" + String(p.height * 1000, 0) +
               "mm r=" + String(p.radius * 1000, 0) + "mm splay=" + String(p.splay * RAD_TO_DEG, 1) +
               "deg crab=" + String(p.crab * RAD_TO_DEG, 0) + " turn=" + String(p.turn * RAD_TO_DEG, 0) +
               " hips=" + String(p.freezeHips ? "frozen" : "active");
  server.send(200, "text/plain", msg);
}

static void handleWalkToggle() {
  bool on = !walkEnabled();
  if (on) {
    walkReset();
    walkSetEnabled(true);
    walkUpdate(0);
    applyWalkPose();
  } else {
    walkSetEnabled(false);
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
      if (!calRef[i]) continue;
      setServoPulse(uiToGlobal(JOINTS[i].ui), derivedMid(i));
    }
  }
  server.send(200, "text/plain", on ? "WALKING" : "STOPPED (mid/default)");
}

static void handleWalkStatus() {
  WalkParams p = walkGetParams();
  String msg = walkEnabled() ? "WALKING" : "STOPPED";
  if (p.freezeHips) msg += " hips=frozen";
  msg += " phase=" + String(walkPhase(), 2);
  msg += " freq=" + String(p.freq, 2);
  uint8_t bad = 0;
  for (uint8_t i = 0; i < 6; i++) if (!walkLegAngles(i).ok) bad++;
  if (bad) msg += " unreachable=" + String(bad);
  server.send(200, "text/plain", msg);
}

static uint32_t lastWalkMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("Hexapod calibrator: hips=default, thigh/shin=max");

  for (uint8_t i = 0; i < SERVO_COUNT; i++) lastPulse[i] = PULSE_DEFAULT;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    calRef[i] = 0;
    calDir[i] = -1;
  }
  loadCal();
  restoreBackupIfEmpty();
  Serial.println(lockShortestSpans());
  Serial.println("CAL_JSON_BEGIN");
  Serial.println(calJson());
  Serial.println("CAL_JSON_END");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(50);

  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) Serial.printf("  found 0x%02X\n", a);
  }

  pca0Ok = i2cPresent(0x40);
  pca1Ok = i2cPresent(0x41);
  if (pca0Ok && pwm0.begin()) {
    pwm0.setPWMFreq(50);
    pwm0.setOutputMode(true);
    Serial.println("PCA0 0x40 ready");
  } else {
    pca0Ok = false;
    Serial.println("PCA0 missing/fail");
  }
  if (pca1Ok && pwm1.begin()) {
    pwm1.setPWMFreq(50);
    pwm1.setOutputMode(true);
    Serial.println("PCA1 0x41 ready");
  } else {
    pca1Ok = false;
    Serial.println("PCA1 missing/fail");
  }

  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    uint8_t ch = uiToGlobal(JOINTS[i].ui);
    uint16_t p = calRef[i] ? derivedMid(i) : PULSE_DEFAULT;
    setServoPulse(ch, p);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to '%s'", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed");
  }

  server.on("/", handleRoot);
  server.on("/walk", handleWalkPage);
  server.on("/api/status", handleStatus);
  server.on("/api/cal", handleCalGet);
  server.on("/api/pulse", handlePulse);
  server.on("/api/cal/save", handleCalSave);
  server.on("/api/cal/dir", handleCalDir);
  server.on("/api/cal/goto", handleCalGoto);
  server.on("/api/cal/all", handleCalAll);
  server.on("/api/cal/group", handleCalGroup);
  server.on("/api/cal/lock", handleCalLock);
  server.on("/api/cal/restore", handleCalRestore);
  server.on("/api/walk/params", handleWalkParams);
  server.on("/api/walk/get", handleWalkGet);
  server.on("/api/walk/toggle", handleWalkToggle);
  server.on("/api/walk/status", handleWalkStatus);
  server.begin();
  Serial.println("Web server started");
  Serial.println("Walk UI: /walk");
  lastWalkMs = millis();
}

void loop() {
  server.handleClient();
  if (walkEnabled()) {
    uint32_t now = millis();
    float dt = (now - lastWalkMs) / 1000.f;
    lastWalkMs = now;
    walkUpdate(dt);
    applyWalkPose();
  } else {
    lastWalkMs = millis();
  }
}
