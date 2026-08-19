# Troubleshooting

Symptoms → checks → fixes. Start with serial monitor output (115200 baud) while reproducing the issue.

---

## WiFi / web page

| Symptom | Fix |
|---------|-----|
| Serial stuck at `Connecting...` | Wrong SSID/password in `wifi_secrets.h`; ESP must be on **2.4 GHz**; re-flash after edit |
| No IP printed | Wait 30 s; check router; try serial-only calibrator features if WiFi fails |
| Phone cannot open `http://<ip>` | PC/phone on **same network** as ESP; disable VPN; try IP from serial exactly |
| `/walk` loads but 3D blank | Browser needs **internet** for Three.js CDN; rest of page still works |

---

## I²C / PCA9685

| Symptom | Fix |
|---------|-----|
| `PCA0 missing/fail` | Check SDA/SCL (GPIO6/5), 3.3 V logic, common ground |
| Only one board found | Second board needs **A0 → V+** for address **0x41** |
| I²C scan finds nothing | Loose wires, wrong pins, or flat battery on ESP32 |
| One leg dead, others OK | Match [servo_map.json](../pca_servo_test/servo_map.json) wiring to UI channel; test with calibrator slider on that channel |

---

## Servos

| Symptom | Fix |
|---------|-----|
| Servo buzzes at stop | Normal briefly at MIN/MAX; back off slider; recalibrate MAX if stop moved |
| Wrong joint moves | Wrong channel in `servo_map.json` / firmware `JOINTS[]` |
| Leg moves backward vs preview | Flip **dir** on that joint (see [CALIBRATION.md](CALIBRATION.md)) |
| Jerky or weak motion | Servo power supply undervoltage; add bulk cap; separate 5 V from USB |
| All servos twitch at boot | Expected short pulse; ensure supply can deliver 18 servos |

---

## Calibration count

| Symptom | Fix |
|---------|-----|
| Less than 18/18 calibrated | Finish hips (DEFAULT), thighs/shins (MAX) per [CALIBRATION.md](CALIBRATION.md) |
| Restore did nothing | Click **Restore snapshot from firmware backup** on `/` or re-flash after `gen_cal_backup.py` |
| Values look random | NVS corrupt — restore snapshot or full re-cal |

---

## Walk / gait (`/walk`)

| Symptom | Fix |
|---------|-----|
| Sliders move preview but not robot | Click **Deploy to robot** |
| Start does nothing | Deploy first; check 18/18 calibrated; serial for errors |
| Robot walks before Start | Only **Start robot** runs motion; switching tabs stops robot |
| Feet don’t lift | Increase **lift**; decrease **stride** or **freq** |
| Hips pegged / leg skips | Lower **turn** rate; disable spin until walk is good; try **Freeze hips** |
| “Unreachable” / bad pose in status | Lower **height** or **radius**; increase **splay** on corners |
| Stretch/Rotate won’t run | Must press **Start robot** on that tab (no auto-start) |
| Control hold doesn’t move | Deploy first; hold D-pad or spin buttons; release to stop |
| Stop pose wrong | **Stop robot** parks thighs between mid and min; hips default — expected |

---

## Flash / PlatformIO

| Symptom | Fix |
|---------|-----|
| Upload fails | Fix `upload_port` in `platformio.ini`; hold BOOT if needed; USB cable data-capable |
| `pio` not found | Install PlatformIO IDE extension or CLI |
| Windows `cp` fails | Use `Copy-Item` for `wifi_secrets.h` |

---

## Simulation vs robot diverge

| Symptom | Fix |
|---------|-----|
| Preview OK, hardware wrong | Calibration / direction issue, not gait math |
| Preview wrong | Open `hexapod_description/view_hexapod.html`; compare link lengths in [hips.json](../hexapod_description/hips.json) to your physical robot |

---

## Still stuck?

1. Serial log from cold boot through first **Start robot**
2. Calibrator **Download calibration JSON** attached to an issue
3. Confirm PCA scan, calibrated count, and which tab + slider values were deployed
