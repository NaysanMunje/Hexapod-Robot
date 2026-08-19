# Servo calibration

Follow this on a **new build**, after rewiring, or if legs move the wrong way. The robot should be **off the ground** and powered.

**Reference while you work:** [JOINT_CONVENTION.md](../pca_servo_test/JOINT_CONVENTION.md) (angles, signs, PWM rules).

**Channel numbers:** [servo_map.json](../pca_servo_test/servo_map.json).

---

## Before you start

1. Flash firmware and open **`http://<esp-ip>/`** (calibrator).
2. Serial log should show `PCA0 0x40 ready` and `PCA1 0x41 ready`.
3. Status line should show how many joints are calibrated (goal: **18/18**).

If this repo’s hardware matches yours, try a quick test first: firmware may load [servo_cal.json](../calibration/servo_cal.json) from flash backup. Use **Restore snapshot from firmware backup** on the calibrator page. If all legs move correctly on **Go MID**, you can skip to the lock-span step below.

---

## Calibrator page overview

Each joint row has:

| Control | Purpose |
|---------|---------|
| Slider + **− / +** | Nudge raw PWM (80–520 µs ticks) |
| **Save DEFAULT** (hip only) | Current pose = model 0° |
| **Save MAX** (thigh/shin) | Current folded hard stop = model MAX |
| **← MIN / MIN →** (or **← +CCW / +CCW →** on hips) | Set direction: higher PWM toward model MIN |
| **Go MAX / Go MID / Go MIN** | Jump to calibrated endpoints |

Global buttons at the top:

- **Lock thighs/shins to shortest max→min span** — run after all thighs and shins have MAX saved
- **All … → MID/MIN/MAX** — test groups
- **Restore snapshot** — load committed JSON from firmware

---

## Order of work

Work **one joint at a time** or leg-by-leg (hip → thigh → shin on FL, then next leg, …). Thighs and shins **must** be calibrated before walking.

### Step 1 — All hips (6 joints)

For each hip:

1. Rotate coxa by hand or slider until the leg matches the **mount default** (middle legs along ±Y; corners outward — see JOINT_CONVENTION).
2. Click **Save DEFAULT**.
3. Tap **← +CCW** or **+CCW →** so that **increasing PWM moves the foot clockwise from above** (toward model MIN = −45°).
4. Test: **Go MAX** ≈ CCW from default; **Go MIN** ≈ CW from default.

Hips do **not** use Save MAX at a mechanical stop.

### Step 2 — All thighs (6 joints)

For each thigh:

1. Move to the **folded hard stop** (femur up against hip / maximum fold).
2. Click **Save MAX**.
3. Set direction so **+pulse opens the thigh toward flat** (model MIN = straight).
4. Test: **Go MIN** extends thigh; **Go MID** halfway; **Go MAX** folded.

### Step 3 — All shins (6 joints)

For each shin:

1. Move to **folded hard stop** (foot under thigh).
2. Click **Save MAX**.
3. Set direction so **+pulse straightens** the shin (model MIN).
4. Test: **Go MIN** straight; **Go MID** halfway; **Go MAX** folded.

### Step 4 — Lock spans

When all 12 thighs and 12 shins have `ref` saved:

1. Click **Lock thighs/shins to shortest max→min span**.
2. Serial / status should confirm locked spans (this build uses ~226 ticks thigh, ~203 shin when all legs match).

Locking keeps all six legs on the same usable range even if one servo has slightly less travel.

### Step 5 — Verify

1. **All calibrated → MID / DEFAULT** — robot should sit in a symmetric rest-like pose.
2. Move each leg through **Go MIN** and **Go MAX** once — no buzzing for more than a moment at stops.
3. Download **calibration JSON** from the page and save to `calibration/servo_cal.json` if you want the backup committed (optional).

---

## Direction mistakes (most common error)

Symptom: **Go MIN** moves the wrong way or hits a stop immediately.

Fix: flip **MIN ← / → MIN** (or hip **+CCW** side), then re-test MIN/MID/MAX. You do **not** need to re-save ref unless the reference pose was wrong.

---

## After calibration

1. Open **`/walk`**
2. **Deploy to robot**
3. Try **Walk** tab with **Freeze hips** on and low stride

If gait IK reports unreachable feet, check **height**, **radius**, and **lift** in Settings — start from defaults (95 mm height, 140 mm radius).

---

## Saving calibration into firmware

To bake your table into the next flash for other boards:

```bash
cd pca_servo_test
python gen_cal_backup.py   # reads ../calibration/servo_cal.json
pio run -t upload
```

Empty NVS on a new chip will then auto-restore your snapshot on boot.
