# Joint rotation standard (model ↔ servos)

Robot frame: **+X forward, +Y left, +Z up**.  
All six legs use the **same** kinematic signs (no left/right axis flip in the model). Servo mounting mirrors are handled only by each joint’s **direction** flag in calibration.

Naming: UI `FL/ML/RL/FR/MR/RR` = model `LF/LM/LR/RF/RM/RR`.

PWM snapshot for this robot: [`../calibration/servo_cal.json`](../calibration/servo_cal.json).

---

## Zero poses

| Joint | Angle 0 (model) |
|-------|------------------|
| **Hip** | Mount default: middle legs horizontal (±Y); corners perpendicular **outward** to the adjacent **79.893 mm** body edge (`yaw0` in `hips.json`) |
| **Thigh** | Femur horizontal in the coxa’s radial plane (collinear with coxa, flat) |
| **Shin** | Shin collinear with thigh (fully open) |

---

## Hip (special)

Hips are **not** calibrated to min/max stops. Only the **mount default** is saved (mid / model 0°).

- Side buttons mark which slider direction is **+CCW from above** (`← +CCW` / `+CCW →`).
- Gait later uses `pulse = default + scale * (−θ)` with that direction (θ = model hip angle).
- Soft ±45° limits can be applied in software without calibrating those PWM ends.

## Thigh / shin

Still calibrate from **MAX** hard stop; MIN = remaining travel; MID = midpoint.
Side buttons: **MIN ←** / **→ MIN**.

---

## Positive model angles (same on every leg)

| Joint | + direction | Range | MAX | MIN |
|-------|-------------|-------|-----|-----|
| **Hip** | **CCW from above** (right-hand about +Z). Foot swings counterclockwise around the body from the default. | −45° … +45° | +45° (CCW) | −45° (CW) |
| **Thigh** | Femur tip / knee **rises** (pitch up from horizontal) | 0° … 102.5° | **102.5°** hard stop (thigh folded toward hip) | **0°** (straight / flat) |
| **Shin** | Foot folds **back under** the thigh (knee bend) | 0° … 122.4° | **122.4°** hard stop (shin folded) | **0°** (straight with thigh) |

These are exactly the signs used by `view_hexapod.html` IK / gait.

---

## How to set servo direction in the calibrator

Calibration stores one reference pulse, then derives the rest. **Direction** answers:

> Does **increasing PWM** move the joint toward **model MIN**?

| `dir` | Meaning |
|-------|---------|
| **+ pulse** | Higher PWM → toward model MIN |
| **− pulse** | Higher PWM → toward model MAX (i.e. lower PWM → MIN) |

### Hip
1. Align coxa to **mount default** → **Save DEFAULT** (this is model 0° / mid).
2. Nudge PWM. Watch the foot from above.
3. Pick direction so **+pulse moves CW from above** (toward model MIN = −45°).
4. Check: **Go MAX** = CCW from default; **Go MIN** = CW from default.

### Thigh
1. Ease to the **folded hard stop** (thigh up against hip) → **Save MAX**.
2. Pick direction so **+pulse opens the thigh toward flat/straight** (model MIN).
3. Check: **Go MIN** lowers/extends the thigh; **Go MID** is halfway.

### Shin
1. Ease to the **folded hard stop** → **Save MAX**.
2. Pick direction so **+pulse opens the shin toward straight** (model MIN).
3. Check: **Go MIN** straightens the shin; **Go MID** is halfway.

---

## Mapping model angle → PWM (firmware)

Once `ref` and `dir` are set:

**Hip** (`ref` = default):  
`pulse(θ) = ref + dir * ticks_per_deg * (−θ)`  
so θ = +45° (MAX) and θ = −45° (MIN) land on the derived endpoints.

**Thigh / shin** (`ref` = MAX):  
`pulse(θ) = ref + dir * ticks_per_deg * (θ_max − θ)`  
so θ = θ_max → `ref`, θ = 0 → derived MIN.

(`ticks_per_deg ≈ 400/180` with the current PCA pulse scale.)
