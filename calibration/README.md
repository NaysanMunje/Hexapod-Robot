# Servo calibration backup

Snapshot of this robot’s PWM table, dumped from ESP32 NVS on **2026-08-17**.

| File | Use |
|------|-----|
| [`servo_cal.json`](servo_cal.json) | Source of truth (commit this) |
| [`../pca_servo_test/include/calibration_backup.h`](../pca_servo_test/include/calibration_backup.h) | Same numbers compiled into firmware |

## What the fields mean

- **Hip `ref`:** mount default (model 0°). `max`/`min` here are only ±45° software ends, not hard stops.
- **Thigh/shin `ref`:** folded mechanical MAX. `min`/`mid` are derived.
- **`dir`:** `+1` = higher PWM toward model MIN; `-1` = the opposite.
- **`locked_span_thigh` / `_shin`:** shortest available max→min among all six of that joint (226 / 203 ticks on this dump).

Joint signs: [`../pca_servo_test/JOINT_CONVENTION.md`](../pca_servo_test/JOINT_CONVENTION.md).  
Channel map: [`../pca_servo_test/servo_map.json`](../pca_servo_test/servo_map.json).

## Restore on the robot

Firmware already contains this snapshot.

- **Empty NVS** (new ESP, or flash wipe that cleared NVS): applied automatically on boot.
- **Force overwrite:** calibrator page → **Restore snapshot from firmware backup**, or `GET /api/cal/restore`.

## Refresh after you re-calibrate

1. Open `http://<esp-ip>/` → **Download calibration JSON**.
2. Replace `calibration/servo_cal.json` (keep the metadata fields if you want).
3. From `pca_servo_test/`: `python gen_cal_backup.py`
4. `pio run -t upload`

Or copy the JSON between `CAL_JSON_BEGIN` / `CAL_JSON_END` on the serial log after reset.
