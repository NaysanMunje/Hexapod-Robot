# Calibration backup

Example PWM table for this robot: [`servo_cal.json`](servo_cal.json).

| Task | Doc |
|------|-----|
| Calibrate a new build | [../docs/CALIBRATION.md](../docs/CALIBRATION.md) |
| Joint angles & PWM math | [../pca_servo_test/JOINT_CONVENTION.md](../pca_servo_test/JOINT_CONVENTION.md) |

**Restore on device:** calibrator → **Restore snapshot**, or auto-load on empty NVS.

**Refresh backup after re-cal:** download JSON from robot → replace `servo_cal.json` → `python gen_cal_backup.py` in `pca_servo_test/` → re-flash.
