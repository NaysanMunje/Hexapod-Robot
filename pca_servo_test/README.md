# Firmware (`pca_servo_test`)

| Task | Doc |
|------|-----|
| Flash, walk UI, first steps | [../USAGE.md](../USAGE.md) |
| Calibrate servos | [../docs/CALIBRATION.md](../docs/CALIBRATION.md) |
| Fix problems | [../docs/TROUBLESHOOTING.md](../docs/TROUBLESHOOTING.md) |
| Joint convention | [JOINT_CONVENTION.md](JOINT_CONVENTION.md) |
| Channel wiring | [servo_map.json](servo_map.json) |

```bash
cp include/wifi_secrets.h.example include/wifi_secrets.h   # then edit WiFi
pio run -t upload
```

After editing `www/walk.html`: `python gen_walk_html.py` then re-flash.
