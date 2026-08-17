# Hexapod

18-DOF 3D-printed hexapod: CAD-derived stick kinematics, ESP32-S3 firmware, web calibration, and a tripod gait with a live 3D preview.

Robot frame is **+X forward, +Y left, +Z up**. All six legs use the same joint signs; left/right servo mirrors are handled only by per-joint direction flags in calibration.

## Hardware

| Item | Detail |
|------|--------|
| MCU | ESP32-S3 DevKitC-1 |
| Servos | 18× MG996R-class |
| Drivers | 2× PCA9685 — `0x40` (default) and `0x41` (A0 bridged) |
| I²C | SDA = GPIO6, SCL = GPIO5 |
| PWM | 50 Hz, tick window ~80–520 |

Servo wiring (UI channel numbers) is in [`pca_servo_test/servo_map.json`](pca_servo_test/servo_map.json).

## Repo layout

```
hexapod_description/     Stick / URDF model + browser viewers
pca_servo_test/          PlatformIO firmware (calibrator + walk + 3D preview)
calibration/             Servo PWM snapshot dumped from the robot (2026-08-17)
```

## Kinematics / simulation (no robot needed)

Open in a browser (needs internet for Three.js):

- [`hexapod_description/view_hexapod.html`](hexapod_description/view_hexapod.html) — full robot + tripod gait
- [`hexapod_description/view_leg.html`](hexapod_description/view_leg.html) — one leg
- [`hexapod_description/view_body.html`](hexapod_description/view_body.html) — body plate + hips

Link lengths, hip `yaw0`, and joint limits: [`hexapod_description/README.md`](hexapod_description/README.md), [`hexapod_description/hips.json`](hexapod_description/hips.json).  
Joint signs used by both the sim and firmware: [`pca_servo_test/JOINT_CONVENTION.md`](pca_servo_test/JOINT_CONVENTION.md).

## Firmware

Requires [PlatformIO](https://platformio.org/).

```bash
cd pca_servo_test
cp include/wifi_secrets.h.example include/wifi_secrets.h
# edit SSID / password
pio run -t upload
```

Default serial port is `COM4` (`platformio.ini`). After boot the serial log prints the station IP.

| Page | URL |
|------|-----|
| Calibration | `http://<ip>/` |
| Walk + 3D preview | `http://<ip>/walk` |

The `/walk` page loads Three.js from a CDN, so the browser needs internet. **Sliders only update the preview** until you click **Deploy to robot**. **Freeze hips** holds coxa servos at their calibrated default.

If NVS is empty (new chip / wiped flash), firmware loads [`calibration/servo_cal.json`](calibration/servo_cal.json) automatically. The calibrator page can also **Download calibration JSON** or **Restore snapshot from firmware backup**.

More detail: [`pca_servo_test/README.md`](pca_servo_test/README.md).

## Calibration backup

Live PWM table from this robot (2026-08-17):

- [`calibration/servo_cal.json`](calibration/servo_cal.json) — human-readable snapshot
- [`pca_servo_test/include/calibration_backup.h`](pca_servo_test/include/calibration_backup.h) — compiled copy

How to refresh after re-calibrating: [`calibration/README.md`](calibration/README.md).

## GitHub

`pca_servo_test/include/wifi_secrets.h` is gitignored. Copy the `.example` file locally and never commit real WiFi passwords.

`.pio/` build artifacts are also ignored.
