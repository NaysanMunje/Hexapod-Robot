# Hexapod

18-servo hexapod with ESP32-S3 firmware, web calibration, and a browser gait UI. Same stick model in simulation and on the robot.

**New here?** Read **[USAGE.md](USAGE.md)** end to end, then calibrate with **[docs/CALIBRATION.md](docs/CALIBRATION.md)** if your servos are not already matched to this build.

## Quick start

1. Wire **2× PCA9685** (`0x40`, `0x41`) and **18 servos** — see [Hardware](#hardware) in [USAGE.md](USAGE.md).
2. `cd pca_servo_test` → copy `include/wifi_secrets.h.example` to `wifi_secrets.h` → set SSID/password → `pio run -t upload`.
3. Serial monitor shows `Open http://<ip>` → open **`/walk`** → **Deploy to robot** → **Start robot** on the Walk tab.

Use **`/`** (calibrator root) only for first-time or post-rebuild **calibration**.

## Documentation

| Doc | When to read it |
|-----|------------------|
| **[USAGE.md](USAGE.md)** | Setup, wiring, flashing, walk UI, safe first steps |
| **[docs/CALIBRATION.md](docs/CALIBRATION.md)** | Step-by-step servo calibration (required on a new build) |
| **[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)** | WiFi, I²C, wrong leg moves, gait issues |
| **[pca_servo_test/JOINT_CONVENTION.md](pca_servo_test/JOINT_CONVENTION.md)** | Joint angles, signs, PWM mapping (reference while calibrating) |
| **[pca_servo_test/servo_map.json](pca_servo_test/servo_map.json)** | Which UI channel is which leg/joint |
| **[calibration/servo_cal.json](calibration/servo_cal.json)** | Example PWM snapshot (auto-loaded if NVS is empty) |

## Repo layout

| Folder | Purpose |
|--------|---------|
| `pca_servo_test/` | PlatformIO firmware |
| `hexapod_description/` | URDF + offline HTML preview (`view_hexapod.html`) |
| `calibration/` | Committed calibration backup |

## Simulation (no robot)

Open [`hexapod_description/view_hexapod.html`](hexapod_description/view_hexapod.html) in a browser (needs internet for Three.js). Same tabs as `/walk`; nothing moves on hardware.

## Secrets

`pca_servo_test/include/wifi_secrets.h` is gitignored. Never commit real WiFi passwords.
