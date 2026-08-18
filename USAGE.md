# Using the hexapod

## First-time setup

1. Install [PlatformIO](https://platformio.org/).
2. Copy WiFi credentials:
   ```bash
   cd pca_servo_test
   cp include/wifi_secrets.h.example include/wifi_secrets.h
   ```
3. Flash firmware (default port `COM4` in `platformio.ini`):
   ```bash
   pio run -t upload
   ```
4. Open the serial monitor — note the IP after WiFi connects.

Power on the PCA9685 boards (I²C on GPIO5/GPIO6) before expecting servos to move.

## Web pages

| URL | Purpose |
|-----|---------|
| `http://<ip>/` | Servo calibration (first build or after hardware changes) |
| `http://<ip>/walk` | Gaits, live control, optional 3D preview |

The walk page needs internet (Three.js CDN) for the 3D view in Settings.

## Walk page — quick guide

**Tabs:** Walk · Ripple · Wave · Spin · Stretch · Rotate · Control

1. Pick a tab (gait or motion type).
2. Open **Settings** if you want to change speed, stride, lift, body height, etc., or see the 3D preview.
3. Tap **Deploy to robot** after changing sliders (preview-only until deployed).
4. Press **Start robot** on the main screen to run. **Stop robot** parks the legs (thighs partly lowered, shins mid, hips default).

**Control tab:** hold the D-pad to walk; hold **↺ CCW** or **↻ CW** to spin in place. Release to stop that motion.

**Stretch / Rotate:** only run when **Start robot** is pressed on those tabs (they do not auto-start when you switch tabs).

**Freeze hips** (Settings): keeps coxa at default while thighs/shins still move — useful for tripod walking.

## Simulation only (no hardware)

Open [`hexapod_description/view_hexapod.html`](hexapod_description/view_hexapod.html) in a browser. Same tabs and sliders; nothing is sent to the robot.

## Calibration

Use `http://<ip>/` when joints were rebuilt or PWM limits changed. See [`pca_servo_test/JOINT_CONVENTION.md`](pca_servo_test/JOINT_CONVENTION.md).

If flash/NVS is empty, firmware loads the committed snapshot in [`calibration/servo_cal.json`](calibration/servo_cal.json) automatically.

## After editing the walk UI

```bash
cd pca_servo_test
python gen_walk_html.py
pio run -t upload
```
