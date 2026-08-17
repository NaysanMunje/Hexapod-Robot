# Firmware (`pca_servo_test`)

PlatformIO project for the ESP32-S3: I²C PWM, web calibrator, tripod gait, and the same 3D stick preview as `hexapod_description/view_hexapod.html`.

## Build

```bash
cp include/wifi_secrets.h.example include/wifi_secrets.h
pio run -t upload          # port: COM4 in platformio.ini
pio device monitor
```

After WiFi connects, serial prints `Open http://<ip>`.

## Files

| Path | Role |
|------|------|
| `src/main.cpp` | PCA9685, NVS calibration, HTTP UI/API, angle→PWM |
| `src/walk.cpp` | Tripod gait + IK |
| `include/walk.h` | Gait parameters / API |
| `include/calibration_backup.h` | Compiled PWM snapshot (generated) |
| `include/gait_preview_html.h` | `/walk` page (generated from `www/walk.html`) |
| `www/walk.html` | Editable walk UI + Three.js preview |
| `servo_map.json` | UI channel → PCA board/channel |
| `JOINT_CONVENTION.md` | Model joint zeros, signs, PWM mapping |
| `gen_walk_html.py` | Rebuild `gait_preview_html.h` after editing `www/walk.html` |
| `gen_cal_backup.py` | Rebuild `calibration_backup.h` from `../calibration/servo_cal.json` |

## HTTP API

Calibration (`/`):

| Path | Action |
|------|--------|
| `GET /api/status` | PCA presence, IP, calibrated count |
| `GET /api/cal` | Full JSON (same shape as `calibration/servo_cal.json` joints) |
| `GET /api/pulse?ch=&pulse=` | Raw PCA tick (global channel = UI − 1) |
| `GET /api/cal/save?i=&pulse=` | Save hip DEFAULT or thigh/shin MAX |
| `GET /api/cal/dir?i=&dir=` | `+1` = higher PWM toward model MIN |
| `GET /api/cal/goto?i=&which=` | `max` / `mid` / `min` (hips: `mid` only) |
| `GET /api/cal/all?which=` | All joints |
| `GET /api/cal/group?joint=&which=` | `thigh` or `shin` |
| `GET /api/cal/lock` | Lock thigh/shin span to shortest available |
| `GET /api/cal/restore` | Overwrite NVS with the committed snapshot |

Walk (`/walk`):

| Path | Action |
|------|--------|
| `GET /api/walk/get` | Current params JSON |
| `GET /api/walk/params?...` | Set freq, stride (mm), lift, height, radius, splay, crab, turn, freezeHips |
| `GET /api/walk/toggle` | Start / stop (stop parks at mid/default) |
| `GET /api/walk/status` | Walking flag, phase, unreachable feet |

On boot the serial log also prints `CAL_JSON_BEGIN` … `CAL_JSON_END`.

## PWM mapping

See `JOINT_CONVENTION.md`. Short version:

- **Hip:** `pulse(θ°) = ref − dir × ticks_per_deg × θ` with `ref` = default
- **Thigh/shin:** `pulse(θ°) = ref + dir × span × (1 − θ/θmax)` with `ref` = MAX

`span` is the locked shortest max→min (thigh 226, shin 203 on the 2026-08-17 snapshot).

## Regenerating generated headers

```bash
python gen_walk_html.py
python gen_cal_backup.py
```
