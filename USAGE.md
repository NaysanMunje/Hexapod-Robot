# Using the hexapod

This guide takes you from parts on the bench to a walking robot. If something fails, see [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md).

---

## What you need

| Item | Notes |
|------|--------|
| **ESP32-S3 DevKitC-1** | Firmware target in `pca_servo_test/platformio.ini` |
| **18× MG996R-class servos** | One hip, thigh, shin per leg × 6 legs |
| **2× PCA9685** | Addresses **0x40** (default) and **0x41** (A0 bridged to V+) |
| **5 V supply for servos** | Separate from USB; common ground with ESP32 |
| **I²C wiring** | SDA → **GPIO6**, SCL → **GPIO5** |
| **WiFi** | 2.4 GHz network the ESP can join |
| **PC** | [PlatformIO](https://platformio.org/) (VS Code extension or CLI) |

Leg naming on the web UI: **FL, ML, RL** (left) and **FR, MR, RR** (right). Forward is **+X** on the body.

---

## Hardware

### PCA9685 addresses

| Board | I²C address | Channels |
|-------|-------------|----------|
| PCA #0 | `0x40` | UI channels 1–16 |
| PCA #1 | `0x41` (A0 jumpered) | UI channels 17–32 |

On boot the serial log must show `PCA0 0x40 ready` and `PCA1 0x41 ready`. If either is **missing/fail**, fix wiring and power before continuing.

### Servo channel map

Each joint has a **UI channel number** (1–32) used on the calibrator page. The full map is in [`pca_servo_test/servo_map.json`](pca_servo_test/servo_map.json). Example:

| Leg | Hip | Thigh | Shin |
|-----|-----|-------|------|
| FL | 12 | 8 | 22 |
| ML | 11 | 7 | 21 |
| RL | 10 | 5 | 17 |
| FR | 23 | 19 | 13 |
| MR | 29 | 28 | 9 |
| RR | 26 | 18 | 1 |

If your wiring differs, update `servo_map.json` **and** the `JOINTS[]` table in `src/main.cpp` together (same project; that file is the firmware source of truth for channel numbers).

### Power and safety

- **Never** drive servos from ESP32 3.3 V pins — use the PCA9685 with a proper 5 V servo supply.
- Start calibration with the robot **off the ground** or legs free to move.
- Keep fingers clear when pressing **Start robot** or running **Go MAX/MIN** on the calibrator.
- Stop with **Stop robot** or power off if a joint buzzes against a hard stop.

---

## Software setup

### 1. WiFi credentials

```bash
cd pca_servo_test
```

Copy the example secrets file:

- **Linux / macOS:** `cp include/wifi_secrets.h.example include/wifi_secrets.h`
- **Windows (PowerShell):** `Copy-Item include/wifi_secrets.h.example include/wifi_secrets.h`

Edit `include/wifi_secrets.h` and set `WIFI_SSID` and `WIFI_PASSWORD`.

### 2. Serial port

Default upload port is **COM4** in `platformio.ini`. Change `upload_port` and `monitor_port` to match your board (e.g. `COM3`, `/dev/ttyUSB0`).

Find the port: Device Manager (Windows), `pio device list`, or the PlatformIO toolbar.

### 3. Flash firmware

```bash
pio run -t upload
pio device monitor
```

At 115200 baud you should see:

1. I²C scan finding `0x40` and `0x41`
2. PCA ready messages
3. WiFi connecting, then **`Open http://192.168.x.x`**

Connect your phone or PC to the **same WiFi** and open that URL.

### 4. Pre-loaded calibration (this repo’s robot)

If your hardware matches this build, firmware may already work: on **empty NVS** it loads [`calibration/servo_cal.json`](calibration/servo_cal.json) automatically.

**Check:** calibrator page (`http://<ip>/`) should show **18/18 calibrated**. If counts are lower or motion looks wrong, follow **[docs/CALIBRATION.md](docs/CALIBRATION.md)**.

---

## Web pages

| URL | Use |
|-----|-----|
| `http://<ip>/` | **Calibration** — set DEFAULT/MAX, direction, lock spans |
| `http://<ip>/walk` | **Gaits** — preview, deploy params, start/stop robot |

The walk page loads Three.js from a CDN when you open **Settings** → 3D preview. The browser needs **internet** for that panel only; tabs and robot control work offline once the page is loaded.

---

## Walk page (`/walk`)

### Layout

- **Top tabs:** Walk · Ripple · Wave · Spin · Stretch · Pitch · Crouch · Twist · Roll · Nod · Swirl · Rotate · Control
- **Settings** (overlay): sliders, 3D preview, **Deploy to robot**, **Freeze hips**
- **Start robot / Stop robot** on the main screen (except Control tab, which uses hold-to-move)

### Important behavior

| Action | What it does |
|--------|----------------|
| Move sliders | Updates **preview** (and 3D if Settings open) only |
| **Deploy to robot** | Sends current sliders to the ESP; required before walking |
| **Start robot** | Runs the active tab’s motion on hardware |
| **Stop robot** | Stops gait; parks legs (hips default, shins mid, thighs 75% toward min) |
| **Freeze hips** | Coxa stay at default; thighs/shins still walk (good first setting) |

Sliders do **not** live-update the robot until you deploy.

### Recommended first walk

1. Tab: **Walk**
2. Settings: leave defaults if unsure — **freq 0.8 Hz**, **stride 60 mm**, **lift 30 mm**, **height 95 mm**, **radius 140 mm**, **splay 22°**, **Freeze hips** on
3. **Deploy to robot**
4. Lift robot clear of the table → **Start robot**
5. If motion is smooth, try lowering stride or freq; if feet drag, increase lift slightly
6. **Stop robot** when done

### Tabs (short)

| Tab | Motion |
|-----|--------|
| **Walk** | Tripod gait (3+3), 50% duty — default forward walk |
| **Ripple** | One leg at a time, alternating sides rear→front |
| **Wave** | One leg at a time clockwise around the body |
| **Spin** | In-place yaw; set **turn rate** in Settings; stride should be 0 |
| **Stretch** | Feet planted · lean left ↔ right (**Start robot**) |
| **Pitch** | Feet planted · lean forward ↔ back |
| **Crouch** | Feet planted · squat up ↔ down |
| **Twist** | Feet planted · yaw left ↔ right (uncheck Freeze hips) |
| **Roll** | Feet planted · chassis tilts left ↔ right (~14°) |
| **Nod** | Feet planted · chassis tips nose ↔ tail (~14°) |
| **Swirl** | Feet planted · chassis tilts while circling (most motion) |
| **Rotate** | Feet planted · lean in a circle (body stays level) |
| **Control** | Hold D-pad to walk; hold **↺ CCW** / **↻ CW** to spin; release to stop |

Switching away from a locomotion tab **stops** the robot if it was running.

### Spin and Control

- **Spin tab:** deploy with **turn rate** (default 20°/s), **stride 0**, then **Start robot**
- **Control tab:** turn rate comes from Settings; hold spin buttons while deployed

Keep turn rate modest (~15–25°/s) until you confirm hips are not saturating.

---

## Calibration (summary)

Full procedure: **[docs/CALIBRATION.md](docs/CALIBRATION.md)**.

Quick rules:

1. **Hip:** mechanical default → **Save DEFAULT** → set direction (+CCW from above)
2. **Thigh / shin:** folded hard stop → **Save MAX** → set direction (+pulse opens toward straight)
3. After all 12 thighs and 12 shins have MAX saved: **Lock thighs/shins to shortest max→min span**
4. Use **Go MIN / Go MID / Go MAX** on each joint to verify before walking

Reference: **[pca_servo_test/JOINT_CONVENTION.md](pca_servo_test/JOINT_CONVENTION.md)**.

---

## Offline simulation

Open [`hexapod_description/view_hexapod.html`](hexapod_description/view_hexapod.html) locally. Tune gaits and clearances without hardware. Defaults match the firmware walk page.

Other viewers: `view_leg.html` (single leg), `view_body.html` (chassis).

---

## Maintainers: editing the walk UI

After changing `pca_servo_test/www/walk.html`:

```bash
cd pca_servo_test
python gen_walk_html.py
pio run -t upload
```

After re-calibrating and downloading JSON from the robot:

```bash
# save to calibration/servo_cal.json, then:
python gen_cal_backup.py
pio run -t upload
```

See [calibration/README.md](calibration/README.md).

---

## Checklist before first walk

- [ ] Both PCA boards detected on serial boot
- [ ] Calibrator shows 18/18 (or calibration finished per CALIBRATION.md)
- [ ] **Lock spans** run for thighs and shins
- [ ] **Deploy to robot** clicked after slider changes
- [ ] Robot supported or clear of obstacles
- [ ] **Stop robot** tested

Done — use [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) if anything misbehaves.
