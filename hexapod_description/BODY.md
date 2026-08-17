# Hexapod body (center plate)

## Dimensions (mm)
| Feature | Value |
|---------|--------|
| Top / bottom edges | **123.215** |
| Other four edges | **79.893** each |
| Top–bottom (flat-to-flat) | **146.564** |
| Left–right tip-to-tip | **186.859** (derived) |

Opposite sides parallel; left/right symmetric.

## Hip mounts = **vertices** (mm)
Frame: +X forward, +Y left.

| Leg | X | Y |
|-----|--------|--------|
| **LF** | 73.282 | 61.608 |
| **LM** | 0 | 93.430 |
| **LR** | -73.282 | 61.608 |
| **RF** | 73.282 | -61.608 |
| **RM** | 0 | -93.430 |
| **RR** | -73.282 | -61.608 |

Orientation of each coxa (`yaw0`) is the **mount default**, not radial from center: LM/RM along ±Y; corners perpendicular outward to the adjacent 79.893 mm side. Numbers are in [`hips.json`](hips.json).

## Files
- [`urdf/body.urdf`](urdf/body.urdf)
- [`view_body.html`](view_body.html)
- [`hips.json`](hips.json)
