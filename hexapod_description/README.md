# Hexapod leg description (stick / URDF)

## One-leg dimensions (from your CAD)

| Segment | Length | Cross-section |
|---------|--------|----------------|
| Hip / coxa | **53.00 mm** | **10 mm** square |
| Thigh / femur | **77.36 mm** | **30 mm** square |
| Shin part 1 | **22.358 mm** | plate, **70 mm** wide at the knee |
| Shin part 2 | **71.518 mm** (interior angle with part1 = **167.2°**) | tapers toward the foot |
| Shin part 3 | **31.14 mm** (interior angle with part2 = **159.9°**) | tapers to a **point** |

### Joint limits (CAD max pose)
Angles are **interior angles between segments**, same convention as the 167.2° / 159.9° labels in the sketch.

| Joint | Meaning | Range |
|-------|---------|--------|
| Thigh | interior angle with hip | **77.5°** (hard stop) … **180°** (straight) |
| Shin | interior angle with thigh | **57.6°** (hard stop) … **180°** (straight) |

At the hard stops the thigh sits 102.5° up from the hip and the shin folds 122.4° back from the thigh, which reproduces the drawing.

**IK tibia (knee→foot straight):** ≈ **122.7 mm**
The three shin segments collapse to one effective link of **122.70 mm** at **−15.49°** from shin part 1, which is what the IK solves against.

## Hip mounting
All six hips sit at hexagon **vertices**. Each hip's coxa zero (`yaw0`) is the mechanical default, **not** radial from the body center:

| Leg | yaw0 | Default direction |
|-----|------|-------------------|
| LM / RM | ±90° | Horizontal (±Y) |
| LF / RF | ±66.528° | ⊥ outward to the adjacent 79.893 mm side |
| LR / RR | ±113.472° | ⊥ outward to the adjacent 79.893 mm side |

Legs are **not** mirrored in the model — every leg uses the same joint axes and signs, so one IK routine serves all six. The right side's mirrored servo mounting is handled as a per-joint direction flag in the firmware calibration table.

## Gait
- [`view_hexapod.html`](view_hexapod.html) — **menu** (links to walk & spin)
- [`view_walk.html`](view_walk.html) — **forward / crab walk** preview
- [`view_spin.html`](view_spin.html) — **in-place spin** preview

Both gait pages run a **tripod gait** (LF·RM·LR / RF·LM·RR, 50% duty) driven by real IK: foot targets are placed in the body frame, then solved for coxa yaw, femur, and knee.

- Corner legs get a **forward/outward splay** (default **22°**) so the shins (70 mm wide at the knee, tapering to a point) stay clear of the middle pair (target ≥20 mm free air). The viewer reports live clearance at the wide knee end.
- Body speed is derived from stride and cadence for **no-slip** feet: `v = stride × cadence / duty`.
- **In-place spin:** set stride to 0 and yaw rate ≠ 0. Each foot sweeps tangentially (`v = ω × r` at its footprint); coxa yaw does most of the work. Keep |ω| modest (~20 °/s) to stay inside hip ±45°.
- **Walk + yaw:** both stride and turn nonzero → the body follows an arc (radius `v/ω`).
- Slider ranges are limited to poses that stay off the hard stops. Defaults (95 mm body height, 140 mm foot radius, 60 mm stride, 30 mm lift) keep every joint **≥17° from its limit**; the status readout names the tightest joint at all times.

## Files
- [`urdf/leg_one.urdf`](urdf/leg_one.urdf) — single leg
- [`urdf/body.urdf`](urdf/body.urdf) — body + hip frames
- [`urdf/hexapod.urdf`](urdf/hexapod.urdf) — **full body + 6 legs**
- [`_gen_hexapod_urdf.py`](_gen_hexapod_urdf.py) — regenerates `hexapod.urdf`
- [`view_leg.html`](view_leg.html) — one leg
- [`view_body.html`](view_body.html) — body only
- [`view_hexapod.html`](view_hexapod.html) — gait menu
- [`view_walk.html`](view_walk.html) — full robot walk animation
- [`view_spin.html`](view_spin.html) — full robot spin animation
- [`hips.json`](hips.json) — hip XYZ + coxa `yaw0` at vertices

## View (ROS 2 example)
```bash
# after sourcing ROS
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro urdf/leg_one.urdf)"
# + joint_state_publisher_gui + rviz2
```

Or open `leg_one.urdf` in a URDF viewer / VS Code URDF extension.
