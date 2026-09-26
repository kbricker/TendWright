# Mini modular conveyor — build guide

Hive plan **#835** (v0 rig) · **#840** (v1 loop)

v0 is two straights and one tapered-roller curve, three motors, an open line. s1 travels +X, the curve turns left, s2 travels +Y. Each straight's motor is on its left plate. Nothing has been printed yet.

Shopping list: [`docs/conveyor/conveyor-order-2026-09-25.md`](../../docs/conveyor/conveyor-order-2026-09-25.md).

---

## Regenerating

```
"%LOCALAPPDATA%/Programs/FreeCAD 1.1/bin/freecadcmd.exe" cad/conveyor/build_parts.py
uv run python cad/conveyor/render.py
uv run python cad/conveyor/sim_conveyor.py
uv run python cad/conveyor/sim_conveyor.py --sweep
uv run python cad/conveyor/sim_conveyor.py --view
```

`build_parts.py` is the only place a dimension is declared. It writes `parts/*.stl`, `parts/*.step`, `build.log`, and `parts/geometry.json`. The sim reads that file and declares none of its own module dimensions. The payload (32 × 32 × 16 mm, 30 g) is the sim's spec.

`freecadcmd` swallows stdout and returns 0 even when the script raises. The run succeeded if `build.log` ends with `=== build complete ===`.

Sim flags:

- `--speed` — straight belt speed, m/s. Default 0.155, which is 270 RPM at the Ø11 neutral axis, the speed the cone centreline is cut to match.
- `--curve-speed` — centreline speed of the curve, m/s. Default: same as `--speed`.
- `--mu` — belt friction, TPU on the part. Default 0.9.
- `--mu-curve` — cone friction, PLA+ on the part. Default 0.35.
- `--offset` — entry offset, mm, toward the inside of the turn. Default 0. −8 is the outer edge.
- `--seconds` — replaces the acceptance time limit. A cap that misses the exit is a failure.
- `--frames` — nominal-run PNGs in `renders/sim/`. Default 6. `--sweep` draws none.
- `--sweep` — the acceptance matrix. Writes `renders/sim/sweep.json`. Exit 0 only if every run passes and the spreads hold.
- `--view` — interactive viewer. The only mode that puts the part back on the belt.

### FreeCAD traps

From [`CableCell/cad/README.md`](../../../CableCell/cad/README.md):

1. `Shape.translate()` mutates in place and returns `None`.
2. `freecadcmd` sets `__name__` to the module basename, so a `__main__` guard never fires. There isn't one.
3. Routing an STL through a `Mesh::Feature` crashes the process. Meshes go out via `Mesh.Mesh(shape.tessellate(dev)).write(path)`.

---

## Print the coupons first

A coupon is twenty minutes. A wrong bore in the full set is a day. Print these, caliper them, then rebuild before printing a module.

**Plate 1**

| file | with | material | what it is for |
|---|---|---|---|
| `coupon_bracket_end.stl` | `roller_driven.stl`, `roller_idler.stl` | PETG plate, PLA+ rollers | enclosure ears on the motor plate, and the roller bores |
| `coupon_curve.stl` | `roller_cone_driven.stl`, `roller_cone_idler.stl`, one O-ring on groove A | PETG sector, PLA+ cones | both ear holes and their nut pockets, and the O-ring in the groove |
| `coupon_infeed_end.stl` | one `tensioner_block.stl` | PETG | the block must slide on the rail, and the jack boss must take two M3 nuts |

The infeed coupon is the plate and the block side by side on the bed, with a gap, so the slide can actually be tried. Nested on the rail, the block would print in mid-air.

**What to caliper, and the parameter it feeds**

- The O-ring you pick: inside diameter → `oring_id`, cross-section → `oring_cs`. The grooves are cut for a placeholder (build.log: ID window 18.971–22.541 mm at a 2.00 mm section). The chart on the kit is not the measurement.
- The enclosure, face of the ear plate to the gearbox face → `encl_face_to_gearbox`. The model uses 2.5 mm, which is the worst-case reading of the drawing. The shaft engagement in the log (6.300 mm) moves if this does.
- Then rerun `build_parts.py`. The D-bore, the Ø4.4 roller bore and the take-up slot confirm the machine's hole offset; they are not there to discover it. Bores are modelled +0.15 mm on radius, because this machine prints holes about that much undersize.

**Plate 2 — belt test**, once TPU arrives. One straight belt: a cylinder **Ø79.8 mean × 50 mm tall × 1.0 mm wall**, standing upright. That diameter is `straight.print_cyl_dia` in `geometry.json`, the neutral-axis length divided by π. The 1.0 mm wall is what keeps belt-thickness / pulley-diameter at 10 on a Ø10 roller; 1.5 mm would fight the wrap.

Rollers and cones on these plates print in the orientation in the table below.

---

## Full part set — v0

v1 counts belong to #840. PETG where the part is loaded and shares holes with the plates. PLA+ where the fit against the belt or the shaft is the point, so a different shrink does not open the bore the coupon just checked.

| file | qty | material | orientation |
|---|---|---|---|
| `bracket_straight_motor.stl` | 2 | PETG | flat |
| `bracket_straight_plain.stl` | 2 | PETG | flat |
| `roller_driven.stl` | 2 | PLA+ | **axis vertical**, brim |
| `roller_idler.stl` | 2 | PLA+ | **axis vertical**, brim |
| `roller_cone_driven.stl` | 1 | PLA+ | **big end down**, brim |
| `roller_cone_idler.stl` | 5 | PLA+ | **big end down**, brim |
| `slider_bed_straight.stl` | 2 | PLA+ | belt face up |
| `return_guide_straight.stl` | 2 | PLA+ | belt face up |
| `tie_bar.stl` | 4 | PETG | joiner-nut pockets up |
| `tensioner_block.stl` | 4 | PETG | underside down |
| `joiner.stl` | 2 | PETG | flat, holes vertical |
| `curve_frame.stl` | 1 | PETG | base down |
| `curve_keeper.stl` | 1 | PETG | as exported, base down |
| straight belt cylinder | 2 | **TPU 95A** | upright |

The curve has no belt. Five O-rings, from the kit, link its rollers.

### Why the rollers print vertical

The D-flat is the only thing transmitting drive torque. Axis-vertical, the layers are discs and the flat's load is circumferential, in the plane of the layers. Printed on its side, that load is interlayer adhesion.

### Why the cones print big end down

The spool and the driven spigot sit outboard of the big end. On the bed, that end is the base and the cone narrows as it rises, so the taper is not an overhang. The D-flat still bears in the plane of the layers. The STL is already in that orientation; the placed rollers in the assembly are shaved flush to the frame faces, and the print file is the unshaved one.

### Bed and return guide

Both are held by a closed groove in each plate. The tongue is captured in X and Z by the first plate; the second plate closes Y. Belt drag pushes the bed toward the discharge, and the discharge wall of the groove is what stops it. The guide uses the same grooves, so it does not compete with the joiner for the top of the tie bar. Its top is square: the two long edges sit outside the belt, so there is nothing there to crown. It stays 0.5 mm under the taut return run (`return_guide_top` 18.5 mm, `return_run_z` 19 mm).

### Tie bars, tensioners, joiners

- Two tie bars per straight, one within 25 mm of each end. They set the plates at `inner_width` and keep the module square. M3 through the plate into a captive nut in the upright. The top face is the joint plane the joiner sits on. Print them pockets-up so the joiner nuts drop in; the plate-nut pockets stay horizontal holes.
- A tensioner block on each plate's outer face, at the infeed. It slides on a rail and carries the idler rod. An M3×16 through two captive nuts in a fixed boss pushes the block toward the module face. Belt tension keeps the block on the screw tip. The outer end of the slot is the hard stop: at full take-up the idler axis is at `nose_edge`, which is the design span.
- One joiner part for both joints. The two end tie bars mirror about the module centre, so the same plate sets the 1.5 mm frame gap at J1 (s1 → curve) and J2 (curve → s2). M3 down into nuts pocketed from below, so nothing hangs under the table. There is no straight-to-straight joint in v0.

---

## Hardware

From `geometry.json` → `hardware`. This is s1 + curve + s2 + the two joiners.

| | size | length | qty | where |
|---|---|---|---|---|
| screw | M3 | 8 mm | 8 | tie bars |
| screw | M3 | 16 mm | 4 | tensioner jacks |
| screw | M3 | 12 mm | 8 | joiners |
| screw | M3 | 10 mm | 2 | curve keeper |
| screw | M4 | 8 mm | 6 | motor ears |
| nut | M3 | — | 26 | 8 tie + 8 jack (two per screw) + 8 joiner + 2 keeper |
| nut | M4 | — | 6 | motor ears |
| rod | Ø4 | 73.0 mm | 2 | straight idler |
| rod | Ø4 | 51.3 mm | 2 | straight driven stub |
| rod | Ø3 | 70.358 mm | 5 | curve idler |
| rod | Ø3 | 24.239 mm | 1 | curve driven stub |

The curve idler's cut length is face to face. The solid in the interference check stops 0.4 mm short, so a square end on the tilted axis does not enter the keeper.

**O-rings.** Five, alternating grooves A and B, three of A and two of B (`curve.n` is 6). From `build.log`, for the placeholder section of 2.00 mm:

| groove | pitch c | spool D | crown clearance |
|---|---|---|---|
| A | 22.587 mm | 9.821 mm | 2.398 mm |
| B | 23.669 mm | 9.132 mm | 3.140 mm |

Replace these by calipering the ring and rebuilding. Do not order a printed drive ring.

---

## Key dimensions

From `geometry.json` and `build.log`.

| | |
|---|---|
| Belt | 50 mm wide, 1.0 mm wall, carry surface z = 30 mm, belt top 31 mm |
| Straights | 120 mm, nose axis 6 mm in from each face |
| Frame gap | 1.5 mm, both joints |
| Transfer span, both joints | inner 10.5 mm, centreline 13.0 mm, outer 15.5 mm |
| Taper | k = 0.200, so cone diameter = 0.2 × plan radius |
| Rollers on the curve | 6, pitch 15.704°, driven roller is the middle one |
| Axle tilt | 5.739° down toward the outside, which is what keeps the crown at z = 31 |
| Curve centreline radius | 55 mm |
| Shaft engagement | 6.300 mm into a 10 mm D-bore |
| Straight belt | path 250.6 mm at the neutral axis, printed mean Ø 79.8 mm |
| Jack-screw thread in the nuts | 5.20 mm at both ends of the 8 mm travel |

The outer span is the long one. A part entered on the outer edge has 15.5 mm of nothing at each joint, which is what the sim's −8 mm offset is aimed at.

---

## Assembly

The belt goes on from one side, with one plate off. Nothing in the loop except the rollers and the bed. Tie bars and the return guide are below it. Tensioners and the motor are outboard of the plates.

### A straight

1. **Captive nuts, then the motor enclosure, on the left plate.** Tie-bar nuts go into the uprights from the plate face. Two M3 nuts go into the jack boss; the pocket opens toward the block, which is not on yet. M4 nuts go into the ear bosses. The enclosure bolts on through those ears.
2. **Rollers, bed, and both tie bars on that plate.** The bed's tongues drop into the grooves from the open side. One plate then holds the bed in X and Z. The tie bars bolt through the plate and set the width.
3. **Belt, from the open side,** over both rollers and the bed as one loop.
4. **Return guide, under the return run,** into its own grooves. It is outside the loop, so it can follow the belt. It still has to be in before the second plate, because that plate closes the groove.
5. **Driven stub, then the second plate.** The stub drops into the roller's far bore and stops 0.5 mm short of the D-bore step. The plain plate's bore is blind, with the floor in a boss on the outer face, so the stub cannot walk out through the plate and cannot walk into the D-bore either.
6. **Tensioners.** Block on the rail, jack screw through the two nuts, taken up until the idler rod is against the outer end of the slot.

### The curve

Rods first, then the rollers on them, then the O-rings over the spools (groove A, B, A, B, A), then the keeper over the outer rod ends, then the motor on the outer wall. The keeper is what stops the idler rods walking out. The driven stub is the short Ø3 rod.

### The joints

A joiner on s1's discharge tie bar and the curve's entry pad. The same part on the curve's exit pad and s2's infeed tie bar. Screws go down into the nuts that were pocketed from below. That sets the frame gap and lines the lanes up. Bolt the joints after both modules are assembled; the joiner is the last part, not a fixture the modules are built on.

---

## Wiring

Three motors for v0. The order list has the part numbers.

- **TB6612FNG** drivers. Not a DRV8833: that tops out at 10.8 V and cannot run these 12 V motors. v0 uses two of the four boards in the order; each board has two channels.
- **Bench supply, set to 12 V,** is the motor rail (`VM`). Set the rear switch to 115 V before first use, set voltage and a low current limit with the output off, then turn the output on. A low limit trips on a wiring mistake instead of killing a driver.
- **Pico 2 W** logic. The Pico runs from USB. Tie the supply's **− terminal to the Pico's GND**, not the green earth post. The driver splits VM from VCC, and it only works if they share a return.
- **Gear Motor Input Board** soldered onto each motor's spade tabs, then a **female/male jumper** from the driver channel to that header. That is the connector. A motor comes off without the iron.
- Size the supply from the measured stall current, which the bench supply's display gives you. The listing's 1.6 A per motor is the number to beat, not the one to trust.

Firmware is in [`hardware/conveyor/`](../../hardware/conveyor/README.md). `uv run python -m hardware.conveyor.selftest` runs it without a Pico.

- Commands are absolute (`set duty=X`). A duplicate is harmless; a delta is not.
- Duty is checked 0–100 % at the firmware boundary and rejected past that, not clamped. Clamping 150 to 100 hides a host bug as a slow conveyor.
- A command timeout coasts every motor. A motor left driven after the host goes away is the unsafe state.
- Stop-all on init, so a reset does not inherit a spinning motor.
- An N20 may not break stiction at low duty. The firmware kicks once at full duty, then settles.
- Keep PWM above audible or the motors whine.

---

## The sim

The drive is MuJoCo's own friction on a surface that is already moving. A force computed in Python is linear in the slip below the regularisation speed, which makes the part's yaw an explicit damper. That damper went unstable at µ 1.2 and at a tight regularisation, and the heading did not converge as the regularisation was reduced. The belt slab is a slide joint along the module's travel. Each nose and each cone is a hinge. Every step puts the joint position back to zero and the joint velocity at the commanded surface speed: the slab is only as long as the flat run, and the collision slices are faceted, so letting either integrate would walk the belt away and roll the crown points. The solver still sees the velocity. The flat run moves at the commanded speed. Around the nose the outer fibre is faster, because the belt's neutral axis is inside the surface the part can touch (0.169 m/s when the flat run is at 0.155). On the curve the crown of every roller matches Ω ẑ × (p − C), Ω = curve speed / centreline radius. The hinge sign is whichever of the two matches that field; it is −1. The joints carry enough armature that a contact does not change their speed inside a step. µ on a drive geom is the module's µ. MuJoCo takes the larger of a pair, and the part is set to 0.05 so the drive's value is the one that acts. Rails and walls are 0.04. Drive contacts are condim 3, because the slices already produce the torsional moment. The cone is elliptic, multiccd stays on, and noslip iterations stop a stuck contact from creeping at the soft-constraint rate.

The timestep is 0.5 ms and noslip is 60. Ten iterations at 0.5 ms left the exit yaw 1.1° away from the same run at 0.25 ms. At 60 the 0.5 ms run is within 0.1° and 0.1 mm of the 0.25 ms run and of a 0.125 ms run. Thirty iterations already saturates the 0.5 ms step (60 and 100 print the same yaw), but a 0.25 ms step at 30 iterations moved 0.9°, so the default is 60. A headless nominal run takes 0.39 s. The sweep takes 6.5 s on 15 workers.

A part set down at rest on the cones, drives held, meets two rollers, 17 contacts on each, spread 32.3 mm along the crown. Tilt is 0.010°. It sits 0.240 mm above the height it rests at on s1. Both surfaces are at 31 mm. The belt's four contacts sink 0.25 mm and the cones' 34 sink 0.01 mm, and that difference is the 0.24 mm.

The nominal run (0.155 m/s, µ 0.9 / 0.35, offset 0) **fails**. Exit code 1.

| | |
|---|---|
| Entry offset | −0.00 mm |
| Exit offset | −2.46 mm |
| Exit yaw | 84.7° (limit is 90° ± 5°) |
| Dip | 0.12 mm, on s2 |
| Max tilt | 0.38°, on s1 |
| Rail contacts | none |
| Time to the exit station | 1.36 s, limit 3.03 s |

Yaw error against the path tangent: +0.57° at the entry face, −4.73° at mid-curve, −7.47° at the exit face, −5.34° at the exit station. The part turns with the rollers and sits a few degrees behind the tangent. The straight belt after the curve takes about two of those degrees back. It is one velocity across the whole part, so it cannot finish the correction, and 84.7° is the heading that remains.

At the same speed with the friction matched, exit offset and exit yaw are:

| µ belt / curve | off | exit offset | exit yaw | |
|---|---|---|---|---|
| 0.35/0.35 | −8 | −10.20 mm | 89.6° | FAIL, s2 outer rail |
| 0.35/0.35 | 0 | −2.19 mm | 88.5° | PASS |
| 0.35/0.35 | +8 | 6.36 mm | 91.2° | PASS |
| 0.90/0.90 | −8 | −10.54 mm | 90.0° | FAIL, s2 outer rail |
| 0.90/0.90 | 0 | −2.23 mm | 91.3° | PASS |
| 0.90/0.90 | +8 | 6.00 mm | 93.2° | PASS |

### Sweep

36 runs: three speeds, four friction pairs, three entry offsets. **9 pass, 27 fail.** Exit code 1. The spreads fail at every offset (limit 1.0 mm and 2.0°). Full rows are in `renders/sim/sweep.json`. Nothing was changed to make a row pass.

| speed | µ belt / curve | off | entry | exit | yaw | dip | tilt | rails | |
|---|---|---|---|---|---|---|---|---|---|
| 0.030 | 0.30/0.25 | −8 | −8.00 | −8.51 | 88.4 | 0.00 | 0.49 | 0 | PASS |
| 0.030 | 0.90/0.35 | −8 | −8.00 | −8.54 | 81.9 | 0.00 | 0.49 | 0 | FAIL |
| 0.030 | 1.20/0.50 | −8 | −8.00 | −8.41 | 83.2 | 0.00 | 0.50 | 0 | FAIL |
| 0.030 | 1.20/0.25 | −8 | −8.00 | −8.72 | 79.4 | 0.00 | 0.50 | 5 | FAIL |
| 0.080 | 0.30/0.25 | −8 | −8.00 | −9.38 | 88.7 | 0.00 | 0.44 | 0 | PASS |
| 0.080 | 0.90/0.35 | −8 | −8.00 | −9.40 | 83.6 | 0.00 | 0.45 | 0 | FAIL |
| 0.080 | 1.20/0.50 | −8 | −8.00 | −9.43 | 84.3 | 0.00 | 0.46 | 0 | FAIL |
| 0.080 | 1.20/0.25 | −8 | −8.00 | −8.54 | 78.6 | 0.00 | 0.44 | 7 | FAIL |
| 0.155 | 0.30/0.25 | −8 | −8.00 | −10.34 | 90.2 | 0.11 | 0.44 | 5 | FAIL |
| 0.155 | 0.90/0.35 | −8 | −8.00 | −4.69 | 35.9 | 0.08 | 0.50 | 15 | FAIL |
| 0.155 | 1.20/0.50 | −8 | −8.00 | −9.42 | 83.5 | 0.09 | 0.36 | 7 | FAIL |
| 0.155 | 1.20/0.25 | −8 | −8.00 | −4.40 | 39.5 | 0.08 | 0.36 | 13 | FAIL |
| 0.030 | 0.30/0.25 | 0 | −0.00 | −0.44 | 86.8 | 0.00 | 0.47 | 0 | PASS |
| 0.030 | 0.90/0.35 | 0 | −0.00 | −0.59 | 81.9 | 0.00 | 0.56 | 0 | FAIL |
| 0.030 | 1.20/0.50 | 0 | −0.00 | −0.60 | 82.8 | 0.00 | 0.55 | 0 | FAIL |
| 0.030 | 1.20/0.25 | 0 | −0.00 | −0.79 | 76.9 | 0.00 | 0.56 | 0 | FAIL |
| 0.080 | 0.30/0.25 | 0 | −0.00 | −1.20 | 87.6 | 0.00 | 0.44 | 0 | PASS |
| 0.080 | 0.90/0.35 | 0 | 0.00 | −1.28 | 84.7 | 0.00 | 0.54 | 0 | FAIL |
| 0.080 | 1.20/0.50 | 0 | 0.00 | −1.22 | 86.5 | 0.00 | 0.60 | 0 | PASS |
| 0.080 | 1.20/0.25 | 0 | 0.00 | −1.41 | 81.2 | 0.00 | 0.60 | 0 | FAIL |
| 0.155 | 0.30/0.25 | 0 | 0.00 | −2.42 | 89.1 | 0.11 | 0.45 | 0 | PASS |
| 0.155 | 0.90/0.35 | 0 | −0.00 | −2.46 | 84.7 | 0.12 | 0.38 | 0 | FAIL |
| 0.155 | 1.20/0.50 | 0 | −0.00 | −2.54 | 83.8 | 0.12 | 0.38 | 0 | FAIL |
| 0.155 | 1.20/0.25 | 0 | −0.00 | −2.46 | 81.3 | 0.12 | 0.38 | 0 | FAIL |
| 0.030 | 0.30/0.25 | +8 | 8.00 | 7.71 | 86.9 | 0.00 | 0.47 | 0 | PASS |
| 0.030 | 0.90/0.35 | +8 | 8.00 | 7.34 | 81.3 | 0.00 | 0.55 | 0 | FAIL |
| 0.030 | 1.20/0.50 | +8 | 8.00 | 7.78 | 80.0 | 0.00 | 0.54 | 0 | FAIL |
| 0.030 | 1.20/0.25 | +8 | 8.00 | 7.37 | 76.1 | 0.00 | 0.53 | 0 | FAIL |
| 0.080 | 0.30/0.25 | +8 | 8.00 | 6.93 | 92.0 | 0.00 | 0.44 | 0 | PASS |
| 0.080 | 0.90/0.35 | +8 | 8.00 | 6.76 | 83.9 | 0.00 | 0.54 | 0 | FAIL |
| 0.080 | 1.20/0.50 | +8 | 8.00 | 6.72 | 83.8 | 0.00 | 0.55 | 0 | FAIL |
| 0.080 | 1.20/0.25 | +8 | 8.00 | 6.92 | 81.1 | 0.00 | 0.54 | 0 | FAIL |
| 0.155 | 0.30/0.25 | +8 | 8.00 | 5.86 | 86.7 | 0.11 | 0.44 | 0 | PASS |
| 0.155 | 0.90/0.35 | +8 | 8.00 | 6.13 | 83.6 | 0.12 | 0.50 | 0 | FAIL |
| 0.155 | 1.20/0.50 | +8 | 8.00 | 5.92 | 81.9 | 0.12 | 0.35 | 0 | FAIL |
| 0.155 | 1.20/0.25 | +8 | 8.00 | 5.96 | 75.1 | 0.09 | 0.42 | 0 | FAIL |

| entry offset | exit-offset spread | exit-yaw spread | |
|---|---|---|---|
| −8 mm | 5.94 mm | 54.3° | FAIL |
| 0 | 2.10 mm | 12.2° | FAIL |
| +8 mm | 1.92 mm | 16.9° | FAIL |

What the failures are:

- **Yaw** is the common one, 26 rows. A grippier belt than the cones leaves the part behind the tangent. The close pair (0.30 / 0.25) is the one that usually lands inside 5° of 90°. The worst row is not a small lag: 0.155 m/s, µ 0.9 / 0.35, offset −8, exit yaw 35.9°. Its yaw error is +0.58° at the entry face, −6.25° at mid-curve, −44.4° at the exit face and −54.1° at the station, and it meets `s2_rail0`.
- **Dip and tilt** are not failures. The largest dip is 0.12 mm and the largest tilt is 0.60°.
- **Rails.** Six rows, and every contact is `s2_rail0`, the outer plate of s2. They are the outer entry, mostly at 0.155 m/s.
- **Routing is not independent of speed and friction.** The same entry offset does not come out in the same place. The spreads fail at every offset.

---

## The belt corner

Superseded 2026-09-25. It is in git history at `7c560e0`. That layout kept the part's yaw instead of turning it with the path. This one turns the part by the roller speeds, and the sim above is the measurement of whether that yaw actually arrives at 90°.
