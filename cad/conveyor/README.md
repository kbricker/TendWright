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

`build_parts.py` is the only place a dimension is declared. It writes `parts/*.stl`, `parts/*.step`, `build.log`, and `parts/geometry.json`. The sim reads that file and declares none of its own module dimensions. The payload is the sim's spec: a 20 mm cube of 4 g, or with `--block` the 32 × 32 × 16 mm, 30 g block the corner was first sized for.

`freecadcmd` swallows stdout and returns 0 even when the script raises. The run succeeded if `build.log` ends with `=== build complete ===`.

Sim flags:

- `--speed` — straight belt speed, m/s. Default 0.139, the curve's top speed: 270 RPM on the cones' centreline. At 270 RPM the belt would do 0.155, so the straights run at about 90% duty to match the curve.
- `--curve-speed` — centreline speed of the curve, m/s. Default: `--speed`, capped at the curve's top speed.
- `--block` — the 32 × 32 × 16 mm, 30 g block instead of the 20 mm cube.
- `--mu` — belt friction, TPU on the part. Default 0.9.
- `--mu-curve` — cone friction, TPU on the part. Default 0.9.
- `--offset` — entry offset, mm, toward the inside of the turn. Default 0. Parts load within 4 mm of the lane centre.
- `--seconds` — replaces the acceptance time limit, on the nominal run and on every sweep row. A cap that misses the exit is a failure. `--view` does not take it; the viewer runs until the window closes.
- `--frames` — nominal-run PNGs in `renders/sim/`. Default 6. `--sweep` draws none.
- `--sweep` — the acceptance matrix. Writes `renders/sim/sweep.json`. Exit 0 only if every run passes its gates. The spreads are written with the rows and do not decide the exit code.
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
| `coupon_curve.stl` | `roller_cone_driven.stl`, `roller_cone_idler.stl`, one O-ring on groove A | PETG sector, TPU 95A cones | both ear holes and their nut pockets (the nuts drop in from the top), a cone that turns freely, and the O-ring in groove A |
| `coupon_infeed_end.stl` | one `tensioner_block_left.stl` | PETG | the block must slide on the rail, and the jack boss must take two M3 nuts, one along the screw and one from the top |

The infeed coupon is the plate and the block side by side on the bed, with a gap, so the slide can actually be tried. Nested on the rail, the block would print in mid-air.

`coupon_curve.stl` is the PETG sector alone. The cones and the O-ring named in that row are their own files, in their own materials.

**What to caliper, and the parameter it feeds**

- The O-ring: the grooves are cut for a 16 × 2 ring from the kit at 18% stretch (build.log: ID window 14.08–16.08 mm at a 2.00 mm section). On the first coupon a 20 × 2 at 10% slipped and an 18 × 2 at about 21% drove. The ring has to drive every cone and be no tighter than that, because its pull presses each cone onto its rod. If the 16 × 2 slips or drags, caliper it (`oring_id`, `oring_cs`) and rebuild.
- The enclosure, face of the ear plate to the gearbox face → `encl_face_to_gearbox`. The model uses 2.5 mm, which is the worst-case reading of the drawing. The shaft engagement in the log (6.300 mm) moves if this does.
- Then rerun `build_parts.py`. The D-bore, the Ø4.4 roller bore and the take-up slot confirm the machine's hole offset; they are not there to discover it. Bores are modelled +0.15 mm on radius, because this machine prints holes about that much undersize.
- Each cone, idler and driven, turns freely on the Ø3 rod and on the stub. If it drags or rocks, change `cone_bore_d` and rebuild. That bore is Ø3.7, looser than the +0.15 rule, because TPU grips steel.
- The O-ring must seat in groove A. The spool under it is TPU.

**Plate 2 — belt test**, once TPU arrives. One straight belt, `belt_straight.stl`: a cylinder **Ø79.8 mean × 50 mm tall × 1.0 mm wall**, standing upright. That diameter is `straight.print_cyl_dia` in `geometry.json`, the neutral-axis length divided by π. The 1.0 mm wall is what keeps belt-thickness / pulley-diameter at 10 on a Ø10 roller; 1.5 mm would fight the wrap.

Rollers and cones on these plates print in the orientation in the table below.

---

## Full part set — v0

v1 counts belong to #840. PETG where the part is loaded and shares holes with the plates. PLA+ on the straight rollers, where the fit against the belt or the shaft is the point, so a different shrink does not open the bore the coupon just checked. The cones are TPU 95A, the same material as the belts.

| file | qty | material | orientation |
|---|---|---|---|
| `bracket_straight_motor.stl` | 2 | PETG | standing, as exported |
| `bracket_straight_plain.stl` | 2 | PETG | standing, as exported |
| `roller_driven.stl` | 2 | PLA+ | **axis vertical**, brim |
| `roller_idler.stl` | 2 | PLA+ | **axis vertical**, brim |
| `roller_cone_driven.stl` | 1 | **TPU 95A** | **big end down**, brim |
| `roller_cone_idler.stl` | 7 | **TPU 95A** | **big end down**, brim |
| `slider_bed_straight.stl` | 2 | PLA+ | belt face up |
| `return_guide_straight.stl` | 2 | PLA+ | belt face up |
| `tie_bar.stl` | 4 | PETG | joiner-nut pockets up |
| `tensioner_block_left.stl` | 2 | PETG | underside down, motor plate |
| `tensioner_block_right.stl` | 2 | PETG | underside down, plain plate |
| `joiner.stl` | 2 | PETG | flat, holes vertical |
| `curve_frame.stl` | 1 | PETG | base down |
| `curve_keeper.stl` | 1 | PETG | as exported, base down |
| `belt_straight.stl` | 2 | **TPU 95A** | upright |

The curve has no belt. Seven 16 × 2 O-rings, from the kit, link its rollers.

The eight cones are 42.2 g of TPU 95A, beside the two belt cylinders: 5.15 g for each idler and 6.12 g for the driven cone. That is the STL volume, 34.87 cm³, times 1.21 g/cm³, and it is a solid upper bound.

### Why the rollers print vertical

The D-flat is the only thing transmitting drive torque. Axis-vertical, the layers are discs and the flat's load is circumferential, in the plane of the layers. Printed on its side, that load is interlayer adhesion.

### Why the cones print big end down

The spool and the driven spigot sit outboard of the big end. On the bed, that end is the base and the cone narrows as it rises, so the taper is not an overhang. The D-flat still bears in the plane of the layers. The STL is the placed cone, in that orientation. The axle tilt leaves a lip of a few hundredths of a millimetre past each end face; it stays inside the frame gap.

### Bed and return guide

Both are held by a closed groove in each plate. The tongue is captured in X and Z by the first plate; the second plate closes Y. Belt drag pushes the bed toward the discharge, and the discharge wall of the groove is what stops it. The guide uses the same grooves, so it does not compete with the joiner for the top of the tie bar. Its top is square: the two long edges sit outside the belt, so there is nothing there to crown. It stays 0.5 mm under the taut return run (`return_guide_top` 18.5 mm, `return_run_z` 19 mm).

### Tie bars, tensioners, joiners

- Two tie bars per straight, one within 25 mm of each end. They set the plates at `inner_width` and keep the module square. M3 through the plate into a captive nut in the upright. Slide that nut in from the end of the upright before the plate goes on. The channel is the nut across flats, 5.8 mm, so the flats bear on the walls and the nut cannot spin while the screw is turned. 1.2 mm of the bar stays between the nut and the plate, so the plates cannot pull apart and let the bed tongues out of their grooves. The top face is the joint plane the joiner sits on. Print them pockets-up so the joiner nuts drop in.
- A tensioner block on each plate's outer face, at the infeed. The motor-plate block and the plain-plate block are mirrors — the rail groove and the bore are not symmetric — so they print as `tensioner_block_left.stl` and `tensioner_block_right.stl`, two of each. Each slides on a rail and carries the idler rod. An M3×16 through two captive nuts in a fixed boss pushes the block toward the module face. A web between the nuts takes the screw's reaction: the block-side nut slides in along the screw, and the head-side nut drops in from the top. Belt tension keeps the block on the screw tip. The outer end of the slot is the hard stop: at full take-up the idler axis is at `nose_edge`, which is the design span. The 8 mm travel shortens the belt path by 16 mm, which is the slack for sliding the loop on from the open side. The bed stops short of the flange at full slack, so with the belt tensioned the carry is unsupported for 14.7 mm behind the infeed nose.
- One joiner part for both joints. The two end tie bars mirror about the module centre, so the same plate sets the 1.5 mm frame gap at J1 (s1 → curve) and J2 (curve → s2). M3 down into the nuts. The tie-bar nuts drop in from below, and the pad nuts slide in from the joint face before the straight module is set against the curve, so nothing hangs under the table. There is no straight-to-straight joint in v0.

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
| rod | Ø3 | 69.950 mm | 7 | curve idler |
| rod | Ø3 | 24.292 mm | 1 | curve driven stub |

The curve idler is 69.950 mm, seated on the blind-hole bottom. Its square end, on the 5.127° tilt, stays 0.2 mm inside the keeper's inner face. The straight idler is flush with the outsides of the tensioner blocks, the straight stub is seated on the blind floor and 0.5 mm short of the D-bore, and the curve stub stops 0.5 mm short of the cone's bore bottom. Those cuts are the solids in the interference check.

**O-rings.** Seven 16 × 2 rings, alternating grooves A and B, four of A and three of B (`curve.n` is 8), each at 18% stretch. From `build.log`:

| groove | pitch c | spool D | crown clearance |
|---|---|---|---|
| A | 16.443 mm | 10.772 mm | 1.046 mm |
| B | 17.230 mm | 10.271 mm | 1.653 mm |

The axles are skewed by about 11.4°, so each tangent leaves the groove plane at 0.099 mm of axial offset per millimetre of span. The centre clears the rim after about 2.6 mm, 0.259 mm off the plane on A and 0.254 mm on B, and 0.60 mm out to the flange. That 0.60 mm is the climb to the lip. The groove section is still a 1.15 mm radius circle, with its centre 0.15 mm outside the pitch circle, so the floor is exactly one cord radius under the pitch circle. A cord resting on the floor is centred on the pitch circle, and the stretch of that seated path is 0.180. The section is swept ±0.359 mm along the axle.

Replace these by calipering the ring and rebuilding. Do not order a printed drive ring.

---

## Key dimensions

From `geometry.json` and `build.log`.

| | |
|---|---|
| Belt | 50 mm wide, 1.0 mm wall, carry surface z = 30 mm, belt top 31 mm |
| Straights | 120 mm, nose axis 6 mm in from each face |
| Frame gap | 1.5 mm, both joints |
| Transfer span, both joints | inner 10.2 mm, centreline 12.4 mm, outer 14.6 mm |
| Taper | k = 0.179, so cone diameter = 0.179 × plan radius: Ø5.2 at the small end, Ø9.8 on the centreline, Ø14.5 at the big end |
| Rollers on the curve | 8, pitch 11.392°, driven roller is the fourth |
| Gap between cones | 0.53 mm at the small end, 1.00 mm on the centreline, 1.47 mm at the big end |
| Axle tilt | 5.127° down toward the outside, which is what keeps the crown at z = 31 |
| Curve centreline radius | 55 mm |
| Curve top speed | 0.139 m/s at 270 RPM, against the belt's 0.155 |
| Cone plain bore | Ø3.7 (`cone_bore_d`), on the Ø3 rod and the driven stub |
| Shaft engagement | 6.300 mm into a 10 mm D-bore |
| Straight belt | path 250.6 mm at the neutral axis, printed mean Ø 79.8 mm |
| Jack-screw thread in the nuts | both nuts fully crossed, 2.6 mm each, at both ends of the 8 mm travel |
| Take-up | 8 mm of travel, 16 mm of belt slack, 14.7 mm of carry unsupported behind the infeed nose |

The outer span is the long one. A part entered on the outer edge has 14.6 mm of nothing at each joint. Parts load within 4 mm of the lane centre. Past that, an outer-edge part at full speed reaches s2's outer plate.

---

## Assembly

The belt still goes on from the open side, with the plain plate off. Nothing in the loop except the rollers and the bed. The joiners go in before that bed, and before the cones. Their screws stand vertically under the lane: once the bed and the belt are on, no hex key reaches the tie-side screws, and with the cones in, the pad-side screws meet the cones (cones 2 and 3 at J1, 6 and 7 at J2).

Separating two joined modules means taking that straight's plain plate, belt and bed back out first. That is the cost of this joiner. A later loop should not copy a joint whose screws are buried under the belt.

### Frame, motor plates, joiners

1. **Curve frame.** Pad nuts slide in from both joint faces. M4 ear nuts drop in from the top of the pad. Keeper nuts drop in from the top of the outer wall.
2. **Each straight's motor plate, then its tie bars.** Tie-bar nuts in from the end of each upright first, flats against the channel walls. Jack nuts into the boss: the block-side nut along the screw, the head-side nut down the top slot. M4 nuts into the ear bosses from the inboard face. Bolt the tie bars through the motor plate only. Then the motor, M4s from the outboard side. The plain plate stays off.
3. **Both joiners.** One on s1's discharge tie bar and the curve's entry pad, one on the curve's exit pad and s2's infeed tie bar. Four M3s down through each plate, into the nuts already in the tie bar and the pad. The cones are not in yet, and neither is the bed.

### Cones, ring before the rod

A closed ring has to encircle the spool, and once the rod runs through the cone and both walls the ring cannot get there. Each ring goes onto its cone before that cone's rod.

The free half of a ring is a loop of radius 6.43 mm. Left beside the spool toward the wall, it meets the frame; toward the lane it clears by 1.40 mm. Held up, above the walls, it clears them by 1.90 mm. Hold that loop up while the cone goes in, then carry it across to the next cone.

1. **Cone 1.** Loop ring 1–2 onto groove A. Drop the cone in (60.76 mm along the axle, 60.51 mm across, walls 63.00 mm apart, 2.49 mm to spare). With both rings seated the drop clearance is still 0.537 mm. Slide the Ø3 rod in from outside the outer wall; the hole clears it by 0.150 mm. The free loop waits, held up, for cone 2.
2. **Cone 2.** Pass it through ring 1–2 so groove A is in that ring, and loop ring 2–3 onto groove B. Drop in. Rod. Hold ring 2–3's free loop up.
3. **Cone 3.** Through ring 2–3 onto groove B, and ring 3–4 onto groove A. Drop in. Rod. Hold ring 3–4's free loop up.
4. **Driven cone.** Pass it through ring 3–4 (groove A) and loop ring 4–5 onto groove B before it goes in. The stub is already in the small-end bore. Spigot first, out through the outer wall: push until the stub clears the inner wall (4.02 mm), then back so the stub seats. With both rings on, the clearance at that 4.02 mm push is 0.150 mm. The relief is the B ring's crown plus 0.4 mm, radius 6.54 mm — the bare spool is not the widest thing on the cone — and the cone is free for 4.77 mm, a margin of 0.75 mm. The outer pad keeps the Ø8 spigot bore. Hold ring 4–5's free loop up.
5. **Cone 5.** Through ring 4–5 onto groove B, and ring 5–6 onto groove A. Drop in. Rod.
6. **Cone 6.** Through ring 5–6 onto groove A, and ring 6–7 onto groove B. Drop in. Rod.
7. **Cone 7.** Through ring 6–7 onto groove B, and ring 7–8 onto groove A. Drop in. Rod.
8. **Cone 8.** Through ring 7–8 onto groove A. Drop in. Rod.

**Keeper**, then the **curve motor**. The keeper screws come in from outside the outer wall, over the rod ends. The M4s come in from outboard of the pad. The motor stays off until the driven cone is seated.

### Each straight, after its joiner

1. **Bed and driven roller together, then the idler roller, the idler rod and the belt** from the open side, over the rollers and the bed as one loop. The bed's end notches take the driven roller's flanges, so neither can slide past the other along the axle. Hold the flanges in the notches and push both home at once: the roller onto the shaft, and the bed's tongue into the motor plate's groove.
2. **Return guide**, under the return run, into its own grooves. It has to be in before the plain plate, because that plate closes the groove.
3. **Driven stub, then the plain plate.** The stub drops into the roller's far bore and stops 0.5 mm short of the D-bore step. The plain plate's bore is blind. Its two tie screws come in from the outboard face.
4. **Tensioners.** Block on each rail, jack screw through the two nuts, taken up until the idler rod is against the outer end of the slot. The hex key comes in along the screw from the head, outboard of the plate.

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

The drive is MuJoCo's own friction on a surface that is already moving. A force computed in Python is linear in the slip below the regularisation speed, which makes the part's yaw an explicit damper. That damper went unstable at µ 1.2 and at a tight regularisation, and the heading did not converge as the regularisation was reduced. The belt slab is a slide joint along the module's travel. Each nose and each cone is a hinge. Every step puts the joint position back to zero and the joint velocity at the commanded surface speed: the slab is only as long as the flat run, and the collision slices are faceted, so letting either integrate would walk the belt away and roll the crown points. The solver still sees the velocity. The flat run moves at the commanded speed. Around the nose the outer fibre is faster, because the belt's neutral axis is inside the surface the part can touch (0.169 m/s when the flat run is at 0.155). On the curve the crown of every roller matches Ω ẑ × (p − C), Ω = curve speed / centreline radius. The hinge sign is whichever of the two matches that field; it is −1. The joints carry enough armature that a contact does not change their speed inside a step. µ on a drive geom is the module's µ. MuJoCo takes the larger of a pair, and the part's sliding friction is 0, so the pair is the drive geom's value exactly, including a commanded 0. Rails and walls keep 0.04. Drive contacts are condim 3, because the slices already produce the torsional moment. The cone is elliptic, multiccd stays on, and noslip iterations stop a stuck contact from creeping at the soft-constraint rate.

The timestep is 0.5 ms and noslip is 60. Ten iterations at 0.5 ms left the exit yaw 1.1° away from the same run at 0.25 ms. At 60 the 0.5 ms run is within 0.1° and 0.1 mm of the 0.25 ms run and of a 0.125 ms run. Thirty iterations already saturates the 0.5 ms step (60 and 100 print the same yaw), but a 0.25 ms step at 30 iterations moved 0.9°, so the default is 60. A headless nominal run takes 0.40 s. The sweep takes 7.2 s on 15 workers.

The cube set down at rest on the cones, drives held, meets two rollers, 11 contacts on each, spread 20.1 mm along the crown. Tilt is 0.066°. It sits 0.108 mm above the height it rests at on s1. Both surfaces are at 31 mm; the difference is how far the belt and the cones sink under the part.

The cones are TPU 95A. With PLA cones the nominal run, µ 0.9 on the belt and 0.35 on the cones, exited at 84.7°, and 9 of 36 runs in that sweep passed. At each handoff the grippier belt held the part's heading, and the curve's rotation field carried that lag out to s2.

The nominal run (the cube, 0.139 m/s, µ 0.9 / 0.9, offset 0) **passes**. Exit code 0. The block's nominal run passes too: exit offset −2.06 mm, yaw 91.2°, dip −0.03 mm, tilt 0.39°.

| | |
|---|---|
| Entry offset | 0.00 mm |
| Exit offset | −2.10 mm |
| Exit yaw | 93.7° (limit is 90° ± 6°) |
| Dip | −0.05 mm, on s1 |
| Max tilt | 0.78°, at the entry transfer |
| Rail contacts | none |
| Time to the exit station | 1.52 s, limit 3.26 s |

Yaw error against the path tangent: +2.27° at the entry face, +1.15° at mid-curve, +0.30° at the exit face, +3.71° at the exit station.

### Sweep

45 runs with the cube: speeds 0.030, 0.080 and 0.139 m/s; belt and cone µ of 0.6/0.6, 0.9/0.9, 1.2/1.2, 0.9/0.7 and 0.7/0.9; entry offsets −4, 0 and +4 mm. The last two pairs are the same TPU about 20% apart, which is what two prints of it can do. **43 pass, and 2 fail on yaw alone.** Exit code 1. Full rows are in `renders/sim/sweep.json`. Nothing was changed to make a row pass. Kyle accepted those two misses on 2026-10-08, with the cube's worst tilt of 4.73°, and left the real check to the printed coupon.

Parts load within 4 mm of the lane centre. Past that, an outer-edge part at full speed reaches s2's outer plate.

A run passes when the exit yaw is inside 90 ± 6°, the exit offset has moved at most 3.0 mm from the entry offset, no rail is touched, the dip is at most 1.0 mm, the tilt is at most 5°, and the exit station is reached inside the time limit. The dip is measured from the mean height while the whole part is on s1's flat run: its back past the infeed nose, its front 2 mm short of the discharge nose.

Per entry offset, the spread of exit offset and of exit yaw across speed and µ is printed below and stored in `sweep.json`. Kyle accepted the speed-dependent drift, so the spreads are reported and the exit code ignores them. The exit code is 0 only when every run passes the gates above.

| speed | µ belt / curve | off | entry | exit | yaw | dip | tilt | rails | |
|---|---|---|---|---|---|---|---|---|---|
| 0.030 | 0.60/0.60 | −4 | −4.00 | −4.48 | 91.9 | −0.01 | 2.56 | 0 | PASS |
| 0.030 | 0.90/0.90 | −4 | −4.00 | −4.51 | 92.1 | −0.01 | 2.86 | 0 | PASS |
| 0.030 | 1.20/1.20 | −4 | −4.00 | −4.50 | 91.8 | −0.01 | 3.04 | 0 | PASS |
| 0.030 | 0.90/0.70 | −4 | −4.00 | −4.47 | 90.2 | −0.01 | 4.73 | 0 | PASS |
| 0.030 | 0.70/0.90 | −4 | −4.00 | −4.49 | 93.2 | −0.01 | 2.48 | 0 | PASS |
| 0.080 | 0.60/0.60 | −4 | −4.00 | −5.29 | 91.9 | −0.01 | 2.22 | 0 | PASS |
| 0.080 | 0.90/0.90 | −4 | −4.00 | −5.34 | 91.1 | −0.02 | 2.26 | 0 | PASS |
| 0.080 | 1.20/1.20 | −4 | −4.00 | −5.33 | 91.8 | −0.01 | 2.40 | 0 | PASS |
| 0.080 | 0.90/0.70 | −4 | −4.00 | −5.32 | 90.5 | −0.01 | 2.16 | 0 | PASS |
| 0.080 | 0.70/0.90 | −4 | −4.00 | −5.33 | 93.2 | −0.03 | 2.11 | 0 | PASS |
| 0.139 | 0.60/0.60 | −4 | −4.00 | −6.28 | 90.8 | −0.04 | 1.13 | 0 | PASS |
| 0.139 | 0.90/0.90 | −4 | −4.00 | −6.29 | 92.1 | 0.00 | 1.55 | 0 | PASS |
| 0.139 | 1.20/1.20 | −4 | −4.00 | −6.37 | 92.5 | −0.05 | 1.22 | 0 | PASS |
| 0.139 | 0.90/0.70 | −4 | −4.00 | −6.22 | 90.5 | −0.03 | 1.74 | 0 | PASS |
| 0.139 | 0.70/0.90 | −4 | −4.00 | −6.23 | 92.0 | −0.05 | 1.11 | 0 | PASS |
| 0.030 | 0.60/0.60 | 0 | 0.00 | −0.47 | 93.5 | −0.01 | 1.03 | 0 | PASS |
| 0.030 | 0.90/0.90 | 0 | 0.00 | −0.49 | 92.4 | −0.01 | 2.44 | 0 | PASS |
| 0.030 | 1.20/1.20 | 0 | 0.00 | −0.52 | 92.8 | −0.01 | 1.04 | 0 | PASS |
| 0.030 | 0.90/0.70 | 0 | 0.00 | −0.50 | 91.4 | −0.01 | 2.44 | 0 | PASS |
| 0.030 | 0.70/0.90 | 0 | 0.00 | −0.50 | 94.3 | −0.01 | 0.95 | 0 | PASS |
| 0.080 | 0.60/0.60 | 0 | 0.00 | −1.18 | 91.9 | −0.03 | 1.50 | 0 | PASS |
| 0.080 | 0.90/0.90 | 0 | 0.00 | −1.25 | 93.2 | −0.03 | 0.85 | 0 | PASS |
| 0.080 | 1.20/1.20 | 0 | 0.00 | −1.36 | 95.3 | −0.03 | 3.24 | 0 | PASS |
| 0.080 | 0.90/0.70 | 0 | 0.00 | −1.19 | 91.2 | −0.03 | 1.45 | 0 | PASS |
| 0.080 | 0.70/0.90 | 0 | 0.00 | −1.21 | 94.6 | −0.03 | 0.94 | 0 | PASS |
| 0.139 | 0.60/0.60 | 0 | 0.00 | −2.10 | 93.0 | −0.05 | 0.48 | 0 | PASS |
| 0.139 | 0.90/0.90 | 0 | 0.00 | −2.10 | 93.7 | −0.05 | 0.78 | 0 | PASS |
| 0.139 | 1.20/1.20 | 0 | 0.00 | −2.12 | 93.3 | −0.02 | 1.11 | 0 | PASS |
| 0.139 | 0.90/0.70 | 0 | 0.00 | −2.04 | 92.6 | −0.05 | 0.99 | 0 | PASS |
| 0.139 | 0.70/0.90 | 0 | 0.00 | −2.09 | 94.7 | −0.05 | 0.52 | 0 | PASS |
| 0.030 | 0.60/0.60 | +4 | 4.00 | 3.54 | 94.4 | −0.01 | 0.47 | 0 | PASS |
| 0.030 | 0.90/0.90 | +4 | 4.00 | 3.53 | 93.6 | −0.01 | 0.46 | 0 | PASS |
| 0.030 | 1.20/1.20 | +4 | 4.00 | 3.55 | 92.9 | −0.01 | 0.60 | 0 | PASS |
| 0.030 | 0.90/0.70 | +4 | 4.00 | 3.53 | 92.5 | −0.01 | 0.52 | 0 | PASS |
| 0.030 | 0.70/0.90 | +4 | 4.00 | 3.55 | 95.2 | −0.01 | 0.43 | 0 | PASS |
| 0.080 | 0.60/0.60 | +4 | 4.00 | 2.85 | 94.9 | −0.03 | 0.29 | 0 | PASS |
| 0.080 | 0.90/0.90 | +4 | 4.00 | 2.89 | 93.9 | −0.03 | 0.31 | 0 | PASS |
| 0.080 | 1.20/1.20 | +4 | 4.00 | 2.87 | 92.5 | −0.03 | 0.32 | 0 | PASS |
| 0.080 | 0.90/0.70 | +4 | 4.00 | 2.85 | 93.2 | −0.03 | 0.62 | 0 | PASS |
| 0.080 | 0.70/0.90 | +4 | 4.00 | 2.89 | 96.3 | −0.03 | 0.33 | 0 | FAIL, yaw |
| 0.139 | 0.60/0.60 | +4 | 4.00 | 2.03 | 94.5 | −0.05 | 0.42 | 0 | PASS |
| 0.139 | 0.90/0.90 | +4 | 4.00 | 1.98 | 93.9 | −0.05 | 0.34 | 0 | PASS |
| 0.139 | 1.20/1.20 | +4 | 4.00 | 1.99 | 92.9 | −0.05 | 0.56 | 0 | PASS |
| 0.139 | 0.90/0.70 | +4 | 4.00 | 1.98 | 90.1 | −0.03 | 1.23 | 0 | PASS |
| 0.139 | 0.70/0.90 | +4 | 4.00 | 2.05 | 96.6 | −0.05 | 0.34 | 0 | FAIL, yaw |

| entry offset | exit-offset spread | exit-yaw spread |
|---|---|---|
| −4 mm | 1.894 mm | 2.933° |
| 0 | 1.654 mm | 4.031° |
| +4 mm | 1.576 mm | 6.477° |

Both failures are at +4 mm with belt µ 0.70 and cones 0.90: 96.27° at 0.080 m/s and 96.57° at 0.139 m/s. The faster one's yaw error is +3.39° at the entry face, +2.34° at mid-curve, +1.85° at the exit face and +6.57° at the station, so most of it builds on s2 after the curve. The largest offset change is 2.37 mm, at 0.139 m/s, µ 1.2 / 1.2, entry −4 mm, exit −6.37 mm. The dip runs from −0.05 mm to 0.00 mm, so the part stays at the height it had on s1. The largest tilt is 4.73°, at the exit transfer, at 0.030 m/s, µ 0.9 / 0.7, entry −4 mm. No row touches a rail.

---

## The belt corner

Superseded 2026-09-25. It is in git history at `7c560e0`. That layout kept the part's yaw instead of turning it with the path. This one turns the part by the roller speeds, and the sim above is the measurement of whether that yaw actually arrives at 90°.
