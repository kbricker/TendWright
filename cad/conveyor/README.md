# Mini modular conveyor — build guide

Hive plan **#835** (v0 rig) · **#840** (v1 loop) · **#843** (components beyond the corner)

v0 is **two straight modules + one corner, three motors, an open line.** It exists to prove
two things nobody knew: that a part survives the handoff gap between independently driven
modules, and that it corners. Both are proven in simulation; nothing has been printed yet.

Shopping list and costs live in [`docs/conveyor/conveyor-bom-v1-loop-2026-08-08.md`](../../docs/conveyor/conveyor-bom-v1-loop-2026-08-08.md).
This file is how to make and assemble the thing.

---

## Regenerating everything

```
"%LOCALAPPDATA%/Programs/FreeCAD 1.1/bin/freecadcmd.exe" cad/conveyor/build_parts.py
uv run python cad/conveyor/render.py            # PNGs of every part and assembly
uv run python cad/conveyor/sim_conveyor.py      # headless, writes frames + a trajectory
uv run python cad/conveyor/sim_conveyor.py --view
```

`build_parts.py` holds **one parameter block** and is the single source of every dimension.
It writes `parts/*.stl`, `parts/*.step`, a step-by-step `build.log`, and **`parts/geometry.json`**.

**Nothing downstream re-declares a dimension.** `sim_conveyor.py` reads `geometry.json`.
It used to mirror the constants by hand, which meant a dimension change left the sim
faithfully validating a design that no longer existed — while still reporting a number.

Sim flags worth knowing: `--speed`, `--corner-speed`, `--mu`, `--seconds`, `--frames`,
`--deadplate`.

### FreeCAD traps this script is written around

From [`CableCell/cad/README.md`](../../../CableCell/cad/README.md) — read them before editing:

1. `Shape.translate()` mutates in place and returns `None`. Everything here is built at its
   final position or uses `translated()`.
2. `freecadcmd` sets `__name__` to the module basename, so a `__main__` guard never fires.
   There isn't one.
3. Routing an STL through a `Mesh::Feature` crashes the process. Meshes go out via
   `Mesh.Mesh(shape.tessellate(dev)).write(path)`.

---

## Print the coupon first

**Do not print a full module yet.** The coupon costs 20 minutes and the belt test an hour,
and either could move a dimension. Printing 76 parts before those two is how you get 76
parts with the same wrong bore.

**Plate 1 — fit coupon** (needs nothing you don't already have):

| file | qty | material | orientation |
|---|---|---|---|
| `coupon_bracket_end.stl` | 1 | PETG | flat on the plate, no supports |
| `roller_driven.stl` | 1 | PLA+ | **axis VERTICAL**, brim |
| `roller_idler.stl` | 1 | PLA+ | **axis VERTICAL**, brim |

Then caliper the Ø4.4 axle bore, the Ø3.3 D-bore and the take-up slot.

**Plate 2 — belt test**, once TPU arrives. One straight belt: a cylinder **Ø79.8 mean ×
50 mm tall × 1.0 mm wall**, standing upright. This is the last genuine unknown in the build.

---

## Full part set

Per module: 2 side brackets · 2 rollers · 1 slider bed · 2 tensioner blocks · 1 return guide.
Plus guide rails and frame connectors.

| file | v0 | v1 | material | orientation |
|---|---|---|---|---|
| `bracket_straight_motor.stl` | 2 | 4 | PETG | flat |
| `bracket_straight_plain.stl` | 2 | 4 | PETG | flat |
| `bracket_corner_motor.stl` | 1 | 4 | PETG | flat |
| `bracket_corner_infeed.stl` | 1 | 4 | PETG | flat |
| `roller_driven.stl` | 3 | 8 | PLA+ | **axis vertical**, brim |
| `roller_idler.stl` | 3 | 8 | PLA+ | **axis vertical**, brim |
| `slider_bed_straight.stl` | 3 | 8 | PLA+ | flat, smooth side up |
| `return_guide_straight.stl` | 2 | 4 | PLA+ | flat, crowned side up |
| `return_guide_corner.stl` | 1 | 4 | PLA+ | flat, crowned side up |
| `guide_rail.stl` | 1 | 4 | PLA+ | flat |
| TPU belt cylinders | 3 | 8 | **TPU 95A** | upright |

PETG where it is loaded, PLA+ where the fit matters — the same split CableCell uses.
Tree supports on, per the standing profile.

### Why the rollers print vertical

The D-flat is the only thing transmitting drive torque. Printed **axis-vertical**, the
layers are horizontal discs and the flat's bearing load is circumferential — **in-plane**,
the strong direction. Printed horizontally it would bear across layers on interlayer
adhesion and roughly halve the margin. Ø13 × 53 mm is slender, so the brim is doing real work.

### Bore tolerance

Bores are modelled at **nominal +0.15 mm on radius**, matching the offset CableCell
measured on this machine (printed holes come out undersize). Expect a Ø3.3 modelled D-bore
to print near Ø3.1 — a light slip fit on the shaft. The coupon **confirms** that offset; it
is not there to discover it.

---

## Key dimensions

Read from `parts/geometry.json`, never retyped.

| | |
|---|---|
| Belt width | 50 mm |
| Carry surface | z = 30 mm; belt top 31 mm |
| Rollers | **Ø10 at both ends**, discharge one driven |
| Nose inset from module face | 6 mm |
| Frame gap, module to module | 1.5 mm |
| **Transfer span, straight → corner** (crosses the corner's *side*) | **12.0 mm** |
| **Transfer span, corner → straight** (crosses the straight's *end*) | **13.5 mm** |
| Belt path, straight / corner | 250.6 / 150.6 mm at the **neutral axis** |
| Printed cylinder mean Ø | 79.8 / 47.9 mm, 1.0 mm wall |
| Shaft engagement in the D-bore | 5–6 mm, shaft-limited |

**90° and 180° are the only transfer angles this architecture supports.** A rectangular
module presents two faces, and only those two turns put one square to the incoming travel.
At any other angle the gap becomes a wedge — a 60° turn leaves one corner of the part over
30 mm of nothing while the other is fully supported. Composed 90° turns reach any heading.
See #843.

---

## Assembly order

1. **Rollers onto the brackets.** Idler goes in the **infeed** take-up slot; driven roller
   at the **discharge** end. The idler rides a Ø4 stub axle in both plates.
2. **Motor into the saddle**, shaft first, from outboard. Its Ø4 boss passes through the
   plate; the shaft crosses the plate and side gap and lands 5–6 mm into the driven
   roller's D-bore. Tighten the M2 clamp screw onto the can.
   *There is no face-mount pattern* — the motor is held on its 10 × 12 body, so any
   GA12-N20-class motor fits regardless of vendor.
3. **Return guide** between the plates, under the *lower* run, crowned edge up. It sits
   **0.5 mm below the taut return line** — a correctly tensioned belt never touches it and
   it only catches sag. Do not shim it up to contact: that adds drag to every module in the
   system to solve a problem the tensioner already solved. It stops a nose diameter short of
   each axis so it can never intrude on the arc where the belt is wrapping.
4. **Slider bed** between the plates, under the carry run. Optionally face it with UHMW or
   PTFE tape: published practice puts a PU belt on UHMW at µ 0.03–0.06 against 0.15–0.30 on
   steel, and printed PLA sits nearer the steel end. Cuts belt drag 5–10×. Unnecessary at
   v0's 10–16× torque margin; worth it at eight motors on one supply.
5. **Belt over the rollers**, then take up slack by sliding the infeed nose outward and
   locking the tensioner blocks. The slot runs inboard from the tensioned position, so the
   fully-tensioned axis is the outer limit — the design span is what you actually get.
   The belt loop encircles the slider bed but passes *above* the return guide, which hangs
   outside the loop — so the guide can go in before or after the belt, the bed cannot.
6. **Butt the modules** at a 1.5 mm frame gap using the frame connectors. The corner's
   infeed side plate is cut flush with the carry plane; **that face must stay clear** —
   a full-height plate stands 4 mm proud of its own belt and is a kerb across the exact
   face the part has to cross.
7. **Guide rail** on the corner's far side. It backs up the design; it does not make it
   work. The part settles into its lane by belt traction and never reaches the rail at any
   speed or friction tested.

---

## Wiring

TB6612FNG ×2 for v0, ×4 for v1. Two supplies, one ground — the classic first-integration failure.

- **12 V PSU** → driver `VM` (motor rail)
- **Pico 2 3V3** → driver `VCC` (logic rail)
- **PSU ground and Pico ground tied together** — the TB6612FNG splits VM from VCC, and it only works if they share a return. Get this wrong and it either does nothing or misbehaves in ways that look exactly like a firmware bug.
- Per driver: `PWMA/AIN1/AIN2` and `PWMB/BIN1/BIN2` from Pico GPIO, plus a shared `STBY` (held high to enable).
- Each driver channel → 2-core → one N20 motor.
- **Not the DRV8833** — tops out at 10.8 V and cannot drive 12 V motors at all.
- **Pico 2, not Pico.** 8 motors × (PWM + IN1 + IN2) + STBY = 25 of a Pico's 26 GPIO, leaving nothing for a sensor. The RP2350 has 12 PWM slices (24 channels) against the RP2040's 8 (16).
- **JST-XH pairs or screw terminals** at each motor so a module can be unplugged.
- Size the PSU from **measured** stall current, not the datasheet.

### Firmware notes

Built — see [`hardware/conveyor/`](../../hardware/conveyor/README.md) for the protocol, pin map
and bring-up CLI. `uv run python -m hardware.conveyor.selftest` checks it without a Pico.

- Commands are **absolute, not deltas** — `set duty=X`, never `increase speed`. That makes
  every command idempotent, so a duplicate is harmless.
- Duty is validated **0–100 % at the firmware boundary**, not on the host, and out-of-range is
  **rejected** — never clamped. Silently saturating 150 to 100 turns a host bug into a
  conveyor that merely runs slower than someone expected.
- **Command timeout that coasts every motor.** A motor left driven after the host goes away
  is the one genuinely unsafe state.
- **Stop-all on init** so a reset never inherits a spinning motor.
- An N20 may not break stiction at low duty — the firmware wants a brief **kick-start pulse**
  at full duty before settling to the commanded value.
- Keep PWM above audible or the motors whine at low duty.
