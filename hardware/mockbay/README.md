# hardware/mockbay — the mock CNC bay (plan #619)

The physical stand-in for the CNC: a printed nest fixture the arm loads
40×40×20 mm blanks into, with a KW12-3 roller microswitch under the
pocket floor as the part-present sensor, read by a Pico over USB.

Pieces:
- `nest_fixture.scad` — parametric OpenSCAD model (source of truth)
- `nest_fixture.stl` — rendered with the default parameters below;
  re-render after editing params: `openscad -o nest_fixture.stl nest_fixture.scad`
- `../pico/firmware/main.py` — Pico MicroPython firmware
- `../pico/` — host-side reader (`NestReader`, the PicoCell sensor
  backend) + `uv run python -m hardware.pico.watch` wiring checker

## How the switch mounts (rev 2 — bottom-load)

The switch mounts **from underneath** (no insertion tunnel): it sits
lever-up on a **separate printed riser block** inside a fully open-bottom
bay, positioned so the **roller sits under the pocket center**
(`roller_x_offset`) and the **pressed roller lands ~0.3 mm below the
blank seat** (the stack is derived from the measured lever geometry).
The whole switch-top envelope is open through the pocket floor — the
lever rises through the window and the 40 mm blank bridges it with >6 mm
bearing all round. Retention: an M2 screw/pin per mounting hole, driven
from the **outside** −Y face through the wall, through the switch hole,
into a pilot in the far wall. Assembly: riser on the bench, switch on the
riser, lower the fixture over both, drive the pins, clamp down (the
bench closes the bay). Wires run in a groove under the flange to the −X
edge. **The riser height is the engagement tuner** — print a few at
±0.5 mm (`riser_h`) and let the bench test pick; the fixture itself
never needs reprinting to tune.

## ⚠ MEASURE BEFORE PRINTING (calipers + one bench test)

The STL in the repo uses nominal values — verify these, edit the params
at the top of the `.scad`, re-render, then print:

| Param | Nominal | How to verify |
|---|---|---|
| `blank_xy`, `blank_h` | 40.0 / 20.0 | calipers on an actual cut wax blank |
| `pocket_clear` | 0.30/side | Kyle's A1 tolerance test print (PETG) |
| `sw_len`, `sw_w`, `sw_h` | 27.0 / 10.4 / 16.0 | calipers on a KW12-3 body (**lever excluded** from `sw_h`) |
| `sw_hole_pitch`, `sw_hole_h`, `sw_hole_d` | 22.0 / 5.0 / 2.0 | calipers on the KW12-3 mounting holes (pitch, height above base, bore) |
| `roller_x_offset` | 11.0 | roller contact point ahead of body center |
| `lever_free_h`, `lever_pressed_h` | 19.0 / 16.5 | roller TOP above body base, lever free / fully pressed |
| `bay_clear` | 0.4/side | switch drop-in fit, from the A1 tolerance test |

**Bench test — switch operating force (the design's #1 open risk):** a
machinable-wax blank weighs only ~30–40 g (≈0.3–0.4 N). A standard-force
KW12-3 needs more than that; roller-lever variants trip at less, but
whether *your* switches trip under a wax blank must be tested by hand
before committing to this geometry: press a blank onto the roller — if it
doesn't click reliably, options are (a) lighter-force switch variant,
(b) lever bent for more mechanical advantage, (c) heavier (aluminum)
test blank, (d) fall back to the FSR pad. The bay geometry is parametric
either way.

## Print settings (Bambu A1)

- PETG, 0.2 mm layers, 4 walls, 30 % infill, no supports
- Orientation: as modeled — pocket UP (the chamfer funnel faces up, so
  elephant's-foot lands on the flange bottom, not the lead-in; the floor
  window edges are short bridges the A1 handles without support)
- Assembly: drop the switch into the bay from below (lever up, roller
  toward pocket center), drive the M2 pins from the −Y outer face,
  route wires through the underside groove, then clamp to the bench

## Switch wiring

```
KW12-3 COM  ──►  Pico GND (any GND pin)
KW12-3 NO   ──►  Pico GP16
```

Normally-open to GND with the Pico's internal pull-up: pin reads 1 when
the nest is empty, 0 when a blank presses the roller. The firmware
inverts this so `nest: true` = part seated. (Using NO means a wiring
break reads "empty" — the fail-safe direction: the FSM refuses to
believe a part is seated rather than believing a phantom.)

## Pico flash procedure

1. Hold BOOTSEL while plugging the Pico 2 into cell1 (or any box).
2. Copy the MicroPython UF2 for Pico 2 onto the RPI-RP2 drive
   (micropython.org/download → Pico 2); it reboots as a serial device.
3. Copy the firmware: `mpremote fs cp hardware/pico/firmware/main.py :main.py`
   (or use Thonny). `mpremote` comes with MicroPython tooling — on cell1,
   `uvx mpremote` runs it without installing anything.
4. Verify: `uv run python -m hardware.pico.watch` — you should see live
   state flips when you press the switch (the identify/hello line repeats
   every ~5 s, since the boot-time one is lost before the host connects).
5. cell1 udev note: with the arm, GRBL, and Pico all on USB-serial,
   enumeration order WILL swap — pin names with udev rules
   (`/dev/tty-pico` etc.) per the hardware plan before P6 wiring.
