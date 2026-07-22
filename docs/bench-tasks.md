# Kyle's Bench Task List

Everything physical, in dependency order. Software for every step is already
merged and synced on cell1. Companion docs: `arm-build-day.md` (arm detail),
`hardware/mockbay/README.md` (fixture measure/print/wire detail),
`hardware/bench/README.md` (tool usage).

## 1. Arm assembly (parts printed ✅)

- [ ] Set servo IDs 1–6 (base→gripper) BEFORE assembly — servo loose on bench,
      one at a time: `ssh cell1` → `uv run python -m hardware.bench.set_id`
      (power off between servos; the tool refuses >1 servo on the bus)
- [ ] Center each servo, THEN attach horns at neutral (use the printed jig)
- [ ] Assemble follower per Seeed wiki; snug screws, not gorilla
- [ ] 12V PSU only (5V one is the leader's — shelve it)

## 2. Arm bring-up (tools on cell1, `hardware/bench/`)

- [ ] `scan` — all 6 servos report in, sane voltage/temp
- [ ] `monitor` — torque off, move joints by hand, positions track
- [ ] `jog` — one joint at a time, small moves, hand near the switch
- [ ] `teach` / replay a slow air-move — no load, reduced speed

## 3. Mock bay: measure before printing (calipers)

Full table with nominals: `hardware/mockbay/README.md`. Measure and send:

- [ ] Blank: actual X/Y and height (nominal 40 / 20)
- [ ] KW12-3 body: length / width / height (lever EXCLUDED)
- [ ] KW12-3 mounting holes: pitch, height above base, bore
- [ ] Roller contact point offset from body center
- [ ] Roller top height: lever free AND fully pressed
- [ ] Terminal protrusion below base (+ do terminals exit bottom or end?)
- [ ] Pocket + bay clearances from the A1 tolerance test print

## 4. Mock bay: the 10-second risk test

- [ ] Press a blank onto a KW12-3 roller by hand — does it CLICK reliably?
      (Wax blank ≈ 30–40 g; hollow PETG dummy is lighter. If no click:
      lighter-force switch / bend lever / heavier blank / FSR fallback —
      geometry is parametric for all of them.)
- [ ] Hunt ~50 mm of 2 mm rod for retention pins (music wire / dowel;
      1.75 mm filament is a hair loose)

## 5. Print queue (PETG unless noted)

- [ ] Dummy blanks ×6 — 40×40×20 cubes (Bambu Studio: Add Primitive → Cube)
- [ ] Nest fixture — AFTER measurements go back and spark re-renders the STL
      (pocket up, no supports, 4 walls, 30% infill)
- [ ] Riser blocks at nominal and ±0.5 mm — bench test picks the winner
- [ ] (later, after camera scouting) wall wedge bracket for the camera

## 6. Pico + switch

- [ ] Flash MicroPython + firmware (procedure: `hardware/mockbay/README.md`)
- [ ] Wire: KW12-3 COM → Pico GND, NO → GP16
- [ ] Verify: `uv run python -m hardware.pico.watch` — state flips on press

## 7. Camera scouting session (any time after arm bring-up starts)

- [ ] Plug ELP camera into cell1; verify it's the -L36 variant (screw-thread
      manual-focus barrel)
- [ ] `uv run python -m hardware.bench.campreview` — live view + tag overlay
- [ ] Try heights/angles on the back wall (start ~28–32" up, ~20–25° down);
      judge by what the SOFTWARE sees; focus the barrel at chosen distance
- [ ] Mark the spot → overwatch designs the wedge bracket → print → mount →
      final AprilTag sheet at the size the live view proved

## 8. Whenever a monitor is on cell1 (Hive #617)

- [ ] BIOS → UMA Frame Buffer Size = 512M (reclaims ~1.5 GB RAM)
- [ ] Verify: `free -h` shows ~6.7 GB

## The convergence

Arm assembled + nest printed + switch wired + camera mounted =
**bench milestone**: camera finds blank → arm picks → sets into nest →
switch confirms seat → dwell → arm unloads. The P1 orchestrator already
runs this cycle; the hardware just has to show up.
