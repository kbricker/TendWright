# Kyle's Bench Task List

Everything physical, in dependency order. Software for every step is already
merged and synced on cell1. Companion docs: `arm-build-day.md` (arm detail),
`hardware/mockbay/README.md` (fixture measure/print/wire detail),
`hardware/bench/README.md` (tool usage).

---

# OPEN — as of 2026-07-31

Things that need Kyle's hands, newest work first. Each names the plan it
serves, so closing the item closes something. **The list below the divider
predates the build and needs a status sweep** — the arm is assembled,
calibrated and clamped, and both cameras are mounted, so several of its
sections are stale. It is kept rather than pruned because nobody has
confirmed item by item which of the mock-bay and Pico work actually
happened.

## A. Affix 3 AprilTags to the arm  — 713.8, 716.4

Print `docs/arm-apriltags-25mm.html` (**IDs 16–18, 25 mm black square**).
Check the ruler bar reads exactly 100 mm before trusting the print — a
tag printed to the wrong size makes every pose wrong by the same factor,
silently.

Why these three sites and not others: each of the arm's six joints was
put through the cell model at seven poses (REST, STAGE and five CRANE
poses) and projected into `low`'s frame. m3 (elbow) is **out of frame in
all seven** — do not bother with it. The three that survive:

| tag | site | seen in | reads | what it is for |
|---|---|---|---|---|
| **16** | **the base**, on a fixed face | 7 of 7 | 70 px | the anchor. Never moves, so its apparent pose IS the camera's pose — if the stand ever shifts, this is what tells you |
| **17** | **the m2 shoulder housing** | 7 of 7 | 74 px | second reference, moves with j1 only |
| **18** | **the wrist**, on the m5/m6 assembly | 6 of 7 | 76 px | the hand-eye target — the one that actually tracks the tool |

- **Face them at the camera.** `low` sits at the table's short end and
  sees the arm broadside from its upper left, 460 mm downrange and about
  31° off to one side. A tag on a face pointing straight up is the
  grazing-incidence case that already costs detections on the table
  tags — point them at the lens, not at the ceiling.
- **25 mm is sized by the wrist, not by the camera.** m5→m6 is only
  36.2 mm centre to centre, so 25 mm (31.25 mm printed with its quiet
  zone) is about the largest that fits there. One size across all three
  keeps a single number in the pose estimator.
- **IDs 16–18 are a new block on purpose.** 0–7 are the 40 mm sheet and
  8–15 the 20 mm sheet, so ID already implies size; reusing one would
  make the same ID ambiguous between two physical sizes.
- STAGE is the one pose where the wrist tag leaves the top of frame. It
  is a high pose; nothing to fix, just do not expect tag 18 there.

**AFTER sticking them on, the measurement is the point.** A tag whose
transform to its link is unknown is decoration. For each one record the
offset from a feature the model knows (a joint centre, a screw line) and
which way its +X points. Send those and I will put them in the cell
model. Until then they are pretty, not useful.

## B. `low` camera — the two open 717.5 checks

- [ ] **Push test.** Deliberate hand pressure on the stand from several
      directions; the lens pose must not move measurably. This is the
      mount's only real requirement.
- [ ] **Tag lock as written** — stand a 40 mm tag (ID 0–3, unused) *up*
      in the grasp zone rather than flat on the table, and say so; I
      re-run the detector in two minutes. Measured 2026-07-31: the
      stand is **not** the problem — see the note below.

**The 43%/53% miss rate was the detector, not the mount.** `quad_decimate`
defaults to 2.0, which halves the image before quad detection. At 2.0
`low` sees tags 4 and 5 in 43% and 53% of frames with bit errors; at
**1.0 it is 120/120 and 120/120**, hamming 0, centre jitter 0.09 px.
`bench` locks 34 px tags at decimate 2.0 all day, so it is not tag size —
it is that decimation destroys a *foreshortened* tag, and `low` looks
along the table where everything flat is foreshortened. Cost is about
half the frame rate (4.7 fps against 8.8 at 1080p). That trade belongs to
**713.10 / 713.11**, not to the stand.

## C. Restart camserve on cell1

**Not Kyle's job to sync cell1** — files there are mine to manage (Kyle
2026-07-31: *"we dont git anything on cell1, you manage the files
there"*). It had drifted 5 commits behind and is now current at
`45b2690`; it had been missing **9ba1c6f, the twin's j1 left/right
mirror fix**, so anything gating on the twin over there was using the
mirrored model.

- [ ] Restart `camserve` — **needs Kyle's say-so** (standing rule: no
      service restarts on cell1 without asking). The running process is
      >22 h old and predates `472291a`, so `/status` there still has no
      `health` field. It is now also running old code against new files
      on disk, which is fine for anything imported at startup and a
      genuine hazard for anything imported lazily inside a request.

## D. Small things noticed while working

- [ ] The **KP303 bench strip** (`192.168.86.90`) answers ping but did
      not show up in `kasa list` discovery on 2026-07-31. The other two
      Kasa devices did. Worth a look before anything relies on the
      guarded `Arm` outlet.
- [ ] `low` is aimed low. Rendering its view from the cell model, the
      arm sits crammed into the top-left corner and **the bottom ~60% of
      the frame is empty near tabletop**. The 7° down tilt was chosen
      deliberately to keep the near rows in frame; the render is what
      that choice actually costs. Worth revisiting under **713.11**, not
      before — re-aiming invalidates any hand-eye already captured.

## E. Blocked on Kyle, from earlier work

- [ ] **714.6** — one item left: *`sim/ik.py` solves for a target on the
      arm's left and the arm actually goes left*. Needs the arm powered.
- [ ] **712.11** — needs a real batch run, a deliberately-failing run,
      and **explicit authorization for unattended operation** recorded on
      the plan. That last one is a decision, not work.
- [ ] **716.4** — fix the arm, place the cameras, measure the obstacles.
      This is the big one: 11 open Vision plans and 6 Cell plans are
      waiting behind it.

---

# LEGACY — predates the build, needs a status sweep

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
- [ ] `calibrate capture` — guided (torque off, can't move the arm): sweep
      each joint, pose at rest, nudge each joint positive; writes
      `calibration.json` — commit it / send it to spark
- [ ] `jog` — one joint at a time, small moves, hand near the switch
- [ ] `teach` / replay a slow air-move — no load, reduced speed
- [ ] `exercise` — scripted limber-up: wake → rest → sweep every joint
      through 70% of its calibrated range → rest → torque off. Needs
      `calibration.json`; starts only from the rest pose; ANY key = e-stop

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
