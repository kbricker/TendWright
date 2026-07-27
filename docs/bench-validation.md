# Bench validation session

Closing out the tickets that have been waiting on hardware. Everything
here runs on **cell1** unless it says otherwise.

Written 2026-07-27. Deployed and verified on cell1 at `4b26fef` — every
no-hardware selftest passes there already (see Phase 0).

---

## What this session closes

| Phase | Ticket | The question it answers | Motion? |
|---|---|---|---|
| 1 | **#647** joint semantic frames | Does the degree readout match the arm you can see? | none |
| 2 | **#670** SO-101 model swap | Is the model the *right arm*? | none |
| 3 | **#660** motion rig | Does the arm move the way the sim said it would? | yes |

Phase 3 also re-runs **#643 / #648 / #649** (exercise, twin gate, guards),
which are closed but whose code paths all changed underneath them today.

**Deferred — waiting on parts:** #656 multi-cam and #671 vision
checkpoints. Nothing in this document needs a camera.

---

## Read this before powering on

**Three things changed today that have never touched hardware.** Treat
the first motion run as a first run, not a repeat of a known-good one.

1. **The twin is a different model.** It ran on Menagerie's SO-ARM100 —
   a different arm. It now runs on the real SO-101. Every collision
   baseline was re-derived against the new geometry.
2. **The pre-flight gate is a new code path.** It used to check 21 corner
   poses with lockstep interpolation; it now samples the motion profile
   and checks **10,084** poses. Same routine, different machinery.
3. **The arm's rest pose in the sim comes from a different source** —
   calibration.json rather than a model keyframe, because the SO-101
   ships none.

The reassurance: the new model independently re-derived **the exact
clearance envelope already shipped** (m2 span 40%, elbow 90°, m4 90°),
and still predicts both real bench collisions. So the safety envelope is
confirmed, not changed. But that is the model agreeing with itself —
Phase 2 is where it gets checked against your actual arm.

### Safety

- The **power switch is the hard e-stop.** Any keypress during the
  routine is the soft one (halt-and-hold, torque cuts on your Enter).
- Servos hold their last command while bus power is on. To make the arm
  safe to handle, cut power — do not rely on a tool having exited.
- Keep the workspace clear and the gripper empty for Phase 3.
- The USB-C control cable was not fouling the routine last time, but
  check it before the run.

---

## Phase 0 — no hardware (already done, listed so you can re-run it)

Everything below already passes on cell1. Re-run if you want a baseline
before touching anything:

```bash
cd ~/TendWright
uv run python -m hardware.units selftest
uv run python -m hardware.bench.guards selftest
uv run python -m sim.clip
uv run python -m sim.twin selftest
uv run python -m sim.twin validate     # must predict BOTH bench collisions
uv run python -m sim.twin frames
uv run python -m hardware.bench.camserve --selftest
```

All eight report OK. `sim.twin validate` is the important one — it is
the model asserting it still predicts the two collisions the bench
actually had.

---

## Phase 1 — read-only: does the readout match reality? (#647)

**No motion. Torque stays off. Nothing is commanded.**

Power on the bus, then:

```bash
uv run python -m hardware.bench.scan            # all six respond?
uv run python -m hardware.bench.calibrate show  # the readout under test
```

Leave the arm in its natural torque-off slump and compare. Expected, at
the calibrated rest pose:

| joint | reads about | what that means physically |
|---|---|---|
| m1 shoulder_pan | **+7°** | near mid-travel |
| m2 shoulder_lift | **−86°** | upper arm ~horizontal, leaned back into the fold |
| m3 elbow_flex | **+162°** | elbow closed right up |
| m4 wrist_flex | **+70°** | gripper tipped well down |
| m5 wrist_roll | **0°** | at its reference |
| m6 gripper | **~10% open** | jaws nearly shut |

**Pass:** the signs and rough magnitudes match what you're looking at.
Specifically — m2 negative means *leaned back toward the fold*, m3
positive means *closed*, m4 positive means *tipped down*. Getting a sign
backwards is the failure this is looking for.

**Expect a few degrees of slop.** The torque-off slump does not
reproduce itself; it has been seen up to ~5° off on m5. Off by a few
degrees is fine. Off by 90°, or the wrong sign, is a real finding.

Then jog one joint at a time and confirm the *direction* convention:

```bash
uv run python -m hardware.bench.jog --ids 1
```

`+` should move the joint in the direction its label claims. m1 and m5
are the two that were bench-verified before and are worth re-confirming,
because they are the two the model **cannot** derive — pan and roll have
no geometric reference to check themselves against.

> **Closes #647** if the readout matches. This is the ticket's only open
> item.

---

## Phase 2 — is this the right arm? (#670)

**No motion. Calipers.**

Every software test so far only proves the model agrees with *itself*.
Nothing in the repo can catch "we vendored the wrong robot" — only a
tape measure can.

```bash
uv run python -m sim.rig spec
```

Compare against the physical arm, centre of joint to centre of joint:

| span | model says | measured | ok? |
|---|---|---|---|
| m1 → m2 | **64.8 mm** | | |
| m2 → m3 | **116.0 mm** | | |
| m3 → m4 | **135.0 mm** | | |
| m4 → m5 | **63.7 mm** | | |
| m5 → m6 | **36.2 mm** | | |
| m1 centre above the mounting plane | **62.4 mm** | | |

**The two that matter most are m2→m3 (116 mm) and m3→m4 (135 mm)** —
the upper arm and forearm. They dominate where the tool ends up, and
they were *identical* between the SO-100 and SO-101 models, so if they
are wrong, both models were wrong and the whole gate is built on sand.

**Pass:** within a few millimetres. Joint centres are hard to eyeball,
so ±5 mm is a reasonable bar; a 20 mm miss is a real finding.

Optional second check — a distinctive pose. Jog m2 to tick **1780**,
m3 to **1292**, m4 to **2158**. All three read 0° and the arm should be
**standing straight up**, upper arm, forearm and gripper all in line.
That is the frame convention made visible: if it isn't straight, a zero
is wrong.

> **Closes #670** if the lengths check out. The old SO-100 model is
> still vendored (unused) specifically so reverting stays cheap until
> this passes — once it does, tell me and I'll delete it.

---

## Phase 3 — does the arm move like the sim? (#660)

**This is the motion phase.** Workspace clear, gripper empty, hand near
the power switch.

### 3a. Gate only, no motion

```bash
uv run python -m sim.twin exercise
```

Takes ~2.4 s on cell1. Expect:

```
collision gate: 10084 poses simulated
  note: joint 2 clamped up to 0.2 deg ...
  note: joint 6 clamped up to 1.0 deg ...
  clip: 21 poses, 100.7 s at speed 200 ticks/s, acceleration 15
CLEAR
```

The two clamp notes are **expected and benign** — the model's joint
limits are narrower than the arm's calibrated range on m2 and m6, by a
fraction of a degree and a degree. They are printed even on a clean gate
on purpose, because an anchoring mistake is loudest exactly when the
gate passes.

**Note that `100.7 s` figure.** That is the sim's prediction of how long
the routine's *motion* takes.

### 3b. The run

```bash
uv run python -m hardware.bench.exercise
```

It will pre-flight (gate + pose check), print the plan, and wait for
`y`. Watch for:

- **wake with no lurch** — torque comes on against the *present*
  position, so nothing should snap
- **smooth ramp to rest**, then joints sweeping one at a time
- **the clearance opening before the m2 sweep** — m3 and m4 open to 90°
  first, and the tool refuses to sweep until the encoders confirm they
  actually did
- **ends at rest, torque off**

**Time it with a stopwatch.** The sim predicts 100.7 s of motion.

> **Honest limit:** the real run will be **longer** than 100.7 s,
> because the clip duration counts motion only and the tool also waits
> for each joint to settle. So the check is loose: *not shorter than
> ~100 s, and not wildly longer.* If it comes in at 40 s or 400 s, the
> servo register semantics the profile rests on (speed = ticks/s,
> acceleration = ×100 ticks/s²) are wrong, and that is worth knowing
> immediately.

**Pass:** routine completes, no collision, no lurch, e-stop works if you
test it, arm ends torqued-off at rest, and the elapsed time is in the
right neighbourhood.

### 3c. The tight check — needs a small tool I haven't built

The loose stopwatch test cannot tell you whether the arm *tracked* the
profile, only whether it finished in roughly the right time. The real
test records encoder positions throughout the run and compares them
against the frames the sim showed at the same moments.

That is #660's acceptance item (`fa38d5ae`) and it needs a `--trace`
option on `exercise` that logs `(t, per-joint position)` to CSV, plus an
offline comparison. **Say the word and I'll build it before the
session** — it is small, and it is the difference between "the sim is
faithful to the arm's intent" and "the sim is faithful to the arm."

> **Advances #660.** The plan stays open regardless — the pose library,
> clip runner and FSM port are still to come.

---

## What to send me afterwards

Whatever you have — but these three specifically:

1. **The `calibrate show` output**, and whether the poses matched.
2. **The five link measurements** from Phase 2.
3. **The elapsed time** of the exercise run, and anything that looked
   wrong during it.

If something fails, the useful thing is *what you saw*, not a
diagnosis — I have the models and can chase the cause from a symptom.

---

## Deferred until the cameras land

- **#656 multi-cam** — picker, tile view, unplug isolation. Needs a
  second camera.
- **#671 vision checkpoints** — filed, not started. Needs cameras and
  AprilTags placed.
- **On-demand capture** (`/capture?label=...`) already ships and is
  selftested against fake cameras; it will work the moment a camera is
  plugged in. Worth exercising early — it is the primitive the
  checkpoints sit on.
