# Glossary

The vocabulary this project actually uses, by layer. Every number here
is the constant in the code, not a rounding.

---

## 1. The arm

| term | meaning |
|---|---|
| **m1..m6** / **j1..j6** | The six servos, base to gripper: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`. `m` and `j` are the same joint; `j` is the shorthand in tables. |
| **tick** | The servo's native position unit. 4096 ticks per revolution, so **1 tick = 0.0879°**. Single-turn range 0–4095, centre 2048. Everything the bus sends or reads is ticks. |
| **rest** / **fold** / **slump** | Where gravity parks the arm with torque off — a compact fold. These servos have **no brakes**, so cutting power anywhere else makes the arm fall. Rest is the captured version of that position, and it is the safe place to start and end. |
| **overhaul** | A joint being driven backwards by the load it is carrying, rather than holding. j4 does this under gravity. |

## 2. Calibration — what turns a tick into a meaning

Lives in `calibration.json`, captured by `calibrate capture`, torque-off.

| term | meaning |
|---|---|
| **min / rest / max** | The joint's measured travel limits and its rest tick. A pose outside min..max is refused at load, not clamped. |
| **sign** | Which way the encoder counts when the joint moves in its canonical positive direction. Measured, not chosen. |
| **frame** | The *ratified display convention*: which tick reads as **zero**, and which direction is **+**. This is what lets tools speak degrees instead of ticks. Hand-edited and human-approved, never captured — and re-capturing a joint **drops** its frame, because the geometry may have moved. |
| **span** | For `exercise`, the percentage of a joint's calibrated range to sweep. `--span 70` means 70% of min..max. |
| **anchor** | How the twin ties a joint's *model* zero to the *real* arm. Three kinds: `"frame"` (probed from the model — the pitch chain j2/j3/j4), `"rest"` (model zero pinned to physical rest — j1/j5), `"min"` (the gripper). |

## 3. Poses and clips — how a move is written down

| term | meaning |
|---|---|
| **pose** | A named whole-arm position, authored in **degrees** (gripper: % open) so the file is readable. Lives in `poses.json`. |
| **pose library** | `poses.json` — the named poses, in one place, so a height that matters is measured once and every clip using it moves when it is re-measured. |
| **clip** | An ordered walk through poses at one motion profile. A JSON file (`crane-tour.json`, `pan-wiggle.json`). **This is the unit you run.** |
| **edge** | One move: pose A → pose B. The unit the gate validates and the trace records. A clip of 21 poses has 20 edges. |
| **motion profile** | `speed` (ticks/s) and `acceleration` (×100 ticks/s²). These are not a model of the arm's motion — **they are written straight into the servo's registers**, so the sim animates the same numbers the arm obeys. |
| **trapezoid** | The shape of a servo move: ramp up, cruise, ramp down. Short moves never reach cruise and are **triangular** — that is the common case, not an edge case. Each joint runs its own ramp and arrives when its own travel is done, so a short-travel joint finishes first and then holds. |
| **carry-forward** | Inside a clip, a pose names only the joints it *changes*; the rest inherit from the pose before. The first pose must name every joint. |
| **holds** | Joints that must **physically already be** where a pose puts them — read back from the encoders before the edge into it is commanded, and re-checked while it plays. "A commanded hold is not a held joint." |

## 4. The sim

| term | meaning |
|---|---|
| **twin** | The MuJoCo model of the arm (`sim/twin.py`). Answers one question: *would this pose or path collide?* |
| **rig** | Forward kinematics (`sim/rig.py`). Answers *where is each joint, in mm* — the spatial vocabulary for authoring poses. |
| **rig frame** | Origin = m1's centrepoint, **+X = the direction the arm reaches, +Z = up, +Y = the arm's LEFT** (right-handed, as in East-North-Up). Say it out loud before reading a `y`: this was mirrored for five days and nothing caught it. |
| **gate** | The check that refuses a move the twin predicts will hit something. Runs **twice**: once on the whole clip before anything moves, then again per-edge from the arm's *measured* position as it goes. |
| **contact margin** | **5 mm**. Geoms are each given half (2.5 mm) because MuJoCo sums the two, so "contact" is reported before metal actually touches. |
| **structural nesting** | Link pairs that legitimately nest inside each other in the fold. Proximity never fails them — only real penetration does. |
| **refuse / refusal** | The house rule: a tool that cannot establish a precondition **errors with an actionable hint** rather than guessing or proceeding. A refusal is the system working. |
| **validation time vs runtime** | A bad edge caught offline by the twin (no bus, no power) versus discovered with the arm moving. The whole point is to move failures to the first. |

## 5. Running and measuring

| term | meaning |
|---|---|
| **approach** | The move from wherever the arm actually is to the clip's *first* pose. Commanded deliberately slower (**120 ticks/s**) because nothing has gated that move from that exact position. |
| **phase** | A trace's label for which edge was playing. **Phase 0 = the approach; phase *n* = the clip's *n*th edge.** |
| **settle** | Waiting for a joint to actually arrive, not just be commanded. Tolerance **25 ticks** (~2.2°). Time spent settling is reported separately from deviation, because the clip does not model a pause. |
| **drift** | How far off-plan the arm was when an edge *began* — reported, not merely tolerated, because a joint that consistently enters off-plan is telling you something. Reported from **10 ticks**. |
| **trace** | A CSV of encoder positions recorded throughout a run, plus a `#` header naming the clip, the profile and the calibration that produced it. Written on **every** exit path, including an e-stop. |
| **deviation** | The gap between what the servo was *told* (the trapezoid) and where it actually was. Some is expected — a real joint lags under gravity. It is the size of the gap, not a defect count. |
| **e-stop** | Any keypress during motion: halts and **holds** (torque stays on). The power switch is the hard one — and it *drops* the arm rather than stopping it. Unattended running has no keypress e-stop at all. |

## 6. The crane family — this arm's shorthand

j2, j3 and j4 all rotate about the **same axis in the same sense**, so
their sum *is* the gripper's angle from vertical. Pinning
**j4 = 180 − j2 − j3** keeps the gripper hanging plumb down, which
collapses the arm into a crane's three independent controls:

| term | joint | meaning |
|---|---|---|
| **slew** | j1 | which way it faces |
| **radius** | j2 | how far out |
| **hoist** | j3 | how high (j4 compensates to stay plumb) |

`CRANE_<slew><radius><hoist>` names a pose in that grid — `CRANE_233` is
slew 2 (centre), radius 3 (far), hoist 3 (low).

**The jaw hangs ~82 mm below the tool point.** A forward-kinematics tool
height is *not* a clearance; nine of the first eleven poses went through
the table on that mistake.

## 7. Tools

All run as `uv run python -m <module>` from the repo root.

| tool | what it does |
|---|---|
| `calibrate` | Captures per-joint range, rest and direction. Torque off, read-only. |
| `jog` | Nudge one joint by hand-eye. Operator-driven. |
| `teach` | Record and replay a hand-posed motion. |
| `runner` | **Play a clip.** `show` gates it without touching the bus; `run` drives the arm; `example` writes a starter clip anchored to this arm. |
| `exercise` | The fixed warm-up routine that sweeps each joint. Predates clips; now runs as one. |
| `batch` | Sequences several traced runs and collects the evidence. |
| `posemachine` | The pose library as a state machine. `validate` asks the twin about **every** edge offline and reports which are refused and whether any pose is unreachable. |
| `sim.twin` | The collision model. `selftest` checks its logic; `validate` checks it still predicts the two collisions the real arm actually had. |
| `sim.trace` | Lays a recorded run over the sim's prediction, per phase. The acceptance test for "the arm goes where the sim said". |
| `camserve` | MJPEG camera streams on cell1:8081. No auth — home LAN only. |
| `kasa` | The switched outlets. Powering off requires the arm verified at rest from the encoders. |

**`selftest` vs `validate`:** a *selftest* checks a module's own logic
with no hardware and no real data. A *validate* checks the real model or
the real library. Both run on the desk; neither touches the arm.

---

## The two rules everything else follows from

1. **Refuse, never guess.** A wrong answer delivered confidently is the
   worst outcome available. Every silent fallback in this codebase has
   eventually produced one.
2. **Acceptance is paired with refusal.** A guard is not proven by a run
   that passed — it is proven by a deliberately bad input that got
   stopped. Every selftest here asserts both directions.
