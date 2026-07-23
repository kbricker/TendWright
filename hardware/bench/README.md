# hardware/bench — SO-101 + camera bring-up tools

Standalone CLI tools for assembly day and the vision scouting session.
All run as `uv run python -m hardware.bench.<tool>` from the repo root
(on cell1 or any machine with the bus adapter / camera attached), and all
exit with a clean one-line error when hardware is absent.

Serial port: auto-detected (`/dev/ttyACM*`/`/dev/ttyUSB*` on Linux, lone
COM port on Windows); override with `--port`.

| Tool | What it does |
|---|---|
| `set_id` | Program ONE loose servo to a bus ID (1–6). Refuses if it sees more than one servo. |
| `scan` | Enumerate the bus: ID, model, firmware, voltage, temp, position per servo. |
| `monitor` | Torque OFF, live positions at ~10 Hz while you move joints by hand. |
| `jog` | Keyboard single-joint moves with soft limits; any unbound key = e-stop. |
| `teach` | `record`: sample a hand-moved trajectory to JSON (torque off). `replay`: play it back slowly after a confirm prompt. |
| `calibrate` | `capture`: guided torque-off capture of each joint's range, the arm's rest pose, and each joint's direction sign → `calibration.json` (atomic write; re-runs merge per joint). `show`: print + validate the file. |
| `campreview` | Live camera window with tag36h11 AprilTag overlay + FPS; `--grab N` for headless snapshots. |

## Assembly-day order

1. **Before closing each joint** — POWER OFF the bus, connect that servo
   ALONE (two servos sharing the factory ID answer as one and can both be
   re-ID'd together — the guard cannot see that), power on, then program
   its ID (base → gripper = 1 → 6):
   `uv run python -m hardware.bench.set_id --new-id 1` … `--new-id 6`
2. **After wiring the full chain** — `scan` should list exactly IDs 1–6,
   sane voltage (~12 V — the SO-ARM101 Pro kit's follower servos and PSU
   are the 12 V variant; only the leader runs lower) and room temperature.
3. **Ranges + wiring order** — `monitor --ids 1-6` (it cuts torque after
   a confirm prompt — support the arm, it drops under gravity), move each
   joint by hand, confirm the right column moves and that no joint's
   readout jumps 0↔4095 mid-swing (that's a horn mounted across the
   encoder wrap — re-mount it one spline tooth over NOW, while it's cheap).
4. **Calibration** — `calibrate capture` (torque off throughout; it cannot
   move the arm). Guided: hand-sweep each joint end to end, pose the arm
   at rest once, nudge each joint in its positive direction. Writes
   `calibration.json` — the ranges feed the jog soft limits and later the
   arm driver / sim mapping. Re-do a single joint after a horn remount
   with `calibrate capture --ids N`; inspect any time with
   `calibrate show`.
5. **First powered motion** — `jog --id N` one joint at a time, small
   steps, hand near the power switch (use the measured range from
   `calibrate show` for `--min/--max`).
6. **First trajectory** — `teach record --out wave.json`, move the arm
   through a simple arc by hand, then
   `teach replay --in wave.json --speed 0.25` with the workspace clear.
7. **Camera scouting** — `campreview` while trying wall-mount positions;
   check tag detection at
   candidate distances/angles (print `docs/bench-apriltags.html` at 100%
   scale — tag36h11 IDs 0–7 at 40 mm).

## Joint sign convention (canonical)

`calibrate capture` records each joint's `sign`: +1 if the encoder counts
up when the joint moves in its **canonical positive direction**, defined
here (base → gripper). This wording is the convention — the arm driver and
MuJoCo mapping consume the recorded sign, never re-guess it:

| ID | Joint | Positive direction |
|---|---|---|
| 1 | shoulder_pan | arm swings counterclockwise, viewed from above |
| 2 | shoulder_lift | upper arm rises away from the base |
| 3 | elbow_flex | forearm rises toward the upper arm (elbow closes) |
| 4 | wrist_flex | gripper tips upward |
| 5 | wrist_roll | gripper rolls counterclockwise, viewed head-on from the front |
| 6 | gripper | jaws close |

## Safety notes

- `set_id` writes EEPROM — power off, connect ONE servo, power on. Always.
- `monitor`, `teach record`, and `calibrate capture` cut torque on every
  listed servo (after a confirm prompt): a raised arm drops under gravity —
  support it first.
- `calibrate` never enables torque and never writes a goal position — it is
  read-only on the bus apart from the torque-off itself, so it cannot move
  the arm. It writes `calibration.json` atomically (temp file + rename); an
  interrupted run never corrupts an existing calibration.
- `jog` starts torque OFF (and enforces it) and every unbound key is an
  e-stop (exit code 3); enabling torque re-syncs to the current hand-moved
  position, so it holds in place. Soft limits default to 200 ticks inside
  the 0–4095 range until real joint limits are measured (`--min/--max`).
- `teach replay` defaults to 25% speed, prompts before moving, approaches
  the start pose slowly and polls until every joint has ARRIVED before
  streaming frames, and torques off on exit or Ctrl+C.
- Cleanup torque-offs are retried and never fail silently — if a servo
  can't be confirmed safe you get a loud warning; the POWER SWITCH is the
  real e-stop. If the USB adapter is yanked mid-session, servos keep bus
  power and hold their last command — same answer: power switch.
- All motion commands use modest servo-side speed/acceleration caps; the
  arm should never snap.
