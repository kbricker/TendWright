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
| `campreview` | Live camera window with tag36h11 AprilTag overlay + FPS; `--grab N` for headless snapshots. |

## Assembly-day order

1. **Before closing each joint** — POWER OFF the bus, connect that servo
   ALONE (two servos sharing the factory ID answer as one and can both be
   re-ID'd together — the guard cannot see that), power on, then program
   its ID (base → gripper = 1 → 6):
   `uv run python -m hardware.bench.set_id --new-id 1` … `--new-id 6`
2. **After wiring the full chain** — `scan` should list exactly IDs 1–6,
   sane voltage (~7.4 V nominal supply) and room temperature.
3. **Ranges + wiring order** — `monitor --ids 1-6` (it cuts torque after
   a confirm prompt — support the arm, it drops under gravity), move each
   joint by hand, confirm the right column moves and note each joint's
   usable tick range (feeds the jog soft limits and later the arm driver).
4. **First powered motion** — `jog --id N` one joint at a time, small
   steps, hand near the power switch.
5. **First trajectory** — `teach record --out wave.json`, move the arm
   through a simple arc by hand, then
   `teach replay --in wave.json --speed 0.25` with the workspace clear.
6. **Camera scouting** — `campreview` while trying wall-mount positions;
   check tag detection at
   candidate distances/angles (print `docs/bench-apriltags.html` at 100%
   scale — tag36h11 IDs 0–7 at 40 mm).

## Safety notes

- `set_id` writes EEPROM — power off, connect ONE servo, power on. Always.
- `monitor` and `teach record` cut torque on every listed servo (after a
  confirm prompt): a raised arm drops under gravity — support it first.
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
