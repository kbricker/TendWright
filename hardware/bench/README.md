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

1. **Before closing each joint** — connect that servo ALONE to the bus and
   program its ID (base → gripper = 1 → 6):
   `uv run python -m hardware.bench.set_id --new-id 1` … `--new-id 6`
2. **After wiring the full chain** — `scan` should list exactly IDs 1–6,
   sane voltage (~7.4 V nominal supply) and room temperature.
3. **Ranges + wiring order** — `monitor --ids 1-6`, move each joint by
   hand, confirm the right column moves and note each joint's usable
   tick range (feeds the jog soft limits and later the arm driver).
4. **First powered motion** — `jog --id N` one joint at a time, small
   steps, hand near the power switch.
5. **First trajectory** — `teach record`, move the arm through a simple
   arc by hand, then `teach replay --speed 0.25` with the workspace clear.
6. **Camera scouting** — `campreview` while trying wall-mount positions;
   check tag detection at
   candidate distances/angles (print `docs/bench-apriltags.html` at 100%
   scale — tag36h11 IDs 0–7 at 40 mm).

## Safety notes

- `set_id` writes EEPROM — one servo on the bus at a time, always.
- `jog` starts torque OFF and every unbound key is an e-stop; soft limits
  default to 200 ticks inside the 0–4095 range until real joint limits
  are measured (then pass `--min/--max`).
- `teach replay` defaults to 25% speed, prompts before moving, approaches
  the start pose extra slowly, and torques off on exit or Ctrl+C.
- All motion commands use modest servo-side speed/acceleration caps; the
  arm should never snap.
