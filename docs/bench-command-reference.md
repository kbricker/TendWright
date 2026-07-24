# Bench Command Reference (SO-101 + camera + mock bay)

One-page operator cheat sheet. Everything runs from the repo root on cell1
(`ssh -t cell1`, then `cd ~/TendWright` — the `-t` matters: the motion
tools refuse to start without a real terminal, since keypresses are the
e-stop). Full detail: `hardware/bench/README.md`.

**Universal:** serial port is auto-detected; override with `--port`.
`--yes` skips a tool's confirm prompt, NEVER its safety checks.
**Exit codes:** 0 done · 1 aborted · 2 error (one line + hint) ·
3 operator e-stop · 130 Ctrl+C.
**The power switch is the only hard e-stop.** A yanked USB adapter leaves
servos powered and HOLDING their last command.

## Servo / arm tools (`uv run python -m hardware.bench.<tool>`)

| Command | What it does | Moves the arm? |
|---|---|---|
| `set_id --new-id N` | Program ONE loose servo to bus ID N (1–6, base→gripper). Refuses if >1 servo on the bus. Power off between servos. | no |
| `scan` / `scan --full` | List servos: ID, model, firmware, voltage (~12 V follower), temp, position. Fast = IDs 0–20; `--full` = 0–253. | no |
| `monitor [--ids 1-6] [--hz 10]` | Torque OFF (confirm; support the arm), live positions while you move joints by hand. Watch for 0↔4095 jumps = horn across the encoder wrap. | no (cuts torque) |
| `jog --id N [--step 20] [--min T --max T]` | Keyboard single-joint moves. Keys: `+`/`-` jog, `[`/`]` step size, `c` center (2048), `t` torque toggle, `q` quit, ANY other key = e-stop. Use min/max from `calibrate show`. | yes |
| `calibrate capture [--ids 1-6] [--out FILE]` | Guided, torque-off, CANNOT move the arm: hand-sweep ranges → rest pose → direction nudges → `calibration.json` (atomic; re-runs merge per joint: `--ids 3` redoes one joint). | no (cuts torque) |
| `calibrate show [--in FILE]` | Print + validate the calibration table. | no |
| `teach record [--out teach.json] [--ids 1-6] [--hz 10]` | Torque off (confirm), sample a hand-moved trajectory; Enter stops. | no (cuts torque) |
| `teach replay --in FILE [--speed 0.25]` | Confirm, sync goals (no lurch), slow-approach frame 0, wait for real arrival, stream frames. Workspace clear. | yes |
| `exercise [--ids RANGE] [--span 70] [--speed 1.0] [--cal FILE]` | Scripted limber-up: wake (no lurch) → rest → sweep each joint through `span`% of its calibrated range, one at a time (others held), distal first: 4 wrist_flex → 5 wrist_roll → 6 gripper → 3 elbow → 2 shoulder → 1 base pan (rest is a compact fold — unfold the light end before the heavy joints move). The elbow holds ~45° open during the shoulder sweep (a refolded elbow gets pressed into the table), then refolds. Ends at rest, torque off. Needs ALL SIX joints in `calibration.json`; starts only from the rest pose; gripper empty. | yes |

### exercise, the 10-second version

```
ssh -t cell1
cd ~/TendWright && uv run python -m hardware.bench.exercise
```

Refuses unless the arm is at its torque-off rest slump. ANY key during
motion = e-stop: the arm halts and HOLDS; torque cuts when you press Enter
with a hand on it. Same held cut on obstruction timeouts, Ctrl+C, and
serial faults — it never drops the arm unannounced. `--ids 2,3` sweeps
only those joints (all six are still woken and held). Good end-of-session
health check.

## Camera + mock bay

| Command | What it does |
|---|---|
| `uv run python -m hardware.bench.campreview [--camera 0] [--width 1280 --height 720]` | Live window with tag36h11 AprilTag overlay + FPS. `--grab N --outdir D` = headless snapshots. `--calib FILE.npz` applies camera intrinsics. |
| `uv run python -m hardware.pico.watch` | Stream the KW12-3 seat-switch state from the Pico bridge; state should flip on press. |

## Sim / orchestrator (reference)

| Command | What it does |
|---|---|
| `uv run python -m sim.validate` | Headless P0 validator: full pick→nest→dwell→unload cycle with task-semantic checkpoints. |

## Files the tools read/write

- `calibration.json` — per-joint `{id, name, min, rest, max, sign}` from
  `calibrate capture`; consumed by `exercise` (and later the arm driver).
  Commit it after capture.
- `teach.json` (or `--out` name) — recorded trajectories for `teach replay`.

## Bring-up order

`scan` → `monitor` → `calibrate capture` → commit calibration.json →
`jog` (one joint, small steps) → `teach` record/replay → `exercise`.
Details: `docs/arm-bring-up.md`; gotchas (12 V vs 7.4 V servos, wrap
re-mounts): `docs/build-day-calibration.md`.
