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

**Everything speaks degrees now** (plan #647). Positions print in each
joint's ratified convention with raw ticks in parentheses — m2's rest is
`-90.0°`, the gripper reads `% open`. The conventions live in
`calibration.json`; edit a joint's `frame` there to change how it reads.

**Motion is gated twice** (plans #648/#649): before anything moves, the
routine is simulated against the digital twin from the arm's MEASURED
pose and refused if it would collide; during motion, held joints are
re-verified from the encoders every sample and the arm halts-and-holds
if one sags. A command sent is never assumed to be a joint moved.

## Servo / arm tools (`uv run python -m hardware.bench.<tool>`)

| Command | What it does | Moves the arm? |
|---|---|---|
| `set_id --new-id N` | Program ONE loose servo to bus ID N (1–6, base→gripper). Refuses if >1 servo on the bus. Power off between servos. | no |
| `scan` / `scan --full` | List servos: ID, model, firmware, voltage (~12 V follower), temp, position. Fast = IDs 0–20; `--full` = 0–253. | no |
| `monitor [--ids 1-6] [--hz 10]` | Torque OFF (confirm; support the arm), live positions while you move joints by hand. Watch for 0↔4095 jumps = horn across the encoder wrap. | no (cuts torque) |
| `jog --id N [--step 20 \| --step-deg D] [--min T --max T]` | Keyboard single-joint moves. Keys: `+`/`-` jog, `[`/`]` step size, `c` go to the joint's zero pose (gripper: half open), `t` torque toggle, `q` quit, ANY other key = e-stop. **Soft limits default to that joint's CALIBRATED range** when `calibration.json` exists — no need to pass min/max. | yes |
| `calibrate capture [--ids 1-6] [--out FILE]` | Guided, torque-off, CANNOT move the arm: hand-sweep ranges → rest pose → direction nudges → `calibration.json` (atomic; re-runs merge per joint: `--ids 3` redoes one joint). | no (cuts torque) |
| `calibrate show [--in FILE]` | Print + validate the calibration table — ranges in each joint's ratified units with its convention spelled out underneath. | no |
| `teach record [--out teach.json] [--ids 1-6] [--hz 10]` | Torque off (confirm), sample a hand-moved trajectory; Enter stops. | no (cuts torque) |
| `teach replay --in FILE [--speed 0.25]` | Confirm, sync goals (no lurch), slow-approach frame 0, wait for real arrival, stream frames. Workspace clear. | yes |
| `exercise [--ids RANGE] [--span 70] [--speed 1.0] [--cal FILE] [--no-gate]` | Scripted limber-up: wake (no lurch) → rest → sweep each joint through `span`% of its calibrated range, one at a time (others held), distal first: 4 wrist_flex → 5 wrist_roll → 6 gripper → 3 elbow → 2 shoulder → 1 base pan. **The whole routine is simulated in the twin first and refused if it would collide.** During the shoulder sweep the elbow AND wrist hold 90° open and m2 is capped at 40% span — the envelope the twin derived after 45° proved insufficient; those holds are then verified from the encoders before the sweep starts and re-checked while it runs. Ends at rest, torque off. Needs ALL SIX joints in `calibration.json`; starts only from the rest pose; gripper empty. | yes |

### exercise, the 10-second version

```
ssh -t cell1
cd ~/TendWright && uv run python -m hardware.bench.exercise
```

Refuses unless the arm is at its torque-off rest slump. ANY key during
motion = e-stop: the arm halts and HOLDS; torque cuts when you press Enter
with a hand on it. Same held cut on obstruction timeouts, guard
violations, Ctrl+C, and serial faults — it never drops the arm
unannounced. `--ids 2,3` sweeps only those joints (all six are still
woken and held). Good end-of-session health check.

If it refuses with a contact report, believe the twin before you reach
for `--no-gate` — it has already been right about two real collisions.

## Simulation tools (`uv run python -m sim.<tool>`) — no hardware needed

| Command | What it does |
|---|---|
| `twin check` | Is the rest pose contact-free? Also prints which joint mappings are still provisional. |
| `twin exercise [--span N]` | Dry-run the exercise routine through the collision gate — same simulation the arm runs before it moves. |
| `twin derive-clearance` | Scan sweep span × elbow × wrist holds for a contact-free combination. How the shipped envelope was chosen. |
| `twin validate` | Regression: the twin must predict both real bench collisions. Run after ANY change to the model, calibration, or mappings. |
| `rig spec` | Joint centerpoints + rotation axes, origin at m1's centerpoint, and the center-to-center link lengths (calipers-checkable against the real arm). |
| `rig where [--deg a,b,c,d,e,f]` | Where every joint and the tool point land for a pose, in mm. Pose authoring without touching hardware. |

## Camera + mock bay

| Command | What it does |
|---|---|
| `uv run python -m hardware.bench.campreview [--camera 0] [--width 1280 --height 720]` | Live window with tag36h11 AprilTag overlay + FPS. `--grab N --outdir D` = headless snapshots. `--calib FILE.npz` applies camera intrinsics. |
| `uv run python -m hardware.pico.watch` | Stream the KW12-3 seat-switch state from the Pico bridge; state should flip on press. |

### Camera scouting quickstart (ELP-USBFHD01M-L36)

The camera is UVC plug-and-play — no driver setup. It's the manual-focus
variant: focus by twisting the lens barrel, judged by the overlay, ONCE at
the final working distance; then never touch it.

```
# live view with tag overlay + FPS (needs a display on cell1)
uv run python -m hardware.bench.campreview --width 1920 --height 1080

# headless: grab 5 stills to check a mount position over plain ssh
uv run python -m hardware.bench.campreview --width 1920 --height 1080 \
    --grab 5 --outdir /tmp/scout
```

Both tools force MJPEG — expect ~30 fps at 1080p / ~60 fps at 720p; a
reading near 5 fps means the format negotiation regressed, flag it.

Remote view from the desk (`camserve`, no display needed on cell1).
Cameras come from `cameras.json`, not flags — wire a camera, then:

```
uv run python -m hardware.bench.cameras discover   # paste-ready entries
uv run python -m hardware.bench.cameras check      # registry vs reality
uv run python -m hardware.bench.camserve           # serve every camera
# then on any LAN machine:
#   http://cell1:8081/                    picker: pick a camera
#   http://cell1:8081/all                 3x3 tile view
#   http://cell1:8081/cam/<name>/         one camera, full resolution
#   http://cell1:8081/status              JSON state of every camera
#   curl -o snap.jpg http://cell1:8081/cam/<name>/snapshot
# raw video: --no-tags · different port: --listen N · Ctrl+C stops it
# viewer only (no interval stills): --no-stills
# NO AUTH - home LAN only, never port-forward it
```

A camera is opened only while something is watching it, so the shared
USB2 uplink carries only what is actually in use. Any camera with
`still_interval_s` set writes full-resolution stills to `stills/<name>/`
on that interval with or without a viewer — that is the primary use;
live views are the scouting aid.

- Judge mount positions by what the SOFTWARE detects, not your eye:
  print `docs/bench-apriltags.html` at 100% scale (tag36h11 IDs 0–7,
  40 mm) and require solid corner locks at the candidate distance/angle.
- Wall-mount starting point: ~28–32" up, ~20–25° downward tilt.
- Focus the barrel LAST, at the chosen distance, then mark the spot.
- Printed brackets live in `cad/camera-mount/` (45° and 60° down, plus a
  slot cap). Regenerate after a parameter change:
  `blender --background --python cad/camera-mount/generate_mounts.py`.

### Known-harmless camera messages

- **"Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=
  wayland to run on Wayland anyway."** — cell1 runs GNOME on Wayland and
  OpenCV's Qt ships only the `xcb` plugin, so the window goes through
  XWayland. Prints once, costs nothing. Do NOT "fix" it by forcing
  `QT_QPA_PLATFORM=wayland` — that plugin isn't in the wheel and the
  window then fails to open at all.
- **"WRN: Matrix is singular."** — from `libapriltag.so`, not our code:
  the detector's homography solve hit a degenerate quad (something
  tag-shaped seen too obliquely to resolve). Harmless, and a sign the
  detector is actually looking. Printed from C to stderr, so no Python
  logging setting will suppress it. A steady stream of them means the
  camera is seeing tag-like clutter — reposition or retighten focus.
- **"Note that Qt no longer ships fonts."** — FIXED, plan #651 follow-up
  (2026-07-25); recorded here because the fix is non-obvious and easy to
  undo. `cv2` overwrites `QT_QPA_FONTDIR` **at import time** to its own
  bundled fonts dir, which ships EMPTY, so Qt had no fonts at all and
  repeated the complaint until it flooded the terminal. `campreview`
  re-points it at the system DejaVu fonts *after* `import cv2` — setting
  it before the import is silently clobbered. If this ever comes back,
  check that `_fix_qt_fonts()` still runs after the import.
- **Two by-path entries per camera** (`...-usb-0:1:1.0-...` and
  `...-usbv2-0:1:1.0-...`) — one physical camera, published under both
  the USB3 and USB2 halves of the same controller. `cameras discover`
  reports one entry per camera and names the alias; use the canonical
  (non-`v2`) path. Registering both spellings would be the same camera
  twice under two names.

### Wiring the camera bus

- Powered hub only (~200 mA/camera), on one of cell1's four main ports —
  those share one USB2 uplink; the other two ports are a separate
  controller, which is where a SECOND hub would go.
- **UARTs never go behind the camera hub** — the servo adapter and the
  Pico plug directly into cell1, so a hub reset can't drop the motion bus
  mid-move.
- 4 m of cable route per camera (1 m pigtail + 3 m extension max). Label
  the hub ports: a camera's identity IS its port, so moving it changes
  its registry entry.
- Keep cable runs clear of the arm's 376 mm reach from the m1 centerpoint
  — a dangling cable is grabbable, and the twin does not model it.

## Sim / orchestrator (reference)

| Command | What it does |
|---|---|
| `uv run python -m sim.validate` | Headless P0 validator: full pick→nest→dwell→unload cycle with task-semantic checkpoints. |

## Files the tools read/write

- `calibration.json` — per-joint `{id, name, min, rest, max, sign, frame}`
  from `calibrate capture`; consumed by `exercise`, `jog`, `monitor`, the
  twin, and the rig. `frame` is the hand-ratified display convention (v2)
  — edit it to change how a joint reads; re-capturing a joint drops its
  frame on purpose, since the geometry may have changed. Commit it after
  capture.
- `cameras.json` — the camera registry: name, location, stable by-path
  identity, capture profiles, still interval. Commit it; `stills/` is
  gitignored.
- `teach.json` (or `--out` name) — recorded trajectories for `teach replay`.

## Bring-up order

`scan` → `monitor` → `calibrate capture` → commit calibration.json →
`jog` (one joint, small steps) → `teach` record/replay → `exercise`.
Details: `docs/arm-bring-up.md`; gotchas (12 V vs 7.4 V servos, wrap
re-mounts): `docs/build-day-calibration.md`.
