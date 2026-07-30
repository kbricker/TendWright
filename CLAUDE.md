# TendWright

Robotic CNC machine-tending cell — a Python learning project built as a ladder
of prototypes (P0–P6) that starts in pure simulation and grows into a real
hobby-scale hardware cell. Full project doc: `README.md` (mirrored in Hive as
spec `spec-tendwright-overview`).

**This is one of Kyle's PERSONAL projects, orchestrated by spark.**

## Hive

- **App:** TendWright (id 8)
- **Plans:** #604 (P0 sim) → #605 (P1 orchestrator) → #606 (P2 vision picking)
  → #607 (P3 inspection ML) → #608 (P4 MES) → #609 (P5 telemetry)
  → #610 (P6 hardware capstone). Rungs build strictly on each other — work them
  in order.

## Layout

| Folder | Component | Rungs |
|---|---|---|
| `sim/` | MuJoCo digital twin (arm, mock CNC, bin, fixture) | P0 |
| `orchestrator/` | Hand-rolled FSM base + cell FSM + MockCell backends | P1 |
| `vision/` | Pick-pose detection, calibration, inspection ML | P2, P3 |
| `mes/` | FastAPI job queue + OEE + Streamlit dashboard | P4 |
| `telemetry/` | Time-series ingestion + predictive maintenance | P5 |
| `hardware/` | GRBL serial driver, hobby-arm driver, HIL wiring | P6 |

## Running

- `uv sync` — set up `.venv` with mujoco/mink/numpy.
- `uv run python -m sim.run_cell` — P0 cell in the interactive viewer.
- `uv run python -m sim.validate --verbose` — headless P0 validation
  (contact policing, drop detection, per-step timeouts); exit 0 = pass.

## Cell controller (hardware bench)

- **cell1** — Minisforum UM350 (Ryzen 3550H, 8GB), **Ubuntu 26.04 LTS**
  (resolute, glibc 2.43), on Kyle's bench. Runs the camera + arm + Pico at P2+.
- **NOT headless** — it boots to `graphical.target` and Kyle uses the GNOME
  desktop directly when he is at the bench. The stack costs ~280 MB resident
  (gnome-shell alone is ~220 MB of the 5.87 GB usable), which is worth knowing
  when accounting for memory but is NOT to be "reclaimed": Kyle 2026-07-29,
  *"I DO use the desktop when im at the bench, so gnome is fine."* Dropping it
  is on the table only for later headless deployments as the cell matures.
- **spark CAN shut cell1 down**, and nothing else as root. A scoped rule
  grants `(root) NOPASSWD: /usr/sbin/poweroff, /usr/sbin/shutdown,
  /usr/bin/systemctl poweroff`. Everything else — `apt`, gdb attach
  (`ptrace_scope=1`) — is handed to Kyle as a command to run, and
  `ssh cell1 'sudo ...'` needs `ssh -t` for the password prompt.
  **Check with `sudo -n -l`, never `sudo -n true`**: the latter probes the
  general `(ALL:ALL) ALL` entry, which requires a password, so it reports
  failure even when a scoped grant exists. That mistake made spark tell Kyle
  it could not power the box down while the rule to do so was already live.
- `unattended-upgrades` is **active and enabled**, so packages update on their
  own. Account for that when a long soak runs: a service restarting mid-run is
  a confound.
- Full lab topology — every device, address, and the command that actuates it — is in **`docs/lab-inventory.md`**.
- **What the cameras can actually read** — measured tag sizes, the ~4 px/cell
  detection floor, which camera answers which question, and what fits on the
  arm — is in **`docs/tag-sensing.md`**. Read it before sizing or placing a
  tag; both of the mistakes it records came from reasoning past a measurement.
- Access: `ssh cell1` (alias in `~/.ssh/config`; key auth, works for Kyle and
  spark — same Windows user). User `kyle` is in `dialout`.
- Two more identical UM350s are spares/future roles (MES box etc.), not yet
  imaged.

## Rules

- **Git identity:** personal repo — commit as `Kyle Bricker <kyle.bricker@gmail.com>`
  (local git config override is set; never reset it). Remote is
  `github.com/kbricker/TendWright` over the default `github.com` SSH host
  (personal key), NOT the WonderForge `github-second.com` alias.
- **Public repo** — never commit secrets, tokens, or machine-specific paths.
- **Dependencies:** the doc lists candidate libraries per layer, but every
  dependency still goes through the no-new-deps gate — Kyle's explicit yes
  before adding anything. Dependencies must be **actively maintained** — no
  stale libraries (PyBullet was rejected for this; last release Jan 2025).
- **Approved stack (P0, per Kyle 2026-07-15):** `mujoco`, `mink`, `numpy`;
  `uv` + `pyproject.toml` for project tooling; MuJoCo Menagerie UR5e +
  gripper model files vendored into the repo (not a dependency).
- **Approved stack (bench toolkit, per Kyle 2026-07-21):** `feetech-servo-sdk`
  + `pyserial` (STS3215 bus control — minimal-SDK route, NOT LeRobot);
  `opencv-python` (camera preview).
- **Tag detection is the SYSTEM `libapriltag`, not a Python package** (Kyle
  2026-07-29, plan #713.5). `pupil-apriltags` was dropped entirely: it is at
  its final release and vendors AprilTag 3.1.x, which leaks 12*sz bytes per
  failed cluster in `quad_segment_maxima` — measured at 11.19 kB per
  `detect()` call. Install with `sudo apt install libapriltag3t64` (Ubuntu
  ships 3.4.5, which has the 2019 fix). The ctypes binding is ours:
  `hardware/bench/apriltag.py`. **There is no Windows build**, so detection
  is cell1-side; `cammanager` treats a missing library as a third veto
  (loud on stderr, `may_detect=false` in `/status`) rather than refusing to
  serve cameras.
- **Python:** simulation-first. Hardware code only lands after the sim version
  of the same rung works.
- **Review:** CodeRabbit is NOT on this repo. Before completing any plan, run
  internal subagent reviews and address findings: adversarial review (try to
  break it — edge cases, failure modes, unsafe hardware states), code-quality
  review, and functional review against the plan checklist. Safety-critical
  paths (anything that can move the arm) get the adversarial pass without
  exception.
