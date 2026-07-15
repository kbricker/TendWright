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
| `orchestrator/` | Cell FSM + OPC UA CNC handshake | P1 |
| `vision/` | Pick-pose detection, calibration, inspection ML | P2, P3 |
| `mes/` | FastAPI job queue + OEE + Streamlit dashboard | P4 |
| `telemetry/` | Time-series ingestion + predictive maintenance | P5 |
| `hardware/` | GRBL serial driver, hobby-arm driver, HIL wiring | P6 |

## Running

- `uv sync` — set up `.venv` with mujoco/mink/numpy.
- `uv run python -m sim.run_cell` — P0 cell in the interactive viewer.
- `uv run python -m sim.validate --verbose` — headless P0 validation
  (contact policing, drop detection, per-step timeouts); exit 0 = pass.

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
- **Python:** simulation-first. Hardware code only lands after the sim version
  of the same rung works.
