# Robotic CNC Machine-Tending Cell — Python Learning Project

A hands-on project to understand where Python lives in a modern automated
manufacturing cell, built as a ladder of prototypes that
starts in pure simulation and grows into a real hobby-scale cell.

**Core use case:** a robot arm tends a CNC machine. The loop never changes —
*CNC finishes → door opens → robot removes the finished part → robot loads a
fresh blank → clamp → door closes → cycle start → repeat* — with a camera
checking parts and a system deciding what runs next.

**Approach:** simulation first, then slot in cheap hobby hardware where a sim
version already exists.

**Goal:** learn *where and how Python is used* in this space by building each
differentiated layer yourself.

---

## 1. The reference stack

Seven layers, from steel-and-microseconds at the bottom to jobs-and-dashboards
at the top.

| Layer | Name | What it does |
|-------|------|--------------|
| 0 | Actuation & sensing | CNC, robot arm, gripper, pneumatic vises/fixtures, cameras, part-presence & door sensors, force/torque |
| 1 | Real-time control & I/O | CNC controller runs G-code; robot controller runs motion; PLC coordinates discrete I/O; safety PLC handles e-stops/light curtains; fieldbus (EtherCAT/PROFINET) or digital-I/O handshakes |
| 2 | Cell orchestration | The "cell controller" — sequences the loop, handshakes with the CNC, commands the robot, runs the state machine, recovers from faults |
| 3 | Perception / vision / ML | Find the blank, verify part presence, estimate pick pose, inspect finished parts, detect tool wear |
| 4 | Motion planning / digital twin | Generate & collision-check trajectories; simulate the whole cell before touching hardware |
| 5 | MES / scheduling / job mgmt | The factory-software layer — what runs next, WIP tracking, traceability, OEE/throughput, quality records |
| 6 | DFM / CAM automation | Ingest CAD (STEP), recognize features, estimate machinability/cost, parametrize toolpaths |

---

## 2. Where Python lives (and where it doesn't)

The encouraging part: almost every *differentiated, interesting* layer is
Python's home turf, and the layers Python is locked out of are the
commoditized vendor black boxes.

### Python's sweet spots
- **Cell orchestration** (Layer 2) — soft-real-time state machine + async I/O
- **Vision & ML** (Layer 3) — localization, inspection, predictive maintenance
- **Simulation & offline motion planning** (Layer 4) — via bindings
- **MES / scheduling / data backend & dashboards** (Layer 5)
- **CAD / CAM / DFM automation** (Layer 6)
- Machine-telemetry ingestion (MTConnect / OPC UA), commissioning scripts, glue

### Where Python does NOT belong
- **Hard real-time servo/motion loops** — robot & CNC controllers, C/C++ on vendor RTOS
- **Safety PLC logic** — certified IEC 61131 ladder
- **Low-level fieldbus timing**

**Mental model:** *the vendor controllers move the metal deterministically;
Python orchestrates, perceives, decides, and records above them.*

---

## 3. The prototype ladder

Each rung builds on the last. #0 is pure sim; hardware only slots in once a
sim version exists. By the end you've built a miniature automated machining
cell in Python.

### P0 — Simulated cell digital twin
Load a UR5e arm (MuJoCo Menagerie model), a table, a mock "CNC" box with a
door, a bin of blanks, and a fixture in **MuJoCo**. Script a hardcoded pick →
load → "machining" delay → unload cycle.
- **Learn:** MJCF scene modeling (MuJoCo also loads URDF), forward/inverse
  kinematics, gripper constraints, scene setup
- **Libraries:** `mujoco`, `mink` (differential IK), `numpy`

### P1 — The cell orchestrator
Drive the sequence with a real finite state machine — **hand-rolled** on a
shared plain-Python FSM base class (states, transitions, guards, hooks,
introspection; no FSM library), zero new dependencies. The "CNC" is a
**mock cell**: a MockCell interface with a sim backend (the P0 cell), a
scriptable fake, and a slot for the physical mock bay (printed nest
fixture + part-present switch via the Pico bridge — see the mock-bay
plan). Command-then-verify throughout: every load/unload is confirmed by
the part-present sensor, never assumed; faults recover in place with a
retry budget. (An OPC UA CNC handshake returns when a real CNC does,
P6-era.)
- **Learn:** soft-real-time orchestration, FSM design, sensor-verified
  handshaking, fault handling
- **This is the most industry-core skill in the ladder — the heart of every
  real machine-tending cell.**

### P2 — Vision-guided picking
Add a simulated camera in MuJoCo (offscreen rendering); detect the blank's
pose and feed pick
coordinates to the orchestrator. Then bring in a **webcam** over a bin and do
the same with OpenCV + ArUco/AprilTags, including **hand-eye calibration**.
- **Learn:** OpenCV, camera calibration, coordinate transforms, optionally a
  learned pose model
- **Libraries:** `opencv-python`, `numpy`, and the system `libapriltag`
  (ctypes binding in `hardware/bench/apriltag.py` — see #713.5 for why the
  `pupil-apriltags` package was dropped)

### P3 — Quality inspection ML
A camera inspects the finished part: pass/fail on a defect, or a pixel→mm
dimensional check. Generate synthetic defect images in sim to train, then
validate on real photos.
- **Learn:** ML for inspection (a staple of factory software), synthetic data,
  anomaly detection
- **Libraries:** `torch` or `scikit-learn`, `opencv`

### P4 — Mini-MES + dashboard
A `FastAPI` + SQLite/Postgres backend with a job queue (part, program, qty),
logging each cycle from the orchestrator (timings + P3 pass/fail), computing
OEE/throughput, exposed via a `Streamlit` dashboard. The MES tells the
orchestrator what to run next.
- **Learn:** MES concepts, traceability, OEE — the web/data backend that *is*
  the factory software

### P5 — Telemetry & predictive maintenance
Ingest machine telemetry (spindle load, cycle time, vibration) into a
time-series store and flag drift/tool-wear with simple ML. Stream from a real
GRBL/MTConnect source later.
- **Libraries:** `asyncua` / MTConnect, `pandas`, `scikit-learn`, Grafana/Streamlit

### P6 — Capstone: hardware-in-the-loop cell
Combine a cheap hobby arm + a **3018 GRBL mini-CNC** + a webcam. The
orchestrator runs the full loop: vision finds a blank → arm loads → GRBL runs a
real tiny engraving/milling program over serial → arm unloads → camera
inspects → MES logs. **GRBL runs the actual G-code motion** — your "vendor
controller" Python orchestrates but never replaces. It makes the
Python/not-Python boundary physical.

---

## 4. Python libraries by layer

| Layer | Libraries |
|-------|-----------|
| Sim / digital twin | `mujoco` (+ MuJoCo Menagerie models), `genesis`, `pinocchio`; RoboDK Python API |
| Kinematics / planning | `mink`, `roboticstoolbox-python`, MoveIt2 (`rclpy`), Drake |
| Robot comms | `ur_rtde` (UR), socket/URScript, `pymodbus`, `asyncua` (OPC UA), ROS2 `rclpy` |
| CNC | `pygcode`, `pyserial` + GRBL, MTConnect agent/client |
| Vision / ML | `opencv-python`, `numpy`, system `libapriltag` (own ctypes binding), `open3d`, `torch`, `scikit-learn`, `ultralytics` (YOLO) |
| Orchestration | plain Python (hand-rolled FSM base — see `orchestrator/fsm.py`), `asyncio` later if the cell goes concurrent |
| MES / data | `FastAPI`, `SQLModel`/`SQLAlchemy`, Postgres/SQLite, `Streamlit`, InfluxDB/TimescaleDB, Grafana |

**Project convention: dependencies must be actively maintained.** The sim
stack is MuJoCo (`mujoco` + `mink` + vendored MuJoCo Menagerie models);
PyBullet was rejected as stale (last release Jan 2025). Project tooling is
`uv` with a `pyproject.toml`.

---

## 5. A note on ROS2

`rclpy` (Python) + MoveIt2 is a clean way to structure orchestration /
perception / planning as nodes, and it's worth learning. But it adds real
overhead, and plenty of production cells aren't ROS-centric. Recommended path: do **P0–P2 without
ROS** to learn the fundamentals unobscured, then optionally re-architect **P6
in ROS2** — which also gives you a strong portfolio story about *why* a company
might build its own orchestration layer instead of adopting ROS.

---

## 6. Hobby hardware shopping list (for later rungs)

- USB webcam — vision projects (P2, P3)
- SO-101 / LeRobot-class arm (~$100–150), or a servo arm on an Arduino — the "robot"
- 3018 Pro CNC (~$120–200) running GRBL — the "CNC" that runs real G-code over serial
- Raspberry Pi or your laptop — the cell controller
- Optional: limit switches / a cheap force sensor — handshake realism

A complete miniature automated machining cell for a few hundred dollars, where everything
above the two controllers is Python you write.

---

## 7. Status & next steps

- [x] **P0** — Simulated cell digital twin (MuJoCo UR5e load/unload)
- [x] **P1** — Cell orchestrator (hand-rolled FSM + mock cell, command-then-verify)
- [ ] **P2** — Vision-guided picking (sim camera → webcam + AprilTags)
- [ ] **P3** — Quality inspection ML
- [ ] **P4** — Mini-MES + dashboard
- [ ] **P5** — Telemetry & predictive maintenance
- [ ] **P6** — Capstone: hardware-in-the-loop cell

**Running** (requires [uv](https://docs.astral.sh/uv/)):

```sh
uv sync                                    # create .venv, install deps
uv run python -m sim.run_cell              # P0: scripted loop in the viewer
uv run python -m sim.validate --verbose    # P0: headless validation
uv run python -m orchestrator.run_cell     # P1: the FSM drives the cell (--fault for a live pick-miss recovery)
uv run python -m orchestrator.validate     # P1: headless (happy path + 3 fault scenarios)
```

P0 simplification to revisit: the gripper fingers close around the part for
looks, but the hold itself is a toggled weld constraint (deterministic, and
now proximity-guarded — closing on air grabs nothing); the fixture clamp
works the same way.

**Next:** the physical mock CNC bay (printed nest fixture + part-present
switch + Pico bridge) so the same FSM can run against real sensors, then
P2 vision-guided picking.
