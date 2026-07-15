# Robotic CNC Machine-Tending Cell — Python Learning Project

A hands-on project to understand where Python lives in an automated
manufacturing cell (Hadrian-style), built as a ladder of prototypes that
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
| 5 | MES / scheduling / job mgmt | The "Opus/Flow" layer — what runs next, WIP tracking, traceability, OEE/throughput, quality records |
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
sim version exists. By the end you've built a miniature Hadrian cell in Python.

### P0 — Simulated cell digital twin
Load a UR5e arm URDF, a table, a mock "CNC" box with a door, a bin of blanks,
and a fixture in **PyBullet** (or MuJoCo). Script a hardcoded pick → load →
"machining" delay → unload cycle.
- **Learn:** URDF, forward/inverse kinematics, gripper constraints, scene setup
- **Libraries:** `pybullet`, `numpy`, `roboticstoolbox-python` / `ikpy`

### P1 — The cell orchestrator
Drive the sequence with a real finite state machine. Run the "CNC" as a
*separate* async service exposing an **OPC UA** interface (`asyncua`) with
states IDLE / MACHINING / DONE / DOOR_OPEN. The orchestrator (`asyncio` +
`transitions`) commands the robot and waits on CNC signals, with timeouts and
error recovery.
- **Learn:** soft-real-time orchestration, industrial handshaking, OPC UA (the
  real protocol), fault handling
- **This is the single most "Hadrian-core" skill in the ladder.**

### P2 — Vision-guided picking
Add a simulated camera in PyBullet; detect the blank's pose and feed pick
coordinates to the orchestrator. Then bring in a **webcam** over a bin and do
the same with OpenCV + ArUco/AprilTags, including **hand-eye calibration**.
- **Learn:** OpenCV, camera calibration, coordinate transforms, optionally a
  learned pose model
- **Libraries:** `opencv-python`, `pupil-apriltags`, `numpy`

### P3 — Quality inspection ML
A camera inspects the finished part: pass/fail on a defect, or a pixel→mm
dimensional check. Generate synthetic defect images in sim to train, then
validate on real photos.
- **Learn:** ML for inspection (a literal Opus capability), synthetic data,
  anomaly detection
- **Libraries:** `torch` or `scikit-learn`, `opencv`

### P4 — Mini-Opus (MES + dashboard)
A `FastAPI` + SQLite/Postgres backend with a job queue (part, program, qty),
logging each cycle from the orchestrator (timings + P3 pass/fail), computing
OEE/throughput, exposed via a `Streamlit` dashboard. The MES tells the
orchestrator what to run next.
- **Learn:** MES concepts, traceability, OEE — the web/data backend that *is*
  Opus/Flow

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
| Sim / digital twin | `pybullet`, `mujoco`, `genesis`, `roboticstoolbox-python`, `pinocchio`; RoboDK Python API |
| Kinematics / planning | `roboticstoolbox-python`, `ikpy`, MoveIt2 (`rclpy`), Drake |
| Robot comms | `ur_rtde` (UR), socket/URScript, `pymodbus`, `asyncua` (OPC UA), ROS2 `rclpy` |
| CNC | `pygcode`, `pyserial` + GRBL, MTConnect agent/client |
| Vision / ML | `opencv-python`, `numpy`, `pupil-apriltags`, `open3d`, `torch`, `scikit-learn`, `ultralytics` (YOLO) |
| Orchestration | `asyncio`, `transitions` / `python-statemachine` |
| MES / data | `FastAPI`, `SQLModel`/`SQLAlchemy`, Postgres/SQLite, `Streamlit`, InfluxDB/TimescaleDB, Grafana |

---

## 5. A note on ROS2

`rclpy` (Python) + MoveIt2 is a clean way to structure orchestration /
perception / planning as nodes, and it's worth learning. But it adds real
overhead, and Hadrian isn't ROS-centric. Recommended path: do **P0–P2 without
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

A complete miniature Hadrian cell for a few hundred dollars, where everything
above the two controllers is Python you write.

---

## 7. Status & next steps

- [ ] **P0** — Simulated cell digital twin (PyBullet UR5e load/unload)
- [ ] **P1** — Cell orchestrator (state machine + OPC UA CNC handshake)
- [ ] **P2** — Vision-guided picking (sim camera → webcam + AprilTags)
- [ ] **P3** — Quality inspection ML
- [ ] **P4** — Mini-Opus MES + dashboard
- [ ] **P5** — Telemetry & predictive maintenance
- [ ] **P6** — Capstone: hardware-in-the-loop cell

**Next:** scaffold P0 as runnable Python — a PyBullet scene with a UR5e loading
and unloading a mock CNC — as the foundation the orchestrator builds on.
