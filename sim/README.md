# sim/

P0 — MuJoCo digital twin of the machine-tending cell (plan #604, done):
UR5e + Robotiq 2F-85 (vendored MuJoCo Menagerie models in `assets/menagerie/`,
attached via the MjSpec API), table, parts bin, mock CNC enclosure with a
sliding door, vise fixture, and outfeed tray.

- `assets/cell.xml` — the static cell scene (MJCF)
- `scene.py` — model composition, gravity compensation, weld helpers
- `control.py` — differential IK (mink) + gripper/door control; waypoints
  complete on the *physical* pinch pose, not the IK twin
- `cycle.py` — StepProgram (the motion segments: pick/load/machine/unload,
  shared with the P1 orchestrator's sim backend; grasp welds are
  proximity-guarded — closing on air grabs nothing) + TendingCycle, the
  P0 scripted loop over those segments
- `runner.py` — StepRunner: executes segments outside the P0 loop (used
  by orchestrator/simcell.py; optional viewer sync)
- `run_cell.py` — interactive viewer: `uv run python -m sim.run_cell`
- `validate.py` — headless validation with task-semantic checkpoints:
  `uv run python -m sim.validate --verbose`

P0 simplification: the grasp and fixture clamp are toggled weld constraints;
the fingers close (partially, sized to the Ø40 mm blanks) for realism only.
