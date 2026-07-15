"""Interactive viewer for the P0 cell: watch the tending cycle run.

Usage: uv run python -m sim.run_cell
"""

from __future__ import annotations

import time

import mujoco
import mujoco.viewer

from . import scene
from .control import CONTROL_DT, ArmController
from .cycle import TendingCycle


def main() -> None:
    model = scene.build_cell_model()
    data = mujoco.MjData(model)
    scene.reset_home(model, data)
    controller = ArmController(model)
    cycle = TendingCycle(model, controller, CONTROL_DT)
    physics_per_control = max(1, round(CONTROL_DT / model.opt.timestep))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            tick_start = time.perf_counter()
            controller.tick(data)
            started = cycle.tick(data)
            if started:
                print(f"[{data.time:7.2f}s] {started}")
            for _ in range(physics_per_control):
                mujoco.mj_step(model, data)
            viewer.sync()
            elapsed = time.perf_counter() - tick_start
            if elapsed < CONTROL_DT:
                time.sleep(CONTROL_DT - elapsed)


if __name__ == "__main__":
    main()
