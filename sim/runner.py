"""StepRunner — execute step lists against the sim outside the P0 loop.

Used by the P1 orchestrator's sim backend to run one StepProgram segment
at a time (the P0 TendingCycle has its own equivalent loop in tick()).
Optionally syncs a mujoco viewer and paces to real time for watching.
"""

from __future__ import annotations

import time

import mujoco

from .control import CONTROL_DT, ArmController
from .cycle import Step


class StepTimeout(Exception):
    def __init__(self, step: Step, timeout: float):
        super().__init__(f"step '{step.name}' did not complete in {timeout}s")
        self.step = step


class StepRunner:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 controller: ArmController, step_timeout: float = 30.0,
                 viewer=None, realtime: bool = False):
        self.model = model
        self.data = data
        self.controller = controller
        self.step_timeout = step_timeout
        self.viewer = viewer
        self.realtime = realtime
        self.physics_per_control = max(1, round(CONTROL_DT / model.opt.timestep))
        self.on_step_start = None  # optional callback(name: str)

    def _tick(self) -> None:
        tick_start = time.perf_counter()
        self.controller.tick(self.data)
        for _ in range(self.physics_per_control):
            mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()
        if self.realtime:
            elapsed = time.perf_counter() - tick_start
            if elapsed < CONTROL_DT:
                time.sleep(CONTROL_DT - elapsed)

    def run(self, steps: list[Step]) -> None:
        """Run steps to completion; raises StepTimeout on a stuck step."""
        for step in steps:
            step.timer = 0.0
            deadline = self.data.time + self.step_timeout
            if self.on_step_start:
                self.on_step_start(step.name)
            step.start(self.model, self.data)
            while not step.done(self.model, self.data):
                self._tick()
                if self.data.time > deadline:
                    raise StepTimeout(step, self.step_timeout)

    def idle(self, seconds: float) -> None:
        """Advance sim time with the controller holding its current goal."""
        end = self.data.time + seconds
        while self.data.time < end:
            self._tick()
