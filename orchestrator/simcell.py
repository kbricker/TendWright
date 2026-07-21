"""SimCell — the MockCell backend that drives the P0 MuJoCo cell.

Task commands run StepProgram segments through a StepRunner; the
part-present "sensor" is derived from physics (a part within tolerance of
the fixture seat), so command-then-verify is exercised for real: a missed
pick leaves the bin blank where it was and the sensor honestly never sees
a seated part.
"""

from __future__ import annotations

import numpy as np
import mujoco

from sim import scene
from sim.control import CONTROL_DT, ArmController
from sim.cycle import StepProgram
from sim.runner import StepRunner, StepTimeout

from .cell import CellTaskError

SEAT_TOL = 0.012  # m — same seat tolerance the P0 validator uses
LIFT_Z = 0.85  # a held part must be at least this high after a pick


class SimCell:
    def __init__(self, viewer=None, realtime: bool = False,
                 verbose: bool = False):
        self.model = scene.build_cell_model()
        self.data = mujoco.MjData(self.model)
        scene.reset_home(self.model, self.data)
        self.controller = ArmController(self.model)
        self.program = StepProgram(self.controller, CONTROL_DT)
        self.runner = StepRunner(self.model, self.data, self.controller,
                                 viewer=viewer, realtime=realtime)
        if verbose:
            self.runner.on_step_start = lambda name: print(
                f"[sim {self.data.time:7.2f}s]   {name}")
        self._queue: list[str] = list(scene.PARTS)
        self._current: str | None = None  # part being cycled right now
        self._inject_pick_miss = False

    # ------------------------------------------------------------- MockCell
    def fetch_blank(self) -> None:
        if not self._queue:
            self._respawn()
        part = self._queue[0]
        grasp = not self._inject_pick_miss
        self._inject_pick_miss = False
        try:
            self.runner.run(self.program.pick(part, grasp=grasp))
        except StepTimeout as exc:
            raise CellTaskError(str(exc)) from exc
        # Device-level verify: is the blank actually up in the gripper?
        z = self.data.joint(f"{part}_free").qpos[2]
        if z < LIFT_Z:
            raise CellTaskError(f"pick missed — {part} still at z={z:.3f}")
        self._current = part

    def load_nest(self) -> None:
        if self._current is None:
            raise CellTaskError("load commanded with nothing held")
        try:
            self.runner.run(self.program.load(self._current))
        except StepTimeout as exc:
            raise CellTaskError(str(exc)) from exc

    def unload_to_tray(self) -> None:
        if self._current is None:
            raise CellTaskError("unload commanded with no active part")
        part = self._current
        try:
            self.runner.run(self.program.unload(part))
        except StepTimeout as exc:
            raise CellTaskError(str(exc)) from exc
        self._queue.remove(part)
        self._current = None

    def safe_retract(self) -> None:
        try:
            self.runner.run(self.program.retract_to_safe(self._current))
        except StepTimeout as exc:
            raise CellTaskError(f"recovery retract failed: {exc}") from exc
        self._current = None

    def part_present(self) -> bool:
        for part in scene.PARTS:
            pos = self.data.joint(f"{part}_free").qpos[:3]
            if np.linalg.norm(pos - scene.FIXTURE_SEAT) < SEAT_TOL:
                return True
        return False

    def mark_machined(self) -> None:
        if self._current is not None:
            self.model.geom(f"{self._current}_geom").rgba = scene.FINISHED_RGBA

    def dwell(self, seconds: float) -> None:
        self.runner.idle(seconds)

    def now(self) -> float:
        return float(self.data.time)

    # ----------------------------------------------------------- test hooks
    def inject_pick_failure_once(self) -> None:
        """Next fetch closes the fingers but never attaches the part — a
        real physics-level pick miss, not a faked sensor reading."""
        self._inject_pick_miss = True

    def _respawn(self) -> None:
        scene.respawn_parts(self.model, self.data)
        self._queue = list(scene.PARTS)
