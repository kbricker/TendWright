"""The scripted P0 machine-tending cycle.

A flat list of steps (move / gripper / weld / door / dwell) is generated for
each part and executed one at a time: pick a blank from the bin, load it into
the CNC fixture, clamp, close the door, "machine" it, then unload the
finished part to the outfeed tray. After all parts are done the blanks
respawn in the bin and the loop starts over. P1 replaces this hardcoded
script with a real FSM + OPC UA handshake.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import mujoco
import numpy as np

from . import scene
from .control import ArmController

YAW = 0.0  # fingers open along world-x: clears bin walls and vise jaws

TRAVEL_Z = 0.95  # safe height over the table outside the CNC
BIN_GRASP_Z = 0.776  # pinch height at a blank standing in the bin
CNC_STAGE = np.array([0.35, 0.24, 0.90])  # in front of the door opening
CNC_ENTRY_Z = 0.88  # travel height inside the enclosure (clears the roof)
SEAT_Z = scene.FIXTURE_SEAT[2] + 0.006  # pinch height at the seated part
TRAY_PLACE_Z = 0.79
MACHINING_TIME = 2.0  # seconds of simulated "cutting"
GRIP_SETTLE = 0.4  # seconds for the fingers to close/open around a part


@dataclass
class Step:
    name: str
    start: Callable[[mujoco.MjModel, mujoco.MjData], None]
    done: Callable[[mujoco.MjModel, mujoco.MjData], bool]
    timer: float = field(default=0.0)


class TendingCycle:
    """Executes the scripted cycle; call tick() once per control step."""

    def __init__(self, model: mujoco.MjModel, controller: ArmController, dt: float):
        self.model = model
        self.controller = controller
        self.dt = dt
        self.loops_completed = 0
        self.parts_completed = 0
        self.steps: list[Step] = self._build_loop()
        self.index = 0
        self._step_started = False

    # ------------------------------------------------------------- primitives
    def _move(self, label: str, pos: np.ndarray, yaw: float = YAW) -> Step:
        ctrl = self.controller
        return Step(
            name=label,
            start=lambda m, d: ctrl.set_goal(pos, yaw),
            done=lambda m, d: ctrl.goal_reached(),
        )

    def _dwell(self, label: str, seconds: float) -> Step:
        def done(m: mujoco.MjModel, d: mujoco.MjData, s=seconds) -> bool:
            step = self.steps[self.index]
            step.timer += self.dt
            return step.timer >= s

        return Step(name=label, start=lambda m, d: None, done=done)

    def _gripper(self, label: str, closed: bool) -> Step:
        ctrl = self.controller

        def done(m: mujoco.MjModel, d: mujoco.MjData) -> bool:
            step = self.steps[self.index]
            step.timer += self.dt
            return step.timer >= GRIP_SETTLE

        return Step(
            name=label,
            start=lambda m, d: ctrl.set_gripper(d, closed),
            done=done,
        )

    def _weld(self, label: str, eq_name: str, active: bool) -> Step:
        return Step(
            name=label,
            start=lambda m, d: scene.set_weld_active(m, d, eq_name, active),
            done=lambda m, d: True,
        )

    def _door(self, label: str, open_: bool) -> Step:
        ctrl = self.controller
        return Step(
            name=label,
            start=lambda m, d: ctrl.set_door(d, open_),
            done=lambda m, d: ctrl.door_at(d, open_),
        )

    def _mark_finished(self, part: str) -> Step:
        def start(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            m.geom(f"{part}_geom").rgba = scene.FINISHED_RGBA

        return Step(name=f"{part}: machined", start=start, done=lambda m, d: True)

    def _respawn(self) -> Step:
        def start(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            scene.respawn_parts(m, d)

        return Step(name="respawn blanks", start=start, done=lambda m, d: True)

    def _count_part(self, part: str) -> Step:
        def start(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            self.parts_completed += 1

        return Step(name=f"{part}: cycle complete", start=start, done=lambda m, d: True)

    # ------------------------------------------------------------ the script
    def _part_steps(self, part: str) -> list[Step]:
        bin_pos = scene.BIN_SLOTS[part]
        tray_pos = scene.TRAY_SLOTS[part]
        seat = scene.FIXTURE_SEAT
        above_bin = np.array([bin_pos[0], bin_pos[1], TRAVEL_Z])
        at_bin = np.array([bin_pos[0], bin_pos[1], BIN_GRASP_Z])
        above_seat = np.array([seat[0], seat[1], CNC_ENTRY_Z])
        at_seat = np.array([seat[0], seat[1], SEAT_Z])
        above_tray = np.array([tray_pos[0], tray_pos[1], TRAVEL_Z])
        at_tray = np.array([tray_pos[0], tray_pos[1], TRAY_PLACE_Z])
        grasp = scene.GRASP_EQ[part]
        clamp = scene.CLAMP_EQ[part]

        return [
            # --- pick a blank from the bin
            self._move(f"{part}: above bin", above_bin),
            self._move(f"{part}: descend to blank", at_bin),
            self._gripper(f"{part}: close gripper", closed=True),
            self._weld(f"{part}: grasp on", grasp, True),
            self._move(f"{part}: lift blank", above_bin),
            # --- load it into the CNC
            self._door(f"{part}: door open (load)", open_=True),
            self._move(f"{part}: stage at door", CNC_STAGE),
            self._move(f"{part}: enter CNC", above_seat),
            self._move(f"{part}: lower into fixture", at_seat),
            self._weld(f"{part}: clamp on", clamp, True),
            self._weld(f"{part}: grasp off", grasp, False),
            self._gripper(f"{part}: open gripper", closed=False),
            self._move(f"{part}: raise clear", above_seat),
            self._move(f"{part}: exit CNC", CNC_STAGE),
            self._door(f"{part}: door close", open_=False),
            # --- machine it
            self._dwell(f"{part}: machining", MACHINING_TIME),
            self._mark_finished(part),
            # --- unload to the tray
            self._door(f"{part}: door open (unload)", open_=True),
            self._move(f"{part}: re-enter CNC", above_seat),
            self._move(f"{part}: down to part", at_seat),
            self._gripper(f"{part}: close gripper", closed=True),
            self._weld(f"{part}: grasp on", grasp, True),
            self._weld(f"{part}: clamp off", clamp, False),
            self._move(f"{part}: raise part", above_seat),
            self._move(f"{part}: exit with part", CNC_STAGE),
            self._door(f"{part}: door close (idle)", open_=False),
            self._move(f"{part}: above tray", above_tray),
            self._move(f"{part}: lower to tray", at_tray),
            self._weld(f"{part}: grasp off", grasp, False),
            self._gripper(f"{part}: open gripper", closed=False),
            self._move(f"{part}: retract", above_tray),
            self._count_part(part),
        ]

    def _build_loop(self) -> list[Step]:
        steps: list[Step] = []
        for part in scene.PARTS:
            steps.extend(self._part_steps(part))
        steps.append(self._respawn())
        return steps

    # --------------------------------------------------------------- runtime
    @property
    def current_step(self) -> Step:
        return self.steps[self.index]

    def tick(self, data: mujoco.MjData) -> str | None:
        """Advance the script; returns the name of a step when it starts."""
        started: str | None = None
        step = self.steps[self.index]
        if not self._step_started:
            step.start(self.model, data)
            self._step_started = True
            started = step.name
        if step.done(self.model, data):
            self.index += 1
            self._step_started = False
            if self.index >= len(self.steps):
                self.index = 0
                self.loops_completed += 1
                for s in self.steps:
                    s.timer = 0.0
        return started
