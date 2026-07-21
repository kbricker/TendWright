"""Machine-tending motion programs for the sim cell.

StepProgram builds waypoint/actuation step lists for each cell task (pick,
load, machine, unload) against the P0 scene. Two consumers:

  * TendingCycle — the P0 scripted demo loop (concatenates every segment
    and plays them forever);
  * the P1 orchestrator's sim backend (orchestrator/simcell.py) — runs one
    segment at a time under FSM control.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import mujoco
import numpy as np

from . import scene
from .control import ArmController

# Fingers open along world-x at yaw = pi/2 (see control.topdown_quat): that
# clears the vise jaws (at +/-y of the seat) and the bin walls at the slots.
YAW = np.pi / 2

TRAVEL_Z = 0.95  # safe height over the table outside the CNC
BIN_GRASP_Z = 0.776  # pinch height at a blank standing in the bin
CNC_STAGE = np.array([0.35, 0.24, 0.90])  # in front of the door opening
CNC_ENTRY_Z = 0.88  # travel height inside the enclosure (clears the door-opening top edge)
SEAT_Z = scene.FIXTURE_SEAT[2] + 0.006  # pinch height at the seated part
TRAY_PLACE_Z = 0.79
MACHINING_TIME = 2.0  # seconds of simulated "cutting" (P0 scripted loop)
GRIP_SETTLE = 0.4  # seconds for the fingers to close/open around a part

SAFE_POSE = np.array([0.35, -0.10, TRAVEL_Z])  # neutral spot over open table


@dataclass
class Step:
    name: str
    start: Callable[[mujoco.MjModel, mujoco.MjData], None]
    done: Callable[[mujoco.MjModel, mujoco.MjData], bool]
    timer: float = 0.0


class StepProgram:
    """Builds step lists for cell tasks; stateless between calls."""

    def __init__(self, controller: ArmController, dt: float):
        self.controller = controller
        self.dt = dt

    # ------------------------------------------------------------- primitives
    def move(self, label: str, pos: np.ndarray, yaw: float = YAW) -> Step:
        ctrl = self.controller
        return Step(
            name=label,
            start=lambda m, d: ctrl.set_goal(pos, yaw),
            done=lambda m, d: ctrl.goal_reached(d),
        )

    def timed(self, label: str, seconds: float,
              start: Callable[[mujoco.MjModel, mujoco.MjData], None]) -> Step:
        """A step that runs `start` and completes after `seconds` of sim time.
        The done() closure captures its own Step, so it stays correct no
        matter which step is current when it is called."""
        step = Step(name=label, start=start, done=lambda m, d: True)

        def done(m: mujoco.MjModel, d: mujoco.MjData) -> bool:
            step.timer += self.dt
            return step.timer >= seconds

        step.done = done
        return step

    def dwell(self, label: str, seconds: float) -> Step:
        return self.timed(label, seconds, lambda m, d: None)

    def gripper(self, label: str, closed: bool) -> Step:
        ctrl = self.controller
        return self.timed(label, GRIP_SETTLE,
                          lambda m, d: ctrl.set_gripper(d, closed))

    def weld(self, label: str, eq_name: str, active: bool) -> Step:
        return Step(
            name=label,
            start=lambda m, d: scene.set_weld_active(m, d, eq_name, active),
            done=lambda m, d: True,
        )

    GRASP_RANGE = 0.05  # m — a grasp weld only latches within this reach

    def grasp_on(self, part: str) -> Step:
        """Activate the grasp weld ONLY if the part is actually between the
        fingers — closing on air (or on a part that fell elsewhere) grabs
        nothing, like a real gripper."""

        def start(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            pinch = d.site(scene.PINCH_SITE).xpos
            part_pos = d.joint(f"{part}_free").qpos[:3]
            if np.linalg.norm(pinch - part_pos) <= self.GRASP_RANGE:
                scene.set_weld_active(m, d, scene.GRASP_EQ[part], True)

        return Step(name=f"{part}: grasp on", start=start, done=lambda m, d: True)

    def door(self, label: str, open_: bool) -> Step:
        ctrl = self.controller
        return Step(
            name=label,
            start=lambda m, d: ctrl.set_door(d, open_),
            done=lambda m, d: ctrl.door_at(d, open_),
        )

    def mark_finished(self, part: str) -> Step:
        def start(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            m.geom(f"{part}_geom").rgba = scene.FINISHED_RGBA

        return Step(name=f"{part}: machined", start=start, done=lambda m, d: True)

    def respawn(self) -> Step:
        def start(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            scene.respawn_parts(m, d)

        return Step(name="respawn blanks", start=start, done=lambda m, d: True)

    # -------------------------------------------------------------- segments
    def pick(self, part: str, grasp: bool = True) -> list[Step]:
        """Pick a blank from its bin slot. grasp=False induces a clean miss
        for fault-recovery validation: the arm goes through the motions but
        never closes the fingers or attaches, leaving the blank untouched
        in its slot (so a retry can genuinely succeed)."""
        bin_pos = scene.BIN_SLOTS[part]
        above = np.array([bin_pos[0], bin_pos[1], TRAVEL_Z])
        at = np.array([bin_pos[0], bin_pos[1], BIN_GRASP_Z])
        steps = [
            self.move(f"{part}: above bin", above),
            self.move(f"{part}: descend to blank", at),
        ]
        if grasp:
            steps.append(self.gripper(f"{part}: close gripper", closed=True))
            steps.append(self.grasp_on(part))
        steps.append(self.move(f"{part}: lift blank", above))
        return steps

    def load(self, part: str) -> list[Step]:
        """Carry the held blank into the CNC and seat it in the fixture."""
        seat = scene.FIXTURE_SEAT
        above_seat = np.array([seat[0], seat[1], CNC_ENTRY_Z])
        at_seat = np.array([seat[0], seat[1], SEAT_Z])
        return [
            self.door(f"{part}: door open (load)", open_=True),
            self.move(f"{part}: stage at door", CNC_STAGE),
            self.move(f"{part}: enter CNC", above_seat),
            self.move(f"{part}: lower into fixture", at_seat),
            self.weld(f"{part}: clamp on", scene.CLAMP_EQ[part], True),
            self.weld(f"{part}: grasp off", scene.GRASP_EQ[part], False),
            self.gripper(f"{part}: open gripper", closed=False),
            self.move(f"{part}: raise clear", above_seat),
            self.move(f"{part}: exit CNC", CNC_STAGE),
            self.door(f"{part}: door close", open_=False),
        ]

    def machine(self, part: str, seconds: float = MACHINING_TIME) -> list[Step]:
        return [
            self.dwell(f"{part}: machining", seconds),
            self.mark_finished(part),
        ]

    def unload(self, part: str) -> list[Step]:
        """Take the finished part out of the fixture to its tray slot."""
        seat = scene.FIXTURE_SEAT
        tray_pos = scene.TRAY_SLOTS[part]
        above_seat = np.array([seat[0], seat[1], CNC_ENTRY_Z])
        at_seat = np.array([seat[0], seat[1], SEAT_Z])
        above_tray = np.array([tray_pos[0], tray_pos[1], TRAVEL_Z])
        at_tray = np.array([tray_pos[0], tray_pos[1], TRAY_PLACE_Z])
        return [
            self.door(f"{part}: door open (unload)", open_=True),
            self.move(f"{part}: re-enter CNC", above_seat),
            self.move(f"{part}: down to part", at_seat),
            self.gripper(f"{part}: close gripper", closed=True),
            self.grasp_on(part),
            self.weld(f"{part}: clamp off", scene.CLAMP_EQ[part], False),
            self.move(f"{part}: raise part", above_seat),
            self.move(f"{part}: exit with part", CNC_STAGE),
            self.door(f"{part}: door close (idle)", open_=False),
            self.move(f"{part}: above tray", above_tray),
            self.move(f"{part}: lower to tray", at_tray),
            self.weld(f"{part}: grasp off", scene.GRASP_EQ[part], False),
            self.gripper(f"{part}: open gripper", closed=False),
            self.move(f"{part}: retract", above_tray),
        ]

    def retract_to_safe(self) -> list[Step]:
        """Recovery: release EVERY hold and park the arm at a neutral pose.

        Unconditionally deactivates all grasp AND clamp welds — a fault can
        strike before the caller's bookkeeping knows which part is held or
        clamped (e.g. mid-fetch, mid-load), and a leaked weld either drags a
        part around welded to an open gripper or leaves the vise clamped on
        a part the controller has forgotten."""
        steps: list[Step] = []
        for part in scene.PARTS:
            steps.append(self.weld(f"{part}: grasp off (recover)",
                                   scene.GRASP_EQ[part], False))
            steps.append(self.weld(f"{part}: clamp off (recover)",
                                   scene.CLAMP_EQ[part], False))
        steps.extend([
            self.gripper("recover: open gripper", closed=False),
            self.move("recover: park", SAFE_POSE),
            self.door("recover: door close", open_=False),
        ])
        return steps


class TendingCycle:
    """The P0 scripted demo loop: every segment for every part, forever."""

    def __init__(self, model: mujoco.MjModel, controller: ArmController, dt: float):
        self.model = model
        self.controller = controller
        self.dt = dt
        self.program = StepProgram(controller, dt)
        self.loops_completed = 0
        self.parts_completed = 0
        self.steps: list[Step] = self._build_loop()
        self.index = 0
        self._step_started = False

    def _count_part(self, part: str) -> Step:
        def start(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            self.parts_completed += 1

        return Step(name=f"{part}: cycle complete", start=start,
                    done=lambda m, d: True)

    def _build_loop(self) -> list[Step]:
        p = self.program
        steps: list[Step] = []
        for part in scene.PARTS:
            steps.extend(p.pick(part))
            steps.extend(p.load(part))
            steps.extend(p.machine(part))
            steps.extend(p.unload(part))
            steps.append(self._count_part(part))
        steps.append(p.respawn())
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
