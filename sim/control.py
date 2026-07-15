"""Arm control: differential IK (mink) + gripper/door/weld helpers.

The controller runs IK on a bare arm-only model (scene.build_ik_model) whose
base sits at the same world pose as in the cell, so end-effector targets are
plain cell/world coordinates. Each control tick it moves an interpolated
target toward the active waypoint, solves one differential-IK step, and
writes the resulting joint positions to the cell model's position servos.
"""

from __future__ import annotations

import mink
import mujoco
import numpy as np

from . import scene

CONTROL_DT = 0.01  # seconds between IK/control updates (physics runs faster)
MAX_LIN_SPEED = 0.5  # m/s target interpolation speed
POS_TOL = 0.006  # m, waypoint completion tolerance
ORI_TOL = 0.05  # rad-ish (rotational error norm) completion tolerance

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 255.0


def topdown_quat(yaw: float) -> np.ndarray:
    """Orientation (wxyz) with the tool axis pointing straight down and the
    fingers opening along world-x when yaw = pi/2."""
    half = 0.5 * yaw
    return np.array([0.0, np.cos(half), np.sin(half), 0.0])


class ArmController:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.ik_model = scene.build_ik_model()
        self.configuration = mink.Configuration(self.ik_model)

        self.ee_task = mink.FrameTask(
            frame_name=scene.PINCH_SITE,
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.5,
            lm_damping=1.0,
        )
        self.posture_task = mink.PostureTask(self.ik_model, cost=1e-3)
        self.tasks = [self.ee_task, self.posture_task]

        # Arm joints share names between the cell model and the IK model.
        self.ik_qpos_idx = np.array(
            [self.ik_model.joint(n).qposadr[0] for n in scene.ARM_JOINTS]
        )
        self.cell_actuator_ids = np.array(
            [model.actuator(n).id for n in scene.ARM_ACTUATORS]
        )
        self.finger_actuator_id = model.actuator(scene.FINGER_ACTUATOR).id
        self.door_actuator_id = model.actuator(scene.DOOR_ACTUATOR).id
        self.door_qpos_adr = model.joint(scene.DOOR_JOINT).qposadr[0]

        # Start at home.
        q = self.ik_model.qpos0.copy()
        q[self.ik_qpos_idx] = scene.HOME_QPOS
        self.configuration.update(q)
        self.posture_task.set_target_from_configuration(self.configuration)

        pinch = self.configuration.get_transform_frame_to_world(
            scene.PINCH_SITE, "site"
        )
        self._target_pos = pinch.translation().copy()
        self._target_quat = pinch.rotation().wxyz.copy()
        self._goal_pos = self._target_pos.copy()
        self._goal_quat = self._target_quat.copy()

    # ------------------------------------------------------------------ moves
    def set_goal(self, pos: np.ndarray, yaw: float) -> None:
        """Start moving the end effector toward a new top-down waypoint."""
        self._goal_pos = np.asarray(pos, dtype=float).copy()
        self._goal_quat = topdown_quat(yaw)
        # Orientation snaps to the goal immediately; position interpolates.
        self._target_quat = self._goal_quat.copy()

    def goal_reached(self) -> bool:
        if not np.allclose(self._target_pos, self._goal_pos, atol=1e-9):
            return False
        err = self.ee_task.compute_error(self.configuration)
        return (
            np.linalg.norm(err[:3]) < POS_TOL
            and np.linalg.norm(err[3:]) < ORI_TOL
        )

    def position_error(self) -> float:
        return float(np.linalg.norm(self.ee_task.compute_error(self.configuration)[:3]))

    def tick(self, data: mujoco.MjData) -> None:
        """One control step: advance the interpolated target, solve IK, and
        write servo setpoints into the cell's actuators."""
        delta = self._goal_pos - self._target_pos
        dist = np.linalg.norm(delta)
        step = MAX_LIN_SPEED * CONTROL_DT
        if dist <= step:
            self._target_pos = self._goal_pos.copy()
        else:
            self._target_pos = self._target_pos + delta * (step / dist)

        target = mink.SE3.from_rotation_and_translation(
            mink.SO3(self._target_quat), self._target_pos
        )
        self.ee_task.set_target(target)
        vel = mink.solve_ik(
            self.configuration, self.tasks, CONTROL_DT, solver="daqp", damping=1e-3
        )
        self.configuration.integrate_inplace(vel, CONTROL_DT)
        data.ctrl[self.cell_actuator_ids] = self.configuration.q[self.ik_qpos_idx]

    # -------------------------------------------------------- gripper / door
    def set_gripper(self, data: mujoco.MjData, closed: bool) -> None:
        data.ctrl[self.finger_actuator_id] = GRIPPER_CLOSED if closed else GRIPPER_OPEN

    def set_door(self, data: mujoco.MjData, open_: bool) -> None:
        data.ctrl[self.door_actuator_id] = scene.DOOR_OPEN if open_ else scene.DOOR_CLOSED

    def door_at(self, data: mujoco.MjData, open_: bool) -> bool:
        target = scene.DOOR_OPEN if open_ else scene.DOOR_CLOSED
        return abs(data.qpos[self.door_qpos_adr] - target) < 0.01
