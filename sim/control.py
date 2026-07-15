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
MAX_ANG_SPEED = 1.5  # rad/s target orientation interpolation speed
POS_TOL = 0.006  # m, waypoint completion tolerance (physical pinch site)
ORI_TOL = 0.05  # rad, waypoint orientation tolerance (physical pinch site)
VEL_TOL = 0.02  # m/s, pinch site must be this settled to complete a waypoint

GRIPPER_OPEN = 0.0
# Partial close sized for the Ø40 mm blanks: pads meet the part surface
# instead of driving past it into each other (the hold itself is the weld).
GRIPPER_CLOSED = 140.0


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

    def goal_reached(self, data: mujoco.MjData) -> bool:
        """A waypoint is complete only when the PHYSICAL pinch site has
        arrived and settled — never trust the IK twin alone: the servos lag
        it by ~0.2 s, which is enough to weld parts in mid-air."""
        if not np.allclose(self._target_pos, self._goal_pos, atol=1e-9):
            return False
        if not np.allclose(self._target_quat, self._goal_quat, atol=1e-9):
            return False
        return (
            self.physical_position_error(data) < POS_TOL
            and self.physical_orientation_error(data) < ORI_TOL
            and self.pinch_speed(data) < VEL_TOL
        )

    def physical_position_error(self, data: mujoco.MjData) -> float:
        pinch = data.site(scene.PINCH_SITE).xpos
        return float(np.linalg.norm(pinch - self._goal_pos))

    def physical_orientation_error(self, data: mujoco.MjData) -> float:
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, data.site(scene.PINCH_SITE).xmat)
        diff = np.empty(3)
        mujoco.mju_subQuat(diff, self._goal_quat, quat)
        return float(np.linalg.norm(diff))

    def pinch_speed(self, data: mujoco.MjData) -> float:
        vel = np.empty(6)
        site_id = data.site(scene.PINCH_SITE).id
        mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_SITE,
                                 site_id, vel, 0)
        return float(np.linalg.norm(vel[3:]))

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

        rot_delta = np.empty(3)
        mujoco.mju_subQuat(rot_delta, self._goal_quat, self._target_quat)
        angle = np.linalg.norm(rot_delta)
        ang_step = MAX_ANG_SPEED * CONTROL_DT
        if angle <= ang_step:
            self._target_quat = self._goal_quat.copy()
        else:
            quat = self._target_quat.copy()
            mujoco.mju_quatIntegrate(quat, rot_delta / angle, ang_step)
            self._target_quat = quat

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
