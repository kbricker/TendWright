"""Scene composition for the P0 digital-twin cell.

Builds the full cell model by attaching the vendored MuJoCo Menagerie UR5e
and Robotiq 2F-85 (sim/assets/menagerie/) onto the static cell scene
(sim/assets/cell.xml) with the MjSpec attach API, so the vendored files are
never edited. Also builds a bare arm-only model for differential IK.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

ASSETS_DIR = Path(__file__).parent / "assets"
CELL_XML = ASSETS_DIR / "cell.xml"
UR5E_XML = ASSETS_DIR / "menagerie" / "universal_robots_ur5e" / "ur5e.xml"
GRIPPER_XML = ASSETS_DIR / "menagerie" / "robotiq_2f85" / "2f85.xml"

ARM_PREFIX = "ur5e/"
GRIPPER_PREFIX = "2f85/"  # nested under the arm -> "ur5e/2f85/..."

PINCH_SITE = "ur5e/2f85/pinch"
GRIPPER_BASE_BODY = "ur5e/2f85/base"
GRIPPER_MOUNT_BODY = "ur5e/2f85/base_mount"
WRIST_BODY = "ur5e/wrist_3_link"
FINGER_ACTUATOR = "ur5e/2f85/fingers_actuator"
DOOR_ACTUATOR = "cnc_door_actuator"
DOOR_JOINT = "cnc_door_joint"
DOOR_OPEN = 0.40
DOOR_CLOSED = 0.0

ARM_JOINTS = [
    "ur5e/shoulder_pan_joint",
    "ur5e/shoulder_lift_joint",
    "ur5e/elbow_joint",
    "ur5e/wrist_1_joint",
    "ur5e/wrist_2_joint",
    "ur5e/wrist_3_joint",
]
ARM_ACTUATORS = [
    "ur5e/shoulder_pan",
    "ur5e/shoulder_lift",
    "ur5e/elbow",
    "ur5e/wrist_1",
    "ur5e/wrist_2",
    "ur5e/wrist_3",
]
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

PARTS = ["part0", "part1", "part2"]
# Standing poses of the blanks in the bin; also used to respawn after a full loop.
BIN_SLOTS = {
    "part0": np.array([0.40, -0.44, 0.771]),
    "part1": np.array([0.50, -0.38, 0.771]),
    "part2": np.array([0.40, -0.32, 0.771]),
}
FIXTURE_SEAT = np.array([0.35, 0.50, 0.83])  # part center when seated in the vise
TRAY_SLOTS = {
    "part0": np.array([0.06, -0.55, 0.78]),
    "part1": np.array([0.15, -0.55, 0.78]),
    "part2": np.array([0.24, -0.55, 0.78]),
}

BLANK_RGBA = np.array([0.69, 0.72, 0.76, 1.0])
FINISHED_RGBA = np.array([0.83, 0.62, 0.24, 1.0])

GRASP_EQ = {part: f"grasp_{part}" for part in PARTS}
CLAMP_EQ = {part: f"clamp_{part}" for part in PARTS}


def _delete_keyframes(spec: mujoco.MjSpec) -> None:
    """Drop child keyframes: they are sized for the child model alone and
    would be padded with invalid (all-zero) free-joint quaternions on attach."""
    while spec.keys:
        spec.delete(spec.keys[0])


def _align_options(spec: mujoco.MjSpec) -> None:
    """Give a spec the same solver options the composed cell uses (from
    cell.xml) so MjSpec.attach doesn't warn about option conflicts."""
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0


def _load_arm_with_gripper() -> mujoco.MjSpec:
    arm = mujoco.MjSpec.from_file(str(UR5E_XML))
    gripper = mujoco.MjSpec.from_file(str(GRIPPER_XML))
    _delete_keyframes(arm)
    _delete_keyframes(gripper)
    _align_options(arm)
    _align_options(gripper)
    arm.attach(gripper, site="attachment_site", prefix=GRIPPER_PREFIX)
    return arm


def build_cell_spec() -> mujoco.MjSpec:
    cell = mujoco.MjSpec.from_file(str(CELL_XML))
    cell.attach(_load_arm_with_gripper(), site="arm_mount", prefix=ARM_PREFIX)

    # Gravity-compensate the articulated machinery (arm, gripper, door) so
    # the position servos track without steady-state sag; the parts keep
    # full gravity so they rest, drop, and settle realistically.
    for body in cell.bodies:
        if body.name.startswith(ARM_PREFIX) or body.name == "cnc_door":
            body.gravcomp = 1.0

    # The gripper mount sits flush against the wrist; keep the pair out of the
    # contact solver.
    exclude = cell.add_exclude()
    exclude.name = "wrist_gripper_mount"
    exclude.bodyname1 = WRIST_BODY
    exclude.bodyname2 = GRIPPER_MOUNT_BODY

    # One toggleable weld per part for the grasp. P0 simplification: the
    # physical fingers close around the part for looks, but the hold itself is
    # a weld constraint activated at grasp time (robust, deterministic).
    for part in PARTS:
        weld = cell.add_equality()
        weld.name = GRASP_EQ[part]
        weld.type = mujoco.mjtEq.mjEQ_WELD
        weld.objtype = mujoco.mjtObj.mjOBJ_BODY
        weld.name1 = GRIPPER_BASE_BODY
        weld.name2 = part
        weld.active = False
    return cell


def build_cell_model() -> mujoco.MjModel:
    return build_cell_spec().compile()


def build_ik_model() -> mujoco.MjModel:
    """Arm + gripper alone, mounted at the same world pose as in the cell,
    so IK targets can be expressed directly in cell/world coordinates."""
    spec = mujoco.MjSpec()
    _align_options(spec)
    site = spec.worldbody.add_site()
    site.name = "arm_mount"
    site.pos = [0.0, 0.0, 0.74]
    spec.attach(_load_arm_with_gripper(), site="arm_mount", prefix=ARM_PREFIX)
    return spec.compile()


def reset_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Put the arm at the home pose (with matching servo targets) and the
    parts at their bin slots."""
    for name, q in zip(ARM_JOINTS, HOME_QPOS):
        data.joint(name).qpos[0] = q
    for name, q in zip(ARM_ACTUATORS, HOME_QPOS):
        data.actuator(name).ctrl[0] = q
    respawn_parts(model, data)
    mujoco.mj_forward(model, data)


def respawn_parts(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Teleport all parts back to their bin slots as fresh blanks."""
    for part in PARTS:
        joint = data.joint(f"{part}_free")
        joint.qpos[:3] = BIN_SLOTS[part]
        joint.qpos[3:] = [1.0, 0.0, 0.0, 0.0]
        joint.qvel[:] = 0.0
        model.geom(f"{part}_geom").rgba = BLANK_RGBA
        data.eq_active[model.equality(GRASP_EQ[part]).id] = 0
        data.eq_active[model.equality(CLAMP_EQ[part]).id] = 0


def set_weld_relpose(model: mujoco.MjModel, data: mujoco.MjData, eq_name: str) -> None:
    """Write the current relative pose of body2 in body1's frame into a weld's
    eq_data so activating it freezes the bodies exactly where they are."""
    eq_id = model.equality(eq_name).id
    body1 = model.eq_obj1id[eq_id]
    body2 = model.eq_obj2id[eq_id]
    p1, p2 = data.xpos[body1], data.xpos[body2]
    q1, q2 = data.xquat[body1], data.xquat[body2]
    q1_inv = np.array([q1[0], -q1[1], -q1[2], -q1[3]])
    rel_pos = np.empty(3)
    mujoco.mju_rotVecQuat(rel_pos, p2 - p1, q1_inv)
    rel_quat = np.empty(4)
    mujoco.mju_mulQuat(rel_quat, q1_inv, q2)
    model.eq_data[eq_id][:3] = 0.0          # anchor (unused for weld)
    model.eq_data[eq_id][3:6] = rel_pos     # relpose position
    model.eq_data[eq_id][6:10] = rel_quat   # relpose quaternion
    model.eq_data[eq_id][10] = 1.0          # torquescale


def set_weld_active(model: mujoco.MjModel, data: mujoco.MjData, eq_name: str, active: bool) -> None:
    eq_id = model.equality(eq_name).id
    if active:
        set_weld_relpose(model, data, eq_name)
    data.eq_active[eq_id] = 1 if active else 0
