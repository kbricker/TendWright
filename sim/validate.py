"""Headless validation of the P0 cell: run the tending cycle and police it.

Checks, every physics step:
  * no forbidden contacts — any arm/gripper geom against the static cell
    (table, bin, CNC shell, door, fixture, tray, floor), or a non-finger
    robot geom against a part;
  * no dropped or escaped parts — a part below the table top or outside the
    table footprint fails the run;
  * progress — each script step must complete within a sim-time budget.

Exits 0 on success, 1 on failure.

Usage: uv run python -m sim.validate [--parts N] [--snapshot PATH.ppm]
"""

from __future__ import annotations

import argparse
import sys

import mujoco

from . import scene
from .control import CONTROL_DT, ArmController
from .cycle import TendingCycle

STEP_TIMEOUT = 30.0  # sim-seconds allowed per script step
PART_Z_MIN = 0.70  # below the table top = dropped
TABLE_XY = (-0.35, 1.05, -0.80, 0.80)  # generous table footprint


def classify_geoms(model: mujoco.MjModel):
    robot, fingers, parts = set(), set(), set()
    finger_bodies = {
        f"{scene.ARM_PREFIX}{scene.GRIPPER_PREFIX}{side}_{link}"
        for side in ("left", "right")
        for link in ("pad", "follower", "silicone_pad")
    }
    part_bodies = set(scene.PARTS)
    for geom_id in range(model.ngeom):
        body_name = model.body(model.geom_bodyid[geom_id]).name
        if body_name in part_bodies:
            parts.add(geom_id)
        elif body_name.startswith(scene.ARM_PREFIX):
            robot.add(geom_id)
            if body_name in finger_bodies:
                fingers.add(geom_id)
    return robot, fingers, parts


def contact_violations(model, data, robot, fingers, parts) -> list[str]:
    bad = []
    for i in range(data.ncon):
        g1, g2 = data.contact[i].geom1, data.contact[i].geom2
        r1, r2 = g1 in robot, g2 in robot
        if r1 and r2:
            continue  # robot self-contact: not policed in P0
        if not r1 and not r2:
            continue  # part/static contacts are fine
        robot_geom, other = (g1, g2) if r1 else (g2, g1)
        if other in parts:
            if robot_geom not in fingers:
                bad.append(
                    f"{model.geom(robot_geom).name} touched part geom "
                    f"{model.geom(other).name}"
                )
        else:
            bad.append(
                f"{model.geom(robot_geom).name} hit static geom "
                f"{model.geom(other).name}"
            )
    return bad


def part_violations(data) -> list[str]:
    bad = []
    xmin, xmax, ymin, ymax = TABLE_XY
    for part in scene.PARTS:
        x, y, z = data.joint(f"{part}_free").qpos[:3]
        if z < PART_Z_MIN:
            bad.append(f"{part} dropped (z={z:.3f})")
        elif not (xmin < x < xmax and ymin < y < ymax):
            bad.append(f"{part} escaped the table (x={x:.2f}, y={y:.2f})")
    return bad


def save_snapshot(model, data, path: str) -> None:
    renderer = mujoco.Renderer(model, height=720, width=1280)
    renderer.update_scene(data, camera="overview")
    pixels = renderer.render()
    renderer.close()
    with open(path, "wb") as f:
        f.write(f"P6\n{pixels.shape[1]} {pixels.shape[0]}\n255\n".encode())
        f.write(pixels.tobytes())
    print(f"snapshot written to {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parts", type=int, default=6,
                        help="part cycles to complete (default 6 = 2 full loops)")
    parser.add_argument("--snapshot", type=str, default=None,
                        help="write a PPM render of the scene mid-run")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    model = scene.build_cell_model()
    data = mujoco.MjData(model)
    scene.reset_home(model, data)
    controller = ArmController(model)
    cycle = TendingCycle(model, controller, CONTROL_DT)
    robot, fingers, parts = classify_geoms(model)

    physics_per_control = max(1, round(CONTROL_DT / model.opt.timestep))
    failures: list[str] = []
    max_ik_err = 0.0
    step_deadline = STEP_TIMEOUT
    snapshot_taken = args.snapshot is None
    t = 0.0

    while cycle.parts_completed < args.parts and not failures:
        controller.tick(data)
        started = cycle.tick(data)
        if started:
            step_deadline = t + STEP_TIMEOUT
            if args.verbose:
                print(f"[{t:7.2f}s] {started}")
        for _ in range(physics_per_control):
            mujoco.mj_step(model, data)
            t = data.time
            bad = contact_violations(model, data, robot, fingers, parts)
            if bad:
                failures.extend(f"[{t:.2f}s] {b}" for b in sorted(set(bad)))
                break
        max_ik_err = max(max_ik_err, controller.position_error())
        failures.extend(f"[{t:.2f}s] {b}" for b in part_violations(data))
        if t > step_deadline:
            failures.append(
                f"[{t:.2f}s] step '{cycle.current_step.name}' timed out "
                f"(>{STEP_TIMEOUT}s)"
            )
        if not snapshot_taken and cycle.parts_completed >= 1:
            save_snapshot(model, data, args.snapshot)
            snapshot_taken = True

    print(f"\nsim time: {t:.1f}s  |  part cycles: {cycle.parts_completed}  |  "
          f"full loops: {cycle.loops_completed}  |  peak IK target lag: "
          f"{max_ik_err * 1000:.1f} mm (transient, mid-move)")
    if failures:
        print(f"FAIL — {len(failures)} violation(s):")
        for failure in failures[:20]:
            print(f"  {failure}")
        return 1
    print(f"PASS — {args.parts} part cycles, no collisions, no drops")
    return 0


if __name__ == "__main__":
    sys.exit(main())
