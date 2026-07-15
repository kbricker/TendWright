"""Headless validation of the P0 cell: run the tending cycle and police it.

Checks, every physics/control step:
  * no forbidden contacts — any arm/gripper geom against the static cell
    (table, bin, CNC shell, door, fixture, tray, floor), or a non-finger
    robot geom against a part;
  * no dropped or escaped parts — a part below the table top or outside the
    table footprint fails the run;
  * task checkpoints — the grasp weld must latch with the pinch actually at
    the part, the part must be seated at the fixture (at rest) when clamped,
    machining must happen seated with the door closed, and each finished
    part must end upright inside its tray slot. These are the assertions
    that make PASS mean "the cell tends parts", not just "nothing crashed";
  * progress — each script step must complete within a sim-time budget.

Exits 0 on success, 1 on failure.

Usage: uv run python -m sim.validate [--parts N] [--snapshot PATH.ppm]
"""

from __future__ import annotations

import argparse
import sys

import mujoco
import numpy as np

from . import scene
from .control import CONTROL_DT, ArmController
from .cycle import TendingCycle

STEP_TIMEOUT = 30.0  # sim-seconds allowed per script step
PART_Z_MIN = 0.70  # below the table top = dropped
TABLE_XY = (-0.25, 0.95, -0.70, 0.70)  # actual table footprint + small margin

GRASP_DIST_TOL = 0.030  # m, pinch-to-part distance for a plausible grasp latch
SEAT_TOL = 0.012  # m, part-to-seat distance while clamped/machining
SETTLE_SPEED = 0.05  # m/s, "at rest" threshold for latch/seat checks
TRAY_XY_TOL = 0.030  # m, placement tolerance around the tray slot
UPRIGHT_MIN = 0.98  # min world-z alignment of the part axis on the tray


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


def part_pos(data, part: str) -> np.ndarray:
    return data.joint(f"{part}_free").qpos[:3].copy()


def part_speed(data, part: str) -> float:
    return float(np.linalg.norm(data.joint(f"{part}_free").qvel[:3]))


def part_upright(data, part: str) -> float:
    """World-z component of the part's body z-axis (1.0 = perfectly upright)."""
    return float(data.body(part).xmat[8])


def part_violations(data) -> list[str]:
    bad = []
    xmin, xmax, ymin, ymax = TABLE_XY
    for part in scene.PARTS:
        x, y, z = part_pos(data, part)
        if z < PART_Z_MIN:
            bad.append(f"{part} dropped (z={z:.3f})")
        elif not (xmin < x < xmax and ymin < y < ymax):
            bad.append(f"{part} escaped the table (x={x:.2f}, y={y:.2f})")
    return bad


class CheckpointMonitor:
    """Task-semantic assertions keyed off cycle step starts."""

    def __init__(self, model: mujoco.MjModel, controller: ArmController):
        self.model = model
        self.controller = controller
        self.machining_part: str | None = None

    def _door_qpos(self, data) -> float:
        return float(data.qpos[self.controller.door_qpos_adr])

    def on_step_start(self, name: str, data) -> list[str]:
        part, _, action = name.partition(": ")
        if part not in scene.PARTS:
            return []
        bad = []
        if action == "grasp on":
            pinch = data.site(scene.PINCH_SITE).xpos
            dist = float(np.linalg.norm(pinch - part_pos(data, part)))
            if dist > GRASP_DIST_TOL:
                bad.append(f"grasp weld latched {dist * 1000:.0f} mm from {part}")
        elif action == "clamp on":
            err = float(np.linalg.norm(part_pos(data, part) - scene.FIXTURE_SEAT))
            if err > SEAT_TOL:
                bad.append(f"{part} clamped {err * 1000:.0f} mm off the fixture seat")
            if part_speed(data, part) > SETTLE_SPEED:
                bad.append(f"{part} clamped while still moving "
                           f"({part_speed(data, part):.2f} m/s)")
        elif action == "machining":
            self.machining_part = part
        elif action == "machined":
            self.machining_part = None
        elif action == "cycle complete":
            x, y, z = part_pos(data, part)
            sx, sy, _ = scene.TRAY_SLOTS[part]
            if abs(x - sx) > TRAY_XY_TOL or abs(y - sy) > TRAY_XY_TOL:
                bad.append(f"{part} finished {abs(x - sx) * 1000:.0f}/"
                           f"{abs(y - sy) * 1000:.0f} mm (x/y) off its tray slot")
            if z > 0.80:
                bad.append(f"{part} finished floating at z={z:.3f}")
            if part_upright(data, part) < UPRIGHT_MIN:
                bad.append(f"{part} finished tipped over on the tray "
                           f"(z-axis alignment {part_upright(data, part):.2f})")
            if part_speed(data, part) > SETTLE_SPEED:
                bad.append(f"{part} still moving at cycle end")
        return [f"checkpoint '{name}': {b}" for b in bad]

    def on_tick(self, data) -> list[str]:
        if self.machining_part is None:
            return []
        part = self.machining_part
        bad = []
        err = float(np.linalg.norm(part_pos(data, part) - scene.FIXTURE_SEAT))
        if err > SEAT_TOL:
            bad.append(f"machining {part} {err * 1000:.0f} mm off the seat")
        if part_speed(data, part) > SETTLE_SPEED:
            bad.append(f"machining {part} while it moves")
        if self._door_qpos(data) > 0.01:
            bad.append(f"machining {part} with the door open "
                       f"({self._door_qpos(data):.3f})")
        return bad


def save_snapshot(model, data, path: str) -> None:
    renderer = mujoco.Renderer(model, height=720, width=1280)
    try:
        renderer.update_scene(data, camera="overview")
        pixels = renderer.render()
    finally:
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
                        help="write a PPM render mid-run (or at the failure)")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    model = scene.build_cell_model()
    data = mujoco.MjData(model)
    scene.reset_home(model, data)
    controller = ArmController(model)
    cycle = TendingCycle(model, controller, CONTROL_DT)
    monitor = CheckpointMonitor(model, controller)
    robot, fingers, parts = classify_geoms(model)

    physics_per_control = max(1, round(CONTROL_DT / model.opt.timestep))
    failures: list[str] = []
    max_weld_err = 0.0
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
            failures.extend(f"[{t:.2f}s] {b}"
                            for b in monitor.on_step_start(started, data))
            if "grasp on" in started or "clamp on" in started or "grasp off" in started:
                max_weld_err = max(
                    max_weld_err, controller.physical_position_error(data))
        for _ in range(physics_per_control):
            mujoco.mj_step(model, data)
            t = data.time
            bad = contact_violations(model, data, robot, fingers, parts)
            if bad:
                failures.extend(f"[{t:.2f}s] {b}" for b in sorted(set(bad)))
                break
        failures.extend(f"[{t:.2f}s] {b}" for b in monitor.on_tick(data))
        failures.extend(f"[{t:.2f}s] {b}" for b in part_violations(data))
        if t > step_deadline:
            failures.append(
                f"[{t:.2f}s] step '{cycle.current_step.name}' timed out "
                f"(>{STEP_TIMEOUT}s)"
            )
        if not snapshot_taken and cycle.parts_completed >= 1:
            save_snapshot(model, data, args.snapshot)
            snapshot_taken = True

    if failures and not snapshot_taken:
        save_snapshot(model, data, args.snapshot)

    print(f"\nsim time: {t:.1f}s  |  part cycles: {cycle.parts_completed}  |  "
          f"full loops: {cycle.loops_completed}  |  worst pinch error at a "
          f"weld toggle: {max_weld_err * 1000:.1f} mm")
    if failures:
        print(f"FAIL — {len(failures)} violation(s):")
        for failure in failures[:20]:
            print(f"  {failure}")
        return 1
    print(f"PASS — {args.parts} part cycles: no collisions, no drops, "
          f"parts seated when machined and upright on their tray slots")
    return 0


if __name__ == "__main__":
    sys.exit(main())
