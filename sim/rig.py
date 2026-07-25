"""Rig geometry — where every joint IS, in one shared reference frame.

Plan #660. The twin (#648) answers "would this pose collide?"; this
answers "where is each joint, and which way does it turn?" — the
spatial vocabulary pose authoring needs. Everything is simulated: the
vendored SO-ARM100 model carries the real arm's geometry, so joint
centerpoints and axes are derived, never measured by hand.

FRAME (Kyle, 2026-07-25): the origin is m1's centerpoint. +Z is up,
so joint heights read directly as height above the base. Poses and
joint positions are always reported in this frame, never in MuJoCo's
world frame (whose origin sits at the arm's mounting plane).

    uv run python -m sim.rig spec              # link geometry + axes
    uv run python -m sim.rig where             # joints at the rest pose
    uv run python -m sim.rig where --deg 0,-90,0,0,0,0

`where` takes a pose either as calibrated ticks (--ticks) or as this
project's human units (--deg, per calibration.json frames), so the
numbers line up with what the bench tools print.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from hardware.errors import BenchError

from .twin import JOINT_MAPS, Twin

# Model joint -> the physical joint number the bench calls it.
JOINT_NUMBER = {jm.model_joint: i for i, jm in JOINT_MAPS.items()}


@dataclass(frozen=True)
class JointPlace:
    """A joint's centerpoint and rotation axis in the rig frame (mm)."""

    number: int
    model_joint: str
    x: float
    y: float
    z: float
    axis: tuple[float, float, float]

    def __str__(self) -> str:
        return (f"m{self.number} {self.model_joint:<12} "
                f"({self.x:7.1f}, {self.y:7.1f}, {self.z:7.1f}) mm  "
                f"axis ({self.axis[0]:+.2f}, {self.axis[1]:+.2f}, "
                f"{self.axis[2]:+.2f})")


class Rig:
    """Forward kinematics in the m1-centered frame."""

    def __init__(self, twin: Twin | None = None):
        self.twin = twin or Twin()
        self.model, self.data = self.twin.model, self.twin.data
        self._jid = {}
        for i, jm in JOINT_MAPS.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                    jm.model_joint)
            if jid < 0:
                raise BenchError(f"model joint {jm.model_joint} not found",
                                 "the vendored model changed")
            self._jid[i] = jid
        # m1's anchor is fixed in the world (it is the base joint), so
        # the frame offset can be captured once.
        self.data.qpos[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self._origin = self.data.xanchor[self._jid[1]].copy()

    def places(self, qpos: np.ndarray | None = None) -> list[JointPlace]:
        if qpos is not None:
            self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)
        out = []
        for i in sorted(JOINT_MAPS):
            jid = self._jid[i]
            rel = (self.data.xanchor[jid] - self._origin) * 1000.0
            ax = self.data.xaxis[jid]
            out.append(JointPlace(
                number=i, model_joint=JOINT_MAPS[i].model_joint,
                x=float(rel[0]), y=float(rel[1]), z=float(rel[2]),
                axis=(float(ax[0]), float(ax[1]), float(ax[2]))))
        return out

    def places_at_ticks(self, ticks: dict[int, int]) -> list[JointPlace]:
        """Joint placement for a pose given in calibrated servo ticks."""
        qpos = self.twin._rest_qpos.copy()
        for i, tick in ticks.items():
            q, _ = self.twin.qpos_of(i, tick)
            qpos[self.twin._adr[i]] = q
        return self.places(qpos)

    def link_lengths(self) -> list[tuple[int, int, float]]:
        """Center-to-center distances down the chain — the numbers that
        can be checked against the physical arm with calipers."""
        places = self.places(np.zeros(self.model.nq))
        out = []
        for a, b in zip(places, places[1:]):
            d = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            out.append((a.number, b.number, d))
        return out

    def tool_point(self, qpos: np.ndarray | None = None,
                   ) -> tuple[float, float, float]:
        """Gripper reference point (moving jaw body origin) in the rig
        frame — the 'where is the hand' number for pose authoring."""
        if qpos is not None:
            self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)
        rel = (self.data.body("Moving_Jaw").xpos - self._origin) * 1000.0
        return float(rel[0]), float(rel[1]), float(rel[2])


def cmd_spec(rig: Rig) -> int:
    print("rig frame: origin = m1 centerpoint, +Z up, mm\n")
    print("joint placement at model zero:")
    for p in rig.places(np.zeros(rig.model.nq)):
        print(f"  {p}")
    print("\nlink lengths, center to center (calipers-checkable):")
    for a, b, d in rig.link_lengths():
        print(f"  m{a} -> m{b}: {d:6.1f} mm")
    x, y, z = rig.tool_point(np.zeros(rig.model.nq))
    print(f"\ntool point at model zero: ({x:.1f}, {y:.1f}, {z:.1f}) mm "
          f"-> reach {math.dist((0, 0, 0), (x, y, z)):.1f} mm")
    print(f"m1 centerpoint sits {rig._origin[2] * 1000:.1f} mm above the "
          f"mounting plane")
    return 0


def cmd_where(rig: Rig, ticks: dict[int, int] | None) -> int:
    cals = rig.twin.cals
    pose = ticks or {i: cals[i].rest for i in sorted(cals)}
    label = "given pose" if ticks else "rest pose"
    print(f"joint placement at the {label} (rig frame, mm):")
    for p in rig.places_at_ticks(pose):
        cal = cals.get(p.number)
        human = (cal.frame.fmt(pose[p.number])
                 if cal and cal.frame and p.number in pose else "")
        print(f"  {p}  {human}")
    x, y, z = rig.tool_point()
    print(f"  tool point ({x:7.1f}, {y:7.1f}, {z:7.1f}) mm")
    return 0


def _parse_pose(rig: Rig, deg: str | None, ticks: str | None,
                ) -> dict[int, int] | None:
    cals = rig.twin.cals
    if deg and ticks:
        raise BenchError("--deg and --ticks are mutually exclusive")
    if not deg and not ticks:
        return None
    raw = (deg or ticks).split(",")
    if len(raw) != len(cals):
        raise BenchError(
            f"expected {len(cals)} comma-separated values, got {len(raw)}",
            "one per joint, m1 first")
    out: dict[int, int] = {}
    for i, value in zip(sorted(cals), raw):
        try:
            v = float(value)
        except ValueError as exc:
            raise BenchError(f"could not parse {value!r}") from exc
        if ticks:
            out[i] = int(v)
            continue
        frame = cals[i].frame
        if frame is None:
            raise BenchError(
                f"joint {i} has no ratified frame — --deg needs one",
                "use --ticks, or ratify the frame in calibration.json")
        out[i] = frame.tick(v)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m sim.rig",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("spec", "where"))
    parser.add_argument("--cal", default="calibration.json")
    parser.add_argument("--deg", default=None,
                        help="pose in human units, comma-separated, m1 first")
    parser.add_argument("--ticks", default=None,
                        help="pose in servo ticks, comma-separated, m1 first")
    args = parser.parse_args()
    try:
        rig = Rig(Twin(cal_path=args.cal))
        if args.command == "spec":
            return cmd_spec(rig)
        return cmd_where(rig, _parse_pose(rig, args.deg, args.ticks))
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
