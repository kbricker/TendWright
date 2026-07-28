"""Digital twin of the SO-101 bench arm — collision-gated motion (plan #648).

Maps calibrated servo ticks onto the vendored MuJoCo model of the
SO-101 (sim/assets/so101/, TheRobotStudio, Apache-2.0) and checks
planned trajectories for self-collision and table contact BEFORE the
real arm moves. The bench tools call `Twin.check_trajectory` as a
pre-flight gate; contacts are predicted, not discovered.

The model is the SO-101 as of plan #670. It previously ran on
Menagerie's SO-ARM100 — a different arm, accepted as close enough
during #648. That package is still vendored at
sim/assets/menagerie/trs_so_arm100/ and NOTHING LOADS IT as a model.
It is retained on purpose, with a job: it is the negative control for
`sim.meshcheck`. The question #670 exists to settle is "is this even
the right arm", and the only way to show a check can answer that is to
aim it at the wrong arm — which rejects all six shared parts and gives
the 43x margin the mesh comparison's threshold sits inside. Delete the
package and that claim becomes unverifiable.

Its two loadable XMLs are renamed `*.WRONG-ARM-DO-NOT-LOAD.xml` so
loading one by accident fails outright instead of quietly producing
collision predictions for a robot we do not own. `sim.meshcheck
selftest` asserts that convention holds, and asserts this module's
MODEL_XML is not inside that directory.

What the swap did and did not change is worth knowing:
  - the derived clearance envelope came out IDENTICAL (m2 span 40%,
    elbow 90 deg, m4 90 deg), independently re-derived, so the shipped
    safety constants are confirmed rather than adjusted;
  - both real bench collisions are still predicted;
  - the arm's observed torque-off slump NO LONGER penetrates the model,
    where the SO-100 read it as 0.14 mm past touching while the real arm
    sat in that pose untouched — the new geometry matches reality better;
  - the joint LIMITS did not improve. Both packages describe them as
    approximate and both are narrower than the calibrated physical range
    on j2 and j6, so `qpos_of` still clamps and still reports it.

Tick -> qpos mapping: per joint, qpos = a + b * x, where x is the
ratified frame's own reading (degrees, or ticks for the gripper's
percentage frame) and (a, b) are MEASURED FROM THE MODEL at construction
by `_derive_anchors` — see JOINT_MAPS for how each joint's zero is
located. Nothing is transcribed: the SO-101 package ships no keyframe to
anchor on, and the geometric derivation reproduced the SO-100's
hand-posed `rest` keyframe to within 0.03 deg (under one tick), which is
what licensed retiring it.

Table: the arm is bolted to the bench, so the world plane z=0 IS the
table. base<->table contact is expected (mounted) and ignored; so is
the gripper closing on its own jaw.

Collision geometry differs from the old package and not always for the
better: the SO-101 ships NO base collision mesh (removed upstream as
problematic) and a far simpler gripper — 3 collidable geoms against the
Menagerie model's 13 hand-built ones. Contacts involving the base are
therefore invisible to the gate. Both bench collisions survive because
they are also caught by table and shoulder pairs, which was checked, not
assumed, before the swap was accepted.

Poses within CONTACT_MARGIN_M of touching count as contact: the mapping
carries real uncertainty, so the gate refuses near-misses too. The one
exception is link pairs that nest structurally in the folded rest pose
(`sim.twin check` prints them) — see the settle note below.

CLI (all take --cal FILE; exercise/derive-clearance take --span PCT):
    uv run python -m sim.twin check             # rest pose contact-free?
    uv run python -m sim.twin exercise          # gate the exercise routine
    uv run python -m sim.twin derive-clearance  # scan span/elbow/wrist
                                                # holds for a clear m2 sweep
    uv run python -m sim.twin validate          # must predict the two
                                                # known bench collisions
    uv run python -m sim.twin selftest          # pin the gate's safety
                                                # contracts (no hardware)
    uv run python -m sim.twin frames            # ratified display frames
                                                # vs the model's geometry

Imports note: pulls hardware.bench.calibrate (and thus the servo SDK)
for the calibration loader — fine anywhere the project runs; #637 gives
the loader an SDK-free home.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from hardware.bench.calibrate import JointCal, load_calibration
from hardware.errors import BenchError
from hardware.units import DegFrame, RAD_PER_TICK, fmt_ticks, span_deg

MODEL_XML = Path(__file__).parent / "assets/so101/so101_new_calib.xml"
# Safety margin: geom pairs within this distance are surfaced as
# near-misses, because the mapping carries real uncertainty (two
# provisional directions, print tolerances, a hand-posed rest anchor)
# and a 5 mm miss in the model is not a pass on a real arm.
#
# Exception, else the gate cries wolf: in the arm's FOLDED REST pose
# several links legitimately nest within a few mm of each other. Those
# specific pairs are baselined at construction — for them only ACTUAL
# penetration counts, while every other pair fails on proximity alone.
CONTACT_MARGIN_M = 0.005
# MuJoCo SUMS the two geoms' margins to get a pair's contact threshold,
# so each geom carries half of what we want. Setting geom_margin to the
# full value silently doubled the gate's reach to 10 mm (fixed 2026-07-25
# after it refused the arm's own resting pose over pairs 9 mm apart).
_GEOM_MARGIN_M = CONTACT_MARGIN_M / 2
# Detecting which pairs nest structurally, and judging whether a pose is
# safe, want OPPOSITE biases: detection should be generous (miss a nesting
# pair and the gate cries wolf forever after), judgement strict. So the
# structural scan runs at a deliberately wider reach than the gate.
# Sampling at exactly the gate threshold is what left Base <-> Upper_Arm
# out — it rests 5.07 mm apart, a 0.07 mm miss, and then failed on
# proximity the moment the arm twitched. 2x also happens to be what the
# doubled-margin bug was accidentally getting right.
STRUCTURAL_DETECT_M = CONTACT_MARGIN_M * 2
INTERP_STEP_DEG = 2.0  # trajectory sampling; finer = slower, less tunneling
CLAMP_REPORT_DEG = 0.05  # ~half a tick: below this, clamping is rounding

# THE SETTLE FROM A MEASURED SLUMP IS NOT ADJUDICABLE — measured, not
# assumed. This is scoped to a trajectory PHASE, not to a region of pose
# space, because that is exactly where the ambiguity lives.
#
# The arm's torque-off slump does not reproduce itself. The slump captured
# as `rest` and the slump observed 2026-07-25 differ by up to 4.7 deg
# (j5), and in the fold the geometry converts joint error into penetration
# depth at roughly a millimetre per degree, so which slump "rest" means
# used to be ambiguous by more than the clearance being judged: on the
# SO-100 model the observed slump read 0.14 mm PAST touching, while the
# real arm sat there untouched.
#
# ON THE SO-101 GEOMETRY THAT PARTICULAR CONFLICT IS GONE (plan #670):
# the observed slump now only reaches PROXIMITY, which structural pairs
# never fail on, so it is accepted with or without the waiver — pinned
# both ways in `selftest`. That is evidence the new geometry matches the
# real arm better, and it is also why the waiver's paired refusal now
# runs on a clearly-labelled SYNTHETIC deeper fold: a safety waiver that
# no test can exercise is a claim, not a property.
#
# The waiver is KEPT because the hazard it covers has not gone away —
# the slump still does not reproduce itself, and a re-capture, a changed
# horn, or a warmer/looser arm can put it back into penetration.
#
# So during the leading settle (measured pose -> rest) structurally-nested
# link pairs are not adjudicated. The moment the arm reaches `rest` the
# model is clean again and every later pose gates normally — which is the
# whole routine, since sweeps move away from the fold.
#
# Two tolerance-based fixes were tried and REJECTED on measurement:
#   - derived from a pose neighbourhood, it grows explosively (1.4 mm at
#     2 deg, 5.8 at 4, 10 at 6) and by 4 deg it excuses TABLE strikes;
#   - derived from the model's own keyframe it comes out 0.000 mm and
#     changes nothing, because the disagreement is not noise around the
#     keyframe — it IS the keyframe being a different slump than today's.
#
# The waiver is DEPTH-UNBOUNDED for structural pairs during the settle —
# stated plainly because it is the sharpest edge here. No depth cap is
# imposed, for three reasons: the settle is the one phase where the twin
# provably cannot resolve the geometry, so a cap would be a number
# pretending to be a measurement; the arm is at torque-off rest, so it is
# resting on itself under gravity rather than being driven into itself;
# and the start pose is independently bounded by check_start_pose, which
# reads the ENCODERS and refuses a start more than 300 ticks off rest or
# outside the calibrated range. If that pre-flight bound is ever loosened,
# revisit this.
#
# What this does NOT relax, ever:
#   - the table. The arm is not built to rest on it, so a table contact
#     is a finding in the settle too. Enforced by construction: table
#     pairs are excluded from the structural set.
#   - pairs that do not nest at rest — they fail on proximity as always.
#   - any pose after the settle.
# The layer actually covering the arm during the settle is not the twin:
# it is `check_start_pose` reading the encoders (which refuses a start
# more than 300 ticks off rest or outside the calibrated range), and the
# in-motion guards.

# Contact pairs that are expected and never gate-failures. (The table
# plane hangs off the worldbody; _body_of_geom reports it as "table".)
# The slump deviation actually observed on 2026-07-25, in ticks from the
# calibrated rest pose. Applied as a DELTA so it tracks a re-capture
# instead of pinning stale absolute ticks. This is the pose the gate
# refused while the arm was sitting in it, untouched.
#
# It is a MEASUREMENT, not a test fixture, which is why the structural
# scan reads it (see `_scan_structural`): the arm's torque-off slump does
# not reproduce itself, so "where the links nest when the arm rests on
# itself" is a small ENVELOPE of measured poses, not one pose. Scanning
# only the calibrated rest missed `shoulder <-> lower_arm` on the SO-101
# geometry — the pair is not within 60 mm at cal rest and comes within
# the gate's reach in the slump, so the gate refused the arm's own
# resting pose (plan #670).
OBSERVED_SLUMP_DELTA = {1: +84, 2: +2, 3: +8, 4: +27, 5: -53, 6: -4}

# Named explicitly because "gripper" is BOTH a joint and a body in this
# model, and the jaw body is named after its mesh. Resolving a joint name
# as a body name silently returns the wrong link — it put the measured
# reach direction 2.9 deg out before this existed.
BASE_BODY = "base"
TOOL_BODY = "moving_jaw_so101_v1"
WRIST_BODY = "gripper"

ALLOWED_PAIRS = {
    frozenset({BASE_BODY, "table"}),         # bolted to the bench
    frozenset({WRIST_BODY, TOOL_BODY}),      # gripper closing on itself
}


@dataclass(frozen=True)
class JointMap:
    """How one calibrated joint lands on a model joint.

    `anchor` says how this joint's zero is LOCATED on the model. Nothing
    here is a number: every anchor is measured from the model at
    construction, so a vendored-model change cannot silently desync the
    map (and the SO-101 package ships no keyframe to anchor on anyway).

      "frame" — the ratified frame's zero has a geometric meaning ("upper
        arm vertical", "forearm in line with the upper arm", "gripper in
        line with the forearm"), so the anchor is MEASURED: probe the
        model, find the qpos where that collinearity holds, and pin the
        frame's zero there. Self-checking, because `sim.twin frames` then
        re-measures the same quantity through a different path.
      "rest" — pan and roll have nothing to be collinear with, so their
        frame zero is not geometric and cannot be derived from the model.
        These pin the physical REST tick to the model's own zero, which
        is the relationship bench-verified on the previous model — kept
        deliberately rather than re-guessed, so the model swap does not
        quietly move a joint that was already confirmed correct.
      "min" — the gripper: physical closed pinned to the model's closed
        limit. Its frame is a percentage, not an angle, so it maps in
        ticks.

    bench_verified False = provisional; `sim.twin check` prints how to
    confirm or flip it."""

    model_joint: str
    anchor: str
    bench_verified: bool


# The SO-101 model uses LeRobot's joint names, which match
# calibration.json's names exactly — one less place for a mismatch than
# the SO-100 package's Rotation/Pitch/Elbow.
#
# Directions are DERIVED, not declared (plan #670). For the pitch chain
# the sign falls out of the same probe that finds the anchor: measure the
# segment angle at two qpos values and the slope IS the direction. That
# probe agrees with the previous model on all three joints, and its
# anchors reproduce that model's hand-posed rest keyframe to within
# 0.03 deg — under a tick — which is what licensed retiring the keyframe.
#
# j1 and j5 stay bench-verified rather than derived: pan and roll have no
# collinear reference, so the model cannot be asked where their zero is.
# Their relationship to the model was confirmed at the bench 2026-07-25
# (Kyle jogged each and reported the sense) and is carried across
# unchanged. The gripper's open direction IS geometric and was verified
# by measuring jaw separation across the model's range on both models.
JOINT_MAPS: dict[int, JointMap] = {
    1: JointMap("shoulder_pan", "rest", True),
    2: JointMap("shoulder_lift", "frame", True),
    3: JointMap("elbow_flex", "frame", True),
    4: JointMap("wrist_flex", "frame", True),
    5: JointMap("wrist_roll", "rest", True),
    6: JointMap("gripper", "min", True),   # physical closed = model closed
}


@dataclass(frozen=True)
class PredictedContact:
    step: int
    pose: dict[int, int]
    body_a: str
    body_b: str
    depth_mm: float


@dataclass
class GateReport:
    poses_checked: int
    contacts: list[PredictedContact]
    clamped_joints: dict[int, float]  # joint id -> worst clamp, deg
    nest_excused: int = 0  # structural contacts waived inside the fold

    @property
    def clean(self) -> bool:
        return not self.contacts

    def summary(self, cals: dict[int, JointCal]) -> str:
        lines = [f"collision gate: {self.poses_checked} poses simulated"]
        for c in self.contacts:
            pose = "  ".join(
                f"j{i}={fmt_ticks(cals[i].frame, t)}"
                for i, t in sorted(c.pose.items()))
            lines.append(f"  CONTACT {c.body_a} <-> {c.body_b} "
                         f"({c.depth_mm:.1f} mm) at step {c.step}: {pose}")
        for i, deg in sorted(self.clamped_joints.items()):
            lines.append(f"  note: joint {i} clamped up to {deg:.1f} deg to "
                         f"the model's range (physical range is wider)")
        if self.nest_excused:
            lines.append(
                f"  note: {self.nest_excused} structural nesting contact(s) "
                f"waived during the settle onto rest — which slump `rest` "
                f"means is ambiguous by more than the clearance being "
                f"judged (table contacts are never waived)")
        return "\n".join(lines)


class Twin:
    def __init__(self, cal_path: str | Path = "calibration.json",
                 model_path: Path = MODEL_XML):
        if not Path(model_path).exists():
            raise BenchError(
                f"no arm model at {model_path}",
                "the vendored Menagerie model is missing from the repo")
        try:
            spec = mujoco.MjSpec.from_file(str(model_path))
            # The arm is bolted to the bench, so the world plane IS the
            # table. size is plane grid spacing, not extent (infinite).
            spec.worldbody.add_geom(
                name="table", type=mujoco.mjtGeom.mjGEOM_PLANE,
                size=[0.0, 0.0, 0.05])
            self.model = spec.compile()
        except ValueError as exc:
            raise BenchError(f"could not load the arm model: {exc}",
                             "the vendored model may be corrupt") from exc
        # Half each, because MuJoCo sums them per pair (see _GEOM_MARGIN_M).
        self.model.geom_margin[:] = _GEOM_MARGIN_M
        self.data = mujoco.MjData(self.model)
        self.cals = load_calibration(Path(cal_path))
        missing = sorted(set(JOINT_MAPS) - set(self.cals))
        if missing:
            raise BenchError(
                f"calibration.json lacks joint(s) {missing}",
                "the twin needs all six joints — calibrate capture first")
        # link pairs that nest structurally in the fold (never the table)
        self._structural: set[frozenset] = set()
        self._adr: dict[int, int] = {}
        self._jid: dict[int, int] = {}
        self._range: dict[int, tuple[float, float]] = {}
        for i, jm in JOINT_MAPS.items():
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, jm.model_joint)
            if jid < 0:
                raise BenchError(f"model joint {jm.model_joint} not found",
                                 "the vendored model changed — fix JOINT_MAPS")
            self._jid[i] = jid
            self._adr[i] = self.model.jnt_qposadr[jid]
            self._range[i] = tuple(self.model.jnt_range[jid])
        self._lin = self._derive_anchors()
        self._rest_qpos = np.zeros(self.model.nq)
        for i in sorted(self.cals):
            self._rest_qpos[self._adr[i]] = self.qpos_of(i, self.cals[i].rest)[0]
        self._scan_structural()


    def _scan_structural(self) -> None:
        """Find the link pairs that nest when the arm rests on itself.

        Scanned over the MEASURED resting poses — the calibrated rest and
        the observed slump — not one pose, because the torque-off slump
        does not reproduce itself and "where the links nest" is therefore
        a small envelope. Both inputs are measurements of the real arm at
        rest; nothing here is a chosen tolerance.

        Scanned at STRUCTURAL_DETECT_M, deliberately wider than the gate
        reach: missing a nesting pair makes the gate cry wolf forever
        after, while including one only ever costs a proximity warning —
        these pairs still fail on real penetration.

        The table is excluded by construction: the arm is not built to
        rest on it, so a table contact is always a finding."""
        poses = [{i: self.cals[i].rest for i in self.cals},
                 {i: self.cals[i].rest + OBSERVED_SLUMP_DELTA.get(i, 0)
                  for i in self.cals}]
        self.model.geom_margin[:] = STRUCTURAL_DETECT_M / 2
        for pose in poses:
            self.data.qpos[:] = self._rest_qpos
            for i, tick in pose.items():
                self.data.qpos[self._adr[i]] = self.qpos_of(i, tick)[0]
            mujoco.mj_forward(self.model, self.data)
            for n in range(self.data.ncon):
                con = self.data.contact[n]
                pair = frozenset({self._body_of_geom(con.geom1),
                                  self._body_of_geom(con.geom2)})
                if "table" not in pair:
                    self._structural.add(pair)
        self.model.geom_margin[:] = _GEOM_MARGIN_M  # back to gate reach

    def reach_yaw_deg(self) -> float:
        """Which way the arm reaches, in the MODEL's own base frame (deg,
        CCW from +x).

        A cell places the arm by the direction it reaches, so whoever
        attaches this model needs to know where the model thinks forward
        is — and packages disagree: the SO-100 reached along its own -Y
        (-90 deg), the SO-101 along +X (0 deg). That 90 deg is exactly
        the constant the bench scene used to carry as a literal `+90`,
        which would have silently pointed the arm at the wrong wall after
        the model swap. Measured, so it follows the model."""
        pose = {i: self.cals[i].rest for i in sorted(self.cals)}
        for j, deg in ((1, 0.0), (2, 60.0), (3, 0.0), (4, 0.0)):
            pose[j] = self.cals[j].frame.tick(deg)   # pan straight, arm out
        self.data.qpos[:] = self._rest_qpos
        for i, tick in pose.items():
            self.data.qpos[self._adr[i]] = self.qpos_of(i, tick)[0]
        mujoco.mj_forward(self.model, self.data)
        bid = lambda n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n)
        v = self.data.xpos[bid(TOOL_BODY)] - self.data.xpos[bid(BASE_BODY)]
        if float(np.hypot(v[0], v[1])) < 1e-3:
            raise BenchError("cannot measure the model's reach direction",
                             "the arm is vertical at the probe pose")
        return math.degrees(math.atan2(v[1], v[0]))

    def m1_offset_m(self) -> tuple[float, float]:
        """Where the m1 axis sits relative to the model's base ORIGIN, in
        the model's own base frame (metres, x/y only).

        Companion to reach_yaw_deg, and it exists for the same reason: a
        cell places the arm by something a person can put a tape on, and
        the model's root body origin is not that. On the SO-101 the
        shoulder_pan axis sits 38.8 mm from the base body origin, so
        attaching the model AT the measured point puts the real pivot
        38.8 mm away from where it was measured — a translation error
        that reads as "the arm is a bit too far in" and is invisible in
        any base-relative check.

        Measured off the model rather than written down, so it follows a
        model swap instead of silently surviving one.
        """
        self.data.qpos[:] = self._rest_qpos
        mujoco.mj_forward(self.model, self.data)
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                JOINT_MAPS[1].model_joint)
        base = self.data.xpos[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)]
        anchor = self.data.xanchor[jid]
        return float(anchor[0] - base[0]), float(anchor[1] - base[1])

    def _seg_angle(self, joint: int) -> float:
        """Signed angle (deg) from the parent direction to the segment
        this joint drives, about the joint's OWN world axis — the
        quantity the ratified frame claims to display. Reads whatever
        pose `self.data` is currently in; the caller sets it."""
        def seg(pair: tuple[str, str]):
            a = self.data.xpos[mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, pair[0])]
            b = self.data.xpos[mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, pair[1])]
            v = b - a
            return v / float(np.linalg.norm(v))

        v = seg(PITCH_SEGMENTS[joint])
        parent = (seg(PITCH_PARENTS[joint]) if joint in PITCH_PARENTS
                  else np.array([0.0, 0.0, 1.0]))
        axis = self.data.xaxis[self._jid[joint]]
        return math.degrees(math.atan2(float(np.cross(parent, v) @ axis),
                                       float(parent @ v)))

    def _derive_anchors(self) -> dict[int, tuple[float, float]]:
        """Locate every joint's zero ON THE MODEL: qpos = a + b * x.

        `x` is the frame's human reading — degrees for the five angular
        joints, TICKS for the gripper (whose frame is a percentage).

        Nothing is hand-entered. For the pitch chain both terms come from
        probing the model at two qpos values: the segment angle is linear
        in qpos, so two samples give the slope (which IS the direction)
        and the offset (which IS the anchor). That is the whole reason
        this survives a model swap — the previous model's hand-posed
        `rest` keyframe had no counterpart in the SO-101 package, and a
        transcribed constant would have been a number nobody could
        re-derive."""
        lin: dict[int, tuple[float, float]] = {}
        for i, jm in JOINT_MAPS.items():
            cal = self.cals[i]
            lo, _hi = self._range[i]
            if jm.anchor == "min":
                # percentage frame: map in ticks, physical closed -> model closed
                lin[i] = (lo - RAD_PER_TICK * cal.min, RAD_PER_TICK)
            elif jm.anchor == "rest":
                # no geometric zero to find; pin physical rest to model zero
                lin[i] = (-math.radians(cal.frame.deg(cal.rest)),
                          math.radians(1.0))
            else:                                   # "frame" — measure it
                probe = math.radians(10.0)
                samples = []
                for q in (0.0, probe):
                    self.data.qpos[:] = 0.0
                    self.data.qpos[self._adr[i]] = q
                    mujoco.mj_forward(self.model, self.data)
                    samples.append(self._seg_angle(i))
                slope = (samples[1] - samples[0]) / 10.0   # deg seg / deg qpos
                if abs(slope) < 0.5:
                    raise BenchError(
                        f"joint {i} ({jm.model_joint}) barely moves the "
                        f"segment it should drive (slope {slope:.3f})",
                        "PITCH_SEGMENTS no longer matches the model's body "
                        "tree — the anchor cannot be measured")
                # qpos_deg = (frame_deg - samples[0]) / slope
                lin[i] = (math.radians(-samples[0] / slope),
                          math.radians(1.0 / slope))
        return lin

    def frame_x(self, i: int, tick: int) -> float:
        """The frame reading `qpos_of` maps from: degrees for the angular
        joints, ticks for the gripper's percentage frame."""
        cal = self.cals[i]
        return tick if JOINT_MAPS[i].anchor == "min" else cal.frame.deg(tick)

    def qpos_of(self, i: int, tick: int) -> tuple[float, float]:
        """(qpos clamped to the model range, clamp magnitude in deg).

        Clamping is UNCONSERVATIVE for a collision gate — a clamped pose
        is less extreme than reality, so a contact at the physical range
        end could go unpredicted. That is why every clamp is reported
        and printed even on a clean gate."""
        a, b = self._lin[i]
        q = a + b * self.frame_x(i, tick)
        lo, hi = self._range[i]
        clamped = min(hi, max(lo, q))
        return clamped, abs(q - clamped) * 180.0 / math.pi

    def tick_of(self, i: int, qpos: float) -> int:
        """The servo tick a model qpos corresponds to — inverse of
        `qpos_of`, for the one direction that did not exist until IK.

        Everything until now flowed ticks -> qpos: the arm is the source
        of truth and the model follows it. A solver runs the other way,
        producing joint angles nothing has ever measured, and they have
        to become ticks before they can be commanded. Rounding to the
        nearest tick is a real quantisation (0.088 deg), so a round-trip
        through here is not the identity — it is the identity to within
        one tick, which the selftest pins rather than assumes.
        """
        a, b = self._lin[i]
        x = (qpos - a) / b
        if JOINT_MAPS[i].anchor == "min":
            return int(round(x))          # gripper: frame_x IS ticks
        return self.cals[i].frame.tick(x)

    def contacts_at(self, pose: dict[int, int], step: int = 0,
                    adjudicate_nesting: bool = True,
                    ) -> tuple[list[PredictedContact], dict[int, float], int]:
        """(contacts, clamps, structural contacts waived).

        adjudicate_nesting=False only during the settle from a measured
        slump — see the REST/settle note at the top of this module."""
        self.data.qpos[:] = self._rest_qpos
        clamps: dict[int, float] = {}
        for i, tick in pose.items():
            q, clamp_deg = self.qpos_of(i, tick)
            self.data.qpos[self._adr[i]] = q
            if clamp_deg > CLAMP_REPORT_DEG:
                clamps[i] = clamp_deg
        mujoco.mj_forward(self.model, self.data)
        found: list[PredictedContact] = []
        excused = 0
        for n in range(self.data.ncon):
            con = self.data.contact[n]
            body_a = self._body_of_geom(con.geom1)
            body_b = self._body_of_geom(con.geom2)
            pair = frozenset({body_a, body_b})
            if pair in ALLOWED_PAIRS:
                continue
            if pair in self._structural:
                # Proximity never fails a pair built to nest.
                if con.dist > 0:
                    continue
                # Penetration does — except during the settle from a
                # measured slump, which the twin cannot resolve. Counted,
                # so the gate can say out loud that it looked away.
                if not adjudicate_nesting:
                    excused += 1
                    continue
            found.append(PredictedContact(
                step=step, pose=dict(pose), body_a=body_a, body_b=body_b,
                depth_mm=max(0.0, -con.dist) * 1000.0))
        return found, clamps, excused

    def _body_of_geom(self, geom_id: int) -> str:
        body_id = self.model.geom_bodyid[geom_id]
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                 body_id) or f"geom{geom_id}"
        # the plane lives on the worldbody; report it by what it is
        return "table" if name == "world" else name

    def check_clip(self, clip, hz: float | None = None,
                   settle_from_measured: bool = False) -> GateReport:
        """Gate the frames a clip ACTUALLY produces (plan #660).

        The difference from `check_trajectory` is the whole point of the
        clip layer: this samples each joint on its own speed/accel ramp,
        exactly as the servos will run it, instead of sliding every
        joint in lockstep between waypoints. Lockstep certified poses
        the arm never occupies and skipped poses it does.

        `settle_from_measured`: the clip's FIRST edge starts from the
        arm's measured slump rather than a planned pose, so structural
        nesting is not adjudicated until it reaches the second pose —
        see the module's settle note. Pass it only when that is true."""
        from sim.clip import DEFAULT_HZ, sample_edge

        contacts: list[PredictedContact] = []
        seen_pairs: set[frozenset] = set()
        clamps: dict[int, float] = {}
        checked = 0
        excused = 0
        if not clip.poses:
            return GateReport(0, [], {}, 0)

        def check(pose: dict[int, int], step: int) -> None:
            nonlocal checked, excused
            settling = settle_from_measured and step <= 1
            found, cl, ex = self.contacts_at(pose, step,
                                             adjudicate_nesting=not settling)
            checked += 1
            excused += ex
            for i, d in cl.items():
                clamps[i] = max(clamps.get(i, 0.0), d)
            for c in found:
                pair = frozenset({c.body_a, c.body_b})
                if pair not in seen_pairs:
                    seen_pairs.add(pair)
                    contacts.append(c)

        check(dict(clip.poses[0].ticks), 0)
        for n, (a, b) in enumerate(clip.edges(), start=1):
            for frame in sample_edge(clip.profile, a, b,
                                     hz if hz is not None else DEFAULT_HZ):
                check(frame, n)
        return GateReport(checked, contacts, clamps, excused)

    def check_trajectory(self, waypoints: list[dict[int, int]],
                         step_deg: float = INTERP_STEP_DEG,
                         settle_from_measured: bool = False) -> GateReport:
        """Interpolate between consecutive waypoints in tick space and
        collision-check every intermediate pose. Contacts are deduped by
        body pair (first occurrence reported).

        settle_from_measured: waypoint 0 is the arm's MEASURED slump
        rather than a planned pose, so structural nesting is not
        adjudicated until the arm reaches waypoint 1 (`rest`). Pass it
        only when that is literally true — see the module note."""
        if not waypoints:
            return GateReport(0, [], {}, 0)
        contacts: list[PredictedContact] = []
        seen_pairs: set[frozenset] = set()
        clamps: dict[int, float] = {}
        checked = 0
        excused = 0

        def check(pose: dict[int, int], step: int) -> None:
            nonlocal checked, excused
            # Step 0 is the measured pose; step 1 is the move onto rest.
            settling = settle_from_measured and step <= 1
            found, cl, ex = self.contacts_at(pose, step,
                                             adjudicate_nesting=not settling)
            checked += 1
            excused += ex
            for i, d in cl.items():
                clamps[i] = max(clamps.get(i, 0.0), d)
            for c in found:
                pair = frozenset({c.body_a, c.body_b})
                if pair not in seen_pairs:
                    seen_pairs.add(pair)
                    contacts.append(c)

        step_ticks = step_deg / span_deg(1)
        check(waypoints[0], 0)
        for n in range(1, len(waypoints)):
            a, b = waypoints[n - 1], waypoints[n]
            ids = sorted(set(a) | set(b))
            worst = max(abs(b.get(i, a.get(i, 0)) - a.get(i, b.get(i, 0)))
                        for i in ids)
            substeps = max(1, math.ceil(worst / step_ticks))
            for s in range(1, substeps + 1):
                f = s / substeps
                pose = {i: round(a.get(i, b[i])
                                 + f * (b.get(i, a[i]) - a.get(i, b[i])))
                        for i in ids}
                check(pose, n)
        return GateReport(checked, contacts, clamps, excused)


# ------------------------------------------------------- exercise sequence
def exercise_waypoints(cals: dict[int, JointCal], span_pct: int,
                       ids: list[int] | None = None,
                       ) -> list[dict[int, int]]:
    """The exercise routine's pose sequence. Delegates to exercise.py's
    gate_waypoints — the bench gate and this CLI simulate the SAME
    definition by construction, not by staying in sync."""
    from hardware.bench.exercise import (
        SPAN_MAX, SWEEP_ORDER, SWEEP_SPAN_CAPS, clamp_goal,
        gate_waypoints, sweep_window)
    sweep_ids = [i for i in SWEEP_ORDER
                 if i in cals and (ids is None or i in ids)]
    rest = {i: clamp_goal(cals[i], cals[i].rest) for i in sorted(cals)}
    windows = {i: sweep_window(
        cals[i], min(span_pct, SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
        for i in sweep_ids}
    return gate_waypoints(cals, rest, windows, sweep_ids)


# ------------------------------------------------------------------- CLI
def _print_provisional() -> None:
    unverified = [i for i, jm in sorted(JOINT_MAPS.items())
                  if not jm.bench_verified]
    if not unverified:
        return
    print(f"\nPROVISIONAL mapping on joint(s) {unverified} — verify at "
          f"the bench before trusting the gate:")
    for i in unverified:
        jm = JOINT_MAPS[i]
        print(f"  j{i} ({jm.model_joint}): jog it in its calibrated "
              f"POSITIVE direction and check the model agrees; if it "
              f"moves the opposite way, flip direction to "
              f"{-jm.direction:+d} in JOINT_MAPS[{i}] (sim/twin.py)")


def cmd_check(twin: Twin) -> int:
    rest = {i: twin.cals[i].rest for i in sorted(twin.cals)}
    found, clamps, excused = twin.contacts_at(rest)
    if found:
        for c in found:
            print(f"rest pose contact: {c.body_a} <-> {c.body_b} "
                  f"({c.depth_mm:.1f} mm)")
    else:
        print(f"rest pose is contact-free "
              f"(margin {CONTACT_MARGIN_M * 1000:.0f} mm)")
    for i, deg in sorted(clamps.items()):
        print(f"  note: joint {i} clamped {deg:.1f} deg to the model's "
              f"range at rest")
    # Never let this be invisible — it is the one place the gate
    # deliberately looks away, so it gets printed every time.
    print("\nstructural nesting pairs (proximity never fails these; "
          "penetration is\nwaived only during a settle from a measured "
          "slump):")
    if not twin._structural:
        print("  none — no link pairs touch at the rest pose")
    for pair in sorted(twin._structural, key=sorted):
        print(f"  {' <-> '.join(sorted(pair))}")
    print("  the table is NEVER in this set — a table contact always fails")
    if excused:
        print(f"  {excused} contact(s) waived at the rest pose itself")
    _print_provisional()
    return 1 if found else 0


def exercise_clip_for(cals: dict[int, JointCal], span_pct: int,
                      ids: list[int] | None = None,
                      profile=None):
    """The exercise routine as a CLIP — poses AND the motion profile.

    Same delegation as `exercise_waypoints`, one layer up: the routine
    is defined once, in exercise.py, and everything that simulates or
    plays it resolves to that definition rather than restating it."""
    from hardware.bench.exercise import (
        ACCELERATION, SPAN_MAX, SPEED_BASE, SWEEP_ORDER, SWEEP_SPAN_CAPS,
        clamp_goal, exercise_clip, sweep_window)
    from sim.clip import MotionProfile

    sweep_ids = [i for i in SWEEP_ORDER
                 if i in cals and (ids is None or i in ids)]
    rest = {i: clamp_goal(cals[i], cals[i].rest) for i in sorted(cals)}
    windows = {i: sweep_window(
        cals[i], min(span_pct, SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
        for i in sweep_ids}
    return exercise_clip(cals, rest, windows, sweep_ids, None,
                         profile or MotionProfile(speed=SPEED_BASE,
                                                  acceleration=ACCELERATION))


def cmd_exercise(twin: Twin, span: int) -> int:
    from sim.clip import clip_duration

    clip = exercise_clip_for(twin.cals, span)
    report = twin.check_clip(clip)
    print(report.summary(twin.cals))
    print(f"  clip: {len(clip.poses)} poses, {clip_duration(clip):.1f} s at "
          f"speed {clip.profile.speed} ticks/s, acceleration "
          f"{clip.profile.acceleration}")
    print("CLEAR" if report.clean else "WOULD COLLIDE")
    _print_provisional()
    return 0 if report.clean else 1

def cmd_derive_clearance(twin: Twin, span: int) -> int:
    """Scan (m2 sweep span, elbow opening, m4 opening) for contact-free
    combinations — the fixed-hold answer the exercise routine needs.
    A fixed hold that clears the table at the sweep's low end can fold
    the arm into itself at the high end, so span is part of the answer."""
    from hardware.bench.exercise import clearance_pose, sweep_window
    cals = twin.cals
    rest = {i: cals[i].rest for i in sorted(cals)}
    best = None
    print(f"{'span%':>5}  {'elbow deg':>9}  {'m4 deg':>6}  result")
    for span_pct in sorted({span, 60, 50, 40, 30, 20}, reverse=True):
        lo2, hi2 = sweep_window(cals[2], span_pct)
        found = None
        for elbow_deg in range(0, 95, 5):
            for m4_deg in (0, elbow_deg // 2, elbow_deg):
                pose = {**rest, 3: clearance_pose(cals[3], elbow_deg)}
                if m4_deg:
                    pose[4] = clearance_pose(cals[4], m4_deg)
                seq = [pose, {**pose, 2: lo2}, {**pose, 2: hi2},
                       {**pose, 2: rest[2]}]
                if twin.check_trajectory(seq).clean:
                    found = (elbow_deg, m4_deg)
                    break
            if found:
                break
        if found:
            print(f"{span_pct:>5}  {found[0]:>9}  {found[1]:>6}  CLEAR")
            if best is None:
                best = (span_pct, *found)
        else:
            print(f"{span_pct:>5}  {'-':>9}  {'-':>6}  no clear hold <=90")
    if best:
        print(f"\nrecommend: m2 span {best[0]}%, elbow hold {best[1]} deg, "
              f"m4 hold {best[2]} deg")
    else:
        print("\nno contact-free fixed-hold m2 sweep found — coordinated "
              "motion or a much smaller span is required")
    return 0

def cmd_validate(twin: Twin, span: int) -> int:
    """The gate must predict the two collisions the bench actually had.

    Both replicas pin their own geometry — span 70 and elbow 45 were the
    conditions of the REAL runs, so they must never follow the shipped
    constants (which the twin itself later changed)."""
    from hardware.bench.exercise import clearance_pose, sweep_window
    cals = twin.cals
    rest = {i: cals[i].rest for i in sorted(cals)}
    lo2, hi2 = sweep_window(cals[2], 70)  # the failed runs' span
    ok = True

    def sweep_with(pose: dict[int, int]) -> GateReport:
        return twin.check_trajectory(
            [pose, {**pose, 2: lo2}, {**pose, 2: hi2}, {**pose, 2: rest[2]}])

    r1 = sweep_with(dict(rest))  # run 1: everything folded
    print("run-1 replica (span 70, m2 sweep, all folded):",
          "predicts contact PASS" if not r1.clean
          else "predicts NO contact FAIL")
    ok &= not r1.clean

    # run 2: elbow held 45 deg (the ORIGINAL hand-tuned hold), m4 folded
    pose2 = {**rest, 3: clearance_pose(cals[3], 45)}
    r2 = sweep_with(pose2)
    print("run-2 replica (span 70, m2 sweep, elbow 45 open, m4 folded):",
          "predicts contact PASS" if not r2.clean
          else "predicts NO contact FAIL")
    ok &= not r2.clean
    if not ok:
        print("MODEL DISAGREES WITH THE BENCH — anchoring or geometry is "
              "off; do not trust the gate until the bench verification")
    _print_provisional()
    return 0 if ok else 1


# ---------------------------------------------------------- frame check
# The pitch chain (j2/j3/j4) all rotate about the SAME world axis, +X at
# pan zero — that is what lets one sign rule cover all three, and it is
# asserted below rather than assumed. Their ratified convention is the
# DH/URDF one: a joint reads ZERO when the segment it drives is collinear
# with its parent, and POSITIVE by the right-hand rule about +X.
# j2's parent is the vertical base column, so its zero is "upper arm
# straight up" — and all three at zero is the arm standing straight up,
# which is eyeball-checkable at the bench.
#
# j1 (pan, about +Z) and j5 (roll, about the tool axis) have no collinear
# reference — nothing to be in line with — so they are not covered here.
# Both were bench-verified by jog on 2026-07-25 instead.
PITCH_SEGMENTS = {
    2: ("upper_arm", "lower_arm"),
    3: ("lower_arm", "wrist"),
    4: ("wrist", "gripper"),
}
PITCH_PARENTS = {
    3: ("upper_arm", "lower_arm"),
    4: ("lower_arm", "wrist"),
}
# The measurement axis is READ FROM THE MODEL (each joint's own world
# axis at the pose being measured), never declared. It was a hardcoded
# world +X for the SO-100 package; the SO-101's base frame is rotated,
# so its pitch axis is +Y, and a constant here would have silently
# measured the wrong angle rather than failing.
FRAME_TOL_DEG = 1.5  # a tick is 0.088 deg; this is slop for mesh origins


def relative_angle(twin: Twin, joint: int, tick: int) -> tuple[float, float]:
    """(signed angle deg, clamp deg) from the parent direction to the
    segment this joint drives, right-hand rule about +X — the quantity
    the ratified frame claims to display.

    The clamp is returned because the MODEL's joint limits are in places
    tighter than the arm's calibrated range (j3's elbow saturates ~5.5
    deg before the calibrated minimum). At a clamped tick the model is
    not showing the pose that was asked for, so it cannot verify a frame
    there — the caller must skip rather than report a false failure.
    """
    pose = {i: twin.cals[i].rest for i in sorted(twin.cals)}
    pose[joint] = tick
    twin.data.qpos[:] = twin._rest_qpos
    clamped = 0.0
    for i, t in pose.items():
        q, clamp_deg = twin.qpos_of(i, t)
        if i == joint:
            clamped = clamp_deg
        twin.data.qpos[twin._adr[i]] = q
    mujoco.mj_forward(twin.model, twin.data)
    return twin._seg_angle(joint), clamped


def cmd_frames(twin: Twin) -> int:
    """Verify each pitch joint's ratified frame against the MODEL.

    Checks the two things a frame can get wrong, at three probe angles:
    where zero sits, and which way positive runs. A frame that passes is
    correct by construction rather than by anyone eyeballing a printout —
    which is how j4 shipped reading exactly backwards for two days.
    """
    fails: list[str] = []
    for j in sorted(PITCH_SEGMENTS):
        cal = twin.cals.get(j)
        if cal is None or not isinstance(cal.frame, DegFrame):
            continue
        print(f"  j{j} ({cal.name}) — {cal.frame.label or 'no label'}")
        for want in (0.0, 30.0, -30.0):
            tick = cal.frame.tick(want)
            if not (cal.min <= tick <= cal.max):
                print(f"      {want:+6.1f} deg -> tick {tick} outside the "
                      f"calibrated range, skipped")
                continue
            got, clamp = relative_angle(twin, j, tick)
            if clamp > CLAMP_REPORT_DEG:
                print(f"      [skip] {want:+6.1f} deg at tick {tick:5d}: "
                      f"the MODEL clamps by {clamp:.1f} deg here, so it "
                      f"cannot verify this angle")
                continue
            ok = abs(got - want) <= FRAME_TOL_DEG
            if not ok:
                fails.append(f"j{j} at {want:+.0f} deg reads {got:+.1f}")
            print(f"      [{'ok ' if ok else 'FAIL'}] frame says "
                  f"{want:+6.1f} deg at tick {tick:5d}; model measures "
                  f"{got:+6.1f} deg")
    print("frame check " + ("OK" if not fails else f"FAILED: {fails}"))
    return 1 if fails else 0


def cmd_selftest(twin: Twin) -> int:
    """Pin the gate's safety contracts. No hardware, no motion.

    Acceptance alone proves nothing — a gate that accepts everything
    would pass it — so every acceptance here is paired with a refusal.
    """
    rest = {i: twin.cals[i].rest for i in sorted(twin.cals)}
    # NOT clamped to the calibrated range on purpose: the real slump had
    # j3 seven ticks PAST its own calibrated max (that joint's max sits
    # one tick above its rest, so the capture never swept past the fold).
    # Clamping the fixture would erase the very penetration under test.
    slump = {i: rest[i] + d
             for i, d in OBSERVED_SLUMP_DELTA.items() if i in rest}
    # SYNTHETIC — 1.5x the measured slump deviation. Not a pose the arm
    # has ever been in, and it is not presented as one: it exists only to
    # drive a structural pair into real penetration so the settle waiver
    # has something to waive. On the SO-101 geometry the MEASURED slump
    # only reaches proximity (which structural pairs never fail on), so
    # without this the waiver would be untested code claiming to be a
    # safety property.
    #
    # 1.5 is bounded on BOTH sides by measurement, not chosen for taste:
    # below ~1.2 nothing penetrates and the test is vacuous, and at 1.6
    # the gripper reaches the TABLE, which is never waived and would
    # make the test fail for the wrong reason. The usable window is
    # narrow, which is itself worth knowing.
    deep = {i: rest[i] + round(1.5 * d)
            for i, d in OBSERVED_SLUMP_DELTA.items() if i in rest}
    lift = {**rest, 2: rest[2] + round(25 / span_deg(1))}
    fails: list[str] = []

    def want(label: str, got_clean: bool, expected: bool) -> None:
        if got_clean != expected:
            fails.append(label)
        print(f"  [{'ok ' if got_clean == expected else 'FAIL'}] {label}")

    def traj(label, wps, expected, settle):
        r = twin.check_trajectory(wps, settle_from_measured=settle)
        want(label, r.clean, expected)

    def pose(label, p, expected, adjudicate=True):
        found, _, _ = twin.contacts_at(p, adjudicate_nesting=adjudicate)
        want(label, not found, expected)

    if any("table" in p for p in twin._structural):
        fails.append("TABLE LEAKED INTO THE STRUCTURAL SET")
        print("  [FAIL] table must never be a structural nesting pair")
    else:
        print("  [ok ] table is excluded from the structural set")

    traj("settle from the OBSERVED slump onto rest is accepted",
         [slump, rest], True, True)
    traj("...and the observed slump no longer needs the waiver at all — "
         "it is accepted without it, because on this geometry it only "
         "comes into proximity, never penetration (it DID penetrate on "
         "the SO-100 model, at a pose the real arm sat in untouched)",
         [slump, rest], True, False)
    traj("a synthetic deeper fold IS accepted during the settle",
         [deep, rest], True, True)
    traj("...and is REFUSED without the settle waiver — so the waiver is "
         "what accepts it, not luck", [deep, rest], False, False)
    traj("the waiver does not leak past the settle",
         [rest, rest, deep], False, True)
    traj("a table strike during the settle is still refused",
         [slump, lift], False, True)
    pose("the run-1 bench collision is still predicted", lift, False)
    pose("rest is clean", rest, True)

    # --- plan #660: the gate and the player must be the SAME motion ---
    # Not "do they read the same waypoints" (they always did) but "does
    # the gate walk the exact frames the viewer plays". Anything the gate
    # interpolates for itself is a path nobody watches, and anything the
    # viewer interpolates for itself is a path nobody gated.
    from sim.clip import DEFAULT_HZ, MotionProfile, sample_clip

    clip = exercise_clip_for(twin.cals, 70)
    frames = sample_clip(clip, DEFAULT_HZ)
    want("the gate checks EXACTLY the frames the viewer plays",
         twin.check_clip(clip).poses_checked == len(frames), True)
    want("...and that is more than one frame per pose, so it is really "
         "walking the motion", len(frames) > 4 * len(clip.poses), True)

    # An edit to the routine must reach the gate. If these matched, the
    # gate would be simulating something the routine no longer says.
    narrow = exercise_clip_for(twin.cals, 30)
    want("editing the routine changes what the gate simulates",
         sample_clip(narrow, DEFAULT_HZ) != frames, True)

    # The profile is part of the definition, not a viewer preference:
    # same poses at a different speed is a different motion.
    slow = exercise_clip_for(twin.cals, 70,
                             profile=MotionProfile(
                                 speed=max(1, clip.profile.speed // 4),
                                 acceleration=clip.profile.acceleration))
    slow_frames = sample_clip(slow, DEFAULT_HZ)
    want("the same poses at a slower profile are a DIFFERENT motion",
         len(slow_frames) > len(frames), True)
    want("...but still end on the same pose",
         slow_frames[-1] == frames[-1], True)

    # The sample rate is a SAFETY parameter, not a smoothness setting: a
    # link that moves further between two samples than the contact
    # margin can pass a thin obstacle that sits between them. Assert the
    # property so DEFAULT_HZ cannot be lowered (or a clip made faster)
    # without this failing.
    def worst_step_mm(fr: list[dict[int, int]]) -> float:
        bid = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_BODY,
                                TOOL_BODY)
        pts = []
        for f in fr:
            twin.data.qpos[:] = twin._rest_qpos
            for i, tk in f.items():
                twin.data.qpos[twin._adr[i]] = twin.qpos_of(i, tk)[0]
            mujoco.mj_forward(twin.model, twin.data)
            pts.append(twin.data.xpos[bid].copy())
        return max((float(np.linalg.norm(b - a)) * 1000.0
                    for a, b in zip(pts, pts[1:])), default=0.0)

    margin_mm = CONTACT_MARGIN_M * 1000.0
    step = worst_step_mm(frames)
    want(f"sampling is fine enough not to tunnel: worst tool step "
         f"{step:.2f} mm vs {margin_mm:.0f} mm margin (want 2x headroom)",
         step < margin_mm / 2.0, True)
    coarse = worst_step_mm(sample_clip(clip, 10.0))
    want(f"...and a deliberately coarse rate DOES breach it "
         f"({coarse:.2f} mm at 10 Hz), so the check can fail",
         coarse > margin_mm, True)
    print("twin selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    return 1 if fails else 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m sim.twin",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=(
        "check", "exercise", "derive-clearance", "validate", "selftest",
        "frames"))
    parser.add_argument("--cal", default="calibration.json")
    parser.add_argument("--span", type=int, default=70,
                        help="sweep span %% (exercise's default)")
    args = parser.parse_args()
    try:
        twin = Twin(cal_path=args.cal)
        if args.command == "check":
            return cmd_check(twin)
        if args.command == "exercise":
            return cmd_exercise(twin, args.span)
        if args.command == "derive-clearance":
            return cmd_derive_clearance(twin, args.span)
        if args.command == "selftest":
            return cmd_selftest(twin)
        if args.command == "frames":
            return cmd_frames(twin)
        return cmd_validate(twin, args.span)
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
