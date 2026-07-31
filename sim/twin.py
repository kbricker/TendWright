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
  - the arm's observed torque-off slump reads 0.503 mm into the model
    (the SO-100 read 0.14 mm), on a structurally-nesting pair, while the
    real arm sits in that pose untouched. This line used to claim the
    slump NO LONGER penetrated and that the new geometry therefore
    matched reality better — true only while j5's direction was reverted
    behind the model swap. Correcting it (see `_rest_direction`) rolls
    the jaw to the other side of a half-millimetre fold. 0.503 mm needs
    3.2 deg of j5 slump error to explain, and the slump's own measured
    scatter on j5 is 4.7 deg, so this is inside the noise rather than
    evidence about the geometry either way;
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

from hardware.bench.calibrate import JointCal, load_joint_calibration
from hardware.errors import BenchError
from hardware.units import DegFrame, RAD_PER_TICK, fmt_ticks, span_deg

MODEL_XML = Path(__file__).parent / "assets/so101/so101_new_calib.xml"
# Safety margin: geom pairs within this distance are surfaced as
# near-misses, because the mapping carries real uncertainty (print
# tolerances, a hand-posed rest anchor) and a 5 mm miss in the model is
# not a pass on a real arm.
#
# This used to say "two provisional directions". Every direction is now
# derived from the model and both rest-anchored ones are corroborated
# by a bench check (`_rest_direction`) — but between #670 and 2026-07-30
# BOTH were silently reverted, and the margin was carrying a defect it
# was never sized for the whole time.
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
# THE CONFLICT IS NOT GONE. This block claimed, from #670 until
# 2026-07-30, that "on the SO-101 geometry the observed slump only
# reaches PROXIMITY ... accepted with or without the waiver — pinned
# both ways in selftest", offered as evidence the new model matched the
# real arm better. Every part of that was an artefact of j5's direction
# having been reverted by the same model swap (`_rest_direction`): with
# j5 correct the slump penetrates by 0.503 mm, the selftest pins it
# needing the waiver, and there is no both-ways pin to go looking for.
#
# It is 3.2 deg of j5 away from clean and the slump's own scatter on j5
# is 4.7 deg, so it argues nothing about the geometry — which is exactly
# why the waiver's paired refusal still runs on a clearly-labelled
# SYNTHETIC deeper fold rather than on the measured one: a safety waiver
# whose only exercise is a pose sitting inside its own measurement noise
# is a claim, not a property.
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

    `bench_verified` means SOMEBODY HAS LOOKED AT THE ARM and confirmed
    the model turns the same way. It does NOT mean the direction was
    hand-entered — since 2026-07-30 every joint's direction is derived
    (the pitch chain from a segment probe, pan and roll from
    `_rest_direction`). It is the difference between "the model says so"
    and "we watched it".

    IT IS NOT A GUARANTEE THAT TODAY'S CODE STILL HONOURS THE LOOK.
    Plan 714.6 is exactly that gap: j1 and j5 were both watched on
    2026-07-25 and both were reverted the next day by a model swap that
    replaced their stored directions with a constant. The flag stayed
    True and was still true — somebody HAD looked — while the mapping it
    described had changed underneath. Deriving the direction is what
    closes it, because a derivation cannot be reverted by deleting a
    field.

    `sim.twin check` lists any joint nobody has watched. All six are
    watched today, so it prints nothing."""

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
# j1 and j5 pin their ZERO to the physical rest tick, because pan and
# roll have nothing to be collinear with and the model cannot be asked
# where their zero is. Their DIRECTION is a different question and is
# derived — see `_rest_direction`, including how a hardcoded +1 reverted
# both of these behind the model swap.
#
# KEEP THE JULY-25 BENCH ANSWERS. They are the physical facts and they
# outlive any model: Kyle jogged both joints on 2026-07-25 (99aa56e),
# j1 came back as shipped and j5 flipped, verified two ways — omega .
# pointing-direction analytically, and jaw displacement empirically. The
# commit also settled the ambiguity that makes roll hard to talk about:
# handedness inverts when you walk around a roll axis, Kyle stood BEHIND
# the arm, and the ratified convention is head-on FROM THE FRONT. That
# sentence is the only written resolution of it; #670 re-guessed it once
# already and this comment exists so nobody has to a third time.
#
# The gripper's open direction IS geometric and was verified by
# measuring jaw separation across the model's range on both models.
JOINT_MAPS: dict[int, JointMap] = {
    # bench_verified = a human has watched this joint move and agreed.
    # Both rest-anchored joints have: j1 on 2026-07-25 and again on
    # 07-30, j5 on 07-25. `_rest_direction` now reproduces both of those
    # answers from the model rather than storing them.
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


def _record(contacts: list[PredictedContact],
            index: dict[frozenset, int], c: PredictedContact) -> None:
    """Remember the DEEPEST occurrence of each body pair, not the first.

    A trajectory samples the same pair many times, and the two gate
    entry points both deduped by keeping whichever sample came first.
    That made `depth_mm` a SAMPLING-PHASE ARTEFACT rather than a
    severity: a pair that closes gradually is first detected the instant
    it enters the 5 mm margin, i.e. at 0.00 mm, and then keeps that 0.00
    however deep it goes.

    Everything downstream reads it as severity. `posegate.check_sequence`
    and `edges.validate_edge` both rank with `max(..., key=depth_mm)` to
    name the worst pair, so both were ranking on first-contact order.
    Measured on the shipped pose library: 10 of 22 refused edges named
    `shoulder <-> gripper (touching)` — first seen at 0.03 mm — while
    `table <-> gripper` on the same trajectory reached 17.27 mm and was
    recorded as 0.00. The report sent the operator to adjust the fold
    while the real event was the gripper driven 17 mm through the bench.

    `step` and `pose` travel with the depth, so "N mm deep at step S"
    now names where it is WORST rather than where it was first grazed.
    Nothing consumes `step` for control flow; both readers print it.
    """
    pair = frozenset({c.body_a, c.body_b})
    at = index.get(pair)
    if at is None:
        index[pair] = len(contacts)
        contacts.append(c)
    elif c.depth_mm > contacts[at].depth_mm:
        contacts[at] = c


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
        self.cals = load_joint_calibration(Path(cal_path))
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

    def _rest_direction(self, i: int) -> int:
        """+1 or -1: does POSITIVE model rotation match the ratified
        frame's positive sense, for a joint anchored at rest?

        THIS EXISTS BECAUSE A MODEL SWAP SILENTLY REVERTED TWO
        BENCH-VERIFIED SIGNS, and the mechanism is worth more than the
        bug. `JointMap` used to carry a per-joint `direction` field.
        #648 shipped j1 and j5 at -1; Kyle jogged both at the bench on
        2026-07-25 (99aa56e) and the answers were recorded — j1
        confirmed, j5 flipped, with the roll's viewpoint ambiguity
        explicitly adjudicated: *"handedness inverts when you walk around
        a roll axis and Kyle stood behind the arm while the capture
        convention is head-on from the front."*

        #670 (daf25b4) then swapped the model SO-100 -> SO-101, deleted
        `direction`, and hardcoded the slope to +1. That number happened
        to equal j1's effective sign, so j1 looked untouched — but the
        SO-100's pan axis is +Z and the SO-101's is -Z, so the same
        constant now meant the opposite rotation. j5 was simply reverted
        to its pre-bench value. A constant replacing a per-joint field
        reverts every verified sign without changing a number.

        NO TEST COULD SEE IT, and the two joints hid for different
        reasons — worth separating, because only one of them is a
        permanent property:

          j1 is PROVABLY invariant for collision. `base` carries no
            collidable geometry (every base geom is visual, contype 0),
            every collidable body descends from `shoulder`, and the only
            world geom is a horizontal infinite plane — so rotating the
            whole arm about the vertical maps the contact set onto
            itself exactly. Measured: 936/936 mirrored poses identical
            to 1e-6 mm. This stops being true the day a base mesh, a
            fixture (714.4) or a wall enters the model.
          j5 is NOT invariant. It only looked so because every pose in
            the shipped library has j5 = 0, where the mapping is the
            same either way. Over the fold region with j5 within +/-25
            deg, 1136 verdicts flip — in BOTH directions, including
            poses the reverted sign called CONTACT that the correct one
            calls CLEAN. The first pose with a rolled jaw would have
            made the gate's answer sign-dependent.

        Found 2026-07-30, five days later, by Kyle looking at the arm:
        parked at j1 tick 1578 (+45.0 deg) it swung to its LEFT while the
        twin put the tool at y -117.7 in the rig frame, its RIGHT.

        WHAT THIS FUNCTION RESTORES, it restores to the bench's own
        answers. Run the rule below against the retired SO-100 — its own
        joints (`Rotation`, `Wrist_Roll`) and its own gripper bodies
        (`Fixed_Jaw`, `Moving_Jaw`) — and it returns +1 for j1 and -1 for
        j5, which is exactly what Kyle measured on 2026-07-25. Its pan
        axis is +Z where the SO-101's is -Z, so the two models need
        OPPOSITE constants to mean the same physical rotation, and the
        rule gets both right without being told. That is the strongest
        evidence available that this is the right rule rather than one
        that happens to fix today's symptom, and it is reproducible:

            SO-100   pan . +Z = +1.000    roll . ref = -0.9995
            SO-101   pan . +Z = -1.000    roll . ref = -0.9846

        USE THE RIGHT BODY NAME. An earlier draft of this paragraph said
        `Fixed_Gripper`, which the SO-100 does not have — it is
        `Fixed_Jaw` — and quoted -0.982. That number is the typo's own
        fingerprint: a name matching nothing silently leaves seven geoms
        instead of thirteen, and -0.982 is Moving_Jaw ALONE. A recipe
        that fails by narrowing its input rather than by raising is
        exactly the shape of thing this function exists to catch.

        It is also why the reference is the gripper's collidable geometry
        and not the `gripperframe` site — the SO-100 has no sites, so a
        site-based rule could not be run against the negative control at
        all, and this paragraph would be an assertion instead of an
        experiment.

        WHY THIS IS DERIVABLE WHEN THE ZERO IS NOT. The module's standing
        argument for leaving pan and roll undeduced is that they "have
        nothing to be collinear with", and that is true — of their ZERO.
        A direction needs no collinearity. It needs only the axis the
        frame's own words name, and both frames name one outright:

          j1 "+ = CCW from above"        -> about the model's up, +Z
          j5 "+ = CCW viewed head-on"    -> about the gripper's own
                                            pointing direction, which is
                                            what "head-on" looks along

        so the sign is the dot product of the model's joint axis with
        that reference. One measurement, no transcribed constant, and it
        follows a model swap the way every other anchor here does.

        `cmd_frames` cannot catch this: it verifies exactly the joints in
        PITCH_SEGMENTS, and j1/j5 are not in it. That gap is the whole
        reason the bug lived — the same check DID catch j4 reading
        backwards, as its docstring says.
        """
        # MEASURED AT qpos 0, AND THAT IS LOAD-BEARING — do not move it
        # to `_rest_qpos` because the joint is rest-anchored, tempting as
        # that reads. The j5 reference is the gripper's collidable
        # geometry, and `TOOL_BODY` is j6's child, so opening the jaws
        # swings it: measured dot vs j6 travel,
        #     0% -0.9909   25% -0.9635   50% -0.9192
        #                  75% -0.8702  100% -0.8551
        # The SIGN never comes near flipping, so the derivation is safe
        # anywhere — but the 0.9 guard below trips past ~70% open, and a
        # BenchError here means PoseGate catches it, goes inactive, and
        # `check_sequence` returns CLEAN. An open jaw must not fail the
        # gate open. Across j1-j5 the reference is invariant to float
        # noise (3e-15 over 300 random in-range poses) — exactly, as
        # it must be, since the axis and the geoms are fixed in one
        # body chain and only j6 moves the centroid.
        self.data.qpos[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        axis = np.array(self.data.xaxis[self._jid[i]], dtype=float)
        if i == 1:
            ref = np.array([0.0, 0.0, 1.0])          # "from above"
        else:
            # "head-on" looks back along the direction THE GRIPPER
            # POINTS, so the reference must be a property of the gripper
            # itself. Take it from where the gripper's own COLLIDABLE
            # GEOMETRY sits relative to the roll axis: the jaws are out
            # in front by construction, on any arm worth the name.
            #
            # Two rejected alternatives, both instructive:
            #   m4 -> m5 is the WRIST LINK, not the gripper. It agrees
            #     here to 16.5 deg and would keep agreeing right up until
            #     somebody remounted the gripper doubled back — the link
            #     is unchanged by that, the pointing direction reverses,
            #     and the guard below would see 0.96 and be delighted.
            #   the `gripperframe` site is the right POINT but the wrong
            #     DEPENDENCY. It is an export artefact that nothing else
            #     in this repo reads, the SO-100 has no sites at all (so
            #     the corroboration below could not be run), and a
            #     vendored refresh that dropped it would raise in
            #     __init__ -> PoseGate catches BenchError -> `active`
            #     goes False -> `check_sequence` returns CLEAN. A missing
            #     cosmetic site must not fail the gate open.
            pts = [self.data.geom_xpos[g] for g in range(self.model.ngeom)
                   if self.model.geom_contype[g] != 0
                   and self._body_of_geom(g) in (WRIST_BODY, TOOL_BODY)]
            if not pts:
                raise BenchError(
                    f"no collidable gripper geometry on bodies "
                    f"{WRIST_BODY}/{TOOL_BODY} to take a pointing "
                    f"direction from",
                    "the vendored model changed — see Twin._rest_direction")
            # Indexed by `i`, not a literal 5 — they are the same joint
            # today and `proj` below already uses `i`, so a third
            # rest-anchored joint would silently measure `ref` from the
            # wrong anchor.
            ref = (np.mean(np.array(pts, dtype=float), axis=0)
                   - np.array(self.data.xanchor[self._jid[i]], dtype=float))
        if i != 1:
            # SAME SIDE, not just aligned. The mean decides the sign, and
            # `abs(d)` cannot see it land on the wrong side of the
            # anchor: the SO-101's three gripper geoms sit 33.8 mm in
            # FRONT of it (projections -23.4, -28.2, -48.1 mm along the
            # roll axis, negative because that axis points back down the
            # arm — which is why `d` is negative and the sign is -1). So
            # ONE collidable geom added ~100 mm BEHIND m5 — a
            # wrist-camera bracket, a slip-ring housing, a cable shroud
            # reaching back over the forearm — drags the mean past the
            # anchor, flips the derived sign, and leaves |d| at 0.9846
            # with the alignment guard reporting itself delighted. That
            # is the round-1 bug with a new mechanism, so it gets its own
            # check.
            #
            # TOLERANCED, and not by taste. `min * max <= 0` refuses any
            # model with a geom sitting EXACTLY on the anchor plane —
            # which straddles nothing, contributes nothing to the sign,
            # and is an ordinary way to model an inline roll servo. This
            # module already carries `FRAME_TOL_DEG ... slop for mesh
            # origins` because onshape-to-robot recentres them between
            # exports. A refusal here raises in __init__, PoseGate
            # catches it, `active` goes False, and `check_sequence`
            # returns CLEAN — the exact fail-open this guard exists to
            # prevent, reintroduced by the guard. 1 mm costs nothing
            # against geoms at 23-48 mm and cannot be reached by float
            # noise around a nominal zero.
            STRADDLE_TOL_M = 0.001
            anchor = np.array(self.data.xanchor[self._jid[i]], dtype=float)
            proj = [float(axis @ (np.array(q, dtype=float) - anchor))
                    for q in pts]
            if min(proj) < -STRADDLE_TOL_M and max(proj) > STRADDLE_TOL_M:
                raise BenchError(
                    f"joint {i}'s gripper geometry straddles its own roll "
                    f"axis ({min(proj) * 1000:+.1f} to {max(proj) * 1000:+.1f} "
                    f"mm), so no centroid can say which way it points",
                    "something on the gripper body reaches back behind the "
                    "wrist — see Twin._rest_direction")
        n = float(np.linalg.norm(ref))
        if n < 1e-9:
            raise BenchError(
                f"cannot measure joint {i}'s frame reference direction",
                "the model's geometry moved — see Twin._rest_direction")
        d = float(axis @ (ref / n))
        # 0.9, not 0.5. The message says these should be PARALLEL, and
        # 0.5 is 60 degrees off parallel — a threshold that admits almost
        # anything while sounding strict. At the measurement pose both
        # joints read 1.000 and 0.9846, so 0.9 costs nothing here — but
        # see the note above: the j5 reference degrades to 0.855 with the
        # jaws fully open, so this margin belongs to qpos 0 and not to
        # the joint in general.
        if abs(d) < 0.9:
            raise BenchError(
                f"joint {i}'s model axis is only {d:.2f} aligned with the "
                f"axis its ratified frame names — they should be parallel",
                "either JOINT_MAPS points at the wrong model joint or the "
                "frame's label no longer describes it")
        return 1 if d > 0 else -1

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
                # No geometric ZERO to find, so pin physical rest to the
                # model's zero. The DIRECTION is a separate question and
                # it IS answerable — see `_rest_direction`.
                s = self._rest_direction(i)
                lin[i] = (-s * math.radians(cal.frame.deg(cal.rest)),
                          s * math.radians(1.0))
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
        worst_of: dict[frozenset, int] = {}
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
                _record(contacts, worst_of, c)

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
        body pair, keeping the DEEPEST occurrence — see `_record` for
        why the first one is the wrong answer.

        settle_from_measured: waypoint 0 is the arm's MEASURED slump
        rather than a planned pose, so structural nesting is not
        adjudicated until the arm reaches waypoint 1 (`rest`). Pass it
        only when that is literally true — see the module note."""
        if not waypoints:
            return GateReport(0, [], {}, 0)
        contacts: list[PredictedContact] = []
        worst_of: dict[frozenset, int] = {}
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
                _record(contacts, worst_of, c)

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
    print(f"\nDERIVED BUT NEVER WATCHED — joint(s) {unverified}. The "
          f"direction is measured off the model, not declared, but no "
          f"human has confirmed the model agrees with the arm:")
    for i in unverified:
        jm = JOINT_MAPS[i]
        print(f"  j{i} ({jm.model_joint}): jog it in its calibrated "
              f"POSITIVE direction and LOOK at it, then compare against "
              f"`sim.rig where`. This is not a formality — j1 and j5 "
              f"were both watched on 2026-07-25 and both were reverted "
              f"the next day by a model swap, and every collision "
              f"verdict stayed green for five days regardless. Only a "
              f"person looking at the arm found it (plan 714.6).")
    # This loop used to print `jm.direction`. The field was real once —
    # #648 shipped it and #670 deleted it — so the reference outlived
    # its target and became an AttributeError. It could not fire,
    # because it runs only for UNVERIFIED joints and the two the swap
    # had just broken were both marked verified: the recovery
    # instructions for a reverted joint were themselves broken, and the
    # reverted joints were the ones claiming not to need them.


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
    # has ever been in, and it is not presented as one: it exists to
    # drive a structural pair UNAMBIGUOUSLY into penetration so the
    # settle waiver has something to waive that is not inside its own
    # measurement noise.
    #
    # THE LOWER BOUND MOVED, 2026-07-30. This used to read "below ~1.2
    # nothing penetrates and the test is vacuous", which was true while
    # j5's direction was reverted behind the model swap. With j5 correct
    # (`_rest_direction`) the measured slump itself penetrates: x1.0 =
    # 0.503 mm, x1.1 = 0.980, x1.2 = 1.348, x1.5 = 2.643. So there is no
    # vacuous floor any more and 1.5 is no longer pinned from below by
    # geometry.
    #
    # It is kept at 1.5 for a different and better reason: 0.503 mm is
    # 3.2 deg of j5 away from clean and the slump's own scatter on j5 is
    # 4.7 deg, so a waiver exercised only at x1.0 would be exercised by a
    # pose that might not penetrate on the next capture. 1.5 clears that
    # scatter outright. The UPPER bound is unchanged and still measured:
    # at 1.6 the gripper reaches the TABLE, which is never waived and
    # would fail the test for the wrong reason.
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
    # THE SLUMP NEEDS THE WAIVER AGAIN, and the reason is worth keeping.
    # This assertion used to read "no longer needs the waiver at all",
    # offered as evidence the SO-101 geometry fits the real arm better
    # than the SO-100's. It was true only while j5's direction was
    # hardcoded backwards: correcting it (2026-07-30) rolls the jaw to
    # the other side of a fold where the clearance is half a millimetre,
    # and the pose goes from 0.000 mm to 0.503 mm of shoulder <-> gripper
    # penetration. j1's sign, measured the same day, changes this by
    # exactly nothing — pan rotates the arm without altering its shape.
    #
    # The real arm sits in that slump untouched, so the model is 0.5 mm
    # pessimistic there. That is inside the uncertainty this module
    # already declares, and on a pair that nests structurally by design.
    #
    # IT IS NOT A TEST OF j5'S SIGN, though an earlier version of this
    # comment offered it as one ("if a bench check shows the two parts
    # clear at the slump, j5 is wrong"). Getting from 0.503 mm to clean
    # takes 3.2 deg of j5, and the slump's own re-capture scatter on j5
    # is 4.7 deg — documented at the top of this module. A criterion
    # that cannot separate a sign error from less noise than the input
    # already carries is worse than no criterion, because acting on it
    # would flip a bench-corroborated sign on half a millimetre. The
    # sign's evidence is the bench check and the SO-100 cross-check in
    # `_rest_direction`, not this pose.
    # PINS TODAY'S CAPTURE, not a property of the arm. `traj` asserts
    # only that the waiver is load-bearing here; the 0.503 mm is in the
    # label so a drift is legible rather than mysterious. A re-capture
    # can move j5 by up to 4.7 deg and 3.2 deg of that clears the
    # contact, so if this ever goes red, re-read OBSERVED_SLUMP_DELTA
    # before suspecting the geometry.
    traj("the observed slump needs the settle waiver on TODAY's capture "
         "— 0.503 mm of shoulder <-> gripper, a structurally nesting "
         "pair (a re-capture may legitimately clear it)",
         [slump, rest], False, False)
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

    # --- WHICH WAY POSITIVE RUNS, for the two joints cmd_frames cannot
    # reach. PITCH_SEGMENTS covers j2/j3/j4, so pan and roll had no
    # direction check of any kind, and both were reverted by #670's
    # model swap for five days (see `_rest_direction`). Nothing failed:
    # j1's sign cannot change a collision verdict at all, and j5's could
    # not either while every library pose had j5 = 0. A magnitude-only
    # assertion would have passed the whole time. These pin the SIGN.
    print("\npositive commands turn the way the ratified frames say")
    from sim.rig import Rig
    _rig = Rig(twin)

    def _tool(deg: dict):
        p = {i: twin.cals[i].frame.tick(deg.get(i, 0.0))
             for i in sorted(twin.cals) if i != 6}
        p[6] = twin.cals[6].rest
        q = twin._rest_qpos.copy()
        for i, t in p.items():
            q[twin._adr[i]] = twin.qpos_of(i, t)[0]
        return np.array(_rig.tool_point(q))

    # j1: "+ = CCW from above". The rig frame is m1-centred with +X the
    # reach and +Z up, and it is right-handed, so +Y is the arm's LEFT
    # (facing +X with +Z up, as in East-North-Up). CCW from above must
    # therefore carry the tool toward +Y.
    out = {2: 15.0, 3: 84.0, 4: 81.0}
    left, right = _tool({**out, 1: 45.0}), _tool({**out, 1: -45.0})
    want(f"j1 +45 puts the tool to the arm's LEFT (+Y), not its right "
         f"(y {left[1]:+.1f} vs {right[1]:+.1f} mm at -45)",
         left[1] > 0 > right[1], True)
    # WRAPPED. An unwrapped atan2 difference is fine at +/-45 but reads
    # -270 the day the probe angles straddle the branch cut.
    swept = (math.degrees(math.atan2(left[1], left[0])
                          - math.atan2(right[1], right[0]))
             + 180.0) % 360.0 - 180.0
    want(f"...and -45 -> +45 sweeps +90 deg CCW about +Z, not -90 "
         f"({swept:+.1f} deg)", abs(swept - 90.0) < 1.0, True)
    # NOT asserting `frame.tick(45.0) == 1578`. That was here and it
    # touches no model at all — it is round(2090 - 45/0.0879), frame
    # arithmetic that cannot fail for any geometry and WILL fail the day
    # j1's zero_tick is re-ratified. A comment wearing an assertion's
    # clothes. The bench fact it was trying to record lives in
    # `_rest_direction`'s docstring, where prose belongs.

    # j5: "+ = CCW viewed head-on", i.e. about the axis the gripper
    # points along — which is what "head-on" looks back down.
    #
    # MEASURED ON THE JAW'S ORIENTATION, not on the tool POSITION. The
    # first version of this crossed the two tool offsets about their own
    # midpoint; those are antiparallel by construction, so the cross
    # product was ~0 and the assertion `turn > 0` passed on a number
    # indistinguishable from zero. It also took the pointing axis at
    # qpos 0 while testing at a bent pose, where the axis has swung 90
    # deg away. Two bugs, one vacuous pass.
    tb = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_BODY, TOOL_BODY)

    def _jaw_axis(deg: dict):
        p = {i: twin.cals[i].frame.tick(deg.get(i, 0.0))
             for i in sorted(twin.cals) if i != 6}
        p[6] = twin.cals[6].rest
        twin.data.qpos[:] = twin._rest_qpos
        for i, t in p.items():
            twin.data.qpos[twin._adr[i]] = twin.qpos_of(i, t)[0]
        mujoco.mj_forward(twin.model, twin.data)
        return twin.data.xmat[tb].reshape(3, 3)[:, 2].copy()

    # THE SAME REFERENCE THE DERIVATION USES, for two reasons. It kept
    # `gripperframe` until 2026-07-30 — eighty lines after
    # `_rest_direction` rejects that site by name — and unguarded:
    # `mj_name2id` returns -1 for a missing name and `site_xpos[-1]`
    # wraps silently to the LAST site, which on this model is
    # `baseframe` at the arm's origin. A vendored refresh dropping
    # `gripperframe` while keeping `baseframe` would have pointed this
    # backwards down the arm, read turn ~= -59.9, and FAILED reporting a
    # mirror that does not exist — on the one assertion whose job is to
    # catch a real one.
    _jaw_axis({**out, 5: 0.0})
    _pts = [twin.data.geom_xpos[g] for g in range(twin.model.ngeom)
            if twin.model.geom_contype[g] != 0
            and twin._body_of_geom(g) in (WRIST_BODY, TOOL_BODY)]
    point = (np.mean(np.array(_pts, dtype=float), axis=0)
             - np.array(twin.data.xanchor[twin._jid[5]], dtype=float))
    point /= float(np.linalg.norm(point))
    u, v = _jaw_axis({**out, 5: -30.0}), _jaw_axis({**out, 5: 30.0})
    turn = math.degrees(math.atan2(float(np.cross(u, v) @ point),
                                   float(u @ v)))
    # 1.0 deg, not 3.0. The original tolerance was absorbing a wrong
    # reference axis — m4 -> m5, the wrist link, tilted 16.5 deg off the
    # gripper's actual pointing direction — and reading 58.9. Taking the
    # axis from the gripper's own collidable geometry, the same
    # reference the derivation uses, reads 59.5: what is left is tick
    # quantisation plus the few degrees between the jaw centroid and the
    # roll axis, which is a real property of the gripper rather than an
    # error in the measurement.
    want(f"j5 -30 -> +30 turns +60 deg CCW about the gripper's pointing "
         f"axis, not -60 ({turn:+.1f} deg)", abs(turn - 60.0) < 1.0, True)
    # NOT asserting `JOINT_MAPS[5].bench_verified`. That is a bool
    # literal 900 lines up — it touches no geometry, cannot fail for any
    # model, and would go red when somebody honestly clears the flag for
    # a new arm, claiming a SIGN was wrong when a FLAG changed. It is
    # the same shape as the `frame.tick(45.0) == 1578` assertion deleted
    # twenty lines above. The bench provenance is prose, in
    # `_rest_direction`; what is asserted here is the geometry.

    # --- a reported depth is a SEVERITY, not a sampling phase ---
    # Both readers rank pairs with max(..., key=depth_mm) to name the
    # worst one. That is only meaningful if depth_mm is the worst depth
    # the pair reached; deduping on first-seen made it the depth at
    # which the pair entered the 5 mm margin, which for a gradual
    # approach is 0.00 however deep it later goes.
    # Re-walk the same trajectory independently of the dedupe under test
    # and take each pair's true maximum.
    # OUT AND BACK, so every pair's maximum is in the MIDDLE. A one-way
    # sweep deepens monotonically to its endpoint, which makes "keep the
    # deepest" and "keep the last" indistinguishable — measured: all
    # three pairs peaked at the final sample, and an unconditional
    # `contacts[at] = c` passed every assertion. The adjacent wrong
    # implementation has to be fenced off, not just the original one.
    route = [rest, lift, rest]
    truth: dict[frozenset, float] = {}
    STEPS = 60
    for leg in range(len(route) - 1):
        a_, b_ = route[leg], route[leg + 1]
        for s in range(STEPS + 1):
            f = s / STEPS
            mid = {i: round(a_[i] + f * (b_.get(i, a_[i]) - a_[i]))
                   for i in a_}
            for k in twin.contacts_at(mid)[0]:
                pair = frozenset({k.body_a, k.body_b})
                truth[pair] = max(truth.get(pair, 0.0), k.depth_mm)
    reported = twin.check_trajectory(route).contacts
    # Or the loop below asserts NOTHING while the suite still prints OK.
    # A recalibration that moves `rest` far enough for this edge to come
    # clean would retire a 17 mm regression guard in silence.
    want("the reporting-depth check has a colliding edge to run on",
         bool(reported), True)
    for c in reported:
        deepest = truth.get(frozenset({c.body_a, c.body_b}), 0.0)
        want(f"{c.body_a} <-> {c.body_b} is reported at its DEEPEST "
             f"({c.depth_mm:.2f} mm vs {deepest:.2f} independently "
             f"measured), not where it was first grazed",
             c.depth_mm >= deepest - 0.5, True)

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
