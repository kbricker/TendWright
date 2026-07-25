"""Digital twin of the SO-101 bench arm — collision-gated motion (plan #648).

Maps calibrated servo ticks onto the vendored MuJoCo Menagerie SO-ARM100
model (sim/assets/menagerie/trs_so_arm100/, Apache-2.0) and checks
planned trajectories for self-collision and table contact BEFORE the
real arm moves. The bench tools call `Twin.check_trajectory` as a
pre-flight gate; contacts are predicted, not discovered.

Tick -> radian mapping: per joint, qpos = anchor_qpos + direction *
RAD_PER_TICK * (tick - anchor_tick). Anchors tie the CAPTURED physical
rest/closed pose to the model's `rest` keyframe (the Menagerie model
ships one that matches the arm's folded slump). Directions were resolved
by FK probes on the model and all six are now bench-verified — j1 pan
and j5 roll were confirmed at the bench 2026-07-25 (see JOINT_MAPS).

Table: the arm is bolted to the bench, so the world plane z=0 IS the
table. Base<->table contact is expected (mounted) and ignored; so is
Fixed_Jaw<->Moving_Jaw (the gripper may close on itself harmlessly).

Poses within CONTACT_MARGIN_M of touching count as contact: the mapping
carries real uncertainty, so the gate refuses near-misses too. The one
exception is link pairs that nest structurally in the folded rest pose
(`sim.twin check` prints them and their tolerance) — see NEST_TOL_DEG.

CLI (all take --cal FILE; exercise/derive-clearance take --span PCT):
    uv run python -m sim.twin check             # rest pose contact-free?
    uv run python -m sim.twin exercise          # gate the exercise routine
    uv run python -m sim.twin derive-clearance  # scan span/elbow/wrist
                                                # holds for a clear m2 sweep
    uv run python -m sim.twin validate          # must predict the two
                                                # known bench collisions

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

from hardware.bench.calibrate import JointCal, load_calibration
from hardware.errors import BenchError
from hardware.units import RAD_PER_TICK, fmt_ticks, span_deg

MODEL_XML = (Path(__file__).parent
             / "assets/menagerie/trs_so_arm100/so_arm100.xml")
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
INTERP_STEP_DEG = 2.0  # trajectory sampling; finer = slower, less tunneling
CLAMP_REPORT_DEG = 0.05  # ~half a tick: below this, clamping is rounding

# THE FOLDED SLUMP IS NOT ADJUDICABLE, and that is measured, not assumed.
#
# In the fold the jaw sits hard against the shoulder and the geometry
# converts joint error into penetration depth at roughly 0.7 mm per degree
# (measured on this model). Meanwhile the arm's torque-off slump does not
# reproduce itself: the slump captured as `rest` and the slump observed
# 2026-07-25 differ by 2.4 deg at the wrist — about 1.7 mm of jaw travel,
# an order of magnitude more than the clearance being judged. The model
# owns exactly ONE rest keyframe, so "rest" is inherently ambiguous by
# more than the thing we are trying to measure.
#
# Therefore: inside this region around the calibrated rest pose, link
# pairs that nest structurally are NOT adjudicated. Attempts to paper
# over it with a tolerance were tried and rejected on measurement —
# derived from a neighbourhood it grows explosively (1.4 mm at 2 deg,
# 5.8 at 4, 10 at 6) and by 4 deg it excuses table strikes; derived from
# the model's own keyframe it comes out 0.000 mm and changes nothing,
# because the disagreement is not noise around the keyframe, it IS the
# keyframe being a different slump than today's.
#
# What this deliberately does NOT relax, anywhere, ever:
#   - the table. The arm is not built to rest on it, so a table contact
#     is always a finding, in the rest region or out of it.
#   - pairs that do not nest at rest. Those fail on proximity as always.
#   - anything outside the region. Normal gating resumes immediately.
# The layer that actually covers the arm here is not the twin: it is
# `check_start_pose` reading the encoders, and the in-motion guards.
# 3.0 covers the 2.4 deg observed with margin, and is a small fraction
# of the 300 ticks (26 deg) the pre-flight already admits as "at rest".
REST_REGION_DEG = 3.0

# Contact pairs that are expected and never gate-failures. (The table
# plane hangs off the worldbody; _body_of_geom reports it as "table".)
ALLOWED_PAIRS = {
    frozenset({"Base", "table"}),          # bolted to the bench
    frozenset({"Fixed_Jaw", "Moving_Jaw"}),  # gripper closing on itself
}


@dataclass(frozen=True)
class JointMap:
    """How one calibrated joint lands on a model joint.

    anchor: which calibration.json tick ('rest' or 'min') corresponds to
    this joint's value in the model's `rest` keyframe — the anchor qpos
    is READ FROM THE MODEL, never duplicated here, so a vendored-model
    update can't silently desync the map. direction: +1 when rising
    physical ticks mean rising model qpos. bench_verified False =
    provisional; `sim.twin check` prints how to confirm or flip it."""

    model_joint: str
    anchor: str
    direction: int
    bench_verified: bool


# Directions from FK probes on the vendored model (2026-07-25):
# +Rotation = CCW from above; +Pitch = rise out of the rest fold;
# +Elbow = fold; +Wrist_Pitch = gripper tips up; +Wrist_Roll = CCW
# about the jaw's local +Y; +Jaw = open. Physical directions from the
# captured calibration signs (calibrate.py JOINT_POSITIVE).
# j1 and j5 were bench-verified 2026-07-25 (Kyle jogged each and reported
# the sense; `jog`'s +/- keys step TICKS, independent of the display
# frame). Both answers cross-check against the direction captured months
# earlier by `calibrate capture`, from a different vantage point:
#   j1: capture recorded sign -1 against "CCW viewed from above", so
#       increasing ticks = CW from above. Model +Rotation is CCW from
#       above => direction -1. Already correct; confirmed, not changed.
#   j5: capture recorded sign -1 against "CCW head-on", so increasing
#       ticks = CW head-on. Kyle observed CW with the gripper pointing
#       at him. FK probe: direction +1 reproduces that (analytically via
#       omega . point_direction, and empirically from jaw displacement);
#       -1 gives the mirror image. FLIPPED from -1.
JOINT_MAPS: dict[int, JointMap] = {
    1: JointMap("Rotation", "rest", -1, True),
    2: JointMap("Pitch", "rest", +1, True),
    3: JointMap("Elbow", "rest", +1, True),
    4: JointMap("Wrist_Pitch", "rest", +1, True),
    5: JointMap("Wrist_Roll", "rest", +1, True),
    6: JointMap("Jaw", "min", +1, True),           # physical closed = model closed
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
                f"waived inside {REST_REGION_DEG} deg of rest — the fold is "
                f"not resolvable (table contacts are never waived)")
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
        self._rest_qpos = self.model.key("rest").qpos.copy()
        # link pairs that nest structurally in the fold (never the table)
        self._structural: set[frozenset] = set()
        self._adr: dict[int, int] = {}
        self._range: dict[int, tuple[float, float]] = {}
        for i, jm in JOINT_MAPS.items():
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, jm.model_joint)
            if jid < 0:
                raise BenchError(f"model joint {jm.model_joint} not found",
                                 "the vendored model changed — fix JOINT_MAPS")
            self._adr[i] = self.model.jnt_qposadr[jid]
            self._range[i] = tuple(self.model.jnt_range[jid])
        # Link pairs touching in the model's folded rest pose are
        # structural — they are built to sit against each other there.
        # The table is excluded by construction: the arm is not built to
        # rest on it, so a table contact is always a finding.
        self.data.qpos[:] = self._rest_qpos
        mujoco.mj_forward(self.model, self.data)
        for n in range(self.data.ncon):
            con = self.data.contact[n]
            pair = frozenset({self._body_of_geom(con.geom1),
                              self._body_of_geom(con.geom2)})
            if "table" not in pair:
                self._structural.add(pair)

    def in_rest_region(self, pose: dict[int, int]) -> bool:
        """Is this pose inside the un-adjudicable fold? (see
        REST_REGION_DEG — EVERY joint must be within it.)"""
        return all(span_deg(abs(t - self.cals[i].rest)) <= REST_REGION_DEG
                   for i, t in pose.items())

    def _anchor_tick(self, i: int) -> int:
        cal = self.cals[i]
        return cal.rest if JOINT_MAPS[i].anchor == "rest" else cal.min

    def qpos_of(self, i: int, tick: int) -> tuple[float, float]:
        """(qpos clamped to the model range, clamp magnitude in deg).

        Clamping is UNCONSERVATIVE for a collision gate — a clamped pose
        is less extreme than reality, so a contact at the physical range
        end could go unpredicted. That is why every clamp is reported
        and printed even on a clean gate."""
        jm = JOINT_MAPS[i]
        anchor_qpos = self._rest_qpos[self._adr[i]]
        q = (anchor_qpos
             + jm.direction * RAD_PER_TICK * (tick - self._anchor_tick(i)))
        lo, hi = self._range[i]
        clamped = min(hi, max(lo, q))
        return clamped, abs(q - clamped) * 180.0 / math.pi

    def contacts_at(self, pose: dict[int, int], step: int = 0,
                    ) -> tuple[list[PredictedContact], dict[int, float], int]:
        """(contacts, clamps, structural contacts excused in the fold)."""
        self.data.qpos[:] = self._rest_qpos
        clamps: dict[int, float] = {}
        for i, tick in pose.items():
            q, clamp_deg = self.qpos_of(i, tick)
            self.data.qpos[self._adr[i]] = q
            if clamp_deg > CLAMP_REPORT_DEG:
                clamps[i] = clamp_deg
        mujoco.mj_forward(self.model, self.data)
        near_rest = self.in_rest_region(pose)
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
                # Penetration does — except inside the fold, which the
                # twin cannot resolve (REST_REGION_DEG). Counted, so the
                # gate can say out loud that it looked away.
                if near_rest:
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

    def check_trajectory(self, waypoints: list[dict[int, int]],
                         step_deg: float = INTERP_STEP_DEG) -> GateReport:
        """Interpolate between consecutive waypoints in tick space and
        collision-check every intermediate pose. Contacts are deduped by
        body pair (first occurrence reported)."""
        if not waypoints:
            return GateReport(0, [], {}, 0)
        contacts: list[PredictedContact] = []
        seen_pairs: set[frozenset] = set()
        clamps: dict[int, float] = {}
        checked = 0
        excused = 0

        def check(pose: dict[int, int], step: int) -> None:
            nonlocal checked, excused
            found, cl, ex = self.contacts_at(pose, step)
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
    print(f"\nstructural nesting pairs (not adjudicated within "
          f"{REST_REGION_DEG} deg of rest):")
    if not twin._structural:
        print("  none — no link pairs touch at the rest pose")
    for pair in sorted(twin._structural, key=sorted):
        print(f"  {' <-> '.join(sorted(pair))}")
    print("  the table is NEVER in this set — a table contact always fails")
    if excused:
        print(f"  {excused} contact(s) waived at the rest pose itself")
    _print_provisional()
    return 1 if found else 0


def cmd_exercise(twin: Twin, span: int) -> int:
    report = twin.check_trajectory(exercise_waypoints(twin.cals, span))
    print(report.summary(twin.cals))
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


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m sim.twin",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=(
        "check", "exercise", "derive-clearance", "validate"))
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
        return cmd_validate(twin, args.span)
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
