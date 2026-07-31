"""The functional reach zone — where this arm can actually grasp.

Plan #716.5. Kyle 2026-07-31: *"the plane is not infinite, you have the
bench specs, and you have the arm specs. there is an arc of a thin
annular region that the arm can reach you should be able to compute this
'functional reach zone' and project it into the mujoco and it should
provide bounding info for whatever mock cell we setup."*

Everything this needs was already measured. `calibration.json` has every
joint's travel, the twin carries the meshes, `bench.json` has the
table's real extents and `cell.json` says where the arm stands on it.
Nobody had multiplied them together.

WHAT "REACH" MEANS HERE, because the word has three answers and only one
of them is useful for siting a fixture. A datasheet number is the tool
point at full stretch in any orientation — the arm is not holding
anything there and the jaws are not pointing at the table. What this
module reports instead is the ring where the JAWS can be put down on the
table, gripper plumb, in a graspable band, with the pose collision-free.
Run `show` for the numbers; they are deliberately not repeated here,
because a constant transcribed into prose is this project's most-logged
defect and a docstring cannot be made to fail a test.

The inner limit is worth expecting: it is not kinematic. The arm cannot
fold tight enough to reach near its own base without `shoulder` and the
jaw colliding, so the inner bound is set by the arm's own body.

THE SYMMETRY THAT MAKES IT CHEAP, and its exact limit. After the slew,
the arm is a rigid subchain rotating about a vertical axis, so the
reachable (radius, height) PROFILE is independent of j1 and one sweep of
j2/j3 gives the whole annulus. `selftest` asserts that AND asserts the
same measurement varies where it must, so the check is not vacuous.

But the symmetry belongs to THE ARM AND AN INFINITE PLANE, not to the
bench. `rig.py` spells out the same fact from the other direction: the
only world geom is a horizontal infinite plane, so rotating the arm
about the vertical maps the contact set exactly onto itself — which is
why the collision gate could not catch the mirrored j1 mapping for four
days. The moment the table becomes BOUNDED that stops being true. So:

    the ANNULUS   is computed against the arm alone, and is j1-symmetric
    the TABLE CLIP is applied geometrically afterwards, and is not

TWO TRAPS THIS MODULE FELL INTO ON ITS FIRST DRAFT, both caught by
review, both recorded because they are cheap to repeat:

  * `gripper` is a JOINT NAME and a BODY NAME in this model, and the
    body it names is the WRIST, not the jaw. `twin.py` says so in a
    comment that exists because the same mistake once put the measured
    reach direction 2.9 deg out. Use `TOOL_BODY`.
  * `geom_rbound` is a BOUNDING-SPHERE radius, so `z - rbound` is a
    lower bound on a geom's lowest point and not the point itself — on
    these jaw meshes it under-reads by about 6.6 mm, which silently
    shifted the whole grasp band upward. The oriented bounding box is
    used instead, and `selftest` measures what is left against the real
    mesh vertices rather than asserting it is zero.

    uv run python -m sim.reach show          # the zone, both frames
    uv run python -m sim.reach at 24.0 70.0  # can a fixture live here?
    uv run python -m sim.reach selftest
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from dataclasses import dataclass

import mujoco
import numpy as np

from hardware.errors import BenchError

from .bench_scene import load_cell
from .rig import Rig
from .twin import TOOL_BODY, Twin

# The band a jaw must sit in to count as able to grasp something resting
# on the table. Not zero: a jaw ON the table is a table strike, and a
# part has height. 5-20 mm spans the 20 mm mock blank from just above the
# surface to its top face.
GRASP_BAND_MM = (5.0, 20.0)

# Sweep resolution in degrees for j2/j3. `--step` exists so the effect of
# this number can be measured rather than assumed; `show` prints the
# radius it produced and the step it used, so two runs can be compared.
DEFAULT_STEP_DEG = 2.0

# Joints whose calibrated frame this module actually reads. j6 (the
# gripper) is excluded deliberately: it never leaves its minimum here and
# is commanded in percent, not degrees.
REQUIRED_FRAMES = (1, 2, 3, 4, 5)


def bench_to_mm(delta: float, to_m: float) -> float:
    """Bench units -> mm. `Scene.to_m` multiplies bench units INTO metres,
    so these two functions are the only place that direction is written
    down; inverting it is a silent factor of 645."""
    return delta * to_m * 1000.0


def mm_to_bench(mm: float, to_m: float) -> float:
    return mm / 1000.0 / to_m


def wrap180(deg: float) -> float:
    """Fold an angle into (-180, 180].

    Not decoration. `atan2` returns (-180, 180], and the rig->bench
    rotation here is 180 deg, so subtracting it lands honest angles in
    (-360, 0] — where a slew of +3.5 reads as -356.5 and every bound
    check on j1's travel fails for a reachable point. The rebuilt
    selftest caught exactly that; the first version could not have,
    because it generated its test angles with the same unwrapped
    arithmetic it was checking.
    """
    return (deg + 180.0) % 360.0 - 180.0


def _require_frames(twin: Twin) -> None:
    for i in REQUIRED_FRAMES:
        cal = twin.cals.get(i)
        if cal is None:
            raise BenchError(
                f"joint {i} is not in calibration.json, and this needs its "
                f"travel",
                f"run `calibrate capture --ids {i}`")
        if cal.frame is None:
            raise BenchError(
                f"joint {i} ({cal.name}) has no ratified frame, so its "
                f"travel cannot be read in degrees",
                "ratify the frame in calibration.json, or re-run "
                f"`calibrate capture --ids {i}`")


def _hand_geoms(model) -> list[int]:
    """Geoms belonging to the hand, so how low the gripper reaches is
    measured off the model rather than from the 82 mm rule of thumb."""
    out = []
    for g in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                  model.geom_bodyid[g]) or ""
        if any(k in (gname + bname).lower() for k in ("jaw", "gripper")):
            out.append(g)
    if not out:
        raise BenchError(
            "no gripper or jaw geoms found in the twin",
            "the vendored model changed; grasp height cannot be judged "
            "without knowing which geoms are the hand")
    return out


def _lowest_point_mm(data, model, geoms: list[int]) -> float:
    """Lowest world z over some geoms, in mm, from their ORIENTED
    bounding boxes.

    Not `rbound`: that is the bounding-SPHERE radius, so `z - rbound`
    under-reads a flat jaw by most of its length. `geom_aabb` is the
    local axis-aligned box (centre + half-extents); rotating its eight
    corners by the geom's frame gives a box that actually tracks the
    orientation, which matters because the jaws rotate through the whole
    sweep. Still an outer bound on a mesh — `selftest` measures the
    residual against the real vertices instead of pretending it is zero.
    """
    lowest = math.inf
    for g in geoms:
        cx, cy, cz, hx, hy, hz = model.geom_aabb[g]
        rot = data.geom_xmat[g].reshape(3, 3)
        pos = data.geom_xpos[g]
        for sx, sy, sz in itertools.product((-1, 1), repeat=3):
            local = np.array([cx + sx * hx, cy + sy * hy, cz + sz * hz])
            lowest = min(lowest, float(pos[2] + (rot @ local)[2]))
    return lowest * 1000.0


@dataclass(frozen=True)
class Annulus:
    """The arm-only graspable ring, in the rig frame. j1-symmetric."""

    r_in_mm: float
    r_out_mm: float
    r_in_kinematic_mm: float      # the same bound with the gate switched off
    slew_min_deg: float           # j1's calibrated travel, frame degrees
    slew_max_deg: float
    # The jaw sits off the j1 axis, so its azimuth is NOT the slew. The
    # offset is a smooth function of radius — measured, it runs from
    # about +2 deg at the inner edge to about -4 deg at the outer, i.e.
    # it CHANGES SIGN across the ring. A single mean was the first
    # design and it was wrong in the dangerous direction: it made points
    # past j1's travel read as reachable at BOTH ends of the arc, on
    # opposite sides, so no constant could have fixed it. Carried as
    # (radius, offset) samples and interpolated.
    az_offset_by_r: tuple
    inner_blockers: dict
    poses: int
    step_deg: float

    @property
    def az_offset_spread_deg(self) -> float:
        offs = [o for _r, o in self.az_offset_by_r]
        return max(offs) - min(offs)

    def az_offset_at(self, r_mm: float) -> float:
        """Tool azimuth minus slew, at this radius.

        `az_rig(slew) = az_offset(r) + slew` holds to machine precision,
        so inverting it recovers the true slew exactly — provided the
        offset is taken at the right radius. np.interp clamps outside
        the sampled range, which is deliberate: past the ring the answer
        is 'out of the ring' anyway, and extrapolating a curve beyond
        its evidence is how the mean version produced false accepts.
        """
        rs = [r for r, _o in self.az_offset_by_r]
        offs = [o for _r, o in self.az_offset_by_r]
        return float(np.interp(r_mm, rs, offs))

    @property
    def arc_deg(self) -> float:
        return abs(self.slew_max_deg - self.slew_min_deg)

    @property
    def band_mm(self) -> float:
        return self.r_out_mm - self.r_in_mm


@dataclass(frozen=True)
class Sample:
    r_mm: float
    jaw_mm: float
    az_rig_deg: float             # tool azimuth in the rig frame
    blocked: tuple                # contacts, empty when gate-clear
    ticks: dict                   # the pose that produced it, so a caller
    #                               can re-pose the model exactly rather
    #                               than reading whatever FK left behind


def profile(twin: Twin, rig: Rig, slew_deg: float,
            band: tuple[float, float] = GRASP_BAND_MM,
            step: float = DEFAULT_STEP_DEG,
            gate: bool = True) -> list[Sample]:
    """Samples at one slew, j4 pinned by the crane relation so the
    gripper hangs plumb — the only orientation that can pick a part off
    a flat table."""
    if not step > 0:
        raise BenchError(
            f"--step must be a positive number of degrees, got {step:g}",
            "2 is the default; smaller is slower and slightly tighter")
    _require_frames(twin)
    cals = twin.cals

    def tick(i, deg):
        return cals[i].frame.tick(float(deg))

    def span(i):
        lo, hi = twin.frame_x(i, cals[i].min), twin.frame_x(i, cals[i].max)
        return (lo, hi) if lo <= hi else (hi, lo)

    t1 = tick(1, slew_deg)
    if not cals[1].min <= t1 <= cals[1].max:
        lo, hi = span(1)
        raise BenchError(
            f"slew {slew_deg:+.1f} deg is outside j1's calibrated travel",
            f"j1 reaches {lo:+.1f}..{hi:+.1f} deg")

    lo2, hi2 = span(2)
    lo3, hi3 = span(3)
    hand = _hand_geoms(twin.model)
    out: list[Sample] = []
    for a in np.arange(lo2, hi2 + step, step):
        for b in np.arange(lo3, hi3 + step, step):
            c = 180.0 - a - b                     # gripper plumb
            t2, t3, t4 = tick(2, a), tick(3, b), tick(4, c)
            if not (cals[2].min <= t2 <= cals[2].max
                    and cals[3].min <= t3 <= cals[3].max
                    and cals[4].min <= t4 <= cals[4].max):
                continue
            pose = {1: t1, 2: t2, 3: t3, 4: t4}
            q = twin._rest_qpos.copy()
            for i, t in pose.items():
                q[twin._adr[i]] = twin.qpos_of(i, t)[0]
            rig.data.qpos[:] = q
            mujoco.mj_forward(rig.model, rig.data)
            # World z=0 IS the table (the arm is bolted to the bench), so
            # a hand geom's world z is already its height above the
            # surface. Adding the rig origin here would double-count m1.
            jaw = _lowest_point_mm(rig.data, rig.model, hand)
            if not band[0] <= jaw <= band[1]:
                continue
            # TOOL_BODY, not "gripper": that name resolves to the WRIST
            # body in this model. See the module docstring.
            tool = (rig.data.body(TOOL_BODY).xpos - rig._origin) * 1000.0
            blocked = ()
            if gate:
                found, _clamps, _excused = twin.contacts_at(pose)
                blocked = tuple(found)
            out.append(Sample(
                r_mm=math.hypot(tool[0], tool[1]), jaw_mm=jaw,
                az_rig_deg=math.degrees(math.atan2(tool[1], tool[0])),
                blocked=blocked, ticks=dict(pose)))
    return out


def repose(twin: Twin, rig: Rig, ticks: dict) -> None:
    """Put the model back at a sample's pose.

    Exists because reading `rig.data` after a sweep reads whatever the
    LAST iteration left there — which is how a selftest ended up
    measuring jaw clearance at a pose 100 mm outside the grasp band and
    reporting it as the band's worst case.
    """
    q = twin._rest_qpos.copy()
    for i, t in ticks.items():
        q[twin._adr[i]] = twin.qpos_of(i, t)[0]
    rig.data.qpos[:] = q
    mujoco.mj_forward(rig.model, rig.data)


def annulus(twin: Twin, rig: Rig, band: tuple[float, float] = GRASP_BAND_MM,
            step: float = DEFAULT_STEP_DEG) -> Annulus:
    """The arm-only graspable ring. Computed at ONE slew, because the
    profile is j1-symmetric — `selftest` is what holds that claim up."""
    _require_frames(twin)
    cals = twin.cals
    lo1, hi1 = twin.frame_x(1, cals[1].min), twin.frame_x(1, cals[1].max)
    lo1, hi1 = min(lo1, hi1), max(lo1, hi1)
    mid = (lo1 + hi1) / 2.0
    gated = profile(twin, rig, mid, band, step, gate=True)
    clear = [s for s in gated if not s.blocked]
    if not clear:
        raise BenchError(
            "no gate-clear pose puts the jaws in the grasp band "
            f"({band[0]:g}-{band[1]:g} mm above the table)",
            "if this is unexpected, the arm cannot reach its own table "
            "with the gripper plumb, which is a finding rather than a "
            "setting — check calibration.json's j2/j3/j4 travel")
    ungated = profile(twin, rig, mid, band, step, gate=False)
    blockers: dict = {}
    for s in gated:
        for x in s.blocked:
            key = f"{x.body_a} <-> {x.body_b}"
            blockers[key] = blockers.get(key, 0) + 1
    # (radius, azimuth offset) sampled across the ring, deduplicated and
    # sorted so it can be interpolated. Several poses land on the same
    # radius; they agree to about 0.001 deg, so averaging the duplicates
    # is honest rather than picking one arbitrarily.
    by_r: dict = {}
    for s in clear:
        by_r.setdefault(round(s.r_mm, 3), []).append(s.az_rig_deg - mid)
    offsets = tuple(sorted((r, sum(v) / len(v)) for r, v in by_r.items()))
    return Annulus(
        r_in_mm=min(s.r_mm for s in clear),
        r_out_mm=max(s.r_mm for s in clear),
        r_in_kinematic_mm=min((s.r_mm for s in ungated), default=math.nan),
        slew_min_deg=lo1, slew_max_deg=hi1,
        az_offset_by_r=offsets,
        inner_blockers=blockers, poses=len(clear), step_deg=step)


# ------------------------------------------------------------------ bench


@dataclass(frozen=True)
class Placement:
    """Where the arm and its ring sit on the bench, in bench units."""

    m1_x: float
    m1_y: float
    rot_deg: float               # rig frame -> bench frame rotation
    to_m: float
    units: str
    surfaces: tuple


def placement(cell, twin: Twin) -> Placement:
    """The rig->bench transform, taken the way simcam takes it.

    NOT `cell.arm.yaw_deg` on its own. That field is the direction the
    arm REACHES at pan zero, and the arm does not reach along its own
    +X — so the rotation that carries rig coordinates onto the bench is
    `arm_yaw - Twin.reach_yaw_deg()`. That is the composition
    `sim/simcam.py` uses and least-squares-verifies at construction;
    using the raw yaw instead rotates every answer by several degrees.
    """
    pose = cell.arm_pose
    if pose is None:
        raise BenchError(
            "the arm's position on the bench is not measured",
            "fill in arm.x / arm.y / arm.yaw_deg in cell.json")
    if not cell.bench.surfaces:
        raise BenchError(
            "bench.json defines no table surfaces, so nothing can be "
            "clipped to a tabletop",
            "add the bench's surfaces before asking where a fixture fits")
    x, y, yaw = pose
    return Placement(
        m1_x=x, m1_y=y, rot_deg=yaw - twin.reach_yaw_deg(),
        to_m=cell.bench.to_m, units=cell.bench.units,
        surfaces=tuple((s.name, *s.corners()) for s in cell.bench.surfaces))


def bench_of(place: Placement, r_mm: float, az_rig_deg: float
             ) -> tuple[float, float]:
    az = math.radians(az_rig_deg + place.rot_deg)
    return (place.m1_x + math.cos(az) * mm_to_bench(r_mm, place.to_m),
            place.m1_y + math.sin(az) * mm_to_bench(r_mm, place.to_m))


def rig_of(place: Placement, bx: float, by: float) -> tuple[float, float]:
    """Bench point -> (radius mm, rig azimuth deg) about the m1 axis."""
    dx = bench_to_mm(bx - place.m1_x, place.to_m)
    dy = bench_to_mm(by - place.m1_y, place.to_m)
    return (math.hypot(dx, dy),
            wrap180(math.degrees(math.atan2(dy, dx)) - place.rot_deg))


def on_table(place: Placement, bx: float, by: float) -> str | None:
    for name, x0, y0, x1, y1 in place.surfaces:
        if x0 <= bx <= x1 and y0 <= by <= y1:
            return name
    return None


def table_limit_mm(place: Placement, az_rig_deg: float, r_out_mm: float,
                   probe_mm: float = 1.0) -> float:
    """How far out along one rig azimuth a table surface actually
    extends, walking outward and stopping where the surface ends.

    Deliberately the CONTIGUOUS extent from the arm outward, not "any
    point that happens to be over a surface": an L-shaped bench can put
    a second surface beyond a gap, and a fixture cannot float across it.
    Returns 0.0 if the ray is off the table from the start.
    """
    last, r = 0.0, 0.0
    while r <= r_out_mm:
        if on_table(place, *bench_of(place, r, az_rig_deg)) is None:
            return last
        last, r = r, r + probe_mm
    return r_out_mm


@dataclass(frozen=True)
class Verdict:
    ok: bool
    reason: str
    hint: str
    radius_mm: float
    slew_deg: float
    surface: str | None


def can_grasp(ann: Annulus, place: Placement, bx: float, by: float
              ) -> Verdict:
    """Can the arm put its jaws down at this bench point?

    Distinct rejections, because a bare "unreachable" does not tell an
    operator which way to move the fixture.

    ORDER MATTERS, and it is RADIUS FIRST. The slew is recovered using
    an azimuth offset measured across the ring, so outside the ring that
    offset is extrapolated and the slew it yields is meaningless — the
    first version reported a fixture 60 mm inside the inner limit as
    "outside j1's travel, swing it round", which is advice that cannot
    help: it is inside the self-collision bound at EVERY slew. Radius is
    a fact about the point alone; check it before anything derived.

    The TABLE is checked before the outer radius on purpose too: where
    the tabletop ends before the arm's reach does, the useful number is
    the distance to the edge, not the kinematic shortfall.
    """
    r, az_rig = rig_of(place, bx, by)
    slew = wrap180(az_rig - ann.az_offset_at(r))
    if r < ann.r_in_mm:
        return Verdict(False,
                       f"{ann.r_in_mm - r:.0f} mm INSIDE the inner limit "
                       f"({ann.r_in_mm:.0f} mm)",
                       "the arm cannot fold that tight without hitting "
                       "itself at any slew — move the fixture away from "
                       "the base", r, slew, on_table(place, bx, by))
    if not ann.slew_min_deg <= slew <= ann.slew_max_deg:
        return Verdict(False,
                       f"slew {slew:+.1f} deg is OUTSIDE j1's travel "
                       f"({ann.slew_min_deg:+.1f}..{ann.slew_max_deg:+.1f})",
                       "swing the fixture around toward the arm's front, "
                       "or turn the arm on its mount",
                       r, slew, None)
    surface = on_table(place, bx, by)
    if surface is None:
        return Verdict(False, "the arm reaches this far but there is NO "
                       "TABLE here",
                       "nothing can sit at this point — move it to where "
                       "the bench actually extends", r, slew, None)
    cap = min(ann.r_out_mm, table_limit_mm(place, az_rig, ann.r_out_mm))
    if r > cap:
        edge = "the table ends" if cap < ann.r_out_mm - 0.5 else "the arm ends"
        return Verdict(False,
                       f"{r - cap:.0f} mm BEYOND the limit ({cap:.0f} mm — "
                       f"{edge} there)",
                       f"move the fixture {r - cap:.0f} mm toward the arm",
                       r, slew, surface)
    return Verdict(True, f"reachable on '{surface}'",
                   f"r {r:.0f} mm, slew {slew:+.1f} deg", r, slew, surface)


# -------------------------------------------------------------- commands


def cmd_show(twin: Twin, rig: Rig, cell, step: float) -> int:
    ann = annulus(twin, rig, step=step)
    place = placement(cell, twin)
    print("functional reach zone — derived from calibration.json and the "
          "twin\nat run time, never from a stored constant.\n")
    print("ARM ONLY (gripper plumb, jaw %.0f-%.0f mm above the table, "
          "gate-clear)" % GRASP_BAND_MM)
    print(f"  radius        {ann.r_in_mm:.0f} .. {ann.r_out_mm:.0f} mm from "
          f"the m1 axis   (band {ann.band_mm:.0f} mm wide)")
    print(f"  arc           {ann.arc_deg:.1f} deg  "
          f"(j1 {ann.slew_min_deg:+.1f} .. {ann.slew_max_deg:+.1f})")
    print(f"  resting on    {ann.poses} gate-clear poses at "
          f"{ann.step_deg:g} deg steps")
    lo_off = ann.az_offset_at(ann.r_in_mm)
    hi_off = ann.az_offset_at(ann.r_out_mm)
    print(f"  tool azimuth  the jaw sits off the j1 axis, so it leads the "
          f"slew by\n                {lo_off:+.2f} deg at the inner edge and "
          f"{hi_off:+.2f} deg at the outer —\n                it CHANGES "
          f"SIGN, so no single offset works; interpolated by radius")
    if ann.inner_blockers:
        print(f"  inner bound   set by SELF-COLLISION, not kinematics: "
              f"kinematics alone reaches {ann.r_in_kinematic_mm:.0f} mm")
        for k, v in sorted(ann.inner_blockers.items(),
                           key=lambda kv: -kv[1])[:3]:
            print(f"                  {v:5d} poses blocked by {k}")

    print("\nON THE BENCH")
    print(f"  m1 axis at    x {place.m1_x:.3f}, y {place.m1_y:.3f} "
          f"{place.units}")
    # Printed to 6 places on purpose. This lands on 179.999976, not 180,
    # and the difference is the 4-decimal rounding of cell.json's stored
    # yaw_deg. It is NOT independent evidence that the frame is right:
    # yaw_deg was WRITTEN as base_square + reach_yaw, so subtracting
    # reach_yaw back out recovers base_square by construction and would
    # for any value. What actually holds the frame up is
    # bench_scene._selftest_arm_base_square, which measures the built
    # model's base rather than reading a stored constant back.
    print(f"  rig -> bench  rotate {place.rot_deg:+.6f} deg (arm yaw minus "
          f"the model's own reach\n                offset — an identity, "
          f"not a check; the check is bench_scene\n                "
          f"--selftest's arm_base_square)")
    for name, x0, y0, x1, y1 in place.surfaces:
        print(f"  surface       {name:<8} x {x0:g}..{x1:g}  y {y0:g}..{y1:g}")

    print("\nCLIPPED BY THE TABLE — reachable, but nothing can sit there")
    lost, worst, sample = 0, 0.0, []
    for k in range(int(ann.arc_deg) + 1):
        slew = ann.slew_min_deg + k
        # The jaw's path at this slew is not a single azimuth — the
        # offset varies with radius — so the ray is walked at the offset
        # belonging to the OUTER edge, which is where a clip can happen.
        cap = table_limit_mm(place, slew + ann.az_offset_at(ann.r_out_mm),
                             ann.r_out_mm)
        if cap < ann.r_out_mm - 0.5:
            lost += 1
            worst = max(worst, ann.r_out_mm - cap)
            sample.append((slew, cap))
    if lost:
        print(f"  {lost} deg of the {ann.arc_deg:.0f} deg arc is clipped; "
              f"deepest cut {worst:.0f} mm off the outer edge")
        for slew, cap in sample[:6]:
            print(f"    slew {slew:+7.1f} deg -> table ends at "
                  f"{cap:.0f} mm (of {ann.r_out_mm:.0f})")
    else:
        print("  nothing clipped: the whole ring is over a surface")

    print("\nSITING A FIXTURE")
    print(f"  ask directly:  uv run python -m sim.reach at <x> <y>   "
          f"({place.units}, bench frame)")
    return 0


def cmd_at(twin: Twin, rig: Rig, cell, step: float, bx: float,
           by: float) -> int:
    ann = annulus(twin, rig, step=step)
    place = placement(cell, twin)
    v = can_grasp(ann, place, bx, by)
    print(f"bench point x {bx:g}, y {by:g} {place.units}")
    print(f"  r {v.radius_mm:.0f} mm from the m1 axis, "
          f"slew {v.slew_deg:+.1f} deg")
    print(f"  {'REACHABLE' if v.ok else 'NOT REACHABLE'}: {v.reason}")
    print(f"  {v.hint}")
    return 0 if v.ok else 1


def cmd_selftest(twin: Twin, rig: Rig, cell) -> int:
    fails = []

    def check(name, ok, detail=""):
        print(f"  [{'ok ' if ok else 'FAIL'}] {name}"
              + (f" ({detail})" if detail else ""))
        if not ok:
            fails.append(name)

    step = 4.0
    cals = twin.cals
    lo1, hi1 = twin.frame_x(1, cals[1].min), twin.frame_x(1, cals[1].max)
    lo1, hi1 = min(lo1, hi1), max(lo1, hi1)
    slews = [lo1 + 1, lo1 / 2, 0.0, hi1 / 2, hi1 - 1]

    # THE claim the whole computation rests on — and the paired control
    # that makes it mean something. Both measurements come out of the
    # SAME profile() call, so the control cannot pass by testing some
    # other function: the radius must be invariant while the tool's
    # rig-frame azimuth, taken from the same samples, must move.
    runs = {s: [x for x in profile(twin, rig, s, step=step) if not x.blocked]
            for s in slews}
    r_in = [min(x.r_mm for x in v) for v in runs.values()]
    r_out = [max(x.r_mm for x in v) for v in runs.values()]
    az = [sum(x.az_rig_deg for x in v) / len(v) for v in runs.values()]
    check("the arm-only radius profile is j1-independent",
          max(r_in) - min(r_in) < 0.5 and max(r_out) - min(r_out) < 0.5,
          f"inner spread {max(r_in) - min(r_in):.3f} mm, outer "
          f"{max(r_out) - min(r_out):.3f} mm over {len(slews)} slews")
    check("...and the SAME samples do move with j1, so that is not a "
          "measurement of nothing",
          max(az) - min(az) > 100.0,
          f"tool azimuth spans {min(az):+.1f}..{max(az):+.1f} deg")

    ann = annulus(twin, rig, step=step)
    check("the inner bound is raised by the gate, not by kinematics",
          ann.r_in_mm > ann.r_in_kinematic_mm + 1.0,
          f"gated {ann.r_in_mm:.0f} mm vs ungated "
          f"{ann.r_in_kinematic_mm:.0f} mm")
    check("...and the poses it removed were blocked by the ARM, not the "
          "table",
          bool(ann.inner_blockers)
          and not any("table" in k for k in ann.inner_blockers),
          ", ".join(sorted(ann.inner_blockers)[:2]))
    check("the ring is an annulus, not a disc",
          ann.r_in_mm > 1.0 and ann.band_mm > 1.0,
          f"{ann.r_in_mm:.0f}..{ann.r_out_mm:.0f} mm")

    # The jaw-height bound is an OUTER bound on a mesh. Measure what is
    # left of it against real transformed vertices rather than asserting
    # it away — and fail if the box has drifted far from the mesh.
    hand = _hand_geoms(twin.model)
    non_mesh = [g for g in hand if twin.model.geom_dataid[g] < 0]
    check("every hand geom is a mesh, so the box-vs-mesh check compares "
          "like with like", not non_mesh,
          f"{len(non_mesh)} primitive geom(s)" if non_mesh
          else f"{len(hand)} meshes")
    worst, worst_at = 0.0, None
    for s in (lo1 / 2, 0.0, hi1 / 2):
        # Every IN-BAND pose, re-posed explicitly. Reading rig.data after
        # profile() returns reads its last sweep iteration, which is not
        # in the band at all.
        for smp in profile(twin, rig, s, step=step):
            repose(twin, rig, smp.ticks)
            box = _lowest_point_mm(rig.data, rig.model, hand)
            true_low = math.inf
            for g in hand:
                mid = twin.model.geom_dataid[g]
                if mid < 0:
                    continue
                adr = twin.model.mesh_vertadr[mid]
                n = twin.model.mesh_vertnum[mid]
                v = twin.model.mesh_vert[adr:adr + n].reshape(-1, 3)
                rot = rig.data.geom_xmat[g].reshape(3, 3)
                zs = (v @ rot.T)[:, 2] + rig.data.geom_xpos[g][2]
                true_low = min(true_low, float(zs.min()) * 1000.0)
            if math.isfinite(true_low) and true_low - box > worst:
                worst, worst_at = true_low - box, smp
    check("the oriented box never claims the jaw is LOWER than the mesh "
          "really is", worst >= 0.0, "one-sided by construction")
    check("...and the gap it leaves is small enough not to move the band",
          worst < 3.0,
          f"worst {worst:.2f} mm"
          + (f" at jaw {worst_at.jaw_mm:.1f} mm" if worst_at else ""))

    # Acceptance from the ARM, not from the inverse of can_grasp: take a
    # pose the sweep actually produced, convert ITS tool position to a
    # bench point, and ask whether the tool agrees it is reachable.
    place = placement(cell, twin)
    real = [x for x in profile(twin, rig, 0.0, step=step) if not x.blocked]
    real.sort(key=lambda x: x.r_mm)
    mid_sample = real[len(real) // 2]

    # The CORNERS, not the middle. A sample taken dead centre of the band
    # and the arc passes for almost any azimuth model — swept over every
    # whole-degree offset, a mid-ring check stayed green for 205 of 360,
    # which is why it let a radius-dependent error through. The inner and
    # outer edges at both ends of j1's travel are where the offset error
    # actually bites, so that is where this asks.
    worst_slew_err = 0.0
    for slew in (lo1 + 1.0, 0.0, hi1 - 1.0):
        got = [x for x in profile(twin, rig, slew, step=step)
               if not x.blocked]
        got.sort(key=lambda x: x.r_mm)
        for label, smp, nudge in (("inner", got[0], +0.5),
                                  ("outer", got[-1], -0.5)):
            # Nudged off the exact bound. `r_in`/`r_out` come from the
            # mid-slew sweep, so a sample AT the edge sits on the
            # comparison to within floating-point noise and the verdict
            # becomes a coin toss — a property of the test, not the arm.
            r = smp.r_mm + nudge
            v = can_grasp(ann, place, *bench_of(place, r, smp.az_rig_deg))
            err = abs(wrap180(v.slew_deg - slew))
            worst_slew_err = max(worst_slew_err, err)
            # Reachability is only owed where there IS a table: the outer
            # edge at the extreme slews is exactly the sliver that hangs
            # over the front edge, and refusing it there is the whole
            # point of the clip. Slew recovery is owed everywhere.
            expect_ok = on_table(place, *bench_of(
                place, r, smp.az_rig_deg)) is not None
            ok = err < 0.5 and (v.ok or not expect_ok)
            check(f"a real {label}-edge pose at slew {slew:+.0f} recovers "
                  f"its slew"
                  + (" and is REACHABLE" if expect_ok else " (off-table, "
                     "so refusal is correct)"), ok,
                  f"r {r:.0f} mm, slew {slew:+.1f} -> "
                  f"{v.slew_deg:+.1f} ({v.reason})")
    check("...so the azimuth model is good across the whole ring",
          worst_slew_err < 0.5,
          f"worst slew recovery error {worst_slew_err:.3f} deg")

    # Refusals, each from a real sample pushed just past one boundary.
    inner, outer = real[0], real[-1]
    for label, r, azr, want in (
            ("inside the inner limit", inner.r_mm - 20.0, inner.az_rig_deg,
             "INSIDE"),
            ("beyond every limit", outer.r_mm + 60.0, outer.az_rig_deg,
             "BEYOND"),
            ("outside j1's arc", mid_sample.r_mm,
             hi1 + ann.az_offset_at(mid_sample.r_mm) + 20.0, "OUTSIDE")):
        v = can_grasp(ann, place, *bench_of(place, r, azr))
        check(f"...and one {label} is REFUSED for that reason",
              not v.ok and want in v.reason, v.reason)

    # The clip Kyle asked for: inside the ring, inside the arc, no table.
    # Start a hair inside the arc: the clipped sliver sits AT j1's travel
    # limit, and probing the exact boundary makes the answer depend on
    # whether a float lands on -102.7000000 or -102.7000001, which is a
    # property of the test rather than of the bench.
    off = None
    for slew in np.arange(lo1 + 0.05, hi1 - 0.05, 0.25):
        for r in np.arange(ann.r_in_mm, ann.r_out_mm, 2.0):
            p = bench_of(place, float(r),
                         float(slew) + ann.az_offset_at(float(r)))
            if on_table(place, *p) is None:
                off = (float(r), float(slew), p)
                break
        if off:
            break
    if off:
        v = can_grasp(ann, place, *off[2])
        check("a point inside the ring with NO TABLE under it is refused "
              "for THAT reason",
              not v.ok and "NO TABLE" in v.reason,
              f"r {off[0]:.0f} mm, slew {off[1]:+.1f} deg -> {v.reason}")
    else:
        check("the whole ring is over a surface, so nothing to clip", True,
              "recorded rather than skipped")

    # Refusals on inputs, not geometry.
    for label, fn in (
            ("--step 0", lambda: profile(twin, rig, 0.0, step=0.0)),
            ("--step negative", lambda: profile(twin, rig, 0.0, step=-3.0)),
            ("a slew outside j1's travel",
             lambda: profile(twin, rig, hi1 + 10.0))):
        try:
            fn()
            check(f"{label} is refused", False, "it was accepted")
        except BenchError as exc:
            check(f"{label} is refused", bool(exc.hint), str(exc)[:52])

    print(f"\nreach selftest {'OK' if not fails else 'FAILED'}"
          + ("" if not fails else f" — {len(fails)}: " + "; ".join(fails)))
    return 0 if not fails else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m sim.reach",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("show", "at", "selftest"))
    parser.add_argument("x", nargs="?", type=float,
                        help="bench x of the point to test (`at` only)")
    parser.add_argument("y", nargs="?", type=float,
                        help="bench y of the point to test (`at` only)")
    parser.add_argument("--cal", default="calibration.json")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP_DEG,
                        help="j2/j3 sweep resolution in degrees")
    args = parser.parse_args()
    try:
        twin = Twin(cal_path=args.cal)
        rig = Rig(twin)
        cell = load_cell()
        if args.command == "show":
            return cmd_show(twin, rig, cell, args.step)
        if args.command == "selftest":
            return cmd_selftest(twin, rig, cell)
        if args.x is None or args.y is None:
            raise BenchError(
                "`at` needs a bench x and y",
                f"e.g. `at 24.0 70.0`, in {cell.bench.units}")
        return cmd_at(twin, rig, cell, args.step, args.x, args.y)
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
