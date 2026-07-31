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

WHAT "REACH" MEANS HERE, because the word has several answers and the
one that matters depends on two choices this module makes explicitly.
A datasheet number is a tool point at full stretch in any orientation —
the arm is not holding anything there and the jaws are not pointing at
the table. What this reports instead is the ring where the GRIP POINT
can be put down over the table, in a graspable band, gate-clear:

  WHICH POINT   the model's `gripperframe` site, at the closed jaw tip.
  WHICH WRIST   gripper plumb by default; `--step` aside, `show` also
                reports the bound with j4 free, because the plumb
                requirement costs real reach and that trade belongs to
                whoever knows the part.

Run `show` for the numbers; they are deliberately not repeated here,
because a constant transcribed into prose is this project's most-logged
defect and a docstring cannot be made to fail a test.

WHICH CONSTRAINT SETS THE INNER LIMIT IS NOT FIXED, and `show` reports
whichever the data supports rather than telling a story. It has now
moved TWICE — once when the reference point moved from the jaw's body
origin to the tip, and again when the grasp band was corrected to mean
height above the table rather than above the m1 anchor. Both times a
draft of this paragraph named the current answer, and both times the
answer changed underneath it. So it no longer names one: run `show`,
which measures it. This text is printed by `--help`, so a constant
transcribed here is a wrong number handed to an operator who asked the
tool what it does.

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

FOUR WAYS THIS MODULE HAS MEASURED THE WRONG POINT, kept because every
one of them produced a confident wrong number and they are all cheap to
repeat. The pattern underneath: a point was inherited from whichever
object was nearest to hand instead of being chosen and named.

  * `gripper` is a JOINT NAME and a BODY NAME in this model, and they
    name different links. The chain is

        lower_arm -> wrist -> gripper -> moving_jaw_so101_v1

    so body `gripper` is the ROLL link (driven by j5, wrist_roll), body
    `wrist` is its parent (driven by j4, wrist_flex), and the jaw is
    named after its mesh. Resolving the joint name as a body name
    silently returns the wrong link — it put the measured reach
    direction 2.9 deg out before `twin.py` named the three explicitly.

    Note what `twin.py`'s `WRIST_BODY = "gripper"` is: a variable name,
    not a claim about anatomy. An earlier draft of THIS bullet read it
    as one and stated that body `gripper` "is the WRIST, not the jaw",
    which is wrong in a way that matters — at the selftest's folded
    golden pose, body `gripper` is 51.87 mm into the shoulder and body
    `wrist` is 8.22 mm, a factor of six riding on which link a reader
    thinks the name means.
  * `geom_rbound` is a BOUNDING-SPHERE radius, so `z - rbound` is a
    lower bound on a geom's lowest point and not the point itself — on
    these jaw meshes it under-read by about 6.6 mm, which silently
    shifted the whole grasp band upward. An oriented bounding box
    replaced it and has since been deleted too: the band is now judged
    at a SINGLE POINT, the grip site, with no bounding volume at all.
    Say that plainly, because it is a real limit and not a detail —
    the gripper housing is wider and lower than the site, so it can be
    touching the table while the grip point still reads 5 mm up. That
    is why a table strike among the inner blockers is legitimate rather
    than a bug. What `selftest` checks is that the site stays ON the
    jaw; nothing here accounts for the jaw's extent.
  * `TOOL_BODY` is the moving jaw's BODY ORIGIN, which sits at the base
    of an 85 mm gripper. Kyle spotted it by eye — "the gripper has a
    length!" — before any number said so. The grip point is the model's
    `gripperframe` site, at the closed tip.
  * "the furthest point of the hand from the j1 axis" IS that tip when
    the arm is extended, and is a corner of the gripper housing when the
    gripper hangs plumb. That draft returned an empty sweep and the tool
    refused. Furthest-from-the-axis is a property of the POSE; the tip
    is a property of the GRIPPER, and only a frame fixed to the gripper
    tracks it through a rotation.

Real jaw fittings will move the grip point again, and that is a ticket
of its own (Kyle 2026-07-31: "need 3d models etc etc"). When it lands,
the site is the anchor it should hang off.

    uv run python -m sim.reach show          # the zone, both frames
    uv run python -m sim.reach at 24.0 70.0  # can a fixture live here?
    uv run python -m sim.reach selftest
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

import mujoco
import numpy as np

from hardware.errors import BenchError

from .bench_scene import load_cell
from .rig import Rig
from .twin import Twin

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


GRIP_SITE = "gripperframe"


def _grip_mm(data, model, origin) -> tuple[float, float, float]:
    """Where the closed jaws meet the work — (radius, height, azimuth).

    TWO FRAMES, AND MIXING THEM COST A DAY. Radius and azimuth are about
    the m1 AXIS, so they are measured from `origin` — a point on that
    axis. Height is above the TABLE, and world z=0 IS the table because
    the arm is bolted to the bench. `origin` is m1's ANCHOR, which sits
    ~62 mm up the column, so subtracting it from z does not convert
    between two frames that share a zero — it silently reports every
    grasp 62 mm lower than it is. The first version did exactly that,
    and the band test then admitted poses with the jaw tip well below
    the tabletop. It surfaced only because the twin insisted a fallen
    arm was 100 mm THROUGH the table, which is not a thing a bolted arm
    can be. One line of this function is in the rig frame and one is in
    the world frame; that is the whole point and it is why the z term
    does not touch `origin`.

    Kyle 2026-07-31: *"most of the time the grip point will be on the
    very tip of the jaw."* The gripper runs 85 mm past its own joint,
    and this module used to report the moving jaw's BODY ORIGIN, which
    sits at the base of that 85 mm and understated every reach figure by
    about 77 mm.

    The point is taken from the model's OWN `gripperframe` site, which
    the vendored file already places within 6.8 mm of the closed jaw
    tip. FIRST ATTEMPT WAS WRONG AND THE FAILURE IS WORTH KEEPING: it
    took the hand's furthest point from the j1 axis, which IS the tip
    when the arm is extended and is a side corner of the gripper housing
    when the gripper hangs plumb — so the whole sweep returned nothing
    and the tool refused. "Furthest from the axis" is a property of the
    pose; "the tip" is a property of the gripper, and only a frame fixed
    to the gripper tracks it through a rotation.

    Real jaw fittings will move this point, and that is a ticket of its
    own (Kyle: "need 3d models etc etc"). Until then the site is the
    honest answer and it is DERIVED, not a constant typed in here.
    """
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, GRIP_SITE)
    if sid < 0:
        raise BenchError(
            f"the model has no `{GRIP_SITE}` site, so the grip point "
            f"cannot be located",
            "the vendored model changed; find the frame that replaced "
            "it rather than falling back to a body origin")
    p = data.site_xpos[sid]
    dx, dy = (p[0] - origin[0]) * 1000.0, (p[1] - origin[1]) * 1000.0
    return (math.hypot(dx, dy), float(p[2]) * 1000.0,
            math.degrees(math.atan2(dy, dx)))


@dataclass(frozen=True)
class Annulus:
    """The arm-only graspable ring, in the rig frame. j1-symmetric."""

    r_in_mm: float
    r_out_mm: float
    r_in_kinematic_mm: float      # the same bound with the gate switched off
    slew_min_deg: float           # j1's calibrated travel, frame degrees
    slew_max_deg: float
    # The jaw sits off the j1 axis, so its azimuth is NOT the slew. The
    # offset is a smooth function of radius, and the SPREAD is what
    # matters: a single mean was the first design and it made points
    # past j1's travel read as reachable at both ends of the arc, which
    # no constant could have fixed. Carried as (radius, offset) samples
    # and interpolated. The actual numbers are not written down here —
    # two successive drafts quoted values ("+2 to -4, it CHANGES SIGN")
    # that the next reference-point change falsified, and `show` prints
    # the measured pair every run.
    az_offset_by_r: tuple
    # Body pairs that blocked poses REACHING FURTHER IN than `r_in_mm`,
    # i.e. the ones whose removal set that bound — not a tally of
    # everything the gate ever refused. See `annulus`.
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
    # TWO FRAMES IN ONE RECORD, deliberately, and it must be read that
    # way: `r_mm` and `az_rig_deg` are about the m1 axis in the RIG
    # frame, while `jaw_mm` is height above the TABLE (world z). They
    # differ by the 62 mm the m1 anchor stands above the bench. Anything
    # that reassembles a 3D point from this — `ik.solve` takes rig-frame
    # mm and adds `rig._origin` — must not take `jaw_mm` for the rig z.
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
            gate: bool = True, plumb: bool = True,
            tilt_step: float = 5.0) -> list[Sample]:
    """Samples at one slew.

    `plumb` pins j4 by the crane relation so the gripper hangs straight
    down. That is the SAFE grasp — jaws closing horizontally on a part
    standing on the table — and it was the module's original hidden
    assumption. It is also expensive: it costs about 20% of the arm's
    reach, because getting the jaws down AND vertical at full stretch
    needs the wrist folded back over the work.

    With `plumb=False` j4 sweeps its whole travel, so the jaws may
    arrive tilted. The part is then gripped at an angle, which is a real
    grasp for a box-shaped blank and not one for something that must be
    picked square. Which of the two bounds applies is a decision about
    the gripper and the part, so both are reported rather than one being
    chosen quietly.
    """
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
    # `_hand_geoms` is still called for its REFUSAL — it raises when the
    # vendored model has no jaw geoms, and this is the cheapest place to
    # find that out. Its result is no longer threaded into `_sample_one`:
    # that was a leftover of the deleted bounding-box height test, and
    # the parameter sat unread for two revisions.
    _hand_geoms(twin.model)
    out: list[Sample] = []
    lo4, hi4 = span(4)
    for a in np.arange(lo2, hi2 + step, step):
        for b in np.arange(lo3, hi3 + step, step):
            wrists = ([180.0 - a - b] if plumb
                      else list(np.arange(lo4, hi4 + tilt_step, tilt_step)))
            for c in wrists:
                _sample_one(twin, rig, band, gate, tick, cals,
                            t1, a, b, c, out)
    return out


def _sample_one(twin, rig, band, gate, tick, cals, t1, a, b, c, out):
    """One (j2, j3, j4) triple, appended to `out` if it lands in band."""
    t2, t3, t4 = tick(2, a), tick(3, b), tick(4, c)
    if not (cals[2].min <= t2 <= cals[2].max
            and cals[3].min <= t3 <= cals[3].max
            and cals[4].min <= t4 <= cals[4].max):
        return
    pose = {1: t1, 2: t2, 3: t3, 4: t4}
    q = twin._rest_qpos.copy()
    for i, t in pose.items():
        q[twin._adr[i]] = twin.qpos_of(i, t)[0]
    rig.data.qpos[:] = q
    mujoco.mj_forward(rig.model, rig.data)
    # `tip_z` is height above the TABLE and the band is in the same
    # units — see `_grip_mm`, where that is the one term not measured
    # from the rig origin, and where getting it wrong cost 62 mm.
    r, tip_z, az = _grip_mm(rig.data, rig.model, rig._origin)
    if not band[0] <= tip_z <= band[1]:
        return
    blocked = ()
    if gate:
        found, _clamps, _excused = twin.contacts_at(pose)
        blocked = tuple(found)
    out.append(Sample(r_mm=r, jaw_mm=tip_z, az_rig_deg=az,
                      blocked=blocked, ticks=dict(pose)))


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
    r_in = min(s.r_mm for s in clear)
    # ONLY the poses that would have reached FURTHER IN than the bound —
    # those are the ones whose removal actually set it, and the name says
    # so. A whole-sweep tally answers a different question and gets read
    # as this one: printed under "inner bound set by SELF-COLLISION" it
    # listed a table strike out at the ring's far edge as the reason the
    # arm cannot fold tight. That mislabelling survived because the sweep
    # was running 62 mm too high to graze the table at all (see
    # `_grip_mm`), so the tally happened to contain only arm-vs-arm pairs
    # and the check reading it stayed green on an accident.
    blockers: dict = {}
    for s in gated:
        if s.r_mm >= r_in:
            continue
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
        r_in_mm=r_in,
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


def zone_tiles(cell, twin: Twin | None = None, rig: Rig | None = None,
               step_deg: float = 1.0,
               sweep_step: float = DEFAULT_STEP_DEG) -> tuple:
    """The reach zone as drawable tiles, in bench coordinates.

    One thin box per angular slice, each spanning the ring radially and
    already CLIPPED to where the tabletop actually extends — so what
    gets drawn is the usable zone, not the theoretical one. A slice with
    no table under it produces no tile rather than a zero-width one.

    Returns (centre x, centre y, yaw deg, half-radial mm, half-tangential
    mm) per tile. Plain numbers on purpose: `bench_scene` draws these and
    must not have to import the twin to do it.
    """
    twin = twin or Twin()
    rig = rig or Rig(twin)
    ann = annulus(twin, rig, step=sweep_step)
    place = placement(cell, twin)

    # BOTH rings, because drawing only the plumb one understates the arm
    # badly enough that Kyle called it wrong by eye twice. The plumb ring
    # is the guaranteed vertical-approach zone; the tilt ring is
    # everywhere the jaws can reach the band at all, and the gap between
    # them is what the plumb requirement costs.
    tilted = [s for s in profile(twin, rig,
                                 (ann.slew_min_deg + ann.slew_max_deg) / 2.0,
                                 step=sweep_step, plumb=False)
              if not s.blocked]
    rings = [("plumb", ann.r_in_mm, ann.r_out_mm)]
    if tilted:
        t_in, t_out = min(s.r_mm for s in tilted), max(s.r_mm for s in tilted)
        if t_out > ann.r_out_mm + 1.0:
            rings.append(("tilt", ann.r_out_mm, t_out))
        if t_in < ann.r_in_mm - 1.0:
            rings.append(("tilt", t_in, ann.r_in_mm))

    tiles = []
    n = max(1, int(round(ann.arc_deg / step_deg)))
    width = ann.arc_deg / n
    for k in range(n):
        slew = ann.slew_min_deg + (k + 0.5) * width
        az = slew + ann.az_offset_at(ann.r_out_mm)
        for kind, lo, hi in rings:
            outer = min(hi, table_limit_mm(place, az, hi))
            if outer <= lo + 0.5:
                continue                   # nothing usable along this ray
            mid = (lo + outer) / 2.0
            cx, cy = bench_of(place, mid, az)
            tiles.append((
                cx, cy, az + place.rot_deg, (outer - lo) / 2.0,
                # Half the chord this slice subtends at its mid radius,
                # plus a hair of overlap so tiles do not show seams.
                mid * math.tan(math.radians(width / 2.0)) * 1.08, kind))
    return tuple(tiles)


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
    # Reported from the measurement rather than narrated, and BOTH
    # branches are live: this offset has been constant across the ring
    # under one choice of reference point and has varied by several
    # degrees under another, so which sentence is true is a fact about
    # today's model and not something to hard-code.
    spread = ann.az_offset_spread_deg
    print(f"  grip azimuth  the grip point sits off the j1 axis, so it "
          f"leads the slew by\n                {lo_off:+.2f} deg at the "
          f"inner edge and {hi_off:+.2f} deg at the outer"
          + (f" — it varies by\n                {spread:.2f} deg across the "
             f"ring, so it is interpolated by radius"
             if spread > 0.05 else
             f" — constant to\n                {spread:.3f} deg, because "
             f"the point is rigid to the gripper"))
    # WHICH constraint binds is measured, not assumed. It moved once
    # already: against the jaw's body origin the gate raised the inner
    # bound from 79 to 110 mm and self-collision was the limit, but
    # against the grip point the arm runs out of geometry before it can
    # fold into itself, and the two bounds coincide. Printing a fixed
    # story here would have gone on naming the wrong cause.
    if ann.r_in_mm > ann.r_in_kinematic_mm + 1.0:
        print(f"  inner bound   set by SELF-COLLISION: the arm folds into "
              f"itself before it\n                runs out of geometry — "
              f"kinematics alone would reach "
              f"{ann.r_in_kinematic_mm:.0f} mm")
        for k, v in sorted(ann.inner_blockers.items(),
                           key=lambda kv: -kv[1])[:3]:
            print(f"                  {v:5d} poses blocked by {k}")
    else:
        print(f"  inner bound   set by GEOMETRY, not self-collision: with "
              f"the grip point held\n                in the band the arm "
              f"cannot fold tight enough to hit itself first")
        # No blocker list here on purpose. `inner_blockers` only counts
        # poses that would have reached INSIDE `r_in_mm`, and a non-empty
        # tally therefore implies the gate raised that bound — which is
        # the other branch. Printing it here was unreachable in all but a
        # 1 mm window, and an unreachable print reads as coverage.

    # What the plumb assumption COSTS, measured rather than assumed. It
    # was a hidden constraint in the first version and Kyle spotted the
    # outer bound as too small by eye before any number said so.
    tilted = [s for s in profile(twin, rig, (ann.slew_min_deg
                                             + ann.slew_max_deg) / 2.0,
                                 step=step, plumb=False)
              if not s.blocked]
    if tilted:
        t_out = max(s.r_mm for s in tilted)
        print(f"\nIF THE GRIPPER MAY TILT (j4 free rather than pinned plumb)")
        print(f"  radius        {min(s.r_mm for s in tilted):.0f} .. "
              f"{t_out:.0f} mm — {t_out - ann.r_out_mm:+.0f} mm on the outer "
              f"edge")
        print(f"  the plumb constraint is what costs the difference: the "
              f"jaws reach\n                further when they are allowed "
              f"to arrive at an angle. Whether that\n                counts "
              f"as a grasp is a decision about the part, not about the arm.")

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
    band_lo, band_hi = GRASP_BAND_MM
    cals = twin.cals
    # BEFORE the golden poses, and this line is load-bearing. `_ticks`
    # dereferences `cals[i].frame`, and the goldens were moved above the
    # first `profile()` call so the gate's verdict would be
    # unconditional — which also moved them above the only thing that
    # had been validating the frames. A half-finished `calibrate
    # capture` then met the suite with a bare AttributeError traceback
    # instead of the BenchError that names the joint and how to fix it.
    # That was this file's round-1 finding, reintroduced by the round-2
    # fix that removed it. Idempotent; `profile()` calls it again.
    _require_frames(twin)

    # THE GATE ITSELF, probed with fixed poses rather than through the
    # sweep, and FIRST so its verdict is unconditional — every later
    # check either consumes the sweep or can be skipped by an early
    # return, and the gate's health is what those all rest on.
    #
    # Why it needs its own probe: everything else here that mentions the
    # gate compares `gated` against `ungated`, but those two `profile()`
    # calls admit the IDENTICAL pose set (the flag only fills in
    # `.blocked`), so "the gate never loosens the inner bound" is a
    # subset theorem that cannot fail. Stub `contacts_at` to return
    # nothing and the whole suite went green while shipping an inner
    # radius of 56 mm instead of 85 — 29 mm optimistic, into the volume
    # where the arm folds through itself.
    #
    # IN DEGREES, NOT TICKS, and an earlier draft asserted the reverse
    # ("ticks, so a frame edit cannot move the poses out from under the
    # assertion"). Backwards: `qpos_of` runs the tick through
    # `cal.frame`, so a fixed TICK moves with a re-ratified frame while
    # a fixed DEGREE cancels through it. Measured on a +170-tick shift
    # of j2's zero: fixed ticks moved the pose 14.94 deg, fixed degrees
    # moved it 0.00. Ticks would have turned an ordinary
    # `calibrate capture` into a false red on the gate.
    # GOLDEN_CLEAR is the EXTENDED pose, and that is worth knowing
    # rather than a coincidence to trip over: on 2026-07-31 the arm
    # STRAIN-FAULTED reaching it and fell flat, yet it is genuinely
    # collision-clear and belongs here. The gate answers "does this
    # pose intersect anything", not "can the arm hold it" — j2 needs
    # 0.864 N.m there against 0.325 at TUCK. Nothing in this module
    # models that, which is plan 716.6.
    GOLDEN_FOLDED = {1: 7.2, 2: 29.4, 3: 144.1, 4: 74.2}
    GOLDEN_CLEAR = {1: 0.0, 2: 90.0, 3: 0.0, 4: 0.0}

    def _ticks(deg):
        return {i: cals[i].frame.tick(v) for i, v in deg.items()}

    hit, _c, _e = twin.contacts_at(_ticks(GOLDEN_FOLDED))
    deepest = max((x.depth_mm for x in hit), default=0.0)
    pairs = sorted({f"{x.body_a} <-> {x.body_b}" for x in hit})
    # 40 mm because the real figure is ~52 and it degrades gracefully
    # under frame drift; without a depth floor the probe can decay into
    # a barely-grazing pose that still satisfies `bool(hit)` while
    # testing almost nothing. The deep pair is shoulder <-> GRIPPER —
    # an earlier draft called it the wrist, which is a different body in
    # this model (`wrist` is the gripper's parent) and is only 8 mm in.
    check("the gate REFUSES a pose with the gripper buried in the "
          "shoulder", bool(hit) and deepest > 40.0
          and any("table" not in p for p in pairs),
          f"{deepest:.0f} mm deep: " + ", ".join(pairs[:2]) if hit
          else "the gate reported it CLEAR")
    clear_hit, _c, _e = twin.contacts_at(_ticks(GOLDEN_CLEAR))
    check("...and PASSES the arm extended into free air, so it is not "
          "simply refusing everything",
          not clear_hit,
          "no contacts" if not clear_hit else
          f"{len(clear_hit)} spurious contacts")
    # NOTHING BETWEEN HERE AND THE SWEEP may read `rig.data` without
    # reposing first: `rig.data IS twin.data`, so the model is currently
    # parked at GOLDEN_CLEAR. The sweep is safe because `_sample_one`
    # and `repose` both overwrite the whole qpos vector — that safety is
    # accidental, not designed, and `repose`'s docstring is the scar
    # from the last time it was assumed.
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
    # A slew with NO gate-clear pose is a real finding, and the bare
    # `min()` below used to meet it with a ValueError traceback — the
    # worst possible signal, because it fires on exactly the broken
    # states this suite exists to diagnose and reports them as a crash
    # rather than a red line.
    empty = [s for s, v in runs.items() if not v]
    check("every sampled slew has at least one gate-clear pose in the band",
          not empty,
          "5 slews across j1's travel" if not empty else
          f"no reachable pose at slew(s) {', '.join(f'{s:+.0f}' for s in empty)}")
    if empty:
        print("\nreach selftest FAILED — the radius profile cannot be "
              "measured without a pose at every slew")
        return 1
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
    # Deliberately does NOT assert which constraint wins. It changed the
    # day the grip point moved from the jaw's body origin to the tip, and
    # an assertion naming a particular winner just goes stale and gets
    # edited to match. What must hold is that the gate never LOOSENS a
    # bound, and that whichever cause is reported is the one the data
    # supports.
    gate_binds = ann.r_in_mm > ann.r_in_kinematic_mm + 1.0
    # DERIVED, and labelled so. Both this and the check below read
    # `gated` against `ungated`, which admit the same pose set — so this
    # one is a subset theorem that cannot fail, and the next one
    # short-circuits on a quantity computed from it. When the gate dies
    # they BOTH print green ("geometry binds", "no arm pair") directly
    # beneath the golden probe's failure, and someone triaging that sees
    # one failure and two reassurances that are artifacts of it. They
    # are kept for their DETAIL text, which is genuinely informative,
    # not for their verdicts.
    check("(derived) the gate never loosens the inner bound",
          ann.r_in_mm >= ann.r_in_kinematic_mm - 0.001,
          f"gated {ann.r_in_mm:.0f} mm vs ungated "
          f"{ann.r_in_kinematic_mm:.0f} mm — "
          + ("self-collision binds" if gate_binds else "geometry binds"))
    # A table strike among these is LEGITIMATE and the check no longer
    # forbids it. Folded tight over its own base the gripper housing is
    # wider than the site, so it can touch down while the grip point is
    # still 5 mm up — a real constraint, and the old condition banned
    # any table pair outright.
    #
    # AN EARLIER DRAFT OF THIS COMMENT GOT THE HISTORY WRONG and it is
    # corrected rather than deleted, because the wrong version is the
    # more tempting story. It said the old check "stayed green on an
    # accident of tally composition" — that the pre-fix tally happened
    # to hold only arm-vs-arm pairs. Measured, the pre-fix tally was
    # EMPTY and `gate_binds` was False, so the check passed through its
    # `(not gate_binds)` short-circuit and never read the tally at all.
    # Loosening a check the day it starts failing is how a real defect
    # gets silenced, so the reason had better be the true one: what
    # makes this safe is not this check, it is that the band check above
    # now measures height directly and catches a mis-framed sweep on the
    # evidence rather than through a proxy.
    arm_pairs = {k: v for k, v in ann.inner_blockers.items()
                 if "table" not in k}
    tbl = sum(v for k, v in ann.inner_blockers.items() if "table" in k)
    check("(derived) ...and if self-collision is what binds, a real arm "
          "pair is named for it",
          (not gate_binds) or bool(arm_pairs),
          f"{sum(arm_pairs.values())} poses arm-vs-arm, {tbl} table — "
          + (", ".join(sorted(arm_pairs)[:2]) or "no arm pair"))
    # "the ring is an annulus, not a disc" used to live here, asserting
    # `r_in > 1` and `band > 1`. The characterization pin below entails
    # both — if it is green, r_in is within 1 mm of 84.7 and the band is
    # ~191 mm — so it could not fail in any state where the pin passes,
    # three lines away. This module's own rule applies: a passing test
    # on dead machinery reads as coverage. Its numbers are in the pin's
    # detail string instead.
    #
    # CHARACTERIZATION, and the only written-down number in this module.
    # It is here because the golden poses above cannot cover the case
    # that matters most. Both of them sit far outside the sampled volume
    # — the folded one at r 46 mm, the clear one at r 441 mm, against a
    # ring of 85..276 — so they pin the gate at two extremes while it
    # can be arbitrarily wrong in between. `r_in` is not set by deep
    # penetrations; it is set by MARGINAL, grazing contacts, and an
    # extreme probe is blind to that regime by construction. Measured:
    # adding four pairs to ALLOWED_PAIRS ships r_in at 59 mm — within
    # 3 mm of the totally-dead-gate number — with both goldens green. A
    # 5 mm depth filter ships 75 mm, likewise green. Those are far more
    # likely defects than the gate dying outright.
    #
    # This module's rule is that PROSE must not carry a constant,
    # because a docstring cannot be made to fail. That is an argument
    # FOR putting the number somewhere that can. If this goes red,
    # exactly one of two things happened, and they are told apart by
    # whether calibration.json moved:
    #   calibration changed  -> re-measure with `show --step 4`, and
    #                           update these two numbers deliberately
    #   calibration did NOT  -> the gate or the geometry changed, which
    #                           is the defect this exists to catch
    RING_AT_STEP_4 = (84.7, 275.6)
    got = (ann.r_in_mm, ann.r_out_mm)
    drift = max(abs(a - b) for a, b in zip(got, RING_AT_STEP_4))
    check("the ring is where it was when this was last measured",
          drift < 1.0,
          f"{got[0]:.1f}..{got[1]:.1f} mm vs recorded "
          f"{RING_AT_STEP_4[0]}..{RING_AT_STEP_4[1]} "
          f"(worst drift {drift:.2f} mm)")

    # The jaw-height bound is an OUTER bound on a mesh. Measure what is
    # left of it against real transformed vertices rather than asserting
    # it away — and fail if the box has drifted far from the mesh.
    # THE ASSUMPTION THAT NOW CARRIES THE ANSWER: `gripperframe` is at
    # the closed jaw tip. Everything this module reports is that site's
    # position, so if the site drifts from the jaw the whole zone is
    # wrong and nothing else would notice.
    #
    # This replaced a check on the oriented-bounding-box jaw height. That
    # check was valid, and it went stale the moment the band test started
    # reading the site instead of the box: it was still green while
    # testing a function the product no longer consults. A passing test
    # on dead machinery is worse than no test, because it reads as
    # coverage.
    hand = _hand_geoms(twin.model)
    non_mesh = [g for g in hand if twin.model.geom_dataid[g] < 0]
    check("every hand geom is a mesh, so the site can be compared against "
          "real vertices", not non_mesh,
          f"{len(non_mesh)} primitive geom(s)" if non_mesh
          else f"{len(hand)} meshes")
    sid = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_SITE, GRIP_SITE)
    worst_gap, worst_at = 0.0, None
    for s in (lo1 / 2, 0.0, hi1 / 2):
        for smp in profile(twin, rig, s, step=step):
            repose(twin, rig, smp.ticks)
            site = rig.data.site_xpos[sid]
            near = math.inf
            for g in hand:
                mid = twin.model.geom_dataid[g]
                adr = twin.model.mesh_vertadr[mid]
                n = twin.model.mesh_vertnum[mid]
                v = twin.model.mesh_vert[adr:adr + n].reshape(-1, 3)
                rot = rig.data.geom_xmat[g].reshape(3, 3)
                w = (v @ rot.T) + rig.data.geom_xpos[g]
                near = min(near, float(np.linalg.norm(w - site,
                                                      axis=1).min()) * 1000.0)
            if near > worst_gap:
                worst_gap, worst_at = near, smp
    check(f"the `{GRIP_SITE}` site stays ON the jaw through the whole "
          f"sweep", worst_gap < 12.0,
          f"furthest it ever sits from a jaw vertex: {worst_gap:.2f} mm"
          + (f" (at r {worst_at.r_mm:.0f} mm)" if worst_at else ""))

    # THE FRAME CONTRACT, checked against the world rather than against
    # `_grip_mm`'s own arithmetic. The band means "above the TABLE" and
    # the table is world z=0, so every sample the sweep admits must have
    # its grip site inside the band measured straight off `site_xpos` —
    # no origin subtraction anywhere in this check, because subtracting
    # the origin is exactly what was wrong. `_grip_mm` used to return
    # height above the m1 ANCHOR, 62 mm up the column, so the sweep
    # silently profiled the arm at 67-82 mm instead of 5-20 and every
    # radius shipped on 2026-07-31 was the ring at the wrong height.
    # Nothing failed: the old checks all compared the module against
    # itself, and both sides moved together.
    # RANKED BY HOW FAR OUTSIDE THE BAND, and the first draft of this
    # ranked by `abs(z - smp.jaw_mm)` instead — the disagreement between
    # the world reading and the reported one. After the fix those two
    # are BITWISE equal, so the seed of 0.0 was never beaten and the
    # offender was never recorded: the check was green with poses 25 mm
    # THROUGH the tabletop. It still went red on the reintroduced origin
    # bug, which is how it passed for a working test — it was a one-line
    # regression test wearing the label of a frame contract. Two
    # reviewers found it independently and that is the only reason it is
    # not in the commit.
    worst_z, worst_r = 0.0, None
    for s in (lo1 / 2, 0.0, hi1 / 2):
        for smp in profile(twin, rig, s, step=step):
            repose(twin, rig, smp.ticks)
            z = float(rig.data.site_xpos[sid][2]) * 1000.0
            dev = max(band_lo - z, z - band_hi)
            if dev > 0.001 and dev > worst_z:
                worst_z, worst_r = dev, (smp.r_mm, z)
    check("every admitted pose really is in the grasp band above the "
          "TABLE, measured in world z",
          worst_r is None,
          f"band {band_lo:g}-{band_hi:g} mm"
          if worst_r is None else
          f"worst offender r {worst_r[0]:.0f} mm sits at z {worst_r[1]:.1f} mm, "
          f"{worst_z:.1f} mm outside the band")

    # ...and the arm is bolted to that table, so a height above it is
    # never negative. A separate claim from the one above: the band
    # could be honoured and the frame still be flipped.
    # TWO CLAIMS, and the second is the one with teeth. That the m1
    # anchor is above the table only says the two frames are not
    # interchangeable; it cannot tell a right answer from a 60 mm wrong
    # one. What the band actually depends on is that the arm is mounted
    # AT z=0 — put the model on a riser and every height in this file is
    # wrong by the riser, with nothing else noticing.
    origin_z = float(rig._origin[2]) * 1000.0
    # Reads `twin.data` without reposing, which the golden-pose comment
    # above warns against — EXEMPT, and here is why, so nobody copies
    # the pattern somewhere it does not hold: `base` is the root body,
    # fixed to the worldbody, so its xpos is a model constant and no
    # qpos can move it. Verified invariant across the rest pose and both
    # goldens. Any body further down the chain would need `repose`.
    base_z = float(twin.data.body("base").xpos[2]) * 1000.0
    check("the rig origin is genuinely ABOVE the table, so the two "
          "frames are not interchangeable",
          origin_z > 1.0,
          f"m1's anchor sits {origin_z:.1f} mm up — the exact error the "
          f"check above exists to catch")
    check("...and the arm is mounted AT the table, which is what makes "
          "world z a height above it",
          abs(base_z) < 0.001,
          f"base body origin at z {base_z:+.4f} mm")

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
