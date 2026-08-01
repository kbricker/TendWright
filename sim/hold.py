"""What the arm can HOLD — the static torque envelope, under its own weight.

Plan #716.6. Kyle 2026-07-31, after the first hardware run put the arm
on the table: *"your goal right now is to understand the range of reach
the arm actually has with no load, we tried to max it and that was too
much for these poor little servo motors lol... knowing what the actual
range is unloaded creates the envelope we will work inside when placing
worksites within the work area."*

`reach.py` answers CAN THE ARM PUT ITS JAWS THERE. This answers CAN IT
STILL BE THERE A SECOND LATER. They are different questions and the
first hardware run is what proved it: `swing-tour` reached EXTENDED,
which `reach.py` calls perfectly clear, and j2 hit the strain guard's
80% peak and let go.

WHERE THE TORQUE COMES FROM. Nothing here is measured with a wrench.
The vendored model carries per-link masses — 632 g in total, of which
147 g is the bolted base, so 485 g MOVES and 385 g of that hangs
outboard of j2 — and MuJoCo's `qfrc_bias` at zero velocity IS the
gravity torque each joint must supply to stand still. Derived at call
time from the same model the gate uses, so it moves when the model
moves.

    `mj_inverse` GIVES THE SAME ANSWER TO MACHINE PRECISION. Two drafts
    of this comment said otherwise and both invented a mechanism, which
    is the failure worth recording, not the API detail.

    Draft 1: "a flat 3.350 N.m at every pose, friction loss dominates."
    Friction never came into it — `dof_frictionloss` is 0.052 and could
    not reach 3.35.

    Draft 2: "`mj_forward` overwrites qacc with the FREE-FALL
    acceleration of an UNACTUATED arm, and `mj_inverse` clamps to
    forcerange." Closer, and still wrong twice. The arm is fully
    actuated: the model carries position servos at kp=998.22, and
    `ctrl=0` commands qpos=0, so at a pose far from zero they SATURATE.
    Measured qacc at j2 in EXTENDED: -90.03 rad/s^2 with the servos
    live, against +16.59 for an actual gravity fall — opposite sign and
    six times larger. And the clamp is in `mj_fwdActuation` during
    `mj_forward`; `mj_inverse` does not clamp anything, it just recovers
    the already-saturated `qfrc_actuator`. Anyone chasing the clamp
    inside `mj_inverse` finds nothing, which is exactly the trap draft 1
    set.

    What is true: `mj_forward` overwrites `data.qacc`, so zeroing qacc
    BEFORE calling it accomplishes nothing. Zero it AFTER and
    `qfrc_inverse` equals `qfrc_bias` to 0.00e+00 at every pose tried.

TWO REGIMES ON THE PLUMB RING, which is the ring to site work against:

    close in   j3 binds, at a FLOOR of about 0.30 N.m. Flat from the
               inner limit out to ~163 mm.
    far out    j2 takes over and climbs roughly linearly with radius,
               because the whole arm becomes a lever about the shoulder.

So reach costs torque only in the outer half. Pulling a worksite in
from 280 mm to 240 mm buys real margin; pulling it from 160 to 120 buys
nothing at all.

WHAT THE FLOOR IS AND IS NOT. It is NOT the linkage: swept over the
whole j2/j3/j4 space the minimum |j3| gravity torque is 0.000 N.m, with
the outboard mass balanced straight over the elbow axis. The floor is
imposed by the PLUMB CONSTRAINT plus the 5-20 mm grasp band together —
holding the jaws vertical and near the table forbids the balanced
postures. An earlier draft said the forearm "hangs off the elbow no
matter how the arm is folded", which is false and would send someone
looking for a linkage fix that does not exist. Raising the band does
not help either: at 55-70 mm the floor RISES to 0.333, and above about
105 mm plumb has no solutions at all.

The tilt ring behaves differently — j2 binds throughout and the floor
is lower — so do not carry the two-regime story across to it.

WHAT IS NOT KNOWN, AND THE SIGN OF THE ERROR IS NOT KNOWN EITHER. The
servo's actual capability is NOT established. One observation exists —
2026-07-31, j2 reported the guard's 80% peak at a pose this module
computes as 0.864 N.m — and one point does not pin a line. Unknown: the
bus voltage (STS3215 stall is ~2.94 N.m at 12 V but ~1.91 at 7.4 V),
whether the load register is linear in torque at all, and how much
gearbox friction adds.

Effects are live in BOTH directions, which is the honest summary:

  * the guard fires at >= 80%, so the true reading may have been
    higher — less capability per percent, envelope OPTIMISTIC;
  * gearbox friction `f` makes low torques read high — OPTIMISTIC, and
    `d(tau_static)/df = 55/80 - 1 = -0.31`, so it can only push this
    way. An earlier draft counted friction on BOTH sides of the ledger,
    once as an offset (optimistic) and again inside the mid-motion
    bundle (conservative). Same physical term; it nets optimistic and
    belongs here only;
  * against those, the reading was taken MID-MOTION, lifting into the
    pose, so some of that 80% was not gravity. Attributing the whole
    reading to the static term — which is what this module does —
    understates the servo's static capability and pushes CONSERVATIVE.
    Sized honestly that is INERTIA ONLY: 0.080 of 0.944 N.m, about
    8.5%. An earlier draft said "as much as a third" by adding friction
    (wrong side, above) and 0.184 of viscous damping — and `dof_damping`
    here is 0.6, a solver-stability value, not a measured property of an
    STS3215. If the damping is real the conservative term reaches ~22%,
    but nothing establishes that it is.

So the net sign is NOT ESTABLISHED, and a draft that asserted the
envelope was unambiguously optimistic was overclaiming in the safe
direction while a draft that called it conservative overclaimed in the
dangerous one. The crossover sits inside the plausible range of `f`:
below about 0.18 N.m of internal friction this module is conservative,
above it optimistic, and the vendored 0.052 and a 10-20%-of-stall rule
of thumb fall on opposite sides of that. Measuring `f` is therefore the
single most useful thing 716.6's bench pass can do.

    uv run python -m sim.hold show        # torque vs radius, both rings
    uv run python -m sim.hold selftest
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import mujoco

from hardware.errors import BenchError

from .reach import DEFAULT_STEP_DEG, profile
from .rig import Rig
from .twin import Twin

# The joints that carry the arm's weight against gravity. j1 is vertical
# so gravity applies no torque about it; j5 rolls the gripper about its
# own axis; j6 is the jaw. Those three are excluded because their
# gravity term is structurally ~zero, not because they were forgotten.
CARRYING_JOINTS = (2, 3, 4)

# THE ONE REAL OBSERVATION, 2026-07-31. `swing-tour` edge 8 of 14,
# lifting into EXTENDED with Kyle watching on cameras: joint 2 reported
# the strain guard's LOAD_PEAK_PCT and torque was cut. This module
# computes that pose as the torque below. It is recorded as a fact about
# a specific event rather than as a calibration, because it is one point
# and a line needs two.
FAULT_JOINT = 2
FAULT_LOAD_PCT = 80.0
FAULT_TORQUE_NM = 0.864

# `guards.LOAD_TRIP_PCT` — the sustained load the bench already treats as
# too much, as opposed to the 80% peak that stops a run outright. Imported
# rather than retyped so the two cannot drift apart.
try:
    from hardware.bench.guards import LOAD_TRIP_PCT
except Exception:                              # pragma: no cover
    LOAD_TRIP_PCT = 55.0

# How much of the derived budget to actually spend. The budget is an
# ESTIMATE built from one observation, and its error runs in both
# directions with the net sign unestablished (see the module docstring —
# an earlier comment here called it an upper bound, which it is not).
# The margin is not a safety factor on a known quantity; it is what
# stands in for a measurement nobody has taken.
DEFAULT_MARGIN = 0.25


def hold_torque(twin: Twin, ticks: dict[int, int]) -> dict[int, float]:
    """Gravity hold torque per carrying joint, N.m, at one pose.

    `qfrc_bias` is Coriolis + centrifugal + gravity; at zero velocity
    the first two vanish and what is left is exactly what each actuator
    must supply to hold the pose. Magnitudes: the sign says which way
    the joint is being pulled and no caller here cares.
    """
    q = twin._rest_qpos.copy()
    for i, t in ticks.items():
        q[twin._adr[i]] = twin.qpos_of(i, t)[0]
    twin.data.qpos[:] = q
    twin.data.qvel[:] = 0
    mujoco.mj_forward(twin.model, twin.data)
    return {i: abs(float(twin.data.qfrc_bias[twin._adr[i]]))
            for i in CARRYING_JOINTS}


def peak_hold(twin: Twin, ticks: dict[int, int]) -> tuple[float, int]:
    """The worst-loaded carrying joint at a pose — (N.m, joint id).

    The peak is what matters, not the sum: each servo is its own limit,
    and a pose is unholdable as soon as ONE of them is over.
    """
    tau = hold_torque(twin, ticks)
    j = max(tau, key=tau.get)
    return tau[j], j


def budget_nm(load_pct: float = LOAD_TRIP_PCT,
              margin: float = DEFAULT_MARGIN) -> float:
    """The torque budget at a given load percent — AN ESTIMATE, not a bound.

    Scaled linearly from the single fault observation, which is the only
    evidence linking the servo's load register to newton-metres.

    AN EARLIER DOCSTRING CLAIMED THIS WAS A PROVEN UPPER BOUND. It is
    not, and the error is instructive. It argued: the guard fires at
    >= 80%, so 100% is AT MOST 0.864/0.80. That step needs the response
    to be LINEAR THROUGH THE ORIGIN — the very assumption the module
    docstring spends a paragraph doubting. With an offset `a` (load
    reading `a + k*tau`) the true full-scale torque is
    `0.864 * (100 - a) / (80 - a)`, which is 1.111 at a=10 and 1.296 at
    a=40 — all ABOVE the "at most" figure. So the inequality ran the
    wrong way whenever the thing it was hedging against was true.

    AND THE SIGN OF THAT EFFECT RUNS THE OTHER WAY FROM WHAT AN EARLIER
    DRAFT OF THIS DOCSTRING SAID. It claimed the linear estimate was
    "conservative for every offset a >= 0" at the default 55%. It is
    OPTIMISTIC there, every time — the true value is
    `0.864 * (p - a) / (80 - a)`, so below the anchor an offset pulls it
    DOWN and above the anchor it pushes it UP:

        load_pct   this fn    a=10    a=20    a=40
             55      0.594   0.555   0.504   0.324   <- fn overstates
             90      0.972   ----    1.008   ----    <- fn understates

    So the offset makes this OPTIMISTIC at the working percent, which is
    the dangerous direction and is why `margin` is not decorative. It is
    also why extrapolating above the anchor is refused: not because it
    would be unsafe (past 80 the offset makes it conservative) but
    because a single anchor cannot support a slope, and a number that
    silently changes which way it errs at 80% is worse than no number.
    """
    if not 0.0 < load_pct <= FAULT_LOAD_PCT:
        raise BenchError(
            f"load percent must be in (0, {FAULT_LOAD_PCT:g}], got "
            f"{load_pct:g}",
            f"the only observation linking load to torque is at "
            f"{FAULT_LOAD_PCT:g}%, and a linear scale extrapolated ABOVE "
            f"its own anchor inverts — the bench trips sustained load at "
            f"{LOAD_TRIP_PCT:g}% anyway")
    if not 0.0 <= margin < 1.0:
        raise BenchError(
            f"margin must be in [0, 1), got {margin:g}",
            "0.25 keeps a quarter of the budget back; 0 spends it all")
    at_full = FAULT_TORQUE_NM / (FAULT_LOAD_PCT / 100.0)
    return at_full * (load_pct / 100.0) * (1.0 - margin)


@dataclass(frozen=True)
class Envelope:
    """The torque-limited working ring, alongside the geometric one."""

    budget_nm: float
    load_pct: float
    margin: float
    plumb: bool
    r_geom_mm: tuple           # what reach.py alone allows
    r_hold_mm: tuple           # what the arm can also HOLD
    # Radii INSIDE r_hold_mm that no posture can hold. The holdable set
    # is not guaranteed contiguous and printing (min, max) as an
    # interval quietly licenses whatever falls in the gaps — on the tilt
    # ring at default sampling, 190 of 1227 radii inside the printed
    # range are over budget, the worst by 76%. Reported rather than
    # smoothed over.
    holes_mm: tuple
    # Worst commanded-vs-simulated clamp across the sampled poses, in
    # degrees. `qpos_of` clamps a tick to the model's joint range and
    # reports by how much; this module evaluates the CLAMPED pose, so a
    # large clamp means the torque belongs to a pose the arm was not
    # actually asked for. Surfaced rather than modelled: on the tilt
    # ring 284 samples clamp, up to 17.6 deg, worst torque error 4.8% —
    # and 0 of them change a verdict today. It is latent, and latent
    # things get found when someone can see the number.
    worst_clamp_deg: float
    floor_nm: float            # cheapest pose anywhere in the ring
    floor_joint: int
    worst_nm: float            # at the geometric outer edge
    # (radius, peak N.m, which joint, RUNNER-UP N.m). The runner-up is
    # carried because "which joint binds" is only meaningful when it
    # binds by a margin. On the tilt ring j2 and j3 sit within a few
    # percent over most of the radii, so the argmax flips on numerical
    # noise — a first draft read that as 386 handovers and printed all
    # of them.
    by_radius: tuple

    @property
    def cost_mm(self) -> float:
        """Outer reach given up to the torque limit."""
        return self.r_geom_mm[1] - self.r_hold_mm[1]

    @property
    def floor_is_binding(self) -> bool:
        """True when NO pose in the ring fits the budget."""
        return self.floor_nm > self.budget_nm


def envelope(twin: Twin, rig: Rig, load_pct: float = LOAD_TRIP_PCT,
             margin: float = DEFAULT_MARGIN, plumb: bool = True,
             step: float = DEFAULT_STEP_DEG,
             samples: list | None = None) -> Envelope:
    """Where the arm can work AND hold, at one slew.

    BEST CASE PER RADIUS, deliberately. Several postures reach the same
    grip radius and they do not cost the same — the arm can be folded
    high or stretched low to put the jaws in the same place. Taking the
    minimum answers "is there ANY way to work here", which is the
    question someone siting a worksite is asking. Taking the mean or the
    worst would refuse radii the arm can plainly manage.
    """
    budget = budget_nm(load_pct, margin)
    cals = twin.cals
    lo1, hi1 = twin.frame_x(1, cals[1].min), twin.frame_x(1, cals[1].max)
    mid = (min(lo1, hi1) + max(lo1, hi1)) / 2.0
    # `samples` lets a caller hand in a sweep it has already paid for —
    # the collision gate is ~99% of the cost here and the selftest needs
    # the same sweep for its own aggregation check. It must be the
    # GATE-CLEAR samples at this slew; passing a different set silently
    # changes the answer, so the parameter is for callers that computed
    # it the same way, not a general injection point.
    clear = samples if samples is not None else [
        s for s in profile(twin, rig, mid, step=step, plumb=plumb)
        if not s.blocked]
    if not clear:
        raise BenchError(
            "no gate-clear pose puts the jaws in the grasp band, so there "
            "is no envelope to bound",
            "run `python -m sim.reach show` — this is a reach failure, "
            "not a torque one")
    best: dict[float, tuple] = {}
    worst_clamp = 0.0
    for s in clear:
        key = round(s.r_mm, 1)
        # `qpos_of` already returns the clamp in DEGREES (see its
        # docstring) — a first pass ran it through math.degrees and
        # would have reported 17.6 deg as 1008.
        for i, tk in s.ticks.items():
            worst_clamp = max(worst_clamp, twin.qpos_of(i, tk)[1])
        tau = hold_torque(twin, s.ticks)
        ranked = sorted(tau.items(), key=lambda kv: -kv[1])
        peak, j = ranked[0][1], ranked[0][0]
        second = ranked[1][1] if len(ranked) > 1 else 0.0
        if key not in best or peak < best[key][0]:
            best[key] = (peak, j, second)
    radii = sorted(best)
    floor_r = min(radii, key=lambda k: best[k][0])
    holdable = [k for k in radii if best[k][0] <= budget]
    lo, hi = (min(holdable), max(holdable)) if holdable else (0.0, -1.0)
    holes = tuple((k, best[k][0]) for k in radii
                  if lo <= k <= hi and best[k][0] > budget)
    return Envelope(
        budget_nm=budget, load_pct=load_pct, margin=margin, plumb=plumb,
        r_geom_mm=(radii[0], radii[-1]),
        r_hold_mm=((lo, hi) if holdable
                   else (float("nan"), float("nan"))),
        holes_mm=holes, worst_clamp_deg=worst_clamp,
        floor_nm=best[floor_r][0], floor_joint=best[floor_r][1],
        worst_nm=best[radii[-1]][0],
        by_radius=tuple((k, *best[k]) for k in radii))


# -------------------------------------------------------------- commands


def cmd_show(twin: Twin, rig: Rig, load_pct: float, margin: float,
             step: float) -> int:
    at_full = FAULT_TORQUE_NM / (FAULT_LOAD_PCT / 100.0)
    print("static holding envelope — what the arm can hold under its own\n"
          "weight, derived from the model's masses at run time.\n")
    print(f"THE ONE MEASUREMENT, and everything below is scaled from it")
    print(f"  2026-07-31    joint {FAULT_JOINT} reported the guard's "
          f"{FAULT_LOAD_PCT:.0f}% peak and torque was cut,")
    print(f"                at a pose this model computes as "
          f"{FAULT_TORQUE_NM:.3f} N.m")
    print(f"  scaled        100% load taken as {at_full:.3f} N.m "
          f"(linear through zero)")
    print(f"  budget        {load_pct:.0f}% sustained, less a "
          f"{margin * 100:.0f}% margin -> "
          f"{budget_nm(load_pct, margin):.3f} N.m")
    # The operator reads THIS, not the docstring, so the retraction has
    # to live here too. Two earlier versions of these lines claimed an
    # "AT MOST" bound and an unambiguous optimism; both were withdrawn
    # in the source and both went on printing for a revision afterwards,
    # which is the worst place for a stale safety claim to hide.
    print(f"  NOT A CALIBRATION, and the direction of the error is NOT\n"
          f"                established. One point cannot pin a line. "
          f"Gearbox friction\n"
          f"                makes this OPTIMISTIC at the working percent; "
          f"the fault\n"
          f"                reading was taken mid-motion, which makes it "
          f"CONSERVATIVE\n"
          f"                by roughly 8%. Which wins depends on a number "
          f"nobody has\n"
          f"                measured yet — that is 716.6's bench pass. "
          f"Treat the\n"
          f"                margin as covering neither.")

    for plumb in (True, False):
        env = envelope(twin, rig, load_pct, margin, plumb, step)
        head = ("PLUMB — jaws vertical, the safe grasp" if plumb
                else "TILT ALLOWED — j4 free, jaws may arrive at an angle")
        print(f"\n{head}")
        print(f"  can reach     {env.r_geom_mm[0]:.0f} .. "
              f"{env.r_geom_mm[1]:.0f} mm   (geometry + collision only)")
        if env.floor_is_binding:
            print(f"  can HOLD      nothing — the cheapest pose in the ring "
                  f"costs {env.floor_nm:.3f} N.m,\n                which is "
                  f"already over the budget")
            continue
        print(f"  can HOLD      {env.r_hold_mm[0]:.0f} .. "
              f"{env.r_hold_mm[1]:.0f} mm   "
              f"(GIVES UP {env.cost_mm:.0f} mm of outer reach)")
        if env.holes_mm:
            worst_r, worst_t = max(env.holes_mm, key=lambda kv: kv[1])
            print(f"  NOT SOLID     {len(env.holes_mm)} sampled radii inside "
                  f"that range are over budget;\n                worst "
                  f"{worst_r:.0f} mm at {worst_t:.3f} N.m "
                  f"({100 * worst_t / env.budget_nm:.0f}% of budget) — the "
                  f"range is not an\n                interval, so check a "
                  f"specific radius rather than trusting the ends")
        print(f"  torque floor  {env.floor_nm:.3f} N.m at j{env.floor_joint} "
              f"— no pose anywhere in the ring is\n                cheaper "
              f"than this, so there is no free place to work")
        print(f"  at full reach {env.worst_nm:.3f} N.m "
              f"({100 * env.worst_nm / at_full:.0f}% of the assumed "
              f"100% load)")
        if env.worst_clamp_deg > 0.5:
            print(f"  clamped       worst pose sits "
                  f"{env.worst_clamp_deg:.1f} deg inside the model's joint "
                  f"range\n                than commanded, so its torque is "
                  f"for a slightly different pose\n                "
                  f"(measured worst error 4.8%, and 0 verdicts change today)")
        # WHICH JOINT BINDS, reported only where it binds by a margin.
        # Two drafts got this wrong in opposite directions: the first
        # printed only the earliest handover and read as if the answer
        # changed once, and the second printed all of them — 386 lines
        # on the tilt ring, because j2 and j3 sit within a few percent
        # there and the argmax flips on noise. Neither was a fact about
        # the arm. What is real is the SEPARATION.
        clear_at = [(r, j, (t - s) / t) for r, t, j, s in env.by_radius
                    if t > 0 and (t - s) / t > 0.05]
        if not clear_at:
            print(f"  binding joint  no single joint binds by more than 5% "
                  f"anywhere in this ring —\n                 j2 and j3 "
                  f"share the load, so neither is the one to relieve")
        else:
            inner_j, outer_j = clear_at[0][1], clear_at[-1][1]
            if inner_j == outer_j:
                print(f"  binding joint j{outer_j} throughout")
            else:
                cross = next(r for r, j, _m in clear_at if j == outer_j)
                print(f"  binding joint j{inner_j} close in, j{outer_j} "
                      f"from about {cross:.0f} mm out")
                print(f"                so pulling a worksite inside "
                      f"{cross:.0f} mm buys margin; inside that\n"
                      f"                the cost is flat and moving it "
                      f"closer buys nothing")

    # The trajectory bound, carried as a NUMBER. This module answers
    # "can the arm hold still there", not "can it get there without
    # exceeding the budget on the way" — and a bare "trajectory not
    # considered" reads as unbounded when it has been measured and is
    # small. Interpolating STAGE to each holdable posture peaks at +0.0%
    # over budget on the plumb ring and +5.0% on the tilt ring (worst
    # case r 275.7 mm, path peak 0.468 against a 0.446 budget).
    print(f"\nGETTING THERE is a separate question, and a bounded one")
    print(f"  these figures are for HOLDING STILL. Approaching a "
          f"worksite can cost\n  more than occupying it: measured worst "
          f"overshoot on the path is +0% on\n  the plumb ring and +5% on "
          f"the tilt ring. Gate the actual clip either way.")

    print("\nTORQUE vs RADIUS (plumb, best posture at each radius)")
    env = envelope(twin, rig, load_pct, margin, True, step)
    # BOTH EDGES are pinned, because bucketing keeps an arbitrary
    # member of each bucket and both ends went missing in turn: the
    # chart read 271 mm against a header saying full reach at 277, then
    # after that was fixed it opened at 90 mm against a header saying
    # the ring starts at 83. A table whose extremes disagree with the
    # summary three lines above it is worse than no table, because the
    # number someone quotes depends on which line they read.
    rows = {round(r / 20) * 20: (r, tau, j) for r, tau, j, _s
            in env.by_radius}
    for r, tau, j, _s in (env.by_radius[0], env.by_radius[-1]):
        rows[round(r / 20) * 20] = (r, tau, j)
    for r, tau, j in sorted(rows.values()):
        bar = "#" * int(round(tau / at_full * 40))
        flag = "  <- over budget" if tau > env.budget_nm else ""
        print(f"  {r:5.0f} mm  j{j}  {tau:6.3f} N.m  {bar}{flag}")
    return 0


def cmd_selftest(twin: Twin, rig: Rig) -> int:
    fails = []

    def check(name, ok, detail=""):
        print(f"  [{'ok ' if ok else 'FAIL'}] {name}"
              + (f" ({detail})" if detail else ""))
        if not ok:
            fails.append(name)

    cals = twin.cals

    def ticks(deg):
        return {i: cals[i].frame.tick(v) for i, v in deg.items()}

    # THE POSE THAT ACTUALLY FAULTED must be the expensive one, and a
    # pose the arm demonstrably held in the same session must be cheap.
    # Both come from the 2026-07-31 run: it reached STAGE, TUCK and swung
    # at both without complaint, and stopped on the way into EXTENDED.
    extended = ticks({1: 0.0, 2: 90.0, 3: 0.0, 4: 0.0})
    tuck = ticks({1: 0.0, 2: 0.0, 3: 88.4, 4: 91.6})
    tau_ext, j_ext = peak_hold(twin, extended)
    tau_tuck, _j = peak_hold(twin, tuck)

    # THE ANCHOR PINNED TO THE MODEL, and this is the most important
    # check in the file. `FAULT_TORQUE_NM` is a frozen constant that
    # every budget is scaled from, while everything it is compared
    # against is recomputed from the model at call time. Without this,
    # the constant can be anywhere in 0.70..1.30 with the whole suite
    # green while the holdable outer edge swings 165..259 mm. A model
    # refresh or a re-ratified frame would silently decouple the
    # servo-capability anchor from the pose it was measured at.
    check("the recorded fault torque still matches what the model says "
          "that pose costs",
          abs(tau_ext - FAULT_TORQUE_NM) < 1e-3,
          f"recorded {FAULT_TORQUE_NM:.3f} N.m vs model "
          f"{tau_ext:.6f} — if this fails the MODEL moved, and the "
          f"constant must be re-derived, not edited to match")
    # WHAT THAT PIN DOES AND DOES NOT CATCH, measured rather than
    # assumed. It is a MASS pin: a +0.2% uniform mass change trips it.
    # It is NOT a frame pin, and an earlier version of this comment
    # claimed it guarded against a re-ratified frame. EXTENDED sits near
    # the stationary point of the gravity torque (d(tau)/d(theta) ~ 0),
    # so 1 deg of j2 moves it by a tenth of the tolerance and a
    # plausible 1-2 deg re-ratification passes cleanly. That is benign —
    # the same flatness means the anchor stays numerically valid under
    # small frame moves — but do not read the pin as covering it.

    check("the pose that faulted on 2026-07-31 is the expensive one",
          tau_ext > tau_tuck * 2.0,
          f"EXTENDED {tau_ext:.3f} N.m at j{j_ext} vs TUCK "
          f"{tau_tuck:.3f} — {tau_ext / tau_tuck:.1f}x")
    check("...and it is over the sustained budget, so this module would "
          "have refused it",
          tau_ext > budget_nm(),
          f"{tau_ext:.3f} N.m vs budget {budget_nm():.3f}")
    check("...while a pose the arm actually held that day is inside it",
          tau_tuck <= budget_nm(),
          f"TUCK {tau_tuck:.3f} N.m vs budget {budget_nm():.3f}")

    # The measured torque must respond to the MODEL, or it is reporting
    # something other than this arm. 716.5 shipped six checks that could
    # not fail; a torque bound that ignores its own inputs would be the
    # same defect where the failure mode is the arm falling over.
    # WHICH JOINTS CARRY, derived from the model rather than trusting
    # the tuple. `CARRYING_JOINTS` excludes j1/j5/j6 on the argument
    # that gravity applies no torque about them; this checks that claim
    # across real poses instead of asserting the membership literally,
    # which would only restate the constant. Dropping j4 from the set
    # changes no answer TODAY (it peaks near 0.1 N.m and never binds) —
    # and that is exactly why it needs a check that is not about today:
    # jaw fittings are a planned ticket, and a heavier hand puts load on
    # the wrist first.
    _lo1, _hi1 = twin.frame_x(1, cals[1].min), twin.frame_x(1, cals[1].max)
    mid_slew = (min(_lo1, _hi1) + max(_lo1, _hi1)) / 2.0
    span = {i: 0.0 for i in (1, 2, 3, 4, 5, 6)}
    # j5 IS SWEPT HERE, and a first version of this check did not sweep
    # it — `profile` pins the wrist roll at rest, so the check cited jaw
    # fittings as its reason for existing while holding fixed the one
    # joint a heavier hand would rotate. A gripper's roll angle is
    # exactly what decides whether its mass hangs off the axis or over
    # it.
    roll = [twin.cals[5].min,
            (twin.cals[5].min + twin.cals[5].max) // 2,
            twin.cals[5].max]
    for smp in profile(twin, rig, mid_slew, step=8.0, plumb=False):
        for t5 in roll:
            q = twin._rest_qpos.copy()
            for i, tk in smp.ticks.items():
                q[twin._adr[i]] = twin.qpos_of(i, tk)[0]
            q[twin._adr[5]] = twin.qpos_of(5, t5)[0]
            twin.data.qpos[:] = q
            twin.data.qvel[:] = 0
            mujoco.mj_forward(twin.model, twin.data)
            for i in span:
                span[i] = max(span[i],
                              abs(float(
                                  twin.data.qfrc_bias[twin._adr[i]])))
    NEGLIGIBLE_NM = 0.02
    should = {i for i, v in span.items() if v > NEGLIGIBLE_NM}
    # TWO CLAIMS, and the first version only made the second. Checking
    # that the model loads exactly {2,3,4} says nothing about whether
    # `hold_torque` reports them: an implementation hardcoding
    # `for i in (2, 3)` left the whole suite green while j4 vanished
    # from every torque this module publishes — defeating the exact
    # future (heavier jaw fittings loading the wrist) the check was
    # written to guard.
    check("`hold_torque` reports every carrying joint, not a subset",
          set(hold_torque(twin, extended)) == set(CARRYING_JOINTS),
          "reports " + ", ".join(f"j{i}" for i in
                                 sorted(hold_torque(twin, extended))))
    check("...and the set of load-carrying joints matches what the model "
          "actually loads",
          should == set(CARRYING_JOINTS),
          "carrying " + ", ".join(f"j{i} {span[i]:.3f}" for i in sorted(span)
                                  if i in should)
          + " | excluded " + ", ".join(f"j{i} {span[i]:.4f}"
                                       for i in sorted(span)
                                       if i not in should))

    # EACH LINK SEPARATELY, because tripling all three together only
    # proves that at least ONE of them reaches the calculation. Pinning
    # any two at baseline and perturbing the third showed all three
    # single-link variants clearing a combined 1.5x gate, so the
    # combined form could not tell "responds to the model" from
    # "responds to one link and ignores the rest".
    before, _j = peak_hold(twin, extended)
    saved = twin.model.body_mass.copy()
    per_link = {}
    for name in ("lower_arm", "wrist", "gripper"):
        try:
            bid = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_BODY,
                                    name)
            twin.model.body_mass[bid] *= 3.0
            per_link[name], _j = peak_hold(twin, extended)
        finally:
            twin.model.body_mass[:] = saved
    deaf = [n for n, v in per_link.items() if v <= before * 1.02]
    check("EVERY outboard link's mass reaches the calculation, tested one "
          "at a time", not deaf,
          ", ".join(f"{n} {v / before:.2f}x" for n, v in per_link.items())
          if not deaf else f"ignored: {', '.join(deaf)}")
    restored, _j = peak_hold(twin, extended)
    check("...and restoring them puts it back exactly, so the probe left "
          "no residue",
          abs(restored - before) < 1e-9,
          f"{restored:.6f} vs {before:.6f}")

    # Gravity is what this measures. Turn it off and the answer must go
    # to zero — this fails if `qfrc_bias` ever starts picking up
    # friction, damping or constraint forces. It is NOT what went wrong
    # with the first `mj_inverse` attempt (that was actuator saturation,
    # see the module docstring); it guards a different, still-real way
    # for this number to stop being pure weight.
    g = twin.model.opt.gravity.copy()
    try:
        twin.model.opt.gravity[:] = 0.0
        zero, _j = peak_hold(twin, extended)
    finally:
        twin.model.opt.gravity[:] = g
    check("with gravity off the hold torque is zero, so this is measuring "
          "weight and not friction",
          zero < 1e-9, f"{zero:.2e} N.m")

    env = envelope(twin, rig, step=4.0)
    # TWO-SIDED, because `cost_mm > 1.0` against an actual ~87 mm has 87x
    # of slack and rules out only "the limit does nothing". That single
    # loose bound was the sole guard on the outer edge, and it let a
    # +/-30% error in the torque scale, a budget that ignored `margin`,
    # and a wrong anchor constant all through. A band fails on any of
    # them. Same triage as reach.py's ring pin: if calibration or the
    # model moved, re-derive; if neither did, the arithmetic changed.
    # AT STEP 4, WHICH IS NOT THE SHIPPED DEFAULT. `show` runs at
    # DEFAULT_STEP_DEG = 2.0 and reports 195 mm; the selftest runs at 4.0
    # for speed and pins 189. Both are correct and they are different
    # numbers for the same quantity, which is exactly the confusion this
    # file warns about elsewhere — so it is said here rather than left
    # for someone to discover by quoting the wrong one.
    HOLD_EDGE_AT_STEP_4 = 189.0
    check("the holdable outer edge is where it was last measured",
          abs(env.r_hold_mm[1] - HOLD_EDGE_AT_STEP_4) < 5.0,
          f"{env.r_hold_mm[1]:.0f} mm vs recorded "
          f"{HOLD_EDGE_AT_STEP_4:.0f} (geometric reach is "
          f"{env.r_geom_mm[1]:.0f}, so {env.cost_mm:.0f} mm given up)")
    check("...and the inner edge is NOT what the torque limit removes",
          env.r_hold_mm[0] <= env.r_geom_mm[0] + 0.5,
          f"holdable from {env.r_hold_mm[0]:.0f} mm vs reachable from "
          f"{env.r_geom_mm[0]:.0f} — close-in poses are cheap")
    # The floor is the finding that shapes where worksites go, so it is
    # asserted rather than merely printed.
    # THE FLOOR, pinned by VALUE and by JOINT, and it doubles as a
    # second torque anchor. The EXTENDED pin constrains exactly one
    # pose, so a torque error shaped to spare that pose slipped through
    # at +/-14%; the floor lives in a different regime (folded in, j3
    # binding) and closes most of that. `floor_joint` was printed and
    # never asserted — returning j9 was green and `show` cheerfully
    # printed "torque floor 0.307 N.m at j9". Which joint binds where is
    # load-bearing prose in this module's own docstring: j3 close in on
    # the plumb ring, j2 throughout on the tilt ring.
    FLOOR_AT_STEP_4 = (0.307, 3)
    check("there is a torque FLOOR — no pose in the ring is free — and "
          "it is where it was measured",
          env.floor_nm > 0.1
          and abs(env.floor_nm - FLOOR_AT_STEP_4[0]) < 0.01
          and env.floor_joint == FLOOR_AT_STEP_4[1],
          f"cheapest anywhere is {env.floor_nm:.3f} N.m at "
          f"j{env.floor_joint}, recorded {FLOOR_AT_STEP_4[0]:.3f} at "
          f"j{FLOOR_AT_STEP_4[1]}")

    # THE TILT RING, which `show` prints and nothing tested. It is also
    # the ONLY ring where `envelope`'s headline design decision — best
    # posture per radius — actually does anything: on the plumb ring at
    # this step every radius holds a single posture, so the comparison
    # branch never runs and min/max/mean/first-seen are indistinguishable.
    # Testing the aggregation on the plumb ring alone tests nothing.
    multi, worst_spread, worst_r = 0, 0.0, None
    seen: dict = {}
    # SWEPT ONCE. `envelope` and this loop used to run the identical
    # collision sweep back to back, 11 s each, for 22 of the suite's
    # 27 s. The independence that matters is recomputing `peak_hold`
    # from the poses — not re-running the gate — so hoisting the sweep
    # costs the check nothing.
    #
    # AT `mid`, NOT 0.0. `envelope` sweeps at j1 mid-travel; this used
    # to sweep at zero and agree only because `reach.py`'s profile is
    # j1-symmetric — an invariant its own docstring warns stops holding
    # the moment the table becomes a bounded box. Then `got == n_min`
    # would fail against correct code.
    lo1, hi1 = twin.frame_x(1, cals[1].min), twin.frame_x(1, cals[1].max)
    mid = (min(lo1, hi1) + max(lo1, hi1)) / 2.0
    tilt_clear = [s for s in profile(twin, rig, mid, step=4.0, plumb=False)
                  if not s.blocked]
    tilt = envelope(twin, rig, plumb=False, step=4.0, samples=tilt_clear)
    for s in tilt_clear:
        seen.setdefault(round(s.r_mm, 1), []).append(
            peak_hold(twin, s.ticks)[0])
    for r, taus in seen.items():
        if len(taus) > 1:
            multi += 1
            if max(taus) - min(taus) > worst_spread:
                worst_spread, worst_r = max(taus) - min(taus), r
    check("the tilt ring really does offer several postures per radius, "
          "so 'best posture' is not dead code",
          multi > 0,
          f"{multi} of {len(seen)} radii have more than one posture; "
          f"worst spread {worst_spread:.3f} N.m at r {worst_r or 0:.0f} mm")
    check("...and the spread is big enough that choosing the WRONG one "
          "would change the answer",
          worst_spread > tilt.budget_nm * 0.25,
          f"{worst_spread:.3f} N.m against a {tilt.budget_nm:.3f} budget")
    # The aggregation must be the MINIMUM, and this compares the SET of
    # holdable radii rather than its outer edge. The edge does not
    # discriminate: at the tilt ring's outer limit every posture happens
    # to be under budget, so min and max agree there and a first draft
    # of this check failed against correct code. Where the choice bites
    # is the interior — the radii one aggregation admits and the other
    # refuses.
    n_min = sum(1 for taus in seen.values()
                if min(taus) <= tilt.budget_nm)
    n_max = sum(1 for taus in seen.values()
                if max(taus) <= tilt.budget_nm)
    got = sum(1 for _r, tau, _j, _s in tilt.by_radius
              if tau <= tilt.budget_nm)
    check("...and `envelope` takes the CHEAPEST posture, not the dearest "
          "or an arbitrary one",
          got == n_min and n_min > n_max,
          f"cheapest admits {n_min} radii, dearest {n_max}; envelope "
          f"admits {got}")

    # THE TILT RING'S ANSWER, not just its postures. Three checks above
    # test how it chooses and none tested what it concluded, so an
    # envelope that inflated the tilt outer edge to the geometric
    # maximum (287 -> 428 mm) and blanked `holes_mm` passed cleanly —
    # while `show` printed that ring as prominently as the plumb one.
    TILT_EDGE_AT_STEP_4 = 287.0
    check("the tilt ring's holdable edge is where it was last measured",
          abs(tilt.r_hold_mm[1] - TILT_EDGE_AT_STEP_4) < 6.0,
          f"{tilt.r_hold_mm[1]:.0f} mm vs recorded "
          f"{TILT_EDGE_AT_STEP_4:.0f} (geometric "
          f"{tilt.r_geom_mm[1]:.0f})")

    # HOLES ARE REPORTED. `r_hold_mm` is printed as an interval and the
    # tilt set is not contiguous; an `envelope` returning `holes_mm=()`
    # was green while `show` advertised 53..287 mm solid with 82 radii
    # inside it over budget, the worst at 176%. That is precisely the
    # wrong answer the field exists to prevent.
    bad = [(r, v) for r, v in tilt.holes_mm if v <= tilt.budget_nm]
    check("the tilt ring's holdable range is reported as NOT solid, and "
          "every hole really is over budget",
          bool(tilt.holes_mm) and not bad,
          f"{len(tilt.holes_mm)} holes, worst "
          f"{max((v for _r, v in tilt.holes_mm), default=0):.3f} N.m vs "
          f"budget {tilt.budget_nm:.3f}"
          + (f" — {len(bad)} are NOT over budget" if bad else ""))

    # The two imported thresholds must stay compatible: `budget_nm` now
    # refuses a load percent above the anchor, and its DEFAULT comes
    # from `guards.LOAD_TRIP_PCT`. Raise that above 80 and every default
    # call raises, making the module unusable — with the refusal hint
    # still cheerfully asserting the bench trips at 55%.
    check("the bench's sustained-load threshold is inside the anchor, so "
          "the default budget is computable",
          LOAD_TRIP_PCT <= FAULT_LOAD_PCT,
          f"LOAD_TRIP_PCT {LOAD_TRIP_PCT:g} vs anchor "
          f"{FAULT_LOAD_PCT:g}")

    # THE BUDGET RESPONDS TO ITS OWN ARGUMENTS. Nothing asserted this,
    # and a `budget_nm` that ignored `margin` entirely passed the whole
    # suite while inflating the budget by a third. `margin` is the only
    # expression of the fact that the servo curve is unmeasured, so a
    # margin that silently does nothing is the worst failure here.
    check("margin actually reduces the budget",
          budget_nm(55.0, 0.5) < budget_nm(55.0, 0.0) * 0.51,
          f"m=0 {budget_nm(55.0, 0.0):.3f} -> m=0.5 "
          f"{budget_nm(55.0, 0.5):.3f} N.m")
    check("...and a bigger load percent means a bigger budget",
          budget_nm(30.0, 0.0) < budget_nm(60.0, 0.0),
          f"30% {budget_nm(30.0, 0.0):.3f} < 60% "
          f"{budget_nm(60.0, 0.0):.3f} N.m")
    # The anchor must reproduce itself: at its own load percent with no
    # margin, the budget IS the observed torque. This is the round trip
    # that ties the scale to the observation.
    check("...and at the anchor's own load percent it returns the "
          "observed torque exactly",
          abs(budget_nm(FAULT_LOAD_PCT, 0.0) - FAULT_TORQUE_NM) < 1e-9,
          f"budget({FAULT_LOAD_PCT:g}%, no margin) = "
          f"{budget_nm(FAULT_LOAD_PCT, 0.0):.6f} vs "
          f"{FAULT_TORQUE_NM:.6f} N.m")

    # Refusals, each matched against the RULE that should have fired
    # rather than merely "some BenchError with a hint" — a validator
    # that rejected every non-default input with one generic message
    # passed the earlier version of these four.
    for label, fn, want in (
            ("a load percent of 0", lambda: budget_nm(0.0), "load percent"),
            ("a load percent above the anchor",
             lambda: budget_nm(90.0), "load percent"),
            ("a margin of 1.0", lambda: budget_nm(55.0, 1.0), "margin"),
            ("a negative margin", lambda: budget_nm(55.0, -0.5), "margin")):
        try:
            fn()
            check(f"{label} is refused", False, "it was accepted")
        except BenchError as exc:
            check(f"{label} is refused for THAT reason",
                  want in str(exc) and bool(exc.hint), str(exc)[:52])
    # ...and the documented-legal endpoints are NOT refused, so the
    # validator cannot pass by rejecting everything.
    for label, fn in (("the anchor's own load percent",
                       lambda: budget_nm(FAULT_LOAD_PCT)),
                      ("a margin of exactly 0",
                       lambda: budget_nm(55.0, 0.0))):
        try:
            fn()
            check(f"{label} is ACCEPTED", True, "as documented")
        except BenchError as exc:
            check(f"{label} is ACCEPTED", False, str(exc)[:52])

    print(f"\nhold selftest {'OK' if not fails else 'FAILED'}"
          + ("" if not fails else f" — {len(fails)}: " + "; ".join(fails)))
    return 0 if not fails else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m sim.hold",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("show", "selftest"))
    parser.add_argument("--cal", default="calibration.json")
    parser.add_argument("--load", type=float, default=LOAD_TRIP_PCT,
                        help=f"load percent to budget against "
                             f"(0-{FAULT_LOAD_PCT:g}; above the anchor the "
                             f"linear scale inverts and is refused)")
    parser.add_argument("--margin", type=float, default=DEFAULT_MARGIN,
                        help="fraction of the budget held back (0-1)")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP_DEG)
    args = parser.parse_args()
    try:
        twin = Twin(cal_path=args.cal)
        rig = Rig(twin)
        if args.command == "selftest":
            return cmd_selftest(twin, rig)
        return cmd_show(twin, rig, args.load, args.margin, args.step)
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
