"""Where do the joints go to put the tool THERE? (plan #606)

Every motion in this repo until now has been authored in joint space —
ticks or degrees, per joint, chosen by a human looking at the arm or the
viewer. That is fine while a person picks the poses. It stops working
the moment a camera does, because an object appears at a Cartesian
position nobody chose in advance, and "move the gripper to (x, y, z)"
was not expressible anywhere in the codebase.

NO NEW DEPENDENCY. `mink` (MuJoCo differential IK) has been in
pyproject.toml since the early ladder planning and was never used; its QP
solver `daqp` came with it. So this module is wiring, not adoption.

WHAT IT ADDS ON TOP OF mink, and why each part is load-bearing:

**Ticks, not qpos.** mink solves in model coordinates. The arm speaks
servo ticks. `Twin.tick_of` bridges them, and the round trip quantises
to 0.088 deg — small, but it means the pose that gets COMMANDED is not
exactly the pose that was SOLVED, so the residual is re-measured after
the conversion rather than taken from the solver.

**Reachability, which the solver does not report.** Ask mink for a point
outside the workspace and it does not fail: it converges to the nearest
point it can reach and stops, quietly. Measured on this arm, that is a
stable non-zero residual (~7.6 mm for a target just outside; the arm
simply stretches toward it). Since vision will hand us positions nobody
vetted, the residual IS the reachability test, and this module refuses
above a threshold instead of returning a confident wrong answer.

**Joint limits that are the ARM's, not the model's.** These differ in
both directions — #670 recorded the calibrated range exceeding the
model's on j2 and j6, and the reverse holds on j3, where the model
permits elbow angles the real arm cannot reach. Constrained by the
model, mink solved happily into that surplus and the tick conversion
clamped it back: solver residual 0.000 mm, actual residual 26 mm,
reported as a success. That bug is why the constraint lives on a private
narrowed model here. "We have the library" was never the same as "we
have IK".

Unregularised solving is also simply worse: on an unreachable target it
settles 722 mm away where this module settles 288 mm, i.e. it fails to
even stretch toward the point.

WHICH WAY IS +Y. Targets are in the RIG frame (`sim/rig.py`): +X is the
direction the arm reaches, +Z is up, and it is right-handed, so **+Y is
the arm's LEFT**. This module is where a lateral coordinate first
acquires meaning — the collision gate never needed one — and it is
therefore the first thing a mirrored joint breaks. It was mirrored:
until 2026-07-30 the twin's j1 mapping put positive pan to the arm's
right, so a solve for a target on the left returned a pose that reaches
right, with a residual of ~0 and a confident SUCCESS. The residual is
computed in the same frame as the target, so it cannot catch a mirror —
no reachability check can. Plan 714.6; `sim.twin selftest` pins the sign
now.

WHAT THIS DOES NOT DO. It answers "can the arm reach there", not "can it
get there safely" — the path is not checked here. `hardware.bench.
posegate` answers that, and the two are complementary: IK finds a pose,
the gate decides whether the arm may travel to it. Nor does it do
orientation: `position_cost` only, so the tool's approach angle is
whatever the solver happens to land on. Grasping will need that (#606).

    uv run python -m sim.ik 250 -40 120       # a target in rig mm
    uv run python -m sim.ik selftest
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field

import numpy as np

# Solver settings, all measured rather than picked. See the selftest,
# which fails if any of them stops holding.
SOLVER = "daqp"          # ships with mink; quadprog/osqp/scs are not installed
ITERATIONS = 300         # 0.0001 mm on reachable targets; 100 gets ~0.01 mm
DT = 0.01
DAMPING = 1e-2
POSTURE_COST = 1e-4      # regulariser only — high enough and it fights the task
LM_DAMPING = 1.0

# Above this, the solver stopped short of the target rather than on it,
# which means the point is outside the workspace (or behind a joint
# limit). Reachable solves land 5 orders of magnitude below this; the
# nearest miss measured was 7.6 mm. Nothing lives in between, so the
# threshold is not delicate.
REACH_TOL_MM = 1.0


@dataclass(frozen=True)
class Solution:
    """A solved pose, and an honest account of how well it worked."""

    ticks: dict[int, int]
    residual_mm: float          # AFTER quantisation to ticks
    solver_residual_mm: float   # before it, i.e. what mink achieved
    target_mm: tuple[float, float, float]
    reached_mm: tuple[float, float, float]
    clamped: dict[int, float] = field(default_factory=dict)

    @property
    def reachable(self) -> bool:
        return self.residual_mm <= REACH_TOL_MM

    def __str__(self) -> str:
        t, r = self.target_mm, self.reached_mm
        head = ("REACHABLE" if self.reachable else
                f"OUT OF REACH by {self.residual_mm:.1f} mm")
        lines = [f"{head} — target ({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) mm, "
                 f"tool lands ({r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f}) mm"]
        if self.reachable:
            lines.append(f"  residual {self.residual_mm:.4f} mm "
                         f"(solver {self.solver_residual_mm:.4f} mm, the rest "
                         f"is rounding to whole ticks)")
        for i, t_ in sorted(self.ticks.items()):
            lines.append(f"  j{i} = {t_}")
        for i, deg in sorted(self.clamped.items()):
            lines.append(f"  note: joint {i} clamped {deg:.2f} deg to the "
                         f"model's range")
        return "\n".join(lines)


class Solver:
    """Cartesian target -> servo ticks, in the rig frame.

    The frame is `sim.rig`'s: millimetres, origin at the m1 rotation
    centre, +Z up. Deliberately the same one `rig spec` and `rig where`
    already print, so a number read off one tool can be handed to this
    one without a conversion nobody wrote down.
    """

    def __init__(self, twin=None):
        import mujoco

        from sim.rig import Rig
        from sim.twin import TOOL_BODY, Twin

        self.twin = twin or Twin()
        self.rig = Rig(self.twin)
        self.model = self.twin.model
        self.tool = TOOL_BODY
        self._mujoco = mujoco
        self._data = mujoco.MjData(self.model)

        # THE SOLVER GETS THE ARM'S LIMITS, NOT THE MODEL'S.
        #
        # These are not the same set, and not in a single direction:
        # #670 recorded the calibrated range exceeding the model's on j2
        # and j6, and the reverse holds on j3 — the model permits elbow
        # angles the real arm cannot reach. Left alone, mink solves
        # happily into that surplus, and converting the result to ticks
        # clamps it back, silently moving the tool. Measured before this
        # was fixed: solver residual 0.000 mm, actual residual 26 mm,
        # reported as a success. A confident wrong answer is the one
        # failure this module exists to prevent, so the constraint moves
        # to where it can prevent it rather than where it is convenient.
        #
        # A private model copy because `twin.model` is shared and the
        # gate must keep judging the arm's real range, not this narrowed
        # one — narrowing THAT would make the gate unconservative.
        from sim.twin import MODEL_XML
        self._ik_model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
        for i, cal in self.twin.cals.items():
            adr = self.twin._adr[i]
            jid = int(np.argmax(self.model.jnt_qposadr == adr))
            # qpos_of already clamps to the model range, so taking the
            # calibrated ends through it yields the INTERSECTION of the
            # two ranges — the poses that are both modellable and
            # physically achievable.
            ends = sorted(self.twin.qpos_of(i, t)[0]
                          for t in (cal.min, cal.max))
            self._ik_model.jnt_range[jid] = ends
            self._ik_model.jnt_limited[jid] = 1

    def tool_at(self, ticks: dict[int, int]) -> tuple[float, float, float]:
        """Where the tool actually ends up for a pose in ticks — forward
        kinematics, used to grade the solver rather than trust it."""
        return self.rig.tool_point(self._qpos_for(ticks))

    def _qpos_for(self, ticks: dict[int, int]) -> np.ndarray:
        qpos = self.twin._rest_qpos.copy()
        for i, tick in ticks.items():
            q, _ = self.twin.qpos_of(i, tick)
            qpos[self.twin._adr[i]] = q
        return qpos

    def solve(self, target_mm, seed_ticks: dict[int, int] | None = None,
              iterations: int = ITERATIONS) -> Solution:
        """Find ticks putting the tool at `target_mm` in the rig frame.

        `seed_ticks` is where the search starts. IK is local, so the seed
        decides WHICH solution you get on an arm with more than one — for
        an incremental move pass the arm's current pose, and the result
        stays near it instead of flipping the elbow to an equally valid
        but wildly different configuration.
        """
        import mink

        target = np.asarray(target_mm, dtype=float)
        seed = dict(seed_ticks) if seed_ticks else {
            i: c.rest for i, c in self.twin.cals.items()}
        q_seed = self._qpos_for(seed)

        # The rig frame is millimetres offset from m1; mink works in the
        # model's metres. Convert here, once, rather than at each caller.
        world = self.rig._origin + target / 1000.0

        task = mink.FrameTask(frame_name=self.tool, frame_type="body",
                              position_cost=1.0, orientation_cost=0.0,
                              lm_damping=LM_DAMPING)
        task.set_target(mink.SE3.from_rotation_and_translation(
            mink.SO3.identity(), world))
        posture = mink.PostureTask(self._ik_model, cost=POSTURE_COST)
        posture.set_target(q_seed)

        cfg = mink.Configuration(self._ik_model)
        cfg.update(q_seed.copy())
        limits = [mink.ConfigurationLimit(self._ik_model)]
        for _ in range(iterations):
            v = mink.solve_ik(cfg, [task, posture], DT, solver=SOLVER,
                              damping=DAMPING, limits=limits)
            cfg.integrate_inplace(v, DT)

        # Grade the SOLVER's own answer...
        self._data.qpos[:] = cfg.q
        self._mujoco.mj_forward(self.model, self._data)
        solver_reached = self.rig.tool_point(cfg.q)
        solver_res = float(np.linalg.norm(np.asarray(solver_reached) - target))

        # ...then again after quantising to whole ticks, because that is
        # the pose the arm will actually be commanded to. Reporting the
        # solver's residual alone would flatter the result.
        ticks, clamped = {}, {}
        for i in sorted(self.twin.cals):
            q = float(cfg.q[self.twin._adr[i]])
            tick = self.twin.tick_of(i, q)
            cal = self.twin.cals[i]
            lo, hi = min(cal.min, cal.max), max(cal.min, cal.max)
            bounded = max(lo, min(hi, tick))
            if bounded != tick:
                clamped[i] = abs(tick - bounded) * 360.0 / 4096.0
            ticks[i] = bounded
        reached = self.tool_at(ticks)
        res = float(np.linalg.norm(np.asarray(reached) - target))
        return Solution(ticks, res, solver_res, tuple(target),
                        tuple(reached), clamped)


def solve(target_mm, seed_ticks=None) -> Solution:
    """One-shot convenience — builds a Solver each call, so prefer the
    class when solving repeatedly (model load dominates)."""
    return Solver().solve(target_mm, seed_ticks)


# --------------------------------------------------------------------


def cmd_solve(args: list[str]) -> int:
    try:
        target = [float(a) for a in args]
    except ValueError:
        print("targets are three numbers: x y z, in rig mm")
        return 2
    if len(target) != 3:
        print("need exactly three numbers: x y z, in rig mm")
        return 2
    s = Solver()
    sol = s.solve(target)
    print(sol)
    if not sol.reachable:
        return 1
    # The gate is a separate question and worth answering in the same
    # breath, because "reachable" reads as "fine" otherwise.
    try:
        from hardware.bench.posegate import PoseGate
        gate = PoseGate(sorted(s.twin.cals))
        rest = {i: c.rest for i, c in s.twin.cals.items()}
        v = gate.check_sequence([rest, sol.ticks], label="ik-target")
        print(f"\nfrom rest, the gate says: {v.detail}")
    except Exception as exc:
        print(f"\n(gate not consulted: {exc})")
    return 0


def selftest() -> int:
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not cond:
            fails.append(name)

    from sim.twin import Twin

    twin = Twin()
    s = Solver(twin)
    rest = {i: c.rest for i, c in twin.cals.items()}

    print("tick <-> qpos round-trip (the bridge IK depends on)")
    # Only ticks the MODEL can actually represent round-trip. qpos_of
    # clamps, and on j2/j6 the calibrated range runs past the model's
    # (#670), so the range ends deliberately do NOT come back — an
    # earlier version of this test used them and read 233 ticks of
    # "error" that was the documented clamp doing its job.
    worst, clamped_seen = 0.0, 0
    for i, cal in sorted(twin.cals.items()):
        lo, hi = min(cal.min, cal.max), max(cal.min, cal.max)
        for tick in (cal.rest, (lo + hi) // 2, lo + (hi - lo) // 4,
                     hi - (hi - lo) // 4, lo, hi):
            q, clamp = twin.qpos_of(i, tick)
            if clamp > 1e-9:
                clamped_seen += 1
                continue
            worst = max(worst, abs(twin.tick_of(i, q) - tick))
    check("unclamped ticks round-trip to within a tick", worst <= 1,
          f"worst {worst:.0f} tick(s)")
    check("...and the clamped ends were skipped, not silently passed",
          clamped_seen > 0, f"{clamped_seen} tick(s) sat outside the model "
                            f"range — the #670 clamp, working as documented")

    print("\nreachable targets — FK out, IK back")
    rng = np.random.default_rng(11)
    errs = []
    for trial in range(6):
        pose = {i: int(rng.uniform(min(c.min, c.max) + 200,
                                   max(c.min, c.max) - 200))
                for i, c in twin.cals.items()}
        target = s.tool_at(pose)           # reachable BY CONSTRUCTION
        sol = s.solve(target)
        errs.append(sol.residual_mm)
        check(f"trial {trial} recovers a real pose", sol.reachable,
              f"residual {sol.residual_mm:.4f} mm")
    # The floor here is NOT the solver — it is the servo. One tick is
    # 0.088 deg, which at this arm's reach is a few tenths of a
    # millimetre at the tool, so no amount of solving gets below it.
    # Worth knowing before anyone expects to grasp to a tighter
    # tolerance than the encoder can express.
    check("reachable solves land at the tick-quantisation floor",
          max(errs) < 0.5, f"worst {max(errs):.4f} mm — the solver itself "
                           f"gets to ~0.0002 mm; the rest is rounding to "
                           f"whole ticks, and is a hardware floor")
    check("...which is far below the reachability threshold",
          max(errs) < REACH_TOL_MM / 2,
          f"{max(errs):.4f} mm vs tol {REACH_TOL_MM} mm")
    check("no reachable solve needed clamping",
          all(not s.solve(s.tool_at({i: c.rest for i, c in twin.cals.items()}
                                    )).clamped for _ in [0]),
          "a clamp here means the solver proposed a pose the arm cannot "
          "reach — the 26 mm bug")

    print("\nunreachable targets must be REFUSED, not answered confidently")
    # Straight up beyond full extension, and far out sideways. Both are
    # outside the workspace by a wide margin.
    for name, target in (("2 m overhead", (0.0, 0.0, 2000.0)),
                         ("1.5 m sideways", (1500.0, 0.0, 100.0)),
                         ("below the table", (200.0, 0.0, -500.0))):
        sol = s.solve(target)
        check(f"{name} is flagged out of reach", not sol.reachable,
              f"residual {sol.residual_mm:.1f} mm")

    print("\nthe seed decides which solution you get")
    target = s.tool_at({**rest, 2: rest[2] + 600, 3: rest[3] - 600})
    near = s.solve(target, seed_ticks=rest)
    far = s.solve(target, seed_ticks={i: (min(c.min, c.max)
                                          + max(c.min, c.max)) // 2
                                      for i, c in twin.cals.items()})
    check("both seeds reach the same point", near.reachable and far.reachable,
          f"{near.residual_mm:.4f} / {far.residual_mm:.4f} mm")
    spread = max(abs(near.ticks[i] - far.ticks[i]) for i in near.ticks)
    print(f"       (they differ by up to {spread} ticks in joint space — "
          f"same tool position, different arm configuration)")

    print("\nthe solver is held to the ARM's limits, not the model's")
    # This is the 26 mm bug, pinned. Solving against the MODEL's limits
    # lets j3 past the arm's calibrated range; converting to ticks then
    # clamps it back and moves the tool, while the solver reports a
    # perfect fit. The check is that the shipped path never produces a
    # tick outside the calibrated range, and that the naive path does —
    # so this cannot pass by the difference having quietly evaporated.
    import mink
    target = s.tool_at({**rest, 2: rest[2] + 400})
    sol = s.solve(target)
    out = {i: t for i, t in sol.ticks.items()
           if not (min(twin.cals[i].min, twin.cals[i].max) <= t
                   <= max(twin.cals[i].min, twin.cals[i].max))}
    check("every solved tick is inside the arm's calibrated range", not out,
          str(out))
    check("...and nothing had to be clamped", not sol.clamped,
          str(sol.clamped))

    world = s.rig._origin + np.asarray(target) / 1000.0
    task = mink.FrameTask(frame_name=s.tool, frame_type="body",
                          position_cost=1.0, orientation_cost=0.0,
                          lm_damping=LM_DAMPING)
    task.set_target(mink.SE3.from_rotation_and_translation(
        mink.SO3.identity(), world))
    posture = mink.PostureTask(s.model, cost=POSTURE_COST)
    posture.set_target(s._qpos_for(rest))
    cfg = mink.Configuration(s.model)                    # the WIDE model
    cfg.update(s._qpos_for(rest).copy())
    for _ in range(ITERATIONS):
        cfg.integrate_inplace(
            mink.solve_ik(cfg, [task, posture], DT, solver=SOLVER,
                          damping=DAMPING,
                          limits=[mink.ConfigurationLimit(s.model)]), DT)
    naive_out = {}
    for i in sorted(twin.cals):
        tick = twin.tick_of(i, float(cfg.q[twin._adr[i]]))
        lo, hi = min(twin.cals[i].min, twin.cals[i].max), \
            max(twin.cals[i].min, twin.cals[i].max)
        if not lo <= tick <= hi:
            naive_out[i] = tick
    check("model-limited solving DOES leave the arm's range (the bug)",
          bool(naive_out), f"joints {naive_out} — outside the calibrated "
                           f"range, which is what used to get clamped")

    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("ik OK")
    return 0


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print("usage: python -m sim.ik X Y Z    (rig mm: origin = m1 "
              "centre, +Z up — the frame `rig spec` prints)")
        print("       python -m sim.ik selftest")
        return 2
    if args[0] == "selftest":
        return selftest()
    return cmd_solve(args)


if __name__ == "__main__":
    raise SystemExit(main())
