"""posemachine — the pose library as a state machine.

Plan #660's last piece. `sim.clip` says what a move IS, `sim.edges` says
whether one is SAFE, and `runner.run_clip` PERFORMS one. What none of
them answer is the question an autonomous cell asks constantly: *given
where the arm is right now, which moves may fire at all?*

    states       named poses (poses.json), plus BETWEEN
    events       "go:<pose>"
    transitions  authored edges between poses
    guards       (a) the twin has validated this edge, and
                 (b) the ENCODERS put the arm at the source pose

Both guards, not either. (a) alone certifies a path from a pose the arm
may not be in — the failure `runner`'s per-edge re-gate exists for. (b)
alone confirms the starting point of a move nobody simulated.

WHY BETWEEN IS A REAL STATE. A clip that stops mid-edge leaves the arm
somewhere with no name. The tempting model is to say it is still at the
pose it left, or already at the one it was heading for; both are lies,
and each one gates the NEXT move from a pose the arm is not in — which
is precisely how a gate stays green while the arm collides. So an
aborted move lands in BETWEEN, and BETWEEN authorises nothing. Getting
out of it is `jog` (gated per step) followed by `resync`, which re-reads
the encoders and only names a state if the arm is really at one.

NOTHING HERE MOVES THE ARM BY ITSELF. `go()` asks the guards, then hands
the edge to `run_clip` — the same executor, the same clip, the same
per-edge gate. This module decides; it does not drive.

    uv run python -m hardware.bench.posemachine show
    uv run python -m hardware.bench.posemachine selftest
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import serial

from hardware.units import fmt_ticks, span_deg
from orchestrator.fsm import Refused, StateMachine, Transition

from .bus import BenchError, run_tool
from .calibrate import JointCal, load_joint_calibration
from .motion import SETTLE_TOL_TICKS
from .posegate import PoseGate

# The arm is "at" a pose when every joint is within this of it.
#
# IMPORTED, not restated. `wait_settle` calls the arm arrived at
# SETTLE_TOL_TICKS, so anything TIGHTER here would refuse to move from
# poses the executor considers reached — the machine would deadlock on
# its own tolerance, and the two numbers drifting apart later is exactly
# how that would happen without anyone editing this file. Not looser
# either: the per-edge gate in `run_clip` covers the slack between
# "settled" and "exactly there", and widening this hands it more than it
# was measured against.
AT_POSE_TOL_TICKS = SETTLE_TOL_TICKS

BETWEEN = "BETWEEN"


class _PoseFsm(StateMachine):
    """The machine PoseMachine owns.

    Two reasons it is a named subclass rather than the base class
    directly. Its refusals are signed with `type(self).__name__`, so an
    operator should see which machine spoke; and its guards READ THE
    SERVO BUS, which the base class needs telling about — a dead servo
    mid-guard is a bench fault carrying its own hint, not a broken state
    machine, and it must reach the CLI as itself."""

    TRANSPARENT_GUARD_ERRORS = (BenchError, serial.SerialException)


#: `pose_distance` to a pose that names nothing. Not zero — see below.
UNREACHABLE = 1 << 30


def pose_distance(a: dict[int, int], b: dict[int, int]) -> int:
    """Worst per-joint difference, in ticks, over the joints B names.

    Over B's joints rather than A's: B is the pose being asked about,
    and a joint it does not mention is a joint it makes no claim about.

    A pose that names NOTHING is therefore infinitely far, not zero.
    The natural reading of "worst difference over an empty set" is 0,
    and that reading is a security hole: every caller here treats a
    small distance as "the arm is there". An empty target made the
    encoder guard pass from any position and made `at()` prefer the
    empty pose over every real one, because 0 wins the nearest-pose
    comparison outright. `library_pose` now refuses to build such a
    pose, so this is the second line of defence — but a guard whose
    failure mode is "authorise everything" gets two.
    """
    if not b:
        return UNREACHABLE
    return max((abs(b[i] - a[i]) for i in b if i in a), default=UNREACHABLE)


class PoseMachine:
    """Which move may fire, given where the arm actually is.

    Not a StateMachine subclass — it OWNS one. The states are data
    (whatever poses.json holds), and StateMachine takes its definition
    from class attributes; building the class dynamically to satisfy
    that would be a lot of metaprogramming to hide one attribute.
    """

    def __init__(self, cals: dict[int, JointCal], poses: dict,
                 edges: list[tuple[str, str]], *,
                 gate=None, cache=None, profile=None,
                 tol: int = AT_POSE_TOL_TICKS):
        from sim.clip import DEFAULT_PROFILE

        unknown = sorted({n for e in edges for n in e} - set(poses))
        if unknown:
            raise BenchError(
                f"edge(s) name pose(s) {unknown}, which are not defined",
                f"known poses: {', '.join(sorted(poses)) or 'none'}")
        if BETWEEN in poses:
            raise BenchError(
                f"'{BETWEEN}' is reserved — it is the state an aborted "
                f"move lands in, not a pose you can author",
                "rename the pose in poses.json")

        self.cals = cals
        self.poses = dict(poses)
        self.edges = list(edges)
        self.gate = gate
        self.cache = cache
        self.tol = tol
        self._verdicts: dict[tuple[str, str], object] = {}

        # ONE profile, or none of this means anything. This object
        # validates edges with `self.profile`; the gate re-checks them
        # with its own. Speed and acceleration decide the PATH between
        # two poses, not just how long it takes, so two profiles are two
        # different trajectories being certified as one — and the
        # disagreement would be invisible, because both answers look
        # like verdicts about "this edge". Adopt the gate's when none is
        # given; refuse when they differ.
        if gate is not None and gate.active:
            if profile is None:
                profile = gate.profile
            elif gate.profile != profile:
                raise BenchError(
                    f"the gate simulates at speed {gate.profile.speed}/"
                    f"accel {gate.profile.acceleration}, but this machine "
                    f"plans at speed {profile.speed}/accel "
                    f"{profile.acceleration}",
                    "build the PoseGate with this profile, or pass none "
                    "here and the gate's is adopted — one path can only "
                    "have one answer")
        self.profile = profile or DEFAULT_PROFILE

        names = sorted(poses) + [BETWEEN]
        # A NAMED subclass, so a refusal says which machine spoke rather
        # than "StateMachine". The states are data, so the class is made
        # here instead of declared — one `type()` call, which is less
        # machinery than teaching the base class to carry a display name.
        machine = _PoseFsm.__new__(_PoseFsm)
        machine.STATES = tuple(names)
        # Starting in BETWEEN is the honest default: nothing has read the
        # encoders yet, so the arm's pose is genuinely unknown. `resync`
        # is what earns a named state.
        machine.INITIAL = BETWEEN
        machine.TRANSITIONS = tuple(
            Transition(f"go:{to}", frm, to, guard=self._guard(frm, to))
            for frm, to in edges)
        StateMachine.__init__(machine)
        self.fsm = machine

    # ------------------------------------------------------------ state
    @property
    def state(self) -> str:
        return self.fsm.state

    def at(self, bus) -> str:
        """Which named pose the arm is actually in, or BETWEEN.

        Ambiguity is resolved to the NEAREST pose, and two poses closer
        together than the tolerance are an authoring problem this cannot
        fix — `show` prints them so they can be seen.
        """
        here = self.measure(bus)
        best, best_d = BETWEEN, None
        for name, pose in self.poses.items():
            d = pose_distance(here, pose.ticks)
            if d <= self.tol and (best_d is None or d < best_d):
                best, best_d = name, d
        return best

    def measure(self, bus) -> dict[int, int]:
        return {i: bus.read_position(i) for i in sorted(self.cals)}

    def resync(self, bus) -> str:
        """Re-read the arm and adopt whatever state it is really in.

        The way out of BETWEEN, and the right thing to call after
        anything outside this module has moved the arm — `jog`, a hand
        on the servos, a crashed tool. It never assumes; it measures.
        """
        return self.fsm.interrupt(self.at(bus), "resync")

    def abort(self, why: str = "aborted mid-edge") -> str:
        """The arm stopped between poses. Say so."""
        return self.fsm.interrupt(BETWEEN, why)

    # ----------------------------------------------------------- guards
    def _guard(self, frm: str, to: str):
        """Both preconditions, as ONE guard, so a refusal names whichever
        failed. `bus` arrives through `fire(..., bus=bus)`."""

        def guard(_machine, bus=None, **_) -> bool | Refused:
            verdict = self.validate(frm, to)
            if verdict is not None and not verdict.clean:
                return Refused(
                    f"the twin refuses the edge {frm} -> {to}: "
                    f"{verdict.detail}",
                    "this path is not safe with the current geometry; "
                    "re-author the pose or route through an intermediate "
                    "one — it will be refused every time, so there is "
                    "nothing to retry")
            if bus is None:
                return Refused(
                    "no bus was supplied, so the arm's real pose is unknown",
                    "pass bus= to fire/go; a move must never be authorised "
                    "against an assumed position")
            here = self.measure(bus)
            want = self.poses[frm].ticks
            d = pose_distance(here, want)
            if d > self.tol:
                off = sorted(i for i in want
                             if i in here and abs(here[i] - want[i]) > self.tol)
                detail = "; ".join(
                    f"j{i} reads {fmt_ticks(self.cals[i].frame, here[i])}, "
                    f"'{frm}' is {fmt_ticks(self.cals[i].frame, want[i])}"
                    for i in off)
                return Refused(
                    f"the arm is not at '{frm}' — {span_deg(d):.1f} deg "
                    f"({d}t) away on joint(s) {off}: {detail}",
                    f"the encoders, not the plan, decide where the arm is; "
                    f"`jog` it to '{frm}' and `resync`, or pick an edge "
                    f"that starts where it actually is")
            return True

        guard.__name__ = f"guard_{frm}_to_{to}".replace(" ", "_")
        return guard

    @property
    def gated(self) -> bool:
        """Is the twin half of the guard actually armed?

        Worth asking out loud. This module's contract is BOTH guards,
        and a machine built without a gate silently enforces only the
        encoder one — which still refuses moves from the wrong pose, but
        certifies nothing about the path. Callers that care must be able
        to tell the difference, and `describe` says it unprompted."""
        return self.gate is not None and self.gate.active

    def validate(self, frm: str, to: str):
        """The twin's verdict on one edge, cached across calls.

        Returns None when there is no twin to ask — a machine built
        without a gate still enforces the encoder guard, and says so
        rather than pretending the path was checked.
        """
        if not self.gated:
            return None
        key = (frm, to)
        if key not in self._verdicts:
            from sim.edges import validate_edge
            self._verdicts[key] = validate_edge(
                self.gate.twin, self.poses[frm], self.poses[to],
                self.profile, cache=self.cache)
        return self._verdicts[key]

    # ------------------------------------------------------------ moves
    def allowed(self, bus) -> list[str]:
        """Poses reachable from here RIGHT NOW, guards included.

        What an operator wants on screen, and what a planner wants
        before it commits: the answer is a function of the encoders, so
        it is asked rather than remembered.
        """
        return [t.target for t in self.fsm.TRANSITIONS
                if t.source == self.state and t.guard(self.fsm, bus=bus)]

    def go(self, bus, to: str, **run_kwargs):
        """Authorise the move, then hand it to the executor.

        The FSM commits to `to` BEFORE the arm moves, which is the only
        ordering that survives a crash: a process that dies mid-edge
        must not leave a machine claiming the arm never left. If the
        edge does not complete, the state is corrected to BETWEEN —
        never silently back to the source, which would assert the arm
        returned somewhere it did not.

        A STRAIN WATCH IS ALWAYS ARMED unless the caller supplies its
        own. This function moves a real arm, and `run_clip`'s `strain`
        defaults to None — which would make the in-motion invariant a
        no-op and leave the servos' own load and temperature reporting
        unread. Every other tool in this toolkit that can move the arm
        constructs one unconditionally; defaulting it off here would be
        a silent exception to that rule.

        The keypress e-stop is NOT defaulted on, because `read_key`
        demands a terminal and raises mid-motion without one — the
        caller knows whether a human is watching and passes `poll_key`.
        """
        from sim.clip import Clip

        from .guards import StrainWatch
        from .runner import check_hold_structure, run_clip

        run_kwargs.setdefault("strain", StrainWatch(sorted(self.cals)))
        frm = self.state
        clip = Clip(f"{frm}->{to}",
                    [self.poses[frm], self.poses[to]], self.profile)
        # STRUCTURE FIRST, BEFORE THE MACHINE COMMITS. This refusal is
        # knowable from the clip alone — it needs no bus and no twin —
        # and `fire` moves the state the instant it passes its guards.
        # Refusing afterwards drove the machine to BETWEEN for a move
        # that never happened and never could, so an authoring mistake
        # cost the operator a resync every time they hit it.
        check_hold_structure(clip, self.cals)
        self.fsm.fire(f"go:{to}", bus=bus)
        try:
            return run_clip(bus, self.cals, clip, gate=self.gate,
                            **run_kwargs)
        except BaseException:
            self.abort(f"{frm} -> {to} did not complete")
            raise

    # ------------------------------------------------------------ human
    def describe(self, bus=None) -> str:
        lines = [f"{len(self.poses)} pose(s), {len(self.edges)} edge(s); "
                 f"state {self.state}"]
        # Unprompted, because "which guards are armed" is not something
        # an operator should have to go and check.
        lines.append(
            "  guards: encoders + twin"
            if self.gated else
            "  guards: ENCODERS ONLY — no twin, so no edge here has been "
            "checked for collision")
        for name in sorted(self.poses):
            out = sorted(t for f, t in self.edges if f == name)
            lines.append(f"  {name:<16} -> {', '.join(out) or '(dead end)'}")
        # Two poses inside the tolerance of each other make `at()`
        # ambiguous. It resolves to the nearer, but the authoring is
        # still wrong and silence would let it stay wrong.
        names = sorted(self.poses)
        for n, a in enumerate(names):
            for b in names[n + 1:]:
                d = pose_distance(self.poses[a].ticks, self.poses[b].ticks)
                if d <= self.tol:
                    lines.append(
                        f"  WARNING: '{a}' and '{b}' are {d}t apart, within "
                        f"the {self.tol}t at-pose tolerance — the arm can "
                        f"satisfy both")
        if bus is not None:
            lines.append(f"  measured: the arm is at {self.at(bus)}")
        return "\n".join(lines)


def load_machine(cal_path: Path, edges: list[tuple[str, str]] | None = None,
                 *, gate=None, profile=None,
                 require_gate: bool = True) -> PoseMachine:
    """Build a machine from calibration.json + poses.json.

    BUILDS THE GATE unless one is handed in. The default edge list is
    fully connected (see below), which only makes sense because the twin
    refuses what the geometry forbids — so a machine built without a
    gate would offer every pose-to-pose move on the strength of the
    encoder check alone. That is not a weaker version of this module's
    contract, it is a different and much more dangerous one, and the
    first version of this function shipped it by omission.

    `require_gate=False` is for the case the gate itself is honest
    about: no calibration, no model, no mujoco. It must be asked for.

    With no explicit edge list, every pose is joined to every other: the
    pose library alone says nothing about which moves are INTENDED, and
    the twin is what says which are SAFE. A fully connected graph plus a
    validating guard proposes everything and lets the gate refuse what
    it must, rather than quietly hiding a legal move because nobody
    listed it.
    """
    from sim.clip import load_poses

    cals = load_joint_calibration(cal_path)
    poses = load_poses(cals)
    if not poses:
        raise BenchError(
            "no poses defined — poses.json is missing or empty",
            "a pose machine is a graph over named poses; author some "
            "first (see sim/clip.py load_poses for the format)")
    if edges is None:
        edges = [(a, b) for a in sorted(poses) for b in sorted(poses)
                 if a != b]
    if gate is None:
        gate = PoseGate(sorted(cals), cal_path, profile=profile)
        if not gate.active and require_gate:
            raise BenchError(
                f"cannot build the collision gate: {gate.reason}",
                "every edge here is validated by the twin before it may "
                "fire; without it the machine would offer moves nothing "
                "has checked. Fix the cause, or pass require_gate=False "
                "and accept an encoder-only machine")
    return PoseMachine(cals, poses, edges, gate=gate, profile=profile)


# ------------------------------------------------------------------ CLI
def _selftest(cal_path: Path) -> int:
    """No arm, no twin: a fake bus proves the guards are consulted and
    that a refusal says something the operator can act on."""
    from sim.clip import MotionProfile, Pose

    fails: list[str] = []

    def want(label: str, ok: bool, detail: str = "") -> None:
        print(f"  {'ok  ' if ok else 'FAIL'} {label}"
              f"{'  ' + detail if detail else ''}")
        if not ok:
            fails.append(label)

    if not cal_path.exists():
        print(f"  no {cal_path} here — cannot build a machine")
        return 1
    from orchestrator.fsm import FsmError, GuardsRefused

    cals = load_joint_calibration(cal_path)
    rest = {i: c.rest for i, c in cals.items()}
    pan = {**rest, 1: rest[1] + 300}
    poses = {"REST": Pose("REST", dict(rest)), "PAN": Pose("PAN", dict(pan))}
    edges = [("REST", "PAN"), ("PAN", "REST")]

    class FakeBus:
        def __init__(self, pos):
            self.pos = dict(pos)

        def read_position(self, i):
            return self.pos[i]

    m = PoseMachine(cals, poses, edges,
                    profile=MotionProfile(speed=300, acceleration=15))
    bus = FakeBus(rest)

    print("a machine starts BETWEEN — nothing has read the arm yet")
    want("initial state is BETWEEN", m.state == BETWEEN, m.state)
    want("...and BETWEEN authorises nothing", m.allowed(bus) == [],
         f"{m.allowed(bus)}")
    try:
        m.fsm.fire("go:PAN", bus=bus)
        want("...so a move out of BETWEEN is refused", False)
    except FsmError as exc:
        want("...so a move out of BETWEEN is refused", True,
             type(exc).__name__)

    print("\nresync adopts what the ENCODERS say, not what was assumed")
    want("resync names the pose the arm is really in",
         m.resync(bus) == "REST", m.state)
    want("...and now the edge from it is offered", m.allowed(bus) == ["PAN"],
         f"{m.allowed(bus)}")

    print("\nthe encoder guard refuses a move from a pose the arm is not in")
    off = FakeBus({**rest, 3: rest[3] + 400})
    try:
        m.fsm.fire("go:PAN", bus=off)
        want("a move from a mis-posed arm is refused", False)
    except GuardsRefused as exc:
        want("a move from a mis-posed arm is refused", True)
        want("...naming the joint that is wrong", "j3" in str(exc),
             str(exc).splitlines()[-1][:90])
        want("...and saying what to DO about it, not just no",
             "jog" in str(exc) and "resync" in str(exc))
        want("...while the state is unchanged, because nothing moved",
             m.state == "REST", m.state)

    print("\nan aborted move lands in BETWEEN, never back at the source")
    m.abort("selftest")
    want("abort leaves the machine BETWEEN", m.state == BETWEEN, m.state)
    want("...and it is recorded, not silent",
         any("selftest" in h.event for h in m.fsm.history),
         f"{[h.event for h in m.fsm.history]}")
    want("...and resync can still recover it once the arm is read",
         m.resync(bus) == "REST")

    print("\nauthoring mistakes are refused at construction")
    try:
        PoseMachine(cals, poses, [("REST", "NOWHERE")])
        want("an edge to an undefined pose is refused", False)
    except BenchError as exc:
        want("an edge to an undefined pose is refused", True, str(exc))
    try:
        PoseMachine(cals, {**poses, BETWEEN: Pose(BETWEEN, dict(rest))}, [])
        want(f"a pose named {BETWEEN} is refused", False)
    except BenchError as exc:
        want(f"a pose named {BETWEEN} is refused", True, str(exc))

    print("\nthe machine says which guards are actually armed")
    want("a gate-less machine reports itself as encoder-only",
         not m.gated and "ENCODERS ONLY" in m.describe(),
         [ln.strip() for ln in m.describe().splitlines()
          if "guards:" in ln][0])

    class FakeGate:
        active = True

        def __init__(self, profile):
            self.profile = profile

    p1 = MotionProfile(speed=300, acceleration=15)
    p2 = MotionProfile(speed=100, acceleration=15)
    try:
        PoseMachine(cals, poses, edges, gate=FakeGate(p1), profile=p2)
        want("a gate and a machine planning at DIFFERENT profiles is "
             "refused", False)
    except BenchError as exc:
        want("a gate and a machine planning at DIFFERENT profiles is "
             "refused", True, str(exc)[:78])
    adopted = PoseMachine(cals, poses, edges, gate=FakeGate(p1))
    want("...and with no profile given, the gate's is adopted rather "
         "than a default silently disagreeing with it",
         adopted.profile == p1, f"{adopted.profile}")

    print("\na bus fault inside a guard stays a BENCH error, not an FSM one")

    class DeadBus:
        def read_position(self, i):
            raise BenchError("servo 3 did not answer",
                             "check the daisy chain")

    try:
        m.fsm.fire("go:PAN", bus=DeadBus())
        want("a dead servo during a guard propagates as BenchError", False)
    except BenchError as exc:
        want("a dead servo during a guard propagates as BenchError", True)
        want("...keeping its own hint, which FsmError would have dropped",
             exc.hint == "check the daisy chain", f"{exc.hint}")
    except FsmError:
        want("a dead servo during a guard propagates as BenchError", False,
             "wrapped in FsmError — the CLI would print a traceback")

    print("\nnear-identical poses are reported, not silently tolerated")
    close = {**poses, "NEARLY": Pose("NEARLY", {**rest, 1: rest[1] + 3})}
    m2 = PoseMachine(cals, close, [("REST", "NEARLY")])
    warned = [ln.strip() for ln in m2.describe().splitlines()
              if "WARNING" in ln]
    want("two poses inside the at-pose tolerance are warned about",
         bool(warned), warned[0] if warned else "")

    print("posemachine selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    return 1 if fails else 0


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.posemachine",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("show", "selftest"))
    parser.add_argument("--cal", default="calibration.json")
    args = parser.parse_args()

    cal_path = Path(args.cal)
    if args.command == "selftest":
        return _selftest(cal_path)

    machine = load_machine(cal_path)
    print(machine.describe())
    print("\nno bus was opened — `at`/`allowed` need the encoders, and "
          "this command deliberately does not touch the arm.",
          file=sys.stderr)
    return 0


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    raise SystemExit(main())
