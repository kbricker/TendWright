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

    # -------------------------------------------------------- validation
    def validate_all(self) -> dict[tuple[str, str], object]:
        """Ask the twin about EVERY edge, with no arm and no bus.

        The graph this machine offers is fully connected by default, on
        the reasoning that the twin refuses what the geometry forbids.
        That is sound at fire time and useless at authoring time: the
        only way to learn that REST -> CRANE_211 can never fire was to
        stand at the bench, resync, ask for it, and be told no. A pose
        library is data, and data that is wrong should say so when it is
        read, not when it is acted on.

        So this walks the whole graph offline.

        WHAT IT DOES NOT DO is spare the arm any work later. An earlier
        version of this docstring claimed the edges it clears "do not
        re-simulate at fire time", and that was false twice over:
        `load_machine` builds no `EdgeCache`, so nothing survives the
        process; and `go()` hands the edge to `run_clip`, whose per-edge
        re-gate runs from the arm's MEASURED pose and is supposed to
        re-simulate — that is the check that catches the arm not being
        where the plan says. The verdicts here are memoised for this
        object's own guard and nothing more.
        """
        if not self.gated:
            # The whole method is "ask the twin". With no twin the honest
            # answer is not a dict of Nones — a caller doing the obvious
            # `sum(v.clean for v in validate_all().values())` gets an
            # AttributeError instead of an answer to the question it
            # asked. `validation_report` guards itself before calling.
            raise BenchError(
                "there is no twin, so no edge can be validated",
                "this machine was built without a collision gate; fix "
                "the cause (see PoseGate.reason) or accept that its "
                "edges are unchecked")
        return {(f, t): self.validate(f, t) for f, t in self.edges}

    def validation_report(self) -> tuple[str, bool]:
        """(report, ok). Not ok when some pose cannot be reached from
        some other pose, in either direction, over CLEAN edges only.

        A REFUSED EDGE IS NOT A FAILURE — it is the gate doing its job,
        and a real library is full of them (in the shipped one, every
        direct REST <-> CRANE move: the gripper scrapes the shoulder on
        the way out of the fold, which is why STAGE exists). What IS a
        failure is the arm being unable to get somewhere at all.

        THAT IS A CONNECTIVITY QUESTION, NOT A DEGREE ONE, and the first
        version of this asked the wrong one. It checked that each pose
        had at least one clean edge in and one out, which every pose in
        a PARTITIONED graph still has. Severing every STAGE <-> CRANE
        edge while leaving REST <-> STAGE clean split this library into
        two islands — {REST, STAGE} and the eleven crane poses, with the
        arm unable to cross — and the report said 112 clear, 44 refused,
        nothing unreachable, exit 0. That is not hypothetical: STAGE is
        the library's single door, its refusals already ship at 0.0-0.3
        mm, and a `calibrate capture` that settles rest a little deeper
        would close it. So this walks the graph.
        """
        if not self.gated:
            return ("no twin, so no edge was checked — this report would "
                    "say nothing"), False
        verdicts = self.validate_all()
        refused = {e: v for e, v in verdicts.items() if not v.clean}
        lines = [f"{len(self.poses)} pose(s), {len(verdicts)} edge(s): "
                 f"{len(verdicts) - len(refused)} clear, {len(refused)} "
                 f"refused by the twin"]
        for name in sorted(self.poses):
            out = sorted(t for (f, t) in refused if f == name)
            if out:
                lines.append(f"  {name} -/-> {', '.join(out)}")
        if refused:
            lines.append("")
            lines.append("why, one example per refusing source:")
            seen: set[str] = set()
            for (f, t), v in sorted(refused.items()):
                if f in seen:
                    continue
                seen.add(f)
                lines.append(f"  {f} -> {t}: {v.detail or 'contact'}")

        # ONE-WAY EDGES, called out because they are the surprising
        # shape. A pair refused in both directions reads as "you cannot
        # go that way" and an operator routes around it. A pair refused
        # in only ONE direction lets the arm travel somewhere it cannot
        # directly come back from, and the discovery happens after the
        # move rather than before. That can be perfectly legitimate —
        # the return leg samples a different path, because each joint
        # runs its own speed profile — but it should never be a
        # surprise, and nothing else in this toolkit would show it.
        oneway = sorted((f, t) for (f, t) in refused
                        if (t, f) in verdicts and verdicts[(t, f)].clean)

        # Reachability over the CLEAN subgraph. ISLANDS, not a root and
        # its complement: an earlier version walked out from
        # sorted(poses)[0] and named everything it could not see, so
        # WHICH SIDE OF A BREAK GOT NAMED was decided by alphabetical
        # order. Add one fixture pose named APPROACH_BIN whose edges the
        # twin refuses and the report says the arm can never reach the
        # eleven crane poses, never mentioning the one that is actually
        # broken. Listing the components says the same thing without the
        # coin flip, and reads better besides.
        clean: dict[str, list[str]] = {n: [] for n in self.poses}
        back: dict[str, list[str]] = {n: [] for n in self.poses}
        for (f, t), v in verdicts.items():
            if v.clean:
                clean[f].append(t)
                back[t].append(f)

        def reach(adj: dict[str, list[str]], root: str) -> set[str]:
            seen, stack = {root}, [root]
            while stack:
                for nxt in adj[stack.pop()]:
                    if nxt not in seen:
                        seen.add(nxt)
                        stack.append(nxt)
            return seen

        # Strongly-connected components: two poses share one exactly when
        # each can reach the other, which is the property "the arm can go
        # there and come back" wants.
        islands: list[list[str]] = []
        placed: set[str] = set()
        for name in sorted(self.poses):
            if name in placed:
                continue
            group = sorted(reach(clean, name) & reach(back, name))
            placed.update(group)
            islands.append(group)
        islands.sort(key=lambda g: (-len(g), g[0]))

        ok = len(islands) <= 1
        if not ok:
            lines.append("")
            lines.append(f"UNREACHABLE — the library is not one connected "
                         f"workspace but {len(islands)} islands, and the arm "
                         f"cannot cross between them")
            for group in islands:
                lines.append(f"  {len(group)} pose(s): {', '.join(group)}")

        if oneway:
            lines.append("")
            # "via another pose" is only true when there IS one. In a
            # two-pose library a one-way edge means the way back does not
            # exist, and the island list above has already said so —
            # these two sections are computed separately and must not
            # contradict each other.
            # PER PAIR, not one verdict for the section. Whether there is
            # a way back is a property of the pair — same island means
            # some clean route exists — and an `all()` over the list let
            # one dead-end pair print " and there is no way back" above a
            # pair the island list two lines up says is fine.
            island_of = {n: i for i, g in enumerate(islands) for n in g}
            lines.append("one-way: the arm can make these moves but not "
                         "the reverse")
            for f, t in oneway:
                back_via = ("the way back is via another pose"
                            if island_of[f] == island_of[t]
                            else "THERE IS NO WAY BACK")
                lines.append(f"  {t} -> {f} is clear, {f} -> {t} is not — "
                             f"{back_via}")
        return "\n".join(lines), ok

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

    print("\nthe whole graph is answerable OFFLINE, before anyone stands "
          "at the bench")

    # A `want` detail must never be the thing that breaks. Computing it
    # eagerly — rep.splitlines()[1], [ln for ln in ... ][0] — turned a
    # REGRESSION into an IndexError traceback partway through the run,
    # so the check it was guarding printed no FAIL and every later check
    # never ran. The failure mode this file exists to catch was the one
    # it could not report.
    def first(lines: str, needle: str) -> str:
        return next((ln.strip() for ln in lines.splitlines()
                     if needle in ln), f"(no line containing {needle!r})")

    class FakeReport:
        def __init__(self, clean):
            self.clean = clean
            self.poses_checked = 3
            # TWO contacts, shallowest first, because the reporter must
            # name the worst rather than the first — see sim/edges.py.
            self.contacts = [] if clean else [
                type("C", (), {"body_a": "table", "body_b": "gripper",
                               "depth_mm": 0.0})(),
                type("C", (), {"body_a": "shoulder", "body_b": "gripper",
                               "depth_mm": 1.5})()]

    class ScriptedTwin:
        """Refuses exactly the edges named; clears everything else."""

        def __init__(self, refuse):
            self.refuse = refuse
            self.seen = 0

        def check_trajectory(self, frames):
            self.seen += 1
            # The edge is identified by its endpoints, which is all the
            # frames carry — first and last.
            key = (tuple(sorted(frames[0].items())),
                   tuple(sorted(frames[-1].items())))
            return FakeReport(key not in self.refuse)

    def scripted(poses_in, edges_in, refuse_pairs):
        refuse = {(tuple(sorted(poses_in[f].ticks.items())),
                   tuple(sorted(poses_in[t].ticks.items())))
                  for f, t in refuse_pairs}
        twin = ScriptedTwin(refuse)
        gate = FakeGate(MotionProfile(speed=300, acceleration=15))
        gate.twin = twin
        return PoseMachine(cals, poses_in, edges_in, gate=gate), twin

    # THE SHAPE OF THE REAL LIBRARY, in miniature: a direct move that the
    # geometry forbids, and a third pose that bridges it. This is not a
    # contrived fixture — REST <-> CRANE is refused on the real arm and
    # STAGE is what makes the library connected anyway.
    trio = {**poses, "STAGE": Pose("STAGE", {**rest, 3: rest[3] - 800})}
    trio_edges = [(a, b) for a in sorted(trio) for b in sorted(trio)
                  if a != b]
    m3, twin3 = scripted(trio, trio_edges, [("REST", "PAN"), ("PAN", "REST")])
    rep3, ok3 = m3.validation_report()
    want("a refused edge is reported at VALIDATION time, with no bus and "
         "no arm", "REST -/-> PAN" in rep3, first(rep3, "-/->"))
    want("...saying what hit what, not just that it said no",
         "shoulder <-> gripper" in rep3,
         first(rep3, "shoulder <-> gripper"))
    # GREPPED ON THE REPORT, not on a `first()` result. `first` returns a
    # placeholder when the needle is missing, and a placeholder contains
    # no "table" either — so the negative grep passed with the fix
    # reverted, printing the absence of evidence as its evidence. Every
    # other `first()` here is a POSITIVE grep and fails safe; this was
    # the one negative one.
    want("...naming the WORST contact, not the first one found — the "
         "fixture reports a 0.0 mm table graze ahead of the real fold",
         "shoulder <-> gripper" in rep3 and "table <-> gripper" not in rep3,
         first(rep3, "shoulder <-> gripper"))
    want("...and the clear edges are counted as clear",
         "4 clear, 2 refused" in rep3, first(rep3, "edge(s)"))
    want("...while a refusal alone is NOT a failure, because the bridge "
         "pose keeps both ends reachable", ok3)

    # Asserting the twin is not asked TWICE proves only that `validate`
    # memoises, which predates this method — the same assertion passes
    # with `validate_all` gutted to `return {}`. So check what the method
    # itself owes: an entry per edge, and the SAME verdict object the
    # guard will later read, so the two can never disagree.
    before = twin3.seen
    all3 = m3.validate_all()
    want("validate_all answers for every edge in the graph",
         set(all3) == set(m3.edges), f"{len(all3)}/{len(m3.edges)}")
    want("...handing back the very verdicts the guard reads, so an edge "
         "cannot be cleared here and re-judged there",
         all(all3[e] is m3.validate(*e) for e in m3.edges))
    want("...without re-simulating what it already knows",
         twin3.seen == before, f"{before} -> {twin3.seen}")

    m5, _ = scripted(trio, trio_edges, [("REST", "PAN")])
    rep5, ok5 = m5.validation_report()
    want("an edge refused in ONE direction only is called out as one-way",
         "one-way" in rep5 and "PAN -> REST is clear" in rep5,
         first(rep5, "is clear"))
    want("...and it is a warning, not a failure — a one-way edge is legal",
         ok5)
    want("...while a pair refused BOTH ways is not called one-way",
         "one-way" not in rep3)

    # Take the bridge away and the same two refusals DO wall a pose off.
    m4, _ = scripted(poses, edges, [("REST", "PAN"), ("PAN", "REST")])
    rep4, ok4 = m4.validation_report()
    want("with no route around it, a walled-off pose IS a failure", not ok4)
    want("...naming BOTH sides, so the report does not depend on which "
         "pose the walk happened to start from",
         "1 pose(s): PAN" in rep4 and "1 pose(s): REST" in rep4,
         first(rep4, "islands"))

    # THE PARTITION. Every pose still has a clean edge in and a clean
    # edge out, so the degree check this replaced said the library was
    # fine — while the arm could not cross between the two halves.
    split = {**trio, "FAR": Pose("FAR", {**rest, 1: rest[1] - 300})}
    split_edges = [(a, b) for a in sorted(split) for b in sorted(split)
                   if a != b]
    cut = [(a, b) for a in ("PAN", "STAGE") for b in ("FAR", "REST")]
    cut += [(b, a) for a, b in cut]
    m6, _ = scripted(split, split_edges, cut)
    rep6, ok6 = m6.validation_report()
    everyone_has_edges = all(
        any(t == n and v.clean for (f, t), v in m6.validate_all().items())
        and any(f == n and v.clean for (f, t), v in m6.validate_all().items())
        for n in split)
    want("a PARTITIONED graph is a failure even though every pose still "
         "has a clean edge in and out", not ok6 and everyone_has_edges)
    want("...saying the library is not one connected workspace",
         "not one connected workspace" in rep6, first(rep6, "UNREACHABLE"))
    want("...and listing BOTH islands, so the operator sees the shape of "
         "the cut rather than one side of it",
         "2 pose(s): PAN, STAGE" in rep6 and "2 pose(s): FAR, REST" in rep6,
         first(rep6, "pose(s): "))

    # BOTH KINDS OF ONE-WAY EDGE IN ONE REPORT. FAR is a SOURCE — every
    # edge INTO it is refused, so the arm can leave it and never return
    # to it — while REST -> PAN is refused with a clean route through
    # STAGE. A single verdict for the whole section printed "there is no
    # way back" over the pair that plainly had one, directly beneath an
    # island list showing the two together.
    mixed_cut = [("REST", "PAN")] + [(n, "FAR") for n in ("PAN", "REST",
                                                          "STAGE")]
    m8, _ = scripted(split, split_edges, mixed_cut)
    rep8, _ = m8.validation_report()
    ways = [ln.strip() for ln in rep8.splitlines() if " is clear," in ln]
    want("a one-way pair WITH a detour and one WITHOUT are described "
         "separately in the same report",
         any("PAN -> REST is clear" in w and "via another pose" in w
             for w in ways)
         and any("FAR -> REST is clear" in w and "NO WAY BACK" in w
                 for w in ways),
         " | ".join(ways[:2]))

    ungated = PoseMachine(cals, poses, edges)
    rep7, ok7 = ungated.validation_report()
    want("a gate-less machine refuses to pretend it validated anything",
         not ok7 and "no edge was checked" in rep7, rep7)
    try:
        ungated.validate_all()
        want("...and validate_all REFUSES rather than answering None for "
             "every edge, which a caller would read as a verdict", False)
    except BenchError as exc:
        want("...and validate_all REFUSES rather than answering None for "
             "every edge, which a caller would read as a verdict", True,
             str(exc))

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
    parser.add_argument("command", choices=("show", "validate", "selftest"))
    parser.add_argument("--cal", default="calibration.json")
    args = parser.parse_args()

    cal_path = Path(args.cal)
    if args.command == "selftest":
        return _selftest(cal_path)

    machine = load_machine(cal_path)
    if args.command == "validate":
        report, ok = machine.validation_report()
        print(report)
        print("\nno arm was touched — every verdict here came from the "
              "twin. A refused edge is the gate working; an UNREACHABLE "
              "pose is an authoring bug.", file=sys.stderr)
        return 0 if ok else 1
    print(machine.describe())
    print("\nno bus was opened — `at`/`allowed` need the encoders, and "
          "this command deliberately does not touch the arm.",
          file=sys.stderr)
    return 0


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    raise SystemExit(main())
