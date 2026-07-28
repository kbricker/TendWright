"""runner — execute a CLIP on the arm. The general motion executor.

Plan #660. Until now nothing could run an arbitrary planned sequence.
`jog` moves one joint per keypress, `teach replay` plays back frames a
human recorded by hand, and `exercise`/`batch` run one routine hard-coded
in Python. A pick-and-place has none of those shapes: it is a sequence
COMPUTED at run time — approach, descend, close, lift, transit, place,
retreat — where the poses come from IK against whatever the camera just
saw. This is the thing that runs it.

(Not to be confused with `sim/runner.py`, which steps the P0 MuJoCo
StepProgram. This one drives servos.)

WHAT IT ADDS OVER THE PARTS IT COMPOSES. Almost everything here already
existed: `sim.clip` defines and samples a clip, `posegate` judges one,
`motion.wait_settle` waits for arrival, `guards.StrainWatch` is the
automatic e-stop, `sim.trace` records. The genuinely new decision is
WHERE THE GATE RUNS, and the answer is: in two places, for two different
questions.

  1. THE WHOLE CLIP, UP FRONT — including the approach to pose 0 from
     wherever the arm is actually sitting. This is the operator's
     question ("may I run this at all?") and it must be answered before
     anything energizes, because a refusal partway through a sequence
     leaves the arm somewhere nobody planned. `teach replay`'s pattern,
     not `jog`'s.

  2. EACH EDGE, FROM THE MEASURED POSE — because the up-front verdict
     provably does not cover the path actually taken. `wait_settle`
     accepts arrival within SETTLE_TOL_TICKS (25), and 25 ticks of error
     on every joint at once is up to 25.8 mm at the tool (measured
     against this arm's model, mid-range pose) versus the gate's 5 mm
     contact margin — FIVE TIMES the margin the up-front verdict was
     computed with. So each edge is re-gated from where the arm really
     is. It costs a fraction of a second and it is the difference
     between gating the plan and gating the motion.

     This is `guards.py`'s lesson at clip scale: a command sent is not a
     joint moved, so verify reality against the plan rather than
     assuming the plan happened.

WHAT HAPPENS MID-CLIP WHEN SOMETHING REFUSES. It halts every joint in
place, holds under torque, and STOPS. It does not attempt to return to
rest, to the previous pose, or to anything else. Three reasons, and they
all say the same thing:

  * The condition that stopped it — an obstruction, a jammed joint, a
    strain trip — is still there. Driving back through it is driving
    into it again.
  * No path back was ever gated. The return move is exactly as
    unchecked as the move that failed.
  * A stop is when the operator most needs the arm to STAY PUT so they
    can see what happened.

Recovery is deliberately a separate, human decision: `jog` is gated
per-step and exists for precisely this. The stop report says so.

A clip may also END anywhere, which `exercise` never does — so torque is
not cut silently at the end either, unless the final pose is near rest
(where letting go is what gravity was already doing).

    uv run python -m hardware.bench.runner example > pick.json
    uv run python -m hardware.bench.runner show --clip pick.json
    uv run python -m hardware.bench.runner run  --clip pick.json --trace runs/
    uv run python -m hardware.bench.runner selftest

ANY key during motion is an E-STOP. Exit codes: 0 done, 1 aborted,
2 error, 3 stopped mid-clip (e-stop, strain trip, or gate refusal),
130 Ctrl+C.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial

from hardware.units import fmt_ticks, span_deg

from .bus import BenchError, FeetechBus, confirm, run_tool
from .calibrate import JointCal, load_calibration
from .guards import StrainWatch
from .motion import EStop, halt_all, wait_settle

# The move to pose 0 is the one nobody authored: it starts from wherever
# the arm happens to be. Slow, for the same reason `teach replay` is.
APPROACH_SPEED_TICKS = 120

# A settled joint is within motion.SETTLE_TOL_TICKS of its target, which
# on this arm is up to ~25.8 mm at the tool if every joint is off at once
# — five times the gate's 5 mm contact margin. Reported when it happens
# so the number is visible rather than folklore, and it is why each edge
# is re-gated instead of trusting the up-front verdict.
DRIFT_REPORT_TICKS = 10


@dataclass
class EdgeResult:
    """What one edge of the clip actually did."""

    index: int
    frm: str
    to: str
    seconds: float
    drift_ticks: int = 0        # how far off-plan the arm was at entry
    regated_poses: int = 0      # poses the per-edge gate checked


@dataclass
class RunOutcome:
    """A completed run. A stopped one raises ClipStopped instead."""

    clip: str
    edges: list[EdgeResult] = field(default_factory=list)
    seconds: float = 0.0
    worst_drift_ticks: int = 0

    def summary(self) -> str:
        return (f"clip '{self.clip}': {len(self.edges)} edge(s) in "
                f"{self.seconds:.1f}s, worst off-plan drift "
                f"{span_deg(self.worst_drift_ticks):.1f} deg "
                f"({self.worst_drift_ticks}t)")


class ClipStopped(BenchError):
    """The clip did not finish. Carries WHERE, so the operator is not
    left guessing which pose the arm is holding between.

    Deliberately not a subclass of the specific fault (strain, e-stop,
    settle failure) — the caller almost always wants the same response
    regardless of which one fired, and the original is on __cause__ for
    the cases that don't."""

    def __init__(self, clip_name: str, edge: int, total: int, frm: str,
                 to: str, why: str, measured: dict[int, int],
                 cals: dict[int, JointCal] | None = None,
                 hint: str | None = None):
        self.clip_name = clip_name
        self.edge = edge
        self.total = total
        self.frm = frm
        self.to = to
        self.why = why
        self.measured = dict(measured)
        super().__init__(
            f"clip '{clip_name}' stopped on edge {edge}/{total} "
            f"({frm} -> {to}): {why}",
            hint or ("the arm is HOLDING between poses — it was not "
                     "returned to rest, because the path back was never "
                     "gated and whatever stopped it may still be there. "
                     "Use `jog` (which gates every step) to walk it "
                     "somewhere safe."))
        self.cals = cals

    def where(self) -> str:
        """The stop pose in human units — what to read before touching
        anything."""
        if not self.measured:
            return "  (no pose was read)"
        return "\n".join(
            f"    joint {i} ({self.cals[i].name if self.cals else '?'}): "
            f"{fmt_ticks(self.cals[i].frame if self.cals else None, t)}"
            for i, t in sorted(self.measured.items()))


# ------------------------------------------------------------- planning
def describe_clip(clip, cals: dict[int, JointCal]) -> str:
    """The plan, in human units, before anything moves."""
    from sim.clip import clip_duration, edge_duration

    lines = [f"clip '{clip.name}': {len(clip.poses)} poses, "
             f"{len(clip.edges())} edge(s), "
             f"{clip_duration(clip):.1f}s at speed "
             f"{clip.profile.speed} accel {clip.profile.acceleration}"]
    for n, (a, b) in enumerate(clip.edges(), start=1):
        moved = [i for i in sorted(b.ticks) if b.ticks[i] != a.ticks.get(i)]
        parts = []
        for i in moved:
            f = cals[i].frame if i in cals else None
            # A joint the previous pose does not pin has no "from" to
            # print. Saying so beats printing tick 0, which is a real
            # position and would read as one.
            frm = fmt_ticks(f, a.ticks[i]) if i in a.ticks else "(unpinned)"
            parts.append(f"j{i} {frm} -> {fmt_ticks(f, b.ticks[i])}")
        lines.append(f"  [{n}] {a.name} -> {b.name}  "
                     f"({edge_duration(clip.profile, a, b):.1f}s)  "
                     f"{', '.join(parts) or 'no movement'}")
    return "\n".join(lines)


def gate_clip(gate, start: dict[int, int], clip) -> 'Verdict':  # noqa: F821
    """Pre-flight the WHOLE clip, approach included.

    `start` leads the sequence because the move from the arm's present
    pose to pose 0 is real motion that no one authored — the same move
    `teach replay` learned to check, and the one most likely to surprise
    because it depends on where the arm was left.

    Resolving first so that a clip pinning only some joints is gated as
    the motion it actually is, rather than as one where the unpinned
    joints snap back to `start` at every pose."""
    resolved = clip.resolved(start)
    return gate.check_sequence(
        [dict(start)] + [dict(p.ticks) for p in resolved.poses],
        label=clip.name)


# ------------------------------------------------------------- execution
def _drift(a: dict[int, int], b: dict[int, int]) -> int:
    """Worst per-joint difference between two poses, in ticks."""
    return max((abs(a[i] - b.get(i, a[i])) for i in a), default=0)


def run_clip(bus, cals: dict[int, JointCal], clip, *,
             gate=None,
             strain: StrainWatch | None = None,
             trace=None,
             poll_key=None,
             approach_speed: int = APPROACH_SPEED_TICKS,
             on_edge=None,
             quiet: bool = False) -> RunOutcome:
    """Drive the arm through `clip`. The library entry point.

    Assumes the caller has ALREADY pre-flighted with `gate_clip` and
    taken the operator's confirmation — this function is the executor,
    not the interview. It still re-gates each edge from the measured
    pose (see the module docstring) because that is a different question
    from the one the pre-flight answered.

    TORQUE IS THE CALLER'S. On success the arm is left HOLDING at the
    final pose: a pick-place sequence runs several clips back to back
    and cutting torque between them would drop whatever is in the
    gripper. On failure every joint is halted in place and the exception
    propagates with torque still on — same rule as `wait_settle`, and
    for the same reason: dropping an arm on an unwarned operator is
    worse than any tidiness this function could offer.

    `on_edge(n, total, pose)` is called with the MEASURED pose just
    before each edge is commanded — an observation point for logging or
    a camera checkpoint. It is deliberately not allowed to change the
    clip: a sequence that rewrites itself mid-run is a sequence the
    pre-flight gate did not check.
    """
    ids = sorted(cals)
    profile = clip.profile
    started = time.monotonic()
    out = RunOutcome(clip=clip.name)

    # Resolve against the arm's ACTUAL pose, once, before anything is
    # gated or commanded — so the gate and the servos are handed the
    # same complete poses. See Clip.resolved.
    clip = clip.resolved({i: bus.read_position(i) for i in ids})
    edges = clip.edges()

    def say(msg: str) -> None:
        if not quiet:
            print(msg)

    def measure() -> dict[int, int]:
        return {i: bus.read_position(i) for i in ids}

    def stop(edge_no: int, frm: str, to: str, why: str,
             hint: str | None = None, halt: bool = True) -> 'ClipStopped':
        """Halt, then build the stop report. Halting FIRST: the arm
        stopping is never delayed by bookkeeping.

        `halt=False` for a refusal raised BEFORE the edge was commanded.
        The arm is already stationary and holding the goal it settled
        on, so re-goaling would be writes with nothing to stop — and it
        would blur the property #699 established for `jog`: a refused
        move sends the servos nothing at all."""
        if halt:
            try:
                halt_all(bus, ids)
            except (BenchError, serial.SerialException, KeyboardInterrupt):
                pass
        try:
            measured = measure()
        except (BenchError, serial.SerialException):
            measured = {}
        return ClipStopped(clip.name, edge_no, len(edges), frm, to, why,
                           measured, cals, hint)

    def invariant() -> None:
        if strain is not None:
            strain.check(bus)

    sink = trace.sample if trace is not None else None

    # --- approach: get to pose 0 before the clip proper begins.
    first = clip.poses[0]
    here = measure()
    approach_drift = _drift(here, first.ticks)
    if approach_drift:
        say(f"approach: moving to '{first.name}' "
            f"({span_deg(approach_drift):.1f} deg / {approach_drift}t away)")
        for i, t in sorted(first.ticks.items()):
            bus.move_to(i, t, speed=approach_speed,
                        acceleration=profile.acceleration)
        if trace is not None:
            trace.phase(f"approach {first.name}", edge=0)
        try:
            wait_settle(bus, dict(first.ticks), approach_speed,
                        f"approach {first.name}", poll_key=poll_key,
                        require_still=False, invariant=invariant,
                        sample_sink=sink)
        except EStop as exc:
            raise stop(0, "start", first.name, "operator e-stop") from exc
        except BenchError as exc:
            raise stop(0, "start", first.name, str(exc)) from exc

    # --- the clip proper, one edge at a time.
    for n, (a, b) in enumerate(edges, start=1):
        here = measure()

        # Where the arm actually is, versus where the plan says it
        # should be. Reported, not merely tolerated — an edge that
        # consistently enters off-plan is telling you something about
        # the joint, and silence would hide it.
        drift = _drift(here, a.ticks)
        out.worst_drift_ticks = max(out.worst_drift_ticks, drift)
        if drift >= DRIFT_REPORT_TICKS and not quiet:
            off = [i for i in sorted(a.ticks)
                   if abs(here[i] - a.ticks[i]) >= DRIFT_REPORT_TICKS]
            say(f"  off-plan at edge {n} by {span_deg(drift):.1f} deg "
                f"({drift}t) on joint(s) {off} — re-gating from the "
                f"measured pose")

        # THE PER-EDGE GATE. From the measured pose, never the planned
        # one: gating the plan against itself would always pass.
        regated = 0
        if gate is not None and gate.active:
            verdict = gate.check_sequence([here, {**here, **b.ticks}],
                                          label=f"{clip.name}[{n}]")
            regated = verdict.poses_checked
            if verdict.refused:
                raise stop(n, a.name, b.name,
                           f"the collision gate refused this edge from the "
                           f"arm's ACTUAL pose — {verdict.detail}",
                           "the whole clip was clear from its planned "
                           "start, so the arm is somewhere the next move "
                           "is not safe from; read the pose below and "
                           "`jog` clear of it",
                           halt=False)

        if on_edge is not None:
            on_edge(n, len(edges), dict(here))

        say(f"[{n}/{len(edges)}] {a.name} -> {b.name}")
        edge_started = time.monotonic()
        for i, t in sorted(b.ticks.items()):
            bus.move_to(i, t, speed=profile.speed,
                        acceleration=profile.acceleration)
        if trace is not None:
            trace.phase(f"{a.name}->{b.name}", edge=n)
        try:
            wait_settle(bus, dict(b.ticks), profile.speed, b.name,
                        poll_key=poll_key, invariant=invariant,
                        sample_sink=sink)
        except EStop as exc:
            raise stop(n, a.name, b.name, "operator e-stop") from exc
        except BenchError as exc:
            # Strain trips, guard violations and settle failures all land
            # here. wait_settle has already halted; `stop` halting again
            # is harmless and covers the paths that have not.
            raise stop(n, a.name, b.name, str(exc)) from exc

        out.edges.append(EdgeResult(
            index=n, frm=a.name, to=b.name,
            seconds=time.monotonic() - edge_started,
            drift_ticks=drift, regated_poses=regated))

    out.seconds = time.monotonic() - started
    return out


# ------------------------------------------------------------------ CLI
_EXAMPLE_COMMENT = (
    "Poses are DEGREES (gripper: percent open). The first pose must name "
    "every calibrated joint; later poses name only what changes and inherit "
    "the rest. `pose` references a name from poses.json instead of `joints`.")

# The fallback, for a machine with no calibration to anchor to. Note it
# is a whole-arm UNFOLD from a folded rest — fine as a shape to copy,
# not something to run blind, which is why the anchored version below is
# preferred whenever a calibration exists.
EXAMPLE = """{
  "version": 1,
  "name": "example",
  "_comment": "%s",
  "_warning": "GENERIC TEMPLATE — these angles are not anchored to any calibration. Run `runner example` on a machine WITH calibration.json for a clip anchored to this arm's rest pose, and always `runner show` before `runner run`.",
  "profile": {"speed": 300, "acceleration": 15},
  "poses": [
    {"name": "home",  "joints": {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 50}},
    {"name": "left",  "joints": {"1": -20}},
    {"name": "right", "joints": {"1": 20}},
    {"name": "home2", "joints": {"1": 0}}
  ]
}
""" % _EXAMPLE_COMMENT

EXAMPLE_PAN_DEG = 15.0


def example_clip_doc(cals: dict[int, JointCal] | None) -> str:
    """A starter clip anchored to THIS arm's rest pose.

    Rest is a compact fold, and panning the base with the arm folded is
    the lowest-inertia move the arm has — the same reasoning that puts
    joint 1 last in `exercise`'s distal-first sweep order. A first
    example that unfolded the whole arm would be a worse thing to hand
    someone whose next command is `run`.
    """
    import json

    from hardware.units import DegFrame

    if not cals:
        return EXAMPLE

    def human(i: int, tick: int) -> float:
        f = cals[i].frame
        return f.deg(tick) if isinstance(f, DegFrame) else f.pct(tick)

    # Two decimals: 0.01 deg is an eighth of a tick (0.088 deg), so the
    # written value converts back to exactly the tick it came from.
    rest = {}
    for i, c in sorted(cals.items()):
        if c.frame is None:
            return EXAMPLE            # cannot author in human units
        rest[str(i)] = round(human(i, c.rest), 2)

    pan_id = min(cals)
    lo, hi = sorted((human(pan_id, cals[pan_id].min),
                     human(pan_id, cals[pan_id].max)))
    base = rest[str(pan_id)]
    left = round(max(lo, min(hi, base + EXAMPLE_PAN_DEG)), 2)
    right = round(max(lo, min(hi, base - EXAMPLE_PAN_DEG)), 2)

    return json.dumps({
        "version": 1,
        "name": "pan-wiggle",
        "_comment": _EXAMPLE_COMMENT,
        "_anchored": (f"Generated from calibration.json: pose 'rest' IS this "
                      f"arm's captured rest pose, and joint {pan_id} pans "
                      f"+/-{EXAMPLE_PAN_DEG:g} deg from it with the arm "
                      f"folded. Re-generate after re-calibrating. Always "
                      f"`runner show` before `runner run`."),
        "profile": {"speed": 250, "acceleration": 12},
        "poses": [
            {"name": "rest", "joints": rest},
            {"name": "left", "joints": {str(pan_id): left}},
            {"name": "right", "joints": {str(pan_id): right}},
            {"name": "home", "joints": {str(pan_id): base}},
        ],
    }, indent=2) + "\n"


def _load(args) -> tuple[dict[int, JointCal], 'Clip']:  # noqa: F821
    from sim.clip import load_clip

    cal_path = Path(args.cal)
    if not cal_path.exists():
        raise BenchError(f"no calibration at {cal_path}",
                         "run `calibrate capture` first — a clip is authored "
                         "in degrees, which only a calibration can convert")
    cals = load_calibration(cal_path)
    return cals, load_clip(cals, Path(args.clip))


def cmd_show(args) -> int:
    """The plan and the gate verdict, without touching the bus."""
    from .posegate import PoseGate

    cals, clip = _load(args)
    print(describe_clip(clip, cals))

    gate = PoseGate(sorted(cals), args.cal, profile=clip.profile)
    print()
    print(gate.banner())
    if gate.active:
        # No arm to read, so gate from the REST pose: this checks the
        # AUTHORED motion. The approach from the arm's real position can
        # only be checked at the bench, and `run` does it.
        rest = {i: c.rest for i, c in cals.items()}
        verdict = gate.check_sequence(
            [rest] + [{**rest, **p.ticks} for p in clip.poses],
            label=clip.name)
        print(f"from the REST pose: {verdict.detail}")
        print("  (`run` re-checks from the arm's actual position, and "
              "re-gates every edge as it goes)")
    return 0


def cmd_run(args) -> int:
    from .posegate import PoseGate
    from .term import read_key, require_interactive

    cals, clip = _load(args)
    ids = sorted(cals)

    if args.unattended:
        # Said out loud on every run, because what is given up is the
        # operator's own abort channel. A flag rather than an isatty()
        # test: `ssh -tt` forces a pty that reports a terminal with no
        # human behind it, so only the operator can know this.
        print("UNATTENDED — no keypress e-stop. The collision gate and "
              "strain guard still run; the POWER SWITCH is the only "
              "human abort.", file=sys.stderr)
    elif not args.yes:
        require_interactive()

    with FeetechBus(args.port) as bus:
        missing = [i for i in ids if bus.ping(i) is None]
        if missing:
            raise BenchError(f"no answer from servo IDs {missing}",
                             "a clip drives every calibrated joint; run "
                             "`scan` to see what the bus can hear")

        print(describe_clip(clip, cals))

        gate = PoseGate(ids, args.cal, profile=clip.profile)
        print()
        print(gate.banner() if not args.no_gate else
              "collision gate SKIPPED (--no-gate) — this clip was NOT checked")

        if not args.no_gate and gate.active:
            start = {i: bus.read_position(i) for i in ids}
            verdict = gate_clip(gate, start, clip)
            print(f"pre-flight: {verdict.detail}")
            if verdict.refused:
                if not args.force:
                    print("not running — re-run with --force to override")
                    return 1
                print("FORCED past the gate — the arm can hit itself",
                      file=sys.stderr)
        print("  ANY key during motion is an E-STOP (halt + hold). "
              "The power switch is the hard e-stop.")

        if not args.yes and not confirm("clear the workspace, then type y "
                                        "to run: "):
            print("aborted")
            return 1

        # The verdict above described the arm as it was BEFORE a prompt
        # that can sit open for minutes. Re-check it against the arm as
        # it is now — someone may have moved it while deciding. Silent
        # unless it changes the answer (`exercise` re-vets its start
        # pose here for the same reason).
        if not args.no_gate and gate.active:
            again = gate_clip(gate, {i: bus.read_position(i) for i in ids},
                              clip)
            if again.refused and not args.force:
                print(f"the arm moved while you were deciding — {again.detail}",
                      file=sys.stderr)
                return 1

        trace = None
        if args.trace:
            from sim.trace import Trace
            dest = Path(args.trace)
            if dest.is_dir() or args.trace.endswith(("/", "\\")):
                stamp = time.strftime("%Y%m%d-%H%M%S")
                dest = dest / (f"{clip.name}-{stamp}"
                               f"-sp{clip.profile.speed}"
                               f"-ac{clip.profile.acceleration}.csv")
            trace = Trace(dest, meta={"speed": clip.profile.speed,
                                      "accel": clip.profile.acceleration,
                                      "clip": clip.name,
                                      "cal": str(args.cal)})
            print(f"tracing the arm's actual path to {dest}")

        strain = StrainWatch(ids)
        try:
            # Wake without lurch: pre-load each goal to the CURRENT
            # position while still torque-off, then enable. Enabling
            # against a stale goal register snaps the joint.
            print("\nwaking (torque on, holding in place)...")
            for i in ids:
                bus.move_to(i, bus.read_position(i),
                            speed=APPROACH_SPEED_TICKS,
                            acceleration=clip.profile.acceleration)
                bus.set_torque(i, True)

            outcome = run_clip(
                bus, cals, clip,
                gate=None if args.no_gate else gate,
                strain=strain, trace=trace,
                poll_key=read_key)
            print(f"\n{outcome.summary()}")
            print(strain.summary())
            # A clip may END ANYWHERE — unlike `exercise`, which always
            # finishes at rest. Cutting torque on an arm holding itself
            # mid-air drops it, so ask first unless the final pose is
            # one gravity would hold anyway.
            if _near_rest(bus, cals):
                print("clip complete — the arm is near its rest pose, "
                      "cutting torque")
            else:
                print("clip complete — the arm is HOLDING away from rest")
                _held_torque_cut(args.unattended)
            return 0
        except ClipStopped as exc:
            print(f"\nSTOPPED: {exc}", file=sys.stderr)
            print("the arm is holding HERE:", file=sys.stderr)
            print(exc.where(), file=sys.stderr)
            print(strain.summary(), file=sys.stderr)
            _held_torque_cut(args.unattended)
            return 3
        except (BenchError, serial.SerialException):
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            _held_torque_cut(args.unattended)
            raise
        except KeyboardInterrupt:
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            _held_torque_cut(args.unattended)
            raise
        finally:
            bus.safe_torque_off(ids)
            if trace is not None:
                written = trace.close()
                if written is not None:
                    print(f"trace: {len(trace)} samples -> {written}")
                    print(f"  compare: uv run python -m sim.trace {written}")
                elif trace.error:
                    print(f"trace: {trace.error}", file=sys.stderr)


def _near_rest(bus, cals: dict[int, JointCal]) -> bool:
    """Is the arm somewhere torque-off is a no-op rather than a drop?

    The rest pose IS the torque-off slump — where gravity already puts
    the arm — so letting go there moves nothing. The tolerance is
    `exercise`'s PREFLIGHT_REST_TOL_TICKS, which answers the mirror
    question (how far from rest a routine may START), and a number that
    means "close enough to rest to be uneventful" should not have two
    different values in one toolkit.

    A read that fails counts as NOT near rest: the safe answer when the
    arm's position is unknown is to warn."""
    from .exercise import PREFLIGHT_REST_TOL_TICKS
    try:
        return all(abs(bus.read_position(i) - c.rest)
                   <= PREFLIGHT_REST_TOL_TICKS for i, c in cals.items())
    except (BenchError, serial.SerialException):
        return False


def _held_torque_cut(unattended: bool = False) -> None:
    """Never drop a holding arm on an unwarned operator.

    Unless there is no operator: on a remote run the prompt has no
    audience, so waiting only keeps the servos energized behind a
    question nobody will answer, and the run cannot report until
    something kills it. Cut, and say why."""
    from .term import flush_input
    if unattended:
        print("\nthe arm was HOLDING under torque; cutting now "
              "(--unattended: nobody is at this terminal to warn).",
              file=sys.stderr)
        return
    try:
        print("\nthe arm is HOLDING under torque. get a hand on it — it "
              "drops when torque cuts.", file=sys.stderr)
        flush_input()
        input("press Enter to cut torque: ")
    except (EOFError, KeyboardInterrupt):
        pass


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.runner",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p):
        p.add_argument("--clip", required=True, help="clip JSON file")
        p.add_argument("--cal", default="calibration.json")
        return p

    common(sub.add_parser("show", help="print the plan and gate it, "
                                       "without touching the arm"))
    p_run = common(sub.add_parser("run", help="execute the clip on the arm"))
    p_run.add_argument("--port", default=None, help="serial port override")
    p_run.add_argument("--trace", metavar="FILE_OR_DIR", default=None,
                       help="record the arm's actual path (a directory "
                            "auto-names, so runs accumulate)")
    p_run.add_argument("--yes", action="store_true", help="skip confirmation")
    p_run.add_argument("--unattended", action="store_true",
                       help="nobody is at this terminal: implies --yes, "
                            "and on a fault the arm's torque is cut "
                            "immediately instead of waiting for someone to "
                            "get a hand on it. There is NO keypress e-stop "
                            "in this mode — the power switch and the "
                            "automatic guards are the only aborts")
    p_run.add_argument("--force", action="store_true",
                       help="run a clip the gate refuses (logged; the arm "
                            "can hit itself)")
    p_run.add_argument("--no-gate", action="store_true",
                       help="skip the collision gate entirely")

    sub.add_parser("example",
                   help="print a starter clip, anchored to this arm's rest "
                        "pose when a calibration is available"
                   ).add_argument("--cal", default="calibration.json")

    args = parser.parse_args()
    # A declared-unattended run has no operator to confirm anything.
    args.yes = getattr(args, "yes", False) or getattr(args, "unattended", False)
    if args.command == "example":
        cal_path = Path(args.cal)
        cals = load_calibration(cal_path) if cal_path.exists() else None
        print(example_clip_doc(cals), end="")
        return 0
    return cmd_show(args) if args.command == "show" else cmd_run(args)


# ------------------------------------------------------------- selftest
def _selftest() -> int:
    """Every acceptance paired with a refusal, against a fake bus.

    The properties that matter are all about what NEVER reaches the
    servos: a refused clip must not command a single move, and a clip
    stopped partway must not command the edges after the stop."""
    import contextlib
    import io

    from hardware.units import DegFrame
    from sim.clip import Clip, MotionProfile, Pose, load_clip

    from .guards import StrainViolation
    from .posegate import PoseGate

    fails: list[str] = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not cond:
            fails.append(name)

    cal_path = Path("calibration.json")
    if not cal_path.exists():
        print("  no calibration.json here — cannot exercise the runner")
        return 1

    cals = load_calibration(cal_path)
    rest = {i: c.rest for i, c in cals.items()}
    ids = sorted(cals)
    profile = MotionProfile(speed=300, acceleration=15)

    def human(i: int, tick: int) -> float:
        """Ticks -> whatever unit this joint's frame is authored in."""
        f = cals[i].frame
        return f.deg(tick) if isinstance(f, DegFrame) else f.pct(tick)

    class FakeBus:
        """An arm that goes exactly where it is told, immediately.

        `stuck` pins a joint's REPORTED position regardless of what it
        was commanded — the plant disagreeing with the plan. `trip_at`
        makes the Nth health read report an overload."""

        def __init__(self, start, stuck=None, trip_at=None):
            self.pos = dict(start)
            self.stuck = dict(stuck or {})
            self.trip_at = trip_at
            self.moves: list[tuple[int, int, int, int]] = []
            self.health_reads = 0

        def ping(self, i):
            return 1

        def read_position(self, i):
            return self.stuck.get(i, self.pos[i])

        def move_to(self, i, tick, speed=400, acceleration=30):
            self.moves.append((i, tick, speed, acceleration))
            self.pos[i] = tick

        def read_health(self, i):
            self.health_reads += 1
            over = (self.trip_at is not None
                    and self.health_reads >= self.trip_at)
            return {"id": i, "load_pct": 95.0 if over else 4.0,
                    "current_ma": 100.0, "temp_c": 30, "volts": 7.4,
                    "status": 0, "faults": [], "plausible": True}

        def set_torque(self, i, on):
            pass

    def clip_of(seq, name="t"):
        return Clip(name, [Pose(n, dict(p)) for n, p in seq], profile)

    def run_quiet(*a, **kw):
        """wait_settle prints a live progress line; the selftest's own
        output is the thing being read here."""
        with contextlib.redirect_stdout(io.StringIO()):
            return run_clip(*a, **kw)

    gate = PoseGate(ids, cal_path, profile=profile)
    if not gate.active:
        print(f"  gate inactive ({gate.reason}) — cannot exercise the runner")
        return 1

    # ---------------------------------------------------------------
    print("a clean clip runs, and runs with the CLIP's profile numbers")
    pan = {**rest, 1: rest[1] + 60}
    clean = clip_of([("home", rest), ("pan", pan), ("home2", rest)])
    bus = FakeBus(rest)
    out = run_quiet(bus, cals, clean, gate=gate, strain=StrainWatch(ids),
                    quiet=True)
    check("both edges ran", len(out.edges) == 2, out.summary())
    check("the servos got the clip's speed/accel, not a default",
          bool(bus.moves)
          and all(s == profile.speed and a == profile.acceleration
                  for _, _, s, a in bus.moves),
          f"{len(bus.moves)} commands")
    check("...and every edge really was re-gated on the way through",
          all(e.regated_poses > 0 for e in out.edges),
          f"{[e.regated_poses for e in out.edges]} poses per edge")

    # ---------------------------------------------------------------
    print("\nthe up-front gate refuses a colliding clip")
    from .exercise import sweep_window
    _, hi2 = sweep_window(cals[2], 70)
    bad = clip_of([("home", rest), ("fold", {**rest, 2: hi2})])
    v = gate_clip(gate, rest, bad)
    check("the run-1 folded-elbow sweep is REFUSED", v.refused, v.detail)
    check("...and it names the colliding links", "<->" in v.detail)
    v_ok = gate_clip(gate, rest, clean)
    check("...while the clean clip passes the same call", v_ok.allowed,
          v_ok.detail)

    print("\n...and the approach to pose 0 is gated too, not just the clip")
    # A clip whose own poses are fine, entered from a start pose that
    # cannot reach pose 0 safely. This is the move nobody authors.
    away = {**rest, 2: hi2}
    v = gate_clip(gate, away, clean)
    check("a clip entered from a bad start pose is REFUSED", v.refused,
          v.detail)

    # ---------------------------------------------------------------
    print("\nthe PER-EDGE gate catches an arm that is not where the plan "
          "assumes")
    # THE case the per-edge gate exists for, and it is not hypothetical:
    # a clip built by IK pins only the joints it needs to move (here,
    # the pan), leaving the rest wherever they are. The authored motion
    # is clean and the up-front gate says so — but joint 2 is physically
    # folded, which only reading the arm can reveal.
    partial = Clip("partial", [Pose("a", {1: rest[1]}),
                               Pose("b", {1: rest[1] + 60})], profile)
    v = gate_clip(gate, rest, partial)
    check("the authored motion passes the up-front gate", v.allowed, v.detail)
    bus = FakeBus(rest, stuck={2: hi2})
    try:
        run_quiet(bus, cals, partial, gate=gate, quiet=True)
        check("...but a folded joint 2 stops it mid-clip", False,
              "the runner ran it anyway")
    except ClipStopped as exc:
        check("...but a folded joint 2 stops it mid-clip", True,
              str(exc)[:100])
        check("...on a real edge, not the approach", exc.edge >= 1,
              f"edge {exc.edge}/{exc.total}")
        check("...blaming the gate, not a settle timeout",
              "collision gate refused" in exc.why, exc.why[:80])
        check("...and nothing was ever commanded", not bus.moves,
              f"{len(bus.moves)} commands issued")
        check("...reporting where the arm is holding", "joint 2" in exc.where())
        check("...and pointing at jog for recovery, not auto-returning",
              "jog" in (exc.hint or ""))

    # ---------------------------------------------------------------
    print("\na strain trip stops the clip and commands NOTHING after it")
    long_clip = clip_of([("home", rest), ("pan", pan), ("home2", rest),
                         ("pan2", pan), ("home3", rest)])
    bus = FakeBus(rest, trip_at=2)
    try:
        run_quiet(bus, cals, long_clip, gate=gate, strain=StrainWatch(ids),
                  quiet=True)
        check("a strain trip stops the clip", False, "it ran to completion")
    except ClipStopped as exc:
        check("a strain trip stops the clip", True, str(exc)[:100])
        check("...on an early edge, not at the end",
              exc.edge < len(long_clip.edges()),
              f"stopped on edge {exc.edge} of {len(long_clip.edges())}")
        check("...with the servo fault preserved as the cause",
              isinstance(exc.__cause__, StrainViolation),
              type(exc.__cause__).__name__)
        # The headline: the remaining edges were never commanded. Count
        # DISTINCT goal positions per joint — a halt re-goals in place,
        # which is a move_to we must not mistake for progress.
        commanded = {t for i, t, _, _ in bus.moves if i == 1}
        check("...and the later edges were never commanded",
              len(commanded) <= 2,
              f"joint 1 saw {len(commanded)} distinct goals for "
              f"{len(long_clip.edges())} edges")

    # ---------------------------------------------------------------
    print("\nan e-stop keypress stops it the same way")
    bus = FakeBus(rest)
    pressed = {"n": 0}

    def press(_timeout):
        pressed["n"] += 1
        return "x" if pressed["n"] >= 2 else None

    try:
        run_quiet(bus, cals, long_clip, gate=gate, poll_key=press, quiet=True)
        check("an e-stop stops the clip", False, "it ran to completion")
    except ClipStopped as exc:
        check("an e-stop stops the clip", True, str(exc)[:100])
        check("...and says so plainly", "e-stop" in exc.why, exc.why)

    # ---------------------------------------------------------------
    print("\nthe end of a clip is not assumed safe to de-energize")
    from .exercise import PREFLIGHT_REST_TOL_TICKS
    check("at rest, torque-off is a no-op and needs no warning",
          _near_rest(FakeBus(rest), cals))
    raised = {**rest, 2: rest[2] + PREFLIGHT_REST_TOL_TICKS + 1}
    check("...but an arm holding away from rest is warned about first",
          not _near_rest(FakeBus(raised), cals),
          f"joint 2 {PREFLIGHT_REST_TOL_TICKS + 1}t off rest")

    class DeadBus(FakeBus):
        def read_position(self, i):
            raise BenchError("bus gone")

    check("...and an unreadable arm counts as NOT safe, never as safe",
          not _near_rest(DeadBus(rest), cals))

    # ---------------------------------------------------------------
    print("\nthe clip FILE format")
    import json
    import tempfile

    with tempfile.TemporaryDirectory() as td:
        d = Path(td)

        def write(body, name="c.json"):
            p = d / name
            p.write_text(json.dumps(body))
            return p

        # A first pose built from the REST ticks, so it is certainly in
        # range, expressed in the human units a clip file is authored in.
        first = {str(i): human(i, rest[i]) for i in ids}
        ok_doc = {"version": 1, "name": "file-clip",
                  "profile": {"speed": 250, "acceleration": 12},
                  "poses": [{"name": "a", "joints": first},
                            {"name": "b", "joints": {"1": first["1"] + 5}}]}
        c = load_clip(cals, write(ok_doc))
        check("a valid clip file loads", len(c.poses) == 2, c.name)
        check("...carrying its own profile", c.profile.speed == 250)
        check("...and later poses inherit the joints they omit",
              set(c.poses[1].ticks) == set(ids),
              f"{len(c.poses[1].ticks)} joints")

        def refuses(label, body):
            try:
                load_clip(cals, write(body, f"{abs(hash(label)) % 9999}.json"))
                check(label, False, "it loaded")
            except BenchError as exc:
                check(label, True, str(exc)[:90])

        refuses("a first pose missing joints is refused",
                {**ok_doc, "poses": [{"name": "a", "joints": {"1": 0.0}},
                                     {"name": "b", "joints": {"1": 5.0}}]})
        refuses("a single-pose clip is refused (no motion)",
                {**ok_doc, "poses": [{"name": "a", "joints": first}]})
        refuses("an out-of-range pose is refused, not clamped",
                {**ok_doc, "poses": [{"name": "a", "joints": first},
                                     {"name": "b", "joints": {"1": 400.0}}]})
        refuses("an unknown joint is refused",
                {**ok_doc, "poses": [{"name": "a", "joints": first},
                                     {"name": "b", "joints": {"9": 0.0}}]})
        refuses("a future format version is refused",
                {**ok_doc, "version": 99})
        refuses("both `pose` and `joints` on one entry is refused",
                {**ok_doc, "poses": [{"name": "a", "joints": first},
                                     {"name": "b", "joints": {"1": 1.0},
                                      "pose": "somewhere"}]})
        refuses("a `pose` reference with no poses.json is refused",
                {**ok_doc, "poses": [{"name": "a", "joints": first},
                                     {"name": "b", "pose": "nowhere"}]})

        # The example the CLI prints must itself be loadable — an
        # example that does not parse is worse than none. Both forms:
        # the generic template and the calibration-anchored one.
        for label, text in (("the generic template", EXAMPLE),
                            ("the anchored example",
                             example_clip_doc(cals))):
            ex = d / f"{label.replace(' ', '_')}.json"
            ex.write_text(text)
            try:
                loaded = load_clip(cals, ex)
                check(f"{label} is a loadable clip", True, loaded.name)
            except BenchError as exc:
                check(f"{label} is a loadable clip", False, str(exc)[:90])
                continue
            if label.startswith("the anchored"):
                # It is offered to someone whose next command is `run`,
                # so it had better survive the gate from rest.
                v = gate_clip(gate, rest, loaded)
                check("...and the anchored one gates CLEAR from rest",
                      v.allowed, v.detail[:80])
                check("...starting exactly at the captured rest pose",
                      loaded.poses[0].ticks == rest,
                      "no approach move needed")

    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("runner OK")
    return 0


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        return _selftest()
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
