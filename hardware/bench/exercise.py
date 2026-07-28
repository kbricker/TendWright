"""exercise — scripted limber-up: wake, go to rest, sweep every joint, rest.

The first scripted-motion tool. It consumes calibration.json (ranges, rest
pose, per-joint soft limits) and REFUSES to move an uncalibrated arm or one
that isn't starting from its rest pose. The routine:

    wake (no lurch) -> rest pose -> per-joint sweep, one joint at a time,
    others holding, DISTAL FIRST (4 wrist_flex, 5 wrist_roll, 6 gripper,
    3 elbow, 2 shoulder, 1 base pan — rest is a compact fold; unfold the
    light end before slinging the heavy joints) -> rest -> torque off

    While the shoulder (2) sweeps, the elbow (3) AND wrist (4) hold
    90 deg open and the sweep is capped at 40% of the calibrated span —
    the contact-free envelope derived from the digital twin (sim.twin,
    plan #648), which correctly predicted both earlier bench collisions.
    They refold after. The whole routine is simulated in the twin as a
    pre-flight collision gate before anything moves (--no-gate to skip).

    uv run python -m hardware.bench.exercise
    uv run python -m hardware.bench.exercise --ids 2,3 --span 50 --speed 0.5

--ids picks which joints SWEEP; every calibrated joint is always checked,
woken, and held (a sweeping shoulder must not whip a limp elbow around).
ANY key during motion is an E-STOP; on e-stop, obstruction, or Ctrl+C the
arm halts and HOLDS while you get a hand on it — torque cuts on your Enter.
Exit codes: 0 done, 1 aborted, 2 error, 3 operator e-stop, 130 Ctrl+C.

Usage: exercise [--ids RANGE] [--span PCT] [--speed F] [--cal FILE]
                [--port PORT] [--yes]
"""

from __future__ import annotations

import argparse
import sys
from functools import partial
from pathlib import Path

import serial

from hardware.units import DEG_PER_TICK, fmt_ticks

from .bus import BenchError, FeetechBus, confirm, run_tool
from .calibrate import (JOINT_NAMES, JointCal, fold_direction,
                        load_calibration)
from .guards import (ENTRY_PHASE, MOTION_PHASE, StrainWatch,
                     check_holds, holds_for)
from .monitor import parse_ids
from .motion import EStop, halt_all, wait_settle
from .term import flush_input, read_key, require_interactive

SPEED_BASE = 200  # servo speed units at --speed 1.0 (gentler than jog's 300)
SPEED_CAP = 400  # servo-side ceiling regardless of --speed
ACCELERATION = 15
# Ceiling on --accel. Higher means the internal profile generator hands
# over to the position loop sooner, which is exactly the regime where a
# gravity-assisted joint runs away (plan #674). Not a tuning knob to
# open wide.
ACCEL_CAP = 60
SPAN_MIN, SPAN_MAX = 10, 90  # sweep % of calibrated span; >=5% end margin
SPAN_DEFAULT = 70
PREFLIGHT_RANGE_MARGIN = 25  # start-pose slack outside [min,max]
PREFLIGHT_REST_TOL_TICKS = 300  # how far from rest the arm may start
# Sweep order is distal-first (Kyle, first bench flow review): rest is a
# folded, compact pose, so unfold the wrist (4), roll it (5), work the
# gripper (6) while the heavy joints hold still, THEN open the elbow (3)
# and shoulder (2), and swing the base (1) last. Proximal-first from a
# fold slings the whole folded stack around — worst inertia, and the
# worst place to discover a range/sign surprise.
SWEEP_ORDER = (4, 5, 6, 3, 2, 1)
# Clearance holds + the m2 span cap are DERIVED FROM THE DIGITAL TWIN
# (`python -m sim.twin derive-clearance`, plan #648, 2026-07-25), which
# also predicted both real bench collisions: the original hand-tuned
# 45-deg elbow hold cleared neither the table at the sweep's low end
# (with the wrist folded) nor the arm's own shoulder at the high end.
# The model's contact-free fixed-hold answer: m2 sweeps at most 40% of
# its span while the elbow AND wrist hold 90 deg open, refolding after.
CLEARANCE_HOLDS = {2: (3, 4)}
CLEARANCE_DEG = 90
CLEARANCE_TICKS = round(CLEARANCE_DEG / DEG_PER_TICK)  # ~1024
SWEEP_SPAN_CAPS = {2: 40}  # per-joint ceiling on --span, from the twin


def show(cal: JointCal, tick: int) -> str:
    return fmt_ticks(cal.frame, tick)


def sweep_window(cal: JointCal, span_pct: int) -> tuple[int, int]:
    """The sweep sub-range: span_pct percent of [min,max], centered."""
    inset = (cal.max - cal.min) * (100 - span_pct) // 200
    return cal.min + inset, cal.max - inset


def clamp_goal(cal: JointCal, position: int) -> int:
    """Every commanded goal stays strictly inside the calibrated range —
    even the rest pose and wake holds (the loader tolerates a rest up to
    25 ticks outside [min,max]; commands must not)."""
    return max(cal.min, min(cal.max, position))


def clearance_pose(cal: JointCal, deg: float = CLEARANCE_DEG) -> int:
    """`deg` open from a folded rest, clamped in-range."""
    return clamp_goal(cal, cal.rest
                      + fold_direction(cal) * round(deg / DEG_PER_TICK))


def gate_waypoints(cals: dict[int, JointCal], rest: dict[int, int],
                   windows: dict[int, tuple[int, int]],
                   sweep_ids: list[int],
                   start: dict[int, int] | None = None,
                   ) -> list[dict[int, int]]:
    """The routine's full pose sequence — the ONE definition shared by
    the bench pre-flight gate and `sim.twin exercise`. `start` (the
    measured present pose) leads, because the wake -> rest move is real
    motion too: preflight admits a start up to 300 ticks off rest."""
    seq: list[dict[int, int]] = []
    if start is not None:
        seq.append(dict(start))
    seq.append(dict(rest))
    for i in sweep_ids:
        lo, hi = windows[i]
        clearance = {j: clearance_pose(cals[j])
                     for j in CLEARANCE_HOLDS.get(i, ()) if j != i}
        hold = {**rest, **clearance}
        if clearance:
            seq.append(dict(hold))
        for target in (lo, hi, rest[i]):
            seq.append({**hold, i: clamp_goal(cals[i], target)})
        if clearance:
            seq.append(dict(rest))
    return seq


def exercise_clip(cals: dict[int, JointCal], rest: dict[int, int],
                  windows: dict[int, tuple[int, int]],
                  sweep_ids: list[int],
                  start: dict[int, int] | None = None,
                  profile: 'MotionProfile | None' = None) -> 'Clip':
    """The routine as a CLIP — poses plus the profile that moves between
    them (plan #660).

    The profile carried here is the one whose `speed` and `acceleration`
    are written into the servo registers below, so the path the gate
    simulates and the path the arm takes come from the same two numbers.
    Sharing only the pose list — which is all `gate_waypoints` ever did —
    made the endpoints agree while leaving the motion between them free
    to differ."""
    from sim.clip import DEFAULT_PROFILE, Clip, Pose

    names = ["start"] if start is not None else []
    seq = gate_waypoints(cals, rest, windows, sweep_ids, start)
    # label the poses for readable refusals; the sequence itself is
    # gate_waypoints', so the two cannot describe different routines
    while len(names) < len(seq):
        names.append(f"p{len(names)}")
    return Clip("exercise", [Pose(n, p) for n, p in zip(names, seq)],
                profile or DEFAULT_PROFILE)


def check_start_pose(bus: FeetechBus, cals: dict[int, JointCal],
                     ids: list[int]) -> None:
    """Refuse to run unless every joint is inside its calibrated range and
    near its rest pose. Never start from an unknown configuration: a big
    mismatch means a changed horn, a stale file, or an arm left propped
    somewhere — a human sorts that out, not this tool."""
    for i in ids:
        c = cals[i]
        pos = bus.read_position(i)
        lo = c.min - PREFLIGHT_RANGE_MARGIN
        hi = c.max + PREFLIGHT_RANGE_MARGIN
        if not lo <= pos <= hi:
            raise BenchError(
                f"joint {i} ({c.name}) reads {show(c, pos)}, outside its "
                f"calibrated range [{show(c, c.min)}, {show(c, c.max)}]",
                "if the horn was remounted, re-run `calibrate capture "
                f"--ids {i}`; otherwise move the arm near its rest "
                "pose and re-run",
            )
        if abs(pos - c.rest) > PREFLIGHT_REST_TOL_TICKS:
            raise BenchError(
                f"joint {i} ({c.name}) reads {show(c, pos)}, "
                f"{abs(pos - c.rest)} ticks from its rest pose "
                f"({show(c, c.rest)})",
                "place the arm at its rest pose (torque-off slump) and "
                "re-run — support it first if it is holding itself (a "
                "crashed tool may have left torque latched on)",
            )


def held_torque_cut(why: str) -> None:
    """The arm is (or may be) holding under torque somewhere mid-routine.
    Never drop it on an unwarned operator: hold until they have a hand on
    it, then let the caller's cleanup cut torque. A Ctrl+C anywhere in
    here (not just at the input) skips ahead to that same cleanup."""
    try:
        print(f"\n{why} — the arm is HOLDING under torque. get a hand on "
              "it — it drops when torque cuts.", file=sys.stderr)
        flush_input()
        input("press Enter to cut torque: ")
    except (EOFError, KeyboardInterrupt):
        pass  # fall through to the caller's torque cut


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.exercise",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ids", default=None,
                        help="joints to sweep (default: all calibrated)")
    parser.add_argument("--span", type=int, default=SPAN_DEFAULT,
                        help=f"sweep %% of each calibrated range, "
                             f"{SPAN_MIN}-{SPAN_MAX} (default {SPAN_DEFAULT})")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="speed factor 0.1-2.0 (default 1.0; capped)")
    parser.add_argument("--cal", default="calibration.json",
                        help="calibration file from calibrate capture")
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--no-gate", action="store_true",
                        help="skip the digital-twin collision gate "
                             "(sim.twin) — bench emergencies only")
    parser.add_argument("--accel", type=int, default=ACCELERATION,
                        metavar="N",
                        help=f"servo acceleration register, 1-{ACCEL_CAP} "
                             f"(default {ACCELERATION}; x100 ticks/s^2). "
                             f"The cheapest lever on a joint that overhauls "
                             f"under gravity - see plan #674")
    parser.add_argument("--trace", metavar="FILE_OR_DIR",
                        help="record the arm's ACTUAL path to CSV, then "
                             "compare it with `python -m sim.trace FILE` "
                             "(plan #660 acceptance test)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt (never the "
                             "preflight checks)")
    args = parser.parse_args()

    if not SPAN_MIN <= args.span <= SPAN_MAX:
        raise BenchError(f"--span must be {SPAN_MIN}-{SPAN_MAX}",
                         "the margin outside the sweep keeps the arm off "
                         "its calibrated end stops")
    if not 0.1 <= args.speed <= 2.0:
        raise BenchError("--speed must be between 0.1 and 2.0")
    if not 1 <= args.accel <= ACCEL_CAP:
        raise BenchError(f"--accel must be 1-{ACCEL_CAP}",
                         "acceleration is REG_ACCELERATION, in units of "
                         "100 ticks/s^2")
    accel = args.accel
    speed = min(SPEED_CAP, round(SPEED_BASE * args.speed))

    require_interactive()  # the e-stop key is the safety channel

    cal_path = Path(args.cal)
    if not cal_path.exists():
        raise BenchError(
            f"no calibration file at {cal_path}",
            "this tool refuses to move an uncalibrated arm — run "
            "`calibrate capture` first (or point --cal at the file)",
        )
    cals = load_calibration(cal_path)
    uncaptured = sorted(set(JOINT_NAMES) - set(cals))
    if uncaptured:
        raise BenchError(
            f"{cal_path} covers joint(s) {sorted(cals)} but the SO-101 "
            f"has 1-6 — joint(s) {uncaptured} would hang limp while the "
            "others sweep",
            "capture the missing joint(s) first: calibrate capture --ids "
            + ",".join(str(i) for i in uncaptured),
        )

    # --ids selects the SWEEP subset; every joint is preflighted, woken,
    # and held regardless — big joints must never sweep past limp,
    # unmonitored distal ones (which is also why a partial calibration
    # file is refused above).
    ids = sorted(cals)
    if args.ids is None:
        sweep_ids = [i for i in SWEEP_ORDER if i in cals]
    else:
        requested = set(parse_ids(args.ids))
        sweep_ids = [i for i in SWEEP_ORDER if i in requested]
        unknown = sorted(requested - set(JOINT_NAMES))
        if unknown:
            raise BenchError(
                f"unknown joint ID(s) {unknown}",
                "the SO-101 follower uses IDs 1-6 (base to gripper)")
        uncalibrated = sorted(requested - set(cals))
        if uncalibrated:
            raise BenchError(
                f"joint(s) {uncalibrated} are not in {cal_path}",
                "capture them first: calibrate capture --ids "
                + ",".join(str(i) for i in uncalibrated),
            )

    with FeetechBus(args.port) as bus:
        missing = [i for i in ids if bus.ping(i) is None]
        if missing:
            raise BenchError(
                f"no answer from servo IDs {missing}",
                "every calibrated joint must be on the bus to be held "
                "during sweeps — run the scan tool",
            )

        check_start_pose(bus, cals, ids)
        # The arm is verified at rest, where torque-off is a no-drop
        # operation — so enforce the assumed baseline instead of trusting
        # it (a crashed tool may have left torque latched on).
        bus.safe_torque_off(ids)

        rest = {i: clamp_goal(cals[i], cals[i].rest) for i in ids}
        # Per-joint span caps from the twin: --span still shrinks a
        # sweep, but can never widen past the derived-safe ceiling.
        windows = {i: sweep_window(
            cals[i], min(args.span, SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
            for i in sweep_ids}
        print(f"exercise routine on {bus.port_name}: sweep joint(s) "
              f"{sweep_ids}, hold {[i for i in ids if i not in sweep_ids] or 'none'}")
        print(f"  wake -> rest -> sweep each joint through {args.span}% of "
              f"its range -> rest -> torque off")
        for i in sweep_ids:
            lo, hi = windows[i]
            print(f"    joint {i} ({cals[i].name:<13}) "
                  f"rest {show(cals[i], rest[i]):>16}  "
                  f"sweep {show(cals[i], lo):>16} -> {show(cals[i], hi)}")
        print("  keep the workspace clear and the gripper empty.")
        print("  ANY key during motion is an E-STOP (halt + hold, torque "
              "cuts on your Enter). The power switch is the hard e-stop.")

        # Pre-flight collision gate: simulate the exact routine in the
        # digital twin BEFORE anything energizes. The gate consumes a
        # CLIP carrying `profile` — the same speed/acceleration written
        # into the servo registers in the execution loop below — so it
        # simulates the path the arm will actually take between poses,
        # not a lockstep glide the servos never perform (plan #660).
        if not args.no_gate:
            try:
                from sim.clip import MotionProfile
                from sim.twin import Twin
            except ImportError as exc:
                raise BenchError(
                    f"collision gate unavailable ({exc})",
                    "run `uv sync` for mujoco, or --no-gate to skip "
                    "(bench emergencies only)") from exc
            start = {i: bus.read_position(i) for i in ids}
            profile = MotionProfile(speed=speed, acceleration=accel)
            report = Twin(cal_path).check_clip(
                exercise_clip(cals, rest, windows, sweep_ids, start, profile),
                # pose 0 is the arm's measured slump, not a plan
                settle_from_measured=True)
            # The summary carries clamp warnings — an anchor/range
            # mismatch is loudest exactly when the gate PASSES, so
            # always print it, never only on failure.
            print(report.summary(cals),
                  file=sys.stderr if not report.clean else sys.stdout)
            if not report.clean:
                raise BenchError(
                    "collision gate predicts contact — refusing to run",
                    "the twin says this routine would collide (details "
                    "above); adjust --span/--ids, or --no-gate only if "
                    "you are CERTAIN the model is wrong")

        if not args.yes and not confirm("type y to start: "):
            print("aborted")
            return 1

        # The pose was vetted BEFORE the prompt, which can sit open for
        # minutes — re-vet after it, right before anything energizes.
        check_start_pose(bus, cals, ids)

        # Built BEFORE the try, so the finally that flushes it can never
        # hit an unbound name on an early failure.
        # The automatic abort. The keypress e-stop needs a human; this
        # does not — the servos report load, temperature and their own
        # fault bits, and a joint pushing against something says so
        # before anything breaks. Watched on EVERY move, not just the
        # guarded sweep, because a collision does not care which phase
        # the routine is in.
        strain = StrainWatch(ids)

        trace = None
        if args.trace:
            import time as _time

            from sim.trace import Trace
            dest = Path(args.trace)
            # A directory means "a batch": auto-name so repeated runs
            # accumulate instead of overwriting each other. Losing run 4
            # of 6 because it reused a filename is a wasted bench trip.
            if dest.is_dir() or args.trace.endswith(("/", "\\")):
                stamp = _time.strftime("%Y%m%d-%H%M%S")
                dest = dest / (f"run-{stamp}-sp{speed}-ac{accel}"
                               f"-sn{args.span}.csv")
            trace = Trace(dest, meta={
                "speed": speed, "accel": accel, "span": args.span,
                "ids": sweep_ids, "cal": str(cal_path),
            })
            print(f"tracing the arm's actual path to {dest}")
        sink = trace.sample if trace is not None else None

        try:
            # Wake without lurch, one servo at a time: pre-load the goal
            # (and speed/accel) to the CURRENT position while still torque
            # off, then enable torque — never against a stale goal register.
            print("\nwaking (torque on, holding in place)...")
            for i in ids:
                bus.move_to(i, clamp_goal(cals[i], bus.read_position(i)),
                            speed=speed, acceleration=accel)
                bus.set_torque(i, True)

            print("moving to the rest pose...")
            for i in ids:
                bus.move_to(i, rest[i], speed=speed,
                            acceleration=accel)
            if trace is not None:
                trace.phase("rest", edge=1)
            wait_settle(bus, rest, speed, "rest", poll_key=read_key,
                        sample_sink=sink,
                        invariant=lambda: strain.check(bus))

            for n, i in enumerate(sweep_ids, start=1):
                lo, hi = windows[i]
                name = cals[i].name
                clearance = {j: clearance_pose(cals[j])
                             for j in CLEARANCE_HOLDS.get(i, ())
                             if j != i}
                hold = {**rest, **clearance}
                print(f"\n[{n}/{len(sweep_ids)}] sweeping joint {i} "
                      f"({name}): {show(cals[i], lo)} -> {show(cals[i], hi)}"
                      f" -> rest {show(cals[i], rest[i])}")
                # Guarded holds (#649): these joints must physically BE
                # open for this sweep to be safe — verified from the
                # encoders on entry, and re-verified every sample while
                # the sweep runs. A commanded hold is not a held joint.
                guard_holds = holds_for(cals, clearance,
                                        {j: fold_direction(cals[j])
                                         for j in clearance})
                guard_why = (f"joint {i} ({name}) sweeps only while these "
                             f"stay clear of the fold")
                hold_invariant = partial(check_holds, bus, guard_holds,
                                         MOTION_PHASE, guard_why)

                def invariant(_h=hold_invariant):
                    strain.check(bus)     # strain first: it is the one
                    _h()                  # that means "stop right now"

                if clearance:
                    for j, pose in clearance.items():
                        print(f"  opening joint {j} ({cals[j].name}) to "
                              f"{show(cals[j], pose)} ({CLEARANCE_DEG} deg "
                              f"clear of the fold) so joint {i} can't "
                              f"press it into the table")
                        bus.move_to(j, pose, speed=speed,
                                    acceleration=accel)
                    if trace is not None:
                        trace.phase(f"j{i} clearance")
                    wait_settle(bus, hold, speed, "clearance",
                                poll_key=read_key, sample_sink=sink,
                                invariant=lambda: strain.check(bus))
                    # ENTRY GUARD: the sweep is unreachable until the
                    # encoders confirm the clearance actually happened.
                    check_holds(bus, guard_holds, ENTRY_PHASE, guard_why)
                    print("  guard: "
                          + ", ".join(h.describe() for h in guard_holds)
                          + " — verified from the encoders")
                for label, target in (("low", lo), ("high", hi),
                                      ("rest", rest[i])):
                    bus.move_to(i, clamp_goal(cals[i], target),
                                speed=speed, acceleration=accel)
                    goals = {**hold, i: clamp_goal(cals[i], target)}
                    if trace is not None:
                        trace.phase(f"j{i} {label}")
                    wait_settle(bus, goals, speed, f"{name} {label}",
                                poll_key=read_key, invariant=invariant,
                                sample_sink=sink)
                if clearance:
                    for j in clearance:
                        print(f"  refolding joint {j} ({cals[j].name}) "
                              f"to rest")
                        bus.move_to(j, rest[j], speed=speed,
                                    acceleration=accel)
                    if trace is not None:
                        trace.phase(f"j{i} refold")
                    wait_settle(bus, rest, speed, "refold",
                                poll_key=read_key, sample_sink=sink,
                                invariant=lambda: strain.check(bus))

            print("\nroutine complete — arm at rest, cutting torque")
            return 0
        except EStop:
            print("\nE-STOP — halting at present position", file=sys.stderr)
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            held_torque_cut("e-stop")
            return 3
        except (BenchError, serial.SerialException):
            # SerialException is the raw pyserial fault _check doesn't
            # wrap — a transient USB glitch mid-sweep must get the same
            # held cut, not a surprise drop. Halt is best-effort (a dead
            # bus just no-ops per joint); the arm may still be holding
            # mid-air — never cut torque on an unwarned operator.
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            held_torque_cut("error")
            raise
        except KeyboardInterrupt:
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            held_torque_cut("interrupted")
            raise
        finally:
            bus.safe_torque_off(ids)
            # THE HEADLINE NUMBER OF A BASELINE RUN. The strain guard
            # has recorded per-joint peak load and temperature the whole
            # way, and until now nothing ever printed it — the guard
            # could stop the arm but could not tell you how close a
            # HEALTHY run came to the same thresholds, which is the one
            # thing needed to judge whether they are set sensibly.
            #
            # In the finally, so it survives every exit path: an aborted
            # or strain-tripped run is exactly when the numbers matter
            # most. After the torque cut, for the same reason the trace
            # write is — the arm being safe is never delayed by
            # bookkeeping.
            print(strain.summary())
            # Written on EVERY exit path — an aborted, e-stopped or
            # obstructed run is exactly when the trace is most worth
            # having.
            if trace is not None:
                written = trace.close()
                if written is not None:
                    print(f"trace: {len(trace)} samples -> {written}")
                    print(f"  compare: uv run python -m sim.trace {written}")
                elif trace.error:
                    print(f"trace: {trace.error}", file=sys.stderr)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
