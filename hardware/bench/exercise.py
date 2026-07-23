"""exercise — scripted limber-up: wake, go to rest, sweep every joint, rest.

The first scripted-motion tool. It consumes calibration.json (ranges, rest
pose, per-joint soft limits) and REFUSES to move an uncalibrated arm or one
that isn't starting from its rest pose. The routine:

    wake (no lurch) -> rest pose -> per-joint sweep, one joint at a time,
    others holding -> rest pose -> torque off

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
from pathlib import Path

from .bus import BenchError, FeetechBus, confirm, run_tool
from .calibrate import JOINT_NAMES, JointCal, load_calibration
from .monitor import parse_ids
from .motion import EStop, halt_all, wait_settle
from .term import flush_input, read_key, require_interactive

SPEED_BASE = 200  # servo speed units at --speed 1.0 (gentler than jog's 300)
SPEED_CAP = 400  # servo-side ceiling regardless of --speed
ACCELERATION = 15
SPAN_MIN, SPAN_MAX = 10, 90  # sweep % of calibrated span; >=5% end margin
SPAN_DEFAULT = 70
PREFLIGHT_RANGE_MARGIN = 25  # start-pose slack outside [min,max]
PREFLIGHT_REST_TOL_TICKS = 300  # how far from rest the arm may start


def sweep_window(cal: JointCal, span_pct: int) -> tuple[int, int]:
    """The sweep sub-range: span_pct percent of [min,max], centered."""
    inset = (cal.max - cal.min) * (100 - span_pct) // 200
    return cal.min + inset, cal.max - inset


def clamp_goal(cal: JointCal, position: int) -> int:
    """Every commanded goal stays strictly inside the calibrated range —
    even the rest pose and wake holds (the loader tolerates a rest up to
    25 ticks outside [min,max]; commands must not)."""
    return max(cal.min, min(cal.max, position))


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
                f"joint {i} ({c.name}) reads {pos}, outside its "
                f"calibrated range [{c.min}, {c.max}]",
                "if the horn was remounted, re-run `calibrate capture "
                f"--ids {i}`; otherwise move the arm near its rest "
                "pose and re-run",
            )
        if abs(pos - c.rest) > PREFLIGHT_REST_TOL_TICKS:
            raise BenchError(
                f"joint {i} ({c.name}) reads {pos}, "
                f"{abs(pos - c.rest)} ticks from its rest pose ({c.rest})",
                "place the arm at its rest pose (torque-off slump) and "
                "re-run — support it first if it is holding itself (a "
                "crashed tool may have left torque latched on)",
            )


def held_torque_cut(bus: FeetechBus, ids: list[int], why: str) -> None:
    """The arm is (or may be) holding under torque somewhere mid-routine.
    Never drop it on an unwarned operator: hold until they have a hand on
    it, then let the caller's cleanup cut torque."""
    print(f"\n{why} — the arm is HOLDING under torque. get a hand on it — "
          "it drops when torque cuts.", file=sys.stderr)
    flush_input()
    try:
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

    # --ids selects the SWEEP subset; every calibrated joint is preflighted,
    # woken, and held regardless — big joints must never sweep past limp,
    # unmonitored distal ones.
    ids = sorted(cals)
    if args.ids is None:
        sweep_ids = ids
    else:
        sweep_ids = sorted(dict.fromkeys(parse_ids(args.ids)))
        unknown = sorted(set(sweep_ids) - set(JOINT_NAMES))
        if unknown:
            raise BenchError(
                f"unknown joint ID(s) {unknown}",
                "the SO-101 follower uses IDs 1-6 (base to gripper)")
        uncalibrated = sorted(set(sweep_ids) - set(cals))
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
        windows = {i: sweep_window(cals[i], args.span) for i in sweep_ids}
        print(f"exercise routine on {bus.port_name}: sweep joint(s) "
              f"{sweep_ids}, hold {[i for i in ids if i not in sweep_ids] or 'none'}")
        print(f"  wake -> rest -> sweep each joint through {args.span}% of "
              f"its range -> rest -> torque off")
        for i in sweep_ids:
            lo, hi = windows[i]
            print(f"    joint {i} ({cals[i].name:<13}) rest {rest[i]:>4}"
                  f"  sweep {lo:>4} -> {hi:>4}")
        print("  keep the workspace clear and the gripper empty.")
        print("  ANY key during motion is an E-STOP (halt + hold, torque "
              "cuts on your Enter). The power switch is the hard e-stop.")
        if not args.yes and not confirm("type y to start: "):
            print("aborted")
            return 1

        # The pose was vetted BEFORE the prompt, which can sit open for
        # minutes — re-vet after it, right before anything energizes.
        check_start_pose(bus, cals, ids)

        try:
            # Wake without lurch, one servo at a time: pre-load the goal
            # (and speed/accel) to the CURRENT position while still torque
            # off, then enable torque — never against a stale goal register.
            print("\nwaking (torque on, holding in place)...")
            for i in ids:
                bus.move_to(i, clamp_goal(cals[i], bus.read_position(i)),
                            speed=speed, acceleration=ACCELERATION)
                bus.set_torque(i, True)

            print("moving to the rest pose...")
            for i in ids:
                bus.move_to(i, rest[i], speed=speed,
                            acceleration=ACCELERATION)
            wait_settle(bus, rest, speed, "rest", poll_key=read_key)

            for n, i in enumerate(sweep_ids, start=1):
                lo, hi = windows[i]
                name = cals[i].name
                print(f"\n[{n}/{len(sweep_ids)}] sweeping joint {i} "
                      f"({name}): {lo} -> {hi} -> rest {rest[i]}")
                for label, target in (("low", lo), ("high", hi),
                                      ("rest", rest[i])):
                    bus.move_to(i, clamp_goal(cals[i], target),
                                speed=speed, acceleration=ACCELERATION)
                    goals = {**rest, i: clamp_goal(cals[i], target)}
                    wait_settle(bus, goals, speed, f"{name} {label}",
                                poll_key=read_key)

            print("\nroutine complete — arm at rest, cutting torque")
            return 0
        except EStop:
            print("\nE-STOP — halting at present position", file=sys.stderr)
            halt_all(bus, ids)
            held_torque_cut(bus, ids, "e-stop")
            return 3
        except BenchError:
            # wait_settle timeouts arrive here already halted; comm errors
            # may not be halted, but the arm may still be holding mid-air —
            # same rule either way: never cut torque on an unwarned operator.
            held_torque_cut(bus, ids, "error")
            raise
        except KeyboardInterrupt:
            halt_all(bus, ids)
            held_torque_cut(bus, ids, "interrupted")
            raise
        finally:
            bus.safe_torque_off(ids)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
