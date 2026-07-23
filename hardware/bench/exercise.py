"""exercise — scripted limber-up: wake, go to rest, sweep every joint, rest.

The first scripted-motion tool. It consumes calibration.json (ranges, rest
pose, per-joint soft limits) and REFUSES to move an uncalibrated arm or one
that isn't starting from its rest pose. The routine:

    wake (no lurch) -> rest pose -> per-joint sweep, one joint at a time,
    others holding -> rest pose -> torque off

    uv run python -m hardware.bench.exercise
    uv run python -m hardware.bench.exercise --ids 2,3 --span 50 --speed 0.5

ANY key during motion is an E-STOP: the arm halts at its present position
and holds while you get a hand on it, then torque cuts on your Enter.
Exit codes: 0 done, 1 aborted, 2 error, 3 operator e-stop, 130 Ctrl+C.

Usage: exercise [--ids RANGE] [--span PCT] [--speed F] [--cal FILE]
                [--port PORT] [--yes]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .bus import BenchError, FeetechBus, confirm, run_tool
from .calibrate import JOINT_NAMES, JointCal, load_calibration
from .monitor import parse_ids
from .term import flush_input, read_key

SAMPLE_S = 0.05  # settle-poll cadence; also the e-stop key latency bound
SPEED_BASE = 200  # servo speed units at --speed 1.0 (gentler than jog's 300)
SPEED_CAP = 400  # servo-side ceiling regardless of --speed
ACCELERATION = 15
SPAN_MIN, SPAN_MAX = 10, 90  # sweep % of calibrated span; >=5% end margin
SPAN_DEFAULT = 70
SETTLE_TOL_TICKS = 25  # "arrived" position tolerance (matches teach)
STILL_TICKS = 4  # per-sample movement below this counts as "still"
STILL_SAMPLES = 3  # consecutive still+arrived samples = settled
SETTLE_GRACE_S = 5.0  # deadline slack beyond the ideal travel time
PREFLIGHT_REST_TOL_TICKS = 300  # how far from rest the arm may start


class EStop(Exception):
    """Operator pressed a key during motion."""


def sweep_window(cal: JointCal, span_pct: int) -> tuple[int, int]:
    """The sweep sub-range: span_pct percent of [min,max], centered."""
    inset = (cal.max - cal.min) * (100 - span_pct) // 200
    return cal.min + inset, cal.max - inset


def move(bus: FeetechBus, servo_id: int, target: int, speed: int) -> None:
    bus.move_to(servo_id, target, speed=speed, acceleration=ACCELERATION)


def halt_all(bus: FeetechBus, ids: list[int]) -> None:
    """Stop motion by re-goaling every joint to where it is right now."""
    for servo_id in ids:
        try:
            move(bus, servo_id, bus.read_position(servo_id), SPEED_BASE)
        except BenchError:
            pass  # halting is best-effort; torque-off cleanup still runs


def wait_settle(bus: FeetechBus, targets: dict[int, int], speed: int,
                label: str) -> None:
    """Poll until every joint in targets is at its target AND still.

    Plant-gated: settles on actual position + observed stillness, never on
    the command or a timer. Polls the keyboard between samples — ANY key
    raises EStop. A joint that never settles (obstruction, too weak) halts
    the arm and errors out with torque cut by the caller's finally.
    """
    ids = sorted(targets)
    prev = {i: bus.read_position(i) for i in ids}
    still: dict[int, int] = {i: 0 for i in ids}
    worst_travel = max(abs(prev[i] - targets[i]) for i in ids)
    # Servo speed units approximate ticks/s closely enough for a deadline.
    deadline = (time.monotonic() + SETTLE_GRACE_S
                + worst_travel / max(1, speed))
    while True:
        start = time.monotonic()
        done = True
        worst = 0
        for i in ids:
            pos = bus.read_position(i)
            err = abs(pos - targets[i])
            worst = max(worst, err)
            if err <= SETTLE_TOL_TICKS and abs(pos - prev[i]) <= STILL_TICKS:
                still[i] += 1
            else:
                still[i] = 0
            prev[i] = pos
            if still[i] < STILL_SAMPLES:
                done = False
        print(f"\r  {label}: worst error {worst:>4} ticks   ",
              end="", flush=True)
        if done:
            print()
            return
        if time.monotonic() > deadline:
            print()
            halt_all(bus, ids)
            lagging = sorted(i for i in ids if still[i] < STILL_SAMPLES)
            raise BenchError(
                f"joint(s) {lagging} did not settle at their target "
                f"(worst error {worst} ticks) — halted",
                "a joint may be obstructed or fighting gravity at this "
                "speed; clear the workspace or try --speed 1.0, and check "
                "the calibrated range for that joint",
            )
        key = read_key(timeout=max(0.0, SAMPLE_S
                                   - (time.monotonic() - start)))
        if key is not None:
            print()
            raise EStop


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
                        help="skip the confirmation prompt")
    args = parser.parse_args()

    if not SPAN_MIN <= args.span <= SPAN_MAX:
        raise BenchError(f"--span must be {SPAN_MIN}-{SPAN_MAX}",
                         "the margin outside the sweep keeps the arm off "
                         "its calibrated end stops")
    if not 0.1 <= args.speed <= 2.0:
        raise BenchError("--speed must be between 0.1 and 2.0")
    speed = min(SPEED_CAP, max(20, round(SPEED_BASE * args.speed)))

    if not sys.stdin.isatty():
        raise BenchError(
            "this tool needs an interactive terminal — the e-stop key is "
            "the safety channel",
            "allocate one: `ssh -t cell1 '...'` (note -t), or run it from "
            "a shell on the box",
        )

    cal_path = Path(args.cal)
    if not cal_path.exists():
        raise BenchError(
            f"no calibration file at {cal_path}",
            "this tool refuses to move an uncalibrated arm — run "
            "`calibrate capture` first (or point --cal at the file)",
        )
    cals = load_calibration(cal_path)

    if args.ids is None:
        ids = sorted(cals)
    else:
        ids = list(dict.fromkeys(parse_ids(args.ids)))
        uncalibrated = sorted(set(ids) - set(cals))
        if uncalibrated:
            raise BenchError(
                f"joint(s) {uncalibrated} are not in {cal_path}",
                "capture them first: calibrate capture --ids "
                + ",".join(str(i) for i in uncalibrated),
            )

    with FeetechBus(args.port) as bus:
        missing = [i for i in ids if bus.ping(i) is None]
        if missing:
            raise BenchError(f"no answer from servo IDs {missing}",
                             "run the scan tool to see what is on the bus")

        # Preflight: never start from an unknown configuration. The arm at
        # torque-off rest is exactly the pose calibration recorded, so a
        # large mismatch means a changed horn, a stale file, or an arm left
        # propped somewhere — a human sorts that out, not this tool.
        pose = {i: bus.read_position(i) for i in ids}
        for i in ids:
            c = cals[i]
            if not c.min - SETTLE_TOL_TICKS <= pose[i] <= c.max + SETTLE_TOL_TICKS:
                raise BenchError(
                    f"joint {i} ({c.name}) reads {pose[i]}, outside its "
                    f"calibrated range [{c.min}, {c.max}]",
                    "if the horn was remounted, re-run `calibrate capture "
                    f"--ids {i}`; otherwise move the arm near its rest "
                    "pose and re-run",
                )
            if abs(pose[i] - c.rest) > PREFLIGHT_REST_TOL_TICKS:
                raise BenchError(
                    f"joint {i} ({c.name}) reads {pose[i]}, "
                    f"{abs(pose[i] - c.rest)} ticks from its rest pose "
                    f"({c.rest})",
                    "place the arm at its rest pose (torque-off slump) and "
                    "re-run — this tool only starts from rest",
                )

        windows = {i: sweep_window(cals[i], args.span) for i in ids}
        print(f"exercise routine for joint(s) {ids} on {bus.port_name}:")
        print(f"  wake -> rest -> sweep each joint through {args.span}% of "
              f"its range -> rest -> torque off")
        for i in ids:
            lo, hi = windows[i]
            print(f"    joint {i} ({cals[i].name:<13}) rest {cals[i].rest:>4}"
                  f"  sweep {lo:>4} -> {hi:>4}")
        print("  keep the workspace clear and the gripper empty.")
        print("  ANY key during motion is an E-STOP (halt, then guided "
              "torque cut). The power switch is the hard e-stop.")
        if not args.yes and not confirm("type y to start: "):
            print("aborted")
            return 1

        try:
            # Wake without lurch, one servo at a time: pre-load the goal
            # (and speed/accel) to the CURRENT position while still torque
            # off, then enable torque — never against a stale goal register.
            print("\nwaking (torque on, holding in place)...")
            for i in ids:
                move(bus, i, bus.read_position(i), speed)
                bus.set_torque(i, True)

            rest = {i: cals[i].rest for i in ids}
            print("moving to the rest pose...")
            for i in ids:
                move(bus, i, rest[i], speed)
            wait_settle(bus, rest, speed, "rest")

            for n, i in enumerate(ids, start=1):
                lo, hi = windows[i]
                name = cals[i].name
                print(f"\n[{n}/{len(ids)}] sweeping joint {i} ({name}): "
                      f"{lo} -> {hi} -> rest {rest[i]}")
                for label, target in (("low", lo), ("high", hi),
                                      ("rest", rest[i])):
                    move(bus, i, target, speed)
                    goals = {**rest, i: target}
                    wait_settle(bus, goals, speed, f"{name} {label}")

            print("\nroutine complete — arm at rest, cutting torque")
            return 0
        except EStop:
            print("\nE-STOP — halting at present position", file=sys.stderr)
            halt_all(bus, ids)
            print("the arm is HOLDING under torque. get a hand on it — "
                  "it drops when torque cuts.", file=sys.stderr)
            flush_input()
            try:
                input("press Enter to cut torque: ")
            except (EOFError, KeyboardInterrupt):
                pass  # fall through to the finally's torque cut
            return 3
        finally:
            bus.safe_torque_off(ids)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
