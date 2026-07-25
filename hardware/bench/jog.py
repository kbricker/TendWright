"""jog — incremental single-joint moves with soft limits and an e-stop key.

    uv run python -m hardware.bench.jog --id 3

Keys:
  +/=   jog positive        -/_  jog negative
  [ ]   halve / double the step size
  c     go to the joint's zero pose (frame zero; gripper: half open;
        2048 uncalibrated)
  t     toggle torque on/off
  q     quit (torque off)
  ANY OTHER KEY = E-STOP: torque off immediately and exit.

With calibration.json present, the jogged joint's soft limits default to
its CALIBRATED range (tighter and truer than the generic guard) and
positions print in its ratified units, ticks in parens. --step-deg jogs
by degrees instead of ticks. Uncalibrated joints keep the raw-tick
behavior — jog works before any calibration exists.

Exit codes: 0 quit, 2 error, 3 operator e-stop, 130 Ctrl+C.
Torque always starts OFF and is cut again on every exit path.

Usage: jog --id N [--port PORT] [--step TICKS | --step-deg D]
           [--min T] [--max T] [--cal FILE]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from hardware.units import DEG_PER_TICK, PctFrame, fmt_ticks

from .bus import POSITION_RANGE, BenchError, FeetechBus, run_tool
from .term import read_key

CENTER = 2048
JOG_SPEED = 300
JOG_ACCELERATION = 30


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.jog",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--id", type=int, required=True, dest="servo_id",
                        help="servo bus ID to jog")
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--step", type=int, default=None,
                        help="ticks per keypress (default 20)")
    parser.add_argument("--step-deg", type=float, default=None,
                        help="degrees per keypress (instead of --step)")
    parser.add_argument("--min", type=int, default=None,
                        dest="soft_min", help="soft limit low (ticks)")
    parser.add_argument("--max", type=int, default=None,
                        dest="soft_max", help="soft limit high (ticks)")
    parser.add_argument("--cal", default="calibration.json",
                        help="calibration file for limits + unit display")
    args = parser.parse_args()

    if args.step is not None and args.step_deg is not None:
        raise BenchError("--step and --step-deg are mutually exclusive")
    if args.step_deg is not None and args.step_deg <= 0:
        raise BenchError("--step-deg must be positive")
    if args.step is not None and args.step <= 0:
        raise BenchError("--step must be positive")

    # Calibrated joint: soft limits default to the CALIBRATED range and
    # positions display in the ratified frame. Lazy import for symmetry
    # with monitor (whose parse_ids IS module-imported by calibrate);
    # #637 gives load_calibration a shared home.
    cal = None
    cal_path = Path(args.cal)
    if cal_path.exists():
        from .calibrate import load_calibration
        cal = load_calibration(cal_path).get(args.servo_id)
    frame = cal.frame if cal else None

    soft_min = args.soft_min if args.soft_min is not None else (
        cal.min if cal else POSITION_RANGE[0] + 200)
    soft_max = args.soft_max if args.soft_max is not None else (
        cal.max if cal else POSITION_RANGE[1] - 200)
    if soft_min >= soft_max:
        raise BenchError("--min must be below --max")

    if args.step_deg is not None:
        step0 = max(1, round(args.step_deg / DEG_PER_TICK))
    else:
        step0 = args.step if args.step is not None else 20

    def show(tick: int) -> str:
        return fmt_ticks(frame, tick)

    with FeetechBus(args.port) as bus:
        if bus.ping(args.servo_id) is None:
            raise BenchError(f"servo {args.servo_id} did not answer",
                             "run the scan tool to see what is on the bus")
        # Enforce the advertised starting state instead of assuming it: a
        # previous tool may have died with torque latched on.
        bus.set_torque(args.servo_id, False)
        target = bus.read_position(args.servo_id)
        step = step0
        torque_on = False
        name = f" ({cal.name})" if cal else ""
        limits_src = "calibrated" if (cal and args.soft_min is None
                                      and args.soft_max is None) else "soft"
        # 'c' pose: frame zero for angles, half-open for the gripper's
        # percent frame (its 0 is the fully-closed jaw), 2048 uncalibrated
        if frame is None:
            center = CENTER
        elif isinstance(frame, PctFrame):
            center = frame.tick(50)
        else:
            center = frame.tick(0)
        print(f"jogging servo {args.servo_id}{name} on {bus.port_name} — "
              f"position {show(target)}, {limits_src} limits "
              f"[{show(soft_min)}, {show(soft_max)}]")
        print("torque is OFF; press 't' to enable before jogging. "
              "+/- jog, [ ] step size, c zero pose, q quit, "
              "any other key = E-STOP")

        try:
            while True:
                key = read_key(timeout=0.5)
                if key is None:
                    continue
                if key in ("+", "="):
                    delta = step
                elif key in ("-", "_"):
                    delta = -step
                elif key == "[":
                    step = max(1, step // 2)
                    print(f"\nstep = {step}")
                    continue
                elif key == "]":
                    step = min(500, step * 2)
                    print(f"\nstep = {step}")
                    continue
                elif key == "c":
                    delta = center - target
                elif key == "t":
                    if torque_on:
                        bus.set_torque(args.servo_id, False)
                        torque_on = False
                        print("\ntorque OFF")
                    else:
                        # The joint may have been hand-moved (or sagged) while
                        # torque was off, and the servo's goal register may be
                        # stale from any earlier session. Re-sync the target
                        # AND pre-load the goal to the current position while
                        # still torque-off, so enabling torque holds in place
                        # instead of lurching to an old goal.
                        target = bus.read_position(args.servo_id)
                        bus.move_to(args.servo_id, target,
                                    speed=JOG_SPEED, acceleration=JOG_ACCELERATION)
                        bus.set_torque(args.servo_id, True)
                        torque_on = True
                        print(f"\ntorque ON — holding at {show(target)}")
                    continue
                elif key == "q":
                    print("\nquitting — torque off")
                    return 0
                else:
                    print("\nE-STOP — torque off", file=sys.stderr)
                    return 3

                if not torque_on:
                    print("\ntorque is OFF — press 't' first")
                    continue
                clamped = max(soft_min, min(soft_max, target + delta))
                if clamped != target + delta:
                    print(f"\nsoft limit — clamped to {show(clamped)}")
                target = clamped
                bus.move_to(args.servo_id, target,
                            speed=JOG_SPEED, acceleration=JOG_ACCELERATION)
                pos = bus.read_position(args.servo_id)
                print(f"\rtarget {show(target):>16}  now {show(pos):>16} "
                      f"(moving)   ", end="", flush=True)
        finally:
            bus.safe_torque_off([args.servo_id])


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
