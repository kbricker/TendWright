"""jog — incremental single-joint moves with soft limits and an e-stop key.

    uv run python -m hardware.bench.jog --id 3

Keys:
  +/=   jog positive        -/_  jog negative
  [ ]   halve / double the step size
  c     go to center (2048)
  t     toggle torque on/off
  q     quit (torque off)
  ANY OTHER KEY = E-STOP: torque off immediately and exit.

Exit codes: 0 quit, 2 error, 3 operator e-stop, 130 Ctrl+C.
Torque always starts OFF and is cut again on every exit path.

Usage: jog --id N [--port PORT] [--step TICKS] [--min T] [--max T]
"""

from __future__ import annotations

import argparse
import sys

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
    parser.add_argument("--step", type=int, default=20, help="ticks per keypress")
    parser.add_argument("--min", type=int, default=POSITION_RANGE[0] + 200,
                        dest="soft_min", help="soft limit low (ticks)")
    parser.add_argument("--max", type=int, default=POSITION_RANGE[1] - 200,
                        dest="soft_max", help="soft limit high (ticks)")
    args = parser.parse_args()

    if args.soft_min >= args.soft_max:
        raise BenchError("--min must be below --max")

    with FeetechBus(args.port) as bus:
        if bus.ping(args.servo_id) is None:
            raise BenchError(f"servo {args.servo_id} did not answer",
                             "run the scan tool to see what is on the bus")
        # Enforce the advertised starting state instead of assuming it: a
        # previous tool may have died with torque latched on.
        bus.set_torque(args.servo_id, False)
        target = bus.read_position(args.servo_id)
        step = args.step
        torque_on = False
        print(f"jogging servo {args.servo_id} on {bus.port_name} — "
              f"position {target}, soft limits [{args.soft_min}, {args.soft_max}]")
        print("torque is OFF; press 't' to enable before jogging. "
              "+/- jog, [ ] step size, c center, q quit, any other key = E-STOP")

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
                    delta = CENTER - target
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
                        print(f"\ntorque ON — holding at {target}")
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
                clamped = max(args.soft_min, min(args.soft_max, target + delta))
                if clamped != target + delta:
                    print(f"\nsoft limit — clamped to {clamped}")
                target = clamped
                bus.move_to(args.servo_id, target,
                            speed=JOG_SPEED, acceleration=JOG_ACCELERATION)
                pos = bus.read_position(args.servo_id)
                print(f"\rtarget {target:>4}  now {pos:>4} (moving)   ",
                      end="", flush=True)
        finally:
            bus.safe_torque_off([args.servo_id])


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
