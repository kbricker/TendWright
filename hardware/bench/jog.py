"""jog — incremental single-joint moves with soft limits and an e-stop key.

    uv run python -m hardware.bench.jog --id 3

Keys:
  +/=   jog positive        -/_  jog negative
  [ ]   halve / double the step size
  c     go to center (2048)
  t     toggle torque on/off
  q     quit (torque off)
  ANY OTHER KEY = E-STOP: torque off immediately and exit.

Usage: jog --id N [--port PORT] [--step TICKS] [--min T] [--max T]
"""

from __future__ import annotations

import argparse
import sys

from .bus import POSITION_RANGE, BenchError, FeetechBus, run_tool
from .term import read_key

CENTER = 2048


def run() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", type=int, required=True, dest="servo_id")
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
                    torque_on = not torque_on
                    bus.set_torque(args.servo_id, torque_on)
                    if not torque_on:
                        target = bus.read_position(args.servo_id)
                    print(f"\ntorque {'ON' if torque_on else 'OFF'}")
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
                bus.move_to(args.servo_id, target, speed=300, acceleration=30)
                pos = bus.read_position(args.servo_id)
                print(f"\rtarget {target:>4}  actual {pos:>4}   ",
                      end="", flush=True)
        finally:
            bus.set_torque(args.servo_id, False)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
