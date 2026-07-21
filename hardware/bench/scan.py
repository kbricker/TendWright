"""scan — enumerate the servo bus and report per-servo health.

    uv run python -m hardware.bench.scan            # IDs 0-20 (fast)
    uv run python -m hardware.bench.scan --full     # all IDs 0-253

Usage: scan [--port PORT] [--full]
"""

from __future__ import annotations

import argparse
import sys

from .bus import SCAN_IDS, BenchError, FeetechBus, run_tool


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.scan",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--full", action="store_true",
                        help="scan the full ID range 0-253 (slower)")
    args = parser.parse_args()

    ids = SCAN_IDS if args.full else list(range(0, 21))
    with FeetechBus(args.port) as bus:
        print(f"scanning {bus.port_name} (IDs {ids[0]}-{ids[-1]})...")
        found = bus.scan(ids, progress=args.full)
        if not found:
            raise BenchError(
                "no servos found",
                "check servo power and cabling; use --full if a servo may "
                "have an ID above 20",
            )
        print(f"{'ID':>3}  {'model':>6}  {'fw':>6}  {'volt':>5}  "
              f"{'temp':>4}  {'pos':>5}")
        for servo_id in found:
            i = bus.info(servo_id)
            print(f"{i.servo_id:>3}  {i.model:>6}  {i.firmware:>6}  "
                  f"{i.voltage:>4.1f}V  {i.temperature:>3}C  {i.position:>5}")
        print(f"{len(found)} servo(s) on the bus")
        return 0


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
