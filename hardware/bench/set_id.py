"""set_id — program a single loose STS3215 to a bus ID (1-6).

Run this for each servo DURING assembly, before its joint closes up.
Procedure: POWER OFF the bus, connect exactly ONE servo, power on, then:

    uv run python -m hardware.bench.set_id --new-id 3

Safety: refuses to write unless exactly one ID answers on the bus. Note
the guard counts *IDs*, not servos — two factory-fresh servos both at the
default ID answer as one and can end up sharing the new ID (recoverable
only by disconnecting one). Hence: one servo physically connected, always.

Usage: set_id --new-id N [--port PORT] [--yes]
"""

from __future__ import annotations

import argparse
import sys

from .bus import SCAN_IDS, BenchError, FeetechBus, confirm, run_tool


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.set_id",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--new-id", type=int, required=True,
                        help="bus ID to program (1-6 for the SO-101 joints)")
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    args = parser.parse_args()

    if not 1 <= args.new_id <= 6:
        raise BenchError(f"--new-id {args.new_id} is outside 1-6",
                         "SO-101 joints use IDs 1-6 (base to gripper)")

    with FeetechBus(args.port) as bus:
        print(f"scanning bus on {bus.port_name} (full ID range, ~10s)...")
        found = bus.scan(SCAN_IDS, progress=True)

        if not found:
            raise BenchError(
                "no servo answered on the bus",
                "check servo power (7.4V supply on) and the cable; two servos "
                "sharing one ID also answer garbled — connect ONE servo only",
            )
        if len(found) > 1:
            raise BenchError(
                f"{len(found)} servos on the bus (IDs {found}) — refusing to "
                "write an ID",
                "connect exactly ONE servo when programming IDs",
            )

        current = found[0]
        info = bus.info(current)
        print(f"found one servo: ID {current}  model {info.model}  "
              f"fw {info.firmware}  {info.voltage:.1f}V  {info.temperature}C")

        if current == args.new_id:
            print(f"servo already has ID {args.new_id} — nothing to do")
            return 0

        if not args.yes and not confirm(
                f"program ID {current} -> {args.new_id}? [y/N] "):
            print("aborted")
            return 1

        bus.change_id(current, args.new_id)
        if bus.ping(args.new_id) is None:
            raise BenchError(
                f"servo did not answer at new ID {args.new_id} after the write",
                "power-cycle the servo and re-run the scan tool",
            )
        print(f"OK — servo now answers at ID {args.new_id}")
        return 0


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
