"""monitor — torque-off live position telemetry while moving joints by hand.

Turns torque OFF on the given servos, then prints their positions at
~10 Hz on one refreshing line. Move each joint by hand and watch the
numbers to verify wiring order and ranges. Ctrl+C to stop.

    uv run python -m hardware.bench.monitor --ids 1-6

Usage: monitor [--ids RANGE] [--port PORT] [--hz N]
"""

from __future__ import annotations

import argparse
import sys
import time

from .bus import BenchError, FeetechBus, run_tool


def parse_ids(spec: str) -> list[int]:
    """'1-6' or '1,2,5' -> [ints]."""
    ids: list[int] = []
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if "-" in chunk:
            lo, hi = chunk.split("-", 1)
            ids.extend(range(int(lo), int(hi) + 1))
        elif chunk:
            ids.append(int(chunk))
    if not ids:
        raise BenchError(f"could not parse --ids '{spec}'")
    return ids


def run() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", default="1-6", help="servo IDs (default 1-6)")
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--hz", type=float, default=10.0, help="update rate")
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    with FeetechBus(args.port) as bus:
        present = [i for i in ids if bus.ping(i) is not None]
        missing = sorted(set(ids) - set(present))
        if not present:
            raise BenchError(f"none of the servos {ids} answered",
                             "run the scan tool to see what is on the bus")
        if missing:
            print(f"warning: no answer from IDs {missing}", file=sys.stderr)

        for servo_id in present:
            bus.set_torque(servo_id, False)
        print(f"torque OFF on {present} — move joints by hand, Ctrl+C to stop")

        period = 1.0 / max(0.1, args.hz)
        try:
            while True:
                readings = [f"id{sid}:{bus.read_position(sid):>4}"
                            for sid in present]
                print("\r" + "  ".join(readings) + "   ", end="", flush=True)
                time.sleep(period)
        except KeyboardInterrupt:
            print()
            return 0


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
