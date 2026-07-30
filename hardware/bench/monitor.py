"""monitor — torque-off live position telemetry while moving joints by hand.

Cuts torque on the given servos (confirm prompt — support the arm!), then
prints their positions at ~10 Hz on one refreshing line. Move each joint
by hand and watch the numbers to verify wiring order and ranges. Ctrl+C
to stop (torque stays off — that is the tool's resting state).

Positions print in each joint's ratified units (degrees / % open) when
calibration.json carries a frame for it; raw ticks otherwise, or always
with --raw. Pre-calibration bring-up needs no calibration file.

    uv run python -m hardware.bench.monitor --ids 1-6

Usage: monitor [--ids RANGE] [--port PORT] [--hz N] [--cal FILE] [--raw]
               [--yes]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .bus import (BenchError, FeetechBus, confirm,
                  confirm_torque_cut, run_tool)


def parse_ids(spec: str) -> list[int]:
    """'1-6' or '1,2,5' -> [ints]."""
    ids: list[int] = []
    try:
        for chunk in spec.split(","):
            chunk = chunk.strip()
            if "-" in chunk:
                lo, hi = (int(part) for part in chunk.split("-", 1))
                if hi < lo:
                    raise BenchError(f"reversed range '{chunk}' in --ids",
                                     "write it low-high, e.g. '1-6'")
                if hi - lo > 253:
                    raise BenchError(f"range '{chunk}' is too wide",
                                     "servo bus IDs only go 0-253")
                ids.extend(range(lo, hi + 1))
            elif chunk:
                ids.append(int(chunk))
    except ValueError as exc:
        raise BenchError(f"could not parse --ids '{spec}': {exc}",
                         "use forms like '1-6' or '1,2,5'") from exc
    if not ids:
        raise BenchError(f"could not parse --ids '{spec}'")
    return ids


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.monitor",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ids", default="1-6", help="servo IDs (default 1-6)")
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--hz", type=float, default=10.0, help="update rate")
    parser.add_argument("--cal", default="calibration.json",
                        help="calibration file for unit display (optional)")
    parser.add_argument("--raw", action="store_true",
                        help="always print raw ticks")
    parser.add_argument("--yes", action="store_true",
                        help="skip the support-the-arm confirmation")
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    # Lazy import: calibrate imports parse_ids from THIS module — a
    # module-level import back the other way would be a cycle. (The
    # loader's proper shared home is #637's load_joint_calibration move.)
    frames = {}
    cal_path = Path(args.cal)
    if not args.raw and cal_path.exists():
        from .calibrate import load_joint_calibration
        try:
            frames = {i: c.frame for i, c in
                      load_joint_calibration(cal_path).items()
                      if c.frame is not None}
        except BenchError as exc:
            print(f"warning: {cal_path} unusable ({exc}) — printing raw "
                  f"ticks", file=sys.stderr)
    with FeetechBus(args.port) as bus:
        present = [i for i in ids if bus.ping(i) is not None]
        missing = sorted(set(ids) - set(present))
        if not present:
            raise BenchError(f"none of the servos {ids} answered",
                             "run the scan tool to see what is on the bus")
        if missing:
            print(f"warning: no answer from IDs {missing}", file=sys.stderr)

        if not confirm_torque_cut(present, args.yes):
            print("aborted")
            return 1
        for servo_id in present:
            bus.set_torque(servo_id, False)
        print(f"torque OFF on {present} — move joints by hand, Ctrl+C to stop")

        period = 1.0 / max(0.1, args.hz)
        while True:
            readings = []
            for sid in present:
                pos = bus.read_position(sid)
                frame = frames.get(sid)
                value = frame.fmt(pos) if frame else f"{pos:>4}t"
                readings.append(f"id{sid}:{value:>9}")
            print("\r" + "  ".join(readings) + "   ", end="", flush=True)
            time.sleep(period)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
