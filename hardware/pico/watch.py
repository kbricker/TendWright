"""watch — live view of the Pico bridge's nest switch (wiring check).

    uv run python -m hardware.pico.watch

Prints every state change plus a heartbeat line; Ctrl+C to stop.

Usage: watch [--port PORT]
"""

from __future__ import annotations

import argparse
import sys
import time

from hardware.errors import make_run_tool

from .reader import NestReader

run_tool = make_run_tool("check the Pico's USB cable, then re-run")


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.pico.watch",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial port override")
    args = parser.parse_args()

    with NestReader(args.port) as reader:
        state = reader.nest_state()
        print(f"connected to {reader.port_name}"
              + (f" ({reader.hello})" if reader.hello else
                 " (hello line arrives within ~5s)"))
        print(f"nest: {'OCCUPIED' if state else 'empty'} — press the switch "
              f"or seat a blank; Ctrl+C to stop")
        while True:
            new = reader.nest_state()
            if new != state:
                state = new
                print(f"[{time.strftime('%H:%M:%S')}] nest -> "
                      f"{'OCCUPIED' if state else 'empty'}")
            time.sleep(0.05)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
