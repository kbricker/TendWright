"""run — drive and observe the conveyor Pico (bring-up + the v0 proofs).

    uv run python -m hardware.conveyor.run watch
    uv run python -m hardware.conveyor.run hold --motor s1 --duty 60
    uv run python -m hardware.conveyor.run demo

`hold` keeps one motor running until Ctrl+C, feeding the firmware's
watchdog. It is also how you run plan #835's unplug test: start it, pull
the USB, and the motors must coast within the timeout window.

`demo` is the per-segment proof for #835 — every motor, both directions,
two distinct speeds, one at a time so a failure names its own segment.

Usage: run {watch,hold,demo} [--port PORT]
"""

from __future__ import annotations

import argparse
import sys
import time

from hardware.errors import make_run_tool

from .driver import ConveyorDriver

run_tool = make_run_tool("check the conveyor Pico's USB cable, then re-run")

KEEPALIVE_S = 0.25   # firmware coasts at 1000 ms without traffic
DEMO_DUTIES = (35, 75)
DEMO_HOLD_S = 2.5


def _report(driver: ConveyorDriver) -> None:
    for doc in driver.drain_replies():
        if "error" in doc:
            print(f"  firmware: {doc['error']} — {doc.get('detail', '')}")
        elif "note" in doc:
            print(f"  firmware: {doc['note']}")
        elif doc.get("rejected"):
            print(f"  REJECTED: {doc['rejected']}")


def _describe(state: dict) -> str:
    parts = []
    for name in sorted(state):
        m = state[name]
        parts.append(f"{name}={m['duty']:>3}%{m['dir'][0]}" if m["duty"]
                     else f"{name}=  off")
    return "  ".join(parts)


def cmd_watch(driver: ConveyorDriver) -> int:
    print(f"connected to {driver.port_name}"
          + (f" ({driver.hello})" if driver.hello else
             " (hello line arrives within ~5s)"))
    print("streaming motor state; Ctrl+C to stop")
    last = None
    while True:
        state = driver.state()
        if state != last:
            last = state
            print(f"[{time.strftime('%H:%M:%S')}] {_describe(state)}")
        _report(driver)
        time.sleep(0.05)


def cmd_hold(driver: ConveyorDriver, motor: str, duty: int,
             direction: str) -> int:
    print(f"connected to {driver.port_name}")
    driver.set(motor, duty=duty, direction=direction)
    print(f"{motor} -> {duty}% {direction}; Ctrl+C to stop")
    print("(pull the USB to test the watchdog — motors must coast)")
    while True:
        driver.keepalive()
        state = driver.state()
        print(f"\r{_describe(state)}   ", end="", flush=True)
        _report(driver)
        time.sleep(KEEPALIVE_S)


def cmd_demo(driver: ConveyorDriver) -> int:
    print(f"connected to {driver.port_name}")
    state = driver.state()
    motors = sorted(state)
    print(f"motors: {', '.join(motors)}")
    print(f"each one alone, both directions, {DEMO_DUTIES[0]}% then "
          f"{DEMO_DUTIES[1]}% — watch the belt, not the terminal\n")

    for motor in motors:
        for direction in ("fwd", "rev"):
            for duty in DEMO_DUTIES:
                driver.set(motor, duty=duty, direction=direction)
                print(f"  {motor} {direction} {duty}% ", end="", flush=True)
                deadline = time.monotonic() + DEMO_HOLD_S
                while time.monotonic() < deadline:
                    driver.keepalive()
                    time.sleep(KEEPALIVE_S)
                observed = driver.state()[motor]
                ok = observed["duty"] == duty and observed["dir"] == direction
                print("ok" if ok else f"MISMATCH — firmware reports {observed}")
                _report(driver)
        driver.set(motor, duty=0)
        time.sleep(0.3)

    driver.stop()
    print("\nall stopped. Every segment that ran at two speeds in both "
          "directions satisfies #835's bonus requirement.")
    return 0


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.conveyor.run",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial port override")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("watch", help="stream motor state")
    sub.add_parser("demo", help="per-segment two-speed bidirectional proof")
    hold = sub.add_parser("hold", help="hold one motor running")
    hold.add_argument("--motor", required=True)
    hold.add_argument("--duty", type=int, default=60)
    hold.add_argument("--dir", dest="direction", default="fwd",
                      choices=("fwd", "rev"))
    args = parser.parse_args()

    with ConveyorDriver(args.port) as driver:
        if args.cmd == "watch":
            return cmd_watch(driver)
        if args.cmd == "hold":
            return cmd_hold(driver, args.motor, args.duty, args.direction)
        return cmd_demo(driver)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
