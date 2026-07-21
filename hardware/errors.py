"""Shared error type + CLI wrapper for the hardware tools.

Lives outside hardware.bench.bus so serial-only consumers (the Pico
bridge) don't transitively import the Feetech servo SDK.
"""

from __future__ import annotations

import sys
from typing import Callable

import serial


class BenchError(Exception):
    """A user-facing failure: printed as one line + optional hint, exit 2."""

    def __init__(self, message: str, hint: str | None = None):
        super().__init__(message)
        self.hint = hint


def make_run_tool(serial_lost_hint: str) -> Callable[[Callable[[], int]], int]:
    """Build a main() wrapper: BenchError -> clean one-line exit code 2.
    serial_lost_hint is device-specific advice for a mid-session unplug."""

    def run_tool(run: Callable[[], int]) -> int:
        try:
            return run()
        except BenchError as exc:
            print(f"error: {exc}", file=sys.stderr)
            if exc.hint:
                print(f"hint:  {exc.hint}", file=sys.stderr)
            return 2
        except serial.SerialException as exc:
            print(f"error: serial device lost mid-session ({exc})",
                  file=sys.stderr)
            print(f"hint:  {serial_lost_hint}", file=sys.stderr)
            return 2
        except EOFError:
            print("\naborted (stdin closed — pass --yes for scripted use)",
                  file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            print()
            return 130

    return run_tool
