"""Cross-platform single-keypress input for the interactive tools."""

from __future__ import annotations

import sys


if sys.platform == "win32":
    import msvcrt

    def read_key(timeout: float | None = None) -> str | None:
        """One keypress, or None if timeout elapses without input."""
        import time

        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch in ("\x00", "\xe0"):  # arrow/function key prefix
                    msvcrt.getwch()
                    return ""
                return ch
            if deadline is not None and time.monotonic() >= deadline:
                return None
            time.sleep(0.01)

else:
    import select
    import termios
    import tty

    def read_key(timeout: float | None = None) -> str | None:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if not ready:
                return None
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
