"""Cross-platform single-keypress input for the interactive tools."""

from __future__ import annotations

import sys


def require_interactive() -> None:
    """Refuse early when stdin can't deliver keypresses — for tools whose
    e-stop key is the safety channel, this must fail BEFORE any motion."""
    if not sys.stdin.isatty():
        from hardware.errors import BenchError

        raise BenchError(
            "this tool needs an interactive terminal",
            "allocate one: `ssh -t cell1 '...'` (note -t), or run it "
            "from a shell on the box",
        )


if sys.platform == "win32":
    import msvcrt

    def flush_input() -> None:
        """Drop pending keypresses (e.g. a stray extra Enter after a
        read_key phase) so the next input() prompt actually waits."""
        while msvcrt.kbhit():
            msvcrt.getwch()

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

    def flush_input() -> None:
        """Drop pending keypresses (e.g. a stray extra Enter after a
        read_key phase) so the next input() prompt actually waits."""
        if sys.stdin.isatty():
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)

    def read_key(timeout: float | None = None) -> str | None:
        require_interactive()
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
