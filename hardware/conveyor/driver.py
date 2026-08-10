"""Host side of the conveyor motor bridge (plan #835).

Opens the conveyor Pico's USB-CDC port, consumes its JSON lines, and
sends absolute motor commands. Deliberately mirrors
hardware.pico.reader.NestReader — bounded drain, staleness is a typed
error, never a silently-frozen reading — rather than generalizing it
into a shared abstraction the two bridges would then both be coupled to.

Port resolution differs from NestReader's on purpose. Both boards enumerate
under the same Raspberry Pi USB VID, so once the conveyor is plugged in
alongside the nest bridge, "the single device with VID 0x2E8A" no longer
identifies anything. This resolver PROBES each candidate — sends a ping and
waits for a reply naming the conveyor firmware — so it stays correct with
any number of RP2 boards attached.

The watchdog is a deadman, so a running motor requires a live host: the
firmware coasts every motor if nothing arrives within its timeout window.
Any call here counts as traffic, and keepalive() exists for a caller that
wants motors held without otherwise having something to say.
"""

from __future__ import annotations

import json
import os
import sys
import time

import serial
from serial.tools import list_ports

from hardware.errors import BenchError

PICO_VID = 0x2E8A  # Raspberry Pi (MicroPython CDC)
BAUD = 115200  # nominal; USB-CDC ignores it
UDEV_ALIAS = "/dev/tty-conveyor"
FIRMWARE_ID = "tendwright-conveyor"

DIRECTIONS = ("fwd", "rev")
PROBE_SECONDS = 1.5


def _probe(port: str) -> bool:
    try:
        with serial.Serial(port, BAUD, timeout=0.2) as ser:
            ser.write(b'{"cmd":"ping"}\n')
            ser.flush()
            deadline = time.monotonic() + PROBE_SECONDS
            while time.monotonic() < deadline:
                line = ser.readline()
                if not line:
                    continue
                try:
                    doc = json.loads(line.decode("ascii", "replace"))
                except json.JSONDecodeError:
                    continue
                if doc.get("hello") == FIRMWARE_ID:
                    return True
                if doc.get("ack") == "ping" and "motors" in doc:
                    return True
    except (serial.SerialException, OSError):
        return False
    return False


def resolve_conveyor_port(port: str | None) -> str:
    if port:
        return port
    if sys.platform.startswith("linux") and os.path.exists(UDEV_ALIAS):
        return UDEV_ALIAS
    candidates = [p.device for p in list_ports.comports() if p.vid == PICO_VID]
    if not candidates:
        raise BenchError(
            "no Pico found (USB VID 0x2E8A)",
            "plug the conveyor Pico in; if MicroPython isn't flashed yet see "
            "hardware/conveyor/README.md",
        )
    found = [p for p in sorted(candidates) if _probe(p)]
    if len(found) == 1:
        return found[0]
    if not found:
        raise BenchError(
            f"found {len(candidates)} RP2 board(s) but none is running the "
            f"conveyor firmware ({', '.join(sorted(candidates))})",
            "is hardware/conveyor/firmware/main.py copied to the Pico as "
            "main.py? the nest bridge answers to a different hello",
        )
    raise BenchError(
        f"multiple conveyor Picos found: {', '.join(found)}",
        "pick one explicitly (--port), or set up the udev alias",
    )


class ConveyorDriver:
    """Sends absolute motor commands and tracks the streamed state. A stale
    stream is an error, never a stale duty reported as current."""

    def __init__(self, port: str | None = None, stale_after: float = 1.0):
        self.port_name = resolve_conveyor_port(port)
        self.stale_after = stale_after
        try:
            self._ser = serial.Serial(self.port_name, BAUD, timeout=0.2)
        except serial.SerialException as exc:
            raise BenchError(f"could not open {self.port_name}: {exc}",
                             "check the cable and (Linux) dialout membership"
                             ) from exc
        self.last: dict | None = None
        self._last_at = 0.0
        self.hello: dict | None = None
        self.replies: list[dict] = []

    def close(self) -> None:
        # Explicit stop before deinit: closing the port would eventually trip
        # the firmware's watchdog anyway, but only after the timeout window,
        # and "eventually" is not what you want from a teardown path.
        try:
            self._send({"cmd": "stop"})
            self._ser.close()
        except Exception:
            pass

    def __enter__(self) -> "ConveyorDriver":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _handle(self, line: bytes) -> None:
        try:
            doc = json.loads(line.decode("ascii", "replace"))
        except json.JSONDecodeError:
            return  # partial line at connect time
        if "hello" in doc:
            self.hello = doc
        elif "motors" in doc and "seq" in doc:
            self.last = doc
            self._last_at = time.monotonic()
        else:
            # Acks, errors and watchdog notices. Bounded so a long run with a
            # chatty firmware cannot grow this without limit.
            self.replies.append(doc)
            del self.replies[:-64]

    def _pump(self) -> None:
        try:
            while self._ser.in_waiting:
                line = self._ser.readline()
                if not line:
                    return
                self._handle(line)
        except serial.SerialException as exc:
            raise BenchError(
                f"lost the conveyor Pico on {self.port_name}: {exc}",
                "check the USB cable, then re-run",
            ) from exc

    def _send(self, doc: dict) -> None:
        try:
            self._ser.write((json.dumps(doc) + "\n").encode("ascii"))
            self._ser.flush()
        except serial.SerialException as exc:
            raise BenchError(
                f"lost the conveyor Pico on {self.port_name}: {exc}",
                "check the USB cable, then re-run",
            ) from exc

    def state(self) -> dict:
        self._pump()
        if self.last is None:
            deadline = time.monotonic() + self.stale_after
            while self.last is None and time.monotonic() < deadline:
                time.sleep(0.02)
                self._pump()
            if self.last is None:
                raise BenchError(
                    f"no data from the conveyor Pico on {self.port_name}",
                    "is main.py flashed? a serial terminal should show a "
                    "JSON line every 50 ms",
                )
        if time.monotonic() - self._last_at > self.stale_after:
            raise BenchError(
                f"conveyor stream went stale (> {self.stale_after}s without "
                f"data)",
                "check the USB connection; the firmware may have crashed — "
                "power-cycle the Pico",
            )
        return dict(self.last["motors"])

    def set(self, motor: str, duty: int | None = None,
            direction: str | None = None) -> None:
        # Validated here as well as in the firmware, because the useful error
        # is the one raised at the call site with a traceback, not a JSON
        # rejection that surfaces two poll cycles later.
        if duty is None and direction is None:
            raise BenchError(f"set({motor!r}) with nothing to set",
                             "pass duty= and/or direction=")
        if duty is not None and not 0 <= duty <= 100:
            raise BenchError(f"duty {duty} out of range 0-100",
                             "duty is a percentage; direction is separate")
        if direction is not None and direction not in DIRECTIONS:
            raise BenchError(f"direction {direction!r} is not one of "
                             f"{', '.join(DIRECTIONS)}")
        cmd: dict = {"cmd": "set", "motor": motor}
        if duty is not None:
            cmd["duty"] = duty
        if direction is not None:
            cmd["dir"] = direction
        self._send(cmd)

    def set_many(self, specs: dict[str, dict]) -> None:
        self._send({"cmd": "set", "motors": specs})

    def stop(self) -> None:
        self._send({"cmd": "stop"})

    def keepalive(self) -> None:
        self._send({"cmd": "ping"})

    def drain_replies(self) -> list[dict]:
        self._pump()
        out, self.replies = self.replies, []
        return out
