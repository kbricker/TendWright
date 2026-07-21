"""Host-side reader for the Pico sensor bridge.

Opens the Pico's USB-CDC serial port, consumes the firmware's JSON lines,
and exposes the latest debounced nest state. This is the sensor half of
orchestrator.cell.PicoCell.

Port resolution: --port/argument wins; else (Linux) a udev alias
/dev/tty-pico if present (see the hardware plan's udev-rule note); else
the single serial device with the Raspberry Pi USB VID (0x2E8A). Note
that VID matches any RP2-family CDC board — on a bench with several USB
serial devices, set up the udev alias.
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
UDEV_ALIAS = "/dev/tty-pico"


def resolve_pico_port(port: str | None) -> str:
    if port:
        return port
    if sys.platform.startswith("linux") and os.path.exists(UDEV_ALIAS):
        return UDEV_ALIAS
    picos = [p.device for p in list_ports.comports() if p.vid == PICO_VID]
    if len(picos) == 1:
        return picos[0]
    if not picos:
        raise BenchError(
            "no Pico found (USB VID 0x2E8A)",
            "plug the Pico in; if MicroPython isn't flashed yet see "
            "hardware/mockbay/README.md",
        )
    raise BenchError(f"multiple Picos found: {', '.join(sorted(picos))}",
                     "pick one explicitly (port argument / --port)")


class NestReader:
    """Polls the firmware's JSON stream; stale data is an error, never a
    silently-frozen sensor reading."""

    def __init__(self, port: str | None = None, stale_after: float = 1.0):
        self.port_name = resolve_pico_port(port)
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

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    def __enter__(self) -> "NestReader":
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
        elif "nest" in doc:
            self.last = doc
            self._last_at = time.monotonic()

    def _pump(self) -> None:
        """Drain lines already BUFFERED, keeping the newest sample. Bounded:
        only reads while bytes are waiting, so a healthy 20 Hz stream can
        never trap us in the drain loop (a partial trailing line costs at
        most one 0.2 s readline timeout)."""
        try:
            while self._ser.in_waiting:
                line = self._ser.readline()
                if not line:
                    return
                self._handle(line)
        except serial.SerialException as exc:
            raise BenchError(
                f"lost the Pico on {self.port_name}: {exc}",
                "check the USB cable, then re-run",
            ) from exc

    def nest_state(self) -> bool:
        self._pump()
        if self.last is None:
            # First read after connect: wait up to stale_after for one sample.
            deadline = time.monotonic() + self.stale_after
            while self.last is None and time.monotonic() < deadline:
                time.sleep(0.02)
                self._pump()
            if self.last is None:
                raise BenchError(
                    f"no data from the Pico on {self.port_name}",
                    "is main.py flashed? a serial terminal should show a "
                    "JSON line every 50 ms",
                )
        if time.monotonic() - self._last_at > self.stale_after:
            raise BenchError(
                f"Pico stream went stale (> {self.stale_after}s without data)",
                "check the USB connection; the firmware may have crashed — "
                "power-cycle the Pico",
            )
        return bool(self.last["nest"])
