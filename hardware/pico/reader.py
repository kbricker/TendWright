"""Host-side reader for the Pico sensor bridge.

Opens the Pico's USB-CDC serial port, consumes the firmware's JSON lines,
and exposes the latest debounced nest state. This is the sensor half of
orchestrator.cell.PicoCell.

Port resolution: --port/argument wins; else (Linux) a udev alias
/dev/tty-pico if present; else the serial device with the Raspberry Pi
USB VID (0x2E8A) — and if more than one board answers to that VID, the
one whose output looks like THIS firmware (plan #848).

The VID matches any RP2-family CDC board, so "the single 0x2E8A device"
stopped identifying anything once the conveyor bridge (#835) put a second
Pico on the bench permanently.
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
FIRMWARE_ID = "tendwright-pico"
PROBE_SECONDS = 1.5


def looks_like_nest_bridge(doc: dict) -> bool:
    # Identified by its STREAM, not only its hello. The firmware emits a
    # nest sample every 50 ms and re-emits hello only every ~5 s, so waiting
    # for the hello would make a probe 100x slower than it needs to be for
    # no extra certainty — no other board on this bench sends {"nest": ...}.
    return doc.get("hello") == FIRMWARE_ID or "nest" in doc


def _probe(port: str) -> bool:
    # Read-only. The conveyor bridge answers a ping, but this one never reads
    # stdin, so probing has to be passive — which is also why it must stay
    # passive here: writing to a board we have not identified yet is how you
    # send a motor command to a sensor, or worse, the reverse.
    try:
        with serial.Serial(port, BAUD, timeout=0.2) as ser:
            deadline = time.monotonic() + PROBE_SECONDS
            while time.monotonic() < deadline:
                line = ser.readline()
                if not line:
                    continue
                try:
                    doc = json.loads(line.decode("ascii", "replace"))
                except json.JSONDecodeError:
                    continue  # partial line at connect time
                if isinstance(doc, dict) and looks_like_nest_bridge(doc):
                    return True
    except (serial.SerialException, OSError):
        return False
    return False


def resolve_pico_port(port: str | None) -> str:
    if port:
        return port
    if sys.platform.startswith("linux") and os.path.exists(UDEV_ALIAS):
        return UDEV_ALIAS
    picos = [p.device for p in list_ports.comports() if p.vid == PICO_VID]
    if not picos:
        raise BenchError(
            "no Pico found (USB VID 0x2E8A)",
            "plug the Pico in; if MicroPython isn't flashed yet see "
            "hardware/mockbay/README.md",
        )
    if len(picos) == 1:
        # Fast path: one candidate needs no probe. If it turns out to be some
        # other RP2 board, NestReader says so within stale_after — a clearer
        # failure than a 1.5 s probe that reports "no nest bridge found" while
        # the only board attached is sitting right there.
        return picos[0]
    found = [p for p in sorted(picos) if _probe(p)]
    if len(found) == 1:
        return found[0]
    if not found:
        raise BenchError(
            f"found {len(picos)} RP2 board(s), none running the nest bridge "
            f"({', '.join(sorted(picos))})",
            "is hardware/pico/firmware/main.py flashed as main.py? the "
            "conveyor bridge answers to a different hello",
        )
    raise BenchError(f"multiple nest bridges found: {', '.join(found)}",
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
