"""Thin, safe wrapper around the Feetech servo bus (STS3215).

Isolates every scservo_sdk detail (registers, endianness, comm-result
handling) so the tools stay tiny and an assembly-day register surprise is
a one-file fix. All hardware/port failures surface as BenchError with an
actionable hint — tools convert those to clean one-line exits.
"""

from __future__ import annotations

import glob
import sys
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports
from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from hardware.errors import BenchError, make_run_tool

BAUD = 1_000_000
PROTOCOL_END = 0  # STS series is little-endian ("SCS_END = 0")

# STS3215 memory map (Feetech STS series; matches the LeRobot control table)
REG_FIRMWARE_MAJOR = 0
REG_FIRMWARE_MINOR = 1
REG_MODEL = 3  # u16
REG_ID = 5  # EEPROM
REG_TORQUE_ENABLE = 40
REG_ACCELERATION = 41
REG_GOAL_POSITION = 42  # u16
REG_GOAL_SPEED = 46  # u16
REG_LOCK = 55  # 0 = EEPROM unlocked, 1 = locked
REG_PRESENT_POSITION = 56  # u16
REG_PRESENT_VOLTAGE = 62  # units of 0.1 V
REG_PRESENT_TEMPERATURE = 63  # deg C

# Feetech IDs 0-253 are all valid (254 is broadcast); a servo can sit at 0.
SCAN_IDS = list(range(0, 254))
POSITION_RANGE = (0, 4095)  # single-turn tick range, 2048 = center


@dataclass
class ServoInfo:
    servo_id: int
    model: int
    firmware: str
    voltage: float  # V
    temperature: int  # deg C
    position: int  # ticks


def resolve_port(port: str | None) -> str:
    """Use the given port, or find the likely servo adapter for this OS."""
    if port:
        return port
    if sys.platform.startswith("linux"):
        acm = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
        if len(acm) == 1:
            return acm[0]
        if not acm:
            raise BenchError(
                "no serial adapter found (/dev/ttyACM* or /dev/ttyUSB*)",
                "plug in the servo bus adapter; check `ls /dev/ttyACM*` and that "
                "your user is in the `dialout` group",
            )
        raise BenchError(
            f"multiple serial adapters found: {', '.join(acm)}",
            "pick one explicitly with --port",
        )
    candidates = [p.device for p in list_ports.comports()]
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise BenchError(
            "no COM ports found",
            "plug in the servo bus adapter, or pass --port COMx",
        )
    raise BenchError(
        f"multiple COM ports found: {', '.join(sorted(candidates))}",
        "pick one explicitly with --port COMx",
    )


class FeetechBus:
    """Open serial bus + packet handler with friendly errors."""

    def __init__(self, port: str | None, baud: int = BAUD):
        self.port_name = resolve_port(port)
        self._port = PortHandler(self.port_name)
        self._packet = PacketHandler(PROTOCOL_END)
        try:
            ok = self._port.openPort()
        except serial.SerialException as exc:
            raise BenchError(
                f"could not open {self.port_name}: {exc}",
                "check the cable, that nothing else has the port open, and "
                "(Linux) that your user is in the `dialout` group",
            ) from exc
        if not ok:
            raise BenchError(f"could not open {self.port_name}")
        if not self._port.setBaudRate(baud):
            self._port.closePort()
            raise BenchError(f"could not set baud rate {baud} on {self.port_name}")

    # ------------------------------------------------------------- lifecycle
    def close(self) -> None:
        try:
            self._port.closePort()
        except Exception:
            pass

    def __enter__(self) -> "FeetechBus":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------ raw
    def _clear_latch(self) -> None:
        """Reset the SDK's is_using flag. An exception mid-transaction leaves
        it latched True, after which EVERY call returns COMM_PORT_BUSY without
        touching the wire — fatal for torque-off cleanup paths. We are strictly
        single-threaded, so a set flag at call entry is always stale."""
        self._port.is_using = False

    def _check(self, servo_id: int, comm: int, error: int, what: str) -> None:
        if comm != COMM_SUCCESS:
            raise BenchError(
                f"servo {servo_id}: {what} failed — "
                f"{self._packet.getTxRxResult(comm).strip()}",
                "check power, wiring, and that the servo ID is on the bus "
                "(run the scan tool)",
            )
        if error != 0:
            raise BenchError(
                f"servo {servo_id}: {what} — device error "
                f"{self._packet.getRxPacketError(error).strip() or error}"
            )

    def read_u8(self, servo_id: int, addr: int, what: str) -> int:
        self._clear_latch()
        value, comm, error = self._packet.read1ByteTxRx(self._port, servo_id, addr)
        self._check(servo_id, comm, error, what)
        return value

    def read_u16(self, servo_id: int, addr: int, what: str) -> int:
        self._clear_latch()
        value, comm, error = self._packet.read2ByteTxRx(self._port, servo_id, addr)
        self._check(servo_id, comm, error, what)
        return value

    def write_u8(self, servo_id: int, addr: int, value: int, what: str) -> None:
        self._clear_latch()
        comm, error = self._packet.write1ByteTxRx(self._port, servo_id, addr, value)
        self._check(servo_id, comm, error, what)

    def write_u16(self, servo_id: int, addr: int, value: int, what: str) -> None:
        self._clear_latch()
        comm, error = self._packet.write2ByteTxRx(self._port, servo_id, addr, value)
        self._check(servo_id, comm, error, what)

    # ------------------------------------------------------------- services
    def ping(self, servo_id: int) -> int | None:
        """Model number if a servo answers at this ID, else None."""
        self._clear_latch()
        model, comm, _error = self._packet.ping(self._port, servo_id)
        return model if comm == COMM_SUCCESS else None

    def scan(self, ids: list[int], progress: bool = False) -> list[int]:
        """IDs (from the given list) that answer a ping."""
        found = []
        for i, servo_id in enumerate(ids):
            if progress and i % 25 == 0 and i:
                print(f"  ...scanned {i}/{len(ids)}", file=sys.stderr)
            if self.ping(servo_id) is not None:
                found.append(servo_id)
        return found

    def info(self, servo_id: int) -> ServoInfo:
        model = self.read_u16(servo_id, REG_MODEL, "read model")
        fw_major = self.read_u8(servo_id, REG_FIRMWARE_MAJOR, "read firmware")
        fw_minor = self.read_u8(servo_id, REG_FIRMWARE_MINOR, "read firmware")
        voltage = self.read_u8(servo_id, REG_PRESENT_VOLTAGE, "read voltage")
        temp = self.read_u8(servo_id, REG_PRESENT_TEMPERATURE, "read temperature")
        pos = self.read_u16(servo_id, REG_PRESENT_POSITION, "read position")
        return ServoInfo(
            servo_id=servo_id,
            model=model,
            firmware=f"{fw_major}.{fw_minor}",
            voltage=voltage / 10.0,
            temperature=temp,
            position=pos,
        )

    def read_position(self, servo_id: int) -> int:
        return self.read_u16(servo_id, REG_PRESENT_POSITION, "read position")

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        self.write_u8(
            servo_id, REG_TORQUE_ENABLE, 1 if enabled else 0,
            f"torque {'on' if enabled else 'off'}",
        )

    def safe_torque_off(self, servo_ids: list[int]) -> None:
        """Best-effort torque cut for cleanup/e-stop paths: clears the SDK
        latch, retries each servo, and NEVER fails silently — any servo that
        could not be safed is shouted to stderr (the power switch is then the
        real e-stop)."""
        failed: list[int] = []
        for servo_id in servo_ids:
            for attempt in range(3):
                try:
                    self._clear_latch()
                    self.set_torque(servo_id, False)
                    break
                except (BenchError, serial.SerialException):
                    if attempt == 2:
                        failed.append(servo_id)
                    else:
                        time.sleep(0.05)
        if failed:
            print(
                f"\n*** WARNING: could not confirm torque OFF for servo(s) "
                f"{failed} — the arm may still be energized and holding. "
                f"Cut power at the switch before touching it. ***",
                file=sys.stderr,
            )

    def move_to(self, servo_id: int, position: int, speed: int = 400,
                acceleration: int = 30) -> None:
        """Command a position with modest speed/acceleration defaults."""
        lo, hi = POSITION_RANGE
        position = max(lo, min(hi, int(position)))
        self.write_u8(servo_id, REG_ACCELERATION, acceleration, "set acceleration")
        self.write_u16(servo_id, REG_GOAL_SPEED, speed, "set speed")
        self.write_u16(servo_id, REG_GOAL_POSITION, position, "set goal position")

    def change_id(self, current_id: int, new_id: int) -> None:
        """EEPROM ID rewrite: unlock -> write -> re-lock (on the new ID)."""
        self.write_u8(current_id, REG_LOCK, 0, "unlock EEPROM")
        self.write_u8(current_id, REG_ID, new_id, "write new ID")
        time.sleep(0.1)  # let EEPROM settle before addressing the new ID
        self.write_u8(new_id, REG_LOCK, 1, "re-lock EEPROM")


# Servo-bus flavored CLI wrapper (the unplug hint is servo-specific).
run_tool = make_run_tool(
    "servos keep bus power and HOLD their last command — use the power "
    "switch, then reconnect and re-run scan")


def confirm(prompt: str) -> bool:
    """y/yes confirmation (EOFError handled by run_tool)."""
    return input(prompt).strip().lower() in ("y", "yes")
