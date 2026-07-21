"""The MockCell interface the orchestrator drives, plus the fake backend.

A "cell" bundles what the orchestrator needs from the world: task-level
arm commands, the part-present sensor, and the cell's clock. Backends:

  * FakeCell (here) — instant, deterministic, scriptable faults; for FSM
    logic tests.
  * SimCell (orchestrator/simcell.py) — the P0 MuJoCo cell.
  * PicoCell (here, slot) — part_present from the #619 Pico switch bridge;
    arm commands arrive with the real arm driver (P6-era).

Command-then-verify doctrine: backends do NOT guarantee task success —
fetch_blank raises CellTaskError only for failures the device itself can
detect; whether a part is actually seated is always the sensor's word.
"""

from __future__ import annotations

from typing import Protocol


class CellTaskError(Exception):
    """A task-level failure the backend itself detected (e.g. pick missed)."""


class MockCell(Protocol):
    def fetch_blank(self) -> None: ...
    def load_nest(self) -> None: ...
    def unload_to_tray(self) -> None: ...
    def safe_retract(self) -> None: ...
    def part_present(self) -> bool: ...
    def mark_machined(self) -> None: ...
    def dwell(self, seconds: float) -> None: ...
    def now(self) -> float: ...


class FakeCell:
    """Deterministic in-memory cell with scriptable faults and a virtual
    clock (dwell just advances it — tests run instantly)."""

    def __init__(self, fail_picks: int = 0, suppress_seat: bool = False):
        self.fail_picks = fail_picks  # this many fetches raise before one works
        self.suppress_seat = suppress_seat  # sensor never reports the part
        self.holding = False
        self.nest_loaded = False
        self.parts_done = 0
        self.machined = 0
        self._time = 0.0
        self.log: list[str] = []

    def fetch_blank(self) -> None:
        self.dwell(1.0)
        if self.fail_picks > 0:
            self.fail_picks -= 1
            self.log.append("fetch: MISSED")
            raise CellTaskError("pick missed (scripted)")
        self.holding = True
        self.log.append("fetch: ok")

    def load_nest(self) -> None:
        self.dwell(1.0)
        if self.holding:
            self.holding = False
            self.nest_loaded = True
        self.log.append(f"load: nest_loaded={self.nest_loaded}")

    def unload_to_tray(self) -> None:
        self.dwell(1.0)
        if self.nest_loaded:
            self.nest_loaded = False
            self.parts_done += 1
        self.log.append(f"unload: parts_done={self.parts_done}")

    def safe_retract(self) -> None:
        self.dwell(0.5)
        self.holding = False
        self.log.append("safe_retract")

    def part_present(self) -> bool:
        return self.nest_loaded and not self.suppress_seat

    def mark_machined(self) -> None:
        self.machined += 1

    def dwell(self, seconds: float) -> None:
        self._time += seconds

    def now(self) -> float:
        return self._time


class PicoCell:
    """Backend slot for the #619 physical mock bay: part_present comes from
    the Pico switch bridge; arm task commands need the real arm driver and
    raise until that exists (P6-era)."""

    def __init__(self, port: str | None = None):
        # Deferred import: the reader module lands with plan #619.
        from hardware.pico import NestReader  # type: ignore[import-not-found]

        self._reader = NestReader(port)
        import time as _time

        self._clock = _time.monotonic

    def part_present(self) -> bool:
        return self._reader.nest_state()

    def now(self) -> float:
        return self._clock()

    def dwell(self, seconds: float) -> None:
        import time as _time

        _time.sleep(seconds)

    def mark_machined(self) -> None:
        pass

    def _no_arm(self) -> None:
        raise NotImplementedError(
            "PicoCell has no arm driver yet — sensor-only until the SO-101 "
            "driver lands (P6-era)")

    def fetch_blank(self) -> None:
        self._no_arm()

    def load_nest(self) -> None:
        self._no_arm()

    def unload_to_tray(self) -> None:
        self._no_arm()

    def safe_retract(self) -> None:
        self._no_arm()
