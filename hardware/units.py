"""Joint semantic frames — ticks <-> human units (degrees / percent open).

The bench servos speak encoder ticks (4096/rev); Kyle speaks degrees with
a per-joint convention ("m1 zero is mid-range, left 90 right 90; m2 closed
is -90"). A joint's *frame* pins that convention: which tick reads as
zero, which encoder direction is displayed positive, or — for the gripper
— which ticks mean closed and open. Frames are RATIFIED data (chosen by
Kyle, stored in calibration.json v2), not captured measurements.

Pure math, zero project imports, no servo SDK: the digital twin (plan
#648) consumes the tick<->radian map from here headlessly.

    deg = positive * (tick - zero_tick) * 360 / 4096

Self-test:  uv run python -m hardware.units
"""

from __future__ import annotations

import math
from dataclasses import dataclass

TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV  # 0.087890625
RAD_PER_TICK = math.tau / TICKS_PER_REV


@dataclass(frozen=True)
class DegFrame:
    """Angle convention: zero anchor + which direction displays positive.

    `positive` relates ENCODER counts to DISPLAY sign and is independent
    of the captured motion `sign` in calibration.json (that one records
    which way the encoder counts for the joint's canonical positive
    motion; this one records which way Kyle wants to read as +)."""

    zero_tick: int
    positive: int  # +1 or -1
    label: str = ""  # human wording of the convention, shown by tools

    def __post_init__(self) -> None:
        # Guard raw construction too (the #648 twin builds frames in
        # code, not just from JSON) — same rules as frame_from_dict.
        if type(self.zero_tick) is not int \
                or not 0 <= self.zero_tick < TICKS_PER_REV:
            raise ValueError("zero_tick must be a tick 0-4095")
        if type(self.positive) is not int or self.positive not in (-1, 1):
            raise ValueError("positive must be -1 or 1")

    def deg(self, tick: float) -> float:
        return self.positive * (tick - self.zero_tick) * DEG_PER_TICK

    def tick(self, deg: float) -> int:
        return round(self.zero_tick + self.positive * deg / DEG_PER_TICK)

    def rad(self, tick: float) -> float:
        return self.positive * (tick - self.zero_tick) * RAD_PER_TICK

    def tick_from_rad(self, rad: float) -> int:
        return round(self.zero_tick + self.positive * rad / RAD_PER_TICK)

    def fmt(self, tick: float) -> str:
        return f"{self.deg(tick):+.1f}°"


@dataclass(frozen=True)
class PctFrame:
    """Gripper convention: 0% = fully closed, 100% = fully open."""

    closed_tick: int
    open_tick: int
    label: str = ""

    def __post_init__(self) -> None:
        for v in (self.closed_tick, self.open_tick):
            if type(v) is not int or not 0 <= v < TICKS_PER_REV:
                raise ValueError("closed/open_tick must be ticks 0-4095")
        if self.closed_tick == self.open_tick:
            raise ValueError("closed_tick and open_tick must differ")

    def pct(self, tick: float) -> float:
        span = self.open_tick - self.closed_tick
        return 100.0 * (tick - self.closed_tick) / span

    def tick(self, pct: float) -> int:
        span = self.open_tick - self.closed_tick
        return round(self.closed_tick + pct / 100.0 * span)

    def fmt(self, tick: float) -> str:
        return f"{self.pct(tick):.0f}% open"


Frame = DegFrame | PctFrame


def span_deg(ticks: float) -> float:
    """Frame-free magnitude conversion — spans/deltas need no zero anchor."""
    return ticks * DEG_PER_TICK


def fmt_ticks(frame: Frame | None, tick: int) -> str:
    """Shared display form: human units with ticks in parens when a
    frame exists, plain ticks otherwise."""
    if frame is not None:
        return f"{frame.fmt(tick)} ({tick}t)"
    return str(tick)


def frame_to_dict(frame: Frame) -> dict:
    if isinstance(frame, DegFrame):
        return {"unit": "deg", "zero_tick": frame.zero_tick,
                "positive": frame.positive, "label": frame.label}
    return {"unit": "pct_open", "closed_tick": frame.closed_tick,
            "open_tick": frame.open_tick, "label": frame.label}


def frame_from_dict(doc: object) -> Frame:
    """Strictly validating parse; raises ValueError on any malformation
    (the calibration loader turns that into its own BenchError)."""
    if not isinstance(doc, dict):
        raise ValueError("frame must be an object")
    unit = doc.get("unit")
    label = doc.get("label", "")
    if not isinstance(label, str):
        raise ValueError("frame label must be a string")
    # Range/type rules live in the dataclasses' __post_init__ — one
    # validator for JSON and raw construction alike.
    if unit == "deg":
        return DegFrame(zero_tick=doc.get("zero_tick"),
                        positive=doc.get("positive"), label=label)
    if unit == "pct_open":
        return PctFrame(closed_tick=doc.get("closed_tick"),
                        open_tick=doc.get("open_tick"), label=label)
    raise ValueError(f"unknown frame unit {unit!r}")


def _selftest() -> None:
    f = DegFrame(zero_tick=2048, positive=1)
    assert f.deg(2048) == 0.0
    assert abs(f.deg(3072) - 90.0) < 1e-9
    assert f.tick(90.0) == 3072
    assert abs(f.rad(3072) - math.pi / 2) < 1e-9
    assert f.tick_from_rad(math.pi / 2) == 3072

    g = DegFrame(zero_tick=2090, positive=-1)  # shoulder_pan proposal
    assert g.deg(922) > 102.0 and g.deg(3259) < -102.0
    assert g.tick(g.deg(922)) == 922  # exact roundtrip on integer ticks

    p = PctFrame(closed_tick=961, open_tick=2446)
    assert p.pct(961) == 0.0 and p.pct(2446) == 100.0
    assert 10.0 < p.pct(1114) < 11.0
    assert p.tick(0) == 961 and p.tick(100) == 2446

    inv = PctFrame(closed_tick=2446, open_tick=961)  # reversed direction
    assert inv.pct(2446) == 0.0 and inv.pct(961) == 100.0

    assert abs(span_deg(512) - 45.0) < 1e-9

    for frame in (f, g, p, inv):
        assert frame_from_dict(frame_to_dict(frame)) == frame

    for bad in (None, {}, {"unit": "deg"}, {"unit": "deg", "zero_tick": -1,
                "positive": 1}, {"unit": "deg", "zero_tick": 0, "positive": 2},
                {"unit": "deg", "zero_tick": 0, "positive": True},
                {"unit": "deg", "zero_tick": 100.0, "positive": 1},
                {"unit": "pct_open", "closed_tick": 5, "open_tick": 5},
                {"unit": "furlongs"}):
        try:
            frame_from_dict(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted bad frame {bad!r}")

    for ctor in (lambda: DegFrame(-5, 1), lambda: DegFrame(0, 0),
                 lambda: PctFrame(7, 7), lambda: PctFrame(-1, 5)):
        try:
            ctor()
        except ValueError:
            pass
        else:
            raise AssertionError("raw construction guard missed")

    assert fmt_ticks(f, 3072) == "+90.0° (3072t)"
    assert fmt_ticks(None, 3072) == "3072"
    print("units selftest OK")


if __name__ == "__main__":
    _selftest()
