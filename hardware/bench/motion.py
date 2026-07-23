"""Shared motion helpers for tools that command arm positions.

Settle detection is plant-gated: a joint counts as arrived only when its
ACTUAL position is at the target and it has stopped moving — never the
command, never a timer. (The #604 lesson: the physical servos lag the
command; gating on anything but the plant latches too early.)
"""

from __future__ import annotations

import time
from typing import Callable

import serial

from .bus import BenchError, FeetechBus

SETTLE_TOL_TICKS = 25  # "arrived" position tolerance
STILL_TICKS = 4  # per-sample movement below this counts as "still"
STILL_SAMPLES = 3  # consecutive still+arrived samples = settled
SETTLE_GRACE_S = 5.0  # deadline slack beyond the ideal travel time
SAMPLE_S = 0.05  # settle-poll cadence; also the e-stop key latency bound
HALT_SPEED = 200  # moot in practice: a halt re-goals to the present position


class EStop(Exception):
    """The caller's poll_key reported a keypress during motion."""


def halt_all(bus: FeetechBus, ids: list[int]) -> None:
    """Stop motion by re-goaling every joint to where it is right now.

    Best-effort PER JOINT: one dead servo must not stop the others from
    being halted — the caller's torque-off cleanup still runs either way.
    """
    for servo_id in ids:
        try:
            bus.move_to(servo_id, bus.read_position(servo_id),
                        speed=HALT_SPEED)
        except (BenchError, serial.SerialException):
            continue


def wait_settle(bus: FeetechBus, targets: dict[int, int], speed: int,
                label: str,
                poll_key: Callable[[float], str | None] | None = None,
                ) -> None:
    """Poll until every joint in targets is at its target AND still.

    poll_key (when given) is called between samples with a timeout; any
    non-None return raises EStop — the caller owns the halt/hold response.
    Without poll_key the wait is uninterruptible except Ctrl+C (teach's
    replay approach). A joint that never settles (obstruction, too weak)
    halts the arm and raises BenchError; the arm is left HOLDING at its
    present position — the caller decides how torque comes off.
    """
    ids = sorted(targets)
    prev = {i: bus.read_position(i) for i in ids}
    still: dict[int, int] = {i: 0 for i in ids}
    worst_travel = max(abs(prev[i] - targets[i]) for i in ids)
    # Servo speed units approximate ticks/s closely enough for a deadline.
    deadline = (time.monotonic() + SETTLE_GRACE_S
                + worst_travel / max(1, speed))
    while True:
        start = time.monotonic()
        done = True
        worst = 0
        for i in ids:
            pos = bus.read_position(i)
            err = abs(pos - targets[i])
            worst = max(worst, err)
            if err <= SETTLE_TOL_TICKS and abs(pos - prev[i]) <= STILL_TICKS:
                still[i] += 1
            else:
                still[i] = 0
            prev[i] = pos
            if still[i] < STILL_SAMPLES:
                done = False
        print(f"\r  {label}: worst error {worst:>4} ticks   ",
              end="", flush=True)
        if done:
            print()
            return
        if time.monotonic() > deadline:
            print()
            halt_all(bus, ids)
            lagging = sorted(i for i in ids if still[i] < STILL_SAMPLES)
            raise BenchError(
                f"joint(s) {lagging} did not settle at their target "
                f"(worst error {worst} ticks) — halted in place",
                "a joint may be obstructed or too weak for this pose or "
                "speed; clear the workspace and re-run (check the joint's "
                "calibrated range if it repeats)",
            )
        remaining = max(0.0, SAMPLE_S - (time.monotonic() - start))
        if poll_key is None:
            time.sleep(remaining)
        else:
            key = poll_key(remaining)
            if key is not None:
                print()
                raise EStop
