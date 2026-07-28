"""Guarded motion states — safety invariants the machine VERIFIES.

Plan #649. The bench tools were linear scripts: "open the elbow, then
sweep the shoulder" was a sequence of commands, and bench run 2 proved
what that misses — the elbow had been commanded open earlier, refolded
in between, and the shoulder swept into the table anyway. A command sent
is not a joint moved.

So the rule becomes a verified property with two checkpoints:

  * ENTRY GUARDS read the ACTUAL encoders. The sweep is unreachable
    unless the held joints physically read open enough right now.
  * IN-MOTION INVARIANTS re-check the same holds every settle sample
    while the sweep runs; a joint that sags, is bumped, or browns out
    aborts the routine to halt-and-hold instead of continuing.

The check is DIRECTIONAL and tight. Directional because the danger is a
joint falling back TOWARD its fold — one bumped further open is safer,
not a reason to abort mid-air. Tight because the twin (#648) measured
the real envelope: from the 90-deg hold, an elbow sag of about 5 deg
(3 deg if the wrist sags too) is enough to put the gripper through the
table during the m2 sweep. The tolerance below sits under that.

This complements the digital twin (#648) rather than duplicating it: the
twin predicts whether a PLAN would collide, offline, from a model that
can be wrong; guards check whether REALITY still matches the plan, live,
from the encoders. Twin catches bad plans; guards catch a bad world.

Selftest (no hardware — a fake bus forces every violation path):

    uv run python -m hardware.bench.guards
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

from hardware.units import Frame, fmt_ticks, span_deg

from .bus import BenchError

# How far a held joint may sag back toward its fold before the guard
# refuses. NOT a comfort setting: the twin measured only ~35 ticks
# (3.1 deg) of combined elbow+wrist sag before the m2 sweep drives the
# gripper into the table, so this must stay well under that. It is
# deliberately close to motion.SETTLE_TOL_TICKS (25) — a joint that
# cannot hold within a couple of degrees of its clearance is a fault,
# not a tolerance to widen.
HOLD_SAG_TOL_TICKS = 30

ENTRY_PHASE = "clearance entry guard"
MOTION_PHASE = "in-motion invariant"


@dataclass(frozen=True)
class Hold:
    """A joint that must physically BE open for the next move to be safe.

    `opening` is the tick direction that opens this joint away from its
    fold (+1 or -1), so the check can be one-sided: sagging back toward
    the fold fails, opening further does not."""

    joint: int
    tick: int
    opening: int
    frame: Frame | None = None
    tol: int = HOLD_SAG_TOL_TICKS

    def sag(self, actual: int) -> int:
        """Ticks the joint has fallen back toward its fold (<=0 = open
        at least as far as commanded)."""
        return (self.tick - actual) * self.opening

    def ok(self, actual: int) -> bool:
        return self.sag(actual) <= self.tol

    def describe(self, actual: int | None = None) -> str:
        want = fmt_ticks(self.frame, self.tick)
        if actual is None:
            return f"joint {self.joint} open to {want}"
        return (f"joint {self.joint} reads {fmt_ticks(self.frame, actual)}, "
                f"needs {want} (sagged {span_deg(self.sag(actual)):.1f} deg "
                f"toward its fold, limit {span_deg(self.tol):.1f})")


class GuardViolation(BenchError):
    """A safety precondition or in-motion invariant did not hold.

    Carries the offending holds so callers can report every failing
    joint at once instead of the first one found."""

    def __init__(self, holds: list[tuple[Hold, int]], phase: str,
                 why: str = "", hint: str | None = None):
        self.holds = holds
        detail = "; ".join(h.describe(actual) for h, actual in holds)
        super().__init__(
            f"{phase}: {detail}" + (f" — {why}" if why else ""),
            hint or "the arm is not in the state this move requires; "
                    "check for an obstruction, a slipped horn, or a servo "
                    "that did not reach its goal, then re-run",
        )


def check_holds(bus, holds: list[Hold], phase: str, why: str = "") -> None:
    """Read every hold's joint and raise GuardViolation if any sagged.

    Reads ALL joints before raising: one bad joint should not hide a
    second. Never trusts a commanded value — that is the whole point."""
    bad = [(hold, actual) for hold, actual in
           ((h, bus.read_position(h.joint)) for h in holds)
           if not hold.ok(actual)]
    if bad:
        raise GuardViolation(bad, phase, why)


def holds_for(cals, hold_ticks: dict[int, int],
              opening: dict[int, int]) -> list[Hold]:
    """Build Holds from a commanded pose plus each joint's opening
    direction, carrying frames so violations read in human units."""
    return [Hold(joint=i, tick=t, opening=opening[i],
                 frame=cals[i].frame if i in cals else None)
            for i, t in sorted(hold_ticks.items())]


# --------------------------------------------------------------- selftest
class _FakeBus:
    """Minimal position-reporting bus. `drift` injects a joint that
    reports somewhere other than where it was told — the physical
    failure the guards exist to catch."""

    def __init__(self, positions: dict[int, int],
                 drift: dict[int, int] | None = None):
        self.positions = dict(positions)
        self.drift = dict(drift or {})
        self.reads = 0

    def read_position(self, joint: int) -> int:
        self.reads += 1
        return self.positions[joint] + self.drift.get(joint, 0)


class _FakeCal:
    def __init__(self, frame=None):
        self.frame = frame


def _selftest() -> None:
    from hardware.units import DegFrame

    cals = {2: _FakeCal(DegFrame(zero_tick=1823, positive=1)),
            3: _FakeCal(DegFrame(zero_tick=3135, positive=-1)),
            4: _FakeCal(DegFrame(zero_tick=2090, positive=1))}
    # elbow + wrist held 90 deg open; both fold toward HIGHER ticks, so
    # opening is -1 (matches exercise's clearance_pose direction)
    commanded = {3: 2111, 4: 1928}
    opening = {3: -1, 4: -1}
    why = "elbow and wrist must stay open"

    def holds_of() -> list[Hold]:
        return holds_for(cals, commanded, opening)

    holds = holds_of()

    # 1. holds satisfied -> no raise, and every joint really was read
    bus = _FakeBus({3: 2111, 4: 1930})
    check_holds(bus, holds, ENTRY_PHASE, why)
    assert bus.reads == 2, bus.reads

    # 2. one joint never made it (the run-2 failure: refolded elbow)
    bus = _FakeBus({3: 3135, 4: 1928})  # elbow back at its fold
    try:
        check_holds(bus, holds, ENTRY_PHASE, why)
    except GuardViolation as exc:
        assert len(exc.holds) == 1 and exc.holds[0][0].joint == 3
        assert "joint 3 reads" in str(exc) and why in str(exc)
        assert exc.hint
    else:
        raise AssertionError("refolded elbow passed the guard")

    # 3. BOTH joints off -> both reported, not just the first
    bus = _FakeBus({3: 3135, 4: 2952})
    try:
        check_holds(bus, holds, ENTRY_PHASE, why)
    except GuardViolation as exc:
        assert sorted(h.joint for h, _ in exc.holds) == [3, 4]
    else:
        raise AssertionError("two bad joints passed the guard")

    # 4. sag boundary: at the limit passes, one tick past fails
    bus = _FakeBus(commanded, drift={3: HOLD_SAG_TOL_TICKS})
    check_holds(bus, holds, ENTRY_PHASE, why)
    bus = _FakeBus(commanded, drift={3: HOLD_SAG_TOL_TICKS + 1})
    try:
        check_holds(bus, holds, ENTRY_PHASE, why)
    except GuardViolation:
        pass
    else:
        raise AssertionError("sag beyond the limit passed the guard")

    # 5. DIRECTIONAL: opening further than commanded is safe, never a
    #    refusal — only sagging back toward the fold fails
    bus = _FakeBus(commanded, drift={3: -800, 4: -800})
    check_holds(bus, holds, MOTION_PHASE, why)

    # 6. mid-motion sag: the same holds re-checked during a sweep
    bus = _FakeBus(commanded)
    check_holds(bus, holds, MOTION_PHASE, why)
    bus.drift[3] = 400  # elbow sags under load
    try:
        check_holds(bus, holds, MOTION_PHASE, why)
    except GuardViolation as exc:
        assert MOTION_PHASE in str(exc)
    else:
        raise AssertionError("a sagging joint survived the invariant")

    # 7. violations report in human units (frames carried through)
    bus = _FakeBus({3: 3135, 4: 1928})
    try:
        check_holds(bus, holds, ENTRY_PHASE, why)
    except GuardViolation as exc:
        assert "deg" in str(exc) and "°" in str(exc), str(exc)
    else:
        raise AssertionError("a refolded elbow passed the guard")

    # 8. a frameless joint still reports (raw ticks), never crashes
    plain = holds_for({}, {5: 2062}, {5: -1})
    bus = _FakeBus({5: 3000})
    try:
        check_holds(bus, plain, ENTRY_PHASE)
    except GuardViolation as exc:
        assert "joint 5 reads 3000" in str(exc), str(exc)
    else:
        raise AssertionError("frameless joint skipped the guard")

    # 9. describe() with no reading — the pre-move announcement path
    assert "open to" in holds[0].describe()

    # 10. per-hold tol override is honoured
    tight = [Hold(joint=3, tick=2111, opening=-1, tol=5)]
    assert tight[0].ok(2116) and not tight[0].ok(2117)

    _selftest_in_motion(cals, commanded, opening)
    print("guards selftest OK")


def _selftest_in_motion(cals, commanded, opening) -> None:
    """The real integration: wait_settle must ABORT a move when the
    invariant fails, instead of riding the plan to its target."""
    import io
    import contextlib

    from .motion import wait_settle

    class _MovingBus(_FakeBus):
        """A sweeping joint that walks toward its goal each read, plus
        a hold that collapses partway through."""

        def __init__(self, positions, sweeper, goal, sag_at=None,
                     sag_joint=None):
            super().__init__(positions)
            self.sweeper, self.goal = sweeper, goal
            self.sag_at, self.sag_joint = sag_at, sag_joint
            self.samples = 0
            self.halted = False

        def read_position(self, joint):
            if joint == self.sweeper:
                self.samples += 1
                if self.sag_at and self.samples >= self.sag_at:
                    self.drift[self.sag_joint] = 900  # hold collapses
                cur = self.positions[joint]
                self.positions[joint] = cur + max(-60, min(60, self.goal - cur))
            return super().read_position(joint)

        def move_to(self, joint, tick, speed=0, acceleration=0):
            self.halted = True  # halt_all re-goals to present position

    holds = holds_for(cals, commanded, opening)
    quiet = contextlib.redirect_stdout(io.StringIO())  # settle progress

    # a) invariant satisfied throughout -> the move completes normally
    bus = _MovingBus({2: 1000, **commanded}, sweeper=2, goal=1200)
    with quiet:
        wait_settle(bus, {2: 1200}, 200, "sweep",
                    invariant=lambda: check_holds(bus, holds, MOTION_PHASE))

    # b) a hold collapses mid-move -> GuardViolation propagates out of
    #    wait_settle, and wait_settle halted the arm on its way out
    bus = _MovingBus({2: 1000, **commanded}, sweeper=2, goal=3000,
                     sag_at=3, sag_joint=3)
    try:
        with quiet:
            wait_settle(bus, {2: 3000}, 200, "sweep",
                        invariant=lambda: check_holds(bus, holds,
                                                      MOTION_PHASE))
    except GuardViolation as exc:
        assert exc.holds[0][0].joint == 3
        assert bus.positions[2] < 3000, "the sweep should not have finished"
        assert bus.halted, "wait_settle did not halt on invariant failure"
    else:
        raise AssertionError("wait_settle ignored a failing invariant")

    # c) ORDERING: a hold that collapses on the very sample the sweep
    #    arrives must still abort — the invariant runs before the
    #    arrival test, so settling can never outrace a violation.
    bus = _MovingBus({2: 1140, **commanded}, sweeper=2, goal=1200,
                     sag_at=1, sag_joint=4)
    try:
        with quiet:
            wait_settle(bus, {2: 1200}, 200, "sweep",
                        invariant=lambda: check_holds(bus, holds,
                                                      MOTION_PHASE))
    except GuardViolation as exc:
        assert exc.holds[0][0].joint == 4
    else:
        raise AssertionError("arrival outraced the invariant")




# --------------------------------------------------------- strain guard
# Plan #676. The e-stop is a keypress, so it needs a human. THIS is the
# abort that does not: the servo reports how hard it is working, and a
# joint pushing against something says so before anything breaks.
#
# The failure this exists to pre-empt: the STS3215 has its own overload
# protection, and that protection CUTS TORQUE. On an arm, a torque cut
# is not a safe state — the arm goes limp and falls, from wherever it
# was, at whatever angle. Stopping under control while the servo is
# still holding is strictly better than being dropped by it.
#
# Every threshold is a REFUSAL point, not a target. They sit below the
# servo's own trip points so we stop first.
LOAD_TRIP_PCT = 55.0        # sustained load; brief peaks are normal
LOAD_PEAK_PCT = 80.0        # a jam — confirm once, then stop
TEMP_TRIP_C = 55            # STS3215 protects itself around 70
TEMP_WARN_C = 45
VOLTS_MIN = 6.0             # brown-out: a sagging rail drops the arm
LOAD_TRIP_SAMPLES = 3       # consecutive, so one noisy read is not a stop

# NO SINGLE REGISTER READ MAY STOP THE ARM.
#
# Measured 2026-07-27: joint 4 reported 74 C mid-sweep and tripped the
# temperature guard, aborting the run. A `scan` seconds later read the
# same joint at 31 C, and peak load across the whole run was 11%. The
# servo was never hot — one corrupted byte on the bus was, and it cost a
# bench run.
#
# The principle was already understood for load, where LOAD_TRIP_SAMPLES
# exists precisely "so one noisy read is not a stop". It simply was not
# applied to temperature, voltage, or the fault bits, each of which
# tripped on a single sample. A corrupted read is indistinguishable from
# a real fault in one observation and trivially distinguishable in two,
# so every trip now needs consecutive confirmation.
#
# The cost is bounded and small: 2 samples is ~100 ms at motion.SAMPLE_S,
# and the servo's own protection remains the backstop underneath. The
# cost of NOT doing it is a guard that cries wolf, which is worse than no
# guard — an operator who has seen three false stops starts reaching for
# --no-gate.
TRIP_CONFIRM_SAMPLES = 2

# TEMPERATURE IS RATE-LIMITED BY PHYSICS, so the filter is too.
#
# A first attempt used a flat 15 C step. It caught the 74 C misread and
# then missed a 46 C one on the same afternoon — the arm read 32 C
# seconds later — because 14 C is under 15 C. A flat step is the wrong
# shape: what is impossible is not a SIZE of change but a RATE of one,
# and the right bound depends on how long it has been since the last
# reading.
#
# A servo warms over minutes. Between two 50 ms samples the real change
# is far below the register's 1 C resolution, so consecutive readings
# should differ by a count or two at most; over a multi-second pause
# between phases, a few degrees is fair. 2 C/s is already generous for a
# servo doing real work, and the quantum covers resolution and dither.
#
# At the 50 ms cadence this allows ~2.1 C between samples, which is why
# both misreads are now caught; across a 10 s gap it allows 22 C, which
# is why a genuinely heating arm is not slandered as a bus error.
TEMP_RATE_C_PER_S = 2.0
TEMP_QUANTUM_C = 2.0

# Temperature is the SLOWEST quantity the servo reports, so confirming a
# trip over a longer window is physically free: a real overheat lasts
# minutes and 5 samples is a quarter of a second. Load gets 3 and
# everything else 2 because those can genuinely change that fast.
TEMP_TRIP_SAMPLES = 5


class StrainViolation(BenchError):
    """A servo reported it was working too hard, or complained outright."""


class StrainWatch:
    """Per-joint strain state across a move.

    Sustained load needs consecutive samples because a single high read
    during acceleration is normal and stopping on it would make the guard
    useless — but a PEAK is acted on immediately, because by the time you
    have three samples of 80% something is already jammed.
    """

    def __init__(self, ids: list[int]):
        self.ids = list(ids)
        # One counter per (joint, condition): a joint flickering between
        # two different complaints has confirmed neither.
        self._streak: dict[tuple[int, str], int] = {}
        # Peak load and peak temperature are tracked SEPARATELY. They
        # used to share one stored sample — the max-load one — so the
        # reported temperature was whatever happened to accompany peak
        # load rather than the hottest reading seen. On 2026-07-27 that
        # printed "hottest 32 C on joint 6" for a run aborted by a 74 C
        # reading on joint 4: two independent maxima read off one record.
        self.peak_load: dict[int, dict] = {}
        self.peak_temp: dict[int, dict] = {}
        self._last_temp: dict[int, tuple[int, float]] = {}
        self._prev_load: dict[int, float] = {}
        self.raw_peak_load: dict[int, float] = {}
        self.unreadable = False
        self.discarded = 0      # readings rejected as physically impossible

    def _confirm(self, i: int, kind: str, needed: int) -> bool:
        """Count consecutive samples of one complaint on one joint.

        Returns True only once `needed` in a row have been seen, so a
        single corrupted register read can never stop the arm."""
        key = (i, kind)
        self._streak[key] = self._streak.get(key, 0) + 1
        return self._streak[key] >= needed

    def _clear(self, i: int, *kinds: str) -> None:
        for kind in kinds:
            self._streak.pop((i, kind), None)

    def check(self, bus) -> None:
        for i in self.ids:
            try:
                h = bus.read_health(i)
            except BenchError:
                # A health read that fails must not stop the arm on its
                # own — the position reads that actually gate motion are
                # still working, and a flaky diagnostic register is not a
                # reason to abort a good run.
                self.unreadable = True
                continue
            if not h["plausible"]:
                self.unreadable = True
                continue

            # Discard a temperature that moved faster than a servo can.
            # Counted, never silent: a rising discard count is the bus
            # telling you it is corrupting reads, which is worth knowing
            # even though it is not worth stopping for.
            now = time.monotonic()
            last = self._last_temp.get(i)
            if last is None:
                temp_ok = True
            else:
                prev_c, prev_t = last
                allowed = TEMP_QUANTUM_C + TEMP_RATE_C_PER_S * (now - prev_t)
                temp_ok = abs(h["temp_c"] - prev_c) <= allowed
            if temp_ok:
                self._last_temp[i] = (h["temp_c"], now)
            else:
                self.discarded += 1

            # PEAK LOAD IS REPORTED CONFIRMED, not raw. The same bus that
            # invents temperatures can invent a load spike, and a single
            # corrupted sample inflating the headline number would make
            # the arm look closer to its limits than it is — which is the
            # opposite of what this number is consulted for. A value only
            # counts once two consecutive samples both reach it. The raw
            # maximum is kept alongside, and reported when the two
            # disagree, because that gap is itself the bus-noise signal.
            before = self._prev_load.get(i)
            self._prev_load[i] = h["load_pct"]
            self.raw_peak_load[i] = max(self.raw_peak_load.get(i, 0.0),
                                        h["load_pct"])
            if before is not None:
                confirmed = min(before, h["load_pct"])
                prev = self.peak_load.get(i)
                if prev is None or confirmed > prev["load_pct"]:
                    self.peak_load[i] = {**h, "load_pct": confirmed}

            prev = self.peak_temp.get(i)
            if temp_ok and (prev is None or h["temp_c"] > prev["temp_c"]):
                self.peak_temp[i] = h

            if h["faults"]:
                if self._confirm(i, "fault", TRIP_CONFIRM_SAMPLES):
                    raise StrainViolation(
                        f"joint {i} reports {', '.join(h['faults'])}",
                        "the servo is about to protect itself by cutting "
                        "torque, which would drop the arm — power down and "
                        "let it cool before re-running")
            else:
                self._clear(i, "fault")

            if h["volts"] < VOLTS_MIN:
                if self._confirm(i, "volts", TRIP_CONFIRM_SAMPLES):
                    raise StrainViolation(
                        f"joint {i} sees {h['volts']:.1f} V (limit "
                        f"{VOLTS_MIN})",
                        "a sagging supply cuts torque and drops the arm; "
                        "check the supply before re-running")
            else:
                self._clear(i, "volts")

            if temp_ok and h["temp_c"] >= TEMP_TRIP_C:
                if self._confirm(i, "temp", TEMP_TRIP_SAMPLES):
                    raise StrainViolation(
                        f"joint {i} is at {h['temp_c']} C (limit "
                        f"{TEMP_TRIP_C})",
                        "let the arm cool; a batch of back-to-back runs "
                        "heats the shoulder joints most")
            elif temp_ok:
                self._clear(i, "temp")

            if h["load_pct"] >= LOAD_PEAK_PCT:
                # Still the fast path — confirmed in 2 samples rather
                # than 3 — because by the time you have three samples of
                # 80% something is already jammed.
                if self._confirm(i, "peak", TRIP_CONFIRM_SAMPLES):
                    raise StrainViolation(
                        f"joint {i} hit {h['load_pct']:.0f}% load",
                        "something is in the way, or the pose is beyond what "
                        "this joint can hold — clear the workspace and check "
                        "the arm before re-running")
            else:
                self._clear(i, "peak")

            if h["load_pct"] >= LOAD_TRIP_PCT:
                if self._confirm(i, "load", LOAD_TRIP_SAMPLES):
                    raise StrainViolation(
                        f"joint {i} held {h['load_pct']:.0f}% load for "
                        f"{LOAD_TRIP_SAMPLES} samples",
                        "sustained strain — the arm is pushing against "
                        "something rather than moving freely")
            else:
                self._clear(i, "load")

    def summary(self) -> str:
        if not self.peak_load:
            return ("strain: no readable health data — the servo status "
                    "registers did not return plausible values, so strain "
                    "did NOT gate this run")
        peak = max(self.peak_load.values(), key=lambda h: h["load_pct"])
        notes = []
        raw = max(self.raw_peak_load.values(), default=0.0)
        # A raw maximum well above the confirmed one means single samples
        # are landing far from their neighbours — the same bus noise the
        # temperature filter counts, showing up in a channel where
        # physics cannot rule it out. Shown, not silently smoothed away.
        if raw >= peak["load_pct"] + 10.0:
            notes.append(f"raw peak {raw:.0f}% unconfirmed by the "
                         f"neighbouring sample")
        if self.unreadable:
            notes.append("some reads unusable")
        if self.discarded:
            notes.append(f"{self.discarded} impossible temperature "
                         f"reading(s) discarded")
        note = f" ({'; '.join(notes)})" if notes else ""
        if not self.peak_temp:
            return (f"strain: peak load {peak['load_pct']:.0f}% on joint "
                    f"{peak['id']}, no usable temperature{note}")
        hot = max(self.peak_temp.values(), key=lambda h: h["temp_c"])
        return (f"strain: peak load {peak['load_pct']:.0f}% on joint "
                f"{peak['id']}, hottest {hot['temp_c']} C on joint "
                f"{hot['id']}{note}")


def _selftest_strain() -> None:
    """The strain guard, against a fake servo. Every acceptance paired
    with a refusal — a guard that only ever passes is decoration."""
    fails: list[str] = []

    def want(label: str, ok: bool, detail: str = "") -> None:
        if not ok:
            fails.append(label)
        print(f"  [{'ok ' if ok else 'FAIL'}] {label}"
              + (f"  — {detail}" if detail and not ok else ""))

    class Fake:
        def __init__(self, **over):
            self.h = {"id": 1, "load_pct": 5.0, "current_ma": 100.0,
                      "temp_c": 30, "volts": 7.4, "status": 0,
                      "faults": [], "plausible": True}
            self.h.update(over)

        def read_health(self, i):
            return {**self.h, "id": i}

    def trips(bus, n=5):
        w = StrainWatch([1])
        try:
            for _ in range(n):
                w.check(bus)
            return False, w
        except StrainViolation:
            return True, w

    def trips_ramp(factory, start, end):
        """Walk the temperature up in steps the filter accepts, so the
        end value is believed. Pairs with the jump test: the filter must
        reject an impossible RATE, never a genuinely heating servo.

        Steps by the quantum because back-to-back samples have ~zero
        elapsed time, so the rate term contributes nothing — this is the
        tightest the filter ever is."""
        w = StrainWatch([1])
        t = start
        try:
            while t < end:
                t = min(end, t + TEMP_QUANTUM_C)
                for _ in range(TEMP_TRIP_SAMPLES):
                    w.check(factory(temp_c=int(t)))
            return False
        except StrainViolation:
            return True

    ok, w = trips(Fake())
    want("a healthy servo does NOT trip", not ok)
    want("...and its peak is still reported", "peak load" in w.summary())

    want("an OVERLOAD fault bit trips", trips(Fake(faults=["OVERLOAD"],
                                                   status=0x20))[0])
    want("an OVERHEAT fault bit trips", trips(Fake(faults=["OVERHEAT"],
                                                   status=0x04))[0])
    want("a brown-out trips (a sagging rail drops the arm)",
         trips(Fake(volts=5.0))[0])
    want("over-temperature trips below the servo's own protection",
         trips(Fake(temp_c=TEMP_TRIP_C))[0])
    want("a load PEAK trips fast", trips(Fake(load_pct=LOAD_PEAK_PCT),
                                         n=TRIP_CONFIRM_SAMPLES)[0])
    want("sustained load trips only after several samples",
         trips(Fake(load_pct=LOAD_TRIP_PCT), n=LOAD_TRIP_SAMPLES)[0]
         and not trips(Fake(load_pct=LOAD_TRIP_PCT),
                       n=LOAD_TRIP_SAMPLES - 1)[0])

    # ---- THE 2026-07-27 FALSE STOP. One corrupted register read must
    # never stop the arm, whatever it claims. Each of these is a real
    # fault condition presented for exactly ONE sample.
    for label, over in (("a fault bit", {"faults": ["OVERLOAD"]}),
                        ("a brown-out", {"volts": 5.0}),
                        ("an over-temperature", {"temp_c": 90}),
                        ("a load peak", {"load_pct": 99.0})):
        w = StrainWatch([1])
        try:
            w.check(Fake(**over))
            want(f"ONE sample of {label} does not stop the arm", True)
        except StrainViolation as exc:
            want(f"ONE sample of {label} does not stop the arm", False,
                 str(exc))
    want("...but a sustained fault DOES stop it",
         trips(Fake(volts=5.0), n=TRIP_CONFIRM_SAMPLES)[0]
         and trips(Fake(load_pct=99.0), n=TRIP_CONFIRM_SAMPLES)[0])
    want("...and temperature needs a longer window, being the slowest "
         "quantity the servo reports",
         trips(Fake(temp_c=90), n=TEMP_TRIP_SAMPLES)[0]
         and not trips(Fake(temp_c=90), n=TEMP_TRIP_SAMPLES - 1)[0])

    # A flicker between two DIFFERENT complaints confirms neither.
    w = StrainWatch([1])
    try:
        for b in (Fake(volts=5.0), Fake(load_pct=99.0), Fake(volts=5.0),
                  Fake(load_pct=99.0)):
            w.check(b)
        want("alternating DIFFERENT complaints confirm neither", True)
    except StrainViolation as exc:
        want("alternating DIFFERENT complaints confirm neither", False,
             str(exc))

    # The physical-impossibility filter. BOTH misreads seen on the bench
    # on 2026-07-27 are pinned here: 74 C after 31 C (caught by the first
    # version of this filter) and 46 C after 32 C (which that version
    # MISSED, because 14 C was under its flat 15 C step).
    for label, hot in (("+43 C in one sample", 74), ("+14 C in one sample", 46)):
        w = StrainWatch([1])
        w.check(Fake(temp_c=32))
        for _ in range(6):
            w.check(Fake(temp_c=hot))   # would trip several times, if believed
        want(f"an impossible temperature jump is discarded ({label})",
             w.discarded == 6, f"{w.discarded} of 6 discarded")
        want(f"...and the arm was NOT stopped by it ({label})", True)
    w = StrainWatch([1])
    w.check(Fake(temp_c=32))
    w.check(Fake(temp_c=74))
    want("...and the run says so out loud rather than hiding it",
         "discarded" in w.summary(), w.summary())
    want("...while the same value reached GRADUALLY does trip",
         trips_ramp(Fake, 31, 74))

    # Peak load is reported CONFIRMED: one corrupted sample must not
    # inflate the headline headroom number.
    w = StrainWatch([1])
    for pct in (8.0, 9.0, 95.0, 8.0, 9.0):     # 95 is a lone bad byte
        w.check(Fake(load_pct=pct))
    want("a lone load spike does not become the reported peak",
         w.peak_load[1]["load_pct"] <= 9.0,
         f"reported {w.peak_load[1]['load_pct']}%")
    want("...but the raw maximum is kept, so the noise is not hidden",
         w.raw_peak_load[1] == 95.0, f"raw {w.raw_peak_load[1]}%")
    want("...and the summary shows both when they disagree",
         "raw" in w.summary(), w.summary())
    w = StrainWatch([1])
    for _ in range(4):
        w.check(Fake(load_pct=42.0))           # a genuine sustained load
    want("a REAL sustained load is reported at its true value",
         w.peak_load[1]["load_pct"] == 42.0,
         f"reported {w.peak_load[1]['load_pct']}%")

    # Peak load and peak temperature are independent maxima. Reading
    # both off one stored sample reported 'hottest 32 C' for a run that
    # died at 74 C.
    # Peak load happens while COOL; peak temperature happens while the
    # load is low. Reading both off one stored sample cannot report both.
    w = StrainWatch([1])
    for load, temp in ((40.0, 30), (40.0, 31), (5.0, 32), (5.0, 33)):
        w.check(Fake(load_pct=load, temp_c=temp))
    s = w.summary()
    want("summary reports the true peak load", "40%" in s, s)
    want("...AND the true peak temperature, not the one that happened to "
         "accompany peak load", "33 C" in s, s)

    # a brief spike must NOT stop a good run
    w = StrainWatch([1])
    hot, cool = Fake(load_pct=LOAD_TRIP_PCT), Fake(load_pct=5.0)
    try:
        for b in (hot, cool, hot, cool, hot, cool):
            w.check(b)
        want("alternating spikes do not trip (the counter resets)", True)
    except StrainViolation:
        want("alternating spikes do not trip (the counter resets)", False)

    # implausible readings must DISABLE the guard, not trip it
    ok, w = trips(Fake(plausible=False, load_pct=99.0, temp_c=200))
    want("implausible readings never trip the guard — a misread register "
         "must not stop a good arm", not ok)
    want("...and the run says out loud that strain did not gate it",
         "did NOT gate" in w.summary())

    print("strain selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    if fails:
        raise SystemExit(1)


if __name__ == "__main__":
    _selftest()
    _selftest_strain()
    sys.exit(0)
