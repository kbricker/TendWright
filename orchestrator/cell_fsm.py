"""The cell orchestrator FSM: one machine-tending part cycle, with faults.

Built on the hand-rolled StateMachine base. The happy path chains through
hooks — each state's on_enter performs its task against the MockCell
backend and fires the outcome event — so a single fire("start") runs the
whole part cycle to PART_DONE or lands in FAULT with a recorded reason.

Command-then-verify: loading is only "done" when the part-present sensor
says the part is seated (VERIFYING), machining requires it to STAY seated,
and unloading is only "done" when the sensor reads empty again. Sensor
decisions are debounced (consecutive agreeing samples), so one bounce of
a physical switch can neither fake a seat nor abort machining.

    IDLE -> FETCHING -> LOADING -> VERIFYING -> MACHINING -> UNLOADING
         -> PART_DONE (-> IDLE)
    any task failure -> FAULT -> RECOVERING -> IDLE (retry), or
    FAULT -> HALTED once the retry budget is spent. HALTED is absorbing.

Honest P1 limitation: recovery restores a SAFE cell (arm parked, all
holds released), but a retry only genuinely succeeds for faults that left
the blank in its bin slot (e.g. a pick miss). Faults that displace the
part (mid-load, mid-machining) walk the retry budget to a clean HALTED —
fail-safe, not fail-operational. Re-homing displaced parts needs vision
(P2); the nest-occupied pre-check below keeps those retries from ever
loading into an occupied fixture.
"""

from __future__ import annotations

from .cell import CellTaskError, MockCell
from .fsm import StateMachine, Transition

ACTIVE_STATES = ("FETCHING", "LOADING", "VERIFYING", "MACHINING", "UNLOADING")


def _can_retry(machine: "CellFsm", **_data) -> bool:
    return machine.fault_count <= machine.max_retries


def _make_transitions() -> tuple[Transition, ...]:
    table = [
        Transition("start", "IDLE", "FETCHING"),
        Transition("picked", "FETCHING", "LOADING"),
        Transition("placed", "LOADING", "VERIFYING"),
        Transition("seated", "VERIFYING", "MACHINING"),
        Transition("machined", "MACHINING", "UNLOADING"),
        Transition("removed", "UNLOADING", "PART_DONE"),
        Transition("reset", "PART_DONE", "IDLE"),
        # task_failed is legal only from the active working states — never
        # from PART_DONE (would mint a fresh retry budget), FAULT (would
        # double-count), or HALTED (must stay absorbing).
        *[Transition("task_failed", s, "FAULT") for s in ACTIVE_STATES],
        # Guard order matters: retry while budget remains, else give up.
        Transition("recover", "FAULT", "RECOVERING", guard=_can_retry),
        Transition("recover", "FAULT", "HALTED"),
        Transition("recovered", "RECOVERING", "IDLE"),
        # halt is legal from anywhere EXCEPT HALTED (absorbing safety state).
    ]
    table.extend(
        Transition("halt", s, "HALTED")
        for s in ("IDLE", *ACTIVE_STATES, "PART_DONE", "FAULT", "RECOVERING"))
    return tuple(table)


class CellFsm(StateMachine):
    STATES = ("IDLE", "FETCHING", "LOADING", "VERIFYING", "MACHINING",
              "UNLOADING", "PART_DONE", "FAULT", "RECOVERING", "HALTED")
    INITIAL = "IDLE"
    TRANSITIONS = _make_transitions()
    # HALTED is absorbing — the transition table already refuses every
    # event out of it, and this closes the other door: `interrupt` is an
    # ungated state change for when reality moved on, and nothing that
    # happens in the cell un-halts a machine that was halted for safety.
    ABSORBING = ("HALTED",)

    SENSOR_POLL = 0.05  # s between sensor samples
    SENSOR_DEBOUNCE = 3  # consecutive agreeing samples to accept a reading

    def __init__(self, cell: MockCell, seat_timeout: float = 15.0,
                 machining_time: float = 2.0, unload_verify_timeout: float = 5.0,
                 max_retries: int = 2, verbose: bool = False):
        self.cell = cell
        self.seat_timeout = seat_timeout
        self.machining_time = machining_time
        self.unload_verify_timeout = unload_verify_timeout
        self.max_retries = max_retries
        self.verbose = verbose
        self.parts_done = 0
        self.fault_count = 0  # per-part fault budget; resets on PART_DONE
        self.fault_reasons: list[str] = []
        super().__init__()

    # ------------------------------------------------------------------ util
    def _say(self, message: str) -> None:
        if self.verbose:
            print(f"[fsm {self.cell.now():7.2f}s] {message}")

    def _fail(self, what: str, exc: Exception) -> None:
        self._say(f"{what} FAILED: {exc}")
        self.fire("task_failed", reason=f"{what}: {exc}")

    def _task(self, action, ok_event: str, what: str) -> None:
        """Run a backend task; ANY exception becomes a task_failed fault —
        an unexpected backend blow-up must never strand the machine
        mid-state (that path bypasses the whole FAULT/HALT safety net)."""
        try:
            action()
        except Exception as exc:
            self._fail(what, exc)
            return
        self.fire(ok_event)

    def _debounced(self, want_present: bool) -> bool:
        """True once SENSOR_DEBOUNCE consecutive samples agree."""
        agree = 0
        while agree < self.SENSOR_DEBOUNCE:
            if self.cell.part_present() == want_present:
                agree += 1
            else:
                return False
            if agree < self.SENSOR_DEBOUNCE:
                self.cell.dwell(self.SENSOR_POLL)
        return True

    def _wait_sensor(self, want_present: bool, timeout: float) -> bool:
        deadline = self.cell.now() + timeout
        while self.cell.now() < deadline:
            if self._debounced(want_present):
                return True
            self.cell.dwell(self.SENSOR_POLL)
        return False

    # ----------------------------------------------------------------- hooks
    def on_enter_FETCHING(self, **_):
        # Never fetch toward an occupied nest: a part a previous fault left
        # seated (or clamped) there would be collided with on load.
        try:
            occupied = self._debounced(True)
        except Exception as exc:
            self._fail("nest pre-check", exc)
            return
        if occupied:
            self.fire("task_failed",
                      reason="nest already occupied before fetch — needs "
                             "manual clearing (or vision, P2)")
            return
        self._say("fetching blank from bin")
        self._task(self.cell.fetch_blank, "picked", "fetch")

    def on_enter_LOADING(self, **_):
        self._say("loading nest")
        self._task(self.cell.load_nest, "placed", "load")

    def on_enter_VERIFYING(self, **_):
        self._say("waiting for part-present")
        try:
            seated = self._wait_sensor(True, self.seat_timeout)
        except Exception as exc:
            self._fail("seat sensor", exc)
            return
        if seated:
            self.fire("seated")
        else:
            self.fire("task_failed",
                      reason=f"part never seated within {self.seat_timeout}s")

    def on_enter_MACHINING(self, **_):
        self._say(f"machining for {self.machining_time}s")
        try:
            end = self.cell.now() + self.machining_time
            vanished = False
            while self.cell.now() < end:
                if self._debounced(False):  # debounced: one bounce won't abort
                    vanished = True
                    break
                self.cell.dwell(0.1)
            if not vanished:
                self.cell.mark_machined()
        except Exception as exc:
            self._fail("machining", exc)
            return
        if vanished:
            self.fire("task_failed", reason="part vanished mid-machining")
        else:
            self.fire("machined")

    def on_enter_UNLOADING(self, **_):
        self._say("unloading to tray")
        try:
            self.cell.unload_to_tray()
            empty = self._wait_sensor(False, self.unload_verify_timeout)
        except Exception as exc:
            self._fail("unload", exc)
            return
        if empty:
            self.fire("removed")
        else:
            self.fire("task_failed", reason="nest still occupied after unload")

    def on_enter_PART_DONE(self, **_):
        self.parts_done += 1
        self.fault_count = 0
        self._say(f"part cycle complete ({self.parts_done} done)")

    def on_enter_FAULT(self, reason: str = "unspecified", **_):
        self.fault_count += 1
        self.fault_reasons.append(reason)
        self._say(f"FAULT #{self.fault_count}: {reason}")

    def on_enter_RECOVERING(self, **_):
        self._say("recovering: safe retract")
        try:
            self.cell.safe_retract()
        except Exception as exc:
            self._say(f"recovery failed: {exc}")
            self.fire("halt", reason=f"recovery failed: {exc}")
            return
        self.fire("recovered")

    def on_enter_HALTED(self, reason: str = "", **_):
        self._say(f"HALTED {('— ' + reason) if reason else ''}")

    # ---------------------------------------------------------------- runner
    def run_part(self) -> bool:
        """Run one part cycle, retrying through faults. True when a part
        completed; False when the machine halted. The SAME machine instance
        carries on across faults — recovery is re-entry, not restart."""
        while True:
            if self.state == "PART_DONE":
                self.fire("reset")
            if self.state == "HALTED":
                return False
            if self.state != "IDLE":
                raise RuntimeError(f"run_part from unexpected state {self.state}")
            self.fire("start")
            if self.state == "PART_DONE":
                return True
            if self.state == "FAULT":
                self.fire("recover")  # -> RECOVERING -> IDLE, or HALTED
                continue
            if self.state == "HALTED":
                return False

    def run(self, parts: int) -> bool:
        for _ in range(parts):
            if not self.run_part():
                return False
        return True
