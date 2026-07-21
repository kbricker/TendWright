"""The cell orchestrator FSM: one machine-tending part cycle, with faults.

Built on the hand-rolled StateMachine base. The happy path chains through
hooks — each state's on_enter performs its task against the MockCell
backend and fires the outcome event — so a single fire("start") runs the
whole part cycle to PART_DONE or lands in FAULT with a recorded reason.

Command-then-verify: loading is only "done" when the part-present sensor
says the part is seated (VERIFYING), machining requires it to STAY seated,
and unloading is only "done" when the sensor reads empty again.

    IDLE -> FETCHING -> LOADING -> VERIFYING -> MACHINING -> UNLOADING
         -> PART_DONE (-> IDLE)
    any task failure -> FAULT -> RECOVERING -> IDLE (retry), or
    FAULT -> HALTED once the retry budget is spent.
"""

from __future__ import annotations

from .cell import CellTaskError, MockCell
from .fsm import StateMachine, Transition


def _can_retry(machine: "CellFsm", **_data) -> bool:
    return machine.fault_count <= machine.max_retries


class CellFsm(StateMachine):
    STATES = ("IDLE", "FETCHING", "LOADING", "VERIFYING", "MACHINING",
              "UNLOADING", "PART_DONE", "FAULT", "RECOVERING", "HALTED")
    INITIAL = "IDLE"
    TRANSITIONS = (
        Transition("start", "IDLE", "FETCHING"),
        Transition("picked", "FETCHING", "LOADING"),
        Transition("placed", "LOADING", "VERIFYING"),
        Transition("seated", "VERIFYING", "MACHINING"),
        Transition("machined", "MACHINING", "UNLOADING"),
        Transition("removed", "UNLOADING", "PART_DONE"),
        Transition("reset", "PART_DONE", "IDLE"),
        Transition("task_failed", "*", "FAULT"),
        # Guard order matters: retry while budget remains, else give up.
        Transition("recover", "FAULT", "RECOVERING", guard=_can_retry),
        Transition("recover", "FAULT", "HALTED"),
        Transition("recovered", "RECOVERING", "IDLE"),
        Transition("halt", "*", "HALTED"),
    )

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
        self.fault_count = 0  # consecutive faults on the current part
        self.fault_reasons: list[str] = []
        super().__init__()

    # ------------------------------------------------------------------ util
    def _say(self, message: str) -> None:
        if self.verbose:
            print(f"[fsm {self.cell.now():7.2f}s] {message}")

    def _task(self, action, ok_event: str, what: str) -> None:
        try:
            action()
        except CellTaskError as exc:
            self._say(f"{what} FAILED: {exc}")
            self.fire("task_failed", reason=f"{what}: {exc}")
        else:
            self.fire(ok_event)

    def _wait_sensor(self, want_present: bool, timeout: float) -> bool:
        deadline = self.cell.now() + timeout
        while self.cell.now() < deadline:
            if self.cell.part_present() == want_present:
                return True
            self.cell.dwell(0.05)
        return False

    # ----------------------------------------------------------------- hooks
    def on_enter_FETCHING(self, **_):
        self._say("fetching blank from bin")
        self._task(self.cell.fetch_blank, "picked", "fetch")

    def on_enter_LOADING(self, **_):
        self._say("loading nest")
        self._task(self.cell.load_nest, "placed", "load")

    def on_enter_VERIFYING(self, **_):
        self._say("waiting for part-present")
        if self._wait_sensor(True, self.seat_timeout):
            self.fire("seated")
        else:
            self.fire("task_failed",
                      reason=f"part never seated within {self.seat_timeout}s")

    def on_enter_MACHINING(self, **_):
        self._say(f"machining for {self.machining_time}s")
        end = self.cell.now() + self.machining_time
        while self.cell.now() < end:
            if not self.cell.part_present():
                self.fire("task_failed", reason="part vanished mid-machining")
                return
            self.cell.dwell(0.1)
        self.cell.mark_machined()
        self.fire("machined")

    def on_enter_UNLOADING(self, **_):
        self._say("unloading to tray")
        try:
            self.cell.unload_to_tray()
        except CellTaskError as exc:
            self.fire("task_failed", reason=f"unload: {exc}")
            return
        if self._wait_sensor(False, self.unload_verify_timeout):
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
        except CellTaskError as exc:
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
