"""A small, hand-rolled finite-state-machine base class (zero dependencies).

Kyle's call over an FSM library: the project will grow several state
machines (cell orchestrator now; arm driver, GRBL wrapper, MES job flow
later) and they should all share one simple, fully-understood core.

Usage: subclass StateMachine, declare STATES / INITIAL / TRANSITIONS, and
optionally define `on_enter_<STATE>` / `on_exit_<STATE>` methods. Fire
events with .fire(event, **data); on_enter hooks may themselves fire
follow-up events (the state is committed before on_enter runs, so
chaining is safe). Firing from an on_exit hook is FORBIDDEN and raises —
exit hooks run before the transition commits, so an inner fire would
corrupt the state; keep exit hooks to cleanup only.

    class Door(StateMachine):
        STATES = ("OPEN", "CLOSED")
        INITIAL = "CLOSED"
        TRANSITIONS = (
            Transition("open", "CLOSED", "OPEN"),
            Transition("close", "OPEN", "CLOSED"),
        )

A guard returns True to allow and False to decline; prefer returning
`Refused(why, hint)`, which declines AND says what the operator should
do. When every matching transition declines, `fire` raises
`GuardsRefused` carrying all their reasons — distinct from the plain
`FsmError` meaning "no such transition", because the two need different
responses.

`interrupt(state, why)` is the one ungated state change: it corrects
the machine's belief when reality moved on (a move aborted mid-way, an
e-stop). It runs NO hooks and refuses to leave an ABSORBING state. Read
its docstring before using it — "set the state" and "enter the state"
are not the same operation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable


class FsmError(Exception):
    """Bad machine definition or an event fired from a state with no match."""


@dataclass(frozen=True)
class Refused:
    """A guard's NO, with the reason attached.

    Plan #649's finding, and #660 inherits it: a guard that answers
    `False` throws away the only thing the operator needed. "event
    'sweep' not allowed in state 'REST'" is true and useless; "the arm
    reads 340 ticks from REST, so this move would start somewhere the
    gate never checked — jog it back or re-run from BETWEEN" is the same
    refusal with the next action in it.

    FALSY ON PURPOSE, so a guard can return it wherever it returned
    `False` and every existing `bool` guard keeps working untouched.
    Returning rather than raising also preserves the fallback chain: a
    refusal is not an error, it is one transition declining, and a later
    transition may still be the right answer. `fire` collects every
    refusal and reports them together only if NOTHING matched.
    """

    why: str
    hint: str = ""

    def __bool__(self) -> bool:
        return False

    def __str__(self) -> str:
        return self.why + (f" — {self.hint}" if self.hint else "")


@dataclass(frozen=True)
class Transition:
    """event fired in source state -> target state (if guard passes).

    source "*" matches any state. Guard is called as guard(machine, **data);
    among several matches, the first declared transition whose guard passes
    wins — later ones act as fallbacks.

    A guard returns True to allow, and either False or a `Refused` to
    decline. Prefer `Refused`: it is what the operator reads.
    """

    event: str
    source: str
    target: str
    guard: Callable[..., bool | Refused] | None = None


class GuardsRefused(FsmError):
    """Every transition that could have fired declined, and said why.

    Distinct from the plain "no such transition" FsmError so a caller
    can tell "you asked for something impossible" from "the machine is
    not currently in a state where this is safe" — and so the refusal
    text survives to whoever is standing at the bench.
    """

    def __init__(self, machine: str, event: str, state: str,
                 refusals: list[tuple['Transition', 'Refused | None']]):
        self.event = event
        self.state = state
        self.refusals = refusals
        lines = []
        for t, why in refusals:
            # A guard that answered a bare False has nothing to say. Name
            # it anyway rather than omitting it, so the operator can see
            # which guard is the silent one and go fix it.
            lines.append(f"  {t.source} --{t.event}--> {t.target}: "
                         + (str(why) if why is not None
                            else "refused (guard gave no reason)"))
        super().__init__(
            f"{machine}: event {event!r} was refused in state {state!r} by "
            f"{len(refusals)} guard(s):\n" + "\n".join(lines))


@dataclass(frozen=True)
class HistoryEntry:
    event: str
    source: str
    target: str


class StateMachine:
    STATES: tuple[str, ...] = ()
    INITIAL: str = ""
    TRANSITIONS: tuple[Transition, ...] = ()
    HISTORY_LIMIT = 200

    # Exception types a guard may raise that are NOT machine-definition
    # bugs and must reach the caller unchanged. Empty by default, which
    # in an `except` clause matches nothing — so the base behaviour is
    # unchanged and a machine has to opt in.
    #
    # A guard that only reads the machine's own fields can raise only
    # because of a bug, and wrapping it in FsmError with the transition
    # named is the most useful thing to do. A guard that reads HARDWARE
    # is different: a dead servo raises a BenchError carrying its own
    # actionable hint, and wrapping that strips the hint AND changes how
    # the CLI reports it — the operator gets a traceback instead of
    # "error: ... / hint: ...".
    TRANSPARENT_GUARD_ERRORS: tuple[type[BaseException], ...] = ()

    # States that `interrupt` may not leave. HALTED in a safety machine
    # is absorbing on purpose; interrupt is an escape hatch for "reality
    # moved on", and reality does not un-halt a machine.
    ABSORBING: tuple[str, ...] = ()

    def __init__(self) -> None:
        self._validate_definition()
        self._state = self.INITIAL
        self._in_exit_hook = False
        self.history: list[HistoryEntry] = []
        self._run_hook("on_enter", self._state)

    # ------------------------------------------------------------ definition
    def _validate_definition(self) -> None:
        if not self.STATES:
            raise FsmError(f"{type(self).__name__}: STATES is empty")
        if len(set(self.STATES)) != len(self.STATES):
            raise FsmError(f"{type(self).__name__}: duplicate state names")
        if self.INITIAL not in self.STATES:
            raise FsmError(
                f"{type(self).__name__}: INITIAL {self.INITIAL!r} not in STATES")
        if self.HISTORY_LIMIT < 1:
            raise FsmError(f"{type(self).__name__}: HISTORY_LIMIT must be >= 1")
        for t in self.TRANSITIONS:
            if t.source != "*" and t.source not in self.STATES:
                raise FsmError(f"transition {t.event!r}: unknown source {t.source!r}")
            if t.target not in self.STATES:
                raise FsmError(f"transition {t.event!r}: unknown target {t.target!r}")
        # A typo'd hook name would otherwise be silently skipped forever.
        for attr in dir(self):
            for prefix in ("on_enter_", "on_exit_"):
                if attr.startswith(prefix) and attr[len(prefix):] not in self.STATES:
                    raise FsmError(
                        f"{type(self).__name__}: hook {attr!r} does not match "
                        f"any declared state")

    # ---------------------------------------------------------- introspection
    @property
    def state(self) -> str:
        return self._state

    def matching(self, event: str) -> list[Transition]:
        return [t for t in self.TRANSITIONS
                if t.event == event and t.source in (self._state, "*")]

    def can(self, event: str) -> bool:
        return bool(self.matching(event))

    def allowed_events(self) -> list[str]:
        seen: list[str] = []
        for t in self.TRANSITIONS:
            if t.source in (self._state, "*") and t.event not in seen:
                seen.append(t.event)
        return seen

    def describe(self) -> str:
        lines = [f"{type(self).__name__}: state={self._state} "
                 f"(events: {', '.join(self.allowed_events()) or '-'})"]
        for t in self.TRANSITIONS:
            guard = f" [guard {t.guard.__name__}]" if t.guard else ""
            lines.append(f"  {t.source} --{t.event}--> {t.target}{guard}")
        return "\n".join(lines)

    def to_dot(self) -> str:
        """Graphviz digraph of the transition table (for docs/debugging)."""
        edges = [f'  "{t.source}" -> "{t.target}" [label="{t.event}"];'
                 for t in self.TRANSITIONS]
        return "digraph {\n" + "\n".join(edges) + "\n}"

    # ---------------------------------------------------------------- firing
    def fire(self, event: str, **data: Any) -> str:
        """Fire an event; returns the new state. Raises FsmError if no
        transition matches (unknown event OR event not legal in this state),
        if every matching guard declined, if called from inside an on_exit
        hook, or if a guard raises.

        WHY THE TWO NO's ARE DIFFERENT ERRORS. "there is no such
        transition" is a programming mistake — the caller asked for
        something the machine cannot do. "the transition exists and its
        guard said no" is the machine working: a precondition is not
        met, and the operator needs to know WHICH and what to do about
        it. Collapsing the second into the first is how a guard's reason
        got lost (plan #649), so the guards' own words are carried out.
        """
        if self._in_exit_hook:
            raise FsmError(
                f"{type(self).__name__}: fire({event!r}) called from an "
                f"on_exit hook — exit hooks must not fire events")
        matches = self.matching(event)
        if not matches:
            raise FsmError(
                f"{type(self).__name__}: event {event!r} not allowed in state "
                f"{self._state!r} (allowed: "
                f"{', '.join(self.allowed_events()) or '-'})")
        refusals: list[tuple[Transition, Refused | None]] = []
        for t in matches:
            try:
                verdict = True if t.guard is None else t.guard(self, **data)
            except self.TRANSPARENT_GUARD_ERRORS:
                # A guard that reads real hardware can fail for reasons
                # that are not machine-definition bugs — a dead servo, a
                # yanked USB cable. Wrapping those in FsmError would
                # strip the error's own actionable hint AND change how
                # the CLI reports it, so the operator would get a
                # traceback instead of "error: ... / hint: ...". Let
                # them through as themselves; only genuinely unexpected
                # exceptions get wrapped as a definition fault.
                raise
            except Exception as exc:
                raise FsmError(
                    f"{type(self).__name__}: guard for {t.source}"
                    f" --{t.event}--> {t.target} raised {exc!r}") from exc
            if verdict:
                source = self._state
                self._in_exit_hook = True
                try:
                    self._run_hook("on_exit", source, event=event, **data)
                finally:
                    self._in_exit_hook = False
                self._state = t.target
                self.history.append(HistoryEntry(event, source, t.target))
                del self.history[:-self.HISTORY_LIMIT]
                self._run_hook("on_enter", t.target, event=event, **data)
                return self._state
            refusals.append(
                (t, verdict if isinstance(verdict, Refused) else None))
        raise GuardsRefused(type(self).__name__, event, self._state, refusals)

    def interrupt(self, state: str, why: str) -> str:
        """Correct the machine's BELIEF about where it is, because
        reality moved on without asking.

        The one ungated state change, for exactly one situation:
        something outside the machine's model happened — a move aborted
        partway, an operator hit the e-stop — and the machine's idea of
        where it is has become false. There is no legitimate guarded
        transition for that, because a guard decides whether something
        MAY happen and this already did.

        Pretending otherwise is worse than allowing it: a machine that
        must claim it is still at REST after the arm stopped halfway
        somewhere else will confidently gate the next move from a pose
        the arm is not in. Recorded in history with the reason, so the
        jump is never silent.

        **NO HOOKS RUN — not on_exit, and deliberately not on_enter.**
        This is the difference between correcting a belief and executing
        a state, and the two are not the same thing anywhere it matters.
        In `CellFsm` the `on_enter` hooks ARE the work: `interrupt` with
        hooks would run the machining cycle rather than record that it
        was interrupted. `on_exit` is skipped for the mirror reason —
        the state was not exited cleanly, and pretending it was would
        run cleanup for a departure that did not happen. A caller that
        genuinely needs side effects should fire an event.

        Refused out of an ABSORBING state: HALTED exists to be
        terminal, and an escape hatch that can leave it is not one.
        """
        if self._in_exit_hook:
            # Same reason `fire` forbids it: `fire` will overwrite
            # `_state` with the transition's target the moment the hook
            # returns, so the jump would be silently undone.
            raise FsmError(
                f"{type(self).__name__}: interrupt({state!r}) called from "
                f"an on_exit hook — the pending transition would overwrite "
                f"it; exit hooks must not change state")
        if state not in self.STATES:
            raise FsmError(
                f"{type(self).__name__}: cannot interrupt to unknown state "
                f"{state!r} (states: {', '.join(self.STATES)})")
        if self._state in self.ABSORBING:
            raise FsmError(
                f"{type(self).__name__}: {self._state!r} is absorbing and "
                f"cannot be interrupted out of (asked for {state!r})")
        source = self._state
        self._state = state
        self.history.append(HistoryEntry(f"!{why}", source, state))
        del self.history[:-self.HISTORY_LIMIT]
        return self._state

    def _run_hook(self, prefix: str, state: str, **data: Any) -> None:
        hook = getattr(self, f"{prefix}_{state}", None)
        if hook is not None:
            hook(**data)


# ------------------------------------------------------------- selftest
def _selftest() -> int:
    """The guard-refusal and interrupt semantics, in isolation.

    `orchestrator.validate` drives the cell machine end to end; this
    covers the base class's own rules, which are the ones a second
    machine (posemachine) now depends on."""
    fails: list[str] = []

    def want(label: str, ok: bool, detail: str = "") -> None:
        print(f"  {'ok  ' if ok else 'FAIL'} {label}"
              f"{'  ' + detail if detail else ''}")
        if not ok:
            fails.append(label)

    class Boom(Exception):
        pass

    ran: list[str] = []

    class M(StateMachine):
        STATES = ("A", "B", "C", "DONE")
        INITIAL = "A"
        ABSORBING = ("DONE",)
        TRANSPARENT_GUARD_ERRORS = (Boom,)
        TRANSITIONS = (
            # Two matches for one event: the first declines with a
            # reason, the second is the fallback. Both must be tried.
            Transition("go", "A", "B",
                       guard=lambda m, **d: Refused("B is not ready",
                                                    "wait for B")),
            Transition("go", "A", "C"),
            Transition("only", "A", "B",
                       guard=lambda m, **d: Refused("nope", "do the thing")),
            Transition("silent", "A", "B", guard=lambda m, **d: False),
            Transition("boom", "A", "B",
                       guard=lambda m, **d: (_ for _ in ()).throw(Boom("hw"))),
            Transition("bug", "A", "B",
                       guard=lambda m, **d: 1 / 0),
            Transition("finish", "A", "DONE"),
        )

        def on_enter_B(self, **d):
            ran.append("enter_B")

        def on_exit_A(self, **d):
            ran.append("exit_A")

    print("a declining guard does not stop a later transition matching")
    m = M()
    want("the fallback fires when the first guard declines",
         m.fire("go") == "C", m.state)

    print("\nwhen EVERY guard declines, the reasons survive")
    m = M()
    try:
        m.fire("only")
        want("a fully-refused event raises", False)
    except GuardsRefused as exc:
        want("a fully-refused event raises GuardsRefused", True)
        want("...carrying the guard's reason", "nope" in str(exc))
        want("...and its actionable hint", "do the thing" in str(exc))
        want("...and it is still an FsmError, so old callers catch it",
             isinstance(exc, FsmError))
        want("...while the state is unchanged", m.state == "A", m.state)
    try:
        m.fire("silent")
        want("a bare-False guard still raises", False)
    except GuardsRefused as exc:
        want("a bare-False guard still raises", True)
        want("...naming itself as the silent one, rather than vanishing",
             "no reason" in str(exc), str(exc).splitlines()[-1])

    print("\n'no such transition' stays a DIFFERENT error from 'refused'")
    try:
        m.fire("nonexistent")
        want("an unknown event raises", False)
    except GuardsRefused:
        want("an unknown event is NOT reported as a guard refusal", False)
    except FsmError as exc:
        want("an unknown event is a plain FsmError", True, str(exc)[:60])

    print("\na guard that touches hardware fails as ITSELF")
    try:
        m.fire("boom")
        want("a declared-transparent error propagates unwrapped", False)
    except Boom:
        want("a declared-transparent error propagates unwrapped", True)
    except FsmError:
        want("a declared-transparent error propagates unwrapped", False,
             "it was wrapped in FsmError, losing its hint and exit code")
    try:
        m.fire("bug")
        want("...while an undeclared one is still wrapped as a bug", False)
    except FsmError as exc:
        want("...while an undeclared one is still wrapped as a bug", True,
             str(exc)[:60])

    print("\ninterrupt corrects the BELIEF; it does not execute the state")
    m = M()
    ran.clear()
    want("interrupt moves the state", m.interrupt("B", "reality") == "B")
    want("...and runs NO hooks — entering B would have done B's work",
         ran == [], f"{ran}")
    want("...recording the jump with its reason, never silently",
         m.history[-1].event == "!reality" and m.history[-1].source == "A",
         f"{m.history[-1]}")
    try:
        m.interrupt("NOWHERE", "typo")
        want("interrupting to an unknown state is refused", False)
    except FsmError as exc:
        want("interrupting to an unknown state is refused", True)
        want("...listing the states it could have meant",
             "A" in str(exc) and "DONE" in str(exc))

    print("\nan ABSORBING state cannot be escaped by interrupt either")
    m = M()
    m.fire("finish")
    want("the machine reached the absorbing state", m.state == "DONE")
    try:
        m.interrupt("A", "reset")
        want("interrupt out of an absorbing state is refused", False)
    except FsmError as exc:
        want("interrupt out of an absorbing state is refused", True,
             str(exc)[:70])
    want("...and it really is still absorbing", m.state == "DONE")

    print("\nneither fire nor interrupt may run from an on_exit hook")

    class Sneaky(StateMachine):
        STATES = ("A", "B", "C")
        INITIAL = "A"
        TRANSITIONS = (Transition("go", "A", "B"),)
        caught: list[str] = []

        def on_exit_A(self, **d):
            for name, call in (("fire", lambda: self.fire("go")),
                               ("interrupt",
                                lambda: self.interrupt("C", "sneaky"))):
                try:
                    call()
                    self.caught.append(f"{name}: ALLOWED")
                except FsmError:
                    self.caught.append(f"{name}: refused")

    s = Sneaky()
    s.fire("go")
    want("fire from an on_exit hook is refused",
         "fire: refused" in s.caught, f"{s.caught}")
    want("interrupt from an on_exit hook is refused too — the pending "
         "transition would overwrite it",
         "interrupt: refused" in s.caught, f"{s.caught}")
    want("...and the transition completed normally", s.state == "B")

    print("fsm selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
