"""A small, hand-rolled finite-state-machine base class (zero dependencies).

Kyle's call over an FSM library: the project will grow several state
machines (cell orchestrator now; arm driver, GRBL wrapper, MES job flow
later) and they should all share one simple, fully-understood core.

Usage: subclass StateMachine, declare STATES / INITIAL / TRANSITIONS, and
optionally define `on_enter_<STATE>` / `on_exit_<STATE>` methods. Fire
events with .fire(event, **data); hooks may themselves fire follow-up
events (the state is committed before on_enter runs, so chaining is safe).

    class Door(StateMachine):
        STATES = ("OPEN", "CLOSED")
        INITIAL = "CLOSED"
        TRANSITIONS = (
            Transition("open", "CLOSED", "OPEN"),
            Transition("close", "OPEN", "CLOSED"),
        )
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable


class FsmError(Exception):
    """Bad machine definition or an event fired from a state with no match."""


@dataclass(frozen=True)
class Transition:
    """event fired in source state -> target state (if guard passes).

    source "*" matches any state. Guard is called as guard(machine, **data);
    among several matches, the first declared transition whose guard passes
    wins — later ones act as fallbacks.
    """

    event: str
    source: str
    target: str
    guard: Callable[..., bool] | None = None


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

    def __init__(self) -> None:
        self._validate_definition()
        self._state = self.INITIAL
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
        for t in self.TRANSITIONS:
            if t.source != "*" and t.source not in self.STATES:
                raise FsmError(f"transition {t.event!r}: unknown source {t.source!r}")
            if t.target not in self.STATES:
                raise FsmError(f"transition {t.event!r}: unknown target {t.target!r}")

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
        transition matches (unknown event OR event not legal in this state)."""
        for t in self.matching(event):
            if t.guard is None or t.guard(self, **data):
                source = self._state
                self._run_hook("on_exit", source, event=event, **data)
                self._state = t.target
                self.history.append(HistoryEntry(event, source, t.target))
                del self.history[:-self.HISTORY_LIMIT]
                self._run_hook("on_enter", t.target, event=event, **data)
                return self._state
        raise FsmError(
            f"{type(self).__name__}: event {event!r} not allowed in state "
            f"{self._state!r} (allowed: {', '.join(self.allowed_events()) or '-'})")

    def _run_hook(self, prefix: str, state: str, **data: Any) -> None:
        hook = getattr(self, f"{prefix}_{state}", None)
        if hook is not None:
            hook(**data)
