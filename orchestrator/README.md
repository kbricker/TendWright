# orchestrator/

P1 — the cell orchestrator (plan #605, re-scoped 2026-07-21: no OPC UA,
no FSM library — both hand-rolled/deferred per Kyle).

- `fsm.py` — the shared plain-Python StateMachine base (states,
  transitions, guards, enter/exit hooks, introspection). Every future
  TendWright FSM builds on this.
- `cell.py` — the MockCell interface + FakeCell (scriptable faults,
  virtual clock) + the PicoCell sensor-backend slot for plan #619.
- `simcell.py` — MockCell backend driving the P0 MuJoCo cell.
- `cell_fsm.py` — the tending cycle FSM: fetch → load → verify-seated →
  machine → unload → verify-empty, with FAULT/RECOVERING/HALTED and a
  retry budget. Command-then-verify: sensors confirm every hand-off.
- `validate.py` — headless: `uv run python -m orchestrator.validate`
- `run_cell.py` — watch the FSM drive the sim:
  `uv run python -m orchestrator.run_cell` (add `--fault` to watch a
  pick-miss recovery live)
