"""Headless validation of the P1 orchestrator.

Four scenarios, one FSM instance each (faults recover by re-entry, never
by rebuilding the machine):

  1. sim happy path — a full 3-part loop against the P0 MuJoCo cell;
  2. sim induced pick miss — the first fetch physically leaves the blank
     in the bin (weld never attaches); FSM must fault, recover, retry,
     and still finish the part;
  3. fake never-seats — sensor suppressed; FSM must burn its retry budget
     and land in HALTED cleanly;
  4. fake flaky pick — two scripted pick failures, then success.

Exits 0 on success, 1 on failure.

Usage: uv run python -m orchestrator.validate [--verbose]
"""

from __future__ import annotations

import argparse
import sys

from .cell import FakeCell
from .cell_fsm import CellFsm
from .simcell import SimCell


def check(name: str, condition: bool, detail: str, failures: list[str]) -> None:
    print(f"  {'ok' if condition else 'FAIL'}  {name}: {detail}")
    if not condition:
        failures.append(f"{name}: {detail}")


def states_visited(fsm: CellFsm) -> set[str]:
    return {entry.target for entry in fsm.history}


def scenario_sim_happy(verbose: bool, failures: list[str]) -> None:
    print("scenario 1: sim happy path (3 parts, full loop)")
    cell = SimCell(verbose=verbose)
    fsm = CellFsm(cell, verbose=verbose)
    done = fsm.run(3)
    check("completed", done and fsm.parts_done == 3,
          f"parts_done={fsm.parts_done} state={fsm.state}", failures)
    check("no faults", "FAULT" not in states_visited(fsm),
          f"fault_reasons={fsm.fault_reasons}", failures)
    check("sim time sane", 0 < cell.now() < 300,
          f"{cell.now():.1f}s", failures)


def scenario_sim_pick_miss(verbose: bool, failures: list[str]) -> None:
    print("scenario 2: sim induced pick miss (physics-level)")
    cell = SimCell(verbose=verbose)
    cell.inject_pick_failure_once()
    fsm = CellFsm(cell, verbose=verbose)
    done = fsm.run_part()
    visited = states_visited(fsm)
    check("recovered and completed", done and fsm.parts_done == 1,
          f"parts_done={fsm.parts_done} state={fsm.state}", failures)
    check("fault path exercised", {"FAULT", "RECOVERING"} <= visited,
          f"visited={sorted(visited)}", failures)
    check("pick miss was the reason",
          any("pick missed" in r for r in fsm.fault_reasons),
          f"reasons={fsm.fault_reasons}", failures)


def scenario_fake_never_seats(verbose: bool, failures: list[str]) -> None:
    print("scenario 3: fake cell, part never seats -> HALTED")
    cell = FakeCell(suppress_seat=True)
    fsm = CellFsm(cell, seat_timeout=2.0, verbose=verbose)
    done = fsm.run_part()
    check("halted", not done and fsm.state == "HALTED",
          f"state={fsm.state}", failures)
    check("retry budget spent", fsm.fault_count == fsm.max_retries + 1,
          f"fault_count={fsm.fault_count}", failures)
    check("reasons recorded",
          all("never seated" in r for r in fsm.fault_reasons)
          and len(fsm.fault_reasons) == fsm.max_retries + 1,
          f"reasons={fsm.fault_reasons}", failures)


def scenario_fake_flaky_pick(verbose: bool, failures: list[str]) -> None:
    print("scenario 4: fake cell, two pick failures then success")
    cell = FakeCell(fail_picks=2)
    fsm = CellFsm(cell, verbose=verbose)
    done = fsm.run_part()
    check("completed after retries", done and fsm.parts_done == 1,
          f"state={fsm.state}", failures)
    check("two faults recorded", len(fsm.fault_reasons) == 2,
          f"reasons={fsm.fault_reasons}", failures)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m orchestrator.validate",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    failures: list[str] = []
    scenario_fake_flaky_pick(args.verbose, failures)
    scenario_fake_never_seats(args.verbose, failures)
    scenario_sim_pick_miss(args.verbose, failures)
    scenario_sim_happy(args.verbose, failures)

    if failures:
        print(f"\nFAIL — {len(failures)} check(s):")
        for failure in failures:
            print(f"  {failure}")
        return 1
    print("\nPASS — FSM drives the sim cell end to end; faults recover "
          "in-place; exhausted retries halt cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
