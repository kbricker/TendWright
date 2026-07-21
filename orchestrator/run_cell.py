"""Watch the P1 orchestrator drive the sim cell in the viewer.

Unlike sim.run_cell (the P0 hardcoded script), this runs the real FSM:
every state change prints, and the motion you see is commanded task by
task with sensor verification between.

Usage: uv run python -m orchestrator.run_cell [--parts N] [--fault]
"""

from __future__ import annotations

import argparse

import mujoco.viewer

from .cell_fsm import CellFsm
from .simcell import SimCell


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m orchestrator.run_cell",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--parts", type=int, default=6,
                        help="part cycles to run (default 6)")
    parser.add_argument("--fault", action="store_true",
                        help="induce a physics-level pick miss on part 1")
    args = parser.parse_args()

    cell = SimCell(verbose=True, realtime=True)
    with mujoco.viewer.launch_passive(cell.model, cell.data) as viewer:
        cell.runner.viewer = viewer
        if args.fault:
            cell.inject_pick_failure_once()
        fsm = CellFsm(cell, verbose=True)
        ok = fsm.run(args.parts)
        print(f"\n{'complete' if ok else 'HALTED'} — parts done: "
              f"{fsm.parts_done}, faults: {len(fsm.fault_reasons)}")


if __name__ == "__main__":
    main()
