"""teach — record joint trajectories by hand; replay them at reduced speed.

Record (cuts torque so you can move the arm by hand — support it first!):

    uv run python -m hardware.bench.teach record --out wave.json

Replay (confirm prompt; approaches the start pose slowly and WAITS until
every joint has actually arrived before streaming frames):

    uv run python -m hardware.bench.teach replay --in wave.json --speed 0.25

Usage:
  teach record [--out FILE] [--ids RANGE] [--hz N] [--port PORT] [--yes]
  teach replay --in FILE [--speed F] [--port PORT] [--yes]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .bus import BenchError, FeetechBus, confirm, run_tool
from .monitor import parse_ids
from .motion import wait_settle
from .term import read_key

FORMAT_VERSION = 1
REPLAY_SPEED_TICKS = 250  # servo-side speed cap during replay moves
APPROACH_SPEED_TICKS = 120  # extra-slow move to the first frame
MIN_RECORD_HZ = 0.5
MAX_RECORD_HZ = 30.0


def record(args: argparse.Namespace) -> int:
    ids = parse_ids(args.ids)
    if not MIN_RECORD_HZ <= args.hz <= MAX_RECORD_HZ:
        raise BenchError(f"--hz must be {MIN_RECORD_HZ}-{MAX_RECORD_HZ}")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)  # fail HERE, not after recording

    with FeetechBus(args.port) as bus:
        present = [i for i in ids if bus.ping(i) is not None]
        if sorted(present) != sorted(ids):
            missing = sorted(set(ids) - set(present))
            raise BenchError(f"no answer from servo IDs {missing}",
                             "recording needs every joint; run the scan tool")
        print(f"about to cut torque on servos {ids} — if the arm is raised "
              f"it WILL drop under gravity.")
        if not args.yes and not confirm("support the arm, then type y to continue: "):
            print("aborted")
            return 1
        for servo_id in ids:
            bus.set_torque(servo_id, False)
        print("torque OFF — move the arm by hand.")
        print(f"recording at {args.hz:.1f} Hz; press Enter (or Ctrl+C) to stop")

        frames: list[list[int]] = []
        period = 1.0 / args.hz
        try:
            while True:
                start = time.monotonic()
                frames.append([bus.read_position(i) for i in ids])
                print(f"\r{len(frames)} frames", end="", flush=True)
                key = read_key(timeout=max(0.0, period - (time.monotonic() - start)))
                if key in ("\r", "\n"):
                    break
        except KeyboardInterrupt:
            pass
        print()

        if len(frames) < 2:
            raise BenchError("recorded fewer than 2 frames — nothing to save")
        out.write_text(json.dumps({
            "version": FORMAT_VERSION,
            "ids": ids,
            "hz": args.hz,
            "frames": frames,
        }, indent=None))
        print(f"saved {len(frames)} frames ({len(frames) / args.hz:.1f}s) "
              f"for servos {ids} -> {out}")
        return 0


def load_recording(path: Path) -> tuple[list[int], float, list[list[int]]]:
    if not path.exists():
        raise BenchError(f"no such file: {path}")
    bad = BenchError(f"{path} is not a teach recording",
                     "expected JSON {version, ids, hz, frames} from teach record")
    try:
        doc = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise BenchError(f"{path} is not valid JSON: {exc}") from exc
    if not isinstance(doc, dict):
        raise bad
    ids, hz, frames = doc.get("ids"), doc.get("hz"), doc.get("frames")
    if (not isinstance(ids, list) or not ids
            or not all(isinstance(i, int) for i in ids)):
        raise bad
    if not isinstance(hz, (int, float)) or not MIN_RECORD_HZ <= hz <= MAX_RECORD_HZ:
        raise bad
    if (not isinstance(frames, list) or len(frames) < 2
            or not all(isinstance(f, list) and len(f) == len(ids)
                       and all(isinstance(p, int) for p in f) for f in frames)):
        raise bad
    return ids, float(hz), frames


def approach_start_pose(bus: FeetechBus, ids: list[int],
                        first: list[int]) -> None:
    """Move slowly to frame 0 and poll until every joint has ARRIVED —
    never start streaming frames on a timer guess. Arrival-only settle
    (require_still=False): replay's original semantics, tolerant of a
    gravity-loaded joint dithering inside the tolerance."""
    for servo_id, pos in zip(ids, first):
        bus.move_to(servo_id, pos, speed=APPROACH_SPEED_TICKS)
    wait_settle(bus, dict(zip(ids, first)), APPROACH_SPEED_TICKS,
                "approach", require_still=False,
                fail_hint="a joint may be obstructed or too weak for the "
                          "pose — torque has been cut")


def replay(args: argparse.Namespace) -> int:
    ids, hz, frames = load_recording(Path(args.infile))
    if not 0.05 <= args.speed <= 1.0:
        raise BenchError("--speed must be between 0.05 and 1.0")

    with FeetechBus(args.port) as bus:
        missing = [i for i in ids if bus.ping(i) is None]
        if missing:
            raise BenchError(f"no answer from servo IDs {missing}")

        current = [bus.read_position(i) for i in ids]
        first = frames[0]
        drift = max(abs(a - b) for a, b in zip(current, first))
        print(f"replaying {len(frames)} frames for servos {ids} at "
              f"{args.speed:.0%} speed ({len(frames) / hz / args.speed:.1f}s)")
        print(f"largest joint move to reach the start pose: {drift} ticks")
        if not args.yes and not confirm("clear the workspace, then type y to run: "):
            print("aborted")
            return 1

        try:
            # Pre-load each goal to the present position BEFORE enabling
            # torque — the goal register may be stale from an earlier
            # session, and enabling against it lurches (same rule as jog).
            for servo_id in ids:
                bus.move_to(servo_id, bus.read_position(servo_id),
                            speed=APPROACH_SPEED_TICKS)
                bus.set_torque(servo_id, True)
            approach_start_pose(bus, ids, first)

            period = 1.0 / hz / args.speed
            for n, frame in enumerate(frames[1:], start=2):
                start = time.monotonic()
                for servo_id, pos in zip(ids, frame):
                    bus.move_to(servo_id, pos, speed=REPLAY_SPEED_TICKS)
                print(f"\rframe {n}/{len(frames)}", end="", flush=True)
                elapsed = time.monotonic() - start
                if elapsed < period:
                    time.sleep(period - elapsed)
            print("\ndone — torque off")
            return 0
        finally:
            bus.safe_torque_off(ids)


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.teach",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p_rec = sub.add_parser("record", help="record a trajectory by hand")
    p_rec.add_argument("--out", default="teach.json", help="output JSON file")
    p_rec.add_argument("--ids", default="1-6", help="servo IDs (default 1-6)")
    p_rec.add_argument("--hz", type=float, default=10.0, help="sample rate")
    p_rec.add_argument("--port", default=None, help="serial port override")
    p_rec.add_argument("--yes", action="store_true",
                       help="skip the support-the-arm confirmation")

    p_rep = sub.add_parser("replay", help="replay a recorded trajectory")
    p_rep.add_argument("--in", dest="infile", required=True, help="recording file")
    p_rep.add_argument("--speed", type=float, default=0.25,
                       help="speed factor 0.05-1.0 (default 0.25)")
    p_rep.add_argument("--port", default=None, help="serial port override")
    p_rep.add_argument("--yes", action="store_true", help="skip confirmation")

    args = parser.parse_args()
    return record(args) if args.command == "record" else replay(args)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
