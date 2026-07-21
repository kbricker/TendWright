"""teach — record joint trajectories by hand; replay them at reduced speed.

Record (torque off, move the arm by hand, Enter stops):

    uv run python -m hardware.bench.teach record --out wave.json

Replay (confirm prompt, moves to the start pose slowly first):

    uv run python -m hardware.bench.teach replay --in wave.json --speed 0.25

Usage:
  teach record [--out FILE] [--ids RANGE] [--hz N] [--port PORT]
  teach replay --in FILE [--speed F] [--port PORT] [--yes]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .bus import BenchError, FeetechBus, run_tool
from .monitor import parse_ids
from .term import read_key

FORMAT_VERSION = 1
REPLAY_SPEED_TICKS = 250  # servo-side speed cap during replay moves
APPROACH_SPEED_TICKS = 120  # extra-slow move to the first frame


def record(args: argparse.Namespace) -> int:
    ids = parse_ids(args.ids)
    out = Path(args.out)
    with FeetechBus(args.port) as bus:
        present = [i for i in ids if bus.ping(i) is not None]
        if sorted(present) != sorted(ids):
            missing = sorted(set(ids) - set(present))
            raise BenchError(f"no answer from servo IDs {missing}",
                             "recording needs every joint; run the scan tool")
        for servo_id in ids:
            bus.set_torque(servo_id, False)
        print(f"torque OFF on {ids} — move the arm by hand.")
        print(f"recording at {args.hz:.0f} Hz; press Enter (or Ctrl+C) to stop")

        frames: list[list[int]] = []
        period = 1.0 / max(0.5, args.hz)
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


def replay(args: argparse.Namespace) -> int:
    path = Path(args.infile)
    if not path.exists():
        raise BenchError(f"no such file: {path}")
    try:
        doc = json.loads(path.read_text())
        ids, hz, frames = doc["ids"], float(doc["hz"]), doc["frames"]
    except (json.JSONDecodeError, KeyError) as exc:
        raise BenchError(f"{path} is not a teach recording: {exc}") from exc
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
        if not args.yes:
            answer = input("clear the workspace, then type y to run: ")
            if answer.strip().lower() != "y":
                print("aborted")
                return 1

        try:
            for servo_id in ids:
                bus.set_torque(servo_id, True)
            # Slow approach to the first frame, then wait for it to settle.
            for servo_id, pos in zip(ids, first):
                bus.move_to(servo_id, pos, speed=APPROACH_SPEED_TICKS)
            time.sleep(max(1.0, drift / 300))

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
            for servo_id in ids:
                try:
                    bus.set_torque(servo_id, False)
                except BenchError:
                    pass


def run() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_rec = sub.add_parser("record", help="record a trajectory by hand")
    p_rec.add_argument("--out", default="teach.json", help="output JSON file")
    p_rec.add_argument("--ids", default="1-6", help="servo IDs (default 1-6)")
    p_rec.add_argument("--hz", type=float, default=10.0, help="sample rate")
    p_rec.add_argument("--port", default=None, help="serial port override")

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
