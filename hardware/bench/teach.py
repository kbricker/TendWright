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

from hardware.units import span_deg

from .bus import (BenchError, FeetechBus, confirm,
                  confirm_torque_cut, require_present, run_tool)
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
        require_present(bus, ids,
                        "recording needs every joint; run the scan tool")
        if not confirm_torque_cut(ids, args.yes):
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
        }, indent=None), encoding="utf-8")
        print(f"saved {len(frames)} frames ({len(frames) / args.hz:.1f}s) "
              f"for servos {ids} -> {out}")
        return 0


def load_recording(path: Path) -> tuple[list[int], float, list[list[int]]]:
    if not path.exists():
        raise BenchError(f"no such file: {path}")
    bad = BenchError(f"{path} is not a teach recording",
                     "expected JSON {version, ids, hz, frames} from teach record")
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
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


def gate_recording(bus: FeetechBus, ids: list[int], frames: list[list[int]],
                   current: list[int], no_gate: bool) -> str:
    """Pre-flight the WHOLE recording before the confirm prompt (#699).

    Replay used to be ungated. A recording is *usually* safe because Kyle
    moved the arm through it by hand — but "usually" leans on three
    assumptions that are not checked anywhere: that the file has not been
    edited, that replay starts from the pose it was recorded from, and
    that the APPROACH to frame 0 (a move nobody ever recorded, computed
    from wherever the arm happens to be sitting) is itself safe. That
    approach move is the one most likely to collide and the one least
    likely to have been thought about.

    Gating here rather than per-frame because the entire trajectory is
    known in advance, so the operator can be told before committing —
    the same reason `exercise` pre-flights.
    """
    if no_gate:
        return ("collision gate SKIPPED (--no-gate) — this recording was "
                "NOT checked")
    from .calibrate import JOINT_NAMES
    from .posegate import PoseGate

    gate = PoseGate(sorted(JOINT_NAMES))
    if not gate.active:
        return gate.banner()
    absent = [i for i in gate.ids if bus.ping(i) is None]
    if absent:
        return (f"collision gate INACTIVE — joint(s) {absent} did not "
                f"answer, so the arm's pose cannot be read. This recording "
                f"was NOT checked.")

    # Joints the recording does not drive still decide whether the ones it
    # does may move, so they are read and held at their present position.
    here = {i: bus.read_position(i) for i in gate.ids}
    here.update(dict(zip(ids, current)))

    poses = [dict(here)]
    for frame in frames:
        poses.append({**here, **dict(zip(ids, frame))})
    # `here` came off the encoders, so poses[0] is a FACT — the same
    # reason runner's gate_clip passes this. Without it, replay from a
    # cold torque-off start is refused at step 0 over the arm's own
    # resting slump (measured: shoulder <-> gripper, 0.22 mm), and the
    # documented escape is --force, which prints "the arm can hit
    # itself". Training the operator to force past a safety gate for a
    # benign condition is exactly what the runner fix existed to stop,
    # and this consumer was missed when that fix landed.
    verdict = gate.check_sequence(poses, label="replay", from_measured=True)
    if verdict.allowed:
        return (f"collision gate CLEAR — {verdict.poses_checked} poses "
                f"checked, including the approach to frame 0 (the bench "
                f"and anything on it are NOT modelled)")
    return verdict.detail


def replay(args: argparse.Namespace) -> int:
    ids, hz, frames = load_recording(Path(args.infile))
    if not 0.05 <= args.speed <= 1.0:
        raise BenchError("--speed must be between 0.05 and 1.0")

    with FeetechBus(args.port) as bus:
        require_present(bus, ids,
                        "replay drives every recorded joint; run the "
                        "scan tool to see what the bus can hear")

        current = [bus.read_position(i) for i in ids]
        first = frames[0]
        drift = max(abs(a - b) for a, b in zip(current, first))
        print(f"replaying {len(frames)} frames for servos {ids} at "
              f"{args.speed:.0%} speed ({len(frames) / hz / args.speed:.1f}s)")
        print(f"largest joint move to reach the start pose: "
              f"{span_deg(drift):.1f} deg ({drift} ticks)")

        verdict = gate_recording(bus, ids, frames, current, args.no_gate)
        print(verdict)
        if verdict.startswith("REFUSED"):
            if not args.force:
                print("not replaying — re-run with --force to override")
                return 1
            print("FORCED past the gate — the arm can hit itself",
                  file=sys.stderr)

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
    p_rep.add_argument("--force", action="store_true",
                       help="replay a recording the collision gate refuses "
                            "(logged; the arm can hit itself)")
    p_rep.add_argument("--no-gate", action="store_true",
                       help="skip the collision gate entirely")

    args = parser.parse_args()
    return record(args) if args.command == "record" else replay(args)


def _selftest() -> int:
    """The gate must refuse a colliding recording and clear a safe one."""
    from sim.twin import Twin

    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not cond:
            fails.append(name)

    if not Path("calibration.json").exists():
        print("  no calibration.json here — cannot exercise the gate")
        return 1

    cals = Twin("calibration.json").cals
    rest = {i: c.rest for i, c in cals.items()}
    ids = sorted(rest)

    class FakeBus:
        def ping(self, i):
            return 1

        def read_position(self, i):
            return rest[i]

    bus = FakeBus()

    def frames_for(seq):
        return [[p.get(i, rest[i]) for i in ids] for p in seq]

    print("a safe recording clears")
    safe = frames_for([rest, {**rest, 1: rest[1] + 60}, rest])
    v = gate_recording(bus, ids, safe, [rest[i] for i in ids], False)
    check("small pan wiggle is CLEAR", v.startswith("collision gate CLEAR"), v)

    print("\na colliding recording is refused")
    from hardware.bench.exercise import sweep_window
    _, hi2 = sweep_window(cals[2], 70)
    bad = frames_for([rest, {**rest, 2: hi2}])
    v = gate_recording(bus, ids, bad, [rest[i] for i in ids], False)
    check("the run-1 sweep is REFUSED", v.startswith("REFUSED"), v)
    check("...and it names the colliding links", "<->" in v)

    print("\n--no-gate says so rather than passing quietly")
    v = gate_recording(bus, ids, bad, [rest[i] for i in ids], True)
    check("skipped gate announces itself", "SKIPPED" in v and "NOT" in v, v)

    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("teach gate OK")
    return 0


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        return _selftest()
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
