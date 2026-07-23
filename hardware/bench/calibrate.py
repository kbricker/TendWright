"""calibrate — guided capture of per-joint range, rest pose, and direction.

Runs entirely TORQUE OFF and read-only: this tool never enables torque and
never writes a goal position, so nothing it does can move the arm. Support
the arm when torque cuts — it drops under gravity.

    uv run python -m hardware.bench.calibrate capture             # all joints
    uv run python -m hardware.bench.calibrate capture --ids 3     # redo joint 3
    uv run python -m hardware.bench.calibrate show

Capture walks you through: (1) hand-sweep each joint end to end while min/max
are tracked live, (2) pose the whole arm at rest once, (3) nudge each joint
in its canonical positive direction so the encoder sign is recorded. Writes
calibration.json atomically; re-runs merge per joint (an existing file keeps
every joint you did not re-capture).

Usage:
  calibrate capture [--ids RANGE] [--out FILE] [--port PORT] [--yes]
  calibrate show [--in FILE]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from .bus import POSITION_RANGE, BenchError, FeetechBus, confirm, run_tool
from .monitor import parse_ids
from .term import read_key

FORMAT_VERSION = 1
SAMPLE_HZ = 20.0
# Between 50 ms samples a hand-moved joint cannot travel this far; a jump
# this size is the single-turn encoder wrapping 0 <-> 4095 mid-range, which
# means the horn is mounted so the joint's travel crosses the wrap.
WRAP_JUMP_TICKS = 1500
MIN_SPAN_TICKS = 150
REST_TOL_TICKS = 25
DIR_MIN_DELTA_TICKS = 30
REST_ATTEMPTS = 3

# Canonical TendWright joint names and positive directions for the SO-101
# follower. The positive-direction wording BELOW IS THE CONVENTION — the
# recorded sign says which way the encoder counts when the joint moves this
# way, and the future arm driver / MuJoCo mapping consumes it.
JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
JOINT_POSITIVE = {
    1: "arm swings counterclockwise, viewed from above",
    2: "upper arm rises away from the base",
    3: "forearm rises toward the upper arm (elbow closes)",
    4: "gripper tips upward",
    5: "gripper rolls counterclockwise, viewed head-on from the front",
    6: "jaws close",
}


@dataclass
class JointCal:
    id: int
    name: str
    min: int
    rest: int
    max: int
    sign: int


def _valid_tick(value: object) -> bool:
    lo, hi = POSITION_RANGE
    return isinstance(value, int) and not isinstance(value, bool) \
        and lo <= value <= hi


def load_calibration(path: Path) -> dict[int, JointCal]:
    """Load + strictly validate a calibration file -> {joint id: JointCal}."""
    bad = BenchError(
        f"{path} is not a valid calibration file",
        "expected JSON {version, joints:[{id,name,min,rest,max,sign}]} from "
        "calibrate capture — fix or delete it, or point --out elsewhere",
    )
    try:
        doc = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise BenchError(
            f"could not read {path}: {exc}",
            "fix or delete the file, or point --out elsewhere",
        ) from exc
    if not isinstance(doc, dict) or doc.get("version") != FORMAT_VERSION:
        raise bad
    joints = doc.get("joints")
    if not isinstance(joints, list) or not joints:
        raise bad
    result: dict[int, JointCal] = {}
    for entry in joints:
        if not isinstance(entry, dict):
            raise bad
        try:
            cal = JointCal(**{k: entry[k]
                              for k in ("id", "name", "min", "rest", "max",
                                        "sign")})
        except (KeyError, TypeError) as exc:
            raise bad from exc
        if (cal.id not in JOINT_NAMES or cal.id in result
                or not isinstance(cal.name, str)
                or not all(_valid_tick(v)
                           for v in (cal.min, cal.rest, cal.max))
                or cal.sign not in (-1, 1)
                or cal.max - cal.min < MIN_SPAN_TICKS
                or not cal.min - REST_TOL_TICKS
                        <= cal.rest <= cal.max + REST_TOL_TICKS):
            raise bad
        result[cal.id] = cal
    return result


def write_calibration(path: Path, cals: dict[int, JointCal]) -> None:
    """Atomic write: an interrupted run never corrupts an existing file."""
    doc = {
        "version": FORMAT_VERSION,
        "joints": [asdict(cals[i]) for i in sorted(cals)],
    }
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n")
    os.replace(tmp, path)


def print_table(cals: dict[int, JointCal]) -> None:
    print(f"{'ID':>3}  {'joint':<13}  {'min':>4}  {'rest':>4}  {'max':>4}  "
          f"{'span':>4}  {'sign':>4}")
    for i in sorted(cals):
        c = cals[i]
        print(f"{c.id:>3}  {c.name:<13}  {c.min:>4}  {c.rest:>4}  "
              f"{c.max:>4}  {c.max - c.min:>4}  {c.sign:>+4}")


def sweep_joint(bus: FeetechBus, servo_id: int) -> tuple[int, int, bool]:
    """Live-track min/max while the joint is hand-swept; Enter finishes.

    Returns (min, max, wrapped). A too-small span re-prompts instead of
    returning — Enter-without-sweeping must not produce a garbage range.
    """
    period = 1.0 / SAMPLE_HZ
    prev = bus.read_position(servo_id)
    lo = hi = prev
    wrapped = False
    print(f"  sweep joint {servo_id} ({JOINT_NAMES[servo_id]}) slowly end to "
          f"end by hand, then press Enter")
    while True:
        start = time.monotonic()
        pos = bus.read_position(servo_id)
        if abs(pos - prev) > WRAP_JUMP_TICKS:
            wrapped = True
        prev = pos
        lo, hi = min(lo, pos), max(hi, pos)
        flag = "  ** WRAP **" if wrapped else ""
        print(f"\r  pos {pos:>4}  min {lo:>4}  max {hi:>4}  "
              f"span {hi - lo:>4}{flag}   ", end="", flush=True)
        key = read_key(timeout=max(0.0, period - (time.monotonic() - start)))
        if key in ("\r", "\n"):
            if not wrapped and hi - lo < MIN_SPAN_TICKS:
                print(f"\n  span is only {hi - lo} ticks (need "
                      f"{MIN_SPAN_TICKS}+) — keep sweeping, Enter when done")
                continue
            print()
            return lo, hi, wrapped


def capture_rest(bus: FeetechBus, ids: list[int],
                 ranges: dict[int, tuple[int, int]]) -> dict[int, int]:
    """One whole-arm rest pose; every (non-wrapped) joint must land inside
    its swept range, else the sweep missed part of the joint's travel."""
    for attempt in range(1, REST_ATTEMPTS + 1):
        input("\npose the WHOLE arm at its rest/neutral pose "
              "(per the assembly guide), then press Enter: ")
        rest = {i: bus.read_position(i) for i in ids}
        off = [i for i in ids if i in ranges and not
               ranges[i][0] - REST_TOL_TICKS
               <= rest[i] <= ranges[i][1] + REST_TOL_TICKS]
        if not off:
            return rest
        for i in off:
            print(f"  joint {i} ({JOINT_NAMES[i]}) reads {rest[i]} — outside "
                  f"its swept range {ranges[i]}")
        if attempt < REST_ATTEMPTS:
            print("  re-pose the arm and try again (or Ctrl+C and re-sweep "
                  "the joint(s) above — their sweep may have missed range)")
    raise BenchError(
        f"rest pose kept landing outside the swept range for joint(s) {off}",
        "the sweep for those joints missed part of their travel — re-run: "
        f"calibrate capture --ids {','.join(str(i) for i in off)}",
    )


def capture_direction(bus: FeetechBus, servo_id: int, rest: int) -> int:
    """Nudge the joint in its canonical positive direction; the tick delta's
    sign is the recording. Re-prompts until the nudge is unambiguous."""
    while True:
        input(f"  nudge joint {servo_id} ({JOINT_NAMES[servo_id]}) in its "
              f"POSITIVE direction — {JOINT_POSITIVE[servo_id]} — hold it "
              f"there and press Enter: ")
        delta = bus.read_position(servo_id) - rest
        if abs(delta) >= DIR_MIN_DELTA_TICKS:
            sign = 1 if delta > 0 else -1
            print(f"  moved {delta:+d} ticks -> sign {sign:+d}")
            return sign
        print(f"  moved only {delta:+d} ticks — need at least "
              f"±{DIR_MIN_DELTA_TICKS}; nudge it further and hold")


def capture(args: argparse.Namespace) -> int:
    ids = parse_ids(args.ids)
    unknown = sorted(set(ids) - set(JOINT_NAMES))
    if unknown:
        raise BenchError(f"unknown joint ID(s) {unknown}",
                         "the SO-101 follower uses IDs 1-6 (base to gripper)")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)  # fail HERE, not after capture
    existing = load_calibration(out) if out.exists() else {}

    with FeetechBus(args.port) as bus:
        missing = [i for i in ids if bus.ping(i) is None]
        if missing:
            raise BenchError(f"no answer from servo IDs {missing}",
                             "run the scan tool to see what is on the bus")

        redo = sorted(set(ids) & set(existing))
        if redo:
            print(f"{out} exists — re-capturing joint(s) {redo}, keeping "
                  f"the other {len(existing) - len(redo)}")
        print("this tool stays TORQUE OFF throughout — it cannot move the "
              "arm; you move the joints by hand.")
        print(f"about to cut torque on servos {ids} — if the arm is raised "
              f"it WILL drop under gravity.")
        if not args.yes and not confirm("support the arm, then type y to continue: "):
            print("aborted")
            return 1
        for servo_id in ids:
            bus.set_torque(servo_id, False)

        try:
            ranges: dict[int, tuple[int, int]] = {}
            wrapped_ids: list[int] = []
            print(f"\n--- step 1/3: range sweeps ({len(ids)} joint(s)) ---")
            for n, servo_id in enumerate(ids, start=1):
                print(f"\n[{n}/{len(ids)}]", end="")
                lo, hi, wrapped = sweep_joint(bus, servo_id)
                if wrapped:
                    wrapped_ids.append(servo_id)
                    print(f"  joint {servo_id} crossed the encoder 0/4095 "
                          f"wrap — its range is unusable (fix the horn "
                          f"mounting; details at the end)")
                else:
                    ranges[servo_id] = (lo, hi)

            print("\n--- step 2/3: rest pose ---")
            rest = capture_rest(bus, ids, ranges)

            good_ids = [i for i in ids if i not in wrapped_ids]
            captured: dict[int, JointCal] = {}
            if good_ids:
                print("\n--- step 3/3: direction nudges ---")
                for servo_id in good_ids:
                    sign = capture_direction(bus, servo_id, rest[servo_id])
                    lo, hi = ranges[servo_id]
                    captured[servo_id] = JointCal(
                        id=servo_id, name=JOINT_NAMES[servo_id],
                        min=lo, rest=rest[servo_id], max=hi, sign=sign)

            if captured:
                merged = {**existing, **captured}
                write_calibration(out, merged)
                print(f"\nsaved {len(captured)} joint(s) to {out} "
                      f"({len(merged)} total):")
                print_table(merged)
            if wrapped_ids:
                raise BenchError(
                    f"joint(s) {wrapped_ids} crossed the encoder wrap during "
                    f"the sweep — NOT saved",
                    "re-mount that horn one spline tooth away from the wrap, "
                    "then re-run: calibrate capture --ids "
                    + ",".join(str(i) for i in wrapped_ids),
                )
            return 0
        finally:
            bus.safe_torque_off(ids)


def show(args: argparse.Namespace) -> int:
    path = Path(args.infile)
    if not path.exists():
        raise BenchError(f"no such file: {path}",
                         "run calibrate capture first")
    cals = load_calibration(path)
    print(f"{path} — {len(cals)} joint(s), format v{FORMAT_VERSION}")
    print_table(cals)
    missing = sorted(set(JOINT_NAMES) - set(cals))
    if missing:
        print(f"not yet captured: joint(s) {missing} — "
              f"calibrate capture --ids {','.join(str(i) for i in missing)}")
    return 0


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.calibrate",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p_cap = sub.add_parser("capture", help="guided calibration capture")
    p_cap.add_argument("--ids", default="1-6", help="servo IDs (default 1-6)")
    p_cap.add_argument("--out", default="calibration.json",
                       help="output JSON file (merged per joint on re-runs)")
    p_cap.add_argument("--port", default=None, help="serial port override")
    p_cap.add_argument("--yes", action="store_true",
                       help="skip the support-the-arm confirmation")

    p_show = sub.add_parser("show", help="print + validate a calibration file")
    p_show.add_argument("--in", dest="infile", default="calibration.json",
                        help="calibration file (default calibration.json)")

    args = parser.parse_args()
    return capture(args) if args.command == "capture" else show(args)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
