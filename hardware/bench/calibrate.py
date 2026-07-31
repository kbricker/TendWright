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

v2 files additionally carry an optional per-joint "frame" — the RATIFIED
display convention (which tick reads as zero, which direction is +, or the
gripper's closed/open ticks) that makes tools speak degrees / % open
instead of ticks. Frames are hand-edited into calibration.json and never
captured; re-capturing a joint DROPS its frame (the geometry may have
changed — re-ratify). See hardware/units.py for the shape.

Usage:
  calibrate capture [--ids RANGE] [--out FILE] [--port PORT] [--yes]
  calibrate show [--in FILE]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from hardware.units import DegFrame, Frame, frame_from_dict, frame_to_dict

from .bus import (POSITION_RANGE, BenchError, FeetechBus,
                  confirm_torque_cut, require_present, run_tool)
from .monitor import parse_ids
from .term import flush_input, read_key

# v2 adds the optional per-joint "frame" (semantic display convention —
# see hardware/units.py). v1 files still load, with no frames.
FORMAT_VERSION = 2
LOADABLE_VERSIONS = (1, 2)
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
    # Field names mirror the JSON keys — renaming any of them changes the
    # on-disk file format. `frame` is the ratified display convention
    # (None until Kyle ratifies one); it is data Kyle edits, not a
    # captured measurement.
    id: int
    name: str
    min: int
    rest: int
    max: int
    sign: int
    frame: Frame | None = None


def fold_direction(cal: JointCal) -> int:
    """The tick direction that OPENS this joint away from its fold: the
    rest pose sits near one range end (the fold), so opening moves
    toward the other. Guards use it to tell sagging-back-toward-the-fold
    (dangerous) from opening-further (safe).

    Lives here, not in exercise.py, because it is a property of a
    calibration: which way the joint OPENS is geometry, independent of
    whichever display convention the frame happens to use.

    Deliberately NOT tied to the frame's `positive`. An earlier version
    of this file checked that the two agreed, on the theory that every
    label promises "positive = opening". Adopting the DH/right-hand-rule
    convention (2026-07-25) broke that premise: j3 and j4 now read
    positive TOWARD the fold, by design. Frame correctness is verified
    geometrically instead — `sim.twin frames`, which has the model.
    """
    return -1 if cal.rest > (cal.min + cal.max) // 2 else 1


def _valid_tick(value: object) -> bool:
    lo, hi = POSITION_RANGE
    return (type(value) is int and lo <= value <= hi)


def _rest_ok(lo: int, hi: int, rest: int) -> bool:
    return lo - REST_TOL_TICKS <= rest <= hi + REST_TOL_TICKS


def _joint_ok(cal: JointCal) -> bool:
    """One shared validity predicate for load AND pre-write — capture must
    never produce a file its own loader rejects."""
    return (type(cal.id) is int
            and cal.id in JOINT_NAMES
            and cal.name == JOINT_NAMES[cal.id]
            and all(_valid_tick(v) for v in (cal.min, cal.rest, cal.max))
            and type(cal.sign) is int and cal.sign in (-1, 1)
            and cal.max - cal.min >= MIN_SPAN_TICKS
            and _rest_ok(cal.min, cal.max, cal.rest))


def load_joint_calibration(path: Path) -> dict[int, JointCal]:
    """Load + strictly validate a calibration file -> {joint id: JointCal}."""
    bad = BenchError(
        f"{path} is not a valid calibration file",
        "expected JSON {version, joints:[{id,name,min,rest,max,sign,"
        "frame?}]} from calibrate capture — fix or delete it, or point "
        "the tool at a different file",
    )
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise BenchError(
            f"could not read {path}: {exc}",
            "fix or delete the file, or point the tool at a different file",
        ) from exc
    version = doc.get("version") if isinstance(doc, dict) else None
    if type(version) is not int or version not in LOADABLE_VERSIONS:
        raise bad
    joints = doc.get("joints")
    if not isinstance(joints, list) or not joints:
        raise bad
    result: dict[int, JointCal] = {}
    for entry in joints:
        if not isinstance(entry, dict):
            raise bad
        frame = None
        if entry.get("frame") is not None:
            try:
                frame = frame_from_dict(entry["frame"])
            except ValueError as exc:
                raise BenchError(
                    f"{path}: joint {entry.get('id')} has an invalid "
                    f"frame: {exc}",
                    "frames are hand-ratified — fix the frame object in "
                    "the file (see hardware/units.py for the shape)",
                ) from exc
        try:
            cal = JointCal(id=entry["id"], name=entry["name"],
                           min=entry["min"], rest=entry["rest"],
                           max=entry["max"], sign=entry["sign"],
                           frame=frame)
        except KeyError as exc:
            raise bad from exc
        if not _joint_ok(cal) or cal.id in result:
            raise bad
        result[cal.id] = cal
    return result


def write_calibration(path: Path, cals: dict[int, JointCal]) -> None:
    """Atomic write: an interrupted run never corrupts an existing file."""
    joints = []
    for i in sorted(cals):
        entry = asdict(cals[i])
        # asdict flattens the frame dataclass WITHOUT its unit tag —
        # serialize it through the canonical converter instead.
        del entry["frame"]
        if cals[i].frame is not None:
            entry["frame"] = frame_to_dict(cals[i].frame)
        joints.append(entry)
    doc = {"version": FORMAT_VERSION, "joints": joints}
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def print_table(cals: dict[int, JointCal]) -> None:
    """Human units first (per the joint's ratified frame), ticks in a
    trailing column; the convention label gets its own indented line so
    rows stay inside an 80-col terminal."""
    print(f"{'ID':>3}  {'joint':<13}  {'min':>10}  {'rest':>10}  "
          f"{'max':>10}  {'ticks m/r/M':>14}  {'sign':>4}")
    for i in sorted(cals):
        c = cals[i]
        ticks = f"{c.min}/{c.rest}/{c.max}"
        if c.frame is not None:
            lo, rest, hi = (c.frame.fmt(v) for v in (c.min, c.rest, c.max))
            label = c.frame.label or "-"
        else:
            lo, rest, hi = "-", "-", "-"
            label = "no frame — ratify one in calibration.json"
        print(f"{c.id:>3}  {c.name:<13}  {lo:>10}  {rest:>10}  {hi:>10}  "
              f"{ticks:>14}  {c.sign:>+4}")
        print(f"{'':>5}  {label}")


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
                 ranges: dict[int, tuple[int, int]],
                 rerun_spec: str) -> dict[int, int]:
    """One whole-arm rest pose; every joint must land inside its swept
    range, else the sweep missed part of the joint's travel."""
    for attempt in range(1, REST_ATTEMPTS + 1):
        flush_input()
        input("\npose the WHOLE arm at its rest/neutral pose "
              "(per the assembly guide), then press Enter: ")
        rest = {i: bus.read_position(i) for i in ids}
        off = [i for i in ids if not _rest_ok(*ranges[i], rest[i])]
        if not off:
            return rest
        for i in off:
            print(f"  joint {i} ({JOINT_NAMES[i]}) reads {rest[i]} — outside "
                  f"its swept range {ranges[i]}")
        if attempt < REST_ATTEMPTS:
            print("  re-pose the arm and try again (or Ctrl+C and re-run — "
                  "the sweep for the joint(s) above may have missed range)")
    raise BenchError(
        f"rest pose kept landing outside the swept range for joint(s) {off}",
        "the sweep for those joints missed part of their travel; NOTHING "
        f"from this run was saved — re-run: calibrate capture "
        f"--ids {rerun_spec}",
    )


def capture_direction(bus: FeetechBus, servo_id: int) -> int:
    """Nudge the joint in its canonical positive direction; the tick delta's
    sign is the recording. The baseline is read fresh before the first
    prompt (torque is off, so joints drift between phases) and KEPT across
    too-small retries — the user is mid-nudge then, and re-reading would
    measure from their hand's position. A delta that jumps the encoder wrap
    forces a release-and-settle step before a new baseline."""
    baseline = bus.read_position(servo_id)
    while True:
        flush_input()
        input(f"  nudge joint {servo_id} ({JOINT_NAMES[servo_id]}) in its "
              f"POSITIVE direction — {JOINT_POSITIVE[servo_id]} — hold it "
              f"there and press Enter: ")
        delta = bus.read_position(servo_id) - baseline
        if abs(delta) > WRAP_JUMP_TICKS:
            print("  the reading jumped across the encoder wrap — release "
                  "the joint")
            flush_input()
            input("  let it settle away from its end stop, then press "
                  "Enter: ")
            baseline = bus.read_position(servo_id)
            continue
        if abs(delta) >= DIR_MIN_DELTA_TICKS:
            sign = 1 if delta > 0 else -1
            print(f"  moved {delta:+d} ticks -> sign {sign:+d}")
            return sign
        print(f"  moved only {delta:+d} ticks — need at least "
              f"±{DIR_MIN_DELTA_TICKS}; nudge it further and hold")


def capture(args: argparse.Namespace) -> int:
    ids = list(dict.fromkeys(parse_ids(args.ids)))  # dedupe, keep order
    unknown = sorted(set(ids) - set(JOINT_NAMES))
    if unknown:
        raise BenchError(f"unknown joint ID(s) {unknown}",
                         "the SO-101 follower uses IDs 1-6 (base to gripper)")
    out = Path(args.out)
    # Fail on an unwritable destination HERE, not after the guided capture.
    out.parent.mkdir(parents=True, exist_ok=True)
    probe = out.with_name(out.name + ".tmp")
    try:
        probe.touch()
        probe.unlink()
    except OSError as exc:
        raise BenchError(f"cannot write next to {out}: {exc}",
                         "pick a writable --out location") from exc
    existing = load_joint_calibration(out) if out.exists() else {}

    with FeetechBus(args.port) as bus:
        require_present(bus, ids,
                        "run the scan tool to see what is on the bus")

        redo = sorted(set(ids) & set(existing))
        if redo:
            print(f"{out} exists — re-capturing joint(s) {redo}, keeping "
                  f"the other {len(existing) - len(redo)}")
        print("this tool stays TORQUE OFF throughout — it cannot move the "
              "arm; you move the joints by hand.")
        if not confirm_torque_cut(ids, args.yes):
            print("aborted")
            return 1

        try:
            for servo_id in ids:
                bus.set_torque(servo_id, False)

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

            good_ids = [i for i in ids if i not in wrapped_ids]
            captured: dict[int, JointCal] = {}
            if good_ids:
                print("\n--- step 2/3: rest pose ---")
                rest = capture_rest(bus, good_ids, ranges, args.ids)
                print("\n--- step 3/3: direction nudges ---")
                for servo_id in good_ids:
                    sign = capture_direction(bus, servo_id)
                    lo, hi = ranges[servo_id]
                    # A re-captured joint DROPS its old frame: the numbers
                    # changed (possibly a horn remount), so the ratified
                    # zero anchor can no longer be trusted.
                    if servo_id in existing and existing[servo_id].frame:
                        print(f"  note: joint {servo_id} had a ratified "
                              f"frame — dropped (geometry may have "
                              f"changed); re-ratify it in the output file")
                    captured[servo_id] = JointCal(
                        id=servo_id, name=JOINT_NAMES[servo_id],
                        min=lo, rest=rest[servo_id], max=hi, sign=sign)

            bad_caps = sorted(c.id for c in captured.values()
                              if not _joint_ok(c))
            if bad_caps:
                raise BenchError(
                    f"servo(s) {bad_caps} reported positions outside "
                    f"0-4095 — nothing saved",
                    "that usually means wheel/multi-turn mode leftovers; "
                    "power-cycle, run the scan tool, and re-capture",
                )

            # A wrapped joint's PRE-EXISTING entry is dropped too: the fix is
            # a horn remount, after which the old numbers are wrong anyway.
            stale = [i for i in wrapped_ids if i in existing]
            merged = {i: c for i, c in {**existing, **captured}.items()
                      if i not in wrapped_ids}
            try:
                if captured or stale:
                    if merged:
                        write_calibration(out, merged)
                    elif out.exists():
                        out.unlink()
            except OSError as exc:
                raise BenchError(
                    f"could not write {out}: {exc}",
                    "this run's data was lost — fix the location and re-run",
                ) from exc
            if captured:
                print(f"\nsaved {len(captured)} joint(s) to {out} "
                      f"({len(merged)} total):")
                print_table(merged)
            if stale:
                print(f"\nremoved stale prior entr"
                      f"{'y' if len(stale) == 1 else 'ies'} for wrapped "
                      f"joint(s) {stale} from {out}"
                      + ("" if merged else " — the file is gone (it held "
                         "nothing else)"))
            if wrapped_ids:
                raise BenchError(
                    f"joint(s) {wrapped_ids} crossed the encoder wrap during "
                    f"the sweep — not saved",
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
    cals = load_joint_calibration(path)
    print(f"{path} — {len(cals)} joint(s)")
    print_table(cals)
    missing = sorted(set(JOINT_NAMES) - set(cals))
    if missing:
        print(f"not yet captured: joint(s) {missing} — "
              f"calibrate capture --ids {','.join(str(i) for i in missing)}")
    # Whether a frame's zero and sign are geometrically right needs the
    # arm model, which lives on the other side of the import edge (the
    # twin imports this module). `sim.twin frames` does that check.
    print("\nframe zeros/signs are verified against the model by: "
          "uv run python -m sim.twin frames")
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
