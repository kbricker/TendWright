"""jog — incremental single-joint moves with soft limits and an e-stop key.

    uv run python -m hardware.bench.jog --id 3

Keys:
  +/=   jog positive        -/_  jog negative
  [ ]   halve / double the step size
  c     go to the joint's zero pose (frame zero; gripper: half open;
        2048 uncalibrated)
  t     toggle torque on/off
  q     quit (torque off)
  ANY OTHER KEY = E-STOP: torque off immediately and exit.

With calibration.json present, the jogged joint's soft limits default to
its CALIBRATED range (tighter and truer than the generic guard) and
positions print in its ratified units, ticks in parens. --step-deg jogs
by degrees instead of ticks. Uncalibrated joints keep the raw-tick
behavior — jog works before any calibration exists.

The status line refreshes ~10x/s from the ENCODER, showing the commanded
target, the actual position, the offset in ticks, and what the joint is
really doing (moving / settling / holding / torque off). Target and
actual do not become equal: a servo settles into a deadband a few ticks
off its goal, so "holding" means inside the shared settle tolerance, not
identical numbers.

COLLISION GATE (plan #699). Every step is checked against the digital
twin before it reaches the bus, and refused if the arm would hit itself
or the table. Soft limits cannot do this: they are per-joint, and
self-collision is a function of the WHOLE pose — two joints each well
inside their own range still fold the arm through itself, which is
exactly the collision the bench actually had. So jog re-reads all six
joints on every step, because the five it is not driving still decide
whether the sixth may move.

A refused step leaves the joint where it was and says which links would
touch. `--force` commands it anyway (logged to stderr) — kept because
jog is a testing tool and exploring near a limit is its job. If a joint
is off the bus the gate goes INACTIVE and says so; `--no-gate` skips it
without the notice. The gate models the arm and the table ONLY: the
bench, fixtures, anything in the gripper and the cable are all invisible
to it, so a clear verdict means "it will not hit itself", never "safe".

Exit codes: 0 quit, 2 error, 3 operator e-stop, 130 Ctrl+C.
Torque always starts OFF and is cut again on every exit path.

Usage: jog --id N [--port PORT] [--step TICKS | --step-deg D]
           [--min T] [--max T] [--cal FILE] [--force] [--no-gate]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from hardware.units import DEG_PER_TICK, PctFrame, fmt_ticks

from .bus import POSITION_RANGE, BenchError, FeetechBus, run_tool
from .motion import SETTLE_TOL_TICKS, STILL_TICKS
from .term import read_key

CENTER = 2048
JOG_SPEED = 300
JOG_ACCELERATION = 30
# Refresh cadence for the live position line. The same read_key timeout
# that idles the loop drives the poll, so this is also the key latency.
POLL_S = 0.1


def joint_state(pos: int, prev: int, target: int, torque_on: bool) -> str:
    """What the joint is ACTUALLY doing — read from the plant, never
    assumed from the command that was just sent.

    'arrived' uses the shared SETTLE_TOL_TICKS: a servo settles into a
    deadband and does NOT land on the exact goal tick, so demanding
    equality would mean never reporting arrival (which is precisely what
    the old hardcoded '(moving)' string did).
    """
    if not torque_on:
        return "torque off"
    if abs(pos - target) > SETTLE_TOL_TICKS:
        return "moving"
    if abs(pos - prev) > STILL_TICKS:
        return "settling"
    return "holding"


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.jog",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--id", type=int, required=True, dest="servo_id",
                        help="servo bus ID to jog")
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--step", type=int, default=None,
                        help="ticks per keypress (default 20)")
    parser.add_argument("--step-deg", type=float, default=None,
                        help="degrees per keypress (instead of --step)")
    parser.add_argument("--min", type=int, default=None,
                        dest="soft_min", help="soft limit low (ticks)")
    parser.add_argument("--max", type=int, default=None,
                        dest="soft_max", help="soft limit high (ticks)")
    parser.add_argument("--cal", default="calibration.json",
                        help="calibration file for limits + unit display")
    parser.add_argument("--force", action="store_true",
                        help="command moves the collision gate refuses "
                             "(logged loudly; the arm can hit itself)")
    parser.add_argument("--no-gate", action="store_true",
                        help="skip the collision gate entirely — for when "
                             "the other joints are off the bus")
    args = parser.parse_args()

    if args.step is not None and args.step_deg is not None:
        raise BenchError("--step and --step-deg are mutually exclusive")
    if args.step_deg is not None and args.step_deg <= 0:
        raise BenchError("--step-deg must be positive")
    if args.step is not None and args.step <= 0:
        raise BenchError("--step must be positive")

    # Calibrated joint: soft limits default to the CALIBRATED range and
    # positions display in the ratified frame. Lazy import for symmetry
    # with monitor (whose parse_ids IS module-imported by calibrate);
    # #637 gives load_joint_calibration a shared home.
    cal = None
    cal_path = Path(args.cal)
    if cal_path.exists():
        from .calibrate import load_joint_calibration
        cal = load_joint_calibration(cal_path).get(args.servo_id)
    frame = cal.frame if cal else None

    soft_min = args.soft_min if args.soft_min is not None else (
        cal.min if cal else POSITION_RANGE[0] + 200)
    soft_max = args.soft_max if args.soft_max is not None else (
        cal.max if cal else POSITION_RANGE[1] - 200)
    if soft_min >= soft_max:
        raise BenchError("--min must be below --max")

    if args.step_deg is not None:
        step0 = max(1, round(args.step_deg / DEG_PER_TICK))
    else:
        step0 = args.step if args.step is not None else 20

    def show(tick: int) -> str:
        return fmt_ticks(frame, tick)

    # The gate needs the WHOLE arm: whether the forearm hits the upper
    # arm is a function of every joint, not of the one being jogged. So
    # jog has to read joints it does not command. Joints that do not
    # answer disable the gate rather than being guessed — a pose we
    # cannot see is a pose we cannot judge (plan #699).
    from .posegate import PoseGate
    gate_ids: list[int] = []
    gate = None
    if not args.no_gate:
        from sim.clip import MotionProfile

        from .calibrate import JOINT_NAMES
        gate = PoseGate(sorted(JOINT_NAMES),
                        cal_path=cal_path,
                        profile=MotionProfile(speed=JOG_SPEED,
                                              acceleration=JOG_ACCELERATION))

    with FeetechBus(args.port) as bus:
        if bus.ping(args.servo_id) is None:
            raise BenchError(f"servo {args.servo_id} did not answer",
                             "run the scan tool to see what is on the bus")
        if gate is not None and gate.active:
            gate_ids = [i for i in gate.ids if bus.ping(i) is not None]
            absent = [i for i in gate.ids if i not in gate_ids]
            if absent:
                gate = None
                print(f"collision gate INACTIVE — joint(s) {absent} did not "
                      f"answer, so the arm's pose cannot be read. Moves are "
                      f"NOT checked for self-collision. (--no-gate to skip "
                      f"this check silently)")
        # Enforce the advertised starting state instead of assuming it: a
        # previous tool may have died with torque latched on.
        bus.set_torque(args.servo_id, False)
        target = bus.read_position(args.servo_id)
        step = step0
        torque_on = False
        name = f" ({cal.name})" if cal else ""
        limits_src = "calibrated" if (cal and args.soft_min is None
                                      and args.soft_max is None) else "soft"
        # 'c' pose: frame zero for angles, half-open for the gripper's
        # percent frame (its 0 is the fully-closed jaw), 2048 uncalibrated
        if frame is None:
            center = CENTER
        elif isinstance(frame, PctFrame):
            center = frame.tick(50)
        else:
            center = frame.tick(0)
        print(f"jogging servo {args.servo_id}{name} on {bus.port_name} — "
              f"position {show(target)}, {limits_src} limits "
              f"[{show(soft_min)}, {show(soft_max)}]")
        print("torque is OFF; press 't' to enable before jogging. "
              "+/- jog, [ ] step size, c zero pose, q quit, "
              "any other key = E-STOP")
        print(f"live from the encoder; 'holding' = within "
              f"{SETTLE_TOL_TICKS} ticks of target (servos settle into a "
              f"deadband, they do not land exactly on it)")
        if gate is not None:
            print(gate.banner())
            if args.force:
                print("--force: refusals will be OVERRIDDEN and logged. "
                      "The arm can hit itself.")
        elif args.no_gate:
            print("collision gate SKIPPED (--no-gate) — moves are NOT "
                  "checked for self-collision")

        prev_pos = target

        def refresh() -> int:
            """Re-read the plant and repaint the status line."""
            nonlocal prev_pos
            pos = bus.read_position(args.servo_id)
            state = joint_state(pos, prev_pos, target, torque_on)
            prev_pos = pos
            print(f"\rtarget {show(target):>16}  now {show(pos):>16}  "
                  f"{pos - target:+5d}t  {state:<10}", end="", flush=True)
            return pos

        try:
            while True:
                key = read_key(timeout=POLL_S)
                # Idle tick: this is what makes the readout live. Without
                # it the line froze at whatever was true microseconds
                # after the goal was written, so it never converged.
                if key is None:
                    refresh()
                    continue
                if key in ("+", "="):
                    delta = step
                elif key in ("-", "_"):
                    delta = -step
                elif key == "[":
                    step = max(1, step // 2)
                    print(f"\nstep = {step}")
                    continue
                elif key == "]":
                    step = min(500, step * 2)
                    print(f"\nstep = {step}")
                    continue
                elif key == "c":
                    delta = center - target
                elif key == "t":
                    if torque_on:
                        bus.set_torque(args.servo_id, False)
                        torque_on = False
                        print("\ntorque OFF")
                    else:
                        # The joint may have been hand-moved (or sagged) while
                        # torque was off, and the servo's goal register may be
                        # stale from any earlier session. Re-sync the target
                        # AND pre-load the goal to the current position while
                        # still torque-off, so enabling torque holds in place
                        # instead of lurching to an old goal.
                        target = bus.read_position(args.servo_id)
                        bus.move_to(args.servo_id, target,
                                    speed=JOG_SPEED, acceleration=JOG_ACCELERATION)
                        bus.set_torque(args.servo_id, True)
                        torque_on = True
                        print(f"\ntorque ON — holding at {show(target)}")
                    continue
                elif key == "q":
                    print("\nquitting — torque off")
                    return 0
                else:
                    print("\nE-STOP — torque off", file=sys.stderr)
                    return 3

                if not torque_on:
                    print("\ntorque is OFF — press 't' first")
                    continue
                clamped = max(soft_min, min(soft_max, target + delta))
                if clamped != target + delta:
                    print(f"\nsoft limit — clamped to {show(clamped)}")

                # Ask the twin BEFORE the bus. The whole arm is re-read
                # every step rather than cached from startup: torque is
                # off on the joints we are not driving, so they sag, and
                # a stale pose would gate an arm that is not the one on
                # the bench.
                if gate is not None:
                    here = {i: bus.read_position(i) for i in gate_ids}
                    verdict = gate.check(here, {args.servo_id: clamped})
                    if verdict.refused:
                        if not args.force:
                            # `target` is deliberately NOT updated: a
                            # refused step leaves the joint exactly where
                            # it was, not part-way.
                            print(f"\n{verdict.detail}"
                                  f"\n  not moving — press [ for a smaller "
                                  f"step, or re-run with --force")
                            continue
                        print(f"\nFORCED past the gate: {verdict.detail}",
                              file=sys.stderr)

                target = clamped
                bus.move_to(args.servo_id, target,
                            speed=JOG_SPEED, acceleration=JOG_ACCELERATION)
                refresh()  # keeps the line live while a key is held down
        finally:
            bus.safe_torque_off([args.servo_id])


def _selftest() -> int:
    """Prove a refused step never reaches the bus.

    jog is one interactive loop around a live serial port, so this fakes
    both ends — the bus and the keyboard — and asserts on what the fake
    bus was ASKED to do. Testing the verdict alone would not do: the
    whole failure mode being guarded against is a gate that computes the
    right answer and then commands the move anyway.
    """
    import sys as _sys
    from unittest import mock

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
    from hardware.bench.exercise import sweep_window
    _, hi2 = sweep_window(cals[2], 70)          # the run-1 collision

    class FakeBus:
        """Answers like the arm parked at its calibrated rest."""

        port_name = "fake"

        def __init__(self, *a, **k):
            self.moves = []
            self.pos = dict(rest)

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def ping(self, i):
            return 1

        def set_torque(self, i, on):
            pass

        def read_position(self, i):
            return self.pos[i]

        def move_to(self, i, tick, **k):
            self.moves.append((i, tick))
            self.pos[i] = tick

        def safe_torque_off(self, ids):
            pass

    def drive(argv, keys):
        """Run jog with a scripted keyboard, return the fake bus."""
        bus = FakeBus()
        it = iter(keys)
        with mock.patch.object(_sys, "argv", ["jog", *argv]), \
                mock.patch(f"{__name__}.FeetechBus", lambda *a, **k: bus), \
                mock.patch(f"{__name__}.read_key",
                           lambda timeout=None: next(it, "q")):
            run()
        return bus

    # Jogging j2 from rest toward the sweep end is the collision. Use a
    # step big enough to reach it in one press.
    step = hi2 - rest[2]

    print("a refused step must not reach the bus")
    bus = drive(["--id", "2", "--step", str(step)], ["t", "+", "q"])
    # The 't' key writes a hold-in-place goal at the CURRENT position;
    # that is not the jog step, so filter to moves that actually go
    # somewhere new.
    stepped = [m for m in bus.moves if m[1] != rest[2]]
    check("the gate refused and nothing new was commanded", not stepped,
          f"moves={bus.moves}")
    check("the joint is still at rest", bus.pos[2] == rest[2],
          f"{bus.pos[2]} vs rest {rest[2]}")

    print("\n--force must get through, because jog is a testing tool")
    bus = drive(["--id", "2", "--step", str(step), "--force"],
                ["t", "+", "q"])
    stepped = [m for m in bus.moves if m[1] != rest[2]]
    check("the forced step WAS commanded", bool(stepped), f"{stepped}")

    print("\n--no-gate leaves the old behaviour intact")
    bus = drive(["--id", "2", "--step", str(step), "--no-gate"],
                ["t", "+", "q"])
    stepped = [m for m in bus.moves if m[1] != rest[2]]
    check("the ungated step WAS commanded", bool(stepped), f"{stepped}")

    print("\na safe step is still allowed with the gate on")
    bus = drive(["--id", "1", "--step", "40"], ["t", "+", "q"])
    stepped = [m for m in bus.moves if m[1] != rest[1]]
    check("a small pan step went through", bool(stepped), f"{stepped}")

    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("jog gate OK")
    return 0


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        return _selftest()
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
