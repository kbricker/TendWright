"""exercise — scripted limber-up: wake, go to rest, sweep every joint, rest.

The first scripted-motion tool. It consumes calibration.json (ranges, rest
pose, per-joint soft limits) and REFUSES to move an uncalibrated arm or one
that isn't starting from its rest pose. The routine:

    wake (no lurch) -> rest pose -> per-joint sweep, one joint at a time,
    others holding, DISTAL FIRST (4 wrist_flex, 5 wrist_roll, 6 gripper,
    3 elbow, 2 shoulder, 1 base pan — rest is a compact fold; unfold the
    light end before slinging the heavy joints) -> rest -> torque off

    While the shoulder (2) sweeps, the elbow (3) AND wrist (4) hold
    90 deg open and the sweep is capped at 40% of the calibrated span —
    the contact-free envelope derived from the digital twin (sim.twin,
    plan #648), which correctly predicted both earlier bench collisions.
    They refold after. The whole routine is simulated in the twin as a
    pre-flight collision gate before anything moves (--no-gate to skip).

    uv run python -m hardware.bench.exercise
    uv run python -m hardware.bench.exercise --ids 2,3 --span 50 --speed 0.5

--ids picks which joints SWEEP; every calibrated joint is always checked,
woken, and held (a sweeping shoulder must not whip a limp elbow around).
ANY key during motion is an E-STOP; on e-stop, obstruction, or Ctrl+C the
arm halts and HOLDS while you get a hand on it — torque cuts on your Enter.

The routine itself is a CLIP (plan #660): `exercise_clip` is the one place
its poses, its guarded holds and its motion profile are written down, and
`runner.run_clip` executes exactly what the twin simulated. This file owns
the interview and the torque discipline around that, not a second copy of
the sequencing.

Exit codes: 0 done, 1 aborted, 2 error (nothing moved), 3 STOPPED MID-
ROUTINE — e-stop, strain trip, guard violation or a gate refusal, all of
which leave the arm parked between poses — 130 Ctrl+C.

--unattended is for a REMOTE run with nobody at the keyboard: it implies
--yes, gives up the keypress e-stop (the gate and the strain guard still
run; the power switch is then the only human abort), and cuts torque
immediately on a fault instead of waiting for a hand that is not there.

Usage: exercise [--ids RANGE] [--span PCT] [--speed F] [--cal FILE]
                [--port PORT] [--yes] [--unattended]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import serial

from hardware.units import DEG_PER_TICK, fmt_ticks

from .bus import (BenchError, FeetechBus, confirm, held_torque_cut,
                  require_present, run_tool)
from .calibrate import (JOINT_NAMES, JointCal, fold_direction,
                        load_joint_calibration)
from .guards import StrainWatch
from .monitor import parse_ids
from .motion import EStop, halt_all
from .posegate import PoseGate
from .runner import ClipStopped, run_clip
from .term import flush_input, read_key, require_interactive

SPEED_BASE = 200  # servo speed units at --speed 1.0 (gentler than jog's 300)
SPEED_CAP = 400  # servo-side ceiling regardless of --speed
ACCELERATION = 15
# Ceiling on --accel. Higher means the internal profile generator hands
# over to the position loop sooner, which is exactly the regime where a
# gravity-assisted joint runs away (plan #674). Not a tuning knob to
# open wide.
ACCEL_CAP = 60
SPAN_MIN, SPAN_MAX = 10, 90  # sweep % of calibrated span; >=5% end margin
SPAN_DEFAULT = 70
PREFLIGHT_RANGE_MARGIN = 25  # start-pose slack outside [min,max]
PREFLIGHT_REST_TOL_TICKS = 300  # how far from rest the arm may start
# Sweep order is distal-first (Kyle, first bench flow review): rest is a
# folded, compact pose, so unfold the wrist (4), roll it (5), work the
# gripper (6) while the heavy joints hold still, THEN open the elbow (3)
# and shoulder (2), and swing the base (1) last. Proximal-first from a
# fold slings the whole folded stack around — worst inertia, and the
# worst place to discover a range/sign surprise.
SWEEP_ORDER = (4, 5, 6, 3, 2, 1)
# Clearance holds + the m2 span cap are DERIVED FROM THE DIGITAL TWIN
# (`python -m sim.twin derive-clearance`, plan #648, 2026-07-25), which
# also predicted both real bench collisions: the original hand-tuned
# 45-deg elbow hold cleared neither the table at the sweep's low end
# (with the wrist folded) nor the arm's own shoulder at the high end.
# The model's contact-free fixed-hold answer: m2 sweeps at most 40% of
# its span while the elbow AND wrist hold 90 deg open, refolding after.
CLEARANCE_HOLDS = {2: (3, 4)}
CLEARANCE_DEG = 90
CLEARANCE_TICKS = round(CLEARANCE_DEG / DEG_PER_TICK)  # ~1024
SWEEP_SPAN_CAPS = {2: 40}  # per-joint ceiling on --span, from the twin


def show(cal: JointCal, tick: int) -> str:
    return fmt_ticks(cal.frame, tick)


def sweep_window(cal: JointCal, span_pct: int) -> tuple[int, int]:
    """The sweep sub-range: span_pct percent of [min,max], centered."""
    inset = (cal.max - cal.min) * (100 - span_pct) // 200
    return cal.min + inset, cal.max - inset


def clamp_goal(cal: JointCal, position: int) -> int:
    """Every commanded goal stays strictly inside the calibrated range —
    even the rest pose and wake holds (the loader tolerates a rest up to
    25 ticks outside [min,max]; commands must not)."""
    return max(cal.min, min(cal.max, position))


def clearance_pose(cal: JointCal, deg: float = CLEARANCE_DEG) -> int:
    """`deg` open from a folded rest, clamped in-range."""
    return clamp_goal(cal, cal.rest
                      + fold_direction(cal) * round(deg / DEG_PER_TICK))


def exercise_clip(cals: dict[int, JointCal], rest: dict[int, int],
                  windows: dict[int, tuple[int, int]],
                  sweep_ids: list[int],
                  start: dict[int, int] | None = None,
                  profile: 'MotionProfile | None' = None) -> 'Clip':
    """THE routine — the single definition of what exercise does.

    Named poses, the guarded holds each one depends on, and the profile
    that moves between them (plan #660). Everything else that describes
    or performs this routine is a projection of this function:
    `gate_waypoints` drops the names and the holds for callers that want
    bare waypoints; the bench pre-flight gate simulates it; `run_clip`
    executes it. There is no second place where the sequence is written
    down, which is the point — the duplication this replaces is how the
    gate and the executor came to disagree in the first place.

    `start` (the measured present pose) leads when given, because the
    wake -> rest move is real motion too: preflight admits a start up to
    300 ticks off rest, and that move has to be gated like any other.

    The profile carried here is the one whose `speed` and `acceleration`
    are written into the servo registers, so the path the gate simulates
    and the path the arm takes come from the same two numbers.

    THE HOLDS ARE THE SAFETY-CRITICAL PART. While the shoulder sweeps,
    the elbow and wrist must physically BE 90 deg open — a fact the twin
    derived (#648) and that #649 established cannot be taken on trust
    from the fact that they were commanded there. Carrying them on the
    poses means the executor enforces them because the CLIP says so,
    rather than because `exercise` happens to contain the code that
    checks them.
    """
    from sim.clip import DEFAULT_PROFILE, Clip, Pose

    poses: list[Pose] = []
    if start is not None:
        poses.append(Pose("start", dict(start)))
    poses.append(Pose("rest", dict(rest)))
    for i in sweep_ids:
        lo, hi = windows[i]
        clearance = {j: clearance_pose(cals[j])
                     for j in CLEARANCE_HOLDS.get(i, ()) if j != i}
        hold = {**rest, **clearance}
        # The joints that must stay open for THIS joint's sweep. Empty
        # for an unguarded joint, so the same builder covers both.
        held = tuple(sorted(clearance))
        if clearance:
            # Opening them is its own edge, and it is NOT guarded — this
            # is the move that establishes the condition, so requiring
            # the condition beforehand would be circular.
            poses.append(Pose(f"j{i} clearance", dict(hold)))
        for label, target in (("low", lo), ("high", hi), ("rest", rest[i])):
            poses.append(Pose(f"j{i} {label}",
                              {**hold, i: clamp_goal(cals[i], target)},
                              held))
        if clearance:
            poses.append(Pose(f"j{i} refold", dict(rest)))
    return Clip("exercise", poses, profile or DEFAULT_PROFILE)


def gate_waypoints(cals: dict[int, JointCal], rest: dict[int, int],
                   windows: dict[int, tuple[int, int]],
                   sweep_ids: list[int],
                   start: dict[int, int] | None = None,
                   ) -> list[dict[int, int]]:
    """The routine's pose sequence as bare dicts.

    A PROJECTION of `exercise_clip`, not a parallel definition: callers
    that only want waypoints (`sim.twin exercise_waypoints`, and through
    it the viewer) get exactly the poses the arm will be commanded to,
    because there is only one place they are computed."""
    return [dict(p.ticks) for p in
            exercise_clip(cals, rest, windows, sweep_ids, start).poses]


def check_start_pose(bus: FeetechBus, cals: dict[int, JointCal],
                     ids: list[int]) -> None:
    """Refuse to run unless every joint is inside its calibrated range and
    near its rest pose. Never start from an unknown configuration: a big
    mismatch means a changed horn, a stale file, or an arm left propped
    somewhere — a human sorts that out, not this tool."""
    for i in ids:
        c = cals[i]
        pos = bus.read_position(i)
        lo = c.min - PREFLIGHT_RANGE_MARGIN
        hi = c.max + PREFLIGHT_RANGE_MARGIN
        if not lo <= pos <= hi:
            raise BenchError(
                f"joint {i} ({c.name}) reads {show(c, pos)}, outside its "
                f"calibrated range [{show(c, c.min)}, {show(c, c.max)}]",
                "if the horn was remounted, re-run `calibrate capture "
                f"--ids {i}`; otherwise move the arm near its rest "
                "pose and re-run",
            )
        if abs(pos - c.rest) > PREFLIGHT_REST_TOL_TICKS:
            raise BenchError(
                f"joint {i} ({c.name}) reads {show(c, pos)}, "
                f"{abs(pos - c.rest)} ticks from its rest pose "
                f"({show(c, c.rest)})",
                "place the arm at its rest pose (torque-off slump) and "
                "re-run — support it first if it is holding itself (a "
                "crashed tool may have left torque latched on)",
            )


def _selftest(cal_path: Path) -> int:
    """The #660 acceptance for this routine, off the bench.

    Plan #660's contract is that the sim and the arm cannot describe
    different motion. Sharing a function is not proof of that — the
    2026-07-26 incident had the gate and the viewer sharing
    `gate_waypoints` and still disagreeing by 231 mm, because they
    consumed it through different code. So these checks are about
    IDENTITY, not agreement: the same object reaching every consumer,
    and an edit to it appearing in all of them or in none.
    """
    fails: list[str] = []

    def want(label: str, ok: bool, detail: str = "") -> None:
        print(f"  {'ok  ' if ok else 'FAIL'} {label}"
              f"{'  ' + detail if detail else ''}")
        if not ok:
            fails.append(label)

    if not cal_path.exists():
        print(f"  no {cal_path} here — cannot build the routine")
        return 1
    from sim.clip import MotionProfile, sample_clip

    cals = load_joint_calibration(cal_path)
    # `run` checks this after argument parsing; the selftest returns
    # before reaching that, and a calibration missing a clearance joint
    # would then reach `cals[j]` inside exercise_clip as a bare
    # KeyError. Every tool here must fail with one clear line, never a
    # traceback (see hardware/bench/__init__.py).
    missing = sorted(set(CLEARANCE_HOLDS) | {j for js in
                                             CLEARANCE_HOLDS.values()
                                             for j in js})
    absent = [j for j in missing if j not in cals]
    if absent:
        print(f"  {cal_path} is missing joint(s) {absent}, which the "
              f"routine's clearance holds need — cannot build it")
        return 1
    rest = {i: clamp_goal(cals[i], cals[i].rest) for i in cals}
    sweep_ids = [i for i in SWEEP_ORDER if i in cals]
    windows = {i: sweep_window(cals[i],
                               min(SPAN_DEFAULT,
                                   SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
               for i in sweep_ids}
    profile = MotionProfile(speed=SPEED_BASE, acceleration=ACCELERATION)

    def build(win):
        return exercise_clip(cals, rest, win, sweep_ids, None, profile)

    clip = build(windows)

    # --- ONE definition: every other view of the routine is derived.
    print("the routine has ONE definition; everything else projects from it")
    want("gate_waypoints is exercise_clip's poses, not a second sequence",
         gate_waypoints(cals, rest, windows, sweep_ids)
         == [dict(p.ticks) for p in clip.poses],
         f"{len(clip.poses)} poses")
    try:
        from sim.twin import exercise_waypoints
        want("...and so is the twin's exercise_waypoints, which the "
             "viewer plays",
             exercise_waypoints(cals, SPAN_DEFAULT)
             == [dict(p.ticks) for p in clip.poses])
    except ImportError as exc:                       # no mujoco here
        print(f"  --   twin not importable ({exc}); skipping its projection")

    # --- an edit reaches BOTH consumers, by the same amount.
    print("\nan edit to the routine reaches the gate AND the servos")
    narrow = dict(windows)
    narrow[1] = sweep_window(cals[1], SPAN_MIN)
    edited = build(narrow)
    j1_before = {p.ticks[1] for p in clip.poses}
    j1_after = {p.ticks[1] for p in edited.poses}
    want("narrowing joint 1's window really did change the routine",
         j1_before != j1_after,
         f"j1 span {max(j1_before) - min(j1_before)}t -> "
         f"{max(j1_after) - min(j1_after)}t")
    # THE EXECUTOR HAS TO BE IN THIS LOOP OR THE TEST PROVES NOTHING.
    # Comparing the sampled frames against the clip's own pose ticks
    # compares one object with itself: it would pass unchanged if
    # `run_clip` clamped, rescaled or re-derived every goal on its way
    # to the servos. So drive a fake bus and read back what the SERVOS
    # were actually told. This is the plan's own lesson — "shared code
    # is not proof two consumers agree" — applied to its own acceptance
    # test, which failed it on the first writing.
    frames = sample_clip(edited)

    class RecordingBus:
        """An arm that goes exactly where it is told, immediately, and
        remembers every goal register it was handed."""

        def __init__(self, start):
            self.pos = dict(start)
            self.moves: list[tuple[int, int, int, int]] = []

        def read_position(self, i):
            return self.pos[i]

        def move_to(self, i, tick, speed=400, acceleration=30):
            self.moves.append((i, tick, speed, acceleration))
            self.pos[i] = tick

        def read_health(self, i):
            return {"id": i, "load_pct": 4.0, "current_ma": 100.0,
                    "temp_c": 30, "volts": 7.4, "status": 0,
                    "faults": [], "plausible": True}

        def set_torque(self, i, on):
            pass

    import contextlib
    import io

    from .runner import run_clip

    bus = RecordingBus({i: p.ticks[i] for i in cals
                        for p in [edited.poses[0]]})
    with contextlib.redirect_stdout(io.StringIO()):
        run_clip(bus, cals, edited, gate=None, quiet=True)
    commanded = {i: {t for j, t, _, _ in bus.moves if j == i} for i in cals}

    sim_j1 = {f[1] for f in frames}
    want("the SERVOS were commanded to the ticks the routine names",
         commanded[1] == j1_after,
         f"commanded {sorted(commanded[1])} vs authored {sorted(j1_after)}")
    want("...and the gate's simulated envelope for j1 is EXACTLY the "
         "commanded envelope — one definition, reaching both",
         (min(sim_j1), max(sim_j1))
         == (min(commanded[1]), max(commanded[1])),
         f"sim [{min(sim_j1)}, {max(sim_j1)}] "
         f"vs commanded [{min(commanded[1])}, {max(commanded[1])}]")
    want("...on EVERY joint, not just the edited one",
         all((min(f[i] for f in frames), max(f[i] for f in frames))
             == (min(commanded[i]), max(commanded[i]))
             for i in cals if commanded[i]))
    stray = {i: sorted(commanded[i] - {p.ticks[i] for p in edited.poses})
             for i in cals}
    want("...and NOTHING was commanded that the routine does not name "
         "(no clamping, no rescaling, no extra goals)",
         not any(stray.values()), f"{ {i: v for i, v in stray.items() if v} }")

    # --- the guarded holds are IN the clip, so the executor enforces
    # them because the routine says so — not because this file does.
    print("\nthe shoulder's clearance guard travels with the clip")
    guarded = {p.name: p.holds for p in clip.poses if p.holds}
    expected = tuple(sorted(CLEARANCE_HOLDS.get(2, ())))
    want("the j2 sweep poses hold the joints the twin says they must",
         guarded and all(h == expected for h in guarded.values()),
         f"{sorted(guarded)} hold {expected}")
    want("...and the clearance and refold moves do NOT hold them — "
         "requiring the condition those moves establish would be circular",
         all(not p.holds for p in clip.poses
             if p.name.endswith(("clearance", "refold"))))
    held_ticks = {p.ticks[j] for p in clip.poses for j in p.holds}
    want("...at the twin-derived clearance, read off the pose itself",
         held_ticks == {clearance_pose(cals[j])
                        for j in CLEARANCE_HOLDS.get(2, ())},
         f"{sorted(held_ticks)}")

    # --- the known-bad edge, refused at VALIDATION time.
    print("\nthe run-2 geometry is refused by the gate, not by the bench")
    gate = PoseGate(sorted(cals), cal_path, profile=profile)
    if not gate.active:
        print(f"  --   gate inactive ({gate.reason}); skipping")
    else:
        # The real bench collision: the routine's OWN shoulder sweep,
        # run with the elbow and wrist still FOLDED. This is what run 2
        # did. The two calls below differ in exactly one thing — the
        # clearance — so the verdicts isolate it.
        _, hi2 = windows[2]
        clear = {j: clearance_pose(cals[j])
                 for j in CLEARANCE_HOLDS.get(2, ())}
        bad = gate.check_sequence([dict(rest), {**rest, 2: hi2}],
                                  label="folded-elbow shoulder sweep")
        want("the routine's shoulder sweep, run FOLDED, is REFUSED",
             bad.refused, bad.detail)
        want("...naming what would hit what, so it is actionable",
             "<->" in bad.detail)
        ok = gate.check_sequence([{**rest, **clear},
                                  {**rest, **clear, 2: hi2}],
                                 label="guarded shoulder sweep")
        want("...while the SAME sweep with the clearance held is allowed, "
             "so the refusal is about the geometry and not a gate that "
             "says no to everything", ok.allowed, ok.detail)

    print("exercise selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    return 1 if fails else 0


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.exercise",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ids", default=None,
                        help="joints to sweep (default: all calibrated)")
    parser.add_argument("--span", type=int, default=SPAN_DEFAULT,
                        help=f"sweep %% of each calibrated range, "
                             f"{SPAN_MIN}-{SPAN_MAX} (default {SPAN_DEFAULT})")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="speed factor 0.1-2.0 (default 1.0; capped)")
    parser.add_argument("--cal", default="calibration.json",
                        help="calibration file from calibrate capture")
    parser.add_argument("--port", default=None, help="serial port override")
    parser.add_argument("--no-gate", action="store_true",
                        help="skip the digital-twin collision gate "
                             "(sim.twin) — bench emergencies only")
    parser.add_argument("--accel", type=int, default=ACCELERATION,
                        metavar="N",
                        help=f"servo acceleration register, 1-{ACCEL_CAP} "
                             f"(default {ACCELERATION}; x100 ticks/s^2). "
                             f"The cheapest lever on a joint that overhauls "
                             f"under gravity - see plan #674")
    parser.add_argument("--trace", metavar="FILE_OR_DIR",
                        help="record the arm's ACTUAL path to CSV, then "
                             "compare it with `python -m sim.trace FILE` "
                             "(plan #660 acceptance test)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt (never the "
                             "preflight checks)")
    parser.add_argument("--unattended", action="store_true",
                        help="nobody is at this terminal: implies --yes, "
                             "and on a fault the arm's torque is cut "
                             "immediately instead of waiting for someone "
                             "to get a hand on it. There is NO keypress "
                             "e-stop in this mode — the power switch and "
                             "the automatic guards are the only aborts")
    parser.add_argument("--selftest", action="store_true",
                        help="check the routine's ONE-definition property "
                             "and its known-bad edge, with no bus and no "
                             "arm; exits non-zero on failure")
    args = parser.parse_args()
    if args.selftest:
        return _selftest(Path(args.cal))
    # A declared-unattended run has no operator to confirm anything, so
    # demanding the y/n prompt would only strand it.
    args.yes = args.yes or args.unattended

    if not SPAN_MIN <= args.span <= SPAN_MAX:
        raise BenchError(f"--span must be {SPAN_MIN}-{SPAN_MAX}",
                         "the margin outside the sweep keeps the arm off "
                         "its calibrated end stops")
    if not 0.1 <= args.speed <= 2.0:
        raise BenchError("--speed must be between 0.1 and 2.0")
    if not 1 <= args.accel <= ACCEL_CAP:
        raise BenchError(f"--accel must be 1-{ACCEL_CAP}",
                         "acceleration is REG_ACCELERATION, in units of "
                         "100 ticks/s^2")
    accel = args.accel
    speed = min(SPEED_CAP, round(SPEED_BASE * args.speed))

    if not args.unattended:
        require_interactive()  # the e-stop key is the safety channel
    else:
        # Said out loud on every run, because the thing being given up
        # is the operator's own abort channel. What remains: the twin's
        # pre-flight gate, the strain guard (which needs no human), and
        # the power switch.
        print("UNATTENDED — no keypress e-stop. The collision gate and "
              "strain guard still run; the POWER SWITCH is the only "
              "human abort.", file=sys.stderr)

    cal_path = Path(args.cal)
    if not cal_path.exists():
        raise BenchError(
            f"no calibration file at {cal_path}",
            "this tool refuses to move an uncalibrated arm — run "
            "`calibrate capture` first (or point --cal at the file)",
        )
    cals = load_joint_calibration(cal_path)
    uncaptured = sorted(set(JOINT_NAMES) - set(cals))
    if uncaptured:
        raise BenchError(
            f"{cal_path} covers joint(s) {sorted(cals)} but the SO-101 "
            f"has 1-6 — joint(s) {uncaptured} would hang limp while the "
            "others sweep",
            "capture the missing joint(s) first: calibrate capture --ids "
            + ",".join(str(i) for i in uncaptured),
        )

    # --ids selects the SWEEP subset; every joint is preflighted, woken,
    # and held regardless — big joints must never sweep past limp,
    # unmonitored distal ones (which is also why a partial calibration
    # file is refused above).
    ids = sorted(cals)
    if args.ids is None:
        sweep_ids = [i for i in SWEEP_ORDER if i in cals]
    else:
        requested = set(parse_ids(args.ids))
        sweep_ids = [i for i in SWEEP_ORDER if i in requested]
        unknown = sorted(requested - set(JOINT_NAMES))
        if unknown:
            raise BenchError(
                f"unknown joint ID(s) {unknown}",
                "the SO-101 follower uses IDs 1-6 (base to gripper)")
        uncalibrated = sorted(requested - set(cals))
        if uncalibrated:
            raise BenchError(
                f"joint(s) {uncalibrated} are not in {cal_path}",
                "capture them first: calibrate capture --ids "
                + ",".join(str(i) for i in uncalibrated),
            )

    with FeetechBus(args.port) as bus:
        require_present(bus, ids,
                        "every calibrated joint must be on the bus to be "
                        "held during sweeps — run the scan tool")

        check_start_pose(bus, cals, ids)
        # The arm is verified at rest, where torque-off is a no-drop
        # operation — so enforce the assumed baseline instead of trusting
        # it (a crashed tool may have left torque latched on).
        bus.safe_torque_off(ids)

        rest = {i: clamp_goal(cals[i], cals[i].rest) for i in ids}
        # Per-joint span caps from the twin: --span still shrinks a
        # sweep, but can never widen past the derived-safe ceiling.
        windows = {i: sweep_window(
            cals[i], min(args.span, SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
            for i in sweep_ids}
        print(f"exercise routine on {bus.port_name}: sweep joint(s) "
              f"{sweep_ids}, hold {[i for i in ids if i not in sweep_ids] or 'none'}")
        print(f"  wake -> rest -> sweep each joint through {args.span}% of "
              f"its range -> rest -> torque off")
        for i in sweep_ids:
            lo, hi = windows[i]
            print(f"    joint {i} ({cals[i].name:<13}) "
                  f"rest {show(cals[i], rest[i]):>16}  "
                  f"sweep {show(cals[i], lo):>16} -> {show(cals[i], hi)}")
        for i in sweep_ids:
            opened = [j for j in CLEARANCE_HOLDS.get(i, ()) if j != i]
            if opened:
                print(f"    joint {i} sweeps only while joint(s) {opened} "
                      f"hold {CLEARANCE_DEG} deg clear of the fold, so it "
                      f"cannot press them into the table — verified from "
                      f"the encoders, not assumed from the command")
        print("  keep the workspace clear and the gripper empty.")
        print("  ANY key during motion is an E-STOP (halt + hold, torque "
              "cuts on your Enter). The power switch is the hard e-stop.")

        # THE CLIP. Built ONCE, here, and then both gated and executed —
        # so "what was simulated" and "what the arm does" are the same
        # object, not two constructions that happen to agree today
        # (plan #660). Everything below consumes this variable; nothing
        # rebuilds the sequence.
        try:
            from sim.clip import MotionProfile
        except ImportError as exc:
            raise BenchError(
                f"the clip layer is unavailable ({exc})",
                "run `uv sync`") from exc
        profile = MotionProfile(speed=speed, acceleration=accel)
        gate = None
        if not args.no_gate:
            gate = PoseGate(ids, cal_path, profile=profile)
            if not gate.active:
                raise BenchError(
                    f"collision gate unavailable ({gate.reason})",
                    "run `uv sync` for mujoco, or --no-gate to skip "
                    "(bench emergencies only)")

        def build_and_gate(why: str):
            """Read the arm, build the routine from THAT reading, and
            simulate it.

            Pose 0 is the measured start, and `run_clip` drives the arm
            to pose 0 before the clip proper begins — an approach move
            that no gate covers, because `check_clip` treats pose 0 as
            where the arm already is. So pose 0 must never be a STALE
            reading: torque is off from here until the wake below, the
            confirm prompt can sit open for minutes, and a limp arm
            creeps. Building from an old reading would send the arm
            backwards to a pose it left, over a path nothing simulated.

            Hence this runs twice — once for the operator to judge, once
            after they answer. The second pass costs a couple of seconds
            of MuJoCo and removes the whole class of problem.

            The start is CLAMPED, unlike the raw reading: `check_start_pose`
            admits up to PREFLIGHT_RANGE_MARGIN ticks outside the
            calibrated range, and pose 0 is COMMANDED. Every commanded
            goal stays strictly inside the range (see `clamp_goal`) —
            the approach is not an exception to that rule.
            """
            here = {i: clamp_goal(cals[i], bus.read_position(i))
                    for i in ids}
            built = exercise_clip(cals, rest, windows, sweep_ids, here,
                                  profile)
            if gate is None:
                return built
            report = gate.twin.check_clip(
                built,
                # pose 0 is the arm's measured slump, not a plan
                settle_from_measured=True)
            # The summary carries clamp warnings — an anchor/range
            # mismatch is loudest exactly when the gate PASSES, so
            # always print it, never only on failure.
            print(report.summary(cals),
                  file=sys.stderr if not report.clean else sys.stdout)
            if not report.clean:
                raise BenchError(
                    f"collision gate predicts contact ({why}) — refusing "
                    f"to run",
                    "the twin says this routine would collide (details "
                    "above); adjust --span/--ids, or --no-gate only if "
                    "you are CERTAIN the model is wrong")
            return built

        # Pre-flight: simulate the exact routine in the digital twin
        # BEFORE anything energizes. The gate consumes the CLIP, carrying
        # `profile` — the same speed/acceleration written into the servo
        # registers — so it simulates the path the arm will actually take
        # between poses, not a lockstep glide the servos never perform.
        build_and_gate("pre-flight")

        if not args.yes and not confirm("type y to start: "):
            print("aborted")
            return 1

        # The pose was vetted BEFORE the prompt, which can sit open for
        # minutes — re-vet after it, right before anything energizes.
        check_start_pose(bus, cals, ids)
        # ...and re-build from the pose the arm is in NOW, so what runs
        # is what was just simulated. If the arm moved while the prompt
        # was open, this is where the twin sees it.
        clip = build_and_gate("re-checked after the prompt")

        # Built BEFORE the try, so the finally that flushes it can never
        # hit an unbound name on an early failure.
        # The automatic abort. The keypress e-stop needs a human; this
        # does not — the servos report load, temperature and their own
        # fault bits, and a joint pushing against something says so
        # before anything breaks. Watched on EVERY move, not just the
        # guarded sweep, because a collision does not care which phase
        # the routine is in.
        strain = StrainWatch(ids)
        # NO KEYPRESS E-STOP MEANS NO KEY POLLING. `read_key` calls
        # require_interactive() itself, so polling for an abort
        # nobody can issue does not merely do nothing — it raises
        # mid-motion and aborts the run it was meant to protect.
        poll = None if args.unattended else read_key

        trace = None
        if args.trace:
            import time as _time

            from sim.trace import Trace
            dest = Path(args.trace)
            # A directory means "a batch": auto-name so repeated runs
            # accumulate instead of overwriting each other. Losing run 4
            # of 6 because it reused a filename is a wasted bench trip.
            if dest.is_dir() or args.trace.endswith(("/", "\\")):
                stamp = _time.strftime("%Y%m%d-%H%M%S")
                dest = dest / (f"run-{stamp}-sp{speed}-ac{accel}"
                               f"-sn{args.span}.csv")
            trace = Trace(dest, meta={
                "speed": speed, "accel": accel, "span": args.span,
                "ids": sweep_ids, "cal": str(cal_path),
            })
            print(f"tracing the arm's actual path to {dest}")

        try:
            # Wake without lurch, one servo at a time: pre-load the goal
            # (and speed/accel) to the CURRENT position while still torque
            # off, then enable torque — never against a stale goal register.
            print("\nwaking (torque on, holding in place)...")
            for i in ids:
                bus.move_to(i, clamp_goal(cals[i], bus.read_position(i)),
                            speed=speed, acceleration=accel)
                bus.set_torque(i, True)

            # THE ROUTINE, run by the general executor. Everything the
            # bespoke loop here used to do — settle on the plant, the
            # strain invariant, the clearance entry guard and its
            # in-motion re-check, the trace phases — `run_clip` does,
            # from the clip the gate just simulated. What exercise adds
            # on top is its own: the pre-flight interview above and the
            # torque discipline below.
            #
            # It also gains something the hand-rolled loop never had:
            # every edge is RE-GATED from the arm's measured pose, so a
            # joint that settled 25 ticks off-plan is judged where it
            # actually is rather than where the plan assumed.
            outcome = run_clip(bus, cals, clip, gate=gate, strain=strain,
                               trace=trace, poll_key=poll)
            print(f"\n{outcome.summary()}")
            print("routine complete — arm at rest, cutting torque")
            return 0
        except ClipStopped as exc:
            # run_clip converts an e-stop, a strain trip, a guard
            # violation and a settle failure into one stop report that
            # says WHERE the arm is holding. exercise still distinguishes
            # the operator's e-stop in its exit code, because a human
            # pressing a key is not a fault.
            print(f"\nSTOPPED: {exc}", file=sys.stderr)
            print("the arm is holding HERE:", file=sys.stderr)
            print(exc.where(), file=sys.stderr)
            # The hint is the recovery instruction and str() does not
            # carry it — see the same print in runner.cmd_run.
            if exc.hint:
                print(f"hint:  {exc.hint}", file=sys.stderr)
            estop = isinstance(exc.__cause__, EStop)
            held_torque_cut("e-stop" if estop else "stopped", args.unattended)
            return 3
        except EStop:
            # Only reachable from the wake loop above, which run_clip
            # does not own. Kept rather than folded in: dropping it would
            # make a keypress during wake an unhandled error.
            print("\nE-STOP — halting at present position", file=sys.stderr)
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            held_torque_cut("e-stop", args.unattended)
            return 3
        except (BenchError, serial.SerialException):
            # SerialException is the raw pyserial fault _check doesn't
            # wrap — a transient USB glitch mid-sweep must get the same
            # held cut, not a surprise drop. Halt is best-effort (a dead
            # bus just no-ops per joint); the arm may still be holding
            # mid-air — never cut torque on an unwarned operator.
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            held_torque_cut("error", args.unattended)
            raise
        except KeyboardInterrupt:
            try:
                halt_all(bus, ids)
            except KeyboardInterrupt:
                pass
            held_torque_cut("interrupted", args.unattended)
            raise
        finally:
            bus.safe_torque_off(ids)
            # THE HEADLINE NUMBER OF A BASELINE RUN. The strain guard
            # has recorded per-joint peak load and temperature the whole
            # way, and until now nothing ever printed it — the guard
            # could stop the arm but could not tell you how close a
            # HEALTHY run came to the same thresholds, which is the one
            # thing needed to judge whether they are set sensibly.
            #
            # In the finally, so it survives every exit path: an aborted
            # or strain-tripped run is exactly when the numbers matter
            # most. After the torque cut, for the same reason the trace
            # write is — the arm being safe is never delayed by
            # bookkeeping.
            print(strain.summary())
            # Written on EVERY exit path — an aborted, e-stopped or
            # obstructed run is exactly when the trace is most worth
            # having.
            if trace is not None:
                written = trace.close()
                if written is not None:
                    print(f"trace: {len(trace)} samples -> {written}")
                    print(f"  compare: uv run python -m sim.trace {written}")
                elif trace.error:
                    print(f"trace: {trace.error}", file=sys.stderr)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
