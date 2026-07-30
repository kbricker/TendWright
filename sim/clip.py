"""Motion clips — ONE definition of a move, for the sim and the arm.

Plan #660. The contract Kyle set (2026-07-26): *"we can run things in
mojoco, see them and validate them in the sim before running on the real
arm, but we will always get the same motion we see in mojoco."*

That needs two things to be true, and before this module only the first
was:

1. **Same poses.** The gate and the viewer already shared
   `gate_waypoints()`, so the ENDPOINTS agreed.
2. **Same path between poses.** They did not. The sim slid every joint
   linearly and in lockstep, all arriving together; each real servo runs
   its own trapezoidal speed/accel ramp and arrives when its own travel
   is done. On any multi-joint move the arm traced a genuinely different
   path than the sim between identical endpoints — so poses the gate
   certified were poses the arm never occupied, and the arm passed
   through poses the gate never checked.

So a clip carries its MOTION PROFILE, not just its waypoints, and one
sampler turns (poses + profile) into timed frames. Three consumers use
that one sampler:

    sim.twin       — collision-gates the sampled frames
    sim.bench_scene — plays the sampled frames in the viewer
    hardware.bench — commands the servos with the SAME profile numbers

The third is the point. `MotionProfile.speed` and `.acceleration` are
written straight into the servo's registers, so the profile the sim
animates is not a model OF the arm's motion — it is the arm's motion
parameters.

WHAT THIS DOES NOT CLAIM. The trapezoid here is the servo's commanded
profile, not observed behaviour: a real joint lags under gravity, and a
loaded joint lags more. The sim is therefore the arm's INTENT, faithful
to what the servo was told. Closing the remaining gap needs encoder
traces compared against these frames, which is this plan's bench
acceptance test — not something any amount of code here can assert.

Register semantics (Feetech STS3215), which the timing rests on:
    REG_GOAL_SPEED  — ticks/s directly
    REG_ACCELERATION — units of 100 ticks/s^2
Both are documented values, and both are only CONFIRMED by that same
bench trace. If the arm turns out to arrive systematically early or
late, suspect these two constants first.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from hardware.errors import BenchError

POSES_JSON = Path(__file__).resolve().parent.parent / "poses.json"

# REG_ACCELERATION counts in hundreds of ticks/s^2 (STS3215).
ACCEL_UNIT_TICKS_S2 = 100.0

# Playback/gating sample rate.
#
# The bound that matters is not "frames look smooth" — it is that a link
# must not move further between two samples than the gate's contact
# margin, or a thin obstacle fits BETWEEN samples and the gate walks
# past it. Measured on the exercise routine (tool travel per sample,
# against a 5 mm margin):
#
#     100 Hz  10084 poses   worst 0.95 mm   5x headroom   <- here
#      50 Hz   5045 poses   worst 1.90 mm   2.6x
#      25 Hz   2531 poses   worst 3.79 mm   barely under
#      10 Hz   1018 poses   worst 9.48 mm   TUNNELS
#
# For contrast the angle-stepped gate this replaced sampled every 2 deg,
# about 23 ticks, roughly 12 mm of tool travel at full reach — over
# TWICE the margin. Its smaller pose count was under-sampling, not
# efficiency.
#
# 100 Hz costs 2.4 s to gate the whole routine, so the headroom is free.
# `sim.twin selftest` asserts the property rather than trusting this
# comment: it measures the worst step and fails if it approaches the
# margin, and pairs that with a deliberately coarse rate that must
# breach it.
DEFAULT_HZ = 100.0


@dataclass(frozen=True)
class MotionProfile:
    """The shape of a move, in the servo's own units.

    These two numbers go into REG_GOAL_SPEED and REG_ACCELERATION
    unchanged. That is what couples the simulated path to the real one:
    there is no separate "sim speed" that could drift from the commanded
    speed, because they are the same field."""

    speed: int = 400            # ticks/s
    acceleration: int = 30      # x100 ticks/s^2

    def __post_init__(self) -> None:
        if self.speed <= 0:
            raise BenchError(f"motion profile speed must be positive, "
                             f"got {self.speed}",
                             "speed is REG_GOAL_SPEED, in ticks/s")
        if self.acceleration <= 0:
            raise BenchError(f"motion profile acceleration must be "
                             f"positive, got {self.acceleration}",
                             "acceleration is REG_ACCELERATION, in units "
                             "of 100 ticks/s^2")

    @property
    def accel(self) -> float:
        """ticks/s^2."""
        return self.acceleration * ACCEL_UNIT_TICKS_S2

    def duration(self, distance: float) -> float:
        """Seconds for ONE joint to travel `distance` ticks.

        Trapezoidal, and triangular when the move is too short to reach
        cruise speed — which is the common case for the small corrective
        moves in a routine, so it is not an edge case to skip."""
        d = abs(float(distance))
        if d == 0.0:
            return 0.0
        d_accel = self.speed ** 2 / self.accel     # ramp up + ramp down
        if d <= d_accel:
            return 2.0 * math.sqrt(d / self.accel)         # triangular
        return 2.0 * (self.speed / self.accel) + (d - d_accel) / self.speed

    def travelled(self, distance: float, t: float) -> float:
        """Signed ticks covered by time `t` into a move of `distance`."""
        d = abs(float(distance))
        sign = 1.0 if distance >= 0 else -1.0
        if d == 0.0 or t <= 0.0:
            return 0.0
        total = self.duration(d)
        if t >= total:
            return sign * d
        d_accel = self.speed ** 2 / self.accel
        if d <= d_accel:                                    # triangular
            half = total / 2.0
            if t <= half:
                return sign * 0.5 * self.accel * t * t
            r = total - t
            return sign * (d - 0.5 * self.accel * r * r)
        t_ramp = self.speed / self.accel
        if t <= t_ramp:
            return sign * 0.5 * self.accel * t * t
        if t <= total - t_ramp:
            return sign * (0.5 * self.speed * t_ramp
                           + self.speed * (t - t_ramp))
        r = total - t
        return sign * (d - 0.5 * self.accel * r * r)


DEFAULT_PROFILE = MotionProfile()


@dataclass(frozen=True)
class Pose:
    """A named whole-arm pose, in calibrated ticks.

    `holds` names joints that must PHYSICALLY be where this pose puts
    them — read back from the encoders before the edge ending here is
    commanded, and re-read every sample while it plays. It is plan
    #649's guarded hold ("a commanded hold is not a held joint") carried
    at clip scale, and it is what lets a routine whose safety depends on
    another joint staying open be expressed as data rather than as
    bespoke Python.

    ON THE TARGET POSE RATHER THAN THE EDGE, for two reasons:

    * There is no Edge object here — `edges()` derives pairs from the
      pose list. Holds indexed per-edge would be a second list to keep
      in step with the first, and an edit that inserted a pose would
      silently shift every hold onto the wrong move.
    * The tick a hold is checked against is `ticks[j]` — the SAME field
      the servo is commanded from. There is no separate "guard value"
      that could drift from the commanded value, which is the class of
      bug this whole module exists to remove.

    A hold naming a joint the pose does not pin is refused at
    construction: there would be no position to check it against, and
    inventing one (the previous pose? the arm's current position?) is
    exactly the guess a guard must never make.

    Note what that check does and does not reach. It fires on poses
    built in CODE, where a missing joint really is missing. A pose
    loaded from a clip file is complete by carry-forward before it gets
    here, so it always passes — which is why `load_clip` carries the
    separate rule that a hold must be INTRODUCED by a pose that states
    the position it asserts. Inherited ticks are the danger there, not
    absent ones.
    """

    name: str
    ticks: dict[int, int]
    holds: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        missing = [j for j in self.holds if j not in self.ticks]
        if missing:
            raise BenchError(
                f"pose '{self.name}' holds joint(s) {missing}, which it "
                f"does not position",
                "a held joint needs a commanded tick to be checked "
                "against; pin it in this pose or drop the hold")

    # `merged(**override)` used to live here and was removed 2026-07-30:
    # it had no callers, and it could not have had any — `**kwargs`
    # requires string keywords while every joint id is an int, so
    # `merged(**{3: 100})` raises "keywords must be strings". Use
    # `Pose(p.name, {**p.ticks, 3: 100}, p.holds)`, or `Clip.resolved`,
    # which is what actually carries joints forward.


@dataclass
class Clip:
    """An ordered walk through poses, at one motion profile.

    The profile lives on the CLIP rather than on the player, so that
    "how this move is made" travels with "what the move is". A player
    that supplied its own speed could animate a path the arm will never
    take, which is the whole defect this module exists to remove."""

    name: str
    poses: list[Pose]
    profile: MotionProfile = field(default_factory=lambda: DEFAULT_PROFILE)

    def edges(self) -> list[tuple[Pose, Pose]]:
        return list(zip(self.poses, self.poses[1:]))

    def resolved(self, start: dict[int, int]) -> 'Clip':
        """Every pose made COMPLETE, carrying joints forward.

        A clip built in code — by IK, by a pick-place planner — pins only
        the joints it cares about and leaves the rest implied. Implied
        is fine for authoring and dangerous for gating: a pose that does
        not mention joint 3 is not a pose where joint 3 is absent, it is
        a pose where joint 3 is wherever the previous pose left it. The
        gate and the servos must both be handed the second reading, and
        the only way to guarantee they agree is to resolve it once, here,
        before either of them sees the clip.

        `start` supplies the joints the FIRST pose does not pin — in
        practice the arm's measured position. `load_clip` resolves at
        load time instead (its first pose must be complete, so a file
        means the same motion wherever it is run); calling this on an
        already-complete clip is a no-op.
        """
        out: list[Pose] = []
        carry = dict(start)
        for p in self.poses:
            carry = {**carry, **p.ticks}
            out.append(Pose(p.name, dict(carry), p.holds))
        return Clip(self.name, out, self.profile)


def edge_duration(profile: MotionProfile, a: Pose, b: Pose) -> float:
    """Seconds until EVERY joint of the edge has arrived.

    Each joint runs its own ramp, so the edge takes as long as its
    longest-travelling joint — and the others are already parked by
    then. Lockstep interpolation is exactly what this replaces."""
    ids = sorted(set(a.ticks) | set(b.ticks))
    return max((profile.duration(b.ticks.get(i, a.ticks[i])
                                 - a.ticks.get(i, b.ticks[i]))
                for i in ids), default=0.0)


def sample_edge(profile: MotionProfile, a: Pose, b: Pose,
                hz: float = DEFAULT_HZ) -> list[dict[int, int]]:
    """Frames from `a` to `b`, each joint on its OWN ramp.

    A short-travel joint finishes early and then HOLDS, which is what
    the arm does and what lockstep interpolation gets wrong: under
    lockstep the short joint crawls, staying in places the real one has
    already left."""
    ids = sorted(set(a.ticks) | set(b.ticks))
    deltas = {i: b.ticks.get(i, a.ticks[i]) - a.ticks.get(i, b.ticks[i])
              for i in ids}
    total = edge_duration(profile, a, b)
    if total <= 0.0:
        return [dict(b.ticks)]
    steps = max(1, int(math.ceil(total * hz)))
    out: list[dict[int, int]] = []
    for n in range(1, steps + 1):
        t = total * n / steps
        out.append({i: round(a.ticks.get(i, b.ticks[i])
                             + profile.travelled(deltas[i], t))
                    for i in ids})
    return out


def sample_clip(clip: Clip, hz: float = DEFAULT_HZ,
                ) -> list[dict[int, int]]:
    """Every frame of a clip, starting at its first pose.

    This is THE definition of what the clip looks like in motion. The
    collision gate, the viewer and (via the profile) the servos all
    resolve to this one function, so an edit to a clip cannot reach one
    of them without reaching the others."""
    if not clip.poses:
        return []
    frames = [dict(clip.poses[0].ticks)]
    for a, b in clip.edges():
        frames.extend(sample_edge(clip.profile, a, b, hz))
    return frames


def clip_duration(clip: Clip) -> float:
    return sum(edge_duration(clip.profile, a, b) for a, b in clip.edges())


# --------------------------------------------------------- pose library
def pose_from_doc(cals: dict, name: str, body: dict) -> Pose:
    """One authored pose — human units in, validated ticks out.

    Shared by `load_poses` and the clip loader so a pose means the same
    thing wherever it is written down. Every value is range-checked
    against the CALIBRATED range here, at load, rather than clamped
    later at command time: a pose the arm cannot reach is an authoring
    mistake, and silently clamping it moves the tool somewhere the
    author never asked for (the same failure #670 recorded in IK).
    """
    ticks: dict[int, int] = {}
    for key, value in body.items():
        try:
            i = int(key)
        except (TypeError, ValueError):
            raise BenchError(
                f"pose '{name}' has non-numeric joint key {key!r}",
                "joint keys are servo ids, e.g. \"3\"") from None
        cal = cals.get(i)
        if cal is None:
            raise BenchError(
                f"pose '{name}' names joint {i}, which is not in "
                f"calibration.json",
                "calibrate capture the joint, or drop it from the pose")
        if cal.frame is None:
            # Without a frame there is no way to tell 45 degrees from 45
            # ticks, and the two differ by a factor of 11. Refuse rather
            # than pick one.
            raise BenchError(
                f"pose '{name}' names joint {i} ({cal.name}), which has no "
                f"calibrated frame, so a human-unit value cannot be read",
                f"re-run `calibrate capture --ids {i}` to establish the "
                f"joint's zero and direction")
        try:
            tick = cal.frame.tick(float(value))
        except (TypeError, ValueError):
            raise BenchError(
                f"pose '{name}' gives joint {i} a non-numeric value "
                f"{value!r}",
                "pose values are degrees (gripper: percent open)") from None
        if not cal.min <= tick <= cal.max:
            raise BenchError(
                f"pose '{name}' puts joint {i} ({cal.name}) at "
                f"{value} -> tick {tick}, outside its calibrated "
                f"range {cal.min}..{cal.max}",
                "a pose outside the calibrated range is unreachable; "
                "re-author it or re-calibrate the joint")
        ticks[i] = tick
    return Pose(name, ticks)


def load_poses(cals: dict, path: Path = POSES_JSON) -> dict[str, Pose]:
    """Named poses as DATA, in human units, validated against the
    calibrated range on load.

    Authored in degrees (and gripper percent) rather than ticks so that
    a pose file is readable and reviewable — a wall of tick counts is
    neither, and this file is meant to be edited by hand."""
    if not path.exists():
        return {}
    try:
        doc = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise BenchError(f"could not read {path.name}: {exc}",
                         "poses.json must be valid JSON") from exc
    return {name: pose_from_doc(cals, name, body)
            for name, body in (doc.get("poses") or {}).items()}


# ---------------------------------------------------------- clip library
CLIP_FORMAT_VERSION = 1


def _holds_from_doc(where: str, label: str, raw, calibrated: set) -> tuple:
    """Parse a pose entry's `holds` list — joint ids, nothing else.

    Deliberately NOT accepting an angle here. A hold is checked against
    the pose's own commanded tick; letting a file state a second number
    would create exactly the gap between "what was commanded" and "what
    is guarded" that the hold exists to close.
    """
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise BenchError(f"{where}: pose '{label}' has a non-list `holds`",
                         "holds is a list of servo ids, e.g. [3, 4]")
    out: list[int] = []
    for item in raw:
        try:
            j = int(item)
        except (TypeError, ValueError):
            raise BenchError(
                f"{where}: pose '{label}' holds {item!r}, which is not a "
                f"joint id", "holds is a list of servo ids, e.g. [3, 4]"
            ) from None
        if j not in calibrated:
            raise BenchError(
                f"{where}: pose '{label}' holds joint {j}, which is not in "
                f"calibration.json",
                f"capture it (`calibrate capture --ids {j}`), or drop it "
                f"from `holds`")
        if j not in out:
            out.append(j)
    return tuple(sorted(out))


def load_clip(cals: dict, path: Path | str,
              library: dict[str, Pose] | None = None) -> Clip:
    """A clip authored as JSON — the file form of what `Clip` holds.

        {"version": 1, "name": "pick",
         "profile": {"speed": 300, "acceleration": 15},
         "poses": [{"name": "home",  "joints": {"1": 0, "2": -20, ...}},
                   {"name": "above", "joints": {"2": -45}},
                   {"name": "down",  "pose": "grip_height"}]}

    A pose may also carry `"holds": [3, 4]` — joints whose commanded
    position in THIS pose is a safety precondition, read back from the
    encoders before the edge into it is commanded and re-read while it
    plays. See `Pose.holds`.

    Two authoring conveniences, both chosen to remove a way to be wrong
    rather than to save typing:

    * A pose may `"pose"`-reference an entry in poses.json, so a height
      that matters lives in ONE place and every clip using it moves when
      it is re-measured.
    * A pose names only the joints it CHANGES; the rest carry forward
      from the pose before it. Re-stating six joints per waypoint is how
      a typo in an unrelated joint gets into a clip unnoticed.

    The FIRST pose must name every calibrated joint. Carry-forward needs
    somewhere to start, and the alternative — inheriting from wherever
    the arm happens to be sitting — would make the same file mean a
    different motion on every run, which is exactly the property this
    module exists to prevent. Getting to that first pose from the arm's
    actual position is the runner's approach move, and it is gated.
    """
    path = Path(path)
    if not path.exists():
        raise BenchError(f"no such clip: {path}",
                         "clips are JSON; see `runner example` for the shape")
    try:
        doc = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise BenchError(f"could not read {path.name}: {exc}",
                         "a clip file must be valid JSON") from exc
    if not isinstance(doc, dict):
        raise BenchError(f"{path.name} is not a clip",
                         "expected a JSON object with a `poses` list")
    version = doc.get("version", CLIP_FORMAT_VERSION)
    if version != CLIP_FORMAT_VERSION:
        raise BenchError(
            f"{path.name} is clip format version {version}, this build "
            f"reads version {CLIP_FORMAT_VERSION}",
            "the file was written by a different version of the tools")

    prof = doc.get("profile") or {}
    if not isinstance(prof, dict):
        raise BenchError(f"{path.name}: `profile` must be an object",
                         "e.g. {\"speed\": 300, \"acceleration\": 15}")
    try:
        profile = MotionProfile(
            speed=int(prof.get("speed", DEFAULT_PROFILE.speed)),
            acceleration=int(prof.get("acceleration",
                                      DEFAULT_PROFILE.acceleration)))
    except (TypeError, ValueError) as exc:
        raise BenchError(f"{path.name}: profile speed and acceleration "
                         f"must be whole numbers ({exc})") from None

    entries = doc.get("poses")
    if not isinstance(entries, list) or len(entries) < 2:
        raise BenchError(
            f"{path.name} has no motion — a clip needs at least two poses",
            "a single pose is a position, not a move")

    library = library if library is not None else load_poses(cals)
    want = set(cals)
    poses: list[Pose] = []
    for n, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise BenchError(f"{path.name}: pose {n} is not an object")
        label = str(entry.get("name") or f"p{n}")
        if "pose" in entry and "joints" in entry:
            # Silently merging them would make the file's meaning depend
            # on a precedence rule nobody wrote down.
            raise BenchError(
                f"{path.name}: pose '{label}' has both `pose` and "
                f"`joints`",
                "use one or the other")
        if "pose" in entry:
            ref = entry["pose"]
            if ref not in library:
                known = ", ".join(sorted(library)) or "none defined"
                raise BenchError(
                    f"{path.name}: pose '{label}' references '{ref}', "
                    f"which is not in poses.json",
                    f"known poses: {known}")
            named = library[ref].ticks
        elif "joints" in entry:
            named = pose_from_doc(cals, label, entry["joints"] or {}).ticks
        else:
            raise BenchError(
                f"{path.name}: pose '{label}' names no joints",
                "give it `joints` or a `pose` reference")
        if not poses:
            missing = sorted(want - set(named))
            if missing:
                raise BenchError(
                    f"{path.name}: the first pose ('{label}') must name "
                    f"every calibrated joint; missing {missing}",
                    "carry-forward starts here, so a joint absent from the "
                    "first pose has no value to carry")
            ticks = dict(named)
        else:
            ticks = {**poses[-1].ticks, **named}
        extra = sorted(set(ticks) - want)
        if extra:
            raise BenchError(
                f"{path.name}: pose '{label}' names joint(s) {extra}, "
                f"which are not calibrated")
        holds = _holds_from_doc(path.name, label, entry.get("holds"), want)
        # A hold must be INTRODUCED by a pose that states the position
        # it is asserting, and may then be inherited by poses that keep
        # holding it. Without this rule carry-forward silently supplies
        # a value the author never looked at:
        #
        #     open   {"3": 90}            elbow open
        #     sweepA {"2": 40, holds:[3]} guards j3 at 90  — correct
        #     refold {"3": 0}             elbow folded again
        #     sweepB {"2": 40, holds:[3]} guards j3 at 0   — USELESS
        #
        # sweepB looks identical to sweepA and certifies the folded
        # configuration. The author wrote the same characters twice and
        # got a guard protecting nothing. Requiring the first pose of a
        # hold to name the joint makes that line impossible to write by
        # accident: sweepB must say what it is guarding, and saying it
        # is what reveals the value is wrong.
        inherited = poses[-1].holds if poses else ()
        silent = sorted(j for j in holds
                        if j not in named and j not in inherited)
        if silent:
            raise BenchError(
                f"{path.name}: pose '{label}' holds joint(s) {silent} "
                f"without positioning them, and the pose before it does "
                f"not hold them either",
                "the pose that first holds a joint must say where — "
                "otherwise the guarded value is whatever an earlier pose "
                "happened to leave there. Add it to `joints`, or hold it "
                "from the pose that opens it")
        poses.append(Pose(label, ticks, holds))

    return Clip(str(doc.get("name") or path.stem), poses, profile)


# ------------------------------------------------------------- selftest
def _selftest() -> None:
    """Pin the property the whole module exists for: joints arrive on
    their OWN schedule, and every acceptance is paired with a refusal."""
    p = MotionProfile(speed=400, acceleration=30)
    fails: list[str] = []

    def want(label: str, ok: bool) -> None:
        if not ok:
            fails.append(label)
        print(f"  [{'ok ' if ok else 'FAIL'}] {label}")

    # A long move takes longer than a short one — the property lockstep
    # interpolation destroys.
    want("a longer move takes longer", p.duration(2000) > p.duration(200))
    want("...and NOT proportionally, because of the ramps "
         "(lockstep would be exactly 10x)",
         abs(p.duration(2000) / p.duration(200) - 10.0) > 0.5)

    # Travel is monotone and lands exactly on the endpoint.
    d = 1500
    t_end = p.duration(d)
    want("a move ends exactly on its target",
         abs(p.travelled(d, t_end) - d) < 1e-6)
    want("...and does not overshoot past the end",
         abs(p.travelled(d, t_end * 2) - d) < 1e-6)
    want("travel is monotone", all(
        p.travelled(d, t_end * k / 50) <= p.travelled(d, t_end * (k + 1) / 50)
        + 1e-9 for k in range(50)))
    want("a negative move mirrors a positive one",
         abs(p.travelled(-d, t_end * 0.3) + p.travelled(d, t_end * 0.3)) < 1e-9)

    # Short moves are triangular; the servo never reaches cruise speed.
    short = 10
    want("a short move stays under the cruise-speed distance",
         p.duration(short) < 2.0 * p.speed / p.accel + 1e-9)

    # THE headline: a two-joint edge where one joint travels 10x further.
    a = Pose("a", {1: 1000, 2: 1000})
    b = Pose("b", {1: 1100, 2: 2000})
    frames = sample_edge(p, a, b, hz=200.0)
    arrive = {i: next(n for n, f in enumerate(frames) if f[i] == b.ticks[i])
              for i in (1, 2)}
    want("the short-travel joint arrives FIRST (the arm's behaviour)",
         arrive[1] < arrive[2])
    want("...and then holds at its target while the other finishes",
         all(f[1] == b.ticks[1] for f in frames[arrive[1]:]))
    want("both joints are at the target on the final frame",
         frames[-1] == {1: 1100, 2: 2000})
    # Lockstep would have them arrive together — assert we are NOT that.
    want("this is NOT lockstep interpolation",
         arrive[1] != arrive[2])

    # Carry-forward: the property a code-built clip depends on, and the
    # one whose absence would let the gate judge a different motion than
    # the servos perform.
    sparse = Clip("sparse", [Pose("x", {1: 100}), Pose("y", {2: 200}),
                             Pose("z", {1: 300})])
    r = sparse.resolved({1: 0, 2: 0, 3: 7})
    want("resolving fills the first pose from the start pose",
         r.poses[0].ticks == {1: 100, 2: 0, 3: 7}, )
    want("...and a joint moved in pose 1 STAYS moved in pose 2",
         r.poses[1].ticks == {1: 100, 2: 200, 3: 7})
    want("...while a joint nobody pins holds the start value throughout",
         all(p.ticks[3] == 7 for p in r.poses))
    want("...and the unresolved clip really did differ (else this is "
         "testing nothing)",
         sparse.poses[1].ticks != r.poses[1].ticks)
    want("resolving an already-complete clip changes nothing",
         Clip("c", [a, b]).resolved({1: 0, 2: 0}).poses[1].ticks == b.ticks)

    # Guarded holds. The property that matters is that a hold survives
    # every transformation a clip goes through on its way to the arm —
    # a hold silently dropped by `resolved` would disarm the guard while
    # the routine still looked correct.
    guarded = Pose("sweep", {1: 100, 2: 200, 3: 300}, holds=(2, 3))
    want("a pose carries its holds", guarded.holds == (2, 3))
    gclip = Clip("g", [Pose("open", {1: 0, 2: 200, 3: 300}), guarded])
    want("...and RESOLVING preserves them (the transformation every "
         "clip goes through before it is run)",
         [p.holds for p in gclip.resolved({1: 0, 2: 0, 3: 0}).poses]
         == [(), (2, 3)])
    want("...while resolving still fills the ticks it always did",
         gclip.resolved({1: 0, 2: 0, 3: 0}).poses[1].ticks
         == {1: 100, 2: 200, 3: 300})
    try:
        Pose("bad", {1: 100}, holds=(2,))
        want("holding a joint the pose does not position is refused", False)
    except BenchError:
        want("holding a joint the pose does not position is refused", True)

    empty = Clip("empty", [])
    want("an empty clip samples to nothing", sample_clip(empty) == [])
    one = Clip("one", [a])
    want("a single-pose clip is just that pose", sample_clip(one) == [a.ticks])

    for bad, label in ((0, "zero speed"), (-1, "negative speed")):
        try:
            MotionProfile(speed=bad)
            want(f"{label} is refused", False)
        except BenchError:
            want(f"{label} is refused", True)
    try:
        MotionProfile(acceleration=0)
        want("zero acceleration is refused", False)
    except BenchError:
        want("zero acceleration is refused", True)

    print("clip selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    if fails:
        raise SystemExit(1)


if __name__ == "__main__":
    _selftest()
