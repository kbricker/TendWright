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
from hardware.units import DegFrame, span_deg

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
    """A named whole-arm pose, in calibrated ticks."""

    name: str
    ticks: dict[int, int]

    def merged(self, **override: int) -> 'Pose':
        return Pose(self.name, {**self.ticks, **override})


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
    out: dict[str, Pose] = {}
    for name, body in (doc.get("poses") or {}).items():
        ticks: dict[int, int] = {}
        for key, value in body.items():
            try:
                i = int(key)
            except ValueError:
                raise BenchError(
                    f"pose '{name}' has non-numeric joint key {key!r}",
                    "joint keys are servo ids, e.g. \"3\"") from None
            cal = cals.get(i)
            if cal is None:
                raise BenchError(
                    f"pose '{name}' names joint {i}, which is not in "
                    f"calibration.json",
                    "calibrate capture the joint, or drop it from the pose")
            tick = (cal.frame.tick(float(value))
                    if isinstance(cal.frame, DegFrame)
                    else cal.frame.tick(float(value)))
            if not cal.min <= tick <= cal.max:
                raise BenchError(
                    f"pose '{name}' puts joint {i} ({cal.name}) at "
                    f"{value} -> tick {tick}, outside its calibrated "
                    f"range {cal.min}..{cal.max}",
                    "a pose outside the calibrated range is unreachable; "
                    "re-author it or re-calibrate the joint")
            ticks[i] = tick
        out[name] = Pose(name, ticks)
    return out


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
