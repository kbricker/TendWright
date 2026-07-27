"""Refuse a commanded move that the twin says will collide (plan #699).

The gate has existed since #648, and until now exactly two tools used it:
`exercise` and `batch`, both of which run a routine written in advance.
The interactive tool did not. `jog` could drive the arm into itself and
nothing would stop it — its only protection was each joint's calibrated
min/max, and a per-joint range CANNOT see a self-collision, because
whether the forearm hits the upper arm is a function of the WHOLE pose.
Two joints each comfortably inside their own limits still fold the arm
through itself; that is precisely the run-1 bench collision the twin was
built to predict.

What made it survivable is that Kyle jogs standing at the bench with a
hand near the power switch. **The human was the gate.** That is a real
mitigation and it is why this was never urgent — but it was also the
entire mitigation, and it was nowhere in writing.

WHY THIS IS A SEPARATE MODULE. Two reasons, both learned the hard way in
this repo. First, jog knows about ONE joint; the gate needs six, so the
adapter has to be told the whole pose and the caller has to go get it —
making that a parameter rather than a hidden bus read is what lets this
be tested at all. Second, #699 wants the same check on every tool that
can write a goal position, and a check copy-pasted into four tools is
four places to drift.

WHAT THIS DOES NOT DO. It knows only what the model knows, and the
model's world is the arm plus a ground plane: 13 collidable geoms and
the table surface. The bench, the fixtures, the object in the gripper
and the cable are all absent (#673 owns that). So a clean verdict means
"the arm will not hit ITSELF or the table", never "the move is safe".
Saying that out loud matters more here than in `exercise`, because an
interactive tool invites trusting the last thing it printed.

    uv run python -m hardware.bench.posegate selftest
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .bus import BenchError


@dataclass(frozen=True)
class Verdict:
    """The answer, plus enough to explain a refusal to a human."""

    allowed: bool
    detail: str                 # one line, always populated
    poses_checked: int = 0
    gated: bool = True          # False = nothing was actually checked

    @property
    def refused(self) -> bool:
        return not self.allowed


class PoseGate:
    """Twin-backed pre-check for a single commanded move.

    Construction never raises for a missing or unusable calibration —
    `jog` is explicitly documented to work before any calibration
    exists, and breaking that to add a safety check would be a poor
    trade. Instead the gate goes INACTIVE and says why, and every
    verdict it returns afterwards carries `gated=False` so the caller
    cannot mistake "nothing objected" for "it was checked". That is the
    same shape `StrainWatch` uses for implausible sensor readings: a
    guard that cannot do its job says so out loud rather than passing
    quietly.
    """

    def __init__(self, ids, cal_path: str | Path = "calibration.json",
                 profile=None):
        self.ids = sorted(ids)
        self.reason = ""
        self._twin = None
        self._profile = profile
        try:
            from sim.clip import DEFAULT_PROFILE
            from sim.twin import Twin
            if self._profile is None:
                self._profile = DEFAULT_PROFILE
            if not Path(cal_path).exists():
                self.reason = f"no calibration at {cal_path}"
                return
            twin = Twin(cal_path)
            missing = [i for i in self.ids if i not in twin.cals]
            if missing:
                self.reason = (f"joint(s) {missing} are not calibrated, so "
                               f"the model cannot be posed")
                return
            self._twin = twin
        except BenchError as exc:
            self.reason = str(exc)
        except Exception as exc:                     # model missing, import
            self.reason = f"{type(exc).__name__}: {exc}"

    @property
    def active(self) -> bool:
        return self._twin is not None

    def banner(self) -> str:
        """What to print at startup, so the operator knows which it is."""
        if self.active:
            return ("collision gate ACTIVE — moves that would hit the arm "
                    "or the table are refused (the bench and anything on "
                    "it are NOT modelled)")
        return (f"collision gate INACTIVE — {self.reason}. "
                f"Moves are NOT checked for self-collision.")

    def check(self, current: dict[int, int],
              target: dict[int, int]) -> Verdict:
        """Would moving from `current` to `target` collide?

        Both are WHOLE-ARM poses in calibrated ticks. `target` may name
        only the joints that move; the rest are carried from `current`.
        The path is sampled on the real motion profile, not just the two
        endpoints — a step small enough to look safe at both ends can
        still sweep something through something else, which is the
        entire reason the clip layer exists.
        """
        if not self.active:
            return Verdict(True, f"not checked ({self.reason})", 0, False)

        missing = [i for i in self.ids if i not in current]
        if missing:
            # Refusing here rather than guessing: a pose we cannot see is
            # a pose we cannot judge, and defaulting the unknown joints
            # would gate a DIFFERENT arm than the one on the bench.
            return Verdict(False, f"cannot check — no position for joint(s) "
                                  f"{missing}", 0, True)

        if dict(current) == {**current, **target}:
            return Verdict(True, "no movement", 0, True)
        return self.check_sequence([dict(current), {**current, **target}],
                                   label="jog-step")

    def check_sequence(self, poses: list[dict[int, int]],
                       label: str = "sequence") -> Verdict:
        """Gate a whole walk through poses, not just one step.

        `teach replay` knows its entire trajectory in advance, so it can
        be told before it commits rather than partway through — which is
        also the only way to catch the APPROACH to the first frame, a
        move nobody recorded and the one most likely to surprise.
        """
        if not self.active:
            return Verdict(True, f"not checked ({self.reason})", 0, False)
        if len(poses) < 2:
            return Verdict(True, "no movement", 0, True)
        missing = sorted({i for i in self.ids if i not in poses[0]})
        if missing:
            return Verdict(False, f"cannot check — no position for joint(s) "
                                  f"{missing}", 0, True)

        from sim.clip import Clip, Pose

        report = self._twin.check_clip(Clip(
            label, [Pose(f"p{n}", dict(p)) for n, p in enumerate(poses)],
            self._profile))
        if report.clean:
            return Verdict(True, f"clear ({report.poses_checked} poses)",
                           report.poses_checked, True)
        a, b = poses[0], poses[-1]

        # The WORST contact, not the first one found. Order of discovery
        # is an artefact of geom numbering, and reporting it made the
        # gate name a 0.00 mm table graze while a 0.18 mm arm-through-arm
        # fold went unmentioned in the same refusal.
        c = max(report.contacts, key=lambda k: k.depth_mm)
        moved = sorted(i for i in b if b[i] != a.get(i))
        # Hitting the table and folding through itself are different
        # problems with different fixes, so the message says which.
        kind = ("table contact" if "table" in (c.body_a, c.body_b)
                else "SELF-COLLISION")
        depth = (f"{c.depth_mm:.2f} mm deep" if c.depth_mm >= 0.005
                 else "touching")
        others = len({frozenset((k.body_a, k.body_b))
                      for k in report.contacts}) - 1
        return Verdict(
            False,
            f"REFUSED — {kind}: {c.body_a} <-> {c.body_b}, {depth}"
            + (f" (+{others} other pair(s))" if others > 0 else "")
            + f" at step {c.step}"
            + (f"; joint {moved[0] if len(moved) == 1 else moved} moving"
               if moved else "")
            + f", {report.poses_checked} poses checked",
            report.poses_checked, True)


# --------------------------------------------------------------------


def selftest() -> int:
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not cond:
            fails.append(name)

    import json
    import tempfile

    cal_path = Path("calibration.json")
    have_cal = cal_path.exists()

    print("inactive-gate behaviour (the path that must never pass silently)")
    with tempfile.TemporaryDirectory() as td:
        g = PoseGate([1, 2, 3], Path(td) / "nope.json")
        check("missing calibration leaves the gate inactive", not g.active,
              g.reason)
        v = g.check({1: 2000}, {1: 2100})
        check("...and its verdicts are allowed but NOT gated",
              v.allowed and not v.gated, v.detail)
        check("...and the banner says so out loud",
              "INACTIVE" in g.banner() and "NOT checked" in g.banner())
        # A calibration that exists but omits a joint the caller asked for.
        if have_cal:
            doc = json.loads(cal_path.read_text())
            doc["joints"] = [j for j in doc["joints"] if j["id"] != 4]
            p = Path(td) / "partial.json"
            p.write_text(json.dumps(doc))
            g2 = PoseGate([1, 2, 3, 4, 5, 6], p)
            check("a partly-calibrated arm leaves the gate inactive",
                  not g2.active, g2.reason)

    if not have_cal:
        print("\n  (no calibration.json here — skipping the live-gate cases)")
        print(f"\n{'FAILED' if fails else 'posegate OK'}")
        return 1 if fails else 0

    print("\nactive gate")
    ids = [1, 2, 3, 4, 5, 6]
    g = PoseGate(ids, cal_path)
    check("builds against the real calibration", g.active, g.reason)
    check("banner names the limits of what it checked",
          "NOT modelled" in g.banner())

    from sim.twin import Twin
    rest = {i: c.rest for i, c in Twin(cal_path).cals.items()}

    v = g.check(rest, {})
    check("a no-op move is allowed", v.allowed and v.poses_checked == 0,
          v.detail)

    v = g.check(rest, {1: rest[1] + 40})
    check("a small pan step from rest is allowed", v.allowed, v.detail)
    check("...and it really did check a path, not just two endpoints",
          v.poses_checked > 2, f"{v.poses_checked} poses")

    print("\nit must REFUSE — the property the whole module exists for")
    # The run-1 bench collision: elbow folded, m2 swept across. This is a
    # real collision the arm actually had, and both joints stay inside
    # their calibrated ranges the whole way — which is exactly why
    # per-joint soft limits could never have caught it.
    from hardware.bench.exercise import sweep_window
    cals = Twin(cal_path).cals
    lo2, hi2 = sweep_window(cals[2], 70)
    v = g.check(rest, {2: hi2})
    check("refuses the run-1 folded-elbow sweep", v.refused, v.detail)
    check("...names both offending links", v.refused and "<->" in v.detail)
    inside = cals[2].min <= hi2 <= cals[2].max
    check("...with joint 2 INSIDE its calibrated range the whole way",
          inside, f"{cals[2].min} <= {hi2} <= {cals[2].max} — a per-joint "
                  f"soft limit could never have caught this")

    # Self-collision and table contact are reported differently on
    # purpose. Assert BOTH labels actually occur, so the distinction is
    # not decorative — an earlier revision reported whichever contact was
    # found first, and named a 0.00 mm table graze while an arm-through-
    # arm fold in the same move went unmentioned.
    check("reports SELF-COLLISION where the arm folds through itself",
          "SELF-COLLISION" in v.detail, v.detail)
    # Reaching down and out with the elbow open puts the jaw on the table
    # WITHOUT folding the arm into itself — the case where the worst
    # contact really is the table. Found by sweeping j2 x j3; sweeping j2
    # alone never produces it, because every table touch from rest comes
    # with a deeper self-collision alongside it.
    reach = g.check(rest, {2: 1795, 3: 2732})
    check("reports table contact separately where it hits the table",
          reach.refused and "table contact" in reach.detail, reach.detail)

    print("\nunknown joints are refused, not guessed")
    v = g.check({1: rest[1], 2: rest[2]}, {1: rest[1] + 40})
    check("a pose missing joints is REFUSED", v.refused and v.gated, v.detail)

    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("posegate OK")
    return 0


def main() -> int:
    import sys
    if len(sys.argv) > 1 and sys.argv[1] != "selftest":
        print(f"unknown command {sys.argv[1]!r}; use selftest")
        return 2
    return selftest()


if __name__ == "__main__":
    raise SystemExit(main())
