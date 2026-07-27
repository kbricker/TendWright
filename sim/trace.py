"""Trace: what the arm ACTUALLY did, against what the sim said it would.

Plan #660's acceptance test. Everything else in the clip layer proves
the sim is self-consistent — that the gate, the viewer and the servo
commands all resolve to one definition. None of that is evidence about
the real arm. This is.

    # at the bench, during a run
    uv run python -m hardware.bench.exercise --trace run1.csv

    # afterwards, anywhere
    uv run python -m sim.trace run1.csv

WHY ALIGNMENT IS PER-PHASE, not by wall-clock from the start. The clip's
duration counts MOTION only; the real routine also waits for each joint
to settle before commanding the next move. Comparing on absolute time
would accumulate that settle overhead and report a growing "error" that
is really just the sim not modelling a pause. So each phase is aligned
at ITS OWN start, compared over the sim's predicted duration for that
phase, and whatever the arm spends settling afterwards is reported
separately as overhead rather than counted as deviation.

WHAT A DEVIATION MEANS. The sim plays the servo's COMMANDED trapezoid.
A real joint lags it under gravity and lags more under load, so some
deviation is expected and is not a defect — it is the size of the gap
between intent and plant. The number this prints is that gap. A
systematic early/late arrival across every joint points at the register
semantics (speed = ticks/s, acceleration = x100 ticks/s^2); a single
joint deviating points at that joint.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

from hardware.errors import BenchError
from hardware.units import span_deg

FIELDS = ("t", "phase", "edge", "j1", "j2", "j3", "j4", "j5", "j6")


class Trace:
    """Records (time, phase, edge index, joint positions) during a run.

    Deliberately dumb and append-only: it is a witness, not a
    participant. Every method swallows its own errors, because a
    recorder that can abort a moving arm is a hazard, not a diagnostic.
    """

    def __init__(self, path: Path | str, meta: dict | None = None):
        # A batch of runs is uninterpretable unless each file says what
        # produced it. The profile is written INTO the trace so the
        # comparison uses the run's own speed/accel rather than assuming
        # the defaults — comparing a --speed 0.5 run against the default
        # reference would look exactly like a broken arm.
        self.meta = dict(meta or {})
        self.path = Path(path)
        self._phase = "start"
        self._edge = 0
        self._t0: float | None = None
        self._rows: list[dict] = []
        self.error: str | None = None

    def phase(self, name: str, edge: int | None = None) -> None:
        self._phase = name
        if edge is not None:
            self._edge = edge
        else:
            self._edge += 1

    def sample(self, t: float, positions: dict[int, int]) -> None:
        try:
            if self._t0 is None:
                self._t0 = t
            row = {"t": round(t - self._t0, 4), "phase": self._phase,
                   "edge": self._edge}
            for i in range(1, 7):
                row[f"j{i}"] = positions.get(i, "")
            self._rows.append(row)
        except Exception as exc:                     # never abort a move
            self.error = str(exc)

    def close(self) -> Path | None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("w", newline="") as fh:
                if self.meta:
                    fh.write("#" + json.dumps(self.meta) + "\n")
                w = csv.DictWriter(fh, fieldnames=FIELDS)
                w.writeheader()
                w.writerows(self._rows)
            return self.path
        except OSError as exc:
            self.error = f"could not write {self.path}: {exc}"
            return None

    def __len__(self) -> int:
        return len(self._rows)


# ------------------------------------------------------------ compare
def load(path: Path) -> tuple[list[dict], dict]:
    """(samples, meta). meta carries the profile the run actually used."""
    meta: dict = {}
    try:
        text = Path(path).read_text().splitlines()
        while text and text[0].startswith("#"):
            try:
                meta.update(json.loads(text.pop(0)[1:]))
            except json.JSONDecodeError:
                pass
        rows = list(csv.DictReader(text))
    except OSError as exc:
        raise BenchError(f"could not read {path}: {exc}",
                         "run exercise with --trace FILE first") from exc
    if not rows:
        raise BenchError(f"{path} has no samples",
                         "the run may have aborted before any motion")
    out = []
    for r in rows:
        rec = {"t": float(r["t"]), "phase": r["phase"], "edge": int(r["edge"]),
               "pos": {i: int(r[f"j{i}"]) for i in range(1, 7)
                       if r.get(f"j{i}") not in (None, "")}}
        out.append(rec)
    return out, meta


ALIGN_TOL_TICKS = 60   # generous: settle tolerance is 25, plus sag


def build_reference(rows: list[dict], cals, span: int, meta: dict | None = None):
    """The clip the run was actually executing, INCLUDING its start pose.

    The start pose matters and getting it wrong is not subtle: the real
    routine begins with a wake -> rest move, so its first recorded phase
    is that move. A clip built without a start pose has no such edge, and
    every phase then lines up against the wrong one. That shifted the
    whole comparison by one edge and reported a good run as 164 deg out
    (2026-07-27) — the arm was fine; the tool was reading the wrong row.

    The start pose is recoverable from the trace itself: the first
    sample IS where the arm was when the run began.

    The PROFILE and SPAN come from the trace's own metadata when it has
    any, not from the defaults. Comparing a --speed 0.5 run against the
    default reference would show every phase running "slow" by exactly
    the factor the operator chose — indistinguishable from a broken arm.
    """
    from hardware.bench.exercise import (SPAN_MAX, SWEEP_ORDER,
                                         SWEEP_SPAN_CAPS, clamp_goal,
                                         exercise_clip, sweep_window)
    from sim.clip import MotionProfile
    from hardware.bench.exercise import ACCELERATION, SPEED_BASE

    meta = meta or {}
    span = int(meta.get("span", span))
    ids = meta.get("ids")
    sweep_ids = [i for i in SWEEP_ORDER
                 if i in cals and (ids is None or i in ids)]
    rest = {i: clamp_goal(cals[i], cals[i].rest) for i in sorted(cals)}
    windows = {i: sweep_window(cals[i],
                               min(span, SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
               for i in sweep_ids}
    profile = MotionProfile(speed=int(meta.get("speed", SPEED_BASE)),
                            acceleration=int(meta.get("accel", ACCELERATION)))
    return exercise_clip(cals, rest, windows, sweep_ids,
                         dict(rows[0]["pos"]), profile)


def check_alignment(by_edge: dict, edges: list) -> list[str]:
    """Does each recorded phase actually END where its edge says it should?

    An alignment error is silent and catastrophic — it produces a full
    table of confident, meaningless numbers. So it is CHECKED, not
    assumed: every phase's final sample must sit near its edge's target
    pose. This is the test that would have caught the off-by-one
    immediately instead of blaming the arm."""
    bad = []
    for idx, samples in sorted(by_edge.items()):
        if not (1 <= idx <= len(edges)):
            bad.append(f"phase {idx} has no matching edge")
            continue
        target = edges[idx - 1][1].ticks
        final = samples[-1]["pos"]
        worst = max((abs(final[i] - t) for i, t in target.items()
                     if i in final), default=0)
        if worst > ALIGN_TOL_TICKS:
            bad.append(f"phase {idx} ({samples[0]['phase']}) ended "
                       f"{worst} ticks from where edge {idx} targets")
    return bad


def compare(path: Path, span: int = 70) -> int:
    """Lay the recorded run over the sim's own prediction, per phase."""
    from sim.clip import edge_duration
    from sim.twin import Twin

    rows, meta = load(path)
    twin = Twin()
    clip = build_reference(rows, twin.cals, span, meta)
    edges = clip.edges()

    by_edge: dict[int, list[dict]] = {}
    for r in rows:
        by_edge.setdefault(r["edge"], []).append(r)

    print(f"{len(rows)} samples over {rows[-1]['t']:.1f} s, "
          f"{len(by_edge)} phases recorded; reference clip has "
          f"{len(edges)} edges")
    if meta:
        print(f"  run profile (from the trace): speed "
              f"{clip.profile.speed} ticks/s, acceleration "
              f"{clip.profile.acceleration}, span {meta.get('span', span)}%"
              + (f", ids {meta['ids']}" if meta.get("ids") else ""))
    else:
        print("  NOTE: this trace carries no profile metadata (recorded "
              "before that existed).\n  Comparing against the DEFAULTS — if "
              "the run used --speed/--span/--accel, the numbers\n  below "
              "will be wrong in exactly the way a broken arm looks.")

    misaligned = check_alignment(by_edge, edges)
    if len(misaligned) > max(1, len(by_edge) // 4):
        print("\nREFUSING TO COMPARE — the recorded run and the reference "
              "clip do not line up:")
        for m in misaligned[:6]:
            print(f"  {m}")
        if len(misaligned) > 6:
            print(f"  ... and {len(misaligned) - 6} more")
        print("\nEvery phase must end near its edge's target pose. When "
              "they do not, the\ncomparison would print a full table of "
              "confident, meaningless numbers.\nLikely causes: the run "
              "used --ids or a --span other than "
              f"{span}, or it was\naborted partway. Re-run the comparison "
              "with the same --span the arm used.")
        return 1
    if misaligned:
        print(f"  note: {len(misaligned)} phase(s) ended away from target "
              f"(within tolerance overall): {misaligned[0]}")
    print()
    print(f"{'phase':<22} {'real s':>7} {'sim s':>7} {'settle s':>8} "
          f"{'worst dev':>10}  joint")
    worst_overall = 0.0
    worst_where = ""
    for idx in sorted(by_edge):
        samples = by_edge[idx]
        if not (1 <= idx <= len(edges)):
            continue
        a, b = edges[idx - 1]
        predicted = edge_duration(clip.profile, a, b)
        t0 = samples[0]["t"]
        real = samples[-1]["t"] - t0
        # Evaluate the sim's position at each sample's EXACT time via the
        # profile itself, rather than snapping to the nearest pre-computed
        # frame. Snapping added its own error — during fast motion one
        # frame is many ticks, so a good run measured 4x worse than it
        # was. The comparison must not manufacture the deviation it
        # reports.
        deltas = {i: b.ticks.get(i, a.ticks[i]) - a.ticks.get(i, b.ticks[i])
                  for i in sorted(set(a.ticks) | set(b.ticks))}
        # Compare only over the sim's predicted window; anything after it
        # is the arm settling, which the clip does not model.
        worst, worst_j = 0.0, 0
        for s in samples:
            rel = s["t"] - t0
            if rel > predicted:
                break
            for i, actual in s["pos"].items():
                if i not in deltas:
                    continue
                expect = (a.ticks.get(i, b.ticks[i])
                          + clip.profile.travelled(deltas[i], rel))
                dev = abs(actual - expect)
                if dev > worst:
                    worst, worst_j = dev, i
        settle = max(0.0, real - predicted)
        deg = span_deg(worst)
        if deg > worst_overall:
            worst_overall, worst_where = deg, f"{samples[0]['phase']} j{worst_j}"
        print(f"{samples[0]['phase'][:22]:<22} {real:7.1f} {predicted:7.1f} "
              f"{settle:8.1f} {deg:9.1f}d  j{worst_j}")
    print()
    print(f"worst deviation from the sim: {worst_overall:.1f} deg "
          f"({worst_overall / span_deg(1):.0f} ticks) at {worst_where}")
    print("\nHow to read it:")
    print("  * a few degrees is EXPECTED — the sim plays the commanded")
    print("    trapezoid and a real joint lags it under gravity.")
    print("  * every joint late by a similar factor => the servo register")
    print("    semantics are off (speed ticks/s, accel x100 ticks/s^2).")
    print("  * ONE joint deviating => that joint: load, friction, or a")
    print("    calibration/anchoring problem specific to it.")
    print("  * 'settle s' is time the arm spent arriving that the clip")
    print("    does not model. Large values mean the profile is optimistic")
    print("    about how fast the plant actually converges.")
    return 0


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0
    try:
        return compare(Path(sys.argv[1]))
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
