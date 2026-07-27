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

    def __init__(self, path: Path | str):
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
def load(path: Path) -> list[dict]:
    try:
        with Path(path).open(newline="") as fh:
            rows = list(csv.DictReader(fh))
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
    return out


def compare(path: Path, span: int = 70) -> int:
    """Lay the recorded run over the sim's own frames, per phase."""
    from sim.clip import edge_duration
    from sim.twin import Twin, exercise_clip_for

    rows = load(path)
    twin = Twin()
    clip = exercise_clip_for(twin.cals, span)
    edges = clip.edges()

    by_edge: dict[int, list[dict]] = {}
    for r in rows:
        by_edge.setdefault(r["edge"], []).append(r)

    print(f"{len(rows)} samples over {rows[-1]['t']:.1f} s, "
          f"{len(by_edge)} phases recorded; clip has {len(edges)} edges")
    if len(by_edge) != len(edges):
        print("  NOTE: phase count differs from the clip's edge count — the "
              "run may have been aborted, or --ids/--span differed from the "
              "defaults compared here. Per-phase numbers below are still "
              "valid; the totals are not comparable.")
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
