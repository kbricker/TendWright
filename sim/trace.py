"""Trace: what the arm ACTUALLY did, against what the sim said it would.

Plan #660's acceptance test. Everything else in the clip layer proves
the sim is self-consistent — that the gate, the viewer and the servo
commands all resolve to one definition. None of that is evidence about
the real arm. This is.

    # at the bench, during a run
    uv run python -m hardware.bench.exercise --trace run1.csv
    uv run python -m hardware.bench.runner run --clip crane-tour.json \
        --trace runs/

    # afterwards, on either machine — FROM THE REPO ROOT. The trace file
    # can live anywhere; the shell cannot (`package = false`, so
    # `python -m sim.trace` needs the repo on sys.path).
    uv run python -m sim.trace runs/run1.csv
    uv run python -m sim.trace runs/run1.csv --clip crane-tour.json
    uv run python -m sim.trace selftest        # no hardware, no trace

WHICH REFERENCE IT COMPARES AGAINST — the run's own, not a guess. A
trace written by `runner` carries the name of the clip it played; one
written by `exercise` carries that routine's span and joint set instead.
So the reference is chosen by what the file says: a trace naming a clip
is compared against THAT CLIP, loaded from its file, and one naming none
is compared against the exercise routine rebuilt from its span/ids.

This is the second half of a fix, and the first half is why the
distinction is written down here. Until 2026-07-30 the reference was
ALWAYS the exercise routine, so the crane tour was compared against a
completely different motion. It did not print nonsense — the alignment
check below refused — but the refusal blamed `--span` and `--ids` for a
run that used neither, and the tool had the answer in its own header the
whole time. **A clip named in the metadata and not found is a refusal,
never a fall back to exercise:** falling back is what turned "you are
holding the wrong reference" into "your arm did something strange".

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
from pathlib import Path, PureWindowsPath

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
            # utf-8 explicitly: the bench Pi's locale is utf-8 and the
            # desk's is cp1252, and a trace is written on one machine to
            # be read on the other. Without this the clip NAME — which
            # now selects a file — round-trips through two different
            # codecs.
            with self.path.open("w", newline="", encoding="utf-8") as fh:
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
    """(samples, meta). meta carries the profile the run actually used.

    A DAMAGED HEADER IS A REFUSAL, NOT AN EMPTY ONE. This used to be
    `except json.JSONDecodeError: pass`, which was harmless while the
    metadata only tuned the profile — but the header now names the CLIP,
    so `meta = {}` no longer means "an old trace", it means "compare
    against the exercise routine instead". One lost byte in the header of
    a crane-tour trace and the tool reported `reference 'exercise'`, said
    the trace carried no metadata (with the metadata visible on line 1),
    and told the operator to re-run with a different `--span`. That is
    precisely the failure this module was rewritten to remove, rebuilt
    out of a `pass`.

    Read as utf-8-sig: a trace opened and saved by Notepad or Excel picks
    up a BOM, which defeats the `#` test and hands the header line to the
    CSV reader as its column names."""
    meta: dict = {}
    headers = 0
    try:
        text = Path(path).read_text(encoding="utf-8-sig").splitlines()
        while text and text[0].startswith("#"):
            headers += 1
            line = text.pop(0)[1:]
            try:
                doc = json.loads(line)
            except json.JSONDecodeError as exc:
                raise BenchError(
                    f"{path}: the metadata header is not valid JSON ({exc})",
                    "the first line is a `#` followed by a JSON object "
                    "written by the run. Repair it, or pass --clip PATH. "
                    "Do NOT just delete the line: it names the clip that "
                    "ran, and without it this compares against the "
                    "exercise routine and blames --span") from None
            # `dict.update` also accepts a sequence of pairs, so a header
            # of [["clip","x"]] would quietly become {"clip": "x"} — a
            # second, undocumented syntax for the one field that steers
            # which file gets opened.
            if not isinstance(doc, dict):
                raise BenchError(
                    f"{path}: the metadata header is {type(doc).__name__}, "
                    f"not an object",
                    'expected e.g. #{"speed": 250, "clip": "crane-tour"}')
            meta.update(doc)
        rows = list(csv.DictReader(text))
    except OSError as exc:
        raise BenchError(f"could not read {path}: {exc}",
                         "run exercise with --trace FILE first") from exc
    if not rows:
        raise BenchError(f"{path} has no samples",
                         "the run may have aborted before any motion")
    out = []
    # Counted from the header lines actually consumed, so the number
    # names the line the operator will find. Inferring it from `meta`
    # being non-empty was close but not the same question: `#{}` is a
    # header that parses to nothing.
    first_data_line = headers + 2
    for n, r in enumerate(rows, start=first_data_line):
        try:
            rec = {"t": float(r["t"]), "phase": r["phase"],
                   "edge": int(r["edge"]),
                   "pos": {i: int(r[f"j{i}"]) for i in range(1, 7)
                           if r.get(f"j{i}") not in (None, "")}}
        except (KeyError, TypeError, ValueError) as exc:
            raise BenchError(f"{path}: row {n} is malformed ({exc})",
                             f"expected columns {', '.join(FIELDS)}"
                             ) from None
        out.append(rec)
    return out, meta


ALIGN_TOL_TICKS = 60   # generous: settle tolerance is 25, plus sag

REPO_ROOT = Path(__file__).resolve().parent.parent


def _int(meta: dict, key: str, default: int) -> int:
    """A whole number out of the trace's header, or a clear refusal.

    `load` refuses a header it cannot parse at all, so a header that
    parses cleanly but carries `"speed": null` must not be the one thing
    that reaches the operator as a bare TypeError traceback."""
    raw = meta.get(key, default)
    if raw is None:
        return default
    if isinstance(raw, bool) or not isinstance(raw, (int, float, str)):
        raise BenchError(
            f"the trace's {key} is {raw!r}, which is not a number",
            "the header is the `#` JSON line at the top of the CSV")
    try:
        return int(raw)
    except (TypeError, ValueError):
        raise BenchError(
            f"the trace's {key} is {raw!r}, which is not a number",
            "the header is the `#` JSON line at the top of the CSV"
        ) from None


def find_clip_file(name: str, near: Path | None = None) -> Path:
    """Locate the clip file a trace names, or refuse saying where it looked.

    A FALLBACK, not the primary route. Traces written from 2026-07-30
    carry `clip_file`, the path the clip was actually loaded from, and
    that is what `clip_reference` prefers — because the NAME and the
    FILE are not the same string and cannot be made so. `runner example`
    emits a clip named `pan-wiggle` that the docs have the operator save
    as `pan.json`; a clip may also live in a subdirectory. Guessing
    `<name>.json` gets both wrong. It is kept because traces recorded
    before that field existed have nothing else to go on, and for those
    it usually works.

    Treated as a filename and nothing else. A trace is data — it can be
    copied off the bench machine, edited, or written by an older build —
    so a `clip` field containing a path separator is refused rather than
    followed out of the search directories."""
    if not isinstance(name, str) or not name or name.startswith("."):
        raise BenchError(
            f"the trace names clip {name!r}, which is not a plain clip name",
            "pass the file explicitly with --clip PATH")
    # PureWindowsPath catches a backslash on Linux too: a trace written
    # on the Pi and one written on the desk are the same data, and a
    # separator that is inert on one platform must not be followed on
    # the other just because it was read there.
    if Path(name).name != name or PureWindowsPath(name).name != name:
        raise BenchError(
            f"the trace names clip {name!r}, which is not a plain clip name",
            "pass the file explicitly with --clip PATH")
    tried: list[Path] = []
    # The trace's own directory first: runs and the clips that produced
    # them get archived together, and that copy is the right one.
    roots = [near] if near is not None else []
    roots += [Path.cwd(), REPO_ROOT]
    for d in dict.fromkeys(roots):
        p = d / f"{name}.json"
        if p.exists():
            return p
        tried.append(p)
    raise BenchError(
        f"the trace was recorded playing clip '{name}', and no file for "
        f"it was found",
        "looked in " + ", ".join(str(t) for t in tried)
        + " — pass it with --clip PATH")


def _resolve_recorded(recorded: str, near: Path | None) -> Path:
    """The clip file the run recorded, found here — or a refusal.

    The recorded string is whatever the operator typed after `--clip`,
    so it is usually relative to a directory that is not this one. Try it
    as given, then by its filename BESIDE THE TRACE — a run and the clip
    that produced it get archived together, and that copy is the right
    one.

    Deliberately no wider than that. Searching the CWD or the repo root
    by filename would resolve a recorded `clips/crane-tour.json` onto the
    repo's own `crane-tour.json`, which is a different file that happens
    to share a name — the same silent substitution the caller refuses to
    make from the clip's NAME, arrived at one step later."""
    p = Path(recorded)
    if p.exists():
        return p
    if near is not None:
        cand = near / p.name
        if cand.exists():
            return cand
    raise BenchError(
        f"the run recorded its clip as {recorded!r}, and that file is not "
        f"here",
        f"the path is relative to wherever the run was launched. Pass the "
        f"file with --clip PATH — do NOT rely on a same-named clip in this "
        f"directory being the one that ran")


def _find_calibration(recorded) -> tuple[Path, str]:
    """The calibration the run used — (path, a NOTE to print, or "").

    The trace records a PATH, and `runner` records the default `--cal`
    value verbatim, so it is normally the CWD-relative "calibration.json".
    That resolves differently depending on where the operator is standing,
    so the repo root is searched as well. Deliberately NOT the trace's own
    directory, unlike `find_clip_file`: a clip is archived beside its run,
    a calibration is not.

    WHAT NO PATH CAN CATCH: `calibrate capture` rewrites calibration.json
    IN PLACE. Comparing an older trace after a re-capture reads the NEW
    frames under the OLD name and nothing here can tell — the file
    exists, and it is even the right file. Every reference pose is
    authored in degrees and converted through those frames, so the whole
    table shifts and it looks like a changed arm. Distrust comparisons
    of traces recorded before a re-calibration; closing it properly needs
    a fingerprint written into the trace, which is a follow-up.
    """
    default = REPO_ROOT / "calibration.json"
    if not isinstance(recorded, str) or not recorded:
        return default, ""
    p = Path(recorded)
    if p.exists():
        return p, ""
    here = REPO_ROOT / p.name
    if here.exists():
        return here, (f"  NOTE: the run recorded calibration {recorded!r}, "
                      f"which is not at that path from here.\n  Using "
                      f"{here} — if it is not the same file, every number "
                      f"below is shifted.")
    return default, (f"  NOTE: the run recorded calibration {recorded!r}, "
                     f"which is not here.\n  Falling back to {default}.")


def clip_reference(cals, meta: dict, clip_path: Path | str | None = None,
                   near: Path | None = None):
    """The clip named in the trace, at the profile the trace recorded.

    WHERE THE FILE COMES FROM, in falling order of how much it can be
    trusted: an explicit `--clip`, then `clip_file` from the trace (the
    path the run actually loaded), then `<name>.json` guessed from the
    clip's name. Only the first two are reliable — see `find_clip_file`.

    TWO SOURCES, AND THE TRACE WINS ON THE PROFILE. The file supplies the
    POSES; the trace supplies the speed/acceleration the servos were
    actually given. They are normally the same numbers — `runner` writes
    the clip's own profile into the trace — and when they are not, the
    file has been edited since the run. That is worth saying out loud:
    the endpoints may have moved too, and the alignment check only
    catches it if they moved far enough.
    """
    from sim.clip import Clip, MotionProfile, load_clip

    name = meta.get("clip")
    recorded = meta.get("clip_file")
    has_record = isinstance(recorded, str) and bool(recorded)
    guessed = False
    if clip_path is not None:
        path = Path(clip_path)
    elif has_record:
        # A RECORDED PATH THAT WILL NOT RESOLVE IS A REFUSAL, not a
        # licence to guess. The recorded path is CWD-relative — `runner`
        # writes `str(args.clip)` verbatim — so `--clip clips/tour.json`
        # run from one directory does not resolve from another, and a
        # trace scp'd off the Pi rarely resolves at all. Falling through
        # to `<name>.json` there was actively dangerous: any working copy
        # of a clip keeps the original `name`, so the guess landed on the
        # REPO's copy and the name-agreement check compared the guess
        # against itself and passed. The tool then reported the
        # difference between two different clips as the ARM's 27-degree
        # deviation. Refusing costs the operator one `--clip`; guessing
        # cost a wrong answer stated confidently.
        path = _resolve_recorded(recorded, near)
    else:
        path = find_clip_file(name, near)
        guessed = True
    clip = load_clip(cals, path)
    # CHECKED ON EVERY ROUTE THE OPERATOR DID NOT TYPE. `clip_file` is a
    # path out of a data file, and a path out of a data file can point
    # anywhere — a trace copied between machines carries a path that
    # means something different here, and a corrupt or hand-edited one
    # can name any JSON on disk. Requiring the loaded clip to still call
    # itself what the trace says it played is the cheap half of the
    # answer; printing the resolved path (below) is the other half.
    #
    # Only an explicit --clip skips it. That IS the operator saying
    # "this file, I know what I am doing" — comparing a run against an
    # edited or renamed copy is a legitimate thing to want, and there
    # would otherwise be no way to do it.
    if clip_path is None and name is not None and clip.name != name:
        raise BenchError(
            f"the trace was recorded playing clip '{name}', but "
            f"{path} calls itself '{clip.name}'",
            "the file was renamed, its `name` changed, or the trace's "
            "recorded path means something else on this machine; pass "
            "the right file with --clip PATH")
    # ALWAYS PRINTED, never only on trouble. Which file supplied the
    # reference is the single most load-bearing fact in the output, and
    # every way of getting it wrong — a stale copy in the working
    # directory, a recorded path that means something else on this
    # machine, a name that matched the wrong file — is invisible unless
    # the answer is on screen. Silence reads as "checked".
    print(f"  clip file: {path.resolve()}"
          + ("  (GUESSED from the name — this trace predates `clip_file`; "
             "pass --clip if wrong)" if guessed else ""))
    profile = MotionProfile(speed=_int(meta, "speed", clip.profile.speed),
                            acceleration=_int(meta, "accel",
                                              clip.profile.acceleration))
    # Only meaningful when the trace named THIS clip AND we are holding
    # the file it actually ran. With --clip on an exercise trace,
    # meta["speed"] is that run's --speed and the file was never its
    # source; on a GUESSED file the mismatch is evidence we opened the
    # wrong file, not that anyone edited this one. Both would be
    # confident statements about the wrong thing.
    if (meta.get("clip") is not None and not guessed
            and (profile.speed, profile.acceleration)
            != (clip.profile.speed, clip.profile.acceleration)):
        print(f"  NOTE: {path.name} now says speed {clip.profile.speed} / "
              f"acceleration {clip.profile.acceleration}, but the run used "
              f"{profile.speed} / {profile.acceleration}.\n  The file has "
              f"been edited since — comparing against the RUN's profile. "
              f"Its poses may have moved too.")
    return Clip(clip.name, clip.poses, profile)


def build_reference(rows: list[dict], cals, span: int, meta: dict | None = None,
                    clip_path: Path | str | None = None,
                    near: Path | None = None):
    """The motion the run was actually executing.

    A trace naming a clip is compared against that clip; one naming none
    is the exercise routine, rebuilt below. See the module docstring for
    why a named clip that cannot be found is refused rather than quietly
    replaced by the exercise routine.
    """
    meta = meta or {}
    if clip_path is not None or meta.get("clip") is not None:
        return clip_reference(cals, meta, clip_path, near)
    return exercise_reference(rows, cals, span, meta)


def exercise_reference(rows: list[dict], cals, span: int, meta: dict):
    """The exercise routine, INCLUDING its start pose.

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
    span = _int(meta, "span", span)
    ids = meta.get("ids")
    if ids is not None and not isinstance(ids, list):
        raise BenchError(f"the trace's ids field is {ids!r}, not a list",
                         "the header is the `#` JSON line at the top of "
                         "the CSV")
    sweep_ids = [i for i in SWEEP_ORDER
                 if i in cals and (ids is None or i in ids)]
    rest = {i: clamp_goal(cals[i], cals[i].rest) for i in sorted(cals)}
    windows = {i: sweep_window(cals[i],
                               min(span, SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
               for i in sweep_ids}
    profile = MotionProfile(speed=_int(meta, "speed", SPEED_BASE),
                            acceleration=_int(meta, "accel", ACCELERATION))
    return exercise_clip(cals, rest, windows, sweep_ids,
                         dict(rows[0]["pos"]), profile)


def phase_edges(clip, start: dict[int, int]) -> dict[int, tuple]:
    """`run_clip`'s OWN phase numbering, reproduced exactly.

    The executor stamps every sample with a phase index, and it is not
    "the nth edge" — it is:

        0     the approach: wherever the arm was, to the clip's pose 0
        1..N  the clip's own edges, in order

    Recovering that mapping by inspection would be a guess, and the
    comparison is precisely the tool that must not guess. So it is
    reproduced here from the same rule `run_clip` follows, and phase 0
    is a real edge with a real target rather than a row to skip.

    The exercise routine goes through the same executor and obeys the
    same rule. Its phase 0 is usually a hair of drift rather than a real
    move — `exercise_clip` is built from a read taken inside
    `build_and_gate`, and `run_clip` re-reads after a MuJoCo gate pass, a
    confirm prompt and six torque-on round trips, so the two readings
    differ by a tick or two and the approach IS emitted. An earlier
    version of this comment claimed exercise traces never carry a phase 0
    at all, which is wrong and was worth getting right: it is the routine
    there is no trace on hand to test against, so the comment is the only
    thing standing in for the evidence.

    EACH EDGE CARRIES THE PROFILE IT WAS COMMANDED WITH, which is why
    this returns a triple rather than the pose pair. The approach is
    deliberately slower than the clip — `run_clip` sends it at
    APPROACH_SPEED_TICKS, keeping the clip's acceleration — because it
    starts from wherever the arm happens to be and nothing has gated
    that first move from that exact position. Comparing it against the
    clip's speed predicts roughly half the duration it really takes and
    charges the difference to the arm as deviation."""
    from sim.clip import MotionProfile, Pose

    edges = {n: (a, b, clip.profile)
             for n, (a, b) in enumerate(clip.edges(), start=1)}
    if clip.poses:
        from hardware.bench.runner import APPROACH_SPEED_TICKS
        approach = MotionProfile(speed=APPROACH_SPEED_TICKS,
                                 acceleration=clip.profile.acceleration)
        # A PARTIAL FIRST SAMPLE IS REFUSED, not filled in. `load` drops
        # any joint whose column is blank, and an incomplete start pose
        # used to reach `edge_duration` — whose `.get(i, a.ticks[i])`
        # default is evaluated eagerly — as a bare KeyError.
        #
        # The obvious repair, treating an unread joint as already AT the
        # target, is worse than the crash. It asserts zero travel, so if
        # the unread joint is the one that moved, the approach's
        # predicted duration collapses to 0.0, the comparison window is
        # empty, and seconds of real motion get booked as "settle" with
        # nothing refusing. Nothing in this repo can produce such a
        # sample anyway — `wait_settle`'s sink always writes every joint
        # it targets, and a failed read raises rather than omitting a
        # column — so this only ever fires on a hand-edited or foreign
        # trace, where a clear refusal is exactly what is wanted.
        first = clip.poses[0]
        missing = sorted(set(first.ticks) - set(start))
        if missing:
            raise BenchError(
                f"the trace's first sample records no position for joint(s) "
                f"{missing}, which the clip's first pose ('{first.name}') "
                f"sets",
                "the approach cannot be measured without knowing where the "
                "arm started; the trace is incomplete")
        edges[0] = (Pose("start", {i: start[i] for i in first.ticks}),
                    first, approach)
    return edges


def check_alignment(by_edge: dict, edges: dict) -> tuple[list[str], int]:
    """Does each recorded phase actually END where its edge says it should?

    An alignment error is silent and catastrophic — it produces a full
    table of confident, meaningless numbers. So it is CHECKED, not
    assumed: every phase's final sample must sit near its edge's target
    pose. This is the test that would have caught the off-by-one
    immediately instead of blaming the arm."""
    bad, structural = [], []
    for idx, samples in sorted(by_edge.items()):
        pair = edges.get(idx)
        if pair is None:
            structural.append(f"phase {idx} has no matching edge")
            continue
        target = pair[1].ticks
        final = samples[-1]["pos"]
        shared = [abs(final[i] - t) for i, t in target.items() if i in final]
        # NO SHARED JOINTS IS NOT PERFECT ALIGNMENT. `max(..., default=0)`
        # scored an empty intersection as zero ticks out, so a trace whose
        # position columns were all blank passed every phase and the
        # refusal gate never fired — it went on to crash inside the table
        # instead. An unanswerable question must not return the reassuring
        # answer.
        if not shared:
            structural.append(f"phase {idx} ({samples[0]['phase']}) records "
                              f"no joint that edge {idx} targets")
            continue
        if max(shared) > ALIGN_TOL_TICKS:
            bad.append(f"phase {idx} ({samples[0]['phase']}) ended "
                       f"{max(shared)} ticks from where edge {idx} targets")
    # Structural mismatches first, and see `compare`: they are never
    # spent against the tolerance budget. "Ended 61 ticks out" is a
    # question about the arm; "has no matching edge" means the trace and
    # the reference are not the same routine, and no proportion of that
    # is acceptable.
    return structural + bad, len(structural)


def compare(path: Path, span: int = 70,
            clip_path: Path | str | None = None) -> int:
    """Lay the recorded run over the sim's own prediction, per phase."""
    from sim.clip import edge_duration
    from sim.twin import Twin

    rows, meta = load(path)
    cal, cal_note = _find_calibration(meta.get("cal"))
    # Resolved and REPORTED before the Twin is built, because building it
    # is what fails. An earlier version picked the calibration with
    # `Twin(cal_path=cal) if Path(cal).exists() else Twin()` and called
    # the second branch a fallback — but `Twin`'s own default is the same
    # CWD-relative "calibration.json", so it was the identical lookup,
    # and it raised before the NOTE explaining it could print. Comparing
    # a trace from `runs/` then died complaining the calibration file was
    # corrupt. "Afterwards, anywhere" has to mean it.
    print(f"  calibration: {cal.resolve()}")
    if cal_note:
        print(cal_note)
    twin = Twin(cal_path=str(cal))
    clip = build_reference(rows, twin.cals, span, meta, clip_path,
                           near=Path(path).resolve().parent)

    by_edge: dict[int, list[dict]] = {}
    for r in rows:
        by_edge.setdefault(r["edge"], []).append(r)
    edges = phase_edges(clip, rows[0]["pos"])
    print(f"{len(rows)} samples over {rows[-1]['t']:.1f} s, "
          f"{len(by_edge)} phases recorded; reference '{clip.name}' has "
          f"{len(clip.edges())} edges")
    if meta:
        print(f"  run profile (from the trace): speed "
              f"{clip.profile.speed} ticks/s, acceleration "
              f"{clip.profile.acceleration}"
              + (f", span {meta['span']}%" if meta.get("span") else "")
              + (f", ids {meta['ids']}" if meta.get("ids") else ""))
    else:
        print("  NOTE: this trace carries no profile metadata (recorded "
              "before that existed).\n  Comparing against the DEFAULTS — if "
              "the run used --speed/--span/--accel, the numbers\n  below "
              "will be wrong in exactly the way a broken arm looks.")

    misaligned, structural = check_alignment(by_edge, edges)
    if structural or len(misaligned) > max(1, len(by_edge) // 4):
        print("\nREFUSING TO COMPARE — the recorded run and the reference "
              "clip do not line up:")
        for m in misaligned[:6]:
            print(f"  {m}")
        if len(misaligned) > 6:
            print(f"  ... and {len(misaligned) - 6} more")
        print("\nEvery phase must end near its edge's target pose. When "
              "they do not, the\ncomparison would print a full table of "
              "confident, meaningless numbers.")
        # Keyed on what the TRACE says it played, not on whether --clip
        # was passed. Passing --clip at an exercise trace is an easy
        # paste — `runner` prints a --clip line after every traced run —
        # and the old wording sent the operator to `git checkout` a file
        # that had nothing to do with their run.
        if meta.get("clip") is not None:
            print(f"Likely cause: '{clip.name}' has been EDITED since this "
                  f"run — the poses it\nnames are not the poses the arm "
                  f"was given. Check it out at the run's\ncommit, or point "
                  f"--clip at the version that ran.")
        elif clip_path is not None:
            print(f"This trace names no clip — it is an `exercise` run, and "
                  f"--clip pointed it\nat {Path(clip_path).name}, which is "
                  f"not what it played. Drop --clip.")
        else:
            # NOT "re-run with the right --span": the span comes from the
            # trace's own header and always wins over the flag, so that
            # advice could not work and the number quoted was the CLI's,
            # not the run's. An operator who followed it got a
            # byte-identical refusal quoting their own input back.
            used = meta.get("span")
            print(f"The run's own span was {used if used is not None else span}"
                  f" and it is read from the trace, so --span\ncannot change "
                  f"this. Likely causes: the run was aborted partway, or the "
                  f"trace\nand the routine are from different versions of "
                  f"the tools.")
        return 1
    if misaligned:
        print(f"  note: {len(misaligned)} phase(s) ended away from target "
              f"(within tolerance overall): {misaligned[0]}")
    print()
    print(f"{'phase':<22} {'real s':>7} {'sim s':>7} {'settle s':>8} "
          f"{'worst dev':>10}  joint")
    worst_overall = 0.0
    worst_where = ""
    thin: list[str] = []
    for idx in sorted(by_edge):
        samples = by_edge[idx]
        pair = edges.get(idx)
        if pair is None:
            continue
        a, b, prof = pair
        predicted = edge_duration(prof, a, b)
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
        judged = 0
        for s in samples:
            rel = s["t"] - t0
            if rel > predicted:
                break
            judged += 1
            for i, actual in s["pos"].items():
                if i not in deltas:
                    continue
                expect = (a.ticks.get(i, b.ticks[i])
                          + prof.travelled(deltas[i], rel))
                dev = abs(actual - expect)
                if dev > worst:
                    worst, worst_j = dev, i
        settle = max(0.0, real - predicted)
        deg = span_deg(worst)
        # SUPPRESSED FOR THE APPROACH ONLY, and only when it is the one
        # sample. Phase 0's start pose IS the trace's own first sample,
        # so at rel = 0 the expected position is the measured one and a
        # "0.0d" there is arithmetic, not measurement — and the approach
        # is usually exactly one sample, because `wait_settle(
        # require_still=False)` returns after one poll whenever the drift
        # is already inside SETTLE_TOL_TICKS.
        #
        # NOT extended to phases 1..N, which an earlier version did. For
        # those, `a` is the PLANNED pose, so the deviation at rel = 0 is
        # entry drift — how far off-plan the arm was when the edge was
        # commanded. That is a real measurement, and suppressing it threw
        # away a 35-degree miss and printed a clean bill of health.
        told = idx != 0 or judged >= 2
        if told and deg > worst_overall:
            worst_overall, worst_where = deg, f"{samples[0]['phase']} j{worst_j}"
        shown = f"{deg:9.1f}d" if told else f"{'--':>10}"
        joint = f"j{worst_j}" if told and worst_j else ""
        if not told:
            thin.append(samples[0]["phase"])
        print(f"{samples[0]['phase'][:22]:<22} {real:7.1f} {predicted:7.1f} "
              f"{settle:8.1f} {shown}  {joint}")
    print()
    print(f"worst deviation from the sim: {worst_overall:.1f} deg "
          f"({worst_overall / span_deg(1):.0f} ticks) at {worst_where}")
    if thin:
        # Counted from the rows actually suppressed, not recomputed by a
        # second rule — the two disagreed, and a phase could print '--'
        # with no footnote explaining it.
        print(f"  ({len(thin)} phase(s) shown '--': the approach's start "
              f"pose is the trace's own\n   first sample, so a lone sample "
              f"there measures nothing: {', '.join(thin)})")
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


# ------------------------------------------------------------ selftest
SETTLE_FIXTURE_S = 0.4


def _perfect_run(clip, start: dict[int, int], hz: float = 20.0,
                 settle: float = SETTLE_FIXTURE_S):
    """The trace a PERFECT arm would leave playing `clip`.

    DELIBERATELY DOES NOT CALL `phase_edges`. It re-derives the phase
    numbering and the approach profile straight from `run_clip`'s own
    rule — approach at index 0 and at APPROACH_SPEED_TICKS, then the
    clip's edges from 1 — so that the generator and the reader are two
    independent statements of the same contract. Written the obvious way
    (generate from `phase_edges`, read back with `phase_edges`) the test
    passed no matter what the mapping was: renumber the executor's edges
    and both halves move together, reporting OK while every real
    comparison silently lines up against the wrong edge.

    THE SETTLE TIME IS APPENDED TO EACH PHASE, not left as a gap before
    the next one. A gap is invisible: `compare` measures each phase from
    its OWN first sample, so time between phases is never observed, every
    settle cell read 0.0, and the assertion that claimed to check the
    column passed by matching a `0.4` in the duration columns instead —
    it still passed with the gap removed entirely. The arm really does
    keep being sampled while it converges after arriving, so the honest
    fixture is trailing samples at the target."""
    from sim.clip import edge_duration
    from hardware.bench.runner import APPROACH_SPEED_TICKS
    from sim.clip import MotionProfile, Pose

    pairs = [(n, a, b, clip.profile)
             for n, (a, b) in enumerate(clip.edges(), start=1)]
    first = clip.poses[0]
    here = {**first.ticks,
            **{i: v for i, v in start.items() if i in first.ticks}}
    if here != first.ticks:              # `if approach_drift:` in run_clip
        pairs.insert(0, (0, Pose("start", here), first,
                         MotionProfile(speed=APPROACH_SPEED_TICKS,
                                       acceleration=clip.profile.acceleration)))

    rows: list[tuple[float, str, int, dict]] = []
    t = 0.0
    for idx, a, b, prof in pairs:
        label = f"approach {b.name}" if idx == 0 else f"{a.name}->{b.name}"
        total = edge_duration(prof, a, b)
        ids = sorted(set(a.ticks) | set(b.ticks))
        deltas = {i: b.ticks.get(i, a.ticks[i]) - a.ticks.get(i, b.ticks[i])
                  for i in ids}
        steps = max(1, int(total * hz))
        for k in range(steps + 1):
            rel = total * k / steps
            rows.append((t + rel, label, idx,
                         {i: round(a.ticks.get(i, b.ticks[i])
                                   + prof.travelled(deltas[i], rel))
                          for i in ids}))
        if settle:
            # Parked at the target, still being polled — overhead the
            # clip does not model, and it must land in `settle s` rather
            # than in `worst dev`.
            rows.append((t + total + settle, label, idx, dict(b.ticks)))
        t += total + settle
    return rows


def _column(text: str, header: str) -> list[float]:
    """Read one numeric column out of the printed table, by its heading.

    Assertions grep the whole page otherwise, and a substring found
    anywhere on it is not the same claim as a value in a column. That is
    not hypothetical here: `" 0.4" in text` was checking the settle
    column and matching the duration columns of a different row."""
    lines = text.splitlines()
    head = next((n for n, ln in enumerate(lines) if header in ln), None)
    if head is None:
        return []
    at = lines[head].index(header) + len(header)
    out = []
    for ln in lines[head + 1:]:
        if not ln.strip():
            break
        cell = ln[:at].rsplit(None, 1)[-1] if ln[:at].strip() else ""
        # The deviation column carries a unit suffix ("36.7d"), and a
        # suppressed row carries "--". Neither is a parse failure worth
        # abandoning the column for: skip the row, keep reading.
        cell = cell.rstrip("d")
        try:
            out.append(float(cell))
        except ValueError:
            continue
    return out


def _write_trace(path: Path, rows, meta: dict, shift: int = 0) -> Path:
    tr = Trace(path, meta=meta)
    for t, label, idx, pos in rows:
        tr.phase(label, edge=idx + shift)
        tr.sample(t, pos)
    tr.close()
    return path


def _selftest() -> int:
    """Pin the reference SELECTION, which is the thing that was wrong.

    Every acceptance is paired with a refusal, because the failure this
    replaces was a silent substitution: the tool held a reference that
    was not the run's and reported it as though it were."""
    import contextlib
    import io
    import tempfile

    from hardware.bench.calibrate import load_joint_calibration
    from hardware.bench.runner import APPROACH_SPEED_TICKS

    fails: list[str] = []

    def want(label: str, ok: bool) -> None:
        if not ok:
            fails.append(label)
        print(f"  [{'ok ' if ok else 'FAIL'}] {label}")

    def refuses(label: str, fn, saying: str = "") -> str:
        """Assert a refusal, and — when given — assert WHICH refusal.

        `saying` is not decoration. Without it these pass on any
        BenchError at all, so a guard can be deleted and the assertion
        still goes green because some later check refuses for an
        unrelated reason: drop the separator guard and `runs/x` still
        raises, now with "no file for it was found" while it quietly
        resolves outside the search directory."""
        try:
            fn()
        except BenchError as exc:
            ok = saying.lower() in str(exc).lower()
            want(label if ok else f"{label} [wrong refusal: {exc}]", ok)
            return str(exc)
        want(label, False)
        return ""

    cals = load_joint_calibration(REPO_ROOT / "calibration.json")
    rows = [{"pos": {i: c.rest for i, c in cals.items()}}]

    # --- the clip a trace names is the clip it is compared against.
    want("a clip name resolves to its file",
         find_clip_file("crane-tour") == REPO_ROOT / "crane-tour.json")

    # EVERY clip file in the repo must agree with its own name, because
    # `find_clip_file` is the only route left for a trace older than
    # `clip_file` and it can do nothing but guess `<name>.json`. This is
    # a property of the DATA, not the code, so it is checked here rather
    # than assumed: `home.json` declared itself 'pan-wiggle' until
    # 2026-07-30, which made every trace of it unresolvable and would
    # have made a same-named neighbour resolvable INSTEAD. Renamed to
    # pan-wiggle.json; this keeps the next one from landing.
    disagree = []
    for f in sorted(REPO_ROOT.glob("*.json")):
        try:
            doc = json.loads(f.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            continue
        # A clip has a LIST of poses; poses.json has an object.
        if not isinstance(doc, dict) or not isinstance(doc.get("poses"), list):
            continue
        if str(doc.get("name") or f.stem) != f.stem:
            disagree.append(f"{f.name} calls itself {doc.get('name')!r}")
    want("...and every clip file in the repo is named after the clip "
         "inside it" + (f" ({'; '.join(disagree)})" if disagree else ""),
         not disagree)
    tour = build_reference(rows, cals, 70, {"clip": "crane-tour",
                                            "speed": 250, "accel": 12})
    want("...and a trace naming it builds THAT clip as the reference",
         tour.name == "crane-tour")
    # Counted from the clip, never written as a literal. crane-tour.json
    # is a live artifact whose own notes say the middle of the route is
    # free to reorder; a hard-coded 20 turns a legitimate edit into a
    # KeyError that ABORTS the run, taking the end-to-end checks below
    # with it and never printing the OK/FAILED summary. A test that
    # cannot survive its fixture being edited is a test that will be
    # deleted the first time someone edits it.
    n_edges = len(tour.edges())
    want("...with the clip's own poses, not the exercise routine's",
         n_edges >= 2 and len(tour.poses) == n_edges + 1)

    # THE REGRESSION. Before 2026-07-30 this returned the exercise
    # routine for every trace, so the assertion above could pass by
    # accident only if exercise happened to have 20 edges — pin the
    # negative directly.
    ex = build_reference(rows, cals, 70, {"speed": 250, "accel": 12})
    want("a trace naming NO clip still builds the exercise routine",
         ex.name != "crane-tour" and len(ex.edges()) > 0)
    want("...and the two references are genuinely different motions",
         [p.ticks for p in ex.poses] != [p.ticks for p in tour.poses])

    msg = refuses("a named clip that cannot be found is REFUSED",
                  lambda: build_reference(rows, cals, 70,
                                          {"clip": "no-such-clip"}))
    want("...and the refusal names the clip rather than blaming the arm",
         "no-such-clip" in msg)
    refuses("a clip name that is a path is refused",
            lambda: find_clip_file("../../etc/passwd"), "not a plain clip name")
    refuses("...and so is one that escapes with a bare separator",
            lambda: find_clip_file("runs/x"), "not a plain clip name")
    # A backslash is inert on Linux and a separator on Windows. The
    # bench Pi writes traces the desk reads, so the guard cannot depend
    # on which machine is doing the reading.
    refuses("...and a WINDOWS separator, even when read on Linux",
            lambda: find_clip_file(r"runs\x"), "not a plain clip name")
    # ...and pin the check that does that work directly. On Windows the
    # POSIX arm catches `runs\x` first, so the assertion above cannot
    # reach the Windows arm and passes with it deleted — it is live only
    # on cell1. `PureWindowsPath` is platform-INDEPENDENT, so the
    # predicate itself can be pinned from either machine.
    want("...via a check that is live on BOTH platforms",
         PureWindowsPath("runs\\x").name != "runs\\x")
    refuses("a non-string clip name is refused, not coerced",
            lambda: find_clip_file(42), "not a plain clip name")

    # The recorded PATH beats the guessed name — the whole reason
    # `clip_file` exists is that a clip's name and its filename differ
    # (`runner example` names its clip 'pan-wiggle' and the docs save it
    # as pan.json).
    with tempfile.TemporaryDirectory() as tmp:
        odd = Path(tmp) / "saved-under-another-name.json"
        odd.write_text(
            (REPO_ROOT / "crane-tour.json").read_text(encoding="utf-8"),
            encoding="utf-8")
        by_path = build_reference(rows, cals, 70,
                                  {"clip": "crane-tour",
                                   "clip_file": str(odd),
                                   "speed": 250, "accel": 12})
        want("a trace's recorded clip_file is used even when the file "
             "stem differs from the clip name",
             [p.ticks for p in by_path.poses] == [p.ticks for p in tour.poses])
        # ...but a recorded path is NOT blindly trusted. It is a path out
        # of a data file: a trace copied between machines carries one
        # that means something else here, and a corrupt one can name any
        # JSON on disk. If the file it lands on does not still call
        # itself what the trace says was played, refuse.
        odd2 = Path(tmp) / "renamed.json"
        odd2.write_text(
            (REPO_ROOT / "crane-tour.json").read_text(encoding="utf-8")
            .replace('"name": "crane-tour"', '"name": "something-else"'),
            encoding="utf-8")
        refuses("...but a recorded path landing on a DIFFERENT clip is "
                "refused",
                lambda: build_reference(rows, cals, 70,
                                        {"clip": "crane-tour",
                                         "clip_file": str(odd2),
                                         "speed": 250, "accel": 12}),
                saying="calls itself")
        # ...and an explicit --clip overrides all of it, because that is
        # the operator saying so. Comparing a run against an edited or
        # renamed copy is a legitimate thing to want.
        forced = build_reference(rows, cals, 70,
                                 {"clip": "crane-tour", "speed": 250,
                                  "accel": 12}, clip_path=odd2)
        want("...while an explicit --clip overrides the name check",
             forced.name == "something-else")

    # --- run_clip's phase numbering, reproduced.
    edges = phase_edges(tour, rows[0]["pos"])
    want("phase 0 is the APPROACH to the clip's first pose",
         edges[0][1] is tour.poses[0])
    want("...and it starts from where the arm actually was",
         edges[0][0].ticks == rows[0]["pos"])
    want("...commanded at the approach speed, not the clip's",
         edges[0][2].speed == APPROACH_SPEED_TICKS
         and edges[0][2].speed != tour.profile.speed)
    want("...while keeping the clip's acceleration",
         edges[0][2].acceleration == tour.profile.acceleration)
    want("phase n is the clip's nth edge",
         edges[1][:2] == tour.edges()[0]
         and edges[n_edges][:2] == tour.edges()[n_edges - 1])
    want("...and there is no phase past the last edge",
         n_edges + 1 not in edges)

    # --- end to end, on a trace a perfect arm would have left.
    meta = {"speed": 250, "accel": 12, "clip": "crane-tour",
            "cal": str(REPO_ROOT / "calibration.json")}
    # Start away from pose 0 so the approach is a REAL edge with real
    # motion — a zero-drift start would skip the one phase whose
    # profile differs, and skip the case this test exists for.
    start = {i: c.rest for i, c in cals.items()}
    start[1] = start[1] + 200
    perfect = _perfect_run(tour, start)
    with tempfile.TemporaryDirectory() as tmp:
        good = _write_trace(Path(tmp) / "good.csv", perfect, meta)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = compare(good)
        text = buf.getvalue()
        want("a run matching the plan exactly compares CLEAN", code == 0)
        want("...and reports essentially no deviation",
             "worst deviation from the sim: 0.0 deg" in text)
        want("...including the approach phase, which is in the table",
             "approach rest" in text)
        want("...and says which clip file it compared against",
             "clip file:" in text and "crane-tour.json" in text)
        # READ OUT OF THE SETTLE COLUMN, not grepped from the page. The
        # substring check this replaces (`" 0.4" in text`) matched the
        # real-s/sim-s cells of an unrelated 0.4 s edge, and still
        # passed with the settle fixture removed entirely.
        settles = _column(text, "settle s")
        want("...and reports the settle time in the SETTLE column",
             settles and max(settles) >= SETTLE_FIXTURE_S - 0.05)
        want("...on every phase, since every phase settled",
             settles and min(settles) >= SETTLE_FIXTURE_S - 0.05)
        # The teeth: without the fixture the column must go to zero. A
        # test that cannot fail is not evidence, and this exact one did
        # not fail when it should have.
        flat = _write_trace(Path(tmp) / "nosettle.csv",
                            _perfect_run(tour, start, settle=0.0), meta)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            compare(flat)
        want("...and WITHOUT it the column is zero (so the check can fail)",
             max(_column(buf.getvalue(), "settle s") or [1.0]) < 0.05)

        # The mirror: the same samples, one phase out of step. This is
        # the shape of the 2026-07-27 off-by-one, and it must refuse.
        bad = _write_trace(Path(tmp) / "shifted.csv", perfect, meta, shift=1)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = compare(bad)
        text = buf.getvalue()
        want("the same run shifted by one phase is REFUSED", code == 1)
        want("...naming an edited clip, not --span, for a clip run",
             "EDITED" in text and "--span" not in text)

        # --- a damaged header must REFUSE, not become the exercise
        # routine. This is the original defect rebuilt out of a `pass`.
        raw = good.read_text(encoding="utf-8").splitlines()
        hurt = Path(tmp) / "cut.csv"
        hurt.write_text("\n".join([raw[0][:-8]] + raw[1:]),
                        encoding="utf-8")
        msg = refuses("a header that will not parse is REFUSED",
                      lambda: compare(hurt))
        want("...rather than silently falling back to exercise",
             "JSON" in msg or "header" in msg)
        listy = Path(tmp) / "listy.csv"
        listy.write_text("\n".join(['#[["clip", "crane-tour"]]'] + raw[1:]),
                         encoding="utf-8")
        refuses("a header that is not an object is REFUSED",
                lambda: compare(listy), "not an object")
        # A BOM defeats the `#` test and hands the header to the CSV
        # reader as its column names.
        bom = Path(tmp) / "bom.csv"
        bom.write_bytes(b"\xef\xbb\xbf"
                        + good.read_text(encoding="utf-8").encode("utf-8"))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = compare(bom)
        want("a trace re-saved with a BOM still reads", code == 0)

        # --- a trace with no position data must not score as perfect.
        # Two guards, and each needs its own fixture because the first
        # one fires earlier: blanking EVERY row also blanks the run's
        # first sample, which is the approach's start pose.
        blanked = Path(tmp) / "blank.csv"
        blanked.write_text(
            "\n".join(raw[:2] + [",".join(r.split(",")[:3] + [""] * 6)
                                 for r in raw[2:]]) + "\n",
            encoding="utf-8")
        msg = refuses("a trace whose first sample has no positions is "
                      "REFUSED", lambda: compare(blanked))
        want("...because the approach's start pose cannot be known",
             "first sample" in msg)
        # Keep the run's first sample; blank the rest. Now every phase
        # ENDS with nothing the edge targets — the case that used to
        # score as flawless alignment via `max(..., default=0)`.
        tail = Path(tmp) / "tail-blank.csv"
        tail.write_text(
            "\n".join(raw[:3] + [",".join(r.split(",")[:3] + [""] * 6)
                                 for r in raw[3:]]) + "\n",
            encoding="utf-8")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = compare(tail)
        want("a phase recording nothing its edge targets is REFUSED, "
             "not scored perfect", code == 1)
        want("...saying so rather than reporting zero ticks out",
             "no joint" in buf.getvalue())

        # --- a RECORDED clip path that does not resolve must refuse
        # rather than fall back to guessing `<name>.json`. The guess is
        # guaranteed to satisfy the name check (any working copy keeps
        # the original name), so it substituted the repo's clip silently
        # and reported the difference as the arm's deviation.
        gone = dict(meta, clip_file="somewhere/else/crane-tour.json")
        lost = _write_trace(Path(tmp) / "lost.csv", perfect, gone)
        msg = refuses("a recorded clip path that is not here is REFUSED",
                      lambda: compare(lost))
        want("...naming the recorded path, not a same-named local file",
             "somewhere/else" in msg)
        # ...but the same path resolves by filename beside the trace,
        # which is how a run and its clip travel together.
        (Path(tmp) / "crane-tour.json").write_text(
            (REPO_ROOT / "crane-tour.json").read_text(encoding="utf-8"),
            encoding="utf-8")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = compare(lost)
        want("...while a copy beside the trace IS found", code == 0)

        # --- entry drift on a real edge is REPORTED, not suppressed.
        # Only the approach may print '--'; phases 1..N start from the
        # PLANNED pose, so a deviation at their first sample measures how
        # far off-plan the arm was, which is a measurement.
        drifted = [(t, lab, idx, dict(pos)) for t, lab, idx, pos in perfect]
        hit = next(n for n, r in enumerate(drifted) if r[2] == 2)
        drifted[hit][3][3] += 400                    # 400 ticks, ~35 deg
        dtrace = _write_trace(Path(tmp) / "drift.csv", drifted, meta)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            compare(dtrace)
        want("entry drift on a clip edge is reported, not suppressed",
             max(_column(buf.getvalue(), "worst dev") or [0]) > 30)

        # --- the '--' rule, which needs a phase that actually triggers
        # it. `_perfect_run` samples at 20 Hz, so nothing in the fixtures
        # above is ever suppressed, and BOTH this rule and the over-broad
        # version it replaced passed unchanged.
        #
        # Thin the approach to its single first sample — exactly what
        # `wait_settle(require_still=False)` leaves on a small drift.
        def only_first(rows, phase):
            """Keep every row except phase `phase`, of which keep one."""
            first = next(n for n, r in enumerate(rows) if r[2] == phase)
            return [r for n, r in enumerate(rows)
                    if r[2] != phase or n == first]

        thin_rows = only_first(perfect, 0)
        thinned = _write_trace(Path(tmp) / "thin.csv", thin_rows, meta)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = compare(thinned)
        text = buf.getvalue()
        want("a one-sample APPROACH is shown '--', not 0.0d",
             code == 0 and any(ln.rstrip().endswith("--")
                               for ln in text.splitlines()))
        want("...and the footnote says which phase and why",
             "approach rest" in text.split("worst deviation")[-1])

        # The mirror, and the defect round 2 shipped: the SAME thinning
        # on a real clip edge must still report, because there `a` is the
        # planned pose and a first-sample deviation is entry drift. Under
        # the over-broad rule this printed '--' and a clean bill of
        # health while the arm was 35 degrees off plan.
        edge_thin = [(t, lab, idx, {**pos, 3: pos[3] + 400} if idx == 2
                      else dict(pos))
                     for t, lab, idx, pos in only_first(perfect, 2)]
        et = _write_trace(Path(tmp) / "edge-thin.csv", edge_thin, meta)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            compare(et)
        want("a one-sample CLIP EDGE still reports its entry drift",
             max(_column(buf.getvalue(), "worst dev") or [0]) > 30)

        # --- a malformed row names the line the operator will find.
        lines = good.read_text(encoding="utf-8").splitlines()
        broke = list(lines)
        broke[4] = broke[4].replace(broke[4].split(",")[0], "notanumber", 1)
        wrong = Path(tmp) / "badrow.csv"
        wrong.write_text("\n".join(broke) + "\n", encoding="utf-8")
        msg = refuses("a malformed row is refused", lambda: compare(wrong),
                      "malformed")
        want("...naming the line number it is actually on (file line 5)",
             "row 5" in msg)

        # --- the CLI parser, which nothing above goes through.
        argv = sys.argv
        try:
            # Deliberately NOT the repo's crane-tour.json: that is
            # exactly where the name-guess fallback lands, so pointing
            # --clip at it passes whether the flag is honoured or
            # silently dropped.
            elsewhere = Path(tmp) / "under-another-name.json"
            elsewhere.write_text(
                (REPO_ROOT / "crane-tour.json").read_text(encoding="utf-8"),
                encoding="utf-8")
            sys.argv = ["sim.trace", str(good), "--clip", str(elsewhere)]
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                code = main()
            want("main() honours --clip", code == 0
                 and elsewhere.name in buf.getvalue()
                 and "GUESSED" not in buf.getvalue())
            sys.argv = ["sim.trace", str(good), "--span", "70%"]
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                code = main()
            want("...and a percent-suffixed --span is accepted, not a "
                 "traceback", code == 0)
            sys.argv = ["sim.trace", str(good), "--span", "wide"]
            code = main()
            want("...while a non-numeric --span exits 2, distinct from "
                 "a refusal's 1", code == 2)
        finally:
            sys.argv = argv

        # --- one unmatched phase is structural and is never spent
        # against the tolerance budget.
        extra = Path(tmp) / "extra.csv"
        extra.write_text("\n".join(raw) + f"\n99.0,ghost,{n_edges + 1},2010"
                                          f",800,3130,2950,2060,1110\n",
                         encoding="utf-8")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = compare(extra)
        want("a phase with no matching edge REFUSES on its own", code == 1)

    print("trace selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    return 1 if fails else 0


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0
    if sys.argv[1] == "selftest":
        return _selftest()
    argv = sys.argv[1:]
    clip_path = None
    span = 70
    rest: list[str] = []
    while argv:
        arg = argv.pop(0)
        if arg == "--clip":
            if not argv:
                print("error: --clip needs a path", file=sys.stderr)
                return 2
            clip_path = argv.pop(0)
        elif arg == "--span":
            if not argv:
                print("error: --span needs a number", file=sys.stderr)
                return 2
            raw = argv.pop(0)
            try:
                # `exercise --span` is a percentage, so "70%" is the
                # natural thing to type. A bare int() raises ValueError
                # here and exits 1 — the SAME code as "REFUSING TO
                # COMPARE", so a caller reading the exit code cannot
                # tell a typo from a verdict.
                span = int(raw.rstrip("%"))
            except ValueError:
                print(f"error: --span wants a whole number, got {raw!r}",
                      file=sys.stderr)
                return 2
        else:
            rest.append(arg)
    if len(rest) != 1:
        print("usage: python -m sim.trace TRACE.csv [--clip FILE] "
              "[--span N]", file=sys.stderr)
        return 2
    try:
        return compare(Path(rest[0]), span, clip_path)
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
