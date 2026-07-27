"""batch — run the exercise across a matrix of conditions, collecting evidence.

Plan #676. For gathering sim-vs-arm evidence at volume: sequence a set of
conditions, leave a trace and camera stills for each, and stop the moment
anything fails.

    uv run python -m hardware.bench.batch --accel 5,15,30,60 --out runs/
    uv run python -m hardware.bench.batch --repeat 3 --out runs/
    uv run python -m hardware.bench.batch --accel 5,15 --repeat 2 --dry-run

THE NUMBERS DECIDE; THE PICTURES EXPLAIN. Every run's pass/fail comes
from measured quantities — the tool's exit code, whether the arm came
back to rest, the deviation `sim.trace` reports. Stills are captured for
every run and READ only when a row is bad. Model inference is not in
this loop; it is what you call after the loop has already decided
something went wrong.

## It invokes the real tool

Each run is a subprocess call to `hardware.bench.exercise`, never an
import-and-drive. exercise.py owns the collision gate, the encoder
guards, the wake ordering and the torque-off contract; a runner that
re-implemented any of that would be a SECOND motion path free to drift
from the first, which is the exact defect class the clip layer exists to
remove. One motion path, guaranteed by construction.

## Safety

Every protection the tool has survives a batch: pre-flight gate,
start-pose check, entry guards, in-motion invariants, halt-and-hold,
torque off on every exit. `--yes` skips the prompt, never the checks.

**The e-stop does not survive.** It is a keypress, so it needs a human
at the bench. Running a batch unattended means the guard set is the last
line of defence and there is nothing behind it. That is a decision to
make deliberately, not a consequence of a runner existing. The mitigations
here — stop on first failure, health check between runs, hard caps — are
real but they are not a person.

Usage: batch [--speed LIST] [--accel LIST] [--span LIST] [--repeat N]
             [--out DIR] [--camera-url URL] [--max-runs N] [--max-minutes N]
             [--dry-run] [--yes]
"""

from __future__ import annotations

import argparse
import itertools
import json
import subprocess
import sys
import time
from pathlib import Path

from hardware.errors import BenchError

from .bus import confirm, run_tool

# Hard ceilings. A bug in the condition builder must not be able to
# exercise the arm indefinitely; these bound it regardless of the matrix.
MAX_RUNS_CAP = 60
MAX_MINUTES_CAP = 240
RUN_TIMEOUT_S = 900          # one run that hangs stops the batch
CAMERA_TIMEOUT_S = 15


def _floats(text: str) -> list[float]:
    return [float(x) for x in text.split(",") if x.strip()]


def _ints(text: str) -> list[int]:
    return [int(x) for x in text.split(",") if x.strip()]


def conditions(args) -> list[dict]:
    """The matrix, expanded. Repeats are ADJACENT so a drifting arm shows
    up as a trend within a condition rather than being smeared across the
    whole batch."""
    out = []
    for speed, accel, span in itertools.product(
            _floats(args.speed), _ints(args.accel), _ints(args.span)):
        for rep in range(args.repeat):
            out.append({"speed": speed, "accel": accel, "span": span,
                        "rep": rep + 1})
    return out


def capture(url: str | None, label: str) -> dict:
    """Ask camserve for a labelled capture set. Never fatal.

    Evidence is not a precondition for the motion having been safe, so a
    camera that is missing, unplugged or slow is RECORDED and stepped
    over. A batch that refuses to run because a camera is absent would
    be trading the data we came for against the data we would like."""
    if not url:
        return {"skipped": "no camera url"}
    import urllib.error
    import urllib.request
    try:
        with urllib.request.urlopen(f"{url}/capture?label={label}",
                                    timeout=CAMERA_TIMEOUT_S) as r:
            return json.loads(r.read())
    except (urllib.error.URLError, OSError, ValueError) as exc:
        return {"error": str(exc)}


def health(cal: str, port: str | None) -> tuple[bool, str]:
    """Between runs: does every servo answer, and is the arm back at rest?

    Checked BETWEEN runs and not only before the first, because drift
    accumulating over a batch is exactly what unattended operation is
    blind to. The exercise tool's own start-pose check would refuse the
    next run anyway — this reports it as a health failure rather than as
    a mysterious run failure."""
    from .bus import FeetechBus
    from .calibrate import load_calibration
    from .exercise import PREFLIGHT_REST_TOL_TICKS

    try:
        cals = load_calibration(Path(cal))
        with FeetechBus(port) as bus:
            worst, worst_id = 0, 0
            for i, c in sorted(cals.items()):
                pos = bus.read_position(i)
                err = abs(pos - c.rest)
                if err > worst:
                    worst, worst_id = err, i
            if worst > PREFLIGHT_REST_TOL_TICKS:
                return False, (f"joint {worst_id} is {worst} ticks from rest "
                               f"(limit {PREFLIGHT_REST_TOL_TICKS})")
            return True, f"all joints answer; worst {worst} ticks from rest"
    except (BenchError, OSError) as exc:
        return False, str(exc)


def deviation(trace: Path) -> dict:
    """The headline number from sim.trace, so the manifest carries the
    result and not just the fact that a run happened."""
    try:
        from sim.trace import compare
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            compare(trace)
        text = buf.getvalue()
        line = next((l for l in text.splitlines()
                     if "worst deviation" in l), "")
        return {"summary": line.strip(), "refused": "REFUSING" in text}
    except Exception as exc:                  # analysis must never be fatal
        return {"error": str(exc)}


def run() -> int:
    parser = argparse.ArgumentParser(
        prog="python -m hardware.bench.batch",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", default="1.0", help="comma list (default 1.0)")
    parser.add_argument("--accel", default="15", help="comma list (default 15)")
    parser.add_argument("--span", default="70", help="comma list (default 70)")
    parser.add_argument("--repeat", type=int, default=1,
                        help="runs per condition (default 1)")
    parser.add_argument("--out", default="runs",
                        help="directory for traces + manifest (default runs/)")
    parser.add_argument("--cal", default="calibration.json")
    parser.add_argument("--port", default=None)
    parser.add_argument("--camera-url", default=None,
                        help="camserve base URL, e.g. http://localhost:8081 "
                             "(optional; a batch without it still collects "
                             "traces)")
    parser.add_argument("--max-runs", type=int, default=MAX_RUNS_CAP)
    parser.add_argument("--max-minutes", type=int, default=MAX_MINUTES_CAP)
    parser.add_argument("--dry-run", action="store_true",
                        help="print the matrix and exit — moves nothing")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation (never the tool's "
                             "own pre-flight checks)")
    args = parser.parse_args()

    if args.repeat < 1:
        raise BenchError("--repeat must be at least 1")
    plan = conditions(args)
    cap = min(args.max_runs, MAX_RUNS_CAP)
    if len(plan) > cap:
        raise BenchError(
            f"matrix expands to {len(plan)} runs, over the {cap} cap",
            "narrow the lists or raise --max-runs (hard ceiling "
            f"{MAX_RUNS_CAP})")
    budget_s = min(args.max_minutes, MAX_MINUTES_CAP) * 60

    print(f"batch: {len(plan)} run(s)")
    for n, c in enumerate(plan, 1):
        print(f"  {n:>2}. speed {c['speed']} accel {c['accel']} "
              f"span {c['span']}  (rep {c['rep']}/{args.repeat})")
    print(f"  budget: {budget_s // 60} min, per-run timeout "
          f"{RUN_TIMEOUT_S // 60} min")
    print(f"  camera: {args.camera_url or 'none — traces only'}")
    if args.dry_run:
        print("\ndry run — nothing moved")
        return 0

    print("\nTHE ARM WILL MOVE, repeatedly, unattended if you walk away.")
    print("  The e-stop is a KEYPRESS: it does not exist when nobody is here.")
    print("  Everything else still applies — gate, guards, halt-and-hold, "
          "torque off.")
    if not args.yes and not confirm("type y to start the batch: "):
        print("aborted")
        return 1

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    traces = out / "traces"
    traces.mkdir(exist_ok=True)
    manifest = out / "batch.json"
    started = time.monotonic()
    rows: list[dict] = []

    def save() -> None:
        # Rewritten after EVERY run: a batch that dies still leaves a
        # readable manifest for everything that finished.
        manifest.write_text(json.dumps(
            {"started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                          time.gmtime()),
             "planned": len(plan), "completed": len(rows), "runs": rows},
            indent=2))

    for n, c in enumerate(plan, 1):
        if time.monotonic() - started > budget_s:
            print(f"\nbudget of {budget_s // 60} min reached — stopping "
                  f"after {len(rows)} run(s)")
            break
        tag = f"run{n:02d}-sp{c['speed']}-ac{c['accel']}-sn{c['span']}"
        print(f"\n=== [{n}/{len(plan)}] {tag}")

        ok, why = health(args.cal, args.port)
        print(f"  health: {why}")
        if not ok:
            rows.append({"n": n, **c, "tag": tag, "status": "health-failed",
                         "detail": why})
            save()
            print("  STOPPING: the arm is not in a fit state to start "
                  "another run.")
            return 1

        before = capture(args.camera_url, f"{tag}-before")
        cmd = [sys.executable, "-m", "hardware.bench.exercise",
               "--trace", str(traces) + "/", "--speed", str(c["speed"]),
               "--accel", str(c["accel"]), "--span", str(c["span"]),
               "--cal", args.cal, "--yes"]
        if args.port:
            cmd += ["--port", args.port]
        t0 = time.monotonic()
        try:
            proc = subprocess.run(cmd, timeout=RUN_TIMEOUT_S)
            code = proc.returncode
        except subprocess.TimeoutExpired:
            code = -1
        dur = time.monotonic() - t0
        after = capture(args.camera_url, f"{tag}-after")

        # newest trace in the directory is this run's
        found = sorted(traces.glob("*.csv"), key=lambda p: p.stat().st_mtime)
        trace = found[-1] if found else None
        row = {"n": n, **c, "tag": tag, "exit": code,
               "seconds": round(dur, 1),
               "trace": str(trace) if trace else None,
               "stills": {"before": before, "after": after},
               "status": "ok" if code == 0 else "run-failed"}
        if trace is not None and code == 0:
            row["deviation"] = deviation(trace)
        rows.append(row)
        save()

        print(f"  exit {code} in {dur:.0f} s"
              + (f" -> {trace.name}" if trace else ""))
        if row.get("deviation", {}).get("summary"):
            print(f"  {row['deviation']['summary']}")
        if code != 0:
            print("  STOPPING: a batch must never turn one failure into "
                  "many.\n  Evidence for this run is on disk; the stills "
                  "are worth reading now.")
            return 1

    save()
    print(f"\nbatch complete: {len(rows)} run(s) -> {manifest}")
    print("  read the manifest first; open stills only for rows that "
          "are not 'ok'.")
    return 0


def main() -> int:
    if "--selftest" in sys.argv:
        _selftest()
        return 0
    return run_tool(run)


def _selftest() -> None:
    """Matrix expansion and caps. No hardware, no subprocess, no motion."""
    fails: list[str] = []

    def want(label: str, ok: bool) -> None:
        if not ok:
            fails.append(label)
        print(f"  [{'ok ' if ok else 'FAIL'}] {label}")

    class A:
        speed, accel, span, repeat = "1.0", "5,15", "70", 2

    plan = conditions(A())
    want("matrix expands over every axis", len(plan) == 4)
    want("repeats are ADJACENT, so drift shows as a trend inside a "
         "condition rather than smeared across the batch",
         [c["accel"] for c in plan] == [5, 5, 15, 15])

    class B(A):
        speed, accel, span, repeat = "0.5,1.0", "5,15,30", "50,70", 3

    want("a wide matrix is large enough to need the cap",
         len(conditions(B())) == 36)
    want("the hard cap is below what a careless matrix reaches",
         MAX_RUNS_CAP < 2 * len(conditions(B())))
    want("camera capture with no url is skipped, not an error",
         capture(None, "x").get("skipped") is not None)
    want("camera capture against a dead url is an ERROR the manifest "
         "keeps, not an exception",
         "error" in capture("http://127.0.0.1:9", "x"))
    print("batch selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    if fails:
        raise SystemExit(1)


if __name__ == "__main__":
    sys.exit(main())
