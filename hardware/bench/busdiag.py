"""busdiag — is the servo bus telling the truth? Read-only.

Plan #704-adjacent, born 2026-07-27. The strain guard aborted two bench
runs on temperature readings the arm never had: joint 4 reported 74 C
and later 46 C, and `scan` read 31-32 C seconds afterwards. The guard
now filters those out, but filtering a lie is not the same as knowing
why it was told — and the same bus carries the POSITION reads that the
collision gate, the settle test and the sim-vs-arm trace all rest on. A
corrupted position read is indistinguishable from a real one.

Kyle's two questions, which this tool exists to answer:

  1. "I have a hard time believing if the board had issues, it would
     just be with temp?" — Right, and the honest answer is that
     temperature is simply the only register with a PHYSICS filter
     behind it. A wrong position looks exactly like a real position.
     So: measure every register, not just the one that happens to be
     checkable.

  2. "Is it just one motor or several?" — A single bad servo is a
     different problem from a bad bus, and nothing recorded which.
     Every count here is per joint.

THE HYPOTHESIS THIS IS BUILT TO TEST. The bad values are not noise: 74
and 46 are perfectly believable numbers, and a corrupted byte is usually
obvious garbage. A well-formed response to the WRONG request is not — if
a late reply from an earlier transaction is consumed as this one's
answer, the checksum passes and the value is real, just from somewhere
else. `read_health` issues FIVE separate transactions per joint (load,
status, temp, volts, current) and the settle loop interleaves position
reads between them, so there is plenty for a stale reply to be confused
with. That predicts something specific and testable: reading the
temperature register ALONE, with nothing else in flight, should be
clean, while reading it inside the full five-register sequence should
not. Mode B against mode A is that experiment.

This tool NEVER commands motion, never changes torque, and never writes
a register. It is safe to run on a live arm at any time.

    uv run python -m hardware.bench.busdiag --seconds 60
    uv run python -m hardware.bench.busdiag --seconds 60 --solo-only
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import Counter, defaultdict

from .bus import (REG_PRESENT_TEMPERATURE, BenchError, FeetechBus,
                  run_tool)
from .monitor import parse_ids

# A joint's temperature over a minute-long read loop with no motion is
# essentially constant, so its own median is the reference. Anything
# further than this from it is a misread, not weather.
OUTLIER_C = 3


def _pct(n: int, total: int) -> str:
    return f"{100.0 * n / total:5.2f}%" if total else "    -"


def sample_full(bus: FeetechBus, ids: list[int], seconds: float) -> dict:
    """Mode A — the full five-register health read, exactly as
    StrainWatch drives it. This is the pattern that misbehaved."""
    rows: dict[int, list[dict]] = defaultdict(list)
    errors: Counter = Counter()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for i in ids:
            try:
                rows[i].append(bus.read_health(i))
            except BenchError:
                errors[i] += 1
        print(f"\r  mode A (full health read): "
              f"{sum(len(v) for v in rows.values())} samples", end="",
              flush=True)
    print()
    return {"rows": rows, "errors": errors}


def sample_solo(bus: FeetechBus, ids: list[int], seconds: float) -> dict:
    """Mode B — the temperature register ALONE, nothing else in flight.

    If the corruption is a stale reply being matched to the wrong
    request, this is the condition under which it cannot happen."""
    temps: dict[int, list[int]] = defaultdict(list)
    errors: Counter = Counter()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for i in ids:
            try:
                temps[i].append(bus.read_u8(i, REG_PRESENT_TEMPERATURE,
                                            "read temp"))
            except BenchError:
                errors[i] += 1
        print(f"\r  mode B (temperature only): "
              f"{sum(len(v) for v in temps.values())} samples", end="",
              flush=True)
    print()
    return {"temps": temps, "errors": errors}


def outliers(values: list[int]) -> tuple[int, list[int], float]:
    """(count, the offending values, the median they departed from)."""
    if len(values) < 3:
        return 0, [], float(values[0]) if values else 0.0
    med = statistics.median(values)
    bad = [v for v in values if abs(v - med) > OUTLIER_C]
    return len(bad), bad, med


def spikes(values: list[int]) -> tuple[int, int, int, list[int]]:
    """Separate SPIKES from DRIFT. (spikes, worst step, longest run, samples)

    A median test cannot tell a corrupted reading from a servo that is
    genuinely warming: both look like values far from the middle. The
    difference is in the neighbours. A real temperature moves by one
    count and STAYS there; a corrupted one differs from the readings on
    BOTH sides of it and the next sample is back where it was.

    This is the same reasoning the strain guard's rate filter uses, which
    is the point — if this says drift, the guard is discarding real data
    and its threshold is wrong. `longest run` is how many samples in a
    row an off-median value persisted: 1 means spike, many means the
    servo really did go there."""
    if len(values) < 3:
        return 0, 0, 0, values
    med = statistics.median(values)
    n_spike = 0
    worst_step = 0
    longest = 0
    run = 0
    for k, v in enumerate(values):
        if k:
            worst_step = max(worst_step, abs(v - values[k - 1]))
        off = abs(v - med) > OUTLIER_C
        run = run + 1 if off else 0
        longest = max(longest, run)
        if off and 0 < k < len(values) - 1:
            if (abs(v - values[k - 1]) > OUTLIER_C
                    and abs(v - values[k + 1]) > OUTLIER_C):
                n_spike += 1
    return n_spike, worst_step, longest, values


def report_full(data: dict, ids: list[int]) -> list[int]:
    """Per-joint anomaly rates across EVERY register, not just temp."""
    rows, errors = data["rows"], data["errors"]
    print("\nmode A — full health read, per joint")
    print(f"  {'joint':>5} {'samples':>8} {'temp med':>9} {'temp bad':>9} "
          f"{'volt bad':>9} {'load>50':>8} {'comm err':>9}")
    dirty = []
    for i in ids:
        rs = rows.get(i, [])
        if not rs:
            print(f"  {i:>5} {'no data':>8}")
            continue
        temps = [r["temp_c"] for r in rs]
        volts = [r["volts"] for r in rs]
        n_temp, bad_temps, med = outliers(temps)
        # Voltage is just as constant as temperature on a bench supply,
        # so the same test applies and asks the real question: is this
        # ONE register misbehaving or the whole read?
        vmed = statistics.median(volts) if volts else 0.0
        n_volt = sum(1 for v in volts if abs(v - vmed) > 0.5)
        n_load = sum(1 for r in rs if r["load_pct"] > 50)
        if n_temp or n_volt:
            dirty.append(i)
        print(f"  {i:>5} {len(rs):>8} {med:>9.0f} "
              f"{n_temp:>4} {_pct(n_temp, len(rs))} "
              f"{n_volt:>4} {_pct(n_volt, len(rs))} "
              f"{n_load:>8} {errors.get(i, 0):>9}")
        if bad_temps:
            counts = Counter(bad_temps).most_common(6)
            print(f"        bad temps seen: "
                  + ", ".join(f"{v}C x{c}" for v, c in counts))
    return dirty


def correlate(data: dict, ids: list[int]) -> None:
    """Does a bad temperature equal another register from the same read?

    The stale-reply hypothesis predicts yes: the value is genuine, it
    just belongs to a different question. Anything that lands here names
    the register whose answer was handed over by mistake."""
    print("\nwhere do the bad values come from?")
    hits = Counter()
    total_bad = 0
    for i in ids:
        rs = data["rows"].get(i, [])
        temps = [r["temp_c"] for r in rs]
        _, bad, med = outliers(temps)
        badset = set(bad)
        for r in rs:
            if r["temp_c"] not in badset or abs(r["temp_c"] - med) <= OUTLIER_C:
                continue
            total_bad += 1
            t = r["temp_c"]
            # Every other number available in the same health read.
            if t == int(r["volts"] * 10):
                hits["voltage raw (reg 62)"] += 1
            if t == r["status"]:
                hits["status (reg 65)"] += 1
            if t == int(round(r["current_ma"] / 6.5)) & 0xFF:
                hits["current low byte (reg 69)"] += 1
            if t == int(r["load_pct"] * 10) & 0xFF:
                hits["load low byte (reg 60)"] += 1
    if not total_bad:
        print("  (no bad temperatures to explain)")
        return
    if not hits:
        print(f"  {total_bad} bad reading(s), NONE matching another "
              f"register in the same sample.")
        print("  => not a mis-routed reply from this joint's own read "
              "sequence. Suspect a reply from ANOTHER JOINT, a partial "
              "frame, or a genuinely flipped bit.")
        return
    print(f"  {total_bad} bad reading(s); matches found:")
    for name, n in hits.most_common():
        print(f"    {n:>5} ({_pct(n, total_bad)}) equal to this sample's "
              f"{name}")
    print("  => a bad temperature carrying another register's value is a "
          "MIS-ROUTED REPLY, not line noise: the byte is real, the "
          "question it answers is not.")


def report_solo(data: dict, ids: list[int]) -> list[int]:
    temps, errors = data["temps"], data["errors"]
    print("\nmode B — temperature register alone, nothing else in flight")
    print(f"  {'joint':>5} {'samples':>8} {'temp med':>9} {'temp bad':>9} "
          f"{'comm err':>9} {'isolat':>7} {'wstep':>6} {'runmax':>6}")
    dirty = []
    for i in ids:
        vs = temps.get(i, [])
        if not vs:
            print(f"  {i:>5} {'no data':>8}")
            continue
        n, bad, med = outliers(vs)
        n_spike, step, longest, _ = spikes(vs)
        if n:
            dirty.append(i)
        print(f"  {i:>5} {len(vs):>8} {med:>9.0f} "
              f"{n:>4} {_pct(n, len(vs))} {errors.get(i, 0):>9} "
              f"{n_spike:>7} {step:>6} {longest:>6}")
        if bad:
            counts = Counter(bad).most_common(6)
            print(f"        bad temps seen: "
                  + ", ".join(f"{v}C x{c}" for v, c in counts))
    print("\n  isolated = differs from BOTH neighbours (a corrupted read).")
    print("  worst step = largest change between consecutive samples.")
    print("  longest run = samples in a row an off-median value held;")
    print("                1 means spikes, many means the servo really "
          "went there.")
    return dirty


def verdict(a_dirty: list[int], b_dirty: list[int], ids: list[int],
            ran_a: bool, ran_b: bool) -> None:
    print("\n" + "=" * 60)
    if ran_a and ran_b:
        if a_dirty and not b_dirty:
            print("VERDICT: the register is clean when read ALONE and dirty "
                  "inside the five-register health read.")
            print("  That is a protocol/timing fault, not electrical noise "
                  "and not a bad servo — a reply being matched to the wrong "
                  "request. Cable length is not the cause and replacing it "
                  "will not help.")
            print("  Next: raise the packet timeout, and/or stop issuing "
                  "five separate transactions per joint per sample.")
        elif a_dirty and b_dirty:
            print("VERDICT: dirty in BOTH modes — the corruption does not "
                  "need concurrent transactions to appear.")
            print("  That points at the link itself (adapter, cable, "
                  "termination, baud) or at the servos, not at the read "
                  "pattern. Cable and adapter are now worth swapping.")
        elif not a_dirty and not b_dirty:
            print("VERDICT: clean in both modes over this window.")
            print("  The fault is intermittent — it appeared during MOTION, "
                  "and this run was static. Re-run alongside a moving arm "
                  "before concluding anything.")
        else:
            print("VERDICT: dirty reading the register ALONE but clean in "
                  "the full sequence — unexpected; treat the sample sizes "
                  "with suspicion and re-run for longer.")
    common = sorted(set(a_dirty) & set(b_dirty)) if ran_a and ran_b else []
    every = sorted(set(a_dirty) | set(b_dirty))
    if not every:
        print("Joints implicated: none.")
    elif len(every) == 1:
        print(f"Joints implicated: ONLY joint {every[0]} — a single servo, "
              f"not the bus. Swap it with a known-good one to confirm.")
    elif len(every) == len(ids):
        print(f"Joints implicated: ALL of {every} — the bus or the adapter, "
              f"not any one servo.")
    else:
        print(f"Joints implicated: {every} of {ids} — neither one servo nor "
              f"all of them. Note whether these are the ones furthest down "
              f"the daisy chain.")
    if common:
        print(f"  (dirty in both modes: {common})")
    print("=" * 60)


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.busdiag",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ids", default="1-6", help="servo IDs")
    parser.add_argument("--seconds", type=float, default=45.0,
                        help="sample window PER MODE (default 45)")
    parser.add_argument("--port", default=None)
    parser.add_argument("--solo-only", action="store_true",
                        help="only the single-register mode")
    parser.add_argument("--full-only", action="store_true",
                        help="only the five-register health read mode")
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    ran_a = not args.solo_only
    ran_b = not args.full_only
    a_dirty: list[int] = []
    b_dirty: list[int] = []

    with FeetechBus(args.port) as bus:
        missing = [i for i in ids if bus.ping(i) is None]
        if missing:
            raise BenchError(f"no answer from servo IDs {missing}")
        print(f"bus diagnostic on {bus.port_name}, joints {ids}")
        print("READ ONLY — no motion, no torque change, no register write.\n")

        if ran_a:
            a = sample_full(bus, ids, args.seconds)
            a_dirty = report_full(a, ids)
            correlate(a, ids)
        if ran_b:
            b = sample_solo(bus, ids, args.seconds)
            b_dirty = report_solo(b, ids)

    verdict(a_dirty, b_dirty, ids, ran_a, ran_b)
    return 0


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
