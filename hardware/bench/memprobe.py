"""Find out where a long-running process is putting its memory (plan #704).

camserve reached 3.7 GB of cell1's 5.2 GB and was still growing at about
21 MB/min with one viewer attached. Two hypotheses were tested and both
were wrong — the AprilTag detector does not leak (600 detects grew peak
RSS by 0.2 MB), and it is not thread stacks (26 threads) or descriptors
(7). The memory is 3.77 GB of ordinary private-dirty heap.

So stop guessing. This periodically records where the process actually
allocated, and the difference between one sample and the next is the
leak — no theory required.

WHAT IT COSTS. `tracemalloc` roughly doubles allocation cost and adds a
frame-capture per allocation, so this is a DIAGNOSTIC, not something to
leave on. camserve gates it behind `--debug-memory`, off by default, and
says loudly at startup when it is on.

WHY BOTH TRACEMALLOC AND A TYPE HISTOGRAM. They are blind to different
things, and knowing which blindness you are looking at is most of the
diagnosis.

  tracemalloc names the line that allocated — usually the answer. But it
  only sees allocations made through Python's allocator: a C extension
  using its own malloc (OpenCV buffers, V4L2 frames) is invisible to it.
  RSS climbing while tracemalloc stays flat therefore MEANS something,
  and what it means is "a C extension", which is a finding rather than a
  dead end.

  The gc type histogram counts live objects by type — but only
  GC-TRACKED ones. `bytes`, `bytearray`, `int` and `str` hold no
  references, so the collector never tracks them and they NEVER appear
  here, however many are alive. This was found by the selftest failing
  on exactly that case, and it matters for #704 specifically: a leak of
  retained JPEG `bytes` is precisely what camserve is suspected of, and
  the histogram would show nothing at all. tracemalloc catches that one
  (it did, in the selftest). The histogram's job is the other half —
  retained wrappers, handler objects, closures, frames.

Neither alone is enough, which is why both run.

    uv run python -m hardware.bench.memprobe selftest
"""

from __future__ import annotations

import gc
import sys
import threading
import time
from collections import Counter
from pathlib import Path

# How many frames of Python stack to keep per allocation site. Deeper is
# more useful and more expensive; 12 reaches through the HTTP handler
# into the capture loop, which is the span that matters here.
TRACE_DEPTH = 12

# Allocation sites reported per sample. Leaks are usually one site, but
# printing only the top one hides the case where two grow together.
TOP_SITES = 12

# Live object types reported per sample.
TOP_TYPES = 12

# Run the gc type histogram only every Nth sample.
#
# `gc.get_objects()` walks every tracked object in the process and
# stalls it while it does — measured as a ~175 ms pause on camserve,
# which is visible as a 2-second dip in the fps readout because the
# sliding window is 30 frames. Kyle noticed it as "the FPS is jumping
# around", and he was right.
#
# The tracemalloc half is cheap per sample and is also the half that
# matters for #704 (it sees `bytes`; the histogram structurally cannot).
# So the expensive half was buying the weaker signal every time. It now
# runs occasionally — often enough to catch a leak of retained wrappers,
# rarely enough that the stream stays smooth.
TYPES_EVERY = 10


def rss_mb() -> float:
    """Resident set size in MB, or 0.0 where it cannot be read.

    Reads /proc rather than using `resource.getrusage`, whose ru_maxrss
    is the PEAK and therefore never comes back down — it cannot show a
    leak being fixed, only that one happened.
    """
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return float(line.split()[1]) / 1024.0
    except OSError:
        pass
    return 0.0


def type_histogram() -> Counter:
    """Live object counts by type name — sees what tracemalloc cannot."""
    counts: Counter = Counter()
    for obj in gc.get_objects():
        try:
            counts[type(obj).__name__] += 1
        except Exception:                       # pragma: no cover
            continue
    return counts


def sample(baseline, prev_types: Counter | None,
           with_types: bool = True) -> tuple[str, object, Counter | None]:
    """One report: RSS, the biggest growth sites, and type-count deltas.

    `with_types=False` skips the gc walk, which is the expensive part —
    see TYPES_EVERY.
    """
    import tracemalloc

    snap = tracemalloc.take_snapshot()
    types = type_histogram() if with_types else None
    lines = [f"--- {time.strftime('%Y-%m-%d %H:%M:%S')}  "
             f"RSS {rss_mb():.0f} MB  "
             f"tracemalloc {tracemalloc.get_traced_memory()[0] / 1e6:.0f} MB "
             f"traced ---"]

    lines.append("  growth since baseline, by allocation site:")
    for stat in snap.compare_to(baseline, "lineno")[:TOP_SITES]:
        if stat.size_diff <= 0:
            continue
        frame = stat.traceback[0]
        lines.append(f"    {stat.size_diff / 1e6:+8.1f} MB  "
                     f"{stat.count_diff:+8d} blocks  "
                     f"{Path(frame.filename).name}:{frame.lineno}")

    if types is not None and prev_types is not None:
        grown = sorted(((types[k] - prev_types.get(k, 0), k) for k in types),
                       reverse=True)[:TOP_TYPES]
        lines.append("  live objects, change since the previous sample:")
        for delta, name in grown:
            if delta <= 0:
                continue
            lines.append(f"    {delta:+9d}  {name}  (now {types[name]})")
    return "\n".join(lines), snap, types


def probe_loop(stop: threading.Event, out: Path, interval_s: float) -> None:
    """Write a memory report every `interval_s` until `stop` is set."""
    import tracemalloc

    tracemalloc.start(TRACE_DEPTH)
    # Let the process finish importing and opening cameras before the
    # baseline: everything allocated during startup is not a leak, and
    # counting it as one buries the real growth under a constant.
    stop.wait(min(interval_s, 30.0))
    baseline = tracemalloc.take_snapshot()
    prev = type_histogram()
    with out.open("a", encoding="utf-8") as fh:
        fh.write(f"\n=== memprobe started {time.strftime('%Y-%m-%d %H:%M:%S')}"
                 f", baseline RSS {rss_mb():.0f} MB, "
                 f"interval {interval_s:.0f}s ===\n")
        fh.flush()
        n = 0
        while not stop.wait(interval_s):
            n += 1
            # The first sample carries a histogram so a leak already in
            # progress is visible immediately; after that it is periodic.
            want_types = (n == 1) or (n % TYPES_EVERY == 0)
            try:
                text, _, types = sample(baseline, prev, with_types=want_types)
                if types is not None:
                    prev = types
            except Exception as exc:            # never take the server down
                text = f"--- memprobe sample failed: {exc!r} ---"
            fh.write(text + "\n")
            fh.flush()


def start(out: Path, interval_s: float = 300.0) -> threading.Event:
    """Begin probing in the background. Returns the stop event."""
    stop = threading.Event()
    threading.Thread(target=probe_loop, args=(stop, out, interval_s),
                     name="memprobe", daemon=True).start()
    return stop


# --------------------------------------------------------------------


def selftest() -> int:
    import tempfile

    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not cond:
            fails.append(name)

    print("a leak of RAW BYTES — the shape camserve is suspected of")
    leak: list = []

    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "mem.log"
        stop = start(out, interval_s=0.4)
        deadline = time.time() + 4.0
        while time.time() < deadline:
            leak.append(bytearray(200_000))
            time.sleep(0.005)
        stop.set()
        time.sleep(0.6)
        text = out.read_text(encoding="utf-8")

    check("the probe wrote a report", "memprobe started" in text,
          f"{len(text)} bytes")
    check("tracemalloc names this file as the growing site",
          "memprobe.py:" in text,
          "the leak was allocated here, so it must be blamed here")
    grew = [l for l in text.splitlines() if "MB" in l and "+" in l]
    check("it reports positive growth", bool(grew),
          grew[0].strip() if grew else "no growth lines")
    # Recorded as a KNOWN BLINDNESS, not skipped. bytes/bytearray hold no
    # references so the collector never tracks them, and no number of
    # live ones will ever show in the histogram. If this ever starts
    # failing, CPython changed and the docstring above is wrong.
    check("...and the type histogram does NOT see bytearray (known blind)",
          "bytearray" not in text,
          "untracked types are invisible to gc.get_objects — which is "
          "why tracemalloc is the half that matters for #704")

    print("\na leak of TRACKED objects — the half the histogram covers")
    tracked: list = []

    class Retained:                    # a class IS gc-tracked
        __slots__ = ("payload",)

        def __init__(self):
            self.payload = [0] * 128

    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "tracked.log"
        stop = start(out, interval_s=0.4)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            tracked.append(Retained())
            time.sleep(0.001)
        stop.set()
        time.sleep(0.6)
        ttext = out.read_text(encoding="utf-8")
    check("the histogram names the retained class", "Retained" in ttext,
          "this is what it is for")

    print("\nit must not invent a leak where there is none")
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "quiet.log"
        stop = start(out, interval_s=0.4)
        time.sleep(2.5)
        stop.set()
        time.sleep(0.5)
        quiet = out.read_text(encoding="utf-8")
    big = [l for l in quiet.splitlines()
           if "MB" in l and "+" in l and "RSS" not in l
           and abs(float(l.split("MB")[0].split()[-1])) > 5.0]
    check("an idle process reports no large growth", not big,
          str(big[:2]))

    print("\nhousekeeping")
    check("rss_mb works or honestly returns 0",
          isinstance(rss_mb(), float),
          f"{rss_mb():.0f} MB (0 means this platform has no /proc)")
    check("the type histogram sees real objects",
          type_histogram().get("dict", 0) > 10)

    del leak
    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("memprobe OK")
    return 0


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        return selftest()
    print("usage: python -m hardware.bench.memprobe selftest")
    print("       (camserve runs this via --debug-memory)")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
