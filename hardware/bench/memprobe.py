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

THE THIRD INSTRUMENT: `malloc_info()`. Both of the above look at the
process from inside Python. Neither can tell the difference between the
two remaining explanations for #704, which are opposites and want
opposite fixes:

  the memory is FREED but the allocator has not returned it to the OS
  (fragmentation, or corrupted chunk headers that defeat coalescing)
      -> RSS is honest, the heap is mostly holes, the fix is allocator-
         level: periodic trim, a different allocation pattern, or the
         corruption itself.

  the memory is genuinely still ALLOCATED by a C extension
      -> the fix is code: something on the tags path never frees.

glibc answers this directly. `malloc_info(0, stream)` dumps per-arena
XML including how many bytes sit free in the bins against how many the
arena has taken from the system. Free ~= system means holes; free ~= 0
means the memory is live. `arena_report()` calls it and `heap_summary()`
turns the XML into that one ratio.

WHY IT IS AN ENDPOINT AND NOT A FLAG. On 2026-07-29 a `malloc_trim(0)`
call injected with `gdb -p` took SIGSEGV inside `__malloc_trim` and
killed a camserve that had spent 3 h 42 m growing to 1.26 GB — the
specimen was destroyed and the question went unanswered. gdb had every
thread stopped mid-flight; from inside a request handler the allocator
locks are held properly and the same call is safe. So this ships as
`/debug/memory` on the running server: no attach, no restart to begin
diagnosing, and nothing to lose if it goes wrong. Diagnosing a leak must
not require killing the leak.

`?trim=1` then calls `malloc_trim(0)` as the confirmation — RSS dropping
proves the free-but-unreturned reading. It is opt-in because the
read-only number should be taken first, while the heap is untouched.

STILL UNTESTED FROM HERE: heap CORRUPTION as such. If the ratio says
"mostly free" the follow-up is a run under `MALLOC_CHECK_=3`, which
needs no code — glibc aborts the process on a detected bad chunk, and
the abort is the result.

    uv run python -m hardware.bench.memprobe selftest
"""

from __future__ import annotations

import ctypes
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


# --------------------------------------------------------- glibc arenas

# A "mostly holes" heap and a "mostly live" heap want opposite fixes, so
# the ratio between free bytes and system bytes is the number this whole
# module exists to produce. These are the cuts used to say so in words.
# They are deliberately far apart: between them the answer is "neither
# reading is safe", which is a legitimate result and better than a
# confident label straddling the boundary.
FREE_RATIO_HOLES = 0.20     # >= this: the allocator is sitting on holes
FREE_RATIO_LIVE = 0.05      # <= this: the memory is genuinely allocated

_LIBC: ctypes.CDLL | None = None
_LIBC_TRIED = False
# camserve is a ThreadingHTTPServer, so two requests can land in here at
# once. Without this lock the second thread can see _LIBC_TRIED already
# set while the first is still mid-setup, get None, and report "not
# glibc" on a machine that plainly is one — a diagnostic lying about the
# platform it is diagnosing.
_LIBC_LOCK = threading.Lock()


def _libc() -> ctypes.CDLL | None:
    """glibc, or None where there isn't one (Windows, macOS, musl).

    Returning None rather than raising is deliberate: this module's
    selftest runs on Kyle's Windows desk as well as on cell1, and a
    diagnostic that cannot be imported off-target is a diagnostic nobody
    runs. Callers report the absence instead of failing.
    """
    global _LIBC, _LIBC_TRIED
    with _LIBC_LOCK:
        if _LIBC_TRIED:
            return _LIBC
        _LIBC_TRIED = True
        try:
            lib = ctypes.CDLL("libc.so.6", use_errno=True)
            lib.malloc_info                      # noqa: B018 - probe
            lib.malloc_trim                      # noqa: B018 - probe
            # open_memstream returns a FILE*. Left at the default int
            # restype ctypes truncates it to 32 bits and the first fwrite
            # into it segfaults the process — the exact class of mistake
            # that killed camserve once already, so it is spelled out.
            lib.open_memstream.restype = ctypes.c_void_p
            lib.open_memstream.argtypes = [
                ctypes.POINTER(ctypes.POINTER(ctypes.c_char)),
                ctypes.POINTER(ctypes.c_size_t)]
            lib.malloc_info.argtypes = [ctypes.c_int, ctypes.c_void_p]
            lib.malloc_info.restype = ctypes.c_int
            lib.malloc_trim.argtypes = [ctypes.c_size_t]
            lib.malloc_trim.restype = ctypes.c_int
            lib.fclose.argtypes = [ctypes.c_void_p]
            lib.free.argtypes = [ctypes.c_void_p]
            _LIBC = lib
        except (OSError, AttributeError):
            _LIBC = None
    return _LIBC


def arena_report() -> str:
    """glibc's own account of every arena, as XML. "" if not glibc.

    Safe to call from a request handler: malloc_info takes each arena's
    lock the normal way. That is the whole reason this is not a gdb
    attach — see the module docstring.
    """
    lib = _libc()
    if lib is None:
        return ""
    buf = ctypes.POINTER(ctypes.c_char)()
    size = ctypes.c_size_t(0)
    stream = lib.open_memstream(ctypes.byref(buf), ctypes.byref(size))
    if not stream:
        return ""
    try:
        lib.malloc_info(0, stream)
    finally:
        # fclose flushes AND finalizes buf/size — they are not valid
        # until it has run, so this is not merely cleanup.
        lib.fclose(stream)
    try:
        return (ctypes.string_at(buf, size.value).decode("ascii", "replace")
                if size.value else "")
    finally:
        lib.free(buf)


def heap_summary(xml_text: str) -> dict:
    """Reduce malloc_info's XML to the numbers that decide #704.

    Split out from the call so it can be tested against a fixture on any
    platform — the parser is where the mistakes live, and it should not
    need a leaking glibc process to exercise.

    `free_bytes` is fastbins + the rest of the free lists: memory the
    process has already released, which the allocator is still holding.
    `system_bytes` is what the arenas took from the kernel. The ratio
    between them is the finding.
    """
    import xml.etree.ElementTree as ET

    out: dict = {"arenas": 0, "system_bytes": 0, "free_bytes": 0,
                 "mmap_bytes": 0, "heaps": []}
    if not xml_text.strip():
        return out
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError:
        return out

    def sizes(el) -> dict:
        got: dict = {}
        for child in el:
            if child.tag in ("total", "system", "aspace"):
                try:
                    got[(child.tag, child.get("type"))] = int(
                        child.get("size", 0))
                except ValueError:
                    continue
        return got

    for heap in root.findall("heap"):
        s = sizes(heap)
        out["heaps"].append({
            "nr": heap.get("nr"),
            "system_bytes": s.get(("system", "current"), 0),
            "free_bytes": (s.get(("total", "fast"), 0)
                           + s.get(("total", "rest"), 0)),
        })
    out["arenas"] = len(out["heaps"])
    top = sizes(root)
    out["system_bytes"] = top.get(("system", "current"), 0)
    out["free_bytes"] = (top.get(("total", "fast"), 0)
                         + top.get(("total", "rest"), 0))
    out["mmap_bytes"] = top.get(("total", "mmap"), 0)
    return out


def read_verdict(system_bytes: int, free_bytes: int) -> str:
    """Say in words which of the two explanations the ratio supports.

    Written down rather than left to whoever reads the JSON, so the
    conclusion is reproducible and does not depend on the reader
    remembering which way round the reasoning goes.
    """
    if system_bytes <= 0:
        return "no glibc arena data — cannot say"
    ratio = free_bytes / system_bytes
    if ratio >= FREE_RATIO_HOLES:
        return (f"{ratio:.0%} of the heap is FREE but not returned to the "
                f"OS — the leak is allocator-level (fragmentation, or "
                f"corrupted chunks that defeat coalescing), not live data. "
                f"Confirm with ?trim=1; then try MALLOC_CHECK_=3.")
    if ratio <= FREE_RATIO_LIVE:
        return (f"only {ratio:.0%} of the heap is free — the memory is "
                f"genuinely still ALLOCATED. Something on the tags path "
                f"never frees it; the fix is code, not an allocator knob.")
    return (f"{ratio:.0%} free — between the cuts, neither reading is safe "
            f"yet. Let it run longer and sample again.")


def malloc_trim() -> int | None:
    """Ask glibc to return free heap to the OS. None if not glibc.

    Returns 1 if it released anything, 0 if it could not. Safe here and
    emphatically not safe from gdb — see the module docstring.
    """
    lib = _libc()
    return None if lib is None else int(lib.malloc_trim(0))


def memory_report(trim: bool = False, raw: bool = False) -> dict:
    """One JSON-able answer to "where is the memory, and is it live?".

    This is what camserve's /debug/memory serves.
    """
    import tracemalloc

    xml_text = arena_report()
    summary = heap_summary(xml_text)
    mb = 1024.0 * 1024.0
    report: dict = {
        "rss_mb": round(rss_mb(), 1),
        "glibc": _libc() is not None,
        "arenas": summary["arenas"],
        "system_mb": round(summary["system_bytes"] / mb, 1),
        "free_mb": round(summary["free_bytes"] / mb, 1),
        "mmap_mb": round(summary["mmap_bytes"] / mb, 1),
        "free_pct": (round(100.0 * summary["free_bytes"]
                           / summary["system_bytes"], 1)
                     if summary["system_bytes"] else None),
        "verdict": read_verdict(summary["system_bytes"],
                                summary["free_bytes"]),
        "tracemalloc_on": tracemalloc.is_tracing(),
        "heaps": [{"nr": h["nr"],
                   "system_mb": round(h["system_bytes"] / mb, 1),
                   "free_mb": round(h["free_bytes"] / mb, 1)}
                  for h in summary["heaps"]],
    }
    if not report["glibc"]:
        report["note"] = ("not glibc — malloc_info is unavailable here. "
                          "RSS is still reported. Run this on cell1.")
    if trim:
        before = rss_mb()
        released = malloc_trim()
        after = rss_mb()
        report["trim"] = {
            "released": released,
            "rss_before_mb": round(before, 1),
            "rss_after_mb": round(after, 1),
            "returned_mb": round(before - after, 1),
        }
    if raw:
        report["raw_xml"] = xml_text
    return report


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

    # Sampled every time, because free-vs-live over the whole run is a
    # time series and the endpoint only ever gives a spot reading. It is
    # also the cheap half: malloc_info walks bin headers, not objects.
    arenas = heap_summary(arena_report())
    if arenas["system_bytes"]:
        lines.append(
            f"  glibc heap: system {arenas['system_bytes'] / 1e6:.0f} MB, "
            f"free-in-bins {arenas['free_bytes'] / 1e6:.0f} MB "
            f"({100.0 * arenas['free_bytes'] / arenas['system_bytes']:.1f}%), "
            f"mmap {arenas['mmap_bytes'] / 1e6:.0f} MB, "
            f"{arenas['arenas']} arena(s)")

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

    print("\nthe glibc arena reading — free-but-held vs genuinely live")
    # Parsed from a fixture, not from this process, so the parser is
    # exercised identically on cell1 and on Kyle's Windows desk. A live
    # heap also cannot be made to hold a KNOWN ratio on demand, and a
    # test that only asserts "some number came back" would not have
    # caught a totals-vs-per-heap mix-up.
    fixture = """<malloc version="1">
<heap nr="0">
<sizes><size from="17" to="32" total="64" count="2"/></sizes>
<total type="fast" count="2" size="64"/>
<total type="rest" count="3" size="900000"/>
<system type="current" size="1000000"/>
<system type="max" size="1000000"/>
</heap>
<heap nr="1">
<sizes/>
<total type="fast" count="0" size="0"/>
<total type="rest" count="1" size="100"/>
<system type="current" size="200000"/>
<system type="max" size="200000"/>
</heap>
<total type="fast" count="2" size="64"/>
<total type="rest" count="4" size="900100"/>
<total type="mmap" count="1" size="50000"/>
<system type="current" size="1200000"/>
<system type="max" size="1200000"/>
</malloc>"""
    got = heap_summary(fixture)
    check("every arena is counted", got["arenas"] == 2, str(got["arenas"]))
    check("system bytes come from the TOTALS, not one arena",
          got["system_bytes"] == 1_200_000, str(got["system_bytes"]))
    check("free bytes are fastbins PLUS the rest of the free lists",
          got["free_bytes"] == 900_164, str(got["free_bytes"]))
    check("mmap'd memory is kept separate from arena memory",
          got["mmap_bytes"] == 50_000, str(got["mmap_bytes"]))
    check("...and each arena is reported on its own, so one bad arena "
          "cannot hide behind a healthy total",
          [h["system_bytes"] for h in got["heaps"]] == [1_000_000, 200_000])

    # REFUSALS. This parser runs inside a diagnostic endpoint on a server
    # that must not fall over while being asked why it is unwell.
    check("garbage XML yields zeros, not an exception",
          heap_summary("<not-xml")["system_bytes"] == 0)
    check("...and so does the empty string a non-glibc host returns",
          heap_summary("")["arenas"] == 0)

    check("a mostly-free heap is read as holes, not as live data",
          "allocator-level" in read_verdict(1_200_000, 900_164))
    check("...and a mostly-live heap is read the OTHER way, which is the "
          "distinction the whole endpoint exists to make",
          "genuinely still ALLOCATED" in read_verdict(1_200_000, 1_000))
    check("...and a ratio between the cuts refuses to call it",
          "neither reading is safe" in read_verdict(1_000_000, 100_000))
    check("no arena data refuses to guess",
          "cannot say" in read_verdict(0, 0))

    print("\nthe live call (glibc only)")
    on_glibc = _libc() is not None
    report = memory_report(trim=False)
    check("a report always comes back, on any platform",
          "rss_mb" in report and "glibc" in report,
          f"glibc={report['glibc']}")
    if on_glibc:
        xml = arena_report()
        check("malloc_info returns glibc's own XML", "<malloc" in xml,
              f"{len(xml)} bytes")
        check("...and this process's heap parses to something real",
              heap_summary(xml)["system_bytes"] > 0)
        check("the report carries a verdict, not just numbers",
              bool(report["verdict"]))
        trimmed = memory_report(trim=True)
        check("malloc_trim runs IN-PROCESS without dying — the whole point "
              "after the gdb attach segfaulted camserve",
              trimmed["trim"]["released"] in (0, 1),
              f"released={trimmed['trim']['released']}, "
              f"returned {trimmed['trim']['returned_mb']} MB")
    else:
        check("off-glibc it says so plainly instead of pretending",
              report["glibc"] is False and "not glibc" in report["note"])

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
