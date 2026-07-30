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
killed a camserve that had spent 3 h 42 m growing to 1.26 GB. An
instrument that needs a restart to exist cannot diagnose a leak that
takes hours to become visible, because the restart is the one thing that
destroys the evidence. So this ships on the running server instead.

BUT NOT BECAUSE GDB WAS THE PROBLEM — that was the first explanation and
it is wrong. `gdb -p ... call` is an INFERIOR CALL: it runs in the
target process, with the target's own libc, taking the same arena mutex
an in-process call takes. A thread merely holding that mutex would have
DEADLOCKED. It faulted instead, which means it acquired the lock and then
walked a bad pointer — so the SIGSEGV is evidence ABOUT THE HEAP, not
about gdb.

That matters here because `malloc_info` is not a summary read. glibc
walks the free lists to build it (`p = REVEAL_PTR(p->fd)` over every
fastbin, `r = r->fd` over every bin), so:

  **on a corrupt heap this endpoint dies on contact, in-process, and
  cannot tell you that is why.** That is the blind spot, and it is not
  the one an earlier version of this file claimed.

`?trim=1` is strictly worse: `mtrim` calls `malloc_consolidate`, which
WRITES through `fd`/`bk` while unlinking. Treat it as an operation that
may kill the process, fire it deliberately and last, and take the
read-only numbers first. It is opt-in for that reason, not merely for
tidiness.

Every successful read is therefore also a corruption test. The #704 soak
walked these lists every 15 s for an hour without faulting, which is
what actually argued corruption down — no separate run was needed.

NOT A FOLLOW-UP: `MALLOC_CHECK_=3`. glibc 2.34 moved malloc debugging
out of libc entirely; the tunable is inert without
`LD_PRELOAD=libc_malloc_debug.so.0`. cell1 runs 2.39, so a run under it
would abort nothing and be written up as "no corruption detected" — a
null result manufactured by the experiment not existing. It also cannot
see fragmentation, which is what a "mostly free" reading is about.

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

# When the free lists are THIS long with chunks THIS small, the heap is
# fragmented regardless of what the byte ratio says — and that is a
# separate finding with a separate fix. Calibrated against the #704
# measurement, where 174,505 free chunks averaging 156 B coexisted with a
# byte ratio of 15%, i.e. squarely inside the "ambiguous" band. The count
# was unambiguous the whole time.
MANY_CHUNKS = 50_000
SMALL_CHUNK_BYTES = 512     # mean below this = tiny holes, not usable space

# Below this share of anonymous memory, glibc's arenas do not account for
# enough of the process for the ratio above to mean anything, and
# read_verdict refuses instead of pronouncing.
#
# CALIBRATED, not guessed. The first draft used 0.90 and fired immediately
# on a freshly started, perfectly healthy camserve: arenas 9.4 MB + mmap
# 62.4 MB against 79.8 MB anonymous is 89.9%, so the very first real
# reading was a false alarm. The missing ~10% is thread stacks and
# CPython's own pool arenas, which are always there and are not a finding.
# 0.75 still catches the case this exists for — a leak living entirely
# outside the arenas, where coverage collapses rather than dips.
MIN_COVERAGE = 0.75

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
            lib.free.restype = None          # it returns void, not int
            _LIBC = lib
        except (OSError, AttributeError):
            _LIBC = None
    return _LIBC


def arena_report() -> str:
    """glibc's own account of every arena, as XML. "" if not glibc.

    Takes each arena's lock the normal way, and glibc prints only after
    releasing it, so this cannot self-deadlock against the capture
    threads. It is NOT risk-free: it walks the free lists, so a corrupt
    heap faults here. See the module docstring — that hazard is the
    finding, not an oversight.

    Raises OSError rather than returning "" when glibc IS present but the
    call fails; "" means "no glibc" and the two must not be confusable.
    """
    lib = _libc()
    if lib is None:
        return ""
    buf = ctypes.POINTER(ctypes.c_char)()
    size = ctypes.c_size_t(0)
    stream = lib.open_memstream(ctypes.byref(buf), ctypes.byref(size))
    if not stream:
        raise OSError(f"open_memstream failed "
                      f"(errno {ctypes.get_errno()}) — this is NOT the same "
                      f"as 'not glibc', and returning \"\" here would have "
                      f"been reported as 'no arena data'")
    rc = -1
    try:
        rc = lib.malloc_info(0, stream)
    finally:
        # fclose flushes AND finalizes buf/size — they are not valid
        # until it has run, so this is not merely cleanup. The free()
        # belongs in this same finally: with it after the try, an
        # exception from malloc_info left the memstream buffer allocated
        # forever — a leak inside the leak detector.
        lib.fclose(stream)
        text = (ctypes.string_at(buf, size.value).decode("ascii", "replace")
                if size.value else "")
        # size.value == 0 also covers the case where a failed final
        # realloc inside _IO_mem_finish leaves buf NULL. Do not "simplify"
        # this guard away: without it that path is string_at(NULL, n).
        lib.free(buf)
    if rc != 0:
        raise OSError(f"malloc_info returned {rc} — the document may be "
                      f"truncated, which must not be reported as absent data")
    return text


def heap_summary(xml_text: str) -> dict:
    """Reduce malloc_info's XML to the numbers that decide #704.

    Split out from the call so it can be tested against REAL captured
    glibc output on any platform — the parser is where the mistakes live,
    and it should not need a leaking glibc process to exercise.

    THE FIELDS, and why there are this many. The first version of this
    reported `free_bytes` alone and that number could not answer the
    question, twice over:

    `free_bytes` is glibc's fast + rest totals, and **`rest` includes each
    arena's top chunk** (glibc seeds `avail = chunksize(av->top)` before
    summing the bins — added in 2.31 for BZ #24026). Top-chunk slack is
    not fragmentation: `malloc_trim` hands it straight back. So the honest
    fragmentation figure is `bins_bytes`, summed from the `<sizes>`
    elements, and `top_bytes` is the difference. On cell1 at 133 MB of
    arena that difference was 8.0 MB of the 22.9 MB reported free.

    `free_chunks` is the number that actually found the leak. Free BYTES
    sat in a flat 15-27 MB band for 40 minutes while free CHUNKS climbed
    from 141,114 to 174,505 — because each leaked live block splits what
    would otherwise coalesce into one span, so the COUNT tracks the leak
    while the total does not. It also explains the CPU: glibc searches
    bins on allocation, so a long free list makes every malloc slower,
    and per-frame cost doubled from 46 ms to 90 ms over that window.
    """
    import xml.etree.ElementTree as ET

    out: dict = {"arenas": 0, "system_bytes": 0, "free_bytes": 0,
                 "bins_bytes": 0, "top_bytes": 0, "free_chunks": 0,
                 "mmap_bytes": 0, "heaps": []}
    if not xml_text.strip():
        return out
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError:
        return out

    def totals(el) -> dict:
        """(tag, type) -> (size, count) for this element's own children."""
        got: dict = {}
        for child in el:
            if child.tag in ("total", "system", "aspace"):
                try:
                    got[(child.tag, child.get("type"))] = (
                        int(child.get("size", 0)),
                        int(child.get("count", 0)))
                except ValueError:
                    continue
        return got

    def bins(el) -> tuple[int, int]:
        """Bytes and chunk count actually sitting in this arena's bins.

        Summed from <sizes><size .../></sizes>, which is the only place
        glibc reports per-bin occupancy and therefore the only way to
        separate real free chunks from the top chunk.
        """
        b = n = 0
        for size in el.findall("sizes/size"):
            try:
                b += int(size.get("total", 0))
                n += int(size.get("count", 0))
            except ValueError:
                continue
        return b, n

    def free_of(t: dict) -> tuple[int, int]:
        fs, fc = t.get(("total", "fast"), (0, 0))
        rs, rc = t.get(("total", "rest"), (0, 0))
        return fs + rs, fc + rc

    for heap in root.findall("heap"):
        t = totals(heap)
        free_b, free_n = free_of(t)
        bin_b, bin_n = bins(heap)
        out["heaps"].append({
            "nr": heap.get("nr"),
            "system_bytes": t.get(("system", "current"), (0, 0))[0],
            "free_bytes": free_b,
            "bins_bytes": bin_b,
            "free_chunks": bin_n,
        })
    out["arenas"] = len(out["heaps"])
    top = totals(root)
    out["system_bytes"] = top.get(("system", "current"), (0, 0))[0]
    out["free_bytes"], _ = free_of(top)
    out["mmap_bytes"] = top.get(("total", "mmap"), (0, 0))[0]
    # Per-arena sums, not the document totals: glibc's top-level <total>
    # has no <sizes> block to sum, so bins and chunk counts only exist
    # per arena.
    out["bins_bytes"] = sum(h["bins_bytes"] for h in out["heaps"])
    out["free_chunks"] = sum(h["free_chunks"] for h in out["heaps"])
    out["top_bytes"] = max(0, out["free_bytes"] - out["bins_bytes"])
    return out


def anon_bytes() -> int:
    """Anonymous resident memory, from smaps_rollup. 0 if unavailable.

    The right denominator for "does glibc account for the growth?". RSS
    is the WRONG one — it includes ~52 MB of mapped shared libraries here,
    which no allocator statistic will ever explain, so comparing against
    RSS makes a healthy process look unaccounted-for.
    """
    try:
        for line in Path("/proc/self/smaps_rollup").read_text().splitlines():
            if line.startswith("Anonymous:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return 0


def read_verdict(summary: dict, anon: int = 0) -> str:
    """Say in words which explanation the numbers support.

    Written down rather than left to whoever reads the JSON, so the
    conclusion is reproducible and does not depend on the reader
    remembering which way round the reasoning goes.

    THE COVERAGE GATE COMES FIRST, and it is the branch this function
    was missing. The question has three answers, not two — holes, live,
    or "the growth is somewhere glibc cannot see" (direct mmap by a
    library, thread stacks, CPython's own pool arenas). Without the gate
    this happily pronounced on 7 MB of arena inside a 1.26 GB process.

    Fragmentation is judged on `bins_bytes`, NOT `free_bytes`: the latter
    includes top-chunk slack, which trims away trivially and is not a
    fault. And a large `free_chunks` count with a small mean size is
    called out separately, because that shape — many tiny holes wedged
    between live blocks — has a different fix from bulk unreturned
    memory, and it is what #704 turned out to be.

    For a spot reading this is all one can say. The stronger statistic is
    the DERIVATIVE across two samples (Δfree / Δsystem: if new memory
    arrives mostly free it is holes, mostly used it is live), which needs
    the time series the sampler CSV and the periodic log carry.
    """
    system = summary.get("system_bytes", 0)
    if system <= 0:
        return "no glibc arena data — cannot say"

    parts = []
    accounted = system + summary.get("mmap_bytes", 0)
    if anon > 0 and accounted < MIN_COVERAGE * anon:
        return (f"glibc accounts for only {100.0 * accounted / anon:.0f}% of "
                f"{anon / 1048576:.0f} MB anonymous — the growth is somewhere "
                f"malloc_info cannot see (direct mmap, thread stacks, "
                f"CPython pool arenas). Do NOT read the ratio below as the "
                f"whole story; diff /proc/PID/smaps_rollup instead.")

    bins = summary.get("bins_bytes", 0)
    chunks = summary.get("free_chunks", 0)
    ratio = bins / system
    if ratio >= FREE_RATIO_HOLES:
        parts.append(f"{ratio:.0%} of the heap sits FREE in the bins — "
                     f"allocator-level, not live data")
    elif ratio <= FREE_RATIO_LIVE:
        parts.append(f"only {ratio:.0%} of the heap is free in bins — the "
                     f"memory is genuinely still ALLOCATED, so the fix is "
                     f"code, not an allocator knob")
    else:
        parts.append(f"{ratio:.0%} free in bins — between the cuts; take a "
                     f"second sample and read the trend, not this number")

    if chunks >= MANY_CHUNKS and bins / max(chunks, 1) < SMALL_CHUNK_BYTES:
        parts.append(f"and it is FRAGMENTED: {chunks:,} free chunks averaging "
                     f"{bins / max(chunks, 1):.0f} B, which is live blocks "
                     f"wedged between them preventing coalescing — expect "
                     f"allocation to get slower as this grows")

    top = summary.get("top_bytes", 0)
    if top > 0.1 * system:
        parts.append(f"({top / 1048576:.0f} MB of the reported free total is "
                     f"top-chunk slack, excluded above — that trims away and "
                     f"is not a fault)")
    return "; ".join(parts) + "."


def malloc_trim() -> int | None:
    """Ask glibc to return free heap to the OS. None if not glibc.

    Returns 1 if it released anything, 0 if it could not.

    THE MOST DANGEROUS CALL IN THIS FILE. `mtrim` runs
    `malloc_consolidate` first, which UNLINKS chunks — writing through
    `fd`/`bk` — and then walks every bin. On a corrupt heap that is a
    SIGSEGV or a `malloc_printerr` abort, and being in-process rather
    than under gdb does not help. Expect it to be fatal sometimes; take
    every read-only measurement before calling it.
    """
    lib = _libc()
    return None if lib is None else int(lib.malloc_trim(0))


def memory_report(trim: bool = False, raw: bool = False) -> dict:
    """One JSON-able answer to "where is the memory, and is it live?".

    This is what camserve's /debug/memory serves.

    `free_chunks` and `mean_chunk_bytes` are here because they are what
    solved #704 while `free_mb` said "ambiguous" for forty minutes.
    `accounted_pct` is here so a verdict can never again be read as a
    statement about the whole process when it covers a fraction of it.
    """
    import tracemalloc

    xml_text = arena_report()
    summary = heap_summary(xml_text)
    anon = anon_bytes()
    mb = 1024.0 * 1024.0
    system = summary["system_bytes"]
    chunks = summary["free_chunks"]
    report: dict = {
        "rss_mb": round(rss_mb(), 1),
        "anon_mb": round(anon / mb, 1),
        "glibc": _libc() is not None,
        "arenas": summary["arenas"],
        "system_mb": round(system / mb, 1),
        "free_mb": round(summary["free_bytes"] / mb, 1),
        "bins_mb": round(summary["bins_bytes"] / mb, 1),
        "top_mb": round(summary["top_bytes"] / mb, 1),
        "live_mb": round((system - summary["free_bytes"]) / mb, 1),
        "free_chunks": chunks,
        "mean_chunk_bytes": (round(summary["bins_bytes"] / chunks)
                             if chunks else None),
        "mmap_mb": round(summary["mmap_bytes"] / mb, 1),
        # Against BINS, not the fast+rest total: top-chunk slack is not
        # fragmentation and counting it as such is what produced a
        # confident wrong reading.
        "bins_pct": (round(100.0 * summary["bins_bytes"] / system, 1)
                     if system else None),
        # Anonymous, never RSS — RSS carries ~52 MB of mapped shared
        # libraries here that no allocator statistic will ever explain.
        "accounted_pct": (round(100.0 * (system + summary["mmap_bytes"])
                                / anon, 1) if anon else None),
        "verdict": read_verdict(summary, anon),
        "tracemalloc_on": tracemalloc.is_tracing(),
        "heaps": [{"nr": h["nr"],
                   "system_mb": round(h["system_bytes"] / mb, 1),
                   "free_mb": round(h["free_bytes"] / mb, 1),
                   "bins_mb": round(h["bins_bytes"] / mb, 1),
                   "free_chunks": h["free_chunks"]}
                  for h in summary["heaps"]],
    }
    if not report["glibc"]:
        report["note"] = ("not glibc — malloc_info is unavailable here. "
                          "RSS is still reported. Run this on cell1.")
    if trim:
        report["trim"] = _trim_report()
    if raw:
        report["raw_xml"] = xml_text
    return report


def _trim_report() -> dict:
    """Run malloc_trim and say honestly what happened.

    Two failure modes are reported rather than papered over. Off glibc
    there is nothing to trim, and emitting zeros would read as "it ran
    and released nothing". And rss_mb()'s 0.0 is a FAILURE SENTINEL that
    is also a valid value, so a failed /proc read would otherwise
    manufacture a returned_mb of plus or minus the whole heap and present
    it as measurement.
    """
    if _libc() is None:
        return {"released": None, "note": "not glibc — nothing to trim"}
    before = rss_mb()
    released = malloc_trim()
    after = rss_mb()
    if before <= 0 or after <= 0:
        return {"released": released,
                "note": "trim ran, but RSS could not be read either side — "
                        "no returned_mb is reported rather than a fabricated "
                        "one"}
    return {"released": released,
            "rss_before_mb": round(before, 1),
            "rss_after_mb": round(after, 1),
            "returned_mb": round(before - after, 1)}


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

    # Sampled every time, because the DERIVATIVE across samples is a
    # stronger statistic than any spot reading: memory arriving mostly
    # free means holes, mostly used means a live leak, and no threshold
    # is needed to say so. The endpoint can only ever give one point.
    #
    # Cost is real but bounded — measured at 19-24 ms per call on cell1
    # at 133 MB of arena with 174,000 free chunks, and it grows with the
    # chunk count, so re-measure if this ever looks like a stall. That is
    # why it is not gated the way the gc walk is: 20 ms every 300 s is
    # 0.007%, where gc.get_objects() was 175 ms and visible in the fps.
    try:
        arenas = heap_summary(arena_report())
    except OSError as exc:
        arenas = {"system_bytes": 0}
        lines.append(f"  glibc heap: unreadable ({exc})")
    if arenas["system_bytes"]:
        chunks = arenas["free_chunks"]
        lines.append(
            f"  glibc heap: system {arenas['system_bytes'] / 1e6:.0f} MB, "
            f"live {(arenas['system_bytes'] - arenas['free_bytes']) / 1e6:.0f} "
            f"MB, in-bins {arenas['bins_bytes'] / 1e6:.0f} MB in "
            f"{chunks:,} chunks "
            f"(mean {arenas['bins_bytes'] / max(chunks, 1):.0f} B), "
            f"top {arenas['top_bytes'] / 1e6:.0f} MB, "
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

    print("\nthe glibc arena reading — parsed from REAL captured output")
    # The fixture is 9.2 kB of genuine malloc_info output captured from
    # the leaking camserve on cell1 (14 arenas, 133 MB, mid-leak), NOT
    # hand-written. The first version of this test WAS hand-written from
    # the author's belief about the format, and it was wrong in two ways
    # that mattered: real output carries <aspace type="subheaps"> and it
    # OMITS <total type="fast"> entirely when the fastbins are empty. A
    # fixture invented from assumption cannot catch a parser built from
    # the same assumption.
    real = Path(__file__).parent / "testdata" / "malloc_info_real.xml"
    if not real.exists():
        check("the real captured fixture is present", False, str(real))
    else:
        got = heap_summary(real.read_text(encoding="utf-8"))
        check("every arena is counted", got["arenas"] == 14,
              str(got["arenas"]))
        check("system bytes come from the TOTALS, not one arena",
              got["system_bytes"] == 133_304_320, str(got["system_bytes"]))
        check("a MISSING <total type='fast'> is absent, not zero-crashing — "
              "real glibc omits it when the fastbins are empty",
              got["free_bytes"] == 22_856_524, str(got["free_bytes"]))
        check("mmap'd memory is kept separate from arena memory",
              got["mmap_bytes"] == 57_147_392, str(got["mmap_bytes"]))
        check("free CHUNKS are counted, which is the signal that found "
              "#704 while the byte ratio called it ambiguous",
              got["free_chunks"] == 141_096, str(got["free_chunks"]))
        # The finding this fixture exists to pin down, in real data.
        h13 = [h for h in got["heaps"] if h["nr"] == "13"][0]
        check("top-chunk slack is NOT counted as fragmentation: heap 13 "
              "reports 132,144 free bytes with ZERO chunks in its bins, "
              "which is one untouched top chunk and nothing else",
              h13["free_bytes"] == 132_144 and h13["bins_bytes"] == 0)
        check("...so across the capture 7.6 MB of the 21.8 MB 'free' is "
              "slack, separated out instead of blamed on fragmentation",
              got["top_bytes"] == 7_987_860 and got["bins_bytes"] == 14_868_664,
              f"top {got['top_bytes']}, bins {got['bins_bytes']}")
        check("...and each arena is reported on its own, so one leaking "
              "arena cannot hide behind a healthy total",
              max(got["heaps"], key=lambda h: h["system_bytes"])["nr"] == "10")

    # REFUSALS. This parser runs inside a diagnostic endpoint on a server
    # that must not fall over while being asked why it is unwell.
    check("garbage XML yields zeros, not an exception",
          heap_summary("<not-xml")["system_bytes"] == 0)
    check("...and so does the empty string a non-glibc host returns",
          heap_summary("")["arenas"] == 0)

    def verdict(system, bins, chunks=0, top=0, mmap=0, anon=0):
        return read_verdict({"system_bytes": system, "bins_bytes": bins,
                             "free_chunks": chunks, "top_bytes": top,
                             "mmap_bytes": mmap}, anon)

    check("a mostly-free heap is read as holes, not as live data",
          "allocator-level" in verdict(1_200_000, 900_164))
    check("...and a mostly-live heap is read the OTHER way, which is the "
          "distinction the whole endpoint exists to make",
          "genuinely still ALLOCATED" in verdict(1_200_000, 1_000))
    check("...and a ratio between the cuts refuses to call it",
          "read the trend" in verdict(1_000_000, 100_000))
    check("no arena data refuses to guess", "cannot say" in verdict(0, 0))
    # The branch that was missing entirely, and that #704 turned out to be.
    check("many tiny free chunks are called out as FRAGMENTED even when "
          "the byte ratio is in the ambiguous band — the #704 shape",
          "FRAGMENTED" in verdict(155_000_000, 22_000_000, chunks=174_505))
    check("...and a heap with the same bytes in FEW chunks is not, "
          "because large free spans are reusable and tiny ones are not",
          "FRAGMENTED" not in verdict(155_000_000, 22_000_000, chunks=40))
    # The coverage gate: the third answer the two-way test could not give.
    check("when glibc accounts for only a fraction of anonymous memory it "
          "REFUSES to pronounce, instead of describing 7 MB of arena as "
          "though it explained a 1.2 GB process",
          "cannot see" in verdict(7_000_000, 300_000, anon=1_200_000_000))
    check("...and with good coverage it does pronounce",
          "cannot see" not in verdict(1_200_000, 1_000, mmap=100_000,
                                      anon=1_300_000))
    # The exact reading from a freshly started, HEALTHY camserve on cell1,
    # which the first cut of this gate flagged at 89.9%. A gate that cries
    # wolf on a clean process buries the signal it exists to raise, so the
    # numbers that caused the false alarm are pinned here rather than the
    # threshold being quietly nudged.
    check("a healthy idle camserve does NOT trip the coverage gate — its "
          "missing ~10% is thread stacks and CPython pool arenas, not a leak",
          "cannot see" not in verdict(9_400_000, 700_000, chunks=206,
                                      mmap=62_400_000, anon=79_800_000))
    check("top-chunk slack is disclosed rather than silently dropped",
          "slack" in verdict(1_000_000, 10_000, top=500_000))

    print("\nthe live call (glibc only)")
    on_glibc = _libc() is not None
    report = memory_report(trim=False)
    check("a report always comes back, on any platform",
          "rss_mb" in report and "glibc" in report,
          f"glibc={report['glibc']}")
    check("the default really is read-only — no trim unless asked. This "
          "belongs here and not only in camserve, because it is a property "
          "of the report, not of query-string parsing",
          "trim" not in report)
    if on_glibc:
        xml = arena_report()
        check("malloc_info returns glibc's own XML", "<malloc" in xml,
              f"{len(xml)} bytes")
        check("...and this process's heap parses to something real",
              heap_summary(xml)["system_bytes"] > 0)
        check("the report carries a verdict, not just numbers",
              bool(report["verdict"]))
        check("...and says what fraction of anonymous memory it accounts "
              "for, so the verdict's scope is never implied",
              report["accounted_pct"] is not None,
              f"{report['accounted_pct']}% of {report['anon_mb']} MB anon")
        trimmed = memory_report(trim=True)
        check("malloc_trim runs IN-PROCESS and reports what it released",
              trimmed["trim"]["released"] in (0, 1),
              f"released={trimmed['trim']['released']}")
    else:
        check("off-glibc it says so plainly instead of pretending",
              report["glibc"] is False and "not glibc" in report["note"])
        # Paired with the above: the previous version emitted zeros for
        # rss_before/after/returned off-glibc, which reads as "trim ran
        # and freed nothing" on a platform that has no malloc_trim.
        off = memory_report(trim=True)["trim"]
        check("...and a trim off-glibc says 'nothing to trim' rather than "
              "reporting a fabricated 0 MB returned",
              off["released"] is None and "returned_mb" not in off)

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
