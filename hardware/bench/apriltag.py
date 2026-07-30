"""AprilTag detection against the SYSTEM libapriltag (plan #713.5).

Replaces `pupil-apriltags`, which is gone from pyproject.toml. That package
is at its newest release (1.0.4.post11) and vendors AprilTag 3.1.x from
2019, in which `quad_segment_maxima()` allocates two buffers and then
returns early over both frees:

    366  int    *maxima      = malloc(sizeof(int)   *sz);
    367  double *maxima_errs = malloc(sizeof(double)*sz);
    380  if (nmaxima < 4)
    381      return 0;              <-- leaks 12*sz bytes
    403  free(maxima_errs);   461  free(maxima);

Every candidate cluster that fails the four-maxima test leaks, and most of
them fail. Measured on cell1 at **11.19 kB per detect() call**, dead linear
over 400 calls, on an image with real clusters. Upstream fixed it in
5ed87e741c (released in v3.1.3, six weeks after the snapshot Pupil
vendored); Pupil's fork is dormant with no leak issue filed, so no release
is coming. Ubuntu ships 3.4.5, which has that fix and six years of others.

WHY WE HAND-ROLL THE BINDING. The whole surface this repo uses is three
attributes — `tag_id`, `center`, `corners` — plus a constructor and
`detect()`. Ubuntu's package is the C library only; there is no system
Python binding. A ctypes wrapper over a clean, stable C API is exactly the
kind of thing that gets written rather than depended on (see Kyle's bar:
a dependency should do something MESSY we don't want to be distracted by).
This file is that wrapper, and it is smaller than the diff would have been
to patch and rebuild the old one.

TWO HAZARDS, both of which are why this file is commented the way it is:

1. `apriltag_detector_add_family` IS NOT AN EXPORTED SYMBOL. It is a
   `static inline` in apriltag.h that calls `apriltag_detector_add_family_bits(
   td, fam, 2)`. Looking it up on the .so fails at import time; the real
   symbol is the _bits one.

2. `apriltag_detector_detect` returns a zarray the CALLER owns, and the
   whole point of this change is not leaking. `apriltag_detections_destroy`
   is called in a `finally`, so an exception while reading fields cannot
   leak the detections we were reading.

PIXEL COORDINATES: THESE ARE OpenCV + 0.5 PX. READ THIS BEFORE CALIBRATING.
--------------------------------------------------------------------------
AprilTag changed its coordinate convention in cb522aec14, first released in
**v3.4.3** (2025-02-16): *"the convention that (0,0) be the left top corner
of the first pixel is adopted"*. The pupil-apriltags build this replaced
vendored 3.1.x, which predates that change, so THE NUMBERS MOVED when the
library was swapped.

OpenCV uses the opposite convention — an integer coordinate is a pixel
CENTRE, and the image's top-left corner is (-0.5, -0.5). So every `center`
and `corners` value here is **OpenCV coordinates plus 0.5 px in both axes**.

On the bench camera (fx ~1290 px, 675 mm mount) that is ~0.39 mrad,
~0.26 mm laterally. Small, but SYSTEMATIC AND DIRECTIONAL, which is the
error class a hand-eye solve silently absorbs into the wrong parameter
instead of showing as residual.

  Harmless if intrinsics AND pose both come from AprilTag corners — the
  offset lands in cx, cy and cancels.

  A real bias the moment sources are mixed: chessboard corners from
  cv2.findChessboardCorners, any .npz intrinsics captured in the
  pupil-apriltags era, or any recorded pixel reference from before this
  change.

Pick one convention, convert at the boundary, and pin it in a test. #713.8
(hand-eye) is the first consumer that will care.

NOT EXPOSED, deliberately: `homography` (the 3x3 H) and estimate_tag_pose /
pose_R / pose_t / pose_err. Nothing in this repo used them and cv2.solvePnP
on the four corners is the better route, but _Detection.H is declared so
re-exposing it is cheap if 713.8 wants it. A decision taken, not a gap.

STRUCT LAYOUTS ARE PINNED TO 3.4.5 and were read out of that tag's headers
rather than assumed — apriltag.h, common/image_types.h, common/zarray.h.
The old binding declared `refine_edges` and `debug` as `int` where 3.4.5
has them as `bool`; that happened to still work because the shrink is
absorbed by alignment padding, which is luck, not compatibility. Anything
here that reads a field must be re-checked against a new major version.

    uv run python -m hardware.bench.apriltag selftest
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
from pathlib import Path

import numpy as np

from hardware.errors import BenchError

# The family this cell uses everywhere. 36h11 is the default for a reason:
# the largest Hamming distance of the standard families, so a misread is
# far less likely than with 25h9 or 16h5 on a printed sheet.
TAG_FAMILY = "tag36h11"

# Families libapriltag exports a create/destroy pair for. Listed rather
# than accepting any string, so a typo is a clear error instead of a
# confusing dlsym failure.
FAMILIES = ("tag36h11", "tag25h9", "tag16h5", "tagCircle21h7",
            "tagCircle49h12", "tagCustom48h12", "tagStandard41h12",
            "tagStandard52h13")

# Where to look for the shared library, in order. The env var comes first
# so a specific build can be forced without editing code — useful when
# testing a patched library against the packaged one.
_ENV_VAR = "TENDWRIGHT_APRILTAG_LIB"
_SONAMES = ("libapriltag.so.3", "libapriltag.so", "libapriltag.3.dylib",
            "libapriltag.dylib", "apriltag.dll")


class _Detection(ctypes.Structure):
    """apriltag_detection_t — apriltag.h, v3.4.5.

    ctypes reproduces C's padding, so the 4 bytes after decision_margin
    (before the 8-aligned H pointer) are handled without declaring them.
    """

    _fields_ = [
        ("family", ctypes.c_void_p),        # not read; freed by the library
        ("id", ctypes.c_int),
        ("hamming", ctypes.c_int),
        ("decision_margin", ctypes.c_float),
        ("H", ctypes.c_void_p),             # matd_t*, not read here
        ("c", ctypes.c_double * 2),         # centre, pixels
        ("p", (ctypes.c_double * 2) * 4),   # corners, pixels
    ]


class _ImageU8(ctypes.Structure):
    """image_u8_t — common/image_types.h. We build it; the library reads it.

    "Only reads it" would be too strong, and the difference is a real trap.
    With quad_decimate <= 1 apriltag runs image_u8_gaussian_blur_parallel
    IN PLACE on the image it was handed, and a negative quad_sigma writes
    sharpened pixels straight back into the buffer. Since
    np.ascontiguousarray hands back the CALLER'S array when it is already
    contiguous — the normal case — the caller's frame would come back
    modified. detect() copies defensively when that combination is in play;
    at the defaults (2.0 / 0.0) the library rebuilds the image internally
    and never touches ours.
    """

    _fields_ = [
        ("width", ctypes.c_int32),
        ("height", ctypes.c_int32),
        ("stride", ctypes.c_int32),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


class _DetectorPrefix(ctypes.Structure):
    """The first six fields of apriltag_detector_t — apriltag.h, v3.4.5.

    Only a PREFIX, deliberately: everything from `qtp` onward (the
    threshold params, profiler, family list, workerpool, mutex) is never
    read or written here, so a layout change past `debug` cannot corrupt
    anything we touch. `refine_edges` and `debug` are C `bool` — one byte,
    NOT int. ctypes reproduces the padding C inserts before the 8-aligned
    `decode_sharpening`, so the offsets are 0/4/8/12/16/24.

    Detector._verify_layout checks this fits before anything is written
    through it. Change these fields and _FRESH together, never one alone.
    """

    _fields_ = [
        ("nthreads", ctypes.c_int),
        ("quad_decimate", ctypes.c_float),
        ("quad_sigma", ctypes.c_float),
        ("refine_edges", ctypes.c_bool),
        ("decode_sharpening", ctypes.c_double),
        ("debug", ctypes.c_bool),
    ]


class _ZArray(ctypes.Structure):
    """zarray_t — common/zarray.h. Holds apriltag_detection_t POINTERS, so
    el_sz is 8 and each element must be dereferenced."""

    _fields_ = [
        ("el_sz", ctypes.c_size_t),
        ("size", ctypes.c_int),
        ("alloc", ctypes.c_int),
        ("data", ctypes.POINTER(ctypes.c_char)),
    ]


class Detection:
    """One tag, with the fields this repo actually uses.

    Deliberately a plain Python object holding COPIES: the C detection is
    freed the moment detect() returns, so anything that outlived it as a
    view into that memory would be a use-after-free. numpy arrays here own
    their data.
    """

    __slots__ = ("tag_id", "center", "corners", "hamming", "decision_margin")

    def __init__(self, det: _Detection):
        self.tag_id = int(det.id)
        self.hamming = int(det.hamming)
        self.decision_margin = float(det.decision_margin)
        self.center = np.array([det.c[0], det.c[1]], dtype=np.float64)
        self.corners = np.array([[det.p[i][0], det.p[i][1]]
                                 for i in range(4)], dtype=np.float64)

    def __repr__(self) -> str:
        return (f"Detection(tag_id={self.tag_id}, "
                f"center=({self.center[0]:.1f}, {self.center[1]:.1f}), "
                f"margin={self.decision_margin:.0f})")


_LIB: ctypes.CDLL | None = None


def _load() -> ctypes.CDLL:
    """Find and bind libapriltag, or explain how to install it.

    Cached: dlopen per Detector would be wasteful, and the prototypes only
    need declaring once.
    """
    global _LIB
    if _LIB is not None:
        return _LIB

    tried: list[str] = []
    forced = os.environ.get(_ENV_VAR)
    candidates = [forced] if forced else []
    candidates += list(_SONAMES)
    found = ctypes.util.find_library("apriltag")
    if found:
        candidates.append(found)

    lib = None
    for name in candidates:
        if not name:
            continue
        tried.append(name)
        try:
            lib = ctypes.CDLL(name)
            break
        except OSError:
            continue
    if lib is None:
        raise BenchError(
            "libapriltag not found — tag detection is unavailable",
            f"install it: sudo apt install libapriltag3t64  (Ubuntu 26.04 "
            f"ships 3.4.5). Tried: {', '.join(tried)}. Override the path "
            f"with {_ENV_VAR}=/path/to/libapriltag.so. There is no Windows "
            f"package, so detection is cell1-side; the sim and bench tools "
            f"that need it must run there.")

    # A library that DLOPENED but lacks our symbols must become a
    # BenchError like any other "no usable library", not an AttributeError.
    # Callers catch BenchError to degrade gracefully — cammanager's third
    # veto, campreview's run_tool, simcam's NOT RUN — and an AttributeError
    # sails straight past all three and stops camserve from starting, which
    # is the exact outcome the veto exists to prevent. Reachable with a
    # wrong TENDWRIGHT_APRILTAG_LIB, an unrelated apriltag.dll on PATH, or
    # a stripped build.
    try:
        _declare(lib)
    except AttributeError as exc:
        raise BenchError(
            f"{getattr(lib, '_name', '?')} loaded but is missing "
            f"{exc.args[0] if exc.args else 'a required symbol'}",
            "that is not the AprilTag library, or it is a partial build. "
            f"Check {_ENV_VAR} and your PATH/ld.so config.") from exc
    _LIB = lib
    return _LIB


def _declare(lib: ctypes.CDLL) -> None:
    """Pin every prototype. Raises AttributeError if a symbol is absent."""
    lib.apriltag_detector_create.restype = ctypes.c_void_p
    lib.apriltag_detector_create.argtypes = []
    lib.apriltag_detector_destroy.restype = None
    lib.apriltag_detector_destroy.argtypes = [ctypes.c_void_p]
    # The _bits form, NOT apriltag_detector_add_family — that one is a
    # static inline in the header and does not exist in the library. 2 is
    # what the inline passes.
    lib.apriltag_detector_add_family_bits.restype = None
    lib.apriltag_detector_add_family_bits.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
    lib.apriltag_detector_detect.restype = ctypes.POINTER(_ZArray)
    lib.apriltag_detector_detect.argtypes = [ctypes.c_void_p,
                                             ctypes.POINTER(_ImageU8)]
    lib.apriltag_detections_destroy.restype = None
    lib.apriltag_detections_destroy.argtypes = [ctypes.POINTER(_ZArray)]
    for fam in FAMILIES:
        try:
            getattr(lib, f"{fam}_create").restype = ctypes.c_void_p
            getattr(lib, f"{fam}_create").argtypes = []
            getattr(lib, f"{fam}_destroy").restype = None
            getattr(lib, f"{fam}_destroy").argtypes = [ctypes.c_void_p]
        except AttributeError:
            continue        # an older library without this family is fine


def library_path() -> str:
    """Which library got loaded — for /status and for bug reports.

    Worth surfacing: "detection behaves differently on two machines" is
    otherwise a long afternoon.
    """
    _load()
    return getattr(_LIB, "_name", "unknown")


class Detector:
    """A tag detector. One per user; `detect()` is NOT thread-safe.

    The C detector holds native state and apriltag's own threadpool, so
    callers that share one across threads must serialize themselves —
    cammanager holds _DETECT_LOCK for exactly this reason.
    """

    def __init__(self, families: str = TAG_FAMILY, nthreads: int = 1,
                 quad_decimate: float = 2.0, quad_sigma: float = 0.0,
                 refine_edges: bool = True, decode_sharpening: float = 0.25,
                 debug: bool = False):
        # Set FIRST, before anything that can raise. The package this
        # replaces got this wrong: its __del__ read self.tag_detector_ptr,
        # so any failure in __init__ produced a confusing
        # "AttributeError in __del__" on top of the real error. Seen for
        # real while testing this plan.
        self._td = None
        self._fams: list[tuple[str, int]] = []
        self._lib = _load()

        # dict.fromkeys rather than set(): dedupes but keeps the order the
        # caller wrote. Registering the same family twice makes the library
        # return EVERY tag twice, which is not a crash and not an error —
        # just a silently doubled result that reads as phantom detections.
        names = list(dict.fromkeys(f.strip() for f in families.split(",")
                                   if f.strip()))
        unknown = [n for n in names if n not in FAMILIES]
        if unknown:
            raise BenchError(
                f"unknown tag family {unknown[0]!r}",
                f"known families: {', '.join(FAMILIES)}")
        if not names:
            raise BenchError("no tag family given",
                             f"pass one of: {', '.join(FAMILIES)}")

        td = self._lib.apriltag_detector_create()
        if not td:
            raise BenchError("apriltag_detector_create() returned NULL",
                             "the library is present but would not "
                             "initialise; out of memory?")
        self._td = td
        try:
            self._verify_layout(td)
            for name in names:
                try:
                    make = getattr(self._lib, f"{name}_create")
                except AttributeError as exc:
                    # _load() deliberately tolerates a library that lacks a
                    # family, so this is a state the loader anticipates and
                    # the constructor must not crash on.
                    raise BenchError(
                        f"this libapriltag has no {name} family",
                        f"loaded {library_path()}; it may be an older or "
                        f"cut-down build") from exc
                fam = make()
                if not fam:
                    raise BenchError(f"{name}_create() returned NULL", "")
                self._fams.append((name, fam))
                self._lib.apriltag_detector_add_family_bits(td, fam, 2)
            self._configure(nthreads, quad_decimate, quad_sigma,
                            refine_edges, decode_sharpening, debug)
        except BaseException:
            # A half-built detector must not leak its C allocations, and
            # must not be left for __del__ to guess about.
            self.close()
            raise

    # What apriltag_detector_create() sets before it returns, in 3.4.5.
    # Six distinct values in six different types is a usable FINGERPRINT of
    # the struct layout, which is why they are pinned here.
    _FRESH = (("nthreads", 1), ("quad_decimate", 2.0), ("quad_sigma", 0.0),
              ("refine_edges", True), ("decode_sharpening", 0.25),
              ("debug", False))

    def _verify_layout(self, td: int) -> None:
        """Refuse to write through the prefix unless it demonstrably fits.

        THE FAILURE THIS PREVENTS IS SILENT AND FATAL. _configure writes
        raw bytes at fixed offsets into a struct we do not own. If a future
        libapriltag inserts a field or narrows `decode_sharpening`, writing
        8 bytes at offset 16 lands on `timeprofile_t *tp` instead — and the
        first thing apriltag_detector_detect does is dereference it. That
        is a SIGFPE/SIGSEGV in a camera thread with no Python traceback, on
        the first detect rather than at construction.

        Nothing else here checks a version: `_SONAMES` will happily load a
        bare `libapriltag.so` symlink that points at a future .so.4, and
        library_path() reports a path, not a version. So instead of trusting
        the soname, read back the six values the library has just written
        itself and confirm they land where we expect. Costs one struct read
        per detector; turns memory corruption into an install message.
        """
        p = ctypes.cast(td, ctypes.POINTER(_DetectorPrefix)).contents
        bad = [(f, want, getattr(p, f)) for f, want in self._FRESH
               if getattr(p, f) != want]
        if bad:
            fields = "; ".join(f"{f}: expected {w!r}, read {g!r}"
                               for f, w, g in bad)
            raise BenchError(
                "libapriltag's detector struct is not the layout this "
                "binding was written against, so its tunables will NOT be "
                f"written ({fields})",
                f"loaded {library_path()}. This binding is pinned to "
                f"AprilTag 3.4.5. Re-read apriltag.h for the installed "
                f"version and update _DetectorPrefix and _FRESH together.")

    def _configure(self, nthreads, quad_decimate, quad_sigma, refine_edges,
                   decode_sharpening, debug) -> None:
        """Write the tunables through the struct prefix.

        apriltag exposes no setters; the fields are written directly. Only
        the first six are declared, in 3.4.5's types — note `refine_edges`
        and `debug` are C `bool` (1 byte) here, not `int`. Everything after
        `debug` (qtp, the profiler, the family list, the mutex) is left
        entirely alone, so a layout change beyond this prefix cannot
        corrupt anything we touch. _verify_layout has already confirmed the
        prefix itself fits.
        """
        p = ctypes.cast(self._td, ctypes.POINTER(_DetectorPrefix)).contents
        p.nthreads = int(nthreads)
        p.quad_decimate = float(quad_decimate)
        p.quad_sigma = float(quad_sigma)
        p.refine_edges = bool(refine_edges)
        p.decode_sharpening = float(decode_sharpening)
        p.debug = bool(debug)
        # Does this configuration make the library write into the buffer we
        # hand it? See _ImageU8. Computed once here rather than per frame,
        # and False at the defaults, so the common path still passes the
        # numpy buffer through with no copy.
        self._may_mutate = (float(quad_decimate) <= 1.0
                            or float(quad_sigma) != 0.0)

    def detect(self, gray: np.ndarray) -> list[Detection]:
        """Detect tags in a single-channel uint8 image.

        Returns plain Python objects; nothing points into C memory once
        this returns.
        """
        if self._td is None:
            raise BenchError("detector is closed", "construct a new one")
        if gray.ndim != 2:
            raise BenchError(
                f"detect() wants a single-channel image, got shape "
                f"{gray.shape}",
                "convert first: cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)")
        if gray.dtype != np.uint8:
            raise BenchError(f"detect() wants uint8, got {gray.dtype}", "")
        if gray.size == 0:
            # NOT pedantic input validation — this one is a process kill.
            # numpy reports strides (0, 0) for any empty 2-D array, so the
            # stride we hand C below is 0, and apriltag's threshold() passes
            # it as the ALIGNMENT to image_u8_create_alignment(), whose
            # first statement is `stride % alignment`. That is SIGFPE:
            # whole process, no traceback. Today the default
            # quad_decimate=2.0 hides it by rebuilding the image with a
            # fresh aligned stride first, but quad_decimate is a public
            # constructor argument and 1.0 feeds our stride straight in.
            raise BenchError(
                f"detect() got an empty image, shape {gray.shape}",
                "an empty frame reaches the C library as stride 0 and "
                "divides by zero inside it; check the capture first")
        # The library reads through `buf` directly, so the buffer must be
        # C-contiguous AND must outlive the call. ascontiguousarray may
        # copy; binding the result to a local keeps that copy alive for the
        # duration, which a chained expression would not.
        buf = np.ascontiguousarray(gray)
        if getattr(self, "_may_mutate", False) and buf is gray:
            # ascontiguousarray returns the CALLER'S array unchanged when it
            # is already contiguous, and this configuration blurs/sharpens
            # in place — so without this the caller's frame comes back
            # modified. Only reached off the defaults.
            buf = buf.copy()
        h, w = buf.shape
        im = _ImageU8(width=w, height=h, stride=buf.strides[0],
                      buf=buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))

        za = self._lib.apriltag_detector_detect(self._td, ctypes.byref(im))
        if not za:
            return []
        try:
            out: list[Detection] = []
            arr = za.contents
            for i in range(arr.size):
                # Each element is an apriltag_detection_t*, so read the
                # pointer out of the array and then dereference it.
                slot = ctypes.cast(
                    ctypes.addressof(arr.data.contents) + i * arr.el_sz,
                    ctypes.POINTER(ctypes.POINTER(_Detection)))
                out.append(Detection(slot.contents.contents))
            return out
        finally:
            # In a finally, not after the loop: this is the free the whole
            # plan is about, and an exception while reading fields must not
            # be able to skip it.
            self._lib.apriltag_detections_destroy(za)
            del buf         # explicit: the C side must not outlive it

    def close(self) -> None:
        """Release the C detector and families. Idempotent.

        State is cleared BEFORE the frees, not after. If a free raised with
        the old ordering, `_td` still held the destroyed pointer and __del__
        would free it again — a double-free primitive sitting behind a
        selftest that asserts "close() is idempotent, so teardown cannot
        double-free". Taking the values first makes that claim true rather
        than true-on-the-happy-path.

        Destroying the detector does NOT destroy its families: apriltag.h
        says so outright ("Not freed on apriltag_destroy; ... The user
        should ultimately destroy the tag family"), so freeing both here is
        upstream's own pattern and not a double free.
        """
        td, fams = getattr(self, "_td", None), getattr(self, "_fams", [])
        self._td, self._fams = None, []
        if td is not None:
            self._lib.apriltag_detector_destroy(td)
        for name, fam in fams:
            try:
                getattr(self._lib, f"{name}_destroy")(fam)
            except AttributeError:
                continue

    def __enter__(self) -> Detector:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self) -> None:
        # getattr with a default throughout close(), because __del__ can
        # run on an object whose __init__ raised before setting anything.
        try:
            self.close()
        except Exception:
            pass


# --------------------------------------------------------------------


def selftest() -> int:
    """Detect REAL generated tags, and prove the leak is gone.

    Both halves matter. A binding that returns no detections would pass a
    "did it crash" test, and a binding that detects perfectly while leaking
    would defeat the entire purpose of the change.
    """
    fails: list[str] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        print(f"  {'ok  ' if ok else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not ok:
            fails.append(name)

    try:
        _load()
    except BenchError as exc:
        # NOT "return 0 quietly". This file is the ONLY home of the
        # leak-regression assertion for #713.5, so a green line here on a
        # machine that ran zero checks is the worst possible output. simcam
        # already gets this right; match it.
        print(f"libapriltag unavailable: {exc}")
        print("apriltag PARTIAL — NOTHING WAS TESTED. Every check in this "
              "file needs the system library, including the leak regression "
              "gate that is #713.5's only guard.")
        print("Run it where the library is: ssh cell1 'cd ~/TendWright && "
              "uv run python -m hardware.bench.apriltag selftest'")
        return 0

    print(f"library: {library_path()}\n")
    from .tagsheet import tag_png

    import cv2

    print("it must READ REAL TAGS, not merely fail to crash")
    det = Detector(families=TAG_FAMILY)
    ids = (0, 3, 7)
    for tid in ids:
        img = cv2.imdecode(np.frombuffer(tag_png(tid), np.uint8),
                           cv2.IMREAD_GRAYSCALE)
        big = cv2.resize(img, (img.shape[1] * 12, img.shape[0] * 12),
                         interpolation=cv2.INTER_NEAREST)
        big = cv2.copyMakeBorder(big, 90, 90, 90, 90, cv2.BORDER_CONSTANT,
                                 value=255)
        found = det.detect(big)
        check(f"tag {tid} decodes to itself",
              [d.tag_id for d in found] == [tid],
              f"got {[d.tag_id for d in found]}")
        if found:
            d = found[0]
            check(f"...with a centre inside the image and 4 corners",
                  0 < d.center[0] < big.shape[1] and 0 < d.center[1] < big.shape[0]
                  and d.corners.shape == (4, 2),
                  f"centre ({d.center[0]:.0f}, {d.center[1]:.0f}), "
                  f"margin {d.decision_margin:.0f}")

    print("\nREFUSALS")
    blank = np.full((400, 400), 255, np.uint8)
    check("a blank image detects nothing", det.detect(blank) == [])
    try:
        det.detect(np.zeros((10, 10, 3), np.uint8))
        check("a colour image is refused, not silently misread", False)
    except BenchError:
        check("a colour image is refused, not silently misread", True)
    try:
        det.detect(np.zeros((10, 10), np.float32))
        check("a non-uint8 image is refused", False)
    except BenchError:
        check("a non-uint8 image is refused", True)
    try:
        Detector(families="tag99h99")
        check("an unknown family is refused", False)
    except BenchError:
        check("an unknown family is refused", True)
    d2 = Detector()
    d2.close()
    d2.close()
    check("close() is idempotent, so teardown cannot double-free", True)
    try:
        d2.detect(blank)
        check("a closed detector refuses to detect", False)
    except BenchError:
        check("a closed detector refuses to detect", True)

    print("\nTHE POINT OF THE CHANGE: it must not leak")
    # A checkerboard makes thousands of candidate clusters that mostly FAIL
    # the four-maxima test, which is precisely the path that leaked 11.19
    # kB/call in the old library. Measured in live arena bytes, because RSS
    # and Windows working set both proved unusable for this during #713.5.
    from .memprobe import arena_report, heap_summary

    def sample() -> tuple[float, int]:
        s = heap_summary(arena_report())
        return ((s["system_bytes"] - s["free_bytes"]) / 1024.0,
                s["free_chunks"])

    # REFUSE TO SCORE WITHOUT AN INSTRUMENT. heap_summary("") returns zeros,
    # so with no arena data base == 0, per == 0, and the headline check
    # prints a confident pass manufactured by the meter not existing — the
    # exact failure memprobe's own docstring argues against at length.
    try:
        probe_bytes = heap_summary(arena_report())["system_bytes"]
    except OSError as exc:
        probe_bytes, exc_note = 0, f" ({exc})"
    else:
        exc_note = ""
    if probe_bytes <= 0:
        check("the leak gate has a working instrument", False,
              f"no glibc arena data{exc_note} — this check CANNOT pass here "
              f"and must not be reported as if it did")
    else:
        yy, xx = np.mgrid[0:1080, 0:1920]
        checker = (((yy // 40) + (xx // 40)) % 2 * 255).astype(np.uint8)
        det.detect(checker)                  # warm-up: setup is not leak
        base_kb, base_chunks = sample()
        calls = 200
        for _ in range(calls):
            det.detect(checker)
        now_kb, now_chunks = sample()
        per = (now_kb - base_kb) / calls
        # 0.2, not 1.0. The measured value on 3.4.5 is 0.000, and the old
        # library was 11.19 — so a 1.0 bar would wave through 0.9 kB/call,
        # which at 30 fps is 1.6 MB/min and still fills cell1 overnight. A
        # regression gate set above the failure it is guarding against is
        # not a gate.
        check("a cluster-rich frame does not leak (old library: 11.19 "
              "kB/call)", per < 0.2, f"{per:+.3f} kB/call over {calls} calls")
        # Bytes alone were the number that lied for forty minutes on #704;
        # the chunk count is what caught it. Guard both.
        per_chunk = (now_chunks - base_chunks) / calls
        check("...and does not fragment the heap either, which is the "
              "signal bytes missed on #704",
              per_chunk < 1.0, f"{per_chunk:+.2f} free chunks/call")
    det.close()

    print()
    if fails:
        print(f"apriltag FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("apriltag OK")
    return 0


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        return selftest()
    print("usage: python -m hardware.bench.apriltag selftest")
    try:
        print(f"library: {library_path()}")
    except BenchError as exc:
        print(f"library: NOT FOUND — {exc}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
