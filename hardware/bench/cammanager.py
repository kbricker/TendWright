"""Camera manager — refcounted lazy capture for the camera bus.

Plan #656. Eight cameras on one powered hub share a single 480 Mbps
USB2 uplink, so a camera must only claim bandwidth while something is
actually watching it. Each camera gets a capture thread that starts on
its FIRST subscriber and stops after its last one leaves (plus a short
linger, so a page reload doesn't thrash the device open/closed).

Failure is per-camera by construction: a camera that is unplugged, busy,
or dead records an error on its own entry and its viewers see that
error. It never takes down the server, the other cameras' streams, or
anyone's interval capture.

Selftest (no hardware — a fake capture forces every lifecycle path):

    uv run python -m hardware.bench.cammanager
"""

from __future__ import annotations

import os
import sys
import threading
import time

os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")

import cv2  # noqa: E402

from hardware.errors import BenchError  # noqa: E402

from .apriltag import TAG_FAMILY, Detector  # noqa: E402
from .campreview import (FpsCounter, annotate,  # noqa: E402
                         describe_negotiated, focus_score, read_frame,
                         set_queue_depth)
from .cameras import CameraSpec, Profile  # noqa: E402

JPEG_QUALITY = 80
LINGER_S = 3.0  # keep a camera open this long after the last subscriber
FRAME_WAIT_S = 5.0
# One detector per manager, not per camera: an apriltag detector is
# heavyweight and holds native state plus its own threadpool, and our
# binding says plainly that detect() is not thread-safe. Only
# full-resolution sessions detect (see _run), so this lock serializes at
# most a couple of streams rather than the whole fleet.
_DETECT_LOCK = threading.Lock()


def _area(p: Profile) -> int:
    return p.width * p.height


class FrameBox:
    """Latest-JPEG buffer between one capture loop and its clients.

    Replaced wholesale under the lock; the Condition wakes waiters so
    nobody polls. A slow client always gets the LATEST frame next —
    frames are dropped for it, never queued.
    """

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self.closed = False

    def put(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def close(self) -> None:
        with self._cond:
            self.closed = True
            self._cond.notify_all()

    def latest(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    def next_after(self, seq: int) -> tuple[int, bytes | None]:
        with self._cond:
            self._cond.wait_for(
                lambda: self.closed or self._seq > seq, FRAME_WAIT_S)
            if self.closed or self._seq <= seq:
                return seq, None
            return self._seq, self._jpeg


class _Run:
    """One capture session: its own stop flag, frame box, and thread.

    Per-SESSION, never reused. A wedged camera's thread can outlive the
    stop that retired it (cv2.read on a dead USB device blocks past any
    reasonable join), and if that orphan shared the next session's stop
    flag and box it would un-stop itself and close the live session's
    box on its way out. Separate state means an orphan can only ever
    affect the session it belongs to.
    """

    def __init__(self, profile: Profile):
        self.profile = profile
        self.box = FrameBox()
        # Frames delivered, and the rate they arrived at. BOTH are
        # per-session: `Camera.fps` used to be camera-wide and written
        # unguarded from the capture loop, so a thread still blocked in
        # `cv2.read` when its session was retired came back seconds
        # later and re-armed the number on an idle camera — the phantom
        # viewer this was supposed to remove.
        self.frames = 0
        self.fps = 0.0
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None
        # Set once the capture device is genuinely open. Per-SESSION for
        # the same reason everything else here is: an orphaned session
        # that opened must not make its replacement look open.
        self.opened = False
        # Whether this session is currently running tag detection. An
        # Event rather than a bool because the capture thread reads it
        # every iteration while subscribers set it from HTTP threads —
        # so detection switches on and off LIVE, with no reopen. Starts
        # clear; Camera._sync_tags_locked owns every write to it.
        self.tags = threading.Event()
        # Same opt-in shape as tags: a live sharpness readout for
        # focusing the barrel, off unless someone asked for it.
        self.focus = threading.Event()


class Camera:
    """One registry entry plus its live capture state."""

    def __init__(self, spec: CameraSpec, detector: Detector | None):
        self.spec = spec
        self._detector = detector
        # THREE FIELDS, THREE DIFFERENT QUESTIONS. They were one, and one
        # was not enough — see `health` for the session that cost.
        #
        #   error         a fault that is true RIGHT NOW. Cleared when a
        #                 new attempt starts (see _start_locked).
        #   last_error    the same text, kept as HISTORY. Never cleared,
        #                 so an intermittent can be diagnosed after it
        #                 has gone away.
        #   _last_ok      did the last attempt actually produce a
        #                 FRAME? None until one has been made. Not "did
        #                 it open" — see `health`.
        self.error: str | None = None
        self.last_error: str | None = None
        self.last_error_at: float | None = None
        self.last_ok_at: float | None = None
        self._last_ok: bool | None = None
        # What the device actually negotiated, and how it differs from what
        # the registry asked for. Both None until the camera has been opened.
        self.negotiated: str | None = None
        self.mode_note: str | None = None
        # Whether the driver queue was made shallow for freshness. False
        # means we are consuming every frame, so a deep queue costs no
        # staleness and buys back the throughput a shallow one destroys.
        self.shallow_queue = False
        self._run_state: _Run | None = None
        self._subs = 0
        # Subscribers who asked for detection, counted separately from
        # _subs. Detection runs while this is above zero and stops when
        # the last consumer of it leaves — the camera keeps streaming.
        self._tag_subs = 0
        self._focus_subs = 0
        # Latest sharpness reading, for whoever is turning the barrel.
        self.focus_score = 0.0
        self._lock = threading.Lock()
        self._release_at = 0.0
        self._retired: list[threading.Thread] = []

    @property
    def profile(self) -> Profile | None:
        run = self._run_state
        return run.profile if run else None

    @property
    def fps(self) -> float:
        """The live session's frame rate, or 0.0 when nobody is
        watching. Derived rather than stored: a camera with no session
        HAS no frame rate, and every version of this that stored it
        needed a reset somewhere and eventually missed one."""
        run = self._run_state
        return run.fps if run else 0.0

    @property
    def streaming(self) -> bool:
        """Is the device open RIGHT NOW — not merely spoken for.

        `profile is not None` is true from the moment a session is
        acquired, which is before the device is opened and before the
        open can fail. `/status` printed that as `open`, so a camera
        that had just failed to open reported itself open."""
        run = self._run_state
        return run is not None and run.opened

    @property
    def health(self) -> str:
        """"ok" | "failed" | "unknown" — the question an operator is
        actually asking, answered instead of implied.

        THE SESSION THIS EXISTS BECAUSE OF (2026-07-30). `/status` showed
        `open=False` with a stale-looking `error`, so I told Kyle the low
        camera was down for the arm run. He had it on screen and said so
        — *"I see the low cam in camserv"* — then talked himself out of
        it: *"oh hmm no maybe its stale"*. He was right the first time.

        Neither field was lying. `error` correctly reported that the LAST
        attempt had failed; `open` correctly reported that nobody was
        watching an on-demand camera. Read together they implied "broken",
        which neither of them was entitled to say. A status surface that
        makes the operator distrust their own eyes is worse than no
        status surface.

        "unknown" means NO FRAME HAS EVER BEEN SEEN, and it is
        deliberately not a diagnosis. Usually it is just an on-demand
        camera nobody has opened yet — inventing "ok" for that would be
        the same class of overclaim in the other direction. But it also
        covers a camera that opens and then delivers nothing, because
        `grab` deliberately records no verdict from a caller's timeout
        (see `grab`). So read it as "nobody asked", never as "fine": the
        thing that turns it into an answer is grabbing a frame."""
        if self._last_ok is None:
            return "unknown"
        return "ok" if self._last_ok else "failed"

    # ------------------------------------------------------ subscription
    @property
    def may_detect(self) -> bool:
        """Whether detection is PERMITTED on this camera at all.

        THREE vetoes, none of which can ever turn detection on: the
        server-wide `--no-tags`, the per-camera `tags:` in cameras.json,
        and libapriltag being absent from the machine (#713.5 — it is a
        system package now, and there is no Windows build). The first and
        third present identically here, as "no detector exists"; the
        difference is visible in camserve's stderr at startup. Nothing
        here turns detection on — only an explicit request does that
        (see acquire).
        """
        return self._detector is not None and self.spec.tags

    @property
    def detecting(self) -> bool:
        """Whether detection is running RIGHT NOW. For /status."""
        run = self._run_state
        return bool(run and run.tags.is_set())

    def acquire(self, profile: Profile, tags: bool = False,
                focus: bool = False) -> _Run:
        """Register interest and return the session to read frames from.

        Callers hold the returned _Run, so a later restart cannot swap
        the box out from under them — their stream simply ends when
        their session's box closes.

        A higher-resolution request wins: a solo viewer arriving on a
        camera held open at tile resolution restarts it, because nobody
        should be served a downscale they did not ask for. That does end
        the tile viewers' current streams (they reload).

        `tags` is the same rule applied to perception: the session runs
        detection while ANY current subscriber asked for it. Unlike the
        resolution rule it needs no restart — the capture loop re-reads
        the flag every frame. **Whatever is passed here must be passed
        back to release()**, or the count drifts and detection either
        never stops or stops while someone still wants it.
        """
        retire: _Run | None = None
        with self._lock:
            self._subs += 1
            if tags:
                self._tag_subs += 1
            if focus:
                self._focus_subs += 1
            try:
                run = self._run_state
                if run is not None and run.thread is not None \
                        and run.thread.is_alive():
                    if _area(run.profile) >= _area(profile):
                        self._release_at = 0.0
                        self._sync_tags_locked()
                        return run  # already open at least this large
                    retire = run
                self._start_locked(profile)
                return self._run_state
            except BaseException:
                # Never leak a subscription on a failed start — a leaked
                # refcount pins the camera open (and its bandwidth) for
                # the life of the process. The tag count leaks worse: it
                # would pin DETECTION on with nobody watching.
                self._subs = max(0, self._subs - 1)
                if tags:
                    self._tag_subs = max(0, self._tag_subs - 1)
                if focus:
                    self._focus_subs = max(0, self._focus_subs - 1)
                raise
            finally:
                if retire is not None:
                    self._retire_locked(retire)

    def release(self, tags: bool = False, focus: bool = False) -> None:
        """Drop a subscription. The flags MUST match what acquire got."""
        with self._lock:
            self._subs = max(0, self._subs - 1)
            if tags:
                self._tag_subs = max(0, self._tag_subs - 1)
            if focus:
                self._focus_subs = max(0, self._focus_subs - 1)
            self._sync_tags_locked()
            if self._subs == 0:
                self._release_at = time.monotonic() + LINGER_S

    def _sync_tags_locked(self) -> None:
        """Point the live session's detect flag at the current demand.

        Called from every path that changes `_tag_subs` OR replaces the
        session, which is the part that is easy to miss: a solo viewer
        arriving restarts the camera, and the NEW session must inherit
        detection from a tag subscriber who is still attached to the
        old one. Deriving the flag from the count in one place — rather
        than setting it at each call site — is what makes that hold.
        """
        run = self._run_state
        if run is None:
            return
        if self._tag_subs > 0 and self.may_detect:
            run.tags.set()
        else:
            run.tags.clear()
        if self._focus_subs > 0:
            run.focus.set()
        else:
            run.focus.clear()
            self.focus_score = 0.0

    def reap(self) -> None:
        """Janitor: close idle cameras once their linger has expired, and
        collect retired threads that have since finished."""
        with self._lock:
            if (self._subs == 0 and self._run_state is not None
                    and self._release_at
                    and time.monotonic() >= self._release_at):
                self._retire_locked(self._run_state)
            self._retired = [t for t in self._retired if t.is_alive()]

    # ------------------------------------------------------------ thread
    def _verdict(self, run: _Run, ok: bool, message: str = "") -> None:
        """Record an attempt's outcome. CALLER MUST HOLD `_lock`.

        THE LIVE SESSION ONLY, like everything else in this class.

        An earlier version ordered verdicts by attempt number instead,
        to stop a success being dropped when its session was retired
        while still opening. That defence is unreachable: the same round
        moved the success from the OPEN to the first FRAME, and a
        session retired during its open exits at `while not
        run.stop.is_set()` before its first read — so it never reaches
        the frame loop and has no success to drop. Instrumenting every
        call in that exact interleaving confirmed it: only the live
        session ever speaks.

        Being ungated was not free. It let a grab interrupted by
        ordinary teardown — a tile session retired because a solo viewer
        arrived — write `last_error` stamped `now` next to `health: ok`,
        which is the two-fields-imply-a-third bug this plan exists to
        remove, one field over from where it started."""
        if self._run_state is not run:
            return
        self._last_ok = ok
        if ok:
            self.last_ok_at = time.time()
        else:
            self.last_error = message
            self.last_error_at = time.time()

    def _start_locked(self, profile: Profile) -> None:
        run = _Run(profile)
        # A new attempt starts with a clean slate: a stale error from a
        # previous session must not make this one look failed before it
        # has even opened (that turned one transient EBUSY into a
        # permanently dead camera).
        self.error = None
        self._release_at = 0.0
        run.thread = threading.Thread(
            target=self._run, args=(run,),
            name=f"cam-{self.spec.name}", daemon=True)
        self._run_state = run
        # Before start, not after: a restart triggered by a solo viewer
        # must not drop detection for a tag subscriber still attached to
        # the session being retired.
        self._sync_tags_locked()
        run.thread.start()

    def _retire_locked(self, run: _Run) -> None:
        """Signal a session to end and drop it as the current one, so
        `_run_state` is only ever a session that has not been retired.

        Never joins: a wedged read can block far longer than any timeout
        worth holding a lock for, and the session's state is private, so
        an orphan is harmless. The janitor sweeps finished threads."""
        run.stop.set()
        run.box.close()
        if self._run_state is run:
            self._run_state = None
            self._release_at = 0.0
        if run.thread is not None and run.thread is not threading.current_thread():
            self._retired.append(run.thread)

    def _open(self, profile: Profile) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(self.spec.path)
        if not cap.isOpened():
            cap.release()
            raise BenchError(
                f"camera {self.spec.name} did not open ({self.spec.path})",
                "unplugged, on a different hub port, or already in use")
        # MJPEG before size: over USB2 the ELP otherwise negotiates YUY2
        # and silently caps 1080p at ~5 fps.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, profile.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, profile.height)
        cap.set(cv2.CAP_PROP_FPS, profile.fps)
        # Depth follows intent, and is NOT a constant — an unconditional
        # one-frame queue is what held 1080p to half rate. See
        # campreview.set_queue_depth for the measurements.
        self.shallow_queue = set_queue_depth(cap, profile.fps)
        self._record_negotiated(cap, profile)
        return cap

    def _record_negotiated(self, cap: cv2.VideoCapture,
                           profile: Profile) -> None:
        """Record what the device actually gave us, for /status.

        The comparison itself lives in `campreview.describe_negotiated` —
        its docstring carries the measurements and the reasoning. It is
        shared because campreview opens cameras directly and had exactly
        the same blind spot. Surfaced, never raised: a fallback mode is
        still usable and one camera must not take the server down.
        """
        self.negotiated, self.mode_note = describe_negotiated(
            cap, profile.width, profile.height, profile.fps)
        if self.mode_note:
            print(f"camera {self.spec.name}: {self.mode_note}",
                  file=sys.stderr, flush=True)

    def _run(self, run: _Run) -> None:
        cap = None
        # Detection is read from run.tags EVERY iteration, not decided
        # once here. Kyle, 2026-07-28: "if the cameras are on, we get a
        # live feed but there is no other software running like openCV
        # brains ... until we are actually using those brains."
        #
        # So it costs nothing while nobody is consuming it, and it turns
        # on the moment someone asks — no reopen, no dropped stream. It
        # replaces a static `run.profile == self.spec.solo` test, which
        # bound perception to the CAMERA when it belongs to the CONSUMER.
        # Software pacing. The device ignores CAP_PROP_FPS entirely (see
        # _record_negotiated), so the only thing that holds a camera to its
        # configured rate is this loop declining to read faster. Measured
        # cost of not doing it: the side camera ran at ~95 fps against a
        # configured 10, producing ~68 MB/s of frames to decode and throw
        # away, and camserve burned ~190% of a core.
        #
        # fps <= 0 means unpaced, matching what 0 already means for
        # still_interval_s. It must not quietly acquire a second meaning.
        period = 1.0 / run.profile.fps if run.profile.fps > 0 else 0.0
        try:
            cap = self._open(run.profile)
            # THE DEVICE IS ACTUALLY OPEN — the first moment that is true,
            # and the only place it can be known. `profile is not None`
            # is set at acquire, before this line runs and before it can
            # fail, which is why it never meant what `/status` printed.
            with self._lock:
                if self._run_state is run:
                    run.opened = True
            counter = FpsCounter()
            stamped = 0.0
            next_frame = time.monotonic()
            while not run.stop.is_set():
                frame = read_frame(cap)
                run.fps = counter.tick()
                if run.focus.is_set():
                    self.focus_score = focus_score(frame)
                    # TOP-RIGHT corner (Kyle 2026-07-28: "move that focus
                    # number to the top corner not the bottom"). Right
                    # rather than left because annotate() already writes
                    # the size/fps/tag line at top-left, and with
                    # ?focus=1&tags=1 — the combination that actually
                    # matters, peak the number then check the tags still
                    # lock — the two would print over each other.
                    label = f"focus {self.focus_score:.0f}"
                    (tw, th), _ = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 1.3, 3)
                    cv2.putText(frame, label,
                                (max(10, frame.shape[1] - tw - 14), th + 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.3,
                                (0, 255, 255), 3)
                if run.tags.is_set():
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    with _DETECT_LOCK:
                        dets = self._detector.detect(gray)
                    annotate(frame, dets, self.fps)
                ok, buf = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    run.box.put(buf.tobytes())
                    run.frames += 1
                    # HEALTH IS A FRAME, NOT AN OPEN. Setting it when the
                    # device opened was the obvious place and the wrong
                    # one: a camera starved of USB isochronous bandwidth
                    # — the most likely real fault on a shared uplink,
                    # and the one `grab`'s own hint names — opens
                    # perfectly and then never yields a picture. It
                    # reported health "ok" while every snapshot 503'd,
                    # which is the same lie as 713.12 with the sign
                    # flipped. `ok` now means a picture actually arrived.
                    # THE VERDICT IS TAKEN ONCE; the TIMESTAMP keeps
                    # moving. `last_ok_at` is documented in three places
                    # as the way to age a stale "ok", and short-circuiting
                    # the whole block on `_last_ok is not True` froze it
                    # at the first frame after the last failure — so a
                    # camera streaming right now reported an "ok" from
                    # last week and the preflight could not tell.
                    #
                    # Throttled rather than per-frame: at 100 Hz this
                    # would otherwise take the lock a hundred times a
                    # second per camera to write a number nobody reads
                    # that often.
                    now = time.monotonic()
                    if self._last_ok is not True or now - stamped > 1.0:
                        stamped = now
                        with self._lock:
                            self._verdict(run, True)
                if period:
                    # Sleep OUTSIDE _DETECT_LOCK — holding or awaiting the
                    # shared detect lock while sleeping would starve every
                    # other camera in the fleet, which is a different bug
                    # (713.6) that this must not make worse.
                    next_frame += period
                    delay = next_frame - time.monotonic()
                    if delay > 0:
                        # stop.wait() rather than sleep() so a camera closed
                        # mid-period tears down immediately instead of
                        # blocking teardown for up to one frame period.
                        if run.stop.wait(delay):
                            break
                    else:
                        # Running behind: reset rather than accumulate debt,
                        # or a slow patch makes the loop sprint to catch up.
                        next_frame = time.monotonic()
        except BenchError as exc:
            self._fail(run, str(exc))
        except Exception as exc:  # never let one camera kill the server
            self._fail(run, f"{type(exc).__name__}: {exc}")
        finally:
            # THE SESSION IS OVER, so it is no longer open. Without this
            # a camera whose device was unplugged mid-stream kept
            # reporting `open: true` for the several seconds until its
            # last viewer noticed — with the capture released and the
            # thread dead, which is the precise opposite of what the
            # field promises. Per-session state, so it can never touch a
            # replacement.
            run.opened = False
            run.box.close()
            if cap is not None:
                cap.release()

    def _fail(self, run: _Run, message: str) -> None:
        """Record an error only if this session is still the live one —
        a retired session failing on its way out must not mark a healthy
        replacement as broken.

        THE HISTORY IS RECORDED UNDER THE SAME GUARD, deliberately. An
        earlier version recorded it either way, on the theory that an
        orphan's death is still evidence — but a session retired by a
        profile upgrade dying on its way out is ordinary teardown, not a
        fault, and it stamped `last_error` with the current time next to
        `health: ok`. Two fields implying a third neither is entitled to
        say is the exact bug this plan exists to remove; reintroducing it
        one field over to preserve a diagnostic is not a trade worth
        making."""
        with self._lock:
            if self._run_state is run:
                self.error = message
            self._verdict(run, False, message)

    # ------------------------------------------------------------ frames
    def grab(self, profile: Profile, timeout: float = 8.0,
             tags: bool = False) -> bytes:
        """Open if needed, wait for one fresh frame, release. The whole
        interval-stills path — bandwidth held only for the grab.

        `tags` defaults OFF, and that is a fix rather than a default:
        before per-request opt-in, grab() ran at the solo profile and so
        always got the overlay, meaning every interval still and every
        capture set on disk had green tag boxes and an fps readout burned
        into the pixels. Those frames are exactly what 713.8 wants for
        intrinsics calibration, and drawn-on frames are not measurable.
        """
        run = self.acquire(profile, tags=tags)
        try:
            deadline = time.monotonic() + timeout
            seq = 0
            while time.monotonic() < deadline:
                seq, jpeg = run.box.next_after(seq)
                if jpeg is not None:
                    return jpeg
                if run.box.closed:  # session ended — the error explains why
                    break
            # DELIBERATELY DOES NOT RECORD A FAULT. A grab timing out
            # is this caller's disappointment, not a verdict on the
            # camera, and three rounds of review produced three distinct
            # defects from trying to make it one:
            #
            #   * it overwrote the root cause ("camera d did not open
            #     (/dev/...)") with "no frame in 5s" — in the one field
            #     whose whole job is to say why;
            #   * it stamped `error` on a session that was streaming
            #     fine to other viewers, where nothing could clear it
            #     until that session ended — and `error` tears down
            #     every MJPEG stream on the camera;
            #   * narrowed to "sessions that have delivered nothing", it
            #     still fired in the window before a slow camera's FIRST
            #     frame, which is exactly where a 1080p sensor
            #     negotiating on a busy uplink sits.
            #
            # A camera that opens and produces nothing therefore reports
            # health "unknown" — no frame has been seen — rather than
            # "ok". That is the honest answer and it is the SAFE one:
            # the documented gate is `health == "ok"`, so an unknown
            # camera does not pass it. The old behaviour this plan
            # started from, reporting such a camera as healthy, was the
            # dangerous direction.
            raise BenchError(
                f"camera {self.spec.name}: "
                + (self.error or f"no frame in {timeout:.0f}s"),
                "the camera may be unplugged, in use, or the USB bus "
                "may be out of isochronous bandwidth (too many cameras "
                "open at once)")
        finally:
            self.release(tags=tags)


class CameraManager:
    """Owns every registered camera and the janitor that closes idle ones."""

    def __init__(self, specs: list[CameraSpec], tags: bool = True):
        detector = None
        if tags:
            try:
                detector = Detector(families=TAG_FAMILY)
            except BenchError as exc:
                # A THIRD veto, alongside --no-tags and per-camera `tags:`
                # — see Camera.may_detect. libapriltag is a system package
                # (#713.5) and there is no Windows build, so a machine
                # without it must still serve cameras rather than refuse to
                # start. Said LOUDLY on purpose: silent loss of detection
                # is how an operator concludes the tags are broken and
                # spends an afternoon on the camera instead of on apt.
                # /status reports may_detect=false for every camera too.
                print(f"TAG DETECTION OFF: {exc}", file=sys.stderr)
        self.cameras: dict[str, Camera] = {
            s.name: Camera(s, detector) for s in specs}
        self.order = [s.name for s in specs]
        self._stop = threading.Event()
        self._janitor = threading.Thread(
            target=self._reap_loop, name="cam-janitor", daemon=True)
        self._janitor.start()

    def get(self, name: str) -> Camera:
        cam = self.cameras.get(name)
        if cam is None:
            raise KeyError(name)
        return cam

    def _reap_loop(self) -> None:
        while not self._stop.wait(1.0):
            for cam in self.cameras.values():
                cam.reap()

    def shutdown(self) -> None:
        self._stop.set()
        for cam in self.cameras.values():
            with cam._lock:
                if cam._run_state is not None:
                    cam._retire_locked(cam._run_state)


# --------------------------------------------------------------- selftest
def _selftest() -> None:
    """Lifecycle only — no real camera, no HTTP. Proves the refcount,
    the profile upgrade, per-session isolation, and failure containment.
    """
    import numpy as np

    from .cameras import CameraSpec

    opens: list[tuple[str, int]] = []

    class FakeCap:
        """A capture that can be told to fail, or to WEDGE (never return
        a frame) — the case that used to orphan a thread."""

        # A path containing "flaky" fails its FIRST open and works from
        # then on — the intermittent this whole plan is about, and the
        # one shape the old harness could not express: every fake was
        # permanently dead or permanently fine, so no test could cross
        # the failed -> recovered transition.
        flaked: set = set()

        def __init__(self, path):
            self.path, self.alive = str(path), True
            self.fail = "dead" in self.path
            if "flaky" in self.path and self.path not in FakeCap.flaked:
                FakeCap.flaked.add(self.path)
                self.fail = True
            self.wedge = "wedge" in self.path
            # STARVED, not dead: opens fine, then delivers frames far
            # slower than any caller will wait. This is what a camera
            # squeezed off a saturated USB2 uplink actually does, and it
            # is the case where "did it open" and "can I get a picture"
            # give opposite answers.
            self.starve = "starve" in self.path
            # Opens and streams, then the device goes away — an unplug
            # mid-stream. The only shape that reaches `_run`'s teardown
            # with the session still live.
            self.dies = "dies" in self.path
            self.reads = 0
            # A "lying" device models the real ELP: it accepts whatever you
            # set and then reports a different mode. Everything reads back
            # what was set unless the path says otherwise.
            self.lies = "lying" in self.path
            self.props: dict = {}

        def isOpened(self):
            return not self.fail

        def set(self, prop, value):
            if prop == cv2.CAP_PROP_FRAME_WIDTH:
                opens.append((self.path, int(value)))
            self.props[prop] = value
            return True

        def get(self, prop):
            """Report back the negotiated mode. A real UVC camera reports
            what it ACTUALLY does, which is not always what you asked for —
            that gap is what _record_negotiated exists to catch."""
            if self.lies:
                if prop == cv2.CAP_PROP_FRAME_WIDTH:
                    return 640.0
                if prop == cv2.CAP_PROP_FRAME_HEIGHT:
                    return 480.0
                if prop == cv2.CAP_PROP_FPS:
                    return 120.101
            return float(self.props.get(prop, 0))

        def read(self):
            if self.dies:
                self.reads += 1
                if self.reads > 3:
                    return False, None
            if self.starve:
                # Longer than FRAME_WAIT_S: `next_after` blocks for that
                # regardless of the caller's own deadline, so a shorter
                # starve still delivers a frame and proves nothing.
                time.sleep(FRAME_WAIT_S + 3.0)
                return True, np.zeros((8, 8, 3), np.uint8)
            if self.wedge:
                time.sleep(0.05)
                return True, None if False else np.zeros((4, 4, 3), np.uint8)
            time.sleep(0.01)
            return True, np.zeros((8, 8, 3), np.uint8)

        def release(self):
            self.alive = False

    def wait_for(predicate, what, timeout=5.0):
        """Capture threads start asynchronously — never assert on thread
        progress immediately (that raced and passed only on fast hosts)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError(what)

    real_cap, cv2.VideoCapture = cv2.VideoCapture, FakeCap
    try:
        big, small = Profile(320, 240, 10), Profile(80, 60, 5)
        spec = CameraSpec("t", "bench", "/dev/fake/t", big, small, tags=False)
        dead = CameraSpec("d", "-", "/dev/fake/dead", big, small, tags=False)
        cam, cam_dead = Camera(spec, None), Camera(dead, None)

        # 1. lazy: no capture until acquired
        assert not opens
        run = cam.acquire(small)
        assert cam.profile == small, cam.profile
        wait_for(lambda: opens, "tile acquire never opened the device")

        # 2. a second subscriber at the same size reuses the session
        before = len(opens)
        run2 = cam.acquire(small)
        assert run2 is run and len(opens) == before, "reopened needlessly"

        # 3. a LARGER request restarts at the larger profile...
        run3 = cam.acquire(big)
        assert run3 is not run, "upgrade did not start a new session"
        assert cam.profile == big, cam.profile
        # ...and the retired session's box is closed so its viewers exit
        assert run.box.closed, "retired session left its viewers hanging"

        # 4. a SMALLER request afterwards does not shrink the session
        run4 = cam.acquire(small)
        assert run4 is run3 and cam.profile == big

        # 4a. a well-behaved device produces no mode note. Guards against
        #     the check crying wolf on every camera, which would train
        #     everyone to ignore it.
        assert cam.negotiated == "320x240", cam.negotiated
        assert cam.mode_note is None, cam.mode_note

        # 4b. a LYING device is caught. This is the real ELP behaviour:
        #     asked for 640x360@10, delivers 640x480 at 120 fps, reports
        #     no error. Silence here is what hid a 10x overrun for weeks.
        liar = Camera(CameraSpec("l", "-", "/dev/fake/lying",
                                 Profile(640, 360, 10),
                                 Profile(640, 360, 10), tags=False), None)
        liar.acquire(Profile(640, 360, 10))
        wait_for(lambda: liar.negotiated is not None, "liar never opened")
        assert liar.negotiated == "640x480", liar.negotiated
        assert liar.mode_note and "640x480" in liar.mode_note, liar.mode_note
        assert "120" in liar.mode_note, "native rate not reported"
        liar.release()

        # 4c. PACING actually paces. The fake returns a frame every 10 ms
        #     (~100 fps); a profile asking for 20 must observe ~20, not 100.
        #     This is the whole fix: the device ignores CAP_PROP_FPS, so
        #     only the loop declining to read can hold the rate.
        paced = Camera(CameraSpec("p", "-", "/dev/fake/p",
                                  Profile(320, 240, 20),
                                  Profile(320, 240, 20), tags=False), None)
        pr = paced.acquire(Profile(320, 240, 20))
        wait_for(lambda: pr.box.latest() is not None, "paced cam never ran")
        seq, frames = 0, 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            seq, jpeg = pr.box.next_after(seq)
            if jpeg is not None:
                frames += 1
        rate = frames / (time.monotonic() - t0)
        paced.release()
        assert rate < 40, f"pacing did not engage: {rate:.0f} fps"
        assert rate > 8, f"pacing overshot and starved the stream: {rate:.0f}"

        # 5. refcount: still open with subscribers, reaped after linger
        for _ in range(4):
            cam.release()
        assert cam.profile is not None, "released too eagerly"
        cam.reap()
        assert cam.profile is not None, "reaped before the linger expired"
        cam._release_at = time.monotonic() - 1  # fast-forward the linger
        cam.reap()
        assert cam.profile is None, "idle camera was never reaped"

        # 6. failure is contained and does NOT persist across attempts —
        #    one transient failure must not kill a camera forever
        try:
            cam_dead.grab(big, timeout=2)
        except BenchError:
            pass
        else:
            raise AssertionError("a dead camera returned a frame")
        assert cam_dead.error, "no error recorded"
        assert cam.error is None, "one camera's failure leaked to another"
        jpeg = cam.grab(big, timeout=5)
        assert jpeg[:2] == b"\xff\xd8", "healthy camera stopped working"
        assert cam.error is None, "a stale error survived a good session"

        # 6b. THE STATUS SURFACE, across all four transitions (713.12).
        #     Each of these read wrong before, and the pair of them read
        #     "broken" for a camera that was fine.
        fresh = Camera(CameraSpec("f", "-", "/dev/fake/f", big, small,
                                  tags=False), None)
        assert fresh.health == "unknown", "a never-opened camera claimed health"
        assert not fresh.streaming, "a never-opened camera read as streaming"
        assert fresh.last_error is None, "history before anything happened"

        assert cam_dead.health == "failed", "a dead camera did not read failed"
        assert not cam_dead.streaming, (
            "a camera that FAILED to open read as streaming — `profile is "
            "not None` is set at acquire, before the open can fail")
        assert cam_dead.last_error, "a failure left no history"
        # THE ROOT CAUSE, not a symptom of waiting for it. `grab` used
        # to overwrite this with "no frame in 2s" on its way out, which
        # is the message that reaches the 503 body, the capture-set
        # manifest and `/status` — so the one field whose job is to say
        # WHY said nothing at all.
        assert "did not open" in cam_dead.last_error, (
            f"the root cause was overwritten by a timeout message: "
            f"{cam_dead.last_error!r}")
        assert cam_dead.last_error_at, "a failure left no timestamp"

        # The headline: healthy while open, and STILL healthy once idle.
        # `open` going False is not a fault, and reading it as one is the
        # whole bug.
        live = cam.acquire(big)
        wait_for(lambda: cam.streaming, "a live camera never read as streaming")
        assert cam.health == "ok", "an open camera did not read ok"
        # Let a real rate register, or the idle-fps assertion below
        # passes whether or not `fps` is ever reset — it starts at 0.0.
        wait_for(lambda: cam.fps > 0, "the live camera never measured a rate")

        # `last_ok_at` must ADVANCE while the camera keeps working —
        # checked HERE, with the session still live, because after the
        # reap no frames flow and the assertion could not fail. Merely
        # asserting it is truthy passed while the stamp was frozen at
        # the first frame after the last failure, so a camera streaming
        # right now reported an "ok" from days ago and the preflight the
        # docs describe ("grab a frame, then check its age") could not
        # work.
        first_ok = cam.last_ok_at
        assert first_ok, "a working camera never stamped a success"
        wait_for(lambda: (cam.last_ok_at or 0) > first_ok,
                 "last_ok_at never advanced while the camera streamed",
                 timeout=6.0)

        cam.release()
        cam._release_at = time.monotonic() - 1
        cam.reap()
        assert not cam.streaming, "a reaped camera still read as streaming"
        assert cam.fps == 0.0, (
            "an idle camera kept the last session's frame rate — which "
            "reads as 'someone is streaming this right now' and sent an "
            "operator hunting a phantom viewer")
        assert cam.health == "ok", (
            "a camera nobody is watching read as unhealthy — 713.12, the "
            "reading that had me tell Kyle a working camera was down")
        assert cam.error is None, "idle is not an error"

        # 6c. THE TRANSITION THAT MATTERS: fail, then recover. Nothing
        #     covered it before — every fake camera was permanently dead
        #     or permanently fine — so `last_error`'s "never cleared"
        #     guarantee was unpinned, and adding `self.last_error = None`
        #     to `_start_locked` (the natural next edit) passed the whole
        #     suite while silently deleting the diagnostic.
        flaky = Camera(CameraSpec("fl", "-", "/dev/fake/flaky", big, small,
                                  tags=False), None)
        try:
            flaky.grab(big, timeout=2)
        except BenchError:
            pass
        assert flaky.health == "failed", "a failed camera did not read failed"
        first_error = flaky.last_error
        assert first_error, "the failure left no history"

        jpeg = flaky.grab(big, timeout=5)          # same camera, now fine
        assert jpeg[:2] == b"\xff\xd8", "the flaky camera never recovered"
        assert flaky.health == "ok", (
            "a recovered camera still read as failed — the whole point")
        assert flaky.error is None, "a superseded fault is not current"
        assert flaky.last_error == first_error, (
            "recovery ERASED the history that explains the intermittent — "
            "`last_error` is the record, not a copy of `error`")
        assert flaky.last_ok_at, "a recovery left no timestamp"

        # 6d. A camera that OPENS and then never yields a frame must not
        #     read healthy. Health is a picture, not a file handle — and
        #     starved USB bandwidth, the likeliest real fault on a shared
        #     hub, looks exactly like this.
        wedge = Camera(CameraSpec("wd", "-", "/dev/fake/starve", big, small,
                                  tags=False), None)
        try:
            wedge.grab(big, timeout=1)
            raise AssertionError("a wedged camera returned a frame")
        except BenchError:
            pass
        assert wedge.health != "ok", (
            "a camera that opens but never produces a frame read as "
            "healthy — every snapshot from it 503s, and `health == \"ok\"` "
            "is the documented gate")
        assert wedge.health == "unknown", (
            "a starved camera should read UNKNOWN — no frame has been "
            "seen, which is the honest answer and safely fails the gate. "
            "Calling it `failed` from the caller's timeout is what "
            "overwrote root causes and poisoned live sessions")

        # 6e. A session whose device went away mid-stream is NOT open,
        #     even while a subscriber still holds it. `run.opened` was
        #     set once and never cleared, so `/status` reported
        #     `open: true` for a camera with the capture released and
        #     the thread dead — the exact opposite of what it promises.
        dying = Camera(CameraSpec("dy", "-", "/dev/fake/dies", big, small,
                                  tags=False), None)
        d1 = dying.acquire(big)
        wait_for(lambda: dying.streaming, "the dying camera never opened")
        wait_for(lambda: not d1.thread.is_alive(), "the session never died")
        assert dying.health == "failed", "a dead session read healthy"
        assert not dying.streaming, (
            "a camera whose capture is released and whose thread is dead "
            "still read as open")

        # 6f. ONLY THE LIVE SESSION SPEAKS. An orphan dying on its way
        #     out is ordinary teardown, and letting it write the verdict
        #     stamped `last_error` with the current time next to
        #     `health: ok` — the same two-fields-imply-a-third bug this
        #     plan removes, one field over.
        held = Camera(CameraSpec("hd", "-", "/dev/fake/hd", big, small,
                                 tags=False), None)
        held.grab(big, timeout=5)                   # establish health ok
        assert held.health == "ok" and held.last_error is None
        orphan = _Run(big)                          # never the live session
        held._fail(orphan, "an orphan died on its way out")
        assert held.health == "ok", "a retired session set the verdict"
        assert held.error is None, "a retired session set the live error"
        assert held.last_error is None, (
            "ordinary teardown fabricated a fault in the history field")

        # 6g. A GRAB THAT TIMES OUT RECORDS NOTHING. It is one
        #     caller's disappointment, not a verdict on the camera —
        #     and every attempt to make it one produced a defect:
        #     overwriting the root cause, and stamping `error` on a
        #     session streaming happily to other viewers, where nothing
        #     could clear it and `error` tears down every stream.
        busy = Camera(CameraSpec("by", "-", "/dev/fake/by", big, small,
                                 tags=False), None)
        b1 = busy.acquire(big)
        wait_for(lambda: b1.frames > 0, "the busy camera never delivered")
        try:
            busy.grab(big, timeout=0)               # instant timeout
        except BenchError:
            pass
        assert busy.error is None, (
            "an unlucky grab poisoned a live, streaming session")
        assert busy.health == "ok", "a working camera was marked failed"
        assert busy.last_error is None, (
            "a caller's timeout was written into the failure history")
        busy.release()
        del live

        # 7. a wedged session cannot corrupt its replacement: retire it
        #    while its thread still runs, then start a fresh one
        wedged = Camera(CameraSpec("w", "-", "/dev/fake/wedge", big, small,
                                   tags=False), None)
        w1 = wedged.acquire(big)
        wait_for(lambda: w1.thread.is_alive(), "wedged session never ran")
        with wedged._lock:
            wedged._retire_locked(w1)
        w2 = wedged.acquire(big)
        assert w2 is not w1 and not w2.box.closed
        time.sleep(0.3)  # let the orphan run through its exit path
        assert not w2.box.closed, "the orphan closed the live session's box"
        wedged.release()

        # 8. subscriptions are not leaked when a start fails
        boom = Camera(CameraSpec("b", "-", "/dev/fake/b", big, small,
                                 tags=False), None)
        orig = threading.Thread.start

        def explode(self):
            raise RuntimeError("cannot start thread")

        threading.Thread.start = explode
        try:
            boom.acquire(big)
        except RuntimeError:
            pass
        else:
            raise AssertionError("failed start was not raised")
        finally:
            threading.Thread.start = orig
        assert boom._subs == 0, f"leaked subscription: {boom._subs}"

        # ---------------------------------------------------------- #713.6
        # Perception is opt-in PER REQUEST. These assert on whether the
        # detector was actually CALLED, not on the flag — a flag that is
        # set while the loop ignores it would pass a weaker test and is
        # exactly the failure mode worth catching.
        class FakeDetector:
            def __init__(self):
                self.calls = 0

            def detect(self, gray):
                self.calls += 1
                return []

        def detects_climb(det, run, what, should=True):
            """Did detection run over the next handful of frames?"""
            before, seq = det.calls, 0
            for _ in range(6):
                seq, _jpeg = run.box.next_after(seq)
            climbed = det.calls > before
            assert climbed == should, what

        det = FakeDetector()
        tagged = Camera(CameraSpec("g", "-", "/dev/fake/g", big, small,
                                   tags=True), det)

        # 9a. THE HEADLINE: a plain viewer pays nothing for perception.
        plain = tagged.acquire(small)
        wait_for(lambda: plain.box.latest() is not None, "plain never ran")
        detects_climb(det, plain, "detection ran for a viewer who never "
                                  "asked for it — the whole point of 713.6",
                      should=False)

        # 9b. ...and asking turns it on, live, with no reopen. The plain
        #     viewer's session object is the same one.
        reopens = len(opens)
        asked = tagged.acquire(small, tags=True)
        assert asked is plain, "asking for tags needlessly restarted"
        assert len(opens) == reopens, "asking for tags reopened the device"
        detects_climb(det, plain, "detection did not start when requested")

        # 9c. the last consumer of perception leaving stops it, WITHOUT
        #     stopping the stream the plain viewer is still watching.
        tagged.release(tags=True)
        detects_climb(det, plain, "detection outlived the only subscriber "
                                  "who wanted it", should=False)
        assert plain.box.latest() is not None, "the plain stream died with it"

        # 9d. THE TRAP. A tag subscriber sits on the tile session; a solo
        #     viewer arrives and restarts the camera. The NEW session must
        #     inherit detection — the old one is being retired underneath
        #     a subscriber who is still attached and still wants it.
        tagged.acquire(small, tags=True)
        upgraded = tagged.acquire(big)
        assert upgraded is not plain, "expected a restart at the larger size"
        assert upgraded.tags.is_set(), \
            "a restart dropped detection for a subscriber who still wanted it"
        detects_climb(det, upgraded, "detection did not survive the restart")
        tagged.release(tags=True)
        for _ in range(3):
            tagged.release()

        # 9e. the vetoes still veto. cameras.json `tags: false` (and the
        #     server-wide --no-tags, modelled by detector=None) can only
        #     turn detection OFF — asking cannot override either.
        vdet = FakeDetector()
        vetoed = Camera(CameraSpec("v", "-", "/dev/fake/v", big, small,
                                   tags=False), vdet)
        assert not vetoed.may_detect, "per-camera veto ignored"
        vrun = vetoed.acquire(small, tags=True)
        wait_for(lambda: vrun.box.latest() is not None, "vetoed never ran")
        detects_climb(vdet, vrun, "a vetoed camera detected anyway",
                      should=False)
        vetoed.release(tags=True)

        # 9f. a failed start must not leak the TAG count either. A leaked
        #     _subs pins bandwidth; a leaked _tag_subs pins the CPU-hot
        #     detector on with nobody watching, which is worse.
        boom2 = Camera(CameraSpec("b2", "-", "/dev/fake/b2", big, small,
                                  tags=True), FakeDetector())
        threading.Thread.start = explode
        try:
            boom2.acquire(big, tags=True)
        except RuntimeError:
            pass
        else:
            raise AssertionError("failed start was not raised")
        finally:
            threading.Thread.start = orig
        assert boom2._tag_subs == 0, f"leaked tag sub: {boom2._tag_subs}"

        # 9g. an unbalanced release cannot drive the count negative, which
        #     would silently make the NEXT genuine request a no-op.
        boom2.release(tags=True)
        assert boom2._tag_subs == 0, "tag count went negative"
    finally:
        cv2.VideoCapture = real_cap
    print("cammanager selftest OK")


if __name__ == "__main__":
    _selftest()
