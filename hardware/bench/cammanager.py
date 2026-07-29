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
from pupil_apriltags import Detector  # noqa: E402

from hardware.errors import BenchError  # noqa: E402

from .campreview import (FpsCounter, TAG_FAMILY, annotate,  # noqa: E402
                         describe_negotiated, focus_score, read_frame,
                         set_queue_depth)
from .cameras import CameraSpec, Profile  # noqa: E402

JPEG_QUALITY = 80
LINGER_S = 3.0  # keep a camera open this long after the last subscriber
FRAME_WAIT_S = 5.0
# One detector per manager, not per camera: pupil_apriltags detectors are
# heavyweight and hold GIL-bound native state. Only full-resolution
# sessions detect (see _run), so this lock serializes at most a couple
# of streams rather than the whole fleet.
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
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None
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
        self.error: str | None = None
        self.fps = 0.0
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

    # ------------------------------------------------------ subscription
    @property
    def may_detect(self) -> bool:
        """Whether detection is PERMITTED on this camera at all.

        Two vetoes, both of which can only turn detection off, never on:
        the server-wide `--no-tags` (no detector exists) and the
        per-camera `tags:` in cameras.json. Nothing here turns detection
        on — only an explicit request does that (see acquire).
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
            counter = FpsCounter()
            next_frame = time.monotonic()
            while not run.stop.is_set():
                frame = read_frame(cap)
                self.fps = counter.tick()
                if run.focus.is_set():
                    self.focus_score = focus_score(frame)
                    cv2.putText(frame, f"focus {self.focus_score:7.0f}",
                                (10, frame.shape[0] - 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.1,
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
            run.box.close()
            if cap is not None:
                cap.release()

    def _fail(self, run: _Run, message: str) -> None:
        """Record an error only if this session is still the live one —
        a retired session failing on its way out must not mark a healthy
        replacement as broken."""
        with self._lock:
            if self._run_state is run:
                self.error = message

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
        detector = Detector(families=TAG_FAMILY) if tags else None
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

        def __init__(self, path):
            self.path, self.alive = str(path), True
            self.fail = "dead" in self.path
            self.wedge = "wedge" in self.path
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
