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
import threading
import time

os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")

import cv2  # noqa: E402
from pupil_apriltags import Detector  # noqa: E402

from hardware.errors import BenchError  # noqa: E402

from .campreview import FpsCounter, TAG_FAMILY, annotate, read_frame  # noqa: E402
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


class Camera:
    """One registry entry plus its live capture state."""

    def __init__(self, spec: CameraSpec, detector: Detector | None):
        self.spec = spec
        self._detector = detector
        self.error: str | None = None
        self.fps = 0.0
        self._run_state: _Run | None = None
        self._subs = 0
        self._lock = threading.Lock()
        self._release_at = 0.0
        self._retired: list[threading.Thread] = []

    @property
    def profile(self) -> Profile | None:
        run = self._run_state
        return run.profile if run else None

    # ------------------------------------------------------ subscription
    def acquire(self, profile: Profile) -> _Run:
        """Register interest and return the session to read frames from.

        Callers hold the returned _Run, so a later restart cannot swap
        the box out from under them — their stream simply ends when
        their session's box closes.

        A higher-resolution request wins: a solo viewer arriving on a
        camera held open at tile resolution restarts it, because nobody
        should be served a downscale they did not ask for. That does end
        the tile viewers' current streams (they reload).
        """
        retire: _Run | None = None
        with self._lock:
            self._subs += 1
            try:
                run = self._run_state
                if run is not None and run.thread is not None \
                        and run.thread.is_alive():
                    if _area(run.profile) >= _area(profile):
                        self._release_at = 0.0
                        return run  # already open at least this large
                    retire = run
                self._start_locked(profile)
                return self._run_state
            except BaseException:
                # Never leak a subscription on a failed start — a leaked
                # refcount pins the camera open (and its bandwidth) for
                # the life of the process.
                self._subs = max(0, self._subs - 1)
                raise
            finally:
                if retire is not None:
                    self._retire_locked(retire)

    def release(self) -> None:
        with self._lock:
            self._subs = max(0, self._subs - 1)
            if self._subs == 0:
                self._release_at = time.monotonic() + LINGER_S

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
        return cap

    def _run(self, run: _Run) -> None:
        cap = None
        # Tag detection only at full resolution: a 3x3 grid of tiles
        # cannot show tag IDs usefully, and detection is the one CPU-hot
        # step — running it for eight tile streams would starve the
        # capture threads that feed them.
        detect = (self._detector is not None and self.spec.tags
                  and run.profile == self.spec.solo)
        try:
            cap = self._open(run.profile)
            counter = FpsCounter()
            while not run.stop.is_set():
                frame = read_frame(cap)
                self.fps = counter.tick()
                if detect:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    with _DETECT_LOCK:
                        dets = self._detector.detect(gray)
                    annotate(frame, dets, self.fps)
                ok, buf = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    run.box.put(buf.tobytes())
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
    def grab(self, profile: Profile, timeout: float = 8.0) -> bytes:
        """Open if needed, wait for one fresh frame, release. The whole
        interval-stills path — bandwidth held only for the grab."""
        run = self.acquire(profile)
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
            self.release()


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

        def isOpened(self):
            return not self.fail

        def set(self, prop, value):
            if prop == cv2.CAP_PROP_FRAME_WIDTH:
                opens.append((self.path, int(value)))
            return True

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
    finally:
        cv2.VideoCapture = real_cap
    print("cammanager selftest OK")


if __name__ == "__main__":
    _selftest()
