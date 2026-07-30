"""campreview — live camera view with AprilTag (tag36h11) detection overlay.

    uv run python -m hardware.bench.campreview                # camera 0
    uv run python -m hardware.bench.campreview --camera 1 --width 1920 --height 1080

Shows resolution + measured FPS, draws detected tag corners/centers/IDs.
Keys in the window: s = save a snapshot PNG, q/Esc = quit.
The view window opens 960 px wide (~quarter of the bench 1080p screen),
drag-resizable; override with --view-width. Capture, detection, and
snapshots always run at full camera resolution — only the view scales.
The loop is paced in software to --fps (default 30): these cameras
deliver up to 120 fps and ignore the rate you ask them for, so nothing
else holds a preview to a sane cost. Headless cell1 note: run over
`ssh -X cell1`, or use --grab to save N frames to disk without a window.

Usage: campreview [--camera N] [--width W] [--height H] [--fps N]
                  [--calib FILE.npz] [--view-width PX] [--grab N]
                  [--outdir DIR]

Selftest (no camera, no window — pacing math + mode negotiation):

    uv run python -m hardware.bench.campreview --selftest
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# Must be set before cv2 imports: silences the native layer's WARN/ERROR
# chatter so our clean one-line errors stay clean.
os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")

# Belt and braces for the font problem fixed below: if Qt still cannot
# use the fonts we point it at, silence that ONE message category
# rather than let it drown the terminal. Real Qt/display errors still
# print.
os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts.warning=false")
_USER_FONTDIR = os.environ.get("QT_QPA_FONTDIR")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from .apriltag import TAG_FAMILY, Detector  # noqa: E402


def _fix_qt_fonts() -> None:
    """Point Qt at fonts that exist.

    The opencv-python wheel bundles Qt5 built without fontconfig AND
    overwrites QT_QPA_FONTDIR **at import time** to its own bundled
    fonts directory — which ships EMPTY (0 ttf files, verified on
    cell1). Qt therefore has no fonts at all and repeats "Note that Qt
    no longer ships fonts" until it floods the terminal.

    So this must run AFTER `import cv2` (setting it before is silently
    clobbered), and before the first window is created — Qt reads the
    variable when its platform plugin initialises. An explicit operator
    setting always wins.
    """
    current = os.environ.get("QT_QPA_FONTDIR", "")
    if _USER_FONTDIR:  # operator asked for a specific directory
        os.environ["QT_QPA_FONTDIR"] = _USER_FONTDIR
        return
    try:
        if current and any(Path(current).glob("*.ttf")):
            return  # whatever it points at actually has fonts
    except OSError:
        pass
    for candidate in ("/usr/share/fonts/truetype/dejavu",
                      "/usr/share/fonts/truetype/liberation",
                      "/usr/share/fonts/truetype/noto",
                      "/usr/share/fonts/truetype",
                      "/usr/share/fonts"):
        try:
            if any(Path(candidate).rglob("*.ttf")):
                os.environ["QT_QPA_FONTDIR"] = candidate
                return
        except OSError:
            continue


_fix_qt_fonts()

# Camera tools import from hardware.errors, NOT .bus — keeps the Feetech
# servo SDK out of their import graph entirely (errors.py exists for this).
from hardware.errors import BenchError, make_run_tool  # noqa: E402

FPS_WINDOW = 30  # sliding-window samples for the FPS readout (~1-3 s)
# Default view-window width: 960x540 is a quarter of the bench's 1080p
# screen by area — Kyle never watches the preview at 100%. The clamp is a
# sanity envelope, not a screen query (single known bench monitor).
VIEW_WIDTH_DEFAULT = 960
VIEW_WIDTH_MIN, VIEW_WIDTH_MAX = 160, 3840
# Default preview rate. 30 is the fastest a human gains anything from and
# is the native rate of the 1080p mode; the cameras will happily deliver
# 120 at 640x480 if nobody stops them. 0 means unpaced, matching what 0
# already means for still_interval_s in the registry.
PREVIEW_FPS_DEFAULT = 30.0
# Ask for a one-frame driver queue only when the requested rate is below
# this fraction of what the device natively runs — see set_queue_depth.
QUEUE_SHALLOW_RATIO = 0.8

# Camera-flavored CLI wrapper (vs bus.py's servo-flavored unplug hint).
run_tool = make_run_tool("unplug/replug the camera and re-run")


def focus_score(frame) -> float:
    """How sharp the middle of the frame is. Higher is sharper.

    Variance of the Laplacian — the standard focus measure. A fixed
    M12 lens is focused by ROTATING THE BARREL, and by eye that is a
    guess: the eye is poor at the last quarter turn and a live preview
    at 30 fps flatters everything. A number you can maximise is a
    different job.

    Measured on the CENTRE HALF only. Turning a barrel changes the
    magnification slightly, and vignetting and lens softness at the
    corners move with focus too, so scoring the whole frame mixes those
    in and the peak drifts. The centre keeps the content constant.

    ABSOLUTE VALUES MEAN NOTHING — it scales with scene texture,
    exposure and resolution. Only the trend as you turn the barrel is
    information. Peak it, then stop.
    """
    h, w = frame.shape[:2]
    roi = frame[h // 4:h - h // 4, w // 4:w - w // 4]
    if roi.size == 0:
        return 0.0
    if roi.ndim == 3:
        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(roi, cv2.CV_64F).var())


def next_deadline(deadline: float, period: float) -> float:
    """The next frame deadline, one period on.

    RESETS rather than accumulating debt: if the loop fell behind — a
    slow detection pass, a large snapshot write — the deadline is pulled
    up to now instead of leaving a backlog the loop would then sprint to
    burn off. A preview that briefly stutters should return to its rate,
    not overshoot it.
    """
    deadline += period
    now = time.monotonic()
    return deadline if deadline > now else now


class FpsCounter:
    """Measured FPS over a sliding window of frame timestamps."""

    def __init__(self):
        self._times: list[float] = []

    def tick(self) -> float:
        self._times.append(time.monotonic())
        self._times = self._times[-FPS_WINDOW:]
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / (self._times[-1] - self._times[0])


def read_frame(cap: cv2.VideoCapture):
    """One frame, or the shared dead-camera BenchError."""
    ok, frame = cap.read()
    if not ok:
        raise BenchError("camera stopped returning frames",
                         "unplug/replug the camera and retry")
    return frame


def describe_negotiated(cap: cv2.VideoCapture, width: int, height: int,
                        fps: float) -> tuple[str, str | None]:
    """Compare what we asked the device for against what it gave us.

    Returns `(negotiated, note)` — the actual "WxH" the device settled
    on, and a human-readable note describing every gap, or None when it
    gave us exactly what we asked for.

    WHY THIS EXISTS. A UVC camera accepts any resolution and any frame
    rate you set and then delivers whatever mode it actually has, with
    no error anywhere. Measured on the ELP-USBFHD01M-L36, 2026-07-28:

        asked 640x360 @10  ->  got 640x480 @120, observed 95 fps
        asked 640x480 @10  ->  got 640x480 @120, observed 46 fps
        asked 1920x1080@10 ->  got 1920x1080@30, observed 27 fps

    Its native MJPG modes are 320x240, 640x480, 800x600, 1280x720,
    1280x1024 and 1920x1080 — so camserve's tile profile of 640x360 was
    never a real mode, and CAP_PROP_FPS is ignored outright at every
    size.

    The registry said 640x360@10 and the camera ran 640x480 at ~95 fps
    for weeks without a single line of output saying so. That silence
    is the actual defect: a wrong number you can see is a bug, and a
    wrong number you cannot see is a wrong model of the system. So the
    mismatch is REPORTED, never raised — a fallback mode is still
    perfectly usable, and one camera must never take a server down.

    Callers are expected to pace in software regardless of what the
    device claims (see cammanager._run and campreview.run); the rate
    half of the note says what pacing is protecting against.
    """
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
           int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    notes = []
    if got != (width, height):
        notes.append(f"asked {width}x{height}, "
                     f"device gave {got[0]}x{got[1]}")
    # The device's reported rate is what the MODE runs at, not what we
    # requested — treat a mismatch as information, not as a failure,
    # because software pacing is what actually holds the rate.
    dev_fps = cap.get(cv2.CAP_PROP_FPS)
    if dev_fps and fps and abs(dev_fps - fps) > 1.0:
        notes.append(f"device runs {dev_fps:g} fps natively, "
                     f"pacing to {fps:g}")
    return f"{got[0]}x{got[1]}", "; ".join(notes) or None


def set_queue_depth(cap: cv2.VideoCapture, want_fps: float) -> bool:
    """Shallow driver queue ONLY when we are pacing below the device.

    Returns whether the queue was made shallow, so callers can report it.

    A one-frame queue keeps a PACED loop honest: it sleeps most of the
    period, and without a shallow queue it would wake to a frame that was
    captured while it slept. That is the reason `CAP_PROP_BUFFERSIZE = 1`
    was here unconditionally.

    It is also why 1080p ran at half rate for weeks. Measured on cell1,
    2026-07-28, ELP-USBFHD01M-L36 with JPEG re-encode in the loop:

        1920x1080  bufsize=1        14.9 fps   read 57.2 ms
        1920x1080  driver default   29.8 fps   read 25.8 ms
        1280x720   bufsize=1        29.8 fps   read 26.8 ms
        640x480    bufsize=1        49.3 fps   read 17.0 ms

    With one buffer the driver cannot fill the next frame while we are
    still working on this one, so any loop whose per-frame work exceeds
    the frame period drops to a subharmonic — every second frame, hence
    almost exactly half of 30. Below 1080p the work is small enough that
    it never bites, which is why this hid in plain sight.

    So the depth follows the intent: **pacing well below what the device
    produces wants freshness; running flat out wants throughput.** When
    we consume every frame anyway, a deep queue costs no staleness — the
    frame we get is the one that just arrived either way.
    """
    dev_fps = cap.get(cv2.CAP_PROP_FPS)
    # No rate asked for (unpaced) or no rate reported → assume flat out.
    shallow = bool(want_fps > 0 and dev_fps > 0
                   and want_fps < dev_fps * QUEUE_SHALLOW_RATIO)
    if shallow:
        # Best effort: not all V4L2 backends honour it, which is why
        # pacing never depends on it.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return shallow


def open_camera(index: int, width: int, height: int,
                fps: float = 0.0) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise BenchError(
            f"could not open camera {index}",
            "check the USB connection; try --camera 1 (or 2); on Linux, "
            "`ls /dev/video*` shows what exists",
        )
    # Request MJPEG BEFORE the frame size: over USB 2.0 the ELP camera
    # otherwise negotiates uncompressed YUY2, which silently caps 1080p at
    # ~5 fps. Best-effort — backends that ignore FOURCC just keep working.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps > 0:
        cap.set(cv2.CAP_PROP_FPS, fps)
    set_queue_depth(cap, fps)
    _, note = describe_negotiated(cap, width, height, fps)
    if note:
        print(f"camera {index}: {note}", file=sys.stderr, flush=True)
    return cap


def load_camera_calibration(path: str | None):
    if not path:
        return None, None
    try:
        data = np.load(path)
        return data["K"], data["dist"]
    except Exception as exc:
        raise BenchError(f"could not load calibration {path}: {exc}",
                         "expected an .npz with arrays 'K' and 'dist'") from exc


def annotate(frame, detections, fps: float) -> None:
    h, w = frame.shape[:2]
    for det in detections:
        corners = det.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                     (0, 220, 0), 2)
        cx, cy = det.center.astype(int)
        cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
        cv2.putText(frame, str(det.tag_id), (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 0), 2)
    cv2.putText(frame, f"{w}x{h}  {fps:5.1f} fps  tags: {len(detections)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.campreview",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=0, help="camera index")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=PREVIEW_FPS_DEFAULT,
                        help=f"paced capture rate (default "
                             f"{PREVIEW_FPS_DEFAULT:g}; 0 = unpaced, read as "
                             "fast as the device delivers). These cameras "
                             "ignore the rate you ask them for and will run "
                             "up to 120 fps, so this is the only thing "
                             "holding the preview to a sane cost.")
    parser.add_argument("--calib", default=None,
                        help=".npz with K + dist to undistort")
    parser.add_argument("--view-width", type=int, default=VIEW_WIDTH_DEFAULT,
                        help="view window width in px (default "
                             f"{VIEW_WIDTH_DEFAULT}, ~quarter of a 1080p "
                             f"screen; clamped {VIEW_WIDTH_MIN}-"
                             f"{VIEW_WIDTH_MAX}; capture stays full "
                             "resolution)")
    parser.add_argument("--grab", type=int, default=0, metavar="N",
                        help="headless: save N annotated frames and exit")
    parser.add_argument("--outdir", default=".", help="snapshot directory")
    # Handled in main() before the parser runs, so it works with no other
    # arguments and never opens a device. Declared here for --help.
    parser.add_argument("--selftest", action="store_true",
                        help="run the pure-logic tests and exit "
                             "(no camera, no window)")
    args = parser.parse_args()

    if args.fps < 0:
        raise BenchError(f"--fps must be 0 or positive, got {args.fps:g}",
                         "0 means unpaced; omit the flag for the "
                         f"{PREVIEW_FPS_DEFAULT:g} fps default")

    K, dist = load_camera_calibration(args.calib)
    detector = Detector(families=TAG_FAMILY)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    view_width = min(VIEW_WIDTH_MAX, max(VIEW_WIDTH_MIN, args.view_width))
    if not args.grab:
        try:
            # WINDOW_NORMAL decouples window size from frame size (the
            # default AUTOSIZE locks the window to full capture resolution,
            # which fills the bench screen) and makes it drag-resizable.
            # Both flags are literally 0 — the OR documents intent; what
            # matters is the absence of WINDOW_AUTOSIZE (0x1).
            cv2.namedWindow("campreview",
                            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        except cv2.error as exc:
            raise BenchError(
                "cannot open a display window (headless session?)",
                "use --grab N to save frames without a window, or run over "
                "`ssh -X cell1`",
            ) from exc

    cap = open_camera(args.camera, args.width, args.height, args.fps)
    session = int(time.time())
    fps_counter = FpsCounter()
    saved = 0
    view_sized = False  # size once from the first frame's real aspect
    # Software pacing, same reasoning as cammanager._run: the device
    # ignores CAP_PROP_FPS, so this loop declining to read faster is the
    # only thing that holds the rate. Without it a preview window runs at
    # whatever the mode delivers — up to 120 fps for a picture a human is
    # looking at, with AprilTag detection on every one of those frames.
    period = 1.0 / args.fps if args.fps > 0 else 0.0
    next_frame = time.monotonic()
    try:
        while True:
            frame = read_frame(cap)
            if K is not None:
                frame = cv2.undistort(frame, K, dist)

            fps = fps_counter.tick()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            annotate(frame, detections, fps)

            if args.grab:
                path = outdir / f"campreview_{session}_{saved:03d}.png"
                cv2.imwrite(str(path), frame)
                saved += 1
                print(f"saved {path}  ({len(detections)} tags)")
                if saved >= args.grab:
                    return 0
                # Paced here too, so `--grab N` samples over N/fps seconds
                # rather than returning N near-identical frames from the
                # same instant. No window to service, so a plain sleep.
                delay = next_frame - time.monotonic()
                if period and delay > 0:
                    time.sleep(delay)
                next_frame = next_deadline(next_frame, period)
                continue

            if not view_sized:
                # The camera negotiates its own frame size (it may ignore
                # --width/--height), so derive the view height from the
                # first delivered frame — then never touch the size again,
                # so an operator drag-resize sticks.
                h, w = frame.shape[:2]
                cv2.resizeWindow("campreview", view_width,
                                 max(1, round(view_width * h / w)))
                view_sized = True
            cv2.imshow("campreview", frame)
            # waitKey IS the pacing sleep: it services the window's event
            # loop, so the remainder of the frame period is spent there
            # rather than in a time.sleep that would freeze the UI. Capped
            # at 50 ms per call so q/Esc still answer promptly at low
            # rates, and looped until the deadline. A keypress returns
            # early — that costs one early frame, and next_deadline puts
            # the schedule straight again.
            key = 255
            while True:
                delay_ms = 1
                if period:
                    remaining = next_frame - time.monotonic()
                    delay_ms = min(50, max(1, int(remaining * 1000)))
                pressed = cv2.waitKey(delay_ms) & 0xFF
                if pressed != 255:
                    key = pressed
                    break
                if not period or time.monotonic() >= next_frame:
                    break
            next_frame = next_deadline(next_frame, period)
            if key in (ord("q"), 27):
                return 0
            if key == ord("s"):
                path = outdir / f"campreview_{int(time.time())}.png"
                cv2.imwrite(str(path), frame)
                print(f"saved {path}")
    finally:
        cap.release()
        cv2.destroyAllWindows()


class _FakeCap:
    """Enough of cv2.VideoCapture for describe_negotiated, no device.

    `lies` reproduces the real ELP-USBFHD01M-L36: it accepts whatever
    mode you set and then reports 640x480 at its native 120.101 fps.
    """

    def __init__(self, width, height, fps, lies=False):
        self.lies, self._asked = lies, (width, height, fps)
        self.sets: dict = {}

    def set(self, prop, value):
        self.sets[prop] = value
        return True

    def get(self, prop):
        w, h, fps = self._asked
        if self.lies:
            w, h, fps = 640, 480, 120.101
        return float({cv2.CAP_PROP_FRAME_WIDTH: w,
                      cv2.CAP_PROP_FRAME_HEIGHT: h,
                      cv2.CAP_PROP_FPS: fps}.get(prop, 0))


def selftest() -> int:
    """Exercise the pure logic — no camera, no window, no display.

        uv run python -m hardware.bench.campreview --selftest
    """
    # 1. On schedule: exactly one period per step, no drift. The period is
    # long relative to the loop body, so nothing here should ever clamp.
    period, start = 0.05, time.monotonic()
    deadline = start
    for _ in range(5):
        deadline = next_deadline(deadline, period)
    assert abs(deadline - (start + 5 * period)) < 1e-9, deadline

    # 2. Behind schedule: pull up to now, do NOT bank the debt. This is
    # the invariant that stops a stuttered loop sprinting to catch up.
    late = next_deadline(time.monotonic() - 10.0, period)
    assert abs(late - time.monotonic()) < 0.05, late

    # 3. Unpaced (period 0) never puts a deadline in the future.
    assert next_deadline(time.monotonic() - 1.0, 0.0) <= time.monotonic()

    # 4. A well-behaved device says nothing — without this the note would
    # cry wolf on every camera and operators would learn to ignore it.
    got, note = describe_negotiated(_FakeCap(1280, 720, 30), 1280, 720, 30)
    assert (got, note) == ("1280x720", None), (got, note)

    # 5. The real camera's lie is caught, and the note names both halves.
    got, note = describe_negotiated(
        _FakeCap(640, 360, 10, lies=True), 640, 360, 10)
    assert got == "640x480", got
    assert "asked 640x360" in note and "device gave 640x480" in note, note
    assert "120.101 fps natively" in note and "pacing to 10" in note, note

    # 6. Asking for unpaced means the device's rate is not a mismatch —
    # there is no rate to disagree with.
    _, note = describe_negotiated(_FakeCap(640, 480, 0, lies=True),
                                  640, 480, 0)
    assert note is None, note

    # 7. Queue depth follows intent, not a constant. An unconditional
    # one-frame queue is what held 1080p to 14.9 fps instead of 29.8.
    def depth(want, native):
        cap = _FakeCap(0, 0, native)
        return set_queue_depth(cap, want), cap.sets

    # Pacing far below the device: freshness matters, go shallow.
    shallow, sets = depth(10, 120.101)
    assert shallow and sets.get(cv2.CAP_PROP_BUFFERSIZE) == 1, sets
    # Running at the device's own rate: a shallow queue would cost half
    # the throughput and buy nothing, because every frame is consumed.
    shallow, sets = depth(30, 30)
    assert not shallow and cv2.CAP_PROP_BUFFERSIZE not in sets, sets
    # Unpaced means flat out, which is the same case.
    assert not depth(0, 120.101)[0]
    # A device that reports no rate tells us nothing — do not guess.
    assert not depth(10, 0)[0]

    print("campreview selftest OK")
    return 0


def main() -> int:
    if "--selftest" in sys.argv:
        return selftest()
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
