"""campreview — live camera view with AprilTag (tag36h11) detection overlay.

    uv run python -m hardware.bench.campreview                # camera 0
    uv run python -m hardware.bench.campreview --camera 1 --width 1920 --height 1080

Shows resolution + measured FPS, draws detected tag corners/centers/IDs.
Keys in the window: s = save a snapshot PNG, q/Esc = quit.
The view window opens 960 px wide (~quarter of the bench 1080p screen),
drag-resizable; override with --view-width. Capture, detection, and
snapshots always run at full camera resolution — only the view scales.
Headless cell1 note: run over `ssh -X cell1`, or use --grab to save N
frames to disk without a window.

Usage: campreview [--camera N] [--width W] [--height H] [--calib FILE.npz]
                  [--view-width PX] [--grab N] [--outdir DIR]
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

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from pupil_apriltags import Detector  # noqa: E402

# Camera tools import from hardware.errors, NOT .bus — keeps the Feetech
# servo SDK out of their import graph entirely (errors.py exists for this).
from hardware.errors import BenchError, make_run_tool  # noqa: E402

TAG_FAMILY = "tag36h11"
FPS_WINDOW = 30  # sliding-window samples for the FPS readout (~1-3 s)
# Default view-window width: 960x540 is a quarter of the bench's 1080p
# screen by area — Kyle never watches the preview at 100%. The clamp is a
# sanity envelope, not a screen query (single known bench monitor).
VIEW_WIDTH_DEFAULT = 960
VIEW_WIDTH_MIN, VIEW_WIDTH_MAX = 160, 3840

# Camera-flavored CLI wrapper (vs bus.py's servo-flavored unplug hint).
run_tool = make_run_tool("unplug/replug the camera and re-run")


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


def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture:
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
    return cap


def load_calibration(path: str | None):
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
    args = parser.parse_args()

    K, dist = load_calibration(args.calib)
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

    cap = open_camera(args.camera, args.width, args.height)
    session = int(time.time())
    fps_counter = FpsCounter()
    saved = 0
    view_sized = False  # size once from the first frame's real aspect
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
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                return 0
            if key == ord("s"):
                path = outdir / f"campreview_{int(time.time())}.png"
                cv2.imwrite(str(path), frame)
                print(f"saved {path}")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
