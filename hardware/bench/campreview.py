"""campreview — live camera view with AprilTag (tag36h11) detection overlay.

    uv run python -m hardware.bench.campreview                # camera 0
    uv run python -m hardware.bench.campreview --camera 1 --width 1920 --height 1080

Shows resolution + measured FPS, draws detected tag corners/centers/IDs.
Keys in the window: s = save a snapshot PNG, q/Esc = quit.
Headless cell1 note: run over `ssh -X cell1`, or use --grab to save N
frames to disk without a window.

Usage: campreview [--camera N] [--width W] [--height H] [--calib FILE.npz]
                  [--grab N] [--outdir DIR]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from pupil_apriltags import Detector

from .bus import BenchError, run_tool

TAG_FAMILY = "tag36h11"


def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise BenchError(
            f"could not open camera {index}",
            "check the USB connection; try --camera 1 (or 2); on Linux, "
            "`ls /dev/video*` shows what exists",
        )
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0, help="camera index")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--calib", default=None,
                        help=".npz with K + dist to undistort")
    parser.add_argument("--grab", type=int, default=0, metavar="N",
                        help="headless: save N annotated frames and exit")
    parser.add_argument("--outdir", default=".", help="snapshot directory")
    args = parser.parse_args()

    K, dist = load_calibration(args.calib)
    detector = Detector(families=TAG_FAMILY)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    cap = open_camera(args.camera, args.width, args.height)
    times: list[float] = []
    saved = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise BenchError("camera stopped returning frames",
                                 "unplug/replug the camera and retry")
            if K is not None:
                frame = cv2.undistort(frame, K, dist)

            times.append(time.monotonic())
            times = times[-30:]
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            annotate(frame, detections, fps)

            if args.grab:
                path = outdir / f"campreview_{saved:03d}.png"
                cv2.imwrite(str(path), frame)
                saved += 1
                print(f"saved {path}  ({len(detections)} tags)")
                if saved >= args.grab:
                    return 0
                continue

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
