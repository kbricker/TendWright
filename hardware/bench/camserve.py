"""camserve — LAN MJPEG stream of the bench camera, for remote iteration.

    uv run python -m hardware.bench.camserve                  # 720p, tags on
    uv run python -m hardware.bench.camserve --width 1920 --height 1080

Serves (LAN only, NO auth — never port-forward this):

    http://cell1:8081/          page embedding the live stream
    http://cell1:8081/stream    multipart MJPEG (browser / VLC / OpenCV)
    http://cell1:8081/snapshot  single current JPEG (curl-able)

The overlay (default on, --no-tags to disable) draws AprilTag detections —
the stream shows what the SOFTWARE sees, not just video. Ctrl+C stops the
server and releases the camera. This tool never touches the servo bus
(and never even imports the servo SDK).

Usage: camserve [--camera N] [--width W] [--height H] [--listen PORT]
                [--no-tags]
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import NoReturn

# Must be set before cv2 imports: silences the native layer's WARN/ERROR
# chatter so our clean one-line errors stay clean.
os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")

import cv2  # noqa: E402
from pupil_apriltags import Detector  # noqa: E402

from hardware.errors import BenchError  # noqa: E402
from .campreview import (FpsCounter, TAG_FAMILY, annotate, open_camera,  # noqa: E402
                         read_frame, run_tool)

JPEG_QUALITY = 80
STREAM_FPS_CAP = 20.0  # per-client send cap; capture runs at camera rate
FRAME_WAIT_S = 5.0  # client wait for a fresh frame before re-checking
# A stalled reader (zero-window TCP: sleeping phone, paused player) must
# error its handler thread out, not wedge it forever in wfile.write.
STREAM_SEND_TIMEOUT_S = 10.0
# With the camera stalled, a vanished client is undetectable (disconnects
# only surface on write) — after this many empty waits, drop the client so
# its thread is reaped; a live viewer just reconnects.
MAX_IDLE_WAITS = 6  # x FRAME_WAIT_S = ~30 s without frames

INDEX_HTML = (b"<!doctype html><title>camserve</title>"
              b"<body style='margin:0;background:#111;display:grid;"
              b"place-items:center;height:100vh'>"
              b"<img src='/stream' style='max-width:100vw;max-height:100vh'>")


class FrameBox:
    """Latest-JPEG buffer between the capture loop and client threads.

    Replaced wholesale under the lock; the Condition wakes waiting clients
    so nobody polls. A slow client always gets the LATEST frame next —
    frames are dropped for it, never queued. seq lets a client skip frames
    it has already sent; close() releases every waiter for shutdown.
    """

    def __init__(self):
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
        """Block until a frame newer than seq exists (or timeout/close);
        (same seq, None) means try again — or stop, if closed."""
        with self._cond:
            self._cond.wait_for(
                lambda: self.closed or self._seq > seq, FRAME_WAIT_S)
            if self.closed or self._seq <= seq:
                return seq, None
            return self._seq, self._jpeg


def make_handler(box: FrameBox):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # one terse line per request
            print(f"  {self.address_string()} {fmt % args}", file=sys.stderr)

        def _send_bytes(self, payload: bytes, ctype: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):  # noqa: N802 (http.server API name)
            try:
                if self.path in ("/", "/index.html"):
                    self._send_bytes(INDEX_HTML, "text/html")
                elif self.path == "/snapshot":
                    jpeg = box.latest()
                    if jpeg is None:
                        self.send_error(503, "no frame captured yet")
                    else:
                        self._send_bytes(jpeg, "image/jpeg")
                elif self.path == "/stream":
                    self._stream()
                else:
                    self.send_error(404)
            except OSError:
                pass  # client left / socket fault mid-write — never fatal

        def _stream(self) -> None:
            self.connection.settimeout(STREAM_SEND_TIMEOUT_S)
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = 0
            idle = 0
            min_period = 1.0 / STREAM_FPS_CAP
            last = 0.0
            while True:
                # Rate-cap BEFORE fetching, so the frame written after the
                # sleep is the freshest one, not a pre-sleep leftover.
                sleep = min_period - (time.monotonic() - last)
                if sleep > 0:
                    time.sleep(sleep)
                seq, jpeg = box.next_after(seq)
                if jpeg is None:
                    if box.closed:
                        return
                    idle += 1
                    if idle >= MAX_IDLE_WAITS:
                        return  # camera stalled — reap this client thread
                    continue
                idle = 0
                last = time.monotonic()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")

    return Handler


def capture_loop(cap: cv2.VideoCapture, box: FrameBox,
                 detector: Detector | None) -> NoReturn:
    """Read frames at camera rate, annotate, JPEG-encode into the box.
    Runs on the MAIN thread so Ctrl+C lands here; exits only by raising
    (dead camera -> BenchError, Ctrl+C -> KeyboardInterrupt)."""
    fps_counter = FpsCounter()
    while True:
        frame = read_frame(cap)
        fps = fps_counter.tick()
        if detector is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            annotate(frame, detector.detect(gray), fps)
        ok, buf = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            box.put(buf.tobytes())


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.camserve",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=0, help="camera index")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--listen", type=int, default=8081,
                        help="HTTP port (LAN only, no auth; default 8081)")
    parser.add_argument("--no-tags", action="store_true",
                        help="raw video, skip the AprilTag overlay")
    args = parser.parse_args()

    box = FrameBox()
    try:
        server = ThreadingHTTPServer(("", args.listen), make_handler(box))
    except OSError as exc:
        raise BenchError(
            f"could not listen on port {args.listen}: {exc}",
            "is another camserve already running? pick a different "
            "--listen port",
        ) from exc
    server.daemon_threads = True
    # Serve starts FIRST so the finally's server.shutdown() always has a
    # running serve_forever to stop (shutdown before serve_forever would
    # hang); clients just get 503 snapshots until the camera is up.
    threading.Thread(target=server.serve_forever, daemon=True).start()

    cap: cv2.VideoCapture | None = None
    try:
        detector = None if args.no_tags else Detector(families=TAG_FAMILY)
        cap = open_camera(args.camera, args.width, args.height)
        host = socket.gethostname()
        print(f"serving http://{host}:{args.listen}/  "
              f"(/stream, /snapshot; tags "
              f"{'off' if args.no_tags else 'on'})")
        print("LAN only, no auth — never port-forward this. "
              "Ctrl+C to stop.")
        capture_loop(cap, box, detector)
    finally:
        try:
            box.close()  # release every waiting client thread
            server.shutdown()
            server.server_close()
            if cap is not None:
                cap.release()
        except KeyboardInterrupt:
            pass  # a second Ctrl+C must not skip the remaining cleanup


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
