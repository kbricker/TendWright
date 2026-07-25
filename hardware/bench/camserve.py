"""camserve — the camera bus: many cameras, one viewer, interval stills.

    uv run python -m hardware.bench.camserve            # registry cameras
    uv run python -m hardware.bench.camserve --no-stills

Serves (LAN only, NO auth — never port-forward this):

    http://cell1:8081/                  picker: pick a camera, or All
    http://cell1:8081/all               3x3 tile view (first 9 cameras)
    http://cell1:8081/cam/<name>/       one camera, full resolution
    http://cell1:8081/cam/<name>/stream multipart MJPEG
    http://cell1:8081/cam/<name>/snapshot   single current JPEG (curl-able)
    http://cell1:8081/status            JSON: every camera's state

Cameras come from cameras.json (see `python -m hardware.bench.cameras`).
Each one is opened ONLY while something is watching it — eight cameras
share one USB2 uplink, so bandwidth is claimed on demand and released a
few seconds after the last viewer leaves. Tiles subscribe at a reduced
profile; the solo view gets full resolution.

STILLS-FIRST: any camera with still_interval_s set has its full-res
frames written to disk on that interval whether or not anyone is
watching, with rotating retention. That runs without the viewer.

A camera that is unplugged or busy shows its error on its own tile and
never affects the others. This tool never touches the servo bus (and
never imports the servo SDK).

Usage: camserve [--registry FILE] [--listen PORT] [--no-tags]
                [--no-stills] [--stills-dir DIR]
"""

from __future__ import annotations

import argparse
import html
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from hardware.errors import BenchError

from .cameras import TILE_CAP, load_registry
from .cammanager import CameraManager
from .campreview import run_tool

STREAM_FPS_CAP = 20.0
SEND_TIMEOUT_S = 10.0
MAX_IDLE_WAITS = 6  # x FrameBox wait = ~30 s of no frames -> reap client
STILLS_DIR_DEFAULT = "stills"
# Shorter than the interactive grab: interval capture runs unattended
# and one wedged camera blocking the whole schedule is worse than one
# missed frame.
STILL_TIMEOUT_S = 5.0
PAGE_CSS = (
    "body{margin:0;background:#111;color:#ddd;font:14px system-ui}"
    "a{color:#6cf;text-decoration:none}a:hover{text-decoration:underline}"
    "header{padding:8px 12px;background:#000;display:flex;gap:16px;"
    "align-items:center;flex-wrap:wrap}"
    ".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:4px;"
    "padding:4px}.cell{position:relative;background:#000;min-height:120px}"
    ".cell img{width:100%;display:block}"
    ".cap{position:absolute;left:0;bottom:0;background:#000a;padding:2px 6px}"
    ".err{padding:12px;color:#f88}"
)


def _page(title: str, body: str) -> bytes:
    return (f"<!doctype html><title>{html.escape(title)}</title>"
            f"<style>{PAGE_CSS}</style>{body}").encode()


def make_handler(mgr: CameraManager, stills_dir: Path):
    class Handler(BaseHTTPRequestHandler):
        timeout = SEND_TIMEOUT_S

        def log_message(self, fmt, *args):
            print(f"  {self.address_string()} {fmt % args}", file=sys.stderr)

        def _send(self, payload: bytes, ctype: str, code: int = 200) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        # ------------------------------------------------------- routes
        def do_GET(self):  # noqa: N802 (http.server API name)
            try:
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                if path == "/":
                    return self._picker()
                if path == "/all":
                    return self._tiles()
                if path == "/status":
                    return self._status()
                if path.startswith("/cam/"):
                    parts = path[len("/cam/"):].split("/")
                    name = parts[0]
                    rest = parts[1] if len(parts) > 1 else ""
                    if name not in mgr.cameras:
                        return self.send_error(404, f"no camera {name!r}")
                    if rest == "":
                        return self._solo_page(name)
                    if rest == "stream":
                        return self._stream(name, tile=False)
                    if rest == "tile":
                        return self._stream(name, tile=True)
                    if rest == "snapshot":
                        return self._snapshot(name)
                self.send_error(404)
            except OSError:
                pass  # client left mid-write — never fatal

        def _nav(self, here: str) -> str:
            links = [f"<a href='/cam/{n}/'>{html.escape(n)}</a>"
                     if n != here else f"<b>{html.escape(n)}</b>"
                     for n in mgr.order]
            allink = "<b>all</b>" if here == "all" else "<a href='/all'>all</a>"
            return (f"<header><a href='/'>cameras</a> {allink} "
                    + " ".join(links) + "</header>")

        def _picker(self) -> None:
            rows = []
            for name in mgr.order:
                cam = mgr.get(name)
                state = cam.error or ("live" if cam.profile else "idle")
                rows.append(
                    f"<li><a href='/cam/{name}/'>{html.escape(name)}</a>"
                    f" — {html.escape(cam.spec.location or 'unplaced')}"
                    f" <small>[{html.escape(state)}]</small></li>")
            body = (self._nav("") +
                    "<div style='padding:12px'>"
                    f"<p><a href='/all'>tile view (first {TILE_CAP})</a></p>"
                    "<ul>" + "".join(rows) + "</ul></div>")
            self._send(_page("cameras", body), "text/html")

        def _trouble(self, cam) -> str:
            """Why this camera has no picture, in words — a broken-image
            icon tells the operator nothing."""
            if cam.error:
                return html.escape(cam.error)
            if not cam.spec.present:
                return (f"not plugged in — nothing at {html.escape(cam.spec.path)}"
                        f" (run: python -m hardware.bench.cameras check)")
            return ""

        def _solo_page(self, name: str) -> None:
            cam = mgr.get(name)
            trouble = self._trouble(cam)
            view = (f"<div class='err'>{trouble}</div>" if trouble else
                    f"<img src='/cam/{name}/stream' style='max-width:100vw'>")
            body = (self._nav(name) +
                    f"<div style='padding:4px'>{view}"
                    f"<p>{html.escape(cam.spec.location or '')} "
                    f"&middot; {cam.spec.solo}</p></div>")
            self._send(_page(f"camera {name}", body), "text/html")

        def _tiles(self) -> None:
            cells = []
            for name in mgr.order[:TILE_CAP]:
                cam = mgr.get(name)
                trouble = self._trouble(cam)
                inner = (f"<div class='err'>{trouble}</div>" if trouble else
                         f"<img src='/cam/{name}/tile'>")
                cells.append(
                    f"<div class='cell'><a href='/cam/{name}/'>{inner}</a>"
                    f"<span class='cap'>{html.escape(name)}"
                    f"<br><small>{html.escape(cam.spec.location or '')}"
                    f"</small></span></div>")
            body = self._nav("all") + "<div class='grid'>" + "".join(cells) + "</div>"
            self._send(_page("all cameras", body), "text/html")

        def _status(self) -> None:
            out = []
            for name in mgr.order:
                cam = mgr.get(name)
                out.append({
                    "name": name,
                    "location": cam.spec.location,
                    "path": cam.spec.path,
                    "present": cam.spec.present,
                    "open": cam.profile is not None,
                    "profile": str(cam.profile) if cam.profile else None,
                    "fps": round(cam.fps, 1),
                    "error": cam.error,
                    "still_interval_s": cam.spec.still_interval_s,
                })
            self._send(json.dumps({"cameras": out}, indent=2).encode(),
                       "application/json")

        def _snapshot(self, name: str) -> None:
            cam = mgr.get(name)
            try:
                jpeg = cam.grab(cam.spec.solo)
            except BenchError as exc:
                return self.send_error(503, str(exc))
            self._send(jpeg, "image/jpeg")

        def _stream(self, name: str, tile: bool) -> None:
            cam = mgr.get(name)
            profile = cam.spec.tile if tile else cam.spec.solo
            run = cam.acquire(profile)  # raises before any bytes are sent
            try:
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seq, idle, last = 0, 0, 0.0
                min_period = 1.0 / min(STREAM_FPS_CAP, max(1, profile.fps))
                while True:
                    sleep = min_period - (time.monotonic() - last)
                    if sleep > 0:
                        time.sleep(sleep)
                    seq, jpeg = run.box.next_after(seq)
                    if jpeg is None:
                        if run.box.closed or cam.error:
                            return
                        idle += 1
                        if idle >= MAX_IDLE_WAITS:
                            return
                        continue
                    idle = 0
                    last = time.monotonic()
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            finally:
                cam.release()

    return Handler


def stills_loop(mgr: CameraManager, outdir: Path, stop: threading.Event,
                ) -> None:
    """Interval still capture — the stills-first path. Each camera keeps
    its own schedule; captures are STAGGERED so eight cameras never open
    at once on a shared uplink. A failure is logged and retried next
    tick, never fatal."""
    due: dict[str, float] = {}
    start = time.monotonic()
    for n, name in enumerate(mgr.order):
        cam = mgr.get(name)
        if cam.spec.still_interval_s:
            # stagger: spread first captures across the interval so eight
            # cameras never open at once on a shared uplink
            due[name] = start + (n * cam.spec.still_interval_s
                                 / max(1, len(mgr.order)))
    if not due:
        return
    print(f"interval stills: {len(due)} camera(s) -> {outdir}/")
    while not stop.wait(0.5):
        for name, when in list(due.items()):
            if time.monotonic() < when:
                continue
            cam = mgr.get(name)
            try:
                jpeg = cam.grab(cam.spec.solo, timeout=STILL_TIMEOUT_S)
            except BenchError as exc:
                print(f"  stills {name}: {exc}", file=sys.stderr)
                jpeg = None
            finally:
                # Schedule from COMPLETION, not from the sweep's start:
                # a slow or wedged grab must not make the next capture
                # instantly due (which would spin), and one bad camera
                # must not drag every other camera's schedule with it.
                due[name] = time.monotonic() + cam.spec.still_interval_s
            if jpeg is None:
                continue
            cam_dir = outdir / name
            cam_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            try:
                (cam_dir / f"{name}_{stamp}.jpg").write_bytes(jpeg)
                _rotate(cam_dir, cam.spec.still_keep)
            except OSError as exc:
                print(f"  stills {name}: could not write ({exc})",
                      file=sys.stderr)


def _rotate(cam_dir: Path, keep: int) -> None:
    shots = sorted(cam_dir.glob("*.jpg"))
    for old in shots[:-keep] if len(shots) > keep else []:
        try:
            old.unlink()
        except OSError:
            pass


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.camserve",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--registry", default="cameras.json",
                        help="camera registry (see the cameras tool)")
    parser.add_argument("--listen", type=int, default=8081,
                        help="HTTP port (LAN only, no auth; default 8081)")
    parser.add_argument("--no-tags", action="store_true",
                        help="raw video everywhere, skip AprilTag overlays")
    parser.add_argument("--no-stills", action="store_true",
                        help="viewer only — skip interval still capture")
    parser.add_argument("--stills-dir", default=STILLS_DIR_DEFAULT)
    args = parser.parse_args()

    specs = load_registry(args.registry)
    mgr = CameraManager(specs, tags=not args.no_tags)
    stills_dir = Path(args.stills_dir)

    try:
        server = ThreadingHTTPServer(
            ("", args.listen), make_handler(mgr, stills_dir))
    except OSError as exc:
        mgr.shutdown()
        raise BenchError(
            f"could not listen on port {args.listen}: {exc}",
            "is another camserve already running? pick a different "
            "--listen port") from exc
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    stop = threading.Event()
    stills_thread: threading.Thread | None = None
    if not args.no_stills:
        stills_thread = threading.Thread(
            target=stills_loop, args=(mgr, stills_dir, stop),
            name="stills", daemon=True)
        stills_thread.start()

    host = socket.gethostname()
    print(f"camera bus: {len(specs)} camera(s) from {args.registry}")
    for s in specs:
        print(f"  {s.name:<12} {s.location or '-'}  "
              f"{'present' if s.present else 'NOT PRESENT'}")
    print(f"serving http://{host}:{args.listen}/  "
          f"(picker, /all tiles, /cam/<name>/, /status)")
    print("LAN only, no auth — never port-forward this. Ctrl+C to stop.")
    try:
        while True:  # main thread parks here so Ctrl+C lands cleanly
            time.sleep(3600)
    finally:
        stop.set()
        for step in (mgr.shutdown, server.shutdown, server.server_close):
            try:
                step()
            except (KeyboardInterrupt, OSError):
                continue
    return 0


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
