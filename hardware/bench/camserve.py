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
    http://cell1:8081/debug/memory      JSON: where the heap went (#704)

ON-DEMAND CAPTURE — snap and KEEP, on a trigger rather than a timer:

    /cam/<name>/capture?label=pre-grasp     one camera
    /capture?label=pre-grasp                every camera, as one SET
    /capture?label=pre-grasp&cams=a,c       a named subset

Answers with a manifest: the set id, what each camera produced, and how
wide a window the set spans. Frames from one set share the set id in
their filenames, so three cameras' views of the same moment stay
correlated on disk without the manifest. A single camera is still a set
of one, so a workflow step reads the same either way.

A SET IS NOT AN INSTANT. Cameras on a shared uplink are grabbed one at
a time on purpose, so a set spans `spread_ms`; every frame carries its
own offset. Capture sets while the arm is STOPPED, or the frames are
different moments and the manifest will say so.

Cameras come from cameras.json (see `python -m hardware.bench.cameras`).
Each one is opened ONLY while something is watching it — eight cameras
share one USB2 uplink, so bandwidth is claimed on demand and released a
few seconds after the last viewer leaves. Tiles subscribe at a reduced
profile; the solo view gets full resolution.

PERCEPTION IS OPT-IN, PER REQUEST. Streaming a camera costs capture and
JPEG encode, nothing more. AprilTag detection is the one CPU-hot step
(~35 ms/frame at 1080p) and runs only while some consumer has asked for
it — add `?tags=1` to a stream, tile, or snapshot URL, or use the
toggle on the solo page:

    /cam/<name>/stream?tags=1           detection + overlay
    /cam/<name>/stream                  raw frames, no detector at all

`tags:` in cameras.json and `--no-tags` are VETOES: either can forbid
detection on a camera, neither can turn it on. Silence means off.
`/status` reports `detecting` (running right now) and `may_detect`
(permitted at all). Interval stills and capture sets never detect —
they are the frames calibration wants, and a drawn-on frame is not
measurable.

STILLS-FIRST: any camera with still_interval_s set has its full-res
frames written to disk on that interval whether or not anyone is
watching, with rotating retention. That runs without the viewer.

A camera that is unplugged or busy shows its error on its own tile and
never affects the others. This tool never touches the servo bus (and
never imports the servo SDK).

Usage: camserve [--registry FILE] [--listen PORT] [--no-tags]
                [--no-stills] [--stills-dir DIR] [--selftest]
"""

from __future__ import annotations

import argparse
import html
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

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
# Set manifests are tiny; keep far more of them than frames so a
# set's record outlives the images its per-camera rotation drops.
MANIFEST_KEEP = 2000
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
                if path == "/debug/memory":
                    return self._debug_memory()
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
                    if rest == "capture":
                        return self._capture([name])
                if path == "/capture":
                    return self._capture(None)
                self.send_error(404)
            except OSError:
                pass  # client left mid-write — never fatal

        # Capture is a side effect, so POST is the correct verb — but GET
        # is accepted too, deliberately: on a home-LAN bench tool, being
        # able to trigger a labelled grab from a browser bookmark or a
        # bare `curl URL` is worth more than the purity.
        do_POST = do_GET

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

        def _flag(self, name: str) -> bool:
            """Read a boolean query flag. SILENCE MEANS NO."""
            raw = (parse_qs(urlparse(self.path).query).get(name)
                   or [""])[0].strip().lower()
            return raw in ("1", "true", "yes", "on")

        def _wants_tags(self) -> bool:
            """Did this request ask for perception?

            SILENCE MEANS NO. That is the whole point of #713.6 — the
            brains belong to the consumer, not to the camera, so a
            request that does not ask for them does not pay for them.
            `tags:` in cameras.json and `--no-tags` are vetoes layered on
            top (see Camera.may_detect); neither can turn detection on.
            """
            return self._flag("tags")

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
            tags = self._wants_tags()
            focus = self._flag("focus")
            q = "&".join([p for p, on in (("tags=1", tags), ("focus=1", focus))
                          if on])
            src = f"/cam/{name}/stream" + (f"?{q}" if q else "")
            view = (f"<div class='err'>{trouble}</div>" if trouble else
                    f"<img src='{src}' style='max-width:100vw'>")
            # The toggle is not a nicety. Detection is off unless asked
            # for, so without a visible control an operator opening this
            # page sees no overlays and concludes tag detection is
            # broken. The link says what it costs, because it is the
            # single most expensive thing this server can be asked to do.
            if cam.may_detect:
                toggle = (f"<a href='/cam/{name}/?tags=0'>tag overlay: "
                          f"<b>on</b> &mdash; turn off</a>" if tags else
                          f"<a href='/cam/{name}/?tags=1'>tag overlay: off "
                          f"&mdash; turn on</a> <small>(~35 ms/frame)</small>")
            else:
                toggle = ("<small>tag overlay unavailable &mdash; disabled "
                          "for this camera in cameras.json, or the server "
                          "was started with --no-tags</small>")
            # Focusing aid: a live sharpness number burned into the frame,
            # so the barrel can be turned against a value instead of an
            # impression. Kept out of the way unless asked for.
            fq = ("?tags=1&focus=1" if tags else "?focus=1")
            ftog = (f"<a href='/cam/{name}/{'?tags=1' if tags else ''}'>"
                    f"focus meter: <b>on</b> &mdash; turn off</a>" if focus
                    else f"<a href='/cam/{name}/{fq}'>focus meter: off "
                         f"&mdash; turn on</a> <small>(maximise it while "
                         f"turning the lens barrel)</small>")
            body = (self._nav(name) +
                    f"<div style='padding:4px'>{view}"
                    f"<p>{html.escape(cam.spec.location or '')} "
                    f"&middot; {cam.spec.solo} &middot; {toggle} "
                    f"&middot; {ftog}</p></div>")
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
                    # What the device actually gave us, and how that differs
                    # from what was asked for. A camera silently running a
                    # different mode than the registry claims is the exact
                    # thing that hid a 10x frame-rate overrun for weeks.
                    "negotiated": cam.negotiated,
                    "mode_note": cam.mode_note,
                    "fps": round(cam.fps, 1),
                    # Whether the CPU-hot perception layer is running right
                    # now, and whether it is even permitted to. Detection
                    # is per-request as of #713.6, so "is it on" is no
                    # longer answerable from cameras.json alone — and an
                    # expensive thing running invisibly is how the last
                    # two bugs in this file stayed hidden.
                    "detecting": cam.detecting,
                    # Sharpness of the centre half, live, while someone is
                    # focusing. 0 when nobody asked. Only the TREND means
                    # anything -- see campreview.focus_score.
                    "focus_score": round(cam.focus_score, 1),
                    "may_detect": cam.may_detect,
                    "error": cam.error,
                    "still_interval_s": cam.spec.still_interval_s,
                })
            self._send(json.dumps({"cameras": out}, indent=2).encode(),
                       "application/json")

        def _debug_memory(self) -> None:
            """Where the heap actually went, and whether it is live (#704).

                /debug/memory                      read-only
                /debug/memory?raw=1                ...plus glibc's raw XML
                /debug/memory?trim=<pid>           ...then malloc_trim(0)

            ALWAYS ON, unlike --debug-memory, which switches on
            tracemalloc and taxes every allocation. An endpoint that needs
            a restart to exist cannot diagnose a leak that takes hours to
            show, because the restart destroys the evidence.

            `trim` TAKES THIS PROCESS'S PID, not a boolean, and that is
            deliberate. malloc_trim consolidates and unlinks free chunks,
            which can abort the process outright; a plain `?trim=1` was
            reachable by a browser prefetch, a bookmark or a stale tab,
            and firing it by accident at hour three of a soak destroys
            the specimen. Requiring the pid means the caller has looked
            at the running process. There is no auth on this server, so
            this is the only guard available and it is not security —
            it is a guard against the operator's own reflexes.

            Allocator internals belong in memprobe, not here; its module
            docstring carries the reasoning and the hazards.
            """
            from .memprobe import memory_report

            # send_error messages must be PURE ASCII: they go into the HTTP
            # status line and BaseHTTPRequestHandler encodes that latin-1,
            # so an em-dash raises UnicodeEncodeError inside the error
            # path and the client gets nothing at all. The selftest caught
            # exactly that here.
            want = (parse_qs(urlparse(self.path).query).get("trim") or [""])[0]
            trim = want.strip() == str(os.getpid())
            if want.strip() and not trim:
                return self.send_error(
                    400, f"?trim= must be this process's pid ({os.getpid()}); "
                         f"it can abort the server, so it is not a boolean")
            try:
                report = memory_report(trim=trim, raw=self._flag("raw"))
            except Exception as exc:        # a probe must not kill its patient
                # Explicitly NOT letting this escape: do_GET's outer
                # `except OSError: pass` would swallow a ctypes OSError
                # into no response and no log line, which reads as "the
                # endpoint is broken" instead of naming the failure.
                return self.send_error(503, f"memory probe failed: {exc!r}")
            self._send(json.dumps(report, indent=2).encode(),
                       "application/json")

        def _capture(self, names: list[str] | None) -> None:
            """Snap one camera, or every camera as one correlated set.

                /cam/bench/capture?label=pre-grasp     one camera
                /capture?label=pre-grasp               all of them
                /capture?label=pre-grasp&cams=a,b      a named subset

            Answers with the manifest, so the caller learns the set id,
            what each camera produced, and how wide a window the set
            actually spans. A single-camera capture is still a set of
            one - same id scheme, same manifest - so a workflow step
            reads the same either way and does not need two code paths.
            """
            query = parse_qs(urlparse(self.path).query)
            label = (query.get("label") or [""])[0]
            if names is None and query.get("cams"):
                names = [n for n in query["cams"][0].split(",") if n]
            try:
                manifest = capture_set(mgr, stills_dir, names, label)
            except BenchError as exc:
                return self.send_error(503, str(exc))
            body = json.dumps(manifest, indent=2).encode()
            # 200 if anything was captured, 503 if nothing was: a caller
            # that only checks the status code still cannot mistake a
            # total failure for a success. Partial sets are 200 with
            # complete=false, and gating callers must read that flag.
            code = 200 if any(f["ok"] for f in manifest["frames"]) else 503
            self._send(body, "application/json", code)

        def _snapshot(self, name: str) -> None:
            cam = mgr.get(name)
            try:
                jpeg = cam.grab(cam.spec.solo, tags=self._wants_tags())
            except BenchError as exc:
                return self.send_error(503, str(exc))
            self._send(jpeg, "image/jpeg")

        def _stream(self, name: str, tile: bool) -> None:
            cam = mgr.get(name)
            profile = cam.spec.tile if tile else cam.spec.solo
            # Read once and pass the SAME value to release, so the tag
            # refcount cannot drift no matter what happens in between.
            tags, focus = self._wants_tags(), self._flag("focus")
            run = cam.acquire(profile, tags, focus)  # raises before bytes
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
                cam.release(tags, focus)

    return Handler


# One set capture at a time, process-wide. Cameras on a shared USB
# uplink are opened one after another on purpose (see stills_loop's
# stagger); two overlapping sets would interleave those opens and put
# every camera on the bus at once, which is the exact bandwidth
# collision the staggering exists to avoid. A second request waits.
_SET_LOCK = threading.Lock()
_SET_LOCK_WAIT_S = 30.0


def _safe_label(label: str) -> str:
    """A label ends up in a filename, so keep it to safe characters and
    bounded length. Empty after cleaning = no label, not a crash."""
    keep = "".join(c if (c.isalnum() or c in "-_") else "-"
                   for c in label.strip())[:48].strip("-")
    return keep


def capture_set(mgr: CameraManager, outdir: Path,
                names: list[str] | None = None, label: str = "",
                ) -> dict:
    """Capture one still from each named camera as a correlated SET.

    Returns a manifest describing what was actually captured. Also
    written to disk next to the frames, so a set is self-describing
    later without the caller having kept anything.

    A SET IS NOT AN INSTANT, and the manifest is explicit about it.
    Cameras sharing a USB uplink are grabbed one at a time by design, so
    a three-camera set spans a window (`spread_ms`) rather than a moment.
    Every frame carries its own offset from the set's start. If the arm
    is moving, those are genuinely different moments and nothing here
    can pretend otherwise — which is why the intended use is a set
    captured while the arm is STOPPED at a keyframe.

    Partial failure is reported, never hidden: `complete` is false if any
    requested camera did not produce a frame. A caller gating on a set
    (a #671 vision checkpoint) must treat incomplete as FAILED, not as
    "use what we got".
    """
    wanted = list(names) if names else list(mgr.order)
    unknown = [n for n in wanted if n not in mgr.cameras]
    if unknown:
        raise BenchError(f"no camera named {unknown[0]!r}",
                         f"known cameras: {', '.join(mgr.order) or '(none)'}")
    tag = _safe_label(label)
    if not _SET_LOCK.acquire(timeout=_SET_LOCK_WAIT_S):
        raise BenchError("another capture set is still running",
                         "sets are serialized so cameras never all open at "
                         "once on a shared uplink; retry shortly")
    try:
        set_id = time.strftime("%Y%m%d-%H%M%S") + f"-{int(time.time() * 1000) % 1000:03d}"
        t0 = time.monotonic()
        frames: list[dict] = []
        for name in wanted:
            cam = mgr.get(name)
            offset_ms = round((time.monotonic() - t0) * 1000)
            entry: dict = {"camera": name, "t_offset_ms": offset_ms,
                           "ok": False, "path": None, "bytes": 0,
                           "error": None}
            try:
                jpeg = cam.grab(cam.spec.solo, timeout=STILL_TIMEOUT_S)
            except BenchError as exc:
                entry["error"] = str(exc)
                frames.append(entry)
                continue
            stem = f"{set_id}_{name}" + (f"_{tag}" if tag else "")
            cam_dir = outdir / name
            try:
                cam_dir.mkdir(parents=True, exist_ok=True)
                path = cam_dir / f"{stem}.jpg"
                path.write_bytes(jpeg)
                _rotate(cam_dir, cam.spec.still_keep)
            except OSError as exc:
                entry["error"] = f"could not write: {exc}"
                frames.append(entry)
                continue
            entry.update(ok=True, path=str(path), bytes=len(jpeg))
            frames.append(entry)
        spread = max((f["t_offset_ms"] for f in frames), default=0)
        manifest = {
            "set_id": set_id,
            "label": tag,
            "captured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                          time.gmtime()),
            "requested": wanted,
            "complete": all(f["ok"] for f in frames) and bool(frames),
            # the window the set spans - NOT an instant; see the docstring
            "spread_ms": spread,
            "frames": frames,
        }
        try:
            sets_dir = outdir / "sets"
            sets_dir.mkdir(parents=True, exist_ok=True)
            (sets_dir / f"{set_id}.json").write_text(
                json.dumps(manifest, indent=2))
            _rotate_json(sets_dir, MANIFEST_KEEP)
        except OSError as exc:
            manifest["manifest_error"] = str(exc)
        return manifest
    finally:
        _SET_LOCK.release()


def _rotate_json(d: Path, keep: int) -> None:
    docs = sorted(d.glob("*.json"))
    for old in docs[:-keep] if len(docs) > keep else []:
        try:
            old.unlink()
        except OSError:
            pass


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
    parser.add_argument("--debug-memory", metavar="FILE", default=None,
                        help="periodically log where memory is going, to "
                             "FILE (plan #704). Slows every allocation — "
                             "diagnostic only, never leave it on")
    parser.add_argument("--debug-memory-interval", type=float, default=300.0,
                        help="seconds between memory samples (default 300)")
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

    mem_stop = None
    if args.debug_memory:
        from .memprobe import start as _mem_start
        mem_path = Path(args.debug_memory)
        mem_stop = _mem_start(mem_path, args.debug_memory_interval)
        # Announced loudly on purpose: tracemalloc taxes every
        # allocation, so a run left in this mode would look like a
        # performance regression to whoever inherits it.
        print(f"DEBUG MEMORY ON -> {mem_path} every "
              f"{args.debug_memory_interval:.0f}s. This slows allocation; "
              f"turn it off once #704 is understood.")
    try:
        while True:  # main thread parks here so Ctrl+C lands cleanly
            time.sleep(3600)
    finally:
        stop.set()
        if mem_stop is not None:
            mem_stop.set()
        for step in (mgr.shutdown, server.shutdown, server.server_close):
            try:
                step()
            except (KeyboardInterrupt, OSError):
                continue
    return 0


def main() -> int:
    if "--selftest" in sys.argv:
        _selftest()
        return 0
    return run_tool(run)


def _selftest() -> None:
    """Capture sets end to end over REAL HTTP, against fake cameras.

    No hardware, no servo bus. Every acceptance is paired with a
    refusal, because a capture route that always says yes would pass a
    happy-path test and still be useless as a checkpoint.
    """
    import shutil
    import tempfile
    import urllib.error
    import urllib.request

    import cv2
    import numpy as np

    from .cameras import CameraSpec, Profile

    fails: list[str] = []

    def want(label: str, ok: bool) -> None:
        if not ok:
            fails.append(label)
        print(f"  [{'ok ' if ok else 'FAIL'}] {label}")

    class FakeCap:
        def __init__(self, path):
            self.path, self.fail = str(path), "dead" in str(path)
            self.props: dict = {}

        def isOpened(self):
            return not self.fail

        def set(self, prop, value):
            self.props[prop] = value
            return True

        def get(self, prop):
            """Model a well-behaved device: reports back what was set.
            cammanager reads these to detect the case where a real camera
            silently negotiates a different mode — see _record_negotiated.
            A double without get() makes the capture thread die on open,
            which is exactly how this was caught."""
            return float(self.props.get(prop, 0))

        def read(self):
            time.sleep(0.01)
            return True, np.zeros((8, 8, 3), np.uint8)

        def release(self):
            pass

    big, small = Profile(320, 240, 10), Profile(80, 60, 5)
    specs = [CameraSpec("a", "left", "/dev/fake/a", big, small, tags=False),
             CameraSpec("b", "mid", "/dev/fake/b", big, small, tags=False),
             CameraSpec("c", "right", "/dev/fake/c", big, small, tags=False)]
    real_cap, cv2.VideoCapture = cv2.VideoCapture, FakeCap
    tmp = Path(tempfile.mkdtemp(prefix="camserve-selftest-"))
    srv = None
    try:
        mgr = CameraManager(specs, tags=False)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(mgr, tmp))
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        base = f"http://127.0.0.1:{srv.server_address[1]}"

        def get(url: str):
            with urllib.request.urlopen(base + url, timeout=20) as r:
                return r.status, json.loads(r.read())

        # --- one camera, labelled
        code, one = get("/cam/b/capture?label=pre-grasp")
        want("a single camera captures on demand",
             code == 200 and one["complete"] and len(one["frames"]) == 1)
        want("...and is still a SET (same id scheme, one manifest), so a "
             "caller needs no second code path",
             bool(one["set_id"]) and one["frames"][0]["camera"] == "b")
        want("...with the label in the filename",
             "pre-grasp" in one["frames"][0]["path"])

        # --- all cameras, correlated
        code, allset = get("/capture?label=step3")
        want("all cameras capture as one set",
             code == 200 and allset["complete"]
             and len(allset["frames"]) == 3)
        want("...sharing ONE set id, which is what correlates them",
             all(allset["set_id"] in f["path"] for f in allset["frames"]))
        want("...each carrying its own offset, so the set is not claimed "
             "to be an instant",
             all("t_offset_ms" in f for f in allset["frames"])
             and allset["spread_ms"] >= 0)
        want("distinct sets get distinct ids",
             allset["set_id"] != one["set_id"])

        # --- a named subset
        code, sub = get("/capture?cams=a,c&label=pair")
        want("a named subset captures only those cameras",
             [f["camera"] for f in sub["frames"]] == ["a", "c"])

        # --- the manifest is on disk, and is the index
        man = tmp / "sets" / f"{allset['set_id']}.json"
        want("the set is self-describing on disk", man.exists())
        want("...and the manifest names every frame's path",
             json.loads(man.read_text())["frames"] == allset["frames"])

        # --- REFUSALS (the half that makes the acceptances mean anything)
        try:
            get("/cam/nope/capture")
            want("an unknown camera is refused", False)
        except urllib.error.HTTPError as exc:
            want("an unknown camera is refused", exc.code == 404)

        dead = CameraSpec("d", "-", "/dev/fake/dead", big, small, tags=False)
        mgr2 = CameraManager([specs[0], dead], tags=False)
        srv2 = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(mgr2, tmp))
        threading.Thread(target=srv2.serve_forever, daemon=True).start()
        b2 = f"http://127.0.0.1:{srv2.server_address[1]}"
        with urllib.request.urlopen(b2 + "/capture?label=partial",
                                    timeout=20) as r:
            partial = json.loads(r.read())
        want("a set with ONE dead camera is marked incomplete, not passed "
             "off as fine", partial["complete"] is False)
        want("...while still keeping the frames that did work",
             any(f["ok"] for f in partial["frames"]))
        want("...and says WHICH camera failed and why",
             any(f["error"] for f in partial["frames"] if not f["ok"]))
        try:
            urllib.request.urlopen(b2 + "/cam/d/capture", timeout=20)
            want("a capture where NOTHING worked is an error status, so a "
                 "caller checking only the code cannot mistake it", False)
        except urllib.error.HTTPError as exc:
            want("a capture where NOTHING worked is an error status, so a "
                 "caller checking only the code cannot mistake it",
                 exc.code == 503)
        srv2.shutdown()

        # --- labels reach filenames safely
        code, nasty = get("/capture?cams=a&label=../../etc/passwd%20x")
        want("a hostile label cannot escape the stills directory",
             ".." not in nasty["frames"][0]["path"]
             and Path(nasty["frames"][0]["path"]).resolve().is_relative_to(
                 tmp.resolve()))

        # --- #704: the leak endpoint answers on a RUNNING server. That
        # property is the point of it, so it is tested over HTTP rather
        # than by calling memory_report directly — memprobe's own
        # selftest already covers the numbers.
        code, mem = get("/debug/memory")
        want("memory can be diagnosed on a live server, without the "
             "restart that destroys the evidence",
             code == 200 and "rss_mb" in mem and "verdict" in mem)
        want("...and reading it does NOT switch on the expensive "
             "instrument — that is what lets it be always-on",
             mem["tracemalloc_on"] is False)
        want("...and it reports how much of the process it actually "
             "accounts for, so the verdict cannot be read as covering "
             "memory it never saw", "accounted_pct" in mem)
        want("...and the free-chunk COUNT, which is the number that "
             "found #704 while the byte ratio said 'ambiguous'",
             "free_chunks" in mem and "mean_chunk_bytes" in mem)
        want("the default is read-only — no trim unless asked",
             "trim" not in mem)
        code, rawed = get("/debug/memory?raw=1")
        want("...and glibc's raw XML is available when the summary is "
             "not enough",
             code == 200 and "raw_xml" in rawed and "raw_xml" not in mem)
        # The REFUSAL that matters most on this endpoint: malloc_trim can
        # abort the process, so a value a browser prefetch could produce
        # must not be enough to fire it.
        try:
            get("/debug/memory?trim=1")
            want("a bare ?trim=1 is REFUSED — it can abort the server, so "
                 "it must not be reachable by a prefetch or a stale tab",
                 False)
        except urllib.error.HTTPError as exc:
            want("a bare ?trim=1 is REFUSED — it can abort the server, so "
                 "it must not be reachable by a prefetch or a stale tab",
                 exc.code == 400)
        code, trimmed = get(f"/debug/memory?trim={os.getpid()}")
        want("...while the pid of this very process does fire it, so the "
             "guard is a speed bump for reflexes and not a lockout",
             code == 200 and "trim" in trimmed)

        # --- #713.6: perception is opt-in per REQUEST, proved over HTTP.
        # cammanager's selftest covers the refcount semantics; this covers
        # the chain from a query string to the capture loop, which is the
        # part a unit test cannot see.
        tagspec = CameraSpec("t", "-", "/dev/fake/t", big, small, tags=True)
        mgr3 = CameraManager([tagspec], tags=True)
        srv3 = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(mgr3, tmp))
        threading.Thread(target=srv3.serve_forever, daemon=True).start()
        b3 = f"http://127.0.0.1:{srv3.server_address[1]}"

        def cam_status():
            with urllib.request.urlopen(b3 + "/status", timeout=20) as r:
                return json.loads(r.read())["cameras"][0]

        def page(url: str) -> str:
            with urllib.request.urlopen(b3 + url, timeout=20) as r:
                return r.read().decode()

        def hold_stream(qs: str, secs: float = 1.5):
            """Open a stream, hold it briefly, then abandon it."""
            def pull():
                try:
                    with urllib.request.urlopen(
                            b3 + "/cam/t/stream" + qs, timeout=secs + 5) as r:
                        end = time.monotonic() + secs
                        while time.monotonic() < end and r.read(4096):
                            pass
                except Exception:
                    pass  # abandoning a stream is the normal case here
            t = threading.Thread(target=pull, daemon=True)
            t.start()
            return t

        def settles(pred, secs: float = 6.0) -> bool:
            end = time.monotonic() + secs
            while time.monotonic() < end:
                if pred():
                    return True
                time.sleep(0.05)
            return False

        want("a camera says whether perception is even permitted",
             cam_status()["may_detect"] is True)

        t_plain = hold_stream("", 2.0)
        want("a plain stream opens the camera...",
             settles(lambda: cam_status()["open"]))
        want("...and does NOT run detection, which is the whole point of "
             "713.6 — the brains belong to the consumer, not the camera",
             cam_status()["detecting"] is False)
        t_plain.join(timeout=10)

        t_tags = hold_stream("?tags=1", 2.0)
        want("asking for tags turns detection on",
             settles(lambda: cam_status()["detecting"] is True))
        t_tags.join(timeout=10)
        want("...and it stops again when that consumer leaves",
             settles(lambda: cam_status()["detecting"] is False))

        solo = page("/cam/t/")
        want("the solo page offers a way to turn the overlay ON — without "
             "a visible control, an operator sees no overlays and "
             "concludes detection is broken",
             "tags=1" in solo and "turn on" in solo)
        want("...and a way back off once it is on",
             "turn off" in page("/cam/t/?tags=1"))
        srv3.shutdown()
        mgr3.shutdown()
    finally:
        if srv is not None:
            srv.shutdown()
        cv2.VideoCapture = real_cap
        shutil.rmtree(tmp, ignore_errors=True)

    print("camserve selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    if fails:
        raise SystemExit(1)


if __name__ == "__main__":
    sys.exit(main())
