"""Camera registry + discovery — the cell's camera bus, by name.

Plan #656. One camera was a --camera index; a fleet needs identity,
because /dev/video numbering shuffles across boots and the ELP boards
all report the SAME USB serial (identical firmware), so a serial can't
tell camera 3 from camera 7. What IS stable is the physical port path:
/dev/v4l/by-path/pci-...-usb-0:2.4:1.0-video-index0, where "0:2.4"
means hub port 4 on root port 2. Plug a camera into a different hub
port and its identity changes BY DESIGN — that's what makes it a
location, and why the hub ports get labelled when wired.

    uv run python -m hardware.bench.cameras discover   # what's plugged in
    uv run python -m hardware.bench.cameras list       # what's registered
    uv run python -m hardware.bench.cameras check      # registry vs reality

`discover` prints paste-ready registry entries: plug a camera in, run
it, copy the block into cameras.json, fill in the name and location.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

from hardware.errors import BenchError, make_run_tool

BY_PATH_DIR = Path("/dev/v4l/by-path")
REGISTRY_DEFAULT = "cameras.json"
FORMAT_VERSION = 1
# A UVC camera exposes several /dev/video nodes (capture + metadata);
# only the first interface of each function is the capture node.
CAPTURE_SUFFIX = "-video-index0"
NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
TILE_CAP = 9  # 3x3 — Kyle's starting cap

run_tool = make_run_tool("check the USB hub power and re-plug the camera")


@dataclass(frozen=True)
class Profile:
    """One capture configuration. `solo` is used for the single-camera
    view and for stills (full resolution); `tile` is the reduced one the
    3x3 view subscribes to, because N full-res MJPEG streams do not fit
    a shared USB2 uplink."""

    width: int
    height: int
    fps: int

    def __str__(self) -> str:
        return f"{self.width}x{self.height}@{self.fps}"


@dataclass(frozen=True)
class CameraSpec:
    name: str
    location: str
    path: str  # /dev/v4l/by-path/... (stable, encodes the hub port)
    solo: Profile
    tile: Profile
    tags: bool = True  # AprilTag overlay on this camera's views
    still_interval_s: float = 0.0  # 0 = no interval capture
    still_keep: int = 200  # rotating retention per camera
    notes: str = ""

    @property
    def present(self) -> bool:
        return Path(self.path).exists()


def _profile_from(doc: object, where: str) -> Profile:
    if not isinstance(doc, dict):
        raise BenchError(f"{where}: profile must be an object")
    try:
        w, h, fps = doc["width"], doc["height"], doc["fps"]
    except KeyError as exc:
        raise BenchError(f"{where}: profile needs width, height, fps") from exc
    for label, v in (("width", w), ("height", h), ("fps", fps)):
        if type(v) is not int or not 1 <= v <= 10000:
            raise BenchError(f"{where}: {label} must be a positive integer")
    return Profile(width=w, height=h, fps=fps)


def load_registry(path: str | Path = REGISTRY_DEFAULT) -> list[CameraSpec]:
    """Load + strictly validate cameras.json. Order is display order."""
    p = Path(path)
    if not p.exists():
        raise BenchError(
            f"no camera registry at {p}",
            "run `python -m hardware.bench.cameras discover` and paste the "
            "entries into cameras.json")
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise BenchError(f"could not read {p}: {exc}",
                         "fix the JSON, or re-generate it from discover") from exc
    if not isinstance(doc, dict) or doc.get("version") != FORMAT_VERSION:
        raise BenchError(f"{p} is not a v{FORMAT_VERSION} camera registry")
    entries = doc.get("cameras")
    if not isinstance(entries, list) or not entries:
        raise BenchError(f"{p} lists no cameras")

    specs: list[CameraSpec] = []
    seen_names: set[str] = set()
    seen_paths: set[str] = set()
    for n, entry in enumerate(entries):
        where = f"{p} camera #{n + 1}"
        if not isinstance(entry, dict):
            raise BenchError(f"{where}: must be an object")
        name = entry.get("name")
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise BenchError(
                f"{where}: name must be lowercase letters/digits/_/- "
                f"(got {name!r})",
                "the name appears in URLs and still filenames")
        if name in seen_names:
            raise BenchError(f"{where}: duplicate camera name {name!r}")
        path_s = entry.get("path")
        if not isinstance(path_s, str) or not path_s:
            raise BenchError(f"{where}: path must be a device path string")
        if path_s in seen_paths:
            raise BenchError(
                f"{where}: two cameras share the path {path_s!r}",
                "each entry needs its own hub port — re-run discover")
        interval = entry.get("still_interval_s", 0.0)
        if not isinstance(interval, (int, float)) or interval < 0:
            raise BenchError(f"{where}: still_interval_s must be >= 0")
        if 0 < interval < 1:
            raise BenchError(
                f"{where}: still_interval_s {interval} is below 1 s",
                "sub-second interval capture would hold the USB bus open; "
                "use a live view for that")
        keep = entry.get("still_keep", 200)
        if type(keep) is not int or not 1 <= keep <= 100000:
            raise BenchError(f"{where}: still_keep must be 1-100000")
        tags = entry.get("tags", True)
        if not isinstance(tags, bool):
            raise BenchError(f"{where}: tags must be true or false")
        solo = _profile_from(entry.get("solo"), where + " solo")
        tile = _profile_from(entry.get("tile"), where + " tile")
        seen_names.add(name)
        seen_paths.add(path_s)
        specs.append(CameraSpec(
            name=name, location=str(entry.get("location", "")),
            path=path_s, solo=solo, tile=tile, tags=tags,
            still_interval_s=float(interval), still_keep=keep,
            notes=str(entry.get("notes", ""))))
    return specs


# ------------------------------------------------------------- discovery
@dataclass
class Found:
    path: str
    node: str  # the /dev/videoN it currently resolves to
    port: str  # human summary of the USB port chain
    registered_as: str | None = None
    # other by-path spellings of this same camera (see discover)
    aliases: list[str] = field(default_factory=list)


def _port_of(by_path_name: str) -> str:
    """'pci-0000:04:00.3-usb-0:2.4:1.0-video-index0' -> 'root 2, hub port 4'

    Handles the `usbv2-` spelling too: an xhci controller exposes the
    same physical port under both its USB3 and USB2 companion buses."""
    m = re.search(r"usb(?:v\d+)?-\d+:([\d.]+):", by_path_name)
    if not m:
        return "?"
    chain = m.group(1).split(".")
    if len(chain) == 1:
        return f"root port {chain[0]} (direct, no hub)"
    return f"root port {chain[0]}, hub port {'.'.join(chain[1:])}"


def discover() -> list[Found]:
    """Every capture-capable camera currently attached, by stable path.

    ONE PER PHYSICAL CAMERA. An xhci controller publishes each port
    under two by-path names — `...-usb-0:1:1.0-...` and
    `...-usbv2-0:1:1.0-...` — which are different strings resolving to
    the SAME /dev/video node. Listing both would invite registering one
    camera twice under two names (and the registry's duplicate-path
    check could not catch it, because the paths genuinely differ). The
    non-`v2` spelling is canonical; the alias is reported, not offered.
    """
    if not BY_PATH_DIR.exists():
        raise BenchError(
            f"{BY_PATH_DIR} does not exist",
            "no V4L2 cameras have ever been attached, or this is not "
            "Linux — discovery runs on cell1")
    by_node: dict[str, Found] = {}
    for entry in sorted(BY_PATH_DIR.iterdir()):
        if not entry.name.endswith(CAPTURE_SUFFIX):
            continue
        try:
            node = entry.resolve().name
        except OSError:
            continue
        seen = by_node.get(node)
        if seen is None:
            by_node[node] = Found(path=str(entry), node=node,
                                  port=_port_of(entry.name))
            continue
        # Same camera under a second spelling: keep the canonical one.
        alias, canonical = str(entry), seen.path
        if "usbv2-" in canonical and "usbv2-" not in alias:
            canonical, alias = alias, canonical
            seen.path = canonical
            seen.port = _port_of(Path(canonical).name)
        seen.aliases.append(alias)
    return sorted(by_node.values(), key=lambda f: f.path)


def _suggest_entry(f: Found, index: int) -> dict:
    return {
        "name": f"cam{index}",
        "location": "TODO: where this camera looks",
        "path": f.path,
        "solo": {"width": 1920, "height": 1080, "fps": 30},
        # 640x480, NOT 640x360: the ELP boards have no 360-line mode and
        # silently substitute 480, which is 4:3 where solo is 16:9 — so a
        # 360 default framed the tile view differently from the solo view
        # and nothing said so (#713.7). Suggest a mode that exists.
        "tile": {"width": 640, "height": 480, "fps": 10},
        "tags": True,
        "still_interval_s": 0,
        "still_keep": 200,
    }


def cmd_discover(args: argparse.Namespace) -> int:
    found = discover()
    if not found:
        print("no cameras attached (nothing in /dev/v4l/by-path)")
        return 0
    # Mark what is already registered so adding camera 5 doesn't mean
    # re-diffing the four already named.
    known: dict[str, str] = {}
    if Path(args.registry).exists():
        try:
            known = {s.path: s.name for s in load_registry(args.registry)}
        except BenchError as exc:
            print(f"note: {args.registry} unreadable ({exc}) — treating "
                  f"every camera as new\n", file=sys.stderr)
    print(f"{len(found)} camera(s) attached:\n")
    for f in found:
        # An entry registered under an alias spelling is still registered.
        f.registered_as = known.get(f.path) or next(
            (known[a] for a in f.aliases if a in known), None)
        tag = f"registered as {f.registered_as}" if f.registered_as else "NEW"
        print(f"  {f.node:<12} {f.port:<34} {tag}")
        print(f"    {f.path}")
        for alias in f.aliases:
            print(f"    (same camera, alias: {alias})")

    fresh = [f for f in found if not f.registered_as]
    if not fresh:
        print(f"\nevery attached camera is already in {args.registry}")
        return 0
    n = len(known)
    print(f"\n{len(fresh)} new — paste into the \"cameras\" list of "
          f"{args.registry}, then set name + location:\n")
    # Entries only, comma-separated: this drops INTO an existing list
    # rather than replacing the document.
    blocks = [json.dumps(_suggest_entry(f, n + i), indent=2)
              for i, f in enumerate(fresh, 1)]
    print(",\n".join(blocks))
    return 0


def cmd_list(args: argparse.Namespace) -> int:
    specs = load_registry(args.registry)
    print(f"{args.registry} — {len(specs)} camera(s)")
    for s in specs:
        mark = "OK " if s.present else "MISSING"
        still = (f"stills every {s.still_interval_s:g}s (keep {s.still_keep})"
                 if s.still_interval_s else "no interval stills")
        print(f"  [{mark:>7}] {s.name:<12} {s.location}")
        print(f"            solo {s.solo}  tile {s.tile}  "
              f"tags {'on' if s.tags else 'off'}  {still}")
        print(f"            {s.path}")
    if len(specs) > TILE_CAP:
        print(f"\nnote: {len(specs)} cameras registered but the tile view "
              f"shows the first {TILE_CAP}")
    return 0


def cmd_check(args: argparse.Namespace) -> int:
    """Reconcile the registry against what is actually plugged in."""
    specs = load_registry(args.registry)
    found = {f.path: f for f in discover()}
    by_path = {s.path: s for s in specs}
    missing = [s for s in specs if s.path not in found]
    extra = [f for p, f in found.items() if p not in by_path]
    for s in specs:
        if s.path in found:
            print(f"  OK       {s.name:<12} -> {found[s.path].node} "
                  f"({found[s.path].port})")
    for s in missing:
        print(f"  MISSING  {s.name:<12} {s.location}")
        print(f"           nothing on {_port_of(Path(s.path).name)}")
    for f in extra:
        print(f"  UNKNOWN  {f.node} on {f.port} is not in the registry")
        print(f"           {f.path}")
    if missing or extra:
        print(f"\n{len(missing)} missing, {len(extra)} unregistered")
        print("MISSING = in the registry, not plugged in (or moved to a "
              "different hub port)")
        print("UNKNOWN = plugged in, not in the registry — run "
              "`cameras discover` for a paste-ready entry")
        return 1
    print(f"\nall {len(specs)} registered camera(s) present")
    return 0


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.cameras",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("discover", "list", "check"))
    parser.add_argument("--registry", default=REGISTRY_DEFAULT,
                        help=f"camera registry file (default {REGISTRY_DEFAULT})")
    args = parser.parse_args()
    return {"discover": cmd_discover, "list": cmd_list,
            "check": cmd_check}[args.command](args)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
