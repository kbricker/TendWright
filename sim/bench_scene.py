"""Measured bench geometry — the real cell around the SO-101 (plan #658).

Not to be confused with `sim/scene.py`, which composes the P0 UR5e
digital-twin cell. This module is about the PHYSICAL bench: Kyle's desk,
its walls, and where the arm actually sits on it.

The #648 twin models the arm on an INFINITE ground plane at the base.
That is safe but blind: it protects the arm from itself and from a table
that extends forever, and knows nothing about a table EDGE, a wall, or a
fixture. This module carries the measured cell so the gate can eventually
protect the workspace too.

Measurements live in `bench_scene.json` as data, in whatever units they
were taken in (inches off a tape measure, by default) — a re-measure is
an edit to that file, never a code change.

DATUM (Kyle, 2026-07-26): origin is the back-left corner of the main
table TOP SURFACE, standing at the bench facing it. Everything on the
bench is POSITIVE in all three axes from there:

    +x  right, along the main run
    +y  forward, toward where you stand
    +z  up, off the table top

and anything BELOW the bench — legs, floor, under-table storage — is
negative z. So the floor sits at z = -(height_to_floor), and a wall's
height is measured up from the table top, not from the floor: the arm
is bolted to this surface and can only ever reach what is above it.

Measuring from a table corner is far easier than measuring from the
middle of a robot, so the transform into the arm's frame happens here.

WHAT IS NOT MEASURED IS NOT MODELLED. Every unmeasured value is null and
reported as absent — a guessed wall is worse than no wall, because the
gate would report CLEAR against a fiction. `missing()` lists exactly what
is absent so a clean gate is never mistaken for a safe workspace.

    uv run python -m sim.bench_scene              # summary + ASCII plan
    uv run python -m sim.bench_scene --view       # interactive viewer (desk)
    uv run python -m sim.bench_scene --render DIR # png views
    uv run python -m sim.bench_scene --save-xml F # MJCF you can open anywhere
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

from hardware.errors import BenchError

SCENE_JSON = Path("bench_scene.json")
# Tape measures speak inches here; the model speaks metres.
UNIT_TO_M = {"in": 0.0254, "mm": 0.001, "cm": 0.01, "m": 1.0}


@dataclass(frozen=True)
class Surface:
    """A rectangular slab of table top, datum coords, top face only."""

    name: str
    x: float
    y: float
    width: float
    depth: float
    notes: str = ""

    def corners(self) -> tuple[float, float, float, float]:
        """(x0, y0, x1, y1) in datum units."""
        return self.x, self.y, self.x + self.width, self.y + self.depth

    def contains(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self.corners()
        return x0 <= x <= x1 and y0 <= y <= y1


@dataclass(frozen=True)
class Wall:
    """A vertical face rising from the table, datum coords."""

    name: str
    x: float
    y: float
    width: float
    height: float
    yaw_deg: float = 0.0
    notes: str = ""


@dataclass
class Scene:
    units: str
    surfaces: list[Surface]
    walls: list[Wall] = field(default_factory=list)
    thickness: float | None = None
    height_to_floor: float | None = None
    arm_x: float | None = None
    arm_y: float | None = None
    arm_yaw_deg: float | None = None

    @property
    def to_m(self) -> float:
        return UNIT_TO_M[self.units]

    @property
    def arm_placed(self) -> bool:
        """Can anything be expressed in the ARM's frame yet?

        All three are required together: without yaw the table's extent
        cannot be rotated into the arm's frame at all, and a half-placed
        table is exactly the failure this module exists to prevent.
        """
        return None not in (self.arm_x, self.arm_y, self.arm_yaw_deg)

    def footprint(self) -> tuple[float, float, float, float]:
        """Bounding box over all table surfaces, datum units."""
        xs = [v for s in self.surfaces for v in (s.corners()[0], s.corners()[2])]
        ys = [v for s in self.surfaces for v in (s.corners()[1], s.corners()[3])]
        return min(xs), min(ys), max(xs), max(ys)

    def on_table(self, x: float, y: float) -> bool:
        """Is this datum point over any table surface?"""
        return any(s.contains(x, y) for s in self.surfaces)

    def to_arm_frame(self, x: float, y: float) -> tuple[float, float]:
        """Datum point -> the arm's frame, in METRES.

        The arm's frame is the twin's world: origin at the base, arm
        reaching toward -Y at pan zero. arm_yaw_deg says which datum
        direction that reach points along (0 = toward the front of the
        desk, i.e. +y datum), positive counter-clockwise seen from above.
        """
        if not self.arm_placed:
            raise BenchError(
                "the arm's position on the table is not measured yet",
                "fill in arm.x / arm.y / arm.yaw_deg in bench_scene.json")
        dx = (x - self.arm_x) * self.to_m
        dy = (y - self.arm_y) * self.to_m
        a = math.radians(self.arm_yaw_deg)
        rx = dx * math.cos(a) + dy * math.sin(a)
        ry = -dx * math.sin(a) + dy * math.cos(a)
        # +y datum is forward toward the operator, which is the direction
        # the arm reaches — and the twin reaches toward -Y. Hence the flip.
        return rx, -ry

    def missing(self) -> list[str]:
        """Everything unmeasured, in the order it blocks work."""
        gaps: list[str] = []
        if not self.arm_placed:
            which = [n for n, v in (("x", self.arm_x), ("y", self.arm_y),
                                    ("yaw_deg", self.arm_yaw_deg))
                     if v is None]
            gaps.append(
                f"arm placement on the table ({', '.join(which)}) - until "
                f"this is known the twin CANNOT use the real tabletop and "
                f"keeps its infinite ground plane")
        if self.thickness is None:
            gaps.append("table top thickness")
        if self.height_to_floor is None:
            gaps.append("table height, floor to top surface")
        if not self.walls:
            gaps.append("walls - none measured, so none are modelled")
        return gaps

    def describe(self) -> str:
        lines = [f"bench scene ({self.units}; datum = back-left corner of "
                 f"the main table)"]
        x0, y0, x1, y1 = self.footprint()
        lines.append(f"  table: {len(self.surfaces)} surface(s), footprint "
                     f"{x1 - x0:g} x {y1 - y0:g} {self.units}")
        for s in self.surfaces:
            sx0, sy0, sx1, sy1 = s.corners()
            lines.append(f"    {s.name:<8} x {sx0:g}..{sx1:g}   "
                         f"y {sy0:g}..{sy1:g}   ({s.width:g} x {s.depth:g})")
        if self.thickness is not None:
            lines.append(f"  top thickness:   {self.thickness:g} {self.units}")
        if self.height_to_floor is not None:
            lines.append(f"  height to floor: {self.height_to_floor:g} "
                         f"{self.units}")
        if self.arm_placed:
            lines.append(f"  arm at x {self.arm_x:g}, y {self.arm_y:g}, "
                         f"facing {self.arm_yaw_deg:g} deg")
            if not self.on_table(self.arm_x, self.arm_y):
                lines.append("    WARNING: that point is not over any "
                             "measured table surface")
        for w in self.walls:
            lines.append(f"  wall {w.name}: at x {w.x:g}, y {w.y:g}, "
                         f"{w.width:g} wide, {w.height:g} high")
        return "\n".join(lines)

    def sketch(self, cols: int = 62) -> str:
        """Rough top-down ASCII plan — a cheap way to catch a number typed
        into the wrong field, which a table of figures hides well."""
        x0, y0, x1, y1 = self.footprint()
        w, d = x1 - x0, y1 - y0
        rows = max(6, round(cols * d / w / 2.2)) if w else 6
        grid = [[" "] * cols for _ in range(rows)]
        for s in self.surfaces:
            sx0, sy0, sx1, sy1 = s.corners()
            c0 = int((sx0 - x0) / w * (cols - 1))
            c1 = int((sx1 - x0) / w * (cols - 1))
            r0 = int((sy0 - y0) / d * (rows - 1))
            r1 = int((sy1 - y0) / d * (rows - 1))
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    edge = r in (r0, r1) or c in (c0, c1)
                    grid[r][c] = "+" if edge else "."
        # Walls last so they overwrite table edges — the corner gap is the
        # detail most worth seeing, and it lives exactly where they meet.
        for wall in self.walls:
            for step in range(int(wall.width) + 1):
                wx = wall.x + (step if wall.yaw_deg % 180 == 0 else 0)
                wy = wall.y + (0 if wall.yaw_deg % 180 == 0 else step)
                c = int((wx - x0) / w * (cols - 1))
                r = int((wy - y0) / d * (rows - 1))
                if 0 <= r < rows and 0 <= c < cols:
                    grid[r][c] = "#"
        if self.arm_placed:
            c = int((self.arm_x - x0) / w * (cols - 1))
            r = int((self.arm_y - y0) / d * (rows - 1))
            if 0 <= r < rows and 0 <= c < cols:
                grid[r][c] = "A"
        body = "\n".join("  " + "".join(row).rstrip() for row in grid)
        return (f"  top-down (back of the desk at the top, "
                f"{w:g} x {d:g} {self.units})\n"
                f"  # wall   + table edge   . table top"
                f"{'   A arm' if self.arm_placed else ''}\n{body}")


def _num(doc: dict, key: str, where: str, required: bool = True
         ) -> float | None:
    v = doc.get(key)
    if v is None:
        if required:
            raise BenchError(f"{where}: missing '{key}'",
                             "see bench_scene.json for the expected shape")
        return None
    if type(v) is bool or not isinstance(v, (int, float)):
        raise BenchError(f"{where}: '{key}' must be a number, got {v!r}",
                         "fix the value in bench_scene.json")
    return float(v)


def load_scene(path: Path = SCENE_JSON) -> Scene:
    """Load + strictly validate the measured bench geometry."""
    try:
        doc = json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise BenchError(
            f"no bench scene at {path}",
            "the cell geometry has not been measured yet; the twin runs "
            "on its infinite ground plane without it") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise BenchError(f"could not read {path}: {exc}",
                         "fix or delete the file") from exc
    if not isinstance(doc, dict) or doc.get("version") != 1:
        raise BenchError(f"{path}: unsupported scene format",
                         "expected {version: 1, units, table, arm, walls}")
    units = doc.get("units")
    if units not in UNIT_TO_M:
        raise BenchError(f"{path}: units must be one of "
                         f"{sorted(UNIT_TO_M)}, got {units!r}",
                         "record the units the tape measure was in")

    table = doc.get("table")
    if not isinstance(table, dict):
        raise BenchError(f"{path}: missing 'table' object", "")
    raw = table.get("surfaces")
    if not isinstance(raw, list) or not raw:
        raise BenchError(f"{path}: table.surfaces must be a non-empty list",
                         "at least one rectangle of table top is required")
    surfaces: list[Surface] = []
    seen: set[str] = set()
    for entry in raw:
        if not isinstance(entry, dict):
            raise BenchError(f"{path}: each surface must be an object", "")
        name = entry.get("name")
        if not isinstance(name, str) or not name:
            raise BenchError(f"{path}: every surface needs a name", "")
        if name in seen:
            raise BenchError(f"{path}: duplicate surface name {name!r}",
                             "surface names identify them in reports")
        seen.add(name)
        where = f"{path} surface {name!r}"
        wid = _num(entry, "width", where)
        dep = _num(entry, "depth", where)
        if wid <= 0 or dep <= 0:
            raise BenchError(f"{where}: width and depth must be positive", "")
        surfaces.append(Surface(
            name=name, x=_num(entry, "x", where), y=_num(entry, "y", where),
            width=wid, depth=dep, notes=entry.get("notes", "")))

    walls: list[Wall] = []
    for entry in doc.get("walls") or []:
        where = f"{path} wall {entry.get('name')!r}"
        h = _num(entry, "height", where)
        wid = _num(entry, "width", where)
        if h <= 0 or wid <= 0:
            raise BenchError(f"{where}: width and height must be positive", "")
        walls.append(Wall(
            name=entry.get("name", "wall"), x=_num(entry, "x", where),
            y=_num(entry, "y", where), width=wid, height=h,
            yaw_deg=_num(entry, "yaw_deg", where, required=False) or 0.0,
            notes=entry.get("notes", "")))

    arm = doc.get("arm") or {}
    where = f"{path} arm"
    return Scene(
        units=units, surfaces=surfaces, walls=walls,
        thickness=_num(table, "thickness", where, required=False),
        height_to_floor=_num(table, "height_to_floor", where, required=False),
        arm_x=_num(arm, "x", where, required=False),
        arm_y=_num(arm, "y", where, required=False),
        arm_yaw_deg=_num(arm, "yaw_deg", where, required=False))


WALL_T = 1.0  # rendered wall thickness, datum units - cosmetic only


def build_spec(scene: Scene):
    """MuJoCo spec of the measured bench, in the DATUM frame.

    Metres, table top at z=0, +z up. Geometry only - no arm: this exists
    to look at the measurements, and (once the arm is placed) to become
    the twin's real workspace instead of its infinite ground plane.

    mujoco is imported here, not at module scope, so loading and
    validating measurements never needs the physics stack.
    """
    import mujoco

    m = scene.to_m
    spec = mujoco.MjSpec()
    spec.compiler.degree = True
    # Offscreen framebuffer defaults to 640x480; renders are clipped to it.
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1200
    for s in scene.surfaces:
        x0, y0, x1, y1 = s.corners()
        t = (scene.thickness or 0.75) * m
        spec.worldbody.add_geom(
            name=f"table_{s.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[(x1 - x0) * m / 2, (y1 - y0) * m / 2, t / 2],
            pos=[(x0 + x1) / 2 * m, (y0 + y1) / 2 * m, -t / 2],
            rgba=[0.72, 0.60, 0.44, 1.0])
    for w in scene.walls:
        along_x = w.yaw_deg % 180 == 0
        # The wall FACE sits on the table edge; its body extends outward
        # (behind/left), which is where the real wall is.
        if along_x:
            half = [w.width * m / 2, WALL_T * m / 2, w.height * m / 2]
            pos = [(w.x + w.width / 2) * m, (w.y - WALL_T / 2) * m,
                   w.height * m / 2]
        else:
            half = [WALL_T * m / 2, w.width * m / 2, w.height * m / 2]
            pos = [(w.x - WALL_T / 2) * m, (w.y + w.width / 2) * m,
                   w.height * m / 2]
        spec.worldbody.add_geom(
            name=f"wall_{w.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=half, pos=pos, rgba=[0.85, 0.85, 0.88, 1.0])
    # Two lights: the default headlight alone flattens a box scene into
    # one grey mass, and the corner gap is the thing worth seeing.
    x0, y0, x1, y1 = scene.footprint()
    spec.worldbody.add_light(
        pos=[(x0 + x1) / 2 * m, (y0 + y1) / 2 * m, 2.2],
        dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.55, 0.55, 0.55])
    spec.worldbody.add_light(
        pos=[x1 * m, y1 * m, 1.6], dir=[-0.5, -0.5, -0.8],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.45, 0.45, 0.45])
    return spec


def build_model(scene: Scene):
    return build_spec(scene).compile()


def view(scene: Scene) -> int:
    """Open the bench in MuJoCo's interactive viewer.

    Needs a display, so this is a DESK command - cell1 is headless.
    Blocks until the window is closed.
    """
    import mujoco
    import mujoco.viewer

    model = build_model(scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print("opening the MuJoCo viewer - close the window to exit")
    print("  drag = orbit, right-drag = pan, scroll = zoom")
    if scene.missing():
        print("  NOTE: this is only what has been measured; see the plain "
              "`sim.bench_scene` output for what is absent")
    mujoco.viewer.launch(model, data)
    return 0


def render(scene: Scene, out_dir: Path, width: int = 1280,
           height: int = 860) -> list[Path]:
    """Render the bench from a few angles. Returns the files written."""
    import mujoco
    import numpy as np

    model = build_model(scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    x0, y0, x1, y1 = scene.footprint()
    m = scene.to_m
    centre = [(x0 + x1) / 2 * m, (y0 + y1) / 2 * m, 0.0]
    span = max(x1 - x0, y1 - y0) * m

    # Distance factors are multiples of the scene's largest span. To fit a
    # span across MuJoCo's default 45-degree fovy needs span/(2*tan(22.5))
    # = 1.21 spans; anything less puts the camera INSIDE the walls, which
    # is what the first attempt did.
    # MuJoCo's azimuth is the VIEW DIRECTION, not the camera's position:
    # az 35 puts the camera at -x,-y, i.e. outside both walls looking at
    # their backs. The walls live on the x=0 and y=0 edges, so the open
    # side - where the operator stands - needs the camera at +x,+y, which
    # is a view direction of about 225.
    views = {  # (azimuth, elevation, distance factor)
        # az 270 renders the plan MIRRORED in x (the left-side return
        # appears on the right), which would hide exactly the kind of
        # left/right error this view exists to catch. 90 reads true.
        "top": (90, -89, 1.75),
        "iso": (225, -40, 1.85),
        "operator": (255, -28, 1.8),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    with mujoco.Renderer(model, height=height, width=width) as r:
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        for name, (az, el, dist) in views.items():
            cam.lookat[:] = np.array(centre)
            cam.azimuth, cam.elevation = az, el
            cam.distance = span * dist
            r.update_scene(data, camera=cam)
            path = out_dir / f"bench_{name}.png"
            _write_png(path, r.render())
            written.append(path)
    return written


def _write_png(path: Path, rgb) -> None:
    """Minimal PNG writer - stdlib zlib only, no new dependency."""
    import struct
    import zlib

    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b""))


def main() -> int:
    render_to = None
    if "--render" in sys.argv:
        i = sys.argv.index("--render")
        render_to = Path(sys.argv[i + 1] if len(sys.argv) > i + 1
                         else "scene_render")
    save_xml = None
    if "--save-xml" in sys.argv:
        i = sys.argv.index("--save-xml")
        save_xml = Path(sys.argv[i + 1] if len(sys.argv) > i + 1
                        else "bench_scene.xml")
    try:
        scene = load_scene()
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2
    if save_xml is not None:
        save_xml.write_text(build_spec(scene).to_xml())
        print(f"wrote {save_xml}")
        print(f"  open it standalone with: uv run python -m mujoco.viewer "
              f"--mjcf={save_xml}")
        return 0
    if "--view" in sys.argv:
        return view(scene)
    if render_to is not None:
        for p in render(scene, render_to):
            print(f"wrote {p}")
        return 0
    print(scene.describe())
    print()
    print(scene.sketch())
    gaps = scene.missing()
    if gaps:
        print("\nNOT MODELLED (measure these - do not guess them):")
        for g in gaps:
            print(f"  - {g}")
        print("\nWhile anything above is unmeasured, a clean collision gate "
              "does NOT\nmean the workspace is safe - only that the arm "
              "will not hit itself.")
    else:
        print("\nthe cell is fully measured")
    return 0


if __name__ == "__main__":
    sys.exit(main())
