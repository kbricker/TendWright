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

DATUM (Kyle, 2026-07-26): origin is the INSIDE CORNER of the bench top -
where the main table meets the return and both walls meet. Everything is
positive from there:

    +x  along the RETURN, away from the back wall, toward where you stand
    +y  along the MAIN TABLE, away from the return corner
    +z  up, off the table top

Which axis goes where is forced by HANDEDNESS, not taste. Putting +x
along the main table and +y along the return gives x cross y = -z: a
left-handed frame, which silently mirrors the entire cell. That exact
mistake was made once here and only the 3D render caught it - the ASCII
plan drew the mirrored layout perfectly happily. So +x is the return and
+y is the main table, and it is asserted in code rather than trusted.

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


OUTWARD = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}


@dataclass(frozen=True)
class Wall:
    """A vertical face rising from the table, datum coords.

    `outward` is which way the wall's MATERIAL sits from its face - the
    face is on the table edge, the body is on the far side. It has to be
    stated rather than inferred: a wall on the x=82 edge and a wall on
    the x=0 edge both run along y, and guessing from position breaks the
    moment the table is re-zeroed.
    """

    name: str
    x: float
    y: float
    width: float
    height: float
    yaw_deg: float = 0.0
    outward: str = "-y"
    usable_height: float | None = None
    thickness: float = 1.0     # real wall thickness, datum units
    extends_below: bool = False  # does it continue down under the table?
    notes: str = ""

    @property
    def clear_height(self) -> float:
        """How far up the wall is actually reachable.

        The wall's own height and the space in FRONT of it are different
        numbers: ceiling ducts over the return cut its usable space to 34
        even though the wall itself runs to 48-7/8. Modelling that as a
        shorter wall would be a lie that happens to be conservative -
        and it would quietly move the wall face if anyone later measured
        the real thing. Keep both, and say which is which."""
        return self.height if self.usable_height is None else min(
            self.height, self.usable_height)


@dataclass(frozen=True)
class Fixture:
    """Anything solid held ABOVE the table top - shelf, duct, ceiling.

    `z` is the height of its UNDERSIDE above the table surface, because
    that is the number that decides whether the arm can pass beneath it.
    Null until measured - a shelf at a guessed height is the worst kind
    of obstacle, since the gate would clear a path that does not exist.
    """

    name: str
    x: float
    y: float
    width: float
    depth: float
    z: float | None = None
    thickness: float = 0.75
    kind: str = "shelf"
    notes: str = ""


@dataclass(frozen=True)
class Leg:
    """A table leg. `height` EXCLUDES the table top thickness - it is the
    leg itself, floor to the underside of the top, which is how Kyle
    measured them. Legs are the floor's only measurement: the top is
    level and the floor is not, so differing leg lengths ARE the slope."""

    name: str
    x: float
    y: float
    height: float
    section: str = "2x4"
    along: str = "y"          # which axis the wide (3.5) face spans
    truss: str | None = None  # legs sharing a name are one braced frame
    notes: str = ""


@dataclass(frozen=True)
class Foundation:
    """A concrete footing under a wall, below the table.

    Tied to a WALL by name rather than given free coordinates: the
    footing straddles the wall - it protrudes `front` into the room and
    `back` behind, with the wall's own thickness between - so its extent
    is only meaningful relative to that wall. Stating it independently
    would let the two drift apart on the next re-zero.

    It sits FLUSH on the floor, so it inherits the floor's tilt rather
    than being a level block with a gap under one end.
    """

    name: str
    wall: str
    front: float              # protrusion into the room, from the wall face
    back: float               # protrusion behind the wall
    height: float             # how tall, up from the floor
    notes: str = ""


@dataclass
class Scene:
    units: str
    surfaces: list[Surface]
    walls: list[Wall] = field(default_factory=list)
    fixtures: list[Fixture] = field(default_factory=list)
    legs: list[Leg] = field(default_factory=list)
    foundations: list[Foundation] = field(default_factory=list)
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

    def floor_plane(self) -> tuple[float, float, float] | None:
        """(z0, a, b) for  z = z0 + a*x + b*y  - the floor, in datum units.

        THE TABLE TOP IS LEVEL AND THE FLOOR IS NOT, so the legs are the
        only measurement of the floor: each leg's length is the distance
        from the floor to the underside of the top at that point, and
        differing lengths ARE the slope. Three legs define the plane;
        more are least-squared, so a fourth measurement improves it
        rather than conflicting with it. Returns None below three.
        """
        if len(self.legs) < 3 or self.thickness is None:
            return None
        under = -self.thickness
        rows = [(leg.x, leg.y, 1.0) for leg in self.legs]
        zs = [under - leg.height for leg in self.legs]
        # Normal equations, so this works for 3 legs or 30 without numpy
        # semantics changing under us.
        n = len(rows)
        sxx = sum(r[0] * r[0] for r in rows)
        sxy = sum(r[0] * r[1] for r in rows)
        syy = sum(r[1] * r[1] for r in rows)
        sx = sum(r[0] for r in rows)
        sy = sum(r[1] for r in rows)
        sxz = sum(r[0] * z for r, z in zip(rows, zs))
        syz = sum(r[1] * z for r, z in zip(rows, zs))
        sz = sum(zs)
        m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
        v = [sxz, syz, sz]
        det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
               - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
               + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
        if abs(det) < 1e-9:
            return None  # collinear legs cannot define a plane
        def solve_col(col: int) -> float:
            mm = [row[:] for row in m]
            for r in range(3):
                mm[r][col] = v[r]
            return (mm[0][0] * (mm[1][1] * mm[2][2] - mm[1][2] * mm[2][1])
                    - mm[0][1] * (mm[1][0] * mm[2][2] - mm[1][2] * mm[2][0])
                    + mm[0][2] * (mm[1][0] * mm[2][1] - mm[1][1] * mm[2][0])
                    ) / det
        a, b, z0 = solve_col(0), solve_col(1), solve_col(2)
        return z0, a, b

    def floor_z(self, x: float, y: float) -> float | None:
        plane = self.floor_plane()
        if plane is None:
            return None
        z0, a, b = plane
        return z0 + a * x + b * y

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
        if self.floor_plane() is None:
            gaps.append(f"the floor - it is DERIVED from leg lengths and "
                        f"needs at least 3 non-collinear legs (have "
                        f"{len(self.legs)})")
        if not self.walls:
            gaps.append("walls - none measured, so none are modelled")
        for sh in self.fixtures:
            if sh.z is None:
                gaps.append(
                    f"{sh.kind} '{sh.name}': height of its UNDERSIDE above the "
                    f"table top - NOT MODELLED without it, because that is "
                    f"the number deciding whether the arm passes beneath")
        return gaps

    def describe(self) -> str:
        lines = [f"bench scene ({self.units}; datum = the inside corner "
                 f"where the main table, the return and both walls meet)"]
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
            extra = ("" if w.usable_height is None
                     else f", usable {w.usable_height:g} (overhead blocks "
                          f"the rest)")
            lines.append(f"  wall {w.name}: at x {w.x:g}, y {w.y:g}, "
                         f"{w.width:g} wide, {w.height:g} high{extra}")
        plane = self.floor_plane()
        if plane is not None:
            z0, a, b = plane
            tilt = math.degrees(math.atan(math.hypot(a, b)))
            lines.append(f"  floor: z = {z0:.3f} + {a:.5f}x + {b:.5f}y  "
                         f"({tilt:.2f} deg tilt, solved from "
                         f"{len(self.legs)} legs)")
            for leg in self.legs:
                lines.append(f"    leg {leg.name:<20} x {leg.x:g}, "
                             f"y {leg.y:g}, {leg.height:g} long -> floor "
                             f"{self.floor_z(leg.x, leg.y):.2f}")
        for f in self.foundations:
            lines.append(f"  foundation {f.name}: under wall '{f.wall}', "
                         f"{f.front:g} in front + {f.back:g} behind, "
                         f"{f.height:g} tall, flush on the floor")
        for sh in self.fixtures:
            at = (f"underside {sh.z:g} above the top" if sh.z is not None
                  else "HEIGHT UNKNOWN - not modelled")
            lines.append(f"  {sh.kind} {sh.name}: x {sh.x:g}..{sh.x + sh.width:g}"
                         f", y {sh.y:g}..{sh.y + sh.depth:g}, {at}")
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


def _outward(entry: dict, where: str) -> str:
    v = entry.get("outward", "-y")
    if v not in OUTWARD:
        raise BenchError(f"{where}: outward must be one of "
                         f"{sorted(OUTWARD)}, got {v!r}",
                         "it is the direction the wall material sits from "
                         "its face")
    return v


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
            outward=_outward(entry, where),
            usable_height=_num(entry, "usable_height", where, required=False),
            thickness=_num(entry, "thickness", where, required=False) or 1.0,
            extends_below=bool(entry.get("extends_below", False)),
            notes=entry.get("notes", "")))

    fixtures: list[Fixture] = []
    for entry in (doc.get("fixtures") or doc.get("shelves") or []):
        where = f"{path} shelf {entry.get('name')!r}"
        wid = _num(entry, "width", where)
        dep = _num(entry, "depth", where)
        if wid <= 0 or dep <= 0:
            raise BenchError(f"{where}: width and depth must be positive", "")
        fixtures.append(Fixture(
            name=entry.get("name", "shelf"), x=_num(entry, "x", where),
            y=_num(entry, "y", where), width=wid, depth=dep,
            z=_num(entry, "z", where, required=False),
            thickness=_num(entry, "thickness", where, required=False) or 0.75,
            kind=entry.get("kind", "shelf"),
            notes=entry.get("notes", "")))

    legs: list[Leg] = []
    for entry in doc.get("legs") or []:
        where = f"{path} leg {entry.get('name')!r}"
        h = _num(entry, "height", where)
        if h <= 0:
            raise BenchError(f"{where}: height must be positive", "")
        section = entry.get("section", "2x4")
        if section not in LUMBER:
            raise BenchError(f"{where}: unknown section {section!r}",
                             f"known: {sorted(LUMBER)}")
        legs.append(Leg(
            name=entry.get("name", "leg"), x=_num(entry, "x", where),
            y=_num(entry, "y", where), height=h, section=section,
            along=entry.get("along", "y"), truss=entry.get("truss"),
            notes=entry.get("notes", "")))

    foundations: list[Foundation] = []
    for entry in doc.get("foundations") or []:
        where = f"{path} foundation {entry.get('name')!r}"
        wall_name = entry.get("wall")
        if not any(w.name == wall_name for w in walls):
            raise BenchError(
                f"{where}: wall {wall_name!r} not found",
                f"a footing straddles a wall, so it must name one: "
                f"{[w.name for w in walls]}")
        foundations.append(Foundation(
            name=entry.get("name", "foundation"), wall=wall_name,
            front=_num(entry, "front", where),
            back=_num(entry, "back", where),
            height=_num(entry, "height", where),
            notes=entry.get("notes", "")))

    arm = doc.get("arm") or {}
    where = f"{path} arm"
    return Scene(
        units=units, surfaces=surfaces, walls=walls, fixtures=fixtures, legs=legs, foundations=foundations,
        thickness=_num(table, "thickness", where, required=False),
        height_to_floor=_num(table, "height_to_floor", where, required=False),
        arm_x=_num(arm, "x", where, required=False),
        arm_y=_num(arm, "y", where, required=False),
        arm_yaw_deg=_num(arm, "yaw_deg", where, required=False))


WALL_T = 1.0  # rendered wall thickness, datum units - cosmetic only

# MuJoCo geom groups. The interactive viewer toggles these with the
# number keys, so category = group lets the room be built up in detail
# without the ductwork and ceiling burying the bench.
GROUP_TABLE, GROUP_WALL, GROUP_FIXTURE = 0, 1, 2
GROUP_OVERHEAD, GROUP_STRUCTURE, GROUP_FLOOR = 3, 4, 5
GROUP_NAMES = {GROUP_TABLE: "table", GROUP_WALL: "walls",
               GROUP_FIXTURE: "fixtures/shelves",
               GROUP_OVERHEAD: "ducts/ceiling",
               GROUP_STRUCTURE: "legs/frame/foundation",
               GROUP_FLOOR: "floor"}
FIXTURE_GROUP = {"duct": GROUP_OVERHEAD, "ceiling": GROUP_OVERHEAD}

# Nominal lumber is not actual lumber: a "2x4" is 1.5 x 3.5 inches.
LUMBER = {"2x4": (1.5, 3.5), "2x6": (1.5, 5.5), "2x4_half": (1.5, 1.75),
          "4x4": (3.5, 3.5)}


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
            rgba=[0.72, 0.60, 0.44, 1.0], group=GROUP_TABLE)
    for w in scene.walls:
        along_x = w.yaw_deg % 180 == 0
        ox, oy = OUTWARD[w.outward]
        t = w.thickness
        # The wall FACE sits on the table edge; its body extends outward,
        # which is where the real wall is.
        #
        # A wall that continues BELOW the table runs down to its footing,
        # and its bottom edge follows the sloping floor. A single box
        # cannot have a sloped bottom, so the below-table part is a
        # separate box reaching down to the LOWEST point of that edge -
        # it over-fills slightly at the high end, which is invisible
        # (it is under the table top and inside the footing) and errs
        # toward more material rather than less.
        low = 0.0
        if w.extends_below:
            found = next((f for f in scene.foundations if f.wall == w.name),
                         None)
            if found is not None and scene.floor_plane() is not None:
                ends = ([(w.x, w.y), (w.x + w.width, w.y)] if along_x
                        else [(w.x, w.y), (w.x, w.y + w.width)])
                low = min(scene.floor_z(px, py) for px, py in ends) +                     found.height
        span = w.height - low
        if along_x:
            half = [w.width * m / 2, t * m / 2, span * m / 2]
            pos = [(w.x + w.width / 2) * m, (w.y + oy * t / 2) * m,
                   (low + span / 2) * m]
        else:
            half = [t * m / 2, w.width * m / 2, span * m / 2]
            pos = [(w.x + ox * t / 2) * m, (w.y + w.width / 2) * m,
                   (low + span / 2) * m]
        spec.worldbody.add_geom(
            name=f"wall_{w.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=half, pos=pos, rgba=[0.85, 0.85, 0.88, 1.0],
            group=GROUP_WALL)
    for sh in scene.fixtures:
        if sh.z is None:
            continue  # unmeasured height: not modelled, by policy
        spec.worldbody.add_geom(
            name=f"shelf_{sh.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[sh.width * m / 2, sh.depth * m / 2, sh.thickness * m / 2],
            pos=[(sh.x + sh.width / 2) * m, (sh.y + sh.depth / 2) * m,
                 (sh.z + sh.thickness / 2) * m],
            rgba=[0.66, 0.55, 0.40, 1.0],
            group=FIXTURE_GROUP.get(sh.kind, GROUP_FIXTURE))

    # --- floor, legs, foundations (all below the table) ---
    plane = scene.floor_plane()
    fx0, fy0, fx1, fy1 = scene.footprint()
    if plane is not None:
        z0, pa, pb = plane
        # A tilted slab. zaxis takes the plane normal directly, which
        # beats deriving euler angles for a compound tilt.
        pad = 24.0
        cx, cy = (fx0 + fx1) / 2, (fy0 + fy1) / 2
        nz = 1.0 / math.sqrt(pa * pa + pb * pb + 1.0)
        spec.worldbody.add_geom(
            name="floor", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[(fx1 - fx0 + 2 * pad) * m / 2,
                  (fy1 - fy0 + 2 * pad) * m / 2, 0.5 * m],
            pos=[cx * m, cy * m, (scene.floor_z(cx, cy) - 0.5) * m],
            zaxis=[-pa * nz, -pb * nz, nz],
            rgba=[0.55, 0.54, 0.52, 1.0], group=GROUP_FLOOR)

    for leg in scene.legs:
        w_, d_ = LUMBER[leg.section]
        hx, hy = (d_, w_) if leg.along == "x" else (w_, d_)
        top_under = -(scene.thickness or 0.75)
        base = top_under - leg.height
        spec.worldbody.add_geom(
            name=f"leg_{leg.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[hx * m / 2, hy * m / 2, leg.height * m / 2],
            pos=[leg.x * m, leg.y * m, (base + leg.height / 2) * m],
            rgba=[0.78, 0.65, 0.44, 1.0], group=GROUP_STRUCTURE)

    # Legs sharing a truss name are one braced frame: a top rail under
    # the table top and a bottom rail on the floor, as in Kyle's photo.
    # Derived from the leg pair rather than listed separately, so moving
    # a leg cannot leave its rails behind.
    trusses: dict[str, list[Leg]] = {}
    for leg in scene.legs:
        if leg.truss:
            trusses.setdefault(leg.truss, []).append(leg)
    rail_w, rail_d = LUMBER["2x4"]
    for tname, members in trusses.items():
        if len(members) < 2:
            continue
        a, b = members[0], members[1]
        top_under = -(scene.thickness or 0.75)
        span_x, span_y = abs(b.x - a.x), abs(b.y - a.y)
        cx_, cy_ = (a.x + b.x) / 2, (a.y + b.y) / 2
        run_x = span_x >= span_y
        length = (span_x if run_x else span_y) + rail_w
        half = ([length * m / 2, rail_w * m / 2, rail_d * m / 2] if run_x
                else [rail_w * m / 2, length * m / 2, rail_d * m / 2])
        for label, ztop in (("top", top_under),
                            ("bottom", min(scene.floor_z(a.x, a.y),
                                           scene.floor_z(b.x, b.y))
                             + rail_d if plane else None)):
            if ztop is None:
                continue
            spec.worldbody.add_geom(
                name=f"rail_{tname}_{label}", type=mujoco.mjtGeom.mjGEOM_BOX,
                size=half, pos=[cx_ * m, cy_ * m, (ztop - rail_d / 2) * m],
                rgba=[0.78, 0.65, 0.44, 1.0], group=GROUP_STRUCTURE)

    for f in scene.foundations:
        w = next((wl for wl in scene.walls if wl.name == f.wall), None)
        if w is None or plane is None:
            continue  # nothing to straddle, or no floor to stand on
        along_x = w.yaw_deg % 180 == 0
        ox, oy = OUTWARD[w.outward]
        # Straddles the wall: `front` into the room, the wall's own
        # thickness, then `back` behind. Centre it on that whole span.
        total = f.front + w.thickness + f.back
        mid = (f.front - f.back - w.thickness) / 2 * -1  # about the face
        cx = w.x + (0 if along_x else ox * -mid)
        cy = w.y + (oy * -mid if along_x else 0)
        if along_x:
            cx = w.x + w.width / 2
            half = [w.width * m / 2, total * m / 2, f.height * m / 2]
            cy = w.y + oy * (w.thickness + f.back - f.front) / 2
        else:
            cy = w.y + w.width / 2
            half = [total * m / 2, w.width * m / 2, f.height * m / 2]
            cx = w.x + ox * (w.thickness + f.back - f.front) / 2
        base = scene.floor_z(cx, cy)
        nz = 1.0 / math.sqrt(pa * pa + pb * pb + 1.0)
        spec.worldbody.add_geom(
            name=f"foundation_{f.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=half, pos=[cx * m, cy * m, (base + f.height / 2) * m],
            zaxis=[-pa * nz, -pb * nz, nz],   # flush on the sloping floor
            rgba=[0.80, 0.79, 0.76, 1.0], group=GROUP_STRUCTURE)

    x0, y0, x1, y1 = scene.footprint()
    cx0, cy0 = (x0 + x1) / 2 * m, (y0 + y1) / 2 * m

    # Lighting. Two hard directional lights over bare boxes gave a flat,
    # muddy read with black shadows and a black void behind. A room is
    # mostly BOUNCE, so: strong ambient in the headlight, one key light
    # with shadows for shape, two soft fills to lift the shadow side, and
    # a gradient skybox so the background is not a void.
    spec.visual.headlight.ambient = [0.42, 0.42, 0.45]
    spec.visual.headlight.diffuse = [0.30, 0.30, 0.30]
    spec.visual.headlight.specular = [0.06, 0.06, 0.06]
    spec.add_texture(
        name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.32, 0.36, 0.42], rgb2=[0.10, 0.11, 0.14],
        width=256, height=256)
    key = spec.worldbody.add_light(
        pos=[cx0 - 0.9, cy0 + 1.1, 2.4], dir=[0.35, -0.42, -0.84],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.62, 0.61, 0.58], specular=[0.12, 0.12, 0.12])
    key.castshadow = True
    for pos, direction, level in (
            ([cx0 + 1.4, cy0 + 1.4, 1.7], [-0.5, -0.5, -0.7], 0.26),
            ([cx0, cy0 - 1.2, 1.9], [0.0, 0.62, -0.78], 0.20)):
        fill = spec.worldbody.add_light(
            pos=pos, dir=direction,
            type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
            diffuse=[level, level, level * 1.05])
        fill.castshadow = False

    # A fixed camera at the OPEN corner. Without one the interactive
    # viewer starts on its default free camera, which for this scene can
    # sit outside the walls looking at a flat grey face - navigation then
    # feels broken when it is only occluded. Press [ / ] in the viewer to
    # cycle onto it.
    ox = sum(OUTWARD[w.outward][0] for w in scene.walls) or -1
    oy = sum(OUTWARD[w.outward][1] for w in scene.walls) or -1
    n = math.hypot(ox, oy)
    cx, cy = (x0 + x1) / 2 * m, (y0 + y1) / 2 * m
    reach = max(x1 - x0, y1 - y0) * m * 1.1
    eye = [cx - ox / n * reach, cy - oy / n * reach, reach * 0.75]
    # Aim it by constructing the axes, not by guessing euler angles: a
    # MuJoCo camera looks along its own -z with +y up, so a hand-picked
    # euler triple points it at empty space (it rendered pure black).
    # xyaxes takes the camera's x and y axes directly.
    fx, fy, fz = (cx - eye[0], cy - eye[1], 0.15 - eye[2])
    fn = math.sqrt(fx * fx + fy * fy + fz * fz)
    fx, fy, fz = fx / fn, fy / fn, fz / fn
    # right = forward x world-up, then true-up = right x forward
    rx, ry, rz = fy * 1.0 - fz * 0.0, fz * 0.0 - fx * 1.0, 0.0
    rn = math.hypot(rx, ry) or 1.0
    rx, ry, rz = rx / rn, ry / rn, 0.0
    ux = ry * fz - rz * fy
    uy = rz * fx - rx * fz
    uz = rx * fy - ry * fx
    spec.worldbody.add_camera(
        name="bench", pos=eye, xyaxes=[rx, ry, rz, ux, uy, uz], fovy=50.0)
    # Pin the navigation statistics rather than letting them be inferred:
    # they set the viewer's zoom/pan rate and its near/far clipping.
    spec.stat.center = [cx, cy, 0.15]
    spec.stat.extent = reach
    return spec


def build_model(scene: Scene):
    return build_spec(scene).compile()


def view(scene: Scene, save_view: bool = True,
         path: Path = SCENE_JSON) -> int:
    """Open the bench in MuJoCo's interactive viewer.

    Needs a display, so this is a DESK command - cell1 is headless.
    Blocks until the window is closed.

    Uses launch_PASSIVE rather than launch, because passive hands back a
    handle whose camera can be read: whatever angle you leave the window
    at is saved to the scene file and restored next time (and reused by
    --render). launch() owns its camera and never gives it back.
    """
    import time

    import mujoco
    import mujoco.viewer

    model = build_model(scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print("opening the MuJoCo viewer - close the window to exit")
    print("  left-drag = orbit, right-drag = pan, scroll = zoom")
    print("  [ / ] cycles cameras (there is a fixed 'bench' one)")
    print("  number keys 0-5 toggle geom groups: "
          + ", ".join(f"{g}={n}" for g, n in sorted(GROUP_NAMES.items())))
    if save_view:
        print(f"  the view you leave it at is saved to {path}")
    if scene.missing():
        print("  NOTE: only what has been measured is here; run without "
              "--view to see what is absent")

    print(f"  edit {path} and it reloads automatically, keeping your view")

    # Reload loop. MuJoCo cannot swap a model inside a live viewer, so a
    # measurement change means a new window - but the saved camera is
    # restored immediately, so it reads as a refresh rather than a
    # restart. This exists because the iterate-measure-look loop was
    # otherwise close-the-window-and-retype every single time.
    while True:
        saved = load_view(path)
        last: dict | None = None
        stamp = _mtime(path)
        reload_wanted = False

        with mujoco.viewer.launch_passive(model, data) as v:
            v.opt.geomgroup[:] = 1  # show every group; number keys toggle
            if saved:
                v.cam.azimuth = saved["azimuth"]
                v.cam.elevation = saved["elevation"]
                v.cam.distance = saved["distance"]
                v.cam.lookat[:] = saved["lookat"]
            while v.is_running():
                # Captured every tick, not on exit: once the window closes
                # the handle is dead and the camera cannot be read.
                last = {"azimuth": round(float(v.cam.azimuth), 3),
                        "elevation": round(float(v.cam.elevation), 3),
                        "distance": round(float(v.cam.distance), 5),
                        "lookat": [round(float(c), 5) for c in v.cam.lookat]}
                if _mtime(path) != stamp:
                    reload_wanted = True
                    break
                v.sync()
                time.sleep(1 / 60)

        if save_view and last is not None:
            try:
                # Reads the file to preserve whatever was just edited, so
                # it can catch an editor mid-write. Losing a camera angle
                # is not worth crashing the viewer over.
                store_view(path, last)
            except (OSError, json.JSONDecodeError, ValueError) as exc:
                print(f"  could not save the view ({exc}); carrying on",
                      file=sys.stderr)
            if not reload_wanted:
                print(f"saved view: azimuth {last['azimuth']:g}, elevation "
                      f"{last['elevation']:g}, distance {last['distance']:g}")
        if not reload_wanted:
            return 0

        # Re-read. A half-written file (an editor mid-save) raises; wait
        # and retry rather than dying on a transient.
        for attempt in range(20):
            time.sleep(0.15)
            try:
                scene = load_scene(path)
                break
            except BenchError as exc:
                if attempt == 19:
                    print(f"reload failed: {exc}", file=sys.stderr)
                    return 2
        model = build_model(scene)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        print("scene changed - reloaded")
        for gap in scene.missing():
            print(f"  still missing: {gap}")


def _mtime(path: Path) -> float:
    try:
        return path.stat().st_mtime
    except OSError:
        return 0.0


def load_view(path: Path = SCENE_JSON) -> dict | None:
    """The saved free-camera pose, if the scene file carries one."""
    try:
        doc = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    v = doc.get("view")
    if not isinstance(v, dict):
        return None
    try:
        if not (isinstance(v["lookat"], list) and len(v["lookat"]) == 3):
            return None
        return {"azimuth": float(v["azimuth"]),
                "elevation": float(v["elevation"]),
                "distance": float(v["distance"]),
                "lookat": [float(c) for c in v["lookat"]]}
    except (KeyError, TypeError, ValueError):
        return None  # a malformed view is not worth failing the tool over


def store_view(path: Path, view_pose: dict) -> None:
    """Write the camera pose back, leaving every measurement untouched."""
    doc = json.loads(path.read_text())
    doc["view"] = view_pose
    doc["_view_note"] = ("Saved free-camera pose from the last --view "
                         "session (MuJoCo units, metres). Restored on the "
                         "next --view and used by --render. Delete this "
                         "key to go back to the derived angles.")
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n")
    tmp.replace(path)


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
    # MuJoCo's azimuth is the VIEW DIRECTION, not the camera's position,
    # so a camera standing at the open side needs an azimuth pointing INTO
    # the scene. DERIVE it from the walls instead of hardcoding: each
    # wall's outward vector points away from the room, so their sum points
    # at the open corner, and the view direction is the reverse. Hardcoded
    # angles silently rendered the BACKS of the walls the moment the return
    # changed sides (2026-07-26).
    ox = sum(OUTWARD[w.outward][0] for w in scene.walls)
    oy = sum(OUTWARD[w.outward][1] for w in scene.walls)
    # Calibrated against the known-good case: walls outward (-1,-1) wanted
    # azimuth 225, and atan2(-1,-1) = 225. So it is atan2(oy, ox) directly
    # - negating it points the camera at the backs of the walls.
    base = (math.degrees(math.atan2(oy, ox)) % 360) if (ox or oy) else 225
    views = {  # (azimuth, elevation, distance factor)
        # az 270 renders the plan MIRRORED in x, which would hide exactly
        # the kind of left/right error this view exists to catch. 90 reads
        # true: +x to the right, back wall at the bottom.
        "top": (90, -89, 1.75),
        "iso": (base, -40, 1.85),
        "operator": (base + 30, -28, 1.8),
    }
    # A view saved from the interactive viewer wins - if Kyle found an
    # angle he likes, that is the one worth rendering.
    saved = load_view()
    if saved:
        views["saved"] = (saved["azimuth"], saved["elevation"], None)

    # MuJoCo enables only geom groups 0-2 by default, so the floor
    # (group 5) and legs/foundation (group 4) render invisible unless
    # every group is switched on explicitly.
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    opt.geomgroup[:] = 1

    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    with mujoco.Renderer(model, height=height, width=width) as r:
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        for name, (az, el, dist) in views.items():
            if dist is None and saved:  # the saved pose, verbatim
                cam.lookat[:] = np.array(saved["lookat"])
                cam.distance = saved["distance"]
            else:
                cam.lookat[:] = np.array(centre)
                cam.distance = span * dist
            cam.azimuth, cam.elevation = az, el
            r.update_scene(data, camera=cam, scene_option=opt)
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
