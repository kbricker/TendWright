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

TWO FILES, ON PURPOSE (Kyle, 2026-07-26):

    bench.json   the PLACE - table, walls, floor, footings, framing.
                 Measured once, project-agnostic, and the base for many
                 bench configurations.
    cell.json    THIS PROJECT's use of that place - which bench, where
                 the arm stands, where the cameras are, view prefs.

The bench must not know what is standing on it. Arm and camera
placement change constantly and belong to the project; the room does
not, and belongs to the room.

    uv run python -m sim.bench_scene              # summary + ASCII plan
    uv run python -m sim.bench_scene --bench-only # the place, no project
    uv run python -m sim.bench_scene --view       # interactive viewer (desk)
    uv run python -m sim.bench_scene --exercise   # play the routine (model only)
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

BENCH_JSON = Path("bench.json")   # the PLACE: reusable, project-agnostic
CELL_JSON = Path("cell.json")     # this project's use OF that place
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
    """A leg station in a table truss.

    `height` is the TRUSS HEIGHT: floor to the underside of the table
    top, excluding the 3/4 top. It is NOT the cut length of one 2x4 -
    the truss is legs plus rails, and no single piece is that long
    (Kyle, 2026-07-26). Do not hand this number to a saw.

    That distinction does not disturb the floor: floor-to-underside is
    exactly the quantity the plane is solved from, so truss height is
    the right measurement either way. Legs are the floor's ONLY
    measurement - the top is level and the floor is not, so differing
    truss heights ARE the slope.

    The rendered post spans the full truss height, with the rails drawn
    inside it. The real leg piece is shorter and the members overlap;
    that is a simplification of appearance, not of extent.
    """

    name: str
    x: float
    y: float
    height: float | None = None   # None = infer from the solved floor
    section: str = "2x4"
    along: str = "y"          # which axis the wide (3.5) face spans
    truss: str | None = None  # legs sharing a name are one braced frame
    measured: bool = True     # False = placed ON the floor, not defining it
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
    front: float              # protrusion into the room AT THE FLOOR
    back: float               # protrusion behind the wall
    height: float             # how tall, up from the floor
    front_top: float | None = None   # protrusion at the TOP, if battered
    level_top: bool = False          # level top over a sloping floor
    height_ref: tuple[float, float] | None = None  # where `height` was taken
    notes: str = ""

    @property
    def battered(self) -> bool:
        return self.front_top is not None and abs(
            self.front_top - self.front) > 1e-9


@dataclass(frozen=True)
class Ledger:
    """The 2x4 that carries the table off a wall, running its length.

    Fixed with its WIDE (nominal 4in, actually 3.5) face flat against
    the wall, so it stands 3.5 tall and projects only 1.5 into the room.
    Its top is flush with the underside of the table top - the top rests
    on it. A truss's top rail meets it in a HALF-LAP: a 1/2-thickness
    cutout in this piece. Modelled as plain overlapping boxes; the lap
    is joinery, not extent, and MuJoCo only cares about the envelope.
    """

    name: str
    wall: str
    section: str = "2x4"
    start: float | None = None   # along the wall's run; None = wall start
    end: float | None = None     # None = wall end
    notes: str = ""


@dataclass(frozen=True)
class Beam:
    """A support running along a table EDGE rather than a wall.

    Axis-aligned, top flush under the table top - the top rests on it,
    same as a ledger. ORIENTATION ASSUMPTION: stood on edge (3.5 tall,
    1.5 thick) like a joist, which is the structural default for an
    edge support; the wall ledgers are the other way up because Kyle
    said their wide face lies on the wall.
    """

    name: str
    x0: float
    y0: float
    x1: float
    y1: float
    section: str = "2x4"
    z: float | None = None    # TOP of the beam; None = under the table top
    notes: str = ""


@dataclass(frozen=True)
class Brace:
    """A DIAGONAL member, between two explicit 3-D points.

    Beams are axis-aligned; a 45-degree knee brace is not, so it needs
    its own type rather than a beam with a fudged angle.
    """

    name: str
    x0: float
    y0: float
    z0: float
    x1: float
    y1: float
    z1: float
    section: str = "2x4_half"
    cut0: str | None = None   # axis normal of the plane the P0 end is cut to
    cut1: str | None = None   # same for the P1 end; None = square cut
    notes: str = ""


@dataclass
class Scene:
    units: str
    surfaces: list[Surface]
    walls: list[Wall] = field(default_factory=list)
    fixtures: list[Fixture] = field(default_factory=list)
    legs: list[Leg] = field(default_factory=list)
    foundations: list[Foundation] = field(default_factory=list)
    ledgers: list[Ledger] = field(default_factory=list)
    beams: list[Beam] = field(default_factory=list)
    braces: list[Brace] = field(default_factory=list)
    trusses: dict = field(default_factory=dict)
    thickness: float | None = None
    height_to_floor: float | None = None

    @property
    def to_m(self) -> float:
        return UNIT_TO_M[self.units]

    def footprint(self) -> tuple[float, float, float, float]:
        """Bounding box over all table surfaces, datum units."""
        xs = [v for s in self.surfaces for v in (s.corners()[0], s.corners()[2])]
        ys = [v for s in self.surfaces for v in (s.corners()[1], s.corners()[3])]
        return min(xs), min(ys), max(xs), max(ys)

    def on_table(self, x: float, y: float) -> bool:
        """Is this datum point over any table surface?"""
        return any(s.contains(x, y) for s in self.surfaces)

    def floor_plane(self) -> tuple[float, float, float] | None:
        """(z0, a, b) for  z = z0 + a*x + b*y  - the floor, datum units.

        THE TABLE TOP IS LEVEL AND THE FLOOR IS NOT, so the legs are the
        only measurement of the floor: each leg's height is floor to the
        underside of the top at that point, and differing heights ARE
        the slope. Three legs define the plane; more are least-squared,
        so a fourth measurement improves it rather than conflicting.

        ONLY measured legs count. Inferred ones are placed ON the solved
        plane, so feeding them back would be fitting the plane to its
        own output - harmless arithmetic today, but it would silently
        dilute a real re-measure later.
        """
        known = [l for l in self.legs if l.measured and l.height is not None]
        if len(known) < 3 or self.thickness is None:
            return None
        under = -self.thickness
        rows = [(leg.x, leg.y, 1.0) for leg in known]
        zs = [under - leg.height for leg in known]
        n = len(rows)
        sxx = sum(r[0] * r[0] for r in rows)
        sxy = sum(r[0] * r[1] for r in rows)
        syy = sum(r[1] * r[1] for r in rows)
        sx = sum(r[0] for r in rows)
        sy = sum(r[1] for r in rows)
        sxz = sum(r[0] * z for r, z in zip(rows, zs))
        syz = sum(r[1] * z for r, z in zip(rows, zs))
        sz = sum(zs)
        mm = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
        vv = [sxz, syz, sz]

        def det3(q):
            return (q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1])
                    - q[0][1] * (q[1][0] * q[2][2] - q[1][2] * q[2][0])
                    + q[0][2] * (q[1][0] * q[2][1] - q[1][1] * q[2][0]))

        det = det3(mm)
        if abs(det) < 1e-9:
            return None  # collinear legs cannot define a plane
        out = []
        for col in range(3):
            q = [row[:] for row in mm]
            for r in range(3):
                q[r][col] = vv[r]
            out.append(det3(q) / det)
        a, b, z0 = out
        return z0, a, b

    def floor_z(self, x: float, y: float) -> float | None:
        plane = self.floor_plane()
        if plane is None:
            return None
        z0, a, b = plane
        return z0 + a * x + b * y

    def leg_height(self, leg: Leg) -> float | None:
        """Measured if given, otherwise INFERRED from the solved floor.
        Inference is why the floor had to be right first."""
        if leg.height is not None:
            return leg.height
        fz = self.floor_z(leg.x, leg.y)
        return None if fz is None else -(self.thickness or 0.75) - fz

    def foundation_top_z(self, f: "Foundation", x: float,
                         y: float) -> float | None:
        """Top of a footing AT A POINT, in datum z.

        ONE definition, used by the footing's own geometry AND by any
        wall coming down to meet it. They computed it separately once
        and disagreed, leaving a wall hanging 3.2 in below its footing.
        """
        if self.floor_plane() is None:
            return None
        if not f.level_top:
            return self.floor_z(x, y) + f.height
        # A LEVEL top: `height` is the clearance at ONE measured point,
        # not everywhere. Over a sloping floor the block is taller than
        # `height` wherever the floor drops away - which is why the
        # measuring point is stored with the number.
        if f.height_ref is not None:
            rx, ry = f.height_ref
        else:
            w = next((wl for wl in self.walls if wl.name == f.wall), None)
            if w is None:
                return None
            along_x = w.yaw_deg % 180 == 0
            mid = (w.x + w.width / 2) if along_x else (w.y + w.width / 2)
            rx, ry = (mid, w.y) if along_x else (w.x, mid)
        return self.floor_z(rx, ry) + f.height

    def missing(self) -> list[str]:
        """Everything unmeasured, in the order it blocks work."""
        gaps: list[str] = []
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
            n_meas = sum(1 for l in self.legs
                         if l.measured and l.height is not None)
            lines.append(f"  floor: z = {z0:.3f} + {a:.5f}x + {b:.5f}y  "
                         f"({tilt:.2f} deg tilt, solved from {n_meas} "
                         f"MEASURED legs of {len(self.legs)})")
            for leg in self.legs:
                h_ = self.leg_height(leg)
                tag = "" if leg.measured else "  (INFERRED from the floor)"
                lines.append(f"    truss {leg.name:<20} x {leg.x:g}, "
                             f"y {leg.y:g}, {h_:.2f} tall -> floor "
                             f"{self.floor_z(leg.x, leg.y):.2f}{tag}")
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

    def sketch(self, cols: int = 62,
               arm: tuple[float, float] | None = None) -> str:
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
        if arm is not None:
            c = int((arm[0] - x0) / w * (cols - 1))
            r = int((arm[1] - y0) / d * (rows - 1))
            if 0 <= r < rows and 0 <= c < cols:
                grid[r][c] = "A"
        body = "\n".join("  " + "".join(row).rstrip() for row in grid)
        return (f"  top-down (back of the desk at the top, "
                f"{w:g} x {d:g} {self.units})\n"
                f"  # wall   + table edge   . table top"
                f"{'   A arm' if arm else ''}\n{body}")


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


def load_bench(path: Path = BENCH_JSON) -> Scene:
    """Load + strictly validate a BENCH: the physical place, with no
    reference to what is standing on it. Reusable across projects."""
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise BenchError(
            f"no bench scene at {path}",
            "the cell geometry has not been measured yet; the twin runs "
            "on its infinite ground plane without it") from exc
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
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
        h = _num(entry, "height", where, required=False)
        if h is not None and h <= 0:
            raise BenchError(f"{where}: height must be positive", "")
        section = entry.get("section", "2x4")
        if section not in LUMBER:
            raise BenchError(f"{where}: unknown section {section!r}",
                             f"known: {sorted(LUMBER)}")
        legs.append(Leg(
            name=entry.get("name", "leg"), x=_num(entry, "x", where),
            y=_num(entry, "y", where), height=h, section=section,
            along=entry.get("along", "y"), truss=entry.get("truss"),
            measured=bool(entry.get("measured", h is not None)),
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
            front_top=_num(entry, "front_top", where, required=False),
            level_top=bool(entry.get("level_top", False)),
            height_ref=(tuple(entry["height_ref"])
                        if entry.get("height_ref") else None),
            notes=entry.get("notes", "")))

    beams: list[Beam] = []
    for entry in doc.get("beams") or []:
        where = f"{path} beam {entry.get('name')!r}"
        beams.append(Beam(
            name=entry.get("name", "beam"),
            x0=_num(entry, "x0", where), y0=_num(entry, "y0", where),
            x1=_num(entry, "x1", where), y1=_num(entry, "y1", where),
            section=entry.get("section", "2x4"),
            z=_num(entry, "z", where, required=False),
            notes=entry.get("notes", "")))

    braces: list[Brace] = []
    for entry in doc.get("braces") or []:
        where = f"{path} brace {entry.get('name')!r}"
        braces.append(Brace(
            name=entry.get("name", "brace"),
            x0=_num(entry, "x0", where), y0=_num(entry, "y0", where),
            z0=_num(entry, "z0", where), x1=_num(entry, "x1", where),
            y1=_num(entry, "y1", where), z1=_num(entry, "z1", where),
            section=entry.get("section", "2x4_half"),
            cut0=entry.get("cut0"), cut1=entry.get("cut1"),
            notes=entry.get("notes", "")))

    ledgers: list[Ledger] = []
    for entry in doc.get("ledgers") or []:
        where = f"{path} ledger {entry.get('name')!r}"
        wall_name = entry.get("wall")
        if not any(w.name == wall_name for w in walls):
            raise BenchError(f"{where}: wall {wall_name!r} not found",
                             f"known walls: {[w.name for w in walls]}")
        ledgers.append(Ledger(
            name=entry.get("name", "ledger"), wall=wall_name,
            section=entry.get("section", "2x4"),
            start=_num(entry, "start", where, required=False),
            end=_num(entry, "end", where, required=False),
            notes=entry.get("notes", "")))

    return Scene(
        units=units, surfaces=surfaces, walls=walls, fixtures=fixtures, legs=legs, foundations=foundations,
        ledgers=ledgers, beams=beams, braces=braces,
        trusses=doc.get('trusses') or {},
        thickness=_num(table, "thickness", f"{path} table", required=False),
        height_to_floor=_num(table, "height_to_floor", f"{path} table",
                             required=False))


WALL_T = 1.0  # rendered wall thickness, datum units - cosmetic only

# Vertical field of view for the sim cameras (plan #606).
#
# PLACEHOLDER, and labelled as one because it is the exact shape of
# defect this project keeps hitting: a number derived from a datasheet,
# written into code as a literal, and then trusted as if measured.
#
# Derived for the ELP-USBFHD01M-L36 in cameras.json: 1/2.7" sensor,
# nominal active area 5.37 x 3.02 mm at 16:9, 3.6 mm lens.
#     fovy = 2 * atan(3.02 / 2 / 3.6) = 45.5 deg
# Both the sensor size and the "3.6 mm" are nominal, and real lenses
# miss their nominal focal length by several percent, so this is good
# enough to frame a shot and NOT good enough to measure with.
#
# It is superseded the moment #606's intrinsics calibration runs: fovy
# comes from the calibrated fy and image height as
#     fovy = 2 * atan(height / 2 / fy)
# at which point this constant should be replaced by the per-camera
# `fovy_deg` read from cell.json, not edited here.
DEFAULT_FOVY_DEG = 45.5
# The other two axes, from the same nominal sensor and under the same
# placeholder warning. MuJoCo only wants the vertical, so these are here
# for the coverage arithmetic rather than for rendering — #713.9 sizes
# camera standoff from the HORIZONTAL angle, and until now the repo only
# ever wrote down the vertical, which made every coverage figure a
# rederivation from scratch.
#     fovx = 2 * atan(5.37 / 2 / 3.6) = 73.4 deg
#     fovd = 2 * atan(hypot(5.37, 3.02) / 2 / 3.6) = 81.1 deg
# Worked example, so the next person does not redo it: covering the
# arm's 752 mm-wide reach envelope across the long axis needs
#     h = (752 / 2) / tan(73.4 / 2) = 504 mm
# above the plane, giving 2.55 px/mm at 1920 wide and a 40 mm tag about
# 102 px across.
DEFAULT_FOVX_DEG = 73.4
DEFAULT_FOVDIAG_DEG = 81.1

# MuJoCo geom groups. The interactive viewer toggles these with the
# number keys, so category = group lets the room be built up in detail
# without the ductwork and ceiling burying the bench.
ARM_GROUP = 2          # the arm rides with the fixtures group
GROUP_TABLE, GROUP_WALL, GROUP_FIXTURE = 0, 1, 2
GROUP_OVERHEAD, GROUP_STRUCTURE, GROUP_FLOOR = 3, 4, 5
GROUP_HIDDEN = GROUP_OVERHEAD   # kept in the model, off by default
GROUP_NAMES = {GROUP_TABLE: "table", GROUP_WALL: "walls",
               GROUP_FIXTURE: "fixtures/shelves/ducts/arm",
               GROUP_OVERHEAD: "diagnostics: camera sight lines + arm "
                               "collision hulls (off by default)",
               GROUP_STRUCTURE: "legs/frame/foundation",
               GROUP_FLOOR: "floor"}
# Ducts and ceiling moved in with the fixtures: group 3 now belongs to the
# arm's COLLISION primitives, which the vendored Menagerie model ships
# there. Rendering those on top of the arm's visual meshes makes it look
# doubled, so group 3 is off by default - press 3 in the viewer to see
# what the collision gate actually reasons about.
FIXTURE_GROUP: dict[str, int] = {}

# Nominal lumber is not actual lumber: a "2x4" is 1.5 x 3.5 inches.
LUMBER = {"2x4": (1.5, 3.5), "2x6": (1.5, 5.5), "2x4_half": (1.5, 1.75),
          "4x4": (3.5, 3.5)}


def _add_prism(spec, mujoco, name: str, corners, rgba, group) -> None:
    """A solid from two 4-corner profiles: near-end quad then far-end.

    Built as a mesh because these shapes are trapezoids - a footing
    battered on one face, a wall whose bottom follows the sloping floor
    while its top stays level - and a box cannot taper. They are convex,
    so MuJoCo's collision hull is exact rather than an approximation.

    Face winding is COMPUTED, not hand-written: each triangle is flipped
    if its normal points toward the solid's centroid. Hand-ordering the
    twelve triangles depends on which end is "near" and on the wall's
    handedness, and getting it wrong renders faces inside-out - which is
    exactly what happened. Deriving it removes the whole class of error.
    """
    pts = [list(map(float, c)) for c in corners]
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    cz = sum(p[2] for p in pts) / len(pts)
    quads = [(0, 1, 2, 3), (4, 5, 6, 7),
             (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    faces: list[int] = []
    for a, b, c, d in quads:
        for tri in ((a, b, c), (a, c, d)):
            i, j, k = tri
            ux = pts[j][0] - pts[i][0]
            uy = pts[j][1] - pts[i][1]
            uz = pts[j][2] - pts[i][2]
            vx = pts[k][0] - pts[i][0]
            vy = pts[k][1] - pts[i][1]
            vz = pts[k][2] - pts[i][2]
            nx, ny, nz = (uy * vz - uz * vy,
                          uz * vx - ux * vz,
                          ux * vy - uy * vx)
            # vector from the centroid out to this face
            ox = pts[i][0] - cx
            oy = pts[i][1] - cy
            oz = pts[i][2] - cz
            faces.extend(tri if (nx * ox + ny * oy + nz * oz) > 0
                         else (i, k, j))
    verts: list[float] = []
    for pt in pts:
        verts.extend(pt)
    spec.add_mesh(name=f"mesh_{name}", uservert=verts, userface=faces)
    spec.worldbody.add_geom(
        name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"mesh_{name}",
        rgba=rgba, group=group)


def _add_reach_zone(spec, scene: Scene, tiles) -> None:
    """Lay the arm's functional reach zone on the tabletop.

    `tiles` is plain data — (centre x, centre y, yaw deg, half-radial mm,
    half-tangential mm) per angular slice, already resolved into bench
    coordinates. Deliberately not computed here: the zone needs the twin
    and the calibration, and `sim.reach` imports this module, so pushing
    the arithmetic up the dependency keeps that one-way.

    Drawn in GROUP_TABLE because it IS a statement about the tabletop —
    the part of it the arm can actually work on — so it toggles with the
    surface it describes. Non-colliding: this is an annotation, and a
    zone that could be hit would change every gate verdict in the scene.
    """
    import mujoco

    m = scene.to_m
    # Two rings, two colours. GREEN is the guaranteed vertical-approach
    # zone (gripper plumb); AMBER is the extra the arm reaches when the
    # wrist may arrive tilted. Drawing only the green one understated
    # the arm by 210 mm on the outer edge and read as a bug twice.
    colour = {"plumb": [0.20, 0.85, 0.35, 0.34],
              "tilt": [0.95, 0.70, 0.15, 0.22]}
    for i, t in enumerate(tiles):
        cx, cy, yaw, half_r, half_t = t[:5]
        kind = t[5] if len(t) > 5 else "plumb"
        half = math.radians(yaw) / 2.0
        spec.worldbody.add_geom(
            name=f"reach_{i:04d}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[half_r / 1000.0, half_t / 1000.0, 0.0005],
            pos=[cx * m, cy * m, 0.0015 if kind == "plumb" else 0.0010],
            quat=[math.cos(half), 0.0, 0.0, math.sin(half)],
            rgba=colour.get(kind, colour["plumb"]),
            contype=0, conaffinity=0, group=GROUP_TABLE)


def build_spec(scene: Scene, shadows: bool = False,
               cameras: 'tuple[Camera, ...]' = (),
               arm: 'tuple[float, float, float] | None' = None,
               reach_zone: tuple = ()):
    """MuJoCo spec of the measured bench, in the DATUM frame.

    Metres, table top at z=0, +z up. Geometry only - no arm: this exists
    to look at the measurements, and (once the arm is placed) to become
    the twin's real workspace instead of its infinite ground plane.

    mujoco is imported here, not at module scope, so loading and
    validating measurements never needs the physics stack.
    """
    import mujoco

    m = scene.to_m
    # Solved up front: walls, footings and rails all need it, and the
    # wall loop runs before the floor is drawn.
    plane = scene.floor_plane()
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
        found = next((f for f in scene.foundations if f.wall == w.name), None)
        r0 = (w.x if along_x else w.y)
        r1 = r0 + w.width

        def wxy(run: float, prot: float) -> tuple[float, float]:
            if along_x:
                return run, w.y + oy * prot
            return w.x + ox * prot, run

        if w.extends_below and found is not None and plane is not None:
            # Top LEVEL (the table is level), bottom parallel to the
            # sloping floor where it lands on its footing - so the wall
            # is a trapezoid in elevation, not a box.
            pts = []
            for run in (r0, r1):
                for prot in (0.0, t):
                    pass
                a_ = wxy(run, 0.0)
                b_ = wxy(run, t)
                lo_a = scene.foundation_top_z(found, *a_)
                lo_b = scene.foundation_top_z(found, *b_)
                pts += [[a_[0] * m, a_[1] * m, lo_a * m],
                        [b_[0] * m, b_[1] * m, lo_b * m],
                        [b_[0] * m, b_[1] * m, w.height * m],
                        [a_[0] * m, a_[1] * m, w.height * m]]
            _add_prism(spec, mujoco, f"wall_{w.name}", pts,
                       [0.85, 0.85, 0.88, 1.0], GROUP_WALL)
        else:
            if along_x:
                half = [w.width * m / 2, t * m / 2, w.height * m / 2]
                pos = [(w.x + w.width / 2) * m, (w.y + oy * t / 2) * m,
                       w.height / 2 * m]
            else:
                half = [t * m / 2, w.width * m / 2, w.height * m / 2]
                pos = [(w.x + ox * t / 2) * m, (w.y + w.width / 2) * m,
                       w.height / 2 * m]
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

    LEG_GEOMS: list[tuple] = []  # emitted after trusses set their spans

    # A TRUSS is rails-first: the rails run the full length and the
    # posts are attached to them, not the other way round (Kyle). Rails
    # lie FLAT - wide face horizontal - so each is 1.5 thick vertically
    # and 3.5 across. The top rail runs on to the wall, where it meets
    # the ledger in a half-lap; the lap is joinery, so the boxes simply
    # overlap.
    trusses: dict[str, list[Leg]] = {}
    for leg in scene.legs:
        if leg.truss:
            trusses.setdefault(leg.truss, []).append(leg)
    rail_t, rail_w = LUMBER["2x4"]          # 1.5 thick, 3.5 wide
    truss_posts: dict[str, tuple[float, float]] = {}
    for tname, members in trusses.items():
        if len(members) < 2 or plane is None:
            continue
        cfg = scene.trusses.get(tname, {}) if isinstance(scene.trusses, dict) else {}
        a, b = members[0], members[1]
        top_under = -(scene.thickness or 0.75)
        run_x = abs(b.x - a.x) >= abs(b.y - a.y)
        half_post = LUMBER[a.section][0] / 2
        if run_x:
            lo = min(a.x, b.x) - half_post
            hi = max(a.x, b.x) + half_post
            cross = (a.y + b.y) / 2
        else:
            lo = min(a.y, b.y) - half_post
            hi = max(a.y, b.y) + half_post
            cross = (a.x + b.x) / 2
        floor_lo = min(scene.floor_z(a.x, a.y), scene.floor_z(b.x, b.y))

        # A bottom rail can be RAISED to double as a shelf bearer, in
        # which case it is not on the floor and the posts must carry on
        # down past it. Referenced to the fixture it carries, so moving
        # the shelf moves the rail rather than leaving it behind.
        carries = cfg.get("bottom_rail_carries")
        bottom_top = None
        if carries:
            fx = next((f for f in scene.fixtures if f.name == carries), None)
            if fx is not None and fx.z is not None:
                bottom_top = fx.z          # shelf underside rests on it
        # Posts sit BETWEEN the rails when the bottom rail is on the
        # floor; when it is raised they run all the way to the floor.
        truss_posts[tname] = (None if bottom_top is not None
                              else floor_lo + rail_t, top_under - rail_t)

        wall_name = cfg.get("top_rail_to_wall")
        for label, zc, ends in (
                ("top", top_under - rail_t / 2, "wall"),
                ("bottom", (bottom_top - rail_t / 2) if bottom_top is not None
                 else floor_lo + rail_t / 2, None)):
            r_lo, r_hi = lo, hi
            if ends == "wall" and wall_name:
                w = next((wl for wl in scene.walls if wl.name == wall_name),
                         None)
                if w is not None:
                    face = w.x if (w.yaw_deg % 180) else w.y
                    r_lo, r_hi = min(r_lo, face), max(r_hi, face)
            length = r_hi - r_lo
            mid = (r_lo + r_hi) / 2
            if run_x:
                half = [length * m / 2, rail_w * m / 2, rail_t * m / 2]
                pos = [mid * m, cross * m, zc * m]
            else:
                half = [rail_w * m / 2, length * m / 2, rail_t * m / 2]
                pos = [cross * m, mid * m, zc * m]
            spec.worldbody.add_geom(
                name=f"rail_{tname}_{label}", type=mujoco.mjtGeom.mjGEOM_BOX,
                size=half, pos=pos, rgba=[0.78, 0.65, 0.44, 1.0],
                group=GROUP_STRUCTURE)

    # Beams: supports along table edges, laid FLAT - the wide (3.5) face
    # is what fastens to the underside of the top, so each is 3.5 across
    # and only 1.5 deep vertically (Kyle; they were on edge, 90 out).
    for bm in scene.beams:
        thin, wide = LUMBER[bm.section]      # 1.5 vertical, 3.5 across
        top_under = bm.z if bm.z is not None else -(scene.thickness or 0.75)
        run_x = abs(bm.x1 - bm.x0) >= abs(bm.y1 - bm.y0)
        length = abs(bm.x1 - bm.x0) if run_x else abs(bm.y1 - bm.y0)
        if run_x:
            half = [length * m / 2, wide * m / 2, thin * m / 2]
        else:
            half = [wide * m / 2, length * m / 2, thin * m / 2]
        spec.worldbody.add_geom(
            name=f"beam_{bm.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=half,
            pos=[(bm.x0 + bm.x1) / 2 * m, (bm.y0 + bm.y1) / 2 * m,
                 (top_under - thin / 2) * m],
            rgba=[0.76, 0.63, 0.42, 1.0], group=GROUP_STRUCTURE)

    # Braces: diagonal members. A knee brace is MITRED - its ends are
    # cut so they sit flush against the wall and against the frame,
    # rather than square-cut and overhanging both. That makes the shape
    # a parallelogram, not a box, so it goes through the prism builder.
    for br in scene.braces:
        thin, wide = LUMBER[br.section]
        dx, dz = br.x1 - br.x0, br.z1 - br.z0
        length = math.hypot(dx, dz)
        if length <= 0 or abs(br.y1 - br.y0) > 1e-9:
            continue  # only in-plane braces for now
        ux, uz = dx / length, dz / length
        nx, nz = -uz, ux                      # unit normal, in-plane
        h = thin / 2

        def end_point(px, pz, sign, cut, other_x, other_z):
            """Corner of one long face at one end, mitred to `cut`."""
            ax, az = px + sign * h * nx, pz + sign * h * nz
            if cut == "x":                     # flush to a vertical plane
                t = (other_x - ax) / ux if ux else 0.0
            elif cut == "z":                   # flush to a horizontal one
                t = (other_z - az) / uz if uz else 0.0
            else:
                t = 0.0                        # square cut
            return ax + t * ux, az + t * uz

        prof = [
            end_point(br.x0, br.z0, +1, br.cut0, br.x0, br.z0),
            end_point(br.x1, br.z1, +1, br.cut1, br.x1, br.z1),
            end_point(br.x1, br.z1, -1, br.cut1, br.x1, br.z1),
            end_point(br.x0, br.z0, -1, br.cut0, br.x0, br.z0),
        ]
        pts = []
        for yy in (br.y0 - wide / 2, br.y0 + wide / 2):
            for cx, cz in prof:
                pts.append([cx * m, yy * m, cz * m])
        _add_prism(spec, mujoco, f"brace_{br.name}", pts,
                   [0.76, 0.63, 0.42, 1.0], GROUP_STRUCTURE)

    # Ledgers: wide face flat on the wall, top flush with the table
    # underside, running the wall's length.
    for led in scene.ledgers:
        w = next((wl for wl in scene.walls if wl.name == led.wall), None)
        if w is None:
            continue
        lt, lw = LUMBER[led.section]        # 1.5 projection, 3.5 tall
        ox, oy = OUTWARD[w.outward]
        top_under = -(scene.thickness or 0.75)
        zc = top_under - lw / 2
        along_x = w.yaw_deg % 180 == 0
        run0 = w.x if along_x else w.y
        a_ = led.start if led.start is not None else run0
        b_ = led.end if led.end is not None else run0 + w.width
        length, mid = b_ - a_, (a_ + b_) / 2
        if along_x:
            half = [length * m / 2, lt * m / 2, lw * m / 2]
            pos = [mid * m, (w.y - oy * lt / 2) * m, zc * m]
        else:
            half = [lt * m / 2, length * m / 2, lw * m / 2]
            pos = [(w.x - ox * lt / 2) * m, mid * m, zc * m]
        spec.worldbody.add_geom(
            name=f"ledger_{led.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=half, pos=pos, rgba=[0.74, 0.61, 0.41, 1.0],
            group=GROUP_STRUCTURE)

    for leg in scene.legs:
        w_, d_ = LUMBER[leg.section]
        hx, hy = (d_, w_) if leg.along == "x" else (w_, d_)
        top_under = -(scene.thickness or 0.75)
        h_ = scene.leg_height(leg)
        lo_z, hi_z = truss_posts.get(
            leg.truss or "", (top_under - (h_ or 0.0), top_under))
        if lo_z is None:                      # raised bottom rail
            lo_z = scene.floor_z(leg.x, leg.y)
        spec.worldbody.add_geom(
            name=f"leg_{leg.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[hx * m / 2, hy * m / 2, (hi_z - lo_z) * m / 2],
            pos=[leg.x * m, leg.y * m, (lo_z + hi_z) / 2 * m],
            rgba=[0.78, 0.65, 0.44, 1.0], group=GROUP_STRUCTURE)

    for f in scene.foundations:
        w = next((wl for wl in scene.walls if wl.name == f.wall), None)
        if w is None or plane is None:
            continue
        along_x = w.yaw_deg % 180 == 0
        ox, oy = OUTWARD[w.outward]
        r0 = (w.x if along_x else w.y)
        r1 = r0 + w.width
        near = -f.back - w.thickness
        far_bot = f.front
        far_top = f.front_top if f.front_top is not None else f.front

        def xy(run: float, prot: float) -> tuple[float, float]:
            if along_x:
                return run, w.y + oy * -prot
            return w.x + ox * -prot, run

        def v_floor(run: float, prot: float, drop: float) -> list[float]:
            """A point ON the floor (bottom of the block)."""
            px, py = xy(run, prot)
            return [px * m, py * m, (scene.floor_z(px, py) + drop) * m]

        def v_top(run: float, prot: float) -> list[float]:
            """A point on the block's TOP - via foundation_top_z, so a
            LEVEL top really is level. Computing floor + height here
            instead is what kept the return footing's top parallel to
            the floor after the level-top rule was added: the rule lived
            in the API and never reached the geometry."""
            px, py = xy(run, prot)
            return [px * m, py * m, scene.foundation_top_z(f, px, py) * m]

        # ONE SOLID TRAPEZOID: wide at the floor, narrow where it meets
        # the wall, and sitting a constant height off the floor so the
        # whole thing tilts with the slab.
        pts = []
        for run in (r0, r1):
            pts += [v_floor(run, near, -2.0), v_floor(run, far_bot, 0.0),
                    v_top(run, far_top), v_top(run, near)]
        _add_prism(spec, mujoco, f"foundation_{f.name}", pts,
                   [0.80, 0.79, 0.76, 1.0], GROUP_STRUCTURE)

    # Cameras: a small body on its wall bracket plus a SIGHT LINE along
    # the view direction. The sight line is the point - a camera box
    # tells you nothing about whether it is aimed at the work.
    for cam in cameras:
        if cam.free_standing:
            # An explicit spot on the bench looking along its own yaw —
            # a table-edge stand, a tripod, anything not on a wall.
            px, py = cam.x, cam.y
            a = math.radians(cam.yaw_deg or 0.0)
            ix, iy = math.cos(a), math.sin(a)
        else:
            w = next((wl for wl in scene.walls if wl.name == cam.wall), None)
            if w is None:
                continue
            along_x = w.yaw_deg % 180 == 0
            ox, oy = OUTWARD[w.outward]
            stand = 2.0 if cam.standoff is None else cam.standoff
            # into the room is opposite the wall's outward direction
            ix, iy = -ox, -oy
            if along_x:
                px, py = cam.along, w.y + iy * stand
            else:
                px, py = w.x + ix * stand, cam.along
        # Planned poses are drawn in amber, measured ones in near-black.
        # A glance has to distinguish intent from fact, because both are
        # solid geometry in the same picture.
        body_rgba = ([0.95, 0.70, 0.15, 0.75] if cam.planned
                     else [0.15, 0.15, 0.18, 1.0])
        sight_rgba = ([0.98, 0.72, 0.10, 0.40] if cam.planned
                      else [0.95, 0.35, 0.25, 0.45])
        # A PLANNED camera's sight line is visible by DEFAULT, unlike a
        # measured one's. For a camera that exists, the aim is a
        # diagnostic you go looking for; for one that does not, the aim
        # IS the proposal — hiding it behind a keypress would mean
        # opening the scene to review a plan and seeing nothing of it.
        sight_group = GROUP_FIXTURE if cam.planned else GROUP_HIDDEN
        spec.worldbody.add_geom(
            name=f"camera_{cam.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[1.5 * m / 2, 1.5 * m / 2, 1.2 * m / 2],
            pos=[px * m, py * m, cam.z * m],
            rgba=body_rgba, group=GROUP_FIXTURE)
        t = math.radians(cam.tilt_deg)
        dx, dy, dz = ix * math.cos(t), iy * math.cos(t), -math.sin(t)
        # Stop the sight line where it meets the tabletop, so it reads as
        # "this is the spot it is aimed at" rather than a ray disappearing
        # through the bench. Falls back to the old fixed length for a
        # camera aimed level or upward.
        reach = (cam.z / -dz) if dz < -1e-6 else 40.0
        reach = max(2.0, min(reach, 60.0))
        spec.worldbody.add_geom(
            name=f"camsight_{cam.name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.25 * m / 2, 0.25 * m / 2, reach * m / 2],
            pos=[(px + dx * reach / 2) * m, (py + dy * reach / 2) * m,
                 (cam.z + dz * reach / 2) * m],
            zaxis=[dx, dy, dz],
            rgba=sight_rgba, group=sight_group)
        # A puck where the sight line lands, so the aim point is legible
        # without measuring off the screen.
        spec.worldbody.add_geom(
            name=f"camaim_{cam.name}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[1.2 * m, 0.15 * m, 0],
            pos=[(px + dx * reach) * m, (py + dy * reach) * m,
                 (cam.z + dz * reach) * m],
            rgba=sight_rgba, group=sight_group)

        # A MuJoCo camera at the SAME pose, so the cell can be rendered
        # from where the real camera actually looks (plan #606). It is
        # added here, beside the sight line, rather than anywhere more
        # convenient: both are derived from the same px/py/cam.z and
        # dx/dy/dz, so the diagnostic and the render cannot drift apart.
        # Placed anywhere else, someone would eventually fix one and not
        # the other, and the picture would stop being evidence.
        #
        # MuJoCo cameras look along their own -z with +y up, so the axes
        # are constructed rather than guessed — the same lesson the
        # corner camera above records the hard way.
        fwd = (dx, dy, dz)
        rgt = (fwd[1], -fwd[0], 0.0)          # forward x world-up
        rn = math.hypot(rgt[0], rgt[1]) or 1.0
        rgt = (rgt[0] / rn, rgt[1] / rn, 0.0)
        up = (rgt[1] * fwd[2] - rgt[2] * fwd[1],
              rgt[2] * fwd[0] - rgt[0] * fwd[2],
              rgt[0] * fwd[1] - rgt[1] * fwd[0])
        spec.worldbody.add_camera(
            name=(f"camplan_{cam.name}" if cam.planned else f"cam_{cam.name}"),
            pos=[px * m, py * m, cam.z * m],
            xyaxes=[*rgt, *up],
            fovy=cam.fovy_deg if cam.fovy_deg is not None
            else DEFAULT_FOVY_DEG)

    # The ARM itself, attached from the vendored Menagerie model the twin
    # already uses - one source for the arm's geometry, so the picture and
    # the collision gate can never disagree about its shape.
    if arm is not None:
        ax_, ay_, ayaw = arm
        from sim.twin import MODEL_XML
        try:
            child = mujoco.MjSpec.from_file(str(MODEL_XML))
        except (ValueError, OSError):
            child = None
        if child is not None:
            # yaw says which BENCH direction the arm must reach along; the
            # MODEL has its own idea of forward, and the two packages
            # disagree by 90 deg (SO-100 reached along its own -Y, SO-101
            # along +X). So the rotation is yaw MINUS the model's own
            # reach angle - asked of the model, not written down here.
            # This was a literal `+ 90`, which is exactly -(-90): correct
            # for the old model by coincidence of being transcribed from
            # it, and would have quietly aimed the arm at the wrong wall
            # after the swap (plan #670).
            from sim.twin import Twin
            twin = Twin()
            heading = ayaw - twin.reach_yaw_deg()
            # The cell measures the arm by its M1 AXIS — that is the one
            # feature on a real arm you can put a tape to, and it is what
            # Kyle measures ("the m1 center is 2.25in inset from the
            # edge"). The MODEL's root origin is NOT that point: on the
            # SO-101 the shoulder_pan axis sits 38.8 mm from it. Attaching
            # the model at the measured coordinates therefore put the real
            # pivot 38.8 mm away from where it was measured — 1.5 in of
            # pure translation error, spotted by Kyle in the viewer on
            # 2026-07-28 as "a little bit in".
            #
            # So back-solve the attach origin from the axis instead:
            # rotate the model's own base->m1 offset into bench space and
            # subtract it. Base-relative checks cannot catch this (they
            # move with the arm), which is why it survived until someone
            # looked at the picture against the real bench.
            ox, oy = twin.m1_offset_m()
            ca, sa = (math.cos(math.radians(heading)),
                      math.sin(math.radians(heading)))
            frame = spec.worldbody.add_frame(
                pos=[ax_ * m - (ox * ca - oy * sa),
                     ay_ * m - (ox * sa + oy * ca), 0.0],
                euler=[0.0, 0.0, heading])
            spec.attach(child, prefix="arm_", frame=frame)

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
    key.castshadow = shadows
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
    # Last, so the annotation never participates in the navigation
    # statistics computed above — and note the local `reach` here is the
    # scene's EXTENT, which is why the zone parameter is `reach_zone`.
    if reach_zone:
        _add_reach_zone(spec, scene, reach_zone)
    return spec


def build_model(scene: Scene, shadows: bool = False,
                cameras: 'tuple[Camera, ...]' = (),
                arm: 'tuple[float, float, float] | None' = None,
                reach_zone: tuple = ()):
    """Compile the cell. Pair with `rest_data` to get an MjData posed at
    the arm's rest pose.

    NEVER write `model.qpos0` to move the arm's default pose. qpos0 is
    the joint REFERENCE configuration - MuJoCo measures every hinge angle
    from it - so assigning the rest keyframe to it silently re-zeros all
    six joints and renders every pose at (qpos - rest). It looks like a
    default-pose knob and is not one. That bug shipped 2026-07-26 and
    made the arm play the exercise routine through itself; `_selftest_
    arm_pose_matches_twin` now pins it."""
    return build_spec(scene, shadows, cameras, arm, reach_zone).compile()


def rest_data(model):
    """MjData for `model`, posed at the arm's CALIBRATED rest.

    Rest comes from calibration.json via the twin, not from a keyframe:
    the SO-101 package ships none (plan #670), and a keyframe would be
    someone else's idea of rest anyway, while calibration.json holds the
    pose this arm was actually captured in.

    Never write `model.qpos0` to do this. qpos0 is the joint REFERENCE
    configuration - MuJoCo measures every hinge angle from it - so
    assigning a pose to it silently re-zeroes all six joints and renders
    everything at (qpos - rest). It looks like a default-pose knob and is
    not one; that bug shipped 2026-07-26 and made the arm play the
    exercise routine through itself. Set `data`, never `model`.

    Falls back to a plain reset for a bench with no arm, or if the twin
    is unavailable (no mujoco model / no calibration)."""
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    try:
        from sim.twin import JOINT_MAPS, Twin
        twin = Twin()
    except Exception:                       # bench-only scene, or no cal
        mujoco.mj_forward(model, data)
        return data
    for i, jm in JOINT_MAPS.items():
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                              f"arm_{jm.model_joint}")
        if j >= 0:
            data.qpos[model.jnt_qposadr[j]] = twin.qpos_of(
                i, twin.cals[i].rest)[0]
    mujoco.mj_forward(model, data)
    return data


def view(cell: 'Cell', save_view: bool = True,
         path: Path = CELL_JSON, reach_zone: tuple = ()) -> int:
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

    model = build_model(cell.bench, cell.shadows, cell.cameras,
                        cell.arm_pose, reach_zone)
    data = rest_data(model)
    print("opening the MuJoCo viewer - close the window to exit")
    if reach_zone:
        print(f"  the GREEN patch on the table is the functional reach "
              f"zone ({len(reach_zone)} slices)")
        print("  it is where the JAWS can reach down and grasp, already "
              "clipped to the tabletop")
    print("  left-drag = orbit, right-drag = pan, scroll = zoom")
    print("  [ / ] cycles cameras (there is a fixed 'bench' one)")
    print("  number keys 0-5 toggle geom groups: "
          + ", ".join(f"{g}={n}" for g, n in sorted(GROUP_NAMES.items())))
    if save_view:
        print(f"  the view you leave it at is saved to {path}")
    if cell.missing():
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
            v.opt.geomgroup[:] = 1   # number keys toggle these
            v.opt.geomgroup[GROUP_OVERHEAD] = 0  # arm collision hulls
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
                cell = load_cell(path)
                break
            except BenchError as exc:
                if attempt == 19:
                    print(f"reload failed: {exc}", file=sys.stderr)
                    return 2
        # The zone rides the reload too. It is derived from the arm's
        # placement, so a cell.json edit that moves the arm and leaves a
        # stale zone drawn would be worse than not drawing one at all.
        if reach_zone:
            from .reach import zone_tiles           # local: reach imports us
            reach_zone = zone_tiles(cell)
        model = build_model(cell.bench, cell.shadows, cell.cameras,
                            cell.arm_pose, reach_zone)
        data = rest_data(model)
        print("scene changed - reloaded")
        for gap in cell.missing():
            print(f"  still missing: {gap}")


def _mtime(path: Path) -> float:
    try:
        return path.stat().st_mtime
    except OSError:
        return 0.0


def load_view(path: Path = CELL_JSON) -> dict | None:
    """The saved free-camera pose, if the scene file carries one."""
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
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
    doc = json.loads(path.read_text(encoding="utf-8"))
    doc["view"] = view_pose
    doc["_view_note"] = ("Saved free-camera pose from the last --view "
                         "session (MuJoCo units, metres). Restored on the "
                         "next --view and used by --render. Delete this "
                         "key to go back to the derived angles.")
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def render(scene: Scene, out_dir: Path, width: int = 1280,
           height: int = 860,
           cameras: 'tuple[Camera, ...]' = (),
           arm: 'tuple[float, float, float] | None' = None
           ) -> list[Path]:
    """Render the bench from a few angles. Returns the files written."""
    import mujoco
    import numpy as np

    model = build_model(scene, cameras=cameras, arm=arm)
    data = rest_data(model)

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
    opt.geomgroup[GROUP_OVERHEAD] = 0

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


@dataclass(frozen=True)
class Camera:
    """A camera on a wall bracket, looking down into the cell.

    `z` is the LENS CENTRE above the table top - that is what Kyle
    measures and what decides what the camera can see. `tilt_deg` is
    the bracket's downward angle from horizontal (the printed brackets
    are 45 and 60).
    """

    name: str
    z: float                  # lens centre above the table top
    wall: str | None = None
    along: float | None = None      # position along the wall's run
    # FREE-STANDING alternative to wall/along: an explicit spot on the
    # bench plus the direction it looks. That is what a table-edge stand
    # needs (#717.5) — it is not on a wall, so `wall` cannot place it.
    x: float | None = None
    y: float | None = None
    yaw_deg: float | None = None    # 0 = looks along +x, CCW from above
    tilt_deg: float = 60.0
    standoff: float | None = None   # lens out from the wall FACE
    fovy_deg: float | None = None   # None -> DEFAULT_FOVY_DEG
    # PLANNED means "where we intend to put it", NOT where it is. The
    # cell nulls unmeasured poses precisely so a render cannot pass for
    # evidence, so intent has to be marked rather than merely written
    # down: a planned camera draws its body and sight line (that is the
    # point — you can look at the aim before you drill), but its MuJoCo
    # camera is named `camplan_` rather than `cam_`, so nothing that
    # renders "what the camera sees" picks it up by accident.
    planned: bool = False
    notes: str = ""

    @property
    def free_standing(self) -> bool:
        return self.x is not None and self.y is not None


@dataclass
class Cell:
    """A PROJECT's use of a bench: what stands on it and where.

    Deliberately separate from the Bench, in data and in code. The bench
    is a place - measured once, and the base for many configurations.
    Where the arm and cameras sit is this project's business and changes
    freely, so it must not live inside the file describing the room.
    """

    bench: Scene
    bench_path: Path
    arm_x: float | None = None
    arm_y: float | None = None
    arm_yaw_deg: float | None = None
    # The bench direction the arm's BASE is squared to — the angle a
    # person actually sets with a clamp, as opposed to arm_yaw_deg, which
    # is the direction the arm REACHES and differs by the model's own
    # reach offset. Optional, and purely a checked assertion: nothing
    # builds from it. See _selftest_arm_base_square.
    arm_base_square_deg: float | None = None
    cameras: list = field(default_factory=list)
    shadows: bool = False

    @property
    def arm_pose(self) -> tuple[float, float, float] | None:
        """(x, y, yaw) if fully placed, else None - so callers cannot
        half-place it."""
        if not self.arm_placed:
            return None
        return (self.arm_x, self.arm_y, self.arm_yaw_deg)

    @property
    def arm_placed(self) -> bool:
        """All three are required together: without yaw the bench cannot
        be rotated into the arm's frame at all, and a half-placed arm is
        exactly the failure this refuses to ship."""
        return None not in (self.arm_x, self.arm_y, self.arm_yaw_deg)

    def to_arm_frame(self, x: float, y: float) -> tuple[float, float]:
        """Bench datum point -> the arm's frame, in METRES.

        The arm's frame is the twin's world: origin at the base, arm
        reaching toward -Y at pan zero. arm_yaw_deg says which bench
        direction that reach points along (0 = +x), positive
        counter-clockwise seen from above.
        """
        if not self.arm_placed:
            raise BenchError(
                "the arm's position on the bench is not measured yet",
                f"fill in arm.x / arm.y / arm.yaw_deg in {CELL_JSON}")
        mm = self.bench.to_m
        dx = (x - self.arm_x) * mm
        dy = (y - self.arm_y) * mm
        a = math.radians(self.arm_yaw_deg)
        # DERIVED, not fiddled: the arm reaches along its own -Y, and that
        # must land on the bench direction u = (cos a, sin a). So the arm's
        # +Y axis is -u and its +X is u rotated 90 deg:
        #     arm +X = (-sin a,  cos a)
        #     arm +Y = (-cos a, -sin a)
        # and a bench displacement projects onto them. The previous form
        # was 90 deg out from this docstring - yaw 0 reached along +y, not
        # +x - which would have rotated the entire cell about the arm.
        arm_x = -dx * math.sin(a) + dy * math.cos(a)
        arm_y = -dx * math.cos(a) - dy * math.sin(a)
        return arm_x, arm_y

    def missing(self) -> list[str]:
        gaps = list(self.bench.missing())
        if not self.arm_placed:
            which = [n for n, v in (("x", self.arm_x), ("y", self.arm_y),
                                    ("yaw_deg", self.arm_yaw_deg))
                     if v is None]
            gaps.append(
                f"arm placement on the bench ({', '.join(which)}) - until "
                f"this is known the twin CANNOT use the real tabletop and "
                f"keeps its infinite ground plane")
        if not self.cameras:
            gaps.append("cameras - none placed")
        return gaps

    def describe(self) -> str:
        lines = [self.bench.describe(),
                 f"  (bench loaded from {self.bench_path})"]
        if self.arm_placed:
            lines.append(f"  ARM at x {self.arm_x:g}, y {self.arm_y:g}, "
                         f"facing {self.arm_yaw_deg:g} deg")
            if not self.bench.on_table(self.arm_x, self.arm_y):
                lines.append("    WARNING: that point is not over any "
                             "measured bench surface")
        for cam in self.cameras:
            tag = "PLANNED camera" if cam.planned else "CAMERA"
            where = (f"free-standing at x {cam.x:g}, y {cam.y:g}, facing "
                     f"{cam.yaw_deg or 0:g} deg" if cam.free_standing
                     else f"on wall '{cam.wall}' at {cam.along:g} along")
            lines.append(f"  {tag} {cam.name}: {where}, lens {cam.z:g} "
                         f"above the top, {cam.tilt_deg:g} deg down")
        if any(c.planned for c in self.cameras):
            lines.append("    (amber = PLANNED, not measured — press 3 for "
                         "the sight lines)")
        return "\n".join(lines)

    def sketch(self, cols: int = 62) -> str:
        arm = ((self.arm_x, self.arm_y) if self.arm_placed else None)
        return self.bench.sketch(cols, arm=arm)


def _selftest_arm_frame() -> None:
    """Pin the yaw convention. It is 90-degree-symmetric enough to look
    right while being wrong, and getting it wrong rotates the whole cell
    about the arm - the same class of error as the mirrored bench."""
    bench = Scene(units="in", surfaces=[Surface("t", 0, 0, 10, 10)])
    for yaw, reach in ((0.0, (1.0, 0.0)), (90.0, (0.0, 1.0)),
                       (-90.0, (0.0, -1.0)), (180.0, (-1.0, 0.0))):
        cell = Cell(bench=bench, bench_path=Path("-"), arm_x=0.0, arm_y=0.0,
                    arm_yaw_deg=yaw)
        ax, ay = cell.to_arm_frame(*reach)
        # a point straight out the reach lands on the arm's -Y axis
        assert abs(ax) < 1e-9, (yaw, ax)
        assert ay < 0, (yaw, ay)
        # and 90 deg to its left lands on +X
        left = (-reach[1], reach[0])
        lx, ly = cell.to_arm_frame(*left)
        assert lx > 0 and abs(ly) < 1e-9, (yaw, lx, ly)
    print("arm-frame convention OK")


def _selftest_arm_base_square(cell: 'Cell') -> None:
    """The arm's BASE must sit at the angle the cell says it was clamped
    to, and its M1 AXIS must land on the measured coordinates.

    Both of these were wrong on 2026-07-28 and neither was caught by any
    existing check, because every other test is base-relative and so
    moves with the arm. Kyle found them by looking at the viewer against
    the real bench: "the back edged of the base is parallel with the
    table edge, you have it off angle a few degrees, and a little bit
    in". The angle was 9.3 deg (reach direction confused with base
    heading) and the "little bit" was 38.8 mm (model root origin
    confused with the m1 axis).

    `yaw_deg` is DERIVED — base angle plus the model's reach offset — so
    swapping the arm model silently invalidates it. This check is what
    turns that into a loud failure instead of a crooked arm nobody
    notices.
    """
    import math

    import mujoco

    from sim.twin import BASE_BODY, JOINT_MAPS

    if cell.arm_pose is None or cell.arm_base_square_deg is None:
        print("arm base squareness: skipped (not placed, or no "
              "base_square_to_deg recorded)")
        return
    model = build_model(cell.bench, cell.shadows, cell.cameras, cell.arm_pose)
    data = rest_data(model)
    mujoco.mj_forward(model, data)

    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            f"arm_{BASE_BODY}")
    R = data.xmat[bid].reshape(3, 3)
    got = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    want = cell.arm_base_square_deg
    err = (got - want + 180.0) % 360.0 - 180.0
    assert abs(err) < 0.05, (
        f"the arm's base sits {got:.3f} deg but cell.json says it is "
        f"clamped square to {want:.3f} deg (off by {err:+.3f}). "
        f"yaw_deg is derived as base + Twin.reach_yaw_deg; if the arm "
        f"model changed, re-derive it.")

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                            f"arm_{JOINT_MAPS[1].model_joint}")
    ax, ay = data.xanchor[jid][:2] / cell.bench.to_m
    off = math.dist((ax, ay), (cell.arm_x, cell.arm_y))
    assert off < 1e-3, (
        f"the m1 axis lands at x {ax:.4f}, y {ay:.4f} but was measured at "
        f"x {cell.arm_x:.4f}, y {cell.arm_y:.4f} ({off * 25.4:.1f} mm out) "
        f"- the attach frame is anchored on the wrong point")
    print(f"arm base squareness OK (base {got:.3f} deg vs clamped "
          f"{want:g}; m1 axis lands within {off * 25.4:.3f} mm)")


def _selftest_arm_faces_its_yaw(cell: 'Cell') -> None:
    """The arm must REACH along the yaw the cell asked for.

    The attach rotation composes the cell's yaw with the model's own idea
    of forward, and those are separate facts that no other check couples:
    the pose selftest compares the cell against the twin base-relative,
    so it passes happily with the whole arm rotated. Swapping the model
    turned the reach 90 deg (SO-100 reached along -Y, SO-101 along +X) —
    aiming the arm across the bench instead of at the wall, in a scene
    where every collision claim depends on which way it points.
    """
    import math

    import mujoco

    from sim.twin import BASE_BODY, JOINT_MAPS, TOOL_BODY, Twin

    if cell.arm_pose is None:
        print("arm reach direction: skipped (no arm placed)")
        return
    twin = Twin()
    worst = 0.0
    for yaw in (0.0, 90.0, -90.0, 180.0):
        model = build_model(cell.bench, cell.shadows, cell.cameras,
                            (cell.arm_x, cell.arm_y, yaw))
        data = rest_data(model)
        pose = {i: twin.cals[i].rest for i in sorted(twin.cals)}
        for j, deg in ((1, 0.0), (2, 60.0), (3, 0.0), (4, 0.0)):
            pose[j] = twin.cals[j].frame.tick(deg)
        for i, tick in pose.items():
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                  f"arm_{JOINT_MAPS[i].model_joint}")
            data.qpos[model.jnt_qposadr[j]] = twin.qpos_of(i, tick)[0]
        mujoco.mj_forward(model, data)
        bid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        v = data.xpos[bid("arm_" + TOOL_BODY)] - data.xpos[bid("arm_" + BASE_BODY)]
        got = math.degrees(math.atan2(v[1], v[0]))
        err = abs((got - yaw + 180.0) % 360.0 - 180.0)
        worst = max(worst, err)
        assert err < 0.5, (
            f"cell yaw {yaw:+.0f} deg but the arm reaches {got:+.1f} deg "
            f"({err:.1f} deg out) — the arm is pointed the wrong way")
    print(f"arm reach direction OK (worst {worst:.3f} deg over four yaws; "
          f"this model reaches along {twin.reach_yaw_deg():+.1f} deg of its "
          f"own frame)")


def _selftest_arm_pose_matches_twin(cell: 'Cell') -> None:
    """The arm in the CELL must stand exactly where the TWIN says it does.

    The whole point of the viewer is that watching a routine here is a
    statement about the real arm. That only holds while both models put
    the same ticks in the same place, so this compares every arm body,
    relative to the base, across the exercise routine's own waypoints.

    It exists because that guarantee broke silently: writing the rest
    keyframe into `model.qpos0` (an attempt to open the viewer at rest)
    re-zeroed every hinge, so the cell rendered `qpos - rest` while the
    gate simulated `qpos`. The gate passed the routine, the viewer played
    the arm straight through itself, and nothing failed. A pose is not
    verified by looking right - it is verified against the model that
    makes the safety claim.
    """
    import mujoco
    import numpy as np

    from sim.twin import BASE_BODY, JOINT_MAPS, Twin, exercise_waypoints

    if cell.arm_pose is None:
        print("arm-vs-twin pose: skipped (no arm placed)")
        return
    twin = Twin()
    model = build_model(cell.bench, cell.shadows, cell.cameras, cell.arm_pose)
    data = rest_data(model)
    # Read the body list OFF THE TWIN rather than listing names here. A
    # hardcoded list survived the SO-101 swap holding SO-100 names, and
    # this test still passed: mj_name2id returns -1 for a missing name
    # and xpos[-1] wraps to the last body, so both sides compared the
    # same wrong link and agreed perfectly. Every lookup is asserted
    # below for the same reason.
    bodies = [mujoco.mj_id2name(twin.model, mujoco.mjtObj.mjOBJ_BODY, b)
              for b in range(1, twin.model.nbody)]

    def bid(m, n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        assert i >= 0, f"model has no body {n!r}"
        return i

    adr = {}
    for i, jm in JOINT_MAPS.items():
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                              f"arm_{jm.model_joint}")
        assert j >= 0, f"cell model has no joint arm_{jm.model_joint}"
        adr[i] = model.jnt_qposadr[j]
    worst = 0.0
    for pose in exercise_waypoints(twin.cals, 70):
        twin.data.qpos[:] = twin._rest_qpos
        data.qpos[:] = model.qpos0
        for i, tick in pose.items():
            q = twin.qpos_of(i, tick)[0]
            twin.data.qpos[twin._adr[i]] = q
            data.qpos[adr[i]] = q
        mujoco.mj_forward(twin.model, twin.data)
        mujoco.mj_forward(model, data)
        # Compare in each model's own BASE-LOCAL frame. Base-relative
        # world offsets are not comparable: the cell rotates the arm by
        # its attach yaw, so identical poses give vectors that differ by
        # exactly that rotation. Dividing it out is what makes this a
        # test of the POSE rather than of the placement (which is the
        # reach-direction selftest's job).
        tb, cb = bid(twin.model, BASE_BODY), bid(model, "arm_" + BASE_BODY)
        t0 = twin.data.xpos[tb].copy()
        c0 = data.xpos[cb].copy()
        tR = twin.data.xmat[tb].reshape(3, 3).copy()
        cR = data.xmat[cb].reshape(3, 3).copy()
        for n in bodies:
            tv = tR.T @ (twin.data.xpos[bid(twin.model, n)] - t0)
            cv = cR.T @ (data.xpos[bid(model, "arm_" + n)] - c0)
            d = float(np.linalg.norm(tv - cv))
            worst = max(worst, d)
            assert d < 1e-6, (
                f"{n} sits {d * 1000:.1f} mm apart between the cell and the "
                f"twin at {pose} - the viewer is not showing what the gate "
                f"simulated")
    print(f"arm-vs-twin pose OK (worst disagreement {worst * 1000:.4f} mm "
          f"across the exercise routine)")


def animate_clip(cell: 'Cell', clip_path: Path, hz: float = 60.0,
                 reach_zone: tuple = ()) -> int:
    """Play a clip FILE in the viewer, at the pace the arm would run it.

    Same sampler the collision gate uses (`sim.clip.sample_clip`), so
    what is on screen is the motion that was gated rather than a
    lockstep interpolation between waypoints — the servos each run their
    own ramp and a short-travel joint arrives first.

    Nothing on the bench moves. This is the model.
    """
    import time

    import mujoco
    import mujoco.viewer

    from sim.clip import clip_duration, load_clip, sample_clip
    from sim.twin import JOINT_MAPS, Twin

    twin = Twin()
    clip = load_clip(twin.cals, clip_path)
    model = build_model(cell.bench, cell.shadows, cell.cameras,
                        cell.arm_pose, reach_zone)
    data = rest_data(model)

    adr: dict[int, int] = {}
    for i, jm in JOINT_MAPS.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                f"arm_{jm.model_joint}")
        if jid >= 0:
            adr[i] = model.jnt_qposadr[jid]

    frames = sample_clip(clip, hz)
    seconds = clip_duration(clip)
    print(f"clip '{clip.name}': {len(clip.poses)} poses, "
          f"{len(clip.edges())} edges -> {len(frames)} frames")
    print(f"  profile: speed {clip.profile.speed} ticks/s, acceleration "
          f"{clip.profile.acceleration} (x100 ticks/s^2) — the same "
          f"numbers the servos get")
    print(f"  runs {seconds:.1f} s on the bench, played here in real time")
    print("  nothing moves on the bench — this is the model only")
    print("  close the window to stop; it loops")

    saved = load_view()
    with mujoco.viewer.launch_passive(model, data) as v:
        v.opt.geomgroup[:] = 1
        if saved:
            for k, val in saved.items():
                if hasattr(v.cam, k):
                    setattr(v.cam, k, val)
        start = time.time()
        while v.is_running():
            t = (time.time() - start) % max(seconds, 0.001)
            f = frames[min(int(t / seconds * len(frames)), len(frames) - 1)]
            for i, tick in f.items():
                if i in adr:
                    data.qpos[adr[i]] = twin.qpos_of(i, tick)[0]
            mujoco.mj_forward(model, data)
            v.sync()
            time.sleep(1.0 / hz)
    return 0


def animate_exercise(cell: 'Cell', span: int = 70,
                     speed: int | None = None, hz: float = 60.0) -> int:
    """Play the exercise CLIP through the arm in the viewer (plan #660).

    What you watch here is the routine the arm runs, at the pace the arm
    runs it. The clip carries the same MotionProfile whose speed and
    acceleration are written into the servo registers, and the frames
    come from `sim.clip.sample_clip` - the one sampler the pre-flight
    collision gate also uses.

    That last part is the change. This used to interpolate its own way
    between waypoints, sliding every joint in lockstep so they all
    arrived together; the servos each run their own ramp and a
    short-travel joint parks early. The endpoints matched and the motion
    between them did not, which is precisely what "validate it in the
    sim first" cannot survive.

    Playback is REAL TIME against the clip's own duration, so if it
    looks slow here it will be slow on the bench.

    Nothing moves on the bench: this drives qpos directly and never
    touches the servo bus.
    """
    import time

    import mujoco
    import mujoco.viewer

    from hardware.bench.exercise import (
        ACCELERATION, SPAN_MAX, SPEED_BASE, SWEEP_ORDER, SWEEP_SPAN_CAPS,
        clamp_goal, exercise_clip, sweep_window)
    from sim.clip import MotionProfile, clip_duration, sample_clip
    from sim.twin import JOINT_MAPS, Twin

    twin = Twin()
    model = build_model(cell.bench, cell.shadows, cell.cameras, cell.arm_pose)
    data = rest_data(model)

    adr: dict[int, int] = {}
    for i, jm in JOINT_MAPS.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                f"arm_{jm.model_joint}")
        if jid >= 0:
            adr[i] = model.jnt_qposadr[jid]

    cals = twin.cals
    sweep_ids = [i for i in SWEEP_ORDER if i in cals]
    rest = {i: clamp_goal(cals[i], cals[i].rest) for i in sorted(cals)}
    windows = {i: sweep_window(cals[i],
                               min(span, SWEEP_SPAN_CAPS.get(i, SPAN_MAX)))
               for i in sweep_ids}
    profile = MotionProfile(speed=speed or SPEED_BASE,
                            acceleration=ACCELERATION)
    clip = exercise_clip(cals, rest, windows, sweep_ids, None, profile)
    frames = sample_clip(clip, hz)
    seconds = clip_duration(clip)
    print(f"exercise clip: {len(clip.poses)} poses -> {len(frames)} frames "
          f"at {span}% span")
    print(f"  profile: speed {profile.speed} ticks/s, acceleration "
          f"{profile.acceleration} (x100 ticks/s^2) - the same numbers the "
          f"servos get")
    print(f"  runs {seconds:.1f} s on the bench, played here in real time")
    print("  nothing moves on the bench - this is the model only")
    print("  close the window to stop; it loops")

    saved = load_view()
    with mujoco.viewer.launch_passive(model, data) as v:
        v.opt.geomgroup[:] = 1
        v.opt.geomgroup[GROUP_HIDDEN] = 0
        if saved:
            v.cam.azimuth = saved["azimuth"]
            v.cam.elevation = saved["elevation"]
            v.cam.distance = saved["distance"]
            v.cam.lookat[:] = saved["lookat"]
        n = 0
        while v.is_running():
            pose = frames[n % len(frames)]
            for i, tick in pose.items():
                if i in adr:
                    data.qpos[adr[i]] = twin.qpos_of(i, tick)[0]
            mujoco.mj_forward(model, data)
            v.sync()
            n += 1
            time.sleep(1.0 / hz)
    return 0


def load_cell(path: Path = CELL_JSON) -> Cell:
    """Load the project's cell config, and the bench it points at."""
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise BenchError(
            f"no cell config at {path}",
            f"it names the bench to use and where the arm and cameras "
            f"sit on it") from exc
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise BenchError(f"could not read {path}: {exc}",
                         "fix or delete the file") from exc
    bench_path = Path(doc.get("bench") or BENCH_JSON)
    bench = load_bench(bench_path)
    arm = doc.get("arm") or {}
    where = f"{path} arm"
    return Cell(
        bench=bench, bench_path=bench_path,
        arm_x=_num(arm, "x", where, required=False),
        arm_y=_num(arm, "y", where, required=False),
        arm_yaw_deg=_num(arm, "yaw_deg", where, required=False),
        arm_base_square_deg=_num(arm, "base_square_to_deg", where,
                                 required=False),
        # A camera with no measured position is SKIPPED, not defaulted.
        # It exists and streams (cameras.json knows about it) but nobody
        # has measured where it is, and inventing a pose would draw a
        # sight line that looks like evidence — and would make
        # `sim.simcam` render a view no real camera has. Absent is
        # honest; a plausible guess is not.
        cameras=tuple(
            Camera(name=c.get("name", "cam"), wall=c.get("wall"),
                   along=(float(c["along"])
                          if c.get("along") is not None else None),
                   x=float(c["x"]) if c.get("x") is not None else None,
                   y=float(c["y"]) if c.get("y") is not None else None,
                   yaw_deg=(float(c["yaw_deg"])
                            if c.get("yaw_deg") is not None else None),
                   z=float(c["z"]),
                   tilt_deg=float(c.get("tilt_deg") or 60.0),
                   standoff=(float(c["standoff"])
                             if c.get("standoff") is not None else None),
                   fovy_deg=(float(c["fovy_deg"])
                             if c.get("fovy_deg") is not None else None),
                   planned=bool(c.get("planned", False)),
                   notes=c.get("notes", ""))
            for c in (doc.get("cameras") or [])
            if c.get("z") is not None
            and (c.get("along") is not None
                 or (c.get("x") is not None and c.get("y") is not None))),
        shadows=bool((doc.get("render") or {}).get("shadows", False)))


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
    bench_only = "--bench-only" in sys.argv
    try:
        if bench_only:
            cell = Cell(bench=load_bench(), bench_path=BENCH_JSON)
        else:
            cell = load_cell()
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2
    if save_xml is not None:
        save_xml.write_text(build_spec(cell.bench, cell.shadows, cell.cameras, cell.arm_pose).to_xml())
        print(f"wrote {save_xml}")
        print(f"  open it standalone with: uv run python -m mujoco.viewer "
              f"--mjcf={save_xml}")
        return 0
    if "--selftest" in sys.argv:
        _selftest_arm_frame()
        _selftest_arm_base_square(cell)
        _selftest_arm_faces_its_yaw(cell)
        _selftest_arm_pose_matches_twin(cell)
        return 0
    if "--exercise" in sys.argv:
        return animate_exercise(cell)
    if "--clip" in sys.argv:
        i = sys.argv.index("--clip")
        if len(sys.argv) <= i + 1:
            print("error: --clip needs a clip file", file=sys.stderr)
            print("hint:  e.g. --clip swing-tour.json", file=sys.stderr)
            return 2
        zone = ()
        if "--reach" in sys.argv:
            from .reach import zone_tiles
            zone = zone_tiles(cell)
        try:
            return animate_clip(cell, Path(sys.argv[i + 1]), reach_zone=zone)
        except BenchError as exc:
            print(f"error: {exc}", file=sys.stderr)
            if exc.hint:
                print(f"hint:  {exc.hint}", file=sys.stderr)
            return 2
    if "--view" in sys.argv:
        zone = ()
        if "--reach" in sys.argv:
            # Local import: sim.reach imports THIS module for load_cell,
            # so a module-scope import here would be circular.
            from .reach import zone_tiles
            zone = zone_tiles(cell)
        return view(cell, reach_zone=zone)
    if render_to is not None:
        for p in render(cell.bench, render_to, cameras=cell.cameras,
                        arm=cell.arm_pose):
            print(f"wrote {p}")
        return 0
    if bench_only:
        # The PLACE on its own: the project's gaps (arm, cameras) are
        # not the bench's problem and would read as defects in it.
        print(cell.bench.describe())
        print()
        print(cell.bench.sketch())
        gaps = cell.bench.missing()
    else:
        print(cell.describe())
        print()
        print(cell.sketch())
        gaps = cell.missing()
    if gaps:
        print("\nNOT MODELLED (measure these - do not guess them):")
        for g in gaps:
            print(f"  - {g}")
        print("\nWhile anything above is unmeasured, a clean collision gate "
              "does NOT\nmean the workspace is safe - only that the arm "
              "will not hit itself.")
    else:
        print("\nthe bench is fully measured" if bench_only
              else "\nthe cell is fully measured")
    return 0


if __name__ == "__main__":
    sys.exit(main())
