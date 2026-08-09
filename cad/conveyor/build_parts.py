# Mini modular conveyor — parametric part set (Hive plan #835)
#
# Run:  "%LOCALAPPDATA%/Programs/FreeCAD 1.1/bin/freecadcmd.exe" cad/conveyor/build_parts.py
#
# Writes parts/*.stl + parts/*.step and a step-by-step build.log. freecadcmd
# swallows stdout and can die without a traceback, so if a run produces nothing,
# read build.log first — it names the last step that started.
#
# Three FreeCAD scripting traps this file is written around (CableCell/cad/README.md):
#   1. Shape.translate() mutates in place and returns None. Everything here is
#      built at its final position or uses translated().
#   2. freecadcmd sets __name__ to the module basename, so a __main__ guard
#      never fires. There isn't one.
#   3. Routing an STL through a Mesh::Feature crashes the process. Meshes are
#      written directly via Mesh.Mesh(shape.tessellate(dev)).write(path).

import os
import Part
import Mesh
from FreeCAD import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "parts")
LOG = os.path.join(HERE, "build.log")

os.makedirs(OUT, exist_ok=True)
_log = open(LOG, "w")


def step(msg):
    _log.write(msg + "\n")
    _log.flush()


# ---------------------------------------------------------------- parameters
# Coordinates: X along the belt, Y across it, Z up. Change a number here and
# rerun; nothing downstream is hard-coded.

belt_width      = 50.0    # Kyle, 2026-08-08
belt_thickness  = 1.5     # printed TPU loop wall

roller_dia      = 25.0    # drive roller only — the discharge end uses the nose roller below
roller_flange_d = 28.0    # keeps the belt tracking
roller_flange_w = 1.5
roller_len      = belt_width + 2 * roller_flange_w

# Nose roller — the fix for the transfer gap (sim finding, rev 38/40).
# A Ø25 roller in a concentric stadium end sits bracket_h/2 = 17.5 mm inboard of
# the module edge, and that inset was the dominant term in a 27 mm unsupported
# span against a 32 mm part. A small roller on a SQUARED end carries the belt
# apex out to within nose_edge of the module face instead. Ø10 is the industrial
# nose-bar answer scaled to this build.
nose_dia        = 10.0
nose_axle_dia   = 4.0
nose_edge       = 6.0     # module face to nose axis when fully tensioned
nose_travel     = 8.0     # take-up slot; a printed belt only absorbs tolerance

side_gap        = 1.0     # roller end to bracket inner face
wall            = 3.0
mating_wall     = 2.0     # thinner on the face that butts the next module
inner_width     = roller_len + 2 * side_gap
outer_width     = inner_width + 2 * wall

bracket_h       = 35.0
straight_len    = 120.0   # straight module
corner_len      = 70.0    # corner module — same mechanism, built short
frame_gap       = 1.5     # module face to module face (was 4.0)

axle_dia        = 5.0
bearing_od      = 10.0    # MR105ZZ
bearing_t       = 4.0

motor_shaft_d   = 3.0     # N20
motor_shaft_flat= 2.5     # across the D
motor_nose_d    = 12.0
motor_bolt_pitch= 10.0
motor_bolt_d    = 2.2

m3_clear        = 3.2

TESS = 0.04


import math

# The carry surface — the plane the part actually rides on. Both rollers are
# aligned to THIS, not to a shared centreline, which is why the nose axis sits
# higher than the drive axis.
carry_z = bracket_h / 2.0 + roller_dia / 2.0

drive_z = bracket_h / 2.0
nose_z = carry_z - nose_dia / 2.0


def roller_axis_x(module_len):
    # Drive end keeps its concentric stadium. Discharge end is squared, so the
    # nose axis is set by the module face, not by bracket_h.
    return bracket_h / 2.0, module_len - nose_edge


def belt_path_length(module_len):
    # Two pulleys of DIFFERENT radii at different heights — an open-belt wrap,
    # not a stadium. Getting this wrong mis-sizes the printed TPU loop, and a
    # loop that is 3 mm long simply will not tension.
    ax0, ax1 = roller_axis_x(module_len)
    r1 = roller_dia / 2.0
    r2 = nose_dia / 2.0
    d = math.hypot(ax1 - ax0, nose_z - drive_z)
    dr = r1 - r2
    tangent = math.sqrt(d * d - dr * dr)
    alpha = math.asin(dr / d)
    return 2.0 * tangent + r1 * (math.pi + 2 * alpha) + r2 * (math.pi - 2 * alpha)


def unsupported_span(feed_len=None):
    # What the sim measured, computed straight from the parameters so it can
    # never drift from the geometry. Support ends at the nose axis; the corner's
    # belt starts one mating wall + clearance + flange in from its own face.
    feed_len = straight_len if feed_len is None else feed_len
    _, nose_x = roller_axis_x(feed_len)
    return (feed_len - nose_x) + frame_gap + (mating_wall + side_gap + roller_flange_w)


# ------------------------------------------------------------- side bracket
def make_bracket(module_len, t=None, top_z=None):
    # Rounded at the drive end, SQUARED at the discharge end. The square end is
    # the whole point: it lets the nose axis sit nose_edge from the module face
    # instead of bracket_h/2.
    #
    # top_z cuts the plate down flush with the carry plane. A full-height plate
    # stands bracket_h - (carry_z + belt_thickness) = 3.5 mm PROUD of its own
    # belt, which is a side rail on a through face and a kerb on a transfer face.
    # The sim found this the hard way: the part crossed the gap fine and then
    # stopped dead against the receiving module's wall.
    t = wall if t is None else t
    step("bracket: len=%.1f thickness=%.1f (rounded drive end, squared nose end)" % (module_len, t))
    ax0, nose_x = roller_axis_x(module_len)
    r = bracket_h / 2.0

    body = Part.makeBox(module_len - ax0, t, bracket_h, Vector(ax0, 0, 0))
    body = body.fuse(Part.makeCylinder(r, t, Vector(ax0, 0, drive_z), Vector(0, 1, 0)))

    step("bracket: drive-end bore")
    body = body.cut(Part.makeCylinder(bearing_od / 2.0, t, Vector(ax0, -1, drive_z), Vector(0, 1, 0)))

    step("bracket: nose take-up slot")
    sw = nose_axle_dia + 0.5
    # Slot runs INBOARD from the tensioned position — take-up pulls the nose out
    # toward the face, so the fully-tensioned axis is the outer limit and the
    # design span is what you actually get.
    slot = Part.makeBox(nose_travel, t + 2, sw, Vector(nose_x - nose_travel, -1, nose_z - sw / 2.0))
    slot = slot.fuse(Part.makeCylinder(sw / 2.0, t + 2, Vector(nose_x - nose_travel, -1, nose_z), Vector(0, 1, 0)))
    slot = slot.fuse(Part.makeCylinder(sw / 2.0, t + 2, Vector(nose_x, -1, nose_z), Vector(0, 1, 0)))
    body = body.cut(slot)

    step("bracket: cross-member mounting holes")
    for hx in (ax0 + 18.0, nose_x - 18.0):
        body = body.cut(Part.makeCylinder(m3_clear / 2.0, t + 2, Vector(hx, -1, 7.0), Vector(0, 1, 0)))

    if top_z is not None and top_z < bracket_h:
        step("bracket: cut down to z=%.1f (open transfer face)" % top_z)
        body = body.cut(Part.makeBox(module_len + 2 * bracket_h, t + 2, bracket_h,
                                     Vector(-bracket_h, -1, top_z)))

    return body


# ------------------------------------------------------------------ rollers
def _roller_blank():
    r = roller_dia / 2.0
    fr = roller_flange_d / 2.0
    body = Part.makeCylinder(r, belt_width, Vector(0, roller_flange_w, 0), Vector(0, 1, 0))
    body = body.fuse(Part.makeCylinder(fr, roller_flange_w, Vector(0, 0, 0), Vector(0, 1, 0)))
    body = body.fuse(Part.makeCylinder(fr, roller_flange_w,
                                       Vector(0, roller_len - roller_flange_w, 0), Vector(0, 1, 0)))
    return body


def make_roller_nose():
    # Replaces the Ø25 idler entirely. At Ø10 there is no room for an MR105
    # (10 mm OD is the whole roller), so this runs as a plain bearing directly on
    # the Ø4 axle — acceptable on the slack end, where load is belt tension only.
    step("roller/nose: Ø%.0f blank + plain bore" % nose_dia)
    r = nose_dia / 2.0
    fr = (nose_dia + 3.0) / 2.0
    body = Part.makeCylinder(r, belt_width, Vector(0, roller_flange_w, 0), Vector(0, 1, 0))
    body = body.fuse(Part.makeCylinder(fr, roller_flange_w, Vector(0, 0, 0), Vector(0, 1, 0)))
    body = body.fuse(Part.makeCylinder(fr, roller_flange_w,
                                       Vector(0, roller_len - roller_flange_w, 0), Vector(0, 1, 0)))
    body = body.cut(Part.makeCylinder(nose_axle_dia / 2.0 + 0.2, roller_len + 2,
                                      Vector(0, -1, 0), Vector(0, 1, 0)))
    return body


def make_roller_drive():
    step("roller/drive: blank + D-shaft bore + grub screw")
    body = _roller_blank()
    depth = 14.0
    # The bore is the shape of the SHAFT: a Ø3 cylinder with one side flattened
    # to 2.5 across. Build the shaft, then subtract it.
    shaft = Part.makeCylinder(motor_shaft_d / 2.0 + 0.15, depth, Vector(0, -0.5, 0), Vector(0, 1, 0))
    beyond = motor_shaft_flat - motor_shaft_d / 2.0
    sliver = Part.makeBox(motor_shaft_d + 2, depth + 1, motor_shaft_d,
                          Vector(-(motor_shaft_d / 2.0 + 1), -1, beyond))
    shaft = shaft.cut(sliver)
    body = body.cut(shaft)
    # relieve the far end so it can still ride an axle stub
    body = body.cut(Part.makeCylinder(axle_dia / 2.0 + 0.15, roller_len - depth + 1,
                                      Vector(0, depth, 0), Vector(0, 1, 0)))
    # M3 grub screw, radially inward from the outside surface to the bore.
    body = body.cut(Part.makeCylinder(1.6, roller_dia / 2.0,
                                      Vector(0, 6.0, roller_dia / 2.0), Vector(0, 0, -1)))
    return body


# --------------------------------------------------------------- slider bed
def make_slider_bed(module_len, t=None):
    t = wall if t is None else t
    step("slider bed: plate + lead-in chamfers, len=%.1f" % module_len)
    ax0, nose_x = roller_axis_x(module_len)
    span = nose_x - ax0
    bed = Part.makeBox(span, inner_width, wall, Vector(ax0, t, carry_z - wall))
    try:
        edges = [e for e in bed.Edges
                 if abs(e.CenterOfMass.z - carry_z) < 1e-6 and
                 (abs(e.CenterOfMass.x - ax0) < 1e-6 or abs(e.CenterOfMass.x - nose_x) < 1e-6)]
        if edges:
            bed = bed.makeChamfer(1.2, edges)
    except Exception as exc:
        step("slider bed: chamfer skipped (%s)" % exc)
    return bed


# -------------------------------------------------------------- motor mount
def make_motor_mount():
    step("motor mount: plate + nose clearance + N20 bolt pattern")
    plate = Part.makeBox(bracket_h, wall, bracket_h, Vector(0, 0, 0))
    c = bracket_h / 2.0
    plate = plate.cut(Part.makeCylinder(motor_nose_d / 2.0, wall + 2, Vector(c, -1, c), Vector(0, 1, 0)))
    for dz in (-motor_bolt_pitch / 2.0, motor_bolt_pitch / 2.0):
        plate = plate.cut(Part.makeCylinder(motor_bolt_d / 2.0, wall + 2,
                                            Vector(c, -1, c + dz), Vector(0, 1, 0)))
    for dz in (-14.0, 14.0):
        plate = plate.cut(Part.makeCylinder(m3_clear / 2.0, wall + 2,
                                            Vector(6.0, -1, c + dz), Vector(0, 1, 0)))
    return plate


# ---------------------------------------------------------- corner guide rail
def make_guide_rail():
    step("guide rail: L-section")
    rail = Part.makeBox(wall, outer_width, 15.0, Vector(0, 0, 0))
    rail = rail.fuse(Part.makeBox(12.0, outer_width, wall, Vector(0, 0, 0)))
    return rail


# ------------------------------------------------- belt (render only, not printed)
def _tangent_normal(c1, r1, c2, r2, upper):
    # A single line tangent to both circles on the same side shares one normal n,
    # and n·(c2-c1) = r1-r2. That gives acos, NOT asin — asin is the classic
    # wrong turn here and it tilts the carry run by about a degree, which is
    # enough to make the belt sit off the slider bed.
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    dist = math.hypot(dx, dz)
    alpha = math.atan2(dz, dx)
    off = math.acos((r1 - r2) / dist)
    psi = alpha + off if upper else alpha - off
    return math.cos(psi), math.sin(psi)


def make_belt(module_len):
    ax0, nose_x = roller_axis_x(module_len)
    c1, r1 = (ax0, drive_z), roller_dia / 2.0
    c2, r2 = (nose_x, nose_z), nose_dia / 2.0
    bt = belt_thickness

    def prism(grow):
        pts = []
        for upper in (True, False):
            n = _tangent_normal(c1, r1, c2, r2, upper)
            p1 = (c1[0] + (r1 + grow) * n[0], c1[1] + (r1 + grow) * n[1])
            p2 = (c2[0] + (r2 + grow) * n[0], c2[1] + (r2 + grow) * n[1])
            pts.extend([p1, p2] if upper else [p2, p1])
        verts = [Vector(p[0], 0, p[1]) for p in pts]
        poly = Part.makePolygon(verts + [verts[0]])
        return Part.Face(poly).extrude(Vector(0, belt_width, 0))

    def loop(grow):
        s = Part.makeCylinder(r1 + grow, belt_width, Vector(c1[0], 0, c1[1]), Vector(0, 1, 0))
        s = s.fuse(Part.makeCylinder(r2 + grow, belt_width, Vector(c2[0], 0, c2[1]), Vector(0, 1, 0)))
        return s.fuse(prism(grow))

    return loop(bt).cut(loop(0.0))


# -------------------------------------------------------------------- export
def export(shape, name):
    step("export: " + name)
    shape.exportStep(os.path.join(OUT, name + ".step"))
    Mesh.Mesh(shape.tessellate(TESS)).write(os.path.join(OUT, name + ".stl"))


step("=== build start ===")

# The corner's side plates are thinner, because one of them is the face the part
# has to cross. Every mm here is a mm of unsupported span.
corner_outer_width = inner_width + 2 * mating_wall
corner_offset_x = straight_len + frame_gap + corner_outer_width

export(make_bracket(straight_len), "bracket_straight")
export(make_bracket(corner_len, t=mating_wall), "bracket_corner_outer")
export(make_bracket(corner_len, t=mating_wall, top_z=carry_z), "bracket_corner_infeed")
export(make_roller_nose(), "roller_nose")
export(make_roller_drive(), "roller_drive")
export(make_slider_bed(straight_len), "slider_bed_straight")
export(make_motor_mount(), "motor_mount")
export(make_guide_rail(), "guide_rail")


# ---- assemblies, for rendering -------------------------------------------
def assemble(module_len, t=None, with_belt=True, open_face=False):
    # open_face cuts the far side plate (local +y, which is the side a feeding
    # module butts against once this one is rotated) down to the carry plane.
    t = wall if t is None else t
    step("assembly: len=%.1f t=%.1f open_face=%s" % (module_len, t, open_face))
    ax0, nose_x = roller_axis_x(module_len)
    ry = t + side_gap

    parts = [make_bracket(module_len, t),
             make_bracket(module_len, t, top_z=carry_z if open_face else None
                          ).translated(Vector(0, inner_width + t, 0)),
             make_roller_drive().translated(Vector(ax0, ry, drive_z)),
             make_roller_nose().translated(Vector(nose_x, ry, nose_z)),
             make_slider_bed(module_len, t)]

    if with_belt:
        parts.append(make_belt(module_len).translated(Vector(0, ry + roller_flange_w, 0)))

    out = parts[0]
    for p in parts[1:]:
        out = out.fuse(p)
    return out


export(assemble(straight_len), "assembly_straight")
export(assemble(corner_len, t=mating_wall, open_face=True), "assembly_corner")


# Components exported separately so the renderer can colour them independently.
def export_components(module_len, tag, t=None, rot=0.0, offset=None, open_face=False):
    t = wall if t is None else t
    step("components: %s" % tag)
    ax0, nose_x = roller_axis_x(module_len)
    ry = t + side_gap

    def place(s):
        s = s.copy()
        if rot:
            s.rotate(Vector(0, 0, 0), Vector(0, 0, 1), rot)
        if offset is not None:
            s = s.translated(offset)
        return s

    br = make_bracket(module_len, t)
    br = br.fuse(make_bracket(module_len, t, top_z=carry_z if open_face else None
                              ).translated(Vector(0, inner_width + t, 0)))
    export(place(br), tag + "_brackets")

    ro = make_roller_drive().translated(Vector(ax0, ry, drive_z))
    ro = ro.fuse(make_roller_nose().translated(Vector(nose_x, ry, nose_z)))
    export(place(ro), tag + "_rollers")

    export(place(make_slider_bed(module_len, t)), tag + "_bed")
    export(place(make_belt(module_len).translated(Vector(0, ry + roller_flange_w, 0))), tag + "_belt")


export_components(straight_len, "cs")
export_components(corner_len, "cc", t=mating_wall, open_face=True,
                  rot=90.0, offset=Vector(corner_offset_x, 0, 0))

# ---- the L: straight feeding a corner ------------------------------------
step("assembly: L = straight + corner, corner belt perpendicular")
a_str = assemble(straight_len)
a_cor = assemble(corner_len, t=mating_wall, open_face=True)
# rotate the corner 90 deg about Z so its belt runs across the feed, then park
# it beyond the straight's discharge end.
a_cor = a_cor.copy()
a_cor.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 90)
a_cor = a_cor.translated(Vector(corner_offset_x, 0, 0))
export(a_str.fuse(a_cor), "assembly_L")

step("belt path lengths: straight=%.1f mm (TPU cyl dia %.1f) corner=%.1f mm (dia %.1f)"
     % (belt_path_length(straight_len), belt_path_length(straight_len) / math.pi,
        belt_path_length(corner_len), belt_path_length(corner_len) / math.pi))

_ax0, _nose_x = roller_axis_x(straight_len)
step("TRANSFER GAP: support ends x=%.1f, corner belt starts x=%.1f, unsupported=%.1f mm"
     % (_nose_x, corner_offset_x - corner_outer_width + mating_wall + side_gap + roller_flange_w,
        unsupported_span()))
step("  breakdown: nose inset %.1f + frame gap %.1f + corner side inset %.1f"
     % (straight_len - _nose_x, frame_gap, mating_wall + side_gap + roller_flange_w))

# ---- publish the derived geometry ----------------------------------------
# The sim used to re-declare these constants by hand, which meant a dimension
# change here silently left the sim testing the OLD design. It is the same
# defect class as a stale blocker inside a live ticket: both halves individually
# correct, the pair wrong. The CAD is the single source; everything else reads.
# Explicit utf-8 — this repo has an open bug (#751) about JSON going out in the
# platform locale encoding, and there is no reason to add to it.
import json

_c_ax0, _c_nose_x = roller_axis_x(corner_len)
geom = {
    "_generated_by": "cad/conveyor/build_parts.py — do not hand-edit",
    "belt_width": belt_width, "belt_thickness": belt_thickness,
    "roller_dia": roller_dia, "nose_dia": nose_dia,
    "roller_flange_w": roller_flange_w, "side_gap": side_gap,
    "bracket_h": bracket_h, "wall": wall, "mating_wall": mating_wall,
    "inner_width": inner_width, "outer_width": outer_width,
    "carry_z": carry_z, "drive_z": drive_z, "nose_z": nose_z,
    # rail_top is the top of each side plate, [local y=0 side, local y=max side].
    # The corner's y=max plate is its infeed face and is cut to the carry plane
    # so a part can cross onto it; everything else stays full height as a rail.
    "straight": {"len": straight_len, "drive_ax": _ax0, "nose_ax": _nose_x,
                 "t": wall, "outer_width": outer_width,
                 "rail_top": [bracket_h, bracket_h],
                 "belt_len": belt_path_length(straight_len)},
    "corner": {"len": corner_len, "drive_ax": _c_ax0, "nose_ax": _c_nose_x,
               "t": mating_wall, "outer_width": corner_outer_width,
               "rail_top": [bracket_h, carry_z],
               "belt_len": belt_path_length(corner_len),
               "offset_x": corner_offset_x},
    "frame_gap": frame_gap,
    "unsupported_span": unsupported_span(),
}
with open(os.path.join(OUT, "geometry.json"), "w", encoding="utf-8") as fh:
    json.dump(geom, fh, indent=2)
step("export: geometry.json")

step("=== build complete ===")
_log.close()
