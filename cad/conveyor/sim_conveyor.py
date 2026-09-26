# Mini modular conveyor — MuJoCo sim (Hive plan #835)
#
#   uv run python cad/conveyor/sim_conveyor.py            # nominal run, frames in renders/sim/
#   uv run python cad/conveyor/sim_conveyor.py --frames 0 # same run, no meshes, no PNGs
#   uv run python cad/conveyor/sim_conveyor.py --sweep    # acceptance matrix, writes sweep.json
#   uv run python cad/conveyor/sim_conveyor.py --view     # interactive viewer
#
# Visuals are the component STLs. Collision is not: a belt loop is non-convex
# and MuJoCo would fill its convex hull solid, which hides the gap this sim
# exists to measure. Straights keep two nose cylinders, a carry plate and the
# rails that stand above the belt. Each cone is sliced into short frustums so
# a line contact is several points.
#
# The drive is MuJoCo's own friction on a surface that is already moving.
# A force computed in Python is an explicit damper on the part's yaw, and that
# damper went unstable at the friction this sweep has to cover.

import os
import sys
import math
import json
import time
import concurrent.futures
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = os.path.join(HERE, "parts")
OUT = os.path.join(HERE, "renders", "sim")
os.makedirs(OUT, exist_ok=True)

MM = 0.001

with open(os.path.join(PARTS, "geometry.json"), encoding="utf-8") as fh:
    G = json.load(fh)

belt_width = G["belt_width"]
belt_thickness = G["belt_thickness"]
side_gap = G["side_gap"]
roller_flange_w = G["roller_flange_w"]
STR = G["straight"]
straight_len = STR["len"]
nose_r = G["nose_dia"] / 2.0 + belt_thickness
belt_top = G["carry_z"] + belt_thickness
nose_z = G["nose_z"]
CURVE = G["curve"]
S1 = G["s1"]
S2 = G["s2"]
TAB = G["motor_tab"]

# 270 RPM on the Ø11 neutral axis. The cone centreline is cut to the same speed.
NOMINAL_SPEED = 0.155


def belt_y():
    y0 = STR["t"] + side_gap + roller_flange_w
    return y0, y0 + belt_width


def lane_centre():
    y0, y1 = belt_y()
    return (y0 + y1) / 2.0


def placer(rot, ox, oy):
    # rot 90 is the module's left turn: local (x, y) lands on world (−y, x).
    if rot == 0:
        return lambda x, y: (x + ox, y + oy)
    return lambda x, y: (-y + ox, x + oy)


def place_box(rot, ox, oy, cx, cy, cz, hx, hy, hz):
    # Same map as placer. A 90° turn swaps the horizontal half-sizes.
    if rot == 0:
        return (cx + ox, cy + oy, cz), (hx, hy, hz)
    return (-cy + ox, cx + oy, cz), (hy, hx, hz)


PART = (32.0, 32.0, 16.0)
PART_MASS = 0.030

# A contact from the part has to leave the commanded surface speed alone
# inside one step, or the friction is no longer the belt's. These sit far
# above that impulse. Slide armature is a mass, hinge armature an inertia.
SLIDE_ARMATURE = 50.0
HINGE_ARMATURE = 0.5

# Part versus static (rails, walls, floor) versus drive. A nose occupies the
# same volume as the slab, and a cone can sit against a wall; neither pair is
# a contact the part makes.
CON_PART = 1
CON_STATIC = 2
CON_DRIVE = 4


def m(v):
    return v * MM


def rgba(c, a=1.0):
    return "%.3f %.3f %.3f %.3f" % (c[0] / 255, c[1] / 255, c[2] / 255, a)


BRACKET_C = (208, 206, 198)
ROLLER_C = (196, 132, 74)
BED_C = (74, 126, 178)
RETURN_C = (108, 152, 120)
BELT_C = (58, 58, 62)
PART_C = (200, 69, 43)
MOTOR_C = (118, 120, 124)
ORING_C = (42, 40, 38)
KEEPER_C = (186, 148, 96)
TIE_C = (96, 108, 112)
TENSION_C = (176, 132, 86)
JOIN_C = (150, 158, 132)

VISUALS = [
    ("cs_brackets", BRACKET_C), ("cs_rollers", ROLLER_C), ("cs_bed", BED_C),
    ("cs_return", RETURN_C), ("cs_belt", BELT_C), ("cs_tiebars", TIE_C),
    ("cs_tension", TENSION_C), ("cs_motor", MOTOR_C),
    ("cv_frame", BRACKET_C), ("cv_rollers", ROLLER_C), ("cv_orings", ORING_C),
    ("cv_keeper", KEEPER_C), ("cv_motor", MOTOR_C),
    ("s2_brackets", BRACKET_C), ("s2_rollers", ROLLER_C), ("s2_bed", BED_C),
    ("s2_return", RETURN_C), ("s2_belt", BELT_C), ("s2_tiebars", TIE_C),
    ("s2_tension", TENSION_C), ("s2_motor", MOTOR_C),
    ("jn_1", JOIN_C), ("jn_2", JOIN_C),
]


def _argv(flag, default):
    if flag not in sys.argv:
        return default
    return type(default)(sys.argv[sys.argv.index(flag) + 1])


STRAIGHT_SPEED = _argv("--speed", NOMINAL_SPEED)
CURVE_SPEED = _argv("--curve-speed", STRAIGHT_SPEED)
MU_BELT = _argv("--mu", 0.9)
MU_CURVE = _argv("--mu-curve", 0.9)
ENTRY_OFFSET = _argv("--offset", 0.0)
DT = _argv("--dt", 0.0005)
# Soft contacts creep at the constraint time constant. Without this, a part
# that should be stuck to the belt slowly walks its heading. Ten iterations
# left a degree of yaw against a halved step. Thirty saturates this timestep
# but not the halved one; sixty agrees with both.
NOSLIP = _argv("--noslip", 60)


def _stl_faces(path):
    import struct
    with open(path, "rb") as fh:
        fh.read(80)
        return struct.unpack("<I", fh.read(4))[0]


def _load_stl(path):
    import struct
    with open(path, "rb") as fh:
        fh.read(80)
        n = struct.unpack("<I", fh.read(4))[0]
        raw = fh.read(n * 50)
    rec = np.frombuffer(raw, dtype=np.uint8).reshape(n, 50)
    return np.ndarray((n, 3, 3), dtype="<f4", buffer=rec[:, 12:48].copy()).astype(np.float64)


def _write_stl(path, verts, faces):
    import struct
    tri = np.ascontiguousarray(verts[faces], dtype="<f4")
    n = tri.shape[0]
    packed = np.zeros((n, 50), dtype=np.uint8)
    packed[:, 12:48] = tri.view(np.uint8).reshape(n, 36)
    with open(path, "wb") as fh:
        fh.write(b"decimated visual".ljust(80, b"\0"))
        fh.write(struct.pack("<I", n))
        fh.write(packed.tobytes())


_VIS_PATH = {}


def visual_file(name):
    # MuJoCo refuses an STL over 200k faces. The cone and O-ring exports are
    # past that; a 0.15 mm weld keeps the shape and stays under the limit.
    # Collision does not use these files.
    src = os.path.join(PARTS, name + ".stl")
    cached = _VIS_PATH.get(name)
    if cached and os.path.exists(cached):
        return cached
    if _stl_faces(src) <= 180000:
        _VIS_PATH[name] = src
        return src
    tris = _load_stl(src)
    pitch = 0.15
    q = np.round(tris / pitch).astype(np.int64)
    uniq, inv = np.unique(q.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = faces[keep]
    dest = os.path.join(OUT, "_vis_%s.stl" % name)
    _write_stl(dest, uniq.astype(np.float64) * pitch, faces)
    _VIS_PATH[name] = dest
    return dest


def mesh_assets():
    out = []
    for n, _ in VISUALS:
        p = visual_file(n).replace("\\", "/")
        out.append('<mesh name="%s" file="%s" scale="%g %g %g"/>' % (n, p, MM, MM, MM))
    return "\n    ".join(out)


def cone_frame(theta_deg):
    th = math.radians(theta_deg)
    alpha = math.radians(CURVE["alpha_deg"])
    ca, sa = math.cos(alpha), math.sin(alpha)
    u = (math.cos(th) * ca, math.sin(th) * ca, -sa)
    e_th = (-math.sin(th), math.cos(th), 0.0)
    e_up = (sa * math.cos(th), sa * math.sin(th), ca)
    return u, e_th, e_up, ca, alpha


def frustum_vertices(theta_deg, s_lo, s_hi):
    # Two circles. The hull is the slice of the cone between them. The apex
    # stays out of it, or the hull would fill the lane.
    u, e_th, e_up, _, alpha = cone_frame(theta_deg)
    cx, cy = CURVE["centre"]
    z_top = CURVE["z_top"]
    verts = []
    for s in (s_lo, s_hi):
        rad = s * math.tan(alpha)
        ox = cx + s * u[0]
        oy = cy + s * u[1]
        oz = z_top + s * u[2]
        for i in range(32):
            a = 2.0 * math.pi * i / 32.0
            c, s_ = math.cos(a), math.sin(a)
            verts.append((
                ox + rad * (c * e_th[0] + s_ * e_up[0]),
                oy + rad * (c * e_th[1] + s_ * e_up[1]),
                oz + rad * (c * e_th[2] + s_ * e_up[2]),
            ))
    return verts


def n_frusta():
    # The crown is a straight line of length r_out − r_in. Pieces no longer
    # than 4 mm, because one contact on a full-length crown can land anywhere
    # and the part then rocks about the two points it happens to get.
    length = CURVE["cone_r"][1] - CURVE["cone_r"][0]
    return int(math.ceil(length / 4.0 - 1e-9))


def frustum_ranges():
    alpha = math.radians(CURVE["alpha_deg"])
    ca = math.cos(alpha)
    r0, r1 = CURVE["cone_r"]
    s0, s1 = r0 * ca, r1 * ca
    n = n_frusta()
    return [(s0 + (s1 - s0) * k / n, s0 + (s1 - s0) * (k + 1) / n) for k in range(n)]


def along_crown(theta_deg, p):
    # Millimetres from the small-end crown, along the top line.
    u, _, e_up, ca, alpha = cone_frame(theta_deg)
    s0 = CURVE["cone_r"][0] * ca
    tan_a = math.tan(alpha)
    d = tuple(u[i] + tan_a * e_up[i] for i in range(3))
    sec = 1.0 / ca
    ux, uy, uz = d[0] / sec, d[1] / sec, d[2] / sec
    rad0 = s0 * tan_a
    cx, cy = CURVE["centre"]
    p0 = (
        cx + s0 * u[0] + rad0 * e_up[0],
        cy + s0 * u[1] + rad0 * e_up[1],
        CURVE["z_top"] + s0 * u[2] + rad0 * e_up[2],
    )
    return (p[0] - p0[0]) * ux + (p[1] - p0[1]) * uy + (p[2] - p0[2]) * uz


def cone_assets():
    lines = []
    for i, th in enumerate(CURVE["theta_deg"]):
        for k, (s_lo, s_hi) in enumerate(frustum_ranges()):
            flat = " ".join("%.6f %.6f %.6f" % (m(x), m(y), m(z))
                            for x, y, z in frustum_vertices(th, s_lo, s_hi))
            lines.append('<mesh name="cone%d_%d" vertex="%s"/>' % (i, k, flat))
    return "\n    ".join(lines)


def visual_geom(mesh, colour):
    return ('<geom type="mesh" mesh="%s" contype="0" conaffinity="0" group="1" density="0" '
            'rgba="%s"/>' % (mesh, rgba(colour)))


def _body(name, pos, inertial, joint_xml, geoms):
    ix, iy, iz = inertial
    px, py, pz = pos
    return ('<body name="%s" pos="%g %g %g">\n      '
            '<inertial pos="%g %g %g" mass="0.02" diaginertia="1e-4 1e-4 1e-4"/>\n      '
            '%s\n      %s\n    </body>'
            % (name, px, py, pz, ix, iy, iz, joint_xml, "\n      ".join(geoms)))


def _slide(name, axis):
    return ('<joint name="%s" type="slide" axis="%g %g %g" armature="%g"/>'
            % (name, axis[0], axis[1], axis[2], SLIDE_ARMATURE))


def _hinge(name, pos, axis):
    return ('<joint name="%s" type="hinge" pos="%g %g %g" axis="%g %g %g" armature="%g"/>'
            % (name, pos[0], pos[1], pos[2], axis[0], axis[1], axis[2], HINGE_ARMATURE))


def straight_bodies(tag, rot, ox, oy, visuals):
    # Rails are only the plate standing above the belt. A face cut flush with
    # the carry plane is not a kerb, and the transfer depends on that.
    p = placer(rot, ox, oy)
    y0, _y1 = belt_y()
    ymid = lane_centre()
    a0, a1 = STR["drive_ax"], STR["nose_ax"]
    g = [visual_geom(n, c) for n, c in visuals]

    for i, yc in enumerate((STR["t"] / 2.0, STR["outer_width"] - STR["t"] / 2.0)):
        rt = STR["rail_top"][i]
        if rt <= belt_top:
            continue
        rx, ry = p(straight_len / 2.0, yc)
        rsx, rsy = ((straight_len / 2.0, STR["t"] / 2.0) if rot == 0
                    else (STR["t"] / 2.0, straight_len / 2.0))
        g.append('<geom name="%s_rail%d" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s"/>'
                 % (tag, i, m(rsx), m(rsy), m((rt - belt_top) / 2.0),
                    m(rx), m(ry), m((rt + belt_top) / 2.0), rgba(BRACKET_C, 0.0)))

    # The tab is the motor plate's ear, inboard of the wall. The full-length
    # rail does not cover that boss, and a part on the inside of the lane can.
    tcx = (TAB["x0"] + TAB["x1"]) / 2.0
    tcy = (TAB["y0"] + TAB["y1"]) / 2.0
    tcz = (TAB["z0"] + TAB["z1"]) / 2.0
    (px, py, pz), (hx, hy, hz) = place_box(
        rot, ox, oy, tcx, tcy, tcz,
        (TAB["x1"] - TAB["x0"]) / 2.0, (TAB["y1"] - TAB["y0"]) / 2.0,
        (TAB["z1"] - TAB["z0"]) / 2.0)
    g.append('<geom name="%s_tab" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s"/>'
             % (tag, m(hx), m(hy), m(hz), m(px), m(py), m(pz), rgba(BRACKET_C, 0.0)))

    bodies = ['<body name="%s" pos="0 0 0">\n      %s\n    </body>' % (tag, "\n      ".join(g))]

    # The flat run is one velocity. A slide that the step is not allowed to
    # integrate: the slab is only as long as the carry, and walking it would
    # pull the surface out from under the part.
    cx, cy = p((a0 + a1) / 2.0, ymid)
    half_len, half_wid = (a1 - a0) / 2.0, belt_width / 2.0
    sx, sy = (half_len, half_wid) if rot == 0 else (half_wid, half_len)
    slide = (1.0, 0.0, 0.0) if rot == 0 else (0.0, 1.0, 0.0)
    belt = ('<geom class="drive" name="%s_belt" type="box" size="%g %g %g" pos="0 0 0" rgba="%s"/>'
            % (tag, m(sx), m(sy), m(1.0), rgba(BELT_C, 0.0)))
    centre = (m(cx), m(cy), m(belt_top - 1.0))
    bodies.append(_body(tag + "_belt", centre, (0.0, 0.0, 0.0), _slide(tag + "_belt", slide), [belt]))

    # Same reason as the cone slices. A nose is a line contact, and one
    # reported point on a 50 mm cylinder is not a support. The hinge is the
    # nose axis; the outer fibre is the belt wrapped over it.
    euler = "90 0 0" if rot == 0 else "0 90 0"
    # Positive qvel has to send the crown along the module's travel. On s2 the
    # nose lies along X, and the right-hand sense of +X sends the crown backward.
    axis = (0.0, 1.0, 0.0) if rot == 0 else (-1.0, 0.0, 0.0)
    n_ax = int(math.ceil(belt_width / 5.0 - 1e-9))
    seg = belt_width / n_ax
    for name, ax in (("infeed", a0), ("driven", a1)):
        slices = []
        for k in range(n_ax):
            y_c = y0 + (k + 0.5) * seg
            gx, gy = p(ax, y_c)
            slices.append(
                '<geom class="drive" name="%s_%s%d" type="cylinder" size="%g %g" pos="%g %g %g" '
                'euler="%s" rgba="%s"/>'
                % (tag, name, k, m(nose_r), m(seg / 2.0),
                   m(gx), m(gy), m(nose_z), euler, rgba(BELT_C, 0.0)))
        jx, jy = p(ax, ymid)
        joint = tag + "_" + name
        bodies.append(_body(
            joint, (0.0, 0.0, 0.0), (m(jx), m(jy), m(nose_z)),
            _hinge(joint, (m(jx), m(jy), m(nose_z)), axis), slices))
    return bodies


def curve_bodies(visuals):
    # The walls are not driven. The rollers are, one hinge each, so the crown
    # can follow the turn instead of a single speed for the whole curve.
    g = [visual_geom(n, c) for n, c in visuals]

    # Only the wall above the carry plane can meet the part. Boxes tile the
    # arc at 5°, tangent to it, and stop on the entry and exit faces so a box
    # does not stand in the transfer.
    cx, cy = CURVE["centre"]
    zc = (CURVE["wall_top"] + CURVE["z_top"]) / 2.0
    hz = (CURVE["wall_top"] - CURVE["z_top"]) / 2.0
    half = math.radians(2.5)
    nbox = int(round(90.0 / 5.0))
    for which, (r0, r1) in (("in", CURVE["inner_wall"]), ("out", CURVE["outer_wall"])):
        rmid = (r0 + r1) / 2.0
        hrad = (r1 - r0) / 2.0
        htang = rmid * math.tan(half)
        for i in range(nbox):
            th = math.radians(-90.0 + 2.5 + 5.0 * i)
            px = cx + rmid * math.cos(th)
            py = cy + rmid * math.sin(th)
            tx, ty = -math.sin(th), math.cos(th)
            rx, ry = math.cos(th), math.sin(th)
            g.append(
                '<geom name="c_%s%d" type="box" size="%g %g %g" pos="%g %g %g" '
                'xyaxes="%g %g 0 %g %g 0" rgba="%s"/>'
                % ("in" if which == "in" else "out", i,
                   m(htang), m(hrad), m(hz), m(px), m(py), m(zc),
                   tx, ty, rx, ry, rgba(BRACKET_C, 0.0)))
    bodies = ['<body name="c" pos="0 0 0">\n      %s\n    </body>' % ("\n      ".join(g))]

    # Hinge through the apex, along the roller. The crown is a point on this
    # body; which sign makes it follow the left turn is checked, not assumed,
    # because the axis direction is a convention.
    apex = (m(cx), m(cy), m(CURVE["z_top"]))
    for i, theta in enumerate(CURVE["theta_deg"]):
        u, _, _, _, _ = cone_frame(theta)
        slices = []
        for k in range(n_frusta()):
            slices.append(
                '<geom class="drive" name="c_roll%d_%d" type="mesh" mesh="cone%d_%d" rgba="%s"/>'
                % (i, k, i, k, rgba(ROLLER_C, 0.0)))
        name = "cone%d" % i
        bodies.append(_body(name, (0.0, 0.0, 0.0), apex, _hinge(name, apex, u), slices))
    return bodies


def joiner_body():
    g = [visual_geom(n, c) for n, c in VISUALS if n.startswith("jn_")]
    return '<body name="joiners" pos="0 0 0">\n      %s\n    </body>' % ("\n      ".join(g))


def build_xml(visuals):
    # Headless runs never load the meshes. Two processes welding the same STL
    # into renders/sim/ corrupted a sweep, and the meshes are not what is measured.
    part_z = belt_top + PART[2] / 2.0 + 0.5
    if visuals:
        s1v = [(n, c) for n, c in VISUALS if n.startswith("cs_")]
        s2v = [(n, c) for n, c in VISUALS if n.startswith("s2_")]
        cv = [(n, c) for n, c in VISUALS if n.startswith("cv_")]
        meshes = mesh_assets()
        extra = [joiner_body()]
    else:
        s1v, s2v, cv = [], [], []
        meshes = ""
        extra = []
    bodies = []
    bodies.extend(straight_bodies("s1", S1["rot"], S1["offset"][0], S1["offset"][1], s1v))
    bodies.extend(curve_bodies(cv))
    bodies.extend(straight_bodies("s2", S2["rot"], S2["offset"][0], S2["offset"][1], s2v))
    bodies.extend(extra)
    # multiccd: a box on a cylinder or a cone is a line contact. One reported
    # point can sit anywhere along that line, and the torque about it is then
    # noise. Every point on the line has to count.
    # Rails stay at a low friction. A guide that grips harder than the belt
    # would decide the heading, which is the failure this drive is here to leave behind.
    return """
<mujoco model="mini_conveyor">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" noslip_iterations="{noslip}">
    <flag multiccd="enable"/>
  </option>
  <default>
    <geom contype="{st}" conaffinity="{pt}" condim="3" friction="0.04 0.005 0.0001"/>
    <default class="drive">
      <geom contype="{dr}" conaffinity="{pt}" condim="3" density="0" friction="0.9 0.005 0.0001"/>
    </default>
  </default>
  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.5 0.5 0.5" specular="0.1 0.1 0.1"/>
    <rgba haze="0.95 0.94 0.92 1"/>
    <global offwidth="1600" offheight="1000"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.90 0.90 0.88" rgb2="0.98 0.98 0.96"
             width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.86 0.85 0.82"
             rgb2="0.92 0.91 0.88" width="256" height="256"/>
    <material name="gridmat" texture="grid" texrepeat="12 12" reflectance="0.05"/>
    {meshes}
    {cones}
  </asset>

  <worldbody>
    <light pos="0.15 -0.05 0.45" dir="-0.3 0.35 -1" directional="true" diffuse="0.55 0.55 0.55"/>
    <geom name="floor" type="plane" size="1 1 0.05" material="gridmat" friction="0.8 0.01 0.001"/>

    {bodies}

    <body name="part" pos="{px} {py} {pz}">
      <freejoint name="partfree"/>
      <!-- The pair takes the larger sliding friction. The part is 0 so the
           drive geom's value is the pair exactly, including a commanded 0.
           condim 3: the slices already make a torsional moment. -->
      <geom name="partgeom" type="box" size="{hx} {hy} {hz}" mass="{pm}"
            contype="{pt}" conaffinity="{pa}" condim="3"
            rgba="{pc}" friction="0.0 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>
""".format(
        meshes=meshes,
        cones=cone_assets(),
        bodies="\n\n    ".join(bodies),
        dt=DT,
        noslip=int(NOSLIP),
        st=CON_STATIC, dr=CON_DRIVE, pt=CON_PART, pa=CON_STATIC | CON_DRIVE,
        px=m(40.0), py=m(lane_centre()), pz=m(part_z),
        hx=m(PART[0] / 2.0), hy=m(PART[1] / 2.0), hz=m(PART[2] / 2.0),
        pm=PART_MASS, pc=rgba(PART_C))


def gid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


_MODEL = None
_MODEL_VIS = None


def compiled_model(visuals):
    # Friction and the timestep are written onto the compiled model per run.
    # A second compile is only for the meshes, which a measurement does not use.
    global _MODEL, _MODEL_VIS
    if _MODEL is None or _MODEL_VIS != visuals:
        _MODEL = mujoco.MjModel.from_xml_string(build_xml(visuals))
        _MODEL_VIS = visuals
    _MODEL.opt.timestep = DT
    _MODEL.opt.noslip_iterations = int(NOSLIP)
    return _MODEL


def _neutral_r_m():
    # The flat run moves at the belt's neutral axis. Around the nose the outer
    # fibre, which is what the part can touch, is faster.
    return (G["nose_dia"] / 2.0 + belt_thickness / 2.0) * MM


def drive_records(straight_speed, curve_speed):
    recs = []
    outer = nose_r * MM
    omega_nose = straight_speed / _neutral_r_m()
    for tag, mod in (("s1", S1), ("s2", S2)):
        rot = mod["rot"]
        p = placer(rot, mod["offset"][0], mod["offset"][1])
        a0, a1 = STR["drive_ax"], STR["nose_ax"]
        hat = np.array([1.0, 0.0, 0.0]) if rot == 0 else np.array([0.0, 1.0, 0.0])
        cx, cy = p((a0 + a1) / 2.0, lane_centre())
        recs.append({
            "name": tag + "_belt",
            "kind": "belt",
            "point": np.array([m(cx), m(cy), m(belt_top)]),
            "expect": hat * straight_speed,
            "vel": straight_speed,
            "mag": straight_speed,
        })
        for name, ax in (("infeed", a0), ("driven", a1)):
            jx, jy = p(ax, lane_centre())
            recs.append({
                "name": "%s_%s" % (tag, name),
                "kind": "nose",
                "point": np.array([m(jx), m(jy), m(nose_z) + outer]),
                "expect": hat * (omega_nose * outer),
                "vel": omega_nose,
                "mag": omega_nose,
            })
    # Ω > 0 is the left turn. |ω| = Ω / sin α because a crown point at plan
    # radius r is r·sin α off the roller axis. The sign is not set here.
    omega_field = curve_speed / (CURVE["r_c"] * MM)
    alpha = math.radians(CURVE["alpha_deg"])
    mag = abs(omega_field) / math.sin(alpha) if math.sin(alpha) else 0.0
    cx, cy = CURVE["centre"]
    for i, theta in enumerate(CURVE["theta_deg"]):
        th = math.radians(theta)
        px = cx + CURVE["r_c"] * math.cos(th)
        py = cy + CURVE["r_c"] * math.sin(th)
        point = np.array([m(px), m(py), m(CURVE["z_top"])])
        expect = omega_field * np.array([-(point[1] - m(cy)), point[0] - m(cx), 0.0])
        recs.append({
            "name": "cone%d" % i,
            "kind": "cone",
            "point": point,
            "expect": expect,
            "vel": mag,
            "mag": mag,
        })
    return recs


def bind_drives(model, recs):
    out = []
    for rec in recs:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, rec["name"])
        if jid < 0:
            raise RuntimeError("missing drive joint %s" % rec["name"])
        rec = dict(rec)
        rec["qadr"] = int(model.jnt_qposadr[jid])
        rec["vadr"] = int(model.jnt_dofadr[jid])
        rec["bid"] = int(model.jnt_bodyid[jid])
        rec["point"] = np.ascontiguousarray(rec["point"], dtype=np.float64)
        rec["expect"] = np.ascontiguousarray(rec["expect"], dtype=np.float64)
        out.append(rec)
    return out


def kick(data, drives, moving=True):
    # qpos stays put. The slab is only the length of the flat run, and the
    # slices are faceted: letting either integrate would walk the belt away
    # and roll the crown points the part is standing on. qvel is what the
    # contact solver reads as the surface speed.
    for d in drives:
        data.qpos[d["qadr"]] = 0.0
        data.qvel[d["vadr"]] = d["vel"] if moving else 0.0


def _prepare_jac(model, data):
    # mj_jac reads cdof, which the position pass does not fill.
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_comVel(model, data)


def _jac_vel(model, data, drive):
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, None, drive["point"], int(drive["bid"]))
    return jacp @ np.asarray(data.qvel)


def _rel_err(got, expect):
    got = np.asarray(got, dtype=np.float64)
    expect = np.asarray(expect, dtype=np.float64)
    scale = float(np.linalg.norm(expect))
    if scale < 1e-9:
        return 0.0 if float(np.linalg.norm(got)) < 1e-6 else 1.0
    return float(np.linalg.norm(got - expect)) / scale


def _apply_cone_sign(drives, sign):
    for d in drives:
        if d["kind"] == "cone":
            d["vel"] = sign * d["mag"]


def _cone_sign(model, drives):
    # The crown has to follow Ω ẑ × (p − C). Which way the hinge turns to do
    # that is the axis convention, so both signs are measured and the one that
    # matches is kept.
    data = mujoco.MjData(model)
    errs = {}
    for sign in (1.0, -1.0):
        _apply_cone_sign(drives, sign)
        kick(data, drives, True)
        _prepare_jac(model, data)
        worst = 0.0
        for d in drives:
            if d["kind"] != "cone":
                continue
            worst = max(worst, _rel_err(_jac_vel(model, data, d), d["expect"]))
        errs[sign] = worst
    sign = 1.0 if errs[1.0] <= errs[-1.0] else -1.0
    return sign, errs[1.0], errs[-1.0]


def _balance(model, drives, part_bid):
    # Gravity must not turn a roller. The mass sits on the hinge, and this is
    # the check that it does: one step, part parked clear, commanded speed zero.
    data = mujoco.MjData(model)
    place_part(model, data, part_bid, 0.0, at=(40.0, lane_centre(), 200.0))
    kick(data, drives, moving=False)
    mujoco.mj_step(model, data)
    return max(abs(float(data.qvel[d["vadr"]])) for d in drives)


def _hold(model, drives, part_bid):
    # One step under the part's weight. The armature is there so this change
    # stays negligible beside the commanded speed.
    data = mujoco.MjData(model)
    poses = [(40.0, lane_centre(), belt_top + PART[2] / 2.0 + 0.5, 0.0)]
    th = math.radians(-45.0)
    cx, cy = CURVE["centre"]
    x = cx + CURVE["r_c"] * math.cos(th)
    y = cy + CURVE["r_c"] * math.sin(th)
    poses.append((x, y, CURVE["z_top"] + PART[2] / 2.0 + 0.5, path_tangent_deg(x, y)))
    worst = 0.0
    n = max(1, int(0.05 / model.opt.timestep))
    for x_mm, y_mm, z_mm, yaw in poses:
        place_part(model, data, part_bid, 0.0, at=(x_mm, y_mm, z_mm), yaw_deg=yaw)
        for _ in range(n):
            kick(data, drives, True)
            mujoco.mj_step(model, data)
        kick(data, drives, True)
        commanded = [d["vel"] for d in drives]
        mujoco.mj_step(model, data)
        for d, v0 in zip(drives, commanded):
            if abs(v0) < 1e-12:
                continue
            worst = max(worst, abs(float(data.qvel[d["vadr"]]) - v0) / abs(v0))
    return worst


def prove_drive(model, drives, part_bid, announce):
    sign, err_pos, err_neg = _cone_sign(model, drives)
    _apply_cone_sign(drives, sign)
    chosen = err_neg if sign < 0 else err_pos
    other = err_pos if sign < 0 else err_neg
    lines = ["cone hinge sign %+d  crown err %.4f%%  other %.4f%%"
             % (int(sign), 100.0 * chosen, 100.0 * other)]
    bad = chosen > 0.01
    data = mujoco.MjData(model)
    kick(data, drives, True)
    _prepare_jac(model, data)
    for d in drives:
        got = _jac_vel(model, data, d)
        err = _rel_err(got, d["expect"])
        lines.append(
            "drive %-12s  v %8.4f %8.4f %8.4f  expected %8.4f %8.4f %8.4f  err %7.3f%%"
            % (d["name"], got[0], got[1], got[2],
               d["expect"][0], d["expect"][1], d["expect"][2], 100.0 * err))
        if err > 0.01:
            bad = True
    bal = _balance(model, drives, part_bid)
    lines.append("drive balance  max |qvel| after one unloaded step %.3e" % bal)
    if bal > 1e-5:
        bad = True
    hold = _hold(model, drives, part_bid)
    lines.append("drive hold  max relative qvel change under the part %.3e" % hold)
    if hold > 0.01:
        bad = True
    if announce or bad:
        for line in lines:
            print(line, flush=True)
    if bad:
        raise RuntimeError("drive check failed")


def _set_friction(model, mu_belt, mu_curve):
    if mu_belt < 0.0 or mu_curve < 0.0:
        raise SystemExit(
            "usage: --mu and --mu-curve must be >= 0 (got %.3f / %.3f)"
            % (mu_belt, mu_curve))
    cone_name = None
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        mu = None
        if name == "s1_belt" or name.startswith("s1_infeed") or name.startswith("s1_driven"):
            mu = mu_belt
        elif name == "s2_belt" or name.startswith("s2_infeed") or name.startswith("s2_driven"):
            mu = mu_belt
        elif name.startswith("c_roll"):
            mu = mu_curve
            if cone_name is None:
                cone_name = name
        if mu is not None:
            model.geom_friction[i, 0] = mu
    return cone_name


def _pair_mu(model, drive_name):
    # MuJoCo uses the larger sliding coefficient of the two geoms.
    part = float(model.geom_friction[gid(model, "partgeom"), 0])
    drive = float(model.geom_friction[gid(model, drive_name), 0])
    return max(part, drive)


def _log_mu(model, cone_name):
    belt = _pair_mu(model, "s1_belt")
    cone = _pair_mu(model, cone_name)
    print("effective mu  s1_belt %g  %s %g" % (belt, cone_name, cone), flush=True)


def _rails(model):
    rails = set()
    for tag in ("s1", "s2"):
        for name in ("rail0", "rail1", "tab"):
            rails.add(gid(model, "%s_%s" % (tag, name)))
    nbox = int(round(90.0 / 5.0))
    for which in ("in", "out"):
        for i in range(nbox):
            rails.add(gid(model, "c_%s%d" % (which, i)))
    rails.discard(-1)
    return rails


def setup(straight_speed, curve_speed, mu_belt, mu_curve, visuals=False, announce=False):
    model = compiled_model(visuals)
    cone_name = _set_friction(model, mu_belt, mu_curve)
    if announce:
        _log_mu(model, cone_name)
    data = mujoco.MjData(model)
    part_gid = gid(model, "partgeom")
    part_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "part")
    drives = bind_drives(model, drive_records(straight_speed, curve_speed))
    prove_drive(model, drives, part_bid, announce)
    return model, data, drives, _rails(model), part_gid, part_bid


def pose(data, part_bid):
    # xmat is row-major; columns are the body axes in the world.
    R = np.array(data.xmat[part_bid]).reshape(3, 3)
    body_x = R[:, 0]
    body_z = R[:, 2]
    yaw = math.degrees(math.atan2(body_x[1], body_x[0]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(body_z[2])))))
    p = data.xpos[part_bid]
    return p[0] / MM, p[1] / MM, p[2] / MM, yaw, tilt


def offset_mm(x, y):
    # Signed toward the inside of the left turn, so one number continues
    # across all three modules.
    entry_x = CURVE["entry_face_x"]
    s2_in = S2["offset"][1]
    if x < entry_x:
        return y - lane_centre()
    if y < s2_in:
        r = math.hypot(x - CURVE["centre"][0], y - CURVE["centre"][1])
        return CURVE["r_c"] - r
    return (CURVE["centre"][0] + CURVE["r_c"]) - x


def path_length_m():
    # Centreline, from the start pose to the exit station. The time limit is
    # this length, not a fixed number of seconds, so a slow belt is not failed
    # for still being on the curve.
    entry = CURVE["entry_face_x"]
    s2_in = S2["offset"][1]
    along = (entry - 40.0) + (math.pi / 2.0) * CURVE["r_c"] + (s2_in + 40.0 - CURVE["exit_face_y"])
    return along * MM


def time_limit(speed):
    if speed <= 0.0:
        raise SystemExit(
            "usage: a stopped belt needs --seconds N; without it there is no travel time")
    return path_length_m() / speed * 1.5 + 1.0


def place_part(model, data, part_bid, offset_mm_, at=None, yaw_deg=0.0):
    qadr = model.body_jntadr[part_bid]
    qpos = model.jnt_qposadr[qadr]
    if at is None:
        z = belt_top + PART[2] / 2.0 + 0.5
        at = (40.0, lane_centre() + offset_mm_, z)
    half = math.radians(yaw_deg) * 0.5
    data.qpos[qpos:qpos + 7] = [
        m(at[0]), m(at[1]), m(at[2]), math.cos(half), 0.0, 0.0, math.sin(half)]
    dof = model.jnt_dofadr[qadr]
    data.qvel[dof:dof + 6] = 0
    data.xfrc_applied[part_bid][:] = 0
    mujoco.mj_forward(model, data)


def mix_angle(a, b, w):
    d = (b - a + 180.0) % 360.0 - 180.0
    return a + w * d


def crossed(samples, index, threshold):
    # First time the sample coordinate reaches the station, between the two
    # records that bracket it.
    for i in range(1, len(samples)):
        a, b = samples[i - 1], samples[i]
        if a[index] < threshold <= b[index]:
            span = b[index] - a[index]
            w = 0.0 if span == 0 else (threshold - a[index]) / span
            t = a[0] + w * (b[0] - a[0])
            x = a[1] + w * (b[1] - a[1])
            y = a[2] + w * (b[2] - a[2])
            yaw = mix_angle(a[4], b[4], w)
            return t, x, y, yaw
    return None


def path_tangent_deg(x, y):
    # Heading of the centreline at this COM. On the curve it is the tangent
    # of the left turn; the straights are the two axes that tangent joins.
    entry = CURVE["entry_face_x"]
    exit_y = CURVE["exit_face_y"]
    if x < entry:
        return 0.0
    if y > exit_y:
        return 90.0
    th = math.atan2(y - CURVE["centre"][1], x - CURVE["centre"][0])
    return math.degrees(math.atan2(math.cos(th), -math.sin(th)))


def yaw_error_deg(yaw, tangent):
    return (yaw - tangent + 180.0) % 360.0 - 180.0


def polar_deg(x, y):
    return math.degrees(math.atan2(y - CURVE["centre"][1], x - CURVE["centre"][0]))


def place_of(x, y):
    # Where a dip or a tilt belongs. The transfers are the first and last
    # 20 mm of the arc, plus the frame gap on each side of it.
    entry = CURVE["entry_face_x"]
    exit_y = CURVE["exit_face_y"]
    s2_in = S2["offset"][1]
    arc = math.degrees(20.0 / CURVE["r_c"])
    th = polar_deg(x, y)
    if x < entry - PART[0] / 2.0:
        return "on s1"
    if y > s2_in + 20.0:
        return "on s2"
    if x < entry or th < -90.0 + arc:
        return "entry transfer"
    if th > -arc or y > exit_y:
        return "exit transfer"
    return "on the cones"


def at_polar(samples, target):
    prev = None
    for s in samples:
        th = polar_deg(s[1], s[2])
        if prev is not None and prev[0] < target <= th:
            span = th - prev[0]
            w = 0.0 if span == 0 else (target - prev[0]) / span
            a, b = prev[1], s
            yaw = mix_angle(a[4], b[4], w)
            x = a[1] + w * (b[1] - a[1])
            y = a[2] + w * (b[2] - a[2])
            return yaw_error_deg(yaw, path_tangent_deg(x, y))
        prev = (th, s)
    return None


def station_errors(samples):
    # Yaw minus the path tangent. Positive means the part is ahead of the turn.
    entry_face = CURVE["entry_face_x"]
    exit_face = CURVE["exit_face_y"]
    exit_y = S2["offset"][1] + 40.0
    out = {}
    hit = crossed(samples, 1, entry_face)
    if hit:
        out["entry_face"] = yaw_error_deg(hit[3], path_tangent_deg(hit[1], hit[2]))
    mid = at_polar(samples, -45.0)
    if mid is not None:
        out["mid_curve"] = mid
    hit = crossed(samples, 2, exit_face)
    if hit:
        out["exit_face"] = yaw_error_deg(hit[3], path_tangent_deg(hit[1], hit[2]))
    hit = crossed(samples, 2, exit_y)
    if hit:
        out["exit_station"] = yaw_error_deg(hit[3], path_tangent_deg(hit[1], hit[2]))
    return out


def _x_half(yaw_deg):
    # Square in plan, so a little yaw still pushes a corner past the apex.
    a = math.radians(yaw_deg)
    return PART[0] / 2.0 * abs(math.cos(a)) + PART[1] / 2.0 * abs(math.sin(a))


def on_flat_carry(samples):
    # Position, not a clock. At 0.155 m/s the old 0.3–0.6 s window already
    # includes the discharge nose, and that droop lowers the reference.
    x_in = G["straight"]["drive_ax"]
    x_out = G["straight"]["nose_ax"] - 2.0
    kept = []
    for samp in samples:
        ext = _x_half(samp[4])
        if samp[1] - ext >= x_in and samp[1] + ext <= x_out:
            kept.append(samp)
    return kept


def flat_rest_z(samples):
    kept = on_flat_carry(samples)
    if not kept:
        return None, None
    return sum(s[3] for s in kept) / len(kept), kept[-1][0]


def evaluate(samples, contacts, speed, limit):
    entry_x = CURVE["entry_face_x"] - 30.0
    exit_y = S2["offset"][1] + 40.0
    entry = crossed(samples, 1, entry_x)
    exit_ = crossed(samples, 2, exit_y)
    rest, t_leave = flat_rest_z(samples)
    after = [s for s in samples if rest is not None and s[0] > t_leave]
    if exit_ is not None:
        after = [s for s in after if s[0] <= exit_[0] + 1e-9]
    dip_at = min(after, key=lambda s: s[3]) if after else None
    tilt_at = max(samples, key=lambda s: s[5]) if samples else None
    dip = (rest - dip_at[3]) if rest is not None and dip_at is not None else None
    tilt = tilt_at[5] if tilt_at is not None else None
    reached = exit_ is not None
    entry_off = offset_mm(entry[1], entry[2]) if entry else None
    exit_off = offset_mm(exit_[1], exit_[2]) if exit_ else None
    exit_yaw = exit_[3] if exit_ else None
    reasons = []
    if not reached:
        reasons.append("exit not reached within %.2f s" % limit)
    if rest is None:
        reasons.append("dip unmeasurable")
    elif dip is None or dip > 1.0:
        reasons.append("dip %s mm" % ("?" if dip is None else "%.2f" % dip))
    if tilt is None or tilt > 5.0:
        reasons.append("tilt %s deg" % ("?" if tilt is None else "%.2f" % tilt))
    if contacts:
        reasons.append("rail contact")
    if entry_off is None or exit_off is None or abs(exit_off - entry_off) > 3.0:
        reasons.append("offset change")
    if exit_yaw is None or abs(exit_yaw - 90.0) > 6.0:
        reasons.append("yaw")
    return {
        "entry_offset_mm": entry_off,
        "exit_offset_mm": exit_off,
        "exit_yaw_deg": exit_yaw,
        "dip_mm": dip,
        "max_tilt_deg": tilt,
        "rail_contacts": contacts,
        "reached": reached,
        "t_exit": None if exit_ is None else exit_[0],
        "t_limit": limit,
        "pass": not reasons,
        "reasons": reasons,
        "yaw_error_deg": station_errors(samples),
        "dip_where": None if dip_at is None else place_of(dip_at[1], dip_at[2]),
        "tilt_where": None if tilt_at is None else place_of(tilt_at[1], tilt_at[2]),
    }


def cone_support(model, data, part_gid, part_bid, rest_z):
    # Contacts with the sliced crowns, grouped by roller, measured along the
    # top line. This is the check that the split is a support and not two points.
    groups = {}
    for i in range(data.ncon):
        c = data.contact[i]
        if part_gid not in (c.geom1, c.geom2):
            continue
        other = int(c.geom2 if c.geom1 == part_gid else c.geom1)
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)
        if not name or not name.startswith("c_roll"):
            continue
        roller = int(name.split("_")[1][4:])
        p = (c.pos[0] / MM, c.pos[1] / MM, c.pos[2] / MM)
        groups.setdefault(roller, []).append(along_crown(CURVE["theta_deg"][roller], p))
    x, y, z, _, tilt = pose(data, part_bid)
    rollers = []
    for r, xs in sorted(groups.items()):
        rollers.append({"roller": r, "n": len(xs), "span_mm": max(xs) - min(xs)})
    return {
        "n_contacts": sum(r["n"] for r in rollers),
        "rollers": rollers,
        "tilt_deg": tilt,
        "dz_mm": None if rest_z is None else z - rest_z,
        "x": x, "y": y, "z": z,
    }


def simulate(straight_speed, curve_speed, mu_belt, mu_curve, offset, frames=0, limit=None,
             prove=False, visuals=False, announce_drive=False):
    # Headless on purpose. Nothing here puts the part back on the belt.
    model, data, drives, rails, part_gid, part_bid = setup(
        straight_speed, curve_speed, mu_belt, mu_curve,
        visuals=visuals or frames > 0, announce=announce_drive)
    place_part(model, data, part_bid, offset)
    if limit is None:
        limit = time_limit(straight_speed)
    dt = model.opt.timestep
    steps = int(limit / dt)
    sample_every = max(1, int(round(0.01 / dt)))
    exit_y = S2["offset"][1] + 40.0
    samples = []
    contacts = []
    seen = set()
    shots = []
    proof = None
    renderer = None
    if frames > 0:
        renderer = mujoco.Renderer(model, 900, 1400)
        cam = mujoco.MjvCamera()
        # The line is an L from the origin out to s2's discharge. The camera
        # sits off the outside corner so both belts and the curve read at once.
        cam.lookat[:] = (m(120.0), m(90.0), m(20.0))
        cam.distance = 0.72
        cam.azimuth = 145
        cam.elevation = -32
        # Spread across the travel, not the padded limit. The limit runs past
        # the exit, and shots parked out there never get taken. A stopped belt
        # has no travel, so the shots span the requested duration instead.
        if straight_speed > 0.0:
            travel_steps = max(1, int(path_length_m() / straight_speed / dt))
        else:
            travel_steps = max(1, steps)
        shot_at = set(int(round(travel_steps * k / max(1, frames - 1))) for k in range(frames))
    for i in range(steps):
        kick(data, drives)
        mujoco.mj_step(model, data)
        if i % sample_every == 0:
            x, y, z, yaw, tilt = pose(data, part_bid)
            samples.append((data.time, x, y, z, yaw, tilt))
        for i_c in range(data.ncon):
            c = data.contact[i_c]
            if part_gid not in (c.geom1, c.geom2):
                continue
            other = int(c.geom2 if c.geom1 == part_gid else c.geom1)
            if other not in rails:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)
            key = (round(data.time, 2), name)
            if key not in seen:
                seen.add(key)
                contacts.append({"t": round(data.time, 4), "geom": name})
        if renderer is not None and i in shot_at:
            renderer.update_scene(data, cam)
            shots.append(renderer.render().copy())
        x, y, z, yaw, tilt = pose(data, part_bid)
        if prove and proof is None and x >= CURVE["entry_face_x"] and polar_deg(x, y) >= -45.0:
            rest, _t_leave = flat_rest_z(samples)
            proof = cone_support(model, data, part_gid, part_bid, rest)
        if y >= exit_y and x > CURVE["entry_face_x"]:
            if not samples or abs(samples[-1][0] - data.time) > 1e-9:
                samples.append((data.time, x, y, z, yaw, tilt))
            break
    metrics = evaluate(samples, contacts, straight_speed, limit)
    return samples, metrics, shots, proof


def fmt(v, spec):
    return "?" if v is None else spec % v


def print_metrics(metrics):
    print("entry offset %s mm" % fmt(metrics["entry_offset_mm"], "%.2f"))
    print("exit  offset %s mm   yaw %s deg   t %s s"
          % (fmt(metrics["exit_offset_mm"], "%.2f"),
             fmt(metrics["exit_yaw_deg"], "%.2f"),
             fmt(metrics["t_exit"], "%.2f")))
    print("dip %s mm   max tilt %s deg   rail contacts %d"
          % (fmt(metrics["dip_mm"], "%.2f"),
             fmt(metrics["max_tilt_deg"], "%.2f"),
             len(metrics["rail_contacts"])))
    if metrics["rail_contacts"]:
        for c in metrics["rail_contacts"][:12]:
            print("  rail t=%.3f %s" % (c["t"], c["geom"]))
    ye = metrics.get("yaw_error_deg") or {}
    if ye:
        print("yaw error deg  entry %s  mid %s  exit-face %s  station %s"
              % (fmt(ye.get("entry_face"), "%.2f"), fmt(ye.get("mid_curve"), "%.2f"),
                 fmt(ye.get("exit_face"), "%.2f"), fmt(ye.get("exit_station"), "%.2f")))
    if metrics.get("dip_where") or metrics.get("tilt_where"):
        print("dip at %s   tilt at %s" % (metrics.get("dip_where"), metrics.get("tilt_where")))
    print("PASS" if metrics["pass"] else "FAIL: " + ", ".join(metrics["reasons"]))


def print_proof(proof):
    if not proof:
        print("contact proof: the part never sat fully on the cones")
        return
    print("contact proof  n=%d  tilt %.3f deg  dz %s mm"
          % (proof["n_contacts"], proof["tilt_deg"], fmt(proof["dz_mm"], "%.3f")))
    for r in proof["rollers"]:
        print("  roller %d  contacts %d  span %.1f mm" % (r["roller"], r["n"], r["span_mm"]))


def write_png(path, rgbimg):
    import zlib
    import struct
    h, w, _ = rgbimg.shape
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw.extend(rgbimg[y].tobytes())

    def ch(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    blob = (b"\x89PNG\r\n\x1a\n"
            + ch(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + ch(b"IDAT", zlib.compress(bytes(raw), 6))
            + ch(b"IEND", b""))
    open(path, "wb").write(blob)


def print_spans():
    for which in ("entry", "exit"):
        s = G["spans"][which]
        print("spans %s  inner %.1f  centre %.1f  outer %.1f mm"
              % (which, s["inner"], s["centre"], s["outer"]))


def _settle(model, data, drives, part_bid):
    n = int(1.0 / model.opt.timestep)
    for _ in range(n):
        kick(data, drives, moving=False)
        mujoco.mj_step(model, data)
    return pose(data, part_bid)


def resting_contacts(model, data, drives, part_gid, part_bid):
    # Drives held. The s1 height is the reference, so a contact that sinks on
    # both modules does not show up as a step between them.
    place_part(model, data, part_bid, 0.0)
    _, _, z_s1, _, tilt_s1 = _settle(model, data, drives, part_bid)
    th = math.radians(-45.0)
    cx, cy = CURVE["centre"]
    x = cx + CURVE["r_c"] * math.cos(th)
    y = cy + CURVE["r_c"] * math.sin(th)
    z = CURVE["z_top"] + PART[2] / 2.0 + 0.5
    place_part(model, data, part_bid, 0.0, at=(x, y, z), yaw_deg=path_tangent_deg(x, y))
    _settle(model, data, drives, part_bid)
    proof = cone_support(model, data, part_gid, part_bid, z_s1)
    proof["z_s1"] = z_s1
    proof["tilt_s1"] = tilt_s1
    return proof


def run_nominal():
    # --seconds replaces the acceptance limit. A cap that misses the exit fails;
    # it is not a different, easier test. A stopped belt has no path time, so
    # the duration has to be given.
    if "--seconds" in sys.argv:
        limit = _argv("--seconds", 1.0)
    elif STRAIGHT_SPEED <= 0.0 or CURVE_SPEED <= 0.0:
        print("usage: a stopped belt needs --seconds N; without it there is no travel time")
        return False
    else:
        limit = time_limit(STRAIGHT_SPEED)
    frames = _argv("--frames", 6)
    print_spans()
    print("nominal  speed %.3f  curve %.3f  mu %.2f / %.2f  offset %.1f mm  "
          "dt %.4g s  noslip %d  limit %.2f s"
          % (STRAIGHT_SPEED, CURVE_SPEED, MU_BELT, MU_CURVE, ENTRY_OFFSET,
             DT, int(NOSLIP), limit))
    t0 = time.perf_counter()
    model, data, drives, _rails, part_gid, part_bid = setup(
        STRAIGHT_SPEED, CURVE_SPEED, MU_BELT, MU_CURVE, visuals=False, announce=True)
    rest = resting_contacts(model, data, drives, part_gid, part_bid)
    print("at rest  x %.1f  y %.1f  z %.3f mm  s1 z %.3f mm  s1 tilt %.3f deg"
          % (rest["x"], rest["y"], rest["z"], rest["z_s1"], rest["tilt_s1"]))
    print_proof(rest)
    t_rest = time.perf_counter()
    samples, metrics, shots, proof = simulate(
        STRAIGHT_SPEED, CURVE_SPEED, MU_BELT, MU_CURVE, ENTRY_OFFSET,
        frames=frames, limit=limit, prove=True, visuals=frames > 0, announce_drive=False)
    t_run = time.perf_counter()
    print("in the run")
    print_proof(proof)
    for s in samples:
        print("  t=%5.2f  x=%7.1f  y=%7.1f  z=%6.2f  yaw=%6.1f  tilt=%5.2f  off=%6.2f"
              % (s[0], s[1], s[2], s[3], s[4], s[5], offset_mm(s[1], s[2])))
    if frames > 0:
        for name in os.listdir(OUT):
            if name.startswith("frame") and name.endswith(".png"):
                os.remove(os.path.join(OUT, name))
        for i, img in enumerate(shots):
            write_png(os.path.join(OUT, "frame%02d.png" % i), img)
    print_metrics(metrics)
    print("wall rest %.2f s  run %.2f s" % (t_rest - t0, t_run - t_rest))
    return metrics["pass"]


def rnd(v, n=4):
    if v is None:
        return None
    return round(float(v), n)


def _row(offset, speed, mu_b, mu_c, metrics):
    return {
        "speed": speed,
        "mu": mu_b,
        "mu_curve": mu_c,
        "offset_mm": offset,
        "entry_offset_mm": rnd(metrics["entry_offset_mm"], 3),
        "exit_offset_mm": rnd(metrics["exit_offset_mm"], 3),
        "exit_yaw_deg": rnd(metrics["exit_yaw_deg"], 3),
        "dip_mm": rnd(metrics["dip_mm"], 3),
        "max_tilt_deg": rnd(metrics["max_tilt_deg"], 3),
        "rail_contacts": metrics["rail_contacts"],
        "reached": metrics["reached"],
        "t_exit": rnd(metrics["t_exit"], 3),
        "t_limit": rnd(metrics["t_limit"], 3),
        "pass": metrics["pass"],
        "reasons": metrics["reasons"],
        "yaw_error_deg": {k: rnd(v, 3) for k, v in metrics["yaw_error_deg"].items()},
        "dip_where": metrics["dip_where"],
        "tilt_where": metrics["tilt_where"],
    }


def _requested_seconds():
    if "--seconds" not in sys.argv:
        return None
    return _argv("--seconds", 1.0)


def _sweep_case(case):
    offset, speed, mu_b, mu_c = case
    _, metrics, _, _ = simulate(
        speed, speed, mu_b, mu_c, offset, frames=0, limit=_requested_seconds(),
        visuals=False, announce_drive=False)
    return _row(offset, speed, mu_b, mu_c, metrics)


def run_sweep():
    # No frames. The matrix is the result, and a timed-out run is a failed row.
    print_spans()
    speeds = (0.03, 0.08, 0.155)
    mus = ((0.6, 0.6), (0.9, 0.9), (1.2, 1.2), (0.9, 0.7), (0.7, 0.9))
    offsets = (-4.0, 0.0, 4.0)
    cases = [(offset, speed, mu_b, mu_c)
             for offset in offsets for speed in speeds for mu_b, mu_c in mus]
    jobs = _argv("--jobs", max(1, (os.cpu_count() or 2) - 1))
    cap = _requested_seconds()
    print("sweep  %d runs  jobs %d  dt %.4g s  noslip %d  seconds %s"
          % (len(cases), jobs, DT, int(NOSLIP),
             "path" if cap is None else "%.2f" % cap), flush=True)
    # One announced check, at a speed the matrix actually runs. Workers check
    # again at their own speed and only print if that check fails.
    setup(0.155, 0.155, 0.9, 0.9, visuals=False, announce=True)
    t0 = time.perf_counter()
    with concurrent.futures.ProcessPoolExecutor(max_workers=jobs) as ex:
        futs = [ex.submit(_sweep_case, case) for case in cases]
        done = {}
        for fut in concurrent.futures.as_completed(futs):
            done[fut] = fut.result()
            print("sweep finished %d/%d" % (len(done), len(futs)), flush=True)
    runs = [done[fut] for fut in futs]
    all_pass = True
    for row in runs:
        all_pass = all_pass and row["pass"]
        print("speed %.3f  mu %.2f/%.2f  off %+4.0f  entry %s  exit %s  yaw %s  "
              "dip %s  tilt %s  rails %d  %s"
              % (row["speed"], row["mu"], row["mu_curve"], row["offset_mm"],
                 fmt(row["entry_offset_mm"], "%.2f"),
                 fmt(row["exit_offset_mm"], "%.2f"),
                 fmt(row["exit_yaw_deg"], "%.1f"),
                 fmt(row["dip_mm"], "%.2f"),
                 fmt(row["max_tilt_deg"], "%.2f"),
                 len(row["rail_contacts"]),
                 "PASS" if row["pass"] else "FAIL: " + ", ".join(row["reasons"])))
    by_offset = {}
    for offset in offsets:
        group = [r for r in runs if r["offset_mm"] == offset and r["exit_offset_mm"] is not None]
        offs = [r["exit_offset_mm"] for r in group]
        yaws = [r["exit_yaw_deg"] for r in group]
        off_spread = (max(offs) - min(offs)) if offs else None
        yaw_spread = (max(yaws) - min(yaws)) if yaws else None
        # Reported, not gated. The drift with speed is accepted.
        by_offset[str(int(offset))] = {
            "exit_offset_spread_mm": rnd(off_spread, 3),
            "exit_yaw_spread_deg": rnd(yaw_spread, 3),
        }
        print("offset %+d  exit spread %s mm  yaw spread %s deg"
              % (int(offset), fmt(off_spread, "%.3f"), fmt(yaw_spread, "%.3f")))
    summary = {"all_pass": all_pass, "by_offset": by_offset}
    with open(os.path.join(OUT, "sweep.json"), "w", encoding="utf-8") as fh:
        json.dump({"runs": runs, "summary": summary}, fh, indent=2)
        fh.write("\n")
    worst = max(runs, key=lambda r: 1e9 if r["exit_yaw_deg"] is None else abs(r["exit_yaw_deg"] - 90.0))
    ye = worst["yaw_error_deg"]
    print("worst yaw  speed %.3f  mu %.2f/%.2f  off %+4.0f  entry %s  mid %s  exit-face %s  station %s"
          % (worst["speed"], worst["mu"], worst["mu_curve"], worst["offset_mm"],
             fmt(ye.get("entry_face"), "%.2f"), fmt(ye.get("mid_curve"), "%.2f"),
             fmt(ye.get("exit_face"), "%.2f"), fmt(ye.get("exit_station"), "%.2f")))
    print("sweep wall %.1f s" % (time.perf_counter() - t0))
    print("PASS" if all_pass else "FAIL")
    return all_pass


def run_viewer():
    # The only mode that puts the part back. An open line runs off the end,
    # and the viewer is a demo of the drive, not a measurement.
    import mujoco.viewer
    model, data, drives, _rails, _part_gid, part_bid = setup(
        STRAIGHT_SPEED, CURVE_SPEED, MU_BELT, MU_CURVE, visuals=True, announce=True)
    place_part(model, data, part_bid, ENTRY_OFFSET)
    end_y = S2["offset"][1] + straight_len
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.lookat[:] = (m(120.0), m(90.0), m(20.0))
        v.cam.distance = 0.72
        v.cam.azimuth = 145
        v.cam.elevation = -32
        while v.is_running():
            t0 = time.time()
            for _ in range(20):
                kick(data, drives)
                mujoco.mj_step(model, data)
            p = data.xpos[part_bid]
            if p[1] / MM > end_y or p[2] / MM < belt_top - 10:
                place_part(model, data, part_bid, ENTRY_OFFSET)
            v.sync()
            dt = model.opt.timestep * 20 - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    if "--view" in sys.argv:
        if "--seconds" in sys.argv:
            print("usage: --view runs until the window closes and does not take --seconds")
            sys.exit(2)
        print_spans()
        run_viewer()
    elif "--sweep" in sys.argv:
        sys.exit(0 if run_sweep() else 1)
    else:
        sys.exit(0 if run_nominal() else 1)
