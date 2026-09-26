# Mini modular conveyor — MuJoCo sim (Hive plan #835)
#
#   uv run python cad/conveyor/sim_conveyor.py            # nominal run, frames in renders/sim/
#   uv run python cad/conveyor/sim_conveyor.py --sweep    # acceptance matrix, writes sweep.json
#   uv run python cad/conveyor/sim_conveyor.py --view     # interactive viewer
#
# Visuals are the component STLs. Collision is not: a belt loop is non-convex
# and MuJoCo would fill its convex hull solid, which hides the gap this sim
# exists to measure. Straights keep two nose cylinders, a carry plate and the
# rails that stand above the belt. Each cone is the convex hull of its two
# end circles, which is the cone, so a mesh is honest there.
#
# The surface drive is a force on each contact. The curve's speed changes
# across the part, and a single drag at the centre of mass cannot yaw it with
# the path. Applying that yaw by hand would be deciding the result.

import os
import sys
import math
import json
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

# Regularised Coulomb. Below u0 the force is linear in slip, so the explicit
# step cannot add more speed than the slip it is cancelling. The bound used
# here is µ g dt / u0 ≤ 0.5 at the highest µ the sweep commands.


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
MU_CURVE = _argv("--mu-curve", 0.35)
ENTRY_OFFSET = _argv("--offset", 0.0)
# The U0 study did not converge: 0.01, 0.003 and 0.001 disagree by more than
# 0.5° of yaw. The fallback is the smallest, and dt is what keeps
# 1.2·g·dt/U0 under 0.5, 1.2 being the sweep's highest belt friction.
U0 = _argv("--u0", 0.001)
DT = _argv("--dt", 0.00004)


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
    return ('<geom type="mesh" mesh="%s" contype="0" conaffinity="0" group="1" '
            'rgba="%s"/>' % (mesh, rgba(colour)))


def straight_body(tag, rot, ox, oy, visuals):
    # Rails are only the plate standing above the belt. A face cut flush with
    # the carry plane is not a kerb, and the transfer depends on that.
    p = placer(rot, ox, oy)
    y0, y1 = belt_y()
    ymid = (y0 + y1) / 2.0
    a0, a1 = STR["drive_ax"], STR["nose_ax"]
    euler = "90 0 0" if rot == 0 else "0 90 0"
    fric = 'friction="0.04 0.005 0.0001"'
    g = [visual_geom(n, c) for n, c in visuals]

    # Same reason as the cone slices. A nose is a line contact, and one
    # reported point on a 50 mm cylinder is not a support.
    n_ax = int(math.ceil(belt_width / 5.0 - 1e-9))
    seg = belt_width / n_ax
    for name, ax in (("infeed", a0), ("driven", a1)):
        for k in range(n_ax):
            y_c = y0 + (k + 0.5) * seg
            cx, cy = p(ax, y_c)
            g.append('<geom name="%s_%s%d" type="cylinder" size="%g %g" pos="%g %g %g" '
                     'euler="%s" rgba="%s" %s/>'
                     % (tag, name, k, m(nose_r), m(seg / 2.0),
                        m(cx), m(cy), m(nose_z), euler, rgba(BELT_C, 0.0), fric))

    cx, cy = p((a0 + a1) / 2.0, ymid)
    half_len, half_wid = (a1 - a0) / 2.0, belt_width / 2.0
    sx, sy = (half_len, half_wid) if rot == 0 else (half_wid, half_len)
    g.append('<geom name="%s_belt" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s" %s/>'
             % (tag, m(sx), m(sy), m(1.0), m(cx), m(cy), m(belt_top - 1.0),
                rgba(BELT_C, 0.0), fric))

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

    return '<body name="%s" pos="0 0 0">\n      %s\n    </body>' % (tag, "\n      ".join(g))


def curve_body():
    # Fixed geoms. The rollers do not spin: the traction law imposes the
    # surface velocity, the same way the straight cylinders do.
    fric = 'friction="0.04 0.005 0.0001"'
    g = [visual_geom(n, c) for n, c in VISUALS if n.startswith("cv_")]
    for i in range(CURVE["n"]):
        for k in range(n_frusta()):
            g.append('<geom name="c_roll%d_%d" type="mesh" mesh="cone%d_%d" rgba="%s" %s/>'
                     % (i, k, i, k, rgba(ROLLER_C, 0.0), fric))

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
    return '<body name="c" pos="0 0 0">\n      %s\n    </body>' % ("\n      ".join(g))


def joiner_body():
    g = [visual_geom(n, c) for n, c in VISUALS if n.startswith("jn_")]
    return '<body name="joiners" pos="0 0 0">\n      %s\n    </body>' % ("\n      ".join(g))


def build_xml():
    y0, y1 = belt_y()
    part_z = belt_top + PART[2] / 2.0 + 0.5
    s1v = [(n, c) for n, c in VISUALS if n.startswith("cs_")]
    s2v = [(n, c) for n, c in VISUALS if n.startswith("s2_")]
    bodies = [
        straight_body("s1", S1["rot"], S1["offset"][0], S1["offset"][1], s1v),
        curve_body(),
        straight_body("s2", S2["rot"], S2["offset"][0], S2["offset"][1], s2v),
        joiner_body(),
    ]
    # multiccd: a box on a cylinder or a cone is a line contact. One reported
    # point can sit anywhere along that line, and the torque about it is then
    # noise. Every point on the line has to count.
    return """
<mujoco model="mini_conveyor">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic">
    <flag multiccd="enable"/>
  </option>
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
      <!-- MuJoCo combines a pair's friction by the maximum, so a high value
           here would override the drive geoms and fight the traction law. -->
      <geom name="partgeom" type="box" size="{hx} {hy} {hz}" mass="{pm}"
            rgba="{pc}" friction="0.05 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>
""".format(
        meshes=mesh_assets(),
        cones=cone_assets(),
        bodies="\n\n    ".join(bodies),
        dt=DT,
        px=m(40.0), py=m(lane_centre()), pz=m(part_z),
        hx=m(PART[0] / 2.0), hy=m(PART[1] / 2.0), hz=m(PART[2] / 2.0),
        pm=PART_MASS, pc=rgba(PART_C))


def gid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


_MODEL = None
_MODEL_DT = None


def compiled_model():
    # Speed and friction are applied in Python, so one compile serves every
    # run at this timestep. --dt changes the option, so it compiles again.
    global _MODEL, _MODEL_DT
    if _MODEL is None or _MODEL_DT != DT:
        _MODEL = mujoco.MjModel.from_xml_string(build_xml())
        _MODEL_DT = DT
    return _MODEL


def setup(straight_speed, curve_speed, mu_belt, mu_curve):
    model = compiled_model()
    data = mujoco.MjData(model)
    part_gid = gid(model, "partgeom")
    part_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "part")

    # geom id -> (kind, extra, mu). extra is the travel direction for a straight.
    # Slices share a name prefix; each one is its own drive geom.
    drive = {}
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if not name:
            continue
        if name == "s1_belt" or name.startswith("s1_infeed") or name.startswith("s1_driven"):
            drive[i] = ("straight", (straight_speed, 0.0), mu_belt)
        elif name == "s2_belt" or name.startswith("s2_infeed") or name.startswith("s2_driven"):
            drive[i] = ("straight", (0.0, straight_speed), mu_belt)
        elif name.startswith("c_roll"):
            drive[i] = ("curve", curve_speed, mu_curve)

    rails = set()
    for tag in ("s1", "s2"):
        for name in ("rail0", "rail1", "tab"):
            rails.add(gid(model, "%s_%s" % (tag, name)))
    nbox = int(round(90.0 / 5.0))
    for which in ("in", "out"):
        for i in range(nbox):
            rails.add(gid(model, "c_%s%d" % (which, i)))
    return model, data, drive, rails, part_gid, part_bid


def traction(model, data, drive, part_gid, part_bid):
    # Each contact pushes the part toward that patch's surface velocity, up to
    # µN. Weight on two modules splits the drive in the same proportion, which
    # is what a handoff actually does.
    com = np.array(data.xipos[part_bid])
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, part_bid, vel, 0)
    omega, vcom = vel[0:3], vel[3:6]
    cx, cy = CURVE["centre"]
    C = np.array([m(cx), m(cy), 0.0])
    F = np.zeros(3)
    tau = np.zeros(3)
    for i in range(data.ncon):
        c = data.contact[i]
        if part_gid not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == part_gid else c.geom1
        spec = drive.get(int(other))
        if spec is None:
            continue
        cf = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, cf)
        N = cf[0]
        if N <= 0.0:
            continue
        kind, extra, mu = spec
        p = np.array(c.pos)
        if kind == "straight":
            v_surf = np.array([extra[0], extra[1], 0.0])
        else:
            # Ω about z through C. Positive Ω is the left turn; on the
            # centreline the speed is the commanded curve speed.
            omega_z = extra / m(CURVE["r_c"])
            r = p - C
            v_surf = np.array([-omega_z * r[1], omega_z * r[0], 0.0])
        v_part = vcom + np.cross(omega, p - com)
        slip = v_surf - v_part
        slip[2] = 0.0
        speed = float(np.linalg.norm(slip))
        if speed < 1e-12:
            continue
        force = slip * (mu * N * min(1.0, speed / U0) / speed)
        F += force
        tau += np.cross(p - com, force)
    data.xfrc_applied[part_bid] = np.array([F[0], F[1], F[2], tau[0], tau[1], tau[2]])


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
    return path_length_m() / speed * 1.5 + 1.0


def place_part(model, data, part_bid, offset_mm_):
    qadr = model.body_jntadr[part_bid]
    qpos = model.jnt_qposadr[qadr]
    z = belt_top + PART[2] / 2.0 + 0.5
    data.qpos[qpos:qpos + 7] = [m(40.0), m(lane_centre() + offset_mm_), m(z), 1, 0, 0, 0]
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


def evaluate(samples, contacts, speed, limit):
    entry_x = CURVE["entry_face_x"] - 30.0
    exit_y = S2["offset"][1] + 40.0
    entry = crossed(samples, 1, entry_x)
    exit_ = crossed(samples, 2, exit_y)
    on_s1 = [s for s in samples if 0.3 <= s[0] <= 0.6 and s[1] < CURVE["entry_face_x"]]
    rest = sum(s[3] for s in on_s1) / len(on_s1) if on_s1 else None
    after = [s for s in samples if rest is not None and s[0] > (on_s1[-1][0] if on_s1 else 0.6)]
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
    if dip is None or dip > 1.0:
        reasons.append("dip %s mm" % ("?" if dip is None else "%.2f" % dip))
    if tilt is None or tilt > 5.0:
        reasons.append("tilt %s deg" % ("?" if tilt is None else "%.2f" % tilt))
    if contacts:
        reasons.append("rail contact")
    if entry_off is None or exit_off is None or abs(exit_off - entry_off) > 3.0:
        reasons.append("offset change")
    if exit_yaw is None or abs(exit_yaw - 90.0) > 5.0:
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


def simulate(straight_speed, curve_speed, mu_belt, mu_curve, offset, frames=0, limit=None, prove=False):
    # Headless on purpose. Nothing here puts the part back on the belt.
    model, data, drive, rails, part_gid, part_bid = setup(
        straight_speed, curve_speed, mu_belt, mu_curve)
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
        # the exit, and shots parked out there never get taken.
        travel_steps = max(1, int(path_length_m() / straight_speed / dt))
        shot_at = set(int(round(travel_steps * k / max(1, frames - 1))) for k in range(frames))
    for i in range(steps):
        traction(model, data, drive, part_gid, part_bid)
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
            on_s1 = [s for s in samples if 0.3 <= s[0] <= 0.6 and s[1] < CURVE["entry_face_x"]]
            rest = sum(s[3] for s in on_s1) / len(on_s1) if on_s1 else None
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


def run_nominal():
    # --seconds replaces the acceptance limit. A cap that misses the exit fails;
    # it is not a different, easier test.
    limit = time_limit(STRAIGHT_SPEED)
    if "--seconds" in sys.argv:
        limit = _argv("--seconds", limit)
    frames = _argv("--frames", 6)
    print_spans()
    print("nominal  speed %.3f  curve %.3f  mu %.2f / %.2f  offset %.1f mm  limit %.2f s"
          % (STRAIGHT_SPEED, CURVE_SPEED, MU_BELT, MU_CURVE, ENTRY_OFFSET, limit))
    samples, metrics, shots, proof = simulate(
        STRAIGHT_SPEED, CURVE_SPEED, MU_BELT, MU_CURVE, ENTRY_OFFSET,
        frames=frames, limit=limit, prove=True)
    print_proof(proof)
    for s in samples:
        print("  t=%5.2f  x=%7.1f  y=%7.1f  z=%6.2f  yaw=%6.1f  tilt=%5.2f  off=%6.2f"
              % (s[0], s[1], s[2], s[3], s[4], s[5], offset_mm(s[1], s[2])))
    for name in os.listdir(OUT):
        if name.startswith("frame") and name.endswith(".png"):
            os.remove(os.path.join(OUT, name))
    for i, img in enumerate(shots):
        write_png(os.path.join(OUT, "frame%02d.png" % i), img)
    print_metrics(metrics)
    return metrics["pass"]


def rnd(v, n=4):
    if v is None:
        return None
    return round(float(v), n)


def run_sweep():
    # No frames. The matrix is the result, and a timed-out run is a failed row.
    print_spans()
    speeds = (0.03, 0.08, 0.155)
    mus = ((0.3, 0.25), (0.9, 0.35), (1.2, 0.5), (1.2, 0.25))
    offsets = (-8.0, 0.0, 8.0)
    runs = []
    all_pass = True
    for offset in offsets:
        for speed in speeds:
            for mu_b, mu_c in mus:
                _, metrics, _, _ = simulate(speed, speed, mu_b, mu_c, offset, frames=0)
                row = {
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
                    "yaw_error_deg": {k: rnd(v, 3) for k, v in metrics["yaw_error_deg"].items()},
                    "dip_where": metrics["dip_where"],
                    "tilt_where": metrics["tilt_where"],
                }
                runs.append(row)
                all_pass = all_pass and metrics["pass"]
                print("speed %.3f  mu %.2f/%.2f  off %+4.0f  entry %s  exit %s  yaw %s  "
                      "dip %s  tilt %s  rails %d  %s"
                      % (speed, mu_b, mu_c, offset,
                         fmt(row["entry_offset_mm"], "%.2f"),
                         fmt(row["exit_offset_mm"], "%.2f"),
                         fmt(row["exit_yaw_deg"], "%.1f"),
                         fmt(row["dip_mm"], "%.2f"),
                         fmt(row["max_tilt_deg"], "%.2f"),
                         len(row["rail_contacts"]),
                         "PASS" if row["pass"] else "FAIL"))
    by_offset = {}
    for offset in offsets:
        group = [r for r in runs if r["offset_mm"] == offset and r["exit_offset_mm"] is not None]
        offs = [r["exit_offset_mm"] for r in group]
        yaws = [r["exit_yaw_deg"] for r in group]
        off_spread = (max(offs) - min(offs)) if offs else None
        yaw_spread = (max(yaws) - min(yaws)) if yaws else None
        ok = (off_spread is not None and off_spread <= 1.0
              and yaw_spread is not None and yaw_spread <= 2.0
              and len(group) == 12)
        by_offset[str(int(offset))] = {
            "exit_offset_spread_mm": rnd(off_spread, 3),
            "exit_yaw_spread_deg": rnd(yaw_spread, 3),
            "pass": ok,
        }
        all_pass = all_pass and ok
        print("offset %+d  exit spread %s mm  yaw spread %s deg  %s"
              % (int(offset), fmt(off_spread, "%.3f"), fmt(yaw_spread, "%.3f"),
                 "PASS" if ok else "FAIL"))
    summary = {"all_pass": all_pass, "by_offset": by_offset}
    with open(os.path.join(OUT, "sweep.json"), "w", encoding="utf-8") as fh:
        json.dump({"runs": runs, "summary": summary}, fh, indent=2)
        fh.write("\n")
    print("PASS" if all_pass else "FAIL")
    return all_pass


def run_viewer():
    # The only mode that puts the part back. An open line runs off the end,
    # and the viewer is a demo of the drive, not a measurement.
    import time
    import mujoco.viewer
    model, data, drive, rails, part_gid, part_bid = setup(
        STRAIGHT_SPEED, CURVE_SPEED, MU_BELT, MU_CURVE)
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
                traction(model, data, drive, part_gid, part_bid)
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
        print_spans()
        run_viewer()
    elif "--sweep" in sys.argv:
        sys.exit(0 if run_sweep() else 1)
    else:
        sys.exit(0 if run_nominal() else 1)
