# Mini modular conveyor — MuJoCo sim (Hive plan #835)
#
#   uv run python cad/conveyor/sim_conveyor.py            # render frames to renders/sim/
#   uv run python cad/conveyor/sim_conveyor.py --view     # interactive viewer
#
# Meshes exported by build_parts.py are used for VISUALS. Collision is primitives
# (two cylinders + a plate per belt), because a belt loop is non-convex and MuJoCo
# would collapse its convex hull into a solid block — which would hide the one
# thing this sim exists to test: whether a part survives the gap between modules.
#
# The belt SURFACE drive is applied as a tangential force on whatever is touching
# a belt geom, since MuJoCo has no moving-surface primitive. Everything else —
# the part tipping, bridging, or dropping into the gap — is real contact physics.

import os
import sys
import math
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = os.path.join(HERE, "parts")
OUT = os.path.join(HERE, "renders", "sim")
os.makedirs(OUT, exist_ok=True)

MM = 0.001

# --- geometry, READ from the CAD ------------------------------------------
# These used to be re-declared here by hand. They are now whatever build_parts.py
# last exported, so a dimension change cannot leave the sim validating a design
# that no longer exists. Run build_parts.py first.
import json

with open(os.path.join(PARTS, "geometry.json"), encoding="utf-8") as fh:
    G = json.load(fh)

belt_width = G["belt_width"]
belt_thickness = G["belt_thickness"]
bracket_h = G["bracket_h"]
side_gap = G["side_gap"]
roller_flange_w = G["roller_flange_w"]

STR, COR = G["straight"], G["corner"]
straight_len, corner_len = STR["len"], COR["len"]
corner_dx = COR["offset_x"]

# Two roller radii now — the drive stays Ø25, the discharge nose is Ø10, and
# they are aligned by their TOPS (the carry plane), not by a shared centreline.
drive_r = G["roller_dia"] / 2.0 + belt_thickness
nose_r = G["nose_dia"] / 2.0 + belt_thickness
belt_top = G["carry_z"] + belt_thickness
drive_z, nose_z = G["drive_z"], G["nose_z"]


def belt_y(mod):
    y0 = mod["t"] + side_gap + roller_flange_w
    return y0, y0 + belt_width

PART = (32.0, 32.0, 16.0)
PART_MASS = 0.030

DEADPLATE = "--deadplate" in sys.argv


def m(v):
    return v * MM


def rgba(c, a=1.0):
    return "%.3f %.3f %.3f %.3f" % (c[0] / 255, c[1] / 255, c[2] / 255, a)


BRACKET_C = (208, 206, 198)
ROLLER_C = (196, 132, 74)
BELT_C = (58, 58, 62)
PART_C = (200, 69, 43)
RAIL_C = (43, 108, 176)


def mesh_assets():
    names = ["cs_brackets", "cs_rollers", "cs_bed", "cs_belt",
             "cc_brackets", "cc_rollers", "cc_bed", "cc_belt"]
    out = []
    for n in names:
        p = os.path.join(PARTS, n + ".stl").replace("\\", "/")
        out.append('<mesh name="%s" file="%s" scale="%g %g %g"/>' % (n, p, MM, MM, MM))
    return "\n    ".join(out)


def visual_geom(mesh, colour):
    return ('<geom type="mesh" mesh="%s" contype="0" conaffinity="0" group="1" '
            'rgba="%s"/>' % (mesh, rgba(colour)))


def build_xml():
    s_y0, s_y1 = belt_y(STR)
    c_y0, c_y1 = belt_y(COR)
    s_ymid, c_ymid = (s_y0 + s_y1) / 2.0, (c_y0 + c_y1) / 2.0

    # --- straight module collision: drive + nose cylinders, carry plate ----
    s_cyl = []
    for tag, ax, az, ar in (("drive", STR["drive_ax"], drive_z, drive_r),
                            ("nose", STR["nose_ax"], nose_z, nose_r)):
        s_cyl.append(
            '<geom name="s_%s" type="cylinder" size="%g %g" pos="%g %g %g" '
            'euler="90 0 0" rgba="%s" friction="0.04 0.005 0.0001"/>'
            % (tag, m(ar), m(belt_width / 2.0),
               m(ax), m(s_ymid), m(az), rgba(BELT_C, 0.0)))

    s_plate = ('<geom name="s_belt" type="box" size="%g %g %g" pos="%g %g %g" '
               'rgba="%s" friction="0.04 0.005 0.0001"/>'
               % (m((STR["nose_ax"] - STR["drive_ax"]) / 2.0), m(belt_width / 2.0), m(1.0),
                  m((STR["drive_ax"] + STR["nose_ax"]) / 2.0), m(s_ymid), m(belt_top - 1.0),
                  rgba(BELT_C, 0.0)))

    # Side plates. Only the part standing ABOVE the belt surface can touch the
    # payload, so a plate cut to the carry plane contributes no collision geom
    # at all — which is exactly what an open transfer face should be.
    s_rails = []
    for i, yc in enumerate((STR["t"] / 2.0, STR["outer_width"] - STR["t"] / 2.0)):
        rt = STR["rail_top"][i]
        if rt <= belt_top:
            continue
        s_rails.append(
            '<geom name="s_rail%d" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s"/>'
            % (i, m(straight_len / 2.0), m(STR["t"] / 2.0), m((rt - belt_top) / 2.0),
               m(straight_len / 2.0), m(yc), m((rt + belt_top) / 2.0), rgba(BRACKET_C, 0.0)))

    # --- corner module: rotated 90 deg about Z, then shifted +X ------------
    # local (x,y) -> (-y + corner_dx, x)
    def cpos(x, y, z):
        return (-y + corner_dx, x, z)

    c_cyl = []
    for tag, ax, az, ar in (("drive", COR["drive_ax"], drive_z, drive_r),
                            ("nose", COR["nose_ax"], nose_z, nose_r)):
        px, py, pz = cpos(ax, c_ymid, az)
        c_cyl.append(
            '<geom name="c_%s" type="cylinder" size="%g %g" pos="%g %g %g" '
            'euler="0 90 0" rgba="%s" friction="0.04 0.005 0.0001"/>'
            % (tag, m(ar), m(belt_width / 2.0), m(px), m(py), m(pz), rgba(BELT_C, 0.0)))

    cx, cy, _ = cpos((COR["drive_ax"] + COR["nose_ax"]) / 2.0, c_ymid, 0)
    c_plate = ('<geom name="c_belt" type="box" size="%g %g %g" pos="%g %g %g" '
               'rgba="%s" friction="0.04 0.005 0.0001"/>'
               % (m(belt_width / 2.0), m((COR["nose_ax"] - COR["drive_ax"]) / 2.0), m(1.0),
                  m(cx), m(cy), m(belt_top - 1.0), rgba(BELT_C, 0.0)))

    c_rails = []
    for i, yc in enumerate((COR["t"] / 2.0, COR["outer_width"] - COR["t"] / 2.0)):
        rt = COR["rail_top"][i]
        if rt <= belt_top:
            continue
        px, py, _ = cpos(corner_len / 2.0, yc, 0)
        c_rails.append(
            '<geom name="c_rail%d" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s"/>'
            % (i, m(COR["t"] / 2.0), m(corner_len / 2.0), m((rt - belt_top) / 2.0),
               m(px), m(py), m((rt + belt_top) / 2.0), rgba(BRACKET_C, 0.0)))

    # guide rail on the corner's far side — arrests the incoming +X momentum
    rail_x = corner_dx - c_y0 + 4.0
    guide = ('<geom name="guide" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s"/>'
             % (m(2.0), m(corner_len / 2.0), m(10.0),
                m(rail_x), m(corner_len / 2.0), m(belt_top + 10.0), rgba(RAIL_C)))

    # Dead plate — the industrial answer to a transfer gap. A fixed, unpowered
    # bridge flush with both belt surfaces, so the part is pushed across by the
    # belt behind it instead of nosing into the void. Off by default so the
    # failure it fixes stays reproducible.
    dead = ""
    if DEADPLATE:
        x0 = STR["nose_ax"] + nose_r - 0.5
        x1 = corner_dx - c_y1 + 1.5
        dead = ('<geom name="deadplate" type="box" size="%g %g %g" pos="%g %g %g" '
                'rgba="%s" friction="0.04 0.005 0.0001"/>'
                % (m((x1 - x0) / 2.0), m(belt_width / 2.0), m(1.0),
                   m((x0 + x1) / 2.0), m(s_ymid), m(belt_top - 1.0),
                   rgba((150, 148, 140))))

    part_z = belt_top + PART[2] / 2.0 + 0.5

    return """
<mujoco model="mini_conveyor">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic"/>
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
  </asset>

  <worldbody>
    <light pos="0.1 -0.15 0.4" dir="-0.2 0.4 -1" directional="true" diffuse="0.55 0.55 0.55"/>
    <geom name="floor" type="plane" size="1 1 0.05" material="gridmat" friction="0.8 0.01 0.001"/>

    <body name="straight" pos="0 0 0">
      {vs_brackets}
      {vs_rollers}
      {vs_bed}
      {vs_belt}
      {s_cyl}
      {s_plate}
      {s_rails}
    </body>

    <body name="corner" pos="0 0 0">
      {vc_brackets}
      {vc_rollers}
      {vc_bed}
      {vc_belt}
      {c_cyl}
      {c_plate}
      {c_rails}
      {guide}
      {dead}
    </body>

    <body name="part" pos="{px} {py} {pz}">
      <freejoint name="partfree"/>
      <!-- Low MuJoCo friction on purpose: MuJoCo combines a contact pair's
           friction by MAX, so a high value here would override the belt's and
           fight the explicit traction model in belt_drive(). -->
      <geom name="partgeom" type="box" size="{hx} {hy} {hz}" mass="{pm}"
            rgba="{pc}" friction="0.05 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>
""".format(
        meshes=mesh_assets(),
        vs_brackets=visual_geom("cs_brackets", BRACKET_C),
        vs_rollers=visual_geom("cs_rollers", ROLLER_C),
        vs_bed=visual_geom("cs_bed", RAIL_C),
        vs_belt=visual_geom("cs_belt", BELT_C),
        vc_brackets=visual_geom("cc_brackets", BRACKET_C),
        vc_rollers=visual_geom("cc_rollers", ROLLER_C),
        vc_bed=visual_geom("cc_bed", RAIL_C),
        vc_belt=visual_geom("cc_belt", BELT_C),
        s_cyl="\n      ".join(s_cyl), s_plate=s_plate, s_rails="\n      ".join(s_rails),
        c_cyl="\n      ".join(c_cyl), c_plate=c_plate, c_rails="\n      ".join(c_rails),
        guide=guide, dead=dead,
        px=m(40.0), py=m(s_ymid), pz=m(part_z),
        hx=m(PART[0] / 2.0), hy=m(PART[1] / 2.0), hz=m(PART[2] / 2.0),
        pm=PART_MASS, pc=rgba(PART_C))


# Commanded belt velocities, m/s. This is the thing the whole architecture
# exists to allow: each module driven independently.
STRAIGHT_SPEED = 0.131      # N20 @ 100 RPM on a 25 mm roller
CORNER_SPEED = 0.131
DRIVE_GAIN = 60.0
MU_BELT = 0.9               # belt surface against the part

# The belt geoms are given near-zero MuJoCo friction on purpose: their tangential
# behaviour is modelled here instead, as a drag toward belt speed clipped at the
# traction limit mu*N. That is what a real belt does — it drags a part until the
# part matches the belt, and it cannot pull harder than friction allows. It also
# means a commanded speed of 0 brakes the part, which is correct.
def belt_drive(model, data, straight_ids, corner_ids, part_gid, part_bid):
    touching_s = touching_c = False
    normal = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = c.geom1, c.geom2
        if part_gid not in (g1, g2):
            continue
        other = g2 if g1 == part_gid else g1
        if other in straight_ids:
            touching_s = True
        elif other in corner_ids:
            touching_c = True
        else:
            continue
        f = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, f)
        normal += abs(f[0])

    if not (touching_s or touching_c):
        data.xfrc_applied[part_bid][:3] = (0.0, 0.0, 0.0)
        return

    traction = MU_BELT * max(normal, 0.05 * PART_MASS * 9.81)
    v = data.cvel[part_bid][3:6]

    fx = fy = 0.0
    if touching_s:
        fx = DRIVE_GAIN * PART_MASS * (STRAIGHT_SPEED - v[0])
    if touching_c:
        fy = DRIVE_GAIN * PART_MASS * (CORNER_SPEED - v[1])

    mag = math.hypot(fx, fy)
    if mag > traction and mag > 0:
        fx *= traction / mag
        fy *= traction / mag

    data.xfrc_applied[part_bid][:3] = (fx, fy, 0.0)


def gid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def setup():
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    s_ids = {gid(model, n) for n in ("s_belt", "s_drive", "s_nose")}
    c_ids = {gid(model, n) for n in ("c_belt", "c_drive", "c_nose")}
    part_gid = gid(model, "partgeom")
    part_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "part")
    return model, data, s_ids, c_ids, part_gid, part_bid


def run_headless(seconds=6.0, frames=6):
    model, data, s_ids, c_ids, part_gid, part_bid = setup()
    renderer = mujoco.Renderer(model, 900, 1400)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (m(120), m(32), m(26))
    cam.distance = 0.30
    cam.azimuth = 132
    cam.elevation = -22

    import zlib, struct

    def png(path, rgbimg):
        h, w, _ = rgbimg.shape
        raw = bytearray()
        for y in range(h):
            raw.append(0)
            raw.extend(rgbimg[y].tobytes())

        def ch(t, d):
            return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

        blob = b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) \
            + ch(b"IDAT", zlib.compress(bytes(raw), 6)) + ch(b"IEND", b"")
        open(path, "wb").write(blob)

    steps = int(seconds / model.opt.timestep)
    shot_every = steps // frames
    shot = 0
    track = []
    for i in range(steps):
        belt_drive(model, data, s_ids, c_ids, part_gid, part_bid)
        mujoco.mj_step(model, data)
        if i % max(1, shot_every) == 0 and shot < frames:
            renderer.update_scene(data, cam)
            png(os.path.join(OUT, "frame%02d.png" % shot), renderer.render())
            p = data.xpos[part_bid]
            track.append((round(data.time, 2), round(p[0] * 1000, 1),
                          round(p[1] * 1000, 1), round(p[2] * 1000, 1)))
            shot += 1

    p = data.xpos[part_bid]
    print("t=%.2f  part x=%.1f y=%.1f z=%.1f mm" % (data.time, p[0] * 1000, p[1] * 1000, p[2] * 1000))
    for t in track:
        print("  t=%-5s x=%-7s y=%-7s z=%s" % t)
    return data, part_bid


def run_viewer():
    import mujoco.viewer
    model, data, s_ids, c_ids, part_gid, part_bid = setup()
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.lookat[:] = (m(corner_dx / 2.0 + 20), m(35), m(20))
        v.cam.distance = 0.40
        v.cam.azimuth = 128
        v.cam.elevation = -26
        import time
        while v.is_running():
            t0 = time.time()
            for _ in range(20):
                belt_drive(model, data, s_ids, c_ids, part_gid, part_bid)
                mujoco.mj_step(model, data)
            v.sync()
            dt = model.opt.timestep * 20 - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


def _arg(flag, default):
    return type(default)(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else default


if "--view" in sys.argv:
    run_viewer()
else:
    run_headless(seconds=_arg("--seconds", 6.0), frames=_arg("--frames", 6))
