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

STR, COR, S2 = G["straight"], G["corner"], G["straight2"]
straight_len, corner_len = STR["len"], COR["len"]

# One roller size everywhere now — both ends of every module are Ø10 noses, the
# discharge one driven. So a single radius, and both at the same height.
nose_r = G["nose_dia"] / 2.0 + belt_thickness
belt_top = G["carry_z"] + belt_thickness
nose_z = G["nose_z"]


def belt_y(mod):
    y0 = mod["t"] + side_gap + roller_flange_w
    return y0, y0 + belt_width


# v0 is three modules in two orientations, so placement is a transform rather
# than three hand-written coordinate sets. rot=90 maps local (x,y) -> (-y, x).
MODULES = [
    ("s1", STR, 0, 0.0, 0.0, 0),                                  # feeds +X
    ("c", COR, 90, COR["offset_x"], 0.0, 1),                      # turns, feeds +Y
    ("s2", STR, 90, S2["offset_x"], S2["offset_y"], 1),           # receives end-on, +Y
]


def placer(rot, ox, oy):
    if rot == 0:
        return lambda x, y: (x + ox, y + oy)
    return lambda x, y: (-y + ox, x + oy)

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
             "cc_brackets", "cc_rollers", "cc_bed", "cc_belt",
             "s2_brackets", "s2_rollers", "s2_bed", "s2_belt"]
    out = []
    for n in names:
        p = os.path.join(PARTS, n + ".stl").replace("\\", "/")
        out.append('<mesh name="%s" file="%s" scale="%g %g %g"/>' % (n, p, MM, MM, MM))
    return "\n    ".join(out)


def visual_geom(mesh, colour):
    return ('<geom type="mesh" mesh="%s" contype="0" conaffinity="0" group="1" '
            'rgba="%s"/>' % (mesh, rgba(colour)))


def module_body(tag, mod, rot, ox, oy, meshes):
    # Collision for one module: a nose cylinder at each end plus a carry plate
    # between them, and whichever side plates actually stand above the belt.
    p = placer(rot, ox, oy)
    y0, y1 = belt_y(mod)
    ymid = (y0 + y1) / 2.0
    a0, a1 = mod["drive_ax"], mod["nose_ax"]
    euler = "90 0 0" if rot == 0 else "0 90 0"
    fric = 'friction="0.04 0.005 0.0001"'

    g = [visual_geom(n, c) for n, c in meshes]

    for name, ax in (("infeed", a0), ("driven", a1)):
        cx, cy = p(ax, ymid)
        g.append('<geom name="%s_%s" type="cylinder" size="%g %g" pos="%g %g %g" '
                 'euler="%s" rgba="%s" %s/>'
                 % (tag, name, m(nose_r), m(belt_width / 2.0),
                    m(cx), m(cy), m(nose_z), euler, rgba(BELT_C, 0.0), fric))

    cx, cy = p((a0 + a1) / 2.0, ymid)
    half_len, half_wid = (a1 - a0) / 2.0, belt_width / 2.0
    sx, sy = (half_len, half_wid) if rot == 0 else (half_wid, half_len)
    g.append('<geom name="%s_belt" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s" %s/>'
             % (tag, m(sx), m(sy), m(1.0), m(cx), m(cy), m(belt_top - 1.0),
                rgba(BELT_C, 0.0), fric))

    # Only the part of a plate standing ABOVE the belt can touch the payload, so
    # a plate cut to the carry plane contributes no collision geom at all —
    # which is exactly what an open transfer face should be.
    for i, yc in enumerate((mod["t"] / 2.0, mod["outer_width"] - mod["t"] / 2.0)):
        rt = mod["rail_top"][i]
        if rt <= belt_top:
            continue
        rx, ry = p(mod["len"] / 2.0, yc)
        rsx, rsy = (mod["len"] / 2.0, mod["t"] / 2.0) if rot == 0 else (mod["t"] / 2.0, mod["len"] / 2.0)
        g.append('<geom name="%s_rail%d" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s"/>'
                 % (tag, i, m(rsx), m(rsy), m((rt - belt_top) / 2.0),
                    m(rx), m(ry), m((rt + belt_top) / 2.0), rgba(BRACKET_C, 0.0)))

    return '<body name="%s" pos="0 0 0">\n      %s\n    </body>' % (tag, "\n      ".join(g))


def build_xml():
    s_y0, s_y1 = belt_y(STR)
    s_ymid = (s_y0 + s_y1) / 2.0
    c_y0, c_y1 = belt_y(COR)

    MESHES = {"s1": [("cs_brackets", BRACKET_C), ("cs_rollers", ROLLER_C),
                     ("cs_bed", RAIL_C), ("cs_belt", BELT_C)],
              "c": [("cc_brackets", BRACKET_C), ("cc_rollers", ROLLER_C),
                    ("cc_bed", RAIL_C), ("cc_belt", BELT_C)],
              "s2": [("s2_brackets", BRACKET_C), ("s2_rollers", ROLLER_C),
                     ("s2_bed", RAIL_C), ("s2_belt", BELT_C)]}

    bodies = [module_body(tag, mod, rot, ox, oy, MESHES[tag])
              for tag, mod, rot, ox, oy, _ in MODULES]

    # Guide rail on the corner's far side — arrests the incoming +X momentum so
    # the part squares up before the corner belt carries it away.
    rail_x = COR["offset_x"] - c_y0 + 4.0
    bodies.append(
        '<body name="guide" pos="0 0 0">\n      '
        '<geom name="guide" type="box" size="%g %g %g" pos="%g %g %g" rgba="%s"/>\n    </body>'
        % (m(2.0), m(corner_len / 2.0), m(10.0),
           m(rail_x), m(corner_len / 2.0), m(belt_top + 10.0), rgba(RAIL_C)))

    # Dead plate — the industrial answer to a transfer gap. A fixed, unpowered
    # bridge flush with both belt surfaces. Kept because it is what proved the
    # 27 mm gap could not be bridged: the part goes flat and still stalls, since
    # its weight comes off the belt and the belt loses the traction to push it.
    if DEADPLATE:
        x0 = STR["nose_ax"] + nose_r - 0.5
        x1 = COR["offset_x"] - c_y1 + 1.5
        bodies.append(
            '<body name="dead" pos="0 0 0">\n      '
            '<geom name="deadplate" type="box" size="%g %g %g" pos="%g %g %g" '
            'rgba="%s" friction="0.04 0.005 0.0001"/>\n    </body>'
            % (m((x1 - x0) / 2.0), m(belt_width / 2.0), m(1.0),
               m((x0 + x1) / 2.0), m(s_ymid), m(belt_top - 1.0), rgba((150, 148, 140))))

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

    {bodies}

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
        bodies="\n\n    ".join(bodies),
        px=m(40.0), py=m(s_ymid), pz=m(part_z),
        hx=m(PART[0] / 2.0), hy=m(PART[1] / 2.0), hz=m(PART[2] / 2.0),
        pm=PART_MASS, pc=rgba(PART_C))


# Commanded belt velocities, m/s. This is the thing the whole architecture
# exists to allow: each module driven independently.
def _argv(flag, default):
    return type(default)(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else default


STRAIGHT_SPEED = _argv("--speed", 0.131)    # N20 @ 250 RPM on a 10 mm roller
CORNER_SPEED = _argv("--corner-speed", STRAIGHT_SPEED)
DRIVE_GAIN = 60.0

# Belt-to-part friction. NOT a guess and not a bench unknown — it is bounded by
# published elastomer tribology plus the contact regime this build actually runs in.
#
# Published TPU/PU numbers span 0.3 to >1.0, and the spread is mostly CONTACT
# PRESSURE, not material variation. Elastomer friction is adhesion-dominated, and
# the adhesion term falls monotonically with normal load (power law), so pin-on-disc
# tests at 50 N over a few mm2 — order MPa — report the LOW end. A 30 g part on a
# 32x32 mm footprint is 0.29 kPa, about three orders of magnitude lower, which puts
# this build firmly in the adhesion-dominated regime where the HIGH end applies.
#
# 0.9 is the working value; sweep with --mu to check the design does not depend on it.
MU_BELT = _argv("--mu", 0.9)

# The belt geoms are given near-zero MuJoCo friction on purpose: their tangential
# behaviour is modelled here instead, as a drag toward the belt's surface VELOCITY
# VECTOR, clipped at the traction limit mu*N. That is what a real belt does — it
# drags a part until the part matches the belt, and it cannot pull harder than
# friction allows. It also means a commanded speed of 0 brakes the part.
#
# The vector part matters and was wrong until 2026-08-09. This used to add force
# only along each module's DRIVE axis, which left a part sliding sideways with no
# friction from the model AND near-zero friction from MuJoCo — so lateral motion,
# once started, persisted almost undamped. Real belt friction is isotropic in the
# plane: sliding across a belt is opposed exactly as much as sliding along it.
def belt_drive(model, data, belts, part_gid, part_bid):
    # belts: list of (geom-id set, axis 0=X 1=Y, commanded speed). A part
    # straddling two modules gets pulled by both, which is exactly what happens
    # at a transfer and is the reason a handoff can stall.
    touching = [False] * len(belts)
    normal = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        if part_gid not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == part_gid else c.geom1
        hit = False
        for k, (ids, _, _) in enumerate(belts):
            if other in ids:
                touching[k] = True
                hit = True
        if not hit:
            continue
        f = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, f)
        normal += abs(f[0])

    if not any(touching):
        data.xfrc_applied[part_bid][:3] = (0.0, 0.0, 0.0)
        return

    traction = MU_BELT * max(normal, 0.05 * PART_MASS * 9.81)
    v = data.cvel[part_bid][3:6]

    force = [0.0, 0.0]
    for k, (_, axis, speed) in enumerate(belts):
        if not touching[k]:
            continue
        # This belt's surface velocity as a 2D vector, then drag the part toward
        # it in BOTH components — the cross-axis term is what damps lateral drift.
        surface = [0.0, 0.0]
        surface[axis] = speed
        force[0] += DRIVE_GAIN * PART_MASS * (surface[0] - v[0])
        force[1] += DRIVE_GAIN * PART_MASS * (surface[1] - v[1])

    mag = math.hypot(force[0], force[1])
    if mag > traction and mag > 0:
        force[0] *= traction / mag
        force[1] *= traction / mag

    data.xfrc_applied[part_bid][:3] = (force[0], force[1], 0.0)


def gid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


SPEEDS = {"s1": STRAIGHT_SPEED, "c": CORNER_SPEED, "s2": STRAIGHT_SPEED}


def setup():
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    belts = [({gid(model, "%s_%s" % (tag, s)) for s in ("belt", "infeed", "driven")},
              axis, SPEEDS[tag])
             for tag, _, _, _, _, axis in MODULES]
    part_gid = gid(model, "partgeom")
    part_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "part")
    return model, data, belts, part_gid, part_bid


def run_headless(seconds=6.0, frames=6):
    model, data, belts, part_gid, part_bid = setup()
    renderer = mujoco.Renderer(model, 900, 1400)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (m(150), m(70), m(26))
    cam.distance = 0.42
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
    # --frames 0 means "run it, don't draw it" — the trajectory is the result,
    # the PNGs are a convenience. Guard the divide rather than the caller.
    shot_every = steps // frames if frames > 0 else steps + 1
    shot = 0
    track = []
    for i in range(steps):
        belt_drive(model, data, belts, part_gid, part_bid)
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


def reset_part(model, data, part_bid):
    # v0 is an open line, so the part runs off the end and stops being
    # interesting. Put it back at the top so the viewer loops. This is a DEMO
    # convenience only — run_headless never does it, because the run-off is a
    # real result there and hiding it would be lying to the measurement.
    s_y0, s_y1 = belt_y(STR)
    qadr = model.body_jntadr[part_bid]
    qpos = model.jnt_qposadr[qadr]
    data.qpos[qpos:qpos + 7] = [m(40.0), m((s_y0 + s_y1) / 2.0),
                                m(belt_top + PART[2] / 2.0 + 0.5), 1, 0, 0, 0]
    dof = model.jnt_dofadr[qadr]
    data.qvel[dof:dof + 6] = 0
    data.xfrc_applied[part_bid][:3] = (0.0, 0.0, 0.0)


def run_viewer():
    import mujoco.viewer
    model, data, belts, part_gid, part_bid = setup()
    end_of_line = S2["offset_y"] + straight_len
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.lookat[:] = (m(150), m(70), m(20))
        v.cam.distance = 0.50
        v.cam.azimuth = 128
        v.cam.elevation = -26
        import time
        while v.is_running():
            t0 = time.time()
            for _ in range(20):
                belt_drive(model, data, belts, part_gid, part_bid)
                mujoco.mj_step(model, data)
            p = data.xpos[part_bid]
            if p[1] * 1000 > end_of_line or p[2] * 1000 < belt_top - 10:
                reset_part(model, data, part_bid)
            v.sync()
            dt = model.opt.timestep * 20 - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if "--view" in sys.argv:
    run_viewer()
else:
    run_headless(seconds=_argv("--seconds", 6.0), frames=_argv("--frames", 6))
