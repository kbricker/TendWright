"""generate_mounts.py — parametric wall-mount brackets for the bench camera.

Run headless (plan #653):

    "C:\\Program Files\\Blender Foundation\\Blender 4.5\\blender.exe" \
        --background --python cad/camera-mount/generate_mounts.py

Generates, per tilt angle (45 and 60 deg down from horizontal):
    camera_mount_<angle>.blend   — for eyeballing in Blender
    camera_mount_<angle>.stl     — for the slicer (1 unit = 1 mm)
    preview_<angle>_iso.png / preview_<angle>_side.png — quick renders

Geometry (all mm): a backplate screwed to the wood wall (2 countersunk
woodscrew holes) -> a rectangular arm out from the wall -> an open-frame
cradle tilted down. The camera board (ELP-USBFHD01M-L36, bare 37.5 x 37.5
PCB) slides down the cradle's side-rail grooves from the top and seats
against an end stop by gravity. The cradle front is open for the M12 lens
holder (central ~20x20); the cradle back is a picture-frame with a
WINDOW x WINDOW opening so the 4-pin USB connector on the BACK of the
board and its cable plug pass straight through — nothing behind the board
but air.

Camera facts (researched 2026-07-24, plan #653; Kyle verifies against the
physical unit): board 37.5x37.5, "38x38" class (corner holes at 34 mm
spacing — unused here, the cradle grips the board edges), M12 holder with
20 mm screw spacing, 4-pin USB connector on the back. Unknown online:
connector exit direction and lens protrusion — hence the fully open back
window and open front.
"""

import math
import os
import sys

import bpy
from mathutils import Euler, Vector

# ---------------------------------------------------------------- parameters
P = {
    # camera board
    "board_w": 37.5,        # square PCB edge
    "board_t": 1.6,         # PCB thickness
    "slot_w_slop": 0.9,     # extra groove-floor-to-groove-floor width
    "slot_h_slop": 0.6,     # extra groove height over board thickness
    # cradle
    "rail_w": 5.0,          # side rail width (x)
    "groove_d": 2.2,        # groove depth into each rail
    "front_lip": 3.7,       # rail lip in front of the board plane
    "back_t": 3.0,          # back frame thickness
    "frame_len": 48.0,      # cradle length along the slide direction
    "window": 32.0,         # square opening in the back frame (cable/plug)
    "stop_t": 3.0,          # end-stop wall thickness
    # backplate
    "plate_w": 45.0,
    "plate_h": 70.0,
    "plate_t": 4.0,
    "screw_spacing": 50.0,  # hole centers, vertical
    "screw_shank": 4.2,     # clearance for #6 / 3.5 mm woodscrew
    "screw_head": 8.6,      # countersink head diameter
    # arm
    "arm_w": 16.0,
    "arm_h": 14.0,
    "arm_overlap": 2.0,     # penetration into the cradle frame for the union
    "wall_clearance": 12.0,  # min gap cradle-to-wall (plate 4 + 8 free)
    # variants: tilt below horizontal
    "angles_deg": (45, 60),
}

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


# ------------------------------------------------------------------- helpers
def box(name, sx, sy, sz, cx, cy, cz):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(cx, cy, cz))
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = (sx, sy, sz)
    bpy.ops.object.transform_apply(scale=True)
    return ob


def cyl(name, r1, r2, depth, cx, cy, cz, axis="Y"):
    bpy.ops.mesh.primitive_cone_add(
        vertices=48, radius1=r1, radius2=r2, depth=depth,
        location=(cx, cy, cz))
    ob = bpy.context.active_object
    ob.name = name
    if axis == "Y":  # cone axis defaults to +Z; point it along +Y
        ob.rotation_euler = Euler((-math.pi / 2, 0, 0))
        bpy.ops.object.transform_apply(rotation=True)
    return ob


def boolean(target, tool, op):
    mod = target.modifiers.new(name=op, type="BOOLEAN")
    mod.operation = op
    mod.object = tool
    mod.solver = "EXACT"
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(tool, do_unlink=True)
    return target


def union(target, tool):
    return boolean(target, tool, "UNION")


def cut(target, tool):
    return boolean(target, tool, "DIFFERENCE")


# ------------------------------------------------------------- cradle (local)
# Local frame before tilting: the board lies in XY, its back face at z=0
# (lens looks along -Z, i.e. straight down). +Y is the insertion/top end
# that attaches toward the arm; -Y is the end-stop end.
def build_cradle(p):
    inner_half = (p["board_w"] + p["slot_w_slop"]) / 2 - p["groove_d"]
    rail_in = inner_half                       # rail inner face |x|
    rail_out = rail_in + p["rail_w"]
    groove_floor = inner_half + p["groove_d"]  # board edges reach here
    half_len = p["frame_len"] / 2
    groove_h = p["board_t"] + p["slot_h_slop"]
    stop_in = -half_len + p["stop_t"] + 1.0    # board's -Y edge lands here

    frame_w = 2 * rail_out
    # back picture-frame the connector fires through
    cradle = box("cradle", frame_w, p["frame_len"], p["back_t"],
                 0, 0, groove_h + p["back_t"] / 2)
    cut(cradle, box("window", p["window"], p["window"], 30, 0, 0,
                    groove_h + p["back_t"] / 2))
    # side rails, front lip included
    rail_z0, rail_z1 = -p["front_lip"], groove_h
    for sgn in (-1, 1):
        cradle = union(cradle, box(
            "rail", p["rail_w"], p["frame_len"], rail_z1 - rail_z0,
            sgn * (rail_in + p["rail_w"] / 2), 0, (rail_z0 + rail_z1) / 2))
    # grooves: notches the board edges ride in, open at the +Y entry end,
    # ending short of the stop wall
    for sgn in (-1, 1):
        g_y0, g_y1 = stop_in, half_len + 5
        cradle = cut(cradle, box(
            "groove", p["groove_d"] + 0.01, g_y1 - g_y0, groove_h,
            sgn * (rail_in + p["groove_d"] / 2), (g_y0 + g_y1) / 2,
            groove_h / 2 - 0.005))
    # end stop across the -Y end
    cradle = union(cradle, box(
        "stop", frame_w, p["stop_t"],
        p["front_lip"] + groove_h + p["back_t"],
        0, stop_in - p["stop_t"] / 2,
        (-p["front_lip"] + groove_h + p["back_t"]) / 2))
    return cradle, half_len, groove_h


# ------------------------------------------------------------------- variant
def build_variant(angle_deg, p):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = "MILLIMETERS"

    phi = math.radians(90 - angle_deg)  # rotation about +X; 90deg = level

    cradle, half_len, groove_h = build_cradle(p)
    cradle.rotation_euler = Euler((phi, 0, 0))
    bpy.ops.object.transform_apply(rotation=True)

    # attachment point: center of the back frame's +Y (top) rim, mid-thickness
    a_local = Vector((0, half_len - 4.0, groove_h + p["back_t"] / 2))
    rot = Euler((phi, 0, 0)).to_matrix()
    a_rot = rot @ a_local

    # arm length so the tilted cradle clears the wall plane (y=0)
    min_y = min((rot @ Vector(c))[1] for c in
                [(x, y, z)
                 for x in (-30, 30)
                 for y in (-half_len, half_len)
                 for z in (-p["front_lip"], groove_h + p["back_t"])])
    # cradle translate T_y = plate_t + arm_len - arm_overlap - a_rot.y
    # require min_y + T_y >= wall_clearance
    arm_len = max(
        20.0,
        math.ceil(p["wall_clearance"] - min_y + a_rot.y
                  - p["plate_t"] + p["arm_overlap"]))

    t = Vector((0, p["plate_t"] + arm_len - p["arm_overlap"], 0)) - a_rot
    cradle.location = t
    bpy.ops.object.transform_apply(location=True)

    # backplate + arm
    mount = box("mount", p["plate_w"], p["plate_t"], p["plate_h"],
                0, p["plate_t"] / 2, 0)
    mount = union(mount, box(
        "arm", p["arm_w"], arm_len + 1, p["arm_h"],
        0, p["plate_t"] + (arm_len - 1) / 2, 0))
    mount = union(mount, cradle)

    # woodscrew holes: shank through, countersink at the outer face
    for sz in (-1, 1):
        z = sz * p["screw_spacing"] / 2
        mount = cut(mount, cyl("shank", p["screw_shank"] / 2,
                               p["screw_shank"] / 2, p["plate_t"] + 4,
                               0, p["plate_t"] / 2, z))
        head_r = p["screw_head"] / 2
        sink = head_r - p["screw_shank"] / 2  # 45-deg countersink
        mount = cut(mount, cyl("sink", head_r, p["screw_shank"] / 2,
                               sink, 0, p["plate_t"] - sink / 2 + 0.01, z))
    mount.name = f"CameraMount{angle_deg}"
    return mount, arm_len


def add_preview_rig(target_center):
    key = bpy.data.objects.new("Key", bpy.data.lights.new("Key", "SUN"))
    key.rotation_euler = Euler((math.radians(35), 0, math.radians(25)))
    bpy.context.collection.objects.link(key)
    fill = bpy.data.lights.new("Fill", "SUN")
    fill.energy = 0.6
    fob = bpy.data.objects.new("Fill", fill)
    fob.rotation_euler = Euler((math.radians(-60), 0, math.radians(-140)))
    bpy.context.collection.objects.link(fob)
    cam = bpy.data.objects.new("Camera", bpy.data.cameras.new("Camera"))
    bpy.context.collection.objects.link(cam)
    bpy.context.scene.camera = cam
    return cam


def aim(cam, location, at):
    cam.location = location
    d = Vector(at) - Vector(location)
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def render(path):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.render.resolution_x = 900
    scene.render.resolution_y = 700
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


def main():
    for angle in P["angles_deg"]:
        mount, arm_len = build_variant(angle, P)
        center = (0, 32, -18)
        cam = add_preview_rig(center)
        aim(cam, (170, -130, 100), center)
        render(os.path.join(OUT_DIR, f"preview_{angle}_iso.png"))
        aim(cam, (240, 32, -18), center)
        render(os.path.join(OUT_DIR, f"preview_{angle}_side.png"))
        # looking up into the cradle: slot groove, lens opening, back window
        aim(cam, (90, 190, -120), center)
        render(os.path.join(OUT_DIR, f"preview_{angle}_front.png"))

        stl = os.path.join(OUT_DIR, f"camera_mount_{angle}.stl")
        bpy.ops.object.select_all(action="DESELECT")
        mount.select_set(True)
        bpy.context.view_layer.objects.active = mount
        bpy.ops.wm.stl_export(filepath=stl, export_selected_objects=True)
        bpy.ops.wm.save_as_mainfile(
            filepath=os.path.join(OUT_DIR, f"camera_mount_{angle}.blend"))
        print(f"[mounts] {angle} deg: arm_len={arm_len} -> "
              f"camera_mount_{angle}.blend/.stl")
    print("[mounts] done")


if __name__ == "__main__":
    main()
