"""generate_mounts.py — parametric wall-mount brackets for the bench camera.

Run headless (plan #653):

    "C:\\Program Files\\Blender Foundation\\Blender 4.5\\blender.exe" \
        --background --python cad/camera-mount/generate_mounts.py

Generates, per tilt angle (45 and 60 deg down from horizontal):
    camera_mount_<angle>.blend   — for eyeballing in Blender
    camera_mount_<angle>.stl     — for the slicer (1 unit = 1 mm)
    preview_<angle>_{iso,side,front}.png — quick renders

Geometry (all mm), v2 after Kyle's frame review: a backplate screwed to
the wood wall (two woodscrew holes side by side, centered vertically in
the plate's top section, plus a narrower tail running down the wall) ->
a T-beam neck (wide flange up, web below, for bending stiffness) -> an
open frame tilted down. The frame section is ~15 mm deep: 2.5 mm front
lip, the board slot channel, then a full 10 mm of open cavity behind the
board for slotting it in and for the rear 4-pin USB connector/cable. The
frame back is OPEN except a solid wall across the top, auto-sized per
tilt angle to catch the ENTIRE neck cross-section — the neck is trimmed
flush against that tilted wall and never enters the board's slide path.

The camera board (ELP-USBFHD01M-L36, 38 x 38 x 2 mm Kyle-measured)
drops in from the top along side-rail grooves — forgiving fit, flared
entry, zero force — and gravity seats it against the end stop.
"""

import math
import os

import bpy
from mathutils import Euler, Vector

# ---------------------------------------------------------------- parameters
P = {
    # camera board + fit, ratified exact (Kyle, first-print spec):
    # board 37.5 x 37.5 x 1.5; slot channel EXACTLY 2.0; groove span
    # EXACTLY 38.5 (1 mm side-to-side). No entry flare.
    "board_w": 37.5,
    "board_t": 1.5,
    "slot_w_slop": 1.0,     # extra groove-floor-to-groove-floor width
    "slot_h_slop": 0.5,     # extra groove height over board thickness
    "entry_flare": 0.0,     # disabled per Kyle; >0 re-enables the flare
    "entry_flare_len": 8.0,
    # frame (Kyle: ~1.5 cm deep, slot 2.5 mm from the front edge, 10 mm
    # clear behind the board for slotting + cable wiring)
    "front_lip": 2.5,
    "back_cavity": 10.0,
    "rail_w": 5.0,
    "groove_d": 2.2,        # groove depth into each rail
    "frame_len": 48.0,      # length along the slide direction
    "stop_t": 3.0,          # end-stop wall thickness
    "wall_t": 3.5,          # top back wall (neck landing) thickness
    # backplate: top section carries both screws side by side at its
    # vertical center; a narrower tail runs down the wall below the neck
    "plate_t": 4.0,
    "top_w": 45.0,
    "top_h": 30.0,
    "tail_w": 26.0,
    "tail_drop": 30.0,      # tail length below the neck flange top
    "screw_spacing": 28.0,  # hole centers, horizontal
    "screw_shank": 4.2,     # clearance for #6 / 3.5 mm woodscrew
    "screw_head": 8.6,      # countersink head diameter
    # T-beam neck: flange (top, horizontal) + web (below, vertical).
    # web_h 12->8 (Kyle): a shorter neck silhouette needs a shorter top
    # wall, preserving >= back_open_min of open frame back at 60 deg.
    "flange_w": 22.0,
    "flange_t": 5.0,
    "web_w": 8.0,
    "web_h": 8.0,
    "back_open_min": 10.0,  # guaranteed open gap below the top wall
    "wall_clearance": 12.0,  # min gap frame-to-wall (plate 4 + 8 free)
    # slot cap (Kyle): flat plate on the frame's entry face + a tongue
    # into the slot channel. Tongue thickness = the channel's exact 2.0
    # (deliberately tight — Kyle files the first article to fit). One
    # cap fits both variants (identical entry cross-section).
    "cap_t": 3.0,            # plate thickness
    "cap_tongue_len": 6.0,   # into the channel; seated board edge is 6.5
    "cap_tongue_clear": 0.1,  # width clearance vs the 38.5 groove span
    # variants: tilt below horizontal
    "angles_deg": (45, 60),
    # ---- table-edge stand (plan #717.5), Kyle 2026-07-28: "mount it to
    # the table flush on the outboard side edge so the arm extends up and
    # like you said we have the same camera holder frame."
    #
    # Same frame, same cap, different foundation: a plate flat against
    # the table's OUTBOARD edge face, and a post rising past the table
    # top to hold the camera just above the surface looking back across
    # it. Nothing intrudes on the work area, and the plate registers
    # against a real edge rather than floating on a weighted base —
    # 716.4 forbids a camera that can be nudged.
    # The plate is EXACTLY the tabletop thickness, 3/4 in, so it covers
    # the edge face and nothing more (Kyle 2026-07-28: "make the screw
    # mounting plate 3/4 high, so now you can set the height of the frame
    # exactly where you want the camera off the table"). That split is
    # the point: the plate is fixed by the table, and `lens_h` alone is
    # the knob for how high the camera sits — change it and only the
    # post grows.
    "table_t": 19.05,        # 0.75 in tabletop, from bench.json
    "lens_h": 75.0,          # LENS centre above the table top (~3 in)
    # The lens stands proud of the PCB, so the board is NOT the optical
    # centre — placing the board at lens_h would put the actual lens
    # lower and further forward. Kyle measured ~0.75 in (2026-07-28);
    # no published figure exists for this module (the vendor pages give
    # only the 38x38 board, and M12 holders run 8-16.5 mm), so a part in
    # hand beats a datasheet. Strictly the no-parallax point sits a few
    # mm BEHIND the front element, which is well inside the error already
    # carried by the placeholder FOV — see bench_scene.DEFAULT_FOVY_DEG.
    "lens_offset": 19.05,    # PCB face -> lens, along the optical axis
    # Aimed at the work-zone centre from the SHORT edge of the main
    # table (Kyle: "mounted on the outside short edge of the main table
    # ... centered with a perpendicular angle"). Perpendicular means the
    # yaw is ZERO and the part is symmetric — the earlier 61 deg came
    # from mounting on the LONG front edge, where the work zone can only
    # be seen diagonally. Off the short edge it is dead ahead.
    # 7 deg down, chosen against the actual work zone rather than to
    # centre the frame on it. The trade, computed from the short edge at
    # 75 mm with a 45.5 deg vertical field: the tilt has to be steep
    # enough that the NEAREST work-zone row is still inside the bottom of
    # the frame, and shallow enough to leave headroom above the table for
    # a lifted part. At 7 deg the table is in frame from y 76.9 outward
    # (the work zone ends at 77.05, so it is all covered) and the camera
    # sees ~216 mm above the table at the work-zone centre — a 20 mm
    # blank plus a generous lift. Steeper (8.5) would centre the frame
    # exactly on the work zone but leaves only 4 mm of margin at the near
    # edge; shallower (6) gains headroom and starts losing the near rows.
    "edge_tilt_deg": 7.0,    # down from horizontal
    "edge_yaw_deg": 0.0,     # 0 = square to the edge it is bolted to
    "post_w": 16.0,          # along the edge
    "post_d": 14.0,          # outboard
    "edge_neck_len": 8.0,    # post face -> frame back wall
    "edge_plate_w": 46.0,
    "edge_plate_drop": 0.0,  # extra plate below the tabletop, if wanted
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


def cyl(name, r1, r2, depth, cx, cy, cz):
    bpy.ops.mesh.primitive_cone_add(
        vertices=48, radius1=r1, radius2=r2, depth=depth,
        location=(cx, cy, cz))
    ob = bpy.context.active_object
    ob.name = name
    ob.rotation_euler = Euler((-math.pi / 2, 0, 0))  # axis +Z -> +Y
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


# ------------------------------------------------------------- frame (local)
# Local frame before tilting: the board lies in XY, its front face at
# z=0 (lens looks along -Z, i.e. straight down); the slot channel is
# z in [0, groove_h], the open cavity behind it z in [groove_h,
# groove_h + back_cavity]. +Y is the insertion/top end; -Y the end stop.
def build_frame(p, wall_y0, wall_len):
    groove_h = p["board_t"] + p["slot_h_slop"]
    inner_half = (p["board_w"] + p["slot_w_slop"]) / 2 - p["groove_d"]
    rail_in = inner_half
    rail_out = rail_in + p["rail_w"]
    half_len = p["frame_len"] / 2
    z_front = -p["front_lip"]
    z_cavity = groove_h + p["back_cavity"]   # rails' back face
    z_wall_out = z_cavity + p["wall_t"]      # top wall outer face
    stop_in = -half_len + p["stop_t"] + 1.0  # board's -Y edge lands here
    frame_w = 2 * rail_out

    # side rails, full depth front lip -> cavity back
    frame = box("frame", p["rail_w"], p["frame_len"], z_cavity - z_front,
                -(rail_in + p["rail_w"] / 2), 0, (z_front + z_cavity) / 2)
    frame = union(frame, box(
        "rail", p["rail_w"], p["frame_len"], z_cavity - z_front,
        rail_in + p["rail_w"] / 2, 0, (z_front + z_cavity) / 2))
    # grooves the board edges ride in, open at the +Y entry end. The
    # 0.01 x-fudge pokes into the opening (never the floor) so the cut
    # face is not coplanar with the rail inner face: groove floor lands
    # at exactly rail_in + groove_d per side (span exactly 38.5), and
    # the z-extent is exact (channel 2.0, lip 2.5).
    for sgn in (-1, 1):
        g_y0, g_y1 = stop_in, half_len + 5
        frame = cut(frame, box(
            "groove", p["groove_d"] + 0.01, g_y1 - g_y0, groove_h,
            sgn * (rail_in + p["groove_d"] / 2 - 0.005),
            (g_y0 + g_y1) / 2, groove_h / 2))
        # optional stepped flare at the entry end (off for first print)
        f = p["entry_flare"]
        if f > 0:
            f_y0 = half_len - p["entry_flare_len"]
            frame = cut(frame, box(
                "flare", p["groove_d"] + f + 0.01, half_len + 5 - f_y0,
                groove_h + f,
                sgn * (rail_in + (p["groove_d"] + f) / 2),
                (f_y0 + half_len + 5) / 2, (groove_h + f) / 2 - f - 0.005))
    # end stop across the -Y end, full depth
    frame = union(frame, box(
        "stop", frame_w, p["stop_t"], z_cavity - z_front,
        0, stop_in - p["stop_t"] / 2, (z_front + z_cavity) / 2))
    # solid back wall across the top only — the neck landing. wall_y0 /
    # wall_len come from the variant (the wall may extend past the +Y
    # frame end to catch the whole neck at steep tilts). It dips 0.5
    # into the cavity so the union with the rails has real overlap
    # (tangent contact risks a non-manifold STL).
    frame = union(frame, box(
        "topwall", frame_w, wall_len, p["wall_t"] + 0.5,
        0, wall_y0 + wall_len / 2, (z_cavity - 0.5 + z_wall_out) / 2))
    return frame, half_len, groove_h, z_cavity, z_wall_out, rail_out


# ------------------------------------------------------------------- variant
def build_variant(angle_deg, p):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = "MILLIMETERS"

    phi = math.radians(90 - angle_deg)  # rotation about +X; 90deg = level
    rot = Euler((phi, 0, 0)).to_matrix()

    # Neck silhouette (world z, at the wall): web -7..+7, flange +7..+12
    # -> spans z -7..+12, height 19, centroid +2.5. The wall's outer
    # face must cover all of it (+1.5 margin each way => 22), centered
    # on the centroid. Vertical coverage of a wall strip of local
    # length L is L*sin(phi); at steep tilts the needed L exceeds what
    # fits on the frame, so the wall extends past the +Y frame end.
    web_top = p["web_h"] / 2 + 1
    neck_top = web_top + p["flange_t"]                 # +12
    neck_bot = -(p["web_h"] / 2 + 1)                   # -7
    neck_mid = (neck_top + neck_bot) / 2               # +2.5
    cover = (neck_top - neck_bot) + 3
    wall_len = math.ceil(cover / math.sin(phi))
    half_len_frame = p["frame_len"] / 2
    # the on-frame part of the wall may never close the back below it
    # past back_open_min (Kyle: >= 1 cm open gap on every variant) —
    # any excess extends past the +Y frame end instead
    stop_in = -half_len_frame + p["stop_t"] + 1.0
    on_frame_cap = half_len_frame - stop_in - p["back_open_min"]
    wall_on_frame = min(wall_len, on_frame_cap)
    wall_y0 = half_len_frame - wall_on_frame
    wall_ext = wall_len - wall_on_frame                # past the +Y end

    frame, half_len, groove_h, z_cavity, z_wall_out, rail_out = \
        build_frame(p, wall_y0, wall_len)
    frame.rotation_euler = Euler((phi, 0, 0))
    bpy.ops.object.transform_apply(rotation=True)

    # neck length so the tilted frame clears the wall plane (y=0)
    corners = [(x, y, z)
               for x in (-rail_out, rail_out)
               for y in (-half_len, half_len + wall_ext)
               for z in (-p["front_lip"], z_wall_out)]
    min_y = min((rot @ Vector(c))[1] for c in corners)
    a_local = Vector((0, wall_y0 + wall_len / 2, z_wall_out))
    a_rot = rot @ a_local
    neck_len = max(20.0, math.ceil(
        p["wall_clearance"] - min_y + a_rot.y - p["plate_t"]))

    # place the frame: the top wall's outer-face center meets the neck
    # at the neck silhouette's mid-height, so coverage is symmetric
    t = Vector((0, p["plate_t"] + neck_len, neck_mid)) - a_rot
    frame.location = t
    bpy.ops.object.transform_apply(location=True)

    # backplate: top section + narrower tail down the wall (tail rises
    # 0.5 into the top section for real union overlap)
    flange_top = p["flange_t"] + p["web_h"] / 2 + 1  # neck top z + margin
    mount = box("mount", p["top_w"], p["plate_t"], p["top_h"],
                0, p["plate_t"] / 2, flange_top + p["top_h"] / 2)
    mount = union(mount, box(
        "tail", p["tail_w"], p["plate_t"],
        flange_top + 0.5 + p["tail_drop"],
        0, p["plate_t"] / 2, (flange_top + 0.5 - p["tail_drop"]) / 2))

    # T-beam neck: flange up top (0.5 overlap into the web — tangent
    # unions risk non-manifold STL), web below, trimmed flush to the
    # tilted top wall so nothing enters the board's slide path. The
    # overrun past the wall plane must be generous: the FLANGE (highest
    # point) reaches the tilted plane last — 25 covers both angles.
    y0, y1 = p["plate_t"] - 1, p["plate_t"] + neck_len + 25
    neck = box("flange", p["flange_w"], y1 - y0, p["flange_t"] + 0.5,
               0, (y0 + y1) / 2, web_top + (p["flange_t"] - 0.5) / 2)
    neck = union(neck, box("web", p["web_w"], y1 - y0,
                           p["web_h"] + 2, 0, (y0 + y1) / 2, 0))
    # Trim the neck flush against the tilted wall: n is the wall's
    # outward normal (toward the plate); the cut cube sits on the far
    # side of the wall plane, its boundary 1 mm INSIDE the wall so the
    # kept neck penetrates 1 mm for a solid union.
    n = rot @ Vector((0, 0, 1))
    p0 = Vector((0, p["plate_t"] + neck_len, neck_mid))  # wall face center
    half = box("halfspace", 300, 300, 300, 0, 0, 0)
    half.rotation_euler = Euler((phi, 0, 0))
    half.location = p0 - n * (150 + 1.0)
    neck = cut(neck, half)

    mount = union(mount, neck)
    mount = union(mount, frame)

    # woodscrew holes: horizontal pair at the top section's v-center
    hole_z = flange_top + p["top_h"] / 2
    for sx in (-1, 1):
        x = sx * p["screw_spacing"] / 2
        mount = cut(mount, cyl("shank", p["screw_shank"] / 2,
                               p["screw_shank"] / 2, p["plate_t"] + 4,
                               x, p["plate_t"] / 2, hole_z))
        head_r = p["screw_head"] / 2
        sink = head_r - p["screw_shank"] / 2  # 45-deg countersink
        # cone radius1 sits at the -Y (wall) end after the axis rotation,
        # radius2 at +Y — so shank-into-head order recesses the head at
        # the OUTER face, widening outward for a flat-head screw
        mount = cut(mount, cyl("sink", p["screw_shank"] / 2, head_r,
                               sink, x, p["plate_t"] - sink / 2 + 0.01,
                               hole_z))
    # Where the LENS ends up, relative to the two things an installer can
    # actually reference: the wall face (y=0) and the screw-hole centres.
    # Needed because the operator positions the SCREWS but cares about
    # the LENS, and on a 60 deg bracket those differ by 16.5 mm
    # vertically — the lens is 19.05 mm proud of the PCB along the
    # optical axis, so the steeper the tilt the more of that offset
    # becomes drop rather than reach.
    lens = (rot @ Vector((0, stop_in + p["board_w"] / 2, 0))
            + rot @ Vector((0, 0, -p["lens_offset"])) + t)
    mount.name = f"CameraMount{angle_deg}"
    return mount, neck_len, wall_y0 - stop_in, lens, hole_z


def build_edge_stand(p):
    """Table-edge stand: plate on the outboard edge face, post up, same
    camera frame at the top looking back across the table (plan #717.5).

    Datum for this variant, chosen to be the two things a person can put
    a tape to: **z = 0 is the TABLE TOP, y = 0 is the outboard edge
    face.** The table is at y < 0, the stand at y > 0, so the whole
    mount lives outside the work area and the camera looks back over it.

    The frame is the SAME ratified part the wall brackets use — Kyle:
    "like you said we have the same camera holder frame" — so the board
    fit, the slot, the cap and the print orientation all carry over
    untouched. Only the foundation changes.

    Orientation is the wall variant's tilt plus a 180 deg yaw, and the
    180 is load-bearing: without it the frame's insertion end points
    DOWN and the board would fall out instead of seating against its
    stop. With it, the board still drops in from above under gravity.
    """
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = "MILLIMETERS"

    tilt = p["edge_tilt_deg"]
    phi = math.radians(90 - tilt)
    psi = math.radians(180 + p["edge_yaw_deg"])
    rot = Euler((phi, 0, psi)).to_matrix()

    # top wall sizing, same rule as the wall variants
    web_top = p["web_h"] / 2 + 1
    neck_top = web_top + p["flange_t"]
    neck_bot = -(p["web_h"] / 2 + 1)
    neck_mid = (neck_top + neck_bot) / 2
    wall_len = math.ceil(((neck_top - neck_bot) + 3) / math.sin(phi))
    half_len_frame = p["frame_len"] / 2
    stop_in = -half_len_frame + p["stop_t"] + 1.0
    on_frame_cap = half_len_frame - stop_in - p["back_open_min"]
    wall_on_frame = min(wall_len, on_frame_cap)
    wall_y0 = half_len_frame - wall_on_frame

    frame, half_len, groove_h, z_cavity, z_wall_out, rail_out = \
        build_frame(p, wall_y0, wall_len)
    frame.rotation_euler = Euler((phi, 0, psi))
    bpy.ops.object.transform_apply(rotation=True)

    # THE POST CANNOT BUTT THE BACK WALL FROM BELOW. At 8 deg the frame
    # stands nearly vertical, so its back wall is nearly vertical too and
    # its normal is nearly HORIZONTAL. Trimming a rising column against
    # that plane slices the column lengthwise instead of docking it —
    # the first attempt produced a tapered sliver. So the post stands
    # BESIDE the wall and a horizontal neck docks into it, which is the
    # same arrangement the wall variants use, just rotated into a régime
    # where the neck is horizontal and the wall is steep rather than the
    # other way round.
    n = rot @ Vector((0, 0, 1))          # wall's outward normal
    u = Vector((-n.x, -n.y, 0.0))        # post -> wall, horizontal
    u.normalize()
    y_post = p["post_d"] / 2             # post overlaps the plate in y
    reach = p["post_d"] / 2 + p["edge_neck_len"]

    a_local = Vector((0, wall_y0 + wall_len / 2, z_wall_out))
    a_rot = rot @ a_local
    board_c = rot @ Vector((0, stop_in + p["board_w"] / 2, 0))
    # The optical centre is lens_offset FORWARD of the board along the
    # view direction, so solving for the board height would leave the
    # real lens low by lens_offset*sin(tilt). Solve for the LENS and let
    # the board fall where it must.
    view = rot @ Vector((0, 0, -1))
    lens_from_board = view * p["lens_offset"]
    target = Vector((u.x * reach, y_post + u.y * reach,
                     p["lens_h"] - (board_c.z + lens_from_board.z)
                     + a_rot.z))
    t = target - a_rot
    frame.location = t
    bpy.ops.object.transform_apply(location=True)
    attach = a_rot + t
    lens = board_c + lens_from_board + t

    # Plate flat on the edge face: y in [0, plate_t], top flush with the
    # table top so nothing stands proud of the surface behind it.
    plate_h = p["table_t"] + p["edge_plate_drop"]
    mount = box("plate", p["edge_plate_w"], p["plate_t"], plate_h,
                0, p["plate_t"] / 2, -plate_h / 2)
    # Post: from under the tabletop up to the neck. Spans y 0..post_d so
    # it fully overlaps the plate — a tangent union risks a non-manifold
    # STL, which is why nothing here is left merely touching.
    post_z0, post_z1 = -plate_h, attach.z + p["flange_t"]
    mount = union(mount, box(
        "post", p["post_w"], p["post_d"], post_z1 - post_z0,
        0, y_post, (post_z0 + post_z1) / 2))
    # Horizontal neck, docked into the wall and trimmed flush against it
    # so nothing enters the board's slide path.
    ang = math.atan2(u.y, u.x)
    # Kept INSIDE the post's back face (t0) so it does not stub out into
    # space, and no wider than the post, so the silhouette stays a clean
    # column rather than a flare.
    t0, t1 = -p["post_d"] / 2 + 2.0, reach + 10.0
    mid = Vector((u.x * (t0 + t1) / 2, y_post + u.y * (t0 + t1) / 2, 0))
    neck = box("neck", t1 - t0, p["post_w"], p["web_h"] + p["flange_t"],
               mid.x, mid.y, attach.z)
    neck.rotation_euler = Euler((0, 0, ang))
    bpy.ops.object.transform_apply(rotation=True)
    half = box("halfspace", 400, 400, 400, 0, 0, 0)
    half.rotation_euler = Euler((phi, 0, psi))
    half.location = attach - n * (200 + 1.0)
    neck = cut(neck, half)
    mount = union(mount, neck)
    mount = union(mount, frame)

    # Two woodscrews side by side into the edge face, centred on the
    # tabletop's thickness. Countersunk from the OUTBOARD face.
    hole_z = -p["table_t"] / 2
    for sx in (-1, 1):
        x = sx * p["screw_spacing"] / 2
        mount = cut(mount, cyl("shank", p["screw_shank"] / 2,
                               p["screw_shank"] / 2, p["plate_t"] + 4,
                               x, p["plate_t"] / 2, hole_z))
        head_r = p["screw_head"] / 2
        sink = head_r - p["screw_shank"] / 2
        mount = cut(mount, cyl("sink", p["screw_shank"] / 2, head_r,
                               sink, x, p["plate_t"] - sink / 2 + 0.01,
                               hole_z))
    mount.name = "CameraMountEdgeStand"
    return mount, lens, attach


def build_cap(p):
    """Slot cap, angle-independent: plate on the entry end face (kept
    below the top wall's underside) + tongue riding the grooves at the
    channel's exact thickness. Local frame mirrors the frame end: the
    plate's inner face is y=0; the tongue extends to -tongue_len."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = "MILLIMETERS"

    groove_h = p["board_t"] + p["slot_h_slop"]
    span = p["board_w"] + p["slot_w_slop"]
    frame_w = 2 * (span / 2 - p["groove_d"] + p["rail_w"])
    z0 = -p["front_lip"]
    z1 = groove_h + p["back_cavity"] - 0.5  # top wall underside
    cap = box("cap", frame_w, p["cap_t"], z1 - z0,
              0, p["cap_t"] / 2, (z0 + z1) / 2)
    cap = union(cap, box(
        "tongue", span - p["cap_tongue_clear"], p["cap_tongue_len"] + 0.5,
        groove_h, 0, -(p["cap_tongue_len"] - 0.5) / 2, groove_h / 2))
    cap.name = "CameraMountCap"
    return cap


def add_preview_rig():
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
        mount, neck_len, back_open, lens, hole_z = build_variant(angle, P)
        center = (0, 34, -14)
        cam = add_preview_rig()
        aim(cam, (170, -130, 100), center)
        render(os.path.join(OUT_DIR, f"preview_{angle}_iso.png"))
        aim(cam, (240, 34, -14), center)
        render(os.path.join(OUT_DIR, f"preview_{angle}_side.png"))
        # looking up into the frame: slot, open back, top wall
        aim(cam, (90, 190, -120), center)
        render(os.path.join(OUT_DIR, f"preview_{angle}_front.png"))

        stl = os.path.join(OUT_DIR, f"camera_mount_{angle}.stl")
        bpy.ops.object.select_all(action="DESELECT")
        mount.select_set(True)
        bpy.context.view_layer.objects.active = mount
        bpy.ops.wm.stl_export(filepath=stl, export_selected_objects=True)
        bpy.ops.wm.save_as_mainfile(
            filepath=os.path.join(OUT_DIR, f"camera_mount_{angle}.blend"))
        print(f"[mounts] {angle} deg: neck_len={neck_len} "
              f"back_open={back_open:.1f} -> camera_mount_{angle}.blend/.stl")
        # INSTALL NUMBERS. The lens is what gets aimed; the screws are
        # what gets positioned. Report the offset between them so nobody
        # has to derive it at the wall with a drill in hand.
        print(f"[mounts] {angle} deg INSTALL: lens sits {lens.y:.1f} mm out "
              f"from the wall face, and {hole_z - lens.z:.1f} mm BELOW the "
              f"screw-hole centres")
        print(f"[mounts] {angle} deg          -> screw holes go "
              f"{hole_z - lens.z:.1f} mm ({(hole_z - lens.z)/25.4:.2f} in) "
              f"ABOVE the height you want the lens at")

    stand, lens, attach = build_edge_stand(P)
    cam = add_preview_rig()
    center = (0, 10, P["lens_h"] / 2)
    aim(cam, (210, 190, 150), center)
    render(os.path.join(OUT_DIR, "preview_edge_iso.png"))
    aim(cam, (300, 10, 30), center)          # along the table edge
    render(os.path.join(OUT_DIR, "preview_edge_side.png"))
    aim(cam, (0, -230, 60), (0, 0, P["lens_h"]))   # from over the table
    render(os.path.join(OUT_DIR, "preview_edge_front.png"))
    bpy.ops.object.select_all(action="DESELECT")
    stand.select_set(True)
    bpy.context.view_layer.objects.active = stand
    bpy.ops.wm.stl_export(
        filepath=os.path.join(OUT_DIR, "camera_mount_edge.stl"),
        export_selected_objects=True)
    bpy.ops.wm.save_as_mainfile(
        filepath=os.path.join(OUT_DIR, "camera_mount_edge.blend"))
    print(f"[mounts] edge stand: lens at y={lens.y:.1f} z={lens.z:.1f}, "
          f"back-wall attach z={attach.z:.1f} -> camera_mount_edge.blend/.stl")

    cap = build_cap(P)
    cam = add_preview_rig()
    aim(cam, (85, -95, 50), (0, -2, 4))  # tongue side
    render(os.path.join(OUT_DIR, "preview_cap.png"))
    bpy.ops.object.select_all(action="DESELECT")
    cap.select_set(True)
    bpy.context.view_layer.objects.active = cap
    bpy.ops.wm.stl_export(
        filepath=os.path.join(OUT_DIR, "camera_mount_cap.stl"),
        export_selected_objects=True)
    bpy.ops.wm.save_as_mainfile(
        filepath=os.path.join(OUT_DIR, "camera_mount_cap.blend"))
    print("[mounts] cap -> camera_mount_cap.blend/.stl")
    print("[mounts] done")


if __name__ == "__main__":
    main()
