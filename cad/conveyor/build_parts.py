# Mini modular conveyor — parametric part set (Hive plan #835)
#
# Run:  "%LOCALAPPDATA%/Programs/FreeCAD 1.1/bin/freecadcmd.exe" cad/conveyor/build_parts.py
#
# Writes parts/*.stl + parts/*.step and a step-by-step build.log. freecadcmd
# swallows stdout and can die without a traceback, so if a run produces nothing,
# read build.log first — it names the last step that started.
#
# FreeCAD scripting traps this file is written around (CableCell/cad/README.md):
#   1. Shape.translate() mutates in place and returns None. Use translated(),
#      or call translate() and keep the same object.
#   2. Shape.rotate() mutates in place. copy() first, then rotate() on that
#      copy, and never use rotate()'s return value. export of a placed part
#      does it this way.
#   3. freecadcmd sets __name__ to the module basename, so a __main__ guard
#      never fires. There isn't one.
#   4. Routing an STL through a Mesh::Feature crashes the process. Meshes are
#      written directly via Mesh.Mesh(shape.tessellate(dev)).write(path).
#
# The belt corner (superseded 2026-09-25) is in git history at 7c560e0.
# v0 is two straights and a tapered-roller curve: a fan of cones whose apexes
# meet at the curve centre, so surface speed grows with radius and a part
# follows the arc by geometry alone.

import os
import math
import json
import traceback
import Part
import Mesh
from FreeCAD import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "parts")
LOG = os.path.join(HERE, "build.log")

os.makedirs(OUT, exist_ok=True)
_log = open(LOG, "w")


def step(msg):
    _log.write(msg + "\n")
    _log.flush()


def require(cond, msg):
    # A failed clearance must stop the build before anything is exported.
    if not cond:
        step("FAIL: " + msg)
        raise RuntimeError(msg)


# ---------------------------------------------------------------- parameters
# Coordinates: X along s1's travel, Y across it, Z up. Change a number here
# and rerun; parts/geometry.json is the only place a downstream script gets
# a dimension from.

belt_width      = 50.0    # Kyle, 2026-08-08

# Printed TPU 95A loop wall. 1.0, not 1.5, and the reason is the roller shrink:
# belt practice wants pulley-diameter/belt-thickness >= 10, and Ø10 rollers put
# 1.5 mm at D/t = 6.7. Not a cracking risk — 13% outer-fibre strain is nothing
# against TPU's 400%+ elongation — but a stiff belt fights the wrap and lifts off
# a small nose, which is the exact geometry the nose roller exists to protect.
# 1.0 mm gives D/t = 10.0 and is a clean 2-3 perimeters at 0.4 mm nozzle.
belt_thickness  = 1.0

# BOTH ends are small nose rollers, and the DISCHARGE one is driven.
#
# The Ø25 drive roller is gone. It lived at the infeed end, where a concentric
# stadium put its axis bracket_h/2 inboard of the module face. Both ends have
# to be small so a curve can discharge into a straight, which puts the drive
# on a nose roller. One roller part, the motor lands on the side plate, and
# shaft torque drops with the radius — which is what makes a printed D-bore
# at this diameter reasonable.
nose_dia        = 10.0
nose_axle_dia   = 4.0     # idler stub axle on the straights
nose_edge       = 6.0     # module face to nose axis; the belt at the nose is flush with the face
nose_travel     = 8.0     # take-up slot at the INFEED nose (pull the slack back)
roller_flange_d = 13.0    # keeps the belt tracking
roller_flange_w = 1.5

side_gap        = 1.0     # roller end to bracket inner face
wall            = 3.0
inner_width     = belt_width + 2 * roller_flange_w + 2 * side_gap
outer_width     = inner_width + 2 * wall

bracket_h       = 35.0
straight_len    = 120.0
frame_gap       = 1.5     # module face to module face

# goBILDA 1705-0016-0001 enclosure with a ServoCity 638122 inside.
# Reference geometry for renders and clearance checks — never printed.
# The shaft axis is the centre of the 24 x 16 section; ears are symmetric about it.
encl_along          = 32.0   # body length along the shaft
encl_wide           = 24.0   # ear side of the cross-section
encl_narrow         = 16.0   # other side; this is what overhangs a module face
encl_ear_t          = 2.5    # ear plate, flush with the shaft-end face
encl_lobe_d         = 8.0    # ear lobe
encl_ear_hole_d     = 4.0    # ear bolt hole, as printed on the enclosure
encl_ear_pitch      = 16.0   # ear hole to shaft, along the wide side
# The drawing shows the ear plate crossing the shaft end on a 3.5 mm slot, so
# the gearbox face may sit a plate's thickness behind the mounting face. This
# is the worst-case reading; Kyle calipers the real part.
encl_face_to_gearbox = 2.5

motor_shaft_d       = 3.0
motor_boss_d        = 4.0    # gearbox boss the shaft leaves through
motor_boss_len      = 0.6
motor_shaft_past_boss = 8.7  # 638122 drawing: D-shaft beyond the boss
# 8.7 mm past the boss plus the boss itself, measured from the gearbox face.
motor_shaft_len     = motor_shaft_past_boss + motor_boss_len
motor_shaft_flat    = 2.5    # across the D
motor_bore_depth    = 10.0   # deeper than any shaft reaches, so a long shaft
                              # bottoms in the bore instead of walking the roller
spigot_d            = 7.0    # printed nose that crosses the plate / the curve wall
spigot_hole_d       = 8.0
spigot_recess       = 0.5    # spigot stops this short of the enclosure face;
                              # the shaft crosses that air gap before it enters the bore

m4_clear            = 4.6
m4_nut_af           = 7.3    # M4 nut, 7.0 across flats, +0.3 so a printed pocket takes it
m4_nut_depth        = 3.4
nut_land            = 1.2    # plastic left between a nut pocket and the enclosure face
tab_margin          = 1.5    # motor plate covers the upper ear lobe by at least this
coupon_len          = 30.0   # discharge end of the motor plate, a short fit print

# Return guide sits this far BELOW the taut lower run. It exists to catch sag,
# not to bear on a correctly tensioned belt — zero clearance would add drag to
# every module for nothing.
return_clear    = 0.5
# 3.2 printed about 0.15 undersize on radius and gripped an M3. 3.5 leaves a
# sliding fit after that shrink.
m3_clear        = 3.5
m3_locate       = 3.3     # same snug slip as the curve's axle holes
m3_nut_af       = 5.8     # M3 nut 5.5 across flats, +0.3 so the pocket takes it
m3_nut_depth    = 2.6
m3_head_d       = 5.5
m3_head_h       = 3.0
# Lengths are the shank under the head. Each one is the stack it has to cross
# with the tip ending in air, above the table.
m3_tie_len      = 8.0
m3_jack_len     = 16.0
m3_join_len     = 12.0
m3_keep_len     = 10.0
m4_ear_len      = 8.0

TESS = 0.04

# ---- tapered-roller curve -------------------------------------------------
# The lane is the straights' belt, swung about C. θ is CCW from +X:
# −90° is the entry face (the plane x = straight_len + frame_gap, pointing
# −Y from C), 0° is the exit face (the plane y = C_y).
curve_angle     = 90.0
curve_r_in      = 30.0    # lane inner edge from C
curve_r_out     = curve_r_in + belt_width
curve_r_c       = (curve_r_in + curve_r_out) / 2.0
# Cone diameter = k·r. The straight belt moves at ω·(nose_dia+belt_thickness)/2,
# its neutral axis; the cone surface moves at ω·k·r/2. Equal RPM is then equal
# speed on the centreline: Ø6 at the inner edge, Ø16 at the outer.
curve_k         = (nose_dia + belt_thickness) / curve_r_c
curve_alpha     = math.asin(curve_k / 2.0)   # cone half-angle, and the axle tilt down toward the outside
curve_n         = 6
# End rollers tangent to the faces, so the fan occupies (angle − 2α) and the
# pitch is what is left. 6 leaves ~2 mm between cones at the small end;
# 7 leaves 0.8 mm and 8 interferes.
curve_pitch_deg = (curve_angle - 2.0 * math.degrees(curve_alpha)) / (curve_n - 1)
curve_driven    = 3        # middle of the chain, so no O-ring run is longer than 3 links
curve_axle_d    = 3.0      # the 3 mm 304 rod already ordered
# Idler and driven cones both print in TPU 95A. TPU grips steel, so the plain
# bore — the idler's bore on the rod, and the driven cone's small-end stub
# bore — is a looser running fit than the Ø3.4 PLA bore. The coupon confirms
# it. The D-bore keeps the printed-hole allowance, because grip on the shaft
# is what drives the roller.
cone_bore_d     = 3.7
cone_past_lane  = 1.0      # cone runs this far past each lane edge, so a part never sees the end face
stub_bore_depth = 20.0     # driven cone, small end, plain bore for the stub axle
stub_bore_air   = 0.5      # stub stops this short of that bore's bottom
axle_hole_d     = 3.3      # +0.15 on radius over the rod; the machine prints holes undersize, so this is a snug slip
blind_remain    = 1.0      # inner-wall hole stops this short of the far face, along the axis
inner_wall_inset_far = 7.0 # inner wall, 5 mm thick, outside the lane
inner_wall_inset_near = 2.0
outer_wall_gap  = 1.0      # outer wall starts this far past spool_end, in plan radius
outer_wall_t    = 3.0

# O-rings are real parts from a metric kit. The two sizes are PLACEHOLDERS —
# Kyle calipers the ring he picks and changes these. Grooves are cut to suit.
oring_id        = 20.0
oring_cs        = 2.0
oring_stretch   = 0.10     # installed stretch on the centreline; nitrile, lightly loaded, window 0.06–0.15
spool_shoulder  = 2.5      # plan-radius land ahead of groove A and past groove B
spool_groove_gap = 4.0     # plan radius between groove A and groove B
groove_extra    = 0.15     # under a centred cord; the floor stays at the pitch radius minus this and the cord radius
groove_margin   = 0.1      # flanks sit this far past the axial drift
groove_flange   = 0.6      # spool OD at a groove is D + this·cs; the cord then sits just inside the lips
oring_min_bend  = 4.0      # pitch diameter at least this many cord-widths
oring_below_top = 1.0      # ring crown this far below the carry surface
spool_clear_min = 1.0      # neighbouring spools

keeper_t        = 2.5      # radial cover over the rod ends; an M3 head clamps it
keeper_cap_extra = 1.6     # cap half-width past the axle-hole radius, still clear of the pad
pad_rim         = 1.0      # pad extends this past the nut's points and past the enclosure body
pad_bolt_t      = 5.0      # pad is at least this thick at the bolt holes, outboard of the wall
keepout_grow    = 1.0      # frame cut is the s1 enclosure bbox grown by this on every side
encl_check_grow = 0.5      # s1 enclosure, grown by this, must miss the frame
pad_lift_off    = 0.05     # curve enclosure, moved this far off the pad, must miss the frame
engage_min      = 5.0      # D-flat stress check was done at 5 mm of engagement

# The carry surface — the plane the part rides on. Both nose rollers are aligned
# to THIS by their tops, so the carry run is flat and sits on the slider bed.
carry_z = 30.0
nose_z = carry_z - nose_dia / 2.0
z_top = carry_z + belt_thickness

belt_y0 = wall + side_gap + roller_flange_w
belt_y1 = belt_y0 + belt_width

# Entry face is the plane x = straight_len + frame_gap. C sits on that plane,
# one inner radius outboard of s1's belt +Y edge, so that edge is r = curve_r_in
# and the other edge is r = curve_r_out.
cx = straight_len + frame_gap
cy = belt_y1 + curve_r_in
A = Vector(cx, cy, z_top)

r_a = curve_r_in - cone_past_lane
r_b = curve_r_out + cone_past_lane
r_gA = r_b + spool_shoulder
r_gB = r_gA + spool_groove_gap
spool_end_r = r_gB + spool_shoulder

r_iw0 = curve_r_in - inner_wall_inset_far
r_iw1 = curve_r_in - inner_wall_inset_near
r_ow0 = spool_end_r + outer_wall_gap
r_ow1 = r_ow0 + outer_wall_t

alpha_deg = math.degrees(curve_alpha)
thetas = [-90.0 + alpha_deg + i * curve_pitch_deg for i in range(curve_n)]

# s2 travels +Y. rot 90 sends local (x, y) to world (−y + ox, x + oy), and the
# mirror has already put its motor on local +Y, which lands on world −X.
# local y = belt_y1 (the exit's inner radius) maps to cx + curve_r_in.
s2_ox = cx + curve_r_in + belt_y1
s2_oy = cy + frame_gap


def vnorm(v):
    L = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    return Vector(v.x / L, v.y / L, v.z / L)


def vcross(a, b):
    return Vector(a.y * b.z - a.z * b.y,
                  a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x)


def vdot(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z


def vmul(a, s):
    return Vector(a.x * s, a.y * s, a.z * s)


def vadd(a, b):
    return Vector(a.x + b.x, a.y + b.y, a.z + b.z)


def vsub(a, b):
    return Vector(a.x - b.x, a.y - b.y, a.z - b.z)


def axis_u(theta_deg):
    th = math.radians(theta_deg)
    ca = math.cos(curve_alpha)
    sa = math.sin(curve_alpha)
    return Vector(math.cos(th) * ca, math.sin(th) * ca, -sa)


def axis_frame(theta_deg):
    # ê_θ horizontal, ê_up = u × ê_θ (mostly +Z, a little outward).
    th = math.radians(theta_deg)
    u = axis_u(theta_deg)
    e_th = Vector(-math.sin(th), math.cos(th), 0.0)
    e_up = vcross(u, e_th)
    return u, e_th, vnorm(e_up)


def apply_frame(shape, origin, ax, ay, az):
    m = Matrix()
    m.A11, m.A12, m.A13, m.A14 = ax.x, ay.x, az.x, origin.x
    m.A21, m.A22, m.A23, m.A24 = ax.y, ay.y, az.y, origin.y
    m.A31, m.A32, m.A33, m.A34 = ax.z, ay.z, az.z, origin.z
    m.A44 = 1.0
    out = shape.copy()
    out.transformShape(m)
    return out


def s_on_cylinder(radius, h, v):
    # Axis-parameter where the ray parallel to u, offset (h, v) in (ê_θ, ê_up),
    # meets a vertical cylinder of this radius about C. The outward hit.
    rad = radius * radius - h * h
    if rad <= 0.0:
        require(False, "ray h=%.2f misses cylinder r=%.2f" % (h, radius))
    return (math.sqrt(rad) - v * math.sin(curve_alpha)) / math.cos(curve_alpha)


def roller_axis_x(module_len):
    return nose_edge, module_len - nose_edge


def straight_driven_stub_ys():
    # Pre-mirror frame, the same placement the driven roller gets. The rod
    # sits on the blind-bore floor and stops stub_bore_air short of the step
    # where the D-bore ends, which is what keeps it from entering that bore.
    y_tip = -(wall + side_gap - spigot_recess)
    y_step = (wall + side_gap) + y_tip + motor_bore_depth
    bore_depth = wall + stub_boss_out - stub_floor
    y_out = (inner_width + wall) + bore_depth
    return y_step + stub_bore_air, y_out


def belt_path_length(module_len):
    # Measured at the belt's NEUTRAL AXIS (nose_dia + belt_thickness). That fibre
    # neither stretches nor compresses, so it is the length a printed loop's mean
    # circumference has to match. The roller surface undersizes every belt by
    # pi x thickness, against only nose_travel of take-up to absorb it.
    ax0, ax1 = roller_axis_x(module_len)
    return 2.0 * (ax1 - ax0) + math.pi * (nose_dia + belt_thickness)


def printed_cylinder_dia(module_len):
    return belt_path_length(module_len) / math.pi


def return_run_z():
    # Outer surface of the TAUT lower run. Derived: it moves with nose_dia and
    # belt_thickness, both of which have already changed once in this build.
    return nose_z - nose_dia / 2.0 - belt_thickness


def shaft_engagement():
    # The shaft reaches (motor_shaft_len − encl_face_to_gearbox) past the
    # mounting face, then loses spigot_recess of air before the bore starts.
    # The bore is deeper than that on purpose; engagement is the shaft's reach.
    return min(motor_shaft_len - encl_face_to_gearbox - spigot_recess, motor_bore_depth)


def transfer_span(r):
    # Nose setback + frame gap + how far the end cone's top line sits inboard
    # of the face. That inboard distance is r·sin α because the end roller is
    # placed α off the face so its surface is tangent to the plane.
    return nose_edge + frame_gap + r * math.sin(curve_alpha)


def hex_Rv(af):
    return (af / 2.0) / math.cos(math.pi / 6.0)


def groove_pitch_D(c_g, oid):
    # Belt length is two straight runs plus two half-wraps. The 15.7° skew
    # between neighbouring axles is ignored; the ring twists that little.
    free = math.pi * (oid + oring_cs)
    installed = free * (1.0 + oring_stretch)
    return (installed - 2.0 * c_g) / math.pi


def id_for_D(c_g, D):
    return (D + 2.0 * c_g / math.pi) / (1.0 + oring_stretch) - oring_cs


# --------------------------------------------------------------- vector layout
ca = math.cos(curve_alpha)
sa = math.sin(curve_alpha)
pitch_rad = math.radians(curve_pitch_deg)
# |u_i − u_{i+1}|. Axes share the downward tilt, so the angle between them is
# not the plan pitch; the separation of two points at the same s is s times this.
axis_sep = 2.0 * ca * math.sin(pitch_rad / 2.0)

r_iw0_s = r_iw0 / ca          # axis parameter where the axis meets that cylinder
r_iw1_s = r_iw1 / ca
r_ow0_s = r_ow0 / ca
r_ow1_s = r_ow1 / ca
s_a = r_a * ca
s_b = r_b * ca
s_gA = r_gA * ca
s_gB = r_gB * ca
s_spool_end = spool_end_r * ca
s_hole_bottom = r_iw0_s + blind_remain
# The end is square and the axle tilts, so the rim — not the centre — is what
# meets the keeper. Seat the inner end on the blind hole and stop the rim
# 0.2 mm inside the keeper's inner face.
s_idler_end = (r_ow1 - 0.2 - (curve_axle_d / 2.0) * sa) / ca
idler_rod_len = s_idler_end - s_hole_bottom
stub_len = (s_a + stub_bore_depth - stub_bore_air) - s_hole_bottom

c_A = 2.0 * r_gA * ca * ca * math.sin(pitch_rad / 2.0)
c_B = 2.0 * r_gB * ca * ca * math.sin(pitch_rad / 2.0)
D_A = groove_pitch_D(c_A, oring_id)
D_B = groove_pitch_D(c_B, oring_id)
R_spool_A = D_A / 2.0 + groove_flange * oring_cs / 2.0
R_spool_B = D_B / 2.0 + groove_flange * oring_cs / 2.0
s_mid = 0.5 * (s_gA + s_gB)


def ring_clearance(r_g, D):
    # Crown height: axis height at the groove, plus pitch radius, plus cord radius.
    z_axis = z_top - r_g * sa * ca
    return z_top - (z_axis + D / 2.0 + oring_cs / 2.0)


def spool_clearance(s, R):
    return s * axis_sep - 2.0 * R


clear_A = ring_clearance(r_gA, D_A)
clear_B = ring_clearance(r_gB, D_B)
small_end_clear = spool_clearance(s_a, s_a * math.tan(curve_alpha))
spool_clear_A = spool_clearance(s_b, R_spool_A)
spool_clear_B = spool_clearance(s_mid, R_spool_B)

# id window for this cord, so a kit pick can be checked without rerunning blind.
def passing_id_window():
    lo = []
    hi = []
    for c_g, r_g, s_region in ((c_A, r_gA, s_b), (c_B, r_gB, s_mid)):
        lo.append(id_for_D(c_g, oring_min_bend * oring_cs))
        d_max_crown = 2.0 * (r_g * sa * ca - oring_below_top) - oring_cs
        hi.append(id_for_D(c_g, d_max_crown))
        d_max_spool = s_region * axis_sep - groove_flange * oring_cs - spool_clear_min
        hi.append(id_for_D(c_g, d_max_spool))
    return max(lo), min(hi)


id_lo, id_hi = passing_id_window()


def _dist_roller(s, rho):
    # Distance from an axis-frame point to the roller before the groove is cut.
    def cyl(s0, s1, radius):
        if s0 <= s <= s1:
            return max(0.0, rho - radius)
        end = s0 if s < s0 else s1
        if rho <= radius:
            return abs(s - end)
        return math.hypot(s - end, rho - radius)

    if s_a <= s <= s_b:
        cone = max(0.0, rho - s * math.tan(curve_alpha))
    elif s > s_b:
        rend = s_b * math.tan(curve_alpha)
        cone = (s - s_b) if rho <= rend else math.hypot(s - s_b, rho - rend)
    else:
        r0 = s_a * math.tan(curve_alpha)
        cone = (s_a - s) if rho <= r0 else math.hypot(s_a - s, rho - r0)
    return min(cone,
               cyl(s_b - 0.15, s_mid, R_spool_A),
               cyl(s_mid - 0.05, s_spool_end, R_spool_B))


def _off_circle(point, centre, axis, radius):
    rel = vsub(point, centre)
    h = vdot(rel, axis)
    radial = vsub(rel, vmul(axis, h))
    rho = math.sqrt(max(0.0, vdot(radial, radial)))
    return math.sqrt(h * h + (rho - radius) ** 2), vdot(point, axis), rho


def wrap_rim_offset(theta_here, theta_other, s_g, radius):
    # Fleet is axial: the tangent leaves the groove plane at sin(φ/2) per mm.
    # The centre clears the rim radially before that axial offset can become
    # the full 3D miss, so the number that widens the flanks is the axial one.
    u1 = axis_u(theta_here)
    u2 = axis_u(theta_other)
    c1 = vmul(u1, s_g)
    c2 = vmul(u2, s_g)
    w = vnorm(vcross(u1, u2))
    direction = vnorm(vsub(c2, c1))
    span = vsub(c2, c1).Length

    def at(d, sign):
        point = vadd(vadd(c1, vmul(w, sign * radius)), vmul(direction, d))
        _delta, s, rho = _off_circle(point, c1, u1, radius)
        h = vdot(vsub(point, c1), u1)
        inside = _dist_roller(s, rho) <= 1e-9
        return inside, abs(h), rho - radius

    axial, radial = 0.0, 0.0
    for sign in (1.0, -1.0):
        if not at(0.0, sign)[0]:
            continue
        lo, hi = 0.0, span
        if not at(span, sign)[0]:
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                if at(mid, sign)[0]:
                    lo = mid
                else:
                    hi = mid
        else:
            lo = span
        _inside, h, dr = at(lo, sign)
        if h > axial:
            axial, radial = h, dr
    return axial, radial


def seated_stretch(theta_a, theta_b, s_g, radius):
    # Centreline on the pitch circle through each wrap, and the real tangent
    # between those two circles. That is the path tension actually takes.
    u1 = axis_u(theta_a)
    u2 = axis_u(theta_b)
    c1 = vmul(u1, s_g)
    c2 = vmul(u2, s_g)
    w = vnorm(vcross(u1, u2))
    span = vsub(vadd(c2, vmul(w, radius)), vadd(c1, vmul(w, radius))).Length
    installed = 2.0 * span + 2.0 * math.pi * radius
    free = math.pi * (oring_id + oring_cs)
    return installed / free - 1.0


def wrap_deviation_rows():
    rows = []
    worst = 0.0
    for i in range(curve_n - 1):
        groove = "A" if (i + 1) % 2 == 1 else "B"
        r_g = r_gA if groove == "A" else r_gB
        D = D_A if groove == "A" else D_B
        s_g = r_g * ca
        radius = D / 2.0
        h_a, dr_a = wrap_rim_offset(thetas[i], thetas[i + 1], s_g, radius)
        h_b, dr_b = wrap_rim_offset(thetas[i + 1], thetas[i], s_g, radius)
        stretch = seated_stretch(thetas[i], thetas[i + 1], s_g, radius)
        rows.append((i + 1, i + 2, groove, radius, h_a, dr_a, h_b, dr_b, stretch))
        worst = max(worst, h_a, h_b)
    return rows, worst


wrap_rows, wrap_axial = wrap_deviation_rows()
# Floor stays put. The original circular section is swept along the axis.
groove_section_r = oring_cs / 2.0 + groove_extra
groove_axial = wrap_axial + groove_margin

# Curve enclosure: 16 mm side vertical, 24 mm side horizontal, ears ±pitch.
u_drv, e_th_drv, e_up_drv = axis_frame(thetas[curve_driven - 1])
s_face = s_on_cylinder(r_ow1, encl_ear_pitch, 0.0) + pad_bolt_t
# An M4×8 crosses the 2.5 mm ear and a 3.4 mm nut and still has to come out
# the back. That leaves 2.1 mm, so the land is nut_land and the rest is the
# tip past the nut. Deeper than that, the same screw ends short of the pocket.
s_nut_near = s_face - nut_land
s_nut_far = s_nut_near - m4_nut_depth
m4_Rv = hex_Rv(m4_nut_af)
# Worst corner of the hex, so the pocket face is entirely in the gutter and
# not buried in the curved inner wall.
s_boss_in = s_on_cylinder(r_ow0, encl_ear_pitch + m4_Rv, m4_Rv) - 0.4
s_slab_in = r_ow0_s - 0.8
h_half = encl_ear_pitch + m4_Rv + pad_rim
v_half = encl_narrow / 2.0 + pad_rim
boss_r_curve = m4_Rv + 1.2

# Straight mount bosses. The plate is `wall` thick; the pocket needs depth + land.
boss_extra = m4_nut_depth + nut_land - wall
boss_y1 = wall + boss_extra
boss_r_straight = m4_Rv + 1.0
lobe_r = encl_lobe_d / 2.0
_, nose_x = roller_axis_x(straight_len)
tab_x0 = nose_x - (lobe_r + tab_margin)
tab_x1 = min(nose_x + (lobe_r + tab_margin), straight_len)
tab_z1 = nose_z + encl_ear_pitch + lobe_r + tab_margin

# motor_tab is the straight's local frame AFTER the mirror about y = outer_width/2.
# It bounds the tab and the upper boss: what stands above the plate top or
# inboard of the motor plate. The lower boss is below the belt; it is logged,
# and it is not part of this rail.
motor_tab = {
    "x0": tab_x0,
    "x1": tab_x1,
    "y0": outer_width - boss_y1,
    "y1": outer_width,
    "z0": bracket_h,
    "z1": tab_z1,
}

# --- straight structure ----------------------------------------------------
# The bed stops this clear of the metal of a roller. The old bed ran axis to
# axis and the fuse hid the collision.
bed_gap         = 0.5
# Printed tongues come out fat. This is the same order as the bore allowance,
# so a tongue still enters its groove and the groove wall is what stops belt drag.
fit_gap         = 0.15
# Modelled air under a face that really touches (belt on the bed). A shared
# face makes the interference boolean report a volume for a contact.
bed_standoff    = 0.05
join_standoff   = 0.05    # same idea between the joiner and the tie bar / pad
tongue_d        = 1.2     # into the 3 mm plate; leaves a web, and never reaches the belt
tongue_h        = 1.4
# Joiner top has to stay 2 mm under the return run, and the cones over the
# outer hole bottom out near z 16, so the joint plane sits at 10.
z_j             = 10.0
joiner_t        = 3.0
tie_t           = 8.0     # tall enough for an M3 nut across the bolt
tie_half        = 6.0
tie_inset       = 18.0    # bolt centre; the whole bar then sits inside 25 mm of the face
# and clear of the lower ear boss, which starts near x = nose_x − boss radius
tie_bolt_z      = 6.0
hole_pitch      = 16.0    # joiner holes, each side of the lane centre
# Two nuts. One M3 nut is 2.4 mm thick, and the jack has to keep 3 mm of
# thread at both ends of an 8 mm travel.
jack_nut_n      = 2
jack_nut_depth  = jack_nut_n * m3_nut_depth
block_out       = 6.0     # outboard of the plate; the 10 mm limit is the head's room
block_back      = 5.0     # from the idler axis toward the module face
block_front     = 6.0     # inboard of the axis; the face the screw pushes
rail_h          = 2.5
web             = 5.0     # plastic above a joiner nut, so an M3×12 ends above the table
pad_margin      = 8.0     # joint pad keeps this much plate around a hole

bed_top         = carry_z - bed_standoff
bed_bot         = bed_top - wall
lane_y          = 0.5 * (belt_y0 + belt_y1)
tie_z0          = z_j - tie_t
join_span       = tie_inset + frame_gap + tie_inset   # hole to hole along travel
y_loc_pre       = lane_y + hole_pitch                 # locating hole, before the mirror
y_clr_pre       = lane_y - hole_pitch
x_tie_in        = tie_inset
x_tie_out       = straight_len - tie_inset
idler_axle_len  = outer_width + 2.0 * block_out
# Driven stub, plain plate. A through hole lets the rod walk out, so the bore
# is blind from the inner face and the floor is the retainer. The boss is what
# makes the bore deeper than the 3 mm wall while keeping that floor.
stub_floor      = 1.2
stub_boss_out   = 2.5     # within the 10 mm outboard limit; bore engagement is then 4.3 mm
stub_boss_r     = 5.0     # axis is nose_edge in from the discharge face, so 5 stays inside it
stub_hole_r     = nose_axle_dia / 2.0 + 0.15  # plate locates the rod; the roller bore stays looser so it turns


def main():
    step("=== build start ===")
    require(abs(curve_angle - 90.0) < 1e-9,
            "curve_angle is %.3f; s2's exit face is the plane y=C_y only for a right angle"
            % curve_angle)
    require(curve_driven >= 1 and curve_driven <= curve_n, "curve_driven out of range")

    step("layout: C=(%.3f, %.3f) z_top=%.3f belt y %.3f..%.3f" % (cx, cy, z_top, belt_y0, belt_y1))
    step("layout: s2 rot=90 offset=(%.3f, %.3f) entry_face_x=%.3f exit_face_y=%.3f"
         % (s2_ox, s2_oy, cx, cy))
    step("curve: k=%.5f alpha=%.4f deg pitch=%.4f deg n=%d driven=%d"
         % (curve_k, alpha_deg, curve_pitch_deg, curve_n, curve_driven))
    step("cone_bore_d %.2f mm" % cone_bore_d)
    step("curve theta deg: %s" % ", ".join("%.4f" % t for t in thetas))
    step("cone plan r %.3f..%.3f  spool grooves %.3f %.3f end %.3f"
         % (r_a, r_b, r_gA, r_gB, spool_end_r))
    step("walls inner %.3f..%.3f outer %.3f..%.3f" % (r_iw0, r_iw1, r_ow0, r_ow1))
    step("spans mm: inner %.3f centre %.3f outer %.3f (entry and exit)"
         % (transfer_span(curve_r_in), transfer_span(curve_r_c), transfer_span(curve_r_out)))
    overhang = encl_narrow / 2.0 - nose_edge
    step("straight enclosure overhang past the discharge face: %.3f mm" % overhang)

    step("oring A: c=%.3f D=%.3f crown_clear=%.3f stretch=%.3f"
         % (c_A, D_A, clear_A, oring_stretch))
    step("oring B: c=%.3f D=%.3f crown_clear=%.3f stretch=%.3f"
         % (c_B, D_B, clear_B, oring_stretch))
    step("oring id window at cs=%.2f: %.3f .. %.3f mm" % (oring_cs, id_lo, id_hi))
    for n1, n2, groove, radius, h_a, dr_a, h_b, dr_b, stretch in wrap_rows:
        step("oring %d-%d groove %s  axial drift %.3f / %.3f mm  radial at rim %.3f"
             % (n1, n2, groove, h_a, h_b, max(dr_a, dr_b)))
        step("oring %d-%d  pitch r %.3f  seat r %.3f  stretch %.4f"
             % (n1, n2, radius, radius - groove_section_r, stretch))
        require(abs(stretch - oring_stretch) <= 0.01,
                "oring %d-%d stretch %.4f is outside %.2f ± 0.01"
                % (n1, n2, stretch, oring_stretch))
    step("groove section r %.3f mm  axial sweep +/- %.3f mm  floor at pitch r - %.3f"
         % (groove_section_r, groove_axial, groove_section_r))
    shoulder = min(s_gA - (s_b - 0.15), s_mid - s_gA,
                   s_gB - (s_mid - 0.05), s_spool_end - s_gB)
    flank = groove_axial + groove_section_r
    require(flank < shoulder,
            "groove flank (%.3f mm) breaks the spool shoulder (%.3f)"
            % (flank, shoulder))
    bore_room = min(D_A, D_B) / 2.0 - cone_bore_d / 2.0
    require(groove_section_r < bore_room,
            "groove floor (section %.3f) reaches the bore (room %.3f)"
            % (groove_section_r, bore_room))
    step("spool OD radius A=%.3f B=%.3f" % (R_spool_A, R_spool_B))
    step("clearance small-end cones %.3f mm; spool A at s_b %.3f; spool B at mid %.3f"
         % (small_end_clear, spool_clear_A, spool_clear_B))
    step("rod cut mm: idler %.3f  driven stub %.3f" % (idler_rod_len, stub_len))
    idler_rim_r = s_idler_end * ca + (curve_axle_d / 2.0) * sa
    idler_rim_in = r_ow1 - idler_rim_r
    step("curve idler end rim %.3f mm inside the keeper" % idler_rim_in)
    require(idler_rim_in >= 0.2 - 1e-9,
            "curve idler rim is %.3f mm inside the keeper, need 0.2" % idler_rim_in)
    step("shaft engagement %.3f mm (bore %.1f)" % (shaft_engagement(), motor_bore_depth))
    step("straight bosses: protrusion %.3f mm, inboard face y=%.3f after mirror (both ears)"
         % (boss_extra, outer_width - boss_y1))
    step("pad: s_face=%.3f s_slab_in=%.3f s_boss_in=%.3f h_half=%.3f v_half=%.3f"
         % (s_face, s_slab_in, s_boss_in, h_half, v_half))

    require(id_lo < id_hi, "no oring_id passes at cs=%.2f" % oring_cs)
    require(id_lo - 1e-6 <= oring_id <= id_hi + 1e-6,
            "oring_id %.2f outside %.3f..%.3f" % (oring_id, id_lo, id_hi))
    require(D_A >= oring_min_bend * oring_cs and D_B >= oring_min_bend * oring_cs,
            "pitch diameter below %d·cs (A %.3f B %.3f); id window %.3f..%.3f"
            % (int(oring_min_bend), D_A, D_B, id_lo, id_hi))
    require(clear_A >= oring_below_top and clear_B >= oring_below_top,
            "ring crown clearance A %.3f B %.3f, need >= %.1f; id window %.3f..%.3f"
            % (clear_A, clear_B, oring_below_top, id_lo, id_hi))
    require(spool_clear_A >= spool_clear_min and spool_clear_B >= spool_clear_min,
            "spool clearance A %.3f B %.3f, need >= %.1f" % (spool_clear_A, spool_clear_B, spool_clear_min))
    require(small_end_clear > 0.2,
            "cone small-end clearance %.3f mm; rollers intersect" % small_end_clear)
    require(shaft_engagement() >= engage_min,
            "shaft engagement %.3f < %.1f" % (shaft_engagement(), engage_min))
    require(s_face - spigot_recess > s_spool_end + 1.0,
            "spigot has no room between spool end and pad face")
    require(s_a + stub_bore_depth < s_gA - motor_bore_depth,
            "driven cone bores would meet")
    require(tab_x1 <= straight_len + 1e-9 and tab_x0 >= 0.0,
            "motor tab leaves the module length")
    lower_boss_z1 = nose_z - encl_ear_pitch + boss_r_straight
    upper_boss_z0 = nose_z + encl_ear_pitch - boss_r_straight
    require(lower_boss_z1 < return_run_z(),
            "lower boss top z=%.3f meets the return run at %.3f" % (lower_boss_z1, return_run_z()))
    require(upper_boss_z0 > bracket_h - 1e-6,
            "upper boss dips into the plate (z0=%.3f, plate top %.1f)" % (upper_boss_z0, bracket_h))
    require(nose_x + boss_r_straight <= straight_len and tab_x1 <= straight_len,
            "tab or boss crosses x=module_len")

    # Crown of the enclosure footprint must sit >= 1 mm outside the outer wall.
    face_r_min = s_face * ca + (-encl_narrow / 2.0) * sa
    require(face_r_min >= r_ow1 + 1.0 - 1e-6,
            "pad face plan radius %.3f is not 1 mm outboard of r=%.3f" % (face_r_min, r_ow1))
    # Nut pocket leaves land behind it.
    land_curve = s_face - s_nut_near
    require(land_curve >= nut_land - 1e-9,
            "curve nut pocket leaves %.3f mm, need %.1f" % (land_curve, nut_land))
    step("curve nut land behind pocket: %.3f mm" % land_curve)

    # Blind-hole bottom disk must not break the curved far face.
    # The disk's most inward point moves r_hole·sin α in plan.
    hole_r = axle_hole_d / 2.0
    bottom_plan_r = s_hole_bottom * ca - hole_r * sa
    require(bottom_plan_r > r_iw0,
            "blind hole breaks the inner face (plan r %.3f vs wall %.3f)" % (bottom_plan_r, r_iw0))
    step("blind-hole remaining plan thickness %.3f mm" % (bottom_plan_r - r_iw0))

    # Keeper cap must cover the idler hole and still clear the pad.
    cap_half_mm = hole_r + keeper_cap_extra
    neighbour_gap = r_ow1 * 2.0 * math.sin(pitch_rad / 2.0) - h_half - cap_half_mm
    require(neighbour_gap >= 0.8,
            "keeper cap meets the pad (gap %.3f mm)" % neighbour_gap)
    step("keeper cap half-width %.3f mm, gap to pad %.3f mm" % (cap_half_mm, neighbour_gap))

    step("--- solids ---")
    # Placement probe: FreeCAD's rotate must match the axis formula, big end down.
    probe = Part.makeLine(Vector(s_b, 0, 0), Vector(s_b + 0.2, 0, 0))
    probe.rotate(Vector(0, 0, 0), Vector(0, 1, 0), alpha_deg)
    probe.rotate(Vector(0, 0, 0), Vector(0, 0, 1), thetas[0])
    probe.translate(A)
    got = probe.Vertexes[0].Point
    exp = vadd(A, vmul(axis_u(thetas[0]), s_b))
    require((got - exp).Length < 1e-4,
            "roller placement mismatch %.4f mm" % (got - exp).Length)

    step("bracket + straight rollers")
    br_motor = make_bracket(straight_len, motor_side=True)
    br_plain = make_bracket(straight_len, motor_side=False, belt_side=-1)
    require(x_tie_out + tie_half < nose_x - boss_r_straight - 0.4,
            "discharge tie bar meets the lower ear boss")
    require(x_tie_in - tie_half >= 0.0 and x_tie_in + tie_half <= 25.0,
            "infeed tie bar is not within 25 mm of the face")
    require(straight_len - (x_tie_out + tie_half) <= 25.0
            and straight_len - (x_tie_out - tie_half) <= 25.0,
            "discharge tie bar is not within 25 mm of the face")
    tip = nose_edge + block_front
    nut_a = tip + nose_travel
    nut_b = nut_a + jack_nut_depth

    def jack_engage(tip_x):
        s0, s1 = tip_x, tip_x + m3_jack_len
        return max(0.0, min(s1, nut_b) - max(s0, nut_a))

    e_tight, e_slack = jack_engage(tip), jack_engage(tip + nose_travel)
    step("jack-screw engagement tensioned %.2f mm, slack %.2f mm" % (e_tight, e_slack))
    dx_in = bed_dx(roller_flange_d / 2.0)
    step("take-up %.1f mm, belt slack %.1f mm, unsupported carry %.3f mm"
         % (nose_travel, 2.0 * nose_travel, nose_travel + dx_in))
    step("straight idler rod cut %.3f mm" % idler_axle_len)
    y_stub_in, y_stub_out = straight_driven_stub_ys()
    stub_cut = y_stub_out - y_stub_in
    step("straight driven stub cut %.3f mm" % stub_cut)
    require(stub_floor >= 1.2, "driven stub floor %.2f mm is under 1.2" % stub_floor)
    require(stub_boss_out <= 10.0, "driven stub boss sticks out %.2f mm" % stub_boss_out)
    require(stub_boss_r <= nose_edge,
            "driven stub boss crosses the discharge face")
    require((straight_len - nose_edge) - stub_boss_r >= 0.0,
            "driven stub boss crosses x=0")
    require(stub_cut > 0.0, "driven stub length %.3f mm" % stub_cut)
    require(e_tight >= 3.0 and e_slack >= 3.0,
            "jack-screw engagement %.2f / %.2f mm, need >= 3 at both ends of travel"
            % (e_tight, e_slack))
    require(abs(nose_edge - roller_axis_x(straight_len)[0]) < 1e-9,
            "tensioned idler axis is not at nose_edge")
    require(br_motor.BoundBox.XMax <= straight_len + 1e-6,
            "motor bracket XMax %.3f exceeds module length" % br_motor.BoundBox.XMax)
    require(br_motor.BoundBox.ZMax >= tab_z1 - 0.05,
            "tab did not reach z=%.2f (got %.2f)" % (tab_z1, br_motor.BoundBox.ZMax))
    rol_id = make_roller(False)
    rol_dr = make_roller(True)
    bed = make_slider_bed(straight_len)
    ret = make_return_guide(straight_len)
    belt = make_belt(straight_len)

    step("cone rollers")
    cone_id = make_cone_roller(False)
    cone_dr = make_cone_roller(True)
    placed = []
    for i, th in enumerate(thetas):
        src = cone_dr if (i + 1) == curve_driven else cone_id
        placed.append(place_cone(src, th))
    # The axis sits α off the face, so the top line — what a part rides — is
    # tangent to the plane. The axle also tilts down, and that puts the cone's
    # lower flank about 0.04 mm past a vertical plane through the apex. Shave
    # the end rollers flush. The axes stay where the pitch formula put them.
    entry_cut = Part.makeBox(400, 800, 160, Vector(cx - 400, cy - 400, -40))
    exit_cut = Part.makeBox(800, 400, 160, Vector(cx - 200, cy, -40))
    for i, sol in enumerate(placed):
        bb = sol.BoundBox
        if bb.XMin < cx - 1e-6:
            step("roller %d entry lip %.4f mm, shaved flush" % (i + 1, cx - bb.XMin))
            sol = sol.cut(entry_cut)
        if bb.YMax > cy + 1e-6:
            step("roller %d exit lip %.4f mm, shaved flush" % (i + 1, bb.YMax - cy))
            sol = sol.cut(exit_cut)
        placed[i] = sol
        bb = sol.BoundBox
        require(bb.ZMax <= z_top + 0.02,
                "roller %d rises to z=%.3f, z_top=%.3f" % (i + 1, bb.ZMax, z_top))
        require(bb.ZMax >= z_top - 0.05,
                "roller %d top is z=%.3f, expected the carry plane" % (i + 1, bb.ZMax))
        require(bb.XMin >= cx - 1e-4,
                "roller %d crosses the entry plane (x=%.4f)" % (i + 1, bb.XMin))
        require(bb.YMax <= cy + 1e-4,
                "roller %d crosses the exit plane (y=%.4f)" % (i + 1, bb.YMax))

    step("enclosures")
    encl_straight = make_motor_enclosure("z")
    encl_straight.translate(Vector(nose_x, 0, nose_z))
    encl_curve = make_motor_enclosure("x")
    encl_curve = apply_frame(encl_curve, vadd(A, vmul(u_drv, s_face)),
                              e_th_drv, vmul(u_drv, -1.0), e_up_drv)

    step("curve frame")
    frame = add_joint_pads(make_curve_frame())
    require(frame.BoundBox.XMin >= cx - 0.05,
            "frame crosses the entry plane")
    require(frame.BoundBox.YMax <= cy + 0.05,
            "frame crosses the exit plane")

    step("keeper")
    keeper = make_keeper(cap_half_mm)
    # Bolt holes through the keeper and the outer wall, midway between idlers
    # at the two ends — the gutter there is below the spools.
    bolt_angles = [0.5 * (thetas[0] + thetas[1]), 0.5 * (thetas[4] + thetas[5])]
    z_bolt = 0.5 * (wall + 1.0 + 10.0)
    for ang in bolt_angles:
        # Boss first, then the clearance hole, so the boss cannot plug the hole.
        frame = add_keeper_nut(frame, ang, z_bolt)
        frame = frame.cut(keeper_top_slot(ang, z_bolt))
        cutter = radial_hole(ang, z_bolt, m3_clear / 2.0, r_ow0 - 0.05,
                             r_ow1 + keeper_t + 1.0)
        frame = frame.cut(cutter)
        keeper = keeper.cut(cutter)
        step("keeper bolt at theta %.3f deg, z=%.2f" % (ang, z_bolt))

    step("s1 enclosure vs frame")
    d_s1 = encl_straight_world(encl_straight).distToShape(frame)[0]
    step("s1 enclosure distance to curve frame: %.3f mm (need >= %.2f)" % (d_s1, encl_check_grow))
    require(d_s1 >= encl_check_grow - 1e-6,
            "s1 enclosure (grown %.2f) meets the curve frame, dist %.3f" % (encl_check_grow, d_s1))

    step("curve enclosure vs frame, lifted off the pad")
    lifted = encl_curve.copy()
    lifted.translate(vmul(u_drv, pad_lift_off))
    d_cv = lifted.distToShape(frame)[0]
    step("curve enclosure, %.3f mm off the pad, distance %.3f mm" % (pad_lift_off, d_cv))
    require(d_cv > 0.01,
            "curve enclosure intersects the frame away from the pad face (dist %.4f)" % d_cv)

    step("rollers vs frame")
    for i, sol in enumerate(placed):
        d = sol.distToShape(frame)[0]
        step("roller %d distance to frame %.3f mm" % (i + 1, d))
        require(d > 0.05, "roller %d intersects the frame (dist %.4f)" % (i + 1, d))

    step("o-rings")
    links = []
    link_spec = []
    for i in range(curve_n - 1):
        groove = "A" if (i + 1) % 2 == 1 else "B"
        r_g = r_gA if groove == "A" else r_gB
        D = D_A if groove == "A" else D_B
        link = make_oring_link(thetas[i], thetas[i + 1], r_g, D)
        bb = link.BoundBox
        step("oring %d-%d groove %s zmax %.3f" % (i + 1, i + 2, groove, bb.ZMax))
        require(bb.ZMax <= z_top - 0.5,
                "oring %d-%d reaches z=%.3f" % (i + 1, i + 2, bb.ZMax))
        links.append(link)
        link_spec.append((i, groove, link))
    real_orings = []
    for i in range(curve_n - 1):
        groove = "A" if (i + 1) % 2 == 1 else "B"
        r_g = r_gA if groove == "A" else r_gB
        D = D_A if groove == "A" else D_B
        solid = make_oring_real(thetas[i], thetas[i + 1], r_g, D)
        real_orings.append(solid)
        step("oring real %d-%d volume %.0f mm^3" % (i + 1, i + 2, abs(solid.Volume)))
        require(abs(solid.Volume) > 10.0, "oring %d-%d path did not build" % (i + 1, i + 2))

    # ---------------------------------------------------------------- placed line
    step("--- placed line ---")
    ax0, nose_ax = roller_axis_x(straight_len)
    ry = wall + side_gap
    plain = br_plain.translated(Vector(0, inner_width + wall, 0))
    tie_in = make_tie_bar(x_tie_in)
    tie_out = make_tie_bar(x_tie_out)
    block = make_tensioner_block()
    block_far = mirror_left(block)
    rod = Part.makeCylinder(nose_axle_dia / 2.0, idler_axle_len,
                             Vector(nose_edge, -block_out, nose_z), Vector(0, 1, 0))
    # Seated on the bore floor, stub_bore_air short of the D-bore step.
    # Same frame as the plates, so place_module carries it with them.
    drv_stub = Part.makeCylinder(nose_axle_dia / 2.0, stub_cut,
                                 Vector(nose_ax, y_stub_in, nose_z), Vector(0, 1, 0))
    rol_id_p = rol_id.translated(Vector(ax0, ry, nose_z))
    rol_dr_p = rol_dr.translated(Vector(nose_ax, ry, nose_z))
    belt_p = belt.translated(Vector(0, ry + roller_flange_w, 0))
    fasteners = straight_fasteners()
    local_parts = [
        ("plate_motor", br_motor), ("plate_plain", plain),
        ("roller_idler", rol_id_p), ("roller_driven", rol_dr_p),
        ("bed", bed), ("guide", ret), ("belt", belt_p),
        ("tie_in", tie_in), ("tie_out", tie_out),
        ("block_near", block), ("block_far", block_far),
        ("rod", rod), ("driven_stub", drv_stub), ("motor", encl_straight),
    ] + fasteners

    moving_names = {"roller_idler", "block_near", "block_far", "rod",
                    "scr_jack", "scr_jack_far"}
    s2_off = Vector(s2_ox, s2_oy, 0)

    def world_at(shift):
        # Roller, rod, blocks and the jack screws walk together. The screw is
        # what pushes the block, so a slack check with the screw left behind
        # buries the screw in the block.
        def maybe(name, shape):
            if shift and name in moving_names:
                moved = shape.copy()
                moved.translate(Vector(shift, 0, 0))
                return moved
            return shape

        def placed_straight(tag, rot, offset):
            out = []
            for name, shape in local_parts:
                out.append((tag + "_" + name, place_module(maybe(name, shape), rot, offset)))
            return out

        w = placed_straight("s1", 0.0, None) + placed_straight("s2", 90.0, s2_off)
        for i, sol in enumerate(placed):
            w.append(("cone_%d" % (i + 1), sol))
        for i, link in enumerate(real_orings):
            w.append(("oringR_%d" % (i + 1), link))
        w.append(("frame", frame))
        w.append(("keeper", keeper))
        w.append(("curve_motor", encl_curve))
        for i, th in enumerate(thetas):
            if (i + 1) == curve_driven:
                w.append(("stub", curve_stub_rod(th)))
            else:
                w.append(("crod_%d" % (i + 1), curve_idler_rod(th)))
        for ang in bolt_angles:
            w.append(("keep_screw", keeper_screw(ang, z_bolt)))
        for sign in (-1.0, 1.0):
            w.append(("cv_m4", curve_ear_screw(sign)))
        w.append(("joiner_1", j1))
        w.append(("joiner_2", j2))
        for name, shape in joiner_screw_shapes:
            w.append((name, shape))
        return w

    j1 = place_joiner_j1()
    j2 = place_joiner_j2()
    o1 = xform_point(x_tie_out, y_loc_pre, z_j, 0.0, None)
    o2 = xform_point(x_tie_in, y_loc_pre, z_j, 90.0, s2_off)
    joiner_screw_shapes = (joiner_screws(o1, Vector(1, 0, 0), Vector(0, 1, 0), "j1")
                           + joiner_screws(o2, Vector(0, -1, 0), Vector(-1, 0, 0), "j2"))
    world = world_at(0.0)

    # The boss is on the plain plate's outer face. s1's faces away from the
    # curve; s2's faces the outside of the turn. Either one meeting the frame
    # means the boss, or the plate itself, has crossed the gap.
    for tag in ("s1", "s2"):
        plate = next(s for n, s in world if n == tag + "_plate_plain")
        d_pf = plate.distToShape(frame)[0]
        step("%s plain plate to curve frame %.3f mm" % (tag, d_pf))
        require(d_pf >= 0.5, "%s plain plate meets the curve frame (%.3f mm)" % (tag, d_pf))

    check_holes()
    audit_nuts(br_motor, br_plain, tie_in, frame)
    check_published_rods(world)
    # The belt solid is the tensioned path. At slack that shape is not on the
    # machine yet — the loop is being slid on — so it is not a collision.
    stations = (("take-up", 0.0), ("mid", nose_travel / 2.0), ("slack", nose_travel))
    for label, shift in stations:
        station = world_at(shift)
        moving = shift > 1e-9
        interfere(station, label, skip_belt=moving, moving_only=moving)
        check_screws(station, screw_specs(shift, s2_off), jacks_only=moving)

    # ---------------------------------------------------------------- export
    step("--- export ---")
    export_print(br_motor, "bracket_straight_motor")
    export_print(br_plain, "bracket_straight_plain")
    export_print(roller_to_print(rol_id), "roller_idler")
    export_print(roller_to_print(rol_dr), "roller_driven")
    export_print(to_print(cone_id), "roller_cone_idler")
    export_print(to_print(cone_dr), "roller_cone_driven")
    export_print(drop_to_bed(bed), "slider_bed_straight")
    export_print(drop_to_bed(ret), "return_guide_straight")
    export_print(print_tie(tie_in), "tie_bar")
    export_print(drop_to_bed(block), "tensioner_block")
    export_print(drop_to_bed(make_joiner()), "joiner")
    export(encl_straight, "ref_motor")

    step("coupon: discharge end of the motor plate")
    coupon = br_motor.common(Part.makeBox(
        coupon_len, boss_y1 + 8.0, tab_z1 + 10.0,
        Vector(straight_len - coupon_len, -2.0, -2.0)))
    export_print(coupon, "coupon_bracket_end")
    step("coupon: infeed end, with the tensioner seat")
    coupon_in = br_motor.common(Part.makeBox(
        30.0, block_out + wall + 6.0, bracket_h + 4.0,
        Vector(0.0, -(block_out + 2.0), -1.0)))
    # Print file: the block sits on the bed beside the plate. Nested on the
    # rail it would hang 13 mm up, and the slide fit could not be tried.
    plate_print = drop_to_bed(coupon_in)
    blk_print = drop_to_bed(block)
    blk_print.translate(Vector(plate_print.BoundBox.XMax + 4.0, 0.0, 0.0))
    export_print(Part.makeCompound([plate_print, blk_print]), "coupon_infeed_end")
    # Close-up stays in the assembled pose. These two are not print files.
    export(coupon_in, "tensioner_plate")
    export(block, "tensioner_block_seated")

    export_print(drop_to_bed(frame), "curve_frame")
    export_print(drop_to_bed(keeper), "curve_keeper")

    # The pitch-half sector clips the ear toward roller 2. Open that side until
    # both ear holes and their nut pockets sit inside the coupon.
    cover = math.degrees(math.atan((h_half + 1.0) / r_ow0))
    a0 = thetas[curve_driven - 1] - max(curve_pitch_deg / 2.0, cover)
    a1 = thetas[curve_driven] + curve_pitch_deg / 2.0
    step("coupon_curve: sector %.3f .. %.3f deg (pad cover %.3f)" % (a0, a1, cover))
    coupon_frame = clip_angles(frame, a0, a1)
    coupon_oring = link_spec[curve_driven - 1][2]
    # The cones and the ring are TPU and nitrile. The print file is the PETG
    # sector alone; the render uses the assembled view.
    export(Part.makeCompound([
        coupon_frame,
        placed[curve_driven - 1],
        placed[curve_driven],
        coupon_oring,
    ]), "coupon_curve_view")
    export_print(drop_to_bed(coupon_frame), "coupon_curve")

    def straight_compound(rot, offset):
        parts = [place_module(s, rot, offset) for _, s in local_parts
                 if _ not in ("rod", "motor") and not _.startswith("scr")]
        return Part.makeCompound(parts)

    step("assembly: straight, then mirror onto the left plate")
    export(straight_compound(0.0, None), "assembly_straight")
    export(Part.makeCompound([
        straight_compound(0.0, None),
        frame, Part.makeCompound(placed), Part.makeCompound(links), keeper,
        straight_compound(90.0, s2_off), j1, j2,
    ]), "assembly_v0")

    export_straight_components(br_motor, br_plain, rol_id, rol_dr, bed, ret, belt,
                               encl_straight, "cs", 0.0, None)
    export_straight_components(br_motor, br_plain, rol_id, rol_dr, bed, ret, belt,
                               encl_straight, "s2", 90.0, s2_off)
    export(Part.makeCompound([
        place_module(tie_in, 0.0, None), place_module(tie_out, 0.0, None)]), "cs_tiebars")
    export(Part.makeCompound([
        place_module(block, 0.0, None), place_module(block_far, 0.0, None)]), "cs_tension")
    export(Part.makeCompound([
        place_module(tie_in, 90.0, s2_off), place_module(tie_out, 90.0, s2_off)]), "s2_tiebars")
    export(Part.makeCompound([
        place_module(block, 90.0, s2_off), place_module(block_far, 90.0, s2_off)]), "s2_tension")
    export(j1, "jn_1")
    export(j2, "jn_2")

    j1_box = Part.makeBox(80.0, 70.0, 40.0, Vector(90.0, 0.0, 0.0))
    export(Part.makeCompound([
        place_module(tie_out, 0.0, None), j1, frame.common(j1_box),
    ]), "j1_view")

    export(frame, "cv_frame")
    export(Part.makeCompound(placed), "cv_rollers")
    export(Part.makeCompound(links), "cv_orings")
    export(encl_curve, "cv_motor")
    export(keeper, "cv_keeper")

    ax0, nose_ax = roller_axis_x(straight_len)
    spans = {
        "inner": transfer_span(curve_r_in),
        "centre": transfer_span(curve_r_c),
        "outer": transfer_span(curve_r_out),
    }
    geom = {
        "_generated_by": "cad/conveyor/build_parts.py — do not hand-edit",
        "belt_width": belt_width,
        "belt_thickness": belt_thickness,
        "nose_dia": nose_dia,
        "roller_flange_w": roller_flange_w,
        "side_gap": side_gap,
        "bracket_h": bracket_h,
        "wall": wall,
        "inner_width": inner_width,
        "outer_width": outer_width,
        "carry_z": carry_z,
        "nose_z": nose_z,
        "return_run_z": return_run_z(),
        "return_guide_top": return_run_z() - return_clear,
        "frame_gap": frame_gap,
        "straight": {
            "len": straight_len,
            "drive_ax": ax0,
            "nose_ax": nose_ax,
            "t": wall,
            "outer_width": outer_width,
            # After the mirror the y=0 plate is the plain one; the motor plate
            # carries the tab, so its rail is the tab top.
            "rail_top": [bracket_h, tab_z1],
            "belt_len": belt_path_length(straight_len),
            "print_cyl_dia": printed_cylinder_dia(straight_len),
        },
        "motor_shaft_len": motor_shaft_len,
        "shaft_engagement": shaft_engagement(),
        "motor_side": "left",
        "motor_tab": motor_tab,
        "s1": {"rot": 0, "offset": [0, 0]},
        "s2": {"rot": 90, "offset": [s2_ox, s2_oy]},
        "curve": {
            "centre": [cx, cy],
            "z_top": z_top,
            "r_in": curve_r_in,
            "r_out": curve_r_out,
            "r_c": curve_r_c,
            "k": curve_k,
            "alpha_deg": alpha_deg,
            "n": curve_n,
            "pitch_deg": curve_pitch_deg,
            "theta_deg": thetas,
            "cone_r": [r_a, r_b],
            "driven": curve_driven,
            "cone_bore_d": cone_bore_d,
            "inner_wall": [r_iw0, r_iw1],
            "outer_wall": [r_ow0, r_ow1],
            "wall_top": bracket_h,
            "entry_face_x": cx,
            "exit_face_y": cy,
            "grooves": {
                "A": {"r": r_gA, "c": c_A, "D": D_A},
                "B": {"r": r_gB, "c": c_B, "D": D_B},
            },
            "rod_cut_mm": {"idler": idler_rod_len, "stub": stub_len},
        },
        "spans": {"entry": spans, "exit": dict(spans)},
        # v0 line only. The README reads this; the sim does not.
        "hardware": hardware_block(s2_off, stub_cut),
    }
    with open(os.path.join(OUT, "geometry.json"), "w", encoding="utf-8") as fh:
        json.dump(geom, fh, indent=2)
    step("export: geometry.json")
    step("=== build complete ===")


def bed_dx(radius):
    # Closest corner of the bed to the axis is the lower one. Stay bed_gap
    # off the cylinder there; the top corner is then further away.
    dz = bed_bot - nose_z
    return math.sqrt((radius + bed_gap) ** 2 - dz * dz)


def guide_x_span():
    return (x_tie_in + tie_half + m3_head_d / 2.0 + 1.0,
            x_tie_out - tie_half - m3_head_d / 2.0 - 1.0)


def bed_x_span(module_len):
    # Infeed end clears the flange at full slack. The flange sweeps every
    # station between take-up and slack, and the slack end is the far one.
    # The discharge roller does not travel, so that end stays on the barrel.
    ax0, nose = roller_axis_x(module_len)
    dx_in = bed_dx(roller_flange_d / 2.0)
    dx_out = bed_dx(nose_dia / 2.0)
    return ax0 + nose_travel + dx_in, nose - dx_out


def hex_along_z(af, z0, z1, x, y):
    Rv = hex_Rv(af)
    pts = []
    for i in range(6):
        ang = math.radians(30.0 + 60.0 * i)
        pts.append(Vector(x + Rv * math.cos(ang), y + Rv * math.sin(ang), z0))
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(Vector(0, 0, z1 - z0))


def hex_along_x(af, x0, x1, y, z):
    Rv = hex_Rv(af)
    pts = []
    for i in range(6):
        ang = math.radians(30.0 + 60.0 * i)
        pts.append(Vector(x0, y + Rv * math.cos(ang), z + Rv * math.sin(ang)))
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(Vector(x1 - x0, 0, 0))


def shank(p0, p1, diameter):
    d = p1 - p0
    return Part.makeCylinder(diameter / 2.0, d.Length, p0, d)


def bb_hit(a, b):
    A, B = a.BoundBox, b.BoundBox
    return not (A.XMax < B.XMin or B.XMax < A.XMin or
                A.YMax < B.YMin or B.YMax < A.YMin or
                A.ZMax < B.ZMin or B.ZMax < A.ZMin)


def overlap_volume(a, b):
    try:
        return abs(a.common(b).Volume)
    except Exception as exc:
        step("interference boolean failed (%s); using distance" % exc)
        return 0.0 if a.distToShape(b)[0] > 0.02 else 1.0


def xform_point(x, y, z, rot, offset):
    # Mirror about the module mid-plane first, then the module's placement.
    # That is the same order as place_module, so a hole and the part that
    # carries it land on the same point.
    y = outer_width - y
    if rot:
        a = math.radians(rot)
        c, s = math.cos(a), math.sin(a)
        x, y = x * c - y * s, x * s + y * c
    if offset is not None:
        x += offset.x
        y += offset.y
    return x, y, z


def place_module(shape, rot, offset):
    s = mirror_left(shape)
    if rot:
        s.rotate(Vector(0, 0, 0), Vector(0, 0, 1), rot)
    if offset is not None:
        s.translate(offset)
    return s


# ------------------------------------------------------------- side bracket
def make_bracket(module_len, motor_side=False, belt_side=1):
    # Built with the motor plate at local y in [0, wall] and the motor outboard
    # toward −y. Every finished straight is mirrored about y = outer_width/2,
    # which puts the motor on the +Y plate — the left side of travel, inside
    # the turn — and points the driven roller's D-bore at +Y.
    step("bracket: len=%.1f motor_side=%s" % (module_len, motor_side))
    ax0, nose = roller_axis_x(module_len)
    body = Part.makeBox(module_len, wall, bracket_h, Vector(0, 0, 0))

    step("bracket: infeed take-up slot")
    sw = nose_axle_dia + 0.5
    # Slot runs INBOARD from the tensioned position. Take-up pulls the infeed
    # nose out toward the face, so the design span is what you actually get.
    # The outer end wall is the hard stop. With the axis at nose_edge the rod's
    # surface is against that wall, so take-up cannot pull the design span short.
    end_r = sw / 2.0
    outer_c = ax0 + (end_r - nose_axle_dia / 2.0)
    slot = Part.makeBox(nose_travel, wall + 2, sw, Vector(ax0, -1, nose_z - end_r))
    slot = slot.fuse(Part.makeCylinder(end_r, wall + 2, Vector(outer_c, -1, nose_z), Vector(0, 1, 0)))
    slot = slot.fuse(Part.makeCylinder(end_r, wall + 2,
                                        Vector(ax0 + nose_travel, -1, nose_z), Vector(0, 1, 0)))
    body = body.cut(slot)

    if motor_side:
        step("bracket: enclosure face, tab, M4 nuts")
        body = body.fuse(Part.makeBox(
            tab_x1 - tab_x0, wall, tab_z1 - (bracket_h - 0.2),
            Vector(tab_x0, 0, bracket_h - 0.2)))
        for sign, name in ((-1.0, "lower"), (1.0, "upper")):
            zc = nose_z + sign * encl_ear_pitch
            body = body.fuse(Part.makeCylinder(
                boss_r_straight, boss_y1 - (wall - 0.2),
                Vector(nose, wall - 0.2, zc), Vector(0, 1, 0)))
            step("bracket: %s boss inboard to y=%.2f, centred z=%.2f" % (name, boss_y1, zc))
        body = body.cut(Part.makeCylinder(
            spigot_hole_d / 2.0, wall + 2, Vector(nose, -1, nose_z), Vector(0, 1, 0)))
        for sign in (-1.0, 1.0):
            zc = nose_z + sign * encl_ear_pitch
            body = body.cut(Part.makeCylinder(
                m4_clear / 2.0, boss_y1 + 2, Vector(nose, -1, zc), Vector(0, 1, 0)))
            pocket = hex_along_y(m4_nut_af, boss_y1 - m4_nut_depth, boss_y1 + 0.2, nose, zc)
            body = body.cut(pocket)
    else:
        # Blind from the inner face (y = 0). The stub drops into the roller
        # before this plate goes on; the floor stops it walking out, and the
        # step at the end of the D-bore stops it walking the other way.
        step("bracket: blind bore for the driven stub")
        body = body.fuse(Part.makeCylinder(
            stub_boss_r, stub_boss_out + 0.2,
            Vector(nose, wall - 0.2, nose_z), Vector(0, 1, 0)))
        bore_depth = wall + stub_boss_out - stub_floor
        body = body.cut(Part.makeCylinder(
            stub_hole_r, bore_depth + 0.2,
            Vector(nose, -0.2, nose_z), Vector(0, 1, 0)))

    # The old mid-span M3s had nothing to bolt to. The tie bars carry the
    # bolts now, one near each end.
    for hx in (x_tie_in, x_tie_out):
        body = body.cut(Part.makeCylinder(
            m3_clear / 2.0, wall + 2, Vector(hx, -1, tie_bolt_z), Vector(0, 1, 0)))
    return add_plate_seats(body, belt_side)


def add_plate_seats(body, belt_side):
    # belt_side +1: the belt is at +Y, so the inner face is y = wall and the
    # outer face is y = 0. belt_side −1 is the plain plate before it is shifted
    # across the module: its inner face is the y = 0 face of this solid.
    step("bracket: bed and guide grooves, tensioner seat")
    if belt_side > 0:
        g0, g1 = wall - tongue_d, wall
        rail = Part.makeBox(22.0, rail_h, 3.0, Vector(0.0, -rail_h, 14.0))
        boss = jack_boss(y0=-block_out, y1=0.0)
    else:
        g0, g1 = 0.0, tongue_d
        rail = Part.makeBox(22.0, rail_h, 3.0, Vector(0.0, wall, 14.0))
        boss = jack_boss(y0=wall, y1=wall + block_out)
    # Closed grooves. Belt drag pushes the bed toward the discharge, and the
    # +X wall of the groove is what stops it. The guide uses the same trick
    # so it cannot walk either. Both sit beside the belt, not through the loop.
    for z0, z1, x0, x1 in tongue_grooves():
        body = body.cut(Part.makeBox(x1 - x0, g1 - g0, z1 - z0, Vector(x0, g0, z0)))
    return body.fuse(rail).fuse(boss)


def tongue_grooves():
    # Grooves are fit_gap larger than the tongues on every side.
    bx0, bx1 = bed_x_span(straight_len)
    tx0, tx1 = bx0 + 4.0, bx1 - 4.0
    bz0 = bed_bot + 0.8
    bz1 = bz0 + tongue_h
    gx0, gx1 = guide_x_span()
    gx0, gx1 = gx0 + 2.0, gx1 - 2.0
    top = return_run_z() - return_clear
    gz0 = top - wall + 0.7
    gz1 = gz0 + tongue_h
    return ((bz0 - fit_gap, bz1 + fit_gap, tx0 - fit_gap, tx1 + fit_gap),
            (gz0 - fit_gap, gz1 + fit_gap, gx0 - fit_gap, gx1 + fit_gap))


def jack_stack():
    # Block-side nut, then a web, then the head-side nut, then the web the
    # reaction bears on. Each nut then has plastic on the side the screw
    # pushes it when the tip drives the block.
    a0 = nose_edge + block_front + nose_travel
    a1 = a0 + m3_nut_depth
    b0 = a1 + nut_land
    b1 = b0 + m3_nut_depth
    return a0, a1, b0, b1


def jack_boss(y0, y1):
    # Fixed nut stack inboard of the sliding block. The screw points at the
    # module face and the belt keeps the block against the tip.
    a0, a1, b0, b1 = jack_stack()
    z = nose_z
    y = (y0 + y1) / 2.0
    rv = hex_Rv(m3_nut_af)
    # The block's slack face lands on x = a0. The boss starts there.
    boss = Part.makeBox((b1 + nut_land) - a0, (y1 - y0) + 3.0, 10.0,
                         Vector(a0, y0 - 1.5, z - 5.0))
    # Block-side nut slides in along the screw, from the open face.
    boss = boss.cut(hex_along_x(m3_nut_af, a0 - 0.3, a1, y, z))
    # Head-side nut is between two webs, so it comes in from above.
    boss = boss.cut(hex_along_x(m3_nut_af, b0, b1, y, z))
    # Up and out of the plate's bounding box. The head-side nut is between webs.
    boss = boss.cut(Part.makeBox(b1 - b0, 2.0 * rv, (bracket_h + 25.0) - (z - rv),
                                  Vector(b0, y - rv, z - rv)))
    boss = boss.cut(Part.makeCylinder(
        m3_clear / 2.0, (b1 + nut_land) - a0 + 1.0,
        Vector(a0 - 0.5, y, z), Vector(1, 0, 0)))
    return boss


def hex_along_y(af, y0, y1, x, z):
    Rv = hex_Rv(af)
    pts = []
    for i in range(6):
        ang = math.radians(30 + 60 * i)
        pts.append(Vector(x + Rv * math.cos(ang), y0, z + Rv * math.sin(ang)))
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(Vector(0, y1 - y0, 0))


# ------------------------------------------------------------------ rollers
# One straight roller serves both ends. The idler rides a Ø4 stub. The driven
# one takes the motor's D-shaft through a Ø7 spigot that crosses the plate.
def make_roller(driven=False):
    step("roller: Ø%.0f %s" % (nose_dia, "driven" if driven else "idler"))
    r = nose_dia / 2.0
    fr = roller_flange_d / 2.0
    body = Part.makeCylinder(r, belt_width, Vector(0, roller_flange_w, 0), Vector(0, 1, 0))
    body = body.fuse(Part.makeCylinder(fr, roller_flange_w, Vector(0, 0, 0), Vector(0, 1, 0)))
    body = body.fuse(Part.makeCylinder(fr, roller_flange_w,
                                        Vector(0, roller_len_straight() - roller_flange_w, 0),
                                        Vector(0, 1, 0)))
    if not driven:
        return body.cut(Part.makeCylinder(
            nose_axle_dia / 2.0 + 0.2, roller_len_straight() + 2,
            Vector(0, -1, 0), Vector(0, 1, 0)))

    y_tip = -(wall + side_gap - spigot_recess)
    body = body.fuse(Part.makeCylinder(
        spigot_d / 2.0, -y_tip + 0.2, Vector(0, y_tip, 0), Vector(0, 1, 0)))
    depth = motor_bore_depth
    cut = Part.makeCylinder(motor_shaft_d / 2.0 + 0.15, depth, Vector(0, y_tip, 0), Vector(0, 1, 0))
    beyond = motor_shaft_flat - motor_shaft_d / 2.0
    sliver = Part.makeBox(motor_shaft_d + 2, depth + 1, motor_shaft_d,
                           Vector(-(motor_shaft_d / 2.0 + 1), y_tip - 0.5, beyond))
    body = body.cut(cut.cut(sliver))
    y_far = y_tip + depth
    return body.cut(Part.makeCylinder(
        nose_axle_dia / 2.0 + 0.2, roller_len_straight() - y_far + 1,
        Vector(0, y_far, 0), Vector(0, 1, 0)))


def roller_len_straight():
    return belt_width + 2 * roller_flange_w


# --------------------------------------------------------------- slider bed
def make_slider_bed(module_len):
    # Ends bed_gap clear of each roller, with the corners notched back around
    # the flanges. Tongues on the long edges drop into the plate grooves from
    # the open side, so the bed goes in before the second plate and the groove
    # then holds it in X and Z. Y is the two plates.
    step("slider bed: len=%.1f" % module_len)
    x0, x1 = bed_x_span(module_len)
    bed = Part.makeBox(x1 - x0, inner_width, bed_top - bed_bot, Vector(x0, wall, bed_bot))
    ax0, nose = roller_axis_x(module_len)
    ry = wall + side_gap
    fr = roller_flange_d / 2.0 + bed_gap
    for axis in (ax0, nose):
        for y0, y1 in ((ry, ry + roller_flange_w),
                       (ry + roller_len_straight() - roller_flange_w,
                        ry + roller_len_straight())):
            cutter = Part.makeCylinder(fr, y1 - y0 + 0.4, Vector(axis, y0 - 0.2, nose_z),
                                        Vector(0, 1, 0))
            bed = bed.cut(cutter)
    tz0 = bed_bot + 0.8
    tx0, tx1 = x0 + 4.0, x1 - 4.0
    # Tongues stop fit_gap short of the groove on every face, including the
    # discharge end, which is the wall belt drag bears on.
    bed = bed.fuse(Part.makeBox(tx1 - tx0, tongue_d - fit_gap, tongue_h,
                                 Vector(tx0, wall - (tongue_d - fit_gap), tz0)))
    bed = bed.fuse(Part.makeBox(tx1 - tx0, tongue_d - fit_gap, tongue_h,
                                 Vector(tx0, wall + inner_width, tz0)))
    return bed


def make_return_guide(module_len):
    # Below the loop, so it can be trapped by the plates the same way as the
    # bed. Top stays return_clear under the taut return run.
    step("return guide: len=%.1f" % module_len)
    x0, x1 = guide_x_span()
    top = return_run_z() - return_clear
    bar = Part.makeBox(x1 - x0, inner_width, wall, Vector(x0, wall, top - wall))
    tz0 = top - wall + 0.7
    tx0, tx1 = x0 + 2.0, x1 - 2.0
    bar = bar.fuse(Part.makeBox(tx1 - tx0, tongue_d - fit_gap, tongue_h,
                                 Vector(tx0, wall - (tongue_d - fit_gap), tz0)))
    bar = bar.fuse(Part.makeBox(tx1 - tx0, tongue_d - fit_gap, tongue_h,
                                 Vector(tx0, wall + inner_width, tz0)))
    return bar


def make_tie_bar(x_c):
    # One bar near each end. The top face is the joint plane the joiner sits
    # on; the uprights are the ends, lying on the plate inner faces, with the
    # plate bolts' nuts captive there. Symmetric hole pattern, so one printed
    # part serves both ends and both joints.
    step("tie bar: x=%.1f" % x_c)
    bar = Part.makeBox(2.0 * tie_half, inner_width, tie_t,
                        Vector(x_c - tie_half, wall, tie_z0))
    rv = hex_Rv(m3_nut_af)
    for y_face, inward in ((wall, 1.0), (wall + inner_width, -1.0)):
        # Web toward the plate, so a plate pulling away takes the nut with
        # the bar and not out of it. The nut slides in from the upright's end.
        y_load = y_face + inward * nut_land
        y_far = y_load + inward * m3_nut_depth
        y_lo, y_hi = (y_load, y_far) if y_load < y_far else (y_far, y_load)
        bar = bar.cut(hex_along_y(m3_nut_af, y_lo, y_hi, x_c, tie_bolt_z))
        # Out the upright's +X end. The channel is the whole hex, points included.
        bar = bar.cut(Part.makeBox(
            tie_half + 2.0 * rv + 2.4, y_hi - y_lo, 2.0 * rv + 0.4,
            Vector(x_c - rv - 0.2, y_lo, tie_bolt_z - rv - 0.2)))
        # Past the nut the hole is clearance: an M3×8 tip must not hit plastic.
        reach = m3_tie_len - wall
        hy0 = y_face - 0.4 if inward > 0 else y_face - reach - 0.4
        bar = bar.cut(Part.makeCylinder(
            m3_clear / 2.0, reach + 1.2,
            Vector(x_c, hy0, tie_bolt_z), Vector(0, 1, 0)))
    z_open = tie_z0
    for y_h, dia in ((y_loc_pre, m3_locate), (y_clr_pre, m3_clear)):
        bar = bar.cut(hex_along_z(m3_nut_af, z_open - 0.2, z_open + m3_nut_depth, x_c, y_h))
        bar = bar.cut(Part.makeCylinder(dia / 2.0, tie_t + 1.0,
                                         Vector(x_c, y_h, z_open + m3_nut_depth - 0.2),
                                         Vector(0, 0, 1)))
    return bar


def make_tensioner_block():
    # Slides on the outer-face rail. The idler rod is fixed in the block; the
    # roller turns on the rod. Modelled at full take-up, axis at nose_edge,
    # on the motor plate's outer face (outboard is −Y).
    step("tensioner block")
    ax0, _ = roller_axis_x(straight_len)
    x0 = ax0 - block_back
    x1 = ax0 + block_front
    block = Part.makeBox(x1 - x0, block_out - 0.05, 18.0,
                          Vector(x0, -(block_out), 13.0))
    block = block.cut(Part.makeCylinder(
        nose_axle_dia / 2.0 + 0.1, block_out + 1.0,
        Vector(ax0, -block_out - 0.5, nose_z), Vector(0, 1, 0)))
    # Groove for the rail, fit_gap loose so the block actually slides.
    block = block.cut(Part.makeBox(
        (x1 - x0) + 0.4, rail_h + fit_gap, 3.0 + 2.0 * fit_gap,
        Vector(x0 - 0.2, -(rail_h + fit_gap), 14.0 - fit_gap)))
    return block


def screw_along(p0, direction, length, dia, head_d, head_h):
    d = vnorm(direction)
    shank = Part.makeCylinder(dia / 2.0, length, p0, d)
    head = Part.makeCylinder(head_d / 2.0, head_h, vadd(p0, vmul(d, -head_h)), d)
    return shank.fuse(head)


def straight_fasteners():
    # Motor-plate side, plus the plain-plate side as a y-mirror. place_module
    # mirrors the whole straight once more, and the two stay with their plates.
    out = []
    y_jack = -block_out / 2.0
    # p0 is the head's bearing face. The tip lands on the block; the head
    # sits inboard of the nut, where a hex key can reach it from the side.
    tip_x = nose_edge + block_front
    head = Vector(tip_x + m3_jack_len, y_jack, nose_z)
    jack = screw_along(head, Vector(-1, 0, 0), m3_jack_len, 3.0, m3_head_d, m3_head_h)
    out.append(("scr_jack", jack))
    out.append(("scr_jack_far", mirror_left(jack)))
    for x in (x_tie_in, x_tie_out):
        scr = screw_along(Vector(x, 0.0, tie_bolt_z), Vector(0, 1, 0),
                           m3_tie_len, 3.0, m3_head_d, m3_head_h)
        out.append(("scr_tie", scr))
        out.append(("scr_tie_far", mirror_left(scr)))
    for sign in (-1.0, 1.0):
        p0 = Vector(nose_x, -encl_ear_t, nose_z + sign * encl_ear_pitch)
        out.append(("scr_m4", screw_along(p0, Vector(0, 1, 0), m4_ear_len, 4.0, 7.0, 4.0)))
    return out


def curve_rod(theta, s0, s1, diameter):
    u = axis_u(theta)
    return Part.makeCylinder(diameter / 2.0, s1 - s0, vadd(A, vmul(u, s0)), u)


def keeper_screw(ang, z):
    th = math.radians(ang)
    outward = Vector(math.cos(th), math.sin(th), 0.0)
    p0 = Vector(cx + outward.x * (r_ow1 + keeper_t), cy + outward.y * (r_ow1 + keeper_t), z)
    return screw_along(p0, vmul(outward, -1.0), m3_keep_len, 3.0, m3_head_d, m3_head_h)


def curve_ear_screw(sign):
    p0 = vadd(vadd(A, vmul(u_drv, s_face + encl_ear_t)),
              vmul(e_th_drv, sign * encl_ear_pitch))
    return screw_along(p0, vmul(u_drv, -1.0), m4_ear_len, 4.0, 7.0, 4.0)


def place_joiner_j1():
    x, y, _ = xform_point(x_tie_out, y_loc_pre, z_j, 0.0, None)
    return apply_frame(make_joiner(), Vector(x, y, z_j + join_standoff),
                        Vector(1, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1))


def place_joiner_j2():
    x, y, _ = xform_point(x_tie_in, y_loc_pre, z_j, 90.0, Vector(s2_ox, s2_oy, 0))
    # Local X runs into the curve (−Y); local Y runs toward the clearance hole (−X).
    return apply_frame(make_joiner(), Vector(x, y, z_j + join_standoff),
                        Vector(0, -1, 0), Vector(-1, 0, 0), Vector(0, 0, 1))


def joiner_screws(origin_xy, ax, ay, tag):
    out = []
    for tag_i, p, _near, _far in joiner_screw_defs(origin_xy, ax, ay, tag):
        out.append((tag_i + "_scr", screw_along(p, Vector(0, 0, -1),
                                                 m3_join_len, 3.0, m3_head_d, m3_head_h)))
    return out


def print_tie(shape):
    # Joiner-nut pockets open downward in use. Flip so they open upward on the
    # printer; the plate-nut pockets stay as horizontal holes.
    p = shape.copy()
    p.rotate(Vector(0, 0, 0), Vector(1, 0, 0), 180.0)
    return drop_to_bed(p)


def check_holes():
    def near(a, b, msg):
        d = math.hypot(a[0] - b[0], a[1] - b[1])
        require(d < 0.05, "%s is %.3f mm off" % (msg, d))

    o = xform_point(x_tie_out, y_loc_pre, z_j, 0.0, None)
    c = xform_point(x_tie_out, y_clr_pre, z_j, 0.0, None)
    near((o[0], o[1] + 2.0 * hole_pitch), (c[0], c[1]), "J1 clearance hole vs tie bar")
    pads = joint_curve_holes()
    near((o[0] + join_span, o[1]), (pads[0][0], pads[0][1]), "J1 locating hole vs pad")
    near((o[0] + join_span, o[1] + 2.0 * hole_pitch), (pads[1][0], pads[1][1]),
         "J1 clearance hole vs pad")
    o2 = xform_point(x_tie_in, y_loc_pre, z_j, 90.0, Vector(s2_ox, s2_oy, 0))
    c2 = xform_point(x_tie_in, y_clr_pre, z_j, 90.0, Vector(s2_ox, s2_oy, 0))
    near((o2[0] - 2.0 * hole_pitch, o2[1]), (c2[0], c2[1]), "J2 clearance hole vs tie bar")
    near((o2[0], o2[1] - join_span), (pads[2][0], pads[2][1]), "J2 locating hole vs pad")
    near((o2[0] - 2.0 * hole_pitch, o2[1] - join_span), (pads[3][0], pads[3][1]),
         "J2 clearance hole vs pad")
    step("joiner holes within 0.05 mm of the tie-bar and pad holes")


def make_joiner():
    # One plate for both joints: the two end tie bars mirror about the module
    # centre, and the curve pads repeat that pattern across frame_gap.
    # Local: (0, 0) and (span, 0) are the locating holes; the other edge is
    # clearance, so the pair pins the lane without fighting itself.
    step("joiner")
    m = 7.0
    plate = Part.makeBox(join_span + 2.0 * m, 2.0 * hole_pitch + 2.0 * m, joiner_t,
                          Vector(-m, -m, 0.0))
    for x_h, y_h, dia in ((0.0, 0.0, m3_locate),
                          (0.0, 2.0 * hole_pitch, m3_clear),
                          (join_span, 0.0, m3_locate),
                          (join_span, 2.0 * hole_pitch, m3_clear)):
        plate = plate.cut(Part.makeCylinder(
            dia / 2.0, joiner_t + 2.0, Vector(x_h, y_h, -1.0), Vector(0, 0, 1)))
    return plate


def pad_bounds(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs) - pad_margin, max(xs) + pad_margin,
            min(ys) - pad_margin, max(ys) + pad_margin)


def _pad_exit(x, y, x0, x1, y0, y1):
    # The outer wall stands past the pad, so a slot aimed away from the curve
    # centre runs into it. The nut leaves through the nearer joint face.
    to_entry = (x0 + x1) * 0.5 - cx
    to_exit = cy - (y0 + y1) * 0.5
    if to_entry <= to_exit:
        return Vector(-1, 0, 0), 0.0, x - cx
    return Vector(0, 1, 0), 0.0, cy - y


def pad_nut_slot(x, y, z0, z1, rv, x0, x1, y0, y1):
    direction, _align, dist = _pad_exit(x, y, x0, x1, y0, y1)
    span = dist + rv + 2.0
    wide = 2.0 * rv + 0.4
    if direction.x > 0:
        return Part.makeBox(span, wide, z1 - z0, Vector(x - rv, y - rv - 0.2, z0))
    if direction.x < 0:
        return Part.makeBox(span, wide, z1 - z0, Vector(x + rv - span, y - rv - 0.2, z0))
    if direction.y > 0:
        return Part.makeBox(wide, span, z1 - z0, Vector(x - rv - 0.2, y - rv, z0))
    return Part.makeBox(wide, span, z1 - z0, Vector(x - rv - 0.2, y + rv - span, z0))


def pad_slot_dir(x, y, x0, x1, y0, y1):
    direction, _align, dist = _pad_exit(x, y, x0, x1, y0, y1)
    return direction, dist + hex_Rv(m3_nut_af) + 2.0


def add_joint_pads(frame):
    # The curve's base is at z = wall. These pads bring two patches under the
    # lane, one at each face, up to the joint plane so a joiner can sit flat
    # on the straight and the curve together.
    step("curve joint pads at z=%.1f" % z_j)
    entry, exit_pad = joint_pad_boxes()
    frame = frame.fuse(entry).fuse(exit_pad)
    z_open = z_j - web
    holes = joint_curve_holes()
    bounds = (pad_bounds(holes[:2]), pad_bounds(holes[2:]))
    rv = hex_Rv(m3_nut_af)
    for i, (x_h, y_h, dia) in enumerate(holes):
        frame = frame.cut(Part.makeCylinder(5.0, z_open + 0.2, Vector(x_h, y_h, -0.2),
                                             Vector(0, 0, 1)))
        frame = frame.cut(hex_along_z(m3_nut_af, z_open, z_open + m3_nut_depth, x_h, y_h))
        frame = frame.cut(Part.makeCylinder(dia / 2.0, web + 2.0,
                                             Vector(x_h, y_h, z_open + m3_nut_depth - 0.3),
                                             Vector(0, 0, 1)))
        # The pad is solid below the pocket, so the nut slides in from the side.
        x0, x1, y0, y1 = bounds[0 if i < 2 else 1]
        frame = frame.cut(pad_nut_slot(x_h, y_h, z_open, z_open + m3_nut_depth,
                                        rv, x0, x1, y0, y1))
    return frame


def joint_curve_holes():
    # Post-mirror s1 discharge locating hole is the low-Y one; the curve's
    # matching holes are one join_span further along each module's travel.
    s1 = []
    for y_pre, dia in ((y_loc_pre, m3_locate), (y_clr_pre, m3_clear)):
        x, y, _ = xform_point(x_tie_out, y_pre, z_j, 0.0, None)
        s1.append((x + join_span, y, dia))
    s2 = []
    for y_pre, dia in ((y_loc_pre, m3_locate), (y_clr_pre, m3_clear)):
        x, y, _ = xform_point(x_tie_in, y_pre, z_j, 90.0, Vector(s2_ox, s2_oy, 0))
        # Travel from s2 into the curve is −Y.
        s2.append((x, y - join_span, dia))
    return s1 + s2


def joint_pad_boxes():
    holes = joint_curve_holes()
    def box_for(pts):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        x0, x1 = min(xs) - pad_margin, max(xs) + pad_margin
        y0, y1 = min(ys) - pad_margin, max(ys) + pad_margin
        # Stay on the curve's side of each face.
        return Part.makeBox(x1 - x0, y1 - y0, z_j, Vector(x0, y0, 0.0))
    return box_for(holes[:2]), box_for(holes[2:])


# ---------------------------------------------------- enclosure (not printed)
def make_motor_enclosure(ear_axis):
    # Mounting face at y=0, shaft along +Y toward the roller, body in −Y.
    # ear_axis "z": 16 mm along X, 24 mm vertical, ears at ±Z (a straight).
    # ear_axis "x": 24 mm along X, 16 mm vertical, ears at ±X (the curve).
    step("motor enclosure: reference, ears along %s" % ear_axis)
    if ear_axis == "z":
        hx, hz = encl_narrow / 2.0, encl_wide / 2.0
    else:
        hx, hz = encl_wide / 2.0, encl_narrow / 2.0
    body = Part.makeBox(2 * hx, encl_along, 2 * hz, Vector(-hx, -encl_along, -hz))
    lr = encl_lobe_d / 2.0
    for sign in (-1.0, 1.0):
        if ear_axis == "z":
            lobe = Part.makeCylinder(lr, encl_ear_t,
                                      Vector(0, -encl_ear_t, sign * encl_ear_pitch),
                                      Vector(0, 1, 0))
            a, b = sign * (hz - 2.0), sign * encl_ear_pitch
            z0, z1 = (a, b) if a < b else (b, a)
            plate = Part.makeBox(2 * lr, encl_ear_t, z1 - z0, Vector(-lr, -encl_ear_t, z0))
            hole = Part.makeCylinder(encl_ear_hole_d / 2.0, encl_ear_t + 2.0,
                                      Vector(0, -encl_ear_t - 1.0, sign * encl_ear_pitch),
                                      Vector(0, 1, 0))
        else:
            lobe = Part.makeCylinder(lr, encl_ear_t,
                                      Vector(sign * encl_ear_pitch, -encl_ear_t, 0),
                                      Vector(0, 1, 0))
            a, b = sign * (hx - 2.0), sign * encl_ear_pitch
            x0, x1 = (a, b) if a < b else (b, a)
            plate = Part.makeBox(x1 - x0, encl_ear_t, 2 * lr, Vector(x0, -encl_ear_t, -lr))
            hole = Part.makeCylinder(encl_ear_hole_d / 2.0, encl_ear_t + 2.0,
                                      Vector(sign * encl_ear_pitch, -encl_ear_t - 1.0, 0),
                                      Vector(0, 1, 0))
        body = body.fuse(plate).fuse(lobe).cut(hole)
    boss = Part.makeCylinder(motor_boss_d / 2.0, motor_boss_len,
                              Vector(0, -encl_face_to_gearbox, 0), Vector(0, 1, 0))
    shaft = Part.makeCylinder(motor_shaft_d / 2.0, motor_shaft_len,
                               Vector(0, -encl_face_to_gearbox, 0), Vector(0, 1, 0))
    beyond = motor_shaft_flat - motor_shaft_d / 2.0
    sliver = Part.makeBox(motor_shaft_d + 2, motor_shaft_len + 1, motor_shaft_d,
                           Vector(-(motor_shaft_d / 2.0 + 1), -encl_face_to_gearbox - 0.5, beyond))
    return body.fuse(boss).fuse(shaft.cut(sliver))


def encl_straight_world(build_frame_encl):
    # The check is against the placed straight: mirrored onto the +Y plate.
    return mirror_left(build_frame_encl)


# ------------------------------------------------------------- cone rollers
def make_cone_roller(driven):
    # Apex at the origin, axis +X, big end at larger X. Placement tilts this
    # down by α, swings it to θ and parks the apex on A. Print orientation is
    # a separate copy, axis vertical, big end down.
    step("cone roller: %s" % ("driven" if driven else "idler"))
    r0 = s_a * math.tan(curve_alpha)
    r1 = s_b * math.tan(curve_alpha)
    body = Part.makeCone(r0, r1, s_b - s_a, Vector(s_a, 0, 0), Vector(1, 0, 0))
    # Spool OD is the groove flange. Overlap the cone end so the fuse is a solid.
    body = body.fuse(Part.makeCylinder(R_spool_A, s_mid - (s_b - 0.15),
                                        Vector(s_b - 0.15, 0, 0), Vector(1, 0, 0)))
    body = body.fuse(Part.makeCylinder(R_spool_B, s_spool_end - s_mid + 0.05,
                                        Vector(s_mid - 0.05, 0, 0), Vector(1, 0, 0)))
    # Same floor as the original round section. The flanks are swept along
    # the axis so the fleet angle clears them without dropping the seat.
    body = body.cut(groove_stadium(s_gA, D_A / 2.0, groove_section_r, groove_axial))
    body = body.cut(groove_stadium(s_gB, D_B / 2.0, groove_section_r, groove_axial))
    bore_r = cone_bore_d / 2.0
    if not driven:
        return body.cut(Part.makeCylinder(
            bore_r, s_spool_end - s_a + 2.0, Vector(s_a - 1.0, 0, 0), Vector(1, 0, 0)))

    s_tip = s_face - spigot_recess
    body = body.fuse(Part.makeCylinder(
        spigot_d / 2.0, s_tip - (s_spool_end - 0.15),
        Vector(s_spool_end - 0.15, 0, 0), Vector(1, 0, 0)))
    # Small-end stub bore, 20 mm, opening on the small face.
    body = body.cut(Part.makeCylinder(
        bore_r, stub_bore_depth + 0.3, Vector(s_a - 0.3, 0, 0), Vector(1, 0, 0)))
    # D-bore from the spigot tip back into the roller. Flat on +Z, which lands
    # on ê_up after the placement rotations — the same side as the enclosure shaft.
    depth = motor_bore_depth
    cut = Part.makeCylinder(motor_shaft_d / 2.0 + 0.15, depth,
                             Vector(s_tip - depth, 0, 0), Vector(1, 0, 0))
    beyond = motor_shaft_flat - motor_shaft_d / 2.0
    sliver = Part.makeBox(depth + 1, motor_shaft_d + 2, motor_shaft_d,
                           Vector(s_tip - depth - 0.5, -(motor_shaft_d / 2.0 + 1), beyond))
    return body.cut(cut.cut(sliver))


def groove_torus(s_g, major, minor):
    circ = Part.makeCircle(minor, Vector(s_g, 0, major), Vector(0, 1, 0))
    face = Part.Face(Part.Wire([circ]))
    return face.revolve(Vector(0, 0, 0), Vector(1, 0, 0), 360)


def groove_stadium(s_g, major, section_r, axial):
    # Circle of the original section, swept ±axial. Floor radius is
    # major - section_r on the whole flat, and the end bulbs do not go deeper.
    inner = major - section_r
    outer = major + section_r
    span = 2.0 * axial + 0.2
    tube = Part.makeCylinder(outer, span, Vector(s_g - axial - 0.1, 0, 0), Vector(1, 0, 0))
    tube = tube.cut(Part.makeCylinder(inner, span + 0.4,
                                       Vector(s_g - axial - 0.3, 0, 0), Vector(1, 0, 0)))
    return tube.fuse(groove_torus(s_g - axial, major, section_r)).fuse(
        groove_torus(s_g + axial, major, section_r))


def place_cone(shape, theta_deg):
    s = shape.copy()
    s.rotate(Vector(0, 0, 0), Vector(0, 1, 0), alpha_deg)
    s.rotate(Vector(0, 0, 0), Vector(0, 0, 1), theta_deg)
    s.translate(A)
    return s


def to_print(shape):
    # +X (toward the big end) rotates about Y onto −Z, then the part is sat
    # on the bed. The spool and the driven spigot are outboard of the big end,
    # so they become the base and the cone narrows as it rises.
    p = shape.copy()
    p.rotate(Vector(0, 0, 0), Vector(0, 1, 0), 90.0)
    require(p.BoundBox.ZMin < -1.0, "print rotation did not put the big end down")
    p.translate(Vector(0, 0, -p.BoundBox.ZMin))
    return p


# --------------------------------------------------------------- curve frame
def annular_sector(r0, r1, z0, z1, a0_deg, a1_deg):
    h = z1 - z0
    outer = Part.makeCylinder(r1, h, Vector(cx, cy, z0))
    ring = outer.cut(Part.makeCylinder(max(r0, 0.1), h, Vector(cx, cy, z0)))
    return clip_angles(ring, a0_deg, a1_deg)


def clip_angles(shape, a0_deg, a1_deg):
    # Keep θ in [a0, a1]. The planes pass through C and are vertical.
    a0 = math.radians(a0_deg)
    a1 = math.radians(a1_deg)
    n0 = Vector(-math.sin(a0), math.cos(a0), 0.0)          # toward increasing θ
    n1 = Vector(math.sin(a1), -math.cos(a1), 0.0)           # toward decreasing θ
    origin = Vector(cx, cy, 0.0)
    shape = clip_halfspace(shape, origin, n0)
    return clip_halfspace(shape, origin, n1)


def clip_halfspace(shape, origin, inward):
    n = vnorm(inward)
    tmp = Vector(0, 0, 1) if abs(n.z) < 0.9 else Vector(1, 0, 0)
    x = vnorm(vcross(tmp, n))
    y = vnorm(vcross(n, x))
    span = 800.0
    # Local +Z is the inward normal, so the box starts on the plane itself.
    box = Part.makeBox(span, span, span, Vector(-span / 2.0, -span / 2.0, 0.0))
    return shape.common(apply_frame(box, origin, x, y, n))


def make_curve_frame():
    step("curve frame: base, walls, pad, axle holes")
    base = annular_sector(r_iw0, r_ow1, 0.0, wall, -90.0, 0.0)
    inner = annular_sector(r_iw0, r_iw1, 0.0, bracket_h, -90.0, 0.0)
    outer = annular_sector(r_ow0, r_ow1, 0.0, bracket_h, -90.0, 0.0)
    frame = base.fuse(inner).fuse(outer)

    pad = apply_frame(
        Part.makeBox(2 * h_half, s_face - s_slab_in, 2 * v_half,
                     Vector(-h_half, s_slab_in, -v_half)),
        A, e_th_drv, u_drv, e_up_drv)
    frame = frame.fuse(pad)
    for sign in (-1.0, 1.0):
        p0 = vadd(A, vadd(vmul(u_drv, s_boss_in), vmul(e_th_drv, sign * encl_ear_pitch)))
        frame = frame.fuse(Part.makeCylinder(boss_r_curve, s_face - s_boss_in, p0, u_drv))

    for i, th in enumerate(thetas):
        u = axis_u(th)
        # Blind hole from the roller face back toward the apex, stopping short.
        frame = frame.cut(Part.makeCylinder(
            axle_hole_d / 2.0, (r_iw1_s + 0.6) - s_hole_bottom,
            vadd(A, vmul(u, s_hole_bottom)), u))
        if (i + 1) == curve_driven:
            frame = frame.cut(Part.makeCylinder(
                spigot_hole_d / 2.0, (s_face + 1.0) - (r_ow0_s - 1.0),
                vadd(A, vmul(u, r_ow0_s - 1.0)), u))
        else:
            frame = frame.cut(Part.makeCylinder(
                axle_hole_d / 2.0, (r_ow1_s + 0.6) - (r_ow0_s - 0.4),
                vadd(A, vmul(u, r_ow0_s - 0.4)), u))

    for sign in (-1.0, 1.0):
        h = sign * encl_ear_pitch
        p0 = vadd(A, vadd(vmul(u_drv, s_boss_in - 0.4), vmul(e_th_drv, h)))
        frame = frame.cut(Part.makeCylinder(
            m4_clear / 2.0, s_face - s_boss_in + 1.2, p0, u_drv))
        frame = frame.cut(hex_along_u(m4_nut_af, s_nut_far, s_nut_near, h, 0.0))
        # The bore is the only axial opening, so the nut drops in from above.
        frame = frame.cut(apply_frame(
            Part.makeBox(s_nut_near - s_nut_far, 2.0 * m4_Rv + 0.4, 40.0 + m4_Rv,
                         Vector(s_nut_far, h - m4_Rv - 0.2, -m4_Rv)),
            A, u_drv, e_th_drv, e_up_drv))

    # s1's enclosure overhangs the discharge face into the empty centre. The
    # inner wall's entry end would meet it. Cut the bbox, grown, out of the frame.
    ko = keepout_box()
    step("keep-out x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f"
         % (ko.BoundBox.XMin, ko.BoundBox.XMax, ko.BoundBox.YMin, ko.BoundBox.YMax,
            ko.BoundBox.ZMin, ko.BoundBox.ZMax))
    return frame.cut(ko)


def keepout_box():
    # Body + ears only. The shaft stays inside s1's roller and is not the overhang.
    ko = keepout_grow
    x0 = nose_x - encl_narrow / 2.0 - ko
    x1 = nose_x + encl_narrow / 2.0 + ko
    y0 = outer_width - ko
    y1 = outer_width + encl_along + ko
    z0 = nose_z - (encl_ear_pitch + lobe_r) - ko
    z1 = nose_z + (encl_ear_pitch + lobe_r) + ko
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def hex_along_u(af, s0, s1, h, v):
    Rv = hex_Rv(af)
    pts = []
    for i in range(6):
        ang = math.radians(30 + 60 * i)
        hh = h + Rv * math.cos(ang)
        vv = v + Rv * math.sin(ang)
        pts.append(vadd(A, vadd(vmul(u_drv, s0),
                                 vadd(vmul(e_th_drv, hh), vmul(e_up_drv, vv)))))
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(vmul(u_drv, s1 - s0))


def make_keeper(cap_half_mm):
    # One arc on the outer wall's outer face. A low rail ties the caps together
    # under the pad, so the driven position stays uncovered and the part is
    # still one piece. Caps rise over each idler rod end.
    step("curve keeper")
    cap_deg = math.degrees(cap_half_mm / r_ow1)
    z_rail0 = wall + 1.0
    z_rail1 = 10.0
    z_cap1 = (z_top - r_ow1_s * sa) + hole_cover()
    a0 = thetas[0] - cap_deg
    a1 = thetas[-1] + cap_deg
    rail = annular_sector(r_ow1, r_ow1 + keeper_t, z_rail0, z_rail1, a0, a1)
    parts = [rail]
    for i, th in enumerate(thetas):
        if (i + 1) == curve_driven:
            continue
        parts.append(annular_sector(
            r_ow1, r_ow1 + keeper_t, z_rail1 - 0.3, z_cap1,
            th - cap_deg, th + cap_deg))
    out = parts[0]
    for p in parts[1:]:
        out = out.fuse(p)
    # Rail must stay below the pad. Checked by the fuse not being asked to
    # join them; they are separate parts and the rail top is under the pad.
    return out


def hole_cover():
    return axle_hole_d / 2.0 + keeper_cap_extra


def radial_hole(theta_deg, z, radius, r_from, r_to):
    th = math.radians(theta_deg)
    direction = Vector(math.cos(th), math.sin(th), 0.0)
    start = Vector(cx + direction.x * r_from, cy + direction.y * r_from, z)
    return Part.makeCylinder(radius, r_to - r_from, start, direction)


# ------------------------------------------------------------------- o-rings
def make_oring_link(theta_a, theta_b, r_g, D):
    # Render only. The line of centres is perpendicular to the average axle
    # (the two axes intersect at the apex, so (u2−u1)·(u1+u2) = 0). The cord
    # is a stadium in the plane normal to that average: straight length equals
    # the centre distance, half a turn on each spool. The 15.7° skew between
    # the axles is the part this ignores.
    s = r_g * ca
    u1 = axis_u(theta_a)
    u2 = axis_u(theta_b)
    P1 = vadd(A, vmul(u1, s))
    P2 = vadd(A, vmul(u2, s))
    ex = vnorm(vsub(P2, P1))
    ey = vnorm(vadd(u1, u2))
    ez = vnorm(vcross(ex, ey))
    if ez.z < 0.0:
        ez = vmul(ez, -1.0)
    # Picture only. The real cord, skew included, is what the interference
    # check holds. This section is thinner so the drawing stays in the groove.
    local = oring_stadium((P2 - P1).Length, D / 2.0, oring_cs / 2.0 * 0.45)
    return apply_frame(local, P1, ex, ey, ez)


def torus_about_y(major, minor):
    circ = Part.makeCircle(minor, Vector(major, 0, 0), Vector(0, 0, 1))
    face = Part.Face(Part.Wire([circ]))
    return face.revolve(Vector(0, 0, 0), Vector(0, 1, 0), 360)


def oring_stadium(c, R, cr):
    torus = torus_about_y(R, cr)
    big = R + cr + 2.0
    left_box = Part.makeBox(big + 0.4, 2 * (cr + 1), 2 * big,
                             Vector(-big, -(cr + 1), -big))
    right_box = Part.makeBox(big + 0.4, 2 * (cr + 1), 2 * big,
                              Vector(-0.4, -(cr + 1), -big))
    left = torus.common(left_box)
    right = torus.common(right_box)
    right.translate(Vector(c, 0, 0))
    upper = Part.makeCylinder(cr, c + 0.6, Vector(-0.3, 0, R), Vector(1, 0, 0))
    lower = Part.makeCylinder(cr, c + 0.6, Vector(-0.3, 0, -R), Vector(1, 0, 0))
    return left.fuse(right).fuse(upper).fuse(lower)


# --------------------------------------------------------------- straights
def assemble_straight_local(br_motor, br_plain, rol_id, rol_dr, bed, ret, belt):
    ax0, nose = roller_axis_x(straight_len)
    ry = wall + side_gap
    parts = [
        br_motor,
        br_plain.translated(Vector(0, inner_width + wall, 0)),
        rol_id.translated(Vector(ax0, ry, nose_z)),
        rol_dr.translated(Vector(nose, ry, nose_z)),
        bed,
        ret,
        belt.translated(Vector(0, ry + roller_flange_w, 0)),
    ]
    out = parts[0]
    for p in parts[1:]:
        out = out.fuse(p)
    return out


def mirror_left(shape):
    # Motor plate was y in [0, wall], motor toward −y. Reflecting through the
    # module mid-plane parks it on the +Y plate.
    out = shape.mirror(Vector(0, outer_width / 2.0, 0), Vector(0, 1, 0))
    try:
        if out.Volume < 0:
            out.reverse()
    except Exception as exc:
        step("mirror: orientation not flipped (%s)" % exc)
    return out


def export_straight_components(br_motor, br_plain, rol_id, rol_dr, bed, ret, belt,
                               encl, tag, rot, offset):
    step("components: %s" % tag)
    ax0, nose = roller_axis_x(straight_len)
    ry = wall + side_gap

    def place(s):
        s = mirror_left(s)
        if rot:
            s.rotate(Vector(0, 0, 0), Vector(0, 0, 1), rot)
        if offset is not None:
            s.translate(offset)
        return s

    brackets = br_motor.fuse(br_plain.translated(Vector(0, inner_width + wall, 0)))
    rollers = rol_id.translated(Vector(ax0, ry, nose_z)).fuse(
        rol_dr.translated(Vector(nose, ry, nose_z)))
    export(place(brackets), tag + "_brackets")
    export(place(rollers), tag + "_rollers")
    export(place(encl), tag + "_motor")
    export(place(bed), tag + "_bed")
    export(place(ret), tag + "_return")
    export(place(belt.translated(Vector(0, ry + roller_flange_w, 0))), tag + "_belt")


def drop_to_bed(shape):
    bb = shape.BoundBox
    out = shape.copy()
    out.translate(Vector(-bb.XMin, -bb.YMin, -bb.ZMin))
    return out


# ------------------------------------------------- belt (render only)
def _tangent_normal(c1, r1, c2, r2, upper):
    # A shared normal n has n·(c2−c1) = r1−r2, which is acos. asin tilts the
    # carry run about a degree and lifts the belt off the slider bed.
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    dist = math.hypot(dx, dz)
    alpha = math.atan2(dz, dx)
    off = math.acos((r1 - r2) / dist)
    psi = alpha + off if upper else alpha - off
    return math.cos(psi), math.sin(psi)


def make_belt(module_len):
    ax0, nose = roller_axis_x(module_len)
    c1, r1 = (ax0, nose_z), nose_dia / 2.0
    c2, r2 = (nose, nose_z), nose_dia / 2.0
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


def curve_idler_rod(theta):
    return curve_rod(theta, s_hole_bottom, s_hole_bottom + idler_rod_len, curve_axle_d)


def curve_stub_rod(theta):
    return curve_rod(theta, s_hole_bottom, s_hole_bottom + stub_len, curve_axle_d)


def check_rod_volume(shape, length, diameter, name):
    # A shortened stand-in has the wrong volume. The published cut is the solid.
    expected = math.pi * (diameter / 2.0) ** 2 * length
    rel = abs(abs(shape.Volume) - expected) / expected
    step("rod %s cut %.3f mm  volume err %.5f%%" % (name, length, 100.0 * rel))
    require(rel < 1e-4, "%s is not its published cut of %.3f mm" % (name, length))


def check_published_rods(world):
    n_straight = n_stub_s = n_idler = n_stub_c = 0
    for name, shape in world:
        if name in ("s1_rod", "s2_rod"):
            check_rod_volume(shape, idler_axle_len, nose_axle_dia, name)
            n_straight += 1
        elif name in ("s1_driven_stub", "s2_driven_stub"):
            check_rod_volume(shape, y_stub_len(shape), nose_axle_dia, name)
            n_stub_s += 1
        elif name.startswith("crod_"):
            check_rod_volume(shape, idler_rod_len, curve_axle_d, name)
            n_idler += 1
        elif name == "stub":
            check_rod_volume(shape, stub_len, curve_axle_d, name)
            n_stub_c += 1
    require(n_straight == 2 and n_stub_s == 2 and n_idler == curve_n - 1 and n_stub_c == 1,
            "rod count straight %d stub %d curve-idler %d curve-stub %d"
            % (n_straight, n_stub_s, n_idler, n_stub_c))
    step("straight stub %.2f mm short of the D-bore, seated on the blind floor" % stub_bore_air)
    step("curve stub %.2f mm short of the cone bore bottom" % stub_bore_air)


def y_stub_len(shape):
    # The placed stub is a pure cylinder. Its volume names the cut, and this
    # returns that cut so the checker compares the solid to the published one.
    return stub_cut_length()


def stub_cut_length():
    y0, y1 = straight_driven_stub_ys()
    return y1 - y0


def hex_prism(af, origin, direction, length):
    # Vertical pockets are cut with a vertex up. The same basis here, so the
    # sweep prism is the pocket and not a 30° turn of it.
    d = vnorm(direction)
    tmp = Vector(0, 0, 1) if abs(d.z) < 0.9 else Vector(0, 1, 0)
    x = vnorm(vcross(tmp, d))
    y = vnorm(vcross(d, x))
    rv = hex_Rv(af)
    pts = []
    for i in range(6):
        ang = math.radians(30.0 + 60.0 * i)
        pts.append(vadd(origin, vadd(vmul(x, rv * math.cos(ang)),
                                      vmul(y, rv * math.sin(ang)))))
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(vmul(d, length))


def keeper_top_slot(ang, z):
    # The inner mouth faces the inner wall, so the straight axial path hits it.
    # The nut comes in from the top of the outer wall instead.
    th = math.radians(ang)
    er = Vector(math.cos(th), math.sin(th), 0.0)
    et = Vector(-math.sin(th), math.cos(th), 0.0)
    rv = hex_Rv(m3_nut_af)
    r0 = r_ow0 - m3_nut_depth
    box = Part.makeBox(m3_nut_depth, 2.0 * rv, (bracket_h + 20.0) - (z - rv),
                        Vector(0.0, -rv, z - rv))
    return apply_frame(box, Vector(cx + er.x * r0, cy + er.y * r0, 0.0),
                        er, et, Vector(0, 0, 1))


def add_keeper_nut(frame, ang, z):
    # The outer wall is 3 mm and the nut is 2.6, so the nut lives in a boss on
    # the inner face. An M3×10 then crosses the keeper, the wall and the nut.
    th = math.radians(ang)
    er = Vector(math.cos(th), math.sin(th), 0.0)
    r_far = r_ow0 - m3_nut_depth
    origin = Vector(cx + er.x * r_far, cy + er.y * r_far, z)
    boss_r = hex_Rv(m3_nut_af) + 1.2
    frame = frame.fuse(Part.makeCylinder(boss_r, m3_nut_depth + 0.4, origin, er))
    # Stop on the wall face. Opening past it eats the web the nut bears on.
    return frame.cut(hex_prism(m3_nut_af, origin, er, m3_nut_depth))


def _module_point(x, y, z, rot, offset, double_mirror):
    if not double_mirror:
        y = outer_width - y
    if rot:
        a = math.radians(rot)
        c, s = math.cos(a), math.sin(a)
        x, y = x * c - y * s, x * s + y * c
    if offset is not None:
        x += offset.x
        y += offset.y
    return Vector(x, y, z)


def _module_dir(dx, dy, dz, rot, double_mirror):
    if not double_mirror:
        dy = -dy
    if rot:
        a = math.radians(rot)
        c, s = math.cos(a), math.sin(a)
        dx, dy = dx * c - dy * s, dx * s + dy * c
    return Vector(dx, dy, dz)


def _spec(name, p0, direction, length, dia, nut_near, nut_far, size, where, nuts, spans=None):
    return {"name": name, "p0": p0, "dir": direction, "length": length, "dia": dia,
            "nut_near": nut_near, "nut_far": nut_far, "size": size, "where": where,
            "nuts": nuts, "spans": spans or ((nut_near, nut_far),)}


def joiner_screw_defs(origin_xy, ax, ay, tag):
    head_z = z_j + join_standoff + joiner_t
    across = 2.0 * hole_pitch
    out = []
    for lx, ly in ((0.0, 0.0), (0.0, across), (join_span, 0.0), (join_span, across)):
        p = vadd(Vector(origin_xy[0], origin_xy[1], head_z),
                 vadd(vmul(ax, lx), vmul(ay, ly)))
        if lx < join_span * 0.5:
            top = tie_z0 + m3_nut_depth
            bot = tie_z0
        else:
            top = (z_j - web) + m3_nut_depth
            bot = z_j - web
        out.append((tag, p, head_z - top, head_z - bot))
    return out


def screw_specs(shift, s2_off):
    specs = []
    tip0 = nose_edge + block_front
    modules = ((0.0, None), (90.0, s2_off))
    for rot, offset in modules:
        for mirrored in (False, True):
            p0x = tip0 + shift + m3_jack_len
            p0 = _module_point(p0x, -block_out / 2.0, nose_z, rot, offset, mirrored)
            d = _module_dir(-1.0, 0.0, 0.0, rot, mirrored)
            a0, a1, b0, b1 = jack_stack()
            specs.append(_spec("jack", p0, d, m3_jack_len, 3.0,
                                p0x - a1, p0x - a0,
                                "M3", "tensioner jacks", 2,
                                spans=((p0x - a1, p0x - a0), (p0x - b1, p0x - b0))))
        for x in (x_tie_in, x_tie_out):
            for mirrored in (False, True):
                p0 = _module_point(x, 0.0, tie_bolt_z, rot, offset, mirrored)
                d = _module_dir(0.0, 1.0, 0.0, rot, mirrored)
                specs.append(_spec("tie", p0, d, m3_tie_len, 3.0,
                                    wall + nut_land, wall + nut_land + m3_nut_depth,
                                    "M3", "tie bars", 1))
        for sign in (-1.0, 1.0):
            p0 = _module_point(nose_x, -encl_ear_t, nose_z + sign * encl_ear_pitch,
                               rot, offset, False)
            d = _module_dir(0.0, 1.0, 0.0, rot, False)
            near = (boss_y1 - m4_nut_depth) - (-encl_ear_t)
            far = boss_y1 - (-encl_ear_t)
            specs.append(_spec("m4", p0, d, m4_ear_len, 4.0, near, far,
                                "M4", "motor ears", 1))
    for sign in (-1.0, 1.0):
        p0 = vadd(vadd(A, vmul(u_drv, s_face + encl_ear_t)),
                  vmul(e_th_drv, sign * encl_ear_pitch))
        near = encl_ear_t + nut_land
        specs.append(_spec("cv_m4", p0, vmul(u_drv, -1.0), m4_ear_len, 4.0,
                            near, near + m4_nut_depth, "M4", "motor ears", 1))
    for ang in (0.5 * (thetas[0] + thetas[1]), 0.5 * (thetas[4] + thetas[5])):
        th = math.radians(ang)
        er = Vector(math.cos(th), math.sin(th), 0.0)
        z = 0.5 * (wall + 1.0 + 10.0)
        p0 = Vector(cx + er.x * (r_ow1 + keeper_t), cy + er.y * (r_ow1 + keeper_t), z)
        near = keeper_t + outer_wall_t
        specs.append(_spec("keeper", p0, vmul(er, -1.0), m3_keep_len, 3.0,
                            near, near + m3_nut_depth, "M3", "curve keeper", 1))
    o1 = xform_point(x_tie_out, y_loc_pre, z_j, 0.0, None)
    o2 = xform_point(x_tie_in, y_loc_pre, z_j, 90.0, s2_off)
    for origin, ax, ay, tag in (
            (o1, Vector(1, 0, 0), Vector(0, 1, 0), "j1"),
            (o2, Vector(0, -1, 0), Vector(-1, 0, 0), "j2")):
        for _tag, p, near, far in joiner_screw_defs(origin, ax, ay, tag):
            specs.append(_spec(tag, p, Vector(0, 0, -1), m3_join_len, 3.0,
                                near, far, "M3", "joiners", 1))
    return specs


def is_screw_name(name):
    return ("scr" in name) or name == "cv_m4"


def check_screws(parts, specs, jacks_only):
    if not jacks_only:
        n = sum(1 for name, _shape in parts if is_screw_name(name))
        require(n == len(specs), "placed screws %d, specs %d" % (n, len(specs)))
    plastics = [(name, shape) for name, shape in parts if not is_screw_name(name)]
    for spec in specs:
        if jacks_only and spec["where"] != "tensioner jacks":
            continue
        far_exit = spec["nut_far"]
        for near, far in spec["spans"]:
            nut_t = far - near
            covered = min(spec["length"], far) - max(0.0, near)
            past = spec["length"] - far
            far_exit = max(far_exit, far)
            step("screw %s engagement %.3f mm  nut %.3f mm  tip past far face %.3f mm"
                 % (spec["name"], covered, nut_t, past))
            require(near >= -1e-6 and covered >= nut_t - 1e-6,
                    "%s does not cross its nut (%.3f of %.3f mm)"
                    % (spec["name"], covered, nut_t))
        a = far_exit + 0.05
        b = spec["length"] - 0.05
        if b <= a:
            continue
        d = vnorm(spec["dir"])
        probe = Part.makeCylinder((spec["dia"] - 0.2) / 2.0, b - a,
                                   vadd(spec["p0"], vmul(d, a)), d)
        for pname, pshape in plastics:
            if not bb_hit(probe, pshape):
                continue
            vol = overlap_volume(probe, pshape)
            if vol > 1e-3:
                require(False, "%s bottoms in %s (%.4f mm^3)" % (spec["name"], pname, vol))


def _nut_samples(origin, axis, depth, af, direction, length):
    step_mm = 1.0
    n = max(1, int(math.ceil(length / step_mm)))
    out = []
    for i in range(n + 1):
        h = hex_prism(af, vadd(origin, vmul(direction, length * i / n)), axis, depth)
        out.append(h)
    return out


def check_one_nut(part, name, origin, axis, depth, af, bore_r, insert_dir, insert_len, load_dir):
    # Insertion: the hex, walked from the pocket until it is outside the part.
    length = insert_len
    end = hex_prism(af, vadd(origin, vmul(insert_dir, length)), axis, depth)
    while bb_hit(end, part) and length < 400.0:
        length += 5.0
        end = hex_prism(af, vadd(origin, vmul(insert_dir, length)), axis, depth)
    require(length < 400.0, "%s insertion stays inside the part" % name)
    samples = _nut_samples(origin, axis, depth, af, insert_dir, length)
    for h in samples[:-1]:
        if not bb_hit(h, part):
            continue
        vol = overlap_volume(h, part)
        require(vol <= 1.0e-2, "%s insertion hits material (%.3f mm^3)" % (name, vol))
    require(not bb_hit(samples[-1], part), "%s insertion does not leave the part" % name)
    # Backing: 1.2 mm behind the loaded face, aside from the screw hole.
    web = hex_prism(af, origin, load_dir, nut_land)
    hole = Part.makeCylinder(bore_r, nut_land + 1.0,
                              vadd(origin, vmul(load_dir, -0.4)), load_dir)
    probe = web.cut(hole)
    inside = overlap_volume(probe, part)
    need = abs(probe.Volume)
    step("nut %s  insert %.1f mm  web %.2f mm  probe %.0f/%.0f mm^3"
         % (name, length, nut_land, inside, need))
    require(need > 1.0 and inside >= 0.95 * need,
            "%s backing web is short (%.0f of %.0f mm^3)" % (name, inside, need))


def audit_nuts(motor, plain, tie, frame):
    rv3 = hex_Rv(m3_nut_af)
    a0, a1, b0, b1 = jack_stack()
    # Straight ears. The mouth is the inboard face of the boss.
    for mod in ("s1", "s2"):
        for sign, which in ((-1.0, "lower"), (1.0, "upper")):
            z = nose_z + sign * encl_ear_pitch
            origin = Vector(nose_x, boss_y1 - m4_nut_depth, z)
            check_one_nut(motor, "%s ear %s" % (mod, which), origin, Vector(0, 1, 0),
                          m4_nut_depth, m4_nut_af, m4_clear / 2.0,
                          Vector(0, 1, 0), m4_nut_depth + 6.0, Vector(0, -1, 0))
    # Jack. Block-side nut along the screw; head-side nut from above.
    # Each plate is used on both modules.
    for plate, y, side in ((motor, -block_out / 2.0, "motor"),
                           (plain, wall + block_out / 2.0, "plain")):
        for mod in ("s1", "s2"):
            check_one_nut(plate, "%s jack %s block-side" % (mod, side),
                          Vector(a1, y, nose_z), Vector(-1, 0, 0), m3_nut_depth, m3_nut_af,
                          m3_clear / 2.0, Vector(-1, 0, 0), m3_nut_depth + 6.0, Vector(1, 0, 0))
            check_one_nut(plate, "%s jack %s head-side" % (mod, side),
                          Vector(b1, y, nose_z), Vector(-1, 0, 0), m3_nut_depth, m3_nut_af,
                          m3_clear / 2.0, Vector(0, 0, 1), 5.0 + rv3 + 4.0, Vector(1, 0, 0))
    # Tie bar, one print used at both ends of both modules. Plate nuts from the
    # upright's end; joiner nuts up from below.
    for mod, bar_name in (("s1", "infeed"), ("s1", "discharge"),
                          ("s2", "infeed"), ("s2", "discharge")):
        for y_face, inward, end in ((wall, 1.0, "near"), (wall + inner_width, -1.0, "far")):
            y_load = y_face + inward * nut_land
            origin = Vector(x_tie_in, y_load, tie_bolt_z)
            axis = Vector(0, inward, 0)
            check_one_nut(tie, "%s tie %s plate %s" % (mod, bar_name, end),
                          origin, axis, m3_nut_depth, m3_nut_af, m3_clear / 2.0,
                          Vector(1, 0, 0), tie_half + rv3 + 3.0, Vector(0, -inward, 0))
        for y_h, kind in ((y_loc_pre, "locating"), (y_clr_pre, "clearance")):
            origin = Vector(x_tie_in, y_h, tie_z0 + m3_nut_depth)
            check_one_nut(tie, "%s tie %s joiner %s" % (mod, bar_name, kind),
                          origin, Vector(0, 0, -1), m3_nut_depth, m3_nut_af, m3_clear / 2.0,
                          Vector(0, 0, -1), m3_nut_depth + 6.0, Vector(0, 0, 1))
    # Curve ears, from above. Keeper nuts, from the inner mouth.
    for sign, which in ((-1.0, "lower"), (1.0, "upper")):
        h = sign * encl_ear_pitch
        origin = vadd(A, vadd(vmul(u_drv, s_nut_near), vmul(e_th_drv, h)))
        check_one_nut(frame, "curve ear %s" % which, origin, vmul(u_drv, -1.0),
                      m4_nut_depth, m4_nut_af, m4_clear / 2.0,
                      e_up_drv, v_half + m4_Rv + 5.0, u_drv)
    for ang, which in ((0.5 * (thetas[0] + thetas[1]), "entry"),
                       (0.5 * (thetas[4] + thetas[5]), "exit")):
        th = math.radians(ang)
        er = Vector(math.cos(th), math.sin(th), 0.0)
        z = 0.5 * (wall + 1.0 + 10.0)
        origin = Vector(cx + er.x * r_ow0, cy + er.y * r_ow0, z)
        check_one_nut(frame, "keeper %s" % which, origin, vmul(er, -1.0),
                      m3_nut_depth, m3_nut_af, m3_clear / 2.0,
                      Vector(0, 0, 1), 40.0, er)
    holes = joint_curve_holes()
    bounds = (pad_bounds(holes[:2]), pad_bounds(holes[2:]))
    for i, (x_h, y_h, _dia) in enumerate(holes):
        x0, x1, y0, y1 = bounds[0 if i < 2 else 1]
        direction, length = pad_slot_dir(x_h, y_h, x0, x1, y0, y1)
        origin = Vector(x_h, y_h, (z_j - web) + m3_nut_depth)
        check_one_nut(frame, "pad joiner %d" % (i + 1), origin, Vector(0, 0, -1),
                      m3_nut_depth, m3_nut_af, m3_clear / 2.0,
                      direction, length, Vector(0, 0, 1))


def make_oring_real(theta_a, theta_b, r_g, D):
    # Wraps sit on the pitch circle the stretch uses. The spans are the real
    # tangents between those two circles, axle tilt included. Equal radii make
    # the radii along u1 × u2 the external tangents.
    s = r_g * ca
    u1 = axis_u(theta_a)
    u2 = axis_u(theta_b)
    c1 = vadd(A, vmul(u1, s))
    c2 = vadd(A, vmul(u2, s))
    w = vnorm(vcross(u1, u2))
    radius = D / 2.0
    cr = oring_cs / 2.0

    def tube(p, q):
        # Past the tangent point so the straight run fuses into the arc. The
        # extra 0.25 mm leaves the pitch circle by about 0.006 mm.
        n = vnorm(vsub(q, p))
        p2 = vsub(p, vmul(n, 0.25))
        q2 = vadd(q, vmul(n, 0.25))
        return Part.makeCylinder(cr, vsub(q2, p2).Length, p2, vsub(q2, p2))

    def half(center, axis, other):
        v = vnorm(vcross(axis, w))
        mid = vadd(center, vmul(v, radius))
        if vsub(mid, other).Length < vsub(vadd(center, vmul(v, -radius)), other).Length:
            v = vmul(v, -1.0)
        tangent = vnorm(vcross(axis, w))
        circ = Part.makeCircle(cr, vadd(center, vmul(w, radius)), tangent)
        face = Part.Face(Part.Wire([circ]))
        motion = vnorm(vcross(axis, w))
        angle = 180.0 if vdot(motion, v) >= 0.0 else -180.0
        return face.revolve(center, axis, angle)

    solid = tube(vadd(c1, vmul(w, radius)), vadd(c2, vmul(w, radius)))
    solid = solid.fuse(tube(vsub(c1, vmul(w, radius)), vsub(c2, vmul(w, radius))))
    solid = solid.fuse(half(c1, u1, c2))
    return solid.fuse(half(c2, u2, c1))


def interfere(parts, label, skip_belt, moving_only):
    step("interference %s: %d solids" % (label, len(parts)))

    def moves(name):
        return ("roller_idler" in name or "block_near" in name or "block_far" in name
                or name.endswith("_rod") or "scr_jack" in name)

    tested = 0
    for i, (ni, ai) in enumerate(parts):
        if not moving_only:
            require(ai.BoundBox.ZMin >= -1.0e-3,
                    "%s drops below the table (z=%.4f)" % (ni, ai.BoundBox.ZMin))
        for nj, aj in parts[i + 1:]:
            if moving_only and not (moves(ni) or moves(nj)):
                continue
            if skip_belt and ("belt" in ni or "belt" in nj):
                continue
            if not bb_hit(ai, aj):
                continue
            tested += 1
            ring = ni if ni.startswith("oringR_") else nj if nj.startswith("oringR_") else None
            if ring is not None:
                other = nj if ring == ni else ni
                owned = {"cone_%d" % int(ring.split("_")[1]),
                         "cone_%d" % (int(ring.split("_")[1]) + 1)}
                vol = overlap_volume(ai, aj)
                if other in owned:
                    require(vol <= 1.0e-3,
                            "%s embeds in its groove on %s (%.4f mm^3)" % (ring, other, vol))
                    continue
                require(vol <= 1.0e-3,
                        "%s overlaps %s by %.4f mm^3 at %s" % (ni, nj, vol, label))
                dist = ai.distToShape(aj)[0]
                require(dist > 0.05,
                        "%s touches %s (%.3f mm) at %s" % (ring, other, dist, label))
                continue
            vol = overlap_volume(ai, aj)
            if vol > 1.0e-3:
                require(False, "%s overlaps %s by %.4f mm^3 at %s" % (ni, nj, vol, label))
    step("interference %s pairs %d, none over 1e-3 mm^3" % (label, tested))


def hardware_block(s2_off, straight_stub):
    specs = screw_specs(0.0, s2_off)
    groups = []
    index = {}
    for spec in specs:
        key = (spec["size"], spec["length"], spec["where"])
        if key not in index:
            index[key] = len(groups)
            groups.append({"size": spec["size"], "length_mm": spec["length"],
                           "count": 0, "where": spec["where"], "nuts": 0})
        groups[index[key]]["count"] += 1
        groups[index[key]]["nuts"] += spec["nuts"]
    for g in groups:
        step("hardware %s  %s x %.0f mm  x%d  nuts %d"
             % (g["where"], g["size"], g["length_mm"], g["count"], g["nuts"]))
    nut_m3 = sum(g["nuts"] for g in groups if g["size"] == "M3")
    nut_m4 = sum(g["nuts"] for g in groups if g["size"] == "M4")
    return {
        "screws": [{"size": g["size"], "length_mm": g["length_mm"],
                    "count": g["count"], "where": g["where"]} for g in groups],
        "nuts": [{"size": "M3", "count": nut_m3}, {"size": "M4", "count": nut_m4}],
        "rods": [
            {"diameter_mm": nose_axle_dia, "cut_mm": idler_axle_len, "count": 2,
             "where": "straight idler"},
            {"diameter_mm": nose_axle_dia, "cut_mm": straight_stub, "count": 2,
             "where": "straight driven stub"},
            {"diameter_mm": curve_axle_d, "cut_mm": idler_rod_len, "count": curve_n - 1,
             "where": "curve idler"},
            {"diameter_mm": curve_axle_d, "cut_mm": stub_len, "count": 1,
             "where": "curve driven stub"},
        ],
    }


def roller_to_print(shape):
    # Axis +Y stands up, spigot on top, so the D-flat's load stays in the
    # plane of the layers and the bore is not a hole against the bed.
    p = shape.copy()
    p.rotate(Vector(0, 0, 0), Vector(1, 0, 0), -90.0)
    p = drop_to_bed(p)
    bb = p.BoundBox
    require(bb.ZLength > bb.XLength and bb.ZLength > bb.YLength,
            "roller print is not standing on its axis")
    return p


def export_print(shape, name):
    solids = shape.Solids
    require(len(solids) >= 1, "%s has no solid" % name)
    for i, sol in enumerate(solids):
        z0 = sol.BoundBox.ZMin
        step("print %s solid %d  z %.4f .. %.3f" % (name, i + 1, z0, sol.BoundBox.ZMax))
        require(z0 >= -1.0e-4, "%s solid %d is below the bed (z=%.4f)" % (name, i + 1, z0))
        require(z0 <= 1.0e-3, "%s solid %d floats (zmin=%.4f)" % (name, i + 1, z0))
    export(shape, name)


# -------------------------------------------------------------------- export
def export(shape, name):
    step("export: " + name)
    shape.exportStep(os.path.join(OUT, name + ".step"))
    Mesh.Mesh(shape.tessellate(TESS)).write(os.path.join(OUT, name + ".stl"))


try:
    main()
except Exception:
    step(traceback.format_exc())
    raise
finally:
    _log.close()
