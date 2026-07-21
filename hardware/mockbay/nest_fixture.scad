// TendWright mock CNC bay — nest fixture (plan #619, rev 3)
//
// A self-centering pocket the arm drops 40x40x20 mm blanks into, with a
// KW12-3 roller microswitch under the floor as the part-present sensor.
//
// Topology (rev 3, after two adversarial mech reviews): the switch
// BOTTOM-LOADS — it drops vertically into an open-bottom bay resting on a
// SEPARATE printed riser block, so there is no insertion tunnel and the
// lever needs no travel path. The pocket floor has a window the size of
// the whole switch top envelope; the lever rises through it and the roller
// (positioned under the pocket center via roller_x_offset) meets the
// blank. Riser height derives from the MEASURED lever geometry so the
// pressed roller sits ~flush with the blank seat — reprint riser variants
// (+/-0.5) to tune engagement. Retention: a 2 mm dowel/music-wire pin
// (~50 mm, press-fit into the far-wall pilot) per mounting hole, driven
// from the OUTSIDE -Y face. Wires: terminals drop into the riser slot,
// bend into the riser's bottom channel, exit its -X face into the flange
// underside groove, and out the -X edge.
// ORIENTATION: roller toward +X (pocket center). A flipped switch pins
// perfectly and NEVER triggers — sight the roller through the floor
// window before pinning.
//
// Print PETG, pocket up. Re-render: openscad -o nest_fixture.stl nest_fixture.scad
//
// ============================ MEASURE ME =============================
// Values marked MEASURE are nominal guesses and MUST be verified with
// calipers (switch/blank) or the A1 tolerance test (clearances) before
// printing. Full list: hardware/mockbay/README.md + Hive plan #619.
// =====================================================================

// ---- blank ----------------------------------------------------------
blank_xy        = 40.0;  // MEASURE: actual cut wax blank width/depth
blank_h         = 20.0;  // MEASURE: actual blank height
pocket_clear    = 0.30;  // MEASURE: per-side clearance from A1 tolerance test

// ---- pocket ---------------------------------------------------------
pocket_depth    = 12.0;  // blank stands ~8 mm proud for the gripper
chamfer_w       = 5.0;   // 45-degree lead-in width (self-centering funnel)
floor_t         = 3.0;   // pocket floor slab (outside the switch window)

// ---- KW12-3 switch (MEASURE ALL with the real part) -----------------
sw_len          = 27.0;  // MEASURE: body length (x)
sw_w            = 10.4;  // MEASURE: body width  (y)
sw_h            = 16.0;  // MEASURE: body height, base to top, LEVER EXCLUDED
sw_hole_pitch   = 22.0;  // MEASURE: mounting-hole center spacing (x)
sw_hole_h       = 5.0;   // MEASURE: mounting-hole height above the body base
sw_hole_d       = 2.0;   // MEASURE: hole bore (M2 pin/screw)
roller_x_offset = 11.0;  // MEASURE: roller contact point ahead of body center
lever_free_h    = 19.0;  // MEASURE: roller top above body base, lever free
lever_pressed_h = 16.5;  // MEASURE: roller top above body base, fully pressed
sw_term_len     = 6.0;   // MEASURE: bottom terminal protrusion below the base
press_margin    = 0.3;   // pressed roller sits this far below the blank seat
bay_clear       = 0.4;   // MEASURE-ish: per-side switch drop-in clearance (A1 test)

// ---- body -----------------------------------------------------------
block_xy        = 64.0;  // pocket block outer size
flange_xy       = 92.0;  // base flange (clamp/screw to bench)
flange_t        = 6.0;
clamp_slot_d    = 5.5;   // M5 clearance slots in the flange corners
clamp_slot_l    = 10.0;
riser_h         = 9.0;   // separate riser block the switch rests on; its
                         // height TUNES lever engagement — reprint only the
                         // riser to adjust, never the fixture
riser_term_slot = 8.0;   // slot in the riser top for bottom-exit terminals
wire_groove_w   = 8.0;
wire_groove_d   = 3.0;   // groove depth into the flange underside

$fn = 48;

// ---- derived (echoed for review) ------------------------------------
pocket_xy   = blank_xy + 2 * pocket_clear;
// Switch body center sits at -roller_x_offset so the ROLLER lands at x=0
// (pocket center), where the blank presses it.
sw_cx       = -roller_x_offset;
sw_base_z   = riser_h;                            // switch base z (on the riser)
seat_z      = sw_base_z + lever_pressed_h + press_margin; // blank seat (floor top)
floor_bot   = seat_z - floor_t;
block_h     = seat_z + pocket_depth;
bay_x0      = sw_cx - sw_len/2 - bay_clear;
bay_x1      = sw_cx + sw_len/2 + bay_clear;
bay_y       = sw_w/2 + bay_clear;

echo(pocket_xy=pocket_xy, seat_z=seat_z, floor_bot=floor_bot,
     block_h=block_h, sw_base_z=sw_base_z, sw_cx=sw_cx,
     body_top=sw_base_z+sw_h, roller_free=sw_base_z+lever_free_h,
     roller_pressed=sw_base_z+lever_pressed_h);
assert(sw_base_z + sw_h < seat_z,
       "switch body top would touch the seated blank");
assert(lever_free_h > lever_pressed_h, "lever heights inverted");
assert(pocket_xy > 2*bay_y + 12,
       "floor window too wide for the blank to bridge with bearing");

module clamp_slot() {
    hull() {
        cylinder(d = clamp_slot_d, h = flange_t + 2);
        translate([clamp_slot_l, 0, 0])
            cylinder(d = clamp_slot_d, h = flange_t + 2);
    }
}

module body() {
    translate([-flange_xy/2, -flange_xy/2, 0])
        cube([flange_xy, flange_xy, flange_t]);
    translate([-block_xy/2, -block_xy/2, 0])
        cube([block_xy, block_xy, block_h]);
}

riser_x = bay_x1 - bay_x0 - 1;  // riser footprint (0.5 mm gap per side)
riser_y = 2*bay_y - 1;

module riser() {
    // SEPARATE printed part: the switch rests on this inside the open bay
    // (assembly: riser on bench, switch on riser, lower the fixture over
    // both, drive the pins, clamp down). Height sets lever engagement —
    // print a few at ±0.5 mm and pick the one the bench test likes.
    difference() {
        // 1 mm top-edge chamfer eases the blind lower-over
        hull() {
            translate([-riser_x/2, -riser_y/2, 0])
                cube([riser_x, riser_y, riser_h - 1]);
            translate([-riser_x/2 + 1, -riser_y/2 + 1, 0])
                cube([riser_x - 2, riser_y - 2, riser_h]);
        }
        // Terminal slot: through in Y, floor at z=3 (depth must swallow
        // the measured terminals — asserted below)
        translate([-riser_term_slot/2, -riser_y/2 - 1, 3])
            cube([riser_term_slot, riser_y + 2, riser_h]);
        // Bottom wire channel: from under the slot out the -X face, z 0-4
        // (overlaps the slot floor by 1 mm and the flange groove's z 0-3),
        // so wires drop from the terminals and run straight into the
        // fixture's underside groove.
        translate([-riser_x/2 - 1, -3.5, -1])
            cube([riser_x/2 + 1 + riser_term_slot/2, 7, 5]);
    }
}

// Equality at defaults (6.0 == 9-3) is safe: the slot floor at z=3 only
// exists on the 1.6 mm side ledges — terminals narrower than the 7 mm
// bottom channel clear straight down to z~0. Only >7 mm-wide terminals
// could actually bottom on the ledges.
assert(sw_term_len <= riser_h - 3,
       "terminals would bottom on the riser slot floor and lift the switch");
assert(bay_x1 < blank_xy/2 - 6,
       "floor window +X edge leaves <6 mm blank bearing");

module pocket_cut() {
    translate([-pocket_xy/2, -pocket_xy/2, seat_z])
        cube([pocket_xy, pocket_xy, pocket_depth + 1]);
    hull() {  // 45-degree chamfer funnel at the top
        translate([-pocket_xy/2, -pocket_xy/2, block_h - chamfer_w])
            cube([pocket_xy, pocket_xy, 0.01]);
        translate([-pocket_xy/2 - chamfer_w, -pocket_xy/2 - chamfer_w, block_h])
            cube([pocket_xy + 2*chamfer_w, pocket_xy + 2*chamfer_w, 0.01]);
    }
}

module bay_cut() {
    // Open-bottom drop-in bay: from z=0 (through the flange) up to the
    // pocket floor slab; the switch enters from below before the fixture
    // is clamped to the bench (the bench closes the bay).
    translate([bay_x0, -bay_y, -1])
        cube([bay_x1 - bay_x0, 2*bay_y, floor_bot + 1 + 1]);
    // Full switch-envelope window through the floor slab (full bay
    // footprint — any inset ledge would collide with the switch body top,
    // which sits inside the slab band). The lever and roller rise through
    // it to meet the blank. Support is deliberately THREE-SIDED: the blank
    // bears >6 mm on +X and both Y sides (asserted), while the window's
    // -X end runs past the blank edge (and notches the pocket wall base by
    // ~2 mm — harmless, the blank can shift only pocket_clear). Centroid
    // stays deep inside the support hull, so seating is stable.
    translate([bay_x0, -bay_y, floor_bot - 1])
        cube([bay_x1 - bay_x0, 2*bay_y, floor_t + 3]);
    // 1 mm flare at the bay's bottom rim: eases the blind lower-over and
    // absorbs first-layer elephant's foot.
    hull() {
        translate([bay_x0, -bay_y, 1])
            cube([bay_x1 - bay_x0, 2*bay_y, 0.01]);
        translate([bay_x0 - 1, -bay_y - 1, -1])
            cube([bay_x1 - bay_x0 + 2, 2*bay_y + 2, 0.01]);
    }
}

module pin_holes() {
    // M2 cross-pin per mounting hole, driven from the OUTSIDE -Y block
    // face: clearance bore through the -Y wall, pilot into the +Y wall.
    for (dx = [-sw_hole_pitch/2, sw_hole_pitch/2])
        translate([sw_cx + dx, 0, sw_base_z + sw_hole_h]) {
            translate([0, -block_xy/2 - 1, 0])
                rotate([-90, 0, 0])
                    cylinder(d = sw_hole_d + 0.3,
                             h = block_xy/2 - bay_y + 2);  // through -Y wall
            translate([0, bay_y - 0.5, 0])
                rotate([-90, 0, 0])
                    cylinder(d = sw_hole_d - 0.2, h = 8);  // pilot, +Y wall
        }
}

module wire_groove_cut() {
    // Groove in the flange UNDERSIDE from under the bay out the -X edge;
    // wires drop from the open bay bottom and run beneath the flange.
    translate([-flange_xy/2 - 1, -wire_groove_w/2, -1])
        cube([bay_x0 + 2 + flange_xy/2, wire_groove_w, wire_groove_d + 1]);
}

difference() {
    body();
    pocket_cut();
    bay_cut();
    pin_holes();
    wire_groove_cut();
    // flange clamp slots: inner end at r=50 clears the block corner
    // (~45.3) with washer room; outer end 60 stays inside the corner (~65)
    for (a = [45, 135, 225, 315])
        rotate([0, 0, a])
            translate([50, 0, -1])
                clamp_slot();
}

// the riser prints alongside the fixture
translate([flange_xy/2 + 25, 0, 0]) riser();
