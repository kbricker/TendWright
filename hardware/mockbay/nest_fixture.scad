// TendWright mock CNC bay — nest fixture (plan #619)
//
// A self-centering pocket the arm drops 40x40x20 mm blanks into, with a
// KW12-3 roller microswitch under the floor as the part-present sensor.
// Print in PETG, pocket up (chamfers face up = no elephant's foot in the
// lead-in). Re-render:  openscad -o nest_fixture.stl nest_fixture.scad
//
// ============================ MEASURE ME =============================
// Values marked MEASURE are nominal/datasheet guesses and MUST be
// checked with calipers (switch) or the A1 tolerance test (clearance)
// before printing. The full list also lives in hardware/mockbay/README.md.
// =====================================================================

// ---- blank ----------------------------------------------------------
blank_xy        = 40.0;  // MEASURE: actual cut wax blank width/depth
blank_h         = 20.0;  // MEASURE: actual blank height
pocket_clear    = 0.30;  // MEASURE: per-side clearance from A1 tolerance test

// ---- pocket ---------------------------------------------------------
pocket_depth    = 12.0;  // blank stands 8 mm proud for the gripper
chamfer_w       = 5.0;   // 45-degree lead-in width (self-centering funnel)
floor_t         = 3.0;   // pocket floor above the switch bay

// ---- KW12-3 switch (MEASURE ALL FIVE with the real part) ------------
sw_len          = 27.0;  // MEASURE: body length
sw_w            = 10.4;  // MEASURE: body width  (incl. clearance)
sw_h            = 16.0;  // MEASURE: body height (base to top, lever down)
sw_hole_pitch   = 22.0;  // MEASURE: mounting-hole center spacing
sw_hole_d       = 2.0;   // MEASURE: pilot for M2 self-tappers
lever_window_l  = 16.0;  // MEASURE: roller-lever travel footprint in floor
lever_window_w  = 6.0;   // MEASURE: window width for the roller

// ---- body -----------------------------------------------------------
block_xy        = 64.0;  // pocket block outer size
flange_xy       = 92.0;  // base flange (clamp/screw to bench)
flange_t        = 6.0;
clamp_slot_d    = 5.5;   // M5 clearance slots in the flange corners
clamp_slot_l    = 10.0;

$fn = 48;

pocket_xy   = blank_xy + 2 * pocket_clear;
block_h     = flange_t + sw_h + floor_t + pocket_depth;
bay_z       = flange_t;                 // switch bay sits on the flange top
floor_z     = bay_z + sw_h;             // pocket floor bottom
pocket_z    = floor_z + floor_t;        // pocket floor top

module clamp_slot() {
    hull() {
        cylinder(d = clamp_slot_d, h = flange_t + 2);
        translate([clamp_slot_l, 0, 0])
            cylinder(d = clamp_slot_d, h = flange_t + 2);
    }
}

module body() {
    // flange
    translate([-flange_xy/2, -flange_xy/2, 0])
        cube([flange_xy, flange_xy, flange_t]);
    // pocket block
    translate([-block_xy/2, -block_xy/2, 0])
        cube([block_xy, block_xy, block_h]);
}

module pocket_cut() {
    // straight pocket walls
    translate([-pocket_xy/2, -pocket_xy/2, pocket_z])
        cube([pocket_xy, pocket_xy, pocket_depth + 1]);
    // 45-degree chamfer funnel at the top
    hull() {
        translate([-pocket_xy/2, -pocket_xy/2, block_h - chamfer_w])
            cube([pocket_xy, pocket_xy, 0.01]);
        translate([-pocket_xy/2 - chamfer_w, -pocket_xy/2 - chamfer_w, block_h])
            cube([pocket_xy + 2*chamfer_w, pocket_xy + 2*chamfer_w, 0.01]);
    }
}

module switch_bay_cut() {
    // Bay under the pocket floor, open to the +X face for insertion/wiring.
    // The switch lies on its side wall, lever up, roller poking through the
    // floor window; screw pilots in the -Y bay wall.
    translate([-sw_len/2, -sw_w/2, bay_z])
        cube([block_xy/2 - (-sw_len/2) + 1, sw_w, sw_h]);
    // lever window through the pocket floor (offset toward -X so the roller
    // sits under the blank's footprint, not its exact center)
    translate([-lever_window_l/2, -lever_window_w/2, floor_z - 1])
        cube([lever_window_l, lever_window_w, floor_t + 2]);
    // wiring channel out through the flange edge
    translate([block_xy/2 - 1, -4, bay_z])
        cube([(flange_xy - block_xy)/2 + 2, 8, 6]);
}

module switch_pilots() {
    // Two M2 pilot holes into the -Y wall of the bay, matching the KW12-3
    // mounting holes (switch screwed against that wall).
    for (dx = [-sw_hole_pitch/2, sw_hole_pitch/2])
        translate([dx, -sw_w/2 - 6, bay_z + sw_h/2])
            rotate([-90, 0, 0])
                cylinder(d = sw_hole_d, h = 8);
}

difference() {
    body();
    pocket_cut();
    switch_bay_cut();
    switch_pilots();
    // flange clamp slots, one per corner, radially oriented. Radius must
    // clear the block footprint (block corner is at block_xy/2 * sqrt(2)
    // = ~45 on the diagonal) but stay inside the flange corner (~65).
    for (a = [45, 135, 225, 315])
        rotate([0, 0, a])
            translate([48, 0, -1])
                clamp_slot();
}
