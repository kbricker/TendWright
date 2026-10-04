# Straight conveyor build — 2026-10-03

Plans 835 and 835.1. This builds one straight and runs it on its own motor. The belt is the only part still to print, so it prints first.

## Print the belt now

- **Belt** (`belt_straight.stl`) × 1, in TPU: the thin tube, 80 mm across and 50 mm tall. Print it upright as exported, with no supports and the same settings as the two test cones.
- It prints on its own, before the cone check, which leaves only the four short cones for that check's last plate.
- Everything else for the straight is already printed.

## The pieces

- **Motor plate** (`bracket_straight_motor`): a long plate, 120 × 35 mm, with a tab sticking up at one end. That end has the big round hole, and it's the motor end.
- **Plain plate** (`bracket_straight_plain`): the same plate with no tab. Its motor end has a small round bump on the outside and a hole on the inside.
- At the other end, the slot end, both plates have:
  - a slot;
  - a short ridge, the rail, on the outside below the slot;
  - a lump on the outside, just past the slot's inner end, with nut slots in it.
- Both plates have two thin grooves along the inside, one long and one short.
- **Two tie bars** (`tie_bar`): bars 55 mm long. Each has a slot in its side near each end, and two hex pockets in one face.
- **Two small blocks.** They're mirror images:
  - the block that printed beside the medium coupon plate goes on the motor plate;
  - the new one (`tensioner_block_right`) goes on the plain plate.
- **Driven roller** (PLA): a short peg on one end, with a D-shaped hole in it.
- **Idler roller**: plain, with a hole all the way through.
- **Bed** (`slider_bed_straight`): the bigger flat piece, 88 × 55 mm, with a thin ridge along each long edge. At one end, the two corners of the face that printed on the plate have small curved nicks.
- **Return guide** (`return_guide_straight`): the smaller flat piece, 64 × 55 mm, with the same ridges.
- **Belt**: the TPU tube.

## You need

- One motor, its case and one input board.
- Two Ø4 rods, cut in step 1: 73.0 mm (the long rod) and 51.3 mm (the short rod).
- Screws: four M3 × 8, two M3 × 16 and two M4 × 8. Nuts: eight M3 and two M4.
- Two female/male jumper wires.
- 2.5 and 3 mm hex keys, the soldering iron, a hacksaw, a file and the calipers.

## 1. Cut the rods

For each rod:
1. Mark it 0.5 mm past the length, and cut there.
2. File the cut end flat and square until the piece measures the length end to end.
3. File the sharp edge off both ends.

Within half a millimetre is fine, but err short rather than long.

## 2. Motor

1. Solder the input board onto the motor's two tabs.
2. Fit the motor into its case so the shaft comes out of the face with the two ears.
3. Plug a jumper's female end onto each of the board's two pins, and lead the wires out through a hole in the case.
4. Measure how far the gearbox face, where the shaft comes out, sits below the ear face. Use the calipers' depth rod. The CAD assumes 2.5 mm. Send me the number; nothing waits on it.

## 3. Nuts first

- **Tie bars:** slide an M3 nut into the slot near each end of each bar, until it stops. That's four nuts.
- **Both lumps:**
  - Push an M3 nut into the pocket on the lump's face toward the slot, and drop another into the slot in the lump's top.
  - Thread an M3 × 16, the jack screw, in from the lump's other end, through both nuts. Stop when its tip just shows at the face toward the slot.
  - Hold the nut in the face with a finger while the screw reaches it.

## 4. Motor plate

1. **Tie bars.** Set one bar's end flat on the inside of the motor plate, over each small hole near the bottom edge, with its hex pockets on the underside. Screw an M3 × 8 in from outside, into the nut.
2. **Motor.**
   - Put an M4 nut in each hex pocket on the inside of the plate: one is on the tab, one at the bottom of the motor end.
   - Set the motor on the outside, with the shaft through the big round hole, and put an M4 × 8 through each ear into its nut.
   - If the ears don't line up with the holes, stop and tell me.

## 5. Fill it

Lay the motor plate inside face up, propped on a small box or a can under its middle, so the motor hangs free. Everything in this step drops in from above.

1. **Driven roller and bed, together.** The bed's nicks hold the roller's rims, so neither can slide past the other. They go in as a pair.
   - Hold the roller against the bed's nicked end, with its rims in the nicks and its peg toward the plate.
   - Lower both at once: the roller's D-shaped hole onto the shaft and its peg into the big round hole, and the bed's ridge into the long groove, nicks toward the tie bars.
   - Turn the roller until the flats line up and both drop. Hold the bed up until the belt is on.
2. **Idler roller.** Stand it at the slot end, over the inner end of the slot.
3. **Belt.** Put it on as one loop over both rollers and the bed, with its edges between each roller's rims.
4. **Return guide.** Slide it in between the belt and the tie bars, with its ridge in the short groove.
5. **Short rod.** Drop it into the driven roller's top hole until it stops. About 5 mm sticks out.
6. **Plain plate.**
   - Lay it on top, inside face down. The short rod goes into the hole at its motor end, both ridges go into its grooves, and its slot sits over the idler roller.
   - Put an M3 × 8 in from outside, into each tie bar.

## 6. Long rod and blocks

1. Press the new small block onto one end of the long rod, until the rod is flush with the block's face that has no groove.
2. Drop the rod through the plain plate's slot, the idler roller and the motor plate's slot. The block lands on the plain plate, with its groove over the rail.
3. Stand the straight up on the bottom edges of its plates.
4. Press the coupon block onto the rod's other end, with its groove over the motor plate's rail, until the rod is flush with its outside.
5. Push both blocks toward the middle, against their lumps.

## 7. Tension the belt

Turn the jack screws in, one turn on each side at a time, until the rod sits at the outer end of both slots. That's about 16 turns each. The hex key goes into each screw's head from the middle of the plate.

- If the screws get hard to turn before the rod gets there, stop, and tell me how far the rod still has to go.
- Then check that the belt sits between each roller's rims and the straight sits flat and square.

## 8. Wire it

Put the Pico and one driver board on the breadboard. Solder the driver's header pins on first.

| Pico | Driver |
|---|---|
| GP0, pin 1 | PWMA |
| GP1, pin 2 | AIN1 |
| GP2, pin 4 | AIN2 |
| GP15, pin 20 | STBY |
| 3V3 OUT, pin 36 | VCC |
| GND, pin 38 | GND |

- The motor's two wires go to the driver's two Motor A pins.
- **Bench supply:**
  - Put the rear switch on 115 V.
  - With the output off, set 12 V and a 0.5 A limit.
  - Supply + goes to VM, and supply − to the driver's GND.
  - Before the output ever goes on, check that the 12 V wire goes to VM and nowhere else.

## 9. First run

Tell me when it's wired. Then:
1. I wake cell1.
2. You hold the Pico's BOOTSEL button while you plug it into cell1's free USB port, with the kit's micro-USB cable.
3. I load the firmware, you turn the supply's output on, and I run the motor slowly while you watch.

- The top of the belt should move toward the motor end. If it goes the other way, swap the two motor wires at the driver.
- It's done when the belt runs between the rims without walking off.

## Later

The joiner to the corner isn't part of this build. Fitting it later means taking the plain plate, the belt and the bed back off, because its screws sit under the belt.
