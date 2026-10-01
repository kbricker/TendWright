# Coupon fit check — 2026-09-30

Plans 835.1 and 835.2. Try each fit by hand. Get the calipers out only when a check fails: measure what that check names and send me the numbers.

Printed holes, slots and pockets come out about 0.3 mm smaller than their CAD size. That's expected and already allowed for, so a reading under the CAD size is not a fail on its own.

## The pieces

Three files, four pieces.

- **Tall plate** (`coupon_bracket_end`): tall and thin, with one big round hole and two hex pockets.
- **Medium plate** and **small block**, both from `coupon_infeed_end`. The medium plate has a slot and a rail across its middle. The small block is a real part, the left tensioner block, so keep it.
- **Wedge** (`coupon_curve`): the big wedge-shaped piece, with a wall at its narrow end and a wall at its wide end.

## You need

- Both PLA rollers
- A Ø4 rod and a Ø3 rod
- Two M3 nuts and an M3 × 16 screw
- Two M4 nuts and an M4 screw

## Straight: do these before printing the straight's PETG

**1. Rollers on the Ø4 rod.** Slide each roller onto the rod and spin it. The driven roller takes the rod from its plain end only. The rod stops about 46 mm in, and that's right.
- Pass: spins freely, no rattle.
- Fail, measure: the roller's hole (CAD 4.4 mm) and the rod (4.0).

**2. Stub in the tall plate's round hole.** Put the driven roller's short stub end into the big round hole and turn the roller.
- Pass: goes in easily with a little play, and turns without rubbing.
- Fail, measure: the hole (CAD 8.0) and the stub (CAD 7.0).

**3. M4 nuts in the tall plate.** Press an M4 nut into each hex pocket, then thread the M4 screw in from the other side.
- Pass: each nut sits flat and can't spin, and the screw threads in.
- Fail, measure: the pocket across its flats (CAD 7.3) and the nut across its flats (7.0).

**4. Rod in the slot.** Slide the Ø4 rod along the slot in the medium plate.
- Pass: runs end to end without catching.
- Fail, measure: the slot's width (CAD 4.5) and the rod (4.0).

**5. Block on the rail.** Slide the small block's groove onto the rail across the middle of the medium plate.
- Pass: slides along the rail by hand. Tight is fine; having to force it is not.
- Fail, measure: the rail's height (CAD 3.0) and the groove's width (CAD 3.3).

**6. Rod in the block.** Push the Ø4 rod into the small block's round hole.
- Pass: goes in with a firm push and stays put. It's meant to be tight: the roller turns on the rod, and the rod doesn't turn in the block.
- Fail (won't go in, or falls out), measure: the hole (CAD 4.2).

**7. Nuts in the raised lump.** On the medium plate, push one M3 nut into the raised lump along the screw line, from its open end. Drop the other into its slot from the top. Thread the M3 × 16 screw through both.
- Pass: both nuts seat and can't spin, and the screw goes through both.
- Fail, measure: the pocket across its flats (CAD 5.8) and the nut across its flats (5.5).

**All seven pass:** print the straight's PETG: `bracket_straight_motor`, `bracket_straight_plain`, `tie_bar` × 2 and `tensioner_block_right`.

**Any fail:** send me the numbers. The CAD gets fixed, and that coupon reprints before the straight prints.

## Curve: doesn't hold up the straight

Now:

**8. Ø3 rod in the small holes.** The wedge has a small round hole in its narrow-end wall for each cone, two in all, and one in its wide-end wall. Slide the Ø3 rod into each. The narrow-end holes are dead ends, so the rod stops in them.
- Pass: slides in snug, no wobble.
- Fail, measure: the hole (CAD 3.3) and the rod (3.0).

**9. M4 nuts from the top.** On the wide-end wall, either side of the big round hole, there's a small hole with a hex slot open at the top. Drop an M4 nut into each slot, then put the M4 screw through the small hole from the outside, into the nut.
- Pass: the nut drops in and sits, and the screw threads in.
- Fail, measure: the slot across its flats (CAD 7.3).

After the TPU cones print:

**10. Cones in the wedge.** Fit both cones between the walls, each on a Ø3 rod, and spin each by hand. The driven cone's short stub end sits in the big round hole.
- Pass: each turns freely.
- Fail, measure: the cone's hole (CAD 3.7).

**11. O-ring test.** Loop one 20 × 2 ring from the kit onto groove A of both cones, the groove next to the big end, before the rods go in. Drop the cones in, slide the rods in, and turn the driven cone by hand.
- Pass: the ring stays seated, the idler turns with it, and nothing binds.
- Fail, measure: the ring's inside diameter and its thickness.

## When the motors arrive

These don't hold up any print.

- **12.** The motor's two ears line up with the M4 holes, on the tall plate and on the wedge.
- **13.** The motor shaft slides into the driven roller's D-shaped hole, and the driven cone's, and turns each without slipping.
- **14.** On the motor's housing, measure how far the gearbox face, where the shaft comes out, sits below the flat face with the two ears. Use the calipers' depth rod. The CAD assumes 2.5 mm. Send me the number.

## Nothing to check

The two joiners and the curve keeper have only screw holes.

## How to measure

- Zero the calipers with the jaws closed.
- Rods, stubs, rails and nuts: the big lower jaws. Close them snug; don't squeeze.
- Nuts: flat side to flat side, not corner to corner.
- Holes, slots and pockets: the small upper jaws. Open them inside until they touch both walls, rock a little, and take the biggest reading.
- Holes under about 5 mm: the jaw tips barely fit. Find the biggest drill bit that slides in, then measure the bit's shank.
- Depths: the thin rod that slides out of the end of the calipers.
- Measure twice, and write each number down.
