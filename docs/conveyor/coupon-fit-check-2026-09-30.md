# Coupon fit check — 2026-09-30

Plans 835.1 and 835.2. Try each fit by hand. Get the calipers out only when a check fails: measure what that check names and send me the numbers.

Printed holes, slots and pockets come out about 0.3 mm smaller than their CAD size. That's expected and already allowed for, so a reading under the CAD size is not a fail on its own.

## Which piece is which

- **Motor end** (`coupon_bracket_end`): the tall thin plate with one big round hole and two hex pockets.
- **Infeed end** (`coupon_infeed_end`): the shorter plate with a rail and a slot, plus the loose small block. The block is a real part, the left tensioner block. Keep it.
- **Curve** (`coupon_curve`): the big curved wedge.

## You need

- Both PLA rollers
- A Ø4 rod and a Ø3 rod
- Two M3 nuts and an M3 × 16 screw
- Two M4 nuts and an M4 screw

## Straight: do these before printing the straight's PETG

**1. Rollers on the Ø4 rod.** Slide each roller onto the rod and spin it. The driven roller takes the rod from its plain end only. The rod stops about 46 mm in, and that's right.
- Pass: spins freely, no rattle.
- Fail, measure: the roller's hole (CAD 4.4 mm) and the rod (4.0).

**2. Stub in the motor end's round hole.** Put the driven roller's short stub end into the big round hole and turn the roller.
- Pass: goes in easily with a little play, and turns without rubbing.
- Fail, measure: the hole (CAD 8.0) and the stub (CAD 7.0).

**3. M4 nuts in the motor end.** Press an M4 nut into each hex pocket, then thread the M4 screw in from the other side.
- Pass: each nut sits flat and can't spin, and the screw threads in.
- Fail, measure: the pocket across its flats (CAD 7.3) and the nut across its flats (7.0).

**4. Rod in the slot.** Slide the Ø4 rod along the slot in the infeed end.
- Pass: runs end to end without catching.
- Fail, measure: the slot's width (CAD 4.5) and the rod (4.0).

**5. Block on the rail.** Slide the block's groove over the rail on the infeed end's outside face.
- Pass: slides by hand without forcing, and doesn't flop.
- Fail, measure: the rail's height (CAD 3.0) and the groove's width (CAD 3.3).

**6. Rod in the block.** Push the Ø4 rod into the block's round hole.
- Pass: goes in with a firm push and stays put. It's meant to be tight: the roller turns on the rod, and the rod doesn't turn in the block.
- Fail (won't go in, or falls out), measure: the hole (CAD 4.2).

**7. Nuts in the boss.** Push one M3 nut into the boss along the screw line, from the open end. Drop the other into its slot from the top. Thread the M3 × 16 screw through both.
- Pass: both nuts seat and can't spin, and the screw goes through both.
- Fail, measure: the pocket across its flats (CAD 5.8) and the nut across its flats (5.5).

**All seven pass:** print the straight's PETG: `bracket_straight_motor`, `bracket_straight_plain`, `tie_bar` × 2 and `tensioner_block_right`.

**Any fail:** send me the numbers. The CAD gets fixed, and that coupon reprints before the straight prints.

## Curve: doesn't hold up the straight

Now:

**8. Ø3 rod in the axle holes.** Slide the Ø3 rod into each small round hole. The ones in the inner wall are blind, so the rod stops in them.
- Pass: slides in snug, no wobble.
- Fail, measure: the hole (CAD 3.3) and the rod (3.0).

**9. M4 nuts from the top.** Drop an M4 nut into each ear pocket from the top, then thread the M4 screw through the ear hole.
- Pass: the nut drops in and sits, and the screw threads in.
- Fail, measure: the pocket across its flats (CAD 7.3).

After the TPU cones print:

**10. Cones on the rods.** Each cone turns freely on its Ø3 rod in the coupon.
- Fail, measure: the cone's hole (CAD 3.7).

**11. O-ring test.** Loop one 20 × 2 ring from the kit onto groove A of both cones, the groove next to the big end, before the rods go in. Drop the cones in, slide the rods in, and turn the driven cone by hand.
- Pass: the ring stays seated, the idler turns with it, and nothing binds.
- Fail, measure: the ring's inside diameter and its thickness.

## When the motors arrive

These don't hold up any print.

- **12.** The motor's ears line up with the M4 holes, on the motor end and on the curve.
- **13.** The motor shaft slides into the driven roller's D-shaped hole, and the driven cone's, and turns each without slipping.
- **14.** Caliper the motor's enclosure: the depth from the face of the ear plate to the gearbox face. The CAD assumes 2.5 mm. Send me the number.

## Nothing to check

The two joiners and the curve keeper have only screw holes.

## How to measure

- Zero the calipers with the jaws closed.
- Rods, stubs, rails and nuts: the big lower jaws. Close them snug; don't squeeze.
- Nuts: flat side to flat side, not corner to corner.
- Holes, slots and pockets: the small upper jaws. Open them inside until they touch both walls, rock a little, and take the biggest reading.
- Holes under about 5 mm: the jaw tips barely fit. Find the biggest drill bit that slides in, then measure the bit's shank.
- Measure twice, and write each number down.
