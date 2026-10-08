# Eight-cone wedge check — 2026-10-08

Plan 835.3. The corner now has eight smaller cones, about 1 mm apart in the middle of the lane, linked by 16 × 2 rings. This check tries a new wedge, two new cones and the ring before the rest of the corner prints. Try each check by hand, and get the calipers out only when one fails.

The first wedge, its two cones and the keeper belong to the old six-cone corner, so keep them apart from the new pieces. The joiners haven't changed.

## Print

- **PETG:** the new wedge (`coupon_curve.stl`), with the same settings as the first.
- **TPU:** one long cone (`roller_cone_driven.stl`) and one short cone (`roller_cone_idler.stl`). Print them big end down with a brim, using the same settings as the first two cones.
- **PLA, any infill:** a 20 mm cube. In Bambu Studio, use Add Primitive → Cube.

## The pieces

- **Wedge:** it has a wall at its narrow end and a wall at its wide end.
  - Outside the wide-end wall is a thick block. It has a big round hole through it, and at each end a small hole with a hex slot open at the top.
  - The wide-end wall itself has one small hole with no slot.
  - The narrow-end wall has small holes that stop partway in.
- **Long cone:** a short peg sticks out of its wide end. The hole in its narrow end stops partway in.
- **Short cone:** it has no peg, and its hole runs right through.
- Both cones have a short collar with two grooves at the wide end. **Groove A** is the groove nearer the cone.

## You need

- The two rods from the first check: the 70.0 mm piece and the 24.2 mm piece. They fit the new cones as they are.
- One 16 × 2 O-ring from the kit.
- Two M4 nuts and an M4 screw.

## 1. M4 nuts

Drop an M4 nut into each hex slot. Then put the M4 screw through each small hole under a slot, into its nut.
- Pass: each nut drops in and sits, and the screw threads in.
- Fail, measure: the slot across its flats (CAD 7.3) and the nut across its flats (7.0).

## 2. Rods and cones, in your hand

1. Slide the 70 mm rod into each small hole in the walls, then pull it back out.
2. Slide the short cone onto the 70 mm rod. Hold the rod's ends and spin the cone.
3. Push the 24.2 mm piece into the long cone's narrow-end hole until it stops. About 5 mm sticks out. Hold the piece and spin the cone, then leave the piece in.

- Pass: the rod goes into each hole snug, with no wobble. Each cone spins freely, and the short cone doesn't rock on the rod.
- Fail, measure: the wall's small hole (CAD 3.3), the cone's hole (CAD 3.7) and the rod (3.0).

Then slide the short cone back off the rod.

## 3. Long cone in, ring on

1. Roll the O-ring onto the long cone's groove A, over the peg. Half the ring hangs loose.
2. Put the peg into the big round hole from the inside. Push the cone toward the wide end about 4 mm, until the short piece clears the narrow-end wall.
3. Lower the narrow end, line the short piece up with its hole, and slide the cone back until the piece bottoms in the hole.
4. Hold the ring's loose half up out of the way, and spin the cone with a finger on top.

- Pass: the cone spins freely.
- Fail, measure: the big round hole (CAD 8.0) and the peg (CAD 7.0).

## 4. Short cone in, ring linking them

1. Push the short cone's collar end through the ring's loose half, so the ring sits in its groove A.
2. Set the short cone between the walls. Slide the 70 mm rod in from outside the wide-end wall, through the cone, and into the narrow-end wall until it stops.
3. Turn the long cone with a finger on top, both ways.

- Pass:
  - the ring stays in both grooves, and the short cone turns with the long one;
  - it turns smoothly, without a stiff spot, so the ring isn't pulling the cones hard onto their rods;
  - the gap between the cones is about 1 mm in the middle, and the narrow ends don't touch anywhere in a full turn.
- Fail: tell me what it did: slipped, dragged, came off, or the ends rubbed. Then measure the ring's inside diameter and its thickness.

## 5. The cube

Set the cube across both cones, in the middle of the lane, and turn the long cone slowly.
- Pass: the cube sits flat on both cones and rides across without rocking.

## All five pass: print the rest

- **TPU:** short cone (`roller_cone_idler.stl`) × 6. With this check's two, that makes all eight.
- **PETG:** the frame (`curve_frame.stl`), base down, and the keeper (`curve_keeper.stl`).
- **Rods:** seven Ø3 rods at 69.9 mm, plus this check's 24.2 mm piece. Any rods you cut at 70.0 mm for the old corner still fit.
- **Rings:** seven 16 × 2.

## Any fail

Send me what happened and the numbers.

## How to measure

- Holes under about 5 mm: find the biggest drill bit that slides in, then measure the bit's shank.
- Nuts: flat side to flat side, not corner to corner.
- The O-ring is soft, so close the jaws until they just touch. For the inside diameter, open the small upper jaws inside the ring. For the thickness, use the big lower jaws across the ring.
