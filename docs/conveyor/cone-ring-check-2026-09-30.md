# Cone rollers and O-ring check — 2026-09-30

Plan 835.2. This replaces checks 8–11 in the coupon fit check. Try each check by hand. Apart from cutting the rods, get the calipers out only when a check fails.

## The pieces

- **Wedge** (PETG, `coupon_curve`): it has a wall at its narrow end and a wall at its wide end.
  - The wide-end wall has a big round hole, a small hole with a hex slot open at the top on either side of it, and one more small hole with no slot.
  - The narrow-end wall has two small holes that stop partway in.
- **Long cone** (`roller_cone_driven`): a short peg sticks out of its wide end. The hole in its narrow end stops partway in.
- **Short cone** (`roller_cone_idler`): it has no peg, and its hole runs right through.
- Both cones have a short collar with two grooves at the wide end. **Groove A** is the groove nearer the cone.

Each cone lies with its wide end at the wedge's wide-end wall. The long cone's peg goes in the big round hole. The short cone goes on the side with the small hole that has no slot.

## You need

- One Ø3 rod, cut into the two pieces below.
- Calipers and a file.
- Two M4 nuts and an M4 screw.
- One 20 × 2 O-ring from the kit.

## First, cut the rods

Cut two pieces from the one rod:
- **70.0 mm**, for the short cone.
- **24.2 mm**, for the long cone. The steps call it the short piece.

For each piece:
1. Mark the rod 0.5 mm past the length, and cut there.
2. File the cut end flat and square until the piece measures the length end to end, using the calipers' big jaws.
3. File the sharp edge off both ends, so the piece slides into the holes without catching.

A hair short is fine, but long is not.

Both pieces are real parts. The finished corner uses five 70.0 mm rods and this one 24.2 mm piece, so the other four 70.0 mm rods can be cut now too.

Once the 70 mm rod is all the way in the wedge, its end sits just inside the wall, with nothing to grab. So each cone goes into the wedge only once, and check 2 spins them in your hand first.

## 1. M4 nuts

Drop an M4 nut into each hex slot on the wide-end wall. Then put the M4 screw through each small hole from the outside, into its nut.
- Pass: each nut drops in and sits, and the screw threads in.
- Fail, measure: the slot across its flats (CAD 7.3) and the nut across its flats (7.0).

## 2. Rods and cones, in your hand

1. Slide the 70 mm rod into each small hole in the wedge, then pull it back out. That's the two in the narrow-end wall, which stop partway, and the one in the wide-end wall with no slot.
2. Slide the short cone onto the 70 mm rod. Hold the rod's ends and spin the cone.
3. Push the short piece into the long cone's narrow-end hole until it stops. About 4 mm sticks out. Hold the piece and spin the cone, then leave the piece in.

- Pass: the rod goes into each hole snug, with no wobble. Each cone spins freely, and the short cone doesn't rock on the rod.
- Fail, measure: the wall's small hole (CAD 3.3), the cone's hole (CAD 3.7) and the rod (3.0).

Then slide the short cone back off the rod.

## 3. Long cone in, ring on

1. Roll the O-ring onto the long cone's groove A, over the peg and the end groove. Half the ring hangs loose.
2. Put the peg into the big round hole from the inside. Push the cone toward the wide-end wall about 4 mm, until the short piece clears the narrow-end wall.
3. Lower the narrow end, line the short piece up with its hole, and slide the cone back until the piece bottoms in the hole.
4. Hold the ring's loose half up out of the way, and spin the cone with a finger on top.

- Pass: the cone spins freely.
- Fail, measure: the big round hole (CAD 8.0) and the peg (CAD 7.0).

## 4. Short cone in, ring linking them

1. Push the short cone's collar end through the ring's loose half, so the ring sits in its groove A.
2. Set the short cone between the walls. Slide the 70 mm rod in from outside the wide-end wall, through the cone, and into the narrow-end wall until it stops. Its end stops just inside the wall's outside face.
3. Turn the long cone with a finger on top, both ways.

- Pass: the ring stays in both grooves, the short cone turns with the long one, and nothing binds.
- Fail: measure the ring's inside diameter and its thickness, and tell me what it did: slipped, came off, or bound.

## All four pass: print the rest of the TPU

One plate, with the same settings as the first two cones:

- **Short cone** (`roller_cone_idler.stl`) × 4. With the first two, that's all six cones.
- **Belt** (`belt_straight.stl`) × 1: the thin tube, 80 mm across and 50 mm tall. It stands upright as exported and needs no supports.

That's all the TPU for the straight and the corner.

## Any fail

Send me the numbers. The belt doesn't depend on the cones, so print it on its own while the cones get fixed.

## How to measure

- Holes under about 5 mm: find the biggest drill bit that slides in, then measure the bit's shank.
- Nuts: flat side to flat side, not corner to corner.
- The O-ring is soft, so close the jaws until they just touch. For the inside diameter, open the small upper jaws inside the ring. For the thickness, use the big lower jaws across the ring.
