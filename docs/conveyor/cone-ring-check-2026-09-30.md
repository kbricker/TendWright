# Cone rollers and O-ring check — 2026-09-30

Plan 835.2. This replaces checks 8–11 in the coupon fit check. Try each check by hand, and get the calipers out only when one fails.

## The pieces

- **Wedge** (PETG, `coupon_curve`): it has a wall at its narrow end and a wall at its wide end.
  - The wide-end wall has a big round hole, a small hole with a hex slot open at the top on either side of it, and one more small hole with no slot.
  - The narrow-end wall has two small holes that stop partway in.
- **Long cone** (`roller_cone_driven`): a short peg sticks out of its wide end. The hole in its narrow end stops partway in.
- **Short cone** (`roller_cone_idler`): it has no peg, and its hole runs right through.
- Both cones have a short collar with two grooves at the wide end. **Groove A** is the groove nearer the cone.

Each cone lies with its wide end at the wedge's wide-end wall. The long cone's peg goes in the big round hole. The short cone goes on the side with the small hole that has no slot.

## You need

- One Ø3 rod.
- A 24 mm piece cut off the end of that rod, with the cut end filed smooth. A hair short is fine, but don't cut it long. The rest of the rod is for the short cone.
- Two M4 nuts and an M4 screw.
- One 20 × 2 O-ring from the kit.

## 1. M4 nuts

Drop an M4 nut into each hex slot on the wide-end wall. Then put the M4 screw through each small hole from the outside, into its nut.
- Pass: each nut drops in and sits, and the screw threads in.
- Fail, measure: the slot across its flats (CAD 7.3) and the nut across its flats (7.0).

## 2. Short cone, no ring

1. Set the short cone between the walls.
2. Slide the rod in from outside the wide-end wall, through the cone, and into the narrow-end wall until it stops.
3. Spin the cone with a finger on top.

- Pass: the rod goes in snug with no wobble, and the cone spins freely.
- Fail, measure: the cone's hole (CAD 3.7), the wall's small hole (CAD 3.3) and the rod (3.0).

Then pull the rod and take the cone out.

## 3. Long cone, ring on

1. Roll the O-ring onto the long cone's groove A, over the peg and the end groove. Half the ring hangs loose.
2. Push the 24 mm piece into the narrow-end hole until it stops. About 4 mm sticks out.
3. Put the peg into the big round hole from the inside. Push the cone toward the wide-end wall about 4 mm, until the 24 mm piece clears the narrow-end wall.
4. Lower the narrow end, line the piece up with its hole, and slide the cone back until the piece bottoms in the hole.
5. Hold the ring's loose half up out of the way, and spin the cone with a finger on top.

- Pass: the cone spins freely.
- Fail, measure: the cone's hole (CAD 3.7), the big round hole (CAD 8.0) and the peg (CAD 7.0).

## 4. Both cones, ring linking them

1. Push the short cone's collar end through the ring's loose half, so the ring sits in its groove A.
2. Set the short cone between the walls and slide the rod in, as in check 2.
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
