# What the cameras can actually read

Root reference for perception. Every number here is **measured on this
cell with these cameras** — not a datasheet figure and not a rule of
thumb — because the two times we reasoned from a rule of thumb instead
of a measurement we got it wrong in both directions: once too
pessimistic (writing off a camera that works), once too optimistic
(sizing a tag that will not fit).

Written 2026-07-30, from a lit frame plus the 716.4 mount measurements.

---

## 1. tag36h11 geometry, so the sizes below mean something

A tag has three nested parts and people quote different ones, which is
where sizing mistakes come from:

```
  6 x 6  data cells
  8 x 8  BLACK SQUARE  = data + 1 cell of black border   <- "tag size"
 10 x 10 STICKER       = black + 1 cell of white quiet zone each side
```

So **cell = black / 8** and **sticker = black x 1.25**.

| "tag size" (black) | cell | printed sticker |
|---|---|---|
| 40 mm | 5.0 mm | 50 mm |
| 20 mm | 2.5 mm | 25 mm |
| 10 mm | 1.25 mm | 12.5 mm |

When someone says "a 40 mm tag", the thing you have to find room for is
**50 mm**. That is what made 40 mm impossible on this arm.

## 2. The detection floor is ~4 px/cell, not 5

Measured from one lit `bench` frame, 1920x1080, twelve tags detected by
our own binding (`hardware/bench/apriltag.py`):

| tags | black square | px/cell | decision margin |
|---|---|---|---|
| 4 x 40 mm, frame corners | 53–81 px | 6.7–9.1 | 119–165 |
| 8 x 20 mm, sheet, mid-frame | 32–37 px | **4.0–4.2** | **173–190** |

**The 20 mm tags scored HIGHER margins than the 40 mm ones**, because
they sat nearer frame centre and were less foreshortened. Detection at
4.0 px/cell is not marginal here — a margin of ~180 is far above the
threshold where decoding gets unreliable.

The "~5 px/cell in both axes" figure used in #716.4 was a conservative
rule of thumb. Treat **4.0 px/cell as demonstrated**, and anything below
it as needing its own measurement before being relied on.

## 3. What kills detection is the ANGLE, not the camera

#716.4 found `low` reading zero tags with a full sheet in view, and the
conclusion recorded there is easy to over-apply. The actual finding is
narrow and worth restating exactly:

> A near-horizontal camera cannot read tags lying **flat on the table**.

Measured at a 9.1 deg grazing angle: a flat 40 mm tag presents 6.3 mm
deep, about 110 x 17 px — **2.2 px/cell**; a flat 20 mm tag presents
1.1 px/cell. Both hopeless, at any focus and any resolution we have.

**The same camera reads a tag on a VERTICAL face at 13.7 px/cell
(40 mm).** That is not a marginal pass, it is six times the floor.

So the rule is about tag-plane vs camera-axis angle. `low` is not a
weak camera; it is a camera that cannot see the table plane. Anything
standing up in front of it reads better than `bench` manages looking
down at the table.

## 4. Scale, per camera

| camera | geometry | measured px/mm |
|---|---|---|
| `bench` | 60 deg down, lens ~676 mm above table | **~1.7** at table height, mid-frame (20 mm tag -> ~34 px) |
| `bench` | same, frame corners | ~1.4 (foreshortened) |
| `low` | 7 deg down, along the table | **~2.74** on a vertical face (from 716.4's 13.7 px/cell at 40 mm) |

`bench` gets *better* on the arm than on the table — a gripper working
150 mm up is ~1.3x closer to the lens, so about 2.2 px/mm.

**UNRESOLVED, and it matters:** eyeballing the gripper's apparent size
in a `low` frame (about 450 px across a ~107 mm body) suggests ~4.2
px/mm, which disagrees with 716.4's 2.74. The two were taken at
different distances and one is an eyeball. **Use 2.74 until someone
measures a real tag on the gripper**; the optimistic figure is what
made a 10 mm jaw tag look comfortable when it is not (see below).

## 5. What fits on the arm

Body bounding boxes from `so101_new_calib.xml` — these are **AABBs of
curved castings, not flat faces**, so treat them as an upper bound on
available area:

| body | bounding box (mm) | 25 mm sticker (20 mm tag)? |
|---|---|---|
| base | 91.8 x 111.7 x 135.5 | yes |
| shoulder | 46.1 x 70.7 x 123.4 | yes |
| upper_arm | 78.2 x 68.5 x 142.5 | yes |
| lower_arm | 96.2 x 65.5 x 132.7 | yes |
| wrist | 36.0 x 71.6 x 94.7 | tight — ~5 mm each side |
| gripper | 55.8 x 64.7 x 106.9 | yes |
| moving_jaw | 20.0 x 48.0 x 92.7 | **no** |

A 50 mm sticker (the "40 mm tag") fits nowhere useful. **20 mm is the
working size for this arm.**

The moving jaw at 20 mm wide needs a 10 mm tag (12.5 mm sticker), which
at the conservative 2.74 px/mm gives only **3.4 px/cell — below the
demonstrated floor.** So tagging the jaw to measure gripper opening is
plausible but NOT yet supported by measurement. Confirm before building
on it.

## 6. Which camera answers which question

Not "which camera is better" — they answer different questions, and the
split follows from geometry:

| question | camera | why |
|---|---|---|
| where is the gripper over the table | `bench` | it sees the table plane; `low` cannot resolve position along its own sight line |
| how high is it | `low` | the only view that resolves height |
| wrist roll (j5) | `low` | when the arm reaches along `low`'s sight line the roll axis points at the camera, so roll is clean IN-PLANE rotation. From `bench` the same roll tips the tag out of plane and foreshortens it |
| gripper open/closed | `low` | jaw separation is a lateral, face-on displacement; from above it is foreshortened by however far the arm reaches |
| is the world where the clip assumed | `bench` | table-plane tags, which is what fixtures and parts sit on |

The roll case is the one that inverts intuition: the overhead camera is
worse at measuring an angle whose axis it is looking down.

## 7. Consequences for how we build things

- **Tags for `low` go on vertical faces.** The side of a part, the side
  of a fixture, the side of the gripper. Never the table, never the top
  of a part.
- **Tags for `bench` can lie flat.** That is what the four corner tags
  and the sheet already do.
- **Arm tags are 20 mm**, paper or label only for now. Anything rigid is
  geometry the twin does not model, and #717.4 records the rule: the arm
  must never run carrying something the collision gate does not know
  about. A plate near the wrist also eats into the 3.1 deg sag tolerance
  that was derived for a bare wrist (#649).
- **A tag needs a genuinely planar patch.** The links are curved and
  black; the white quiet zone matters as much as the code, so a tag half
  on a curve is not a smaller tag, it is a missing one.

## 8. How to extend this file

Every row above came from either a detection or a caliper. When adding:
measure, state the conditions (lighting, distance, which camera), and
say plainly which numbers are estimates. The one estimate in here is
flagged twice, because it is the one that already misled a
recommendation within an hour of being made.

Reproduce the px/cell measurement with:

```bash
# on cell1 — libapriltag is Linux-only, there is no Windows build
curl -s -o /tmp/f.jpg "http://127.0.0.1:8081/cam/bench/snapshot?tags=1"
uv run python -c "
import cv2, numpy as np
from hardware.bench.apriltag import Detector
g = cv2.cvtColor(cv2.imread('/tmp/f.jpg'), cv2.COLOR_BGR2GRAY)
d = Detector()
for t in d.detect(g):
    p = np.array(t.corners)
    s = [np.linalg.norm(p[i] - p[(i + 1) % 4]) for i in range(4)]
    print(t.tag_id, f'{min(s):.1f}-{max(s):.1f} px', f'{min(s) / 8:.1f} px/cell',
          f'margin {t.decision_margin:.0f}')
d.close()"
```
