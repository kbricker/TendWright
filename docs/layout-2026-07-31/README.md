# Cell layout, 2026-07-31

The layout record for plan **716.4**, closing its last item: *"Photograph the
final layout from a couple of angles — cheap now, and the only record of what
the numbers refer to."*

Captured as one correlated set through camserve
(`/capture?label=cell-layout-716.4`), set id `20260731-162655-607`, both frames
**2 ms apart**. Kyle 2026-07-31: *"716.4 can close, you have tons of pictures
from the bench cam."*

| file | camera | mount |
|---|---|---|
| `bench.jpg` | `bench` | printed 60° wall bracket, up since 2026-07-28 |
| `low.jpg` | `low` | edge stand bolted through the table's short face, up since 2026-07-30 |

## Why camera frames rather than a phone photo

These come from the two cameras whose poses `cell.json` carries, so they are a
record of **what those cameras actually see** — which is the thing every
downstream number is expressed against. A phone photo would show the room; these
show the frame of reference.

The tradeoff is real and worth stating: **neither camera can see its own mount**,
so this set does NOT document how the cameras are fixed. `bench` is visible in
`low.jpg` only as the dark mass at upper right, and `low` is not in either frame.
If the mounting ever needs to be reproduced from a picture, that picture does not
exist yet.

## What is in frame

**`bench.jpg`** — top-down over the main table. The arm sits at rest at the top
of frame with the gripper hanging over the surface. The 25 mm tag sheet
(`docs/arm-apriltags-25mm.html`) lies centre-table, with four loose tags set
around it at roughly the corners of the working area. The plywood surface is the
table `main` in `bench.json`.

**`low.jpg`** — the grazing view from the table edge, which is the whole point of
the `low` stand (717.5): it sees objects standing ON the surface rather than
looking down at them. The arm is at left, the tag sheet centre, and the
monitor/keyboard at the back of the bench are outside the arm's reach.

## What this does NOT establish

- **Not a camera pose measurement.** Both cameras are still `planned` in
  `cell.json`, deliberately — see 716.4's camera item. Establishing the true pose
  is 713.8 Stage 2's job.
- **Not an obstacle survey.** Kyle 2026-07-31: *"there are no obstacles in the
  arm's reach, just the table surface and the arm body itself."* The monitor and
  keyboard visible in `low.jpg` are behind the reach zone, not in it.
- **The tags on the table are not fixtures.** They are the 717.5 detection work
  in progress and will move.
