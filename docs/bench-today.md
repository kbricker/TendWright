# Bench session — ready to run (as of 2026-07-25)

Everything below is built, merged, and on cell1. Ordered so each item
unblocks the next. Full syntax: `docs/bench-command-reference.md`.

```
ssh -t cell1 && cd ~/TendWright && git pull
```

## 1. Mount the camera (closes #653, #645, #651)

- [ ] Install the printed **60° bracket** on the wall; print the **cap**
      (`cad/camera-mount/camera_mount_cap.stl`) and slide it in — the
      tongue is exactly 2.0 mm, file to fit.
- [ ] Slide the ELP board into the cradle, cable plug through the open
      back. Check: does it seat without force, do the screws sit flush?
- [ ] `uv run python -m hardware.bench.campreview` — window should now
      open **small** (~quarter screen), not fullscreen. *(closes #651)*
- [ ] `uv run python -m hardware.bench.cameras discover` → paste into
      `cameras.json`, set name + location → `cameras check` → `camserve`.
      Open `http://cell1:8081/` from the desk. *(closes #645)*

**Measure while you're there:** the arm's link lengths, to confirm the
model matches the real arm — m1→m2 **107.0**, m2→m3 **116.0**, m3→m4
**135.0**, m4→m5 **60.1**, m5→m6 **31.7** mm. If these check out, the
whole pose library can be authored in sim.

## 2. Ratify the joint conventions (closes #647)

- [ ] `uv run python -m hardware.bench.calibrate show`

Confirm each joint reads the way you'd describe it out loud:

| Joint | Proposed convention | Rest reads |
|---|---|---|
| m1 shoulder_pan | 0 = mid-travel, + = CCW from above | +7.2° |
| m2 shoulder_lift | −90 = folded at rest, + = arm rises | −90.0° |
| m3 elbow_flex | 0 = fully folded, + = opens | +0.0° |
| m4 wrist_flex | 0 = in line with the forearm, + tips the gripper DOWN | +69.8° |
| m5 wrist_roll | 0 = rest, + = CCW head-on | +0.0° |
| m6 gripper | % open | 10% open |

Wrong? Say so, or edit that joint's `frame` in `calibration.json`.

> **m4's row was rewritten 2026-07-30**, and the timeline is the point.
> All four events span seventeen and a half hours:
>
> | when | what | m4 rest reads |
> |---|---|---|
> | `e06ab72` 07-24 21:41 | frame ratified 2090/+1 | **+75.8** |
> | `e09aa34` 07-25 09:39 | *this row written, "+75.8°"* | **+75.8** — correct |
> | `99aa56e` 07-25 14:23 | `positive` → −1 | **−75.8** — row now wrong |
> | `146adbf` 07-25 15:24 | re-ratified 2158/+1 | **+69.8** — wrong again |
>
> So the row was CORRECT when written and was invalidated four hours
> later by a commit that changed a frame, then again an hour after that,
> with nobody re-reading it either time. An earlier version of this note
> said it was "already stale before 99aa56e touched it", which inverts
> the causality and turns a specific failure into "docs rot on their
> own". It is the same mechanism as the j1/j5 story below — a commit
> that never mentions the thing it breaks — and that is worth more than
> the correction itself. The `+75.8°` is kept on purpose: it is the
> reading that exposed the inverted frame in the first place.

## 3. Verify the twin's two provisional joints (unblocks the pose library)

The twin's m2/m3/m4 geometry is confirmed — it predicted both real
collisions. **m1 (pan) and m5 (roll) are unverified guesses.**
*(As written on 07-25. Both were verified that day — and both were
reverted the next. See the outcome below; the original wording is kept
because what happened to it is the point.)*

- [x] `uv run python -m hardware.bench.jog --id 1` — jog positive, watch
      the arm. Does it swing counterclockwise seen from above?
- [x] `uv run python -m hardware.bench.jog --id 5` — jog positive. Does
      the gripper roll counterclockwise seen head-on?

Either answer "no" → tell me, it's a one-constant flip. Until both are
confirmed, the twin's pan/roll predictions carry an asterisk.

*"A one-constant flip" is exactly how they were reverted — see below.*

> **OUTCOME — and it is not the one this checklist expected.**
>
> **Both boxes were ticked, on 2026-07-25 (99aa56e).** Kyle jogged both
> joints; m1 came back correct as shipped, m5 flipped −1 → +1, verified
> two ways and with the roll's viewpoint ambiguity written down. This
> section did its job.
>
> **Then #670's model swap undid both, on 07-26.** It replaced
> `JointMap`'s per-joint `direction` field with a hardcoded `+1`. That
> constant equalled m1's effective sign, so m1 looked untouched — but
> the SO-100's pan axis is +Z and the SO-101's is −Z, so the same number
> now meant the opposite rotation. m5 was simply reverted to its
> pre-bench value. **A bench answer can be undone by a commit that never
> mentions it and changes no number you would think to check.**
>
> Found five days later, on 07-30, the only way it could be: the arm was
> parked at j1 +45 (tick 1578) and Kyle looked at it — *"thats the arms
> left"* — while the twin put the tool at y −117.7, its right. Nothing
> failed in between and nothing could. m1's sign cannot change a
> collision verdict at all; m5's could not either while every pose in
> the library had j5 = 0.
>
> Fixed in `Twin._rest_direction`, which DERIVES both directions from
> the model instead of storing them — and which, fed the retired SO-100's
> axes, returns exactly the two answers Kyle measured on 07-25. Plan
> 714.6 carries the detail.
>
> The lesson is not "verify the joints". They were verified. It is that
> **a verified fact stored as a constant has no defence against a
> refactor**, and the thing it was protecting never complains.

## 4. Exercise the arm — the real test (closes #643)

- [ ] Dry-run first, no hardware: `uv run python -m sim.twin exercise`
      → expect **CLEAR**.
- [ ] Then for real: `uv run python -m hardware.bench.exercise`

What's different from the run that failed:
- it **simulates the whole routine first** from the arm's measured pose
  and refuses if it predicts contact
- during the shoulder sweep the elbow **and wrist** hold 90° open and the
  sweep is capped at 40% span (45° was not enough — the twin showed the
  wrist stack hitting the table at one end and the shoulder at the other)
- it **reads the encoders** to confirm those holds before starting the
  sweep, and re-checks every sample while it runs — a sag over 2.6°
  halts and holds
- everything prints in degrees

Any key = e-stop (halts and HOLDS; torque cuts on your Enter). If it
refuses with a contact report, believe it before reaching for
`--no-gate`.

## 5. Optional, if there's time

- [ ] Wire more cameras: hub on a main port, UARTs stay direct, label the
      ports, 4 m route max per camera. Then discover → paste → check.
- [ ] Set `still_interval_s` on a camera and confirm frames land in
      `stills/<name>/` with no viewer open.
- [ ] `uv run python -m sim.rig spec` and compare to your calipers.
- [ ] Reboot cell1 and re-run `cameras check` — confirms by-path
      identities survive (the last half of the USB topology item).

## What I need back

1. Do the link lengths match?
2. Do the joint conventions read right, or which ones are wrong?
3. m1 and m5 jog directions — do they match the descriptions above?
4. Did exercise complete, or what did the gate/guards say?
5. The camserve URL working from the desk, so I can pull snapshots.
