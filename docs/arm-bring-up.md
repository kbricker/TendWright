# Arm Bring-Up — post-build checklist (SO-101 follower)

State as of 2026-07-23: **build complete.** IDs 1–6 programmed, servos
centered before horn mounting, joint 1's horn re-spotted after a wrap catch.
This is the pick-up-where-we-left-off sequence — run it top to bottom when
back at the bench.

Everything from the repo root on cell1:

```
cd ~/TendWright
```

## 1. Roll call

```
uv run python -m hardware.bench.scan
```

- [ ] Exactly IDs 1–6 answer
- [ ] All ~12.0–12.6 V (a ~7.4 V reading = leader servo mixed in)
- [ ] Room temperature

## 2. Wiring order + wrap sweep

```
uv run python -m hardware.bench.monitor --ids 1-6
```

Cuts torque after a confirm — **support the arm, it drops.** Move each
joint by hand through its full travel:

- [ ] The matching column (and only it) changes per joint, base→gripper = 1→6
- [ ] No readout flips 0↔4095 mid-swing on ANY joint
      (a flip = horn across the encoder wrap → `docs/build-day-calibration.md`,
      section "Re-spotting a horn", then re-check that joint)

## 3. Calibration

```
uv run python -m hardware.bench.calibrate capture
```

Torque stays OFF throughout — the tool cannot move the arm. Three guided
steps (full walkthrough in `docs/build-day-calibration.md`, Phase 3):

1. **Sweeps** — each joint slowly end to end, Enter when min/max stop moving
2. **Rest pose** — whole arm at neutral per the assembly guide, once
3. **Direction nudges** — small push per joint in the direction the prompt
   states, hold, Enter (too small = push further from where you are)

Then:

- [ ] `uv run python -m hardware.bench.calibrate show` — all 6 joints listed,
      spans look like real travel (hundreds-to-thousands of ticks), rest
      inside each range
- [ ] Send the `show` output to spark for a sanity check on ranges + signs
- [ ] Commit the result:
      `git add calibration.json && git commit -m "Follower calibration" && git push`

If a joint needs redoing (horn re-spot, bad sweep):
`uv run python -m hardware.bench.calibrate capture --ids N` — other joints
are kept.

## 4. First powered motion

One joint at a time, hand near the power switch. Use ranges from
`calibrate show`, pulled in ~100 ticks:

```
uv run python -m hardware.bench.jog --id 2 --min <min+100> --max <max-100>
```

Keys (single presses): `t` torque on · `+`/`-` step · `[`/`]` step size ·
`q` quit · anything else = E-STOP.

- [ ] Each joint moves the right way in small steps, no grinding, no stall

## 5. First trajectory

Workspace clear:

```
uv run python -m hardware.bench.teach record --out wave.json
uv run python -m hardware.bench.teach replay --in wave.json --speed 0.25
```

- [ ] A hand-taught arc replays smoothly at quarter speed

**Milestone reached:** the arm is calibrated and moving. Next on the bench
list after this: mock-bay measurements and the camera scouting session
(`docs/bench-tasks.md`, sections 3–7).

## References

- `docs/build-day-calibration.md` — full procedure detail + horn re-spot +
  troubleshooting table
- `hardware/bench/README.md` — tool reference, safety notes, joint sign
  convention
- `docs/bench-tasks.md` — the master bench task list
