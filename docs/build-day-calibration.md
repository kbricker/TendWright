# Build-Day Runbook: Servo Setup & Calibration (SO-101 follower)

The single sequence to follow while assembling and bringing up the arm.
Everything runs on **cell1** from the repo root:

```
ssh cell1
cd ~/TendWright
```

All tools are `uv run python -m hardware.bench.<tool>`. They fail with a
clean one-line error + hint if hardware is missing — safe to run any time.

**Wiring:** 12V PSU → driver board power terminals; driver board → cell1 USB;
servos on the 3-pin bus. USB alone cannot power servos. The 12V supply is the
follower's (Pro kit); the 5V one belongs to the shelved leader set.

---

## Phase 1 — Per servo, BEFORE its joint closes up

Repeat for each servo, base → gripper = ID 1 → 6:

1. **12V OFF.** Connect exactly ONE servo to the driver board.
   (Two factory-fresh servos share the default ID and answer as one — the
   tool's guard cannot see that. One servo physically connected, always.)
2. **12V ON.**
3. **Program the ID:**
   ```
   uv run python -m hardware.bench.set_id --new-id 1     # then 2 … 6
   ```
   It scans (~10 s), shows the servo's voltage/temp as a health check
   (expect **~12.0–12.6 V** — if it reads ~7.4 V, a leader servo got mixed
   in; swap it out), asks y/N, writes, verifies.
4. **Center it to 2048:**
   ```
   uv run python -m hardware.bench.jog --id 1            # matching ID
   ```
   Press `t` (torque on — it holds), then `c` (moves to center), wait for
   it to settle, then `q` (quit, torque off).
5. **Tape a label with the ID on the servo.** From now until its horn is
   screwed on, don't rotate the output shaft — if it gets bumped, just
   re-run step 4.
6. **12V OFF**, swap to the next servo.

### Why centering matters
The encoder is single-turn 0–4095 and **wraps**. Install each horn/bracket
with the servo at 2048 and the printed part at its neutral pose per the
assembly guide (use the printed jig) so the joint's whole travel stays
inside one revolution. A horn mounted across the wrap makes readings jump
0↔4095 mid-swing — `monitor` and `calibrate` both detect this, but the fix
(re-mount one spline tooth over) is much cheaper before the joint is buried.

---

## Re-spotting a horn (the wrap fix)

If `monitor` (or a `** WRAP **` in calibrate) shows a joint's readout
flipping 0↔4095 mid-swing, the horn is mounted across the encoder wrap.
Disassemble down to that horn, then:

1. **Re-center the servo** — it rotated during disassembly. With the horn
   off:
   ```
   uv run python -m hardware.bench.jog --id 1        # matching ID
   ```
   These are single KEYPRESSES inside the tool (no Enter):
   `t` = torque ON (servo stiffens) → `c` = go to center 2048 (it moves) →
   wait for the readout to settle at ~2048 → `q` = quit, torque off.
   Careful: any unbound key in jog is an E-STOP that exits — touch only
   the keys you mean. Don't rotate the shaft after centering.
2. **Hold the joint part at its mid-travel orientation** (joint 1: base
   pointing straight ahead, halfway between its swing extremes) and drop
   the horn onto the spline at the closest-fitting tooth. The spline
   quantizes to ~160 ticks per tooth — landing within a tooth or two of
   2048 (±300) is fine.
3. **Verify before re-burying it:**
   ```
   uv run python -m hardware.bench.monitor --ids 1
   ```
   Sweep end to end. PASS = no 0↔4095 flip anywhere, and min/max each at
   least ~150 ticks away from the ends. Mid-swing near 2048 is the
   nice-to-have; no-flip plus the margins are the requirement.

The exact centering error doesn't matter beyond that — `calibrate capture`
measures and stores the true min/rest/max later. The only unforgivable sin
is travel that touches the wrap.

---

## Phase 2 — After the full chain is wired

1. **Everything answers:**
   ```
   uv run python -m hardware.bench.scan
   ```
   Expect exactly IDs 1–6, ~12 V, room temperature.
2. **Wiring order + wrap check:**
   ```
   uv run python -m hardware.bench.monitor --ids 1-6
   ```
   It cuts torque after a confirm — **support the arm, it drops**. Move each
   joint by hand: the matching column (and only it) should change, and no
   readout should jump 0↔4095 mid-swing. A jump = horn across the wrap →
   re-mount it one tooth over now.

---

## Phase 3 — Calibration (the main event)

```
uv run python -m hardware.bench.calibrate capture
```

Torque stays OFF the whole time — the tool cannot move the arm; you move
the joints by hand. Three guided steps:

1. **Range sweeps** (one joint at a time): slowly move the joint end to end
   — a few seconds per direction, not a flick. The live line shows
   `pos / min / max / span`. When min and max stop changing, press Enter.
   - `** WRAP **` on the line → that horn crosses the encoder wrap. The
     tool will finish, save the good joints, and tell you which to re-mount
     and re-capture.
   - "span is only N ticks" → it won't accept an Enter without a real
     sweep; keep going.
2. **Rest pose** (once): pose the whole arm at its neutral/rest pose per
   the assembly guide, hands off as much as possible, press Enter. If a
   joint reads outside its swept range, your sweep missed travel — it lets
   you re-pose twice, then tells you to re-run.
3. **Direction nudges** (one joint at a time): push the joint a little in
   its stated positive direction — the prompt tells you which way, e.g.
   joint 1 "arm swings counterclockwise, viewed from above" — hold it
   there, press Enter. It needs ≥30 ticks of movement; if it says the
   nudge was too small, push further **from where you are** and press
   Enter again (don't release and restart).

Result: `calibration.json` in the repo root — per joint: min / rest / max /
sign. The write is atomic and re-runs merge per joint.

**Afterwards:**
```
uv run python -m hardware.bench.calibrate show      # inspect any time
git add calibration.json && git commit -m "Follower calibration" && git push
```

**Redo a single joint** (e.g. after re-mounting horn 3):
```
uv run python -m hardware.bench.calibrate capture --ids 3
```
The other joints' entries are kept.

---

## Phase 4 — First powered motion

One joint at a time, hand near the power switch. Use the measured range
from `calibrate show`, pulled in ~100 ticks:

```
uv run python -m hardware.bench.jog --id 2 --min <min+100> --max <max-100>
```

`t` torque on, `+`/`-` small steps, `[`/`]` step size, `q` quit — any other
key is an E-STOP (torque off, exit).

Then a first trajectory, workspace clear:

```
uv run python -m hardware.bench.teach record --out wave.json
uv run python -m hardware.bench.teach replay --in wave.json --speed 0.25
```

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| "no servo answered" | 12V off, bad cable, or two servos sharing an ID — one servo only during set_id; check the supply actually switched on |
| Voltage reads ~7.4 V | Leader servo mixed into the follower pile — swap it; follower servos are the 12 V set |
| `** WRAP **` during sweep | Horn mounted across the 0/4095 boundary — re-mount one spline tooth over, re-run `calibrate capture --ids N` |
| Rest pose "outside its swept range" | Sweep missed part of the joint's travel — re-pose, or re-run capture for that joint |
| Nudge "moved only ±N ticks" | Push further from the current position and press Enter again |
| "reading jumped across the encoder wrap" during a nudge | Joint is at an end stop near the wrap — follow the release-and-settle prompt, nudge from mid-range |
| Servo holds/hums after a tool dies | Servos latch their last command while powered — the power switch is the real e-stop |
| `uv: command not found` over ssh one-liners | Run the PATH fix: `sudo ln -st /usr/local/bin ~/.local/bin/uv ~/.local/bin/uvx` |

More detail: `hardware/bench/README.md` (tool reference + safety notes + the
canonical joint sign convention), `docs/arm-build-day.md` (mechanical
assembly), `docs/reviews/plan631-calibrate-review-log.md` (why the tool
behaves the way it does).
