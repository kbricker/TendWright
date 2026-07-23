# Plan #631 — Internal code-review log: `hardware/bench/calibrate.py`

Date: 2026-07-22/23 (US) · Orchestrator: spark (fast-track, dev + review inline)
Reviewers: three independent fresh-context subagents (adversarial, code-quality,
functional) per the TendWright review policy (no CodeRabbit on this repo), plus
a second adversarial **verification pass** on the fix commit.

| Commit | What |
|---|---|
| `6956f42` | Initial tool + docs (reviews ran against this) |
| `6b7a4d9` | Fixes for all actionable review findings |
| `f6b2954` | Fix for the verification pass's new finding (NEW-1) |
| `fb54b32` | Merge to main |
| `ffc55cf` | Follow-up: stale 7.4V voltage references corrected to 12V (Pro-kit follower) |

Validation evidence: a 16-scenario fake-bus harness (monkeypatched FeetechBus /
read_key / flush_input / input; the fake **hard-fails on any torque-enable or
goal write**, so every scenario doubles as proof of the no-motion property) plus
real-CLI graceful-failure runs (bad port and missing file → one-line BenchError +
hint, exit 2). Harness lived in the session scratchpad (not committed); scenario
list at the bottom of this log.

---

## Design decisions made before/at review (and why)

1. **Torque-off + read-only by construction.** The tool never calls
   `set_torque(True)` and never writes a goal position — the only bus writes are
   torque-off. Chosen so calibration has zero motion risk regardless of bugs.
   Both adversarial passes verified the claim against every helper in `bus.py`.
2. **Sign convention is ours.** The per-joint "positive direction" wording in
   `JOINT_POSITIVE` / the bench README **is the convention**; the recorded sign
   says which way the encoder counts for that motion. Future driver/MuJoCo
   mapping consumes it and never re-guesses.
3. **Merge-per-joint file semantics** (`capture --ids 3` keeps other joints) to
   make the horn-remount recapture cheap; corrupt existing file is a hard error,
   never silently clobbered.
4. **Atomic write at the very end** (same-dir `.tmp` + rename): interrupt at any
   phase leaves an existing file byte-identical.
5. **Wrap detection** = consecutive 20 Hz samples jumping > 1500 ticks. Physics
   argument (verified by reviewer): a hand can't move a joint ~132° in 50 ms, and
   a slow crossing always shows a ~4000-tick numeric jump — so no constructible
   false negative.
6. **Review-driven sibling edits allowed** (term.py `flush_input`, monitor.py
   `parse_ids` guards): bug fixes to code the diff consumes are the review loop
   working, not scope creep. Recorded in the plan's touched-surface audit.

---

## Pass 1a — Adversarial review (on 6956f42)

### MAJORs — all fixed in 6b7a4d9

**A1. Direction sign measured against stale rest reading → invertible sign.**
Delta was `read - rest[id]` where rest came from step 2; torque-off joints drift
between phases (handling, gravity), so by joint 4's nudge the encoder could sit
100+ ticks from its recorded rest, and a genuine positive nudge could record
sign −1. A flipped sign is consumed by the future driver → powered joint moves
opposite to command. **Fix:** baseline read fresh immediately before the nudge
prompt; delta measured against it. (Refined again in the verification pass —
see NEW-1.)

**A2. Rest-failure recovery hint silently discarded other joints' sweeps.**
On 3× rest failure during a full run, nothing had been written, but the hint
said `re-run --ids <failing joint>` — following it would produce a file with one
joint and quiet loss of the other five sweeps. **Fix:** error now states NOTHING
from the run was saved and hints re-running with the original `--ids` spec.

**A3. Wrapped joint's stale pre-existing entry survived with a misleading
"NOT saved" message.** Re-capturing a joint that wraps left its old (now wrong —
the fix is a horn remount) entry in the file, which `show` and future consumers
would trust. **Fix:** wrapped ids are dropped from the pre-existing entries too;
a file left holding nothing is removed; messages state the stale removal.

### MINORs — all fixed in 6b7a4d9

- **A4.** Validation let `True`/`1.0` through for id/sign and never tied `name`
  to `id` → fixed with `type(...) is int` checks and `name == JOINT_NAMES[id]`.
- **A5.** Capture could write values its own loader rejects (servo reporting
  ticks past 4095) → shared `_joint_ok` predicate now runs pre-write; offending
  capture refused with a clean error.
- **A6.** No wrap guard on the nudge delta (a nudge across the encoder edge
  could record an inverted sign) → `abs(delta) > WRAP_JUMP_TICKS` re-prompts.
- **A7.** Initial torque-off loop sat outside try/finally → a comm error mid-cut
  skipped the loud `safe_torque_off` cleanup path. Moved inside the try.
- **A8.** Unwritable `--out` surfaced as a raw traceback *after* the whole
  guided session → up-front writability probe (touch+unlink of the `.tmp` path)
  plus OSError→BenchError around the final write.
- **A9.** `parse_ids`: `--ids 1-999999999` materialized a giant list before any
  check; `6-1` gave an unhelpful error; duplicates ran a joint twice → range
  guards with hints in monitor.py; order-preserving dedupe in calibrate.
- **A10.** read_key→input() mixing: a double-tapped Enter at the end of the last
  sweep auto-accepted the rest-pose prompt (recording the hands-on-arm posture
  as "rest", which validates!) → `flush_input()` in term.py, called before every
  `input()` prompt in the capture flow.

### NOTEs — accepted, not fixed (recorded as known residuals)

- **A11.** Windows `read_key` has no non-interactive guard (POSIX raises); a
  detached scripted run could spin at 20 Hz. Capture is inherently interactive;
  accepted.
- **A12.** errors.py EOF hint says "pass --yes" even when --yes was passed —
  shared wrapper, cosmetic, pre-existing.
- **A13.** A single glitched (but comm-successful) position reading would latch
  `wrapped=True` for the run. Not demonstrable on this hardware (read_u16 checks
  comm result); accepted.
- **A14.** All-joints-wrapped run still prompted for rest — incidentally fixed
  in 6b7a4d9 (rest/direction now run only for good joints).
- **A15.** Windows edge cases on `os.replace` with exotic file states (held
  handles without delete-share, symlinked --out, `.tmp` that is a directory) —
  hard to hit, never corrupts the destination; accepted.
- **A16.** `load_calibration` ignores unknown JSON keys (a misspelled field is
  dropped on next merge rather than flagged). Accepted as forward-compat.

---

## Pass 1b — Code-quality review (on 6956f42)

No MAJORs. Verdict: consistent with the bench-tool family (docstring/argparse/
BenchError/run_tool patterns match teach.py essentially line for line; reuse of
parse_ids/read_key/confirm; comment discipline matches bus.py).

Fixed in 6b7a4d9:
- **Q1.** Loader hints said `--out` but the same loader serves `show` (whose
  flag is `--in`) → flag-neutral wording.
- **Q2.** `not a <= b <= c` split across lines + duplicated rest-tolerance
  predicate → `_rest_ok()` helper used by both call sites.
- **Q3.** Backslash line continuation in `_valid_tick` → parentheses (house
  style).
- **Q4.** `import os` solely for `os.replace` → `Path.replace`, os import
  dropped.
- **Q5.** JointCal fields `id/min/max` shadow builtins — **accepted** (they ARE
  the JSON schema; round-trip via `asdict` is the point) with a comment warning
  that renaming fields changes the file format.
- **Q7.** Dict-comprehension constructor call → explicit kwargs.

Deferred (follow-up candidates, not blocking):
- **Q6.** The torque-cut warning + confirm block now exists in three tools
  (monitor, teach, calibrate) — worth a shared `bus.py` helper someday.
- **Q8.** Two unrelated `load_calibration` functions in `hardware.bench`
  (campreview's camera intrinsics vs calibrate's joints) — confusing grep
  target; rename only if either is promoted out of its module.

Docs verdict: README assembly-day renumbering coherent 1→7; sign-convention
table wording matches `JOINT_POSITIVE` verbatim; task-doc insertions at correct
dependency positions.

---

## Pass 1c — Functional review vs plan checklist (on 6956f42)

All six checklist claims **DELIVERED** with file:line evidence; out-of-scope
list respected (no jog/teach wiring, no tick↔radian, no deps — pyproject/uv.lock
untouched). Ran the safely-runnable paths live (help, show on valid/invalid
crafted files, bad-port capture): all exit correctly.

Findings (both MINOR):
- **F1.** Pre-existing, not this plan: on Windows, port auto-detect picks a lone
  motherboard COM port when the adapter is absent. `--port` sidesteps; noted for
  bench days. Not fixed (bus.py behavior shared by all tools).
- **F2.** Duplicate `--ids` ran a joint twice → fixed via the A9 dedupe.

---

## Pass 2 — Adversarial verification (on 6b7a4d9)

All nine fix claims **CONFIRMED FIXED** with line-level evidence; bookkeeping of
the stale-wrap merge walked case-by-case (captured∩wrapped structurally empty;
sound). New findings:

- **NEW-1 (MINOR) — fixed in f6b2954.** The retry path re-read the baseline
  milliseconds after the "nudge further" message — i.e. while the user's hand
  was still at the failed-nudge position; a release-and-sag before the second
  try could still invert the sign. **Fix:** the first baseline is kept across
  small-delta retries (deltas accumulate correctly); after a wrap-sized jump an
  explicit "release the joint, let it settle, press Enter" step runs before a
  new baseline is read.
- **NEW-2 (NOTE, accepted).** If one servo reads out-of-range while another
  joint wrapped, the `bad_caps` abort preempts the stale-entry removal and wrap
  hint. Requires two simultaneous hardware faults; ordering cosmetic.
- **NEW-3 (NOTE, accepted).** Two concurrent captures on distinct ports sharing
  one `--out` can race on the `.tmp` — fails cleanly via the OSError handler,
  destination never corrupted.
- **NEW-4 (NOTE, accepted).** The writability probe checks the directory, not a
  read-only destination file; that failure path lands in the clean BenchError,
  session data still lost. Residual, acceptable.
- **NEW-5 (NOTE, accepted).** parse_ids dedupe applied only in calibrate;
  monitor/teach still process duplicates (harmless there).

---

## Post-merge follow-up (ffc55cf)

Voltage discrepancy Kyle flagged: bench README said "~7.4 V nominal supply" and
set_id's hint said "7.4V supply". Verified online — TheRobotStudio's baseline
BOM is 7.4V servos/5V PSUs for both arms (the source of the stale numbers), but
Kyle's kit is the **Seeed SO-ARM101 Pro**: follower = 6× 12V STS3215 @ 1:345
with the 12V/2A PSU; the 7.4V servos + 5V PSU are the (unbuilt) leader's. Both
references corrected to 12V. Practical check on assembly day: `scan` voltage
column should read ~12.0–12.6 V; ~7.4 V there means a leader servo got mixed in.

Related decision (chat, same evening): the surplus leader servos are NOT
interchangeable spares — mixed lower gear ratios (3× 1/147, 2× 1/191, 1× 1/345)
are physically different gear trains inside the case, not configurable. They
stay shelved as the future teleop leader set.

---

## Harness scenarios (all 16 PASS on final code)

1. Happy path: 6 joints captured, signs recorded, torque never enabled
2. Wrap on joint 2: flagged + excluded, joints 1/3 still saved, error exit
3. Premature Enter: span guard re-prompts, final range correct
4. Ambiguous nudge (+10): retry keeps first baseline; records sign −1
5. Post-rest drift: sign measured vs fresh baseline, not stale rest
6. Nudge across wrap: release-and-settle step, fresh baseline, then sign
7. Rest outside swept range re-prompts; second pose accepted
8. Rest fails 3×: "nothing saved" + full re-run hint, no file written
9. Merge: `--ids 3` re-captured, other 5 joints preserved
10. Re-capture wraps: stale prior entry dropped from file
11. Re-capture wraps on single-joint file: file removed, not left stale
12. Servo reading past 4095: pre-write validation refuses the file
13. Corrupt existing file: hard error with hint, file untouched
14. Ctrl+C mid-sweep: existing file untouched, cleanup torque-off ran
15. Id parsing: reversed/too-wide/unknown rejected, duplicates deduped
16. `show`: valid file prints; missing / bool-id / name-mismatch /
    bool-or-float-sign / empty all rejected

Plus real-CLI checks: `capture --port COM255` and `show --in nope.json` → clean
one-line error + hint, exit 2; `--help` renders the safety docstring.
