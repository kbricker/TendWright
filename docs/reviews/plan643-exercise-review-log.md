# Plan #643 — Internal code-review log: `hardware/bench/exercise.py`

Date: 2026-07-23 (US) · Orchestrator: spark (fast-track, dev + review inline)
Reviewers: three independent fresh-context subagents (adversarial
hardware-safety, code-quality, functional) per the TendWright review policy
(no CodeRabbit on this repo), plus an adversarial **verification pass** on
the fix commit, which found three new MAJORs of its own — all fixed.

| Commit | What |
|---|---|
| `43a7df8` | Initial tool + docs (round-1 reviews ran against this) |
| `ab35c6f` | Round-1 fixes: held torque cuts, whole-arm preflight, shared settle helper, strict clamping |
| (this commit) | Verification-pass fixes: SerialException held path, full-calibration requirement, teach settle semantics preserved |

Validation evidence: a 44-scenario fake-bus harness (monkeypatched
FeetechBus / read_key / flush_input / input / confirm; the fake
**hard-fails on any torque-enable against a stale goal register and on any
goal strictly outside the calibrated [min,max]**, so every scenario doubles
as proof of the no-lurch and stay-in-range properties) plus real-CLI
graceful-failure runs (missing file, bad --span, teach missing recording →
one-line BenchError + hint, exit 2). Harness lived in the session
scratchpad (not committed); scenario list at the bottom.

## Design decisions made before/at review (and why)

1. **Refuse, don't recover.** Preflight refuses an uncalibrated arm, a
   partially calibrated file (verification catch), a pose outside the
   calibrated range, or a start away from rest. A human sorts out anomalies;
   the tool never guesses.
2. **The held torque cut is the central safety invariant.** On e-stop,
   settle timeout, comm error (BenchError *and* raw SerialException), or
   Ctrl+C mid-motion: halt at present position (best-effort), then HOLD
   under torque until the operator confirms with a hand on the arm. Torque
   never cuts mid-air unannounced. The only instant-cut paths are ones
   where the arm was verified at rest.
3. **--ids narrows the sweep set only.** Every calibrated joint is pinged,
   preflighted, woken, and held — a sweeping shoulder must never whip a
   limp elbow (adversarial catch; the partial-file variant of the same
   hazard was the verification pass's catch).
4. **Wake without lurch** (goal := present position + speed/accel written
   BEFORE torque-enable, per servo) — jog's documented pattern, now
   enforced by the fake. Dev self-review found teach replay violated it
   (torque-enable before any goal write); fixed here as a review-driven
   sibling edit.
5. **Settle is plant-gated and shared.** `motion.wait_settle` (extracted
   per the quality review; teach's parallel implementation deleted, no
   stub) gates on actual position + stillness, never the command (#604
   lesson). teach's approach keeps its original arrival-only semantics via
   `require_still=False` (verification catch: the stillness gate would
   have been a silent behavior change for replay), and the failure hint is
   caller-supplied because torque state after failure differs per tool.
6. **Strict goal clamping.** The loader tolerates a rest up to 25 ticks
   outside [min,max]; commands don't — every goal is clamped into the
   calibrated range (adversarial catch: the harness had baked the slack
   into its own assertion, laundering the spec deviation).
7. **Pose re-validated after the confirm prompt** — the prompt can sit
   open for minutes (stale-baseline class, same shape as #631's direction
   nudge bug).

## Findings (round 1 → outcome)

**Adversarial (hardware-safety):**
- MAJOR — settle-timeout and Ctrl+C paths cut torque instantly with the
  arm mid-sweep, no support-the-arm hold (only e-stop had one). **Fixed**:
  all held-error paths route through the guided cut.
- MAJOR — preflight pose validated before the confirm prompt, then trusted
  forever. **Fixed**: re-validated post-confirm, pre-energize.
- MAJOR — pre-try exits never touch torque; torque-off baseline never
  enforced (a crashed tool may have left torque latched). **Fixed**:
  baseline `safe_torque_off` after the rest-pose validation (verified
  no-drop); refusal paths deliberately leave torque untouched (cutting
  unseen could drop a latched arm) — hint text now says so.
- MAJOR — `--ids 2` swept the shoulder past unchecked, limp distal joints.
  **Fixed**: all calibrated joints held; --ids sweeps only.
- MINOR — goals commandable 25 ticks outside [min,max]. **Fixed**: strict
  clamp + strict fake assertion.
- MINOR — `halt_all` didn't catch SerialException per joint. **Fixed.**
- MINOR — e-stop blind windows during goal-issuance loops (tens of ms).
  **Skipped**: bounded by bus I/O; queued key picked up at next poll.

**Code-quality:**
- MAJOR — second parallel arrive-detection implementation (vs teach).
  **Fixed**: shared `motion.wait_settle`, teach switched, old code deleted.
- MINOR — fourth copy of the ping-preflight boilerplate. **Skipped**:
  deferred to the bench-toolkit hygiene family (#637) with the other
  shared-preflight extractions.
- MINOR — 83-col line + `SETTLE_TOL_TICKS` doing double duty as the
  preflight range margin. **Fixed**: `PREFLIGHT_RANGE_MARGIN`.
- MINOR — isatty hint copy-pasted from term.py. **Fixed**:
  `term.require_interactive()`, POSIX `read_key` reuses it.
- MINOR — `--ids 7` hint pointed at a command that would itself error.
  **Fixed**: JOINT_NAMES validated first, calibrate-style.
- MINOR — unexplained `max(20, …)` speed floor. **Fixed**: dropped
  (unreachable given the --speed range).
- MINOR — README: exercise buried inside the teach step. **Fixed**: own
  assembly-day step 7.

**Functional:** all 7 checklist items DELIVERED. Two spec-wording gaps
judged intent-met and documented (servo-side speed/accel ramp is the
"interpolation"; stillness-over-samples is the "velocity threshold"). Two
harness weaknesses **fixed**: the estop halt-goal check was vacuous (now
asserts post-estop goals equal the position at halt time via a log
sentinel), and a redundant assertion removed.

## Verification pass (on `ab35c6f`) — new findings, all fixed

- MAJOR — raw `serial.SerialException` bypassed every held-cut handler:
  a transient USB glitch mid-sweep → instant torque cut at height.
  **Fixed**: added to the held-path except tuple.
- MAJOR — a *partial* calibration file recreated the limp-joint whip the
  --ids fix closed. **Fixed**: exercise requires all six joints captured.
- MAJOR — shared settle gave teach a "halted in place / HOLDING" message
  that was false there (its finally cuts torque first), plus a stillness
  gate replay never had (spurious-timeout regression risk on dithering
  gravity-loaded joints). **Fixed**: message makes no torque claim,
  `fail_hint` is caller-supplied, teach uses `require_still=False`.
- MINOR — Ctrl+C races inside the handlers (during halt_all / the held
  prompt's print) skipped the prompt. **Fixed**: guarded.
- MINOR — BenchError path never attempted a halt. **Fixed**: best-effort
  halt on every held path.
- MINOR — nothing exercised teach through the shared helper; unused params
  on `held_torque_cut`. **Fixed**: teach replay harness scenarios added
  (including the lurch-fix assertion); params dropped.

## Harness scenarios (44, all passing)

happy ×5 (exit/torque-off/wake-order/coverage/ends-at-rest), subset ×3
(hold-all + sweep-only-subset), no-file ×2, corrupt, range-refuse ×2,
rest-refuse ×2, stale-pose-after-confirm ×2, torque-latched baseline,
partial-file ×3, unknown-id, rest-outside-range clamp, estop ×4 (incl.
halt-at-present assertion), ctrl-c ×3, serial-loss ×3 (incl. held prompt
on raw SerialException), settle-timeout ×3 (incl. held prompt), span/speed
validation ×2, non-tty, declined confirm, span-window containment,
teach-replay ×3 (shared settle, lurch fix, torque-off).
