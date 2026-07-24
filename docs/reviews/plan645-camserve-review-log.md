# Plan #645 — Internal code-review log: `hardware/bench/camserve.py`

Date: 2026-07-24 (US) · Orchestrator: spark (fast-track, dev + review inline)
Reviewers: three independent fresh-context subagents (adversarial with a
concurrency/network focus, code-quality, functional) per the TendWright
review policy, plus an adversarial **verification pass** on the fix commit
(all round-1 fixes held; it added two Low findings and called out two
overstated claims — all addressed).

| Commit | What |
|---|---|
| `20feb0b` | Initial tool + docs (round-1 reviews ran against this) |
| `1160779` | Round-1 fixes: stalled-client timeout, stall reaping, shared camera helpers, import hygiene |
| (this commit) | Verification fixes: handler-wide send timeout, per-step cleanup guards, honest claims |

Validation evidence: a 16-scenario fake-camera harness driving the real
server with real HTTP clients/sockets (multipart framing, JPEG magic,
distinct consecutive frames, concurrent clients, mid-stream disconnect,
503-before-first-frame, tags-on detector path, Ctrl+C → exit 130 with
camera-release and blocked-client-unblock assertions, port-in-use and
no-camera refusals), an import-graph assertion (`scservo_sdk` must not
load — the camera tools' import graph is servo-SDK-free), and a real-
camera smoke of the reworked campreview on the desk webcam. Harness in
the session scratchpad (not committed).

## Design decisions (and why)

1. **Zero new dependencies.** stdlib `ThreadingHTTPServer` + existing
   cv2/pupil-apriltags. `ustreamer`/`go2rtc` documented as the upgrade
   path only.
2. **Latest-frame-only delivery.** `FrameBox` is a Condition-guarded
   single-slot buffer: slow clients drop frames, nothing ever queues.
3. **Capture on the MAIN thread, HTTP on a daemon thread** — inverted
   from the plan's wording (functional review flagged the gap; judged
   intent-met): Ctrl+C must land where the cleanup `finally` lives.
4. **Serve thread starts before the try block** so the finally's
   `server.shutdown()` always has a running accept loop to stop
   (shutdown-before-serve_forever hangs); pre-camera clients get 503.
5. **No auth, LAN-only** — documented posture, restated in tool output.

## Findings (round 1 → outcome)

**Adversarial (concurrency/network):**
- MAJOR — a stalled-but-connected client (zero-window TCP) wedged its
  handler thread forever in `wfile.write`; no socket timeout existed.
  **Fixed**: handler-wide `timeout` attribute (verification pass caught
  that the first fix covered only `/stream`, leaving `/snapshot`'s
  >64 KB writes wedgeable — now every request path has it).
- MAJOR — harness validated only the happy serving half: camera-release
  flag dead, Ctrl+C/130 untested, 503-before-first-frame untested.
  **Fixed**: scenarios added for all of it (16 total).
- MINOR — vanished client never reaped while the camera stalls
  (disconnects only surface on write). **Fixed**: ~30 s of empty waits
  drops the client; a live viewer reconnects.
- MINOR — rate-cap slept AFTER fetching, sending a stale frame.
  **Fixed**: sleep first, then fetch the freshest.
- MINOR — bind-to-try gap: failures between bind and the try leaked the
  listening socket; `server_close` was never called on any path.
  **Fixed**: serve-first ordering + `server_close` in the finally.
- MINOR — Windows `SO_REUSEADDR` lets a second instance silently
  double-bind, so the port-in-use refusal is Linux-only (cell1 = fine).
  **Skipped**: documented; harness tests our handling via injected error.
- MINOR — second Ctrl+C during cleanup could skip steps; a dead-but-not-
  failing camera blocking in native `read()` makes the tool unkillable
  without SIGKILL. **Fixed** (per-step guards — verification pass showed
  the first single-wrap guard still skipped later steps); the native-read
  hang is **accepted** (SIGKILL/unplug; same exposure as campreview).

**Code-quality:**
- MINOR — `run_tool`/`BenchError` imported from `.bus`, dragging the
  servo SDK into a tool advertising "never touches the servo bus" (also
  campreview's pre-existing flaw). **Fixed**: both camera tools import
  from `hardware.errors`; campreview owns a camera-flavored `run_tool`;
  the SDK's absence is asserted in the harness.
- MINOR — FPS window + dead-camera error duplicated verbatim from
  campreview. **Fixed**: shared `FpsCounter`/`read_frame` in campreview,
  inline copies deleted.
- MINOR — unreachable `return 0` + missing `NoReturn`. **Fixed.**
- MINOR — broad `OSError` catch around `do_GET`. **Skipped**: disconnect
  detection is OSError-shaped and a genuine socket fault should also end
  the request quietly; narrowed to the single class with a comment.
- MINOR — camserve example nested inside campreview's doc fence with an
  ambiguous fps bullet; no literal curl example. **Fixed.**

**Functional:** all checklist items DELIVERED. Main-thread-capture
wording gap documented as decision 3 above (**no change needed**);
harness weaknesses folded into the adversarial MAJOR above.

## Verification pass (on `1160779`) — all fixed

- LOW — single-wrap second-Ctrl+C guard still skipped later cleanup
  steps, and a non-KeyboardInterrupt error in cleanup replaced exit 130
  with a traceback. **Fixed**: per-step `(KeyboardInterrupt, OSError)`
  guards.
- LOW — `/snapshot` writes had no send timeout (same stall class as the
  round-1 MAJOR). **Fixed**: handler-wide timeout.
- Claim hygiene: "asserted in CI smoke" was untrue → the assert now
  exists in the harness; the second-Ctrl+C comment now describes what
  the code actually guarantees.

Live validation on cell1 with the real ELP camera (stream from the desk
browser, `/snapshot` via curl, tag lock on the printed sheet) is the one
open checklist item.
