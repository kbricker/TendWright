# Bench Day: A1 Setup + SO-101 Follower Build

Companion to `hardware-shopping-list.md`. Software-side work (the bench toolkit under `hardware/bench/`, MuJoCo) is spark's; this doc is the hands-on sequence.

## 1. Bambu A1 setup (~30 min)
1. Unbox, remove ALL orange shipping restraints (there's one under the bed too), attach spool holder, follow the on-screen guided setup — full auto-calibration (bed level, vibration, flow) runs itself.
2. Install **Bambu Studio** on the PC; bind the printer (LAN mode is fine; cloud optional).
3. Sanity print: the pre-loaded demo model with the starter spool. If it looks clean, the machine is healthy.

## 2. Dial in the filament (one evening, worth it)
1. Load **eSUN PLA+** → in Bambu Studio pick the generic "PLA+" profile (or eSUN PLA+ if listed).
2. Print the **MakerWorld tolerance test** (search "Bambulab Tolerance Test", model 640436). Goal: know which clearance (0.1–0.3mm) prints true — this number matters for every fixture later.
3. PETG can wait — it's for fixtures at P6. When its day comes: dry-ish spool, slower first layer, expect one session of stringing tuning.

## 3. Print the follower arm (PLA+, ~800g, ~a weekend of machine time)
**Source options (same parts either way):**
- Easiest: MakerWorld **"LeRobot SO-101 Arms"** (model 1399268) — pre-sliced for Bambu. Print ONLY the follower parts.
- Authoritative: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) repo → SO101 STLs, 220×220 plates (fit the A1 bed as-is).

**Settings per official docs:** 0.4 nozzle, 0.2mm layers, 15–20% infill, supports only where the docs flag them, parts in the given orientation.

**Also print:** the repo's **assembly/calibration jigs** (motor-horn alignment jig especially) and — recommended — the parallel-gripper mod (roboninecom/SO-ARM100-101-Parallel-Gripper) can wait until the stock gripper's limits are felt.

Start the biggest plate overnight; the rest run while assembling.

## 4. Assemble the follower — the two gotchas that matter
1. **Set servo IDs BEFORE assembly.** Each of the 6 STS3215s needs a unique bus ID (1–6, base→gripper) programmed over the servo adapter board *while the servo is loose on the bench*. Assembling first means tearing joints apart later. Use the bench toolkit: `uv run python -m hardware.bench.set_id` (one servo on the bus at a time; it guards against multi-servo buses).
2. **Center servos before attaching horns.** Command each servo to its center position, THEN screw the horn/bracket at the documented neutral angle (use the printed jig). A horn attached one spline-tooth off = permanent calibration offset.

Other assembly notes:
- Screws bite into plastic: snug, not gorilla — PLA+ bosses strip if overtightened. If one strips: drop of CA glue in the hole, re-drive.
- Route the servo bus daisy-chain per docs; leave slack at joints; the FSR's thin wires (later) will follow the same path.
- 12V PSU = follower. The 5V one is for the leader — set it aside with the leader servos (not building that now).

## 5. First power-on (with spark)
- Follower → USB adapter → PC. Bench toolkit flow: `scan` (expect IDs 1–6), then `monitor --ids 1-6` to sweep each joint by hand and record tick ranges + zero positions.
- First motion test: small joint jogs from Python, one joint at a time, hand hovering near the power switch.
- Nothing gets scripted beyond jogs until the MuJoCo model and real arm agree on zero positions.

## Done-when
- [ ] A1 assembled, demo print clean
- [ ] Tolerance test printed; working clearance number written down: ______ mm
- [ ] All follower plates printed (PLA+)
- [ ] Jigs printed
- [ ] Servo IDs 1–6 set before assembly
- [ ] Horns attached at centered positions via jig
- [ ] Follower assembled, wired, powers up
- [ ] First jog from Python moves the right joint the right way
