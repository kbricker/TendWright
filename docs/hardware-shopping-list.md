# TendWright Hardware Shopping List — Purchase Reconciliation

Reconciled against Kyle's purchases 2026-07-15. Status legend:
✅ bought/decided · ⚠️ bought but verify a detail · ❓ not confirmed bought — gap · ⏳ deliberately deferred

---

## Bought / decided

### ✅ Robot arm — Seeed SO-ARM101 **Pro** Servo Motor Kit (motors only, both arms)
Upgrade over the plan's baseline: follower gets 6× **12V** STS3215 @ 1:345 (~2× torque of 7.4V). Kit includes leader servos (surplus for us — optional manual-jog build later) and **both PSUs** (12V/2A follower + 5V/3A leader). Arm structure gets printed on the A1 from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) STLs (PLA+, ~800g follower).
- [Seeed SO-ARM101 Pro kit](https://www.seeedstudio.com/SO-ARM101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html)

### ✅ 3D printer — Bambu Lab A1 (no AMS) @ $299
- [Bambu A1](https://us.store.bambulab.com/products/a1)

### ⚠️ Camera — ELP OV2710 board cam
**Verify the variant is `ELP-USBFHD01M-L36`**: manual/fixed focus + **3.6mm lens**. Wrong buys in the same family: `-L21` (2.1mm fisheye), `-AF` (autofocus), `-DL36` (IR dome). If the Amazon listing said "autofocus" or "wide angle/170°", exchange it.

### ✅ Force pads — generic thin-film FSR 4-pack (20g–2kg, 8mm sensing disc, 19mm long)
Threshold-grade grip feedback — exactly our use. ⚠️ Small check: does the pack include clincher/crimp connectors? The film tail must not be soldered. If none: add "FSR clincher connector" pack (~$6) or use the breakout's screw terminals with a jumper-wire lash-up.

### ✅ I/O bridge — Freenove Basic Starter Kit for **Pico 2 W** (board incl., pre-soldered headers)
Covers: the Pico, breadboard, jumper wires, resistor assortment (10kΩ for the FSR divider), LEDs/buttons for bring-up, micro-USB cable, 386-page tutorial.

### ✅ Permanent mount — Pico mini breakout board w/ screw terminals (1pc)
Same footprint fits Pico 2 W. ⚠️ Verify it has female sockets (plug-in), not solder-down pads — photo shows which.

### ✅ Cell controller — Kyle's Intel NUC (Core i-series confirmed; $0)
Pending: exact model + RAM (8GB floor) once retrieved from garage. Setup at P2: Ubuntu LTS, `dialout` group, udev rules pinning `/dev/tty-grbl` / `/dev/tty-arm` / `/dev/tty-pico`.

---

## Gaps — CLOSED 2026-07-15 (all confirmed bought / on hand)

### ✅ Filament — bought
Plan was 3× PLA+ (arm ~800g + jigs/mounts/failures + optional leader build) + 2× PETG (nest iterations, jaw inserts, mold).

### ✅ Ring light — bought
Reminder at install: rigid mount, run at 100% brightness (PWM flicker at dim settings), pick one color mode forever, lock camera exposure/WB to match.

### ✅ Switches — KW12-3 roller microswitch pack — bought
Part-in-nest, outfeed, guard-closed (P6 wiring).

### ✅ Hookup wire — on hand (+ more jumper wire in the Freenove kit)
For the FSR run down the arm and switch runs to the breakout.

### ✅ Paper printing — 2D printer on hand
AprilTags + ChArUco calibration board print on paper at **100% scale** (verify with a ruler/calipers), glue to something flat. Generator: [calib.io pattern generator](https://calib.io/pages/camera-calibration-pattern-generator).

---

## Deliberately deferred (correct call — nothing blocked)

### ⏳ Mini-CNC — SainSmart Genmitsu 3018-PROVer V2 ($239)
Order when P5 winds down / P6 approaches. No scarcity risk. Everything before the capstone runs against the mock OPC UA CNC + GRBL simulator.
- [SainSmart direct](https://www.sainsmart.com/products/genmitsu-3018-prover-v2-upgraded-semi-assembled-cnc-router-kit) · [Amazon full kit](https://www.amazon.com/Genmitsu-3018-PROVer-Beginner-Emergency-Stop-Spoilboard/dp/B0CMTJ6CZC) · [Amazon w/o offline controller](https://www.amazon.com/SainSmart-Genmitsu-3018-PROVer-Switches-Emergency-Stop/dp/B07ZFD6SKP)

### ⏳ Machinable wax blanks (~$25)
Only matters when the CNC exists. Order with it. Until then, any ~40×40×20mm object stands in for bench-top pick/place testing.
- [machinablewax.com](https://machinablewax.com/)

---

## No-CNC runway (what all of the above unlocks, in order)
1. Printer arrives → tolerance test print → camera + ring mount
2. NUC → Ubuntu, udev, dialout
3. Camera → P2 calibration, AprilTags, hand-eye
4. Arm kit → print follower structure → assemble → calibrate → drive from Python
5. Bench milestone: **camera finds blank → arm picks → places into prototype nest** (P6's arm-side risk retired, CNC still not needed)
6. Then buy the 3018 + wax; P6 becomes integration, not gamble
