# Wiring Hardware — Custom Servo Cables for the SO-101 Arm

Parts, tools, and workflow for replacing the stock fixed-length STS3215 bus cables with
exact-length custom runs. Researched 2026-07-23; prices volatile — verify at checkout.

## The connector system (know this before buying anything)

- The Feetech STS3215 bus uses **Molex Mini-SPOX 5264** connectors: 3-position, **true 2.50mm pitch**.
- ⚠️ **Not 2.54mm.** The endless Amazon "2.54mm Dupont/JST/KK assortment kits" are the wrong part —
  close enough to look right, wrong enough not to latch on the servo headers. Anything that doesn't
  say **5264** or **Mini-SPOX**, skip.
- Both ends of every cable are the **same part**: a 5264 housing loaded with female crimp terminals.
  The male pins live on the servo/controller PCB headers. (Listings label the housings "male"
  more or less at random — go by part number, not gender words.)
- Pinout: **1 = GND, 2 = Vcc, 3 = Signal/TTL.** Daisy-chain bus — each servo has two identical ports.

## Parts to order

| Item | Part | Qty | Where |
|---|---|---|---|
| Housing | **Molex 50-37-5033** (Mini-SPOX 5264, 3-pos, friction lock) | ~25 | [Newark](https://www.newark.com/molex/50-37-5033/connector-rcpt-3pos-1row-2-5mm/dp/57H1785), DigiKey, Mouser |
| Crimp terminal | **Molex 0008701039** (Mini-SPOX female, 22–28 AWG, tin) | 100 | [DigiKey](https://www.digikey.com/en/products/detail/molex/0008701039/765268) |
| Wire | **22 AWG 3-conductor flat servo ribbon**, 25–50 ft spool, black/red/white | 1 spool | Amazon: "22 AWG 3 wire flat servo cable spool" |

**Why 22 AWG (not the stock ~26):** the follower arm runs 12V/30kg·cm STS3215s and the bus is a
daisy chain — the base cable carries current for every servo downstream. 22 AWG is the thickest
the 5263 terminals accept; take the free headroom. Black/red/white (Futaba-style) matches GND/Vcc/Signal.

**Terminal quantity:** ~12–20 crimps actually needed for the arm; order 100 anyway. Everyone
scraps terminals while dialing in the fold, and they're pennies.

## Tools

| Tool | Pick | Notes |
|---|---|---|
| Flush cutters | Hakko CHP-170 (~$6) | square cuts |
| Wire stripper | Klein 11057 or Engineer PAW-01 (20–30 AWG range) | strip depth matters more than brand |
| Terminal extractor | any "2.54mm / mini terminal extractor" blade (~$5) | the do-over tool — depresses the locking tang so a terminal backs out of the housing undamaged |
| Crimper (optional) | Engineer PA-09 (~$45) or IWISS IWS-2820M (~$25, ratcheting 28–20 AWG) | open-barrel terminals CAN be folded with fine needle-nose pliers (two folds per terminal, see workflow); the crimper buys consistency, not possibility. Avoid generic SN-28B "Dupont crimpers" — they mangle the insulation wings |

## Workflow (per cable end)

1. **Cut** square with flush cutters, to measured length for the specific joint-to-joint run.
2. **Strip ~2.5–3mm.** Gauge against the terminal itself: bare copper fills the front (conductor)
   wings, insulation edge lands under the rear (insulation) wings, no stray strands past the nose.
3. **Crimp — two folds per terminal:** conductor wings onto bare copper first, then insulation
   wings onto the jacket. Needle-nose: fold each pair over in turn. Tug test every crimp.
4. **Insert** into the housing until the terminal's tang clicks behind the housing lance.
   Verify wire colors land on the right positions (GND/Vcc/Signal) BEFORE powering anything.
5. **Do-over:** extractor blade in from the mating face, depress the tang, slide the terminal out.

## Ready-made fallbacks (no crimping)

- [Waveshare 5264 cable 6-pack](https://www.amazon.com/waveshare-5264-3PIN-Servo-Compatible-servos/dp/B0GVDFXF7Q) — 3× 300mm + 3× 900mm, keep as spares.
- [Yoeruyo MX2.54-5264 (3P option)](https://www.amazon.com/Yoeruyo-Connector-Premium-Pre-Crimped-MX2-54-5264/dp/B0CL9KWHLG) — housings + pre-crimped 22 AWG single leads; insert-only workflow, but fixed lead lengths and loose wires instead of ribbon.
- Feetech/generic pre-made "5264 servo cable" in 10/15/20/30cm lengths — search that phrase exactly.

## References

- [Molex 5264 series chart](https://www.molex.com/en-us/products/series-chart/5264) (housings) ·
  [Molex 5263 series chart](https://www.molex.com/en-us/products/series-chart/5263) (terminals)
- Servo spec confirming 5264-3P connector: [WowRobo STS3215 12V/30kg](https://shop.wowrobo.com/products/feetech-sts3215-servo-12v-30kg-high-torque-servo-for-so-arm100),
  [RobotShop STS3215](https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215)
- Companion doc: `hardware-shopping-list.md` (original cell buy list)
