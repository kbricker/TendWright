# Mini modular conveyor — BOM

**v1 loop: 8 modules** (4 straight + 4 corner). Plans #835 (v0 rig) · #840 (this loop).
Dimensions come from `cad/conveyor/parts/geometry.json` — regenerate it, never retype it.
Build and assembly instructions: [`cad/conveyor/README.md`](../../cad/conveyor/README.md).

## 1 · Motors — where they actually come from

- **"N20" is a can size, not a brand** — a 12 mm brushed can, Mabuchi frame class.
- The geared version is **GA12-N20** (also GM12-N20, CHF-GM12-N20): a 12 mm metal spur gearbox on an N20 can, 3 mm D-shaft. Dozens of Shenzhen factories build it to the same envelope.
- Factories with real catalogues: **Shenzhen Chihai Motor** (CHF-GM12-N20) and **TT Motor (Shenzhen) Industrial**.
- **Out of Darts is a Nerf reseller, not a source** — their page says they "sourced these motors for our Jupiter and Juno blasters." A rebadged factory run. Pololu does the same, but specs their own version and publishes the only real torque curves.
- **Direct: search AliExpress for `GA12-N20 12V 300RPM`** — $1.66–3.50 each.

| Source | ea | ×10 | Lead time | Notes |
|---|---|---|---|---|
| AliExpress GA12-N20 | $1.70–3.50 | ~$25 | 2–4 wk | No datasheet, loose RPM binning |
| Out of Darts | $6.99 | $70 | Days, US | 300/600/1000/2000 RPM, QC'd for full-auto Nerf |
| Pololu #3041 (100:1) | $26.45 | $265 | Days, US | Published curves; 4× the price buys nothing here |

**Recommended split:** 3 from Out of Darts now (v0 needs exactly 3, US stock) + 10 GA12-N20 from AliExpress for the loop. The slow order lands about when v0 is proven.

### Spec to order: 12 V · ~300 RPM · 3 mm D-shaft · single-ended

- OFD's "300 RPM" is at 11.1 V (3S) → ~325 RPM at 12 V. AliExpress quotes at 12 V. Both in range.
- **Buy the fast one, not the torquey one.** Torque needed is 0.05 kg·cm — ≥20× margin at any ratio, so torque is not the selector. *Stall* torque is, because a jammed part puts all of it through the printed D-bore: 1.3 kg·cm → 11.4 MPa, **3.5× margin**. A 250:1 (3.0 kg·cm) drops to 1.5× — too tight.
- Belt speed at Ø10 rollers: 300 RPM → **157 mm/s**, down to ~30 mm/s at 20 % duty. PWM only throttles downward.
- Shaft length varies **9–10 mm** by vendor. The bore is 8 mm deep with 5 mm engagement, so either fits without bottoming out.
- **Mounts by clamping the 10 × 12 mm body** — no face pattern, no bolt circle. Any GA12-N20-class motor fits regardless of vendor.
- Buy 2 spares. The gearboxes are the weak point.

## 2 · Geometry costed against

- Belt width **50 mm**, carry surface at z = 30 mm
- Straight module **120 mm**, corner module **70 mm**, outer width **61 mm**
- Rollers **Ø10 at both ends**, discharge one driven
- Loop footprint **~260 × 260 mm**; ~1.6 m of belt total
- Transfer spans: **12.0 mm** into a corner's side, **13.5 mm** into a straight's end

## 3 · Printed parts — 76 pieces, ~460 g

Per module ×8: 2 side brackets · 2 rollers · 1 slider bed · 2 tensioner blocks · 1 return guide. Plus 4 corner guide rails and 8 frame connectors.

| Part | Qty | Material |
|---|---|---|
| Side brackets, motor side | 8 | PETG |
| Side brackets, plain | 8 | PETG |
| Rollers Ø10, idler (plain Ø4 bore) | 8 | PLA+ |
| Rollers Ø10, driven (Ø3 D-bore) | 8 | PLA+ |
| Slider beds | 8 | PLA+ |
| Tensioner blocks | 16 | PETG |
| Return guides | 8 | PLA+ (crowned bar, sits 0.5 mm below the taut return run) |
| Corner guide rails | 4 | PLA+ |
| Frame connectors | 8 | PETG |

- PETG where it's loaded, PLA+ where the fit matters. Tree supports on.
- ~460 g — half a spool. PLA+ and PETG assumed on hand.
- **No separate motor mount.** Driving a nose roller puts the motor on the side plate's outer face.
- **No grub screw** on the driven roller — Ø3.2 through a 3.5 mm wall leaves nothing. The D-flat is the key: 17.9 mm² of bearing area, 143× margin running, 3.5× at stall.
- Bores modelled at **nominal +0.15 mm on radius** (printed holes come out undersize on this machine). Ø3.3 modelled → ~Ø3.1 printed.
- Optionally face the slider beds with **UHMW or PTFE tape** — PU on UHMW runs µ 0.03–0.06 vs 0.15–0.30 on steel, and printed PLA sits nearer steel. Cuts belt drag 5–10×. Unnecessary at v0's margin; worth it at 8 motors on one supply.

## 4 · Belt — printed TPU loops

50 mm closed belt loops aren't something you can buy. Print each as a thin-walled cylinder standing upright — no seam, no splice, no glue.

| Belt | Mean Ø | Height | Wall | TPU |
|---|---|---|---|---|
| Straight ×4 | 79.8 mm | 50 mm | 1.0 mm | ~15 g ea |
| Corner ×4 | 47.9 mm | 50 mm | 1.0 mm | ~9 g ea |

- **Filament: TPU 95A** — roughly skateboard-wheel hardness. Bambu *TPU 95A HF* has an A1 profile. 400–500 % elongation at break is what lets it wrap a Ø10 roller and spring back instead of creasing. 85A is more rubbery and much harder to print.
- **Wall is 1.0 mm, set by the roller.** Belt practice wants pulley-Ø ÷ thickness ≥ 10; Ø10 rollers put 1.5 mm at 6.7. It wouldn't crack, but a stiff belt lifts off a small nose roller — the exact geometry the nose exists to protect. 1.0 mm gives **D/t = 10.0** and prints as 2–3 perimeters at 0.4 mm.
- Diameters are **computed, not estimated** — belt path 250.6 / 150.6 mm, measured at the **neutral axis** (roller Ø + wall), the only length that stays constant as the belt wraps. Measuring at the roller surface undersizes every loop by π × wall.
- **TPU must not go through an AMS** — flexible filament buckles in a long PTFE path. Kyle's A1 runs an external spool with a short direct feed, which is what this wants.
- ~100 g total. One spool covers it several times.
- *Fallback:* PU/PVC belting by the metre, spliced. Cheaper, but the splice is a hand skill and 8 loops is 8 chances to get it wrong.

## 5 · Electronics

| Item | Qty | Notes |
|---|---|---|
| GA12-N20 gearmotor, 12 V ~300 RPM | 8 (+2) | §1 |
| TB6612FNG dual driver breakout | 4 | 2 ch each, **4.5–13.5 V** |
| Raspberry Pi **Pico 2** (RP2350) | 1 | See below |
| 12 V PSU, 5 A, barrel jack | 1 | Size from *measured* stall current |
| Barrel jack breakout | 1 | |
| Perfboard / solderable breadboard | 1 | 4 drivers is past jumper-wire territory |
| 2-core motor wire | ~5 m | |
| JST-XH pairs or screw terminals | 8 | So a module unplugs |
| USB-C cable | 1 | Data, not charge-only |

- **Not the DRV8833** — tops out at 10.8 V, cannot drive 12 V motors.
- **Pico 2, not Pico.** 8 motors × (PWM + IN1 + IN2) + STBY = 25 of a Pico's 26 GPIO. The RP2350 has 12 PWM slices (24 ch) against the RP2040's 8 (16).
- **The 12 V motor rail and the Pico's 5 V USB rail are separate supplies that must share a common ground.** The TB6612FNG splits VM (motor) from VCC (logic, 3.3 V). Get this wrong and it either does nothing or misbehaves in ways that look like a firmware bug.

## 6 · Mechanical hardware

- 4 mm steel rod, 1 m — idler and stub axles, ~70 mm each
- M3 × 16 bolts + nyloc nuts, 60 — frame
- M4 × 20 bolts + nuts, 20 — tensioners
- M2 × 8 bolts, 10 — motor body clamps
- **No bearings.** At Ø10 the bearing OD *is* the roller. Rollers run as plain bearings on the axle; load is belt tension only.

## 7 · Cost

| | |
|---|---|
| 10 motors (AliExpress) | $25 |
| 3 motors (Out of Darts, v0 now) | $21 |
| 4 × TB6612FNG | $8–16 |
| Pico 2 | $5–10 |
| 12 V 5 A PSU + jack | $12–18 |
| Rod + fasteners | $12–18 |
| TPU spool | $20–30 |
| Wire, connectors, perfboard | $10–15 |
| **Total** | **~$115–155** |

## 8 · Verified vs estimated

**Verified:** TB6612FNG 4.5–13.5 V, 1.2 A cont / 3.2 A peak · DRV8833 caps at 10.8 V · RP2040 8 PWM slices, RP2350 12 · A1 build volume 256³ mm · OFD $6.99, 3 mm D-shaft, 34 × 12 × 10 mm, <1 A stall, 3S · Pololu #3041 $26.45 · belt paths and cylinder diameters from `build.log`.

**Estimated:** N20 stall current (measure it — it sizes the PSU) · µ ≈ 0.35 belt-on-PLA-bed, pessimistic · all filament weights · AliExpress prices and lead times · whether TPU feeds cleanly on the A1's external spool.

**Measure before bulk-buying:** print one roller and one bracket, caliper the Ø4.4 axle bore and Ø3.3 D-bore, then commit to rod and fasteners.
