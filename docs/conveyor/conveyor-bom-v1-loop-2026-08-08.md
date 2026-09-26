# Mini modular conveyor — BOM

**v1 loop: 8 modules** (4 straights + 4 tapered-roller curves). Plans #835 (v0 rig) · #840 (this loop).
Dimensions come from `cad/conveyor/parts/geometry.json` — regenerate it, never retype it.
Build and assembly instructions: [`cad/conveyor/README.md`](../../cad/conveyor/README.md).

**Update 2026-09-26:**
- The corner is a tapered-roller curve (plan #835), and its CAD has landed. The belt corner's rows are gone. That design stays in git history at `7c560e0`.
- The straights now have tie bars, jack-screw tensioners and printed joiners, and every motor mounts through ServoCity's N20 enclosure.
- The curve's cones print in TPU (Kyle, 2026-09-26). In sim, PLA cones slicker than the belt left parts 5–14° short of square.
- Current sources, prices and stock: [conveyor-order-2026-09-25.md](conveyor-order-2026-09-25.md).

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

**Recommended split:** 4 ServoCity 638122 now (v0 needs 3, US stock; Out of Darts sold out on 2026-09-25). The loop's motors are more of the same ServoCity motor, with its enclosure and input board, bought once v0 works. They ship from US stock, so there is no lead time to get ahead of, and one motor everywhere keeps every module matched.

### Spec to order: 12 V · ~300 RPM · 3 mm D-shaft · single-ended

- OFD's "300 RPM" is at 11.1 V (3S) → ~325 RPM at 12 V. AliExpress quotes at 12 V. Both in range.
- **Buy the fast one, not the torquey one.** Torque needed is 0.05 kg·cm — ≥20× margin at any ratio, so torque is not the selector. *Stall* torque is, because a jammed part puts all of it through the printed D-bore. The bought motor's 1.2 kg·cm stall loads the D-flat to about 14 MPa, **about 3× margin** in PLA+. A 250:1 (3.0 kg·cm) would drop that to about 1×.
- The belt moves at its neutral axis, Ø11 on a Ø10 roller. The bought 270 RPM motor gives **about 155 mm/s**, down to ~31 mm/s at 20 % duty. PWM only throttles downward.
- Shaft length varies **9–10 mm** by vendor. The driven roller reaches the shaft through a spigot in the side plate. Its D-bore is 10 mm deep and the shaft engages 6.3 mm (`build.log`), so either length fits without bottoming out.
- **Mounts through ServoCity's N20 enclosure:** two M4 ears bolt into captive nuts on the side plate or on the curve's pad. The enclosure also covers the open gearbox. The printed body clamp is gone.
- Buy 2 spares. The gearboxes are the weak point.

## 2 · Geometry costed against

- Belt width **50 mm**. The belt's top, where the part rides, is at z = 31 mm, and the cone tops sit level with it.
- Straight module **120 × 61 mm**. Curve: **six tapered rollers**, Ø6 → Ø16, under a lane 30–80 mm from the curve centre.
- Rollers **Ø10 at both ends**, discharge one driven
- Loop footprint about **311 mm square** between the curves' outer walls: curve centres 123 mm apart, walls out to r 94 mm. Each curve's motor stands out at its corner. #840 lays the loop out.
- Belt only on the straights: 4 × 250.6 mm.
- Transfer spans onto and off every curve: **10.5 / 13.0 / 15.5 mm** at the lane's inner edge, centreline and outer edge.

## 3 · Printed parts — 80 pieces, ~860 g solid

Per straight ×4: 2 side plates · 2 rollers · 1 slider bed · 1 return guide · 2 tie bars · 2 tensioner blocks. Per curve ×4: 1 frame · 6 cone rollers · 1 keeper. Plus 8 joiners, one per joint.

| Part | Qty | Material |
|---|---|---|
| Side plates, motor side | 4 | PETG |
| Side plates, plain | 4 | PETG |
| Rollers Ø10, idler (plain Ø4 bore) | 4 | PLA+ |
| Rollers Ø10, driven (D-bore through a spigot) | 4 | PLA+ |
| Slider beds | 4 | PLA+ |
| Return guides | 4 | PLA+ (flat bar, sits 0.5 mm below the taut return run) |
| Tie bars | 8 | PETG |
| Tensioner blocks | 8 | PETG |
| Curve frames | 4 | PETG |
| Curve keepers | 4 | PETG |
| Cone rollers, idler | 20 | TPU 95A |
| Cone rollers, driven | 4 | TPU 95A |
| Joiners | 8 | PETG |

- PETG where it's loaded, PLA+ where the fit matters, and TPU wherever the part rides. Tree supports on.
- About 860 g if printed solid (from the STL volumes): ~535 g PETG, ~170 g PLA+ and ~160 g TPU. Sparse infill brings that down. PLA+ and PETG assumed on hand.
- **No printed motor mount.** ServoCity's N20 enclosure bolts to the side plate's outer face, or to the curve's pad.
- **No grub screw** on the driven roller — Ø3.2 through a 3.5 mm wall leaves nothing.
  - The D-flat is the key: about 14 mm² of flat over the 6.3 mm engagement. That is roughly 70× margin running and 3× at stall in PLA+.
  - The curve's driven cone is TPU, which is more likely to slip than strip at a jam. The coupon checks that it grips when running.
- Bores modelled at **nominal +0.15 mm on radius** (printed holes come out undersize on this machine). Ø3.3 modelled → ~Ø3.1 printed. The TPU cones' plain bores are opened further, for a running fit on the Ø3 rod; the coupon confirms it.
- Optionally face the slider beds with **UHMW or PTFE tape** — PU on UHMW runs µ 0.03–0.06 vs 0.15–0.30 on steel, and printed PLA sits nearer steel. Cuts belt drag 5–10×. Unnecessary at v0's margin; worth it at 8 motors on one supply.

## 4 · Belt — printed TPU loops

50 mm closed belt loops aren't something you can buy. Print each as a thin-walled cylinder standing upright — no seam, no splice, no glue.

| Belt | Mean Ø | Height | Wall | TPU |
|---|---|---|---|---|
| Straight ×4 | 79.8 mm | 50 mm | 1.0 mm | ~15 g ea |

- **Filament: TPU 95A** — roughly skateboard-wheel hardness. The ordered YOUSU 95A has no Bambu-tuned profile, so print it with Bambu Studio's generic TPU profile, slower. 400–500 % elongation at break is what lets it wrap a Ø10 roller and spring back instead of creasing. 85A is more rubbery and much harder to print.
- **Wall is 1.0 mm, set by the roller.** Belt practice wants pulley-Ø ÷ thickness ≥ 10; Ø10 rollers put 1.5 mm at 6.7. It wouldn't crack, but a stiff belt lifts off a small nose roller — the exact geometry the nose exists to protect. 1.0 mm gives **D/t = 10.0** and prints as 2–3 perimeters at 0.4 mm.
- Diameters are **computed, not estimated** — belt path 250.6 mm, measured at the **neutral axis** (roller Ø + wall), the only length that stays constant as the belt wraps. Measuring at the roller surface undersizes every loop by π × wall.
- **TPU must not go through an AMS** — flexible filament buckles in a long PTFE path. Kyle's A1 runs an external spool with a short direct feed, which is what this wants.
- ~60 g for the four belts, plus ~160 g for the curves' cones (a solid upper bound). One 1 kg spool covers both.
- *Fallback:* PU/PVC belting by the metre, spliced. Cheaper, but the splice is a hand skill and 4 loops is 4 chances to get it wrong.

## 5 · Electronics

| Item | Qty | Notes |
|---|---|---|
| ServoCity 638122 N20 gearmotor, 12 V 270 RPM | 8 (+1) | §1. v0 bought 4 |
| TB6612FNG dual driver breakout | 4 | 2 ch each, **4.5–13.5 V** |
| Raspberry Pi **Pico 2 W** (RP2350) | 1 | On hand: Freenove kit from #717.1. See below |
| Bench supply, 0–30 V / 10 A, current limit, output switch | 1 | WANPTEK TPS-C3010, set to 12 V. Its current display gives the *measured* stall current |
| Perfboard / solderable breadboard | 1 | 4 drivers is past jumper-wire territory |
| 2-core motor wire | ~5 m | |
| N20 enclosure + Gear Motor Input Board A | 8 each | ServoCity. The enclosure is the motor mount; the input board plus female/male jumpers lets a module unplug |
| Micro-USB cable | 1 | On hand, in the Freenove kit |

- **Not the DRV8833** — tops out at 10.8 V, cannot drive 12 V motors.
- **Pico 2, not Pico.** 8 motors × (PWM + IN1 + IN2) + STBY = 25 of a Pico's 26 GPIO. The RP2350 has 12 PWM slices (24 ch) against the RP2040's 8 (16).
- **The 12 V motor rail and the Pico's 5 V USB rail are separate supplies that must share a common ground.** The TB6612FNG splits VM (motor) from VCC (logic, 3.3 V). Get this wrong and it either does nothing or misbehaves in ways that look like a firmware bug.

## 6 · Mechanical hardware

From `geometry.json` → `hardware`, which lists the v0 line (2 straights, 1 curve, 2 joiners). Scaled here to the loop: 4 straights, 4 curves, 8 joiners.

| Item | Per straight | Per curve | Per joiner | Loop |
|---|---|---|---|---|
| M3 × 8, tie bars | 4 | | | 16 |
| M3 × 16, tensioner jacks | 2 | | | 8 |
| M3 × 10, curve keeper | | 2 | | 8 |
| M3 × 12, joiners | | | 4 | 32 |
| M4 × 8, motor ears | 2 | 2 | | 16 |
| M3 nut | 8 | 2 | 4 | 72 |
| M4 nut | 2 | 2 | | 16 |
| Ø4 rod, 73.0 mm, idler | 1 | | | 4 |
| Ø4 rod, 51.3 mm, driven stub | 1 | | | 4 |
| Ø3 rod, 70.0 mm, cone idler | | 5 | | 20 |
| Ø3 rod, 24.2 mm, cone driven stub | | 1 | | 4 |
| Nitrile O-ring, the calipered kit size | | 5 | | 20 |

- Rod: the Ø4 cuts total 0.5 m, two of the 300 mm rods. The Ø3 cuts total 1.5 m, six of the twelve.
- Neither the screw kit's nor the O-ring kit's per-size counts are listed. Before the loop, check the screw kit covers 32 M3 × 12 and 72 M3 nuts, and the O-ring kit holds 20 of the chosen size.
- **No bearings.** At Ø10 the bearing OD *is* the roller. Rollers and cones run as plain bearings on their rods, loaded only by belt tension, O-ring tension and the part.

## 7 · Cost

Live prices and the total are in the [order list](conveyor-order-2026-09-25.md): about $225 for v0 as of 2026-09-25. The loop adds about 5 more ServoCity motor sets. The estimate this section used to carry (~$110–145) was costed before Out of Darts sold out and before the bench supply replaced the 12 V brick.

## 8 · Verified vs estimated

**Verified:** TB6612FNG 4.5–13.5 V, 1.2 A cont / 3.2 A peak · DRV8833 caps at 10.8 V · RP2040 8 PWM slices, RP2350 12 · A1 build volume 256³ mm · OFD $6.99, 3 mm D-shaft, 34 × 12 × 10 mm, <1 A stall, 3S · Pololu #3041 $26.45 · belt paths and cylinder diameters from `build.log` · hardware counts and cut lengths from `geometry.json` · part volumes from the STLs.

**Estimated:** N20 stall current (measure it — it sizes the PSU) · µ ≈ 0.35 belt-on-PLA-bed, pessimistic · all filament weights · the TPU cones' bore fit · AliExpress prices and lead times · whether TPU feeds cleanly on the A1's external spool.

**Measure before bulk-buying:** print the three coupons (README, plate 1), caliper them and check the fits, then commit to rod and fasteners.
