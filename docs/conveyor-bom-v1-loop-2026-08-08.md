# Mini modular conveyor — BOM for the rounded rectangle (v1 loop)

**Plans:** #835 (v0 rig) · #840 (this loop) · drawn up 2026-08-08

**Locked:** 50 mm belt · 12 V N20 motors · Pico + TB6612FNG · 8 modules (4 corners + 4 straights)

**Architecture this BOM assumes:** rollers + belt, derived from `tanius/smallopticalsorter`
(Unlicense). If we go printed link chain instead, see §7 for the delta.

---

## 1 · The motor

**12 V N20 metal gearmotor, ~100 RPM, 3 mm D-shaft.** Sometimes sold as "N20 mini
metal gear motor 12V 100RPM". Anything **60–150 RPM** works; buy the speed you want
as the *maximum*, because PWM only throttles downward.

**Torque is not the constraint — it has 8–12× margin.** The working:

| | |
|---|---|
| Belt + a 50 g part, normal force | ~0.6 N |
| × µ ≈ 0.35, printed belt on printed slider bed | ~0.21 N |
| × ~3 for roller, bearing and tracking drag | **~1 N of belt pull** |
| At a Ø25 mm roller → 1 N × 0.0125 m | **0.0125 N·m ≈ 0.13 kg·cm** |
| A 12 V N20 @ 100 RPM delivers | ~1.0–1.5 kg·cm |

**Speed is what actually decides it.** A Ø25 roller advances 78.5 mm per turn:

| Motor | Top belt speed | At 20% duty |
|---|---|---|
| 60 RPM | 79 mm/s | 16 mm/s |
| **100 RPM** ← | **131 mm/s** | **26 mm/s** |
| 150 RPM | 196 mm/s | 39 mm/s |

100 RPM gives the most usable range: brisk at full, still controllable at a crawl.

**Buy one or two spares.** N20s are cheap and the gearboxes are the weak point.

### Why Ø25 rollers, not tanius's Ø40

Smaller end rollers mean a smaller nose radius, which means **a smaller gap between
adjacent modules** — and that gap is the one thing v0 exists to prove. Smaller rollers
make the hard problem easier. Ø40 on a 3 mm shaft would also be a poor lever ratio.

---

## 2 · Geometry this BOM is costed against

| | |
|---|---|
| Belt width | 50 mm |
| Module outer width | ~62 mm |
| Straight module length | 120 mm |
| Corner module length | 70 mm |
| Roller diameter | 25 mm |
| Height off the desk | ~45 mm |
| **Loop footprint** | **~260 × 260 mm** (70 + 120 + 70 per side) |
| Total belt to produce | ~1.6 m at 50 mm wide |

---

## 3 · Printed parts — 84 pieces, ~500 g

Per module (×8): 2 side brackets · 2 rollers · 1 slider bed · 1 motor mount ·
2 tensioner blocks · 1 return guide = **9 parts**

Plus 4 corner guide rails and 8 frame connectors.

| | Qty | Material | Notes |
|---|---|---|---|
| Side brackets | 16 | PETG | Carries bearing pockets and the tensioner slot |
| Rollers Ø25 | 16 | PLA+ | Needs edge flanges to keep the belt tracking |
| Slider beds | 8 | PLA+ | The wear surface — print smooth side up |
| Motor mounts | 8 | PETG | Takes the D-shaft coupling |
| Tensioner blocks | 16 | PETG | 20 mm sliding travel |
| Return guides | 8 | PLA+ | |
| Corner guide rails | 4 | PLA+ | Arrests the part's incoming momentum |
| Frame connectors | 8 | PETG | Sets and holds the inter-module gap |

**~500 g total**, so half a spool. PETG where it's loaded, PLA+ where the fit matters —
same split as CableCell uses. Tree supports on, per your standing profile.

---

## 4 · The belt — the one genuinely awkward item

50 mm-wide closed belt loops are not something you can just buy. Two real options:

### Option A — printed TPU loops (recommended)

Print each belt as a **thin-walled cylinder standing on the bed**, sized so its
circumference equals the belt path. No seam, no splicing, no glue.

| Belt | Cylinder Ø | Height | Wall | TPU |
|---|---|---|---|---|
| Straight (×4) | 79.1 mm | 50 mm | 1.5 mm | ~23 g each |
| Corner (×4) | 47.3 mm | 50 mm | 1.5 mm | ~14 g each |

**~150 g of TPU total** — one spool covers it with room for reprints.

These diameters are **computed by the CAD, not estimated**: `build.log` reports
belt path 248.5 mm (straight) and 148.5 mm (corner) from the actual roller centres.
Rerun `build_parts.py` after any dimension change and re-read them.

⚠ **TPU should not go through the AMS** — it wants an external spool holder and a
direct feed. Worth confirming that's workable on your A1 before committing, since
it's the only new material in the whole build.

### Option B — bought belting, spliced

PU or PVC conveyor belting by the metre, cut to length and joined. Cheaper per metre
but the splice is a hand skill, it's the weakest point of every loop, and 8 of them
is 8 chances to get it wrong. Only worth it if TPU turns out to be a problem.

---

## 5 · Electronics

| Item | Qty | Notes |
|---|---|---|
| N20 gearmotor, 12 V, 100 RPM, 3 mm D-shaft | 8 (+2 spare) | §1 |
| TB6612FNG dual motor driver breakout | 4 | 2 channels each. **4.5–13.5 V** — covers 12 V |
| Raspberry Pi **Pico 2** (RP2350) | 1 | 24 PWM channels vs the RP2040's 16 — see below |
| 12 V PSU, 5 A, barrel jack | 1 | Size from *measured* stall current, not the datasheet |
| Barrel jack breakout | 1 | |
| Perfboard or solderable breadboard | 1 | 4 drivers + rails is past jumper-wire territory |
| 2-core wire for motors | ~5 m | |
| JST-XH pairs or screw terminals | 8 | So a module unplugs |
| USB-C cable for the Pico | 1 | Data, not charge-only |

**Not the DRV8833** — it tops out at 10.8 V and cannot drive 12 V motors at all.

### Pico vs Pico 2

8 motors × (PWM + IN1 + IN2) + STBY = **25 of a Pico's 26 GPIO**, leaving nothing for a
sensor. The **Pico 2** has 12 PWM slices (24 channels) against the RP2040's 8 slices
(16), so it carries the load with headroom. It's a couple of dollars more. Take it.

### Power topology — the classic first-integration failure

The 12 V motor rail and the Pico's 5 V USB rail are **separate supplies that must share
a common ground.** The TB6612FNG has a split VM (motor, 12 V) / VCC (logic, 3.3 V).
Get this wrong and it either does nothing or behaves erratically in ways that look
like a firmware bug.

---

## 6 · Mechanical hardware

| Item | Qty | Notes |
|---|---|---|
| MR105ZZ bearings (5 × 10 × 4 mm) | 24 | 2 per idler roller + 1 per drive roller far end |
| 5 mm steel rod, 1 m | 1 | Idler axles, ~70 mm each |
| M3 × 16 bolts + nyloc nuts | 60 | Frame assembly |
| M4 × 20 bolts + nuts | 20 | Tensioners |
| M3 × 6 grub screws | 10 | D-shaft roller retention |

Bearing and axle sizes are the **most likely thing here to change** once the CAD is
parameterised — they're set by roller wall thickness, which isn't fixed yet. Don't
bulk-buy these until the first roller is printed and measured.

---

## 7 · If we go printed link chain instead

The chain version drops §4 and §6's bearings, and adds:

- ~2.15 m of printed link chain at 50 mm wide — roughly **610 links** at 3.5 mm pitch
- 3 mm steel rod for hinge pins, ~2.2 m
- Printed sprockets instead of plain rollers
- A **link-pitch tolerance coupon printed and measured first**, before anything else

It's more printing, more tolerance risk, no TPU needed, and considerably more of the
ME exercise this project exists for. It also can't be forked from tanius — that design
is rollers-and-belt throughout.

---

## 8 · Rough cost

| | |
|---|---|
| 8 + 2 N20 motors | $30 – 55 |
| 4 × TB6612FNG | $8 – 16 |
| Pico 2 | $5 – 10 |
| 12 V 5 A PSU + jack | $12 – 18 |
| 24 × MR105ZZ | $8 – 12 |
| Rod + fasteners | $12 – 18 |
| TPU spool (if not on hand) | $20 – 30 |
| Wire, connectors, perfboard | $10 – 15 |
| **Total** | **~$105 – 175** |

PLA+ and PETG assumed on hand.

---

## 9 · What's verified vs what's estimated

**Verified against sources or the machine:**
TB6612FNG 4.5–13.5 V, 1.2 A continuous / 3.2 A peak · DRV8833 caps at 10.8 V ·
RP2040 8 PWM slices / 16 channels, RP2350 12 slices / 24 channels · A1 build volume
256³ mm · `tanius/smallopticalsorter` is Unlicense with `belt_width=50`,
`bracket_length=150`, `roller_diameter=40` defaults · OpenSCAD is not installed on
this machine, FreeCAD 1.1 is.

**Estimated — treat as a starting point, not a spec:**
N20 stall current (measure it — it sizes the PSU) · µ ≈ 0.35 for the friction pair ·
all filament weights · all prices · the 610-link count in §7 · whether TPU feeds
acceptably on your A1 without the AMS.

**The one thing to measure before spending money:** print one roller and one bracket,
check the bearing fit, then buy the bearings. Everything else on this list is
forgiving; a 24-pack of the wrong bearing is not.
