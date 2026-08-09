# Mini modular conveyor — BOM for the rounded rectangle (v1 loop)

**Plans:** #835 (v0 rig) · #840 (this loop) · drawn up 2026-08-08

**Locked:** 50 mm belt · 12 V N20 motors · Pico + TB6612FNG · 8 modules (4 corners + 4 straights)

**Architecture this BOM assumes:** rollers + belt, derived from `tanius/smallopticalsorter`
(Unlicense). If we go printed link chain instead, see §7 for the delta.

---

## 1 · The motor

> **CHANGED 2026-08-09 — buy ~250 RPM, not 100 RPM.** The drive roller moved from
> Ø25 to Ø10 (see below), and belt speed is π × roller × RPM. A 100 RPM motor on a
> Ø10 roller gives **31 mm/s**, not 131. Nothing had been ordered when this changed.

**12 V N20 metal gearmotor, ~250 RPM, 3 mm D-shaft.** Anything **200–400 RPM**
works; buy the speed you want as the *maximum*, because PWM only throttles downward.

**Torque is not the constraint, and the smaller roller improved it.** Shaft torque is
belt pull × roller radius, so halving the radius halves the load on the D-bore:

| | |
|---|---|
| Belt + a 50 g part, normal force | ~0.6 N |
| × µ ≈ 0.35, printed belt on printed slider bed | ~0.21 N |
| × ~3 for roller, bearing and tracking drag | **~1 N of belt pull** |
| At a Ø10 mm roller → 1 N × 0.005 m | **0.005 N·m ≈ 0.05 kg·cm** |
| A 12 V N20 @ 250 RPM delivers | ~0.5–0.8 kg·cm |
| Margin | **10–16×** |

**Speed is what actually decides it.** A Ø10 roller advances 31.4 mm per turn:

| Motor | Top belt speed | At 20% duty |
|---|---|---|
| 150 RPM | 79 mm/s | 16 mm/s |
| **250 RPM** ← | **131 mm/s** | **26 mm/s** |
| 400 RPM | 209 mm/s | 42 mm/s |

250 RPM gives the most usable range: brisk at full, still controllable at a crawl.

**Buy one or two spares.** N20s are cheap and the gearboxes are the weak point.

### Why Ø10 rollers at BOTH ends, and why the drive is on the discharge end

The module gap is what v0 exists to prove, and a roller in a rounded bracket end sits
half a bracket-height (17.5 mm) inboard of the module face — so the belt apex is pushed
away from the transfer by the frame's own shape. The sim measured the result: **27 mm of
unsupported span against a 32 mm part, and the part stranded.**

Squaring the ends and dropping to Ø10 noses brings both insets to 6 mm. Both ends have
to be small, because in the loop **every straight receives from a corner at one end and
discharges into a corner at the other** — so a fat roller anywhere on an end breaks half
the joints in the system. That is what moved the drive onto a nose roller.

Measured spans after the change: **joint A (into a corner's side) 12.0 mm, joint B (into
a straight's end) 13.5 mm** — both proven to transfer in sim.

**No grub screw on the driven roller.** A Ø3.2 hole through a Ø10 roller's 3.5 mm wall
leaves nothing. The D-flat is the key, which is what a D-shaft is for, and 0.005 N·m is
well within what a printed D-bore holds. This is a tolerance question the bench settles,
not the sim.

---

## 2 · Geometry this BOM is costed against

| | |
|---|---|
| Belt width | 50 mm |
| Module outer width | ~62 mm |
| Straight module length | 120 mm |
| Corner module length | 70 mm |
| Roller diameter | 10 mm, both ends, discharge one driven |
| Height off the desk | ~45 mm |
| **Loop footprint** | **~260 × 260 mm** (70 + 120 + 70 per side) |
| Total belt to produce | ~1.6 m at 50 mm wide |

---

## 3 · Printed parts — 76 pieces, ~460 g

Per module (×8): 2 side brackets · 2 rollers · 1 slider bed · 2 tensioner blocks ·
1 return guide = **8 parts**

Plus 4 corner guide rails and 8 frame connectors.

The separate motor mount is **gone** — driving a nose roller puts the N20 on the side
plate's outer face, so its bolt pattern is cut into the bracket. One fewer part per
module, and one fewer stack-up between the shaft and the roller.

| | Qty | Material | Notes |
|---|---|---|---|
| Side brackets, motor side | 8 | PETG | Take-up slot + the N20 face pattern |
| Side brackets, plain | 8 | PETG | Take-up slot + stub-axle bore. Corner infeed plates are cut flush with the carry plane |
| Rollers Ø10, idler | 8 | PLA+ | Plain Ø4 bore, edge flanges to keep the belt tracking |
| Rollers Ø10, driven | 8 | PLA+ | Ø3 D-bore, no grub screw — the flat is the key |
| Slider beds | 8 | PLA+ | The wear surface — print smooth side up. **Consider facing with UHMW or PTFE tape:** published conveyor practice puts a PU belt on UHMW at µ 0.03–0.06 vs 0.15–0.30 on steel, and printed PLA sits nearer the steel end. Cuts belt drag ~5–10×. Not needed at v0's 10–16× torque margin; matters at 8 motors on one supply |
| Tensioner blocks | 16 | PETG | 8 mm sliding travel at the infeed nose |
| Return guides | 8 | PLA+ | |
| Corner guide rails | 4 | PLA+ | Arrests the part's incoming momentum |
| Frame connectors | 8 | PETG | Sets and holds the 1.5 mm inter-module gap |

**~460 g total**, so half a spool. PETG where it's loaded, PLA+ where the fit matters —
same split as CableCell uses. Tree supports on, per your standing profile.

---

## 4 · The belt — the one genuinely awkward item

50 mm-wide closed belt loops are not something you can just buy. Two real options:

### Option A — printed TPU loops (recommended)

**Filament: TPU 95A.** Shore 95A — roughly a skateboard wheel. Bambu's *TPU 95A HF*
has an A1 profile and is the path of least resistance. Elongation at break is
400–500%, which is what makes a printed part work as a belt at all: it bends round a
Ø10 roller and springs back instead of creasing. Softer grades (85A) are more
rubbery and considerably harder to print — not worth it here.

Print each belt as a **thin-walled cylinder standing upright on the plate**, sized so
its circumference equals the belt path. No seam, no splicing, no glue.

| Belt | Cylinder mean Ø | Height | Wall | TPU |
|---|---|---|---|---|
| Straight (×4) | 79.8 mm | 50 mm | 1.0 mm | ~15 g each |
| Corner (×4) | 47.9 mm | 50 mm | 1.0 mm | ~9 g each |

**~100 g of TPU total** — one spool covers it several times over.

**Wall is 1.0 mm, not 1.5, and that is set by the roller.** Belt practice wants
pulley-diameter ÷ belt-thickness ≥ 10; Ø10 rollers put a 1.5 mm wall at **6.7**. It
would not crack — 13% outer-fibre strain is nothing against 400% elongation — but a
stiff belt fights the wrap and lifts off a small nose roller, which is precisely the
geometry the nose exists to protect. 1.0 mm gives **D/t = 10.0** and prints as a
clean 2–3 perimeters at a 0.4 mm nozzle.

These diameters are **computed by the CAD, not estimated**: `build.log` reports belt
path 250.6 mm (straight) and 150.6 mm (corner). Note they are measured at the belt's
**neutral axis** (roller Ø + wall), not at the roller surface — the neutral axis is
the fibre that neither stretches nor compresses, so it is the only length that stays
constant as the belt wraps, and it is what the printed cylinder's *mean* diameter has
to match. Measuring at the roller surface undersizes every loop by π × wall of
circumference, against only 8 mm of take-up travel to absorb it.

Rerun `build_parts.py` after any dimension change and re-read the log.

⚠ **TPU must not go through the AMS** — flexible filament buckles in the long PTFE
path. It wants an external spool holder and a direct feed to the extruder. The A1
supports this, but confirm it with one test print before the design leans on it:
it's the only new material in the whole build.

### Option B — bought belting, spliced

PU or PVC conveyor belting by the metre, cut to length and joined. Cheaper per metre
but the splice is a hand skill, it's the weakest point of every loop, and 8 of them
is 8 chances to get it wrong. Only worth it if TPU turns out to be a problem.

---

## 5 · Electronics

| Item | Qty | Notes |
|---|---|---|
| N20 gearmotor, 12 V, **250 RPM**, 3 mm D-shaft | 8 (+2 spare) | §1 — NOT 100 RPM, the roller is Ø10 now |
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
| ~~MR105ZZ bearings~~ | 0 | **Dropped.** At Ø10 the bearing OD *is* the roller — no room. Rollers run as plain bearings on the axle; load is belt tension only |
| 4 mm steel rod, 1 m | 1 | Idler and stub axles, ~70 mm each |
| M3 × 16 bolts + nyloc nuts | 60 | Frame assembly |
| M4 × 20 bolts + nuts | 20 | Tensioners |
| M2 × 6 bolts | 20 | N20 face mount, straight into the side plate |
| ~~M3 × 6 grub screws~~ | 0 | **Dropped.** Ø3.2 through a 3.5 mm wall leaves nothing; the D-flat carries the 0.005 N·m on its own |

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
| ~~MR105ZZ~~ | $0 — dropped |
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
N20 stall current (measure it — it sizes the PSU) · µ ≈ 0.35 belt-on-PLA-bed, pessimistic (UHMW would be 0.03–0.06) ·
all filament weights · all prices · the 610-link count in §7 · whether TPU feeds
acceptably on your A1 without the AMS.

**The one thing to measure before spending money:** print one roller and one bracket,
check the bearing fit, then buy the bearings. Everything else on this list is
forgiving; a 24-pack of the wrong bearing is not.
