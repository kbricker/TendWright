# Conveyor v0 — order list, 2026-09-25

Plan #835: two straight modules and one tapered-roller curve, three motors. Stock and prices were checked on 2026-09-25; recheck anything marked *unverified* at checkout. Reasoning and full-loop quantities are in the [BOM](conveyor-bom-v1-loop-2026-08-08.md).

## Already on hand

- **Raspberry Pi Pico 2 W:** from the Freenove Basic Starter Kit, bought for #717.1 and never used.
  - Flash the Pico 2 W MicroPython build.
  - The kit's breadboard, jumper wires and micro-USB cable cover the v0 wiring.
- Pico screw-terminal breakout, hookup wire, PLA+ and PETG.
- Wire tools from the arm-cable job ([wiring-hardware.md](../wiring-hardware.md)).
- Soldering iron and solder.

## Order

| Item | Source · part | Qty | Price | Stock |
|---|---|---|---|---|
| N20 gearmotor, 12 V, 270 RPM, D-shaft | [ServoCity 638122](https://www.servocity.com/270-rpm-micro-gear-motor/) | 4 | $12.99 ea | in stock |
| N20 motor enclosure, acetal, M4 mounting holes | [ServoCity 1705-0016-0001](https://www.servocity.com/n20-gear-motor-enclosure/) | 4 | $2.49 ea | in stock |
| Gear Motor Input Board A, solder-on, 2-pin 0.1" header | [ServoCity 605112](https://www.servocity.com/gear-motor-input-board-a/) | 4 | $1.99 ea | in stock |
| TB6612FNG motor driver, headers loose | [Adafruit 2448](https://www.adafruit.com/product/2448) | 4 | $6.95 ea | in stock |
| Bench supply, 0–30 V / 0–10 A, switching, current limit, output on/off, 4-digit display | [WANPTEK TPS-C3010 (Amazon B0DR12RNPY)](https://www.amazon.com/dp/B0DR12RNPY) | 1 | $54.13 | in stock |
| Female/male jumper wires, 20 × 12" | [Adafruit 1952](https://www.adafruit.com/product/1952) | 1 | $3.95 | in stock |
| Freenove Pico 2 W, headers pre-soldered, for the nest bridge | [Amazon B0DRJXPPWL](https://www.amazon.com/dp/B0DRJXPPWL) | 1 | $18.95 | in stock |
| TPU 95A HF, 1 kg | [Bambu Lab](https://us.store.bambulab.com/products/tpu-95a-hf) | 1 | $41.99 | in stock |
| 4 mm × 300 mm 304 stainless rod, 5-pack | [Amazon B0G791YZKV](https://www.amazon.com/dp/B0G791YZKV) | 1 | $6.99 | in stock |
| 3 mm × 300 mm 304 stainless rod, 12-pack | [Amazon B0DCBCRB1C](https://www.amazon.com/dp/B0DCBCRB1C) | 1 | $8.99 | in stock |
| M2 / M2.5 / M3 / M4 socket-head screws, 6–20 mm, with nuts (mxuteuk 888 pc) | [Amazon B0G8F366MV](https://www.amazon.com/dp/B0G8F366MV) | 1 kit | $8.99 | in stock |
| Nitrile O-ring kit, 20 sizes incl. 16×2, 18×2, 20×2, 22×2, 25×2.4 mm (XBVV) | [Amazon B0CBTYXVCV](https://www.amazon.com/dp/B0CBTYXVCV) | 1 kit | $7.59 | in stock |
Total: about $250 before shipping. Kyle ordered the ServoCity part ($69.88) on 2026-09-25 and put the bench supply in his Amazon cart; the rods, screws and O-rings go in the same Amazon cart.

- **The bench supply powers every motor** through the drivers' shared motor rail, set to 12 V. It replaces a $25 12 V brick and its barrel-jack adapter.
  - **Before first use, set the rear 115V/230V switch to 115V.** The listing says so for US use.
  - Use the output button: set 12 V and the limit with the output off, then switch it on. Save bring-up (low limit) and running settings as memory presets.
  - The Tekpower TP3005T ($89.99, linear) was the first pick. The WANPTEK has the same essentials for $36 less, plus the output switch.
  - Set a low current limit for the first power-up, so a wiring mistake trips the limit instead of burning out a driver or the Pico.
  - Its current display gives the motors' measured stall current, which sizes the loop's supply (#840).
  - The Pico still runs from USB. Tie the supply's − terminal to the Pico's GND, not the green earth post.
- **The second board goes to the cell project's nest bridge (#717.1).** The conveyor runs on the kit's Pico 2 W, so both can be built.
  - It's the same Pico 2 W with headers already soldered. Both bridges then use one MicroPython build.
  - It rides in the Amazon cart instead of a separate PiShop order.
- **Motors.** Out of Darts' 300 RPM motor, the original pick, is sold out in every variant.
  - ServoCity's 270 RPM motor runs the belt at 141 mm/s top.
  - Stall is 17 oz-in (1.2 kg·cm) and 1.6 A per motor. Three stalled at once draw 4.8 A, inside the supply's 5 A.
  - The terminals are spade tabs. Solder an input board onto each one. After that, the motor plugs in with a female/male jumper, so motors swap without the iron.
  - The enclosure covers the open gearbox, which is where TPU strings and debris would jam it. It holds the input board inside and bolts on through 4 mm holes, so the side plate's printed clamp becomes an M4 bolt pattern.
- **Four drivers** cover the loop; v0 uses two. Each one needs its header soldered on.
- **The 3 mm rod** is for the roller curve's small rollers. The 4 mm rod is for the straights' axles.
- **O-rings link the curve's rollers.**
  - The rollers are evenly spaced, so every link is the same size.
  - The CAD sizes the grooves to a ring picked from the kit. Caliper it rather than trusting the chart.
  - Printed drive rings are dropped.
- **The TPU is for the straights' belts.** A 50 mm closed loop isn't something you can buy, so they're printed.

## Only if you don't have one

| Tool | Source | Price | Why |
|---|---|---|---|
| Multimeter | [Klein MM325 at DigiKey](https://www.digikey.com/en/products/detail/klein-tools-inc/MM325/16649074) | $39.74 | optional: continuity and voltage checks. The bench supply already shows motor current |

## Later, with the loop

- About 5 more ServoCity sets (motor + enclosure + input board), the same parts as v0 so every module matches in mount and speed. They are in stock in the US with no long lead, so they wait until v0 works. The AliExpress motors are dropped.
- Perfboard.
- UHMW or PTFE tape for the slider beds.
