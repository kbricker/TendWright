# Conveyor v0 — order list, 2026-09-25

Plan #835: two straight modules and one tapered-roller curve, three motors. Stock and prices were checked on 2026-09-25; recheck anything marked *unverified* at checkout. Reasoning and full-loop quantities are in the [BOM](conveyor-bom-v1-loop-2026-08-08.md).

## Already on hand

- **Raspberry Pi Pico 2 W:** from the Freenove Basic Starter Kit, bought for #717.1 and never used.
  - Flash the Pico 2 W MicroPython build.
  - The kit's breadboard, jumper wires and micro-USB cable cover the v0 wiring.
- Pico screw-terminal breakout, hookup wire, PLA+ and PETG.
- Wire tools from the arm-cable job ([wiring-hardware.md](../wiring-hardware.md)).

## Order

| Item | Source · part | Qty | Price | Stock |
|---|---|---|---|---|
| N20 gearmotor, 12 V, 270 RPM, D-shaft | [ServoCity 638122](https://www.servocity.com/270-rpm-micro-gear-motor/) | 4 | $12.99 ea | in stock |
| N20 motor enclosure, acetal, M4 mounting holes | [ServoCity 1705-0016-0001](https://www.servocity.com/n20-gear-motor-enclosure/) | 4 | $2.49 ea | in stock |
| Gear Motor Input Board A, solder-on, 2-pin 0.1" header | [ServoCity 605112](https://www.servocity.com/gear-motor-input-board-a/) | 4 | $1.99 ea | in stock |
| TB6612FNG motor driver, headers loose | [Adafruit 2448](https://www.adafruit.com/product/2448) | 4 | $6.95 ea | in stock |
| Bench supply, 0–30 V / 0–5 A, linear, current limit, 0.01 A display | [Tekpower TP3005T](https://kaito.us/products/tekpower-tp3005t-digital-variable-dc-power-supply-30-volts-5-amps-with-lock) | 1 | $89.99 | add-to-cart |
| Female/male jumper wires, 20 × 12" | [Adafruit 1952](https://www.adafruit.com/product/1952) | 1 | $3.95 | in stock |
| Raspberry Pi Pico 2, for the nest bridge | [PiShop](https://www.pishop.us/product/raspberry-pi-pico-2/) | 1 | $5.00 | in stock |
| Pico header set, solder-on | [PiShop](https://www.pishop.us/product/raspberry-pi-pico-header-set/) | 1 | $2.45 | in stock |
| TPU 95A HF, 1 kg | [Bambu Lab](https://us.store.bambulab.com/products/tpu-95a-hf) | 1 | $41.99 | in stock |
| 4 mm × 300 mm 304 stainless rod, 10-pack | [Harfington p-1063687](https://www.harfington.com/products/p-1063687) | 1 | $13.03 | in stock |
| 3 mm × 300 mm 304 stainless rod, 10-pack | [Harfington p-1063684](https://www.harfington.com/products/p-1063684) | 1 | $10.54 | in stock |
| M2 / M3 / M4 socket-head screws with nuts | any assortment with M2 × 8, M3 × 16, M4 × 20 | 1 kit | ~$15–25 | *unverified* |
| Nitrile O-ring assortment, metric | any kit, search `metric nitrile o-ring assortment` | 1 kit | ~$10–15 | *unverified* |
| **Loop (#840):** GA12-N20 12 V 300 RPM, "with wire" | AliExpress, search `GA12-N20 12V 300RPM` | 10 | ~$2–3.50 ea | *unverified*, 1–4 wk |

Total: about $265 before shipping, screws and O-rings, plus ~$30 for the loop motors.

- **The bench supply powers every motor** through the drivers' shared motor rail, set to 12 V. It replaces a $25 12 V brick and its barrel-jack adapter.
  - Set a low current limit for the first power-up, so a wiring mistake trips the limit instead of burning out a driver or the Pico.
  - Its current display gives the motors' measured stall current, which sizes the loop's supply (#840).
  - The Pico still runs from USB. Tie the supply's − terminal to the Pico's GND, not the green earth post.
- **The second Pico 2 goes to the cell project's nest bridge (#717.1).** The conveyor runs on the kit's Pico 2 W, so both can be built. PiShop's Pico 2 ships without headers; the $2.45 set solders on.
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
| Soldering iron | [Pinecil V2](https://pine64.com/product/pinecil-smart-mini-portable-soldering-iron/) + any 0.8 mm rosin-core solder | $25.99 + solder | driver and Pico headers, motor input boards |
| Multimeter | [Klein MM325 at DigiKey](https://www.digikey.com/en/products/detail/klein-tools-inc/MM325/16649074) | $39.74 | optional: continuity and voltage checks. The bench supply already shows motor current |

## Later, with the loop

- Perfboard.
- UHMW or PTFE tape for the slider beds.
