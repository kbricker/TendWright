# hardware/conveyor — motor bridge for the mini modular conveyor

Plan **#835**. Mechanical design, print plan and BOM live in
[`cad/conveyor/README.md`](../../cad/conveyor/README.md); this is the electronics half.

- `firmware/main.py` — MicroPython for the conveyor Pico 2. Flash: hold BOOTSEL, plug in, copy the MicroPython UF2, copy this file to the board as `main.py`.
- `driver.py` — host-side `ConveyorDriver`
- `run.py` — bring-up CLI
- `selftest.py` — protocol checks, no hardware needed

## Commands

```
uv run python -m hardware.conveyor.selftest              # no Pico required
uv run python -m hardware.conveyor.run watch
uv run python -m hardware.conveyor.run hold --motor s1 --duty 60 --dir rev
uv run python -m hardware.conveyor.run demo
```

## Protocol — JSON lines over USB-CDC

Mirrors the nest bridge (`hardware/pico/`) rather than inventing a second idiom.

**Host → Pico**

- `{"cmd":"set","motor":"s1","duty":60,"dir":"fwd"}` — one motor
- `{"cmd":"set","motors":{"s1":{"duty":60},"c":{"duty":0}}}` — several
- `{"cmd":"stop"}` · `{"cmd":"state"}` · `{"cmd":"ping"}`

**Pico → host**

- `{"hello":"tendwright-conveyor","version":1,...}` — at boot and every ~5 s
- `{"motors":{...},"seq":41,"t_ms":...,"stby":true}` — 20 Hz
- `{"ack":"set","applied":{...},"rejected":{...}}`
- `{"error":"...","detail":"..."}`

## Four things worth knowing before you wire it

- **Commands are absolute, never deltas.** `duty=60`, never "faster". A duplicate is therefore harmless — which matters because a USB retransmit and an impatient operator look identical from here.
- **Out-of-range is rejected, not clamped.** A host asking for `duty=150` gets it back in `rejected`. Clamping to 100 would leave a host bug showing up only as a conveyor that runs slower than expected.
- **The watchdog is a deadman.** No host traffic for 1000 ms and every motor coasts. A running motor therefore needs a live host — `run hold` and `run demo` send keepalives, and `ConveyorDriver.keepalive()` is there for callers with nothing else to say.
- **Stop coasts, it does not brake.** `IN1=IN2=0` is high-Z on a TB6612FNG. Short-braking a loaded belt drive slams the printed D-flat, which has the least margin of anything in the build.

## Pins

Motor names match the sim's `MODULES` table, so a duty in a log maps to a module without a lookup.

| Motor | Driver | PWM | IN1 | IN2 |
|---|---|---|---|---|
| `s1` (first straight) | 0 / A | GP0 | GP1 | GP2 |
| `c` (corner) | 0 / B | GP3 | GP4 | GP5 |
| `s2` (second straight) | 1 / A | GP6 | GP7 | GP8 |

Shared `STBY` on GP15, held low until every channel is known-stopped. PWM at 20 kHz — above audible, or the motors whine at low duty. `MOTORS` is a table, not three hardcoded channels; #840 needs eight.

**Power:** 12 V PSU → driver `VM`, Pico 3V3 → driver `VCC`, **and the two supplies must share a ground.** See `cad/conveyor/README.md`.

## Two Picos, one USB VID

The nest bridge (#717.1) and this board both enumerate as VID `0x2E8A`, so "the single RP2 device" stops identifying anything once both are plugged in. `resolve_conveyor_port` **probes** instead — pings each candidate and waits for a reply naming this firmware.

`hardware/pico/reader.py` still resolves by VID alone and will raise *"multiple Picos found"* when both boards are attached. That is #717.1's code and out of scope here; filed separately. Workarounds until then: pass `--port`, or set up the udev aliases.
