# TendWright Pico bridge firmware (MicroPython) — plan #619
#
# Reads bench sensors and streams JSON lines over USB-CDC at ~20 Hz:
#   {"hello": "tendwright-pico", "version": 1, "inputs": ["nest"]}   (once)
#   {"nest": true, "seq": 41, "t_ms": 123456}                        (stream)
#
# Wiring (KW12-3 part-present switch): COM -> any GND pin, NO -> GP16.
# Internal pull-up: open = 1 on the pin = no part; pressed = 0 = part
# seated ("invert" maps that to nest=true). Add future switches / the FSR
# by extending INPUTS — the loop handles any number of entries.
#
# Flash: hold BOOTSEL, plug in, copy the MicroPython UF2, then copy this
# file to the Pico as main.py (it runs on every boot).

import json
import time

from machine import Pin

INPUTS = {
    # name: (gpio, invert)  — invert=True for switch-to-GND with pull-up
    "nest": (16, True),
}
DEBOUNCE_MS = 15
RATE_HZ = 20
VERSION = 1

_pins = {name: Pin(gpio, Pin.IN, Pin.PULL_UP)
         for name, (gpio, _invert) in INPUTS.items()}
_stable = {}
_candidate = {}
_since = {}


def read_debounced(name):
    """Debounced logical state: a change must hold DEBOUNCE_MS to count."""
    gpio, invert = INPUTS[name]
    raw = _pins[name].value()
    logical = (raw == 0) if invert else (raw == 1)
    now = time.ticks_ms()
    if name not in _stable:
        _stable[name] = logical
        _candidate[name] = logical
        _since[name] = now
        return logical
    if logical != _candidate[name]:
        _candidate[name] = logical
        _since[name] = now
    elif logical != _stable[name] and \
            time.ticks_diff(now, _since[name]) >= DEBOUNCE_MS:
        _stable[name] = logical
    return _stable[name]


def main():
    print(json.dumps({"hello": "tendwright-pico", "version": VERSION,
                      "inputs": sorted(INPUTS.keys())}))
    seq = 0
    period_ms = 1000 // RATE_HZ
    while True:
        start = time.ticks_ms()
        sample = {name: read_debounced(name) for name in INPUTS}
        sample["seq"] = seq
        sample["t_ms"] = start
        print(json.dumps(sample))
        seq += 1
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if elapsed < period_ms:
            time.sleep_ms(period_ms - elapsed)


main()
