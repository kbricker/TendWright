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
    # name: (gpio, invert)  — invert=True for switch-to-GND with pull-up.
    # Digital switches only; the future FSR needs an ADC read path this
    # map doesn't model yet.
    "nest": (16, True),
}
# Effective debounce is max(DEBOUNCE_MS, one sample period) because a
# candidate change is only re-examined at the next sample: at 20 Hz a
# change commits on the 2nd consecutive agreeing sample (50-100 ms).
DEBOUNCE_MS = 30
RATE_HZ = 20
HELLO_EVERY = 100  # re-emit the hello line every N samples (~5 s) — the
                   # boot-time hello is lost if no host session is attached
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
        # First sample seeds the stable state undebounced; a bounce edge
        # caught here self-corrects within ~2 sample periods.
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


def hello():
    print(json.dumps({"hello": "tendwright-pico", "version": VERSION,
                      "inputs": sorted(INPUTS.keys())}))


def main():
    hello()
    seq = 0
    period_ms = 1000 // RATE_HZ
    while True:
        start = time.ticks_ms()
        if seq % HELLO_EVERY == 0 and seq:
            hello()
        sample = {name: read_debounced(name) for name in INPUTS}
        sample["seq"] = seq
        sample["t_ms"] = start
        print(json.dumps(sample))
        seq += 1
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if elapsed < period_ms:
            time.sleep_ms(period_ms - elapsed)


main()
