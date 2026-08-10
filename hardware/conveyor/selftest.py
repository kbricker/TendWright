"""selftest — exercise the conveyor firmware's protocol without a Pico.

    uv run python -m hardware.conveyor.selftest

The firmware is MicroPython and cannot be imported on the desk: it pulls in
`machine`, and `select.poll` doesn't exist on Windows at all. This loads the
source with those two stubbed and a controllable clock patched in, then
drives real command lines through the real handler.

That buys the parts a bench session is worst at proving: that an
out-of-range duty is rejected rather than saturated, that one bad motor in a
multi-motor command doesn't discard the good half, and that the watchdog
coasts a running motor. Checking those with a screwdriver in one hand means
noticing what did NOT happen.

Plain asserts and a __main__ block — the repo has no test framework and
adding one would be a new dependency.
"""

from __future__ import annotations

import json
import os
import sys
import types

FIRMWARE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "firmware", "main.py")


class FakePin:
    OUT = "out"
    IN = "in"
    PULL_UP = "pull_up"

    def __init__(self, gp, mode=None, value=0, **kw):
        self.gp = gp
        self._value = value

    def value(self, v=None):
        if v is None:
            return self._value
        self._value = v
        return None


class FakePWM:
    def __init__(self, pin):
        self.pin = pin
        self._freq = 0
        self._duty = 0

    def freq(self, f):
        self._freq = f

    def duty_u16(self, d):
        self._duty = d


class FakePoll:
    def __init__(self):
        self.ready = False

    def register(self, obj, mask):
        pass

    def poll(self, timeout=0):
        return [] if not self.ready else [(None, 1)]


class FakeClock:
    def __init__(self):
        self.now = 1000

    def advance(self, ms):
        self.now += ms

    def ticks_ms(self):
        return self.now

    def ticks_diff(self, a, b):
        return a - b

    def ticks_add(self, t, d):
        return t + d

    def sleep_ms(self, ms):
        pass


def load_firmware():
    machine = types.ModuleType("machine")
    machine.Pin = FakePin
    machine.PWM = FakePWM
    select = types.ModuleType("select")
    select.POLLIN = 1
    select.poll = FakePoll
    sys.modules["machine"] = machine
    sys.modules["select"] = select

    with open(FIRMWARE, encoding="utf-8") as fh:
        src = fh.read()

    # The firmware calls main() bare at the bottom, matching the sibling nest
    # firmware — that is what running as main.py on boot needs. Strip it, and
    # fail loudly if it is not there rather than exec'ing into the real loop:
    # a self-test that hangs forever is worse than one that errors.
    marker = "\nmain()\n"
    if not src.endswith(marker):
        raise SystemExit(f"{FIRMWARE} no longer ends with a bare main() call "
                         f"— update selftest.py before it hangs")
    src = src[: -len(marker)]

    ns: dict = {"__name__": "conveyor_firmware"}
    exec(compile(src, FIRMWARE, "exec"), ns)

    clock = FakeClock()
    ns["time"] = clock

    emitted: list[dict] = []
    ns["emit"] = emitted.append

    ns["setup"]()
    ns["stop_all"]()
    emitted.clear()
    return ns, clock, emitted


def send(ns, emitted, doc):
    emitted.clear()
    ns["handle"](json.dumps(doc) if isinstance(doc, dict) else doc)
    return list(emitted)


def duty_u16(ns, motor):
    return ns["_pwm"][motor]._duty


def pins(ns, motor):
    return ns["_in1"][motor].value(), ns["_in2"][motor].value()


def check(label, condition, detail=""):
    if condition:
        print(f"  ok    {label}")
        return 0
    print(f"  FAIL  {label}" + (f" — {detail}" if detail else ""))
    return 1


def main() -> int:
    ns, clock, emitted = load_firmware()
    motors = sorted(ns["MOTORS"])
    m0, m1 = motors[0], motors[1]
    fails = 0
    print(f"firmware protocol selftest — motors {', '.join(motors)}\n")

    print("init")
    fails += check("every motor starts stopped",
                   all(ns["_duty"][m] == 0 for m in motors))
    fails += check("every PWM starts at zero",
                   all(duty_u16(ns, m) == 0 for m in motors))
    fails += check("STBY is enabled after the channels are known-stopped",
                   ns["_stby"].value() == 1)

    print("\nabsolute set")
    replies = send(ns, emitted, {"cmd": "set", "motor": m0, "duty": 60,
                                 "dir": "fwd"})
    fails += check("applied and acked",
                   replies and replies[0].get("applied", {}).get(m0)
                   == {"duty": 60, "dir": "fwd"}, str(replies))
    fails += check("PWM follows duty",
                   duty_u16(ns, m0) == int(60 * 65535 // 100),
                   str(duty_u16(ns, m0)))
    fails += check("forward drives IN1 high, IN2 low", pins(ns, m0) == (1, 0))

    send(ns, emitted, {"cmd": "set", "motor": m0, "dir": "rev"})
    fails += check("reverse swaps the IN pins", pins(ns, m0) == (0, 1))
    fails += check("reversing preserves duty", ns["_duty"][m0] == 60)

    before = (ns["_duty"][m0], ns["_dir"][m0], duty_u16(ns, m0))
    send(ns, emitted, {"cmd": "set", "motor": m0, "duty": 60, "dir": "rev"})
    after = (ns["_duty"][m0], ns["_dir"][m0], duty_u16(ns, m0))
    fails += check("a repeated command is idempotent", before == after,
                   f"{before} -> {after}")

    print("\nrejection (never silently saturated)")
    for bad, why in ((150, "over range"), (-5, "under range"),
                     ("fast", "not a number"), (True, "bool is not a duty")):
        replies = send(ns, emitted, {"cmd": "set", "motor": m1, "duty": bad})
        rejected = replies and replies[0].get("rejected", {})
        fails += check(f"duty={bad!r} rejected ({why})", bool(rejected),
                       str(replies))
        fails += check(f"duty={bad!r} left the motor untouched",
                       ns["_duty"][m1] == 0)

    replies = send(ns, emitted, {"cmd": "set", "motor": m1, "dir": "sideways"})
    fails += check("a bad direction is rejected",
                   bool(replies and replies[0].get("rejected")), str(replies))

    replies = send(ns, emitted, "{not json")
    fails += check("malformed json errors instead of crashing",
                   bool(replies) and "error" in replies[0], str(replies))

    replies = send(ns, emitted, {"cmd": "waltz"})
    fails += check("an unknown command is reported",
                   bool(replies) and replies[0].get("error") == "unknown cmd")

    print("\npartial application")
    replies = send(ns, emitted, {"cmd": "set", "motors": {
        m0: {"duty": 40, "dir": "fwd"},
        "nonexistent": {"duty": 50},
        m1: {"duty": 200},
    }})
    reply = replies[0] if replies else {}
    fails += check("the valid motor still landed",
                   reply.get("applied", {}).get(m0) == {"duty": 40,
                                                        "dir": "fwd"},
                   str(reply))
    fails += check("both bad entries are named in the reply",
                   set(reply.get("rejected", {})) == {"nonexistent", m1},
                   str(reply.get("rejected")))
    fails += check("the out-of-range motor did not move", ns["_duty"][m1] == 0)

    print("\nkick-start")
    send(ns, emitted, {"cmd": "set", "motor": m1, "duty": 0})
    send(ns, emitted, {"cmd": "set", "motor": m1, "duty": 20, "dir": "fwd"})
    fails += check("a low duty from rest starts at full",
                   duty_u16(ns, m1) == int(ns["KICK_DUTY"] * 65535 // 100),
                   str(duty_u16(ns, m1)))
    fails += check("the commanded duty is what gets reported",
                   ns["_duty"][m1] == 20)
    clock.advance(ns["KICK_MS"] + 1)
    ns["_kick_until"][m1] = 0
    ns["apply_motor"](m1)
    fails += check("the kick expires to the commanded duty",
                   duty_u16(ns, m1) == int(20 * 65535 // 100),
                   str(duty_u16(ns, m1)))

    send(ns, emitted, {"cmd": "set", "motor": m1, "duty": 0})
    send(ns, emitted, {"cmd": "set", "motor": m1, "duty": 90, "dir": "fwd"})
    fails += check("a high duty from rest is not kicked",
                   duty_u16(ns, m1) == int(90 * 65535 // 100),
                   str(duty_u16(ns, m1)))

    print("\nstop coasts rather than brakes")
    send(ns, emitted, {"cmd": "set", "motor": m0, "duty": 70, "dir": "fwd"})
    replies = send(ns, emitted, {"cmd": "stop"})
    fails += check("every motor reads zero",
                   all(ns["_duty"][m] == 0 for m in motors))
    fails += check("IN1 and IN2 both low (high-Z, not short-brake)",
                   all(pins(ns, m) == (0, 0) for m in motors))
    fails += check("every PWM is zero",
                   all(duty_u16(ns, m) == 0 for m in motors))

    print("\nwatchdog")
    send(ns, emitted, {"cmd": "set", "motor": m0, "duty": 80, "dir": "fwd"})
    emitted.clear()
    clock.advance(ns["COMMAND_TIMEOUT_MS"] - 50)
    ns["service_watchdog"]()
    fails += check("still running just inside the window",
                   ns["_duty"][m0] == 80, str(ns["_duty"][m0]))
    clock.advance(100)
    ns["service_watchdog"]()
    fails += check("coasts every motor once the window passes",
                   all(ns["_duty"][m] == 0 for m in motors))
    fails += check("and says so", any("command timeout" == d.get("error")
                                      for d in emitted), str(emitted))
    emitted.clear()
    ns["service_watchdog"]()
    fails += check("does not re-fire while the host stays away",
                   not emitted, str(emitted))
    replies = send(ns, emitted, {"cmd": "ping"})
    fails += check("a returning host is told motors were coasted",
                   any("note" in d for d in replies), str(replies))

    print("\nstate reporting")
    send(ns, emitted, {"cmd": "set", "motor": m0, "duty": 55, "dir": "rev"})
    replies = send(ns, emitted, {"cmd": "state"})
    reported = replies[0].get("motors", {}) if replies else {}
    fails += check("state reflects the last absolute command",
                   reported.get(m0) == {"duty": 55, "dir": "rev"},
                   str(reported))

    print()
    if fails:
        print(f"{fails} check(s) FAILED")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
