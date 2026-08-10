# TendWright conveyor firmware (MicroPython, Pico 2) — plan #835
#
# Drives N20 gearmotors through TB6612FNG dual drivers and speaks JSON
# lines over USB-CDC. Mirrors the idiom in ../../pico/firmware/main.py
# (hello line + streamed state + host-side bounded drain) rather than
# inventing a second one.
#
# Out (one JSON object per line):
#   {"hello": "tendwright-conveyor", "version": 1, "motors": [...]}
#   {"motors": {"s1": {"duty": 60, "dir": "fwd"}, ...}, "seq": 41,
#    "t_ms": 123456, "stby": true}
#   {"ack": "set", "applied": {...}, "rejected": {...}}
#   {"error": "...", "detail": "..."}
#
# In (one JSON object per line):
#   {"cmd": "set", "motor": "s1", "duty": 60, "dir": "fwd"}
#   {"cmd": "set", "motors": {"s1": {"duty": 60, "dir": "fwd"},
#                             "c": {"duty": 0}}}
#   {"cmd": "stop"}                      coast everything, now
#   {"cmd": "state"}                     reply with the current state
#   {"cmd": "ping"}                      liveness; also feeds the watchdog
#
# Every command is ABSOLUTE, never a delta — "duty=60", never "faster".
# That makes a duplicate harmless, which matters because a USB retransmit
# or an impatient operator is indistinguishable from an intended repeat.
#
# Wiring per TB6612FNG channel: PWMx -> a PWM-capable GP, xIN1/xIN2 -> two
# plain GPs, STBY -> one GP shared by every driver. VM is the 12 V motor
# rail, VCC the Pico's 3V3 logic rail, and the two supplies MUST share a
# ground — see cad/conveyor/README.md.
#
# Flash: hold BOOTSEL, plug in, copy the MicroPython UF2, then copy this
# file to the Pico as main.py (it runs on every boot).

import json
import select
import sys
import time

from machine import Pin, PWM

# name: (pwm_gp, in1_gp, in2_gp). Names match the sim's MODULES table in
# cad/conveyor/sim_conveyor.py so a duty in a log maps to a module without
# a lookup. v0 is 3 motors on 2 drivers; the loop (#840) needs 8, so this
# is a table rather than three hardcoded channels.
MOTORS = {
    "s1": (0, 1, 2),    # driver 0, channel A — first straight
    "c":  (3, 4, 5),    # driver 0, channel B — corner
    "s2": (6, 7, 8),    # driver 1, channel A — second straight
}
STBY_GP = 15

PWM_FREQ = 20000        # above audible; an N20 at low duty whines otherwise
VERSION = 1

# No host traffic for this long and every motor coasts. The one genuinely
# unsafe state in the build is a motor left driven after the host goes
# away, and USB gives no disconnect signal to a device that isn't looking.
COMMAND_TIMEOUT_MS = 1000

# An N20 through a gearbox may not break stiction at low duty. A start
# from rest gets full duty briefly, then settles to what was asked for.
KICK_MS = 120
KICK_BELOW = 40         # only kick when the target is under this
KICK_DUTY = 100

RATE_HZ = 20
HELLO_EVERY = 100       # ~5 s — the boot hello is lost if no host is attached
MAX_LINE = 512          # a longer line is a framing fault, not a command

DIRECTIONS = ("fwd", "rev")

_pwm = {}
_in1 = {}
_in2 = {}
_duty = {}
_dir = {}
_kick_until = {}

_stby = None
_last_cmd_ms = 0
_timed_out = False
_buf = ""
_poll = select.poll()


def emit(doc):
    print(json.dumps(doc))


def setup():
    global _stby, _last_cmd_ms
    for name, (pwm_gp, in1_gp, in2_gp) in MOTORS.items():
        pwm = PWM(Pin(pwm_gp))
        pwm.freq(PWM_FREQ)
        pwm.duty_u16(0)
        _pwm[name] = pwm
        _in1[name] = Pin(in1_gp, Pin.OUT, value=0)
        _in2[name] = Pin(in2_gp, Pin.OUT, value=0)
        _duty[name] = 0
        _dir[name] = "fwd"
        _kick_until[name] = 0
    # STBY low until every channel is known-stopped: the drivers must not
    # be enabled while the IN pins are still floating at power-up.
    _stby = Pin(STBY_GP, Pin.OUT, value=0)
    for name in MOTORS:
        apply_motor(name)
    _stby.value(1)
    _poll.register(sys.stdin, select.POLLIN)
    _last_cmd_ms = time.ticks_ms()


def apply_motor(name):
    duty = _duty[name]
    if duty <= 0:
        # Coast, not brake: IN1=IN2=0 is high-Z on a TB6612FNG. Braking a
        # loaded belt drive slams the printed D-flat, which is the part
        # with the least margin in the whole build.
        _in1[name].value(0)
        _in2[name].value(0)
        _pwm[name].duty_u16(0)
        return
    forward = _dir[name] == "fwd"
    _in1[name].value(1 if forward else 0)
    _in2[name].value(0 if forward else 1)
    if time.ticks_diff(_kick_until[name], time.ticks_ms()) > 0:
        duty = KICK_DUTY
    _pwm[name].duty_u16(int(duty * 65535 // 100))


def set_motor(name, duty, direction):
    was_stopped = _duty[name] <= 0
    if direction is not None:
        _dir[name] = direction
    if duty is not None:
        if was_stopped and duty > 0 and duty < KICK_BELOW:
            _kick_until[name] = time.ticks_add(time.ticks_ms(), KICK_MS)
        elif duty <= 0:
            _kick_until[name] = 0
        _duty[name] = duty
    apply_motor(name)


def stop_all():
    for name in MOTORS:
        _duty[name] = 0
        _kick_until[name] = 0
        apply_motor(name)


def motor_state():
    return {name: {"duty": _duty[name], "dir": _dir[name]} for name in MOTORS}


def parse_one(name, spec):
    # Returns (duty, direction) or raises ValueError with a reason the host
    # can act on. Out-of-range is REJECTED, never silently saturated: a host
    # that asks for 150 has a bug, and clamping hides it behind a conveyor
    # that merely runs a bit slower than the operator expected.
    if name not in MOTORS:
        raise ValueError("unknown motor (have: %s)" % ", ".join(sorted(MOTORS)))
    if not isinstance(spec, dict):
        raise ValueError("expected an object of {duty, dir}")
    duty = spec.get("duty")
    if duty is not None:
        if isinstance(duty, bool) or not isinstance(duty, (int, float)):
            raise ValueError("duty must be a number")
        if duty < 0 or duty > 100:
            raise ValueError("duty %s out of range 0-100" % duty)
        duty = int(duty)
    direction = spec.get("dir")
    if direction is not None and direction not in DIRECTIONS:
        raise ValueError("dir must be one of %s" % ", ".join(DIRECTIONS))
    if duty is None and direction is None:
        raise ValueError("nothing to set (want duty and/or dir)")
    return duty, direction


def handle_set(doc):
    specs = doc.get("motors")
    if specs is None:
        name = doc.get("motor")
        if name is None:
            raise ValueError("set needs either motor= or motors=")
        specs = {name: doc}
    if not isinstance(specs, dict):
        raise ValueError("motors must be an object keyed by motor name")
    applied = {}
    rejected = {}
    # Per-motor, so one bad name cannot silently discard the good half of
    # a multi-motor command. The reply reports what actually landed.
    for name, spec in specs.items():
        try:
            duty, direction = parse_one(name, spec)
        except ValueError as exc:
            rejected[name] = str(exc)
            continue
        set_motor(name, duty, direction)
        applied[name] = {"duty": _duty[name], "dir": _dir[name]}
    return {"ack": "set", "applied": applied, "rejected": rejected}


def handle(line):
    global _last_cmd_ms, _timed_out
    try:
        doc = json.loads(line)
    except (ValueError, TypeError):
        emit({"error": "malformed json", "detail": line[:120]})
        return
    if not isinstance(doc, dict):
        emit({"error": "expected a json object", "detail": line[:120]})
        return
    cmd = doc.get("cmd")

    # Any well-formed command proves the host is alive, including one that
    # then fails to parse its arguments. The watchdog guards against the
    # host DISAPPEARING, not against it sending nonsense.
    _last_cmd_ms = time.ticks_ms()
    if _timed_out:
        _timed_out = False
        emit({"note": "host returned; motors were coasted by the watchdog"})

    if cmd == "set":
        try:
            emit(handle_set(doc))
        except ValueError as exc:
            emit({"error": "bad set", "detail": str(exc)})
    elif cmd == "stop":
        stop_all()
        emit({"ack": "stop", "applied": motor_state()})
    elif cmd in ("state", "ping"):
        emit({"ack": cmd, "motors": motor_state()})
    else:
        emit({"error": "unknown cmd", "detail": repr(cmd)})


def pump_stdin():
    # Bounded by design: poll(0) goes falsy the moment nothing is buffered,
    # so a silent host costs one poll per loop and a chatty one cannot trap
    # us here past what it actually sent.
    global _buf
    lines = []
    while _poll.poll(0):
        ch = sys.stdin.read(1)
        if not ch:
            break
        if ch == "\n":
            lines.append(_buf)
            _buf = ""
        elif ch != "\r":
            _buf += ch
            if len(_buf) > MAX_LINE:
                _buf = ""
                emit({"error": "line too long", "detail": "dropped >%d bytes"
                      % MAX_LINE})
    return lines


def service_watchdog():
    global _timed_out
    if _timed_out:
        return
    if time.ticks_diff(time.ticks_ms(), _last_cmd_ms) < COMMAND_TIMEOUT_MS:
        return
    running = [n for n in MOTORS if _duty[n] > 0]
    _timed_out = True
    if running:
        stop_all()
        emit({"error": "command timeout", "detail":
              "no host traffic for %d ms; coasted %s"
              % (COMMAND_TIMEOUT_MS, ", ".join(sorted(running)))})


def hello():
    emit({"hello": "tendwright-conveyor", "version": VERSION,
          "motors": sorted(MOTORS.keys()),
          "timeout_ms": COMMAND_TIMEOUT_MS})


def main():
    setup()
    stop_all()          # explicit: a reset must never inherit a spinning motor
    hello()
    seq = 0
    period_ms = 1000 // RATE_HZ
    while True:
        start = time.ticks_ms()
        for line in pump_stdin():
            handle(line)
        service_watchdog()
        for name in MOTORS:
            # Re-apply so a kick-start expires on its own without needing a
            # command to land at the right moment.
            if _kick_until[name] and \
                    time.ticks_diff(_kick_until[name], start) <= 0:
                _kick_until[name] = 0
                apply_motor(name)
        if seq % HELLO_EVERY == 0 and seq:
            hello()
        emit({"motors": motor_state(), "seq": seq, "t_ms": start,
              "stby": bool(_stby.value())})
        seq += 1
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if elapsed < period_ms:
            time.sleep_ms(period_ms - elapsed)


main()
