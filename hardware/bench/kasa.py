"""kasa — switch bench power at the TP-Link strip, locally (plan #716.2).

    uv run python -m hardware.bench.kasa list
    uv run python -m hardware.bench.kasa show 192.168.86.90
    uv run python -m hardware.bench.kasa off  192.168.86.90 Light
    uv run python -m hardware.bench.kasa on   192.168.86.90 Light
    uv run python -m hardware.bench.kasa on   192.168.86.90 Arm --confirm Arm

NO DEPENDENCY AND NO ACCOUNT. Kasa devices speak an unauthenticated local
protocol that predates TP-Link's cloud: a JSON command, XOR-autokey
"encrypted" with the key seeded at 171, over udp/9999 for discovery or
tcp/9999 for control. The TCP form needs a 4-byte big-endian length prefix
and the UDP form does not — getting that wrong is the usual reason a first
attempt hangs. That is the whole protocol, so it is hand-rolled here rather
than pulled in: a wrapper around a clean problem is not worth a dependency.

The obfuscation is not security. Anything on the LAN can switch these.
Treat the strip as a physical convenience, never as an interlock.

BOTH DIRECTIONS ARE GATED, FOR DIFFERENT REASONS
------------------------------------------------
This will eventually switch the arm's 12 V supply, and neither direction is
free.

  ON energises servos that may be holding a pose, in a cell that may have
  nobody in front of it. Gate: retype the outlet's alias with --confirm.
  A speed bump against reflexes and stray automation, the same shape as
  camserve's /debug/memory?trim=<pid>.

  OFF drops an arm that has NO BRAKES. The first cut of this file gated
  only `on`, reasoning that an e-stop you have to confirm is not an e-stop.
  That is right for an industrial arm that holds position when de-energised
  and wrong for STS3215 hobby servos, which simply let go — cutting power
  to an extended arm makes it fall on whatever is under it. Kyle
  2026-07-29: *"there must be some auth gate you add to the arms off flow
  ... YOU WILL KNOW if the arm is in or near enough to rest position where
  it can be cycled, also better safe then sorry."*

So `off` on a guarded outlet is gated by MEASUREMENT, not by a prompt: read
the actual encoders and refuse unless every calibrated joint is within
REST_TOL_TICKS of its captured rest pose. Same principle as guards.py —
"a command sent is not a joint moved", so ask the hardware rather than
trusting intent.

  --force overrides either gate. That is deliberate and it is what keeps a
  real e-stop possible: when the arm is doing something worse than falling,
  cutting power NOW is correct and the tool must not stand in the way. Safe
  by default, override explicit.

An outlet already in the requested state is a no-op and skips both gates —
there is nothing to be unsafe about.

Kyle 2026-07-29: nothing is plugged into the KP303 yet, which is exactly
why the switching path was built and exercised NOW — once the arm is on it,
testing the switch is no longer free.

Usage: kasa {list|show|on|off} [target] [outlet] [--confirm ALIAS] [--force]
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

from hardware.errors import BenchError

PORT = 9999
DISCOVERY_TIMEOUT_S = 4.0
CONTROL_TIMEOUT_S = 5.0

# Outlet aliases that may not be switched ON without --confirm. Matched
# case-insensitively against the alias the operator set in the Kasa app,
# so the guard follows the LABEL rather than an outlet index that could be
# rewired. If an outlet is renamed to something not in this list the guard
# stops applying — which is why the list lives in code and is reviewed,
# rather than being inferred from whatever the device happens to report.
GUARDED = ("arm", "servo", "12v", "psu", "spindle", "cnc")


def encrypt(payload: bytes) -> bytes:
    """XOR autokey, key seeded at 171. Obfuscation, not encryption."""
    key, out = 171, bytearray()
    for b in payload:
        key ^= b
        out.append(key)
    return bytes(out)


def decrypt(payload: bytes) -> bytes:
    key, out = 171, bytearray()
    for c in payload:
        out.append(c ^ key)
        key = c
    return bytes(out)


@dataclass
class Outlet:
    index: int
    child_id: str
    alias: str
    on: bool

    @property
    def guarded(self) -> bool:
        return any(g in self.alias.lower() for g in GUARDED)


@dataclass
class Device:
    ip: str
    model: str
    alias: str
    mac: str
    sw_ver: str
    relay_on: bool | None = None          # single-outlet devices only
    outlets: list[Outlet] = field(default_factory=list)

    @property
    def is_strip(self) -> bool:
        return bool(self.outlets)


def _parse(ip: str, info: dict) -> Device:
    dev = Device(ip=ip, model=info.get("model", "?"),
                 alias=info.get("alias", "?"), mac=info.get("mac", "?"),
                 sw_ver=info.get("sw_ver", "?"))
    for i, ch in enumerate(info.get("children", [])):
        dev.outlets.append(Outlet(index=i, child_id=ch.get("id", ""),
                                  alias=ch.get("alias", f"outlet{i}"),
                                  on=bool(ch.get("state"))))
    if not dev.outlets and "relay_state" in info:
        dev.relay_on = bool(info["relay_state"])
    return dev


def query(ip: str, obj: dict, timeout: float = CONTROL_TIMEOUT_S) -> dict:
    """One TCP command/response. Raises BenchError on anything unusable."""
    raw = json.dumps(obj).encode()
    try:
        sock = socket.create_connection((ip, PORT), timeout=timeout)
    except OSError as exc:
        raise BenchError(f"cannot reach {ip}:{PORT} ({exc})",
                         "is it powered and on the LAN? try: "
                         "python -m hardware.bench.kasa list") from exc
    try:
        sock.sendall(struct.pack(">I", len(raw)) + encrypt(raw))
        head = _recv_exactly(sock, 4, ip)
        want = struct.unpack(">I", head)[0]
        body = _recv_exactly(sock, want, ip)
    finally:
        sock.close()
    try:
        return json.loads(decrypt(body))
    except (ValueError, UnicodeDecodeError) as exc:
        raise BenchError(f"{ip} sent something that is not our protocol",
                         "a newer firmware may have moved to KLAP, which "
                         "needs account credentials; see plan 716.2") from exc


def _recv_exactly(sock: socket.socket, n: int, ip: str) -> bytes:
    buf = b""
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except OSError as exc:
            raise BenchError(f"{ip} dropped the connection mid-reply ({exc})",
                             "retry; if it persists, power-cycle the strip"
                             ) from exc
        if not chunk:
            raise BenchError(f"{ip} closed after {len(buf)} of {n} bytes",
                             "retry; if it persists, power-cycle the strip")
        buf += chunk
    return buf


def discover(timeout: float = DISCOVERY_TIMEOUT_S) -> list[Device]:
    """Every Kasa device that answers a udp/9999 broadcast.

    No length prefix here — that is TCP-only, and sending one makes every
    device ignore you silently.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(timeout)
    found: dict[str, Device] = {}
    try:
        sock.sendto(encrypt(b'{"system":{"get_sysinfo":{}}}'),
                    ("255.255.255.255", PORT))
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                break
            except OSError:
                break
            try:
                info = json.loads(decrypt(data))["system"]["get_sysinfo"]
            except (ValueError, KeyError, UnicodeDecodeError):
                continue          # something else on 9999; not ours
            found[addr[0]] = _parse(addr[0], info)
    finally:
        sock.close()
    return sorted(found.values(), key=lambda d: d.ip)


def fetch(ip: str) -> Device:
    """Current state of one device, by address."""
    reply = query(ip, {"system": {"get_sysinfo": {}}})
    try:
        return _parse(ip, reply["system"]["get_sysinfo"])
    except KeyError as exc:
        raise BenchError(f"{ip} gave no sysinfo", f"got: {reply}") from exc


def find_outlet(dev: Device, name: str) -> Outlet:
    """Resolve an outlet by alias (case-insensitive) or by index."""
    if not dev.is_strip:
        raise BenchError(f"{dev.model} at {dev.ip} has no named outlets",
                         "it is a single-relay device; omit the outlet name")
    lowered = name.strip().lower()
    hits = [o for o in dev.outlets if o.alias.lower() == lowered]
    if not hits and lowered.isdigit():
        hits = [o for o in dev.outlets if o.index == int(lowered)]
    if not hits:
        known = ", ".join(f"{o.index}:{o.alias!r}" for o in dev.outlets)
        raise BenchError(f"no outlet {name!r} on {dev.alias}",
                         f"outlets are: {known}")
    if len(hits) > 1:
        raise BenchError(f"{name!r} matches {len(hits)} outlets on "
                         f"{dev.alias}", "rename one, or use the index")
    return hits[0]


def arm_rest_check(cal_path: str = "calibration.json",
                   port: str | None = None) -> tuple[bool, str]:
    """Is every calibrated joint within REST_TOL_TICKS of its rest pose?

    Returns (safe_to_depower, human explanation). Never raises — an
    unreadable bus is a "no", not a crash, because the whole point is to
    answer a yes/no question about hardware that may be absent.

    The servo SDK is imported HERE rather than at module scope so that
    switching a light does not drag the Feetech stack into this tool's
    import graph — the same separation campreview keeps for the camera
    tools (see hardware/errors.py, which exists for it).
    """
    try:
        from .bus import FeetechBus
        from .calibrate import REST_TOL_TICKS, load_calibration
    except Exception as exc:                    # SDK missing, wrong platform
        return False, f"cannot load the servo stack to check the arm ({exc})"
    try:
        cals = load_calibration(Path(cal_path))
    except Exception as exc:
        return False, f"cannot read {cal_path} ({exc})"
    if not cals:
        return False, f"{cal_path} has no calibrated joints"
    try:
        with FeetechBus(port) as bus:
            off = []
            for jid, cal in sorted(cals.items()):
                pos = bus.read_position(jid)
                delta = pos - cal.rest
                if abs(delta) > REST_TOL_TICKS:
                    off.append(f"{cal.name}(id{jid}) {delta:+d} ticks")
    except Exception as exc:
        # Ambiguous on purpose: this is equally "the arm is already
        # unpowered" and "the bus is broken". Both are unknown state, and
        # unknown state is not permission to cut power.
        return False, (f"could not read the arm ({exc}) — it may already be "
                       f"unpowered, or the bus may be faulty; either way "
                       f"this cannot confirm the pose")
    if off:
        return False, ("not at rest: " + ", ".join(off) +
                       f" (tolerance {REST_TOL_TICKS} ticks)")
    return True, f"all {len(cals)} joints within {REST_TOL_TICKS} ticks of rest"


def switch(dev: Device, outlet: Outlet | None, on: bool,
           confirm: str | None = None, force: bool = False,
           rest_check=arm_rest_check) -> None:
    """Turn an outlet (or a single-relay device) on or off.

    Both directions are gated for a guarded outlet, differently, and
    --force overrides either. See the module docstring for why `off` is
    gated at all — these servos have no brakes.

    `rest_check` is injectable so the gate can be tested without an arm.
    """
    current = outlet.on if outlet is not None else dev.relay_on
    if current is not None and current == on:
        return                      # already there; nothing to be unsafe about

    if outlet is not None and outlet.guarded and not force:
        if on:
            if (confirm or "").strip().lower() != outlet.alias.lower():
                raise BenchError(
                    f"outlet {outlet.alias!r} is guarded and will not be "
                    f"switched ON without confirmation",
                    f"it energises hardware that moves. If you mean it: "
                    f"--confirm {outlet.alias}")
        else:
            safe, why = rest_check()
            if not safe:
                raise BenchError(
                    f"refusing to cut power to {outlet.alias!r}: {why}",
                    "these servos have no brakes, so an arm that is not "
                    "folded will FALL when de-energised. Move it to rest "
                    "first. If this is an emergency and a fall is the "
                    "lesser harm, use --force.")
    body = {"system": {"set_relay_state": {"state": 1 if on else 0}}}
    if outlet is not None:
        body = {"context": {"child_ids": [outlet.child_id]}, **body}
    reply = query(dev.ip, body)
    err = reply.get("system", {}).get("set_relay_state", {}).get("err_code")
    if err:
        raise BenchError(f"{dev.ip} refused the switch (err_code {err})",
                         f"full reply: {reply}")


# --------------------------------------------------------------------


def _show(dev: Device) -> None:
    head = f"{dev.ip:<15} {dev.model:<10} {dev.alias!r}"
    if dev.is_strip:
        print(f"{head}  ({len(dev.outlets)} outlets, fw {dev.sw_ver})")
        for o in dev.outlets:
            flag = "  [guarded]" if o.guarded else ""
            print(f"    [{o.index}] {o.alias:<12} "
                  f"{'ON ' if o.on else 'off'}{flag}")
    else:
        state = "?" if dev.relay_on is None else ("ON" if dev.relay_on
                                                  else "off")
        print(f"{head}  {state}  (fw {dev.sw_ver})")


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.kasa",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("list", "show", "on", "off",
                                           "selftest"))
    parser.add_argument("target", nargs="?", help="device IP")
    parser.add_argument("outlet", nargs="?",
                        help="outlet alias or index (strips only)")
    parser.add_argument("--confirm", default=None, metavar="ALIAS",
                        help="retype a guarded outlet's alias to switch it ON")
    parser.add_argument("--force", action="store_true",
                        help="override BOTH guards. For a real emergency: cuts power even if the arm is not folded and will fall")
    args = parser.parse_args()

    if args.action == "selftest":
        return selftest()

    if args.action == "list":
        devices = discover()
        if not devices:
            raise BenchError("no Kasa devices answered on this LAN",
                             "are you on the same subnet? a VLAN or client "
                             "isolation will block the broadcast")
        for dev in devices:
            _show(dev)
        return 0

    if not args.target:
        raise BenchError(f"{args.action} needs a device address",
                         "run `kasa list` to see them")
    dev = fetch(args.target)

    if args.action == "show":
        _show(dev)
        return 0

    outlet = find_outlet(dev, args.outlet) if args.outlet else None
    if dev.is_strip and outlet is None:
        raise BenchError(f"{dev.alias} has {len(dev.outlets)} outlets; "
                         f"say which one",
                         "switching a whole strip at once is not offered - "
                         "it is too easy to take down something unrelated")
    want = args.action == "on"
    switch(dev, outlet, want, args.confirm, args.force)
    after = fetch(args.target)
    # Read back rather than trusting the ack: the point of this tool is
    # knowing the state of the bench, and a command accepted is not a
    # relay moved.
    _show(after)
    return 0


def selftest() -> int:
    """Protocol and guard logic, no device required.

    The live path is exercised separately against the real strip; what is
    checked here is the part that silently corrupts rather than failing
    loudly — the codec, the framing, and the direction-asymmetric guard.
    """
    fails: list[str] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        print(f"  {'ok  ' if ok else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not ok:
            fails.append(name)

    print("the codec")
    msg = b'{"system":{"get_sysinfo":{}}}'
    check("encrypt/decrypt round-trips", decrypt(encrypt(msg)) == msg)
    check("...and the ciphertext is not the plaintext, i.e. it ran at all",
          encrypt(msg) != msg)
    # Pinned against a byte sequence computed from the documented seed. If
    # the seed or the chaining direction is ever "tidied", this catches it
    # where a round-trip test would not — a wrong-but-symmetric codec
    # round-trips perfectly and talks to nothing.
    check("first bytes match the key-171 autokey exactly",
          encrypt(b"{}")[:2] == bytes([171 ^ 0x7B, (171 ^ 0x7B) ^ 0x7D]),
          encrypt(b"{}")[:2].hex())
    check("an empty payload does not explode", encrypt(b"") == b"")

    print("\nwhich outlets are guarded")
    arm_off = Outlet(0, "id0", "Arm", False)
    arm_on = Outlet(0, "id0", "Arm", True)
    light = Outlet(1, "id1", "Light", False)
    check("an outlet named Arm is guarded", arm_off.guarded)
    check("...and one named Light is not", not light.guarded)
    check("matching is case-insensitive and substring, so 'arm psu' counts",
          Outlet(2, "x", "ARM PSU", False).guarded)
    dev = Device("0.0.0.0", "KP303(US)", "strip", "mac", "fw",
                 outlets=[arm_off, light])
    at_rest = lambda: (True, "all joints at rest")          # noqa: E731
    extended = lambda: (False, "not at rest: elbow(id3) +812 ticks")  # noqa: E731

    def refuses(label, fn, expect: str):
        """Assert a refusal AND that it says something useful.

        Searches the hint as well as the message: for a safety gate the
        hint is where the operator is told what to do instead, so an
        unhelpful refusal should fail this test too.
        """
        try:
            fn()
            check(label, False, "it went through")
        except BenchError as exc:
            said = f"{exc} {exc.hint or ''}"
            check(label, expect in said, said[:75])

    print("\nthe ON gate — energising something that moves")
    refuses("ON without --confirm is refused",
            lambda: switch(dev, arm_off, True, rest_check=at_rest), "guarded")
    refuses("...and the WRONG alias does not satisfy it",
            lambda: switch(dev, arm_off, True, confirm="Light",
                           rest_check=at_rest), "guarded")

    print("\nthe OFF gate — these servos have no brakes, so a raised arm "
          "FALLS")
    refuses("OFF is refused while the arm is NOT at rest, and says which "
            "joint and by how much",
            lambda: switch(dev, arm_on, False, rest_check=extended),
            "elbow(id3) +812")
    refuses("...and an UNREADABLE arm is refused too - unknown state is not "
            "permission to cut power",
            lambda: switch(dev, arm_on, False,
                           rest_check=lambda: (False, "could not read the "
                                               "arm (no bus)")),
            "could not read")
    # The acceptance half. No socket exists, so reaching the network at all
    # proves the gate let it through: a BenchError that is about
    # reachability rather than the guard is the pass.
    for label, kwargs in (
            ("OFF is ALLOWED once every joint is within tolerance of rest",
             {"rest_check": at_rest}),
            ("...and --force overrides the rest check, because a real "
             "e-stop must not be blocked by a fall being likely",
             {"rest_check": extended, "force": True})):
        try:
            switch(dev, arm_on, False, **kwargs)
            check(label, False, "expected it to reach the network")
        except BenchError as exc:
            check(label, "cannot reach" in str(exc), str(exc)[:60])
    refuses("--force is NOT implied by anything else: without it, an "
            "extended arm still refuses",
            lambda: switch(dev, arm_on, False, rest_check=extended),
            "no brakes")

    print("\nno-op switching skips both gates")
    # Asking for the state it is already in changes nothing, so there is
    # nothing to be unsafe about — and it must not demand --confirm or go
    # near the servo bus to tell you so.
    def boom():
        raise AssertionError("the rest check must not run for a no-op")
    switch(dev, arm_on, True, rest_check=boom)
    check("a guarded outlet already ON accepts `on` silently", True)
    switch(dev, arm_off, False, rest_check=boom)
    check("...and one already off accepts `off` without touching the bus",
          True)

    print("\noutlet lookup")
    check("by alias, case-insensitively", find_outlet(dev, "light") is light)
    check("by index", find_outlet(dev, "1") is light)
    for bad, why in (("nope", "an unknown alias is refused"),
                     ("9", "an out-of-range index is refused")):
        try:
            find_outlet(dev, bad)
            check(why, False)
        except BenchError:
            check(why, True)
    solo = Device("0.0.0.0", "HS200(US)", "switch", "mac", "fw",
                  relay_on=False)
    try:
        find_outlet(solo, "Arm")
        check("a single-relay device refuses an outlet name", False)
    except BenchError:
        check("a single-relay device refuses an outlet name", True)

    print()
    if fails:
        print(f"kasa FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("kasa OK")
    return 0



def main() -> int:
    try:
        return run()
    except BenchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if exc.hint:
            print(f"hint:  {exc.hint}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print()
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
