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

WHY THE ON DIRECTION IS GATED AND OFF IS NOT
--------------------------------------------
This will eventually switch the arm's 12 V supply. Those two directions are
not symmetric and must not be treated as one "toggle":

  OFF is a SAFETY action. Cutting power to a misbehaving arm is the thing
  you want to happen instantly, from a script, half-asleep, with no
  ceremony. Gating it would be actively harmful — an e-stop you have to
  confirm is not an e-stop.

  ON is the dangerous direction. It energises servos that may be holding a
  pose, in a cell that may have nobody in front of it.

So outlets named in GUARDED are refused an `on` unless the caller retypes
the alias with --confirm. Same shape as camserve's /debug/memory?trim=<pid>:
not security, a speed bump against reflexes and stray automation.

Kyle 2026-07-29: nothing is plugged into the KP303 yet, which is exactly
why the switching path was built and exercised NOW — once the arm is on it,
testing the switch is no longer free.

Usage: kasa {list|show|on|off} [target] [outlet] [--confirm ALIAS]
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from dataclasses import dataclass, field

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


def switch(dev: Device, outlet: Outlet | None, on: bool,
           confirm: str | None = None) -> None:
    """Turn an outlet (or a single-relay device) on or off.

    The guard applies ONLY to `on`. See the module docstring: an off that
    needs ceremony is not a safety control.
    """
    if on and outlet is not None and outlet.guarded:
        if (confirm or "").strip().lower() != outlet.alias.lower():
            raise BenchError(
                f"outlet {outlet.alias!r} is guarded and will not be "
                f"switched ON without confirmation",
                f"it can energise hardware that moves. If you mean it: "
                f"--confirm {outlet.alias}   (turning it OFF never needs "
                f"this)")
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
    switch(dev, outlet, want, args.confirm)
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

    print("\nthe guard, which is DIRECTIONAL")
    arm = Outlet(0, "id0", "Arm", False)
    light = Outlet(1, "id1", "Light", False)
    check("an outlet named Arm is guarded", arm.guarded)
    check("...and one named Light is not", not light.guarded)
    check("matching is case-insensitive and substring, so 'arm psu' counts",
          Outlet(2, "x", "ARM PSU", False).guarded)
    dev = Device("0.0.0.0", "KP303(US)", "strip", "mac", "fw",
                 outlets=[arm, light])
    try:
        switch(dev, arm, True)
        check("switching a guarded outlet ON without --confirm is refused",
              False)
    except BenchError:
        check("switching a guarded outlet ON without --confirm is refused",
              True)
    try:
        switch(dev, arm, True, confirm="Light")
        check("...and the WRONG alias does not satisfy it", False)
    except BenchError:
        check("...and the WRONG alias does not satisfy it", True)
    # The half that matters most, and the one a naive "gate everything"
    # design gets wrong: cutting power must never need ceremony.
    try:
        switch(dev, arm, False)
        check("turning a guarded outlet OFF is NEVER gated - an e-stop you "
              "have to confirm is not an e-stop", False, "no socket, so a "
              "BenchError about reachability is the expected pass")
    except BenchError as exc:
        check("turning a guarded outlet OFF is NEVER gated - an e-stop you "
              "have to confirm is not an e-stop",
              "guarded" not in str(exc), str(exc)[:60])

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
