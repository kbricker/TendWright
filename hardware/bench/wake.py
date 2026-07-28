"""wake — bring cell1 up from the desk, and wait until it is really usable.

Kyle, 2026-07-28: *"I want to sort out all this stuff and get it to wake
on command so we dont need to keep the system up all the time."*

    uv run python -m hardware.bench.wake cell1
    uv run python -m hardware.bench.wake status cell1
    uv run python -m hardware.bench.wake list
    uv run python -m hardware.bench.wake selftest

WHY A TOOL AND NOT A ONE-LINER. `wakeonlan cell1` is one line and tells
you nothing. The three things that actually go wrong with wake-on-LAN
are all invisible to a fire-and-forget packet:

  1. The packet went to the wrong broadcast address and nothing heard it.
  2. The packet was heard and the BIOS ignored it — deep standby or fast
     boot silently disables the NIC's standby power. This is the classic
     failure and it is indistinguishable from "the hardware can't do it"
     unless you know to look.
  3. The box powered on but is not usable yet, because the kernel
     answers ping long before sshd is listening.

So this sends, then WAITS, then reports how long it took. A wake that
reports "up in 34s" is a wake you can put in a script. A wake that
reports nothing is a wake you babysit with a second terminal.

WHY IT POLLS TCP:22 AND NOT PING. Ping is answered by the kernel's
network stack, which comes up well before userspace finishes. Polling
the SSH port asks the question we actually care about — "can I work on
this box now?" — instead of a proxy for it. The cost is that a machine
with sshd disabled looks down; that is the correct answer for our
purposes, since every use of cell1 is over SSH.

WHY MAGIC PACKETS NEED NO ARP. The one genuinely nice property here:
the payload is addressed by MAC and delivered as a layer-2 broadcast,
so it works even though a long-powered-off host has aged out of the ARP
cache entirely (cell1's entry currently reads 00-00-00-00-00-00). This
is why the tool needs the MAC in `hosts.json` and cannot derive it from
the IP — there is nothing alive at that IP to ask.

REFUSES RATHER THAN GUESSES. No MAC recorded → a BenchError naming the
exact command to get it. Same house rule as `bus.resolve_port`: a tool
that guesses at a physical address is a tool that eventually powers on
something that is not yours.
"""

from __future__ import annotations

import argparse
import json
import re
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from hardware.errors import BenchError, make_run_tool

run_tool = make_run_tool("not a serial tool — this hint should never print")

REGISTRY_DEFAULT = "hosts.json"

# Ports 9 (discard) and 7 (echo) are both conventional for WoL, and NICs
# differ in which they watch. The packet is 102 bytes; sending to both
# costs nothing measurable and removes one variable from a failure that
# is already hard to observe.
WOL_PORTS = (9, 7)

# Repeats per address/port pair. A magic packet is UDP to a broadcast
# address, so it is unacknowledged and may simply be dropped by a busy
# switch. Three is enough to make loss unlikely without turning a wake
# into a broadcast storm.
WOL_REPEATS = 3

# How long to wait for sshd after sending, in seconds. A UM350 cold boot
# to a listening sshd is roughly 30-45s; 120 leaves room for a fsck or a
# slow DHCP lease without waiting forever on a box that never woke.
BOOT_TIMEOUT_S = 120

# Seconds between reachability probes while waiting. Fast enough that the
# reported time is meaningful, slow enough not to spin.
POLL_INTERVAL_S = 2.0

# Per-probe TCP connect timeout. Must stay below POLL_INTERVAL_S or the
# poll loop's real period drifts above what the operator was told.
PROBE_TIMEOUT_S = 1.5

SSH_PORT = 22

_MAC_SEPARATORS = re.compile(r"[:\-.\s]")


@dataclass(frozen=True)
class Host:
    """One wakeable machine from the registry."""

    name: str
    address: str          # IP or DNS name, used for the reachability probe
    mac: str | None       # normalised aa:bb:cc:dd:ee:ff, or None if unknown
    broadcast: str        # where the magic packet is sent
    port: int             # port the reachability probe connects to
    note: str


def normalise_mac(raw: str) -> str:
    """Accept the three forms people paste, return one canonical form.

    `ip link` prints aa:bb:cc:dd:ee:ff, Windows prints AA-BB-CC-DD-EE-FF,
    and router lease tables sometimes print aabbccddeeff. All three mean
    the same six bytes, and rejecting two of them would be a paper cut
    every single time this file is edited by hand.
    """
    digits = _MAC_SEPARATORS.sub("", raw).lower()
    if len(digits) != 12 or not all(c in "0123456789abcdef" for c in digits):
        raise BenchError(
            f"not a MAC address: {raw!r}",
            "expected six hex bytes, e.g. 1c:83:41:2f:9b:04 (as printed "
            "by `ip link show enp4s0`)",
        )
    return ":".join(digits[i:i + 2] for i in range(0, 12, 2))


def magic_packet(mac: str) -> bytes:
    """The wire format: six 0xFF bytes then the target MAC sixteen times.

    102 bytes total. The repetition is the whole trick — it is a pattern
    a sleeping NIC can recognise in hardware without a protocol stack.
    """
    raw = bytes.fromhex(normalise_mac(mac).replace(":", ""))
    return b"\xff" * 6 + raw * 16


def default_broadcast(address: str) -> str:
    """Subnet-directed broadcast for a dotted-quad, assuming /24.

    255.255.255.255 is the obvious alternative and is worse: some
    switches and most Wi-Fi APs drop the all-networks broadcast, while
    the subnet-directed form is ordinary traffic. The /24 assumption is
    stated rather than detected — every network this project runs on is
    a home /24, and a registry entry can override `broadcast` explicitly
    when that stops being true.
    """
    parts = address.split(".")
    if len(parts) != 4 or not all(p.isdigit() for p in parts):
        raise BenchError(
            f"cannot derive a broadcast address from {address!r}",
            "set an explicit \"broadcast\" on this host in hosts.json",
        )
    return ".".join(parts[:3] + ["255"])


def load_registry(path: str | Path = REGISTRY_DEFAULT) -> list[Host]:
    """Load + strictly validate hosts.json. Order is display order."""
    path = Path(path)
    if not path.exists():
        raise BenchError(
            f"no host registry at {path}",
            "create hosts.json with an entry per wakeable machine",
        )
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise BenchError(f"{path} is not valid JSON ({exc})",
                         "check for a trailing comma") from exc

    entries = doc.get("hosts")
    if not isinstance(entries, list) or not entries:
        raise BenchError(f"{path} has no \"hosts\" list",
                         "expected {\"hosts\": [{\"name\": ...}]}")

    hosts: list[Host] = []
    seen: set[str] = set()
    for i, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise BenchError(f"{path} host #{i} is not an object")
        name = entry.get("name")
        address = entry.get("address")
        if not name or not address:
            raise BenchError(
                f"{path} host #{i} needs both \"name\" and \"address\"")
        if name in seen:
            raise BenchError(f"{path} has two hosts named {name!r}",
                             "names are how the CLI selects a host")
        seen.add(name)

        # A null mac is a legitimate, expected state — it means "we have
        # not captured this yet". Validate it only when present, so the
        # registry can record the host before the MAC is known and the
        # refusal happens at wake time with a useful hint.
        mac = entry.get("mac")
        hosts.append(Host(
            name=name,
            address=address,
            mac=normalise_mac(mac) if mac else None,
            broadcast=entry.get("broadcast") or default_broadcast(address),
            port=int(entry.get("port", SSH_PORT)),
            note=entry.get("note", ""),
        ))
    return hosts


def find_host(name: str, hosts: list[Host]) -> Host:
    for host in hosts:
        if host.name == name:
            return host
    known = ", ".join(h.name for h in hosts) or "(none)"
    raise BenchError(f"no host named {name!r} in the registry",
                     f"known hosts: {known}")


def is_up(host: Host, timeout: float = PROBE_TIMEOUT_S) -> bool:
    """True if something accepts a TCP connection on the host's port."""
    try:
        with socket.create_connection((host.address, host.port), timeout):
            return True
    except OSError:
        return False


def send_magic(host: Host) -> int:
    """Broadcast the magic packet. Returns the number of datagrams sent."""
    if not host.mac:
        raise BenchError(
            f"no MAC address recorded for {host.name}",
            "magic packets are addressed by MAC, not IP, and a "
            "powered-off host cannot be asked. Get it at the machine with "
            "`ip link show enp4s0`, or from the router's DHCP lease "
            f"table, then put it in {REGISTRY_DEFAULT}",
        )
    packet = magic_packet(host.mac)
    sent = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        for port in WOL_PORTS:
            for _ in range(WOL_REPEATS):
                sock.sendto(packet, (host.broadcast, port))
                sent += 1
    return sent


def wake(host: Host, timeout: float = BOOT_TIMEOUT_S) -> int:
    """Send, then wait for the host to become usable. 0 on success."""
    if is_up(host):
        print(f"{host.name} is already up ({host.address}:{host.port})")
        return 0

    sent = send_magic(host)
    print(f"sent {sent} magic packets for {host.mac} "
          f"to {host.broadcast} on ports {', '.join(map(str, WOL_PORTS))}")
    print(f"waiting up to {timeout:.0f}s for {host.address}:{host.port}...")

    started = time.monotonic()
    while True:
        elapsed = time.monotonic() - started
        if is_up(host):
            print(f"{host.name} is up after {elapsed:.0f}s")
            return 0
        if elapsed >= timeout:
            break
        # Subtract the probe's own cost so the loop period is the stated
        # interval rather than interval + however long the probe blocked.
        time.sleep(max(0.0, POLL_INTERVAL_S - PROBE_TIMEOUT_S))

    raise BenchError(
        f"{host.name} did not come up within {timeout:.0f}s",
        "the packet may have been heard and ignored: deep standby (ErP "
        "or equivalent) and fast boot both cut the NIC's standby power "
        "and defeat wake-on-LAN silently. Check `sudo ethtool "
        "<iface> | grep Wake-on` reads 'g' at the machine",
    )


def cmd_list(hosts: list[Host]) -> int:
    print(f"{'name':<10}  {'address':<16}  {'mac':<17}  {'broadcast':<16}  note")
    for h in hosts:
        mac = h.mac or "(not captured)"
        print(f"{h.name:<10}  {h.address:<16}  {mac:<17}  "
              f"{h.broadcast:<16}  {h.note}")
    missing = [h.name for h in hosts if not h.mac]
    if missing:
        print(f"\n{len(missing)} host(s) cannot be woken until a MAC is "
              f"recorded: {', '.join(missing)}")
    return 0


def cmd_status(host: Host) -> int:
    up = is_up(host)
    print(f"{host.name} ({host.address}:{host.port}) is "
          f"{'up' if up else 'down'}")
    return 0 if up else 1


def selftest() -> int:
    """Everything checkable without a network or a powered-off machine."""
    failures: list[str] = []

    def check(name: str, cond: bool, detail: str = "") -> None:
        print(f"  {'ok  ' if cond else 'FAIL'}  {name}"
              f"{'  — ' + detail if detail and not cond else ''}")
        if not cond:
            failures.append(name)

    print("wake selftest")

    # -- MAC normalisation -------------------------------------------------
    forms = ["1c:83:41:2f:9b:04", "1C-83-41-2F-9B-04", "1c83412f9b04"]
    normed = {normalise_mac(f) for f in forms}
    check("the three MAC spellings normalise to one form",
          normed == {"1c:83:41:2f:9b:04"}, str(normed))

    for bad in ["", "1c:83:41:2f:9b", "1c:83:41:2f:9b:04:05", "zz:83:41:2f:9b:04"]:
        try:
            normalise_mac(bad)
            check(f"rejects malformed MAC {bad!r}", False, "it was accepted")
        except BenchError:
            check(f"rejects malformed MAC {bad!r}", True)

    # -- packet shape ------------------------------------------------------
    pkt = magic_packet("1c:83:41:2f:9b:04")
    check("magic packet is 102 bytes", len(pkt) == 102, f"got {len(pkt)}")
    check("magic packet starts with six 0xFF", pkt[:6] == b"\xff" * 6)
    body = bytes.fromhex("1c83412f9b04")
    check("magic packet repeats the MAC sixteen times",
          pkt[6:] == body * 16)
    # A packet built for one MAC must not wake another. Cheap to check and
    # the failure mode (an off-by-one in the repeat) is otherwise silent.
    check("a different MAC produces a different packet",
          magic_packet("1c:83:41:2f:9b:05") != pkt)

    # -- broadcast derivation ---------------------------------------------
    check("derives a /24 broadcast",
          default_broadcast("192.168.86.202") == "192.168.86.255",
          default_broadcast("192.168.86.202"))
    try:
        default_broadcast("cell1.local")
        check("refuses to derive a broadcast from a hostname", False,
              "it returned something")
    except BenchError:
        check("refuses to derive a broadcast from a hostname", True)

    # -- the refusal that matters right now --------------------------------
    # cell1's MAC is genuinely unknown today, so this is not a hypothetical
    # path: it is the tool's current behaviour and it has to be helpful.
    no_mac = Host("nomac", "192.168.86.202", None, "192.168.86.255", 22, "")
    try:
        send_magic(no_mac)
        check("refuses to wake a host with no MAC", False, "it sent one")
    except BenchError as exc:
        check("refuses to wake a host with no MAC", True)
        check("the refusal names how to get the MAC",
              "ip link" in (exc.hint or ""), exc.hint or "(no hint)")

    # -- registry validation ----------------------------------------------
    try:
        hosts = load_registry()
        check(f"{REGISTRY_DEFAULT} loads", True)
        check(f"{REGISTRY_DEFAULT} has a cell1 entry",
              any(h.name == "cell1" for h in hosts),
              ", ".join(h.name for h in hosts))
    except BenchError as exc:
        check(f"{REGISTRY_DEFAULT} loads", False, str(exc))

    print(f"\n{'FAILED: ' + ', '.join(failures) if failures else 'all passed'}")
    return 1 if failures else 0


def run() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.wake",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", nargs="?", default=None,
                        help="host name to wake, or: status, list, selftest")
    parser.add_argument("target", nargs="?", default=None,
                        help="host name (for status)")
    parser.add_argument("--hosts", default=REGISTRY_DEFAULT,
                        help=f"host registry file (default {REGISTRY_DEFAULT})")
    parser.add_argument("--timeout", type=float, default=BOOT_TIMEOUT_S,
                        help=f"seconds to wait for boot (default "
                             f"{BOOT_TIMEOUT_S:.0f})")
    args = parser.parse_args()

    if args.command == "selftest":
        return selftest()
    if args.command is None:
        parser.print_help()
        return 1

    hosts = load_registry(args.hosts)

    if args.command == "list":
        return cmd_list(hosts)
    if args.command == "status":
        if not args.target:
            raise BenchError("status needs a host name",
                             "e.g. `wake status cell1`")
        return cmd_status(find_host(args.target, hosts))

    return wake(find_host(args.command, hosts), args.timeout)


def main() -> int:
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
