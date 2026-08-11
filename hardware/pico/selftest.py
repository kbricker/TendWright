"""selftest — port resolution for the nest bridge, with no hardware attached.

    uv run python -m hardware.pico.selftest

Plan #848. The bug this guards is only reachable with TWO RP2 boards
plugged in, which is exactly the configuration nobody has while writing the
fix. Stubbing the port list makes every branch reachable at the desk; the
two-board run at the bench then confirms the real thing rather than
discovering it.

Plain asserts and a __main__ block — the repo has no test framework and
adding one would be a new dependency.
"""

from __future__ import annotations

import sys

from hardware.errors import BenchError
from hardware.pico import reader


class FakePort:
    def __init__(self, device, vid=reader.PICO_VID):
        self.device = device
        self.vid = vid


class Stub:
    def __init__(self, ports, nest_ports=()):
        self.ports = ports
        self.nest_ports = set(nest_ports)
        self.probed = []

    def comports(self):
        return self.ports

    def probe(self, port):
        self.probed.append(port)
        return port in self.nest_ports


def with_stub(stub, fn):
    real_ports, real_probe, real_platform = (
        reader.list_ports, reader._probe, sys.platform)
    reader.list_ports = stub
    reader._probe = stub.probe
    try:
        return fn()
    finally:
        reader.list_ports = real_ports
        reader._probe = real_probe


def resolves_to(stub, expected, port=None):
    # Catches BenchError rather than letting it escape. An uncaught raise here
    # would abort the whole run at the first regression and hide every check
    # after it — which is what happened the first time this file was mutation-
    # tested: the suite died instead of reporting one failure.
    try:
        return with_stub(stub, lambda: reader.resolve_pico_port(port)) == expected
    except BenchError:
        return False


def raises(stub, fragment, port=None):
    try:
        with_stub(stub, lambda: reader.resolve_pico_port(port))
    except BenchError as exc:
        return fragment.lower() in str(exc).lower()
    return False


def check(label, condition, detail=""):
    if condition:
        print(f"  ok    {label}")
        return 0
    print(f"  FAIL  {label}" + (f" — {detail}" if detail else ""))
    return 1


def main() -> int:
    fails = 0
    print("nest bridge port resolution (#848)\n")

    print("identification")
    for doc, want, why in (
        ({"hello": "tendwright-pico", "version": 1}, True, "its hello"),
        ({"nest": True, "seq": 3}, True, "a nest sample"),
        ({"hello": "tendwright-conveyor"}, False, "the conveyor's hello"),
        ({"motors": {}, "seq": 3}, False, "a conveyor state line"),
        ({"ack": "set", "applied": {}}, False, "a conveyor ack"),
        ({}, False, "an empty object"),
    ):
        got = reader.looks_like_nest_bridge(doc)
        fails += check(f"{'accepts' if want else 'rejects'} {why}", got == want,
                       str(doc))

    print("\nresolution")
    s = Stub([])
    fails += check("an explicit port wins without touching the port list",
                   resolves_to(s, "COM9", port="COM9") and not s.probed)

    fails += check("no candidates is a clear error",
                   raises(Stub([]), "no Pico found"))

    s = Stub([FakePort("COM3")])
    fails += check("one candidate resolves", resolves_to(s, "COM3"))
    fails += check("...and is NOT probed (fast path preserved)",
                   not s.probed, str(s.probed))

    s = Stub([FakePort("COM3"), FakePort("COM7")], nest_ports=["COM7"])
    fails += check("two candidates, one is the nest bridge — THE BUG #848 FIXES",
                   resolves_to(s, "COM7"))
    fails += check("...and both were probed to find out",
                   sorted(s.probed) == ["COM3", "COM7"], str(s.probed))

    s = Stub([FakePort("COM3"), FakePort("COM7"), FakePort("COM11")],
             nest_ports=["COM11"])
    fails += check("three candidates still finds the one", resolves_to(s, "COM11"))

    s = Stub([FakePort("COM3"), FakePort("COM7")])
    fails += check("two candidates, neither is ours — says so, and names them",
                   raises(s, "none running the nest bridge"))

    s = Stub([FakePort("COM3"), FakePort("COM7")], nest_ports=["COM3", "COM7"])
    fails += check("two nest bridges is ambiguous, not a silent coin-flip",
                   raises(s, "multiple nest bridges"))

    s = Stub([FakePort("COM3"), FakePort("COM7", vid=0x1234)],
             nest_ports=["COM3"])
    fails += check("a non-RP2 device is filtered before probing",
                   resolves_to(s, "COM3"))
    fails += check("...so it is never opened", s.probed == [], str(s.probed))

    print()
    if fails:
        print(f"{fails} check(s) FAILED")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
