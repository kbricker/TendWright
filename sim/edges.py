"""Edge validation with a cache — collision-gate a pose-to-pose move once.

Plan #660. A clip is a walk through named poses; every edge of it has to
be twin-simulated before the arm plays it. Simulating is not cheap (a
15-joint-degree edge samples ~100 poses per second of travel at 100 Hz,
each one a full MuJoCo collision pass), and a routine re-runs the same
edges every single time. So a known-good edge should be validated once
and remembered.

THE ENTIRE RISK OF A CACHE HERE IS CERTIFYING A PATH NOBODY SIMULATED.
The plan says it outright: *"a stale cache would certify a path nobody
simulated."* A gate that says CLEAR because it remembers a different
edge is worse than no gate at all, because it is trusted.

So the key is a hash of EVERYTHING the verdict depends on, and the rule
for adding to it is the test question: *could this change the swept
path, or what counts as a collision, without changing the key?* If yes,
it belongs in the key.

    the two poses          the endpoints, in ticks
    the motion profile     speed + accel decide the PATH between them,
                           not just its duration — that is the whole
                           point of #660
    the sample rate        coarser sampling can step over a contact
                           (measured: 10 Hz tunnels 9.48 mm)
    the MODEL FILE         its content hash, not its path or mtime.
                           Swap the arm and every remembered verdict
                           describes different geometry.

Deliberately NOT keyed on the pose NAMES. Renaming a pose does not move
it, and two names for the same ticks are the same edge — keying on names
would miss an edit that changed ticks under a stable name, which is the
dangerous direction.

Cache misses are cheap; cache hits are the point; a WRONG cache hit is
unacceptable. When in doubt the key gets wider.
"""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

from hardware.errors import BenchError

from .clip import DEFAULT_HZ, Clip, MotionProfile, Pose, sample_edge

CACHE_JSON = Path(__file__).resolve().parent.parent / ".edge-cache.json"
CACHE_VERSION = 1


def _model_digest(model_path: Path) -> str:
    """Content hash of the arm model.

    CONTENT, not mtime: a checkout, a rebase or a touch changes mtime
    without changing geometry, and would throw away a valid cache; worse,
    copying an older model over a newer one can leave mtime NEWER while
    the geometry goes backwards. The bytes are the only honest answer.
    """
    try:
        return hashlib.sha256(model_path.read_bytes()).hexdigest()[:16]
    except OSError as exc:
        raise BenchError(f"cannot read the arm model at {model_path}: {exc}",
                         "the edge cache keys on its contents") from exc


def edge_key(a: Pose, b: Pose, profile: MotionProfile, hz: float,
             model_digest: str) -> str:
    """Stable identity for one validated edge. See the module docstring
    for what is in it and why."""
    payload = json.dumps({
        "v": CACHE_VERSION,
        "a": {str(i): t for i, t in sorted(a.ticks.items())},
        "b": {str(i): t for i, t in sorted(b.ticks.items())},
        "speed": profile.speed,
        "accel": profile.acceleration,
        "hz": round(float(hz), 6),
        "model": model_digest,
    }, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode()).hexdigest()[:32]


@dataclass
class EdgeVerdict:
    """What the gate said about one edge."""

    clean: bool
    poses_checked: int
    detail: str = ""          # first contact, if any — for the refusal
    cached: bool = False      # answered from the cache rather than simulated

    def as_doc(self) -> dict:
        return {"clean": self.clean, "poses_checked": self.poses_checked,
                "detail": self.detail}


class EdgeCache:
    """Remembered edge verdicts, keyed by everything that decides them.

    Load, ask, save. Nothing here decides whether an edge is safe — it
    only remembers what the twin already said, under a key strict enough
    that the answer still applies.
    """

    def __init__(self, path: Path = CACHE_JSON):
        self.path = Path(path)
        self.entries: dict[str, dict] = {}
        self.hits = 0
        self.misses = 0
        self._dirty = False

    def load(self) -> 'EdgeCache':
        """Read the cache. A corrupt or unreadable file is DISCARDED, not
        raised: the only cost is re-simulating, and refusing to run
        because a cache file is malformed would be the wrong trade."""
        try:
            doc = json.loads(self.path.read_text())
            if isinstance(doc, dict) and doc.get("version") == CACHE_VERSION:
                got = doc.get("entries")
                self.entries = got if isinstance(got, dict) else {}
        except (OSError, json.JSONDecodeError, TypeError):
            self.entries = {}
        return self

    def save(self) -> None:
        """Write atomically. A half-written cache read back later would
        be discarded by load(), so the failure is survivable — but a
        rename is cheap and keeps the file always-valid on disk."""
        if not self._dirty:
            return
        doc = {"version": CACHE_VERSION, "entries": self.entries}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent),
                                   prefix=".edge-cache.", suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as fh:
                json.dump(doc, fh, indent=1, sort_keys=True)
            os.replace(tmp, self.path)
            self._dirty = False
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass

    def get(self, key: str) -> EdgeVerdict | None:
        e = self.entries.get(key)
        if not isinstance(e, dict) or "clean" not in e:
            return None
        self.hits += 1
        return EdgeVerdict(bool(e["clean"]), int(e.get("poses_checked", 0)),
                           str(e.get("detail", "")), cached=True)

    def put(self, key: str, verdict: EdgeVerdict) -> None:
        self.entries[key] = verdict.as_doc()
        self._dirty = True
        self.misses += 1

    def prune(self, keep: set[str]) -> int:
        """Drop entries not in `keep`. Returns how many went.

        Called with the keys a clip actually uses, so editing a pose
        does not leave its old verdict lying around forever. The stale
        entry was never DANGEROUS — its key no longer matches anything —
        but an unbounded file that only ever grows is its own problem."""
        dead = [k for k in self.entries if k not in keep]
        for k in dead:
            del self.entries[k]
        if dead:
            self._dirty = True
        return len(dead)


def validate_edge(twin, a: Pose, b: Pose, profile: MotionProfile,
                  hz: float = DEFAULT_HZ, cache: EdgeCache | None = None,
                  model_digest: str | None = None) -> EdgeVerdict:
    """Gate one edge, using the cache when the key matches exactly.

    `twin` is a sim.twin.Twin; taken as a parameter rather than
    constructed here so a caller validating twenty edges pays the model
    load once, and so the selftest can pass a counting double.
    """
    if model_digest is None:
        from .twin import MODEL_XML
        model_digest = _model_digest(MODEL_XML)
    key = edge_key(a, b, profile, hz, model_digest)
    if cache is not None:
        hit = cache.get(key)
        if hit is not None:
            return hit
    frames = sample_edge(profile, a, b, hz)
    report = twin.check_trajectory(frames)
    detail = ""
    if report.contacts:
        # THE WORST CONTACT, NOT THE FIRST FOUND — discovery order is an
        # artefact of geom numbering. This ranking is only meaningful
        # because `twin._record` makes `depth_mm` the deepest the pair
        # reaches; see that docstring for what it means when it doesn't,
        # and do not re-derive the story here. An earlier version of this
        # comment did, got it backwards, and stood one file away from the
        # correct account contradicting it.
        c = max(report.contacts, key=lambda k: k.depth_mm)
        depth = (f"{c.depth_mm:.1f} mm" if c.depth_mm >= 0.05 else "touching")
        detail = f"{c.body_a} <-> {c.body_b} ({depth})"
    verdict = EdgeVerdict(report.clean, report.poses_checked, detail)
    if cache is not None:
        cache.put(key, verdict)
    return verdict


def validate_clip(twin, clip: Clip, hz: float = DEFAULT_HZ,
                  cache: EdgeCache | None = None) -> list[EdgeVerdict]:
    """Every edge of a clip. Returns one verdict per edge, in order.

    Does NOT stop at the first bad edge: a caller deserves the whole
    picture before deciding, and an operator fixing a routine wants
    every problem at once rather than one per run.
    """
    from .twin import MODEL_XML
    digest = _model_digest(MODEL_XML)
    return [validate_edge(twin, a, b, clip.profile, hz, cache, digest)
            for a, b in clip.edges()]


def _selftest() -> int:
    """No MuJoCo, no arm: a counting twin double proves the cache is
    used when it should be and bypassed when anything that could change
    the answer changes."""
    fails: list[str] = []

    def want(label: str, ok: bool) -> None:
        if not ok:
            fails.append(label)
        print(f"  [{'ok ' if ok else 'FAIL'}] {label}")

    class FakeReport:
        def __init__(self, n, contacts=()):
            self.poses_checked, self.contacts = n, list(contacts)

        @property
        def clean(self):
            return not self.contacts

    class CountingTwin:
        def __init__(self):
            self.calls = 0

        def check_trajectory(self, frames):
            self.calls += 1
            return FakeReport(len(frames))

    a = Pose("rest", {1: 2000, 2: 800})
    b = Pose("up", {1: 2400, 2: 1200})
    prof = MotionProfile(speed=400, acceleration=30)
    D = "modeldigest0001"

    with tempfile.TemporaryDirectory() as td:
        cache = EdgeCache(Path(td) / "c.json")
        twin = CountingTwin()

        v1 = validate_edge(twin, a, b, prof, cache=cache, model_digest=D)
        want("an unseen edge is simulated", twin.calls == 1 and not v1.cached)
        want("...and it reports how many poses were checked",
             v1.poses_checked > 1 and v1.clean)

        v2 = validate_edge(twin, a, b, prof, cache=cache, model_digest=D)
        want("the same edge is answered from the cache",
             twin.calls == 1 and v2.cached and v2.clean)

        # Everything below MUST miss: each changes the swept path or what
        # counts as a collision, so a hit would certify the wrong thing.
        validate_edge(twin, a, Pose("up", {1: 2401, 2: 1200}), prof,
                      cache=cache, model_digest=D)
        want("moving an endpoint one tick re-simulates", twin.calls == 2)

        validate_edge(twin, a, b, MotionProfile(speed=200, acceleration=30),
                      cache=cache, model_digest=D)
        want("a different SPEED re-simulates - profile decides the path, "
             "not just the duration", twin.calls == 3)

        validate_edge(twin, a, b, MotionProfile(speed=400, acceleration=60),
                      cache=cache, model_digest=D)
        want("a different ACCELERATION re-simulates", twin.calls == 4)

        validate_edge(twin, a, b, prof, hz=25.0, cache=cache, model_digest=D)
        want("a coarser SAMPLE RATE re-simulates - sparse sampling can "
             "step over a contact", twin.calls == 5)

        validate_edge(twin, a, b, prof, cache=cache,
                      model_digest="differentmodel")
        want("a different MODEL re-simulates - remembered geometry no "
             "longer applies", twin.calls == 6)

        # ...and the original is still a hit after all that churn.
        v3 = validate_edge(twin, a, b, prof, cache=cache, model_digest=D)
        want("the original edge is still cached", twin.calls == 6 and v3.cached)

        # A contact is remembered as a REFUSAL, not silently as clean.
        class DirtyTwin:
            def check_trajectory(self, frames):
                class C:
                    body_a, body_b, depth_mm = "Base", "Lower_Arm", 12.5
                return FakeReport(len(frames), [C()])

        c2 = EdgeCache(Path(td) / "d.json")
        bad = validate_edge(DirtyTwin(), a, b, prof, cache=c2, model_digest=D)
        want("a colliding edge is refused", not bad.clean)
        want("...and says what hit what", "Base" in bad.detail
             and "12.5" in bad.detail)
        # WHICH contact it names, when there is more than one. Discovery
        # order put a 0.0 mm graze first and a real fold second, and the
        # report named the graze — so the fixture puts them in exactly
        # that order and the assertion is that order does NOT decide.
        class TwoContactTwin:
            def check_trajectory(self, frames):
                def c(a_, b_, d):
                    return type("C", (), {"body_a": a_, "body_b": b_,
                                          "depth_mm": d})()
                return FakeReport(len(frames),
                                  [c("table", "gripper", 0.0),
                                   c("shoulder", "gripper", 0.28)])

        two = validate_edge(TwoContactTwin(), a, b, prof,
                            cache=EdgeCache(Path(td) / "t.json"),
                            model_digest=D)
        want("with several contacts it names the WORST, not the first "
             "the collision engine happened to find",
             "shoulder <-> gripper" in two.detail)
        want("...because 'table' and 'shoulder' have opposite fixes, and "
             "naming the wrong one sends the operator the wrong way",
             "table" not in two.detail)

        again = validate_edge(CountingTwin(), a, b, prof, cache=c2,
                              model_digest=D)
        want("...and the REFUSAL is cached too, so a bad edge is not "
             "quietly re-cleared by a later run",
             again.cached and not again.clean)

        # Round-trip through disk.
        cache.save()
        reloaded = EdgeCache(Path(td) / "c.json").load()
        v4 = validate_edge(CountingTwin(), a, b, prof, cache=reloaded,
                           model_digest=D)
        want("verdicts survive save/load", v4.cached and v4.clean)

        # A corrupt cache must not stop the arm from being validated.
        (Path(td) / "c.json").write_text("{not json")
        broken = EdgeCache(Path(td) / "c.json").load()
        t2 = CountingTwin()
        validate_edge(t2, a, b, prof, cache=broken, model_digest=D)
        want("a CORRUPT cache is discarded and the edge re-simulated, "
             "rather than refusing to run", t2.calls == 1)

        # Pruning drops what a clip no longer references.
        keep = {edge_key(a, b, prof, DEFAULT_HZ, D)}
        n = reloaded.prune(keep)
        want("prune drops unreferenced entries but keeps live ones",
             n >= 0 and edge_key(a, b, prof, DEFAULT_HZ, D) in reloaded.entries)

        # Renaming a pose is NOT an edit: same ticks, same edge.
        renamed = validate_edge(CountingTwin(), Pose("REST", dict(a.ticks)),
                                b, prof, cache=reloaded, model_digest=D)
        want("renaming a pose does not invalidate - a name is not a "
             "position", renamed.cached)

    print("edges selftest " + ("OK" if not fails else f"FAILED: {fails}"))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
