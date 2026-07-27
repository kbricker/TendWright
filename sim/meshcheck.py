"""Is the vendored model the arm we actually printed?

Plan #670. Every other test in this repo proves the model agrees with
ITSELF. Nothing in software can catch "we vendored the wrong robot" —
that check has to reach outside the model, to something independently
sourced. The bench-validation doc originally asked Kyle for calipers.

He has something better: `hardware/so101-print/individual/` holds the
STLs he actually sent to the printer. Those are ground truth for the
parts on the bench, and they came from a different download than the
MJCF. So we can compare printed part against simulated part directly,
and answer the question exactly rather than to +/-5 mm by eye.

WHAT MAKES THIS NON-TRIVIAL: the two sets are not the same files. The
vendored meshes carry 2-5x the triangles (re-tessellated for MuJoCo),
they sit in different local frames (MuJoCo re-origins each mesh to its
joint), and they are in DIFFERENT UNITS — print files are millimetres,
MuJoCo's default length unit is metres. Byte comparison is useless and
vertex comparison is meaningless without solving for the pose.

So compare invariants instead. For a closed solid:

    volume                          rigid-invariant, scales as s^3
    surface area                    rigid-invariant, scales as s^2
    eigenvalues of the covariance   rigid-invariant, scales as s^5

Divide out the scale and those become pure SHAPE descriptors, equal for
two meshes of the same object regardless of units, origin, orientation,
or tessellation:

    eig(C) / V^(5/3)                dimensionless
    A^(3/2) / V                     dimensionless

They are compared dimensionless on purpose. It means this tool never has
to be told the print files are in millimetres — it DERIVES the scale
factor from the volume ratio and reports it, so a units mistake shows up
as a finding instead of being assumed away.

WHAT THIS PROVES AND WHAT IT DOES NOT.

  Proves: every printed link is the same part the sim collides with.
  That is the whole of "did we vendor the wrong robot", and it is the
  only open item on #670.

  Does NOT prove: the link OFFSETS. Those live in the MJCF body `pos`
  attributes, not in any mesh, and a correct part can be attached at a
  wrong distance. `spans` addresses that separately by checking the
  ASSEMBLY: adjacent links bolt together, so their placed meshes must
  touch, and a wrong offset pulls them apart by the size of the error.
  That check is coarse — measured sensitivity ~28 mm outward, blind to
  inward errors, see SENSITIVITY_MM — so it retires "these offsets
  belong to a different arm" without claiming millimetre accuracy. Only
  a caliper measures.

Every claim above is pinned by a selftest that has been shown FAILING:
the invariance property against random rotations and scales, the
discrimination property against the vendored SO-100 (the actual wrong
robot), and the fit check against a deliberately broken model. A check
that has only ever passed is worth nothing — that was the #658 lesson.

    uv run python -m sim.meshcheck            # the comparison table
    uv run python -m sim.meshcheck spans      # offsets vs mesh extents
    uv run python -m sim.meshcheck selftest
"""

from __future__ import annotations

import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
VENDORED = REPO / "sim/assets/so101/assets"
PRINTED = REPO / "hardware/so101-print/individual"

# printed file stem -> vendored file stem.
PAIRS: dict[str, str] = {
    "Base_SO101": "base_so101_v2",
    "Base_motor_holder_SO101": "base_motor_holder_so101_v1",
    "Motor_holder_SO101_Base": "motor_holder_so101_base_v1",
    "Motor_holder_SO101_Wrist": "motor_holder_so101_wrist_v1",
    "Moving_Jaw_SO101": "moving_jaw_so101_v1",
    "Rotation_Pitch_SO101": "rotation_pitch_so101_v1",
    "Under_arm_SO101": "under_arm_so101_v1",
    "Upper_arm_SO101": "upper_arm_so101_v1",
    "WaveShare_Mounting_Plate_SO101": "waveshare_mounting_plate_so101_v2",
    "Wrist_Roll_Follower_SO101": "wrist_roll_follower_so101_v1",
    "Wrist_Roll_Pitch_SO101": "wrist_roll_pitch_so101_v2",
}

# Printed parts with no counterpart in the model, and why. Listed so an
# unpaired file is a recorded decision rather than a silent omission.
UNPAIRED_PRINTED = {
    "Seeedstudio_Mounting_Plate_SO101":
        "alternative to the WaveShare plate; the model uses WaveShare",
}
UNPAIRED_VENDORED = {
    "sts3215_03a_v1": "the servo itself - bought, not printed",
    "sts3215_03a_no_horn_v1": "the servo itself - bought, not printed",
}

# A part is judged the same shape when every dimensionless descriptor
# agrees within this relative tolerance. The meshes are independently
# tessellated, so exact agreement is not on offer: a coarser
# triangulation of a curved surface cuts corners and slightly
# under-reports volume.
#
# 2% is not a guess — it sits in a measured gap. Run on this data set:
#
#     worst TRUE pair (printed vs vendored SO-101)      0.09%
#     best  FALSE pair (printed SO-101 vs SO-100)       3.88%
#
# a 40x separation, with the threshold ~20x above the noise floor and
# ~2x below the closest false match. The selftest asserts BOTH ends
# against the real files, so this constant cannot quietly drift away
# from the data that justifies it.
SHAPE_TOL = 0.02

# The SO-100 package, kept vendored from before the #670 swap. It is the
# negative control for this whole module: the failure #670 exists to
# rule out is "we vendored the wrong robot", and the only way to show
# this tool would CATCH that is to point it at the wrong robot.
SO100 = REPO / "sim/assets/menagerie/trs_so_arm100/assets"

# printed SO-101 part -> the SO-100 part playing the same role.
WRONG_ARM: dict[str, str] = {
    "Base_SO101": "Base",
    "Upper_arm_SO101": "Upper_Arm",
    "Under_arm_SO101": "Lower_Arm",
    "Rotation_Pitch_SO101": "Rotation_Pitch",
    "Moving_Jaw_SO101": "Moving_Jaw",
    "Wrist_Roll_Pitch_SO101": "Wrist_Pitch_Roll",
}

# The scale factor every pair must agree on, and how tightly. mm -> m.
EXPECTED_SCALE = 0.001
SCALE_TOL = 0.02


# --------------------------------------------------------------------
# STL reading


def read_stl(path: Path) -> np.ndarray:
    """Triangles from an STL as an (N, 3, 3) array. Binary or ASCII."""
    raw = path.read_bytes()
    if _looks_ascii(raw):
        return _read_ascii(raw)
    return _read_binary(raw, path)


def _looks_ascii(raw: bytes) -> bool:
    """ASCII STLs start with 'solid'. So do SOME binary ones, so also
    check that the binary triangle count matches the file length."""
    if not raw[:5].lower().startswith(b"solid"):
        return False
    if len(raw) < 84:
        return True
    (n,) = struct.unpack_from("<I", raw, 80)
    return len(raw) != 84 + n * 50


def _read_binary(raw: bytes, path: Path) -> np.ndarray:
    if len(raw) < 84:
        raise ValueError(f"{path.name}: too short to be a binary STL")
    (n,) = struct.unpack_from("<I", raw, 80)
    want = 84 + n * 50
    if len(raw) != want:
        raise ValueError(
            f"{path.name}: header claims {n} triangles ({want} bytes) "
            f"but the file is {len(raw)} bytes")
    # Each 50-byte record is normal(3f) + 3 vertices(9f) + attr(uint16).
    # Read as bytes, drop the trailing 2, view as float32, drop the normal.
    body = np.frombuffer(raw, dtype=np.uint8, count=n * 50, offset=84)
    body = body.reshape(n, 50)[:, :48].copy()
    floats = body.view(np.float32).reshape(n, 12)
    return floats[:, 3:].reshape(n, 3, 3).astype(np.float64)


def _read_ascii(raw: bytes) -> np.ndarray:
    verts: list[tuple[float, float, float]] = []
    for line in raw.decode("ascii", "replace").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not verts or len(verts) % 3:
        raise ValueError(f"ASCII STL has {len(verts)} vertices, not a multiple of 3")
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3)


# --------------------------------------------------------------------
# Shape descriptors

# Covariance of the canonical tetrahedron (origin + the three unit axes),
# integrated over its volume: diagonal 1/60, off-diagonal 1/120.
_C_CANON = np.full((3, 3), 1.0 / 120.0) + np.eye(3) * (1.0 / 120.0)


@dataclass(frozen=True)
class Shape:
    """Rigid-transform invariants of a closed triangle mesh."""
    tris: int
    volume: float
    area: float
    extents: tuple[float, float, float]   # bbox, sorted ascending
    cov_eigs: tuple[float, float, float]  # covariance about the centroid

    @property
    def size(self) -> float:
        """Characteristic length: the cube root of the volume."""
        return abs(self.volume) ** (1.0 / 3.0)

    def descriptors(self) -> dict[str, float]:
        """Dimensionless shape numbers — no units, no scale, no pose.

        NOT bounding-box ratios, though they are the obvious thing to
        reach for and this module shipped with them for one revision.
        An axis-aligned bounding box is not rotation-invariant: the AABB
        of a tilted part is larger than the part. The real SO-101 pairs
        agreed to 0.03% on bbox anyway — MuJoCo re-origins its meshes
        with 90-degree rotations (quats like `0.5 0.5 0.5 0.5`), which
        permute the AABB axes rather than growing them, and the extents
        are sorted. That is a coincidence of this data set, not a
        property, and the selftest's random-rotation case is what
        exposed it. The covariance eigenvalues carry the same
        aspect-ratio information and are genuinely invariant, so the
        bbox terms were redundant as well as wrong. `extents` stays on
        the dataclass for reporting and for `spans`, which needs real
        millimetres — it just does not decide any verdict.
        """
        v = abs(self.volume)
        if v <= 0.0:
            raise ValueError("degenerate mesh: volume is zero")
        d = {"area/vol^(2/3)": self.area / v ** (2.0 / 3.0)}
        for i, e in enumerate(self.cov_eigs):
            d[f"cov{i + 1}/vol^(5/3)"] = e / v ** (5.0 / 3.0)
        return d


def shape_of(tris: np.ndarray) -> Shape:
    flat = tris.reshape(-1, 3)
    lo, hi = flat.min(axis=0), flat.max(axis=0)
    extents = tuple(sorted(hi - lo))

    # Everything below builds tetrahedra from the ORIGIN, so a mesh that
    # sits far from the origin relative to its own size produces enormous
    # terms that must cancel down to a tiny result. A 10 mm part 100 mm
    # out needs ~20 digits of cancellation for the covariance — past
    # float64, and the selftest's scale-x0.001 case hits exactly that.
    # Recentring first costs one subtraction and makes every term the
    # order of the part itself. Volume, area and extents are all
    # translation-invariant, so nothing about the result changes.
    tris = tris - (lo + hi) / 2.0

    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    cross = np.cross(b, c)

    # Signed volume via the tetrahedron fan from the origin.
    dets = np.einsum("ij,ij->i", a, cross)
    volume = dets.sum() / 6.0

    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum()

    # Covariance about the origin, accumulated per tetrahedron:
    #   C_tet = det(A) * A @ C_canon @ A.T,  A = [a b c] as columns.
    A = np.stack([a, b, c], axis=2)
    cov = np.einsum("n,nij->ij", dets, A @ _C_CANON @ A.transpose(0, 2, 1))

    # Centroid, then parallel-axis shift so the result is origin-free.
    centroid = np.einsum("n,nj->j", dets, (a + b + c)) / (4.0 * dets.sum())
    cov = cov - volume * np.outer(centroid, centroid)

    # An inward-wound mesh describes the same solid with every sign
    # flipped. Normalise rather than trust the exporter — a mesh whose
    # descriptors all came out negative would mismatch every partner for
    # a reason that has nothing to do with its shape.
    if volume < 0.0:
        volume, cov = -volume, -cov

    eigs = tuple(sorted(np.linalg.eigvalsh((cov + cov.T) / 2.0)))
    return Shape(len(tris), float(volume), float(area),
                 tuple(float(x) for x in extents),
                 tuple(float(x) for x in eigs))


# --------------------------------------------------------------------
# Comparison


@dataclass(frozen=True)
class Verdict:
    printed: str
    vendored: str
    p: Shape
    v: Shape
    scale: float          # derived: vendored units per printed unit
    worst_name: str
    worst_rel: float

    @property
    def shape_ok(self) -> bool:
        return self.worst_rel <= SHAPE_TOL

    @property
    def scale_ok(self) -> bool:
        return abs(self.scale / EXPECTED_SCALE - 1.0) <= SCALE_TOL

    @property
    def ok(self) -> bool:
        return self.shape_ok and self.scale_ok


def compare(printed: Shape, vendored: Shape, name_p: str, name_v: str) -> Verdict:
    dp, dv = printed.descriptors(), vendored.descriptors()
    worst_name, worst_rel = "", 0.0
    for k in dp:
        denom = max(abs(dp[k]), abs(dv[k]), 1e-12)
        rel = abs(dp[k] - dv[k]) / denom
        if rel > worst_rel:
            worst_name, worst_rel = k, rel
    scale = vendored.size / printed.size if printed.size > 0 else float("nan")
    return Verdict(name_p, name_v, printed, vendored, scale, worst_name, worst_rel)


def check_all(printed_dir: Path = PRINTED,
              vendored_dir: Path = VENDORED) -> list[Verdict]:
    out = []
    for stem_p, stem_v in PAIRS.items():
        fp, fv = printed_dir / f"{stem_p}.stl", vendored_dir / f"{stem_v}.stl"
        if not fp.exists():
            raise FileNotFoundError(f"printed STL missing: {fp}")
        if not fv.exists():
            raise FileNotFoundError(f"vendored STL missing: {fv}")
        out.append(compare(shape_of(read_stl(fp)), shape_of(read_stl(fv)),
                           stem_p, stem_v))
    return out


def unlisted_files(printed_dir: Path = PRINTED,
                   vendored_dir: Path = VENDORED) -> tuple[list[str], list[str]]:
    """STLs on disk that neither PAIRS nor the UNPAIRED notes account for.

    Without this a part added to either folder would simply not be
    checked, and the table would still say every pair matched.
    """
    have_p = {f.stem for f in printed_dir.glob("*.stl")}
    have_v = {f.stem for f in vendored_dir.glob("*.stl")}
    return (sorted(have_p - set(PAIRS) - set(UNPAIRED_PRINTED)),
            sorted(have_v - set(PAIRS.values()) - set(UNPAIRED_VENDORED)))


# --------------------------------------------------------------------
# Commands


def cmd_check() -> int:
    stray_p, stray_v = unlisted_files()
    verdicts = check_all()

    print(f"printed  {PRINTED}")
    print(f"vendored {VENDORED}")
    print()
    print(f"{'part':<34} {'tris p/v':>13} {'vol mm^3':>10} "
          f"{'scale':>8} {'worst':>7}  ")
    print("-" * 84)
    for v in verdicts:
        tris = f"{v.p.tris}/{v.v.tris}"
        mark = "ok " if v.ok else "**MISMATCH**"
        print(f"{v.printed:<34} {tris:>13} {abs(v.p.volume):>10.0f} "
              f"{v.scale:>8.5f} {v.worst_rel * 100:>6.2f}%  {mark}")

    bad = [v for v in verdicts if not v.ok]
    print()
    for name, why in UNPAIRED_PRINTED.items():
        print(f"  not compared (printed):  {name} - {why}")
    for name, why in UNPAIRED_VENDORED.items():
        print(f"  not compared (vendored): {name} - {why}")

    if stray_p or stray_v:
        print()
        for n in stray_p:
            print(f"  ** UNACCOUNTED printed STL:  {n}")
        for n in stray_v:
            print(f"  ** UNACCOUNTED vendored STL: {n}")
        print("  (add it to PAIRS or to the UNPAIRED notes with a reason)")

    print()
    scales = [v.scale for v in verdicts]
    print(f"derived scale, vendored per printed unit: "
          f"{min(scales):.5f} .. {max(scales):.5f}  "
          f"(expected {EXPECTED_SCALE} — print files are mm, MuJoCo is m)")

    if bad:
        print()
        for v in bad:
            print(f"MISMATCH {v.printed} vs {v.vendored}: "
                  f"worst descriptor {v.worst_name} off by "
                  f"{v.worst_rel * 100:.1f}%, scale {v.scale:.5f}")
        print(f"\n{len(bad)} of {len(verdicts)} parts DO NOT MATCH")
        return 1

    print(f"\nall {len(verdicts)} printed parts match the model's meshes "
          f"(within {SHAPE_TOL * 100:.0f}%)")
    print("\nThis proves the PARTS are right. It does not measure where they")
    print("are attached — run `spans` for the bracket on that.")
    return 0


# Adjacent links bolt together, so the gap between their placed meshes
# is zero by construction. A wrong body offset pulls them apart by
# exactly the size of the error. 5 mm is well above the sampling noise
# below (the selftest measures the real floor) and far under the
# smallest offset error worth caring about.
GAP_TOL_MM = 5.0

# How big an offset error the fit check can actually SEE, measured by
# bisection on 2026-07-27 (not estimated — the selftest re-derives the
# same behaviour). Servos nest deep inside their holders, so a link has
# to move most of the way out of its socket before any gap opens:
#
#     upper_arm / lower_arm / wrist    ~28 mm outward
#     gripper                          ~8 mm outward
#     any link, inward                 NEVER — overlap reads as contact
#
# So this is a coarse check. It rules out offsets belonging to a
# different arm; it does not verify them to the millimetre, and it is
# not a substitute for a caliper if that precision is ever needed.
SENSITIVITY_MM = 28.0

# Vertices sampled per body when measuring gaps. Brute-force all-pairs,
# so this squares. Sampling can only ever OVER-report a gap (the true
# nearest pair may be skipped), never under-report one — so it produces
# false alarms, not false passes, which is the right way round.
GAP_SAMPLE = 3000


def _body_points(model, data, body_id: int) -> np.ndarray:
    """World-frame mesh vertices of every geom on one body."""
    out = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body_id:
            continue
        mid = model.geom_dataid[g]
        if mid < 0:                       # not a mesh (plane, primitive)
            continue
        adr, num = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        v = model.mesh_vert[adr:adr + num].reshape(-1, 3).astype(np.float64)
        gm = data.geom_xmat[g].reshape(3, 3)
        out.append(v @ gm.T + data.geom_xpos[g])
    if not out:
        return np.zeros((0, 3))
    return np.concatenate(out)


def _min_gap_mm(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest distance between two point clouds, in mm."""
    for pts in (a, b):
        if len(pts) == 0:
            return float("nan")
    if len(a) > GAP_SAMPLE:
        a = a[:: max(1, len(a) // GAP_SAMPLE)]
    if len(b) > GAP_SAMPLE:
        b = b[:: max(1, len(b) // GAP_SAMPLE)]
    best = np.inf
    for chunk in np.array_split(a, max(1, len(a) // 256)):
        d = np.linalg.norm(chunk[:, None, :] - b[None, :, :], axis=2)
        best = min(best, float(d.min()))
    return best * 1000.0


def assembly_gaps(perturb: tuple[str, float] | None = None,
                  ) -> list[tuple[str, str, float]]:
    """Gap between each adjacent pair of links, as assembled by the MJCF.

    `perturb` moves one named body by that many millimetres ALONG ITS OWN
    OFFSET from its parent — i.e. further out along the link, which is
    the direction a wrong link length actually errs in. Negative pulls it
    inward. This is the selftest's way of proving the check can fail
    rather than passing because nothing here can move.
    """
    import mujoco
    from sim.twin import MODEL_XML

    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    if perturb is not None:
        bname, mm = perturb
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
        if bid < 0:
            raise ValueError(f"no body named {bname!r} to perturb")
        off = model.body_pos[bid]
        norm = float(np.linalg.norm(off))
        if norm < 1e-9:
            raise ValueError(f"body {bname!r} sits on its parent; no offset "
                             f"direction to perturb along")
        model.body_pos[bid] = off + off / norm * (mm / 1000.0)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    # Walk the kinematic chain off the model rather than naming bodies —
    # a hardcoded list would silently stop covering the arm the next
    # time the model changes, which is the #670 failure in miniature.
    chain: list[int] = []
    for b in range(1, model.nbody):
        if model.body_parentid[b] in (0, *chain):
            chain.append(b)
    pts = {b: _body_points(model, data, b) for b in chain}

    out = []
    for b in chain:
        p = model.body_parentid[b]
        if p not in pts or len(pts[b]) == 0 or len(pts[p]) == 0:
            continue
        name = lambda i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        out.append((name(p), name(b), _min_gap_mm(pts[p], pts[b])))
    return out


def cmd_spans() -> int:
    """Does the MJCF's ASSEMBLY agree with its own parts?

    The mesh comparison proves each part is right. It says nothing about
    where the parts are attached — those offsets live in the MJCF body
    `pos` attributes, and no STL contains them.

    This closes that gap without a caliper. Adjacent links bolt to each
    other, so their placed meshes touch. If a body offset were wrong by
    20 mm, the two parts would float 20 mm apart in the assembled model
    — visible here as a gap, with no reference measurement needed. The
    model is checked against itself, but against a part of itself
    (the meshes) that has now been independently verified.
    """
    from sim.rig import Rig

    print("Link offsets the model reports (m1 centre to m6 centre, at qpos 0):\n")
    for a, b, mm in Rig().link_lengths():
        print(f"  m{a}->m{b}   {mm:>7.1f} mm")

    print("\nAssembled-fit check — adjacent links bolt together, so the gap")
    print("between their placed meshes must be ~0. A wrong body offset")
    print("separates them by the size of the error.\n")
    print(f"{'parent':<22} {'child':<22} {'gap mm':>8}")
    print("-" * 60)
    bad = 0
    for parent, child, gap in assembly_gaps():
        ok = gap <= GAP_TOL_MM
        bad += 0 if ok else 1
        print(f"{parent:<22} {child:<22} {gap:>8.2f}   "
              f"{'ok' if ok else '** FLOATING **'}")

    print("\nWHAT THIS PROVES: no link is attached at a distance that belongs")
    print("to a different arm. The parts meet where the model says they do.")
    print(f"\nHOW COARSE IT IS (measured, not assumed): a servo nests deep in")
    print(f"its holder, so an offset must err by ~{SENSITIVITY_MM:.0f} mm outward "
          f"(~8 mm at the")
    print("gripper) before a gap opens at all — and an INWARD error never")
    print("shows, because overlap and contact both read as zero distance.")
    print("Millimetre-level verification still wants a caliper. What this")
    print("does retire is the big question — 'is this even the right arm' —")
    print("which the mesh table above settles outright.")
    return 1 if bad else 0


# --------------------------------------------------------------------
# Selftest


def _rand_rigid(tris: np.ndarray, seed: int, scale: float = 1.0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1                        # keep it a rotation
    t = rng.normal(size=3) * 100.0
    moved = (tris.reshape(-1, 3) * scale) @ q.T + t
    return moved.reshape(-1, 3, 3)


def _box(sx: float, sy: float, sz: float) -> np.ndarray:
    """A closed, outward-wound box — a solid with known volume and area."""
    v = np.array([[0, 0, 0], [sx, 0, 0], [sx, sy, 0], [0, sy, 0],
                  [0, 0, sz], [sx, 0, sz], [sx, sy, sz], [0, sy, sz]],
                 dtype=np.float64)
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return np.array([[v[i], v[j], v[k]] for i, j, k in faces])


def selftest() -> int:
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}{'  ' + detail if detail else ''}")
        if not cond:
            fails.append(name)

    print("shape descriptors")
    box = _box(10.0, 20.0, 40.0)
    s = shape_of(box)
    check("box volume", abs(s.volume - 8000.0) < 1e-6, f"{s.volume:.3f}")
    check("box area", abs(s.area - 2800.0) < 1e-6, f"{s.area:.3f}")
    check("box extents", np.allclose(s.extents, (10, 20, 40)), f"{s.extents}")
    # Covariance of a box about its centre: V * s^2/12 per axis.
    want = sorted(8000.0 * np.array([10.0, 20.0, 40.0]) ** 2 / 12.0)
    check("box covariance", np.allclose(s.cov_eigs, want, rtol=1e-9),
          f"{[round(e) for e in s.cov_eigs]} vs {[round(w) for w in want]}")

    print("\ninvariance — the property the whole comparison rests on")
    for seed in (1, 2, 3):
        moved = shape_of(_rand_rigid(box, seed))
        v = compare(s, moved, "box", "moved box")
        check(f"rigid transform seed {seed}", v.worst_rel < 1e-9,
              f"worst {v.worst_rel:.2e}")
    for factor in (0.001, 7.3, 1000.0):
        scaled = shape_of(_rand_rigid(box, 4, scale=factor))
        v = compare(s, scaled, "box", "scaled box")
        check(f"scale x{factor}", v.worst_rel < 1e-9 and
              abs(v.scale / factor - 1.0) < 1e-9,
              f"worst {v.worst_rel:.2e}, derived scale {v.scale:.6g}")

    print("\ndiscrimination — it must REJECT a different part")
    # 10x20x40 vs 10x20x44: same family, 10% longer in one axis only.
    v = compare(s, shape_of(_rand_rigid(_box(10.0, 20.0, 44.0), 5, 0.001)),
                "box", "10% longer box")
    check("rejects a 10%-longer box", not v.shape_ok,
          f"worst {v.worst_rel * 100:.1f}% > tol {SHAPE_TOL * 100:.0f}%")
    # A cube of the SAME VOLUME — volume alone would call these equal.
    cube = _box(*(8000.0 ** (1 / 3),) * 3)
    v = compare(s, shape_of(_rand_rigid(cube, 6, 0.001)), "box", "equal-volume cube")
    check("rejects an equal-volume cube", not v.shape_ok,
          f"worst {v.worst_rel * 100:.1f}%")

    print("\ntessellation robustness — the meshes ARE differently tessellated")
    # Split every triangle into four. Same solid, 4x the triangles: the
    # descriptors must not move, or the whole comparison is measuring
    # mesh density instead of shape.
    fine = box
    for _ in range(2):
        a, b, c = fine[:, 0], fine[:, 1], fine[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        fine = np.concatenate([np.stack([a, ab, ca], 1), np.stack([ab, b, bc], 1),
                               np.stack([ca, bc, c], 1), np.stack([ab, bc, ca], 1)])
    vf = shape_of(fine)
    v = compare(s, vf, "box", "16x tessellated")
    check("16x subdivision changes nothing", v.worst_rel < 1e-9,
          f"{s.tris} -> {vf.tris} tris, worst {v.worst_rel:.2e}")

    print("\nSTL reading")
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        # Binary round-trip.
        bp = Path(td) / "b.stl"
        with bp.open("wb") as fh:
            fh.write(b"\0" * 80 + struct.pack("<I", len(box)))
            for tri in box:
                fh.write(struct.pack("<3f", 0, 0, 0))
                for vert in tri:
                    fh.write(struct.pack("<3f", *vert))
                fh.write(b"\0\0")
        check("binary round-trip", np.allclose(read_stl(bp), box))
        # ASCII round-trip.
        ap = Path(td) / "a.stl"
        lines = ["solid t"]
        for tri in box:
            lines.append("facet normal 0 0 0\n outer loop")
            lines += [f"  vertex {x} {y} {z}" for x, y, z in tri]
            lines.append(" endloop\nendfacet")
        lines.append("endsolid t")
        ap.write_text("\n".join(lines))
        check("ascii round-trip", np.allclose(read_stl(ap), box))
        # A truncated binary STL must raise, not silently read short.
        trunc = Path(td) / "t.stl"
        trunc.write_bytes(bp.read_bytes()[:-40])
        try:
            read_stl(trunc)
            check("truncated STL raises", False)
        except ValueError as exc:
            check("truncated STL raises", True, str(exc)[:52])

    print("\nassembled fit — and proof the fit check can fail")
    gaps = assembly_gaps()
    check("every adjacent link pair touches", bool(gaps) and
          all(g <= GAP_TOL_MM for _, _, g in gaps),
          f"{len(gaps)} pairs, worst {max(g for _, _, g in gaps):.2f} mm "
          f"vs tol {GAP_TOL_MM} mm")
    # Break the model on purpose. Without this the fit check is just an
    # assertion that nothing moved — it has to be shown separating a
    # pair it would otherwise pass.
    # 40 mm, not 20: measured, the interlock absorbs about 28 mm before
    # any gap appears (see SENSITIVITY_MM). A 20 mm shift opens only
    # 1.59 mm and would leave this test failing for an honest reason.
    SHIFT = 40.0
    base = {(p, c): g for p, c, g in gaps}
    for body in ("upper_arm", "lower_arm", "wrist"):
        moved = {(p, c): g for p, c, g in assembly_gaps((body, SHIFT))}
        # "Opened" means the gap now trips the tolerance AND grew doing
        # it. Do not require the gap to grow by some fraction of the
        # shift — it does not, because the link spends most of its
        # travel simply withdrawing from the socket. Tripping is the
        # property under test; how far it trips by is not.
        opened = [k for k in base
                  if moved.get(k, 0.0) > GAP_TOL_MM
                  and moved[k] > base[k] + 1.0]
        check(f"a {SHIFT:.0f} mm error on {body} opens a gap that trips",
              bool(opened),
              f"{[f'{p}->{c} {base[k]:.2f}->{moved[k]:.1f}mm' for k in opened for p, c in [k]]}")

    # And the blind spot, measured rather than asserted. Pulling a link
    # INWARD buries it in its parent; a minimum-distance metric reads
    # overlap and contact identically, so this check cannot see it. Said
    # out loud here so the limitation is a recorded measurement instead
    # of a caveat someone has to take on trust.
    buried = {(p, c): g for p, c, g in assembly_gaps(("upper_arm", -SHIFT))}
    hidden = all(g <= GAP_TOL_MM for g in buried.values())
    print(f"  note {'CONFIRMED' if hidden else 'unexpected'}: a {SHIFT:.0f} mm "
          f"INWARD error stays invisible (worst gap "
          f"{max(buried.values()):.2f} mm) — overlap reads as contact")

    print("\nbookkeeping")
    stray_p, stray_v = unlisted_files()
    check("every printed STL is paired or excused", not stray_p, str(stray_p))
    check("every vendored STL is paired or excused", not stray_v, str(stray_v))

    print("\nthe real parts")
    verdicts = check_all()
    for v in verdicts:
        check(f"{v.printed} matches {v.vendored}", v.ok,
              f"worst {v.worst_rel * 100:.2f}%, scale {v.scale:.5f}")

    print("\nnegative control — the SAME tool pointed at the WRONG arm")
    wrong: list[Verdict] = []
    if not SO100.exists():
        # Loud on purpose. Without this section the module's central
        # claim - that it can tell one arm from another - rests on
        # nothing but synthetic boxes. A silent skip here would leave a
        # passing selftest that proves much less than it appears to.
        print(f"  ** NOT CHECKED: {SO100} is gone.")
        print("  ** The wrong-arm discrimination claim is UNVERIFIED in this")
        print("  ** tree. Last measured 2026-07-27: worst true pair 0.09%,")
        print("  ** best false pair 3.88%. Restore the SO-100 assets to")
        print("  ** re-establish it, or delete this claim from the docstring.")
    else:
        for stem_p, stem_w in WRONG_ARM.items():
            w = compare(shape_of(read_stl(PRINTED / f"{stem_p}.stl")),
                        shape_of(read_stl(SO100 / f"{stem_w}.stl")),
                        stem_p, f"SO100/{stem_w}")
            wrong.append(w)
            check(f"REJECTS SO-100 {stem_w} as {stem_p}", not w.shape_ok,
                  f"worst {w.worst_rel * 100:.2f}%")

    if wrong:
        worst_true = max(v.worst_rel for v in verdicts)
        best_false = min(w.worst_rel for w in wrong)
        check("true and false populations are separated",
              worst_true < SHAPE_TOL < best_false,
              f"true <= {worst_true * 100:.2f}% | tol {SHAPE_TOL * 100:.0f}% "
              f"| false >= {best_false * 100:.2f}%  "
              f"(separation {best_false / worst_true:.0f}x)")

    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("all mesh checks OK")
    return 0


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "check":
        return cmd_check()
    if cmd == "spans":
        return cmd_spans()
    if cmd == "selftest":
        return selftest()
    print(f"unknown command {cmd!r}; use check | spans | selftest")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
