"""A camera inside the sim, so perception can be built before the parts land.

Plan #606. The pick-and-place pipeline has four links — see a thing,
work out where it is, solve a pose, move there — and only the last two
exist. The first two need a camera, and the cameras are still shipping.

They do not need a REAL camera. MuJoCo renders offscreen from any pose,
the cell model already knows where the bench camera sits, and OpenCV can
generate the same tag36h11 markers `campreview` already detects. So the
whole perception chain can be exercised now against a camera whose
ground truth is exact, which is a better place to debug it than a real
lens in real light. When the hardware arrives, only the image source
changes.

WHY THAT ORDER IS WORTH THE TROUBLE. A vision pipeline has two ways to
be wrong — the perception is off, or the arithmetic turning a detection
into a position is off — and against a real camera both are live at
once, so a bad answer says nothing about which. Here the perception is
exact by construction, and anything wrong is arithmetic.

THE FRAME BRIDGE, which is the part with real risk in it. `sim.ik`
speaks the RIG frame (mm, origin at the m1 rotation centre, world-axis
aligned). The cell places the arm at a position AND a yaw, so rig
coordinates are not cell-world coordinates offset — they are rotated
too. Getting that wrong is the #670 attach-angle defect again, and it
would silently put every target in the wrong place.

So the rotation is MEASURED, not asserted: tool positions are compared
between the two models across several poses and the yaw is least-squares
fitted. It comes out at exactly `arm_yaw - twin.reach_yaw_deg()`, which
is what the cell builder uses to attach the arm — so the derivation and
the measurement agree, and `verify_frame()` re-checks it at construction
rather than trusting either.

    uv run python -m sim.simcam render out.png       # what the camera sees
    uv run python -m sim.simcam tags out.png         # with tags on the table
    uv run python -m sim.simcam selftest
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _pick_gl_backend() -> None:
    """Choose an offscreen GL backend before MuJoCo initialises one.

    cell1 is the deployment target and it is driven over ssh, where
    there is no display and MuJoCo's default backend dies with
    `gladLoadGL error`. EGL renders on the GPU without a window and
    works there — verified on cell1, producing frames identical in mean
    brightness to the desk's.

    Set only when nothing is set: an explicit MUJOCO_GL always wins, and
    on a machine that HAS a display the default is already right.
    """
    if os.environ.get("MUJOCO_GL"):
        return
    if sys.platform.startswith("linux") and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        os.environ["MUJOCO_GL"] = "egl"


_pick_gl_backend()

# A tag geom is a thin slab rather than a plane: MuJoCo planes are
# infinite/one-sided and awkward to texture, and a slab also reads
# correctly in the interactive viewer.
TAG_THICK_MM = 1.0

# Pixels per side in the generated marker image. Only affects texture
# sharpness, not geometry — 512 is well past what any render resolves.
TAG_TEXTURE_PX = 512

# White quiet zone around the marker, as a fraction of its side. AprilTag
# detection needs light margin around the black border; without it the
# detector finds nothing at all, which reads as "the pipeline is broken"
# rather than "the tag has no border".
TAG_QUIET = 0.25

# Where the fitted rig->cell yaw must land relative to the derived one.
# They agree to floating-point today; this is a guard against a future
# change to either side, not a tolerance anything needs.
YAW_AGREE_DEG = 0.01


@dataclass(frozen=True)
class Tag:
    """An AprilTag lying flat on the bench, positioned where IK speaks.

    `x_mm`/`y_mm` are RIG-frame — the same numbers `sim.ik` takes and
    `sim.rig where` prints — so a tag can be placed at a target and the
    arm asked to reach it without a conversion in between. `above_mm`
    is height above the table surface.
    """

    tag_id: int
    x_mm: float
    y_mm: float
    size_mm: float = 40.0
    above_mm: float = 0.0


def tag_image(tag_id: int, px: int = TAG_TEXTURE_PX,
              quiet: float = TAG_QUIET) -> np.ndarray:
    """A tag36h11 marker with a white quiet zone, as a grayscale image."""
    import cv2

    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(d, tag_id, px)
    pad = int(px * quiet)
    canvas = np.full((px + 2 * pad, px + 2 * pad), 255, np.uint8)
    canvas[pad:pad + px, pad:pad + px] = marker
    return canvas


class SimCam:
    """The cell, rendered from where a real camera actually sits."""

    def __init__(self, tags: tuple[Tag, ...] = (), cell=None,
                 asset_dir: Path | None = None):
        import mujoco

        from sim.bench_scene import build_spec, load_cell
        from sim.rig import Rig
        from sim.twin import JOINT_MAPS, TOOL_BODY, Twin

        self._mj = mujoco
        self.cell = cell or load_cell()
        self.tags = tuple(tags)
        self.twin = Twin()
        self.rig = Rig(self.twin)
        self._tool = TOOL_BODY

        self._assets = Path(asset_dir or tempfile.mkdtemp(prefix="simcam-"))

        def compiled(with_tags: bool):
            spec = build_spec(self.cell.bench, self.cell.shadows,
                              self.cell.cameras, self.cell.arm_pose)
            if with_tags:
                for t in self.tags:
                    self._add_tag(spec, t)
            return spec.compile()

        # Two builds, because tags are POSITIONED through the very
        # transform that has to be measured off a built model — and the
        # measurement needs the arm, not the tags. Placing a tag before
        # the frame is known would mean guessing the rotation, which is
        # the one thing this module refuses to do. Tags add no dynamics
        # and do not move the arm, so the frame derived without them is
        # the frame with them.
        self._bind(compiled(False))
        self.pose({i: c.rest for i, c in self.twin.cals.items()})
        self.origin = np.array(self.data.xanchor[self._pan_jid])
        self.yaw_deg = self._fit_yaw()
        if self.tags:
            self._bind(compiled(True))
            self.pose({i: c.rest for i, c in self.twin.cals.items()})

    def _bind(self, model) -> None:
        """Adopt a compiled model and re-resolve every name off it."""
        from sim.twin import JOINT_MAPS

        mujoco = self._mj
        self.model = model
        self.data = mujoco.MjData(model)
        self._adr = {}
        for i, jm in JOINT_MAPS.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"arm_{jm.model_joint}")
            if jid < 0:
                raise RuntimeError(f"arm joint arm_{jm.model_joint} is not in "
                                   f"the cell model — the attach prefix "
                                   f"changed")
            self._adr[i] = model.jnt_qposadr[jid]
        self._pan_jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "arm_shoulder_pan")

    # ---------------------------------------------------------------- frame

    def _fit_yaw(self) -> float:
        """Least-squares the rig->cell Z rotation from tool positions.

        Measured rather than taken from cell.json, because the quantity
        that matters is what the COMPILED model does, not what the
        config intended. `verify_frame` then checks the measurement
        against the derivation, so a disagreement is loud instead of
        being silently split between two plausible numbers.
        """
        rest = {i: c.rest for i, c in self.twin.cals.items()}
        num = den = 0.0
        for d1 in (0, 400, -400):
            for d2 in (0, 300):
                pose = {**rest, 1: rest[1] + d1, 2: rest[2] + d2}
                a = np.asarray(self._rig_tool(pose))
                b = (self._cell_tool(pose) - self.origin) * 1000.0
                num += a[0] * b[1] - a[1] * b[0]
                den += a[0] * b[0] + a[1] * b[1]
        return math.degrees(math.atan2(num, den))

    def verify_frame(self) -> tuple[float, float]:
        """(worst round-trip error in mm, disagreement with the derived yaw).

        Both should be ~0. The first says the fitted rotation actually
        reproduces the cell model; the second says it agrees with the
        angle the cell builder used to attach the arm.
        """
        rest = {i: c.rest for i, c in self.twin.cals.items()}
        worst = 0.0
        for d1 in (0, 250, -250, 700):
            pose = {**rest, 1: rest[1] + d1}
            predicted = self.rig_to_world(self._rig_tool(pose))
            worst = max(worst, float(np.linalg.norm(
                predicted - self._cell_tool(pose)) * 1000.0))
        derived = self.cell.arm_pose[2] - self.twin.reach_yaw_deg()
        return worst, abs(self.yaw_deg - derived)

    def rig_to_world(self, xyz_mm) -> np.ndarray:
        """Rig-frame mm -> cell-world metres."""
        th = math.radians(self.yaw_deg)
        x, y, z = np.asarray(xyz_mm, dtype=float)
        return self.origin + np.array([
            (x * math.cos(th) - y * math.sin(th)) / 1000.0,
            (x * math.sin(th) + y * math.cos(th)) / 1000.0,
            z / 1000.0])

    def world_to_rig(self, xyz_m) -> np.ndarray:
        """Cell-world metres -> rig-frame mm."""
        th = math.radians(-self.yaw_deg)
        v = (np.asarray(xyz_m, dtype=float) - self.origin) * 1000.0
        return np.array([v[0] * math.cos(th) - v[1] * math.sin(th),
                         v[0] * math.sin(th) + v[1] * math.cos(th), v[2]])

    # ---------------------------------------------------------------- scene

    def _add_tag(self, spec, t: Tag) -> None:
        import cv2

        path = self._assets / f"tag36h11_{t.tag_id}.png"
        cv2.imwrite(str(path), tag_image(t.tag_id))
        name = f"tag{t.tag_id}"
        spec.add_texture(name=name, type=self._mj.mjtTexture.mjTEXTURE_2D,
                         file=str(path))
        mat = spec.add_material(name=f"{name}_mat")
        mat.textures[self._mj.mjtTextureRole.mjTEXROLE_RGB] = name
        # texuniform off + one repeat = the image maps once per face, so
        # the slab's top face carries exactly one marker.
        mat.texuniform = False
        mat.texrepeat = [1, 1]

        # Placed via the SAME transform the arm targets go through, so a
        # tag put at a rig coordinate is somewhere IK can be asked for.
        half = t.size_mm * (1.0 + 2 * TAG_QUIET) / 2000.0
        pos = self.rig_to_world([t.x_mm, t.y_mm, 0.0])
        pos[2] = (t.above_mm + TAG_THICK_MM / 2) / 1000.0
        spec.worldbody.add_geom(
            name=f"tag_{t.tag_id}", type=self._mj.mjtGeom.mjGEOM_BOX,
            size=[half, half, TAG_THICK_MM / 2000.0],
            pos=pos.tolist(), material=f"{name}_mat",
            contype=0, conaffinity=0)

    def pose(self, ticks: dict[int, int]) -> None:
        """Put the modelled arm at a pose in calibrated ticks."""
        self._mj.mj_resetData(self.model, self.data)
        for i, tick in ticks.items():
            self.data.qpos[self._adr[i]] = self.twin.qpos_of(i, tick)[0]
        self._mj.mj_forward(self.model, self.data)

    def _cell_tool(self, ticks: dict[int, int]) -> np.ndarray:
        self.pose(ticks)
        return np.array(self.data.body(f"arm_{self._tool}").xpos)

    def _rig_tool(self, ticks: dict[int, int]) -> tuple[float, float, float]:
        q = self.twin._rest_qpos.copy()
        for i, tick in ticks.items():
            q[self.twin._adr[i]] = self.twin.qpos_of(i, tick)[0]
        return self.rig.tool_point(q)

    # --------------------------------------------------------------- camera

    def camera_names(self) -> list[str]:
        return [self._mj.mj_id2name(self.model, self._mj.mjtObj.mjOBJ_CAMERA,
                                    i) for i in range(self.model.ncam)]

    def render(self, camera: str = "cam_bench", width: int = 1280,
               height: int = 720) -> np.ndarray:
        """An RGB frame from a named camera. Offscreen — no window."""
        if camera not in self.camera_names():
            raise ValueError(f"no camera {camera!r}; have "
                             f"{self.camera_names()}")
        r = self._mj.Renderer(self.model, height=height, width=width)
        try:
            r.update_scene(self.data, camera=camera)
            return r.render()
        finally:
            r.close()

    def detect(self, frame: np.ndarray):
        """Tag detections in a frame — the SAME detector campreview uses.

        Deliberately the same one: a sim-only detector would prove the
        renderer draws something tag-shaped, not that the pipeline the
        bench runs can read it.
        """
        import cv2
        import pupil_apriltags as pa

        gray = (frame if frame.ndim == 2 else
                cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY))
        return pa.Detector(families="tag36h11").detect(gray)


# --------------------------------------------------------------------

DEMO_TAGS = (Tag(0, 260.0, 0.0), Tag(1, 200.0, 130.0),
             Tag(2, 200.0, -130.0))


def cmd_render(out: str, tags: tuple[Tag, ...] = ()) -> int:
    import cv2

    cam = SimCam(tags)
    err, dis = cam.verify_frame()
    print(f"frame check: round-trip {err:.4f} mm, fitted yaw "
          f"{cam.yaw_deg:.3f} deg (derived agrees to {dis:.4f} deg)")
    print(f"cameras: {cam.camera_names()}")
    frame = cam.render()
    cv2.imwrite(out, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    print(f"wrote {out}  {frame.shape[1]}x{frame.shape[0]}")
    if tags:
        found = cam.detect(frame)
        print(f"detected {len(found)}/{len(tags)} tag(s):")
        for det in sorted(found, key=lambda d: d.tag_id):
            print(f"  id {det.tag_id} at pixel "
                  f"({det.center[0]:.0f}, {det.center[1]:.0f})")
        missing = sorted({t.tag_id for t in tags} -
                         {d.tag_id for d in found})
        if missing:
            print(f"  MISSED: {missing}")
            return 1
    return 0


def selftest() -> int:
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}"
              f"{'  ' + detail if detail else ''}")
        if not cond:
            fails.append(name)

    print("the generated marker is a real tag36h11")
    cam0 = SimCam()
    img = tag_image(7)
    found = cam0.detect(img)
    check("a generated tag detects as itself",
          len(found) == 1 and found[0].tag_id == 7,
          f"{[d.tag_id for d in found]}")

    print("\nthe rig <-> cell frame bridge")
    err, dis = cam0.verify_frame()
    check("rig->world round-trips against the cell model", err < 0.01,
          f"worst {err:.5f} mm")
    check("the fitted yaw agrees with the derived one", dis < YAW_AGREE_DEG,
          f"fitted {cam0.yaw_deg:.4f} vs derived "
          f"{cam0.cell.arm_pose[2] - cam0.twin.reach_yaw_deg():.4f} deg")
    back = cam0.world_to_rig(cam0.rig_to_world([250.0, -40.0, 120.0]))
    check("world_to_rig inverts rig_to_world",
          np.allclose(back, [250.0, -40.0, 120.0], atol=1e-6),
          f"{np.round(back, 6)}")
    # The table top being world z=0 is assumed by every tag placement.
    # It is checkable: meshcheck measured the m1 centre 62.4 mm above the
    # arm's mounting plane, so the anchor height IS that number if the
    # table is at zero.
    check("the table surface is world z = 0",
          abs(cam0.origin[2] - 0.0624) < 0.001,
          f"m1 anchor sits {cam0.origin[2] * 1000:.1f} mm up; meshcheck "
          f"measured the m1 centre 62.4 mm above the mounting plane")

    print("\nthe camera is aimed where cell.json says it is")
    check("the bench camera exists in the model",
          "cam_bench" in cam0.camera_names(), str(cam0.camera_names()))
    backend = os.environ.get("MUJOCO_GL", "(default)")
    try:
        frame = cam0.render(width=640, height=360)
    except Exception as exc:
        # Loud, not skipped. Everything after this point is rendering,
        # so a quiet pass here would report a working perception
        # pipeline on a machine that cannot draw a single frame.
        print(f"  FAIL offscreen rendering is unavailable (MUJOCO_GL="
              f"{backend}): {type(exc).__name__}: {exc}")
        print("       On a headless Linux box this module sets MUJOCO_GL=egl "
              "automatically;")
        print("       if that failed, EGL is missing. Every render check "
              "below is UNTESTED.")
        fails.append("offscreen rendering is unavailable")
        return 1
    check(f"it renders something that is not a void [MUJOCO_GL={backend}]",
          frame.shape == (360, 640, 3) and 20 < frame.mean() < 235,
          f"mean brightness {frame.mean():.1f}")

    print("\ntags on the table, rendered and read back")
    cam = SimCam(DEMO_TAGS)
    frame = cam.render()
    found = {d.tag_id: d for d in cam.detect(frame)}
    for t in DEMO_TAGS:
        check(f"tag {t.tag_id} at rig ({t.x_mm:.0f}, {t.y_mm:.0f}) is seen",
              t.tag_id in found,
              f"pixel ({found[t.tag_id].center[0]:.0f}, "
              f"{found[t.tag_id].center[1]:.0f})" if t.tag_id in found
              else "NOT DETECTED")
    check("no phantom detections", len(found) <= len(DEMO_TAGS),
          f"{sorted(found)}")

    print("\nthe pipeline notices when the tag is not there")
    # An empty scene must yield nothing. Without this the detection
    # checks above would still pass against a detector that returned
    # everything it was asked about.
    check("an empty table detects no tags", not cam0.detect(cam0.render()),
          "")

    print()
    if fails:
        print(f"FAILED: {len(fails)}")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("simcam OK")
    return 0


def main() -> int:
    args = sys.argv[1:]
    if not args or args[0] == "selftest":
        return selftest() if args else _usage()
    if args[0] == "render":
        return cmd_render(args[1] if len(args) > 1 else "simcam.png")
    if args[0] == "tags":
        return cmd_render(args[1] if len(args) > 1 else "simcam_tags.png",
                          DEMO_TAGS)
    return _usage()


def _usage() -> int:
    print("usage: python -m sim.simcam render [OUT.png]")
    print("       python -m sim.simcam tags   [OUT.png]")
    print("       python -m sim.simcam selftest")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
