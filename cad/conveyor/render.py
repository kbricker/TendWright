# Headless STL renderer — z-buffered flat shading, PNG out, numpy + stdlib only.
#
# freecadcmd has no GUI, so there is no way to screenshot a model from the
# build script. This reads the exported STLs directly and rasterises them.
#
#   python cad/conveyor/render.py
#
# Deliberately no new dependencies: numpy is already a TendWright dep, and the
# PNG is written by hand with zlib + struct.

import os
import sys
import struct
import zlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = os.path.join(HERE, "parts")
RENDERS = os.path.join(HERE, "renders")
os.makedirs(RENDERS, exist_ok=True)

SS = 2               # supersample factor
AMBIENT = 0.34
LIGHT = np.array([-0.35, -0.62, 0.70])
LIGHT = LIGHT / np.linalg.norm(LIGHT)
BG = (250, 249, 246)


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        blob = f.read(n * 50)
    arr = np.frombuffer(blob, dtype=np.uint8).reshape(n, 50)
    tris = arr[:, 12:48].copy().view(np.float32).reshape(n, 3, 3).astype(np.float64)
    return tris


def basis(forward):
    f = np.asarray(forward, dtype=np.float64)
    f = f / np.linalg.norm(f)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(f, up)) > 0.99:
        up = np.array([0.0, 1.0, 0.0])
    r = np.cross(f, up)
    r = r / np.linalg.norm(r)
    u = np.cross(r, f)
    return f, r, u


def write_png(path, rgb):
    h, w, _ = rgb.shape
    rows = bytearray()
    for y in range(h):
        rows.append(0)
        rows.extend(rgb[y].tobytes())

    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data +
                struct.pack(">I", zlib.crc32(typ + data) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(rows), 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as fh:
        fh.write(png)


def render(items, out, size=(1200, 780), forward=(-0.42, 1.0, -0.46), margin=0.06):
    W, H = size[0] * SS, size[1] * SS
    f, r, u = basis(forward)

    meshes = []
    for item in items:
        path, colour = item[0], item[1]
        offset = item[2] if len(item) > 2 else (0.0, 0.0, 0.0)
        tris = read_stl(path)
        if len(tris):
            tris = tris + np.asarray(offset, dtype=np.float64)
            meshes.append((tris, np.array(colour, dtype=np.float64)))
    if not meshes:
        return

    allv = np.concatenate([t.reshape(-1, 3) for t, _ in meshes], axis=0)
    sx_all = allv @ r
    sy_all = allv @ u
    x0, x1 = sx_all.min(), sx_all.max()
    y0, y1 = sy_all.min(), sy_all.max()
    span_x = max(x1 - x0, 1e-6)
    span_y = max(y1 - y0, 1e-6)
    scale = min(W * (1 - 2 * margin) / span_x, H * (1 - 2 * margin) / span_y)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0

    zbuf = np.full((H, W), np.inf)
    img = np.zeros((H, W, 3), dtype=np.float64)
    img[:, :] = np.array(BG, dtype=np.float64)

    for tris, colour in meshes:
        e1 = tris[:, 1] - tris[:, 0]
        e2 = tris[:, 2] - tris[:, 0]
        nrm = np.cross(e1, e2)
        ln = np.linalg.norm(nrm, axis=1)
        keep = ln > 1e-12
        tris, nrm, ln = tris[keep], nrm[keep], ln[keep]
        nrm = nrm / ln[:, None]

        facing = nrm @ f
        vis = facing < 0.0                      # f points camera -> scene
        tris, nrm = tris[vis], nrm[vis]

        lit = AMBIENT + (1.0 - AMBIENT) * np.clip(nrm @ LIGHT, 0.0, 1.0)

        px = (tris.reshape(-1, 3) @ r - cx) * scale + W / 2.0
        py = H / 2.0 - (tris.reshape(-1, 3) @ u - cy) * scale
        pz = tris.reshape(-1, 3) @ f
        px = px.reshape(-1, 3)
        py = py.reshape(-1, 3)
        pz = pz.reshape(-1, 3)

        for i in range(len(tris)):
            ax, bx, cx3 = px[i]
            ay, by, cy3 = py[i]
            az, bz, cz = pz[i]

            lo_x = int(np.floor(min(ax, bx, cx3)))
            hi_x = int(np.ceil(max(ax, bx, cx3)))
            lo_y = int(np.floor(min(ay, by, cy3)))
            hi_y = int(np.ceil(max(ay, by, cy3)))
            lo_x = max(lo_x, 0); lo_y = max(lo_y, 0)
            hi_x = min(hi_x, W - 1); hi_y = min(hi_y, H - 1)
            if hi_x < lo_x or hi_y < lo_y:
                continue

            area = (bx - ax) * (cy3 - ay) - (by - ay) * (cx3 - ax)
            if abs(area) < 1e-9:
                continue

            X = np.arange(lo_x, hi_x + 1)[None, :] + 0.5
            Y = np.arange(lo_y, hi_y + 1)[:, None] + 0.5

            w0 = (cx3 - bx) * (Y - by) - (cy3 - by) * (X - bx)
            w1 = (ax - cx3) * (Y - cy3) - (ay - cy3) * (X - cx3)
            w2 = (bx - ax) * (Y - ay) - (by - ay) * (X - ax)
            if area > 0:
                mask = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
            else:
                mask = (w0 <= 0) & (w1 <= 0) & (w2 <= 0)
            if not mask.any():
                continue

            depth = (w0 * az + w1 * bz + w2 * cz) / area
            sub_z = zbuf[lo_y:hi_y + 1, lo_x:hi_x + 1]
            hit = mask & (depth < sub_z)
            if not hit.any():
                continue
            sub_z[hit] = depth[hit]
            img[lo_y:hi_y + 1, lo_x:hi_x + 1][hit] = colour * lit[i]

    img = img.reshape(H // SS, SS, W // SS, SS, 3).mean(axis=(1, 3))
    write_png(out, np.clip(img, 0, 255).astype(np.uint8))
    print("wrote", out)


P = lambda n: os.path.join(PARTS, n + ".stl")

BRACKET = (208, 206, 198)
ROLLER = (196, 132, 74)
BED = (74, 126, 178)
RETURN = (108, 152, 120)
BELT = (58, 58, 62)
MOTOR = (118, 120, 124)
ORING = (42, 40, 38)
KEEPER = (186, 148, 96)
SOLO = (196, 194, 186)

STRAIGHT = lambda pfx: [
    (P(pfx + "_brackets"), BRACKET), (P(pfx + "_rollers"), ROLLER),
    (P(pfx + "_bed"), BED), (P(pfx + "_return"), RETURN),
    (P(pfx + "_belt"), BELT), (P(pfx + "_motor"), MOTOR),
]
CURVE = [
    (P("cv_frame"), BRACKET), (P("cv_rollers"), ROLLER),
    (P("cv_orings"), ORING), (P("cv_keeper"), KEEPER), (P("cv_motor"), MOTOR),
]

SCENES = {
    "straight": STRAIGHT("cs"),
    "curve": CURVE,
    "v0": STRAIGHT("cs") + CURVE + STRAIGHT("s2"),
    "motor_mount": [(P("bracket_straight_motor"), BRACKET), (P("ref_motor"), MOTOR)],
    "coupon": [(P("coupon_bracket_end"), BRACKET), (P("ref_motor"), MOTOR)],
    "coupon_curve": [(P("coupon_curve"), BRACKET)],
    # Print orientation, big end down. The idler is shifted so the two fit in one frame.
    "roller_cone": [(P("roller_cone_driven"), ROLLER),
                    (P("roller_cone_idler"), ROLLER, (42.0, 0.0, 0.0))],
    "return_guide": [(P("return_guide_straight"), RETURN)],
    "roller_driven": [(P("roller_driven"), ROLLER)],
    "roller_idler": [(P("roller_idler"), ROLLER)],
    "bracket": [(P("bracket_straight_motor"), BRACKET)],
}

VIEWS = {
    # Motor is on +Y after the mirror, so the camera sits on that side.
    "straight": (-0.35, -0.85, -0.40),
    "curve": (0.25, -0.85, -0.50),
    "v0": (-0.15, -0.55, -0.70),
    # From below the outboard face, so the lower ear is not hidden by the body.
    "motor_mount": (0.72, 0.45, 0.40),
    "coupon": (0.78, 0.35, 0.42),
    "coupon_curve": (0.15, -0.40, -0.88),
    "roller_cone": (-0.45, 0.55, -0.70),
    "roller_driven": (-0.55, 0.75, -0.38),
    "roller_idler": (-0.55, 0.75, -0.38),
    "bracket": (0.15, 0.95, -0.28),
}

SIZES = {
    "straight": (1200, 780),
    "curve": (1200, 1000),
    "v0": (1600, 1100),
    "motor_mount": (1100, 860),
    "coupon": (1100, 860),
    "coupon_curve": (1200, 1000),
    "roller_cone": (1200, 780),
    "roller_driven": (800, 620),
    "roller_idler": (800, 620),
    "bracket": (1200, 560),
}

want = sys.argv[1:] or list(SCENES)
for name in want:
    render(SCENES[name], os.path.join(RENDERS, name + ".png"),
           size=SIZES.get(name, (1100, 760)),
           forward=VIEWS.get(name, (-0.42, 1.0, -0.46)))
