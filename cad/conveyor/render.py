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
    for path, colour in items:
        tris = read_stl(path)
        if len(tris):
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
BELT = (58, 58, 62)
SOLO = (196, 194, 186)

SCENES = {
    "straight": [(P("cs_brackets"), BRACKET), (P("cs_rollers"), ROLLER),
                 (P("cs_bed"), BED), (P("cs_belt"), BELT)],
    "corner": [(P("cc_brackets"), BRACKET), (P("cc_rollers"), ROLLER),
               (P("cc_bed"), BED), (P("cc_belt"), BELT)],
    "L": [(P("cs_brackets"), BRACKET), (P("cs_rollers"), ROLLER),
          (P("cs_bed"), BED), (P("cs_belt"), BELT),
          (P("cc_brackets"), BRACKET), (P("cc_rollers"), ROLLER),
          (P("cc_bed"), BED), (P("cc_belt"), BELT)],
    "roller_drive": [(P("roller_drive"), ROLLER)],
    "roller_nose": [(P("roller_nose"), ROLLER)],
    "bracket": [(P("bracket_straight"), BRACKET)],
    "bracket_corner": [(P("bracket_corner_infeed"), BRACKET)],
    "motor_mount": [(P("motor_mount"), BRACKET)],
}

VIEWS = {
    "roller_drive": (-0.55, 0.75, -0.38),
    "roller_nose": (-0.55, 0.75, -0.38),
    "bracket": (-0.10, 1.0, -0.22),
    "motor_mount": (-0.45, 0.9, -0.35),
}

SIZES = {
    "L": (1400, 720),
    "roller_drive": (800, 620),
    "roller_nose": (800, 620),
    "bracket": (1200, 480),
    "motor_mount": (760, 620),
}

want = sys.argv[1:] or list(SCENES)
for name in want:
    render(SCENES[name], os.path.join(RENDERS, name + ".png"),
           size=SIZES.get(name, (1100, 760)),
           forward=VIEWS.get(name, (-0.42, 1.0, -0.46)))
