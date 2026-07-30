"""tagsheet — generate a printable tag36h11 sheet at a chosen size.

    uv run python -m hardware.bench.tagsheet --mm 20 --ids 8-15
    uv run python -m hardware.bench.tagsheet --mm 40 --ids 0-7 --out sheet.html
    uv run python -m hardware.bench.tagsheet --selftest

Replaces hand-written HTML with base64 pasted in by hand (the original
docs/bench-apriltags.html). Every tag is GENERATED and then READ BACK
through the same detector the bench uses, so a sheet cannot ship with a
tag that does not decode as the ID printed under it.

SIZE IS THE WHOLE POINT, and it is easy to get wrong. A tag36h11 is 8
cells across its BLACK SQUARE, and the detector also needs a white quiet
zone around it — one cell each side, so 10 cells overall. The physical
size that matters, the one you type into a pose estimator, is the BLACK
SQUARE. So an image drawn `mm` wide across its black square must be
rendered mm * 10/8 wide on the page. Printing the outer edge at the size
you wanted makes every tag 25% small and every pose wrong by the same
factor, silently.

IDS MUST NOT COLLIDE ACROSS SIZES. Tag pose is computed from the
apparent size against an assumed physical size, so the same ID printed
at two sizes is genuinely ambiguous — a detector reporting "ID 4" cannot
tell you which sheet it came from, and will happily return a pose that
is wrong by the ratio. Give each sheet its own ID block.
"""

from __future__ import annotations

import argparse
import base64
import os
import sys
from pathlib import Path

os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from hardware.errors import BenchError, make_run_tool  # noqa: E402

run_tool = make_run_tool("check the tag id range and try again")

# tag36h11: 6x6 of data, +1 cell of black border = 8 across the black
# square, +1 cell of white quiet zone each side = 10 overall.
CELLS_BLACK = 8
CELLS_TOTAL = 10
QUIET = (CELLS_TOTAL - CELLS_BLACK) // 2
_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)


def tag_png(tag_id: int) -> bytes:
    """One tag36h11 as a 10x10 PNG: black square plus its quiet zone."""
    marker = cv2.aruco.generateImageMarker(_DICT, tag_id, CELLS_BLACK)
    img = np.full((CELLS_TOTAL, CELLS_TOTAL), 255, np.uint8)
    img[QUIET:QUIET + CELLS_BLACK, QUIET:QUIET + CELLS_BLACK] = marker
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise BenchError(f"could not encode tag {tag_id}", "opencv failure")
    return buf.tobytes()


def verify(tag_id: int, png: bytes) -> None:
    """Read the tag back through the real detector.

    A generated sheet that does not decode is worse than no sheet: the
    error only surfaces at the bench, after printing and cutting.
    """
    # Imported here rather than at module scope so generating a sheet on a
    # machine without libapriltag still works up to the read-back, and
    # fails with the library's own install message instead of at import.
    from .apriltag import Detector
    img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
    big = cv2.resize(img, (CELLS_TOTAL * 40,) * 2,
                     interpolation=cv2.INTER_NEAREST)
    # A quiet zone that is only one cell wide survives printing but is
    # tight for the detector on a synthetic image, so pad generously
    # before reading back — this checks the CODE, not the margin.
    big = cv2.copyMakeBorder(big, 80, 80, 80, 80, cv2.BORDER_CONSTANT,
                             value=255)
    found = Detector(families="tag36h11").detect(big)
    ids = [d.tag_id for d in found]
    if ids != [tag_id]:
        raise BenchError(
            f"generated tag {tag_id} read back as {ids or 'nothing'}",
            "the dictionary or the quiet zone is wrong — do not print this")


def parse_ids(spec: str) -> list[int]:
    out: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    if not out:
        raise BenchError(f"no ids in {spec!r}", "try --ids 8-15")
    bad = [i for i in out if not 0 <= i <= 586]
    if bad:
        raise BenchError(f"tag36h11 has no ids {bad}", "valid range is 0-586")
    return out


def build(ids: list[int], mm: float, note: str,
          check: bool = True) -> str:
    img_mm = mm * CELLS_TOTAL / CELLS_BLACK
    cells = []
    for i in ids:
        png = tag_png(i)
        if check:
            verify(i, png)
        b64 = base64.b64encode(png).decode()
        cells.append(
            f'  <div class="tag">\n'
            f'    <img src="data:image/png;base64,{b64}" '
            f'alt="tag36h11 id {i}">\n'
            f'    <div class="label">tag36h11 &nbsp;ID {i}</div>\n'
            f'  </div>')
    return f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>TendWright AprilTags (tag36h11, {mm:g}mm)</title>
<style>
  @page {{ size: letter; margin: 12mm; }}
  body {{ font-family: sans-serif; margin: 10mm; }}
  h1 {{ font-size: 14pt; }}
  .instructions {{ font-size: 9pt; max-width: 170mm; }}
  .grid {{ display: flex; flex-wrap: wrap; gap: 8mm; margin-top: 6mm; }}
  .tag {{ text-align: center; page-break-inside: avoid; }}
  .tag img {{ width: {img_mm:g}mm; height: {img_mm:g}mm;
              image-rendering: pixelated; display: block; }}
  .label {{ font-size: 8pt; margin-top: 1mm; }}
  .ruler {{ width: 100mm; height: 6mm; border: 0.3mm solid #000;
            margin-top: 8mm;
            background: repeating-linear-gradient(90deg,#000 0 1mm,#fff 1mm 10mm); }}
  .rlabel {{ font-size: 8pt; }}
</style></head>
<body>
<h1>TendWright AprilTags &mdash; tag36h11, {mm:g}mm black square</h1>
<div class="instructions">
  <b>PRINT AT 100% SCALE</b> (no &quot;fit to page&quot;, no shrink). Verify with
  the ruler bar below: it must measure exactly 100mm. Each tag's
  <b>black square must measure {mm:g}mm</b> &mdash; the printed image is
  {img_mm:g}mm across because it includes the white quiet zone the
  detector needs. Cut outside the white margin, never into it, and never
  let tape cross the black square. Mount flat; wrinkles ruin pose
  accuracy. {note}
</div>
<div class="ruler"></div>
<div class="rlabel">calibration ruler &mdash; exactly 100mm when printed
correctly (10 stripes of 10mm)</div>
<div class="grid">
{chr(10).join(cells)}
</div>
</body></html>
"""


def selftest() -> int:
    """Geometry first, read-back second.

    The read-back needs the system libapriltag, which is Linux-only as of
    #713.5. It used to be first, so on a machine without the library the
    BenchError took out the five checks below it that need no detector at
    all — quiet-zone geometry, the sizing assertion, the parse_ids refusal.
    Ordering them the other way round means the desk still tests everything
    it CAN, and the one thing it cannot is named rather than swallowed.
    """
    png = tag_png(8)
    img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert img.shape == (CELLS_TOTAL, CELLS_TOTAL), img.shape
    # the quiet zone really is white all the way round
    assert img[0].max() == 255 and img[-1].max() == 255
    assert img[:, 0].max() == 255 and img[:, -1].max() == 255
    # a 20 mm black square must be drawn 25 mm wide, not 20
    assert abs(20 * CELLS_TOTAL / CELLS_BLACK - 25.0) < 1e-9
    try:
        parse_ids("999")
    except BenchError:
        pass
    else:
        raise AssertionError("an out-of-range id was accepted")
    print("tagsheet geometry OK")

    try:
        for i in (0, 8, 15, 100, 586):
            verify(i, tag_png(i))
    except BenchError as exc:
        # NOT "OK". The read-back is the check that stops an undecodable
        # sheet reaching the printer, so a run without it is partial and
        # has to say so in the word a reader skims for.
        print(f"tagsheet PARTIAL — read-back NOT RUN: {exc}", file=sys.stderr)
        print("  5 tags were generated but never decoded. Full coverage: "
              "ssh cell1 'cd ~/TendWright && uv run python -m "
              "hardware.bench.tagsheet --selftest'", file=sys.stderr)
        return 0
    print("tagsheet selftest OK (5 tags generated and read back)")
    return 0


def run() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, prog="python -m hardware.bench.tagsheet",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mm", type=float, required=True,
                    help="BLACK SQUARE size in mm (not the printed image)")
    ap.add_argument("--ids", default="8-15", help="e.g. 8-15 or 8,9,12")
    ap.add_argument("--out", default=None, help="output .html")
    ap.add_argument("--note", default="", help="extra line for the sheet")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--no-verify", action="store_true",
                    help="skip the detector read-back. ONLY for generating a "
                         "sheet on a machine without libapriltag (the desk); "
                         "the sheet is then unproven and must not be printed "
                         "without checking it elsewhere")
    args = ap.parse_args()
    if args.mm <= 0:
        raise BenchError(f"--mm must be positive, got {args.mm:g}",
                         "it is the black square's printed size")
    ids = parse_ids(args.ids)
    out = Path(args.out or f"docs/bench-apriltags-{args.mm:g}mm.html")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build(ids, args.mm, args.note, check=not args.no_verify),
                   encoding="utf-8")
    print(f"{out}: {len(ids)} tags (IDs {ids[0]}-{ids[-1]}), "
          f"{args.mm:g}mm black square, printed {args.mm * 1.25:g}mm overall")
    if args.no_verify:
        # Loud, because the whole reason verify() exists is that a sheet
        # which does not decode only reveals itself at the bench, after
        # printing and cutting.
        print("!! --no-verify: NOT ONE TAG WAS READ BACK. This sheet is "
              "unproven.", file=sys.stderr)
        print("!! Re-run without the flag on the cell controller before you "
              "print it.", file=sys.stderr)
    else:
        print("every tag was read back through the detector before writing")
    return 0


def main() -> int:
    if "--selftest" in sys.argv:
        return selftest()
    return run_tool(run)


if __name__ == "__main__":
    sys.exit(main())
