# ⚠ THIS IS NOT THE BENCH ARM

**This package models the SO-ARM100. The arm on the bench is an
SO-ARM101 Pro follower. They are different robots.**

Nothing in TendWright loads this as a model. The live model is
`sim/assets/so101/so101_new_calib.xml`, referenced by `sim/twin.py` as
`MODEL_XML`.

The two loadable XMLs here are renamed `*.WRONG-ARM-DO-NOT-LOAD.xml` so
that loading one by accident fails outright rather than quietly
producing collision predictions for a robot we do not own. Do not
rename them back. `uv run python -m sim.meshcheck selftest` asserts
that convention still holds.

## Then why is it still here?

Because it has a job now. It is the **negative control** for
`sim/meshcheck.py`.

`meshcheck` answers the question plan #670 exists to settle — *is the
model actually the arm we built?* — by comparing the vendored meshes
against `hardware/so101-print/individual/`, the STLs Kyle actually sent
to the printer. But a check that has only ever passed proves nothing.
The only way to show it can tell one arm from another is to aim it at a
different arm:

| | worst disagreement |
|---|---|
| printed vs vendored **SO-101** | **0.09%** |
| printed vs this **SO-100** package | **3.88%** |

That 43× separation is what justifies the 2% threshold, and the
selftest asserts both ends against these files. **Delete this package
and that claim becomes unverifiable** — `meshcheck` will print a loud
`** NOT CHECKED **` warning rather than skipping quietly, but the
evidence is gone either way.

Six STLs in `assets/` carry that load: `Base`, `Upper_Arm`,
`Lower_Arm`, `Rotation_Pitch`, `Moving_Jaw`, `Wrist_Pitch_Roll`. The
rest of the package is inert and could go; it is kept together because
splitting a vendored upstream package is how provenance gets lost.

## Provenance

MuJoCo Menagerie, `trs_so_arm100`, Apache-2.0. `LICENSE`, `README.md`
and `CHANGELOG.md` are upstream's and are left untouched — this file is
the only addition, so the package stays a faithful copy of what was
vendored.

Kyle's call, 2026-07-27: *"it does not hurt to keep those files around
as long as they are clearly marked not in use / wrong model somehow."*
