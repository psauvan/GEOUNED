"""
geo/constants.py

Every SCREAMING_SNAKE_CASE tuning constant used by the `geo` package's
native-backend implementations (`freecad`/`occ`/`ocp`), grouped in one
place instead of being duplicated verbatim (or scattered, one per file)
across them. Several of these -- everything but `MIN_SLIVER_EDGE_LENGTH`/
`DEGENERATE_EDGE_LENGTH_FLOOR`, used by the engine-agnostic
`solid_defects.py` too -- exist only where at least one of the 3 backends
has a real (non-stub) use for them; `freecad`'s own simpler repair
cascade (most of `Gcollapse_split_rings`/`Gsliver_heal`/`Gheal_topology`
are `None`-returning stubs there) means it only ever imports a subset.

Pure Python, no native import -- identical across all 3 engines, exactly
like `vector_geometry.py`/`surface_geometry.py`/`solid_defects.py`.
"""

from __future__ import annotations

import math

MIN_SLIVER_EDGE_LENGTH = 1.0e-3
"""Absolute floor (mm) for find_short_edges' own threshold -- per direct
user instruction: the effective threshold must never drop below this,
even for a solid whose own BoundBox diagonal is small enough that
`rel_tol * diagonal` alone would push it below the model's own working
geometric tolerance (e.g. a tiny decomposed piece), which would make the
detector unable to catch even a genuinely near-zero-length degenerate
edge there."""

DEGENERATE_EDGE_LENGTH_FLOOR = 1.0e-9
"""Lower floor (mm): an edge shorter than this is treated as a
legitimate OCCT *degenerate* edge (a pole singularity on a closed
sphere/cone, where the surface parametrization collapses to a single
point) rather than a genuine CAD defect -- confirmed live, 2026-08-27,
`testing/inputSTEP/Torus/face2.stp` and `tank.stp`: both real,
long-working fixtures have real sphere faces whose own pole edges
measure ~7.7e-15mm (floating-point noise around a mathematically exact
zero, not a real gap), which find_short_edges' own detection wrongly
flagged as corrupted before this floor was added -- a real false
positive that broke `tests/test_cadtocsg.py` outright (the new
"stop"-by-default behavior halted on 2 previously-clean files). Per
direct user instruction, set well below the smallest genuine defect
found anywhere in this project's own corpus work (~4e-4mm, Decomposed/
modelcell_cut1_v2_piece66.stp's own real sliver) while still staying
comfortably above the ~1e-15 floating-point noise floor -- 1e-9 keeps
6 orders of magnitude of margin on the noise side and 5 on the real-
defect side."""

MAX_REPAIR_VOLUME_REL_CHANGE = 3.0e-4
"""Volume-conservation gate shared by every HEALING repair -- `Gdefeature`, `Gcollapse_split_rings`, `Gsliver_heal`,
`Gheal_topology`, `_close_open_solid` and `_repair_non_manifold_solid`'s separate-components pass: a repaired solid is only
trusted if `|dV| / max(|V|, 1) <= 3e-4`. Valid topology alone is NOT enough (a repair can come back BRepCheck-valid and
still lose or invent material, or distort a real neighbouring surface).

One value since 2026-09-19 (it used to be 1e-2, 3e-4, 5e-4 and 1e-3, one per repair). Measured over the 143 test_models files
plus the 30 working_solids fixtures, the relative volume change of every repair each gate evaluated:
  - Gcollapse_split_rings: accepted 6.4e-5 (barrel bottom) .. ~1e-4; rejected 7.0e-4 (LR.stp, TVA_red: valid but CSG-broken,
    d1suned tally 0.0 and 24 lost particles);
  - Gsliver_heal: accepted <= 2.0e-4; rejected >= 5.18e-4 (part2), then 7.7e-4 .. 1.8e-3, 0.17, 0.33 and 1.0;
  - Gdefeature: the 3 measured cases lose ALL the volume (1.0) and are rejected. No legitimate case was observed: the
    historical ones (~1.1e-3, beltline left.step) stop at load and never reach the gate, so their fate at 3e-4 is unknown;
  - Gheal_topology, open-solid repair, non-manifold reconstruction: <= 2e-5, more than 10x below the gate.
Any value in (2.0e-4, 5.18e-4) keeps every measured decision. Caveats: "accepted"/"rejected" is what the previous gates
decided, not ground truth, and the high side of that window is narrow (part2 sits 3.5% above the old 5e-4).

Deliberately NOT applied to `Gmerge_coplanar_planes` (1e-6) nor to Gsplit's `volume_tolerance` (1e-6): those confirm an
(almost) exact operation and stay tight."""

OCCT_FIX_TOLERANCE = 1.0e-6
"""Fixed (never model-scaled) tolerance for native repair/unify calls
whose own algorithm is confirmed crash-prone when given a loose,
geometry-derived tolerance instead -- `ShapeUpgrade_UnifySameDomain.
SetLinearTolerance` specifically (2026-08-30, Solidos/working_solids/
L4-WCS_3.stp solid 75: `unify.Build()` segfaulted -- Windows access
violation, uncatchable by Python -- when given `dist_tol`, a value
scaled to the solid's own BoundBox diagonal; switching to this fixed,
tight value avoided it with zero measurable volume change). Matches the
2 other `ShapeUpgrade_UnifySameDomain` call sites in `occ`/`ocp`
(`_native_fix`, `GSolid.refine()`), both already proven stable across
this whole project's history -- neither ever overrides the linear
tolerance at all, relying on OCCT's own shape-intrinsic default, which
this constant approximates. Only for calls whose own tolerance argument
does not need to track a real physical gap (contrast
`Gcollapse_split_rings`' `sew_tol`, which must scale with the actual gap
it is welding, or `Gdefeature`/`near_surface_pair`'s own tolerance
parameters -- none of those are implicated in this crash class and must
stay variable). `occ`/`ocp` only -- `freecad` has no equivalent native
call."""

OPEN_SEAM_REL_TOL = 1.0e-5
"""Relative (x the solid's own BoundBox diagonal) tolerance below which
two free edges of an open post-split solid are treated as a *doubled
seam* (one trimming-surface boundary curve emitted twice by BOPAlgo,
once per adjacent face -- typical of a plane tangent / near-tangent to a
cylinder), rather than a genuinely missing face. Floored at
`MIN_SLIVER_EDGE_LENGTH` so a tiny decomposed fragment's own small
diagonal can't push it below the model's working geometric tolerance.
Confirmed live (2026-09-04, `Test RoundCorners/rc1.stp`'s RoundCorner
`surf.shape` wedge, ~72mm diagonal): the two duplicate 50mm free edges
sat 4.2e-4mm apart, with 4.2e-4mm micro connector edges bridging the
doubled vertices on the caps -- `rel * 72 = 7.2e-4` (or the 1e-3 floor)
comfortably brackets that while staying orders below any real feature.
Used by `_diagnose_open_solid`/`_close_open_solid` (`occ`/`ocp` only)."""

OPEN_STRIP_GAP_ABS = 0.5
OPEN_STRIP_GAP_REL = 5.0e-3
"""Absolute (mm) and relative (x BoundBox diagonal) ceiling on the width
of a *narrow uncapped slot* -- a thin missing strip face left when a
sliver face is removed from a shell without re-capping. This is the
wider sibling of `OPEN_SEAM_REL_TOL`: same "two near-parallel free edges
a small distance apart, bridged by short connectors" shape, but the gap
is genuinely visible geometry (tenths of a mm), not a sub-micron doubled
seam, so it must additionally be *thin relative to its own length*
(< `OPEN_STRIP_THIN_RATIO` x the strip length) to be trusted as a slot
rather than a real missing face. Confirmed live (2026-09-05,
`rev_pipe.stp` decomposition piece, ~147mm diagonal): a 0.071mm x 10mm
slot beside an R=6 pipe -- 2 near-parallel 10mm free edges 0.071mm apart
(one on the cylinder face, one on a plane), plus a 0.071mm connector
arc. Repair: re-sew every face at a few x the slot width so the two
sides weld into one shared edge. `occ`/`ocp` only."""

OPEN_STRIP_THIN_RATIO = 0.1
"""A candidate uncapped slot's width must be below this fraction of its
paired long free edges' own length to be trusted as a slot (a thin
strip) rather than a genuinely missing full face. See
`OPEN_STRIP_GAP_ABS`."""

DEFAULT_MIN_SOLID_VOLUME = 1.0e-2
"""Default of `Tolerances.min_solid_volume` (mm^3): the smallest volume of a piece worth keeping. ONE value for every place that
discards a piece as too small -- `solid_defects.valid_solid`, `Gsplit`'s fragment filter, `_raw_bop_split`'s reconstructed
fragment and `space_decomposition`'s subregions (they used to be 1e-2, 1e-3, 1e-3 and 1e-3, and `Gsplit` applied two of them in
sequence, so 1e-2 was the one that actually decided). It is a THRESHOLD on one solid; contrast `VOLUME_REF`, the scale of the
relative volume comparisons.

Measured 2026-09-19 on test_models + working_solids: no piece reaches those checks with a volume between 1e-3 and 0.1 mm^3 (21
fragments below 1e-3 are rejected either way; the smallest real pieces are ~0.1 mm^3), so unifying at 1e-2 changes no decision
there. The smallest legitimate piece known is 0.072 mm^3 (Decomposed/modelcell_cut1_v2_piece66.stp), 7x above this value.

History of the check `valid_solid` carries (`Vol_tol = 1e-2` in `decom_utils_generator.py` until ef0077c): its loss let a
near-tangent cut on `esfera/Barrel_bottom.stp` split off a 16 mm^3 / Volume/Area ~1e-5 sliver, propagate a non-volume-conserving
decomposition, and lose 10 MCNP particles -- the Volume/Area check (`DEGENERATE_SOLID_VOL_AREA_RATIO`) is the one that catches it."""

DEGENERATE_SOLID_VOL_AREA_RATIO = 1.0e-3
"""Volume/Area ratio (mm) below which a BOPAlgo split fragment is a thin
sliver, not a real piece -- a 16 mm^3 solid spread over 1.4e6 mm^2 of
surface is not a fragment the decomposition should keep. Companion of
`DEFAULT_MIN_SOLID_VOLUME`; both from the historical
`decom_utils_generator.valid_solid` (`Vol_area_ratio = 1e-3`)."""


# ===========================================================================
# Tolerance constants, one per (role, historical value).
#
# Every hard-coded tolerance literal that used to live inline in GEOUNED/ and geo/ is
# named here by ROLE plus the exponent of the value it had (LENGTH_TOL_E5 == 1e-5 mm).
# The suffix is deliberately visible: unifying a role's values is then a one-line change
# per constant (point every LENGTH_TOL_E* at the same number) that can be bisected on its
# own. These are the future fields of Tolerances.
# ===========================================================================

# Absolute length tolerance (mm): two points/edges/vertices coincide, a length is negligible.
LENGTH_TOL_E12 = 1.0e-12
LENGTH_TOL_E3 = 1.0e-3
LENGTH_TOL_E5 = 1.0e-5
LENGTH_TOL_E6 = 1.0e-6
LENGTH_TOL_E7 = 1.0e-7
LENGTH_TOL_E8 = 1.0e-8

# Angle tolerance (rad) between two directions (also used for the sine of it: |unit x unit|).
ANGLE_TOL_5E2 = 5.0e-2
ANGLE_TOL_E4 = 1.0e-4
ANGLE_TOL_E3 = 1.0e-3
ANGLE_TOL_E6 = 1.0e-6
WINDING_ANGLE_TOL = 2.0e-3

# Tolerance (rad) on surface (U, V) parameters and arc angles.
PARAM_ANGLE_TOL_E4 = 1.0e-4
PARAM_ANGLE_TOL_E5 = 1.0e-5

# Numerical-zero floor of an intermediate quantity (denominator, slope, curvature, ...): guards a
# division or a degenerate branch, not a geometric tolerance.
ZERO_TOL_E10 = 1.0e-10
ZERO_TOL_E12 = 1.0e-12
ZERO_TOL_E6 = 1.0e-6
ZERO_TOL_E8 = 1.0e-8
ZERO_TOL_E9 = 1.0e-9

# Absolute floor (mm) for a *relative* surface-matching tolerance (`Tolerances.relativeTol=True`). Those tolerances
# are `rel * scale`, which is exactly 0 for a surface anchored at the origin (or at another zero-scale reference) --
# and a tolerance of 0 rejects even two bit-identical surfaces (`|0| < 0` is false), so e.g. every plane z=0 would
# get its own surface card. 1e-9 mm is far above float noise on any realistic coordinate (~2e-11 mm at 1e5 mm) and
# far below any real geometric difference.
RELATIVE_TOL_ABS_FLOOR = 1.0e-9

# Dimensionless relative tolerance (fraction of a model/solid scale or of a volume).
REL_TOL_E2 = 1.0e-2
REL_TOL_E3 = 1.0e-3
REL_TOL_E4 = 1.0e-4
REL_TOL_E5 = 1.0e-5
REL_TOL_E6 = 1.0e-6

# Kernel zero (mm^3): a boolean Common of two shapes has content only above this.
VOLUME_MIN_E8 = 1.0e-8

# Tolerance handed to a CAD-kernel operation (fix, sewing, split) or a bound on one.
KERNEL_TOL_E13 = 1.0e-13
KERNEL_TOL_E3 = 1.0e-3
KERNEL_TOL_E6 = 1.0e-6
KERNEL_TOL_E7 = 1.0e-7
KERNEL_TOL_E8 = 1.0e-8
SPLIT_TOL_MAX = 0.1
SPLIT_TOL_MIN = 1.0e-12

# Algorithmic decision thresholds (not tolerances) that used to be inline literals.
DEFAULT_SPLIT_SCALE = 0.1
FINITE_DIFF_STEP = 0.001
MESH_DEFLECTION = 0.1
NOT_PERPENDICULAR_COS_MIN = 0.1
SIDE_FRACTION_GAP_MIN = 0.15

# Default of Tolerances.min_face_width, duplicated as a bare default in several signatures.
DEFAULT_MIN_FACE_WIDTH = 0.1


# Absolute distance (mm) below which two points (vertices, apexes, edge endpoints, centres of mass) are the
# same point. Measured on the test_models corpus (143 files): real coincidences are exactly 0 or < 1e-9 mm and
# distinct points are >= 1e-2 mm, so any value in [1e-8, 1e-5] behaves identically there; 1e-5 keeps a 10x
# margin over the rounding of a STEP written with 6 decimals and stays 10x below Tolerances' surface tolerances.
POINT_POINT_TOL = 1.0e-5

# Corner-to-corner tolerance (mm) when two bounding boxes are compared. Same value as POINT_POINT_TOL, kept as its
# own name because box slop and point coincidence are different physics: OCCT bounding boxes carry a few 1e-6 mm of
# slop (55 of 386 corpus comparisons fall in 1e-6..1e-5) and the old 1e-6 split those cases. Measured 2026-09-19:
# with 1e-5 the test_models corpus (143 files) is identical in pieces, volumes, primitive surfaces and composite
# counts, so the two are unified.
BOX_TOL = POINT_POINT_TOL

# Axis tilt (rad, acos(1 - 1e-5) ~ 4.5e-3) under which the load-time DEFECT detectors (`near_surface_pair`,
# `count_split_ring_pairs`) still call two axes parallel. Deliberately looser than the surface-identity angles
# (Tolerances.*_angle, NUMERIC_TOL): they look for surfaces/circles that are NEARLY coincident but not identical (a
# duplicated micro-trim), so they must reach beyond what identity accepts (57 corpus comparisons fall between 0.26 and
# 2.5 degrees). One value for both detectors.
DEFECT_AXIS_ANGLE = math.acos(1.0 - 1.0e-5)

# Planarity of a BSpline edge (`spline_2D`): the binormal (tangent x normal) at every knot must stay parallel to the one at the first
# knot, to within this angle. A spline that stores a planar curve carries fitting error in its binormal, so exact parallelism is not to
# be expected. Measured 2026-09-20 on test_models + working_solids (187 splines reaching the test): the 32 planar ones lie between 0 and
# 1e-3 rad (17 at <= 1e-12, the other 15 spread over (1e-8, 1e-3]); the 155 non-planar ones start right above (27 in (1e-3, 1e-2], 56 in
# (1e-2, 1e-1], 72 beyond). A property of how the CAD stores the curve, not something the user tunes; it is the value `is_parallel`'s
# default gave here before.
SPLINE_PLANARITY_ANGLE = 1.0e-3

# Numeric precision (mm and rad) of a solid's OWN data: what the CAD kernel returns for entities that touch or coincide inside one
# solid. Measured on test_models + working_solids (2026-09-19/20): every face/edge contact distance is exactly 0 or above 1e-2 mm
# (nothing in between), and a full turn of a periodic parameter closes to floating-point noise. 1e-7 equals OCCT's
# Precision::Confusion. Used for CONTACT (points, edges, faces of one solid) and for the full 2*pi of a periodic parameter. NOT for
# surface identity (the user's per-surface tolerances, always) and NOT to approximate a surface by an axis-aligned one (that is the
# same decision as identity): real data carries ~5e-7 rad of direction noise, above this value.
NUMERIC_TOL = 1.0e-7

# Reference volume (mm^3) of every RELATIVE volume comparison: `abs(a - b) <= tol * max(|reference|, VOLUME_REF)`. Below it a
# relative test is meaningless, so the tolerance becomes the absolute `tol * VOLUME_REF`. It is a characteristic SCALE, not a
# threshold (contrast Tolerances.min_solid_volume, which decides whether a piece is worth keeping) and it must not be tied to
# it: a legitimate repair of a small solid changes its volume by absolute amounts (5e-5 and 8e-5 mm^3 on rev_pipe.stp) that a
# much smaller floor would reject. Measured window for the 3e-4 repair gate: it must accept |dV| = 8.0e-5 and reject the
# 0.17 and 0.20 mm^3 failures (ring.stp, rc17.stp), so VOLUME_REF in (0.27, ~570); 1.0 sits inside (only 4 small-solid cases).
VOLUME_REF = 1.0
