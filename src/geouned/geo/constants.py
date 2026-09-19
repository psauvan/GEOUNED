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

MAX_DEFEATURE_VOLUME_REL_CHANGE = 0.01
"""Gdefeature's own volume-conservation safety net -- reject a healed
result whose Volume differs from the input by more than 1% relative.
Added after a real, dangerous false-pass was found live (2026-08-27,
"beltline left.stp" at the default rel_tol=1e-4): find_short_edges'
own short-edge signature can, on a "half" model with a mirror-symmetry
cut, flag a real symmetry-cut plane alongside a genuine sliver (both
touch the same short edge) -- BRepAlgoAPI_Defeaturing then "successfully"
removed both, IsDone()==True, the result topologically valid AND with
zero remaining short edges (passing every check that existed before this
one) -- while silently DOUBLING the solid's own volume. Every previously-
confirmed *legitimate* repair on this same fixture changed volume by at
most ~0.11%, several orders of magnitude below this bound -- 1% is a
generous, safe margin for a real defect repair (which, by definition,
targets near-zero-volume slivers) while reliably catching a runaway case
like this one. Used by all 3 backends (`freecad`'s own `Gdefeature` is a
real, non-stub implementation -- `Part.Shape.defeaturing()` -- unlike
most of its sibling repair functions)."""

MAX_SPLIT_RING_VOLUME_REL_CHANGE = 3.0e-4
"""Gcollapse_split_rings' own (tighter) volume-conservation net. Unlike
Gdefeature -- whose target slivers can carry a real fraction of a "half"
model's volume, hence its generous 1% -- a split-ring collapse removes
only micron-scale riser bands, so a genuine repair conserves volume to
~1e-4 or better (barrel bottom.stp: 6.4e-5). A larger drift means the
re-sew moved a real adjacent surface: LR.stp (a cylindrical
collapsed-step) comes back valid, dV 7e-4, and CSG-broken (d1suned tally
0.0, 24 lost particles) -- caught by this bound, not by is_valid()."""

MAX_SLIVER_HEAL_VOLUME_REL_CHANGE = 5.0e-4
"""Gsliver_heal's volume-conservation net -- the sole numeric gate (no
`count_split_ring_pairs` check: this repair's own planar cap is a thin
annulus whose two coplanar rims that metric would false-count). With the
`_retrim_freed_quadrics` step, a correct heal conserves volume to ~1e-6
(LR.stp: healed dV 8.6e-7, d1suned tally 0.9985 +/- 0.28%, 0 lost
particles -- the input's own translation was tally 0.0 / 24 lost). A
genuinely wrong fabricated-cap result is ~1e-2, so 5e-4 has ~3 orders of
margin on the good side and ~1.5 on the bad side."""

MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE = 1.0e-3
"""Gheal_topology's volume-conservation gate. Looser than the sliver/
split-ring gates: a failed BOP split can leave the fragment's volume
slightly *inflated* (spurious overlap), and the STEP serialize->deserialize
rebuild that heals it *corrects* that inflation -- so the healed volume
legitimately differs from the (already-wrong) input by more than float
noise. Confirmed on L4_body.stp's Gsplit-#24 fragment: input vol
142946.158 (inflated), healed vol 142946.121 (the true base), dV ~2.6e-7
-- still 3+ orders inside this bound. A genuinely lossy heal (STEP
dropping a real face) would be percent-scale and is rejected."""

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

DEGENERATE_SOLID_VOLUME_FLOOR = 1.0e-2
"""Absolute (mm^3) floor below which a BOPAlgo split fragment is treated
as a degenerate artifact of the cut rather than a real, independent
solid worth decomposing further. Used by `solid_defects.valid_solid`,
the canonical copy of the check `decom_utils_generator.py` carried
(`Vol_tol = 1e-2`) until it was deleted in ef0077c; the `freecad`
backend's own local `valid_solid` copy is replaced by an import of that
one. Restores the pre-ef0077c behaviour whose loss let a near-tangent
cut on `esfera/Barrel_bottom.stp` split off a 16 mm^3 / Volume/Area
~1e-5 sliver, propagate a non-volume-conserving decomposition, and lose
10 MCNP particles."""

DEGENERATE_SOLID_VOL_AREA_RATIO = 1.0e-3
"""Volume/Area ratio (mm) below which a BOPAlgo split fragment is a thin
sliver, not a real piece -- a 16 mm^3 solid spread over 1.4e6 mm^2 of
surface is not a fragment the decomposition should keep. Companion of
`DEGENERATE_SOLID_VOLUME_FLOOR`; both from the historical
`decom_utils_generator.valid_solid` (`Vol_area_ratio = 1e-3`)."""

SAME_SURFACE_AXIS_ANGLE_TOL = math.acos(0.99999)
"""Maximum angle (rad, ~4.47e-3 = 0.256 degrees) between two axes (or two
plane normals) for `surface_geometry`'s `is_same_*_surface` predicates to
treat them as the same direction, either way round.

This is the historical `abs(axis_1.dot(axis_2)) >= 0.99999` threshold
written as the angle it really is. The dot-product form hides its own
size: `1 - cos(theta)` is quadratic in `theta`, so a "1e-5" dot tolerance
is an angle ~45x larger than `Tolerances.pln_angle`/`cyl_angle`'s own
1e-4 rad default, i.e. ~4 mm of deviation per metre of extent. The
value is deliberately UNCHANGED (behavior-preserving refactor); tightening
it is a separate decision that needs a corpus-wide differential scan."""


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
LENGTH_TOL_E9 = 1.0e-9

# Angle tolerance (rad) between two directions (also used for the sine of it: |unit x unit|).
ANGLE_TOL_5E2 = 5.0e-2
ANGLE_TOL_E4 = 1.0e-4
ANGLE_TOL_E3 = 1.0e-3
ANGLE_TOL_E5 = 1.0e-5
ANGLE_TOL_E6 = 1.0e-6
WINDING_ANGLE_TOL = 2.0e-3

# Dimensionless deviation of a unit-vector dot product from 0 or 1 (|cos| test). Quadratic in the
# angle near parallel (~theta^2/2), linear near perpendicular.
DIR_TOL_E4 = 1.0e-4
DIR_TOL_E5 = 1.0e-5
DIR_TOL_E6 = 1.0e-6

# Minimum |cos| between two axes to call them the same line (cosine form of an angle tolerance).
AXIS_COS_MIN_E5 = 0.99999
AXIS_COS_MIN_E6 = 0.999999

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

# Dimensionless relative tolerance (fraction of a model/solid scale or of a volume).
REL_TOL_E2 = 1.0e-2
REL_TOL_E3 = 1.0e-3
REL_TOL_E4 = 1.0e-4
REL_TOL_E5 = 1.0e-5
REL_TOL_E6 = 1.0e-6

# Absolute volume floor (mm^3) below which a piece is discarded as empty.
VOLUME_MIN_E3 = 1.0e-3
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

# Corner-to-corner tolerance (mm) when two bounding boxes are compared. Deliberately NOT POINT_POINT_TOL:
# OCCT bounding boxes carry a few 1e-6 mm of slop (55 of 386 corpus comparisons fall in 1e-6..1e-5), so this
# value decides real outcomes and stays at its historical 1e-6.
BOX_TOL_E6 = 1.0e-6
