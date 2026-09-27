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

PRE_REPAIR_MIN_VOLUME = 1.0e-5
"""Absolute volume floor (mm^3) for an intermediate solid COMPONENT during a
split -- e.g. `remove_tools_from_raw_solids`' own "is the tool volume
substantial enough to bother comparing" gate -- BEFORE that component has
gone through the full reconstruction/repair cascade. Deliberately lower than
`Tolerances.min_solid_volume` (1e-2, the FINAL-stage "is this solid worth
keeping as output" floor): discarding a small-but-real intermediate piece
this early, at the same threshold the final stage uses, risks breaking the
solid's own later reconstruction (per direct user instruction, 2026-09-23 --
not itself re-derived from a fresh corpus measurement, but kept at its
original historical value, the one this role already had before it was
found mislabelled as the unrelated relative-volume `REL_TOL_E5`)."""


# ===========================================================================
# Tolerance constants, one per (role, historical value).
#
# Every hard-coded tolerance literal that used to live inline in GEOUNED/ and geo/ is
# named here by ROLE plus the exponent of the value it had (LENGTH_TOL_E5 == 1e-5 mm).
# The suffix is deliberately visible: unifying a role's values is then a one-line change
# per constant (point every LENGTH_TOL_E* at the same number) that can be bisected on its
# own. These are the future fields of Tolerances.
# ===========================================================================

# Numeric precision (mm and rad) of a solid's OWN data: what the CAD kernel returns for entities that touch or coincide inside one
# solid. Measured on test_models + working_solids (2026-09-19/20): every face/edge contact distance is exactly 0 or above 1e-2 mm
# (nothing in between), and a full turn of a periodic parameter closes to floating-point noise. 1e-7 equals OCCT's
# Precision::Confusion. Used for CONTACT (points, edges, faces of one solid) and for the full 2*pi of a periodic parameter. NOT for
# surface identity (the user's per-surface tolerances, always) and NOT to approximate a surface by an axis-aligned one (that is the
# same decision as identity): real data carries ~5e-7 rad of direction noise, above this value.
NUMERIC_TOL = 1.0e-7

# The smallest meaningful nonzero value a double-precision arithmetic result can carry --
# below this, a computed quantity (a relative distance, a relative volume difference, a
# cross-product/coefficient/parameter-difference guarding a division or a degenerate
# branch) is numerical round-off, not a real geometric feature. Used as a kernel-operation
# tolerance floor (geo/freecad/split.py's split-retry floor), as the "effectively zero"
# cutoff for relative geometric decisions (VoidBox.piece_enclosure_split's contact/
# containment tests, GeounedSolid.check_intersection's volume-embedding test), for
# near-parallel/degenerate direction tests (GPlane/GLine's own intersect_plane/
# intersect_line), and for every other division/degenerate-branch guard formerly split
# across ZERO_TOL_E9/E10/E12 (RadiusOfGyration, quadratic coefficients, edge/cross-product
# lengths, periodic-parameter differences, tan(semi-angle)) -- all of these are the same
# concept, not independently-tuned tolerances, so they share this one constant.
NUMERIC_DOUBLE_TOL = 1.0e-12

# The former LENGTH_TOL_E* family (E3/E5/E6/E7/E8/E12) is gone entirely, 2026-09-23: every
# site was, under a different name, the same role as an already-established constant
# (MIN_SLIVER_EDGE_LENGTH, POINT_POINT_TOL, KERNEL_TOL_E7, NUMERIC_DOUBLE_TOL, NUMERIC_TOL,
# or -- one site, a cylinder radius comparison -- Tolerances.value), values unchanged where
# the role matched exactly. See CLAUDE.md's own "LENGTH_TOL_E*" entries for the full,
# site-by-site reasoning.

# Zero floor (mm^2, NOT mm) for a squared-length difference before a sqrt (e.g. a
# perpendicular-distance-squared obtained as |v|^2 - (v.axis)^2): this subtraction of two
# large, near-equal mm^2 quantities carries far more catastrophic-cancellation noise than a
# plain length or a unit-vector dot/cross product, so it needs its own, looser floor --
# confirmed 2026-09-22, cone-apex/cylinder-axis distance in build_can_params: real residual
# -7.3e-12 on Cans/fwd_can_1.stp+rev_can_1.stp (raises ValueError: math domain error under
# NUMERIC_DOUBLE_TOL=1e-12), 4 orders of magnitude below this floor; every other near-zero
# sample of the same quantity across the 143-file corpus is genuine noise at 1e-93..1e-62.
SQUARED_LENGTH_TOL_E8 = 1.0e-8

# Angle tolerance (rad) between two directions (also used for the sine of it: |unit x unit|).
ANGLE_THRESHOLD = 5.0e-2
WINDING_ANGLE_TOL = 2.0e-3

# Tolerance (rad, range 0-pi) on a surface (U, V) parameter or an arc/periodic angle -- a
# 1D position/extent in parameter space, NOT the angle between two 3D directions (that is
# ANGLE_TOL_*'s own, separate role). Was two values (1e-4/1e-5) for the same role; unified
# 2026-09-22.
PARAM_ANGLE_TOL = 1.0e-5

# Absolute floor (mm) for a *relative* surface-matching tolerance (`Tolerances.relativeTol=True`). Those tolerances
# are `rel * scale`, which is exactly 0 for a surface anchored at the origin (or at another zero-scale reference) --
# and a tolerance of 0 rejects even two bit-identical surfaces (`|0| < 0` is false), so e.g. every plane z=0 would
# get its own surface card. 1e-9 mm is far above float noise on any realistic coordinate (~2e-11 mm at 1e5 mm) and
# far below any real geometric difference.
RELATIVE_TOL_ABS_FLOOR = 1.0e-9

# Dimensionless relative tolerance (fraction of a model/solid scale or of a volume).
# REL_TOL_E2 removed 2026-09-23 (both its real sites were plain "1% of a real geometric span"
# constructions, not a tolerance as such -- per direct user instruction, replaced with the
# literal 0.01 inline, same convention as this project's other non-tolerance factors).
REL_DIST_TOL = 1.0e-3
"""Relative distance tolerance (fraction of a real parameter/length span). Split out of
`REL_TOL_E3` 2026-09-23 (same value, renamed) for `get_adjacent_cylknesurfFace`'s own "is this
edge's V-parameter close to the cylinder/cone's own axial extreme" classification -- a genuine
relative-DISTANCE tolerance, not a relative-VOLUME one (which is what the rest of the
`REL_TOL_E*` family, still under review, is really about)."""

RESEW_CEILING = 1.0e-3
"""Upper bound (fraction of the solid's own diagonal) on the re-sew tolerance
`_resew_faces_to_solid` (`occ`/`ocp` `open_solid_repair.py`) applies when welding a doubled seam
or a missing sliver strip back shut -- caps how far the actual sew tolerance
(`min(max(3*width, seam_tol), RESEW_CEILING * diagonal)`) can grow, so a large gap never risks
welding unrelated, distant geometry together. Split out of `REL_TOL_E3` 2026-09-23 (same value,
renamed): a genuinely different role from `REL_DIST_TOL` (an edge-classification threshold)
despite both being "a fraction of a real length" -- this one is a REPAIR ceiling, not a
detection/classification test, and the two are free to diverge in value later if ever needed."""

COAXIAL_RETRY = 1.0e-3
"""Relative volume-conservation tolerance for `_try_coaxial_cone_split`'s own retry cascade
(`occ`/`ocp` `split_coaxial_cone.py`) once it has moved past the exact (`retry_tolerance == 0.0`)
attempt -- deliberately looser than `Tolerances.volume_tolerance` (used for that exact-case
attempt instead), since a presplit's new edge is only exact to floating-point precision, not
identical to the tool's own BRep representation of the same curve, so a fuzzy BOPAlgo retry needs
real slack. Split out of `REL_TOL_E3` 2026-09-23 (same value, renamed): an algorithmic retry
margin, not something a user would tune per model."""

SPLIT_RING_REL_TOL = 1.0e-3
"""`count_split_ring_pairs`'s own detection tolerance for a "split boundary ring" defect --
governs 3 related checks on the SAME candidate circle pair at once, by design (radius ratio
`|dR|/max(R)`, gap-vs-diagonal fraction, perpendicular-offset-vs-radius ratio): "two circles that
should be one". Split out of `REL_TOL_E3` 2026-09-23 (same value, renamed) -- its own dedicated,
already-well-documented role, distinct from `REL_DIST_TOL`'s single-purpose classification."""
# REL_TOL_E3 removed 2026-09-23: every real site was, under a different name, a genuinely
# distinct role (REL_DIST_TOL, RESEW_CEILING, COAXIAL_RETRY, SPLIT_RING_REL_TOL above, or a
# plain non-tolerance "1% of a real span" construction replaced with a literal) -- same value
# (1e-3) throughout, just never one single concept to begin with.

DEFAULT_SLIVER_EDGE_REL_TOL = 1.0e-4
"""Default of `Tolerances.sliver_edge_rel_tol` (a real, user-facing field -- itself now a plain
1e-4 literal in `GeoTolerances.__init__`, matching every other field's own convention there), for
`geo/solid_defects.py`'s own pure, duck-typed functions (`find_short_edges`/
`find_split_ring_faces`/`check_solid_defects`), which have no `tolerances` object to read from by
signature. Same naming convention as `DEFAULT_MIN_FACE_WIDTH`/`DEFAULT_MIN_SOLID_VOLUME`/
`DEFAULT_SPLIT_SCALE`: a `DEFAULT_<field>` constant backing a real `Tolerances` field's own
default for a context with no `tolerances` object, not an independent role of its own. Split out
of `REL_TOL_E4` 2026-09-23 (same value, renamed) -- every real production call site of these 3
functions already passes `tolerances.sliver_edge_rel_tol` explicitly except
`Gcollapse_split_rings`' own call to `find_split_ring_faces` (occ/ocp), fixed at the same time to
thread it through instead of silently falling back to this default."""

FUSE_REFINE_REL_TOL = 1.0e-4
"""`geo/solid_ops.py::Gfuse_solids`'s own `refine(rel_tol=...)` call after a successful boolean
fuse: deliberately looser than `refine()`'s own strict default (`NATIVE_VOL_RATIO_TOL`, 1e-6) --
merging the redundant tangent-seam faces a boolean fuse leaves behind legitimately moves the
volume by ~1e-6 relative, and the strict guard would then discard the clean, merged result and
keep the redundant-edge one instead (see `Gfuse_solids`'s own docstring for the full derivation
and the real fixture that motivated this). Split out of `REL_TOL_E4` 2026-09-23 (same value,
renamed): a deliberate, well-documented algorithmic choice for this one "fuse then tidy up" step,
unrelated to `DEFAULT_SLIVER_EDGE_REL_TOL`'s own role despite sharing the same historical value."""

# REL_TOL_E5 removed 2026-09-23: its 4 real sites (geo/{occ,ocp}/split.py's
# remove_tools_from_raw_solids) are exactly Tolerances.volume_tolerance's own role (volume
# conservation after a split) -- tolerances threaded through, real value change (1e-5 -> 1e-6)
# accepted per direct user instruction after measurement showed real corpus samples in the
# risky [1e-6, 1e-5] window; verified via corpus diff + a direct d1suned check on the
# affected fixtures (see CLAUDE.md).
# REL_TOL_E6 removed 2026-09-23: every real site was, under a different name, a genuinely
# distinct role (NATIVE_VOL_RATIO_TOL, LINE_COPLANAR_REL_TOL, BOX_UNION_VOL_TOL below, or
# `Tolerances.volume_tolerance`'s own real user-facing field, now a plain 1e-6 literal in its
# own default) -- same value (1e-6) throughout, just never one single concept to begin with.

NATIVE_VOL_RATIO_TOL = 1.0e-6
"""Default of `GSolid.refine()`'s own `rel_tol` (all 3 engines): the volume-invariance guard
`refine()` itself checks after its native `ShapeUpgrade_UnifySameDomain`/`removeSplitter()`
operation, before trusting the refined result. Split out of `REL_TOL_E6` 2026-09-23 (same value,
renamed) -- a tolerance on a native/kernel-adjacent operation's own self-check, not a relative-
volume comparison between two independently-obtained solids (`Tolerances.volume_tolerance`'s own
role) -- kept as its own name even though the two happen to share a value."""

LINE_COPLANAR_REL_TOL = 1.0e-6
"""`GLine.intersect_line`'s own (all 3 engines) skew-vs-coplanar test: two lines are treated as
actually intersecting only if their own perpendicular gap, scaled by `max(|Position1|, |Position2|,
1.0)` for numerical conditioning, stays below this fraction -- otherwise they are skew and no
intersection point exists. Split out of `REL_TOL_E6` 2026-09-23 (same value, renamed): a pure
geometric method with no `tolerances` object available, its own dedicated role distinct from
`Tolerances.volume_tolerance`/`NATIVE_VOL_RATIO_TOL`."""

BOX_UNION_VOL_TOL = 1.0e-6
"""`myBox.add()`'s own (Reversed+Reversed case) exact-union-vs-safe-fallback check: the union of
two boxes is kept as the combined result only if its volume matches `vol(A) + vol(B) - vol(A∩B)`
(the inclusion-exclusion identity, true iff the two boxes leave no gap relative to their own
combined bounding box) within this relative fraction -- otherwise the union is a real
over-approximation and the code falls back to the larger of the two boxes alone. Split out of
`REL_TOL_E6` 2026-09-23 (same value, renamed): `myBox` is pure geometry with no `tolerances`
object available (see `geo/vector_geometry.py`'s own module docstring on `myBox`), its own
dedicated role distinct from `Tolerances.volume_tolerance`/`NATIVE_VOL_RATIO_TOL`/
`LINE_COPLANAR_REL_TOL`."""

# Kernel zero (mm^3): a boolean Common of two shapes has content only above this.
VOLUME_MIN_E8 = 1.0e-8

DEFAULT_FIX_TOLERANCE = 1.0e-6
"""Default of `Tolerances.fix_tolerance` (a real, user-facing field -- itself now a plain 1e-6
literal in both `GeoTolerances.__init__` and `geouned.Tolerances.__init__`, matching every other
field's own convention), for the handful of sites that need a fix tolerance with no `tolerances`
object available: `split_repair.py::_repair_non_manifold_solid`'s own default (all its real call
sites already pass `tolerances.fix_tolerance` explicitly) and `Gload_step`'s own `_native_fix`
call (all 3 engines -- a "plain loading primitive" with no `tolerances` parameter at all, by
design). Split out of `KERNEL_TOL_E6` 2026-09-23 (same value, renamed) -- that name still covers at least 4
other, unrelated CAD-kernel-adjacent roles (differential-geometry query resolution, UV-projection,
face construction), under review one group at a time; this is only the "fix tolerance" group's own
share of it."""

SEW_TOLERANCE = 1.0e-6
"""`BRepBuilderAPI_Sewing`'s own tolerance at the 2 sites (`primitives.py::Gmake_shell`,
`split_repair.py::_separate_edge_joined_components`, both engines) that have no `tolerances`
object available at all (pure low-level construction/repair helpers with no such parameter in
their own signature) -- a raw OCCT kernel-API parameter with no corresponding `Tolerances` field,
same precedent as `KERNEL_TOL_E7`'s own `GeomAPI_IntSS` site (left as a bare intrinsic constant,
2026-09-22). Split out of `KERNEL_TOL_E6` 2026-09-23 (same value, renamed): a genuinely different
CAD-kernel operation from `DEFAULT_FIX_TOLERANCE`'s own `ShapeFix_Shape`/`.fix()` role (joining
faces into a shell vs. repairing an existing shape's topology) despite sharing the same
conservative default value."""

GEOM_PROP_TOL = 1.0e-6
"""`GeomLProp_CLProps`/`GeomLProp_SLProps`'s own tolerance -- the differential-geometry property
calculator OCCT uses for `GEdge.curvature`/`GFace.value_at`/`normal_at`/`tangent_at` (`occ`/`ocp`
only -- `freecad`'s own equivalent methods use FreeCAD's own `Part.Edge.Curve`/`Part.Face.Surface`
objects directly, no such OCCT class or tolerance involved), governing how it resolves a
degenerate/singular parametric point, not a shape-repair or construction operation. Per direct
user decision (2026-09-23): every tolerance handed to an internal BRep-kernel operation is its own
constant, never threaded through `Tolerances` -- these are pure geometric query methods on
`GEdge`/`GFace` with no `tolerances` object in scope regardless. Split out of `KERNEL_TOL_E6`
2026-09-23 (same value, renamed)."""

UV_PROJECTION_TOL = 1.0e-6
"""`ShapeAnalysis_Surface.ValueOfUV`'s own tolerance (`split_coaxial_cone.py`, both engines) --
projects a 3D point onto a cone's own parametric (U, V) surface coordinates during the coaxial-cone
split fallback. Same "internal BRep-kernel operation, own constant" rule as `GEOM_PROP_TOL`. Split
out of `KERNEL_TOL_E6` 2026-09-23 (same value, renamed)."""

FACE_CONSTRUCTION_TOL = 1.0e-6
"""`BRepBuilderAPI_MakeFace`'s own tolerance (`repair.py`, both engines) when building a new face
directly from a surface and UV bounds. Same "internal BRep-kernel operation, own constant" rule as
`GEOM_PROP_TOL`. Split out of `KERNEL_TOL_E6` 2026-09-23 (same value, renamed)."""

SURFACE_INTERSECT_TOL = 1.0e-7
"""`GeomAPI_IntSS`'s own tolerance -- the native surface-surface intersector `GPlane.
intersect_plane`'s own fallback uses once its pure-GVector well-conditioned math isn't reliable
enough (`occ`/`ocp` `topology.py`). Already identified and left as a bare intrinsic constant with
no `Tolerances` field on 2026-09-22 ("no controlo... lo dejamos asi por ahora"); split out of
`KERNEL_TOL_E7` 2026-09-23 into its own name (same value, unchanged) once `KERNEL_TOL_E7` itself
was found to cover a second, unrelated role too (`POINT_CLASSIFY_TOL`, right below) -- a genuinely
different OCCT operation (intersecting two surfaces vs. classifying a point against one) despite
sharing the same historical value."""

POINT_CLASSIFY_TOL = 1.0e-7
"""`BRepTopAdaptor_FClass2d`/`BRepClass3d_SolidClassifier`'s own tolerance -- "is this (u, v) point
inside the face's own trimmed domain" / "is this 3D point inside this solid" queries (`occ`/`ocp`
`topology.py`'s `is_part_of_domain`/`orientation_outward`/others; `freecad`'s own equivalent
`Part.Shape.isInside(probe, tol, False)` call shares this exact role too, already unified under
this same name and value back when it was still `LENGTH_TOL_E7`/`KERNEL_TOL_E7`, 2026-09-23). A raw
kernel-API classification tolerance with no corresponding `Tolerances` field -- same "internal
BRep-kernel operation, own constant" rule as `GEOM_PROP_TOL`. Split out of `KERNEL_TOL_E7`
2026-09-23 (same value, renamed), distinct from `SURFACE_INTERSECT_TOL`'s own different operation."""

# Tolerance handed to a CAD-kernel operation (fix, sewing, split) or a bound on one.
# KERNEL_TOL_E6 removed 2026-09-23: every real site was, under a different name, a genuinely
# distinct internal-BRep-operation role (DEFAULT_FIX_TOLERANCE, SEW_TOLERANCE, GEOM_PROP_TOL,
# UV_PROJECTION_TOL, FACE_CONSTRUCTION_TOL above) -- same value (1e-6) throughout, just never one
# single concept to begin with.
# KERNEL_TOL_E7 removed 2026-09-23: split into SURFACE_INTERSECT_TOL (GeomAPI_IntSS) and
# POINT_CLASSIFY_TOL (BRepTopAdaptor_FClass2d/BRepClass3d_SolidClassifier/freecad's own isInside)
# above -- same value (1e-7), two genuinely different OCCT operations.

TOLERANCE_WELD_FLOOR = 1.0e-3
"""`decom_one_generators.py::generic_split`'s own minimum floor (`max(50 * tolerances.
split_tolerance, TOLERANCE_WELD_FLOOR)`) for detecting a BOPAlgo tolerance-weld symptom: a
fragment's own edge/vertex tolerances (`Gsolid_max_tolerance`) inflated far above the split
tolerance, together with non-manifold edges, signals a near-tangent junction the split papered
over rather than resolved (see the function's own inline comment for the full account). Split out
of `KERNEL_TOL_E3` 2026-09-23 (same value, renamed) -- its own dedicated, already-well-documented
role, the last surviving member of the former `KERNEL_TOL_E*` family."""

EDGE_PROJECTION_TOL = 1.0e-8
"""`GEdge.is_inside(point, tolerance)`'s own tolerance at its one real call site
(`build_shape_functions.py::cut_face`, all 3 engines) -- combines a real point-to-curve projection
distance check (`GeomAPI_ProjectPointOnCurve`) and a parametric-range slack to decide whether an
intersection point genuinely lies on the edge's own finite extent, not just its underlying
infinite curve. An internal BRep-kernel-adjacent operation, its own dedicated constant per the
same rule as `GEOM_PROP_TOL`/`SEW_TOLERANCE`/etc. Split out of `KERNEL_TOL_E8` 2026-09-23 (same
value, renamed)."""
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

# Reference volume (mm^3) of every RELATIVE volume comparison: `abs(a - b) <= tol * max(|reference|, VOLUME_REF)`. Below it a
# relative test is meaningless, so the tolerance becomes the absolute `tol * VOLUME_REF`. It is a characteristic SCALE, not a
# threshold (contrast Tolerances.min_solid_volume, which decides whether a piece is worth keeping) and it must not be tied to
# it: a legitimate repair of a small solid changes its volume by absolute amounts (5e-5 and 8e-5 mm^3 on rev_pipe.stp) that a
# much smaller floor would reject. Measured window for the 3e-4 repair gate: it must accept |dV| = 8.0e-5 and reject the
# 0.17 and 0.20 mm^3 failures (ring.stp, rc17.stp), so VOLUME_REF in (0.27, ~570); 1.0 sits inside (only 4 small-solid cases).
VOLUME_REF = 1.0

# Whether GEOUNED.decompose.decompose_cache.DecomposeCache stages each freshly (re)decomposed solid to its own file
# under decompose_cache/tmp/ as it's computed, so a crash mid-run can be resumed from exactly where it left off on
# the next Settings.load_from_cache=True run. Not a user-facing Settings field -- an internal safety/performance
# knob, per direct user instruction (2026-09-27). The final, consolidated decompose_cache/{solids,enclosures}.bin
# is written unconditionally at the end of any fully successful decomposition run regardless of this flag; this
# only controls the granular, crash-recoverable staging during the run itself.
TMP_CACHE = True
