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
