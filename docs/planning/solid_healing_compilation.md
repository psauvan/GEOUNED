# Solid healing — compilation for a ground-up redesign

Written 2026-11-07, at the user's own explicit request, as the starting
point for a future, dedicated session ("solid healing") whose goal is to
**redesign GEOUNED's whole CAD-defect detection + repair pipeline from
scratch, with a plan, using everything learned so far** — not to patch
the current cascade piece by piece again.

This document does not replace the two existing sources of record:
- `docs/history/geouned_migration_log.md` — the full chronological
  session log (every fix, with its own verification).
- The `reference_cad_defect_recipes` memory (`CLAUDE.md` calls this
  `docs/reference/cad_defect_recipes`, but as of this writing the
  recipe catalog actually lives only in that memory file, not a repo
  file under that path — worth reconciling during the redesign) — a
  per-defect-type catalog (Recipe 0 phantom cut, Recipe 1 split
  boundary ring, Recipe 1b cylindrical variant, Recipe 2 malformed face
  wire, Recipe 3 failed BOP split, plus the `sliver_healing` v0
  algorithm) written earlier in the project (2026-08-28/29), before
  most of what's below existed.

This document's own job is different: pull every repair/detection
mechanism that exists **today**, across the whole codebase and the
whole project history, into one place, so a future redesign starts from
a complete inventory instead of rediscovering pieces one STEP file at a
time, as has happened repeatedly.

## 0. Background — what a plain STEP export/import actually does inside OCCT

Added 2026-11-07, directly at the user's request, to stop treating "the
STEP reader/writer heals things" as a black box (see §6's own
"recurring crutch that masks the real defect" item). Read from the
actual OCCT headers shipped with this project's `ocpenv` conda env
(`Library/include/opencascade/*.hxx` — real class declarations of the
installed OCCT version, not generic documentation), then checked
empirically (OCP, not just read).

**Two entirely separate layers, every STEP read or write:**

1. **Geometric translation** (`StepToTopoDS_TranslateFace`/`_TranslateShell`/
   `_TranslateSolid`/... on read, the symmetric `TopoDSToStep_*` on
   write) — builds/serializes each STEP entity as its own `TopoDS_Face`/
   `TopoDS_Shell`/... 1:1. Never merges or drops anything by itself.
2. **Shape healing/processing** — optional, but ON by default in both
   `STEPControl_Reader` and `STEPControl_Writer`. Both classes call
   `XSAlgo_ShapeProcessor::ProcessShape(shape, operationsFlags, progress)`
   → `ShapeProcess::Perform(context, flags)`, where `flags` is a
   `std::bitset` over a **closed, fixed set of 18 operations**
   (`ShapeProcess.hxx`, enum `Operation`):
   `DirectFaces, SameParameter, SetTolerance, SplitAngle,
   BSplineRestriction, ElementaryToRevolution, SweptToElementary,
   SurfaceToBSpline, ToBezier, SplitContinuity, SplitClosedFaces,
   FixWireGaps, FixFaceSize, DropSmallSolids, DropSmallEdges, FixShape,
   SplitClosedEdges, SplitCommonVertex`. Which ones are on, and their
   thresholds, come from `DE_ShapeFixParameters` (a ~50-field struct:
   `Tolerance3d=1e-6`, `MaxTolerance3d=1.0`, `RemoveSmallAreaFaceMode`,
   `FixSmallAreaWireMode`, ... each a tri-state `FixMode`), settable via
   `STEPControl_Reader/Writer::SetShapeFixParameters(...)`/
   `SetShapeProcessFlags(...)`.

**The classes actually behind the "merge/drop small X" operations**
(for when the redesign needs the primitive directly, not a round trip):

| Operation | Real class | What it does |
|---|---|---|
| `FixShape` | `ShapeFix_Shape` (orchestrates `ShapeFix_Solid`/`_Shell`/`_Face`/`_Wire`/`_Edge`) | Generic topology repair: orientation, free wires, missing seams. **Not** face merging. |
| `FixWireGaps` / `DropSmallEdges` | `ShapeFix_Wireframe::FixWireGaps()` / `FixSmallEdges()` → `MergeSmallEdges(..., theModeDrop, theLimitAngle)` | Closes 2D/3D gaps between adjacent edges' curves; merges (or drops) small edges into their neighbor |
| `FixFaceSize` | `ShapeFix_FixSmallFace` (detector: `ShapeAnalysis_CheckSmallFace`) | The real face-sliver eliminator: `IsSpotFace`/`FixSpotFace` (collapsed-to-a-point face), `IsStripSupport`/`CheckSingleStrip`/`FixStripFace` (degenerate thin-strip face), `CheckPin`/`FixPinFace` (pinched/degenerate corner) |
| `DropSmallSolids` | `ShapeFix_FixSmallSolid` | `Remove(shape, context)` deletes solids below a volume/width threshold; `Merge(shape, context)` fuses them into their neighbor instead |
| `FixFaceSize` (via `FixSmallAreaWireMode`) | `ShapeUpgrade_RemoveInternalWires` | Deletes internal wires (holes) with area < `MinArea()`; deletes the whole face if its outer wire ends up made only of those edges |

**The key finding: face *merging* (`ShapeUpgrade_UnifySameDomain` —
`UnifyFaces()`/`UnifyEdges()`) is *not* in that 18-operation enum at
all.** It is never invoked by a plain STEP read or write, under any
parameter setting — it's a wholly separate, standalone class with its
own API (`Initialize(shape, UnifyEdges, UnifyFaces, ConcatBSplines);
Build();`), that has to be called explicitly. Confirmed empirically
(not just from the header): see `_native_fix`'s own entry in §2 for the
real experiment (a 10-face doubled-coplanar-face shape stays at 10
faces after a genuine `STEPControl_Writer`→file→`STEPControl_Reader`
round trip; only an explicit `UnifySameDomain` call collapses it to 6).

**Consequence for this whole document**: when GEOUNED's own code or
this project's history log says "a STEP round trip fixes/merges X", it
is doing one of two genuinely different things, and the redesign should
keep them conceptually separate:
- Fixing something via the 18 real `ShapeProcess` operations above
  (gap closing, small-edge/face/solid dropping, generic `ShapeFix_Shape`
  repair, and — critically for Recipe 3 — re-serializing a face's wires
  into STEP's own `FACE_BOUND`/`FACE_OUTER_BOUND` entity types, which is
  what actually fixes `BRepCheck_InvalidImbricationOfWires`, not any of
  the 18 operations themselves).
- Fixing/merging something via `_native_fix`'s own, GEOUNED-added
  `ShapeUpgrade_UnifySameDomain` call, which runs on *every* STEP load
  regardless of whether the file was round-tripped or original — see
  §2.

## 1. Where repair happens today — three disconnected layers

GEOUNED's CAD-defect handling is split across three pipeline stages,
each with its **own** detection logic, its **own** tolerance gates, and
very little sharing between them:

1. **Load time** (`Gload_and_process_step`, `geo/{occ,ocp}/io.py` +
   `repair.py`): runs once per solid, right after `STEPControl_Reader`.
   Cascade order: `_native_fix` → `GSolid(...)` → `Gcheck_and_repair`
   (itself: already-clean check → `Gcollapse_split_rings` →
   `Gsliver_heal` → `Gdefeature`) → `Gspline_surface` /
   `Gsubstitute_spline_quadrics` → (since 2026-10-07) `_promote_open_shells`
   for a bare `TopAbs_SHELL` not yet wrapped in a solid.
2. **Post-split** (`geo/{occ,ocp}/split.py` + `split_repair.py` +
   `split_coaxial_cone.py`, called from `generic_split`'s own recursive
   decomposition): repairs a single raw `BOPAlgo_Splitter` result —
   non-manifold-solid reconstruction, phantom-cut separation,
   coaxial-cone retry, a tolerance-weld STEP-round-trip heal-retry.
3. **Post-decomposition** (`geo/solid_ops.py`, `decompose/decom_one_generators.py`):
   operates on the final list of irreducible pieces a whole solid
   decomposed into — `merge_sliver_pieces` (whole-piece
   Volume/Area-ratio sliver fuse-into-neighbor), the 3 `_fix_result_is_sane`
   gated `.fix()` call sites, `Gfuse_solids`'s own opportunistic
   `Gsubstitute_spline_quadrics` retry after `refine()`.

A **fourth, entirely separate layer that nothing above ever checks**:
`conversion/cell_definition.py::simple_solid_definition`'s own per-face
loop, which decides which of a piece's faces actually contribute a term
to the written boolean definition. As of 2026-11-06/07 (see §5 below)
this can silently **drop a real, correctly-shaped face's own
contribution** with no sanity check at all that every face of a
geometrically-valid, defect-free solid ends up represented in the
output — the newest, and in one real case the most consequential,
finding in this whole area.

## 2. Every repair primitive that exists today

### Load-time cascade (`geo/{occ,ocp}/repair.py`)

- **`Gdefeature(solid, faces)`** — `BRepAlgoAPI_Defeaturing` wrapper,
  removes a given face set; refuses near-degenerate riser faces.
- **`Gcollapse_split_rings(solid, min_face_width)`** — drops "riser"
  faces (`find_split_ring_faces`), sews, rebuilds, `ShapeFix_Shape`.
  Accept gate (strengthened 2026-08-28 after a false pass on `LR.stp`):
  valid + `|dV| < MAX_SPLIT_RING_VOLUME_REL_CHANGE` (3e-4) +
  `count_split_ring_pairs` strictly decreased.
- **`Gsliver_heal(solid, min_face_width)`** — the fuller `sliver_healing`
  v0 design: drop sliver faces, find a near-duplicate-surface pair among
  the *remaining* faces, keep the larger/more-valid one, re-trim any
  freed quadric rim onto the kept plane (`_retrim_freed_quadrics`), snap
  a new planar cap (`_snapped_planar_cap`), sew last. Gate: valid +
  `|dV| < MAX_SLIVER_HEAL_VOLUME_REL_CHANGE` (5e-4).
- **`Gcheck_and_repair`** — the per-solid orchestrator: already-clean →
  `Gcollapse_split_rings` → `Gsliver_heal` → `Gdefeature`, first one to
  return non-`None` wins.
- **`_native_fix(native, tolerance)`** (`geo/{occ,ocp}/_native_utils.py`)
  — **not an OCCT mechanism, GEOUNED's own code**, called on *every
  single solid* by both `Gload_step` and `Gload_and_process_step`
  (`geo/{occ,ocp}/io.py`), i.e. on every STEP load, always, before the
  solid ever becomes a `GSolid`. When the raw loaded shape is already
  `BRepCheck`-valid, it runs
  `ShapeUpgrade_UnifySameDomain(native, UnifyEdges=True, UnifyFaces=True,
  ConcatBSplines=True)` directly and adopts the result (rejecting a
  Solid→Shell degradation, same guard as `GSolid.refine()`); otherwise it
  falls back to a plain `ShapeFix_Shape(tolerance)` copy. This is the
  **literal body of `GSolid.fix()`**, extracted so it can also run on the
  raw native shape right after `STEPControl_Reader`, not a second,
  independent implementation.
  **This is the real answer to "why does exporting+reading a STEP file
  merge faces"**: it is not STEP's own read/write pipeline doing it (see
  the empirical check below) — it's this one GEOUNED-side function,
  which runs unconditionally on every load, STEP round-tripped or not.
  A fixture loaded through a bare `STEPControl_Reader` (bypassing
  `Gload_step`/`Gload_and_process_step`) would **not** see any face
  merging at all.
  **Verified empirically, 2026-11-07** (not just read from headers):
  built a 10-face shape (two adjacent boxes fused with a plain
  `BRepAlgoAPI_Fuse`, no glue — the shared wall is removed but the
  top/bottom/front/back faces of each box stay as separate, coplanar,
  edge-adjacent faces, the exact "doubled coplanar face" signature this
  whole document is about). A real file round trip through
  `STEPControl_Writer`/`STEPControl_Reader` alone (OCP, no GEOUNED
  involved) left it at **10 faces, unchanged**; an explicit
  `ShapeUpgrade_UnifySameDomain` call on the same shape collapsed it to
  the true **6** faces of the outer cube. Confirms, independent of the
  header-reading conclusion in the section below, that STEP's own
  read/write pipeline (`XSAlgo_ShapeProcessor`/`ShapeProcess`) does not
  merge faces — only an explicit `UnifySameDomain` call does, and
  `_native_fix` is where GEOUNED makes that call on every load. This
  also matches an already-documented, independent finding in the
  history log (`rev_pipe.stp`'s own `PIECE 0`, ~line 10835): *"`Gheal_topology`
  (STEP round-trip) before `refine()` doesn't merge [7 coincident
  coplanar-face pairs]; `refine()`/`UnifyFaces` *does* merge 18→14
  faces"* — the pure round-trip (`Gheal_topology`, which does NOT call
  `_native_fix`/`UnifySameDomain`, only a raw serialize/deserialize) left
  them unmerged; only the explicit `UnifyFaces` call merged them.
- **`Gheal_topology(solid)`** — STEP serialize→deserialize **entirely in
  memory** (`io.BytesIO`, no temp file). Freezes each face's
  outer-vs-hole wire choice into the STEP entity type, which fixes
  `BRepCheck_InvalidImbricationOfWires` (Recipe 3) that nothing else
  could touch. Gate: valid + `|dV| < MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE`
  (1e-3, looser than the others — the failed split that creates this
  defect also inflates the input volume, and the round trip corrects
  both at once). Also reused as: the `GSolid.refine()` pre-heal
  (2026-10-14, gated on `Gsolid_max_tolerance > UNIFY_PREHEAL_TOLERANCE_FLOOR`),
  and the `generic_split` tolerance-weld retry's own healed copy.
- **`Gtrim_overlapping_coplanar_faces(solid)`** (2026-10-01) — the
  "overlapping coplanar faces" repair: delete the smaller ("phantom")
  face, re-add the larger with the phantom's own boundary as an inner
  wire (hole), re-sew. A second sub-case (2026-10-02,
  `_trim_edge_for_boundary_touching_pair`) handles the phantom sharing a
  real edge with the larger face (touching, not nested): trims the
  larger face's own 2 adjacent edges down to the phantom's far endpoint,
  reusing the phantom's own real edges directly (needs `_order_wire_edges`
  to topologically sort the replacement edges first).
- **`_coerce_to_solid`** — forcibly wraps a healed Shell into a
  `TopoDS_Solid` (`BRepBuilderAPI_MakeSolid`, then `ShapeFix_Solid` as a
  second recovery attempt if the first comes back `BRepCheck`-invalid).
  Used by `Gcollapse_split_rings`; deliberately **not** used by
  `Gtrim_overlapping_coplanar_faces` (tried 2026-10-07, reverted — too
  narrow a layer, broke a different, already-correct fixture).
- **`_promote_open_shells(shape, tolerances)`** (2026-10-07) — scans for
  any free-standing `TopAbs_SHELL` not nested in a solid, repairs via a
  small open-solid-repair registry (below), substitutes in place via
  `ShapeBuild_ReShape`.
- **`open_solid_repair.py`** registry of open-shell repair *causes*:
  `split_duplicate_seam`, `missing_sliver_strip`, and (2026-10-07) a
  third, newly-discovered one, **"retrace spike"** — two free edges
  sharing one vertex, both extending collinearly from it (a boolean cut
  landing exactly tangent to a pre-existing boundary). Detection
  (`_match_retrace_spike`) requires the same HOST FACE, not just shared
  collinear edges (collinear edges on different faces meeting at a
  vertex is a legitimate straight corner, confirmed as a real false
  match found during development). Repair (`_rebuild_face_without_spike`
  / `_close_retrace_spike`) trims only the one wire containing the
  spike, re-adds every other wire of that face unchanged, re-sews, and
  applies a second `ShapeFix_Shape` pass if still invalid with 0 free
  edges (a parametrization residue, not a topology gap).

### Post-split cascade (`geo/{occ,ocp}/split.py`, `split_repair.py`, `split_coaxial_cone.py`)

- **`_repair_non_manifold_solid`** — Recipe 0's donor-face-capping repair
  for a genuinely non-manifold BOP output (one solid where it should be
  two, joined only along edges).
- **`_separate_edge_joined_components`** (2026-08-29, superseded the
  donor-face approach for the strict phantom-cut case) — builds the
  face-adjacency graph excluding non-manifold edges, partitions into
  connected components, sews + rebuilds each as its own solid with NO
  donor faces. Accept gate: every component valid, Σ|vol| matches the
  original to 1e-6 relative.
- **Coaxial-cone retry** (`split_coaxial_cone.py`) — a dedicated
  fallback for the coaxial-cone/cylinder tangency degeneracy, with its
  own escalating `FuzzyValue` retry ladder (an exact attempt at
  `tolerances.volume_tolerance`, then a looser `COAXIAL_RETRY` margin).
- **Tolerance-weld STEP-round-trip heal-retry** (in `_raw_bop_split`,
  2026-09-28, later REPLACED as the trigger condition in 2026-10-07/09)
  — originally fired on `Gsolid_max_tolerance(solid) > TOLERANCE_WELD_FLOOR
  and Gsolid_nonmanifold_edge_count(solid) >= 1`; this heuristic could
  not distinguish a genuinely-stuck-but-correct fragment from a
  genuinely-corrupted one (measured directly: a correct stuck fragment
  of `BC.stp` has 8.67mm native tolerance, two orders of magnitude
  *above* a genuinely broken one's 0.075mm — tolerance magnitude alone
  is not a reliable signal). Replaced by
  **`_some_real_candidate_should_have_split`** (`decom_one_generators.py`):
  an independent, geometry-based ground-truth check — samples points
  anchored to the fragment's own real faces (`_face_anchored_points`,
  reusing `GSolid.find_interior_point`'s own bisection technique
  generalized to a grid) and asks, for each already-tried-and-rejected
  candidate surface, whether real material actually sits on both sides.
  Only retries the heal if the geometry itself says a split *should*
  have happened.
- **BOP-corruption-across-repeated-attempts fix** (2026-09-28,
  `multiplane_add_plane_cyl.stp`): `generic_split`'s own candidate loop
  tries the SAME base solid against many candidate tool surfaces in
  sequence; `BOPAlgo_Splitter` can silently corrupt the base's own
  `BRepCheck` validity (not just tolerance) as a side effect of merely
  *attempting* an intersection that ultimately fails, even with 0
  geometry change and identical vertex positions. Fix: reset tolerance
  AND re-run `.fix()` before every new candidate, not just once at the
  top.
- **`_fix_result_is_sane(fixed, reference_volume)`** (2026-10-13) — a
  sanity check now gating all 3 `.fix()` call sites in
  `decom_one_generators.py`: no face with 0 wires, and
  `abs(fixed.Volume)` stays within `MAX_REPAIR_VOLUME_REL_CHANGE` of
  `abs(reference_volume)` (magnitude comparison — a correct repair can
  legitimately flip orientation, must never balloon/collapse size).
  Root-caused a 3-stage cascading failure on
  `Mixed/base_all8_to_cylinder.stp` where `ShapeFix_Shape` silently
  **inverted** a tiny sliver's orientation (passing `is_valid()`, which
  never checks volume sign), and a second `.fix()` on the
  already-corrupted fragment then collapsed it into genuine garbage
  (`Volume~6.7e166`).

### Post-decomposition (`geo/solid_ops.py`, `decom_one_generators.py`)

- **`merge_sliver_pieces(pieces, tolerances)`** (2026-10-04) — the
  *whole-piece* sliver criterion, deliberately NOT per-face
  `CharacteristicWidth` (that false-flagged a real 133,000mm³ piece on
  `SCDR_90.stp` that merely has 2 individually-thin faces). Criterion:
  `abs(Volume)/Area < SLIVER_SOLID_VOL_AREA_RATIO`. Fuses each flagged
  sliver into its best real neighbor (`_shared_face_area`), single pass
  (iterative merging was tried and reverted — it catastrophically
  over-merged through a solid's own large "envelope" piece touching
  almost everything).
- **`Gfuse_solids`'s own opportunistic post-`refine()` retry** (2026-10-15)
  — when a fuse-and-refine result still carries an unsupported
  (`GeomAbs_SurfaceOfRevolution`) face and a real `tolerances` was
  passed, tries `Gsubstitute_spline_quadrics` and adopts the result only
  if every unsupported face resolved.
- **`GSolid.refine()`'s own two independent pre-heal triggers**
  (2026-10-14): (1) `Gsolid_max_tolerance(solid) > UNIFY_PREHEAL_TOLERANCE_FLOOR`
  (a BOPAlgo tolerance-weld signature); (2) `find_degenerate_plane_faces(solid)`
  — a planar face carrying a native OCCT degenerate edge (the pole
  convention for a sphere/cone/torus, which a plane never legitimately
  has) — found to independently crash `ShapeUpgrade_UnifySameDomain`'s
  `UnifyFaces=True` pass with a native access violation,  *not* caught
  by the tolerance-floor check (confirmed: the crashing shape's own
  tolerance was 8.8e-6, nowhere near the floor). Both run the same
  `Gheal_topology` STEP-round-trip pre-heal when triggered.
- Both `GSolid.refine()` and `_native_fix` now **reject a result that
  silently degraded a genuine `TopoDS_Solid` to a bare `TopoDS_Shell`**
  (2026-10-07) — `ShapeUpgrade_UnifySameDomain`/`ShapeFix_Shape` can do
  this while still reporting `BRepCheck`-valid (closed, correct volume),
  which downstream code would otherwise treat as a Solid when it isn't.

## 3. Every defect detector that exists today (`geo/solid_defects.py`)

- **`valid_solid(solid, min_volume)`** — the oldest, crudest check:
  `Volume < 0`, `Area == 0`, `Volume/Area` ratio, `abs(Volume) < min_volume`.
  Does **not** check `BRepCheck_Analyzer` at all (Recipe 3's own root
  cause: a native-invalid fragment with a huge, inflated-but-plausible
  volume sails straight through this check).
- **`find_short_edges(solid, rel_tol)`** — pathologically short edges
  relative to the solid's own BoundBox diagonal.
- **`find_degenerate_plane_faces(solid)`** (2026-10-14) — a planar face
  carrying a genuine OCCT-native degenerate edge.
- **`find_split_ring_faces(solid, min_face_width, rel_tol)`** — the
  Recipe-1 "riser" detector: a sliver-scale analytic curved face that
  also touches a pathologically short edge.
- **`find_sliver_faces(solid, min_face_width)`** / **`is_sliver_face`**
  — any face whose own `CharacteristicWidth < min_face_width`. **As of
  2026-11-07, confirmed that `check_solid_defects` never calls this
  function at all** — see §5.
- **`count_split_ring_pairs(solid, rel_tol)`** — counts coaxial,
  concentric, near-equal-radius circular-edge pairs on the same curved
  face; used as the Recipe-1b "did the doubled rings actually merge"
  success signal, not a standalone defect flag.
- **`near_surface_pair(surf_a, surf_b, dist_tol)`** — same-kind surfaces
  (plane/cylinder/sphere), coincident axis, single varying parameter
  within `(1e-5, dist_tol)` — the "these should really be one surface"
  signal `Gsliver_heal` uses to pick its keep/drop pair.
- **`find_overlapping_coplanar_faces(solid, rel_tol)`** (2026-10-01) —
  two planar faces on the same infinite plane, not already contiguous,
  the smaller one's boundary projecting inside the larger's trimmed
  domain.
- **`check_solid_defects(solid, sliver_edge_rel_tol, min_face_width)`** —
  the single orchestrator meant to be *the* answer to "is this solid
  clean": `is_valid()` + `find_short_edges` (but **only** flagged if a
  short-edge's own face is ALSO sliver-scale by `CharacteristicWidth` —
  a short edge on an otherwise-normal face is deliberately NOT a defect,
  2026-08-29, `L4_pipes.stp`) + `find_overlapping_coplanar_faces`. **Does
  not call `find_sliver_faces`, `find_split_ring_faces`,
  `find_degenerate_plane_faces`, or `count_split_ring_pairs` at all** —
  each of those is consulted only by its own specific repair function,
  never by the one function callers actually use to ask "is this solid
  OK".

## 4. Defect classes found across the whole project (chronological, condensed)

See `reference_cad_defect_recipes` memory and
`docs/history/geouned_migration_log.md` for the full writeups; this is
the index.

- **Recipe 0 — Phantom cut / "corte fantasma"** (the founding defect of
  the whole migration): a BOP split returns one solid where it should
  split into two, because the two halves touch only along edges
  (zero-area contact) and BOPAlgo treats the shared edges as connective
  tissue. Fixed by `_separate_edge_joined_components`.
- **Recipe 1 / 1b — Split boundary ring** (duplicated micro-trim /
  collapsed step): a single trimming surface appears twice at a
  micro-offset, with parasitic sliver "riser" faces filling the gap and
  every curved face crossing the trim getting a doubled boundary ring.
  Planar variant fixed by `Gcollapse_split_rings`; cylindrical variant
  (needs re-trimming an unbounded quadric, not surface extension) fixed
  by `Gsliver_heal`.
- **Recipe 2 — Malformed face boundary wire**
  (`BRepCheck_UnorientableShape`): an analytic face whose single
  boundary wire has a vertex of edge-degree > 2 (two loops crammed into
  one wire). **No repair found that works** (exhaustively tried); the
  user's own decision: flag + exclude via `corrupted_solids`, no general
  repair planned.
- **Recipe 3 — Failed BOP split → invalid fragment**
  (`BRepCheck_InvalidImbricationOfWires`): one native-invalid face (2
  individually-valid wires improperly nested in UV) in an otherwise
  enormous, nearly-correct-volume fragment. Nothing in-memory repairs
  it; a full STEP serialize→deserialize round trip does (freezes the
  outer-vs-hole wire choice into the entity type). → `Gheal_topology`.
- **Overlapping coplanar faces** (2026-10-01/02): a boolean cut landing
  exactly tangent to a pre-existing surface leaves BOTH the original
  untrimmed face AND the cutting tool's own end-cap face, overlapping,
  instead of one correctly-trimmed annulus. Two sub-cases (nested vs.
  edge-touching) need two different repairs. The SECOND sub-case is
  created **only by GEOUNED's own decomposition itself**, never present
  in the source STEP — only caught because `split_surfaces` was given
  its own explicit check+repair call, separate from the load-time
  cascade.
- **"Retrace spike" open-shell cause** (2026-10-07): a boolean cut
  tangent to a pre-existing boundary leaves a residual edge retracing
  part of its own neighbor's span, producing a bare open shell on STEP
  read. A genuinely general signature (2 free edges, same host face,
  collinear from a shared vertex), found by generalizing from a
  point-sampling-is-meaningless-on-an-open-shell observation.
- **BOPAlgo native-state corruption across repeated candidate attempts**
  (2026-09-28): `generic_split`'s own base solid can be silently
  corrupted (BRepCheck validity, not just tolerance) by merely
  *attempting* an intersection that doesn't end up splitting anything —
  cumulative across a whole candidate search, order-dependent. Isolating
  it required re-running the SAME candidate alone (passed) vs. in
  sequence after N others (failed) — the geometry was never the
  problem, the kernel's own internal state was.
- **Stuck-fragment heal-retry false-negative** (2026-10-07/09): a
  tolerance+non-manifold-edge heuristic for "should I retry healing this
  fragment" could not distinguish a genuinely-irreducible-but-correct
  fragment from a genuinely-corrupted one — tolerance *magnitude* is not
  a reliable signal by itself (measured: the correct case had a
  *larger* native tolerance than the broken one). Needed an independent,
  geometry-based ground-truth check instead of a numeric proxy.
- **Degenerate-edge native access violation** (2026-10-11/14): two
  *independent* trigger signatures for the same crash class
  (`ShapeUpgrade_UnifySameDomain`'s `UnifyFaces=True`) — an inflated
  BRep tolerance (a BOPAlgo tolerance-weld) and, separately, a planar
  face carrying a native degenerate edge (the pole convention, which a
  plane never legitimately needs) — each needed its own, independently
  measured, pre-heal trigger; neither detects the other's case.
- **Native crash from a plain `BRepGProp` query on a genuinely
  degenerate edge** (2026-10-11): `GEdge.__init__` queried linear
  properties unconditionally; a degenerate edge (cone apex, pole) with a
  corrupted tolerance crashed the native call outright. Fixed by
  checking `BRep_Tool.Degenerated_s` first and skipping the query
  entirely.
- **`.fix()` silently accepting a corrupted repair** (2026-10-13): THREE
  stacked call sites in `decom_one_generators.py` could each adopt a
  `ShapeFix_Shape`/`ShapeUpgrade_UnifySameDomain` result that was
  `BRepCheck`-valid but geometrically wrong (orientation-inverted, or
  genuinely corrupted to astronomical volume) — `is_valid()` alone never
  checks volume sign or magnitude. One shared sanity helper
  (`_fix_result_is_sane`) now gates all three.
- **GSolid.refine()/_native_fix silently degrading Solid→Shell**
  (2026-10-07): a native unify/fix pass can report success while
  quietly dropping the Solid wrapper, leaving a bare Shell that still
  reads valid/closed/correct-volume — both functions now reject a
  result with 0 `TopAbs_SOLID` and fall through to the untouched
  original instead.
- **Open-shell promotion at load time** (2026-10-07,
  `STRUCTURAL_PLATE_2#BYZ2RP_31`): a bare `OPEN_SHELL` on STEP read used
  to be an unrecoverable `RuntimeError`. Generic fix: scan for any
  free-standing shell not yet wrapped in a solid, right after
  `reader.OneShape()`, and run it through the open-solid-repair registry
  before anything downstream ever sees it as anything other than an
  ordinary solid.
- **A whole, separate TODAY's finding (2026-11-06/07, see §5)**: a
  decomposed piece can carry *extra, genuinely topologically-valid*
  faces — true micro-slivers, redundant fragments of an
  already-represented analytic surface, and extra faces that don't
  correspond to the correct boundary at all — that **every one of the
  detectors in §3 misses simultaneously**, and whose real downstream
  consequence isn't a crash or an obviously-wrong volume at the solid
  level, but a silently-dropped real face in the *written boolean
  definition* two stages later.

## 5. Today's live case study — the gap none of the above catches

Investigated this session on `part_0_0_0.stp` (a real production
fragment), using a genuinely new verification technique worth keeping
for the redesign: **comparing a piece's own decomposition output,
in-process, face-by-face, against the SAME real region re-converted
completely independently as its own isolated STEP file** — rather than
guessing from the written MCNP text or flipping surface signs blindly
(both tried first this session, both dead ends — see the session's own
transcript for the full, long road there).

Method, generalizable for the redesign:
1. Decompose the whole solid; identify which irreducible piece is
   suspect (here: cross-checked two ways — a direct point-sampling
   Monte Carlo of the WRITTEN definition against the real CAD solid's
   own `is_inside`, AND a per-component d1suned stochastic-volume check
   with each top-level OR-term as its own MCNP cell, SD4 = that
   component's own true CAD volume — the second one being far cleaner
   and the one to standardize on).
2. Export/convert the SAME real sub-region as its own standalone STEP
   file (when one already exists, as it did here from an earlier
   decomposition step) and decompose/convert it completely
   independently.
3. Compare face lists directly (`Surface` type + analytic params +
   `Area` + `Orientation`) between the two.

Result on the real fixture: the isolated piece has exactly **6** real
faces, matching a correct, closed-form shape. The SAME piece, as
produced by decomposing the WHOLE assembly, has **14** faces — the same
6 real ones, plus 8 extra:
- 4 genuine micro-slivers (`Area` ~0.029, `CharacteristicWidth` ~0.0125)
  — exactly what `find_sliver_faces` is built to catch, and does catch
  when called directly — but `check_solid_defects` never calls it.
- 2 redundant fragments of an **already-represented** analytic cylinder
  (same Center/Axis/Radius as a real, correctly-sized face elsewhere in
  the SAME solid) — no detector in §3 looks for "a second face on a
  surface this solid already has, that isn't registered as the same
  surface by `merge_same_surface_faces`".
- 2 more faces of real, non-negligible area (~511.5mm² each) that are
  not coplanar with EACH OTHER (so `find_overlapping_coplanar_faces`
  correctly doesn't flag them as a pair) and don't correspond to any of
  the isolated piece's own 6 real faces at all.

Volume is nearly identical between the two representations (<0.01%
relative) — the actual 3D shape is essentially correct. The real
consequence shows up two pipeline stages later: `cell_definition.py`'s
own per-face loop, faced with this messy 14-face piece, ends up writing
a term using only 2 of the 14 faces (one of the extra "phantom" planes,
plus the correctly-merged outer cylinder) — and **genuinely drops the
real, correct inner-bound cylinder face entirely** (no "Cylinder" term
for it anywhere in the final written definition). The resulting written
region evaluates to ~0.00005 of the component's own true volume in a
clean, independent d1suned check (1e6 histories, 0 lost particles) — not
a near-miss, essentially empty.

Not yet traced (left for the dedicated session, or sooner if wanted):
*which* composite-surface detection path (RevCC / Can / RoundCorner /
MultiPlane) swallows the real inner cylinder face into `omitFaces`
without ever contributing its own term for it, and why the extra 8
faces exist in the first place inside the whole-assembly decomposition
but never arise when the same region is decomposed alone (i.e. whether
they're genuine neighbor-cut artifacts of `generic_split`'s own
recursive splitting, or a repair-cascade defect that simply isn't
covered by any existing detector).

## 6. Architectural weaknesses to address in the redesign (compiled, not yet acted on)

- **`check_solid_defects` is not actually "check everything we know
  how to check"** — it's grown ad hoc, each new detector wired into its
  OWN repair function's own gate, never into the one orchestrator
  callers actually use to decide "is this solid OK". A redesign should
  either make `check_solid_defects` genuinely exhaustive (call every
  detector in §3, report every reason) or drop the pretense that it is
  one and rename/restructure accordingly.
- **No detector exists for "redundant fragment of an already-represented
  surface"** (today's finding) — distinct from `find_overlapping_coplanar_faces`
  (which needs two coplanar PLANAR faces specifically) and from
  `find_sliver_faces` (which only looks at a face's own width, not
  whether another face already covers the same analytic surface
  better). This needs its own detector, built on `is_same_*_surface`
  cross-checked against `merge_same_surface_faces`'s own grouping logic
  — if two faces of the same solid are the same analytic surface by
  `is_same_*_surface` but weren't merged into one `ShellFaceGu` (not
  contiguous, or rejected by a near-miss tolerance — see the session's
  own earlier RoundCorner/cylinder near-miss finding), that's exactly
  the signature to flag.
- **No general technique exists for "this face doesn't belong at all"**
  without an independent reference to compare against — genuinely hard
  in general, but the session's own §5 methodology (compare a
  suspect piece against the SAME region converted in isolation, when
  available) is a real, usable technique worth formalizing as a
  diagnostic tool for the redesign, not just a one-off investigation
  trick.
- **The written-definition stage has zero defensive coverage** — no
  sanity check anywhere confirms that every face of a `check_solid_defects`-
  clean solid ends up contributing *something* to the final written
  boolean term. A real face being silently absorbed into `omitFaces`
  with no term of its own is currently indistinguishable, from the
  outside, from a face that was correctly handled by a composite
  surface. A redesign should consider a structural invariant here (e.g.
  "every face index must appear in exactly one of: a basic-surface
  term, a composite's own consumed-face set with a verified region
  contribution, or an explicitly-logged sliver-discard") with a hard
  failure (not a silent gap) when it's violated.
- **Tolerance gates are scattered and independently tuned** —
  `MAX_SPLIT_RING_VOLUME_REL_CHANGE` (3e-4), `MAX_SLIVER_HEAL_VOLUME_REL_CHANGE`
  (5e-4), `MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE`/`MAX_REPAIR_VOLUME_REL_CHANGE`
  (1e-3/3e-4 at different points in the project's own history — see
  CLAUDE.md's own "Tolerances rule" entries for the full, already fairly
  disciplined measurement history here), `UNIFY_PREHEAL_TOLERANCE_FLOOR`,
  `TOLERANCE_WELD_FLOOR`, `PRE_REPAIR_MIN_VOLUME`, `DEFAULT_MIN_SOLID_VOLUME`,
  `SLIVER_SOLID_VOL_AREA_RATIO`, `DEFAULT_SLIVER_EDGE_REL_TOL`,
  `DEFAULT_MIN_FACE_WIDTH`, `SPLIT_RING_REL_TOL`, `COLLINEAR_SIN_TOL`.
  Each was measured carefully against its own specific fixture at the
  time (the project's own "measure, don't guess" discipline held up
  well) — but there is no single place describing which gate applies to
  which CLASS of repair, or why they differ from each other by the
  factors they do. The redesign should probably keep the
  per-mechanism-measured-value discipline (it has caught real
  regressions, e.g. the `shed_shutter.stp` false-positive-rejection
  entry) but organize the catalog of gates explicitly by what physical
  guarantee each one is protecting, not just list them in
  `geo/constants.py` by historical literal value.
- **STEP round-trip healing is a recurring crutch that masks the real
  defect** — `Gheal_topology`, the degenerate-torus/cone near-duplicate
  investigations, and multiple "exporting this fragment and reloading
  it fixes it, but doesn't tell you WHY" moments throughout the
  project's history all point to the same thing: the STEP reader's own
  healing is a black box that reliably *works* but never *explains*.
  The project's own established counter-technique — a lossless
  `BinTools`/`Gexport_binary`/`Gload_binary` round-trip instead of a
  real STEP round-trip, to inspect the TRUE defect without the reader's
  own silent healing in the way — should be the DEFAULT diagnostic
  reflex in the redesign, not a special trick reached for only after a
  STEP-based repair already "worked" and left the real mechanism
  unexplained.
- **No unified model of "what does a correct, healed solid actually
  guarantee"** — today's 3 layers (load/post-split/post-decomposition)
  each define their own, slightly different notion of "good enough" (a
  gate on volume change + validity + sometimes a structural count like
  `count_split_ring_pairs`), but there's no single, explicit statement
  anywhere of what invariants a solid must satisfy before
  `cell_definition.py` is allowed to trust it. Worth writing that
  invariant set down FIRST in the redesign, then deriving which checks
  are needed to enforce it, rather than continuing to accrete detectors
  reactively per fixture (which is how §3/§4 above actually grew).

## 6b. Reframing: the invariant boundary is `Gsplit`'s own input contract, not a pipeline "stage"

Started live, 2026-11-07, directly at the user's correction to a first
draft of the invariant set below: healing is not a one-time
precondition established at load time and then assumed to persist —
**a fragment must be saneado again every time it is about to enter
`Gsplit`**, not just once before `decompose_solids()` starts.

Why this matters, architecturally: `generic_split`'s own recursive
candidate search calls `Gsplit(base, tool, tolerances)` repeatedly
against an *evolving* `base` — each successful split produces new
fragments that recurse back into the SAME search, and none of them
has ever been through the load-time cascade (`Gcheck_and_repair` /
`Gspline_surface` / `_promote_open_shells`). Today's ad hoc defenses
against exactly this gap are scattered and narrow, not a declared
contract:
- the BOP-corruption-across-repeated-attempts fix (reset tolerance +
  re-run `.fix()` before every new candidate, not just once at the
  top) — `generic_split` re-establishing validity on `base` itself
  before each `Gsplit` attempt;
- `_fix_result_is_sane` gating the 3 `.fix()` call sites in
  `decom_one_generators.py` — a sanity check on the OUTPUT of a repair
  attempt, not a precondition on the input;
- `_some_real_candidate_should_have_split` — a geometry-grounded retry
  trigger, but still reactive (fires only after a stuck fragment is
  already suspected), not a check run routinely before every `Gsplit`
  call;
- `GSolid.refine()`'s two pre-heal triggers — gated on specific,
  independently-measured signatures (tolerance-weld, degenerate-plane-
  edge), not a general "is this fragment healed" check.

None of these is framed as "`Gsplit` has an input contract, and it is
the caller's (`generic_split`'s) job to guarantee it on whatever
fragment it's about to hand over" — each is instead its own
independently-discovered patch for one specific way an un-sanitized
fragment caused a specific downstream failure. The redesign should
flip this: define `Gsplit`'s own INPUT CONTRACT explicitly (the
invariant set below), and make ONE reusable, idempotent
"ensure_healed(solid, tolerances) -> HealedSolid" the thing every
caller invokes.

**Decided, 2026-11-07 (direct user design discussion): checkpoints go
BEFORE each split attempt and BEFORE `cell_definition`, not after load
and after each split.** The pipeline has 3 stages (load →
`generic_split`'s recursive loop → `cell_definition`), so there are
only 2 real gaps to place a checkpoint in; the question was which side
of each gap. Two candidate placements, compared directly against the
actual, already-documented bug history:

- **"After load, after each split"** — fails on the BOP-corruption-
  across-repeated-attempts bug (`multiplane_add_plane_cyl.stp`,
  2026-09-28) by construction: that corruption happens *during a
  FAILED split attempt* (one that splits nothing, 0 geometry change)
  — there is no successful-split output event to hook a "heal after"
  check onto, so `base` keeps degrading silently across repeated
  candidate attempts with nothing in this placement ever re-checking
  it.
- **"Before each split, before `cell_definition`"** — covers that same
  bug directly (it's literally what the bug's own already-applied fix
  does: reset tolerance + re-run `.fix()` before every new candidate,
  not just once at the top), AND still covers everything the first
  placement covers, just later: a fragment produced by a successful
  split that is never split again (an irreducible leaf) simply isn't
  healed until the final "before `cell_definition`" pass — which is
  fine, since nothing downstream needs it healed any earlier than
  that.

The second placement strictly dominates the first — it catches
everything the first one does, plus the one real failure mode the
first one structurally cannot reach. **Decision: checkpoints are
before every `Gsplit` attempt (inside `generic_split`'s own recursive
loop, re-run before each candidate, not just once per fragment) and
before `cell_definition`.**

**Cost mitigation, two-tier check** (resolves the "how expensive is
this" concern): running the FULL detector/repair cascade before every
single candidate attempt would be expensive when a fragment is tried
against many candidates in sequence before one works. Split
`ensure_healed` into:
1. a cheap pre-check (essentially just `BRepCheck_Analyzer` — already
   fast) run before every `Gsplit` attempt;
2. the full, expensive detector battery + repair cascade, invoked only
   when (a) the fragment is entering the recursive loop for the first
   time (freshly produced by a split, never checked), or (b) the cheap
   pre-check itself flags a problem.

## 6d. Feasibility check — "why not just merge every healing function into
one, called once before every split", analyzed against the project's own
history (2026-11-07)

Asked directly by the user, after seeing the inventory in §2/§3: given
how many valuable repair/detector primitives exist, why aren't they
already combined into one function called unconditionally before every
`Gsplit`? Answer, grounded in concrete precedent from this project's own
history, not just architectural taste.

**Why it isn't already done this way — 6 concrete, documented reasons:**

1. **The repairs are not additive, they're alternative diagnoses.**
   `Gcheck_and_repair` is already a small dispatcher ("first non-`None`
   wins" among `Gcollapse_split_rings`/`Gsliver_heal`/`Gdefeature`), but
   `check_solid_defects` — the function callers actually use to ask "is
   this solid OK" — never calls `find_sliver_faces`/
   `find_split_ring_faces`/`find_degenerate_plane_faces`/
   `count_split_ring_pairs` at all (§3). This isn't an oversight: each
   detector was written to feed its OWN repair's own gate, not to
   answer the general question. Merging into one call means building a
   real dispatcher over ALL of them, not concatenating function calls.
2. **Cost**: `generic_split`'s candidate loop tries many surfaces per
   fragment, recursively. The project already optimized exactly this
   once — `_some_real_candidate_should_have_split`'s own entry notes
   `generic_split` now runs `.fix()` first and compares face-count/
   volume "before paying for the expensive rebuild-candidates-plus-
   sample-points check at all", measured to cut `BC.stp`'s
   decomposition time from 69.7s to 23.6s. Running the full detector
   battery before every single candidate attempt (not just once per
   fragment) would multiply that cost by the number of candidates
   tried.
3. **Confirmed native-crash risk if forced unconditionally.**
   `ShapeUpgrade_UnifySameDomain(UnifyFaces=True)` has a confirmed,
   *accepted permanent limitation* access-violation crash
   (`Mixed/ConeSphere.stp`), plus two independently-discovered trigger
   signatures for the same crash class (`UNIFY_PREHEAL_TOLERANCE_FLOOR`
   vs. `find_degenerate_plane_faces`). A "heal with everything, always"
   function would have to keep excluding this pattern explicitly — it
   can never be blind.
4. **Direct historical precedent that "unconditional" breaks things.**
   `Gfuse_solids`'s own `refine()` call was made conditional
   (`len(gsolid.Solids) == 1 and gsolid.is_valid()`) specifically
   because "per direct user instruction, recalling that an earlier
   unconditional `refine()` here errored" — run against a not-properly-
   closed fuse result, it "historically mangled the geometry". This is
   exactly the failure mode a naive "always run the strong repair"
   `ensure_healed` would reproduce if it isn't gated by the same kind
   of precondition.
5. **Volume gates are individually measured, not interchangeable.**
   `MAX_SPLIT_RING_VOLUME_REL_CHANGE` (3e-4), `MAX_SLIVER_HEAL_VOLUME_REL_CHANGE`
   (5e-4), `MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE` (1e-3), ... each was
   measured against a real fixture before being fixed (the whole
   "Tolerances rule" discipline in `CLAUDE.md`). Merging functions
   without a deliberate, re-measured decision about their gates risks
   exactly the class of regression this project has caught before
   (e.g. the `SQUARED_LENGTH_TOL_E8` regression, or `shed_shutter.stp`'s
   own false-positive rejection from an over-tight gate).
6. **Repairs that change topology invalidate the caller's own
   face-index bookkeeping.** Already handled once, narrowly: "When
   `.fix()` does change something, the candidate list is rebuilt fresh
   against the FIXED geometry with a fresh, empty `omitfaces`" — a
   general `ensure_healed` can't be a transparent drop-in call; every
   caller holding face indices across the call needs to know whether
   anything changed.

**Verdict: yes, it's buildable — and §6b/§6c already drafted the right
shape for it** (checkpoint before every `Gsplit` attempt, two-tier
cheap/full check). What's missing is engineering, not design, provided
it respects two non-negotiable constraints learned the hard way above:
never apply the strong repairs unconditionally (always gated by
detection + by an individually-measured volume gate), and never pay
full cost on every single split attempt (hence the two-tier split).

**Concrete remaining work, in order:**
1. Make `check_solid_defects` the one real exhaustive orchestrator
   first (§8 point 3) — the full-cascade tier of `ensure_healed` should
   call *that*, not reinvent a second battery.
2. Keep the known crash-signature exclusions as explicit rules inside
   the full-cascade tier, not as an absence of checking.
3. Verify **idempotency**: running the full cascade on an
   already-healed fragment must be a true no-op, or the small volume
   drift any one repair accepts would compound across every split of
   the recursion, not just once.
4. Verify every merge of behavior against the full `Solidos/test_models`
   corpus, the same discipline already used for every tolerance change
   in this project — never merge gates "by eye".

## 6e. Empirical measurement, 2026-11-07 (branch `unified_healing`) — do the
4 missing detectors produce false positives on real, currently-clean
load-time solids?

Per the user's own explicit instruction before any code change
("primero medir. Y antes de hacer cualquier modificacion crear una
nueva branch 'unified healing'"), measured directly against the real
`Solidos/test_models` corpus (`C:\Users\Patrick\Work\Taller\
GEOUNED_workshop\Solidos\test_models`, excluding `Big_*`) before writing
any production code, on a dedicated `unified_healing` branch.

**Method**: a throwaway probe script (`ocpenv`, scratchpad, not
committed) ran `Gload_and_process_step` on every STEP file (172 unique
files found, 362 raw glob hits before de-duplicating a Windows
case-insensitive double-glob artifact; 1 subprocess per file to isolate
native crashes), and for every resulting `GSolid` recorded both the
CURRENT `check_solid_defects` result and the 4 detectors that function
does *not* call today (`find_sliver_faces`, `find_split_ring_faces`,
`find_degenerate_plane_faces`, `count_split_ring_pairs`).

**Headline numbers**: 232 real solids loaded (1 file, `Mixed/
ConeSphere.stp`, crashed exactly as already documented and accepted —
`exit=0xC0000005`, the known `UnifyFaces` access violation; 3 files
already load as `corrupted` today, unrelated to this check). **0** of
the 232 are flagged by today's `check_solid_defects` at all (at this
load-time, pre-decomposition stage, in this corpus, the gate never
fires — consistent with the project's own repeated finding that the
interesting defects mostly appear mid-decomposition on *fragments*, not
on a file's original raw solids; see the limitation note at the end).
Of those 232 "clean today" solids, **8 (3.45%, across 6 files)** would
trigger at least one of the 4 missing detectors if it were added
naively.

**Digging into those 8 — confirmed false positives, not just
theoretical risk:**

- **`find_sliver_faces` — one confirmed false positive.**
  `Reversed_Cyl_Cones/clamp.stp` solid 0: flags the same 4 corner planes
  (`CharacteristicWidth=0.0811`) that this exact file's own entry
  earlier in this document (the `add_reversedCC` / `generic_split`
  fixes) already states, in the user's own words, are real: *"estas
  caras son reales no son sliver. Es una parte de las dos plano que
  forman una esquina"*. This fixture is already fixed and verified
  (d1suned tally 0.99275, SD4 exact). Adding `find_sliver_faces` to a
  discard/repair gate unfiltered would risk "fixing" (removing) a face
  this project already proved must stay — reproducing the exact defect
  class it took real effort to understand.

- **`count_split_ring_pairs` — several distinct false-positive
  patterns, confirming its own documented caveat
  ("not a standalone defect flag", §3) empirically rather than just by
  design intent.** Re-derived every flagged pair by hand (face type,
  `CharacteristicWidth`, radii, gap) for all 7 hits:
  - `Mixed/sleeve.stp` (13 pairs, face is a `GCylinder` with
    `CharacteristicWidth=153` — nowhere near sliver-scale) and
    `Mixed/ring.stp` (most of its 28 pairs) and `Mixed/
    base_all8_to_cylinder.stp` (5 pairs, `CharacteristicWidth=21`):
    gaps of **1e-13 to 1e-9 mm** — pure floating-point noise from a
    periodic surface's own duplicated seam edges, not a riser defect at
    all.
  - `esfera/Barrel_bottom.stp` (the *historical motivating fixture* for
    `find_split_ring_faces`/`Gcollapse_split_rings` — already repaired
    in this corpus copy, now only 5 faces): 12 pairs, gap≈0.0223mm,
    between a plane (`CharacteristicWidth=103`) and a sphere
    (`CharacteristicWidth=3100`) — ordinary, non-sliver geometry; a
    small real radius difference (1880.01 vs 1879.6mm), not a
    duplicated trim.
  - `Torus/example.stp` (3 solids): gaps of ~5mm on `CharacteristicWidth
    ~9mm` faces — small enough to possibly be a genuine residual
    artifact on this already-known-fragile fixture, but would need its
    own dedicated investigation, not a blind count-based gate.

- **`find_split_ring_faces` and `find_degenerate_plane_faces`: 0 hits,
  0 false positives** across all 232 solids — look safe to add to an
  exhaustive *diagnostic* (not yet the discard gate) as-is, at least at
  this load-time stage.

**Conclusion — directly validates §6d's own caution, with real
numbers**: a naive "throw every detector into one function, call it
everywhere" would have shipped at least 2 concrete, confirmed
regressions (clamp.stp's real corner planes misread as slivers;
Barrel_bottom.stp's own already-fixed geometry re-flagged by a count
that never should have been read as a boolean "defect present").
`count_split_ring_pairs` must stay a *before/after comparator* for a
specific repair, never a standalone yes/no signal. `find_sliver_faces`
needs either a refinement (e.g. excluding small-but-load-bearing
junction planes somehow) or must never be wired into anything that can
discard/remove a face without a second, independent confirmation that
removing it is geometrically safe — which is exactly what `Gdefeature`
already tries to be, separately from detection.

**Known limitation of this measurement, important for the next step**:
this only covers the *load-time* layer (§1's layer 1) — raw solids as
read from a file, before `decompose_solids()` ever runs. Per §5's own
case study, the genuinely interesting defects (redundant surface
fragments, extra slivers) mostly show up on *intermediate fragments*
produced mid-recursion by `generic_split`, which this probe never
touches. The 0% baseline hit rate for the existing gate at this stage,
and the low (but non-zero, and partly false-positive) hit rate for the
4 new detectors, should not be read as "the whole corpus is this safe"
— only as "the load-time layer specifically is this safe, for these 4
detectors". A second measurement pass inside `generic_split`'s own
recursion (instrumenting it to run the same 4 detectors on every
fragment it produces, before any repair) is the natural next
measurement, not yet done.

### 6e.1 Follow-up, same session: are the 2 reported false positives real bugs?

Verified both of §6e's flagged issues directly before touching any code,
per the user's own explicit instruction to investigate before fixing.

**`find_sliver_faces` on `clamp.stp` — confirmed NOT a detector bug, a
tolerance-usage mistake in the probe.** `solid.Volume = 314.646mm^3` ->
`tolerances.scaled(volume).min_face_width = 0.068015` (vs. the bare,
unscaled `0.1` the probe used). `find_sliver_faces(solid, 0.1)` -> 4
hits; `find_sliver_faces(solid, 0.068015)` -> **0 hits**. Exactly
matches this fixture's own already-documented numbers elsewhere in this
file (`generic_split`'s own 2026-10-05/06 entry: "CharacteristicWidth
=0.081115... sitting strictly between the default min_face_width=0.1
... and this solid's own scaled 0.068015"). **Lesson for the unified
design**: any width-based detector MUST be called with
`tolerances.scaled(solid.Volume)`, never the bare field — this is not
optional polish, it's the difference between a real false positive and
none at all.

**`count_split_ring_pairs` — one confirmed, fixable, generalizable bug;
one confirmed real (not false-positive) finding on closer inspection.**

- **Pure floating-point noise (fixable)**: `sleeve.stp` (13 pairs) and
  `base_all8_to_cylinder.stp` (5 pairs) have gaps of `1e-13` to `2e-11`
  mm — literal duplicate-edge-representation noise. Raising the gap's
  lower bound from exactly `0.0` to `POINT_POINT_TOL` (1e-5mm) drops
  both to **0**, with zero effect on every other file's real gaps
  (which sit 3-6 orders of magnitude above that floor).

- **`ring.stp` — a real, generalizable, fixable false positive, root
  cause identified.** After the floor fix, 28 pairs collapse to 4,
  all on face#0 (a plain `GCylinder`, `CharacteristicWidth=9.15mm` --
  nowhere near sliver-scale), gap=0.27mm. Dumped the face's full edge
  list: it's a genuine, intentional notched/grooved real feature (one
  full 540mm-circumference circle at axial position +0.135mm, several
  partial arcs at -0.135mm and -5.135mm, connected by perfectly
  ordinary **5mm**-long straight edges -- the slot walls). `find_split_
  ring_faces` (the sibling detector) already requires the circle pair
  be bridged by a *pathologically short* connecting edge
  (`max(diag*rel_tol, MIN_SLIVER_EDGE_LENGTH)` = 0.028mm for this
  solid) before calling it a defect; `count_split_ring_pairs` has no
  such guard at all -- it only checks circle geometry (coaxial, same
  radius, close centers), so any real, intentional small groove/step
  whose two ends happen to share a radius trips it. **The fix**: add
  the same "bridged by a genuinely short edge" requirement
  `count_split_ring_pairs` is missing, matching its own sibling's
  already-validated convention.

- **`esfera/Barrel_bottom.stp` — re-examined, confirmed a GENUINE
  finding, not a false positive.** Dumped all 5 faces' full edge lists:
  face#1 (the plane, `CharacteristicWidth=103`) and face#4 (the sphere,
  `CharacteristicWidth=3100`) each carry 4 `GLine` edges of length
  **exactly 0.0362817mm** -- genuinely pathologically short (threshold
  for this solid is 0.56mm) -- directly bridging the near-duplicate
  `R=1880.011859`/`R=1880.0`/`R=1879.6` circles. This is the real
  "split boundary ring" signature (a short connector between
  near-coincident circles), just not currently flagged by
  `check_solid_defects` because of the deliberate `L4_pipes.stp` rule
  (2026-08-29): *"a short edge whose faces are all otherwise normal is
  NOT a defect"* -- the host faces here (103mm/3100mm wide) are not
  sliver-scale, so the composite "degenerate/sliver geometry" reason
  correctly stays silent. In other words: this fixture, already
  "repaired" and already converting correctly (d1suned tally 0.9997, 0
  lost particles, per its own earlier entry in this file), still
  carries a **genuine, if currently harmless, microscopic residual**
  of the original defect (2 small `CharacteristicWidth=0.41mm` planes
  bridging the radius change, not the original 6 big risers, but not
  zero either). The proposed "bridged by a short edge" fix above would
  **NOT** suppress this one (the 0.036mm edges genuinely qualify as
  short) -- confirming the fix is a real improvement, not a blanket
  that happens to hide everything.

- **`Torus/example.stp` (solids 5/7/8) -- resolved, same session,
  same method.** Dumped all 3 solids' full face/edge lists. The
  circle pairs are bridged by `GBSpline` edges of 5-38mm -- a torus's
  own real meridian curve between two slightly-different-radius
  latitude circles (solid#5, face#3) or a cylinder-to-torus transition
  (solid#7/#8) -- comfortably above every one of these solids' own
  "pathologically short" thresholds (0.55-0.60mm). Confirmed a false
  positive, same root cause and same fix as `ring.stp`.

**Net effect of the proposed `count_split_ring_pairs` fix (floor +
short-bridging-edge guard), as a pure diagnostic reading**: eliminates
`sleeve.stp`/`base_all8_to_cylinder.stp` (noise), `ring.stp` and all 3
`Torus/example.stp` solids (legitimate geometry) entirely; correctly
KEEPS `Barrel_bottom.stp`'s genuine residual finding. **This conclusion
held up only as a standalone diagnostic -- see the follow-up
immediately below, where applying the exact same fix to the function
`Gcollapse_split_rings` actually calls broke that real repair's own
gate on this exact fixture.**

### 6e.2 Follow-up, same session: the fix was applied, verified against
unit tests, then found to break the REAL repair gate on
`Barrel_bottom.stp` itself -- reverted

Per the user's explicit instruction ("sí termina" -- finish the
`example.stp` investigation first), §6e.1's open item was closed (see
above), and the fix (gap floor `0.0` -> `POINT_POINT_TOL`, plus the
"host face must carry a genuinely short edge" guard) was then applied
to `geo/solid_defects.py::count_split_ring_pairs`. `tests/geo` +
`test_cadtocsg.py` passed clean on `ocp` (260 passed, 7 skipped, zero
regressions) -- but per this project's own standing lesson ("a unit
suite passing is not the same as verifying against the real corpus/
real use", restated explicitly in §6d point 4), a direct re-run of the
real `Gload_and_process_step` pipeline on `Barrel_bottom.stp` was ALSO
done, not just the unit suite -- and it failed: the file no longer
loads at all (`corrupted_indices=[0]`, `gsolids=[None]`).

**Root cause, traced by temporarily instrumenting `Gcollapse_split_rings`
itself (not guessed)**: the fix's OWN standalone-diagnostic reading
(§6e.1, "correctly KEEPS Barrel_bottom.stp's genuine residual finding")
was true in isolation but irrelevant to how the function is actually
*used* -- `Gcollapse_split_rings` calls it twice, as a strict
`pairs_after >= pairs_before -> reject` gate, not as a one-shot reading.
Direct comparison, real numbers, both logics, on the real raw (11-face)
solid and the real repaired (5-face) result:

| | OLD (gap floor 0.0, no guard) | NEW (floor+guard) |
|---|---|---|
| RAW (11 faces), `pairs_before` | **18** (12 real `0.0223mm`-gap pairs on the 6 risers + the sphere, **+ 6 pure noise pairs at `gap=4e-17` on the plane face**) | **12** (the noise pairs correctly dropped) |
| REPAIRED (5 faces), `pairs_after` | **12** (same 12 real pairs, risers now gone) | **12** (same 12 real pairs -- *and the plane face's own 6 pairs, which were noise-level `4e-17` on the RAW solid, are now a real, non-noise `0.0223mm` gap*, because the resewing operation itself changed how that one discrepancy is represented) |
| `pairs_after < pairs_before`? | `12 < 18` -> **True, accepted** | `12 < 12` -> **False, rejected** |

The key, non-obvious finding: the SAME underlying geometric
discrepancy (face#1's own `R=1880.011859` vs `R=1879.6`/`R=1880.0`
boundary mismatch) is numerically **invisible as noise** (`4e-17`) in
the RAW, pre-repair solid, and becomes **visible as a real small gap**
(`0.0223mm`) only *after* `Gcollapse_split_rings`' own resewing step
changes its representation -- it is not that the defect got worse, it's
that the SAME fact got re-expressed differently by the repair itself.
A single shared numeric floor cannot consistently classify both
snapshots of the same fact as "noise" or "real" at once, and the
before/after `<` comparison this repair's own accept-gate depends on
breaks exactly because of that inconsistency. This is a confirmed,
traced, understood mechanism, not a mystery -- see the updated
docstring in `count_split_ring_pairs` itself for the full account.

**Action taken: fully reverted.** `count_split_ring_pairs` is back to
its original form (gap floor `0.0`, no short-edge guard) -- re-verified
`Barrel_bottom.stp` loads correctly again (`check_solid_defects` clean,
5-face result, matching the original, already-documented state) and
the full `tests/geo` + `test_cadtocsg.py` suite is green (260 passed, 7
skipped) on the unchanged code. The function's own docstring now
records this whole attempt-and-revert, including the exact mechanism,
as a permanent warning against re-attempting the same fix the same way.

**The real lesson for the redesign, stated precisely**: `count_split_ring_pairs`'s
real role (a before/after comparator for ONE specific repair call) and
a hypothetical general-purpose "is there a split-ring-like pattern
here" diagnostic are **not the same function**, even though they look
identical in isolation on 6 of 7 test cases. A safe path forward, if
this is revisited, is a genuinely SEPARATE function for the diagnostic
reading (never wired into `Gcollapse_split_rings`' own gate), verified
independently -- not a shared implementation with an added guard. More
broadly, this is a second, concrete, costly-in-investigation-time
confirmation of §6d's own point 4 ("verify every merge of behavior
against the full corpus... never merge gates by eye") and point 5
("volume gates are individually measured, not interchangeable" --
generalizes here to "detector logic is individually measured against
its own real call site, not interchangeable either, even when a
standalone reading looks clean").

### 6e.3 Follow-up, same session: the other 2 "missing" detectors were
checked against their real call site too, with a different outcome for
each -- one wired in safely, one deliberately left out, 2026-11-07
(branch `unified_healing`, commit pending).

Continuing from §6e.1's own measurement (`find_sliver_faces`: 1 hit,
`clamp.stp`; `find_split_ring_faces`/`find_degenerate_plane_faces`: 0
hits each, across the whole 232-solid corpus), the same "verify against
the real call site, not a standalone reading" discipline §6e.2 just
re-learned the hard way was applied to these two before touching
`check_solid_defects`:

- **`find_split_ring_faces`** already has a working repair in the exact
  cascade `check_solid_defects` gates (`Gcollapse_split_rings` calls it
  internally) -- adding it as a named reason only widens WHEN that
  repair gets tried (to a solid with risers but none of the 3 existing
  reasons), with a repair step already there to catch it. Safe.

- **`find_sliver_faces`**'s one real hit, `clamp.stp`, was confirmed to
  be a tolerance-scaling artifact, not a real defect: `tolerances.
  min_face_width` (bare, 0.1) flags its 2 real corner-forming planes
  (`CharacteristicWidth=0.0811mm`); `tolerances.scaled(clamp_volume).
  min_face_width` (0.068, the correctly-calibrated value for this
  314.6mm^3 solid) does not. But tracing this to its real call site
  found something more consequential than "this one solid is a false
  positive": **`Gcheck_and_repair` (occ + ocp) was already passing the
  BARE `tolerances.min_face_width` into `check_solid_defects`, at both
  of its own call sites** -- a pre-existing gap, not something this
  session introduced. It caused no live false positive YET only because
  the one check that reads `min_face_width` today (`degenerate/sliver
  geometry`) additionally requires the same face to touch a
  pathologically short edge, which `clamp.stp`'s own narrow planes
  don't -- `find_sliver_faces` has no such extra requirement, so wiring
  it in without first fixing the scaling would have reintroduced
  exactly the false positive already investigated and explained in
  §6e.1's hypothesis 1. `Gcheck_and_repair` now scales via a new,
  duck-typed `_scaled_min_face_width(tolerances, volume)` helper in each
  engine's own `repair.py` (`getattr(tolerances, "scaled", None)`,
  matching `surface_geometry.py`'s own established `getattr(tolerances,
  "relativeTol", False)` convention for an optional, GEOUNED-only field
  -- `geo` itself has no dependency on `geouned.Tolerances`, and this
  degrades to the bare value if a caller ever passes a bare
  `GeoTolerances` with no `.scaled` method). Safe, once scaled.

- **`find_degenerate_plane_faces`**, by contrast, was confirmed to have
  **no corresponding repair step anywhere in `Gcheck_and_repair`'s own
  cascade** (`Gtrim_overlapping_coplanar_faces`/`Gcollapse_split_rings`/
  `Gsliver_heal`/`find_short_edges`+`Gdefeature` -- none target a
  degenerate edge on a plane face; `find_short_edges` explicitly
  EXCLUDES edges below `DEGENERATE_EDGE_LENGTH_FLOOR`, which is exactly
  what this detector looks for). Its own docstring already says why:
  it was built specifically for `GSolid.refine()`'s own pre-heal gate
  (a different code path entirely, reached during `generic_split`'s
  recursion, not load time), where it correctly triggers a STEP
  round-trip heal before a crash-prone native call. Wiring it into
  `check_solid_defects` would cost nothing on today's corpus (0 hits)
  but would silently convert any FUTURE solid that does trip it into a
  guaranteed load-time discard (`Gcheck_and_repair` returns `(solid,
  False)` the moment no cascade step resolves a flagged defect) --
  trading a currently-correct, working repair path for a worse one, for
  zero present-day benefit. **Deliberately left out of
  `check_solid_defects`**, with the reasoning recorded in that
  function's own docstring so it isn't re-attempted blindly later.

**Result**: `check_solid_defects` gained 2 new reasons ("split boundary
ring", "sliver face"); `count_split_ring_pairs` and
`find_degenerate_plane_faces` stay out, each for its own
independently-verified reason. `Gcheck_and_repair` (both engines) now
scales `min_face_width` to the solid's own volume before calling in,
closing the latent gap `find_sliver_faces` would otherwise have
reopened.

**Verified against the real production path** (`Gload_and_process_step`
-> `Gcheck_and_repair`, not a standalone diagnostic reading), the whole
`Solidos/test_models` corpus (181 unique files, excluding `Big_*`): the
SAME 3 pre-existing load-time discards (`Decomposed/
modelcell_cut1_v2_piece66.stp`, `Decomposed/SCDR_90_piece2.stp`,
`Mixed/SCDR_90_hollow.stp`) and the SAME 1 pre-existing native crash
(`Mixed/ConeSphere.stp`, `ShapeUpgrade_UnifySameDomain`'s own documented
access violation) -- **zero new discards anywhere**. `clamp.stp` (3
solids), `esfera/Barrel_bottom.stp`, `Mixed/sleeve.stp`, `Mixed/
base_all8_to_cylinder.stp`, `Mixed/ring.stp`, and `Torus/example.stp`
(12 solids) -- every fixture this whole investigation touched -- all
load with an empty `corrupted_indices` and unchanged face counts. Full
`tests/geo` + `test_cadtocsg.py` green on both engines (ocp: 260
passed/7 skipped; occ: 260 passed/6 skipped), zero regressions.

## 6c. Invariant set — draft

**A `GSolid` must satisfy, at every one of the boundaries in §6b above
(not just once at load time):**

1. **Full topological validity** — not just the solid-level
   `BRepCheck_Analyzer` result (Recipe 3 proved that can pass while an
   individual face/wire is invalid); check shells/faces/edges/vertices
   too.
2. **Volume/area sanity** — `Volume > 0`, `Area > 0`,
   `Volume/Area` not pathologically small for the WHOLE solid (no
   solid-level sliver).
3. **No phantom/redundant duplicate faces** — every pair of faces on
   the same analytic surface (by the registry's own identity
   tolerance) is either merged into one `ShellFaceGu`, or is
   genuinely, provably non-contiguous (a real distinct feature, not a
   near-miss artifact) — see §6's "redundant fragment" gap and today's
   §5 case study (2 of the 8 extra faces on piece #15 were exactly
   this, undetected by anything in `geo/solid_defects.py`).
4. **No face-level slivers** relative to the solid's own scale
   (`CharacteristicWidth >= min_face_width`), unless explicitly
   diagnosed as a known, already-repaired sliver pattern (riser,
   overlapping-coplanar phantom, retrace-spike residue).
5. **No malformed face boundary wire** (a vertex of edge-degree > 2,
   Recipe 2's own signature) — if found, the solid cannot currently be
   healed and must be excluded, not silently passed through.
6. **No degenerate edge with a corrupted (inflated) tolerance** left
   unaddressed before any native unify-adjacent operation known to
   crash on it.
7. **One physical region = one connected component** — neither a
   phantom-cut merge (two regions joined only along zero-area contact)
   nor an accidental non-manifold split of a single region.

**A written boolean definition must satisfy, relative to a
verified-healed solid:**

A. **Every face index is accounted for in exactly one of 3 ways, each
   explicitly recorded**: (i) contributes its own basic-surface term
   directly; (ii) is consumed by a composite meta-surface whose output
   region *provably depends* on it (not silently absorbed with zero
   contribution — this is exactly what failed in today's §5 case:
   the real inner cylinder face was added to `omitFaces` with no
   record of why, and no term of its own ever appeared); (iii) is
   explicitly discarded as a diagnosed sliver/redundant/phantom face,
   with the resulting volume loss measured and within tolerance.
B. **No `omitFaces.update(...)` without a linked reason** — today it's
   sprinkled through `simple_solid_definition`'s per-face loop with no
   bookkeeping of why a face was omitted; this is the exact mechanism
   that swallowed the real face in today's case study without leaving
   any trace to follow.
C. **Routine point-sampling verification as a pipeline step, not just
   an after-the-fact debugging technique** — compare the written
   definition against `GSolid.is_inside` on a sample of points,
   automatically, for every solid, before accepting its definition as
   final (today this is only ever reached for, ad hoc, after a bug is
   already suspected — see §5's own methodology, which should become
   routine instrumentation rather than a one-off investigation tool).

## 6f. `ensure_healed` built and wired in -- a first real step of the
unification, 2026-11-07, branch `unified_healing`

Picks up directly from §6e's own measurement work and §6d's "concrete
remaining work" list. Two user corrections shaped the final shape of
this step, both worth keeping verbatim for the reasoning:

- After a first proposal to heal `meta_obj.Solids` upstream of both the
  cell definition AND the exported CAD (in `core.py::_decompose_target`,
  before `set_cad_solid()`), the user pushed back on the premise, not
  just the placement: *"no podemos asumir solidos con 'validez
  distinta' segun la parte donde se usa... tenemos que ir sobre una
  base que siempre tratar con el mismo solido 'valido' en todos los
  casos. Para esto primero vamos a volver a investigar el crash de
  ConeSphere.stp"* -- i.e. fix the thing that made native healing
  genuinely unsafe in some contexts FIRST, before deciding how broadly
  to apply it. That investigation (see CLAUDE.md's own `ConeSphere.stp`
  entry) found and fixed the real root cause (`ShapeUpgrade_
  UnifySameDomain`'s `KeepShape`, never tried before), not a workaround
  -- closing the strongest argument against applying healing more
  broadly.
- With that resolved, the user revised the placement decision itself:
  *"Antes de simple_solid_definition (y tambien parra la generacion de
  void) es suficiente que el solid se pueda procesar correctamente para
  repoducir la expresion booleana"* -- i.e. the bar is narrower than
  "one canonical healed geometry propagated everywhere": each consumer
  that needs to WALK a solid's topology to reproduce a boolean
  expression just needs ITS OWN read of that solid to succeed. The
  exported `CADSolid`/STEP output is a separate concern, not something
  this step needs to re-point.

**What got built**: `geo/solid_ops.py::ensure_healed(solid, tolerances)
-> (result, ok, changed)` -- a thin, engine-agnostic dispatch to
`Gcheck_and_repair` (itself already more complete after §6e.3's own 2
new detectors), re-exported from `geo/__init__.py`. Deliberately only
one tier: the cheap `is_valid()` pre-check §6b's own two-tier design
describes for `generic_split`'s hot loop is not implemented, since
nothing calls it there (see below).

**Where it's wired, and where it deliberately ISN'T**:
- `cell_definition.py::build_definition`'s own per-solid loop, right
  before `simple_solid_definition` reads a piece's faces. This is the
  ONE real call site for now, confirmed with the user directly (asked
  explicitly whether void generation also needed its own call site,
  given it's named in the same instruction) -- investigated first:
  void's own real work (`VoidBox`/`get_void_complementary`) reads each
  solid's already-built `.Definition`, never raw topology, so it
  benefits automatically from healing done once here. The one place
  void DOES touch raw `CADSolid` directly (`check_intersection`/
  `piece_enclosure_split`, enclosure-containment only) was confirmed
  out of scope for this pass.
- `generic_split`'s own recursive `Gsplit` loop -- untouched, per
  direct user decision earlier the same session (see §6d/§8): its
  existing ad hoc mechanism (tolerance reset + `.fix()` before every
  candidate) already does the job, heavily tuned, with a real
  historical crash precedent (`SCDR_90.stp`, re-investigated the same
  session per the user's own request and confirmed to be a DIFFERENT,
  already-closed bug -- the `GCompound` wrap/re-extract corruption, not
  a UnifyFaces signature at all) warning against replacing it without a
  demonstrated need.

**A real, previously-unexplained bug closed as a direct side effect,
found while verifying the wiring against the whole corpus (not sought
out)**: a 181-file differential (decompose + `build_solid_definition`,
comparing each solid's own written `Definition` text) found exactly 1
real difference anywhere in the corpus -- `Mixed/
base_all8_to_cylinder.stp`. Traced directly: piece#20 of its own
decomposition carries a genuine sliver face that `find_sliver_faces`
(added to `check_solid_defects` earlier the same session, §6e.3) now
catches and `Gsliver_heal` resolves (14 -> 10 faces, volume conserved
to ~4e-6 relative). A real, same-NPS d1suned before/after (`git
stash`-isolated to just this wiring) confirms it's the real cause, not
correlation: tally `0.84856` (94.7 sigma, 1 lost particle) -> `0.99925`
(0.5 sigma, 0 lost particles) -- matching the project's own
already-documented `0.84323`/116.20-sigma figure for this exact,
long-flagged "priority item for a future session" (see CLAUDE.md and
the now-closed `project_next_session_rrc21_2026_10_30` memory). Neither
of that memory's own 2 standing hypotheses for the cause
(`tangent_cut_plane`'s torus-tangency mechanism; the `spline_2D`
tolerance change) was right -- a concrete reminder that tracing beats
guessing even when a plausible-looking "most recently touched" culprit
is sitting right there.

**Verified**: `tests/geo` + `test_cadtocsg.py` green on both engines
after each step (ocp 260 passed/7 skipped, occ 260 passed/6 skipped);
the 181-file corpus differential found zero OTHER differences anywhere
-- the wiring is inert everywhere this exact defect class doesn't
occur, and a real, independently-measured win where it does.

**What this step does NOT claim**: it is not "check_solid_defects is
now exhaustive" (§6c's own invariant set still has 3 uncovered gaps --
malformed boundary wire, a degenerate edge with a corrupted tolerance,
"one physical region = one connected component" -- each needing its
own detector + its own corpus-measured false-positive check before
being added, same discipline as `find_sliver_faces`/`find_split_ring_
faces` this session). It's one real, measured, narrowly-scoped step,
not the finished redesign.

## 6g. Confirmed: there is still NO single integrated healing function
-- `_raw_bop_split` keeps its own, separate, untouched cascade

Asked directly by the user, same session, after seeing `Gsliver_heal`
called from `geo/{occ,ocp}/split.py` (IDE selection on that exact line):
*"al final no has hecho una funcion de saneamiento integrada... es en
_raw_bop_split que se hace el saneamiento de los solidos, ¿es asi?"* --
and, once confirmed, a direct follow-up: *"las modificaciones hechas
ahora hacen que ahora base_all8_to_cylinder.stp traduce bien. ¿Es por
los test sobre solidos que ahora hace dentro de _raw_bop_split que
antes no hacias?"*

**Both questions answered by reading the code directly, not by
inference**: `git log` on `geo/{occ,ocp}/split.py` shows its last real
commit is `4d25795`, from before this session started -- confirmed also
absent from `git status`'s own modified-file list for the whole
`unified_healing` branch. **`_raw_bop_split` was not touched at all this
session.** Its own `Gsliver_heal` call (line 161 occ / equivalent ocp)
and its own `find_sliver_faces` check earlier in the same function
(line 138, gating whether a `BRepCheck`-valid raw BOP piece still needs
healing) both already existed before `unified_healing` started -- this
is `_raw_bop_split`'s own independent, long-standing, heavily-tuned
cascade (phantom-cut separation -> non-manifold repair ->
`Gsliver_heal` -> `Gheal_topology` round-trip), running DURING
decomposition, on BOP-split fragments -- not the thing this session
built or changed.

**So the `base_all8_to_cylinder.stp` fix is confirmed NOT to come from
`_raw_bop_split`.** It comes from a NEW healing pass that did not exist
before today, running AFTER decomposition is already finished: `check_
solid_defects` (`geo/solid_defects.py`) gained a `find_sliver_faces`
check this session (§6e.3), and `ensure_healed` (§6f, this same day)
wires `Gcheck_and_repair` -- which now runs that check -- into `cell_
definition.py::build_definition`'s own per-piece loop. Piece#20 of this
file's own decomposition passes through `_raw_bop_split` completely
unaffected (same as every prior session), carrying its sliver face all
the way to `m.Solids`; the fix happens one stage later, the first time
anything calls `Gcheck_and_repair` on that already-decomposed piece at
all.

**Net picture, confirmed with the user and left as-is for now (explicit
scope decision, not an oversight)**: `Gsliver_heal` and the other repair
primitives in `repair.py` are genuinely SHARED kernel-level functions,
but they are orchestrated by **two separate, independently-tuned
cascades** that never call into each other: `_raw_bop_split`'s own (post-
split, inside `generic_split`'s recursion) and `Gcheck_and_repair`'s
(load time, and now also post-decomposition via `ensure_healed`). This
is exactly the state §6f's own "what this step does NOT claim" section
already flagged -- `generic_split`/`_raw_bop_split` was deliberately left
out of `ensure_healed`'s wiring, per the user's own standing decision not
to touch that loop's existing ad hoc mechanism without a demonstrated
bug. The §6b "invariant boundary is `Gsplit`'s own input contract, not a
pipeline stage" framing describes what a true unification would need to
look like; this session did not attempt it, and the user confirmed
leaving it as documented, scoped future work rather than acting on it
now.

## 6h. Follow-up, same session: is cost the real reason
`check_solid_defects` isn't run inside `_raw_bop_split`?

Direct follow-up question from the user, right after §6g: *"¿por qué
check_solid_defects no se hace dentro de `_raw_bop_split`? ¿Es porque
consume mucho tiempo y ralentizaría la decomposicion?"*

**Answer: cost is one of 6 documented reasons (§6d above), not the only
one, and arguably not the dominant one.** Restated directly from §6d
for this specific question, since it already analyzes exactly this
"why not just merge everything into `generic_split`'s loop" question:

1. The detectors are alternative diagnoses feeding their own repair's
   own gate, not a general-purpose battery -- merging means building a
   real dispatcher, not concatenating calls.
2. **Cost, the thing asked about** -- real, measured precedent:
   `generic_split` was already optimized once specifically to avoid
   paying an expensive check on every candidate
   (`_some_real_candidate_should_have_split`'s own `.fix()`-first
   short-circuit cut `BC.stp`'s decomposition from 69.7s to 23.6s).
   Running the full detector battery before every single `Gsplit`
   candidate attempt (not once per fragment) would multiply that cost
   by the candidate count.
3. Confirmed native-crash risk if any strong repair runs
   unconditionally (`UnifyFaces=True`, now fixed at the root for
   `ConeSphere.stp`'s own specific signature, but the general
   "never blind" rule still holds for any future signature).
4. Direct historical precedent (`Gfuse_solids.refine()`) that an
   unconditional repair call has already, concretely, mangled geometry
   in production before.
5. Volume-conservation gates are individually measured per repair, not
   interchangeable.
6. A repair that changes topology invalidates the caller's own
   face-index bookkeeping -- not a transparent drop-in call.

**What already exists for the cost problem specifically, but is not yet
built**: §6b's own two-tier design -- a cheap `BRepCheck_Analyzer`-only
pre-check before every `Gsplit` attempt, promoting to the full,
expensive `check_solid_defects` + repair cascade only when the fragment
is new to the recursion or the cheap check itself flags something. This
is designed, not implemented; `ensure_healed` today is ONLY the full
tier, wired ONLY into `build_definition` (§6f) -- never into
`generic_split`.

**Decision, confirmed directly by the user: leave as-is.** *"vale
documenta esto. Por ahora funciona y lo dejamos así. A lo mejor en el
futuro habra que retomar estos puntos de saneamiento."* No code change
from this exchange -- `_raw_bop_split` keeps its own independent
cascade, the two-tier design from §6b remains unimplemented, and this
whole area (§6b/§6d/§6g/§6h together) is the concrete starting point if
a future session resumes the "solid healing" unification -- read these
four sections first, in order, before writing any new code there.

**Immediate follow-up, same session: a full corpus + d1suned run,
requested directly** ("¿has pasado el corpus completo geouned +
d1suned para todos los solidos que solemos chequear?") -- not yet done
up to this point; only the cheap 181-file decompose+build_solid_
definition differential (§6f) and two single-file d1suned checks
(`ConeSphere.stp`, `base_all8_to_cylinder.stp`) had been run. Warranted
specifically because the `ConeSphere.stp` fix touches `_native_fix`,
which every solid passes through unconditionally at load time, on both
engines -- a change with corpus-wide reach, not scoped to one fixture.

Run via the established pipeline (`run_all_conversions_{ocp,occ}_
parallel.py` -> `run_test_models_{ocp,occ}_d1suned.sh` ->
`analyze_test_models_{ocp,occ}.py`, both engines launched in parallel
per the user's own instruction), 175 files of `Solidos/test_models`
excluding `Big_*`:

- **Conversion**: 163/175 on BOTH engines -- the 9 `base_identified_
  faces` non-solid fixtures plus 3 pre-existing failures (`SCDR_90_
  piece2`, `modelcell_cut1_v2_piece66`, `SCDR_90_hollow`). Up from the
  project's own prior 162/175 baseline (4 known failures, `ConeSphere.
  stp` among them) -- `ConeSphere.stp` now converts on both engines.
- **d1suned**: identical buckets on both engines -- 226 solid-cell
  tallies, 213 (94.2%) within 2 sigma, 11 marginal (the same files this
  project has repeatedly documented elsewhere: `fwd_pipe`, `rc24`/
  `rrc1`/`rrc4`/`rrc12`/`rrc5`, 4 `Torus/*`, `SCDR_solid19_solid26`),
  exactly 2 beyond 3 sigma (`shed_solid.stp`, 3.04 sigma, the
  already-documented MC-noise-on-a-small-cell case; `modelcell_cut1_
  1.stp`, 3.30 sigma -- previously recorded as marginal in other corpus
  runs in this same project, i.e. the same file sitting right at the
  historical marginal/fail boundary across independent MC runs, not a
  new defect introduced this session), **0 lost particles anywhere
  (175/175, both engines)**.
- Neither `ConeSphere.stp` nor `base_all8_to_cylinder.stp` shows up in
  either bucket -- both land cleanly within 2 sigma, on both engines.

**Conclusion**: the full, real, corpus-wide verification this project's
own established discipline calls for (not just the cheap structural
differential) confirms both of this session's fixes hold with zero
collateral regression anywhere in the usual test corpus, on both
engines. This closes the outstanding "has the full corpus+d1suned check
actually been run" gap this section started from.

## 7. A related, explicitly abandoned prior attempt (cautionary precedent)

`project_mosaic_surface_design` (memory, branch `mosaic_surface`, commit
`6b60ac4`) — a different kind of restructuring (not solid healing, a
meta-surface construction approach), explicitly abandoned by the user
("este método no va a ningún sitio"). Not solid-healing-related, but
worth remembering as a precedent for *recognizing* when a redesign
direction isn't working and stopping, rather than committing further.

## 8. Suggested starting point for the dedicated session

1. Finish tracing §5's own open thread first (which composite-surface
   path drops the real cylinder face) — it's a live, concrete,
   already-mostly-isolated bug, and finishing it will almost certainly
   surface one more precise data point for the redesign's own detector
   inventory before design work starts.
2. ~~Write down the invariant set... before writing any new detector
   code.~~ **Done, 2026-11-07 — see §6b/§6c.** Checkpoint placement
   decided (before every `Gsplit` attempt + before `cell_definition`,
   not after load/after each split — §6b), the invariant set itself
   drafted (§6c). Not yet done: actually implementing the two-tier
   `ensure_healed` (cheap `BRepCheck_Analyzer` pre-check + full cascade
   on first-entry/pre-check-failure) as real code, and wiring it into
   `generic_split`'s own candidate loop and `simple_solid_definition`'s
   own entry point.
3. Decide whether `check_solid_defects` becomes genuinely exhaustive or
   is restructured/renamed, and do that FIRST — every other detector
   added later should plug into one real orchestrator from day one,
   not get its own bespoke gate the way §2/§3 grew historically. This
   orchestrator is naturally what `ensure_healed`'s own "full cascade"
   branch (§6b) should call.
4. Consider formalizing the "compare against an independently-converted
   reference" technique from §5 as a reusable diagnostic tool (it found
   the real root cause far faster than sign-flipping or text-based MCNP
   experiments did this session, and generalizes to any suspect piece
   that has, or can cheaply be given, a standalone STEP export).
