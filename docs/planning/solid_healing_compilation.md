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
