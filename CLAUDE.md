# GEOUNED — FreeCAD → pyOCC/CadQuery migration context

This file summarizes decisions made in a prior planning conversation
(claude.ai chat) about the GEOUNED codebase, so Claude Code has full
context without needing the original conversation pasted in.

This file was reorganized on 2026-09-12: it used to contain the full,
12,000+ line chronological investigation log inline. That log is now at
`docs/history/geouned_migration_log.md`, unchanged and unabridged — this
file keeps only the project overview, the current architecture, and a
current-state summary. See "Reference docs" below for where everything
else moved.

## Project

- GEOUNED (https://github.com/GEOUNED-org/GEOUNED): converts CAD to
  CSG and CSG to CAD for Monte Carlo transport codes (MCNP, OpenMC,
  PHITS, Serpent). Two pipelines:
  - `CadToCsg` (a.k.a. `GEOUNED` subpackage): STEP -> CSG decomposition.
    Heavily dependent on FreeCAD's `Part` API and, specifically,
    `BOPTools.SplitAPI.slice` for the solid decomposition process
    (`Options.splitTolerance` in the public API docs references this
    directly).
  - `CsgToCad` (a.k.a. `GEOReverse` subpackage): CSG -> CAD reconstruction.
    `export_cad()` writes both a `.step` file AND a native FreeCAD
    `.FCStd` file — this is a hard dependency on FreeCAD's own document
    format, not just its geometry kernel.
- Working branch: user's personal fork at github.com/psauvan/GEOUNED
  (private).

## Motivating problem (why this migration)

FreeCAD's boolean Cut can silently fail to split a solid into two when
the cutting plane's intersection with the solid coincides exactly with
a pre-existing tangency line (e.g. a plane cutting through a corner
where a cylinder is already tangent to two cube faces). Symptom: `Cut`
returns the original, uncut solid instead of raising an error. Root
cause: the resulting fragments only touch along a line (zero-area
contact), making the shape non-manifold in a way OCC's BOP algorithm
doesn't cleanly resolve. Round-tripping through STEP export/import
"fixes" it because the STEP healer effectively re-splits the compound.
Confirmed via `shape.fix()`, `removeSplitter()`, or STEP round-trip in
FreeCAD; at the pythonOCC level, the fix is to build a face-adjacency
graph excluding edges shared by >2 faces (non-manifold edges) and
reconstruct solids per connected component.

This is now solved in the `occ`/`ocp` engines (see "Current
architecture" below) via `Gsplit`'s own non-manifold-solid /
phantom-cut repair cascade — but it is a large, still-growing family of
related BOP degeneracies (coaxial-cone tangencies, tolerance-welded
fragments, open/split-ring shells, ...), each found and fixed one real
fixture at a time. `docs/reference/cad_defect_recipes` (via the
`reference_cad_defect_recipes` memory) and the history log are where the
individual recipes live.

## Decision: migrate to pythonocc-core (pyOCC), not CadQuery

Rationale:
- Fine-grained topological control needed for exactly the kind of
  non-manifold / degenerate boolean cases above (face/edge adjacency
  graphs, `BRepCheck_Analyzer`, `ShapeFix_Shape`, tolerance control on
  `BRepAlgoAPI_*` / `BOPAlgo_Splitter`).
- FreeCAD's `Part` API is itself a wrapper over OCCT — migrating to
  pyOCC removes a layer rather than adding a different one (CadQuery).
- CadQuery is better suited to high-level parametric construction, not
  the low-level topology surgery GEOUNED's decomposition needs. CadQuery
  can still be used opportunistically for simple construction code
  (`CsgToCad`'s primitive-building side) via `cq.Shape(occ_shape)` /
  `.val().wrapped` interop, if convenient — not as the primary dependency.
- Open question / risk: `.FCStd` export in `CsgToCad.export_cad()` has
  no pyOCC/CadQuery equivalent. Resolved: `.FCStd` output is now
  opt-in (`export_cad(format=[...])`), not written unconditionally, and
  stays FreeCAD-only; occ/ocp export real STEP with per-material color
  via XCAF instead.

## Migration history: attempt 1 (retired) — `GeometryBackend` ABC

The migration was first attempted as a classic Adapter / Ports & Adapters
pattern: a `GeometryBackend` ABC (`geometry_backend/geometry_backend_interface.py`)
with neutral, backend-agnostic dataclasses (`GSolid`/`GFace`/`GEdge`/`GVertex`/
`GVector`/`GPlane`/...) that carried a `native` object *and* a reference to
the backend instance that produced them, plus a concrete `FreeCADBackend`
implementation. All of GEOUNED called through a single injected
`_backend = FreeCADBackend()` instance (`_backend.split(gsolid, tool, tol)`,
`_backend.get_faces(gsolid)`, etc.) rather than touching `Part`/`FreeCAD`
directly.

This was fully implemented and rolled out across the entire `GEOUNED`
subpackage (write/, void/, boolean_solids.py, build_region/, load_step.py,
geouned_classes.py, decompose/, conversion/, ...) and validated against
the full test suite. It worked, but after living with it end-to-end the
user judged the `.backend`-carrying dataclasses + `_backend.method(x, ...)`
call convention too heavy and indirect for what it bought — see attempt 2.
**This design has been retired**: `geometry_backend/` and `tests/geometry_backend/`
were deleted once nothing else referenced them.

## Current architecture: attempt 2 — the `geo` package

`src/geouned/geo/` is now the ONLY place in GEOUNED allowed to import a
native CAD kernel (`Part`/`FreeCAD`/`BOPTools`, `OCC.Core.*`, or `OCP.*`)
— `GEOReverse` is explicitly out of scope for this rule (see the Project
section above), but mirrors the same structure at a smaller scale.
Engine-swappability (the original motivation for the attempt-1 ABC) is
achieved at the **module** level instead of dependency injection:
`geo/__init__.py` reads `GEOUNED_CAD_ENGINE` once at import time
(`ocp` by default; `occ` for pythonocc-core/SWIG; `freecad` for the
original `Part`/`BOPTools` implementation) and imports the matching
engine package — the rest of GEOUNED never branches on the engine.

Layout:

- `geo/freecad/`, `geo/occ/`, `geo/ocp/` — one package per engine, each
  with the same file-group split so a fix found in one engine is easy to
  locate and port to the others: `_native_utils.py` (native-conversion
  helpers, `kernel_version`), `topology.py` (analytic surface/curve
  descriptors — `GPlane`/`GCylinder`/`GCone`/`GSphere`/`GTorus`/`GLine`/
  `GCircle`/`GEllipse`/`GBSpline` — plus the neutral topology classes
  `GEdge`/`GWire`/`GFace`/`GShell`/`GSolid`, all mutually recursive so
  kept in one file), `repair.py` (`Gdefeature`/`Gcollapse_split_rings`/
  `Gsliver_heal`/`Gcheck_and_repair`/`Gspline_surface`/`Gheal_topology`
  — the load-time and post-split CAD-defect detection/repair cascade),
  `split.py` (`Gsplit`), `boolean.py` (`Gcut`/`Gcommon`/`Gfuse`),
  `primitives.py` (`Gmake_*` constructors), `io.py` (`Gload_step`/
  `Gload_step_labels`/`Gexport_step`/`Gload_and_process_step`), `queries.py`
  (`Gface_valid`, tolerance/manifold diagnostics). `occ`/`ocp` additionally
  split `split_repair.py` (non-manifold / phantom-cut repair of a raw
  BOPAlgo_Splitter result) and `split_coaxial_cone.py` (the coaxial-cone/
  cylinder degeneracy fallback) out of `split.py`'s own larger cascade —
  `freecad`'s `Gsplit` has no equivalent complexity.
- `geo/vector_geometry.py` — pure math, zero native dependency:
  `GVector`, `GMatrix`, `GBoundBox` and their `to_g*` converters.
- `geo/surface_geometry.py` — pure, duck-typed analytic-surface
  predicates shared by all 3 engines (`is_same_*_surface`,
  `is_coaxial_cone_*_pair`, `is_inside_*`, `find_can_plane`, ...).
- `geo/solid_defects.py` — whole-`GSolid` CAD-defect detection
  (`find_short_edges`, `find_split_ring_faces`, `check_solid_defects`,
  `valid_solid`), used by both the load-time repair cascade and
  `Gsplit`'s own post-split sanity filter.
- `geo/solid_ops.py` — `Gfuse_solids`, a shared policy helper (repair +
  fuse + compound fallback) one layer above the raw `Gfuse` kernel
  primitive, used by both GEOUNED's `build_region/` and GEOReverse's
  `buildSolidCell.py`/`splitFunction.py`.
- `geo/io_utils.py` — `suppress_native_stdout` (silences OCCT's own
  STEP-write console banner) and `GLabelNode` (STEP assembly/label tree).
- `geo/constants.py` — every shared tuning threshold (sliver/degenerate
  edge floors, volume-conservation tolerances for each repair step), so
  no engine carries its own duplicate copy of a literal.
- `geo/__init__.py` — the single import point for the rest of GEOUNED:
  `from ...geo import GSolid, Gmake_cylinder, ...`.

`GEOReverse/Modules/` mirrors this for its own, much smaller
engine-specific surface, reorganized into subpackages by the user
2026-09-12 (after the `BoolSequence` unification below): `engine_dependency/`
(`_freecad_impl.py` / `_occ_impl.py` / `_ocp_impl.py` -- CAD export via
XCAF with per-material color, plus the 7 "exotic quadric" surfaces
GEOUNED's forward pipeline never produces itself; see "Known open items"
-> GEOReverse for which ones are implemented as of 2026-09-13),
dispatched by `Modules/__init__.py` the same way `geo/__init__.py`
does; `CAD/` (`buildCAD.py`/`buildSolidCell.py`/`splitFunction.py` --
the CSG -> CAD solid-reconstruction core); `MCNP_parser/`
(`remh.py`/`MCNPinput.py`, plus the vendored `Parser/` sub-package);
`XML_parser/` (`XMLinput.py`/`XMLParser.py`, for OpenMC XML input);
`Utils/` (`booleanFunction.py`/`boundBox.py`/`cad_export_shared.py`/
`matrix_utils.py`). `Utils/cad_export_shared.py` holds the pure-Python
pieces of the CAD export confirmed duplicated across backends
(2026-09-12): the Universe/Material/Cell label-name f-strings (identical
across all 3 backends) and the per-material color-assignment logic
(identical between `_occ_impl.py`/`_ocp_impl.py` only -- `freecad` has
no color support, see "Environment notes"). `_freecad_impl.py`'s own
`ortoVect`/local `to_gvector_` -- pure `GVector` math with zero native
dependency, confirmed unused outside that one file -- moved to
`geo.vector_geometry.arbitrary_perpendicular`/reused `geo.to_gvector`
directly at the same time; NOT merged with GEOUNED's own, independently-
validated `meta_surfaces_utils.py::_perpendicular_axis` (same kind of
"arbitrary stable perpendicular" idea, different formula, different
unrelated call sites -- kept distinct, see `arbitrary_perpendicular`'s
own docstring). The exotic-quadric dataclasses (`GEllipsoid`,
`GEllipticCylinder`, ...) stay local to each engine's own
`_occ_impl.py`/`_ocp_impl.py` (`_freecad_impl.py` for that engine),
never in `geo` -- deliberate, GEOReverse-only constructions. The one
exception is `Gmake_torus_elliptic` (**moved to `geo/occ/primitives.py`/
`geo/ocp/primitives.py`, 2026-09-13**, right after `Gmake_torus`, not a
separate file -- see "Known open items" -> GEOReverse): unlike the other
exotic quadrics, GEOUNED's own forward pipeline has a real, live need for
degenerate-torus single-sheet construction too --
`GeounedSurface.build_surface()`'s Torus branch now calls it (occ/ocp
only, see same section for the fix and its verification), so it's a
genuinely shared primitive, not a GEOReverse-only one.
`GEOReverse`'s own `build_region`-equivalent
(`CAD/buildSolidCell.py`/`CAD/splitFunction.py`/`Objects.py`) is a
still-separate, not-yet-unified twin of GEOUNED's `build_region/` — see
the portability analysis in the history log for what's already
shareable (box algebra, `BuildDepth`/`SplitSolid` core) versus
genuinely blocked (two divergent surface/cell models -- the
`BoolSequence` split itself was closed out, see below).

`src/geouned/boolean_utils/` (`boolean_function.py`, `boolean_expression_parser.py`)
holds the `BoolSequence` class and its MCNP-syntax parser -- moved here
2026-09-12 from `GEOUNED/utils/boolean_function.py` and the top-level
`boolean_expression_parser.py`, as a genuinely shared module now that
`GEOReverse` depends on it directly (see the `BoolSequence` unification
entry below): zero dependency on anything else in the package, not even
`geo`, importable from both pipelines without either pulling in the
other.

Design points that survive from attempt 1, now living as methods/free
functions instead of ABC methods:
- Faces are eagerly classified into one of 5 analytic surface types
  (plane/cylinder/cone/sphere/torus) via `Gclassify_surface`; composite/
  meta-surfaces (RoundCorner, Can, TCone, MultiRoundCorner, MultiPlane,
  ReversedConeCylinder) are assembled by GEOUNED itself out of these,
  never modeled in `geo` — see `docs/reference/composite_surfaces.md`.
- `Gsplit()` returns a `SplitResult` that must never silently come back
  missing real material. Under `occ`/`ocp` this now includes a real
  repair cascade for the project's original motivating tangency bug
  (non-manifold-solid reconstruction, phantom-cut separation, coaxial-cone
  retry, open-shell/split-ring healing, a gated STEP round-trip heal for
  a BOPAlgo tolerance-weld) — see the history log for how each piece was
  found and verified. `freecad`'s `Gsplit` only has the original
  tolerance-scaling retry; the deeper repairs are occ/ocp-only.

## Current status (as of 2026-09-14)

- All 3 engines pass `tests/geo` + `tests/test_cadtocsg.py` +
  `tests/test_csgtocad.py` **in full** as of 2026-09-14, occ/ocp also
  `tests/test_georeverse_occ_impl.py`/`_ocp_impl.py` (163 passed
  freecad, 178 passed occ, 178 passed ocp) -- confirms all 7 exotic
  quadric surfaces (see "Known open items" -> GEOReverse) are a
  zero-regression addition. `test_cylbox_convertion` (both `[mcnp]`
  and `[openmc_xml]`) used to fail under all 3 engines, root-caused to
  two real bugs (a `Gsplit` tolerance-argument API mismatch, and
  `_find_cone_face` crashing on a bare-`GFace` cutting tool under
  occ/ocp) and fixed -- see "Known open items" -> GEOReverse for the
  detail, and the history log for the full investigation. Separately,
  `Mixed/ConeSphere.stp` still crashes natively under `occ`/`ocp`
  (`ShapeUpgrade_UnifySameDomain` access violation) and is an accepted
  permanent limitation there, translating cleanly only under `freecad`.
- `Solidos/test_models` corpus (the d1suned MCNP stochastic volume check,
  ~140 STEP files excluding `Big_*`): the last full run recorded before
  the most recent 5 commits landed was 92.6% of solid-cell tallies within
  2 sigma, 2 files beyond 3 sigma (the `SCDR_90`/`SCDR_90_hollow` family,
  a long-documented, mostly-closed-out case — see history log), 0 lost
  particles. Since then, `RevCC`'s closed-cylinder/cone-shell rejection,
  the `round_corner_region` split, `Gmerge_coplanar_planes`,
  `Gclose_open_solid` wiring into `_raw_bop_split`, the `eligible_plane`
  sliver-arc fix, and `large_cell_plane_split`'s volume-conservation
  guard have each independently fixed further corpus files — a fresh
  full-corpus re-run with all of them together has not been done yet.
- `GEOReverse` (CsgToCad) debugging work in general is deliberately
  paused: explicit user priority is to finish cleaning up known
  `GEOUNED`/`CadToCsg` bugs first. Do not start GEOReverse-side
  debugging unless asked. The one explicit exception, worked and closed
  2026-09-13/14: the 7 exotic-quadric surfaces (occ/ocp construction +
  `is_inside` correctness) -- see "Known open items" -> GEOReverse for
  what's implemented and the specific validation gaps still open (none
  has been exercised end to end against a real MCNP/OpenMC file yet).

## Known open items

(Supersedes every dated "Pending tasks" checkpoint inside the history
log — those are kept there for their own historical record, but this
list is the current one. Verified against the live code/tests on
2026-09-12 — see the history log's "Known-open-items audit" entry for
how. `Big_complex_cell/modelcell_cut1.stp`/`modelCell_670000.stp`, the
one item that audit found already fixed, has been dropped from this
list — see that entry for the verification numbers.)

### GEOUNED (`CadToCsg`, the forward STEP -> CSG pipeline)

- ~~`AdjacentMultiplanePlanes` needs the same RevCC-to-MultiRoundCorner
  extension~~ -- **done, 2026-09-17** (closes the
  `project_mrc_adjacent_multiplane_pending` memory). Per direct user
  clarification: a Reversed, *open* MultiRoundCorner (see
  `MultiRoundCornerParams.ClosedSet` below) is the same kind of local
  non-convexity as a MultiPlane -- it too can leave one of its own
  shared junction planes exposed right next to a RevCC's cylinder/cone,
  so a RevCC segment bordering one needs the exact same OR-escape
  treatment a bordering MultiPlane already gets. Implemented as a pure
  reuse of the already-verified machinery, not a new code path:
  `cell_definition.py::simple_solid_definition` now builds
  `open_multi_round_corners` (every `MultiRoundCorner` with
  `Orientation == "Reversed"` and `ClosedSet == False`) and passes
  `multiplanes + open_multi_round_corners` into
  `get_reversed_cone_cylinder` -- `_find_adjacent_multiplane_planes`
  only ever reads a candidate's own `.Surf.Planes` (a list of real Plane
  GeounedSurfaces, the same shape for both `MultiPlane` and
  `MultiRoundCorner`), so it needed no change at all beyond its own
  docstring. A Forward or closed (`ClosedSet == True`) MultiRoundCorner
  is excluded -- neither creates the local non-convexity this mechanism
  compensates for, nor has an exposed plane for anything outside the
  group to actually border.
  `MultiRoundCornerParams.ClosedSet` (new field, `basic_functions_part1.py`):
  True when every one of the group's own shared junction planes connects
  exactly 2 neighboring corners (a closed ring); False when at least one
  is touched by only 1 corner (an open chain with a loose end) -- same
  per-plane-degree walk `build_roundC_params` already used to gate
  `convex_planes`'s own `closed` argument, just also stored on the
  result now. Threaded through both places a `MultiRoundCorner`
  `GeounedSurface` is built (`functions.py::get_roundCorner`, and the
  separate decomposition-phase generator,
  `decompose/generators.py::next_roundCorner`).
  **A real, independent bug found and fixed while wiring `ClosedSet`
  in**: `build_roundC_params`'s own gate for even attempting the
  `multi_round`/`closed_set`/`orientation` determination was
  `len(plane_list) > 2` (more than 2 RAW, not-yet-deduplicated shared
  planes) -- but a corner whose own two boundary planes coincide
  (`p1 == p2`, the pre-existing "aligned planes" case) contributes only
  1 raw entry instead of 2, so a genuine chain of 2+ corners could
  raw-collapse to as few as 1-2 total `plane_list` entries and get
  wrongly skipped entirely (silently falling back to independent
  RoundCorners instead of one MultiRoundCorner). Fixed by gating on
  `len(rc_list) > 1` instead (is there more than one real corner in the
  candidate group at all -- the actually-intended condition) --
  `closed_set` itself kept its original, general per-plane-degree walk
  (`rc_planes`/`is_same_plane`, checking whether any plane is touched by
  only 1 corner) unconditionally for any size, rather than adding
  special-cased formulas for small plane counts (a count-based shortcut
  like "closed iff corner count == plane count" was tried and explicitly
  rejected on user review -- true in the "closed ring" direction, but
  not the converse: e.g. several independent, non-contiguous corners
  bounding the same physical wall can coincidentally match such a count
  without being any kind of chain at all).
  **Verified**: full freecad `tests/geo` + `test_cadtocsg.py` (161
  passed) green after both changes. A 141-file differential scan across
  `Solidos/test_models` (`git stash`-based before/after, decompose +
  build_solid_definition only) found 0 new failures and exactly 1 file
  with a real composite-surface-count change --
  `Decomposed/SCDR_solid19_solid26.stp` (`RoundC:2,MultiRoundC:4` ->
  `RoundC:1,MultiRoundC:5`, the `len(rc_list) > 1` gate fix actually
  firing) -- confirmed correct via a real d1suned MCNP stochastic volume
  check on that exact file: tally `1.01145 +/- 0.55%` (2.1 sigma), 0 lost
  particles. The AdjacentMultiplanePlanes-for-MultiRoundCorner escape
  mechanism itself never fired in this corpus (0 hits, instrumented and
  confirmed via a temporary monkeypatch during this same scan) -- no
  file in `Solidos/test_models` currently has a RevCC segment bordering
  an open/Reversed MultiRoundCorner, but per the user's own review, this
  needs no dedicated fixture to trust: it's a direct reuse of
  `_find_adjacent_multiplane_planes`'s own already-validated topological
  walk, not a new, unexercised code path of its own.

### GEOReverse (`CsgToCad`, the reverse CSG -> STEP pipeline)

Deliberately paused as a whole — explicit user priority is to finish
`GEOUNED` first (see "Current status"). These are the specific known
gaps for whenever it's picked back up:

- ~~`test_cylbox_convertion` fails~~ -- **fixed on all 3 engines,
  2026-09-12**. Two independent, real bugs, neither a reconstruction-
  algorithm problem: (1) `geo.Gsplit`'s own `tolerances` argument was
  refactored (2026-08-30) from a plain `tolerance=<float>` keyword to a
  positional `Tolerances` object, but `CAD/splitFunction.py::SplitSolid`
  and `CAD/buildCAD.py::interferencia` were never updated to match --
  every single surface cut in the whole reverse pipeline raised
  `TypeError`, silently swallowed by `SplitSolid`'s own broad
  `except Exception:`, always falling back to the uncut input solid
  (`interferencia`'s own call had no such guard -- would have crashed
  outright the instant any model used a FILL/nested universe). (2)
  `geo/{occ,ocp}/split_coaxial_cone.py::_find_cone_face` assumed its
  `shape` argument always has a `.Faces` list (true for a `GSolid`, the
  only way GEOUNED's own forward pipeline ever calls `Gsplit`) but
  GEOReverse's own cutting tools are bare `GFace` objects (single
  surfaces, never wrapped in a solid) -- crashed with `AttributeError`
  the moment `Gsplit`'s coaxial-cone check ran on one, caught by the
  same broad `except Exception:` in `SplitSolid` and misread as "no
  degeneracy, cut normally" while actually silently no-op'ing every cut.
  Fixed: (1) build a real `Tolerances(split_tolerance=...)` instance at
  both call sites; (2) `_find_cone_face` now checks `isinstance(shape,
  GFace)` directly before assuming `.Faces` exists, in both engines.
  `openmc_xml`'s own expected-volumes baseline (`test_csgtocad.py`) was
  also corrected: the old 5-solid baseline was a pre-migration-FreeCAD
  artifact, cross-validated wrong by `mcnp`'s logically-identical cell 2
  region independently reconstructing to the same single solid under
  both formats now. See the history log for the full investigation.
- ~~`hylife-v06.stp` solid 45's slow decomposition~~ / ~~its round-trip
  volume discrepancy~~ -- **dropped, 2026-09-17, not a GEOUNED or
  GEOReverse bug**: per direct user diagnosis, solid 45 in this file is
  itself an invalid/degenerate CAD solid at the source-STEP level (not a
  decomposition-algorithm problem) -- GEOUNED's O(n^2) same-surface
  face-adjacency cost choking on it, and GEOReverse's own reconstructed
  volume disagreeing with GEOUNED's own d1suned tally on this same
  solid (~1.401x vs ~1.119x), are both downstream symptoms of the same
  bad input, not independent bugs worth chasing on either pipeline's
  own code.
- **GQ/SQ surface-type classifier (`MCNP_parser/MCNPinput.py::gq2params`
  -> `getGQAxis` -> `get_cylinder_parameters`/`get_cone_parameters`/
  `get_hyperboloid_parameters`/`get_ellipsoid_parameters`), fixed
  2026-09-14**: this is the function that decides, from a raw GQ/SQ
  card's 10 coefficients (via eigenvalue decomposition of the quadratic
  form), which specific surface type it is -- cylinder, cone, ellipsoid,
  hyperboloid, etc. Investigated after the user flagged that a *real*
  circular cylinder (a card whose non-axis-aligned rotation makes all 10
  coefficients nonzero) can misclassify as an ellipsoid or hyperbolic
  cylinder once its coefficients have been rounded. Root cause: every
  "are these two eigenvalues equal"/"is this eigenvalue zero" test used
  a fixed *absolute* tolerance (`1e-5` in three places, `1e-3` in one,
  `1e-8` in one) or outright *exact* `== 0` equality -- but the GQ
  equation is invariant under multiplying all 10 coefficients by any
  nonzero scalar, which scales every eigenvalue (and the reduced
  constant `k`) by that same scalar, so no fixed absolute tolerance can
  work across differently-normalized cards representing the same
  surface. Confirmed live against the only real GQ fixture in the repo
  (`tests/csg_files/cylinder_box.mcnp`, a clean circular cylinder): its
  eigenvalues are `[-5.55e-17, 1.0, 1.0]`, and the old `e0 == 0` test
  missed that `-17`-order residual entirely, misrouting classification
  to "hyperboloid" -- it only produced the right final answer by
  accident, via an unrelated large-radius-ratio fallback deep in
  `get_hyperboloid_parameters`. Fixed: every such comparison now uses a
  *relative* tolerance instead, scaled against the eigenvalues' own
  magnitude (`max(abs(eigenvalues))`) -- two new tiers added to a new
  `Tolerances` class in `GEOReverse/Modules/data_class.py`
  (`gq_eigen_zero_rel = 1e-6` for numerical-noise-level zero checks,
  `gq_eigen_equal_rel = 1e-5` for the looser "forgive real MCNP-card
  rounding" pairwise-equal check -- there's a wide, safe margin between
  6-8-significant-figure rounding noise and any genuinely, intentionally
  elliptic real-world design, so this doesn't risk false positives). The
  previously-hardcoded `cylTan`/`coneRad` ratio-based fallbacks were also
  centralized into this same `Tolerances` class (`cylinder_ratio =
  1e3`/`cone_min_radius = 0.1`, values unchanged).
  **Two further, independent bugs found and fixed while verifying this**
  (not tolerance issues, but only surfaced by testing a non-axis-aligned
  cylinder with non-unity eigenvalues, which no existing fixture in the
  repo happened to exercise): (1) `gq2params`'s `Dinv = eVal[:]` was a
  numpy *view*, not a copy -- `Dinv[nonzero] = 1/eVal[nonzero]` silently
  overwrote `eVal` itself in place, corrupting the eigenvalues used by
  every downstream classification step; masked in the one real fixture
  only because that cylinder's own nonzero eigenvalues already happened
  to equal `1.0` (`1/1.0` is a no-op overwrite). Fixed to `Dinv =
  eVal.copy()`. (2) The paraboloid-vs-cylinder `comp` check compared a
  linear-coefficient-scale quantity against the eigenvalue scale
  (dimensionally mismatched); fixed to compare against the diagonalized
  linear-coefficient vector's own magnitude instead.
  Verified: new `tests/test_gq_classification.py` (5 tests -- the real
  fixture, a rotated circular cylinder surviving 6- and 8-significant-
  figure rounding, a genuinely 20%-eccentric elliptic cylinder correctly
  *not* rounded down to circular, and an axis-aligned control) plus
  `test_csgtocad.py::test_cylbox_convertion` (both `mcnp`/`openmc_xml`)
  green on all 3 engines.
  **Unified with `XML_parser/XMLinput.py`'s own classifier, 2026-09-14**:
  that file's separate `gq2cyl` (used for OpenMC-XML input, cylinder/cone
  only) already used relative tolerances, just its own looser local
  literals (`minWTol=5e-2`, `minRTol=1e-3`) playing the exact same "is
  this eigenvalue ~0"/"are these two eigenvalues ~equal" roles as
  `gq_eigen_zero_rel`/`gq_eigen_equal_rel` above -- swapped in directly
  (a 3-4 order-of-magnitude tightening). Re-verified
  `test_cylbox_convertion[openmc_xml]` (the only real exercise of this
  path in the repo) still green on all 3 engines after the tightening,
  so both parsers now classify the same GQ coefficients the same way.
- The 7 "exotic quadric" surfaces GEOReverse's own MCNP/OpenMC-XML
  `GQ`/`SQ` parser can produce -- **all 7 now implemented under `occ`/
  `ocp`, 2026-09-13/14**: `Gmake_ellipsoid`, `Gmake_elliptic_cylinder`,
  `Gmake_torus_elliptic`, `Gmake_elliptic_cone`, `Gmake_hyperboloid`,
  `Gmake_hyperbolic_cylinder`, `Gmake_paraboloid`
  (`GEOReverse/Modules/engine_dependency/_occ_impl.py`/`_ocp_impl.py`;
  the now-unused `_not_implemented()` stub helper was deleted from both
  files). `is_inside()` was also independently verified for all 7 (see
  its own entry below) -- three had real bugs, now fixed.
  **`Gmake_ellipsoid`, 2026-09-13**: implemented in both `occ` and `ocp`,
  per direct user instruction on the construction technique -- draw the
  ellipse curve in a plane, revolve it around an axis in that plane to
  sweep the surface, close/cap if needed, sew to a shell, then a solid.
  For the ellipsoid specifically, only the HALF of the ellipse profile on
  one side of the axis of revolution is built (its own two endpoints then
  land exactly on the axis, i.e. the spheroid's two poles), so a 360-
  degree revolve already produces a closed, watertight shell with no
  separate capping step -- a more robust technique than
  `_freecad_impl.py`'s own (full curve revolved 180 degrees, or a
  parameter-space half revolved 360), which is documented (that file's
  own module docstring) to already fail in `Part.makeSolid` on the
  current FreeCAD version. Verified against the analytic spheroid volume
  (4/3 * pi * perp_radius^2 * rev_radius) for a prolate case, an oblate
  case, a degenerate sphere case, and an arbitrary non-axis-aligned
  orientation, all exact to float precision on both engines -- see
  `tests/test_georeverse_occ_impl.py`/`test_georeverse_ocp_impl.py` and
  the history log for the full derivation and verification script. A
  separate, unrelated bug was found (not fixed) while tracing the
  parameter path: `MCNP_parser/MCNPinput.py::get_ellipsoid_parameters`
  appears to return the axis of revolution as a bare integer index
  (`iaxis`) rather than the corresponding `GVector` direction -- flagged,
  not chased down (no real GQ-ellipsoid MCNP/XML fixture has been found
  to exercise this path end to end yet; the implementation above was
  verified via direct/synthetic parameters, matching how
  `_freecad_impl.py`'s own known ellipsoid gaps were originally verified).
  **`Gmake_elliptic_cylinder`, 2026-09-13**: implemented in both `occ`
  and `ocp` -- an ellipse drawn in the plane normal to the cylinder axis
  and centered on it, extruded `height` along that axis starting at
  `center` (matching `_freecad_impl.py`'s own start-point convention),
  then capped with two planar faces. Verified against the analytic
  volume (`pi * major_radius * minor_radius * height`) and that the
  built solid's bounding box spans exactly `[center, center +
  height*axis]` along the axis.
  **`Gmake_torus_elliptic`, 2026-09-13**: implemented in both `occ` and
  `ocp`, covering both the non-degenerate case (ellipse or circle
  revolved 360 degrees around the torus axis, closed on its own, no
  capping needed) and the degenerate/self-intersecting case (the profile
  crosses the axis at 2 points, splitting it into a long and a short arc;
  only one is kept and revolved -- the long arc gives the "outer" sheet,
  the short arc the "inner" one). Signature:
  `Gmake_torus_elliptic(center, axis, major_radius, minor_radius_a,
  minor_radius_b, outer=None)` -- `major_radius` is the tube center's
  offset from `center` (MCNP's own `R`), `minor_radius_a` is the tube
  cross-section radius along the *same* (radial) direction as
  `major_radius`, `minor_radius_b` along the torus axis direction
  (`minor_radius_a > minor_radius_b` = flattened/oblate torus,
  `minor_radius_a < minor_radius_b` = elongated along the axis,
  `minor_radius_a == minor_radius_b` = plain circular-section torus).
  Parameter names went through one real correction after an initial,
  wrongly-labeled `(r_major_axis_offset, major_radius, minor_radius)`
  version shipped -- caught on user review, fixed with a full call-site
  and test-argument-order audit (positions 2/3 needed swapping, not just
  renaming, since the old "major_radius" paired with the axis direction
  and the old "minor_radius" with the radial direction -- the reverse of
  the corrected `minor_radius_a`/`minor_radius_b` mapping). `outer`
  defaults to `None` = derive from the sign of `major_radius` itself
  (`>= 0` outer, `< 0` inner), matching the round-trip convention already
  established via `GTorus.a_sign`/`torus_sheet_sign` on the forward side
  (`GEOUNED/write/functions.py`'s `radMaj *= surf.a_sign`) -- **confirmed
  this is GEOUNED's own internal encoding, not a real MCNP/OpenMC/
  Serpent/PHITS format feature**: none of those formats has a field to
  disambiguate a degenerate torus's two sheets, so GEOUNED repurposes the
  sign of the written major radius for its own write/read round-trip
  (the same trick already used for a cone's signed `SemiAngle`) rather
  than inventing a new output field. Verified against the closed-form
  torus volume for the non-degenerate case and an independent Pappus
  numerical integration over the kept arc for the degenerate case
  (circular and elliptical, both sheets), plus `BRepCheck_Analyzer`
  validity, on both engines.
  **Moved into `geo/occ/primitives.py`/`geo/ocp/primitives.py`,
  2026-09-13** (right next to `Gmake_torus`, not a separate file --
  an earlier attempt at a dedicated `torus_elliptic.py` sibling file was
  corrected on review: `primitives.py` already holds every other
  `Gmake_*` constructor flat in one file, so a special-cased split wasn't
  justified), re-exported from `GEOReverse/Modules/engine_dependency/
  _occ_impl.py`/`_ocp_impl.py` rather than duplicated there, and wired
  into `geo/__init__.py`'s occ/ocp dispatch (NOT freecad -- freecad's own
  `_freecad_impl.py::Gmake_torus_elliptic` has no outer/inner support and
  stays local, out of scope for this move). `GEOReverse/Modules/
  Objects.py::Torus.buildShape`'s call site, and its own `__init__`
  parameter-validation warnings (which used to mislabel `params[3]`/
  `params[4]` and to always warn on a negative major radius even in the
  legitimate degenerate/inner case), were updated to match.
  **Now consumed by GEOUNED's own forward pipeline too, 2026-09-13**:
  `GeounedSurface.build_surface()`'s Torus branch
  (`GEOUNED/utils/geouned_classes.py`) used to build its degenerate-torus
  cutting tool via plain `Gmake_torus` (the full self-intersecting
  double-sheet primitive) regardless of `Degenerated`/`a_sign` -- a real
  candidate for unnecessary over-cutting wherever a degenerate torus is a
  cutting tool (the two live consumers are
  `decompose/decom_one_generators.py::generic_split()` and
  `utils/boolean_solids.py::build_c_table_from_solids()`/
  `split_solid_fast()`). Fixed: when `tor.Surf.Degenerated` and the
  active engine isn't `freecad` (checked via the newly-imported
  `CAD_ENGINE` from `geo`), it now calls `Gmake_torus_elliptic(center,
  axis, majorR, minorR, minorR, outer=tor.Surf.a_sign > 0)` (a circular
  cross-section, `minor_radius_a == minor_radius_b == minorR`) to build
  only the correct single sheet instead. Imported via a function-local
  `from ...geo import Gmake_torus_elliptic` inside the branch (never a
  top-level import) specifically because that name doesn't exist under
  `geo/freecad/__init__.py` -- freecad keeps today's `Gmake_torus`-only
  behavior unconditionally, both because the `CAD_ENGINE != "freecad"`
  guard skips the new path entirely and because a top-level import would
  have broken freecad-engine GEOUNED at module load time regardless.
  Verified against `Solidos/test_models/Torus/2_degen_torii.stp` (2
  degenerate tori, both inner/`a_sign=-1`) and the non-degenerate
  `Torus/Torus_solid1.stp` control, both converted under all 3 engines:
  identical cell count, volume, and written `TZ` surface cards
  before/after on every engine (occ/ocp now matching `freecad`'s own
  unaffected output byte-for-byte on `2_degen_torii.stp` -- this
  particular fixture's downstream boolean simplification already
  absorbed the extra complexity from the old double-sheet cut, so the
  fix is a correctness improvement with no visible effect on this one
  fixture's output, not a regression risk).
  **`Gmake_elliptic_cone`, 2026-09-13**: implemented in both `occ` and
  `ocp` -- from the apex and axis, an ellipse is drawn in the plane
  normal to the axis at distance `length` from the apex (semi-axes
  `MajorRadius/RefRadius*length`/`MinorRadius/RefRadius*length`, the
  MCNP GQ/SQ scale-with-distance convention -- `RefRadius` is the axial
  distance at which the cross-section ellipse's semi-axes equal
  `MajorRadius`/`MinorRadius` exactly), then a ruled loft
  (`BRepOffsetAPI_ThruSections`, `isSolid=True`) from the apex vertex to
  that ellipse's wire closes directly into a solid -- the apex needs no
  separate capping (it's a single point, not on the revolution axis in
  the ellipsoid/torus/hyperboloid sense, but the loft's own vertex
  degenerate section closes it the same way). `DoubleSheet` fuses the
  forward and axis-reversed single sheets, matching
  `geo.Gmake_cone_double_sheet`'s own circular-cone case. Signature
  matches `_freecad_impl.py::Gmake_elliptic_cone` and the real call
  site (`Objects.py::EllipticCone.buildShape`) exactly: `(apex, axis,
  ref_radius, major_radius, minor_radius, major_axis, minor_axis,
  double_sheet, length)`. Verified against the analytic cone volume
  (`pi/3 * a * b * length` for the base ellipse's own semi-axes `a`/`b`
  at `length`), the double-sheet volume being exactly 2x the single
  sheet's, and the apex/base bounding-box position, on both engines.
  **`Gmake_hyperboloid`/`Gmake_hyperbolic_cylinder`, 2026-09-14**: the
  same hyperbola (`Center`/`MajorRadius`/`MinorRadius`/`MajorAxis`/
  `MinorAxis`) revolved around two different axes gives two different
  real surfaces -- `Gmake_hyperboloid` revolves around `MajorAxis`
  (branch 1's own vertex, on the revolution axis, out to a capped rim at
  `length`; the standard two-sheet hyperboloid, since revolving a
  transverse-axis hyperbola around its own transverse axis always
  produces two disjoint cups); `Gmake_hyperbolic_cylinder` revolves the
  *same* hyperbola around `MinorAxis` instead (the conjugate axis), which
  always gives a single, fully-connected "hourglass" (waist radius
  `MajorRadius` sitting exactly at `Center`, both ends capped since
  neither is on the revolution axis) -- this **supersedes**
  `_freecad_impl.py::GHyperbolicCylinder`'s own extrude-based technique
  (translating two mirrored hyperbola branches along a separate `Axis`
  field) with a genuinely different surface. `GHyperboloid.OneSheet`
  (default `True`, matching the pre-existing dataclass default) means
  "build only branch 1 (positive `MajorAxis` side)"; `False` also mirrors
  branch 1 through `Center` for branch 2 and assembles both as a
  compound (`Gmake_compound`, not a fuse -- the two sheets never touch).
  Both verified against closed-form volumes (`pi*a^2*(L + L^3/(3*b^2))`
  for the cylinder's hourglass segment; the equivalent integral for one
  hyperboloid branch) and `BRepCheck_Analyzer` validity on both engines.
  **`Gmake_paraboloid`, 2026-09-14**: same one-branch-revolve technique
  as `Gmake_hyperboloid` (vertex on the revolution axis needs no capping,
  the far rim does) but always a single sheet -- a parabola has no
  second branch to mirror, so no `OneSheet` flag at all. OCCT's own
  `Geom_Parabola` parametrization conveniently makes the far end's own
  curve parameter *equal* to the rim's radius (`u_end =
  sqrt(4*Focal*length)`), needing no separate rim-radius formula unlike
  the hyperbola case. Returns `None` when `length <= 0` (matches
  `_freecad_impl.py::GParaboloid.build_shape`'s own documented contract
  and the real caller's `if dmax <= 0: return` guard). Verified against
  the analytic volume (`pi*2*Focal*length^2`), vertex/rim position, and
  the `None` contract, on both engines.
  **`is_inside()` independently verified for all 7, 2026-09-14**: 40
  hand-computed ground-truth points (never reusing a class's own
  formula) found `GEllipticCylinder`/`GEllipticCone`/`GParaboloid`
  already correct, and 3 real bugs, now fixed: `GEllipsoid.is_inside`
  had TWO bugs, not just the one `_freecad_impl.py`'s own docstring
  flagged (a `Center`-double-subtraction, present in both branches, and
  a swapped axial/radial radius pairing in *both* branches -- the
  "if" branch, previously believed correct, turned out just as wrong as
  the flagged "else" one); `GHyperboloid.is_inside` had the same
  `Center`-double-subtraction bug (only manifests once `Center` isn't
  the origin) on top of testing a different `OneSheet` meaning than the
  redesigned `build_shape` now uses; `GHyperbolicCylinder.is_inside`
  still tested the superseded extruded-prism definition (only the
  `MajorAxis`/`MinorAxis`-plane projection, ignoring the third axis
  entirely) instead of the new revolve-based one. Fixed, per direct user
  design: `GHyperboloid`'s (two-sheet) region is now computed as the
  *complement* of the same-parameters `GHyperbolicCylinder`'s (one-sheet)
  region -- both come from the same hyperbola, revolved around opposite
  axes -- plus one extra check for `OneSheet=True` (the sign of the axial
  coordinate along `MajorAxis`) to pick the correct branch.
  Full `tests/geo` + `test_cadtocsg.py` + `test_csgtocad.py` (+
  `test_georeverse_*_impl.py` on occ/ocp, now 43 tests each) still green
  on all 3 engines after all of the above (freecad 163 passed, occ 178
  passed, ocp 178 passed, 2026-09-14).
  **End-to-end fixtures + `convert_to_planes` support, 2026-09-14**:
  real MCNP fixtures were built and round-tripped (`MCNP_parser` ->
  `Objects.py` -> `Utils/boundBox.py::convert_to_planes` ->
  `CAD/buildSolidCell.py`/`splitFunction.py` -> STEP export) for 3 of
  the 7 surfaces -- ellipsoid (prolate, capped by nothing since the
  surface is already closed), elliptic cylinder (a=50cm/b=30cm,
  Z-aligned, capped with 2 planes), and elliptic cone (apex at origin,
  RefRadius=100/MajorRadius=50/MinorRadius=30, capped with 2 planes) --
  each verified against its own closed-form analytic volume, exact to
  float precision on both `occ` and `ocp`. This is the first time any
  of the 7 exotic quadrics has been exercised through the real pipeline
  rather than via direct/synthetic `Gmake_*` calls. `convert_to_planes`
  (the bounding-plane approximation `buildSolidCell.py` needs before
  calling the real shape builder, previously only implemented for
  plane/cylinder/cone/sphere/torus) gained 3 new branches:
  `elliptic_cylinder_to_planes` (same 4-tangent-plane idea as
  `cylinder_to_planes`, but the single radius `R` replaced by the
  ellipse's own `major_radius`/`minor_radius` taken directly from the
  surface's stored axes, not `get_orto_axis`), `ellipsoid_to_planes`
  (a `sphere_to_planes`/`cylinder_to_planes` mix -- 4 equatorial planes
  at the same distance from `center`, using whichever radius is
  perpendicular to the revolution axis, plus 2 polar caps at the other
  radius), and `elliptic_cone_to_planes` (same idea as `cone_to_planes`,
  but the major/minor cross-section directions get their own distinct
  half-angle `atan(radius/RefRadius)` instead of one shared value, and
  use only 4 fixed directions -- `+/-major_axis`, `+/-minor_axis` --
  instead of `nface` evenly-spaced ones, since the two directions aren't
  interchangeable). `hyperboloid`/`cylinder_hyperbolic` still have no
  `convert_to_planes` branch -- blocked on the classifier-dispatch
  question below.
  **3 more real, independent bugs found and fixed while building these
  fixtures** (none are tolerance issues -- all pre-existing, unrelated
  to each other):
  1. `MCNP_parser/MCNPinput.py::get_ellipsoid_parameters`'s
     already-flagged axis-as-bare-int bug is now **fixed**: the final
     return used to hand back the raw eigenvector index (`iaxis`, an
     `int`) in the axis-of-revolution slot instead of the matching
     `GVector` -- crashed the instant a real GQ-ellipsoid reached
     `GEllipsoid.build_shape`'s own `(self.Axis - self.MinorAxis)`
     subtraction (no such operator on an `int`). Fixed to
     `eVect.T[iaxis]` via `_gvec(...)`, matching every other
     parameter-getter's own convention.
  2. `MCNP_parser/MCNPinput.py::get_cone_parameters`'s elliptic-cone
     branch computed `Ra`/`Rmin`/`Rmaj` as `abs(1 / eVal[...])` --
     missing a `sqrt`. Eigenvalues carry units of 1/length^2 (quadratic-
     form coefficients), so `1/eVal` has units of length^2, not length;
     the circular-cone branch just above it already gets this right
     (`tan = sqrt(-eVal[...] / eVal[iaxis])`). Silently gave the wrong
     (squared) cross-section scale to every real elliptic cone -- caught
     via the new `elliptic_cone.mcnp` fixture's volume coming back ~6.67x
     too small; fixed to `sqrt(abs(1 / eVal[...]))` for all three, which
     is also the scale-invariant form (the `Rmaj/Ra`, `Rmin/Ra` ratios
     `Gmake_elliptic_cone` actually uses stay constant under any overall
     GQ-coefficient rescaling, matching this session's own established
     scale-invariance principle for this whole classifier).
  3. `CAD/splitFunction.py::surface_side` -- a SEPARATE, parallel
     point-classification implementation from the dataclass-level
     `is_inside()` methods fixed earlier this session in
     `_occ_impl.py`/`_ocp_impl.py` (this one is the one actually invoked
     during real boolean solid-splitting, confirmed via a live traceback
     while converting the `ellipsoid.mcnp` fixture) -- had the same bug
     family, independently: a `Center`-double-subtraction in both its
     `hyperboloid` branch (`v = r - (rX * rAxes[1] + center)`, `r` is
     already relative to `center`) and its `ellipsoid` branch
     (`rY = r - (rX * axis + center)`, plus `rY` was left as a `GVector`
     instead of a scalar distance), and a `radY, radY = radii` typo in
     the `ellipsoid` branch that left `radX` completely undefined
     (`UnboundLocalError` the moment a real ellipsoid reached this code).
     All 3 fixed to match the already-established, already-verified
     logic in `_occ_impl.py::GEllipsoid.is_inside`/`GHyperboloid.is_inside`.
  Verified: `elliptic_cylinder.mcnp`/`ellipsoid.mcnp`/`elliptic_cone.mcnp`
  all convert cleanly and match their analytic volumes on both `occ` and
  `ocp`; full `tests/geo` + `test_cadtocsg.py` + `test_csgtocad.py` +
  `test_georeverse_*_impl.py` + `test_gq_classification.py` +
  `test_boolean_function.py` still green on all 3 engines (199 passed +
  1 skipped on occ/ocp each) after all of the above -- these 3 bug
  fixes are zero-regression.
  **Classifier-dispatch mismatch, resolved 2026-09-15/16** (supersedes
  the "open, unresolved question" this entry used to end on): the
  `hyperboloid`/`cylinder_hyperbolic` GQ stypes are genuinely two
  different surfaces, per direct user clarification -- `hyperboloid`
  (from `get_hyperboloid_parameters`, all 3 eigenvalues nonzero) is
  always a revolution of the same hyperbola, `onesht=False` around its
  own `MajorAxis` (2 disjoint sheets, only the +axis branch is ever
  built, matching the existing `Gmake_hyperboloid` convention unchanged)
  and `onesht=True` around its own `MinorAxis` instead (the connected
  "hourglass", `Gmake_hyperbolic_cylinder`'s own revolve technique,
  unchanged) -- `Objects.py::Hyperboloid.buildShape` now branches on
  `onesht` to pick the axis/technique (previously always called
  `Gmake_hyperboloid`, ignoring `onesht` in this respect). Separately,
  `cylinder_hyperbolic` (from `get_cylinder_parameters`, a genuinely
  zero eigenvalue -- a true flat/straight-prism axis) is a FLAT prism
  (the hyperbola profile translated straight along the zero-eigenvalue
  axis, never revolved) -- restored under occ/ocp as a new
  `Gmake_hyperbolic_prism`/`GHyperbolicPrism` (an open compound of the
  profile's 2 branches, each `BRepPrimAPI_MakePrism`-extruded, no end
  caps -- mirrors `_freecad_impl.py::GHyperbolicCylinder.build_shape`'s
  own pre-existing technique exactly, which stays untouched and is now
  aliased to the same `Gmake_hyperbolic_prism` name via
  `GEOReverse/Modules/__init__.py`'s per-engine dispatch, so
  `Objects.py::HyperbolicCylinder.buildShape` calls one name uniformly
  across all 3 engines). Also fixed in the process: `GHyperbolicPrism`'s
  own `build_shape` used to conflate the Z-extrusion distance and the
  profile's own radial reach into a single `length` argument (ported
  as-is from `_freecad_impl.py`, which has the same conflation) -- wrong
  whenever the two extents genuinely differ (confirmed via a real
  fixture, `hyperbolic_cylinder_test.mcnp`: a 40cm-tall prism bounded by
  a 500cm coaxial cylinder needed the profile to reach ~500cm radially,
  but the old code capped it at ~40cm since it reused the Z-height) --
  split into two independent parameters, `extrusion_length` (along the
  true axis) and `y_reach` (along `MinorAxis`, sized from the real
  `boundBox`'s own projection onto that axis, not reused from the
  Z-height). `boundBox.py` gained `cylinder_hyperbolic_to_planes`
  (reuses the 2-sheet hyperboloid's own vertex/ring technique, called
  once per branch with normals negated -- the real material is the
  channel BETWEEN the two branches, i.e. before each one's own vertex,
  the opposite sense from a hyperboloid's own single real branch, which
  sits BEYOND its vertex). Separately, `surface_side`'s own `hyperboloid`
  branch needed one more sign fix beyond the Center-double-subtraction
  fix noted above: the 2-sheet case (`onesht=False`) needs the OPPOSITE
  boolean sense from the 1-sheet case for the SAME `d`/`Y` comparison
  (`inout = (d - Y) * one` and `inout = one` in the `else` branch, `one
  = 1 if onesht else -1` -- previously both branches used the 1-sheet
  sense unconditionally). `quadric_to_plane`/`plane_definition`'s own
  combinator tags for these were refined again after this fix, directly
  by the user, to keep the box-approximation side consistent with the
  new `surface_side` sign: `hyperboloid` now dispatches to `"hyp1sheet"`
  (onesht=True, an AND of `:`-joined per-branch-OR sub-sequences) or
  `"hyp2sheet"` (onesht=False, a plain flat AND list, but with the
  surface's own signed reference `s` negated right before the final
  `change_surf` calls -- `s = -s`, commented "hyperboloid 2 sheet has
  inverted orientation sign" -- to match `surface_side`'s own flipped
  sense for this case); `cylinder_hyperbolic` dispatches to `"cylhyp"`
  (an AND of the 2 branches' own `:`-joined OR sub-sequences). The exact
  tag names/structure may keep evolving -- `plane_definition` itself
  (`Utils/boundBox.py`) is the source of truth, not this paragraph.
  **STEP round-trip for `Geom_Hyperbola`-based revolution/extrusion
  surfaces, fixed 2026-09-15**: `Geom_SurfaceOfRevolution` built from a
  `Geom_Hyperbola` (the `hyperboloid`/`cylinder_hyperbolic` side faces)
  writes to STEP (AP214) as a valid-looking entity pair, but
  pythonocc-core 7.9's own `STEPControl_Reader` raises translating it
  back (`TransferRoots()` returns 0, the whole root silently dropped) --
  confirmed with a minimal, GEOUNED-free OCCT reproduction (bare
  `Geom_Hyperbola` + `BRepPrimAPI_MakeRevol`, no solid, no caps): the
  write succeeds, the read fails. Not a GEOUNED bug, and not fixable by
  changing how the surface is built. Fixed by converting just the
  revolution surface(s) to an equivalent `Geom_BSplineSurface` via
  `ShapeCustom.ConvertToBSpline` (`revolMode=True` only) -- applied ONLY
  at export time (`_build_tree`, gated by a cheap `_has_revolution_surface`
  face-type scan so the vast majority of solids never pay for it), NOT
  inside the surface constructors themselves (tried first, reverted per
  direct user request: every other consumer of these tools -- `Gsplit`/
  `Gcut`/`Gfuse` boolean cuts, volume/bbox queries -- must keep operating
  on the exact analytic hyperbola for correctness; only the final
  exported STEP document needs the approximation, and only for
  file-format compatibility). The default conversion is visibly coarse
  (~25x14 poles) -- 8 extra knots inserted into each converted face's own
  U/V knot vectors afterward for a denser control net (exact -- knot
  INSERTION, unlike `GeomConvert_ApproxSurface`-based re-approximation
  tried first, doesn't change the surface's own (u,v)->(x,y,z) mapping,
  so the pcurves `ShapeCustom.ConvertToBSpline` already built stay
  valid; the re-approximation attempt produced an invalid solid despite
  still round-tripping through STEP). Also fixed in the same area:
  `export_occ`/`export_ocp`'s own `STEPCAFControl_Writer.Transfer`/
  `.Write` calls weren't wrapped in `geo.io_utils.suppress_native_stdout`
  (unlike `geo`'s own `Gexport_step`), so every export printed OCCT's own
  "Statistics on Transfer (Write)" banner -- now silent.
  **Degenerate circular torus, fixed 2026-09-16**: `Objects.py::
  Torus.buildShape`'s own circular-vs-elliptic branch
  (`if abs(Rb - Rc) < 1e-5 and Ra > 0: Gmake_torus(...) else:
  Gmake_torus_elliptic(...)`) only checked `Ra > 0`, not degeneracy
  (`abs(Ra) < max(Rb, Rc)`) -- a degenerate CIRCULAR torus (equal minor
  radii, but still self-intersecting) took the plain `Gmake_torus`
  branch, whose own `BRepPrimAPI_MakeTorus` builds the FULL,
  self-intersecting double-sheet torus (both lobes merged into one
  ambiguous solid) instead of the correct single sheet
  `Gmake_torus_elliptic` already builds for the elliptic degenerate case
  via its own arc-splitting technique -- this is the SAME class of bug
  already fixed on the forward (`CadToCsg`) side for `GeounedSurface.
  build_surface()`'s Torus branch, just never ported to this (`CsgToCad`)
  side. Confirmed via a real fixture (`torus_circular_degenerate_outer.mcnp`,
  R=30cm, A=B=50cm): `Gsplit`'s own boolean cut against the ambiguous
  self-intersecting solid picked an interior point AT the origin for
  what should have been the cell's own "outside" piece, so the two
  pieces got fused back into the uncut container box instead of properly
  separating. Fixed by widening the degeneracy check to cover BOTH the
  circular and elliptic cases uniformly.
  **Complement-cell / degenerate-torus boundBox, closed out 2026-09-17**
  (supersedes the 2026-09-16 "Pending for next session" entry this used
  to be): that entry conflated two separate things.
  1. A real bug, now fixed directly by the user: `Utils/boundBox.py::
     torus_to_planes`'s degenerate-torus case used to fall through the
     SAME bounding-plane approximation as the non-degenerate torus
     (sized from the full, non-degenerate `majorRadius`/`minorR`
     geometry) -- wrong-shaped for a degenerate single sheet, and the
     proximate cause of the "complement never appears" symptom for the
     torus fixtures specifically. Fixed with a dedicated branch that
     derives the sheet's own tight axial half-height `h` from the real
     degenerate geometry (separately for the `outer`/inner cases and for
     `minorA > minorR` vs. `minorA <= minorR`), instead of reusing the
     non-degenerate shape. This needed knowing degeneracy (and inner/
     outer sense) reliably at every consumer, so a `degenerated` flag
     was threaded through as `Torus.params`'s 6th element: derived once,
     directly from the raw MCNP card's own `Ra` coefficient, at parse
     time (`MCNP_parser/MCNPinput.py::Get_primitive_surfaces`'s torus
     branch: `abs(Ra) < abs(Rc)` -> degenerate, `sign(Ra)` -> inner/
     outer) -- replacing the post-hoc `abs(Ra) < max(Rb, Rc)`
     re-derivation this session had added to `Objects.py::
     Torus.buildShape` -- and carried through `Torus.transform()`,
     `Torus.buildShape()` (now `Gmake_torus_elliptic(..., outer=deg >
     0)` when `deg != 0`, `outer=None` when `deg == 0`),
     `splitFunction.py::surface_side`'s torus branch (unpacked for
     signature symmetry, not otherwise used -- the point-vs-torus
     inequality itself doesn't need degeneracy), and
     `_freecad_impl.py::Gmake_torus_elliptic`/
     `_make_torus_elliptic_native` (now takes `degenerated` directly
     instead of re-deriving it via `abs(r_major_axis_offset) <
     minor_radius`). `XML_parser/XMLinput.py`'s own torus branch always
     sets `deg = 0` -- OpenMC-XML input has no degenerate-torus encoding
     in this path, unaffected.
  2. Per direct user clarification, the remaining part of the original
     symptom -- a torus fixture's own COMPLEMENT ("outside") solid not
     showing up as a distinct, tightly-bounded piece -- is **not a bug**:
     that region is genuinely unbounded (nothing else in these
     single-surface fixtures constrains it), so its own `boundBox`
     correctly falls back to the full universe box -- there is no
     tighter box to compute, the solid really is that large. And this
     kind of cell (everything outside an exotic quadric, out to the
     universe boundary) has no real MCNP/OpenMC modeling interest of its
     own regardless, so its absence (or an imprecise reported volume for
     it) isn't worth chasing further. This reasoning generalizes to
     every OTHER exotic quadric's own complement cell too (ellipsoid,
     hyperboloid, elliptic cylinder/cone, paraboloid) -- not a
     *hidden* bug there either, on the same basis.
  **4 more real, independent bugs found and fixed while finally wiring
  all 14 fixtures into real pytest tests, 2026-09-17** (the "not wired
  into a pytest test yet" gap the entry below used to flag -- writing
  the actual regression assertions against each fixture's own closed-form
  analytic volume immediately surfaced all 4, none previously caught
  because no prior verification of these surfaces/machinery went past
  "does it look roughly right"):
  1. `Utils/boundBox.py::parabola_to_planes` -- every one of its own 32
     tangent-plane approximations (not just the vertex-plane `p0`) had
     its normal built backward (pointing away from material instead of
     toward it, confirmed by cross-checking against the already-working
     `ellipsoid_to_planes`/`cone_to_planes`'s own convention: a bounding
     plane's normal must point from the far boundary point back toward
     the center, so a genuinely-interior point reads `dot>0`), so the
     resulting AND-of-tangent-planes had no satisfying point anywhere --
     `paraboloid.mcnp`'s own cell 1 never got a boundBox at all (`Box is
     None` unconditionally), so it was silently dropped and only its
     complement (the raw universe box) ever appeared. **Fixed directly
     by the user**: negate each tangent plane's own normal (`GPlane.
     from_values(xe, -normal)` instead of `xe, normal)` -- `p0` itself
     was already correctly signed, so this alone was sufficient. Verified:
     `paraboloid.mcnp`'s cell 1 now converts to `157,079,692,133.8 mm^3`
     vs. the closed-form `pi*2*Focal*length^2 = 157,079,632,679.5 mm^3`
     (relative error `3.8e-7`, the usual faceted-approximation residual).
  2. `CAD/splitFunction.py::surface_side`'s own `"torus"` branch used the
     same `(r - Ra)` tube-offset term regardless of `degenerated`'s own
     sign -- correct for the outer sheet and the non-degenerate case
     (confirmed, unaffected by this fix), but wrong for the inner sheet:
     geometrically, a degenerate torus's own inner lobe is *nested inside*
     the naive `(r-Ra)`-based region (both lobes' revolved arcs meet the
     axis at `r=0` and close there like the poles of a revolved half-circle,
     so the outer lobe's own disk-like cross-section at any height
     strictly contains the inner lobe's own smaller one) -- so a point
     genuinely outside the small inner-lobe solid but still radially
     within `Rc` of the algebraic circle at `r=Ra` (e.g. `r=250` for
     `Ra=300, Rc=500`) was still misread as "inside" by the unmodified
     formula. Confirmed live: `Gsplit` itself correctly split the cell's
     own tight bounding box into the two true pieces (the small
     `Gmake_torus_elliptic(outer=False)` lobe, and the box-minus-lobe
     remainder), but `surface_side` then misclassified *both* pieces as
     "inside" `cell 1`'s `-1` term, and fusing them back together
     reproduced the box's own full, uncut volume almost to the last digit
     (`351,232,000 mm^3`, exactly `560*560*1120` -- the boundBox's own
     20%-enlarged dimensions) -- a case where the final wrong answer was
     the *original, unsplit* box, but arrived at via a fully successful
     split immediately undone by misclassification, not via `Gsplit`
     silently declining to cut at all (the earlier, different failure
     mode this whole investigation started from). The correct region test
     for the inner lobe turns out to be the *same* formula with `Ra`
     negated (`(r+Ra)` in place of `(r-Ra)`) -- verified by direct
     derivation from the lobe's own true per-height radius (`r <=
     Rc*sqrt(1-(z/Rb)^2) - Ra`, which rearranges to exactly this) and by
     checking the 3 points that matter (center, the lobe's own true
     boundary, and a point just beyond it) by hand. **Fix**: `if
     degenerated < 0: Ra = -Ra` right after unpacking `surf.params`,
     before the existing formula (which is otherwise untouched). Verified:
     `torus_circular_degenerate_inner.mcnp` -> `57,299,667.48 mm^3` and
     `torus_elliptic_degenerate_inner.mcnp` -> `45,839,733.98 mm^3`, both
     now matching their own closed-form (Pappus-integral-over-the-kept-arc)
     volumes to double-precision (relative error `~1e-10`), and the outer/
     non-degenerate cases (previously already correct) are bit-for-bit
     unchanged since `degenerated >= 0` never enters the new branch.
     Verified no corpus-wide regression: freecad `tests/geo` +
     `test_cadtocsg.py` + `test_csgtocad.py` (163 passed), and a 141-file
     differential composite-surface-count scan across `Solidos/
     test_models` showing 0 changed files (expected -- this fix only
     changes the final volume/split outcome for a degenerate-inner torus,
     never any file's own Can/TCone/RoundC/MultiRoundC/MultiP/RevCC
     classification counts, and no file in that corpus contains a
     degenerate-inner torus to begin with).
  3. `GEOReverse/core.py::CsgToCad.__init__`/`Objects.py::CadCell.__init__`
     both had a classic Python mutable-default-argument bug:
     `def __init__(self, settings: BoxSettings = BoxSettings()):` --
     `BoxSettings()` is evaluated ONCE, at module-import time, so every
     `CsgToCad()`/`CadCell()` call made without an explicit `settings=`
     shared the exact same instance (confirmed live: `CsgToCad().settings
     is CsgToCad().settings` was `True`). Found while chasing down an
     apparent "flaky" test result -- `hyperbolic_cylinder_test.mcnp`
     converted to a visibly different (both plausible-looking, neither
     obviously wrong) volume depending on which *other* fixture had been
     converted earlier in the same process (e.g. right after
     `ellipse_cyl.mcnp`/`elliptic_cone.mcnp` specifically, not after
     others) -- exactly the signature of shared mutable state. **Fixed**:
     the standard idiom, `settings: BoxSettings = None` plus `self.settings
     = settings if settings is not None else BoxSettings()` in the body,
     in both classes (`CadCell`'s own version was never actually
     triggered by any real call site -- every one already passes
     `settings=` explicitly -- but the same antipattern, fixed
     defensively). Confirmed the sharing itself is gone
     (`CsgToCad().settings is CsgToCad().settings` now `False`) -- but
     this alone did **not** fix the actual `hyperbolic_cylinder_test`
     discrepancy (see bug 4), meaning `BoxSettings` sharing specifically
     was never the mechanism, just a real, separate bug surfaced by the
     same investigation.
  4. **The real cause of that same discrepancy, and by far the most
     consequential bug found this session**: `Utils/boundBox.py::
     myBox.add()`/`.mult()` (the OR/AND combinators for the `Orientation=
     "Forward"` (material inside `Box`) / `"Reversed"` (material outside
     `Box`, `Box=None`+Reversed=the whole universe) bounding-box
     approximation used to size every cell's own starting container
     before `Gsplit`) were WRONG -- not just imprecise -- for essentially
     every combination involving one Forward and one Reversed operand,
     and for one `Reversed`+`Reversed` sub-case. Confirmed via a
     systematic empirical audit (methodology directly specified by the
     user: build concrete axis-aligned box pairs A/B covering 8 relative
     configurations -- disjoint, A subset of B, B subset of A, partial
     overlap, each also tried with `notA`/`notB` -- across all 6 relevant
     operand-orientation combinations of `+`/`*`, and check the *real*
     minimal enclosing box of "all the material" by direct point-
     membership sampling, not by trusting either implementation's own
     formulas) -- **20 of the first 32 checks came back UNSAFE**, meaning
     `myBox`'s own claimed material region did not just include some
     extra empty space (always allowed) but actively EXCLUDED real
     material. Root cause: once both operands have a real `Box`, the old
     code always computed `self.Box.union(box.Box)` (in `add`) or
     `self.Box.intersected(box.Box)` (in `mult`) regardless of
     orientation -- correct only for Forward-OR-Forward and
     Forward-AND-Forward respectively; every mixed case needs a
     genuinely different formula since a `Reversed` operand's own `Box`
     represents the *excluded* region, not the material itself. Fixed,
     each verified by re-running the same empirical audit until 0 UNSAFE
     remained (32, then 40 once a targeted 5th configuration -- two
     overlapping boxes whose own union still leaves a gap relative to its
     combined bounding box -- caught one more real case the first pass
     had missed):
     - `add()`, exactly one Reversed (`A + notB`): the true result
       (`universe \\ (B \\ A)`) is generally not expressible as one box at
       all. Exact and safe only when `A` and `B` don't overlap at all
       (then `B \\ A == B` exactly); falls back to the always-safe
       `Box=None` ("material=universe") otherwise, replacing the old
       `union(A,B)` (confirmed unsafe: e.g. disjoint `A`,`B` gave
       Reversed+`union(A,B)`, wrongly excluding all of `A`, which
       trivially must be material since `A subset of (A + notB)`).
     - `add()`, both Reversed (`notA + notB`): De Morgan gives
       `not(A and B)`, i.e. Reversed with `Box` = the *intersection* of
       `A` and `B` (empty when disjoint) -- the old code's `union(A,B)`
       was silently computing the *other* De Morgan identity's answer
       (`not(A or B)`, `mult`'s own case) instead. Intersection of two
       axis-aligned boxes is always itself exactly one box (or empty), so
       this branch needed no further special-casing -- confirmed exact in
       all tested configurations.
     - `mult()`, exactly one Reversed (`A * notB`, i.e. `A \\ B`): always
       a subset of `A` itself, so the safe choice needs no case analysis
       at all -- keep the Forward operand's own `Box` completely
       unchanged, discarding the Reversed operand's `Box` entirely (exact
       whenever the two don't overlap, safe-but-loose otherwise). The old
       code's `self.Box.intersected(box.Box)` here was the Forward-AND-
       Forward formula misapplied -- confirmed unsafe: disjoint `A`,`B`
       gave Forward+`None` (empty!) for `A * notB`, when the true answer
       is all of `A`.
     - `mult()`, both Reversed (`notA * notB`): De Morgan gives
       `not(A or B)`, Reversed with `Box` = union of `A` and `B` --
       *this* one really was already using `union`, but the *union of
       two boxes* -- unlike an intersection -- is only exactly one box
       when the two combine with no gap relative to their own combined
       bounding box (one containing the other, or sharing a full common
       range on one axis and overlapping/touching on another); otherwise
       the bounding box overshoots the true excluded region, which is
       unsafe here specifically because `Reversed`'s `Box` represents
       what's excluded (confirmed unsafe with 2 different configurations:
       disjoint `A`/`B`, and an L-shaped pair of boxes sharing only a
       corner). Fixed by checking the exact inclusion-exclusion identity
       (`vol(union) == vol(A) + vol(B) - vol(A∩B)`, true iff there's no
       gap) and falling back to the larger of the two boxes alone
       (always safe, `A subset of (A union B)` trivially) when it doesn't
       hold.
     Verified: the same empirical audit script, 0 UNSAFE across 40
     checks (27 exact, 13 safe-but-loose); full freecad `tests/geo` +
     `test_cadtocsg.py` + `test_csgtocad.py` (163 passed, 14 skipped);
     `tests/test_csgtocad.py` (all 16, including all 14 exotic-quadric
     fixtures) green on `occ` and `ocp` too; and, directly confirming this
     was the real mechanism behind bug 3's own symptom,
     `hyperbolic_cylinder_test.mcnp` now converts to the identical,
     analytically-correct volume (`19,202,339,805.3 mm^3`) whether run in
     isolation or immediately after `elliptic_cone.mcnp`/`ellipse_cyl.mcnp`
     in the same process -- the flakiness is gone. This is likely the
     single highest-impact fix of this whole multi-day GEOReverse
     investigation: `myBox` sizes the starting container for every
     `Gsplit` call in the entire CSG->CAD pipeline, so a mixed Forward/
     Reversed cell definition silently losing real material (or, in the
     `notA*notB` case, an oversized-but-technically-"safe" box triggering
     an ill-conditioned cut sensitive to unrelated prior floating-point
     state) could plausibly explain multiple previously-unexplained
     GEOReverse oddities from earlier in this project's history, not just
     the one this session happened to chase down.
  **New test fixtures, 2026-09-14/16, wired into a real pytest test
  2026-09-17** (`tests/test_csgtocad.py::test_exotic_quadric_convertion`,
  parametrized over all 14 -- see that same session's entry below for
  the 4 real bugs found and fixed while writing it): all copied into
  `tests/csg_files/`: `ellipsoid.mcnp`, `ellipse_cyl.mcnp`, `elliptic_cone.mcnp`,
  `paraboloid.mcnp`, `hyperboloid_one_sheet.mcnp`,
  `hyperboloid_two_sheet_one_branch.mcnp` (kept at `1 2 -3`, the correct
  sense for the real branch), `hyperboloid_two_sheet_outside.mcnp` (the
  `-1` "outside" sense instead, radially bounded by a coaxial `CZ`
  cylinder so it stays finite), `hyperbolic_cylinder_test.mcnp` (the real
  flat-prism `cylinder_hyperbolic` surface), `cooling_tower.mcnp` (2
  coaxial hourglasses + `PZ` caps, the fixture that originally motivated
  this whole investigation), and 5 torus fixtures --
  `torus_elliptic_nondegenerate.mcnp`, `torus_circular_degenerate_outer/
  inner.mcnp`, `torus_elliptic_degenerate_outer/inner.mcnp` (inner/outer
  selected via the sign of the card's own major radius `R`, per
  `Gmake_torus_elliptic`'s own established round-trip convention). All
  individually verified end to end against their own analytic volumes on
  `occ`/`ocp` (matching within the tiny residual expected from
  `convert_to_planes`'s own faceted approximation) EXCEPT the torus
  fixtures' own complement cell, per the pending item above.
  `GHyperboloid`'s `OneSheet` flag semantics and how `MCNP_parser`/
  `XML_parser` populate it are now confirmed self-consistent (see the
  classifier-dispatch resolution above) -- the semantic-risk flag this
  entry used to carry is closed.
  `freecad`'s own exotic-quadric implementations (`_freecad_impl.py`)
  were deliberately left untouched throughout, and now genuinely
  correspond 1:1 with the occ/ocp techniques for all 7 surfaces
  (including `cylinder_hyperbolic`, now that its own flat-prism
  technique is shared via the `Gmake_hyperbolic_prism` alias) --
  the only remaining engine asymmetry is implementation detail (native
  `Part.Hyperbola`/`toBSpline` vs `Geom_Hyperbola`/`BRepPrimAPI_MakePrism`),
  not a different surface. `freecad`'s own `CAD/splitFunction.py::
  surface_side` is the SAME file across all 3 engines (no per-engine
  branching there), so every fix in this entire section applies equally
  to `freecad`.

### Shared / cross-cutting (touches both pipelines, or is test-fixture housekeeping)

- `GEOUNED`'s `build_region/` and `GEOReverse`'s parallel
  `CAD/buildSolidCell.py`/`CAD/splitFunction.py` remain two separate
  implementations of near-identical logic. A written portability
  analysis exists (see the history log's "`FuseSolid` ->
  `geo.Gfuse_solids`; `build_region` portability analysis" entry) but no
  further code has moved beyond `Gfuse_solids` itself.
- `GEOUNED` and `GEOReverse` used to carry two separate `BoolSequence`
  implementations (a 3-tier `int`/`BoolVariable`/`BoolSurface` system in
  GEOUNED vs. a plain-`int`-only class in GEOReverse). The shared parser
  (`outer_terms`/`redundant`/`is_integer`, now
  `src/geouned/boolean_utils/boolean_expression_parser.py`) was
  extracted first. Two real bugs were then found and fixed in
  GEOReverse's own class: `Objects.py::cleanUndefined()` called a
  nonexistent `.removeSurface(...)`, and `BoolSequence.removeSurf` only
  matched positive surface references, silently leaving a negative
  reference untouched.
- **Unification completed 2026-09-12**: per explicit user direction,
  GEOReverse no longer carries its own `BoolSequence` class at all --
  `GEOReverse/Modules/Utils/booleanFunction.py` now imports the
  canonical class directly (`from ....boolean_utils.boolean_function
  import BoolSequence`; both `GEOUNED` and `GEOReverse` import from this
  shared module, moved there from `GEOUNED/utils/boolean_function.py` --
  see the `boolean_utils` paragraph in "Current architecture") and the
  class itself was left untouched throughout (it is historically
  fragile and load-bearing in GEOUNED). Only 3 free functions
  remain in that file instead of class methods: `remove_surf`/
  `signed_surfaces` (no GEOUNED equivalent at all), and
  `evaluate_three_valued` -- a **thin adapter, not a reimplementation**:
  it calls GEOUNED's own `.evaluate()` and downgrades a non-bool result
  (the residual `BoolSequence` GEOUNED's `.evaluate()` returns for an
  undetermined case) to plain `None`, which every real caller here needs
  (`boundBox.py`'s `isInside`/`splitFunction.py`'s `if inSolid: ... elif
  inSolid is None: ...`). The first attempt at this wrongly reimplemented
  a whole second copy of GEOReverse's old three-valued walk plus its old
  `simplify`/`factorize` -- caught on user review ("no entiendo porque
  has creado estas funciones... y no has implementado un decorador de la
  funcion BoolSequence.evaluate()"); confirmed by randomized testing
  (3000 generated expressions) that the thin wrapper is at least as
  resolving as the hand-rolled walk, and strictly more so in some cases
  (`.evaluate()`'s use of `.substitute()` catches structural
  contradictions -- e.g. an inner OR collapsing until an outer AND is
  left holding both `+n` and `-n` of the same surface -- that a single
  top-down tree walk misses); separately confirmed GEOReverse's old
  `simplify`/`factorize` had exactly one caller, `remh.py::hash_sequence`,
  which is itself dead code (imported, never called) -- so that pair was
  deleted outright rather than ported, and `hash_sequence` (still dead)
  now calls GEOUNED's own `.simplify()` directly.
  Three further real, latent bugs surfaced only once GEOReverse started
  depending on GEOUNED's stricter class, all fixed in GEOReverse's own
  code (not GEOUNED's): `Objects.py::copy()`'s `self.surfaceList[:]`
  (sets don't support slicing, and `get_surfaces_numbers()` can return
  either a tuple -- from `remh.py::Cline`, before the cell definition is
  converted -- or a set -- from `BoolSequence`, after) -- fixed to
  rebuild the same container type explicitly; `boundBox.py::change_surf`
  used to index-assign a bare `True`/`False` directly into a
  `BoolSequence.elements` list (relying on GEOReverse's own old
  `.clean()` to absorb it) -- GEOUNED's class only expects list entries
  to be int literals or `BoolSequence` instances, and crashes `.copy()`
  on a nested bare bool -- fixed to drop the identity element from the
  list outright instead; and `boundBox.py::isInside`/
  `splitFunction.py::surface_side` fed a `numpy.bool_` (from a `GVector`
  dot-product comparison) into `evaluate_three_valued`'s value dict --
  GEOUNED's own `substitute()` branches on `type(val) is not bool`
  (`True` for `numpy.bool_`, unlike GEOReverse's old `type(val) is int`
  check, which defaulted anything non-int including a numpy bool to the
  correct branch), so it silently took the wrong branch and corrupted the
  sequence -- fixed by forcing a plain `bool(...)` at both source sites.
  Verified clean (zero new regressions) across all 3 engines, `freecad`
  also via the real `test_csgtocad.py` pipeline. Covered by
  `tests/test_boolean_function.py`.
- `Solidos/` STEP fixture tree (the test/regression corpus used mainly
  for GEOUNED verification) still has real, unresolved duplicates
  across the triage folders (`Solidos/test_models` is the curated
  regression set; older ad-hoc folders overlap with it in places).

## Reference docs

- `docs/history/geouned_migration_log.md` — the full chronological
  session-by-session investigation log this file used to contain
  inline: every bug found and fixed, the verification methodology used
  (d1suned MCNP volume checks, `Solidos/` corpus differential scans,
  point-sampling against real CAD ground truth), dead ends and why they
  were rejected, and every "Pending tasks" checkpoint in the order it
  was written. Read this for the detailed story behind any fix
  mentioned above, or before repeating an investigation that may
  already have been done.
- `docs/reference/composite_surfaces.md` — consolidated, topic-organized
  reference for the 6 composite/meta-surface types (MultiPlane, Can,
  TCone, RoundCorner, MultiRoundCorner, ReversedConeCylinder): their
  definitions, detection chain, AND/OR region rule, and construction.
- `docs/reports/pyocc_migration_report_{en,es}.pdf` (+ `.html`/`.tex`
  sources) — a written technical report on the FreeCAD -> pyOCC
  migration for external readers.

## Environment notes

- `GEOUNED_CAD_ENGINE` selects the geometry backend: `ocp` (default),
  `occ` (pythonocc-core/SWIG), or `freecad`.
- The pyOCC engines run in dedicated conda environments, not the
  default Python: `pyoccenv` (pythonocc-core,
  `C:\Users\Patrick\Apps\Conda\envs\pyoccenv\python.exe`) and `ocpenv`
  (OCP, `C:\Users\Patrick\Apps\Conda\envs\ocpenv\python.exe`) — see the
  `reference-pyoccenv-path`/`reference-ocpenv-path` memories.
- Always use PowerShell, not Bash, to invoke either pyOCC env's
  `python.exe` directly — Bash gives exit 127 with zero output for
  reasons never root-caused. FreeCAD and either pyOCC engine cannot
  coexist in one process (a bundled-Python DLL conflict) — never mix
  engines in one invocation.
- Workshop conversion/verification scripts (batch conversion, d1suned
  runs, corpus diffs) already exist under `SolidTestMCNP/scripts/` on
  the WSL side — check there before writing a new ad-hoc script (see
  the `reference-d1suned-pipeline` memory).

## Code style preference

- User prefers speaking/planning in Spanish, but ALL code — including
  comments, docstrings, and variable/function names — must be written
  in English.
