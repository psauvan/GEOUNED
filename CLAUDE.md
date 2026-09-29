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
  `GVector`, `GMatrix`, `GBoundBox` and their `to_g*` converters, and,
  since 2026-09-17, `myBox` (the `Forward`/`Reversed` box-arithmetic
  approximation of a boolean cell's material region -- see "Known open
  items" -> Shared/cross-cutting for why it moved here and the real bug
  it carried in both its previous, independently-maintained copies).
- `geo/surface_geometry.py` — pure, duck-typed analytic-surface
  predicates shared by all 3 engines (`is_same_*_surface`,
  `is_coaxial_cone_*_pair`, `is_inside_*`, `find_can_plane`, ...).
- `geo/solid_defects.py` — whole-`GSolid` CAD-defect detection
  (`find_short_edges`, `find_split_ring_faces`, `check_solid_defects`,
  `valid_solid`), used by both the load-time repair cascade and
  `Gsplit`'s own post-split sanity filter.
- `geo/solid_ops.py` — `Gfuse_solids`, a shared policy helper (repair +
  fuse + compound fallback) one layer above the raw `Gfuse` kernel
  primitive; and, since 2026-09-17/18, the full split cascade
  (`BuildDepth`/`BuildSolidParts`/`filterparts`/`getPart`/`SplitBase`/
  `joinBase`/`SplitSolid`/`space_decomposition`) that reconstructs a
  cell's solid by recursively splitting a starting shape against each of
  its own real surfaces and keeping/rejecting/re-splitting the resulting
  pieces per its boolean definition -- used by both GEOUNED's
  `build_region/` (to construct the small solid a composite meta-surface
  itself represents) and GEOReverse's `CAD/buildSolidCell.py` (to
  reconstruct an arbitrary MCNP/OpenMC cell's solid from its own boolean
  definition). See "Known open items" -> Shared/cross-cutting for the
  full unification history.
- `geo/io_utils.py` — `suppress_native_stdout` (silences OCCT's own
  STEP-write console banner) and `GLabelNode` (STEP assembly/label tree).
- `geo/constants.py` — every intrinsic tuning threshold: the repair
  cascade's volume-conservation gates and, since 2026-09-19, every
  tolerance literal that used to be inline in `GEOUNED/`/`geo/`, named by
  ROLE plus the exponent of its historical value (`LENGTH_TOL_E5`,
  `ZERO_TOL_E9`, `KERNEL_TOL_E7`, `POINT_POINT_TOL`, `NUMERIC_TOL`,
  `VOLUME_REF`, `MAX_REPAIR_VOLUME_REL_CHANGE`, ...),
  so no engine carries its own duplicate copy of a literal. See "Known
  open items" -> Shared/cross-cutting -> "Tolerances" for the rule that
  decides what lives here and what is a user-facing `Tolerances` field.
- `geo/tolerances.py` — `GeoTolerances`, the tolerances `geo` itself reads
  (surface identity `pln/cyl/sph/kne/tor_distance`+`_angle`, sliver/kernel
  fields). `geouned.Tolerances` extends it with what only CadToCsg uses;
  GEOReverse uses the base directly (it has no tolerance a user should
  change, decided 2026-09-19).
- `geo/volume_utils.py` — `volume_within(value, expected, rel_tol)`, the ONE
  pattern for every relative volume comparison (`tol * max(|ref|,
  VOLUME_REF)`).
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
(`CAD/buildSolidCell.py`/`CAD/splitFunction.py`/`Objects.py`) shares its
actual split cascade (`BuildDepth`/`BuildSolidParts`/`filterparts`/
`SplitSolid`/`myBox`) with GEOUNED's `build_region/` directly via
`geo/solid_ops.py`/`geo/vector_geometry.py` since 2026-09-17/18 -- see
"Known open items" -> Shared/cross-cutting for the unification itself.
Each pipeline's own surface/cell model (`CadCell`+its exotic-quadric-
aware surface classes vs. `CellObj`+`CellSurface`) and point-
classification code (`surface_side` vs. `CellSurface.is_inside`) remain
genuinely distinct, by design -- see that same entry for why.

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
list — see that entry for the verification numbers.

Status as of 2026-09-18: every entry below (except the new
"Spline-vs-quadric identification" one under "Shared / cross-cutting",
opened this same day) is either closed-and-documented (kept for the
history/rationale, not as a to-do) or an explicitly accepted permanent
limitation (`Mixed/ConeSphere.stp` under occ/ocp; freecad's own
exotic-quadric construction bugs, out of scope per direct user
instruction).

### GEOUNED (`CadToCsg`, the forward STEP -> CSG pipeline)

- **Regression found and fixed, 2026-09-27: `generic_split`'s post-split
  volume-conservation gate (added 2026-09-12, commit `8149030`) was too
  tight for a real, otherwise-correct split, silently leaving a solid
  permanently unsplit.** User-reported symptom:
  `Big_model_reserved/shed_shutter.stp`'s first solid stopped producing
  multiple irreducible pieces even though stepping through `generic_split`
  showed real boolean cuts happening and cut solids being obtained --
  reproduced identically under both `Options.meta_surfaces=True` and
  `False`, ruling that feature out immediately. First hypothesis (that
  this was a side effect of the same session's own large tolerances-
  renaming/retuning effort) was directly tested and ruled out: at the
  pre-tolerances-refactor commit `22f0f51`, the exact same "1 piece, all
  candidates rejected" behavior reproduces with both the old
  (`volume_tolerance=1e-6`) and new (`1e-4`) default, unchanged -- **per
  direct user instruction, this was not accepted as "always broken" or
  pre-existing**: "no, es algo que se ha roto, porque hubo un momento que
  este solido traducia correctamente. vuelve mas atras en los commit". A
  manual binary-search bisection (temporary `git worktree`s at successive
  candidate commits, each tested via the plain `geouned.CadToCsg` public
  API against the user's own exact settings/tolerances since internal
  function signatures drift across old commits) narrowed the regression
  to the exact adjacent-commit boundary `2d16c45` (working, 4 pieces) /
  `8149030` (broken, 1 piece) -- i.e. `8149030` itself is the culprit.
  That commit's own message ("large_cell_plane_split: fix a silent
  BOPAlgo volume loss...") already names the mechanism: it added a
  post-split check in `decom_one_generators.py::generic_split` --
  `if not volume_within(piece_sum, orig_vol, tolerances.volume_tolerance):
  discard the candidate` -- to catch a real, confirmed bug on a
  DIFFERENT file (`Big_complex_cell/modelCell_670000.stp`, a MultiPlane
  candidate silently losing ~26% of a fragment's volume to BOPAlgo
  under-separation) and, per the migration log, a second, earlier case
  losing ~99.93% of a fragment's volume (2 real tiny slivers replacing
  nearly the whole base). The `1e-4` relative threshold for this check
  was chosen directly against that one bug, never measured against a
  genuinely correct split's own natural volume noise -- confirmed live:
  `shed_shutter.stp`'s first solid has 3 real, otherwise-correct
  candidate plane splits, each with a stable ~5.5e-4 relative volume
  EXCESS (not a deficit -- the opposite sign from the under-separation
  bugs this gate targets), reproducible and unchanged at any looser
  tolerance from `1e-3` up to `3e-2` (the accepted split is always the
  same 4 pieces, summing to the same +5.51e-4 relative excess) -- a
  genuine, small, inherent BOPAlgo split-tessellation discrepancy for
  this particular real-world geometry, three orders of magnitude below
  the two known genuine bugs (0.26, 0.9993) this gate was built to catch.
  Because this solid's own heal-retry gate
  (`Gsolid_max_tolerance(solid) > tol_floor and
  Gsolid_nonmanifold_edge_count(solid) >= 1`) never fires here (this
  isn't a tolerance-weld case), every one of its 3 candidates being
  rejected left the solid permanently unsplit with only a WARNING-level
  log line -- no crash, no test failure, nothing surfacing the problem
  short of noticing the missing pieces directly.
  **Measured before fixing** (per this project's own "measure, don't
  guess" convention): a 144-file scan of `Solidos/test_models` (every
  folder except `Big_model_reserved`, per standing policy, and excluding
  the already-documented `Mixed/ConeSphere.stp` native crash) with
  DEFAULT tolerances found **zero** rejection-warning events anywhere in
  the existing corpus -- this false positive is specific to
  `shed_shutter.stp`, and no file provides a data point between the one
  measured legitimate deviation (5.5e-4) and the two known genuine bugs,
  so no tighter value could be justified from real data.
  **Fix**: split a new, dedicated `SPLIT_CANDIDATE_VOLUME_REL_TOL = 1e-2`
  constant out of `geo/constants.py`, used ONLY at this one
  candidate-accept/reject site in `generic_split` -- `tolerances.
  volume_tolerance`'s other roles (`Gsplit`'s own internal tool-removal
  check, `Gmerge_coplanar_planes`, the repair cascade's volume gates, the
  top-level "Lost X%" final warning in `split_surfaces`) are all
  deliberately untouched, since those confirm an (almost) exact operation
  and are correctly tight; this one instead decides whether to keep
  searching for a different candidate surface at all, where a false
  rejection's cost (silently giving up on decomposing the solid
  entirely) is far higher than accepting a slightly-noisy but genuine
  split. `1e-2` gives a roughly symmetric log-scale margin: ~18x above
  the one measured legitimate deviation, ~26x below the smallest known
  genuine under-separation.
  **Verified**: `shed_shutter.stp`'s first solid now decomposes into 4
  pieces (matching the volume-tolerance-loosened experiment used to
  confirm the diagnosis). Full suites green on all 3 engines after the
  fix (ocp 298 passed/1 skipped, occ 298 passed/1 skipped, freecad 309
  passed/14 skipped -- each engine's own established baseline, zero
  regressions). A before/after differential of the same 144-file corpus
  scan (`git worktree` at the pre-fix commit vs. the fixed working tree,
  comparing per-file piece count and total volume) found **zero real
  differences** anywhere -- confirming the widened threshold changes
  nothing for any file that was already decomposing correctly, and
  exists purely to stop rejecting this one previously-silent false
  positive (and any future file that happens to share this class of
  small, genuine split noise).

- **`gen_plane_cone`/`gen_plane_cylinder`: frame-mismatch bug fixed,
  2026-09-28 -- root cause of `Hollow_plates/placa2.stp` missing ~39% of
  material (d1suned tally 0.609).** User-reported symptom: the CSG cell
  definition for this solid's first RevCC-bounded region was missing
  real material; user explicitly rejected treating it as pre-existing
  ("no, es algo que se ha roto, porque hubo un momento que este solido
  traducia correctamente. vuelve mas atras en los commit") and directed
  a `git worktree`-based binary-search bisection (stable high-level
  `geouned.CadToCsg` public API only, `freecad` engine, since internal
  signatures drift across ~578 commits) that narrowed the regression's
  *symptom* to commit `54eefa5` (the `get_surfaces` cylinder/cone-before-
  planes reorder) -- but the underlying defect turned out to predate
  that commit by a wide margin: `54eefa5` only changed which faces end
  up as RevCC chain segments with a local U-parameter frame origin
  different from their shell's own common frame, exposing a latent bug
  rather than introducing one.
  **Root cause**: `ShellFaceGu.U_parameter_range` (`_U_parameter_faces`
  in `geometry_gu.py`) re-expresses each constituent face's own U
  interval into a COMMON reference frame (based on `Faces[0]`'s own
  classification) so `arc_extent` can join/merge intervals across faces
  with different native U origins. `gen_plane_cone`
  (`meta_surfaces_utils.py`, builds a RevCC/TCone segment's own closing
  plane from two boundary points V1/V2) took this COMMON-frame
  `Umin`/`Umax` and used it directly to search the LOCAL, per-face
  tessellated `UVNode` list (`get_shell_UV_nodes`/`face.getUVNodes()`)
  for the matching point -- a genuine frame mismatch whenever a face's
  own local U origin differs from the shell's common frame (routine for
  a real multi-face RevCC chain segment), silently picking the wrong
  UVNode and therefore the wrong V1/V2, which built a wrongly-placed
  closing plane. `get_join_cone_cyl`'s own `extreme_edge` calls (used
  for chain-continuation decisions, not plane construction) had the
  exact same mismatch -- flagged by direct user diagnosis ("La funcion
  extreme_edge en get_join_cone_cyl no coge los edges correctos. Creo
  que ya hemos tenido problemas con esta funcion, no es muy robusta"),
  confirmed via tracing (3/4 calls picked an edge 0.6-2.8 rad away from
  the true target) but confirmed NOT to affect placa2's own tally
  (0.609212 unchanged after fixing this alone -- it only feeds chain-
  continuation logic, not plane placement).
  **A wrong hypothesis tried and reverted first**: swapping
  `gen_plane_cone`'s own `dir2.cross(dir1)` -> `dir1.cross(dir2)` (a
  plausible-looking cross-product-order asymmetry vs.
  `gen_plane_cylinder`'s own formula) made the tally WORSE (0.407, not
  0.609) -- reverted immediately. The cross-product order was never the
  bug; the real defect was V1/V2 themselves being the wrong points
  entirely, not merely sign-flipped -- a blanket formula change broke
  other, previously-correct instances of the same shared function.
  **Fix** (both `gen_plane_cone` and, by the identical pattern,
  `gen_plane_cylinder`): use `Faces[ifacemin].ParameterRange[0]` /
  `Faces[ifacemax].ParameterRange[1]` (each face's own LOCAL boundary,
  which -- by `U_parameter_range`'s own construction -- is exactly what
  `Umin`/`Umax` equal in the common frame) instead of the common-frame
  `Umin`/`Umax` directly, when searching that face's own local UVNode
  list. **Verified via d1suned**: `placa2.stp` tally 0.60921 -> 0.99855
  (+/-0.24%, statistically exact).
  **Side fix found and applied in the same investigation, real but not
  placa2's cause**: `get_tcone_surfaces` (`meta_surfaces.py`) was
  missing the same Forward+OR-rejection / Reversed+AND-to-OR-with-
  continuity "open mouth" handling `get_can_surfaces` already had --
  added for consistency (mirrors the Can logic exactly); confirmed via
  `git stash`-based before/after that it changes 3 written terms on
  `placa2.stp` but leaves its d1suned tally statistically unchanged
  (0.609 -> 0.609212).
  **Verified**: full suites green on all 3 engines (ocp 342 passed/2
  skipped, occ 342 passed/2 skipped, freecad 310 passed/18 skipped --
  combined with the `_is_closed_by_winding` fix below and the user's own
  `cell_definition.py` boolean-simplification re-enable, all in the same
  working tree). A 144-file `Solidos/test_models` differential (full
  pipeline: decompose + build_solid_definition + build_void + export,
  comparing WRITTEN MCNP TEXT, not just decomposed-piece volume -- the
  latter is USELESS for verifying a `build_solid_definition`-stage fix
  like this one, since it never touches `decompose_solids()`'s own
  output) found 12 DIFF files (`Hollow_plates/cylcone_exact_placa3_pos`,
  `placa`, `placa2`, `placa3_axis_aligned`, `placa3_near_origin`,
  `placa3_real_solid`, `placa3_roundtrip`, `placathin`; `Mixed/ring`,
  `sleeve`; `RevCC_regression/cyl_cone`; `Reversed_Cyl_Cones/cyl_cone`)
  and 8 CRASH files, all 8 identical before/after (pre-existing, not
  caused by this fix -- includes the already-documented
  `Enclosures/w_encl.stp` load-time bug). Besides `placa2` itself,
  d1suned-verified: `RevCC_regression/cyl_cone.stp` and
  `Reversed_Cyl_Cones/cyl_cone.stp` (both directly named after the RevCC
  chain-join functionality this fix touches) both give tally 0.99654
  +/-0.29% (1.2 sigma), 0 lost particles, SD4 matches true CAD volume.
  The remaining 8 DIFF files were checked for decomposed-piece VOLUME
  conservation only (identical before/after, as expected/uninformative
  for this stage) -- not individually d1suned-verified.

- **`_is_closed_by_winding`: degenerate-axis-point bug fixed, 2026-09-28
  -- a genuinely closed cone/cylinder (boundary loop passing through its
  own axis, e.g. a full cone's apex) could be wrongly classified as NOT
  closed.** User-reported: on `working_solids/can_cone.stp`, this
  function returned `False` for a cone face the user confirmed by direct
  inspection is closed. **Root cause**: `_angle_function`'s own
  `angle_of(point)` computes `atan2(rel.dot(e2), rel.dot(e1))` where
  `rel` is the point's position perpendicular to the surface's axis --
  for a point sitting ON the axis (e.g. a cone's own apex vertex,
  `rel`'s length ~0), the azimuthal angle is genuinely undefined, but
  `atan2(0, 0)` silently returns `0.0` by convention, not an error. The
  standard OCCT topology for a full closed cone is apex vertex + a seam
  generatrix edge (apex to rim) + the base circle + the seam generatrix
  back (rim to apex) -- walking this loop's sampled points, the samples
  AT the degenerate apex all read angle=0.0 (an arbitrary artifact, not
  a real direction), while the adjacent real samples just off the apex
  read the generatrix's own true, constant angle (confirmed via a
  dedicated per-sample trace on `can_cone.stp`: apex samples all read
  0deg, first real sample off the apex reads 90deg -- a spurious ~90deg
  "jump" with no physical meaning). `_loop_closes_full_turn`'s own run-
  accumulation logic (which requires each monotonic-direction run to
  itself close a whole multiple of 2*pi, correctly needed for a genuine
  annulus-shaped boundary's top/bottom-rim sense reversal) then glued
  this artifact jump onto the real, correct 360deg sweep of the base
  circle, making the combined run read as 450deg (not closing) followed
  by a stray -90deg leftover run (also not closing) -- so a genuinely
  closed cone read as open.
  **Fix**: `_angle_function`'s `angle_of` now returns `None` when the
  point's perpendicular-to-axis distance is below `POINT_POINT_TOL`
  (on the axis, angle undefined); both consumers
  (`_oriented_angle_sweep`'s fast-path single-edge check,
  `_loop_closes_full_turn`'s general per-sample-point walk) filter out
  `None` samples before computing angular deltas, instead of treating
  the arbitrary 0.0 as a real sample. **Verified on `can_cone.stp`**:
  `_is_closed_by_winding` now returns `True`; the per-edge sweep trace
  confirms the two generatrix edges now correctly read sweep~0 (constant
  angle) and the two base-circle-arc edges read 0.5 turn each (summing
  to the correct full turn), matching the geometry exactly.
  **Isolated corpus-wide effect** (a dedicated script comparing the OLD
  vs NEW closure logic directly on every cone/cylinder face of every
  solid in `Solidos/test_models`, without running the full pipeline --
  much faster and cleanly decoupled from the user's own, unrelated
  `cell_definition.py` `comp.expand_regions_to_boolVar()`/`.simplify()`
  re-enable that was also live in the same working tree): only 4/138
  files have any face whose closure result changes --
  `Cans/fwd_can_0.stp`, `fwd_can_1.stp`, `rev_can_0.stp`,
  `rev_can_1.stp` (2/3 faces each). Of these, only `rev_can_0`/
  `rev_can_1` change the final WRITTEN MCNP text under
  `Options.meta_surfaces=True` (the Forward cases build the same final
  surfaces regardless, via a different branch).
  **Confirms this was a real, general, previously-hidden GEOUNED bug,
  not a `meta_surfaces=False`-specific limitation** -- directly
  motivated by the user's own hypothesis while reviewing a separate
  `meta_surfaces=False` corpus scan (see
  `docs/investigations/meta_surfaces_false_corpus_scan_2026-09-28.md`)
  that found `Cans/rev_can_1.stp` losing 10 particles ("no cell found in
  subroutine newcel") under `meta_surfaces=False`: **d1suned confirms
  the fix eliminates this entirely** -- 0 lost particles after (was 10),
  tally 0.99991 +/-0.15% (0.1 sigma), both individually and re-confirmed
  in a full 144-file `meta_surfaces=False` corpus re-run (only change
  from the pre-fix baseline: lost-particle count 1 file -> 0 files;
  every other bucket -- 89.3% within 2 sigma, 11 marginal, the other 8
  files beyond 3 sigma -- bit-identical, confirming this fix touches
  nothing else). Under `Options.meta_surfaces=True` (the default),
  `rev_can_1.stp`'s own d1suned tally is UNCHANGED by the fix (0.99991
  both before and after) -- this file's physical result already
  happened to come out correct there via an incidental alternate
  construction path, which is exactly why the bug went unnoticed under
  the default setting: a full 144-file `meta_surfaces=True` corpus
  re-run with the fix applied reproduces the pre-fix baseline bucket
  numbers bit-for-bit (93.1% within 2 sigma, same 11 marginal, same 2
  known `>3 sigma` files, 0 lost particles) -- no visible change at this
  scan's fidelity, consistent with the fix's benefit being real but
  invisible for every file in this particular corpus under the default
  setting.
  **The other 6 severe new-FAIL files from the same `meta_surfaces=False`
  scan are CONFIRMED unrelated to this bug** (`Mixed/multiplane_add_plane_cyl.stp`,
  `Mixed/double_RC.stp`, `RoundCorners/rrc23.stp`,
  `Complex_cell/modelcell_cut1_1.stp`, `RoundCorners/comp_RC.stp`,
  `Mixed/rev_pipe.stp`/`RoundCorners/rev_pipe.stp` -- the latter two
  confirmed byte-identical duplicate STEP files via `sha1sum`, so really
  6 distinct files, not 7): none appeared in the 4-file isolated-
  winding-effect list, and all 6 were independently verified CORRECT
  under `Options.meta_surfaces=True` via d1suned (e.g.
  `multiplane_add_plane_cyl.stp`'s catastrophic 0.00756 under
  `meta_surfaces=False` becomes 0.99198 +/-0.44% under `meta_surfaces=True`)
  -- these are genuinely explained by composite meta-surfaces
  (Can/TCone/RoundCorner/MultiPlane) being load-bearing for correct
  bounding in these specific geometries, not pure simplification (the
  same reasoning already established for RevCC's own exception in
  `Options.meta_surfaces`'s own entry below), and are NOT a hidden bug
  that also affects the default `meta_surfaces=True` setting. See
  `docs/investigations/meta_surfaces_false_corpus_scan_2026-09-28.md`
  for the full file-by-file writeup, raw d1suned numbers, and this
  conclusion's own before/after comparison table.
  **Verified**: full suites green on all 3 engines (ocp 342 passed/2
  skipped, occ 342 passed/2 skipped, freecad 310 passed/18 skipped --
  same numbers as the `gen_plane_cone` fix above, zero regressions from
  adding this fix on top).

- **`generic_split`: `BOPAlgo_Splitter` silently corrupting the base
  solid's own native shape across repeated "failed" candidate attempts,
  fixed 2026-09-28 -- root cause of `Mixed/multiplane_add_plane_cyl.stp`
  losing ~99% of its material under `Options.meta_surfaces=False`
  (d1suned tally 0.00756).** User-reported: "durante la decomposicion
  cuando tiene que cortar el plano px=0 debe haber un error y el solido
  no se corta" -- directly investigated per the user's own request to
  remove the loop's `try`/`except` and let any real exception surface,
  which showed NO exception was ever raised: `generic_split`'s
  candidate-surface loop tried 16 candidates on the solid's 780.39 mm^3
  fragment, candidate 9 (a real, correct cutting plane) produced 2
  pieces summing to only 503.97 (not 780.39), correctly rejected by the
  existing `SPLIT_CANDIDATE_VOLUME_REL_TOL` volume-conservation guard --
  but no OTHER candidate ever separated the fragment, leaving it
  permanently unsplit with only a log warning.
  **Root cause, confirmed by direct instrumentation**: `Gsplit`'s own
  `base` argument is passed by reference straight into
  `BOPAlgo_Splitter.AddArgument` -- since `generic_split`'s candidate
  loop tries this SAME base solid against up to a dozen+ different
  tool surfaces in sequence (discarding any that don't split it), and
  OCCT's own BOP algorithm can corrupt a shape's own BRep state as a
  side effect of merely ATTEMPTING an intersection (even one that
  ultimately fails to split anything, still 1 output piece), this
  corruption is cumulative and order-dependent across the WHOLE search,
  not scoped to one failed attempt. Re-running candidate 9 in ISOLATION
  (no prior candidates tried) gave the CORRECT 3-piece split (203.97 +
  300.0 + 276.42 = 780.39 exactly) -- proving the candidate itself was
  always right, only the corrupted base made it fail later in sequence.
  A tolerance-only reset (new `Gsolid_set_tolerance`, occ/ocp, via
  `ShapeFix_ShapeTolerance`, called before every new candidate) was
  tried FIRST and confirmed INSUFFICIENT by direct instrumentation:
  candidate 4 (a Plane) raised the base's own max BRep tolerance 5x
  (1.2e-7 -> 6.0e-7) even though it produced only 1 piece (no real
  split); resetting the tolerance back before every subsequent candidate
  kept it pinned at 1.2e-7 throughout, yet candidate 9 STILL misbehaved
  identically -- and a further check (face/edge/vertex counts, exact
  vertex-position fingerprints) found NONE of the base's own geometry
  had changed either, only its `is_valid()` (`BRepCheck_Analyzer`) flag,
  which flipped `True` -> `False` across the same sequence of "failed"
  attempts: BOPAlgo had corrupted the shape's own internal
  parametrization consistency (pcurve/`SameParameter` state), not its
  tolerance or geometry. **Fix**: reset the tolerance AND re-run the
  same `.fix()` validity repair `generic_split` already used once at
  its own top, before EVERY new candidate is tried, not just once.
  Verified this combination (not tolerance alone) restores the correct
  3-piece split on the exact same fragment. freecad has no
  `Gsolid_set_tolerance` (occ/ocp only, guarded by `CAD_ENGINE !=
  "freecad"`); its own `.fix()` re-run still applies there since it's
  engine-agnostic.
  **A second, related bug found investigating the residual gap this
  fix's own d1suned check left behind** (tally improved to 0.98149 but
  the written model's own `SD4` reference volume, 794.83, still didn't
  match the true CAD volume, 786.42 -- ~1.1% off; the user asked
  directly for the true CAD volume and the written SD4 to be compared,
  then pointed out d1suned's own tally at NPS 4e6 -- 0.9873 relative to
  SD4 -- put the REAL, MCNP-traced volume at ~784.75, much closer to
  the true CAD volume than to SD4, meaning the WRITTEN BOOLEAN
  DEFINITION was correct and the bug was specifically in how SD4 itself
  gets computed): `GeounedSolid.update_solids()` (`utils/
  geouned_classes.py`) computed the new piece list's own total volume
  into a local `vol` variable but never assigned it to `self.Volume` --
  a pure oversight, present since this method was written. Separately,
  `core.py::_decompose_target` (both the cache-hit and the normal-
  decomposition call sites) called `m.set_cad_solid()` (which
  recomputes `CADSolid`/`Volume`/`BoundBox` from `self.Solids`) BEFORE
  `m.update_solids(...)` (which installs the FRESH, correctly-decomposed
  piece list) -- so `set_cad_solid()` always ran against the OLD,
  pre-decomposition `self.Solids` reference, which (per the bug above)
  had just been corrupted in place by `main_split`'s own candidate
  search (`Gmake_compound` wraps a solid's native shape by reference,
  never copies it, so `main_split(Gmake_compound(m.Solids), ...)`
  operates on -- and can corrupt -- the exact same shared native object
  `m.Solids[0]` still points to). This is NOT scoped to a volume-display
  cosmetic issue: `self.CADSolid` itself (the real 3D shape other
  pipeline stages read directly, not just derived scalars) could carry
  this same corrupted, stale geometry into `build_void`/`cell_definition`
  (both read `m.CADSolid`) for ANY solid whose own decomposition search
  ever hit this corruption, not only ones where it happened to show up
  as a visibly wrong SD4 number.
  **Fix**: `update_solids()` now also sets `self.Volume`; both call
  sites in `core.py` reordered to call `update_solids()` (install the
  fresh pieces) BEFORE `set_cad_solid()` (recompute CADSolid/Volume/
  BoundBox from them), so every downstream reader always sees geometry
  derived from the correct, final decomposed pieces, never the stale
  pre-decomposition original.
  **Verified together**: `multiplane_add_plane_cyl.stp`'s own d1suned
  check now shows `SD4` exactly matching the true CAD volume (786.4194
  both), tally 0.99200 +/-0.44% (1.8 sigma, ordinary MC noise) --
  0.00756 -> 0.98149 (fix 1 alone, wrong SD4 still) -> 0.99200 with
  exact SD4 (fix 2 added). Full suites green on all 3 engines after
  BOTH fixes (ocp 342 passed/2 skipped, occ 342 passed/2 skipped,
  freecad 310 passed/18 skipped) -- one single access-violation crash
  and one single `PermissionError` were each seen exactly once across
  several repeated runs and did NOT reproduce on immediate retry
  (isolated re-run of the specific failing test passed clean; the
  `PermissionError` was traced to `decompose_cache/tmp/` file-lock
  contention from running all 3 engines' suites concurrently against
  the same repo checkout, not a real regression -- freecad alone, after
  clearing the stale cache dir, passed clean). A 144-file isolated
  corpus differential (this commit vs the immediately preceding one, own
  MCNP-text comparison under the default `Options.meta_surfaces=True`)
  found 0 DIFF, 136 SAME, and the same 8 known pre-existing CRASH files
  (unchanged) -- this pair of fixes is inert at the default setting for
  every file in `Solidos/test_models`, exactly as expected (the
  corruption mechanism needs a genuinely failed-then-later-succeeding
  candidate sequence, which the default `meta_surfaces=True` order
  apparently never triggers for any file in this corpus).
  **Full `meta_surfaces=False` corpus re-run, 2026-09-28**: besides the
  targeted `multiplane_add_plane_cyl.stp` fix (moved out of the >3 sigma
  bucket entirely), 2 files improved as a pure SIDE EFFECT, never
  specifically targeted: `Complex_cell/modelcell_cut1_1.stp` (was
  0.95581/16.51 sigma) and `Mixed/rev_pipe.stp` (was 0.99243/4.01
  sigma, along with its confirmed-duplicate `RoundCorners/rev_pipe.stp`)
  both moved to within 2 sigma. Overall bucket: 89.3% -> **91.4%**
  within 2 sigma, beyond-3-sigma files 9 -> 5, 0 lost particles
  throughout. `RoundCorners/rrc23.stp` and `RoundCorners/comp_RC.stp`
  (both cells) are essentially unchanged (bit-identical or MC-noise-
  level). **`Mixed/double_RC.stp` got WORSE**: 1.24915 (76.7 sigma) in
  the original baseline -> 1.66076 (165.8 sigma) now -- **not yet
  investigated**: no intermediate data point exists between the already-
  committed `_is_closed_by_winding` fix and this pair of fixes to
  attribute which change caused it, or whether it's a real regression
  from today's work at all rather than something the winding fix itself
  already introduced under `meta_surfaces=False` (never re-scanned in
  that specific configuration until now). Flagged for follow-up, not
  blocking these fixes' own commit per direct user instruction.

- **`GCompound`: native `TopoDS_Compound` aggregation of touching solids
  is unreliable in OCCT -- fixed by replacing it with a pure-Python
  container, 2026-09-29. Root-causes and closes the `double_RC.stp`
  regression flagged above, plus `RoundCorners/rrc3.stp` (a NEW
  regression introduced mid-investigation, also closed by this same
  fix) and a real, pre-existing bug in `RoundCorners/rrc23.stp` the
  user found by direct comparison against an independent reference
  solid.** User-reported: "la definicion booleana de
  `working_solids/bara.stp`, y de la tercer componente (despues de
  descomposicion) de `RoundCorners/rrc23.stp` tienen que ser iguales
  porque son los mismos solidos. Sin embargo, en rrc23 la expresion
  booleana no es correcta" -- `bara.stp` is a hand-extracted reference
  solid matching rrc23's own 3rd decomposed piece exactly (same
  geometry). Confirmed: re-exporting `main_split`'s own `comsolid` to
  STEP (the `Settings.debug=True` dump the user was comparing against)
  and reloading it gave that piece the CORRECT volume (235.432606,
  bit-identical to `bara.stp`) and a 7-term boolean definition matching
  `bara.stp`'s own -- but the piece's OWN in-memory volume, immediately
  after `main_split` returns (before any STEP round-trip), was
  319.208410 with only a 6-term definition (missing one bounding
  surface) -- an 83.775804 mm^3 discrepancy, and the true CAD volume
  (via an independent `Gload_step` of `rrc23.stp` itself) confirmed
  235.432606 was correct, 319.208410 was not.
  **Root cause, traced through several wrong hypotheses before finding
  it**: `Gmake_compound` (a bare `TopoDS_Compound` wrap, `builder.Add`
  per shape, no geometric operation) followed by re-exploring it via
  `.Solids` (a fresh `TopExp_Explorer` traversal) is NOT a lossless
  round-trip when the wrapped solids genuinely touch each other (as any
  set of adjacent decomposed pieces always does) -- `generic_split`'s
  own returned pieces are reliably correct (confirmed via direct,
  repeated queries -- individual `.Volume` never lied), but
  `split_surfaces`'s own `comp = Gmake_compound(solid_components)`
  followed later by any consumer reading `comp.Solids`/`comp.Volume`
  could silently misattribute volume between touching neighbors (the
  ~84 mm^3 moved from one piece to another, the TOTAL staying
  conserved) or inflate/deflate the aggregate total outright. Confirmed
  on BOTH `occ` and `ocp` (ruling out a pybind11-vs-SWIG binding bug --
  this is genuine OCCT kernel behavior) and confirmed non-deterministic
  in a way that even defeated a first attempted fix: a tolerance-only
  reset (mirroring the earlier `generic_split` fix above) was tried,
  then a `.fix()`-every-extracted-piece approach with a same-volume
  verification-and-fallback safety net -- both APPEARED to fix
  `rrc23.stp`/`RoundCorners/comp_RC.stp` (whose first solid is the
  literal same geometry as `rrc23.stp`, per direct user confirmation,
  and was fixed by the exact same mechanism) when tested in isolation,
  but running the full corpus surfaced two NEW problems from that same
  approach: (a) `RoundCorners/rrc3.stp` -- previously fine -- started
  showing a genuine cross-piece volume-contamination (18 sigma d1suned
  deviation), confirmed via direct instrumentation to be the SAME
  wrap+re-extract+fix sequence, run standalone, non-deterministically
  giving correct results one time and corrupted results another,
  identically on both engines; (b) `Complex_cell/SCDR_90.stp` -- a
  previously-successful file -- started crashing outright
  (`Standard_Failure: "Courbes non jointives"`, the same
  `ShapeUpgrade_UnifySameDomain` crash class already documented as an
  accepted permanent limitation for `Mixed/ConeSphere.stp`) once
  `.fix()` was applied unconditionally to a piece that never needed it.
  **The real fix, per direct user design instruction**: GEOUNED never
  actually needs a *real* native compound for this bookkeeping at all
  -- `main_split`'s own recursively-collected pieces are just a group
  of independent, irreducible solids to track together (for the cell
  definition, the void generator, cache storage, a debug STEP dump,
  ...), never a single fused/merged shape. New `GCompound` class
  (`geo/solid_ops.py`, exported via `geo/__init__.py` -- the user's own
  explicit placement choice, "la class Gcompound tiene que ir en geo",
  since it's a generic, reusable container concept, not something
  specific to one call site) holds a plain Python list of `GSolid` and
  computes `.Volume`/`.BoundBox` as a sum/union over each piece's own
  individually-queried (always reliable) properties, never through a
  native aggregate operation. `.export_step(filename)` is the one
  remaining native escape hatch, per the user's own explicit request
  ("agregale un metodo de export_step si queremos ver el solido. Este
  metodo junta todos las parte en un step sin juntarlas"): it builds a
  fresh, one-shot `TopoDS_Compound` purely to serialize the pieces to
  disk side by side (never fusing them into each other), and that
  compound is never re-extracted or reused afterward, so it is never
  exposed to the same corruption.
  `decompose/decom_one_generators.py::split_surfaces`/`main_split`
  rewritten to use `GCompound` throughout instead of the wrap-then-
  reextract-then-fix dance: `split_surfaces` now fixes each of
  `generic_split`'s own returned pieces directly (`.fix()` wrapped in
  its own `try`/`except`, keeping the original piece on failure -- the
  same crash-safety the earlier, now-superseded approach already had),
  builds a `GCompound` from them directly (no intermediate native wrap
  at all for the pieces actually returned), and its own "did a fragment
  fail to resolve to a real solid" diagnostic (a pre-existing,
  independent concern about `_raw_bop_split`'s own repair cascade
  leaving a bare Shell/Compound instead of a genuine `TopAbs_SOLID`,
  unrelated to the volume-corruption bug) now probes each fixed piece
  in its OWN single-piece native wrap, one at a time -- never bundling
  several touching pieces into one probe compound, closing off even
  this diagnostic-only check from the same corruption class (flagged
  directly by the user reviewing the fix: "no habria que quitar
  [Gmake_compound] tambien?" -- `Gmake_compound` itself, the raw
  kernel-wrap primitive, was never the bug and is still used for every
  genuinely one-shot native need; only the "wrap many touching
  solids, then trust the aggregate or re-extract" pattern was unsafe).
  `main_split` now takes a plain list of `GSolid` (never a native
  compound) and returns a `GCompound`; its one real call site
  (`core.py::_decompose_target`) simplified to pass `m.Solids` directly
  (no `Gmake_compound` wrap needed at all for the input side either).
  `GeounedSolid.set_cad_solid()` (`utils/geouned_classes.py`) still
  builds a real native compound for `self.CADSolid` (downstream void
  generation / enclosure containment genuinely need real native
  geometry for boolean CSG operations) but now computes `.Volume`/
  `.BoundBox` via the same safe sum/union pattern as `update_solids`,
  never from the compound's own aggregate properties.
  **Verified**: `rrc3.stp` gives the correct volume (882.861421,
  matching an independent `Gload_step`) consistently across repeated
  runs (previously non-deterministic); `rrc23.stp`'s 3rd piece still
  matches `bara.stp` exactly (235.432606, 7-term definition);
  `comp_RC.stp`'s both solids match their true CAD volumes exactly
  (1014.567394, 1235.432606); `SCDR_90.stp` no longer crashes. d1suned
  confirms all three real fixtures: `rrc23.stp` 0.91938 (21.4 sigma) ->
  0.99752 (0.6 sigma); `comp_RC.stp` both cells 0.98084/0.94469 (4.0/
  15.4 sigma) -> 0.99402/0.99791 (1.25/0.55 sigma); full suites green
  on all 3 engines (ocp 342 passed/2 skipped, occ 342 passed/2 skipped,
  freecad 310 passed/18 skipped) both before and after the final
  per-piece-probe safety refinement. A 144-file isolated corpus
  differential (vs the immediately preceding commit) found 27 genuine-
  improvement DIFF files, 108 SAME, and the same known ~8 pre-existing
  CRASH files with neither `SCDR_90.stp` nor `rrc3.stp` among them.
  **Full corpus re-run, both `Options.meta_surfaces` settings, timed**:
  `meta_surfaces=True` 93.1% within 2 sigma (unchanged throughout this
  whole investigation -- this whole bug class needs a genuinely failed-
  then-later-succeeding candidate sequence that the default order
  apparently never triggers in this corpus); `meta_surfaces=False`
  improved to 93.6% within 2 sigma, with `Mixed/double_RC.stp` (the
  regression flagged in the previous entry, now explained: same root
  cause) and `RoundCorners/rrc3.stp` both gone from the failing bucket
  -- leaving exactly ONE beyond-3-sigma cell in EITHER mode, the same
  already-understood `RoundCorners/shed_solid.stp` MC-noise case. A
  188-vs-187 total-cell-count discrepancy between the two modes' own
  analysis runs was tracked down (per direct user question) to a
  19-day-old stale `model.mcnp`/`outp` leftover sitting in the
  `meta_surfaces=True` run directory for `Mixed/SCDR_90_hollow.stp`
  (which fails to convert under EITHER setting, a known load-time
  crash) -- not a real difference; deleted, and both modes' own
  analyses then agreed exactly (187 cells, one shared failing case).
  d1suned simulation time for the full 142-file corpus: ~1000s per
  batch (~2000s combined), consistent across repeated timed runs.
  0 lost particles throughout every check in this whole investigation.

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
- **`get_surfaces` now tries cylinder and cone cutting surfaces before
  planes, and `arc_extent`'s wrap-nesting bug fixed, 2026-09-18**
  (commits `0b8e2e1` and `54eefa5`). Per direct user request, the order
  of the loops in `decompose/generators.py::get_surfaces` (previously
  planes -> cylinders -> cones) was compared against
  cylinders -> cones -> planes by counting *irreducible solids* -- the
  leaves of `generic_split`'s recursive decomposition, i.e. fragments no
  candidate surface can split any further -- per ORIGINAL solid (a
  wrapper around `decom_one_generators.split_surfaces` records one entry
  per solid `main_split` hands it, so a STEP with N solids counts each
  separately). All 143 non-`Big_*` files of `Solidos/test_models`, `ocp`,
  one fresh subprocess per file (isolates the known native crash of
  `Mixed/ConeSphere.stp`), files of the same folder sequential and
  folders in parallel, plus a repeat of the baseline as a determinism
  control (0 differences). Result over the 176 original solids that
  decompose: **346 -> 325 irreducible solids (-6.1%)**; 172 solids
  unchanged, 4 improve (`Mixed/ring` 32 -> 20, `Mixed/sleeve` 28 -> 21,
  `Cans/RevTcan` 6 -> 5, `Cans/Tcan` 3 -> 2), none gets worse. 4 files do
  not decompose under either order: `SCDR_90_piece2`,
  `modelcell_cut1_v2_piece66`, `SCDR_90_hollow` (`SystemExit` at load:
  `corrupted_solids="stop"` -- `check_solid_defects` reports
  "degenerate/sliver geometry", a face with `CharacteristicWidth` below
  `min_face_width` on a pathologically short edge; topology is valid, the
  faces are plane/cylinder/cone only and `Gspline_surface` is False, so
  neither splines nor unsupported surfaces are involved) and
  `ConeSphere` (the native crash). The permanent order was checked to
  reproduce the experiment exactly (325, 0 per-solid differences).
  **A real, independent bug surfaced by the new order**:
  `Mixed/sleeve.stp` raised `ValueError: arc_extent: pairs split into 3
  disconnected groups, not a single arc` under it (from `next_Can` ->
  `closed_cylinder_cone` -> `merge_same_surface_faces` -> `ShellFaceGu.
  _U_parameter_faces`, an exception `generic_split` does not catch, so
  the whole solid's decomposition aborted). `geo/vector_geometry.py::
  arc_extent` sweeps the U intervals in a frame cut at 0/2*pi but never
  compared a pair running ACROSS that cut with the pairs lying under its
  wrapped tail: here 7 faces of one R=115 cylinder in two axial bands,
  one face starting exactly on the cut, whose wrapped tail covers
  everything else -- extra groups, hence the raise. It also had a
  SILENT failure mode: with a single nested group the arc was returned
  truncated at that group's end (about 1 in 20 random single-arc
  configurations returned a wrong endpoint, about 1 in 8 raised; never
  observed in the real corpus, where the order-A results are unchanged).
  Fixed by keeping the sweep exactly as it was and absorbing, in order,
  every group that starts inside the wrapped tail (each may extend it
  further); within `tol` the group's own end wins, so the returned
  values AND face indices (consumed literally as `Faces[ifacemin]`/
  `Faces[ifacemax]` by `extreme_edge`/`get_shell_UV_nodes`) are
  identical to the previous ones wherever those were correct: 0
  differences over 900,000 random single-arc configurations, and the
  order-A decomposition of the whole corpus unchanged (176 solids, 0
  count changes, 0 status changes). Where the previous code raised or
  was wrong, the new one matches ground truth up to `tol` (1e-5).
  `tests/geo/test_vector_geometry.py` gained 7 tests (the real `sleeve`
  pairs, the silent-truncation case, real gaps still raising, a seeded
  randomized check against ground truth); the three targeted ones fail
  against the previous implementation.
  **Verified**: d1suned (NPS 1e6, the pipeline's standard) on the 4
  changed solids under both orders: all 8 tallies within 2 sigma of 1.0,
  0 lost particles, no "fatal error", SD4 = true CAD volume. The 0.38%
  gap between orders in `sleeve` (1.00261 vs 0.99877) is purely
  statistical (1.0 sigma of the difference of two independent estimates,
  0.27%*sqrt(2); MCNP's random-number consumption depends on how cells
  are subdivided, so identical histories cannot be assumed across
  decompositions -- an initial reading of it as a geometric difference
  was wrong, corrected by the user). Exact CAD-side check, no MCNP: the
  irreducible pieces' volumes sum to the original solid's volume within
  1.4e-9 relative in all 8 cases. Full suites green on all 3 engines
  after each commit (freecad 186 passed/16 skipped, occ 220 passed/1
  skipped, ocp 220 passed/1 skipped).
  **Not verified**: a full-corpus d1suned run with the new order -- the
  172 solids whose irreducible COUNT is unchanged could still have
  decomposed differently. Also note: `RevTcan` under the new order has a
  12.6 mm^3 piece, smaller than any piece of the old order (its d1suned
  tally is identical to the old one's). The counting/verification
  scripts were throwaway (session scratchpad), not preserved in the
  repo.

- **`forward_round_corner_region`: `OR_bracket` polarity inverted in two
  branches, fixed 2026-09-25** (`basic_functions_part1.py`).
  `RoundCorners/shed_part.stp` lost particles (d1suned 3.89, 28.9 sigma)
  and `shed_solid.stp` too (0.893 +/- 23.6 %, 10 lost). Cause: both RC
  planes DO point to the material (checked point by point) and
  `cyl_plane_region_conf`'s `OR_p12_bracket` (`z1 . (p1_axis x p2_axis) <
  0`) is consistent; the two Forward branches `not p1_cyl and p2_cyl`
  (`not p1_pd and p2_pd`) and `p1_cyl and not p2_cyl` (`p1_pd and not
  p2_pd`) used the opposite polarity (`if not OR_bracket` -> `if
  OR_bracket`). Found by brute force against the real material of
  synthetic corners (prism with one arc whose ends lie on/behind the
  planes' crossing, 240 generated, 74 where the bit matters): Reversed 45/45
  and other Forward branches 3/3 already right, these two branches 17/17
  wrong the other way round; only `shed_part` in the real corpus reaches
  them. `reversed_round_corner_region` untouched. **Verified**: ocp/occ 244
  passed, freecad 258 passed; test_models MCNP text before/after: 140 of
  142 identical, only `shed_part`/`shed_solid` change; d1suned on the 47
  `RoundCorners`: `shed_part` 1.0048, no lost particles, `shed_solid`
  0.948 +/- 1.8 % at NPS 1e6 turned out to be noise of a 0.249 cm^3 cell
  (independent MC of its region: 0.24938 +/- 0.0002 vs CAD 0.249106; 0.981
  +/- 0.9 % at NPS 4e6). **Not verified**: the tangent-arc case in those two
  branches has no real fixture.

- **`get_join_cone_cyl`'s `closed_set` + `convex_planes`'s `len<3` branch,
  fixed 2026-09-29** -- root cause of lost particles on
  `Big_model_reserved/divertor_cam.stp`'s `STRUCTURAL_PLATE_1#BYZ2PQ_95`
  solid. User-reported: a real R=2mm cylindrical feature in this solid
  is split into two half-cylinders whose real axes are offset by
  ~0.11mm perpendicular to the axis (a genuine defect already present
  in the SOURCE STEP file, confirmed by loading the solid directly, with
  no decomposition at all -- both R=2 faces already carry this exact
  offset there; NOT introduced by GEOUNED's own split/repair cascade).
  User's own first diagnosis: GEOUNED treats the two half-cylinders as
  distinct surfaces (correctly -- their axes really don't coincide) and
  builds two RevCC entries sharing the same additional closing plane
  with opposite sign, `(c1 p1)(c2 -p1)` -> an unconditional `False`.
  **Decomposition itself confirmed correct first** (per direct user
  request, before touching any boolean-definition code): cutting the
  original 6-face solid with either R=2 cylinder alone, as an infinite
  Reversed tool, is a 100.0000%-volume-conserving no-op for BOTH
  cylinders individually; the real decomposition into 2 pieces
  (356.643666 + 0.443686 mm^3, summing back to the original 357.087352
  to float precision) is a genuine, separate, volume-exact split -- the
  0.443686mm^3 piece is the real physical wedge that exists between the
  two slightly-misaligned axes, not a split artifact. So the bug is
  entirely downstream, in the boolean CSG definition.
  **Bug 1, `get_join_cone_cyl`'s own `closed_set`** (`meta_surfaces_utils.py`):
  the recursive RevCC chain walk (`face0` -> adjacent shell `[3,4]`, the
  two half-cylinders' own merged partner) DOES correctly find the two
  segments border each other at BOTH ends (a genuine, degenerate
  2-member closed ring) -- but `omitFaces` (shared across the WHOLE
  solid's meta-surface detection, not just this one chain) already
  marked `[3,4]` consumed after the FIRST junction recursed into it, so
  the SECOND junction's own `adjacent2.Index not in omitFaces` check
  silently failed and that closure was never recorded. Summing each
  segment's own local arc (measured around its own, not-quite-coincident
  axis) then falls 6.36 degrees short of a full turn (353.64 vs 360,
  confirmed via direct numeric trace) even though the loop genuinely
  closes -- `twoPimod`'s own noise tolerance (1e-5 rad) is nowhere near
  enough to absorb a real few-degree parametrization discrepancy like
  this. **Fix, v1 (later narrowed, see below)**: a `wraps` flag,
  set when either `adjacent1`/`adjacent2` points to a face already
  visited anywhere in the SAME top-level walk; `closed_set = twoPimod(
  arc_angle) == 0.0 or wraps[0]`. This alone caused a REAL regression on
  3 previously-passing corpus fixtures (`test_cadtocsg.py`'s
  `input_step_file27/44/45`, i.e. `DoubleCylinder/placa3.step` and 2
  others): its own 3 RevCC chains (root -> child -> grandchild, each 3
  cylcones) have a grandchild whose OTHER end happens to also border the
  ROOT again -- a real topological adjacency, but NOT a closed loop
  (raw arc sums 13-56 degrees short of 360, confirmed measured, vs.
  divertor_cam's genuine 6.36-degree parametrization noise) -- so a bare
  "already visited anywhere in this walk" signal is not a reliable
  closure test; it also fires for a long-range, unrelated revisit.
  **Fix, final**: narrowed to a strictly LOCAL pattern -- `wraps[0]` is
  only set when adjacent2 resolves to the exact same shell adjacent1
  JUST recursed into (or vice versa is structurally impossible since
  adjacent1 is always checked first) -- i.e. THIS node's own two ends
  both meet the SAME single neighbor, the only shape a genuine
  degenerate 2-member ring can take. No `visited` set needed any more,
  just comparing `adjacent2.Index` against `adjacent1`'s own just-merged
  shell indices.
  **Bug 2, `convex_planes`'s `len(plane_list) < 3` early return**
  (`geo/surface_geometry.py`, shared by all 3 engines): ignored its own
  `closed` parameter entirely, deriving `orientation` from a
  position/axis heuristic meant for the OPEN-chain case only. For our
  2-cylcone closed ring, the two segments' own "additional closing
  planes" (`gen_plane_cylinder`) are forced to be the exact SAME
  physical plane seen from opposite senses (axis dot = -1.0 exactly,
  confirmed) -- any real 2-member closed ring has no choice but this,
  since each segment's own local closing plane is the other segment's
  own boundary. The heuristic returned `orientation="Forward"`, so
  `add_reversedCC` computed `plane_region = mult(p1, p2)` = `p1 AND -p1`
  = an unconditional `False`. **Considered and REJECTED per direct user
  correction**: skipping the plane combination entirely whenever
  `closed_set=True` (any number of cylcones) -- wrong in general, per
  the user's own counterexample: 3 coaxial, tangent cylinders (R1<R2<R3)
  define `c1*c2*c3` as two regions that DON'T touch (an inner tube and
  an outer shell); telling them apart still needs the additional planes
  even though the set is topologically closed. **Fix**: only for
  `len(plane_list) < 3` (the 2-cylcone case, where "closed" can ONLY
  mean the degenerate same-plane-both-senses configuration above) does
  `closed=True` force `orientation="Reversed"` (`add_reversedCC`'s `add`/
  OR branch instead of `mult`/AND), which correctly reduces `p + (-p)`
  to an unconditional `True` (no restriction -- exactly right, since a
  closed 2-member ring has no exposed side left for either plane to
  bound). The >=3-plane branch (the real angle-sorting algorithm) is
  untouched, still receiving and using `closed` as before.
  **Verified**: `divertor_cam.stp`'s isolated fragment -- both pieces'
  boolean definitions are now non-contradictory
  (`AND[2 3 4 6 -1]`/`AND[3 5 6 7 -4]`); d1suned (NPS 1e6): tally
  0.99275 +/-0.57% (1.27 sigma), SD4 exactly matches the true CAD
  volume (357.0874 mm^3), 0 lost particles (was 3.89/28.9 sigma/10 lost
  before any fix). Full suites green on all 3 engines after the
  narrowed fix (occ 277 passed/1 skipped, ocp 277 passed/1 skipped,
  freecad 288 passed/16 skipped -- including the 3 fixtures the v1 fix
  had broken). A 143-file `Solidos/test_models` differential (full
  MCNP-text comparison, current commit vs the fix) -- **0 differences**
  anywhere (the 4 known pre-existing conversion failures unaffected) --
  confirming both fixes are inert on the whole existing corpus and only
  change the one fragment that motivated them.
  **Follow-up, same day: full corpus re-run under BOTH
  `Options.meta_surfaces` settings** (144 files, excluding `Big_*`, 8-way
  parallel conversion + 16-way parallel d1suned, both settings run
  concurrently) -- the gap between `meta_surfaces=True` and `=False`
  that the 2026-09-28 investigation (`docs/investigations/
  meta_surfaces_false_corpus_scan_2026-09-28.md`) had recorded (93.1%
  vs 89.3% within 2 sigma, 2 vs 9 files beyond 3 sigma, 0 vs 1 file with
  lost particles) is now **completely closed**: both settings give
  IDENTICAL results -- 190 tallies, 178 (93.7%) within 2 sigma, 11
  marginal (same files, same sigma to within rounding -- ordinary MC
  noise), exactly 1 beyond 3 sigma in EACH (`RoundCorners/shed_solid.stp`,
  the same already-explained MC-noise-on-a-0.25cm^3-cell case), 0 lost
  particles in either. Conversion: 144/148 in both, same 4 pre-existing
  failures as always. Consistent with every fix landed since that
  earlier scan (`_is_closed_by_winding`, `GCompound`, `gen_plane_cone`,
  and this entry's own two fixes) having closed the real gaps that used
  to separate the two settings' behavior -- no code change made in this
  follow-up, purely a re-verification.
  **Follow-up, same day: a THIRD, distinct bug in the same closure
  logic, found by direct user report** -- `closed_set` still came out
  `False` for the exact same `STRUCTURAL_PLATE_1` solid when run under
  the user's own real settings (`Options.meta_surfaces=False`,
  `Settings.load_from_cache=True`, via the real `decompose_cache` at
  the repo root, `Test RoundCorners/myrun.py`) -- `Big_model_reserved`
  is excluded from the 144-file corpus scan above, so this specific
  file was never exercised by that re-verification. Under
  `meta_surfaces=False` this same solid decomposes differently: the
  "good" axis's own faces fragment into 4 raw pieces instead of 2, two
  of them narrow enough to count as slivers by `CharacteristicWidth`.
  `face0` (the defective-axis cylinder) turns out to have **7** boundary
  edges here, not the 2 (`Umin`/`Umax`) `get_join_cone_cyl`'s own simple
  model looks at -- it borders the good-axis shell directly through 3 of
  those extra edges. Root cause, traced down to a genuine, general bug
  in `geometry_gu.py::other_face_edge`'s own `skip_slivers=True` walk
  (used by many callers, not just this one): when it walks PAST a chain
  of thin sliver faces looking for "the real face beyond them", it never
  excludes the walk's OWN starting face from being accepted as that
  "real" result -- confirmed live by a direct recursion trace: from
  `face0`'s own `emax` edge, the walk passes through 2 slivers (both
  genuine fragments of the good axis) and its very last hop lands back
  on `face0` itself (reached through a completely different edge), which
  passes the "big enough" check trivially (it's the original, large
  face) and gets returned as `adjacent2` -- useless for closure
  detection, and exactly why `wraps` (the fix earlier in this same day's
  entry) never fired: `adjacent2.Index` was `face0`'s own index, not a
  member of the good-axis shell at all.
  **Fix**: rather than patch `other_face_edge`'s own general walk (used
  by several other callers, and demonstrably fragile to the exact
  iteration order of a sliver's own edge list -- tried and rejected
  after tracing that naively rejecting the self-match there can just as
  easily wander into an unrelated PLANE neighbor first, depending on
  edge order), added a second, independent, LOCAL closure signal in
  `get_join_cone_cyl` itself: a new `_touches_face_set(face_or_shell,
  target_indices, GUFaces)` helper counts how many of `face_or_shell`'s
  OWN boundary edges (all of them, not just `emin`/`emax`) are also an
  edge of some face already known to belong to `adjacent1`'s own merged
  target shell -- a plain, direct edge `is_same` check, no sliver-skip
  walk involved at all. `>= 2` (found via more than just the one edge
  that already gave us `adjacent1`) is treated the same way the existing
  `wraps` signal is -- the same "both ends meet the same neighbor"
  pattern, just discovered without relying on `emin`/`emax` being the
  only 2 relevant edges of a real face's own boundary. A genuinely open
  chain only ever touches its own child shell through the one edge that
  found it in the first place (count of 1), so this doesn't risk a
  false positive on that case.
  **Verified**: the real case (`myrun.py`'s own settings, the real
  cache) now gives `closed_set=True` and no longer raises
  `build_definition`'s own mismatched-element-count `RuntimeError`; the
  earlier `meta_surfaces=True` fix and `placa3.step` (the v1-fix
  regression) both re-confirmed unaffected; full suites green on all 3
  engines (occ 277 passed/1 skipped, ocp 277 passed/1 skipped, freecad
  288 passed/16 skipped); a 144-file `Solidos/test_models` differential
  (full MCNP-text comparison) -- **0 differences** anywhere (same 4
  known pre-existing failures). A real d1suned check on this exact real
  solid, under the user's own `meta_surfaces=False` settings, was
  started by the user directly (not yet reported back as of this
  commit).

- **Decomposition cache (`Settings.load_from_cache`), implemented
  2026-09-27** -- new feature, not a bug fix: `decompose_solids()` (via
  `main_split`/`generic_split`/`Gsplit`, `decompose/
  decom_one_generators.py`) is the CPU-expensive phase of the pipeline
  (real recursive CAD boolean cuts, per solid); the following phase,
  `build_solid_definition()` (via `build_definition`/
  `simple_solid_definition`, `conversion/cell_definition.py`), is cheap
  by comparison but registers every surface into the model-wide,
  deduplicated `Surfaces` registry (`MetaSurfacesDict`) that assigns
  final surface numbering. Motivating use case: the user edits a STEP
  model incrementally (some solids' shape changes, some get added or
  removed) and wants to skip redecomposing everything that didn't
  change. Went through several rounds of design with the user before
  landing on the final shape below (started as `Settings.rerun`, a
  simple opt-in read/write flag with per-solid STEP files -- see the
  earlier drafts of this entry in git history/the
  `project_decomposition_rerun_cache_design` memory for the road not
  taken and why).
  **Core design, unchanged since the first draft**:
  - Only phase 1's OUTPUT (a solid's own decomposed convex pieces) is
    ever cached -- phase 2 always reruns, for every solid, in the same
    order as today, which is what keeps global surface numbering
    deterministic and is what makes this feature provably unable to
    change any written output, only how long a run takes.
  - Identity key = the solid's own raw STEP label (`GLabelNode.label`,
    read via the existing `Gload_step_labels()`/`load_cad()` loop),
    **not** its position in `meta_list` -- list position breaks the
    moment a solid is added/removed, and can independently drift at load
    time too (`corrupted_solids="remove"`, spline handling,
    `skip_solids`). The **raw**, untrimmed label is used (before
    `LF.get_label()`'s trailing-number trim), so array copies like
    "Bolt 001"/"Bolt 002" stay distinct.
  - **Change detection is user-driven, not automatic** -- deliberately
    NOT a geometric fingerprint (a fingerprint design was fully worked
    out first, then explicitly dropped by the user in favor of
    simplicity: "menos robusta... pero mas facil de implementar").
    Instead, the user adds the literal marker `__modified__` anywhere in
    a solid's STEP label before re-exporting (e.g. `"Bolt 001"` ->
    `"Bolt 001__modified__"`); GEOUNED detects it at load time, forces
    that solid to be redecomposed, and strips the marker back out before
    using the label as the cache identity key. **Explicitly accepted
    trade-off**: a solid edited without the marker silently reuses its
    stale cached decomposition -- a lightweight "warn if volume/face-
    count differ unexpectedly" safety net was offered and explicitly
    declined by the user in favor of the purely marker-driven mechanism.
  - **Safety rule**: any label that is not unique this run (duplicated
    in the current model, or in the loaded cache) is excluded from the
    caching mechanism entirely for every solid sharing it -- always
    redecomposed, never read from or written to under that label. Worst
    case is losing the speed benefit for that solid; identity can never
    be mismatched.
  - Global invalidation: the whole cache is discarded (every solid
    redecomposed, cache rebuilt from scratch) if GEOUNED version /
    `CAD_ENGINE` / `kernel_version()` / any decomposition-relevant
    `Tolerances` field (`split_tolerance`, `fix_tolerance`,
    `volume_tolerance`, `scale`, `scale_up_floor`, `min_solid_volume`) /
    `Options.cut_large_cell` differs from what's stored -- a code or
    tolerance change can make old cached geometry meaningless even with
    no user edit at all.
  **Final semantics of `Settings.load_from_cache` (renamed from
  `rerun`, per direct user redesign request, same day)** -- the
  READ/WRITE split is no longer symmetric:
  - The consolidated cache is written **unconditionally** at the end of
    ANY fully successful decomposition run (both the `solids` and, when
    attempted, `enclosures` passes complete without raising) --
    regardless of `load_from_cache`'s own value. So even a deliberate
    "ignore whatever's cached, redo everything" run (`load_from_cache=
    False`) leaves behind a fresh, trustworthy cache for the next run.
  - `load_from_cache` only gates whether an *existing* cache is read
    back and reused at all -- `True` consults it (subject to the label/
    marker rules above), `False` ignores it completely for reading (but
    still overwrites it with this run's own fresh result on success).
  - `Options.n_thread > 1` is disabled (forced to the sequential path,
    with a one-time warning) whenever ANY cache activity is possible
    this run (`load_from_cache` True, or the always-on write path, or
    `TMP_CACHE`'s own staging) -- concurrent writes to the same
    in-memory/on-disk bookkeeping are not supported in this version.
  **Two-tier on-disk storage (added same day, per direct user request:
  "se podria juntar todos los bin en un unico fichero binario... si el
  codigo se para en medio de la decomposicion... se podria recuperar la
  informacion")**:
  1. `decompose_cache/{solids,enclosures}.bin` + `manifest.json` -- the
     last FULLY successful run's consolidated result. Every processed
     solid's pieces (cache hits and freshly decomposed alike, tracked in
     memory as the run progresses -- no need to re-read anything from
     disk to build this) are packed into ONE compound per namespace and
     written via `Gexport_binary` in one shot; `manifest.json` records
     each label's own `{start, count}` slice into that flat piece list
     (native shape enumeration order through a binary round-trip is
     exactly insertion order -- verified directly on all 3 engines
     before relying on it, an 8-box test with distinct volumes came back
     bit-identical in order every time). A label no longer present this
     run simply never entered the in-memory tracking dict and is
     therefore silently absent from the fresh cache -- reconciliation
     (dropping removed/now-ambiguous labels) falls out for free, no
     explicit deletion step needed (this replaced the first design's own
     explicit per-label `reconcile()` call entirely). The `enclosures`
     section is left completely untouched whenever this run didn't
     attempt that pass at all (`Settings.voidGen=False` or no enclosures
     loaded) -- otherwise a run that simply didn't check enclosures
     would wipe out a previous run's own valid enclosure cache.
  2. `decompose_cache/tmp/` -- one small `.bin` file per solid, written
     the instant it's (re)decomposed (not batched), plus its own
     manifest kept continuously up to date via a temp-file-then-
     `os.replace` atomic write (so it's never more than one solid behind
     real progress, and is never left in a half-written state by a crash
     mid-write). Gated by the internal `TMP_CACHE` constant
     (`geo/constants.py`, default `True`, per direct user instruction --
     NOT a user-facing `Settings` field, an internal safety/performance
     knob). This is what survives if a run is interrupted (raises)
     before reaching the final consolidation step -- the previous run's
     own consolidated `.bin`/`manifest.json` are left completely
     untouched in that case (the consolidation code is only ever reached
     after both passes finish cleanly). The NEXT `load_from_cache=True`
     run's own `load()` overlays `tmp/`'s entries on top of the last
     good consolidated cache (`tmp` wins per label, being the freshest),
     so an interrupted run resumes exactly where it left off -- a solid
     that made it into `tmp` before the crash is not recomputed, and
     neither is one the interrupted run never even reached (served from
     the untouched older consolidated cache instead). `tmp/` is deleted
     once a run's own consolidation succeeds (its data is now folded
     into the fresh consolidated files, so it's no longer needed).
     A single monolithic file kept continuously up to date (the
     initially-proposed alternative to this two-tier split) was
     considered and rejected: `BinTools`'s own multi-shape writer
     (`BinTools_ShapeSet`) is an atomic, whole-file-at-once API, so
     keeping ONE file resumable at every point would mean rewriting the
     entire thing after every single solid -- more expensive as the
     model grows, AND itself a new crash-corruption risk (a crash
     mid-rewrite could lose everything accumulated so far, not just the
     one solid in flight). Two tiers gets the performance benefit of one
     big file for the common (fully successful) case while keeping the
     crash-recovery property of independent, individually-finalized
     per-solid files for the in-progress case.
  **Files touched**: new `GEOUNED/decompose/decompose_cache.py`
  (`DecomposeCache`: `load`/`lookup`/`record`/`store`/`finalize`, plus
  `compute_global_key`/`label_key`/`_atomic_write_json` -- pure
  bookkeeping, no geometric comparison at all, by design); `TMP_CACHE`
  (`geo/constants.py`); `Settings.load_from_cache: bool = False`
  (`utils/data_classes.py`, renamed from `rerun`); `GeounedSolid.
  StepLabel`/`.Modified` (`utils/geouned_classes.py`, plain attributes);
  the `__modified__` marker parsing in `loadfile/load_step.py::load_cad`'s
  existing per-node loop (next to the pre-existing `_m<mat>_`/`_d<dil>_`/
  `enclosure<n>_<parent>_` naming-convention parsing -- confirmed the
  literal string `__modified__` cannot match any of those regexes);
  `core.py::CadToCsg.__init__`/`decompose_solids`/`_decompose_solids`/
  `_decompose_target` wire the cache in (label-usability computed once
  per run via a `collections.Counter`, not incrementally per solid; the
  cache object itself is now ALWAYS constructed, since writing is
  unconditional -- `enabled=settings.load_from_cache` only gates
  reading).
  **Storage format precision the user flagged during review, resolved
  by switching storage format entirely (STEP -> native binary)**: the
  export used for caching must never let a solid's analytic quadric
  surfaces (plane/cylinder/cone/sphere/torus/elliptic-cylinder) get
  substituted for a generic spline/revolution/extrusion representation
  on write -- losing that would silently corrupt what gets reloaded from
  cache, since `Gclassify_surface` needs to recognize the SAME surface
  type it saw before caching. First checked and confirmed against the
  original STEP-based implementation (`Gexport_step`/
  `_export_shapes_step`, `geo/{occ,ocp}/io.py`, is a plain
  `STEPControl_Writer.Transfer`/`.Write(..., STEPControl_AsIs)` with no
  surface substitution of any kind -- the BSpline-conversion-for-STEP-
  reader-compatibility trick documented elsewhere in this file, see
  GEOReverse's own "`Geom_Hyperbola`-based revolution/extrusion
  surfaces" entry, is a wholly separate mechanism specific to
  GEOReverse's own external-facing export path for that one rare
  surface family, never invoked here). But per direct user follow-up
  request ("escribir el objeto CAD que esta en memoria en un fichero
  binario... sin pasar por el step"), STEP itself was dropped from the
  cache path entirely -- this round-trip is purely internal (GEOUNED
  writing to and reading back from its own cache, never opened by any
  other tool), so there is no reason to pay STEP's own exchange-format
  cost, or carry its own unrelated format-translation risk for ANY
  surface type, at all. New `Gexport_binary`/`Gload_binary`
  (`geo/{occ,ocp,freecad}/io.py`, re-exported from `geo/__init__.py`)
  wrap OCCT's own native binary shape serialization directly
  (`BinTools.Write_s`/`Read_s` under ocp, `bintools.Write`/`Read` under
  occ, `Part.Shape.exportBinary`/`importBinary` under freecad -- itself
  the same `BinTools` under the hood) -- not an exchange format, no
  other application reads it, no translation of any kind happens on
  write, so the risk this precision was about cannot arise here
  regardless of surface type.
  **Measured, 2026-09-27** (a small 4-solid cylinder/cone/sphere/torus
  compound, ocp): STEP 14761 bytes / 1.35ms write / 6.51ms read vs.
  binary 4374 bytes / 0.20ms write / 0.12ms read -- ~3.4x smaller, ~7x
  faster to write, ~54x faster to read; confirmed working with
  identical surface-type preservation on all 3 engines (occ: identical
  4374-byte output: the binary format is engine-independent, though
  nothing relies on that -- the cache's own `global_key` already
  invalidates on an engine change regardless; freecad: 4811 bytes, same
  surface types preserved). New test
  `test_cache_export_preserves_analytic_quadric_surfaces` builds a
  cylinder/cone/sphere/torus, round-trips them through
  `Gexport_binary`/`Gload_binary`, and asserts `Gclassify_surface`
  returns the identical set of analytic types afterward -- green on all
  3 engines. Native shape enumeration ORDER through this same round-trip
  was independently verified too (an 8-box compound with distinct
  volumes, order compared before/after on all 3 engines) -- load-bearing
  for the consolidated `.bin` files' own `{start, count}` offset scheme.
  **Verified**: `tests/test_decompose_cache.py` (10 tests: first-run
  population, unchanged-labels-skip-decomposition, the `__modified__`
  marker forcing redecomposition of only that solid, new-label
  add/reuse, removed-label reconciliation, duplicate-label
  always-redecompose, global-key invalidation,
  `load_from_cache=False` always writing but never reading (including a
  3rd run confirming a `False`-produced cache is still consulted once
  `load_from_cache=True`), a simulated mid-run crash (a monkeypatched
  `main_split` raises on its 3rd call) followed by a clean run
  confirming BOTH the `tmp/`-staged solids AND the one solid the crashed
  run never reached are served without recomputation, and the
  quadric-surface round-trip test) -- uses controlled, monkeypatched
  `GLabelNode`s (via `Gload_step_labels`) rather than a real
  XCAF-authored labeled STEP fixture (the INPUT model fixtures built by
  the test's own `_make_step` helper are still real STEP files,
  simulating a user's own CAD export -- only the cache's own internal
  storage is binary), since `Gexport_step` itself has no
  label-assignment API; isolates "does the label-driven caching logic
  behave correctly" from "can a STEP writer embed a given name", which
  is not part of this feature. All green on all 3 engines: ocp 290
  passed, occ 290 passed, freecad 296 passed/14 skipped (each full suite
  + these 10) -- zero regressions.
  **A real, independent bug found and fixed via the user's own real
  workflow (`Test RoundCorners/myrun.py`,
  `Solidos/test_models/Big_model_reserved/shed_shutter.stp`,
  2026-09-27)**: the SECOND run of a real 31-solid model with
  `load_from_cache=True` crashed hard on load with a native
  `OCP.Standard.Standard_Failure: Courbes non jointives` inside
  `Gload_binary` -- traced to `_native_fix`/`.fix(DEFAULT_FIX_TOLERANCE)`,
  which `Gload_binary` (occ/ocp) called on every reloaded solid "to match
  `Gload_step`'s own contract" (the reasoning explicitly flagged as
  unverified in that function's own docstring at the time -- exactly
  where it turned out to be wrong). Root cause: `Gload_step`'s healing
  compensates for STEP's own READER, which can introduce subtle
  topology issues invisible to `BRepCheck_Analyzer` on otherwise-valid-
  looking geometry -- a real, documented problem for THAT loader. A
  `BinTools` round-trip has no such reader-reconstruction step at all
  (it's the exact bit-for-bit geometry `main_split` itself produced);
  forcing a fresh `ShapeUpgrade_UnifySameDomain` unify pass onto already-
  fine geometry can itself introduce a topological failure that was
  never there, as this real fixture demonstrated. Fixed by simply NOT
  healing on this path at all (`geo/{occ,ocp}/io.py::Gload_binary`) --
  `freecad`'s own version never had this call to begin with (its own
  `Gload_step` has no equivalent healing step either), so it was
  unaffected. **Verified**: the exact failing script re-run clean after
  the fix, output confirmed byte-identical between a cold-cache and a
  warm-cache run of the same model (only the written "Creation Date"
  comment line differs) -- the first real end-to-end timing measurement
  of this whole feature's actual motivating benefit: decomposition went
  from 10.2s (cold) to 0.41s (warm) on this real 31-solid model, ~25x.
  Full suites re-verified green on all 3 engines after the fix (290/290/
  296+14skipped, unchanged from before -- this bug was never exercised by
  the existing test suite's own trivial box fixtures, which have no face
  that a unify pass would ever touch either way).
  New regression test `test_cache_round_trip_survives_native_healing_on_
  curved_solids` (a real cylinder solid, not a box) added the same day
  to close the coverage gap this bug exposed -- every other fixture in
  this test file is a plain box (6 planar faces), which never exercised
  the code path that crashed.
  **Not verified**: `Options.n_thread > 1` being forced to the
  sequential path still has no dedicated test.

- **`Options.meta_surfaces` -- opt-out of composite meta-surfaces,
  implemented 2026-09-27**: reproduces GEOUNED's original, pre-meta-
  surface behavior on request -- decomposition and cell definition skip
  Can/TCone/RoundCorner/MultiRoundCorner/MultiPlane detection entirely
  and fall back to the 5 basic analytic surfaces (plane/cylinder/cone/
  sphere/torus) directly, with ONE deliberate exception:
  `ReversedConeCylinder` (RevCC) always runs regardless of this
  setting. Per direct user instruction ("en la decomposicion solo
  habria que by-pasear los get_CAN etc y empezar directamente por los
  planos. En la parte de definicion seria hacer lo mismo aunque alli
  hay que preservar el RevCC") -- RevCC isn't a pure compaction/
  simplification the way Can/TCone/RoundCorner/MultiPlane are: an open
  (non-closed) Reversed cylinder/cone face genuinely needs the extra
  bounding surface RevCC provides to stay correctly bounded (see
  `simple_solid_definition`'s own pre-existing `Cylinder`/`Cone` branch
  comments -- "open reversed orientation is handled by RevCC"), so
  turning it off along with the others would silently produce
  under-bounded, wrong cells wherever that configuration occurs, not
  just a larger written definition.
  **What was already there, just unwired**: both the decomposition-side
  generator (`decompose/generators.py::get_surfaces`) and the
  definition-side function (`conversion/cell_definition.py::
  simple_solid_definition`) already had a `meta_surface(s)` parameter
  doing exactly this bypass -- added at some earlier point in the
  project's history but never threaded from any real `Options`/call
  site (both always ran with the hardcoded default `True`). This
  feature is mostly wiring, not new logic: `decom_one_generators.py::
  generic_split`'s own `get_surfaces(...)` call now passes
  `meta_surface=options.meta_surfaces` (it already had `options` in
  scope); `cell_definition.py::build_definition`'s own
  `simple_solid_definition(...)` call now passes `meta_surfaces=
  Surfaces.options.meta_surfaces` (`MetaSurfacesDict` already stores
  `.options`). The one real code change: `simple_solid_definition`'s own
  RevCC block (`get_reversed_cone_cylinder` + its `component_definition.
  append` loop) was moved OUT of the `if meta_surfaces:` branch to run
  unconditionally, with `multiplanes`/`open_multi_round_corners`
  defaulting to `[]` in the `else` branch (a safe, graceful
  degradation, identical to how RevCC already behaves on a solid with
  no real MultiPlane/open MultiRoundCorner nearby -- not a new code
  path of its own).
  **Verified empirically first** (not just reasoned about): a real,
  minimal fixture with a single round corner (`RoundCorners/
  shed_part.stp`, copied into `testing/inputSTEP/RoundCorners/` from
  the workshop corpus) resolves to `RoundC:1, RevCC:0` with
  `meta_surfaces=True` (the corner absorbed into one composite, no
  RevCC needed) and to `RoundC:0, RevCC:1` with `meta_surfaces=False`
  (the same corner's cylinder face falls back to a basic `Cyl` surface
  that DOES need RevCC to stay bounded) -- confirming the bypass and
  the RevCC exception both fire exactly as intended, on both settings,
  with zero errors either way. Pinned as
  `tests/test_cadtocsg.py::test_options_meta_surfaces_false_bypasses_
  composites_but_keeps_revcc`.
  **Verified**: full suites green on all 3 engines after adding the new
  fixture + test: ocp 293 passed, occ 293 passed, freecad 299 passed/14
  skipped -- zero regressions (the new `Options.meta_surfaces` field
  defaults to `True`, so every existing test's behavior is unchanged).
  **Not verified**: no test exercises `meta_surfaces=False` against a
  fixture with a real Can/TCone/MultiPlane (only RoundCorner/RevCC, via
  `shed_part.stp`) -- the bypass logic is identical for all 4 (same
  `if meta_surfaces:` gate in both `get_surfaces` and
  `simple_solid_definition`), so this is a coverage gap in breadth, not
  a known or suspected functional gap; no d1suned stochastic-volume
  check has been run on `meta_surfaces=False` output for any fixture
  yet.
  **Follow-up, same day: `meta_surfaces` added to the decompose cache's
  own global invalidation key**, per direct user instruction -- it
  changes the candidate-surface order `main_split`'s own recursive
  splitting uses (composite surfaces tried first vs. basic surfaces
  only), so a solid's cached pieces from a `meta_surfaces=True` run are
  not trustworthy for a `meta_surfaces=False` run and vice versa, even
  though the input geometry never changed. One line
  (`decompose_cache.py::compute_global_key`'s own
  `decomposition_params` dict gained `"meta_surfaces":
  options.meta_surfaces`, alongside the pre-existing `cut_large_cell`)
  -- reuses the exact same whole-cache invalidation mechanism already
  in place for a `Tolerances` change, no new mechanism needed. New test
  `test_meta_surfaces_change_invalidates_whole_cache`
  (`tests/test_decompose_cache.py`, now 12 tests) confirms both solids
  redecompose when only this one `Options` field flips. **Verified**:
  full suites green on all 3 engines: ocp 294 passed, occ 294 passed,
  freecad 300 passed/14 skipped -- zero regressions.
  **Follow-up, same day: every cache read/write in `DecomposeCache`
  made defensive**, per direct user instruction, after an intermittent
  native crash was observed inside `Gexport_binary` (occ engine,
  `decompose_cache.py::store`) during a long combined test session --
  not reproduced on 2 immediate later attempts (same command, and the
  one failing test run standalone), so its exact cause stayed
  unconfirmed, but it exposed a real design risk regardless: once the
  consolidated cache is written UNCONDITIONALLY on every successful run
  (not just `load_from_cache=True` ones, see above), this native
  write/read path now runs on every solid of every run under `occ`/
  `ocp` -- an auxiliary, purely-for-next-time optimization must never be
  able to fail an otherwise fully successful decomposition. Every
  `Gexport_binary`/`Gload_binary`/JSON read-write call inside `load`/
  `lookup`/`store`/`finalize` is now wrapped in its own `try`/`except
  Exception`, logging a warning and falling back to the safe default
  (treat as a cache miss on read; skip persisting, keep the run going,
  on write) rather than letting anything propagate. `finalize()`
  specifically: a namespace whose own write fails falls back to
  whatever was cached for it before (untouched), and `tmp/` staging is
  preserved (not cleared) whenever any namespace's write failed, so a
  future run can still recover this run's own progress from it even
  after a finalize-level failure, not just a mid-run crash. New tests
  `test_cache_write_failure_never_crashes_an_otherwise_good_run` and
  `test_cache_read_failure_falls_back_to_redecomposing`
  (`tests/test_decompose_cache.py`, now 14 tests) force `Gexport_binary`/
  `Gload_binary` to always raise and confirm the run still completes
  with correct decomposition results either way. **Verified**: full
  suites green on all 3 engines: ocp 298 passed, occ 298 passed
  (including 2 repeats of the exact combined command that originally
  crashed, both clean -- consistent with the crash being either a fluke
  or something this hardening now tolerates regardless), freecad 304
  passed/14 skipped -- zero regressions.

- **`region_sign` (`meta_surfaces_utils.py`) forced an AND/OR answer for
  a genuinely degenerate input -- fixed 2026-09-27**, found via direct
  user request to verify `region_sign`'s own coherence against a real
  solid (`debug/origSolid_0.stp`, generated via `Settings.debug=True` on
  a real workshop model, called via `decompose/generators.py::
  external_plane`). Method: for every real plane-plane adjacent pair in
  the solid (50 pairs), compared `region_sign`'s own AND/OR answer
  against real point-in-solid ground truth (`GSolid.is_inside` via
  `BRepClass3d_SolidClassifier`, sampling a point offset from the shared
  edge along the "material direction" difference between the two
  faces -- a convex/AND edge must exclude that point, a concave/OR edge
  must include it). 49/50 matched exactly; the one exception (faces 0
  and 9) turned out to be two faces on the EXACT SAME analytic plane
  (identical axis/position to float precision, one a 0.73mm^2 sliver
  next to the other's 364.8mm^2 -- almost certainly a residual same-
  plane boolean-cut fragment, the kind `merge_same_surface_faces`
  exists to consolidate elsewhere in the pipeline, but that merge only
  runs in the cell-definition phase, not in this decomposition-side
  check). Per direct user clarification: when two faces are coplanar
  and share an edge, they represent the exact same infinite plane, so
  the dihedral angle is exactly 0 -- AND and OR both describe the
  identical region (intersecting or unioning one half-space with itself
  gives that same half-space either way), making the question ill-posed
  rather than merely hard to decide; forcing an answer is itself the
  bug, regardless of which answer. (The ground-truth point-sampling
  method used to find this doesn't even apply to this specific pair
  either -- both `vect`s are tangential to the SAME shared plane, so the
  sample point never leaves that plane's own surface, testing boundary
  membership rather than a genuine interior/exterior 3D query -- a
  useful reminder that a verification method built for the general case
  can itself break down exactly at the degenerate case it's being used
  to find.) Fixed: `region_sign` now checks `is_same_surface(s1.Surface,
  s2.Surface, tolerances)` right after confirming a shared edge exists,
  returning `None` (the same "no sign to report" convention already
  used for "no common edge") before any AND/OR logic runs. Every real
  call site (`external_plane`/`cutting_face_number` in
  `decompose/decom_utils_generator.py`; `multiplane`/`multiplane_old`/
  `no_convex` in `utils/meta_surfaces.py`/`meta_surfaces_utils.py`) was
  individually checked to confirm treating `None` as "don't trigger this
  branch" is the semantically correct, safe fallback in every one of
  them, not just a value that happens not to crash. New
  `tests/test_meta_surfaces_utils.py` (2 tests): the exact same face
  compared against itself (the simplest deterministic reproduction of
  "same analytic surface") returns `None`; a plain box's own genuine
  convex corners still correctly resolve to `"AND"`, confirming the new
  gate doesn't suppress real, everyday adjacency. **Verified**:
  re-running the same real-fixture check after the fix -- the
  previously-mismatched pair now correctly falls into "undetermined"
  (skipped), and the other 49 pairs are unaffected; full suites green on
  all 3 engines (ocp 294 passed, occ/freecad confirmed green in the same
  combined runs as the cache-hardening entry above, since both fixes
  landed in the same session's test passes).

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
  `convert_to_planes`'s own faceted approximation), now including the
  degenerate torus fixtures' own inner-lobe volumes (fixed 2026-09-17,
  see the `surface_side` entry below) -- their own complement cell
  legitimately still doesn't resolve to a real solid, per the
  "Complement-cell / degenerate-torus boundBox, closed out 2026-09-17"
  entry above (not a bug, genuinely unbounded, no real modeling
  interest).
  `GHyperboloid`'s `OneSheet` flag semantics and how `MCNP_parser`/
  `XML_parser` populate it are now confirmed self-consistent (see the
  classifier-dispatch resolution above) -- the semantic-risk flag this
  entry used to carry is closed.
  `freecad`'s own exotic-quadric implementations (`_freecad_impl.py`)
  were deliberately left untouched throughout, and were believed to
  correspond 1:1 with the occ/ocp techniques for all 7 surfaces (the
  only claimed asymmetry being implementation detail -- native
  `Part.Hyperbola`/`toBSpline` vs `Geom_Hyperbola`/`BRepPrimAPI_MakePrism`
  -- not a different surface). `freecad`'s own `CAD/splitFunction.py::
  surface_side` is the SAME file across all 3 engines (no per-engine
  branching there), so every fix in this entire section (the
  `surface_side`/`myBox` ones especially) applies equally to `freecad`.
  **However**, running the new `test_exotic_quadric_convertion` (below)
  under `freecad` (2026-09-17) found this "1:1" claim doesn't hold at
  the construction level: 11 of the 14 real fixtures fail to convert at
  all under `freecad` (`"failed cell conversion: [1, 2]"`, no traceback
  surfaced yet) -- only `ellipsoid`, `torus_elliptic_nondegenerate`, and
  one other passed. Not investigated further this session (the test
  itself is `skipif(CAD_ENGINE == "freecad")`, matching
  `test_georeverse_occ_impl.py`/`_ocp_impl.py`'s own existing occ/ocp-only
  precedent) -- see "Known open items" -> GEOReverse's own dedicated
  bullet below for this as a real, separate, tracked gap.
- **`freecad`-engine exotic-quadric conversion, several real, distinct
  bugs found 2026-09-17, deliberately not fixed (per explicit user
  instruction -- freecad's own exotic-quadric surfaces are out of scope
  for this project phase; only the error *reporting* was worth fixing)**:
  `CAD/buildCAD.py::BuildUniverseCells`'s own bare `except:` used to
  swallow the real error entirely -- a failed cell only ever showed up as
  its bare name in the final `"failed cell conversion: [...]"` summary,
  with no way to tell why short of re-adding temporary
  `traceback.print_exc()` instrumentation each time (done, and always
  reverted, several times this session). **Fixed** (this alone, nothing
  about the underlying failures below): `except Exception as e:` now
  prints `f"Cell {NTcell.name} failed to build ({type(e).__name__}):
  {e}"` before continuing.
  Running all 14 fixtures under `GEOUNED_CAD_ENGINE=freecad` with this in
  place (2026-09-17, exact per-fixture mapping, unlike an earlier same-day
  pass at this same list that mismatched a couple of entries) -- 7 of the
  14 raise inside `build_universe()` itself (some of today's other fixes,
  `parabola_to_planes` and `myBox`, apply identically under freecad since
  `Utils/boundBox.py` has no per-engine branching, so `paraboloid.mcnp`/
  `elliptic_cone.mcnp`, which used to fail too, now convert correctly
  there as well):
  - `ellipsoid.mcnp`: `RuntimeError: FreeCAD exception thrown (No shells
    or compsolids found in shape)`.
  - `hyperboloid_one_sheet.mcnp`, `cooling_tower.mcnp`:
    `TypeError: Gmake_hyperbolic_cylinder() got an unexpected keyword
    argument 'v_min'` -- `_freecad_impl.py::Gmake_hyperbolic_cylinder`'s
    own signature was never updated to match the `v_min`/`dmax` keyword
    arguments the occ/ocp versions gained during this project's
    hyperboloid-onesheet-routing work.
  - `hyperbolic_cylinder_test.mcnp`: `TypeError: Gmake_hyperbolic_cylinder()
    takes 7 positional arguments but 8 were given` -- a related but
    distinct arity mismatch (same underlying function, different call
    site).
  - `torus_elliptic_nondegenerate.mcnp`: `ValueError: math domain error`.
  - `torus_circular_degenerate_inner.mcnp`,
    `torus_elliptic_degenerate_inner.mcnp`: `OCCError: BRep_API: command
    not done`.
  The remaining 7 (`ellipse_cyl`, `elliptic_cone`, `paraboloid`,
  `hyperboloid_two_sheet_one_branch`, `hyperboloid_two_sheet_outside`,
  `torus_circular_degenerate_outer`, `torus_elliptic_degenerate_outer`)
  convert without raising, but at least 3 give the WRONG volume relative
  to occ/ocp's own, already-verified values -- not just a units/rounding
  difference, freecad's own construction technique for these is a
  genuinely different (and, here, also buggy) implementation:
  `hyperboloid_two_sheet_one_branch`'s own reported volume
  (`290,462,733,361,630.6`) differs from the correct
  `322,152,573,550,950.4` by ~10%; `torus_circular_degenerate_outer`
  (`7,024,639,999.999999` vs. the correct `1,537,740,327.64`) and
  `torus_elliptic_degenerate_outer` (`5,619,712,000.000001` vs. the
  correct `1,230,192,262.11`) are both wrong by the exact same ~4.567x
  factor -- consistent with `_freecad_impl.py::Gmake_torus_elliptic`
  having no outer/inner sheet support at all (already documented
  elsewhere in this file) and building something other than the intended
  single sheet for a degenerate torus.
  `occ`/`ocp` are fully unaffected and verified
  (`test_exotic_quadric_convertion` green on both, `skipif`'d under
  freecad). Not investigated or fixed further -- freecad's own exotic-
  quadric implementations (`_freecad_impl.py`) are explicitly out of
  scope; this entry exists so a future session doesn't have to
  rediscover the same errors from scratch.

### Shared / cross-cutting (touches both pipelines, or is test-fixture housekeeping)

- **`GEOUNED`'s `build_region/` vs `GEOReverse`'s `CAD/buildSolidCell.py`+
  `CAD/splitFunction.py`, unified 2026-09-17/18** (supersedes the
  "remain two separate implementations... no further code has moved
  beyond `Gfuse_solids`" note this entry used to carry): per direct user
  clarification, the two pipelines' end goals genuinely differ --
  GEOReverse's own plane-approximation-of-surfaces step
  (`Utils/boundBox.py`'s `solid_plane_box`/`convert_to_planes`/`myBox`)
  exists only because it starts with nothing but a boolean surface
  definition and no real CAD solid yet (approximating each surface by a
  handful of planes and intersecting them gives a fast, tight starting
  bounding box before the real, expensive boolean cuts); GEOUNED never
  needs this, since it already has the real CAD solid from the STEP file
  and therefore its real BoundBox directly. Similarly, `BuildDepth`
  itself serves two different end goals: GEOUNED uses it to *construct*
  the small solid a composite meta-surface (RoundCorner/Can/TCone/
  MultiRoundCorner) itself represents, from its own 2-4 primitive
  components (plane/cylinder/cone/sphere only); GEOReverse uses it to
  *reconstruct* an arbitrary MCNP/OpenMC cell's full solid -- starting
  from a (possibly approximate) bounding box, splitting whatever pieces
  come out by each of the cell's own real surfaces one at a time, and
  keeping/rejecting/re-splitting each piece per the cell's boolean
  definition.
  Despite the different end goal, a side-by-side read of both
  implementations found the actual recursive algorithm --
  `BuildDepth`/`BuildSolidParts`/`filterparts`/`getPart`/`SplitBase`/
  `joinBase`/`SplitSolid`'s outer shell -- was essentially line-for-line
  identical in both, and had already drifted in small, silent ways
  (GEOUNED threaded an explicit `tolerances` argument throughout;
  GEOReverse read a bare `Options.splitTolerance` global deep inside
  `SplitSolid` instead; GEOUNED hand-duplicated the exact
  `inSolid if type(inSolid) is bool else None` logic `evaluate_three_valued`
  already provided). Worse, `myBox`'s own box arithmetic (`add`/`mult`)
  turned out to have the exact same real bug in both independently-
  maintained copies -- confirmed empirically (see `geo/vector_geometry.py`'s
  own `myBox` docstring, and the exotic-quadric entry above for how the
  GEOReverse-side bug was originally found and fixed): GEOUNED's own
  `box_intersect`/`plane_region`-based version also returned UNSAFE
  (excluding real material) in the same mixed-orientation cases, except
  it was dead code there -- its one live call site, `filterparts`,
  always constructed both operands as `Forward`, so the buggy branch
  never actually executed. That this drift went unnoticed until it was
  checked by accident is itself the argument for unifying rather than
  continuing to hand-sync two copies.
  **What actually moved**, in 3 steps, each independently verified:
  1. `myBox` (+ its `_box_volume` helper) moved into `geo/vector_geometry.py`
     -- GEOReverse's own, already-fixed-and-empirically-verified copy is
     now the single implementation; GEOUNED's buggy `box_intersect`/
     `plane_region`/`operate_box` (confirmed dead code, zero callers in
     either pipeline besides its own recursion) were deleted outright
     rather than ported. A `.Volume` attribute (GEOUNED's own addition,
     read by `build_shape_functions.py::build_complex_shape`) was folded
     into the shared class.
  2. `evaluate_three_valued` moved into `boolean_utils/boolean_function.py`
     (right next to `BoolSequence` itself, zero new dependency);
     GEOReverse's own `Utils/booleanFunction.py` re-exports it for its
     existing callers, and GEOUNED's `build_region/splitFunction.py`
     (since deleted, see step 3) was updated to call it instead of its
     own hand-duplicated inline version.
  3. `BuildDepth`/`BuildSolidParts`/`filterparts`/`getPart`/`SplitBase`/
     `joinBase`/`SplitSolid`/`space_decomposition` moved into
     `geo/solid_ops.py`, generalized over exactly the two genuine,
     surviving differences: a pluggable `classify(point, surf) -> bool`
     callable (GEOUNED passes `lambda p, s: s.is_inside(p)`; GEOReverse
     passes its own, much richer, ~15-surface-type `CAD/splitFunction.py::
     surface_side` unchanged -- neither pipeline's own point-
     classification code was touched by this move), and
     `hasattr(cell, "build_BoundBox")`/`hasattr(cell, "buildSurfaceShape")`
     guards standing in for what used to be one pipeline's own commented-
     out call and the other's active one (GEOUNED's `CellObj` has neither
     method -- its single, always-Forward `boundBox` is set once up front
     and its surfaces pre-built once, before this cascade ever runs --
     GEOReverse's `CadCell` has both, since an arbitrary CSG cell's own
     subcells genuinely need their own, tighter, lazily-computed box and
     lazily-built surface shapes). `GEOUNED/utils/build_region/
     build_region.py` now keeps only `get_cell_object`/`get_surface`
     (the genuinely GEOUNED-specific translation from a composite meta-
     surface's own `GeounedSurface` tree into the small `CellObj`/
     `CellSurface` this cascade operates on); its sibling
     `build_region/splitFunction.py` had nothing GEOUNED-specific left
     once `SplitBase`/`joinBase`/`SplitSolid` moved out, and was deleted
     outright. `GEOReverse/Modules/CAD/buildSolidCell.py` keeps only
     `BuildSolid()` (now the single place that converts GEOReverse's own
     `Options.splitTolerance` float into a real `Tolerances` instance,
     once, instead of `SplitSolid` re-wrapping it on every call);
     `CAD/splitFunction.py` keeps `surface_side`/`btwPPlanes`/
     `updateSurfacesValues`, its own exotic-quadric-aware point-
     classification machinery, untouched.
     One real, deliberate behavior unification (not a pre-existing
     divergence, decided by direct user instruction): `SplitSolid`'s call
     into `Gsplit` is now wrapped in a `try`/`except` with a `print` on
     failure (falling back to the uncut solid) in **both** pipelines --
     GEOReverse's own copy already had this fallback, but as a fully
     silent `except Exception: Solids = []` (the same "silently swallowed
     error" anti-pattern already fixed elsewhere this session in
     `CAD/buildCAD.py::BuildUniverseCells`); GEOUNED's copy had no
     try/except at all (a `Gsplit` failure there would have crashed
     outright). Both now share the fallback *and* a visible error message.
  **Verified**: full `tests/geo` + `test_cadtocsg.py` + `test_csgtocad.py`
  + `test_boolean_function.py` green on all 3 engines after all 3 steps
  (freecad 179 passed/16 skipped, occ 213 passed/1 skipped, ocp 213
  passed/1 skipped -- matching each engine's own pre-existing baseline,
  zero regressions); the empirical `myBox` safety audit (Forward/Reversed
  x add/mult x 8 relative box configurations) re-run against the new
  shared location, 0 unsafe across 64 checks; `hyperbolic_cylinder_test.mcnp`
  (GEOReverse) converts to the identical volume whether run in isolation
  or immediately after other fixtures in the same process (confirms the
  move didn't reintroduce any shared-state flakiness); a 141-file
  differential composite-surface-count scan across `Solidos/test_models`
  (GEOUNED side, `git stash`-based before/after, decompose +
  build_solid_definition only) found **0 changed files, 0 new failures,
  0 escape-hit-count changes** in either pass -- confirming the whole
  move is behavior-preserving for GEOUNED, as intended.
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
- `Solidos/` STEP fixture tree duplicates, closed out 2026-09-17: a full
  content-hash comparison (not just filename matching) of every
  `Solidos/` triage folder against the curated `Solidos/test_models`
  regression set found exactly 12 byte-identical duplicates (2 in
  `working_solids/`, 4 in `lost_particles/`, 6 of `RevCC_corpus_scan/`'s
  276 files) -- moved (never deleted, per standing workshop policy) into
  a new `Solidos/_archive/<origin_folder>/` tree, preserving their
  origin folder in the path. Everything else flagged by a naive
  filename-only pass turned out to be same-named-but-different-content
  (coincidental `RevCC_corpus_scan` piece/revcc naming) or genuinely
  distinct historical debugging dumps with no `test_models` counterpart
  at all (`lost_particles` has 21 files, only 4 were dupes;
  `RevCC_corpus_scan` has 276, only 6 were dupes) -- left in place, out
  of scope for this pass (per direct user instruction: only exact
  duplicates were to be archived, not a broader triage-folder cleanup).
- **Spline-vs-quadric identification -- new investigation opened
  2026-09-18, branch `spline-quadric-detection` (off
  `georeverse-migration`), unresolved, actively open**: `Gclassify_surface`
  returns `None` for ANY `BSplineSurface` face (occ/ocp; freecad has a
  `findPlane()`-only fallback for the plane case), causing GEOUNED to
  drop the whole solid (`Gspline_surface`, `load_step.py`'s
  `spline_surf`/`corrupted_solids` handling) even when the BSpline is
  secretly, or closely enough, one of the 5 supported analytic quadrics
  (plane/cylinder/cone/sphere/torus) -- occ/ocp's own `Gclassify_surface`
  docstring already flags this as a known, unported gap. Motivating
  real-world source of such faces: GEOReverse's own `_revolution_to_bspline`
  (`_occ_impl.py`/`_ocp_impl.py`) already converts real analytic
  revolution surfaces to BSpline before STEP export for reader
  compatibility (see "GEOReverse" -> "STEP round-trip for
  `Geom_Hyperbola`-based..." above) -- the same kind of "real quadric,
  exported as a spline" case can occur from other CAD tools' STEP output
  too. **Architecture decision (confirmed with the user)**: substitute
  the native surface for real (physically replace the BSplineSurface in
  the loaded solid, at the load-time repair-cascade layer next to
  `Gcheck_and_repair`/`Gheal_topology`/`Gspline_surface` in
  `geo/{occ,ocp}/io.py`), NOT just relabel `GFace.Surf` while leaving the
  native face as a BSpline -- the latter (simpler, zero geometry risk)
  was considered and rejected: `Gsplit`/`Gcut`/`Gfuse` and the coaxial-
  cone/tangency repair cascade (`geo/surface_geometry.py`) all inspect
  the REAL native surface type, not `GFace.Surf`, so only a real
  substitution actually buys back the numerical boolean-robustness this
  whole pyOCC migration exists for; relabeling alone would fix
  classification/MCNP-OpenMC output but leave the exact tangency
  fragility class of bug this project starts from untouched for those
  faces.
  **Detection: validated for all 5 types.** Sample a 15x15 (u,v) grid
  over the face's own domain via `GeomLProp_SLProps` (point + normal),
  fit each candidate type (plane: SVD of centered points; cylinder: axis
  via SVD of mean-centered normals + algebraic circle fit; sphere: linear
  algebraic fit; cone/torus: hand-rolled Levenberg-Marquardt in numpy --
  no scipy in `ocpenv`), compare residuals. Confirmed on a real
  `GeomConvert.SurfaceToBSplineSurface_s`-converted face (exact
  conversion) AND after a genuine STEP write/read round-trip: true-
  positive residuals ~1e-12 to 1e-13, vs. ~1 for a genuine non-quadric
  freeform face (a loft between two non-coaxial circles) -- roughly 9
  orders of magnitude of separation, enormous safety margin for any
  reasonable detection tolerance. One real subtlety: a sphere is a
  degenerate torus (major radius R=0), so the general torus fit also
  converges near-perfectly on a sphere face -- a real detector needs a
  "prefer the simpler model" tie-break (plane < sphere < cylinder < cone
  < torus) rather than bare minimum-residual.
  **Substitution: validated for cylinder/sphere/torus (3 of 5), cone
  unresolved.** `BRep_Builder.UpdateFace(face, new_surf, loc, tol)`
  alone leaves the solid `BRepCheck`-invalid
  (`BRepCheck_UnorientableShape`) -- the face's own edges still carry
  pcurve representations keyed to the OLD surface object. Full working
  recipe: for each of the face's edges, get its 3D curve + parameter
  range (`BRep_Tool.Curve_s`/`BRep_Tool.Range_s` -- note the OCP binding
  does NOT return first/last as out-params despite the C++ signature,
  fetch range separately via `BRep_Tool.Range_s(edge)`), project onto the
  new surface (`GeomProjLib.Curve2d_s`), attach via
  `BRep_Builder.UpdateEdge(edge, pcurve2d, new_surf, loc, tol)`; a
  degenerate (pole/apex) edge has no real 3D curve to project -- build
  its pcurve by hand instead, a `Geom2d_Line` at the vertex's own (u,v)
  on the new surface (found via `ShapeAnalysis_Surface.ValueOfUV` point
  inversion) spanning the surface's full U period; finally
  `BRepLib.SameParameter_s(face, tol, True)` +
  `ShapeFix_Face(face).Perform()`. Confirmed on cylinder/sphere/torus:
  whole-solid `BRepCheck_Analyzer` valid, volume exact to printed
  precision, untouched neighboring faces (e.g. a cylinder's planar caps)
  unaffected -- shared edges just gain an additional pcurve
  representation, nothing is removed. **Cone specifically stays
  `BRepCheck_UnorientableShape`** even when every substituted value
  (surface object, all pcurves including the degenerate apex edge's) is
  the bit-identical ORIGINAL data the valid native cone already had --
  this isolates the bug to the `UpdateFace`/`UpdateEdge` +
  `SameParameter` + `ShapeFix_Face` MECHANISM itself misbehaving
  specifically for a face with exactly one degenerate edge AND a real
  (non-closed) open boundary -- sphere/torus (which work) are fully
  closed surfaces with no independent boundary edge of their own;
  cylinder (which works) has no degenerate edge at all; cone is the only
  one of the 4 with both a degenerate edge and a real open boundary.
  `ShapeFix_Face.Perform()` itself reports `ShapeExtend_OK`/`False`
  ("nothing to fix") for BOTH cylinder and cone, yet only cylinder ends
  up valid afterward -- contradicts the visible before/after validity
  change for cylinder, suggesting a side effect of `ShapeFix_Face`'s own
  `Init`/constructor (or possibly of `BRepLib.SameParameter_s`, tested
  alone and confirmed NOT sufficient by itself for any of the 4 types)
  not reflected in its own reported status. Not isolated further --
  would need OCCT source-level inspection or a minimal C++ reproduction,
  not practical from Python/pyOCC introspection alone.
  **Alternative (boolean-based) approach tried for cone, also
  inconclusive**: rebuild a generous analytic solid via
  `BRepPrimAPI_Make*` and `BRepAlgoAPI_Common` against the original
  solid, letting OCCT's own BOP reconcile topology instead of hand-
  patching pcurves. Hit two new, separate problems: (1) needs a
  topologically-VALID starting solid -- an in-memory
  `GeomConvert`-converted face with no rebuilt pcurves produces an EMPTY
  boolean result, so a real STEP round-trip is needed first to get
  validity; (2) that STEP round-trip itself introduces a real ~0.9%
  volume discrepancy against the analytic value (`BRepGProp` numerical-
  quadrature error integrating over a BSpline surface vs. a true
  analytic one, suspected but not confirmed -- contradicts the ~1e-12
  POINTWISE residuals found during detection, since that was a local
  check, not an integrated one); and (3) even with a valid input, a
  "generous" tool solid that fully contains the original makes
  `BRepAlgoAPI_Common` a no-op that keeps the ORIGINAL imprecise spline
  face rather than adopting the tool's exact analytic surface -- which
  of two near-coincident "same-domain" faces a boolean keeps isn't
  something controlled from the high-level `BRepAlgoAPI_Common` API used
  here.
  Scripts used are throwaway, in the session's scratchpad directory (NOT
  committed, not preserved in the repo): `spline_quadric_probe.py`
  (detection/fitting for all 5 types + false-positive test + STEP
  round-trip noise test), `spline_substitution_probe2.py` (topology-patch
  substitution recipe, cylinder generalized to cone/sphere/torus, all
  diagnostics), `spline_substitution_boolean.py` (the alternative
  boolean-based attempt) -- this CLAUDE.md entry is the only surviving
  record of the investigation.
  **Implemented in production, 2026-09-18, branch `spline-quadric-
  detection` (cylinder + torus only; sphere and cone remain open)**: per
  direct user instruction ("vamos a implementar estos cambios en
  geouned. Para las esferas, cilindros y toros... si la sustitución no
  tiene éxito entonces se sigue la instrucción vigente"), wired the
  validated Option-1 architecture (physically substitute the native
  surface, not just relabel `GFace.Surf`) into real code:
  `geo/{occ,ocp}/spline_quadrics.py` (new files, ~parallel implementations,
  same structure as every other occ/ocp file-pair) provide
  `Gsubstitute_spline_quadrics(solid, tolerances) -> (solid, resolved)`,
  called from `Gload_and_process_step` right where `Gspline_surface`
  already flags a solid: a solid that would previously have gone
  straight to `spline_indices` (triggering the existing remove/stop
  policy) now gets ONE substitution attempt first; only added to
  `spline_indices` if that attempt does not fully resolve it -- the
  exact "try substitution, fall back to the existing policy on failure"
  behavior the user asked for, implemented as an all-or-nothing,
  never-partially-corrupting operation (works on a native-shape COPY;
  the original `GSolid` is returned completely untouched on any
  failure). Two new `Tolerances` fields added
  (`GEOUNED/utils/data_classes.py`): `spline_quadric_fit_rel_tol`
  (1e-6, the detection residual gate) and `spline_quadric_volume_rel_tol`
  (2e-2, the post-substitution volume-conservation gate -- see the real
  bug below for why this isn't tighter). `freecad` is out of scope (not
  touched) -- occ/ocp only, matching every other feature in this section.

  Porting the already-validated scratchpad recipe into real production
  code (called through the REAL `Gload_and_process_step` path, against a
  REAL STEP-loaded solid -- not just an in-memory `GeomConvert`
  conversion, which is all the original research validated) surfaced 2
  more real, independent bugs, neither anticipated by the original
  investigation:
  1. **Shared seam-edge pcurve bug**: a face on a FULLY periodic surface
     (a whole cylinder/sphere/torus) has its own seam edge appear TWICE
     in the wire -- the SAME underlying `TopoDS_Edge` (`IsSame`), once
     FORWARD once REVERSED -- needing TWO distinct pcurve
     representations on the same (surface, location) via the dedicated
     `BRep_Builder.UpdateEdge(E, C1, C2, S, L, Tol)` "closed face"
     overload. The original recipe called the single-pcurve overload
     once per occurrence, which just overwrites the first with the
     second -- losing one side of the seam. This worked BY ACCIDENT for
     an exactly-`GeomConvert`-converted face (`ShapeFix_Face`'s own
     `FixMissingSeam` silently reconstructed the missing side, close
     enough within its own tolerance) but broke for a face that had gone
     through a real STEP write/read round-trip first -- confirmed live:
     cylinder's substituted face came back `BRepCheck_UnorientableShape`
     only in the STEP-round-tripped case, never in the original in-memory
     test. Fixed: detect the duplicate via `IsSame` and set `C2` as `C1`
     translated by exactly the surface's own U period (`Geom2d_Curve.
     Translated`, guaranteed geometrically consistent with `C1` since a
     periodic surface's own pcurve is invariant under a full-period
     shift) -- in one `UpdateEdge` call instead of two. Also discovered:
     a solid's own `_native_fix`/`ShapeFix_Shape` healing pass (already
     applied at load time, before this function ever runs) sometimes
     splits a shared seam edge into two genuinely independent
     (non-`IsSame`) copies on its own -- the fix handles both cases
     (falls back to a single pcurve per copy when they're no longer
     actually shared). **pythonocc-core-specific wrinkle found while
     porting this fix to `occ`**: `Geom2d_Curve.Translated()` is bound at
     the `Geom2d_Geometry` (base-class) level under pythonocc-core,
     returning that wider handle type even though the object is still
     concretely a curve -- `BRep_Builder.UpdateEdge`'s own SWIG overload
     resolution rejects it outright (`TypeError`) unless explicitly
     downcast back via `Geom2d_Curve.DownCast(...)` first. OCP has no
     such issue (a `Geom2d_Curve` comes back directly). This fixed
     cylinder completely (exact volume, valid, even after a real STEP
     round-trip) and made no difference either way for torus (which
     already worked).
  2. **Volume-tolerance false rejection**: even once (1) was fixed,
     cylinder still failed to resolve -- traced to
     `Gsubstitute_spline_quadrics`'s own volume-conservation safety
     check (originally `1e-4` relative) comparing the POST-substitution
     volume against the PRE-substitution one. The substitution itself
     was perfect (exact analytic volume, valid solid) -- but the
     "before" volume, computed by `BRepGProp` integrating over the
     original, still-a-BSpline face, itself carries a real ~0.9%
     numerical-quadrature error after a genuine STEP round-trip (a
     `Geom_CylindricalSurface` integrates its volume exactly; a BSpline
     approximation of the same surface does not, even though the two are
     geometrically all but identical). The tight gate was comparing a
     newly-exact result against an unreliable baseline and rejecting a
     correct substitution. Fixed by widening
     `spline_quadric_volume_rel_tol` to `2e-2` (2%) -- still far below
     the 10-100% scale a genuinely broken substitution mechanism produces
     (confirmed via the original investigation's false-positive testing),
     just no longer measuring noise in the "before" number as if it were
     substitution error. The actual correctness guarantee for a
     substitution comes from the FIT residual check
     (`spline_quadric_fit_rel_tol`, still tight at `1e-6`, with the
     original ~9-orders-of-magnitude separation margin), not this volume
     gate, which is now correctly understood as a coarse sanity net, not
     a precision check.

  **Sphere remains unresolved for a real STEP round-trip** (detection
  succeeds cleanly -- residual ~2e-12 -- but the post-substitution face
  still comes back `BRepCheck_UnorientableShape`), and is a DIFFERENT
  bug from both of the above and from the cone issue: confirmed the
  exact same "duplicate seam" fix that completely resolved cylinder does
  NOT fix sphere (still fails identically with or without it); confirmed
  it is NOT the degenerate-pole pcurve construction either (bit-identical
  to what already works in the in-memory, non-round-tripped case, which
  DOES succeed for sphere -- `BRepCheck_Analyzer` valid, exact volume).
  One promising, NOT-yet-fixed lead found while investigating: bypassing
  `GeomProjLib.Curve2d`'s numerical projection entirely for the seam edge
  and constructing its pcurve directly from the sphere's own analytic
  parametrization (v = latitude) produces a topologically VALID face --
  but with only HALF the correct volume, meaning the direct-construction
  formula has an unresolved sign/branch bug (which of the two meridian
  halves, or which u-branch, isn't being picked consistently) rather
  than being fundamentally the wrong approach. Not pursued further this
  session given the time already spent -- picking this up should start
  from that half-volume clue, not from scratch. Sphere still safely
  falls through to the existing spline-solid policy (remove/stop) when
  it fails, per the all-or-nothing design -- no corruption risk, just
  missing the benefit for this one type until fixed.

  **Verified**: `tests/geo/test_ocp_impl.py`/`test_occ_impl.py` each
  gained 3 new tests (`test_substitute_spline_cylinder`/`_torus`/
  `_sphere_known_limitation`, the last one deliberately asserting
  TODAY's limitation, not the eventually-desired behavior -- update it
  once sphere is fixed) exercising the real `Gload_and_process_step`
  path against a genuinely STEP-round-tripped spline face; both pass (42
  total each, up from 39). Full `tests/geo/test_ocp_impl.py`/
  `test_occ_impl.py` (42 each) and `tests/test_cadtocsg.py` (50, both
  engines) green with zero regressions.

  **Next steps**: superseded by the master TODO list at the end of this
  whole entry ("TODO -- generalized surface-to-quadric identification
  & substitution, 2026-09-18"), which restates the project's own goal
  more broadly (per direct user re-framing, same day) and organizes
  every remaining item -- sphere/cone included -- into one place. Read
  that list, not this paragraph, before picking anything back up.

  **`GEllipticCylinder` -- a 6th BASE analytic surface type for
  GEOUNED's own forward pipeline, implemented 2026-09-18, same session
  and branch.** Motivated directly by a real fixture the user pointed at
  (`Solidos/Spline/SOLID015.stp`, workshop-side): its 2 "spline" faces
  turned out to be neither `BSplineSurface` nor genuinely ambiguous --
  an EXACT `GeomAbs_SurfaceOfExtrusion` of a real `Geom_Ellipse`
  (MajorRadius=73, MinorRadius=69, a straight sweep -- extrusion
  direction exactly parallel/antiparallel to the ellipse's own plane
  normal, confirmed via a dot product of -1.0). Per direct user
  instruction: implement this as a genuine 6th base surface type
  (mirroring Cylinder's own architecture end to end), deliberately NOT
  combined into composite meta-surfaces (RoundCorner/Can/TCone/...) yet,
  and use this as the TEMPLATE for adding the remaining exotic quadrics
  later -- explicitly including, this same time, unifying each new
  surface's construction between GEOUNED and GEOReverse (matching the
  `Gmake_torus_elliptic` precedent) rather than letting the two pipelines
  carry independent copies.

  Full trace of what "mirror Cylinder end to end" meant in practice,
  file by file:
  - `geo/{occ,ocp}/topology.py`: new `GEllipticCylinder` class (Center/
    Axis/MajorRadius/MinorRadius/MajorAxis/MinorAxis, field names
    matching GEOReverse's own dataclass exactly) plus a new
    `Gclassify_surface` branch recognizing `GeomAbs_SurfaceOfExtrusion`
    with a `Geom_Ellipse` basis curve and a straight sweep -- an EXACT
    recognition, no sampling/fitting involved (unlike
    `Gsubstitute_spline_quadrics`'s BSpline detection). An OBLIQUE sweep
    (extrusion direction not parallel to the ellipse's own normal) is
    deliberately NOT recognized -- deriving the true elliptic-cylinder
    axes of an oblique sweep is a harder problem, explicitly out of
    scope for now, same pattern as leaving cone out of the spline
    substitution above. pythonocc-core needed explicit `Geom_
    SurfaceOfLinearExtrusion.DownCast`/`Geom_Ellipse.DownCast` calls
    that OCP's own automatic polymorphic downcast doesn't need (same
    binding-convention difference documented throughout this project).
    `freecad` is untouched -- occ/ocp only.
  - `geo/{occ,ocp}/primitives.py`: `Gmake_elliptic_cylinder` -- moved
    (not duplicated) from `GEOReverse/Modules/engine_dependency/
    _{occ,ocp}_impl.py`, exactly the `Gmake_torus_elliptic` precedent.
    GEOReverse's own files still need updating to re-export this name
    instead of keeping their own local copy (not done yet in this pass
    -- see below).
  - `GEOUNED/utils/basic_functions_part1.py`: `EllipticCylinderOnlyParams`
    (bare surface) and `EllipticCylinderParams` (adds an optional
    truncation `.Plane`, mirroring `CylinderParams` exactly -- a base
    surface still legitimately combines with a bounding plane; that is
    not the composite-meta-surface combination the user asked to defer).
  - `GEOUNED/utils/geouned_classes.py`: `GeounedSurface` dispatch for
    `"EllipticCylinderOnly"`/`"EllipticCylinder"`; `build_surface()`
    branch calling a new `makeEllipticCylinder`
    (`build_shape_functions.py`, mirroring `makeCylinder`); a new
    `MetaSurfacesDict.add_elliptic_cylinder` (region-building, mirrors
    `add_cylinder` including its optional-plane combination) and
    `SurfacesDict.add_elliptic_cylinder` (dedup/numbering, mirrors that
    class's own `add_cylinder`); a new `"EllCyl"` key added to BOTH
    classes' own `surfname` lists and to `extend()`/`add_surface()`'s
    dispatch tables (two genuinely separate lists that both needed the
    new key -- confirmed by grep, nothing else in the codebase
    hardcodes a third copy of either list).
  - `GEOUNED/utils/basic_functions_part2.py`: `is_same_elliptic_cylinder`
    (CSG-level dedup predicate, mirrors `is_same_cylinder`, reuses the
    existing `cyl_distance`/`cyl_angle` tolerances rather than adding new
    ones for this first cut) -- checks MajorRadius, MinorRadius, Axis
    and MajorAxis alignment (`is_parallel`, already sign-agnostic), then
    the perpendicular-offset-from-axis-line distance (never raw Center
    difference, matching `is_same_cylinder`'s own documented reasoning
    for why that would be wrong for an axis point that can slide freely).
  - `geo/surface_geometry.py`: `is_same_elliptic_cylinder_surface`
    (FACE-MERGE-level predicate, a different function from the previous
    bullet -- mirrors `is_same_cylinder_surface`, fixed 1e-5 tolerances,
    used by `geometry_gu.py`'s `_SAME_SURFACE_PREDICATE` dict so
    `merge_same_surface_faces` can recognize two `GEllipticCylinder`
    faces of the same real surface without a `KeyError`). Confirmed this
    dict lookup is reached in practice: the real fixture's 2 raw
    elliptic-cylinder faces get merged into one face BEFORE
    `cell_definition.py`'s per-face loop even runs, verified live via
    instrumentation (only one `gen_elliptic_cylinder` call for the whole
    solid, despite 2 raw faces).
  - `GEOUNED/conversion/cell_definition_functions.py`/`cell_definition.py`:
    `gen_elliptic_cylinder` + a new `isinstance(face.Surface, GU.
    GEllipticCylinder)` branch, mirroring the `GCylinder` branch exactly.
  - `GEOUNED/utils/q_form.py`: new `q_form_elliptic_cyl` -- deliberately
    NOT built by reusing `rotation_matrix(u, v)` (that helper only pins
    the axis mapping, leaving rotation-about-axis free, which matters
    for an ellipse's major/minor orientation but not a circle's). Built
    directly from the symmetric matrix `M = MinorRadius^2*(MajorAxis
    (x) MajorAxis) + MajorRadius^2*(MinorAxis (x) MinorAxis)` (M's null
    space is exactly the cylinder axis, by construction, since MajorAxis/
    MinorAxis/Axis are mutually orthogonal) -- confirmed both
    numerically (residual ~1e-11 sampling real ellipse points, both
    axis-aligned and an arbitrary 3D orientation) and by proving `M .
    Axis = 0` exactly, which makes the whole GQ equation provably
    invariant to sliding the input `Center` along the axis (verified
    numerically too: identical G/H/J/K for 4 wildly different slide
    distances) -- and reduces exactly to `q_form_cyl`'s own matrix (up
    to an immaterial overall `rad^2` scale factor) when MajorRadius ==
    MinorRadius, since `MajorAxis(x)MajorAxis + MinorAxis(x)MinorAxis =
    I - Axis(x)Axis` for an orthonormal frame.
  - `GEOUNED/write/functions.py`: a new `"EllipticCylinderOnly"` branch
    in EACH of the 4 surface writers (`mcnp_surface`/`open_mc_surface`/
    `serpent_surface`/`phits_surface`) -- always the general GQ/quadric
    form, no axis-aligned shortcut of any kind, since none of these 4
    formats has an elliptic-cylinder primitive card at all (unlike
    Cylinder's CX/CY/CZ, and OpenMC's own additional generic
    arbitrary-direction "Cylinder" surface -- neither exists for an
    ellipse).
  - `freecad` safety: confirmed via grep that every `GEllipticCylinder`/
    `Gmake_elliptic_cylinder` reference in core GEOUNED code (outside
    `geo/occ`/`geo/ocp` themselves) is either a function-local import
    (matching the established `Gmake_torus_elliptic` precedent -- doesn't
    exist under `geo/freecad/__init__.py`) or goes through
    `geometry_gu.py`'s `GU.GEllipticCylinder`, which is a real class
    under occ/ocp but an always-`False`-matching placeholder class under
    freecad specifically so `isinstance(face.Surface, GU.
    GEllipticCylinder)` checks elsewhere never raise `AttributeError`
    regardless of engine. The live freecad test suite itself was not
    re-run this session (interpreter path not at hand); this static
    check is the verification on record until a real freecad run
    confirms it.
  - Adjacency/truncation-plane suppression in `decompose/
    decom_utils_generator.py` (the `GCone`/`GCylinder`-pair special case
    that skips adding a redundant plane between two adjacent curved
    faces) was deliberately NOT extended to `GEllipticCylinder` --
    degrades gracefully (a few extra, harmless truncation planes in the
    decomposition, never a correctness issue), left as a known,
    low-priority follow-up rather than in scope for this first cut.

  **Verified against the real motivating fixture**: full `tests/geo/
  test_ocp_impl.py`/`test_occ_impl.py` (42 each) and `tests/
  test_cadtocsg.py` (50, both engines) still green with zero
  regressions after all of the above. `SOLID015.stp` converts end to end
  through the real `CadToCsg` pipeline (load -> decompose -> void ->
  `export_csg` to MCNP) with no crash, previously impossible (its 2
  elliptic-cylinder faces used to make the whole solid an unconvertible
  "spline" solid). A real, live investigation (thought at first to be a
  dedup bug: the exported MCNP file has 9 "GQ" surface cards total, and
  8 of them share suspiciously similar-looking coefficients) turned out
  to be a false alarm on close inspection, not a bug: instrumenting both
  `q_form.q_form_cyl` and the new `q_form.q_form_elliptic_cyl` directly
  showed exactly 8 independent `q_form_cyl` calls (8 real, small,
  unrelated bolt-hole cylinders elsewhere in this real mechanical part,
  radius 0.65/1.25mm, at 2 X/Y positions x 2 Z-heights -- their A-F
  GQ coefficients coincidentally land in the same order of magnitude as
  the elliptic cylinder's own, purely because `q_form_cyl`'s A-F terms
  depend only on axis ORIENTATION, not radius, so same-orientation
  round holes of any size produce visually similar-looking leading
  coefficients) and exactly ONE `q_form_elliptic_cyl` call, confirming
  the elliptic cylinder itself was correctly detected once, deduplicated
  once (from its 2 raw faces), and written once. No corpus-wide/GEOReverse
  regression check has been run yet (out of scope for this pass -- see
  "Next steps" below).

  **Not done in this pass (real, tracked gaps, not silent omissions)**:
  (a) GEOReverse's own `_occ_impl.py`/`_ocp_impl.py` still carry their
  OWN local `Gmake_elliptic_cylinder`/`GEllipticCylinder` definitions
  rather than re-exporting the ones now living in `geo/{occ,ocp}/
  primitives.py` -- the "unify calls between GEOUNED and GEOReverse"
  instruction was satisfied for GEOUNED's own new consumption of this
  primitive, but the actual de-duplication on GEOReverse's side (mirror
  the exact refactor `Gmake_torus_elliptic` went through) is still
  pending; (b) `is_same_elliptic_cylinder`/`is_same_elliptic_cylinder_surface`
  reuse the existing `cyl_distance`/`cyl_angle` tolerances rather than
  getting their own dedicated tolerance fields -- fine for this first
  cut, revisit if a real corpus case needs finer control; (c) the
  MCNP-side GQ output was checked for plausibility (residual math,
  correct call count) but never round-tripped back through
  `MCNP_parser/MCNPinput.py::get_cylinder_parameters` (the REVERSE-side
  elliptic-cylinder classifier already described in "GEOReverse" above)
  to confirm the written coefficients parse back to the exact same
  Center/Axis/MajorRadius/MinorRadius/MajorAxis/MinorAxis -- a natural,
  still-open verification step; (d) no d1suned MCNP stochastic-volume
  check has been run against the real fixture yet, only structural/
  call-count verification.

  **TODO -- generalized surface-to-quadric identification &
  substitution, 2026-09-18 (restated goal, direct user instruction,
  supersedes every narrower "next steps" note above in this entry).**
  The actual purpose of this whole session, per the user's own words:
  for ANY face, regardless of what its native surface type is (NURBS/
  BSpline, `SurfaceOfExtrusion`, `SurfaceOfRevolution`, or anything
  else), check whether it has the characteristics of ANY quadric --
  not just cylinder/sphere/torus, but EVERY quadric already defined
  across GEOUNED and GEOReverse combined (sphere, cylinder, cone,
  torus, elliptic cylinder, elliptic cone, ellipsoid, hyperboloid,
  hyperbolic cylinder/prism, paraboloid, elliptic torus) -- and if it
  does, substitute its definition for the real quadric's, exactly as
  already done this session for cylinder/torus (via BSpline fitting)
  and elliptic cylinder (via exact `SurfaceOfExtrusion` recognition).
  Doing this for each exotic quadric requires first porting it to a
  place shared between GEOUNED and GEOReverse (`geo/{occ,ocp}`), the
  same move already made for `Gmake_torus_elliptic` and (this session)
  `Gmake_elliptic_cylinder`/`GEllipticCylinder`.

  **A. Close out what's already started (cylinder/sphere/torus via
  spline substitution)**
  - [ ] Fix sphere's post-STEP-round-trip substitution failure (the
    half-volume clue: an exact analytic pcurve construction gives a
    valid face at half the correct volume -- a sign/branch bug, not a
    wrong approach). Independent bug from cone -- confirmed unrelated,
    don't assume a shared root cause.
  - [ ] Root-cause cone's `BRepCheck_UnorientableShape` after
    substitution (still completely unaddressed, no production code
    exists for it at all).

  **B. Generalize detection beyond `GeomAbs_BSplineSurface`**
  - [ ] Extend `Gsubstitute_spline_quadrics` (or a new, more general
    mechanism) to operate on ANY unsupported native surface type, not
    only `BSplineSurface`: `SurfaceOfExtrusion`, `SurfaceOfRevolution`,
    and anything else `Gclassify_surface` currently returns `None` for.
  - [ ] Widen the set of candidate quadrics the detector tries beyond
    {cylinder, sphere, torus} to ALL of them (see block C for the full
    list) -- including a "prefer the simplest matching model" tie-break
    analogous to the existing sphere-vs-torus-degenerate one, now
    across a much larger candidate set (e.g. a cylinder is also a
    degenerate elliptic cylinder, MajorRadius==MinorRadius; a sphere is
    also a degenerate ellipsoid; etc. -- these ambiguities need the same
    kind of explicit priority ordering already worked out for sphere
    vs. torus).
  - [ ] Decide, per native surface type, whether EXACT recognition
    (no sampling -- like `GEllipticCylinder`'s `SurfaceOfExtrusion`-of-
    a-`Geom_Ellipse` check) is possible before falling back to sampling
    + fit (like the current BSpline path): a `SurfaceOfExtrusion`/
    `SurfaceOfRevolution` of a KNOWN analytic profile curve (circle,
    ellipse, hyperbola, parabola) should always be recognized exactly,
    the same way elliptic cylinder was -- sampling/fitting should only
    be the fallback for a genuine `BSplineSurface` with no exact basis
    curve to read directly.

  **C. Port each exotic quadric from GEOReverse into `geo/{occ,ocp}`
  (shared), then give it full "base surface" support in GEOUNED**
  For each one, mirror the exact template `GEllipticCylinder` just
  established: move `Gmake_X`/`GX` into `geo/{occ,ocp}/primitives.py`/
  `topology.py` (+ a `Gclassify_surface` branch that recognizes its own
  native OCCT representation exactly, where one exists); `XOnlyParams`/
  `XParams` in `basic_functions_part1.py`; `GeounedSurface` dispatch +
  `build_surface()` branch + a `makeX` in `build_shape_functions.py`;
  `MetaSurfacesDict.add_X` + `SurfacesDict.add_X` (+ a new key in BOTH
  classes' `surfname` lists and their `extend()`/`add_surface()`
  dispatch); `is_same_X` (`basic_functions_part2.py`) + `is_same_X_
  surface` (`geo/surface_geometry.py`) + a `geometry_gu.py`
  `_SAME_SURFACE_PREDICATE` entry (with the occ/ocp-only placeholder-
  class pattern for freecad); `gen_X` + a `cell_definition.py` branch;
  `q_form_X` in `q_form.py` (derived directly from first principles the
  way `q_form_elliptic_cyl` was -- do NOT assume `rotation_matrix(u,v)`
  is reusable, it only pins the axis, not rotation about it, which
  matters for anything with a second, distinguishable in-surface axis);
  an `"XOnly"` branch in all 4 write functions (`mcnp_surface`/
  `open_mc_surface`/`serpent_surface`/`phits_surface`).

  Exotic quadrics still pending (all already exist in GEOReverse, see
  `GEOReverse/Modules/engine_dependency/_{occ,ocp}_impl.py` and this
  entry's own "GEOReverse" section above for each one's construction
  technique):
  - [ ] Ellipsoid (`GEllipsoid`/`Gmake_ellipsoid`)
  - [ ] Elliptic cone (`GEllipticCone`/`Gmake_elliptic_cone`)
  - [ ] Hyperboloid, one AND two sheet (`GHyperboloid`/
    `Gmake_hyperboloid`) -- note `OneSheet` picks the axis/technique,
    see the "classifier-dispatch" entry above for the exact semantics.
  - [ ] Hyperbolic cylinder, the revolution-based "hourglass"
    (`GHyperbolicCylinder`/`Gmake_hyperbolic_cylinder`)
  - [ ] Hyperbolic prism, the flat-extrusion, zero-eigenvalue-axis case
    (`GHyperbolicPrism`/`Gmake_hyperbolic_prism`) -- a genuinely
    different surface from the hyperbolic cylinder above despite the
    similar name, see the "classifier-dispatch mismatch" entry above.
  - [ ] Paraboloid (`GParaboloid`/`Gmake_paraboloid`)
  - [ ] Elliptic torus as a detectable BASE surface in GEOUNED --
    `Gmake_torus_elliptic` already lives in `geo/{occ,ocp}/primitives.py`
    and is already consumed by GEOUNED's own `GeounedSurface.
    build_surface()` for the degenerate-torus CUTTING-TOOL case (see
    "Now consumed by GEOUNED's own forward pipeline too" above) -- but
    that is NOT the same as `Gclassify_surface` recognizing an
    arbitrary face as an elliptic torus and registering it as its own
    base surface the way `GEllipticCylinder` now does. Check whether
    that gap is real before assuming it needs the same amount of new
    code as the others.

  **D. Unify GEOReverse's own local copies once each quadric lands in
  `geo/{occ,ocp}`**
  - [ ] `GEOReverse/Modules/engine_dependency/_occ_impl.py`/
    `_ocp_impl.py` still carry their OWN local `Gmake_elliptic_cylinder`/
    `GEllipticCylinder` -- re-export the now-shared `geo` version
    instead (this session's own "not done in this pass" gap (a) above).
  - [ ] Repeat for every quadric ported in block C, as each one lands.

  **E. Verification still owed for elliptic cylinder (this session's
  own work, not future scope)**
  - [ ] Round-trip the written MCNP GQ card back through `MCNP_parser/
    MCNPinput.py::get_cylinder_parameters` and confirm it recovers the
    same Center/Axis/MajorRadius/MinorRadius/MajorAxis/MinorAxis.
  - [ ] A real d1suned stochastic-volume check against `SOLID015.stp`.
  - [ ] Run `tests/geo/test_freecad_impl.py` for real (not done this
    session, interpreter path wasn't at hand) to confirm freecad is
    genuinely unaffected, not just statically-argued to be safe.
  - [ ] Extend `decom_utils_generator.py`'s `GCone`/`GCylinder`
    adjacent-pair truncation-plane suppression to also cover
    `GEllipticCylinder` (low priority, purely a decomposition-size
    optimization -- current behavior is safe, just not minimal).

  **F. Explicitly out of scope for now (decisions already made, not
  gaps)**
  - Combining any of these base surfaces into composite meta-surfaces
    (RoundCorner/Can/TCone/MultiRoundCorner/ReversedConeCylinder) --
    deferred by direct user instruction.
  - An OBLIQUE extrusion/revolution sweep (profile-curve plane normal
    not parallel to the sweep direction/axis) for any of these -- out
    of scope, matches the already-accepted elliptic-cylinder limitation.

- **Tolerances: intrinsic vs user-modifiable, one notion of "same
  surface", 2026-09-19** (branch `same-surface-tolerances`, commits
  `6d2c9a9`, `b760fa6`, and the measurement/box follow-up). Started as an analysis of why "identical
  surface" had three different criteria: `geo`'s `is_same_*_surface`
  (fixed 1e-5 mm and `dot >= 0.99999`, which is a 4.5e-3 rad angle, 45x
  looser than `Tolerances.pln_angle`), the output-stage `is_same_plane`/...
  driven by `Tolerances` (1e-4, optionally relative), and call sites that
  built a fresh `Tolerances()` and ignored the user's values.
  **The rule (user's, 2026-09-19)**: a value the USER may legitimately
  change (it depends on their model or the output they want) is a field of
  `Tolerances`/`GeoTolerances`; a value intrinsic to the code
  (floating-point floors, kernel query tolerances, algorithm thresholds)
  is a constant in `geo/constants.py` and is never exposed.
  **What was done, in order** (each step verified: `tests/geo` +
  `test_cadtocsg` + `test_csgtocad` + `test_boolean_function` +
  `test_gq_classification` green on all 3 engines, 243/243/214 passed; and
  a differential scan of `Solidos/test_models`, 143 files minus `Big_*`,
  `ocp`, decompose + build_solid_definition, against the pre-change code
  -- pieces, total volume, primitive surfaces, composite counts):
  1. ~320 inline literals became constants named by role+historical value
     (values unchanged, 0 corpus differences). Classification is by an AST
     scan (`Compare` operands, `*tol*` keyword/default arguments,
     positional arguments of native kernel calls such as
     `GeomLProp_SLProps(..., 1e-6)`); factors that are not tolerances
     (mm->cm `* 0.1`, box enlargement `0.2`, `0.95 * dmin`, density
     `< 1e-2`) were deliberately left inline. Several literals had been
     mislabelled as lengths: `cross.length < 1e-3` on unit axes is
     `|sin|` (an angle), `norm < 1e-9` a numerical zero.
  2. `POINT_POINT_TOL = 1e-5` replaces the 1e-5/1e-6/1e-7 point-to-point
     coincidence checks (vertices, apexes, edge endpoints, centres of
     mass). **Measured, not guessed**: a probe recorded the distance
     actually compared at each site over the corpus; real coincidences are
     exactly 0 or < 1e-9 mm and distinct points >= 1e-2 mm (essentially no
     samples in between), so any value in [1e-8, 1e-5] behaves identically
     there. 1e-5 leaves a 10x margin over the rounding of a STEP written
     with 6 decimals and stays 10x below the surface tolerances.
     Box comparisons (`sameBox`, and the 6 overlap-slack sites in
     `geo/*/topology.py`) are NOT point-to-point: OCCT bounding boxes carry
     a few 1e-6 of slop (55 of 386 `sameBox` comparisons and ~15 thousand
     overlap comparisons fall in 1e-6..1e-5), so they first kept their own
     `BOX_TOL_E6 = 1e-6`. Experiment (2026-09-19): with 1e-5 the corpus is
     identical in pieces, volumes, primitive surfaces and composite counts
     and the 3 suites are unchanged, so they now share the value
     (`BOX_TOL = POINT_POINT_TOL`, kept as its own name because box slop
     and point coincidence are different physics and can diverge again).
     Note that this DOES flip decisions (the 55 `sameBox` cases), it just
     does not change any final result on this corpus.
  3. `GeoTolerances` (in `geo/`, so `geo` can import it) holds the fields
     `geo` reads; `geouned.Tolerances` inherits it and keeps its flat
     constructor, so a user always builds ONE object
     (`config["Tolerances"]` unchanged). GEOUNED-only fields (`min_area`,
     `relativeTol`, `add_pln_*`, `distance`/`angle`/`value`) stay in
     `Tolerances`. `tests/geo/test_tolerances_split.py` pins the shared
     defaults so the two classes cannot drift.
  4. The 19 places that built a fresh `Tolerances()` (ignoring
     `CadToCsg(tolerances=...)`) are gone: `tolerances` is threaded as a
     REQUIRED keyword-only parameter through the meta-surface detection
     chain (18 functions: `get_Can`/`get_TCone`/`get_roundCorner`,
     `merge_same_surface_faces`, `other_face_edge(skip_slivers=True)`,
     ...), so a forgotten call site is a `TypeError`, not a silent
     default. `simple_solid_definition` passes the per-solid
     `scaled_tolerances` (as it already did for `get_multiplanes`), which
     changes results only for small pieces under non-default tolerances.
     Direction tests that used `Tolerances().angle`/`.value` are intrinsic
     constants now. `tests/geo/test_no_fresh_tolerances.py` (AST) fails if a
     bare `Tolerances()` reappears in the pipeline (only `core.py`'s public
     constructor and `load_step.py`'s standalone default are allowed).
     The public constructors also no longer share one default
     `Tolerances()` created at import time (same mutable-default bug found
     earlier in `BoxSettings`; `Options()`/`NumericFormat()`/`Settings()`
     defaults of `CadToCsg.__init__` still have it -- not touched).
  5. `is_same_{plane,cylinder,cone,sphere,torus}_surface` (and
     `is_parallel_plane_surface`, `Gmerge_coplanar_planes` on all 3
     engines) take `tolerances` and compare against
     `pln/cyl/kne/sph/tor_distance`+`_angle`; axes are compared as a real
     angle (`atan2`), not `dot >= 0.99999`. The values moved from 1e-5 mm /
     4.5e-3 rad to 1e-4 / 1e-4, with 0 differences on the corpus. In the
     corpus the identity components are bimodal (0 or < 1e-9 vs >= 0.1)
     for cone/cylinder/sphere/torus; only planes have samples in between
     (2 pairs at 1e-5..1e-3 offset, and tilts of 1e-2 rad that the old
     4.5e-3 rad threshold missed by less than a factor 2).
  **Two real bugs fixed on the way**: `Tolerances.relativeTol=True` made
  surfaces at the origin compare as different from themselves (a relative
  tolerance `rel * 0` is 0 and `is_in_tolerance(0, 0, ...)` answers "not
  same"; fixed with a 1e-9 mm floor, `RELATIVE_TOL_ABS_FLOOR`), and the
  `VOLAREA_RATIO` sliver threshold had two values for one concept (1e-2 in
  `solid_ops`, 1e-3 in `constants`; now the 1e-3 one).
  **Measurement of the remaining roles (2026-09-19)**: a throwaway AST
  instrumenter (kept out of the repo, scratchpad) rewrote every single
  `Compare` that mentions a tolerance constant (177 comparisons) so it
  records the value actually compared and its ratio to the threshold; for
  cosine-form thresholds (`|dot| < 1 - tol`) it records the deviation
  `1 - |cos|`. Run over the 143-file corpus (results identical to the
  uninstrumented code). Of 116 comparisons that matter for `ocp`, 86 run
  in the corpus and 30 never do (13 `PARAM_ANGLE_TOL_E5`, 5
  `LENGTH_TOL_E5`, volume floors...): no evidence exists for those.
  - Clean gap (no sample within 10x of the threshold), so any value in the
    gap is equivalent on this corpus: every `ZERO_TOL_*` (373 thousand
    comparisons in `E9` alone; they guard different quantities, so they are
    NOT to be merged), `POINT_POINT_TOL` (26 186 comparisons),
    `LENGTH_TOL_E12`, `VOLUME_MIN_*`, `PARAM_ANGLE_TOL_E5`, and the
    parallel-axes tests in `decom_utils_generator:377`,
    `meta_surfaces_utils:1503/1504`, `functions:329` and
    `geo/surface_geometry:215` (deviations are 0 or < 1e-11, or >= 1e-3).
  - Sensitive (real samples right at the threshold): the direction tests
    have real data from 4.5e-5 to 1.2e-2 rad (`meta_surfaces_utils:1570`
    perpendicularity, |dprod| = 4.5e-5 vs 1e-4; `decom_utils_generator:117`
    sin = 4.8e-6 vs 1e-6; `meta_surfaces_utils:1880` deviation 1.4e-6 vs
    1e-6; `meta_surfaces_utils:1442` sin = 1.2e-2 vs 1e-3), so **a single
    `DIRECTION_ANGLE_TOL` cannot reproduce the current behaviour at both
    `1570` and `117`**: unifying directions changes those branches and is
    an explicit decision, not a refactor. `PARAM_ANGLE_TOL_E4`
    (`meta_surfaces_utils:505`, angular step filter: 2346 of 116 thousand
    near) is algorithmic and stays intrinsic; the volume gates
    (`REL_TOL_E5/E6`) are tied to the user's `volume_tolerance`;
    `AXIS_COS_MIN_E5` in `near_surface_pair` (57 comparisons between 0.26
    and 2.5 degrees) is a near-coincident-surface detector, deliberately
    looser than identity.
  - Relabelled with the same value: `cell_definition_functions:95`
    (`two_pi * (1 - tol)` is a relative angle, now `PARAM_ANGLE_TOL_E5`).
  Known limitation of the method: it measures the corpus, whose STEP files
  are high-precision (coincident points are exactly equal or < 1e-9); a
  low-precision STEP would sit closer to the thresholds, which is why the
  chosen values keep a margin.

  **Second batch: directions, contexts, volumes.** Decisions of the user, each verified by the 3 suites and the
  corpus differential (test_models 143 files + working_solids 30 fixtures,
  0 differences in pieces, volume, primitive surfaces, composite counts vs
  the original 22f0f51 code):
  - **Directions are angles.** Every same-axis/parallel test became an
    angle comparison (`axes_parallel`/`axes_same_direction`/
    `axes_perpendicular` in `geo/surface_geometry.py`, `atan2`-based).
    Perpendicular is `abs(pi/2 - abs(alpha)) < angle_tol` with the SAME
    value as parallel (alpha may be +-pi/2). `DIR_TOL_*`, `AXIS_COS_MIN_*`,
    `ANGLE_TOL_E5`, `LENGTH_TOL_E9`, `NEAR_SURFACE_AXIS_ANGLE`,
    `SAME_SURFACE_AXIS_ANGLE_TOL` are gone. Numerical conditioning, sampled
    flatness, winding and algorithmic margins stay named constants.
  - **Comparison contexts (revised in the third batch, see below).** Surface
    identity was first split in two contexts (faces of one solid with a
    constant `NUMERIC_TOLERANCES`, everything else with the user's values);
    the split was withdrawn for IDENTITY (the user's per-surface tolerances
    apply everywhere) and kept for CONTACT, the full turn of a periodic
    parameter and axes compared with X/Y/Z. `cks_bound_planes` uses
    `Tolerances.distance` (1e-3 -> 1e-4, no regression seen);
    `omit_isolated_planes` uses the user's `pln_angle`; `torus_bound_planes`
    compares its `2*pi` and its axis with `NUMERIC_TOL`.
  - **Defects and repair (load stage).** Kernel-object queries stay as they
    are. `DEFECT_AXIS_ANGLE` (dedicated constant, same value as
    `near_surface_pair`) is the detector's angle. The four volume-
    conservation gates of the repair functions are ONE constant,
    `MAX_REPAIR_VOLUME_REL_CHANGE = 3e-4` (measured: accepted repairs change
    the volume <= ~1e-4 relative, rejected ones >= 5e-4); `Gmerge_coplanar_
    planes` and `Gsplit` keep their own.
  - **One volume pattern.** `volume_within(value, expected, rel_tol,
    reference=None)` (`geo/volume_utils.py`) is `abs(a-b) <= tol *
    max(|ref|, VOLUME_REF)`. `VOLUME_REF = 1 mm^3` is the intrinsic scale
    below which a relative volume comparison makes no sense; it is NOT
    `min_solid_volume`.
  - **One minimum volume.** `Tolerances.min_solid_volume` (default 1e-2
    mm^3, `DEFAULT_MIN_SOLID_VOLUME`) decides every "piece too small"
    discard: `valid_solid(solid, min_volume)`, `Gsplit`'s fragment filter,
    `_raw_bop_split`, `space_decomposition(..., min_volume)`, freecad's
    `check_out_solids`/`remove_solids`. They used to be 1e-2/1e-3/1e-3/1e-3
    (and `Gsplit` applied two in sequence). Measured: no fragment reaches
    those checks with a volume between 1e-3 and 0.1 mm^3, so no decision
    changes; the smallest legitimate piece is 0.072 mm^3
    (`modelcell_cut1_v2_piece66.stp`).
  - Noted, not fixed: `decom_one_generators.split_surfaces` warns "Lost
    ...%" when the compound volume EXCEEDS the original (`volratio` sign).
  **Third batch: stage 3 of the context-by-context review (registry, meta-
  surface detection, `cell_definition`, `SolidGu`).** Analysis first, then the
  user's decisions (2026-09-19). Measured over test_models + working_solids
  (throwaway probes, scratchpad): 110 000 plane comparisons of the registry
  and 76 000 face/edge contact queries.
  - Registry plane identity: the angle (1e-4) sits in a 4-decade clean gap
    (matches <= 3.4e-7 rad, nothing until 1e-2) but the DISTANCE (1e-4) does
    not: 4 matching pairs in (1e-5, 1e-4] and 1 non-matching in (1e-4, 1e-3]
    -- it is a real threshold on this corpus, not noise. Real matches reach
    d = 6.7e-5 mm between different solids, which is what justifies keeping
    the user's tolerance (and not a constant) for identity.
  - Contact (`contiguous_face`, `separate_surfaces`, `commonEdgeFace`,
    `commonVertex`): every distance is exactly 0 or > 1e-2 (nothing in
    (1e-9, 1e-2)), so the value is a matter of principle, not of data.
  - **Decisions.** D1: the sense of a plane in the registry is
    `opposite_sense(a, b)` (sign of the dot product, `geo/surface_geometry`),
    not `is_opposite(., ., pln_angle)`: a plane matched with `add_pln_angle`
    (1e-2) could get its sign decided with `pln_angle` (1e-4); 0 occurrences
    measured. D2: `is_same_surface` and every other surface-identity call
    (also `Gmerge_coplanar_planes`, `_group_coaxial_*`) use the user's
    per-surface tolerances again; `NUMERIC_TOLERANCES` is gone;
    `is_same_cone/sphere/torus` take `tolerances` like plane and cylinder (no
    private 1e-6 defaults). D3: contact between points/edges/faces of one
    solid uses `NUMERIC_TOL` (1e-7; the user will revisit it if a
    low-precision STEP needs it). D4: ONE `merge_periodic_uv` (was duplicated
    with 1e-6 and 1e-5) using `NUMERIC_TOL`, and `angle < pi + NUMERIC_TOL`;
    `Tolerances.relativePrecision` and `.value` are KEPT in the public class
    (JSON configs, docs) but no longer read by anything: the user plans to
    apply absolute/relative precision correctly later. D9: removed an
    accidental `from turtle import distance`.
  - **D6, first tried and withdrawn: "axis vs X/Y/Z uses NUMERIC_TOL".**
    Deciding to write a plane as PX/PY/PZ, a cylinder as C/X.., a cone as
    K/X.. or a torus as TX.. is an APPROXIMATION of the surface by an
    axis-aligned one, i.e. the same decision as "same surface" (dropping
    precision inside the tolerance): it must use the identity's own
    tolerance (`pln/cyl/kne/tor_angle`). With NUMERIC_TOL the corpus showed
    it: real CAD carries direction noise of ~5e-7 rad (7-digit direction
    cosines, above 1e-7), so `2_degen_torii.stp` lost a torus from its cell
    (its axis is 5.2e-7 rad off Z; `gen_torus` returned None) and 10 files
    wrote GQ/P instead of C/Y, PZ, K/X (62 of 557 surfaces of `L4-WCS_4`).
    Now: the 4 writers use the branch's own tolerance, `gen_torus` and the
    torus V/U bounds `tor_angle`, `cone_apex_plane` `kne_angle`,
    `remove_box_faces` `pln_angle`, the registry buckets `pln_angle`;
    `tests/geo/test_numeric_contexts.py` pins that each flips exactly where
    the identity flips. `torus_bound_planes` keeps NUMERIC for its `2*pi` and
    for the edge-circle axis vs the torus axis (not a comparison with X/Y/Z).
    Caveat found on the way: `NUMERIC_TOL = 1e-7` is BELOW the angular noise
    of real data, fine for distances/contact (0 or > 1e-2 on the corpus).
  - **`__eq__` of the surface params removed (D5).** `PlaneParams`,
    `MultiPlanesParams` and `GeounedSurface` no longer define `__eq__` (it
    had constants -- 1e-6 mm, 1e-4 rad -- no access to a `Tolerances`, and
    was called implicitly by `in`, `.index`, `!=`). Census: a runtime probe
    over test_models + working_solids plus an AST scan of every `==`, `!=`,
    `in`, `.index`, `.remove` found 12 sites: 9 statements in
    `build_roundC_params` (`functions.py`, now via `_index_of_plane`), 1 in
    `get_cell_object` (`build_region.py`, which now takes `tolerances`) and 1
    in `add_roundCorner`'s region (`geouned_classes.py`). They use
    `surface_geometry.is_same_oriented_plane_surface(a, b, tolerances)` = same
    plane (user `pln_distance`/`pln_angle`) AND same sense. The distance went
    from 1e-6 to `pln_distance` (1e-4 by default); the corpus and the written
    text did not change. Method: first an `__eq__` that raised, run over the
    suites and both corpora (nothing raised), then deleted; a test pins that
    none is defined again. Anything comparing these objects with `==`/`in` now
    compares identity, so new code must call the predicate explicitly.
  - **`spline_2D` and the edge-curve tests (2026-09-20).** `spline_2D`
    (is this BSpline edge planar?) compares the binormal at every knot with
    the first one; it now uses the intrinsic constant
    `SPLINE_PLANARITY_ANGLE = 1e-3` rad (it was `is_parallel`'s default, same
    value) and takes no tolerance. Measured (187 splines): the 32 planar ones
    lie in [0, 1e-3] (17 at <= 1e-12, 15 in (1e-8, 1e-3]) and the 155
    non-planar ones start right above (27 in (1e-3, 1e-2]). `same_curve` /
    `planar_edges` were measured too: `planar_edges` only ever reaches its
    multi-edge comparison with circles (13 calls) or splines (10), never with
    lines, so the suspicion about `curve.Position` on lines is unreachable on
    the corpus (all-line boundaries end in False anyway); `same_curve`
    compares circles whose centres and radii differ by <= 1e-8 against
    `LENGTH_TOL_E5`, axes exactly parallel or > 0.1 rad apart. No change
    made there.
  - **Plane identity is measured from a point of the plane (the user,
    2026-09-21).** `is_same_plane_surface` no longer compares the offsets
    from the coordinate origin (`Axis . Position`); it measures
    `|plane_1.Axis . (plane_2.Position - plane_1.Position)|` against
    `pln_distance` (after the parallelism test with `pln_angle`). Reason: a
    plane's `Position` is a point in the region where its face is defined,
    so the planes are compared where they are used; the origin-based offset
    added `angle * distance-to-the-origin` (1e-4 rad at 1 m = 0.1 mm) to what
    is a local difference and said nothing about where each face is. It is
    independent of the senses of the axes (antiparallel planes are handled
    by the `abs`), and two parallel planes 3.5 apart (`rc9.stp`) still do not
    match. Note: `plane_1` is the reference, so for planes tilted by up to
    `pln_angle` the result can differ by about `pln_angle * |P2 - P1|` when
    the arguments are swapped. Also affects
    `is_same_oriented_plane_surface`. Verified: suites (ocp/occ 298, freecad
    269) and, against the original 22f0f51 code, decomposition and written
    text (5 formats) over the 173 files of test_models + working_solids that
    existed before: 0 differences. The new fixture
    `working_solids/plates_plane.stp` is the one that changes: MultiPlane
    3 -> 1 (4 pieces and 19 primitive surfaces unchanged, volume 221498.599
    -> .602); not checked with d1suned. NOT changed (as of this entry): the
    registry's `is_same_plane` (`basic_functions_part2.py`, with
    `relativeTol`, fuzzy logging and `add_pln_*`), which still compares the
    offsets from the origin -- the two notions differ again.
  - **Held back / open.** (a) `same_curve`/`planar_edges` keep
    `LENGTH_TOL_E5` (same value as `POINT_POINT_TOL`, a possible relabel).
    (b) `relativeTol` stays as it is (the user, 2026-09-20): with
    `relativeTol=True` the cylinder-axis tolerance scales with `|Center|`, an
    arbitrary point of the axis. (c) the writers decide CX vs C/X with an
    exact `Pos.y == 0.0`.
  - **Duplicated functions: census, plan, and step 1 (surface identity)
    done (2026-09-21/22).** After the point-based plane identity the user
    noticed the registry has its own `is_same_plane` (`basic_functions_part2.py`)
    next to `geo.is_same_plane_surface`. Census (by name, by structural
    similarity of functions >= 40 nodes, by concept; small functions with
    different names may have escaped): (1) surface identity, registry
    (15 call sites) vs `geo` (12): plane origin-based vs point-based, the
    registry's `relativeTol`/fuzzy log/`add_pln_*`, and tori with
    antiparallel axes (different in the registry, same in `geo`); (2)
    `geo.is_parallel/is_opposite` next to `axes_parallel/opposite_sense`,
    plus GEOUNED wrappers in `basic_functions_part1` (`is_same_value`,
    `is_parallel`, `is_in_line` -- with a hidden 1e-3 rad default --,
    `is_in_plane`), ~25 call sites; (3) contact (`contiguous_face`,
    `commonEdgeFace`, `commonVertex`, `separate_surfaces`) = one
    `distance <= NUMERIC_TOL` written four times; (4) the axis-family
    classification repeated in the 4 writers (48 tests); (5) beyond
    tolerances: `redundant`/`outer_terms`/`remove_redundant`/`countP` in 3
    places (`boolean_utils`, GEOReverse `remh.py`, GEOUNED
    `string_functions.py`) that have DIVERGED (12/95/83 differing lines),
    the GEOReverse MCNP/XML parser twins, the exotic-quadric `is_inside`
    copied in 3 engine files, `points_to_coeffs` x2.
    Decided: two tori with antiparallel axes ARE the same surface; the plane
    distance is point-based; `relativeTol` untouched for now. Agreed order:
    (1) identity as ONE `geo` implementation computing deviations, the
    registry a thin wrapper that only adds its effective tolerances and the
    fuzzy log (keeping `check_a_sign`, needed by `SolidGu.same_torus_surf`
    to never merge a degenerate torus's two sheets); (2) parallel/sense;
    (3) one contact predicate; (4) one axis-family helper for the writers;
    (5) the non-tolerance duplicates one at a time, each with an equivalence
    test first.
    **Step 1, done 2026-09-22**: `geo.surface_geometry` is now the ONE
    implementation for all 5 `is_same_*_surface` predicates, including
    `relativeTol` (read via `getattr(tolerances, "relativeTol", False)` --
    `GeoTolerances` itself still has no such field, so GEOReverse's bare
    instances default to absolute, unaffected) and, for the torus, an opt-in
    `check_a_sign` parameter (default `False`, matching the registry's own
    "both sheets of a degenerate torus write as one surface"; `True` only
    from `SolidGu.same_torus_surf`, which must keep them apart). `RELATIVE_TOL_ABS_FLOOR`
    moved from `GEOUNED/utils/data_constants.py` to `geo/constants.py`; a
    new `relative_tolerance(base, scale)` helper is shared by every
    predicate. `geo` also exposes the lower-level pieces the registry needs
    without re-deriving the geometry: `plane_offset`/`plane_within` (explicit
    angle/distance tolerances, so the registry can pass `add_pln_*` for a
    non-real plane without a tolerances-shaped proxy), `cylinder_radius_diff`/
    `cylinder_axis_offset`. `basic_functions_part2.is_same_plane`/
    `is_same_cylinder` are now thin wrappers: the DECISION comes from
    `plane_within`/`is_same_cylinder_surface` (called once), and the near-miss
    diagnostic log (`fuzzy=`, written to `fuzzy_logger`, never read back by
    the pipeline) is computed separately, only when requested, from the same
    shared primitives -- the radius fuzzy-check still fires independently of
    the axis test, matching the original control flow.
    `basic_functions_part2.is_same_cone/is_same_sphere/is_same_torus` are
    DELETED outright (no fuzzy-log call site ever passed `fuzzy=` for these
    three, so there was nothing left to wrap): `MetaSurfacesDict`/`SurfacesDict`'s
    `add_cone`/`add_sphere`/`add_torus`/`get_id` and `SolidGu.same_torus_surf`
    now call `geo.surface_geometry.is_same_cone_surface`/`is_same_sphere_surface`/
    `is_same_torus_surface` directly.
    **Verified**: suites ocp/occ 299 passed/2 skipped, freecad 270 passed/16
    skipped; against the original 22f0f51 code, decomposition and written
    text (5 formats) over the 143 test_models + 30 working_solids files:
    **1 file changes**, `Complex_cell/SCDR_90.stp` (`prim_surfaces` 17 -> 16,
    one plane fewer; pieces/volume/every composite count unchanged) -- two
    near-antiparallel planes (tilt ~1e-6 rad) that used to be kept apart by
    the registry's old origin-based offset (a ~9.65 cm difference measured
    from the origin, for faces that are actually coincident where they are
    defined) now correctly merge, exactly the effect the point-based change
    was for. Confirmed harmless with a direct d1suned check (`verify_one_solid.py`,
    NPS 1e6): the ORIGINAL and the NEW code give the IDENTICAL tally
    (0.99842 +/- 0.23%, 0.7 sigma, SD4 matches the true CAD volume, 0 lost
    particles) -- the extra plane the original code kept was geometrically
    redundant (implied by the cell's other surfaces), so removing it changes
    nothing about the real solid, only the written definition's size.
    `plates_plane.stp` (the new working_solids fixture, no original-code
    baseline) is unaffected by this step (its own MultiPlane 3 -> 1 change
    is from the point-based `geo` change of the previous session).
    Tests: `tests/geo/test_same_surface_tolerances.py` updated (the 3 deleted
    registry names replaced by the `geo` ones; the antiparallel-torus test
    flipped to assert equality; a new `check_a_sign` test); `test_numeric_contexts.py`'s
    cone/torus writer tests updated to import from `geo`.
    **Step 2 (parallel/sense), done 2026-09-22**: `geo.is_parallel`/
    `is_opposite` (the old, single-tolerance-default pair) and their
    `GEOUNED.utils.basic_functions_part1` wrappers (`is_parallel`, plus
    `is_in_line`/`is_in_plane`/`sign_plane`, which also delegated to
    `surface_geometry`) are DELETED. Found, while tracing every call site
    before touching anything, that `is_opposite`/`is_in_line`/`is_in_plane`/
    `sign_plane` were already 100% dead across the whole repo (zero callers
    anywhere, including GEOReverse -- `is_opposite`'s own last live callers
    were migrated to `opposite_sense` back in the D1 step of the FIRST
    tolerances batch): not really a "duplicate to unify" any more, just
    confirmed-dead code found while looking here, removed the same way this
    project has consistently treated dead code elsewhere (e.g. the old
    `box_intersect`/`plane_region`/`operate_box`). `is_parallel` had exactly
    6 remaining call sites, all in `geouned_classes.py` (the plane-bucket
    dispatch, `PX`/`PY`/`PZ` vs axis and `get_id`'s equivalent), all already
    passing an explicit tolerance (`self.tolerances.pln_angle`) -- switched
    directly to `surface_geometry.axes_parallel` (module already imported
    there), so there was no hidden-default behaviour to preserve or lose.
    `geo/__init__.py`'s re-export list and `tests/geo/test_vector_geometry.py`
    updated to match (8 tests for the deleted functions replaced by 1 for
    `axes_parallel`/`opposite_sense`, which already have their own dedicated
    tests in `test_same_surface_tolerances.py`).
    **Verified**: suites ocp/occ 292 passed/2 skipped (299 minus the net 7
    tests removed), freecad 263 passed/16 skipped; against the original
    code, the SAME single diff as step 1 (`SCDR_90.stp`, already explained
    and confirmed harmless with d1suned there) -- zero NEW differences, i.e.
    `is_parallel` -> `axes_parallel` is exactly behaviour-preserving on this
    corpus (the two differ only in `<` vs `<=` at the exact tolerance
    boundary, never hit here).
    **Step 3 (contact), done 2026-09-22 -- narrower than the original census
    entry.** The census had lumped `contiguous_face`/`commonEdgeFace`/
    `commonVertex`/`separate_surfaces` together as "the same
    `distance <= NUMERIC_TOL` written four times"; reading each one found
    that's only true for HALF of them:
    - `commonEdgeFace`'s own face-level pre-filter and `SolidGu.separate_surfaces`'s
      pairwise grouping check both genuinely compare `FaceGu.distToShape(...)
      < NUMERIC_TOL` (both already go through the identical
      `GFace.my_distToshape` underneath) -- a real, safe duplicate. Factored
      into one `geometry_gu.faces_touch(face1, face2)`, used by both. Pure
      deduplication, confirmed zero behaviour change (identical formula,
      identical constant already).
    - `contiguous_face` and `commonVertex`'s own pre-filters are NOT the same
      mechanism, even though both nominally check "close within NUMERIC_TOL":
      `contiguous_face` calls `GEdge.my_distToshape` (BoundBox slack =
      the fixed `BOX_TOL`, then a real edge-edge distance or, if the boxes
      don't overlap, a coarse box-centre-distance); `commonVertex` calls
      `shapes_in_contact` -> `Gin_contact` (BoundBox slack = the CALLER's
      own tolerance, i.e. `NUMERIC_TOL` itself -- 100x tighter than `BOX_TOL`
      -- then a real `BRepExtrema_DistShapeShape` query). Left as two
      separate mechanisms: `commonVertex` turned out to have ZERO real
      exercises anywhere in the 173-file corpus (its one caller,
      `functions.py`'s MultiPlane vertex computation, never reaches it on
      this data), so there is no evidence either way whether swapping it to
      `my_distToshape` would ever change a real decision -- not changed
      without data, per this project's own "measure, don't guess" rule.
    - Separately noted, NOT changed: `separate_surfaces` still does the
      whole-FACE `distToShape` (which for overlapping bounding boxes runs a
      real `BRepAlgoAPI_Common`) where `contiguous_face` already switched to
      the cheaper edge-to-edge `my_distToshape` for the identical kind of
      task (grouping fragments of one analytic surface into connected
      pieces) after a real, documented profiling bottleneck on
      `hylife-v06.stp`. Measured: every torus group `separate_surfaces`
      processes in the corpus has <= 5 faces (170 groups of 1, 27 of 2, 6 of
      3, 2 of 5) -- the O(n^2) whole-face cost is never actually exercised
      at a scale that would matter here, so this is flagged as a possible
      future optimisation, not acted on (no evidence of a real problem, and
      changing the algorithm -- not just its name -- is a different kind of
      change than the deduplication this step is about).
    **Verified**: suites ocp/occ 292 passed/2 skipped, freecad 263 passed/16
    skipped; against the original code, the SAME single diff as steps 1-2
    (`SCDR_90.stp`), zero new differences.
    **Step 4 (writer axis dispatch), done 2026-09-22.** All 4 writers
    (`mcnp_surface`/`open_mc_surface`/`serpent_surface`/`phits_surface`,
    `GEOUNED/write/functions.py`) turned out to genuinely repeat the exact
    same 3-way axis test for Plane/CylinderOnly/ConeOnly/TorusOnly (16 sites,
    48 individual tests) -- unlike step 3's contact functions, this one
    really was 4 copies of the same decision: verified every writer uses
    `axes_same_direction` (not `axes_parallel`) for Plane specifically (the
    written card's own classification, not just its text, depends on which
    way the normal points) and `axes_parallel` for Cylinder/Cone/Torus
    (only the axis LINE matters there) -- CONSISTENTLY across all 4, so this
    distinction is a real, deliberate rule, not an inconsistency, and the
    new shared helper takes it as an explicit `same_direction` flag rather
    than guessing one behaviour for everyone. New
    `geo.surface_geometry.axis_alignment(direction, angle_tol,
    same_direction=False)` returns which cartesian axis (0/1/2) `direction`
    is aligned to, or `None` -- each writer now computes it once per surface
    and dispatches on the index; the actual per-axis FORMATTING (the
    genuinely different part -- PX/CX/KX/TX vs OpenMC's `x-plane`/
    `XPlane`/... vs Serpent's `px`/`cylx`/... vs PHITS' own MCNP-like
    syntax) is untouched, per the agreed "one axis-family helper, each
    writer only formats". Applied via a script processing each writer's
    function body in isolation (not a blind file-wide find/replace) because
    `mcnp_surface`/`open_mc_surface` share the exact same variable names
    (`Dir`, `tolerances`) as each other, and a global substitution would
    have silently cross-matched between them.
    **Verified**: suites ocp/occ 292 passed/2 skipped, freecad 263
    passed/16 skipped; against the original code, the SAME single diff as
    steps 1-3 (`SCDR_90.stp`), zero new differences -- including in the
    WRITTEN text of all 5 formats, confirming the 16-site substitution is
    exactly behaviour-preserving.
    **Step 5 (non-tolerance duplicates), narrowed after investigation, one
    safe piece done 2026-09-22, the rest deliberately left.** The census
    entry's own boolean-string sub-item needed the same correction as steps
    1 and 3: `outer_terms`/`redundant`/`is_integer` were already unified
    2026-09-12 into `boolean_utils/boolean_expression_parser.py` (its own
    docstring says so); what the structural scan actually found were 2
    genuinely different things wearing the same names:
    - `GEOUNED/write/string_functions.py`'s own `redundant(m, geom)` was
      confirmed BYTE-IDENTICAL to the canonical one (only cosmetic variable
      renames, `left_ok`/`right_ok` vs `leftOK`/`rightOK`) -- a real, safe
      duplicate. Replaced with an import from `boolean_utils`; its own copy
      deleted. `remove_redundant` (which has no canonical twin -- it is a
      genuinely different, string_functions-only operation: simplifying a
      GEOUNED-written geometry string for output, not parsing MCNP input
      text into a `BoolSequence`) is untouched, still calls `redundant`.
    - `GEOReverse/Modules/MCNP_parser/remh.py`'s own free `redundant(m,
      geom)` and its `Cline.outer_terms`/`Cline.remove_redundant`/
      `Cline.countP` METHODS are a genuinely different, coupled layer, NOT
      simple duplicates of the free functions: `remh.py`'s `redundant` has
      a real extra check the canonical one lacks (`#`/hash-complement-
      operator awareness, needed by its own `complementary()` text
      transform); `Cline.remove_redundant` carries GEOReverse-specific
      state (comment removal/restoration, a before/after paren-count diff
      in `self.removedp`, an MCNP-complex-cell-wrapping option). Left
      entirely as-is: forcing these into the shared free-function shape
      would be a real `Cline`-architecture refactor, not a deduplication,
      and touches the same historically fragile, load-bearing text-parsing
      code this project has explicitly been cautious with before (see the
      BoolSequence unification entry above).
    - The remaining census sub-items (GEOReverse MCNP/XML parser twins,
      the exotic-quadric `is_inside` copied in the 3 engine files,
      `points_to_coeffs` in `MCNPinput.py` vs `basic_functions_part1.py`)
      are ALL entirely or mostly GEOReverse-side code -- out of scope per
      this project's own standing priority ("GEOReverse debugging work in
      general is deliberately paused... do not start GEOReverse-side
      debugging unless asked", see "Current status" above). Not
      investigated further; flagged here instead of silently expanding
      into paused work.
    **Verified**: suites ocp/occ 292 passed/2 skipped, freecad 263
    passed/16 skipped; against the original code, the SAME single diff as
    steps 1-4 (`SCDR_90.stp`), zero new differences.
    **This closes the duplicated-functions unification for now** -- every
    item that was genuinely GEOUNED-side and genuinely a duplicate (steps
    1-4, plus this one small piece of step 5) is done; what remains is
    either GEOReverse-side (paused) or not a real duplicate once read
    carefully.
  **Stage 4 (void generation), done 2026-09-22.** Read `GEOUNED/void/void.py`
  + `void_box_class.py` in full to find every tolerance-bearing decision.
  Only one real one: `VoidBox.piece_enclosure_split` (only reached when a
  real nested `EnclosureList` exists -- confirmed via `PieceEnclosure`,
  set only from `Enclosure.CADSolid` in `get_void_def`) reused
  `KERNEL_TOL_E13` (documented role: "tolerance handed to a CAD-kernel
  operation") for two genuinely different geometric decisions: a relative
  contact test (`dist/Box.DiagonalLength > Tolerance`) and a relative
  containment test (`(cube_volume-common_volume)/cube_volume <=
  Tolerance`) -- the same role-mismatch pattern found repeatedly
  elsewhere in this review. `options.enlargeBox` (`get_void_complementary`)
  confirmed NOT a tolerance (a box-enlargement margin, already a user
  `Options` field) -- no action needed.
  **Measured**: found a real fixture, `Solidos/test_models/Enclosures/
  w_encl.stp` (a genuine `enclosure1_0_` STEP label). With default
  settings the model is too small to ever split (`maxSurf`/`maxBracket`
  never exceeded, 0 calls). Forced splitting (`maxSurf=1`, `maxBracket=1`,
  `minVoidSize` scaled to the enclosure's own ~430 mm bbox diagonal, not
  the 1 mm first tried -- that value, far below the model's real scale,
  caused runaway near-exponential bisection) gave 24 real calls, ALL
  landing at EXACT `0.0` for both the distance ratio and the volume
  ratio: this particular enclosure is close to box-shaped, so every
  candidate sub-box is either exactly touching or exactly fully
  contained -- no sample anywhere near the `1e-13` threshold. No usable
  data to pick a different value; a curved/irregular synthetic enclosure
  would be needed to get non-trivial samples, which would mean
  fabricating data rather than measuring real usage -- against this
  session's own rule, so not done.
  **User's decision**: the value IS a double-precision arithmetic-zero
  floor, not really a "kernel operation" parameter -- renamed
  `KERNEL_TOL_E13` to `NUMERIC_DOUBLE_TOL` (value unchanged, 1e-13),
  pulled out of the `KERNEL_TOL_E3/E6/E7/E8` family in
  `geo/constants.py` with its own docstring. Applies at BOTH of its real
  call sites: `geo/freecad/split.py`'s split-retry floor (unchanged
  role, just renamed) and `piece_enclosure_split`'s two tests (renamed,
  no `volume_within` refactor -- the user judged the existing plain
  relative-ratio form already correct for a "is this exactly zero"
  check, `volume_within`'s `VOLUME_REF` floor being for a different kind
  of comparison).
  **Dead code removed**: `VoidBox.refine()` and `remove_extra_comp`'s
  `mode="dist"` branch -- confirmed 100% dead (the one call to `.refine()`
  is commented out in `void.py`) -- deleted, matching this session's
  consistent practice with other confirmed-dead code
  (`is_opposite`/`is_in_line`/`is_in_plane`/`sign_plane`,
  `box_intersect`/`plane_region`/`operate_box`). `remove_extra_comp` lost
  its now-single-valued `mode` parameter entirely (its one real caller,
  `VoidBox.__init__`, already only ever used `mode="box"`).
  **Verified**: suites ocp/occ 292 passed/1 skipped, freecad 263
  passed/16 skipped (identical to the pre-change baseline -- a pure
  rename plus deletion of code with zero live callers cannot change any
  test outcome); the `w_encl.stp` enclosure probe re-run after the
  rename gives the bit-identical 24 calls, all `(0.0, 0.0)`; a plain
  default-settings end-to-end run of `w_encl.stp` (`run()` +
  `export_csg`) completes cleanly. No corpus differential needed for the
  void stage specifically -- the existing `write_worker.py` scratchpad
  script runs with `voidGen=False`, so it never reaches this code path
  either before or after; this stage's own real-fixture check above is
  the meaningful verification.
  **Stage 5 (no-overlap), done 2026-09-22.** Read `conversion/
  cell_definition.py::noOverlapCell`/`process_overlap` (only reached with
  `options.forceNoOverlap=True`, default `False`, never enabled anywhere
  in the existing test suite or corpus scripts) and
  `GeounedSolid.check_intersection` (used at load time for enclosure
  containment/overlap checks, and by `void_functions.py::assignEnclosure`
  -- both reachable via a real enclosure fixture). Three findings:
  1. `check_intersection`'s `dtolerance=LENGTH_TOL_E6` parameter was
     confirmed 100% dead: it never appears in the function's own body
     (only in the signature and in a comment describing the OLD,
     Gdistance-based early-out this function used before the 2026-08-24
     fix -- `BoundBox.intersects()`, which replaced it, takes no
     tolerance at all), and none of its 3 real call sites pass it.
     Deleted, matching this session's practice with other confirmed-dead
     parameters/branches.
  2. `check_intersection`'s real tolerance, `vtolerance=ZERO_TOL_E10`
     (the relative-volume-embedding test), measured via `w_encl.stp` (5
     real calls): the same clean-gap pattern as everywhere else in this
     review -- values are exactly `0.0` or of order 1-20, nothing near
     `1e-10`. No evidence to change the value; left as-is.
  3. `noOverlapCell`'s call to `shapes_in_contact(m.CADSolid,
     other_cell.CADSolid)` used that function's default,
     `tolerance=LENGTH_TOL_E6` -- a genuinely different role (contact
     between two DIFFERENT cells' solids, deciding whether an automatic
     complementary cut is needed) from `LENGTH_TOL_E6`'s many other,
     unrelated "small length/degenerate" roles elsewhere in the
     codebase. No fixture gives real data for this specific decision (see
     below). **User's decision**: reuse `NUMERIC_TOL` here instead --
     `shapes_in_contact`'s default changed from `LENGTH_TOL_E6` to
     `NUMERIC_TOL`, which also makes it consistent with its OTHER real
     call site (`meta_surfaces_utils.py:1263`, which already passed
     `NUMERIC_TOL` explicitly) -- both call sites now agree.
  **A real, pre-existing, unrelated bug found while trying to measure
  point 3 empirically (not fixed, out of scope for a tolerance review --
  flagged for whenever GEOUNED debugging is picked up)**: loading ANY
  STEP file with a real enclosure label (`w_encl.stp`) with
  `settings.voidGen=False` crashes `core.py::build_void()`'s "Cleaning
  definition" loop (`core.py:528`,
  `AttributeError: 'list' object has no attribute 'level'` -- some
  cell's `Definition` is a bare `list`, not a `BoolSequence`) --
  reproduces identically with `forceNoOverlap` `True` or `False`
  (confirmed unrelated to this stage's own change), and confirmed via
  `git stash` to already exist on the pre-change code, so definitely not
  introduced by this session. Since it happens inside `build_void()`
  BEFORE `no_overlap_cell()` is ever reached (`core.py::run()` calls
  `no_overlap_cell()` only after `build_void()` returns), `w_encl.stp`
  can never be used to exercise `noOverlapCell`/`shapes_in_contact` at
  all -- a fixture with NO enclosure but two genuinely touching/
  overlapping solid cells would be needed instead, not built this
  session.
  **Verified**: suites ocp/occ 292 passed/1 skipped, freecad 263
  passed/16 skipped (unchanged baseline -- a dead-parameter deletion and
  a default-argument swap between two already-established constants
  can't move any existing test's outcome, and neither is exercised by
  the suite at all, per the finding above).
  **Stage 6 (write), investigated 2026-09-22 -- nothing to change.** Read
  `write/functions.py`, `write_files.py`, `string_functions.py`,
  `mcnp_like/*.py`, `openmc/openmc_format.py`, `utils/q_form.py` in full.
  Every tolerance-bearing decision in the write stage was already
  correctly handled by earlier work: the 16 axis-classification sites
  were unified into `axis_alignment` in step 4 of the duplicated-
  functions plan; the 2 "is this sphere centered at the origin"
  checks (`pnt.is_equal(GVector(0,0,0), tolerances.sph_distance)`,
  writing `SO` vs `S`) already use `sph_distance` -- the sphere's own
  identity tolerance, exactly matching the D6 rule ("approximating a
  surface as axis/origin-aligned uses that surface's own identity
  tolerance") established earlier in this review. The only other
  numeric literals in the write stage (`* 1e-3` for a volume unit
  conversion, `< 1e-2` for a near-zero-density check) were already
  explicitly excluded from this whole review from its very first batch
  ("factors that are not tolerances... density < 1e-2... deliberately
  left inline"). `q_form.py` and `string_functions.py` have no
  tolerance literals at all. No code change, no verification needed.
  **This closes the void/no-overlap/write context-by-context review.**
  What remains open, not part of this review's original scope, is the
  handful of individually-flagged lengths from earlier batches:
  `LENGTH_TOL_E7` (a freecad-only kernel-API `isInside` query
  parameter), `LENGTH_TOL_E8` (`surface_geometry.torus_sheet_sign` + 3
  sites in `utils/build_shape_functions.py`, alongside `KERNEL_TOL_E8`),
  and the degenerate-edge family `LENGTH_TOL_E5` (including
  `same_curve`/`planar_edges`'s own use of it, flagged earlier as
  "same value as `POINT_POINT_TOL`, a possible relabel" but never
  measured).
  **`GPlane.intersect_plane`/`intersect_line` (all 3 engines),
  2026-09-22.** Read alongside the user, who classified every tolerance
  in both methods directly: `ZERO_TOL_E10` (the `dl < ...` cross-product-
  length-near-zero test in `intersect_plane`, deciding "are these two
  planes parallel") and `ZERO_TOL_E12` (the `abs(denom) < ...` dot-
  product-near-zero test in `intersect_line`, deciding "is this line
  parallel to the plane") are BOTH the same `NUMERIC_DOUBLE_TOL` concept
  as `KERNEL_TOL_E13`'s own 2026-09-22 rename (stage 4 above) -- a
  double-precision arithmetic-zero floor -- so both call sites (6 total,
  2 per engine) now use `NUMERIC_DOUBLE_TOL` (1e-13) directly, a real
  tightening from their previous 1e-10/1e-12. `ZERO_TOL_E12` became
  fully unused anywhere in the repo after this and was deleted from
  `geo/constants.py`; `ZERO_TOL_E10` stays (still used by
  `GLine.intersect_line`'s own, separate `crl < ...` check, not
  discussed/touched here, and by `GeounedSolid.check_intersection`'s
  `vtolerance` default from stage 5). `ANGLE_TOL_5E2` (the
  well-conditioned-math vs. native-fallback method-selection threshold
  in `intersect_plane`) was confirmed NOT a tolerance -- an algorithmic
  threshold, correctly already a bare intrinsic constant, no
  `Tolerances` field, left untouched. `KERNEL_TOL_E7` (the native
  `GeomAPI_IntSS` intersector tolerance in the occ/ocp near-parallel
  fallback branch) is a raw OCC/OCP kernel-API parameter introduced
  early in the pyOCC migration -- left as-is for now, per the user
  ("no controlo... lo dejamos asi por ahora").
  **Verified**: suites ocp/occ 292 passed/1 skipped, freecad 263
  passed/16 skipped (unchanged baseline); a full before/after corpus
  differential (143 non-`Big_*` files of `Solidos/test_models`,
  `rich_worker.py`: pieces/volume/primitive-surface/composite counts,
  the current git HEAD vs. the live edited tree) -- **0 differences**,
  confirming the 1e-10->1e-13 and 1e-12->1e-13 tightening changes no
  real decomposition outcome on this corpus (both methods'
  well-conditioned math already only activates once past
  `ANGLE_TOL_5E2`, and real plane/line pairs in the corpus are either
  clearly non-parallel or land in that intermediate zone -- none happen
  to sit between the old and new zero-floors).
  **`ZERO_TOL_E10` eliminated entirely, 2026-09-22** (direct follow-up,
  same session): the user asked to trace every remaining call site and
  replace each with the constant that actually represents its meaning,
  rather than keep a generic catch-all name alive. Two more real sites
  found: `GLine.intersect_line`'s own `crl < ZERO_TOL_E10` (all 3
  engines) is the exact same cross-product-length-near-zero
  "parallel?" test as `GPlane.intersect_plane`'s (just fixed above) --
  swapped to `NUMERIC_DOUBLE_TOL`. `GeounedSolid.check_intersection`'s
  `vtolerance=ZERO_TOL_E10` default (stage 5 of the void/no-overlap/
  write review, already measured there: same "exact 0.0 or clearly
  not" clean-gap pattern as everywhere else) is the same concept
  applied to a relative-volume-embedding test instead of a direction
  vector -- also swapped to `NUMERIC_DOUBLE_TOL`. With all 4 real call
  sites (2 in `GPlane`, 1 in `GLine`, 1 in `check_intersection`) moved,
  `ZERO_TOL_E10` had zero remaining references anywhere in the repo and
  was deleted from `geo/constants.py`.
  Also cleaned up in the same pass: the user directly deleted
  `surface_geometry.torus_sheet_sign`'s own dead `tol: float =
  LENGTH_TOL_E8` parameter (confirmed never read in the function's
  body, a leftover default -- same pattern as `check_intersection`'s
  already-removed `dtolerance`); its now-orphaned `LENGTH_TOL_E8`
  import was dropped from `surface_geometry.py` (the constant itself
  stays defined -- still 3 real uses in
  `GEOUNED/utils/build_shape_functions.py`).
  **A real, likely bug noticed while confirming `check_intersection`'s
  real callers, NOT fixed, flagged for later**: `load_functions.py::
  check_enclosure`'s `same_parent = dict()` is reset INSIDE the `for
  encl in level:` loop (right before it's read by `check_overlap`) --
  so by the time `for encl in same_parent.values(): check_overlap(encl)`
  runs, each list holds only the LAST enclosure processed at that
  level, never the full sibling group `check_overlap`'s own pairwise
  loop (`enclosures[i+1:]`) needs to actually compare anything.
  Confirmed this doesn't affect `check_intersection`'s own real-call
  verification above (`w_encl.stp` only has 1 enclosure, so
  `check_overlap`/the `not_embedded` chain walk were never exercised
  there regardless -- the 5 real calls measured all came from
  `assignEnclosure` instead, the third, unconditional-on-siblings call
  site). Not verified against a real multi-enclosure fixture; not
  fixed.
  **Verification**: per direct user instruction, no test re-run or
  corpus diff for this follow-up (unlike the `intersect_plane`/
  `intersect_line`-in-`GPlane` change just above, which WAS a real
  1e-10/1e-12 -> 1e-13 tightening verified with the 3 suites + a
  143-file corpus differential -- this extension carries the same kind
  of value change, at `GLine.intersect_line` and `check_intersection`,
  just not independently re-verified at the time). Covered
  retroactively by the combined batch differential below.
  **The rest of the `ZERO_TOL_*` family eliminated, `NUMERIC_DOUBLE_TOL`
  retuned to 1e-12, `PARAM_ANGLE_TOL_E4/E5` merged, 2026-09-22 (same
  session, direct continuation).** Per the user's same instruction
  ("en cada llamada ver que significado tiene y sustituirlo por la
  variable que representa"), traced every remaining `ZERO_TOL_E6/E8/E9`
  call site (8 for E9 alone: `geo/*/topology.py`'s `rg_max` division
  guard in `Compactness`/`CharacteristicWidth`, `surface_geometry.py`'s
  quadratic-coefficient and cross-product-of-axes guards,
  `meta_surfaces_utils.py`'s cross-product/edge-vector-length guards,
  `geo/{occ,ocp}/split_coaxial_cone.py`'s periodic-U-difference and
  `tan(semi-angle)` guards) and classified each: all 8 are the same
  "avoid a division/degenerate branch on a value that should be exactly
  zero only in a genuine degenerate case" role as `KERNEL_TOL_E13`'s own
  rename -> all moved to `NUMERIC_DOUBLE_TOL`. Per the user's explicit
  decision, `NUMERIC_DOUBLE_TOL` itself was retuned from `1e-13` to
  `1e-12` at the same time (a global value change affecting every one of
  its call sites, not just the new ones). `ZERO_TOL_E6`'s 6 sites split
  by real role instead of one blanket rename (matching the user's own
  per-site read): the 2 curvature-near-zero "is this edge straight"
  checks in `meta_surfaces_utils.py` (`edge_1D`/`spline_2D`) went to
  `NUMERIC_TOL` (a real geometric classification decision, the user's
  own call after inspecting the code); the 4 finite-difference-slope
  division guards in `geo/{occ,ocp}/repair.py` went to
  `NUMERIC_DOUBLE_TOL` instead (a different role -- avoiding a division
  by a computed slope, not a classification -- confirmed by reading each
  site before applying, not by pattern-matching the file/selection).
  `ZERO_TOL_E9`/`E10`/`E12` all reached zero remaining references and
  were deleted from `geo/constants.py`; `ZERO_TOL_E6`/`E8` likewise once
  their own sole surviving site (`functions.py`'s `sqr` guard, see the
  regression below) moved off them too -- the entire `ZERO_TOL_*` family
  is now gone from the codebase.
  **A real regression found and fixed via the corpus differential**:
  `GEOUNED/utils/functions.py::build_can_params`'s own
  `sqr = cp.dot(cp) - alpha*alpha; adist = sqrt(sqr) if abs(sqr) >=
  tol else 0` (the cone-apex-to-cylinder-axis perpendicular distance,
  for a Can's own cone/cylinder pairing) used `ZERO_TOL_E8` (1e-8) --
  migrated to `NUMERIC_DOUBLE_TOL` (by then 1e-12) along with the other
  8 sites, following the same role reasoning. This broke 2 real
  fixtures: `Cans/fwd_can_1.stp`/`rev_can_1.stp` started raising
  `ValueError: math domain error` (caught by a 143-file before/after
  corpus differential, `rich_worker.py`, run specifically because this
  batch's value jump -- up to 4 orders of magnitude in places -- was
  flagged as higher-risk than earlier single-method changes). Root
  cause, confirmed by direct reproduction and a corpus-wide measurement
  of every real `sqr` value reaching this line (274,119 samples, all
  143 files): `sqr` is a subtraction of two LARGE, near-equal mm^2
  quantities (`cp.dot(cp)` and `alpha*alpha`), which carries far more
  catastrophic-cancellation noise than a unit-vector cross/dot product
  or a plain length -- the real residual on both failing fixtures is
  exactly `-7.275957614183426e-12`, above the new 1e-12 floor but 4
  orders of magnitude below the old 1e-8 one; every OTHER near-zero
  sample of this same quantity across the whole corpus is genuine noise
  at a completely different scale (1e-93..1e-62), so there is a wide,
  clean, well-measured gap to place a dedicated value in. **Per the
  user's own decision** (given 3 options -- a dedicated constant, a
  plain revert, or loosening `NUMERIC_DOUBLE_TOL` globally -- chose the
  dedicated constant): new `SQUARED_LENGTH_TOL_E8 = 1.0e-8` in
  `geo/constants.py` (mm^2, NOT mm -- explicitly documented as a
  different role from the existing, same-VALUE-but-different-UNITS
  `LENGTH_TOL_E8`, so as not to conflate a length tolerance with a
  squared-length cancellation guard), restoring the original,
  long-proven 1e-8 value with a 4-order-of-magnitude margin over the
  measured real residual. This is the one `ZERO_TOL_E8` site that does
  NOT fit `NUMERIC_DOUBLE_TOL`'s role, despite superficially looking
  like the same "guards a division" pattern as the other 8 -- the
  underlying arithmetic (difference of two large squares vs. a
  unit-scale dot/cross product) genuinely differs in how much
  floating-point noise it carries, which is exactly why this session's
  own "read every call site's real meaning, don't pattern-match" rule
  exists.
  Also found and deleted in the same pass, unrelated to tolerances:
  `GEOUNED/utils/basic_functions_part2.py::is_duplicate_in_list` --
  confirmed via a full-repo grep to have ZERO callers anywhere
  (including tests), 100% dead code; deleted along with its
  now-orphaned `math`/`PARAM_ANGLE_TOL_E5` imports. Deleting it
  surfaced a second, unrelated, pre-existing dead-code bug the IDE's
  own unreachable-code diagnostic caught: `is_same_cylinder` (the
  function immediately above it) had a stray, unreachable `return False`
  sitting right after its own real `return same` -- removed too.
  **`PARAM_ANGLE_TOL_E4`/`PARAM_ANGLE_TOL_E5` merged into one
  `PARAM_ANGLE_TOL = 1.0e-5`**, per the user's own reasoning ("es una
  tolerancia de angulo 0-pi") -- both were already documented under the
  same role ("Tolerance (rad) on surface (U, V) parameters and arc
  angles"), genuinely distinct from `ANGLE_TOL_*`'s own role ("angle
  between two 3D DIRECTIONS") despite `PARAM_ANGLE_TOL_E4`'s value
  coinciding with `ANGLE_TOL_E4`'s by accident. 11 call sites across
  `meta_surfaces_utils.py` (6), `vector_geometry.py::arc_extent`,
  `basic_functions_part1.py::twoPimod`, `geometry_gu.py` (5), and
  `meta_surfaces.py` moved to the merged name. Per the user's explicit
  follow-up instruction, `decom_utils_generator.py::torus_bound_planes`'s
  own `is_same_value(params[1]-params[0], twoPi, NUMERIC_TOL)` (the
  torus face's own full-turn-parameter check -- flagged by name in this
  same file's earlier "Comparison contexts" entry) was switched to
  `PARAM_ANGLE_TOL` too, since it's exactly this role, not the
  same-solid-contact role `NUMERIC_TOL` is for. The SAME file's other
  `NUMERIC_TOL` use (`axes_parallel(dir, face.Surface.Axis, NUMERIC_TOL)`,
  a real 3D-direction comparison) was deliberately left untouched, as
  was `geometry_gu.py::merge_periodic_uv`'s own several NUMERIC_TOL-based
  full-turn checks (an established, distinct, deliberate design decision
  from an earlier batch -- "the parameters come from the same solid's
  own faces, so they carry the same numbers" -- not touched without
  being asked).
  **Verified, combined for the whole follow-up batch (ZERO_TOL_E6/E8/E9/
  E10/E12 elimination, NUMERIC_DOUBLE_TOL retune, PARAM_ANGLE_TOL merge,
  the SQUARED_LENGTH_TOL_E8 fix, both dead-code deletions)**: ocp suite
  292 passed/1 skipped (occ/freecad deferred to the end of the session
  per the user's own standing instruction for this session); a 143-file
  before/after corpus differential against the pre-session commit
  (`666fbef`) -- **0 real differences** (the one remaining diff,
  `Decomposed/SCDR_90_piece2.stp`, is the same pre-existing, already-
  documented timeout/degenerate-geometry failure that doesn't decompose
  under either version, just a few seconds' difference in the timing
  text of its own error message).
  **`ANGLE_TOL_*` family analyzed and cleaned up, 2026-09-23 (this was
  the next commit's own batch -- `ae28e58` already covers everything
  above this paragraph).** `ANGLE_TOL_E3`/`ANGLE_TOL_E4` were confirmed
  100% dead code (zero real callers anywhere in the repo, only their
  own definitions) and deleted outright. The 2 real sites of
  `ANGLE_TOL_E6` turned out to be misclassified under its own docstring
  ("angle tolerance between two 3D DIRECTIONS"): `decom_utils_generator.py`'s
  own `cross = axis1.cross(axis2); if cross.length > ANGLE_TOL_E6` is
  the exact same cross-product-of-two-unit-axes-near-zero "are these
  parallel?" test already migrated to `NUMERIC_DOUBLE_TOL` at several
  other sites in the previous batch -- missed there only because that
  sweep searched for `ZERO_TOL_*` names specifically, not `ANGLE_TOL_E6`.
  `surface_geometry.py::is_coaxial_cone_cylinder_pair`'s own
  `semiangle_min` default (`abs(tan(cone.SemiAngle)) < semiangle_min`)
  doesn't compare two directions either -- it tests a single surface's
  own SemiAngle parameter for being numerically zero (a near-cylindrical
  cone), the same "avoid a degenerate branch" role as every other
  `NUMERIC_DOUBLE_TOL` site. Both moved there per the user's decision.
  `ANGLE_TOL_E6` then had zero references left and was deleted.
  `ANGLE_TOL_5E2` (the intersect_plane/intersect_line method-selection
  threshold, confirmed NOT a tolerance in an earlier batch) was renamed
  to `ANGLE_THRESHOLD` at the user's request, to stop it looking like
  one of the tolerance family by name alone -- same value, same 6 call
  sites (3 engines x 2 methods), no behaviour change.
  `WINDING_ANGLE_TOL` (1 real site, `_closes_full_turn`) was confirmed
  to be its own genuinely distinct role -- a 16-sample winding-angle
  quantization tolerance, not a numeric-zero guard nor a direction
  comparison -- and left untouched.
  **Verified**: given this touches `is_coaxial_cone_cylinder_pair`
  (historically fragile coaxial-cone code) with a 6-order-of-magnitude
  tightening (1e-6 -> 1e-12, inherited from `NUMERIC_DOUBLE_TOL`'s own
  already-decided value) -- learning directly from the
  `SQUARED_LENGTH_TOL_E8` regression found earlier this same session --
  a fresh 143-file before/after corpus differential was run
  specifically for this change (not skipped): ocp suite 292 passed/1
  skipped; **0 real differences** (same single pre-existing timeout
  file as every other diff this session, unaffected).
  **Audit of `NUMERIC_DOUBLE_TOL`'s 5 "direction-comparison" sites for a
  possible NUMERIC_TOL downgrade, 2026-09-23 -- measured, no change
  made, kept for future reference.** The user asked for a full list of
  every real `NUMERIC_DOUBLE_TOL` call site (16 distinct roles, see the
  table built for this session), then specifically flagged the 5 whose
  quantity is a cross/dot product of two UNIT direction vectors
  (`GPlane.intersect_plane`, `GPlane.intersect_line`,
  `GLine.intersect_line`, `surface_geometry.find_can_plane`'s
  `n_common`, `decom_utils_generator.py::cks_bound_planes`'s
  `axis1.cross(axis2)`) as possibly too tight: this project's own
  established finding is that real STEP-derived direction cosines carry
  ~5e-7 rad of noise (why `NUMERIC_TOL=1e-7` exists at all), 5 orders of
  magnitude above `NUMERIC_DOUBLE_TOL`'s 1e-12. Measured all 5 directly
  over the 143-file corpus (decompose + build_solid_definition):
  - `intersect_plane`: 1809 real calls, 55 samples "near zero" -- every
    one is pure floating-point noise (1e-15 to 1e-28), nothing anywhere
    near 1e-7.
  - `GPlane.intersect_line`: 0 calls -- its only real callers are in
    GEOReverse (`boundBox.py`, paused pipeline), not exercised by the
    forward `CadToCsg` corpus scan at all.
  - `GLine.intersect_line`: 3854 calls, 0 near.
  - `find_can_plane`: 56 calls, 0 near.
  - `cks_bound_planes`' own `axis1.cross(axis2)` (line 117): 53 calls,
    24 exact zeros, 1 at floating-point noise scale (3e-18), and 12
    samples (3 related fixtures: `Hollow_plates/cylcone_exact_placa3_pos.stp`,
    2 copies of `cyl_cone.stp`) at a real, reproducible, non-noise value
    of `~4.793370862739e-06` -- but that value is itself LARGER than
    `NUMERIC_TOL` (1e-7), so both constants classify it identically
    ("not parallel"); no real sample anywhere in [1e-12, 1e-7].
  A first pass at measuring this last site gave misleading values (up
  to 5e-4) from wrapping `GVector.cross` globally for the whole
  `cks_bound_planes` call -- that captured EVERY cross product in the
  function's own call tree (`planar_edges`/`other_face_edge`/
  `is_same_surface`/...), not just line 117's own; corrected by
  re-executing that exact snippet read-only, isolating only the real
  quantity being asked about. Kept as a reminder: a broad monkeypatch
  measures "everything nearby", not "this one line" -- re-derive the
  exact expression when precision matters.
  **Conclusion, per the user ("lo dejamos así, pero dejalo registrado
  por si sale un bug relacionado")**: no change made to any of the 5
  sites -- the corpus shows no real data landing between
  `NUMERIC_DOUBLE_TOL` (1e-12) and `NUMERIC_TOL` (1e-7) for any of them,
  so there is currently no evidence either constant would behave
  differently here. If a future bug looks like a near-parallel
  direction pair being wrongly classified as "not parallel" (a spurious
  extra bounding plane, a missed axis-alignment merge, a `GPlane`/`GLine`
  intersection unexpectedly returning `None`), re-check this entry and
  these 5 sites first -- the ~5e-7 rad real-noise concern that motivated
  this whole audit is still a real phenomenon in this codebase, it just
  didn't show up as a live problem in the current 143-file corpus.
  **`LENGTH_TOL_E*` family analyzed and renamed to the constant each
  site's real role already matches, 2026-09-23.** Read every real call
  site of `LENGTH_TOL_E3/E5/E6/E7/E8/E12`. `LENGTH_TOL_E6` deliberately
  left untouched (its ~15 sites span several genuinely different roles,
  needs its own closer look); every other member turned out to already
  be an existing, better-named constant under a different name, values
  unchanged:
  - `LENGTH_TOL_E7` (freecad's `orientation_outward`, the native
    `isInside()` tolerance) is the exact same role as `KERNEL_TOL_E7`
    (occ/ocp's own `BRepClass3d_SolidClassifier` tolerance in the
    identical function) -- same value, just named differently per
    engine. Renamed to `KERNEL_TOL_E7`.
  - `LENGTH_TOL_E3` (`meta_surfaces_utils.py`'s "is this edge long
    enough to bother with the angle-sweep" filter) is the exact same
    value (1e-3) and role as the already-established
    `MIN_SLIVER_EDGE_LENGTH`. Renamed.
  - `LENGTH_TOL_E5` (~18 sites, all in `meta_surfaces_utils.py`, plus
    `solid_defects.py`/`functions.py`/`meta_surfaces.py`) is the exact
    same value and role as `POINT_POINT_TOL` -- flagged as a "possible
    relabel, never measured" as far back as the 2026-09-19/20 batch.
    Renamed at every site.
  - `LENGTH_TOL_E8` (`build_shape_functions.py`'s `fix_same_points`/
    `fix_points`/`remove_box_faces`, all literally "are these two
    points close enough to merge") is the same role as `POINT_POINT_TOL`
    at a tighter, undocumented value (1e-8 vs 1e-5) -- **measured**
    before renaming (143-file corpus, all 3 functions instrumented
    directly): the real near-zero residuals top out at ~1.5e-11,
    ~3.7e-11, ~5.5e-12 respectively, three orders of magnitude below
    the old 1e-8 floor and with **zero** samples anywhere in [1e-8,
    1e-5] -- loosening to `POINT_POINT_TOL` changes nothing on this
    corpus. Renamed.
  - `LENGTH_TOL_E12` (`vector_geometry.py::myBox.__init__`'s degenerate-
    bounding-box guard) is the exact same "double-precision arithmetic
    zero" role as `NUMERIC_DOUBLE_TOL`, just still at its own pre-rename
    value (1e-12, coincidentally already equal). **Measured** alongside
    the structurally-identical `LENGTH_TOL_E6`-based guards in
    `solid_ops.py::BuildSolidParts`/`build_region/Objects.py::
    CellObj.makeBox` (2811 + 1371 real boundBox-dimension samples,
    `BuildSolidParts` itself not reachable via decompose + build_solid_
    definition alone on this corpus): **zero** near-zero samples in
    either -- no evidence either value matters here, but `LENGTH_TOL_E6`
    itself is deliberately NOT touched yet (same "needs a closer look"
    reason as above); only `LENGTH_TOL_E12` -> `NUMERIC_DOUBLE_TOL` was
    renamed (a pure rename, same value).
  `LENGTH_TOL_E3/E5/E7/E8/E12` then had zero remaining references and
  were deleted from `geo/constants.py`, leaving only `LENGTH_TOL_E6`
  (with a docstring note explaining it's the one deliberately deferred
  member of this family).
  **`LENGTH_TOL_E6`'s 3 default-argument sites, same session,
  continuation.** Of `LENGTH_TOL_E6`'s remaining sites, 3 are function
  DEFAULT ARGUMENTS rather than inline comparisons -- reviewed
  separately since a default can hide whether it's actually load-bearing:
  - `is_same_value` (both copies, `geo.surface_geometry` and its
    `GEOUNED.utils.basic_functions_part1` wrapper): traced every real
    caller -- ALL of them pass an explicit, context-appropriate
    tolerance (`Surfaces.tolerances.distance` at the time,
    `PARAM_ANGLE_TOL`, `NUMERIC_TOL`); the function's own default is
    exercised ONLY by `tests/geo/test_vector_geometry.py`, never by any
    real pipeline code. Being a fully generic float-comparison helper
    (no fixed physical role of its own -- used for lengths, angles,
    ratios depending on the caller), its default is a pure floating-
    point-zero fallback, not a "length" role -- **per the user's
    decision**, changed to `NUMERIC_DOUBLE_TOL`. The one test that
    exercised the old default at 1e-9 precision was updated to test at
    the new, much tighter precision instead (1e-13 passes, 1e-9 no
    longer does -- documented inline why).
  - `_valid_chain_junction` (`meta_surfaces_utils.py`, the RevCC chain-
    junction topology test): its own `tol` IS load-bearing at its 2 real
    callers (both inside `get_join_cone_cyl`, always at the default --
    no caller passed one explicitly) -- and its real use,
    `(v - V0).length < tol`, is a genuine point-coincidence test, not a
    generic float comparison. Per the user's own direction ("tenemos la
    posibilidad de pasar la tolerancia del objeto tolerance... ¿hay un
    parametro que define una distancia generica entre dos puntos?"),
    the fix was two-part: (a) `tol` dropped its own default entirely
    (now a required keyword-only parameter, matching this whole
    project's "a forgotten call site is a TypeError, not a silent
    default" convention), and (b) resolved the deeper duplication this
    question surfaced -- see the `Tolerances.distance` entry right
    below.
  **`Tolerances.distance` retired entirely, same session, direct
  continuation -- a real architectural duplicate, not just a naming
  mismatch.** The question above ("is there a generic point-to-point
  distance in the Tolerances object?") surfaced that there already WAS
  one, `Tolerances.distance` ("General Distance Tolerance", default
  1e-4, a public/JSON-config field) -- sitting alongside the intrinsic
  `POINT_POINT_TOL` (1e-5) for what turned out to be the exact same
  underlying physical phenomenon. Investigated before deciding which
  direction to unify: `tolerances.distance` has 5 real call sites (3 in
  `cell_definition_functions.py`'s `V_torus_surface`, comparing
  z-heights/radii via `is_same_value`; 2 in `decom_utils_generator.py::
  cks_bound_planes`, the same axis-to-point distance gate already
  measured -- real values are exact noise or clearly large, nothing near
  either 1e-4 or 1e-5), and has **never once been exercised at a
  non-default value** anywhere in the repo or its tests (the one place
  it's passed explicitly, `tests/test_cadtocsg.py`, restates the exact
  same 1e-4 default). Combined with `POINT_POINT_TOL`'s own, already-
  rigorously-measured intrinsic classification (real CAD point
  coincidences are 0/<1e-9mm, distinct points >=1e-2mm, a wide clean gap
  where the exact value never matters) and this file's own prior note
  ("cks_bound_planes uses Tolerances.distance (1e-3 -> 1e-4, no
  regression seen)" -- confirming even a 10x change never mattered
  there either), the case for a real, tunable, per-user `distance` field
  was never actually demonstrated. **Per the user's explicit
  confirmation**: `distance` removed entirely from `Tolerances`
  (constructor parameter, property/setter, docstring entry, the
  `scaled()` passthrough) -- NOT kept-but-unused the way
  `relativePrecision`/`value` were (those are aspirational, "the user
  plans to apply [them] correctly later"; `distance` was a genuine,
  demonstrated duplicate with no real use ever observed, a different
  situation). All 5 real call sites now import and use `POINT_POINT_TOL`
  directly; `tests/test_cadtocsg.py` and `tests/config_cadtocsg_
  complete_defaults.json` (the "every parameter enumerated explicitly"
  fixtures) had their own `distance=`/`"distance"` entries dropped to
  match; `tests/geo/test_tolerances_split.py::
  test_geotolerances_holds_only_what_geo_reads` (which used to pin
  `distance` as a `Tolerances`-only field) updated to instead pin that
  it no longer exists on either class at all.
  **Verified, combined for this whole `LENGTH_TOL_E*`/`distance` batch**:
  ocp suite 292 passed/1 skipped; a 143-file before/after corpus
  differential against the pre-batch commit -- **0 real differences**
  (same single pre-existing timeout file as every other diff this
  session, unaffected). occ/freecad suites still deferred to later in
  the session per direct user instruction.
  **`LENGTH_TOL_E6`'s remaining 9 sites, same session, direct
  continuation -- the whole `LENGTH_TOL_E*` family is now gone.**
  `LENGTH_TOL_E6` was deliberately left untouched in the batch above (it
  spans several genuinely different roles); reviewed site-by-site, per
  direct user decision at each:
  - `probe = point + normal * LENGTH_TOL_E6` (3 engines,
    `orientation_outward`'s own point-just-off-the-surface probe, fed to
    a native point-in-solid classifier): -> `NUMERIC_TOL`, with a new
    inline comment ("increment along the normal to get a point very
    close to the solid's own surface, just off it") documenting the
    role directly at the call site, since the constant's own name
    doesn't carry it the way `MIN_SLIVER_EDGE_LENGTH`/`POINT_POINT_TOL`
    do.
  - `solid_ops.py::BuildSolidParts` and `build_region/Objects.py::
    CellObj.makeBox`'s own degenerate-bounding-box guards (the same
    structural pattern as `myBox.__init__`'s own `LENGTH_TOL_E12`,
    already renamed to `NUMERIC_DOUBLE_TOL` in the batch above, and
    already measured together with these two: 4182 combined real
    boundBox-dimension samples, zero near zero) -> `NUMERIC_DOUBLE_TOL`
    at both, per the user's direct instruction.
  - `surface_geometry.py::find_can_plane`'s own arbitrary-perpendicular
    fallback (`e2 = A.cross(X); if e2.length < TOL: e2 = A.cross(Y)`) --
    the same cross-product-of-two-unit-vectors-near-zero "is the main
    axis parallel to X" degeneracy guard already migrated at several
    other sites -> `NUMERIC_DOUBLE_TOL`.
  - `cell_definition_functions.py`'s plane-to-shell distance gate and
    `boolean_solids.py`'s 2 solid/shell-distance "does this actually
    intersect" gates -- genuine CONTACT tests (not a construction
    offset, not a degenerate-branch guard) -> `NUMERIC_TOL`, per the
    user's direct instruction (not `POINT_POINT_TOL`, which was this
    session's own first guess before asking).
  - `meta_surfaces.py::get_can_surfaces`'s own `abs(s.Surface.Radius -
    cylinder.Surface.Radius) < TOL` (adjacent-cylinder same-radius
    test): the user drew a real, previously-uncaptured distinction --
    this compares a single SCALAR VALUE (a radius), not a spatial
    distance, and should use a dedicated "scalar value comparison"
    tolerance, not `cyl_distance` (which the canonical
    `is_same_cylinder_surface` already overloads for both radius AND
    axis-distance) nor an intrinsic constant. Presented 3 options
    (reuse the existing-but-so-far-unread `Tolerances.value`, add a new
    dedicated field, or something else) -- **user chose reusing
    `Tolerances.value`**: this is its first real, live use anywhere in
    the pipeline (its own docstring previously said "Not read by the
    code at the moment... kept for the same reason [as
    relativePrecision]"). `tolerances` was already threaded into this
    exact call site (`get_can_surfaces(cylinder, solidFaces, *,
    tolerances)`), so no signature change was needed. `Tolerances.value`'s
    own default (1e-6) happens to equal `LENGTH_TOL_E6`'s, so this is
    default-behavior-neutral while making the comparison genuinely
    user-tunable for the first time.
  - `meta_surfaces_utils.py::get_adjacent_cylknesurfFace`'s own
    `if e.Length < TOL: continue` (skip a degenerate/zero-length edge
    before classifying it) -> `NUMERIC_TOL`, per the user's direct
    instruction (a contact/degeneracy-adjacent skip, not the same
    "sliver filter before an expensive computation" role
    `MIN_SLIVER_EDGE_LENGTH` covers).
  `LENGTH_TOL_E6` then had zero remaining references anywhere and was
  deleted from `geo/constants.py` -- **the entire former
  `LENGTH_TOL_E*` family (E3/E5/E6/E7/E8/E12) is now gone**, each site
  folded into whichever already-established constant (or, for the one
  cylinder-radius site, `Tolerances.value`) actually matched its real
  role.
  **Verified**: ocp suite 292 passed/1 skipped after each sub-step; two
  separate 143-file before/after corpus differentials (one after the
  `NUMERIC_TOL`/`tolerances.value` sites, one after the final
  `NUMERIC_DOUBLE_TOL`/`NUMERIC_TOL` pair) -- **0 real differences** in
  both (the second run didn't even show the usual single pre-existing
  timeout-file diff). occ/freecad suites still deferred to later in the
  session per direct user instruction.
  **`REL_TOL_E*` family, same session, direct continuation.**
  `REL_TOL_E2` (2 real sites, `find_can_plane`'s narrow/wide threshold
  and `spline_wires`' own offset, both literally "1% of a real
  geometric span") was, per the user's own direct instruction, not a
  tolerance at all -- replaced with the literal `0.01` inline at both
  sites (with a `# 1%` comment), matching this project's own established
  convention for non-tolerance multiplicative factors (mm->cm `*0.1`,
  box enlargement `0.2`, density `<1e-2`, ...); deleted from
  `geo/constants.py` once unused.
  `REL_TOL_E5` (4 real sites, `geo/{occ,ocp}/split.py::
  remove_tools_from_raw_solids`, the "was the tool solid returned in the
  split results, and is it the same volume" cleanup after a real
  `BOPAlgo_Splitter` call) is exactly `Tolerances.volume_tolerance`'s own
  role (volume conservation) at an unthreaded, hardcoded value --
  **measured first** (given the historically fragile `Gsplit` code path)
  by instrumenting all 3 of the function's own comparisons across the
  143-file corpus: the `ratio2` comparison (`s_volume` vs `tool_volume`)
  came back clean (0 samples in [1e-6, 1e-5]), but `ratio1`
  (`out_volume` vs `in_volume`) had 3 real samples at exactly
  `2.195333e-06` (`Cans/barrel_right.stp`) and `tool_volume_abs` (the
  absolute "is the tool volume negligible" half of the same `if`) had
  **28** real samples scattered through that same window
  (`Mixed/double_RC.stp`, `Mixed/cut_0.stp`, `Torus/example.stp`,
  `Cans/barrel_right.stp`) -- real, reproducible data, not noise,
  meaning swapping to `volume_tolerance`'s default (1e-6) would
  genuinely change these specific decisions, not just rename a value.
  Presented 3 options (thread `tolerances` but keep 1e-5 as this
  context's own override; accept the 1e-6 change and verify with
  d1suned; leave `REL_TOL_E5` as its own separate constant) -- **user
  chose to accept the tighter value and verify**: `tolerances` is now
  threaded through `remove_tools_from_raw_solids` (a new required
  parameter, both engines) and all 4 sites use
  `tolerances.volume_tolerance` directly. Verified: ocp suite 292
  passed/1 skipped; a 143-file before/after corpus differential -- **0
  real differences anywhere, including on the flagged files**
  (`Cans/barrel_right.stp` itself came back byte-identical in pieces/
  volume/composite counts despite its own real near-threshold samples,
  meaning the specific `tool`-removal branch this guards never actually
  changes the FINAL decomposition outcome on this corpus, even though
  the internal comparison itself does flip for it); a direct d1suned
  check on `Cans/barrel_right.stp` (the file with the most affected
  samples) specifically requested by the user -- `tally = 0.99644 +/-
  0.25%` (1.4 sigma from 1.0), 0 lost particles (the "SD4 differs from
  true" note the script also prints is a rounding artifact, 29mm^3 on a
  2.52-billion-mm^3 solid, ~1e-8 relative -- not a real discrepancy).
  `REL_TOL_E5` then had zero remaining references and was deleted from
  `geo/constants.py`.
  **A real unit-mismatch bug found and fixed while threading
  `tolerances` through, same session, direct continuation.** Per the
  user's own explicit rule ("hay que aplicar la tolerancia de volumen
  relativa cuando se comparan dos volumenes y absoluta cuando se
  compara respecto a un valor") -- checked every volume comparison in
  `geo/{occ,ocp}/split.py`. All of them already followed this rule
  except one: `remove_tools_from_raw_solids`'s own `abs(tool_volume) >
  tolerances.volume_tolerance` compares a single ABSOLUTE volume (mm^3)
  against `volume_tolerance`, a DIMENSIONLESS relative ratio (1e-6) --
  comparing incompatible units, left over from the plain
  `REL_TOL_E5`-for-everything version this same session's earlier step
  had just replaced. Every other volume check in the same file already
  correctly picks one or the other (`abs(...) > tolerances.
  min_solid_volume` for an absolute floor at 2 other sites,
  `volume_within(..., tolerances.volume_tolerance)` or `... <
  tolerances.volume_tolerance * <a volume>` for a relative one, 4 other
  sites) -- confirming the rule is already this file's own real
  intent, just missed at this one spot. Fixed: `tool_volume` (the ACTUAL
  volume of the cutting tool solid, "is it substantial enough to bother
  comparing against the output pieces") is an absolute-floor decision,
  the same role `tolerances.min_solid_volume` already serves 2 lines
  above in this very file -- switched to it. `freecad`'s own `Gsplit`
  has no equivalent function (its cascade is simpler, already only uses
  `min_solid_volume` throughout) -- nothing to fix there.
  Separately checked, per the user's own second instruction, whether
  "minimum volume" is defined redundantly in both `geo/constants.py`
  and `Tolerances`: confirmed NOT a duplicate -- `VOLUME_MIN_E8`
  (1e-8, intrinsic, "does a boolean Common's result have genuine
  non-degenerate content") and `Tolerances.min_solid_volume` (default
  `DEFAULT_MIN_SOLID_VOLUME` = 1e-2, user-facing, "is this piece big
  enough to keep as a distinct output solid") are 6 orders of magnitude
  apart and serve genuinely different, already well-documented roles
  (a kernel-noise/empty-intersection floor vs. a meaningful-output-size
  floor) -- correctly separate, no action needed.
  **Verified**: ocp suite 292 passed/1 skipped; a 143-file before/after
  corpus differential covering this whole uncommitted batch
  (`REL_TOL_E2`/`E5` + this fix together) -- **0 real differences**.
  **The `min_solid_volume` fix above corrected, same session, direct
  continuation -- a real pipeline-stage distinction, not just a units
  fix.** The user pointed out `remove_tools_from_raw_solids` operates on
  INTERMEDIATE solid components DURING decomposition, before they have
  gone through the full reconstruction/repair cascade -- discarding a
  small-but-real intermediate piece this early, at `min_solid_volume`'s
  own threshold (1e-2, tuned for the FINAL "is this worth keeping as
  output" decision), risks breaking that solid's own later
  reconstruction. So the units fix above was right in spirit (an
  absolute floor, not the relative `volume_tolerance`) but wrong in
  which absolute floor. New `PRE_REPAIR_MIN_VOLUME = 1.0e-5` in
  `geo/constants.py`, its own dedicated intrinsic constant for this
  earlier pipeline stage -- kept at the same value this exact role
  already had before being (mis)named `REL_TOL_E5`, per the user's own
  explicit choice not to re-derive it from a fresh measurement.
  `remove_tools_from_raw_solids`'s `abs(tool_volume) > ...` now compares
  against this instead of `tolerances.min_solid_volume`; the 2 genuinely
  relative comparisons in the same function keep
  `tolerances.volume_tolerance`, per the user's own explicit
  confirmation to preserve that part unchanged.
  **Second instruction, same message -- a full audit of every "minimum
  volume" constant/field in the codebase, not just what stage 1 of this
  same finding had already checked in `split.py` alone.** Grepped both
  `geo/constants.py`/`geo/tolerances.py` (every named constant/field)
  and a broad pattern search across the whole `geo`/`GEOUNED` source
  for any unnamed/inline "minimum volume" literal -- found none beyond
  the already-catalogued set. The full picture, confirmed already
  correctly homogenized with no duplicates: `Tolerances.min_solid_volume`
  (default `DEFAULT_MIN_SOLID_VOLUME`, 1e-2 -- the FINAL "keep as
  output" floor, already unified across every one of its own real call
  sites since the original 2026-09-19 batch) vs. the new
  `PRE_REPAIR_MIN_VOLUME` (1e-5, the intermediate-stage sibling just
  added) vs. `VOLUME_MIN_E8` (1e-8, a genuinely different role -- "does
  a boolean Common's result have real content", not a "worth keeping"
  decision at all) vs. `VOLUME_REF` (1.0, not a threshold at all -- the
  SCALE below which a relative volume comparison is meaningless).
  `solid_ops.py::space_decomposition`'s own `min_volume` (used by
  `c.Volume < min_volume`) is a real function parameter, not a
  hardcoded literal, fed by callers passing `tolerances.
  min_solid_volume` -- confirms no other scattered copy exists.
  **Verified**: ocp suite 292 passed/1 skipped; a 143-file before/after
  corpus differential -- 0 real differences (the usual single
  pre-existing timeout-file diff, unaffected).
  **`REL_TOL_E3` (all 8 real sites), same session, direct continuation
  -- gone one site at a time, per direct user instruction ("vamos por
  parte"), each edit applied without running tests until the whole
  batch was done together:**
  - `geo/{occ,ocp}/open_solid_repair.py::_resew_faces_to_solid`'s own
    re-sew tolerance ceiling (`max(3*width, seam_tol)` capped at
    `diagonal * REL_TOL_E3`) -> new `RESEW_CEILING` (a repair-operation
    ceiling, not a detection/classification test).
  - `geo/{occ,ocp}/split_coaxial_cone.py::_try_coaxial_cone_split`'s
    retry cascade: the exact (`retry_tolerance == 0.0`) attempt now uses
    `tolerances.volume_tolerance` (a real, threaded parameter, matching
    `Gmerge_coplanar_planes`'s own established "exact operation" role);
    the fuzzy-retry attempt keeps its own, deliberately looser new
    `COAXIAL_RETRY` constant (an algorithmic retry margin, not something
    a user tunes per model).
  - `GEOUNED/utils/meta_surfaces_utils.py`'s 2 apex-nudge offsets (the
    "push a UV probe point slightly off the exact apex" epsilons) were
    not tolerances at all -- literal `0.001 * v...` with a `# small
    positive V nudge off the apex` comment, same convention as
    `REL_TOL_E2`'s own resolution.
  - `geo/surface_geometry.py::find_can_plane`'s own second offset
    (`min(0.001 * half_width, narrow_wide_threshold)`) -- same
    resolution, literal with a `# 0.1% of half_width` comment.
  - `geo/solid_defects.py::count_split_ring_pairs`'s own 3-way "split
    boundary ring" detection tolerance (radius-ratio, gap-vs-diagonal,
    perpendicular-offset-vs-radius, all gated by the SAME value by
    design) -> new `SPLIT_RING_REL_TOL`.
  - `GEOUNED/utils/meta_surfaces_utils.py`'s axial-extreme edge
    classification tolerance (`get_adjacent_cylknesurfFace`, "is this
    edge's V-parameter close to the cylinder/cone's own axial extreme")
    -> new `REL_DIST_TOL` (a genuine relative-DISTANCE role, confirmed
    by the user to be distinct from any of the relative-VOLUME roles
    that make up the rest of this family).
  - `decompose/decom_one_generators.py`'s own "Lost ...%" warning
    print (the compound-volume-exceeds-original sign-inverted message,
    a known, separately-tracked bug -- NOT fixed here) -> swapped
    directly to `REL_TOL_E4`, per direct user instruction, for coherence
    with `generic_split`'s own `volume_within` check just below it in
    the same function -- not re-measured, a deliberate "make these two
    related checks share one threshold" choice rather than a
    measurement-driven one.
  `REL_TOL_E3` then had zero remaining references and was deleted from
  `geo/constants.py` -- every one of its 8 real sites turned out to be a
  genuinely distinct role wearing the same historical value (1e-3), not
  one single concept.
  **`REL_TOL_E4`/`REL_TOL_E6` review, same session, direct continuation.**
  `REL_TOL_E4` itself is untouched -- still a real, single, coherent role
  (`sliver_edge_rel_tol`'s own default and `find_short_edges`/
  `find_split_ring_faces`/`check_solid_defects`'s shared parameter,
  `solid_ops.py::BuildSolidParts`'s own `refine(rel_tol=REL_TOL_E4)` call,
  now also the "Lost...%" warning above) -- not part of this cleanup.
  `REL_TOL_E6`'s remaining 3 real sites (after `REL_TOL_E5`'s own
  unrelated removal above) were reviewed one at a time:
  - `GSolid.refine()`'s own default `rel_tol` (all 3 engines: `freecad`/
    `occ`/`ocp` `topology.py`) -- the user first explored wiring
    `tolerances` through every `refine()` call site as a real parameter,
    but declined once tracing the call graph showed it would require
    touching core constructors that have no `tolerances` parameter at
    all, some shared with paused GEOReverse code -- too large a
    refactor for what this is. Instead, just renamed the hardcoded
    default in place: `REL_TOL_E6` -> `NATIVE_VOL_TOL` -> (immediately
    renamed again, same session) `NATIVE_VOL_RATIO_TOL` -- its own
    dedicated role (a native/kernel-adjacent operation's own
    volume-invariance self-check after `ShapeUpgrade_UnifySameDomain`/
    `removeSplitter()`), distinct from `Tolerances.volume_tolerance`
    (a relative-volume comparison between two independently-obtained
    solids) even though the two happen to share a value.
  - `GLine.intersect_line`'s own skew-vs-coplanar test (all 3 engines,
    `REL_TOL_E6 * scale_ref`) -> new `LINE_COPLANAR_REL_TOL` -- a pure
    geometric method with no `tolerances` object available, confirmed
    via direct "sí".
  - `geo/{occ,ocp}/repair.py::Gmerge_coplanar_planes`'s own
    `volume_within(result.Volume, solid.Volume, REL_TOL_E6)` check ->
    `tolerances.volume_tolerance` directly (a real, threaded parameter
    of that function, and already the exact same role
    `MAX_REPAIR_VOLUME_REL_CHANGE`'s own docstring had already
    documented it as), confirmed via direct "sí".
  - `geo/vector_geometry.py::myBox.add()`'s own Reversed+Reversed
    exact-union-vs-safe-fallback check (the inclusion-exclusion identity
    test deciding whether a box union is exact or must fall back to the
    larger operand alone) -> new `BOX_UNION_VOL_TOL` -- `myBox` is pure
    geometry with no `tolerances` object available either, same
    reasoning as `LINE_COPLANAR_REL_TOL`.
  `REL_TOL_E6` then had zero remaining references anywhere (including
  `geo/tolerances.py`'s own `GeoTolerances.__init__`, where
  `volume_tolerance: float = REL_TOL_E6` became a plain `1.0e-6` literal
  in place, matching every other field's own already-established
  convention of a literal default in that one constructor) and was
  deleted from `geo/constants.py` -- **the entire former `REL_TOL_E*`
  family (E2/E3/E5/E6) is now gone**, `REL_TOL_E4` the only survivor,
  unchanged and still a single coherent role.
  A stale direct reference to the now-deleted `REL_TOL_E6` in
  `tests/geo/test_repair_volume_gate.py::
  test_exact_operations_keep_their_own_tight_gate` (asserting it stays
  below `MAX_REPAIR_VOLUME_REL_CHANGE`) was updated to assert
  `NATIVE_VOL_RATIO_TOL` instead, with a comment explaining
  `Gmerge_coplanar_planes` itself no longer needs a dedicated mention
  there since it now shares `Tolerances().volume_tolerance` directly
  with the line just above it.
  **Verified**: ocp suite 292 passed/1 skipped (after fixing the one
  stale test reference above); a 143-file before/after corpus
  differential covering this whole `REL_TOL_E3`/`E4`/`E6` batch together
  -- 0 real differences (every one of these changes is either a pure
  rename at an unchanged value, or a print-message threshold swap with
  no effect on any solid's own pieces/volume/surface counts). occ/
  freecad suites still deferred to later in the session per direct user
  instruction.
  **`REL_TOL_E4` itself, same session, direct continuation -- the last
  surviving member of the whole family, closed out.** Traced its
  remaining 4 real sites: `GeoTolerances.sliver_edge_rel_tol`'s own
  default -> a plain `1.0e-4` literal (matching `volume_tolerance`'s own
  just-established convention); `geo/solid_defects.py`'s 3 pure,
  duck-typed functions (`find_short_edges`/`find_split_ring_faces`/
  `check_solid_defects`, which have no `tolerances` object to read from
  by signature) -> new `DEFAULT_SLIVER_EDGE_REL_TOL`, matching the
  already-established `DEFAULT_<field>` naming convention
  (`DEFAULT_MIN_FACE_WIDTH`/`DEFAULT_MIN_SOLID_VOLUME`/
  `DEFAULT_SPLIT_SCALE`) for a constant that backs a real `Tolerances`
  field's own default rather than being an independent role; `geo/
  solid_ops.py::Gfuse_solids`'s own `refine(rel_tol=REL_TOL_E4)` (a
  deliberate, well-documented, looser-than-`NATIVE_VOL_RATIO_TOL` guard
  for merging a boolean fuse's own redundant tangent-seam faces) -> new
  `FUSE_REFINE_REL_TOL`, its own dedicated role despite sharing the
  historical value; `decompose/decom_one_generators.py`'s paired "Lost
  ...%" warning + `generic_split`'s own `volume_within` sanity check
  (already deliberately sharing one threshold, per an earlier
  instruction this same session) -> `tolerances.volume_tolerance`
  directly, once the value question below was settled.
  **A real bug found and fixed while tracing these sites**:
  `geo/{occ,ocp}/repair.py::Gcollapse_split_rings` called
  `find_split_ring_faces(solid, min_face_width)` without its own
  `rel_tol` argument, silently falling back to the constant default even
  though `tolerances` is the function's own parameter, right there in
  scope, one line above (`min_face_width = tolerances.min_face_width`)
  -- so a user's own `sliver_edge_rel_tol` override was silently ignored
  by this one riser-detection call. Fixed: both engines now pass
  `tolerances.sliver_edge_rel_tol` explicitly.
  **`Tolerances.volume_tolerance`'s own default raised from `1e-6` to
  `1e-4`, per direct user decision, MEASURED first.** The user asked
  where `volume_tolerance` is actually used before deciding -- it is the
  core volume-conservation guard threaded through essentially all of
  `Gsplit`'s own repair/retry cascade (`check_changed_ok`,
  `remove_tools_from_raw_solids`, `Gsplit`'s own final check,
  `_try_coaxial_cone_split`'s exact-retry case, `Gmerge_coplanar_
  planes`), not a niche site -- so raising it 100x is a real,
  potentially high-impact change to the whole split cascade's strictness,
  not a simple rename. A dedicated 143-file corpus scan comparing
  `volume_tolerance=1e-6` (the then-current default) against a
  hypothetical `1e-4` found **exactly 1 real difference**:
  `Hollow_plates/placa2.stp`'s own final volume shifts from
  `4008775.44` to `4008758.34` mm^3 (17.1 mm^3, ~4.3e-6 relative) with
  its piece count and every composite-surface count (MultiP/RevCan/
  RevTCone/RoundC/RevCC) completely unchanged -- some repair step
  along the way accepts a marginally more volume-drifted intermediate
  result at the looser threshold, with no visible effect on the final
  decomposition's own topology. User confirmed the change given this
  result. `GeoTolerances`/`geouned.Tolerances`'s own defaults (`geo/
  tolerances.py`, `GEOUNED/utils/data_classes.py`) both updated to
  `1.0e-4`; `decom_one_generators.py`'s 2 sites now read
  `tolerances.volume_tolerance` directly instead of a separate,
  independent `REL_TOL_E4` constant, per the user's own explicit
  decision that these should share the SAME numeric value as
  `volume_tolerance` going forward, not just coincidentally match it.
  `REL_TOL_E4` then had zero remaining references anywhere and was
  deleted from `geo/constants.py` -- **the entire `REL_TOL_E*` family
  (E2 through E6) is now completely gone**.
  **Verified**: ocp suite 292 passed/1 skipped; a 143-file before/after
  corpus differential covering this whole final sub-batch (the 3
  renames, the `Gcollapse_split_rings` bug fix, and the real
  `volume_tolerance` value change together) -- the SAME single real
  difference as the dedicated `volume_tolerance` measurement above
  (`Hollow_plates/placa2.stp`, ~4.3e-6 relative), confirming the other 3
  changes in this sub-batch are exactly behavior-preserving on their
  own. occ/freecad suites still deferred to the end of the session per
  direct user instruction.
  **`KERNEL_TOL_E*` family (E3/E6/E7/E8) analyzed and entirely
  eliminated, same session, direct continuation.** Much larger and more
  heterogeneous than every prior family reviewed: 4 constants covering
  9 genuinely distinct roles, mostly under `KERNEL_TOL_E6` sharing
  nothing but a historical value. Per the user's own general rule
  stated mid-review ("en general todas la tolerancias asociadas a
  opraciones de BREP internas sera asociadad as parametros constantes"):
  every tolerance handed to an internal BRep-kernel operation (a native
  OCCT/OCP call with no corresponding `Tolerances` field) stays its own
  dedicated intrinsic constant, never threaded through `Tolerances` --
  settling, for this whole family, the same "constant vs. threaded"
  question the earlier `KERNEL_TOL_E7`/`GeomAPI_IntSS` site had already
  been individually decided for (2026-09-22).
  **Fix tolerance group** (`KERNEL_TOL_E6`, already literally
  `Tolerances.fix_tolerance`'s own role) -- 5 real sites: the field's
  own default -> plain `1.0e-6` literal (matching `volume_tolerance`'s
  own established convention); `split_repair.py::
  _repair_non_manifold_solid`'s own default (all real call sites
  already pass `tolerances.fix_tolerance` explicitly) -> new
  `DEFAULT_FIX_TOLERANCE`; `Gload_step`'s own `_native_fix` call (both
  engines -- a "plain loading primitive" with no `tolerances` parameter
  at all) -> `DEFAULT_FIX_TOLERANCE`. **2 real bugs fixed**:
  `solid_ops.py::Gfuse_solids`'s `fused.fix(KERNEL_TOL_E6)` was
  hardcoded despite `tolerances` being a real (possibly-`None`, GEOReverse
  calls it bare in several places) parameter -- now
  `tolerances.fix_tolerance if tolerances is not None else
  DEFAULT_FIX_TOLERANCE`; `Gload_and_process_step`'s own `_native_fix`
  call (both engines) ignored its own, real, in-scope `tolerances`
  parameter -- now `tolerances.fix_tolerance` directly.
  **Per direct user follow-up instruction**: `geo/tolerances.py`
  (`GeoTolerances`, the internal base class) now references the named
  `DEFAULT_<field>` constant directly for every field that has one
  (`fix_tolerance` -> `DEFAULT_FIX_TOLERANCE`, `sliver_edge_rel_tol` ->
  `DEFAULT_SLIVER_EDGE_REL_TOL`, alongside the pre-existing
  `min_face_width`/`min_solid_volume`/`scale`) instead of repeating a
  bare literal that happens to match -- `volume_tolerance`/
  `split_tolerance`/`scale_up_floor` have no such sibling constant and
  stay bare literals, matching every other field's own convention.
  `geouned.Tolerances` (`data_classes.py`, the public subclass) is
  deliberately untouched -- it has no `geo.constants` dependency at all
  by design (a public constructor should show plain numbers, not
  internal constant names).
  **Sewing tolerance group** (`BRepBuilderAPI_Sewing`) -- 2 real sites
  (`primitives.py::Gmake_shell`, `split_repair.py::
  _separate_edge_joined_components`, both engines), neither with a
  `tolerances` object in scope -> new `SEW_TOLERANCE`, a raw kernel-API
  parameter with no `Tolerances` field, same precedent as
  `KERNEL_TOL_E7`'s own `GeomAPI_IntSS` site.
  **Differential-geometry group** (`GeomLProp_CLProps`/
  `GeomLProp_SLProps`, `occ`/`ocp` only) -- 10 sites (curvature/
  value_at/normal_at/tangent_at, both engines) -> new `GEOM_PROP_TOL`.
  **UV-projection group** (`ShapeAnalysis_Surface.ValueOfUV`,
  `split_coaxial_cone.py`) -- 6 sites (both engines) -> new
  `UV_PROJECTION_TOL`.
  **Face-construction group** (`BRepBuilderAPI_MakeFace`, `repair.py`'s
  own `_retrim_freed_quadrics`) -> new `FACE_CONSTRUCTION_TOL`. **A
  real cross-engine asymmetry found**: `occ`'s own version reused its
  caller's own `dist_tol` parameter (a real, geometry-scaled value,
  `max(diag * min_length_ratio, MIN_SLIVER_EDGE_LENGTH)`) for this
  `MakeFace` call, while `ocp`'s own version used a separate, hardcoded
  `KERNEL_TOL_E6` instead -- same function, same purpose, drifted
  tolerance between engines. Per direct user decision ("aplicar a OCC
  lo mismo que ocp y crear la constante FACE_CONSTRUCTION_TOL"): both
  engines now use the new intrinsic constant, `occ` no longer reuses
  `dist_tol` for this specific call (its own separate use of `tol` for
  the function's own distance comparison a few lines above is
  untouched). **Verified specifically for this real behavior change**:
  a dedicated occ-engine 143-file before/after corpus scan (current
  HEAD vs. the edited tree) -- 0 real differences (the usual single
  pre-existing timeout-file diff, unaffected).
  **Point-classification / surface-intersection group**
  (`KERNEL_TOL_E7`, 2 genuinely different OCCT operations sharing one
  name) -- split into `SURFACE_INTERSECT_TOL` (`GeomAPI_IntSS`, `GPlane.
  intersect_plane`'s own native fallback, 1 site per engine -- already
  identified and left as a bare constant on 2026-09-22, now given its
  own distinct name once the second role sharing `KERNEL_TOL_E7` was
  found) and `POINT_CLASSIFY_TOL` (`BRepTopAdaptor_FClass2d`/
  `BRepClass3d_SolidClassifier`, 6 sites per engine, plus `freecad`'s
  own `Part.Shape.isInside` call -- the same role, already unified
  under this exact name/value back when it was still
  `LENGTH_TOL_E7`/`KERNEL_TOL_E7`).
  **Edge-projection tolerance** (`KERNEL_TOL_E8`) -- 1 real site,
  `build_shape_functions.py::cut_face`'s own `e.is_inside(point,
  KERNEL_TOL_E8)` call (the only real caller anywhere of `GEdge.
  is_inside(point, tolerance)`, a real parameterized method combining a
  point-to-curve projection distance and a parametric-range slack, not
  a value buried inside a native wrapper). Flagged as a genuine
  borderline case (a real, single, already-parameterized method with
  `tolerances` in scope at its one call site, unlike the other groups)
  -- user's decision: intrinsic constant regardless, new
  `EDGE_PROJECTION_TOL`.
  **`TOLERANCE_WELD_FLOOR`** -- `KERNEL_TOL_E3`'s own single,
  already-well-documented site (`decom_one_generators.py::generic_split`'s
  own BOPAlgo tolerance-weld detection floor) renamed for consistency,
  closing out the family (no role change, same value).
  `KERNEL_TOL_E3/E6/E7/E8` then had zero remaining references anywhere
  and were deleted from `geo/constants.py` -- **the entire former
  `KERNEL_TOL_E*` family is now gone**.
  **Verified**: all 3 engines' full suites green (ocp 292 passed/1
  skipped, occ 292 passed/1 skipped, freecad 258 passed/16 skipped, 0
  failures anywhere); a 143-file before/after corpus differential under
  `ocp` -- 0 real differences; a SEPARATE dedicated 143-file
  before/after corpus differential under `occ` specifically (to cover
  the real `_retrim_freed_quadrics` behavior change above) -- likewise
  0 real differences.
  **Registry near-miss ("fuzzy") log extended and de-duplicated,
  2026-09-27** (`GEOUNED/utils/basic_functions_part2.py`; diagnostic
  only, never changes an identity decision).
  - `Fuzzy(index, dtype, val, tol)` (the user cut it down to the stored
    surface's index plus `val`/`tol`; every call site dropped the
    now-unused surface/`options`/`tolerances`/`numeric_format`
    arguments, and `is_same_plane`/`is_same_cylinder` lost their
    `options`/`numeric_format` parameters -- callers in
    `geouned_classes.py` and the positional call in
    `cell_definition_functions.py` updated).
  - New near-miss quantities, each logged when its deviation falls
    between 0.5x and 2x its own tolerance (same band as the existing
    ones), each independent of the others: cylinder axis ANGLE
    (`cylAng`, vs `cyl_angle`), sphere radius and centre distance
    (`sphRad`/`sphCen`, vs `sph_distance`), cone semi-angle, apex
    distance and axis angle (`coneSAng`/`coneApx`/`coneAng`, vs
    `kne_angle`/`kne_distance`). Angles are absolute; distances scale
    with `relativeTol` exactly as in the decision. New registry
    wrappers `is_same_sphere`/`is_same_cone` (decision from
    `geo.surface_geometry.is_same_*_surface`, plus the log) are used by
    `SurfacesDict.add_sphere`/`add_cone` (new `fuzzy` parameter); cones
    get `fuzzy=True` where the cylinder equivalent already had it (torus
    VSurface, `kneCan`, `cc`). Shared quantities live in
    `geo/surface_geometry.py` (`axes_angle`, `sphere_radius_diff`,
    `sphere_center_offset`, `cone_semiangle_diff`, `cone_apex_offset`),
    used by both the predicates and the log, like the cylinder ones.
    Not covered: tori, and the planes' fuzzy still only logs distance.
  - **Repeated entries removed**: the same stored surface was compared
    again, and logged again, every time another face of the same
    surface was registered (the values differ only in the last digits).
    `Fuzzy` now writes one entry per (stored index, quantity, outcome,
    `val/tol` to 3 decimals); `reset_fuzzy_log()` clears that memory when
    `CadToCsg` starts. Measured on `divertor_cam_cut2.stp` (default
    tolerances, decompose + build_solid_definition): 66 `Fuzzy` calls ->
    36 entries. That is the floor for what an entry now identifies (the
    new surface is no longer printed).
  - **Loggers cleared on start**: `utils/log_utils.py::setup_logger` now
    removes and closes the handlers (and filters) an earlier run left on
    the process-global logger before adding its own. Before, every new
    `CadToCsg` in the same process added another `FileHandler` and each
    message was written once per previous run, to that run's files too.
    Side effect: two `CadToCsg` objects alive at once now log only to the
    most recent one's files.
  Tests: `tests/geo/test_same_surface_tolerances.py` (band of each new
  near-miss, off without `fuzzy`, de-duplication) and the new
  `tests/geo/test_log_utils.py`. Verified: full suites of all 3 engines
  green after this batch (ocp 320 passed/1 skipped, occ 320 passed/1
  skipped, freecad 286 passed/16 skipped, 0 failures).

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
- `docs/investigations/meta_surfaces_false_corpus_scan_2026-09-28.md` —
  first full-corpus d1suned comparison of `Options.meta_surfaces=True`
  vs `False`: the raw file-by-file failure list, and the conclusion that
  splits it into one real, shared, now-fixed GEOUNED bug
  (`_is_closed_by_winding`, see its own "Known open items" entry) vs 6
  files that are a genuine, unfixed `meta_surfaces=False`-specific
  limitation (composite meta-surfaces load-bearing for correct bounding
  in those geometries, not pure simplification).

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
