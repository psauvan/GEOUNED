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
degenerate-torus single-sheet construction too (its current
`build_surface()` doesn't have it yet -- a known gap, same section), so
it's a genuinely shared primitive, not a GEOReverse-only one.
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

## Current status (as of 2026-09-13)

- All 3 engines pass `tests/geo` + `tests/test_cadtocsg.py` +
  `tests/test_csgtocad.py` **in full** as of 2026-09-13, occ/ocp also
  `tests/test_georeverse_occ_impl.py`/`_ocp_impl.py` (163 passed/2
  skipped freecad, 154 passed occ, 154 passed ocp) -- confirms the
  `Gmake_ellipsoid`/`Gmake_elliptic_cylinder`/`Gmake_torus_elliptic`
  work below is a zero-regression addition. `test_cylbox_convertion`
  (both `[mcnp]`
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
- `GEOReverse` (CsgToCad) work is deliberately paused: explicit user
  priority is to finish cleaning up known `GEOUNED`/`CadToCsg` bugs
  first. Do not start GEOReverse-side debugging unless asked.

## Known open items

(Supersedes every dated "Pending tasks" checkpoint inside the history
log — those are kept there for their own historical record, but this
list is the current one. Verified against the live code/tests on
2026-09-12 — see the history log's "Known-open-items audit" entry for
how. `Big_complex_cell/modelcell_cut1.stp`/`modelCell_670000.stp`, the
one item that audit found already fixed, has been dropped from this
list — see that entry for the verification numbers.)

### GEOUNED (`CadToCsg`, the forward STEP -> CSG pipeline)

- `AdjacentMultiplanePlanes` still needs the same RevCC-to-
  MultiRoundCorner extension flagged since 2026-08-21 — see the
  `project_mrc_adjacent_multiplane_pending` memory.
- `hylife-v06.stp` solid 45's slow decomposition (an O(n^2) same-surface
  face-adjacency cost on many duplicated cylinder/torus fragments) —
  root-caused, not fixed, explicitly deprioritized by the user.

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
- The `hylife-v06.stp` round-trip volume discrepancy: the reconstructed
  CAD volume comes back ~1.401x the true solid vs. GEOUNED's own
  d1suned tally of ~1.119x on the same unfixed file — the two don't
  agree, and this was never chased down once the real GEOUNED-side
  fix for that investigation was found via a different route.
- The 7 "exotic quadric" surfaces GEOReverse's own MCNP/OpenMC-XML
  `GQ`/`SQ` parser can produce: `Gmake_ellipsoid`,
  `Gmake_elliptic_cylinder`, `Gmake_torus_elliptic` (all three
  **implemented under `occ`/`ocp`, 2026-09-13** -- see below);
  `Gmake_elliptic_cone`, `Gmake_hyperboloid`, `Gmake_hyperbolic_cylinder`,
  `Gmake_paraboloid` remain unimplemented stubs in the `occ`/`ocp`
  backends (`GEOReverse/Modules/engine_dependency/_occ_impl.py`/
  `_ocp_impl.py`).
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
  legitimate degenerate/inner case), were updated to match. **Still not
  consumed by GEOUNED's own forward pipeline**: `GeounedSurface.
  build_surface()`'s Torus branch (`GEOUNED/utils/geouned_classes.py`)
  builds its degenerate-torus cutting tool via plain `Gmake_torus` (the
  full self-intersecting double-sheet primitive) and never reads the
  surface's own `Degenerated`/`a_sign` fields -- confirmed, not yet
  fixed, a real candidate for unnecessary over-cutting during boolean
  decomposition wherever a degenerate torus is a cutting tool (the two
  live consumers are `decompose/decom_one_generators.py::generic_split()`
  and `utils/boolean_solids.py::build_c_table_from_solids()`/
  `split_solid_fast()`). Deliberately deferred (per explicit user
  sequencing, "primero miramos el movimiento a geo" -- the move above
  came first) to a separate follow-up; verification target when it
  happens: `Solidos/test_models/Torus/2_degen_torii.stp` (2 degenerate
  tori, both inner/`a_sign=-1`, already confirmed converting cleanly
  today at tally `1.00032 +/- 0.17%`, 0 lost particles) as the
  before/after regression check, `Torus/Torus_solid1.stp` (non-
  degenerate) as the zero-behavior-change control.

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
