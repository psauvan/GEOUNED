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
XCAF with per-material color, plus the 6 "exotic quadric" surfaces
GEOUNED never produces, still unimplemented stubs under occ/ocp),
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
own docstring). The 6 exotic-quadric dataclasses themselves stay in
`_freecad_impl.py`, not `geo` -- deliberate, see the paragraph below.
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

## Current status (as of commit `8149030`, 2026-09-12)

- All 3 engines pass `tests/geo` + `tests/test_cadtocsg.py` +
  `tests/test_csgtocad.py` **in full** as of 2026-09-12 -- the latter is
  a new addition to that list: `test_cylbox_convertion` (both `[mcnp]`
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
- The 6 "exotic quadric" surfaces (`Gmake_elliptic_cone`,
  `Gmake_hyperboloid`, `Gmake_ellipsoid`, `Gmake_elliptic_cylinder`,
  `Gmake_hyperbolic_cylinder`, `Gmake_paraboloid`) remain unimplemented
  stubs in the `occ`/`ocp` backends
  (`GEOReverse/Modules/engine_dependency/_occ_impl.py`/`_ocp_impl.py`).

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
