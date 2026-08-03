# GEOUNED — FreeCAD → pyOCC/CadQuery migration context

This file summarizes decisions made in a prior planning conversation
(claude.ai chat) about the GEOUNED codebase, so Claude Code has full
context without needing the original conversation pasted in.

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
  no pyOCC/CadQuery equivalent. Options: keep FreeCAD as an optional
  dependency solely for that export, drop `.FCStd` output, or treat it
  as a later migration phase.

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

`src/geouned/geo/` is now the ONLY place in GEOUNED allowed to import
`Part`/`FreeCAD`/`BOPTools` (`GEOReverse` is explicitly out of scope for
all of this, per the Project section above). Structure:

- `geo/vector_geometry.py` — pure math, zero FreeCAD dependency: `GVector`,
  `GBoundBox`, `GLabelNode`, and geometric predicates (`is_same_plane_surface`,
  `is_parallel`, `plane_value_at`, ...). This is what a future `_occ_impl.py`
  would reuse unchanged.
- `geo/_freecad_impl.py` — the FreeCAD-specific implementation: analytic
  surface/curve descriptor classes (`GPlane`, `GCylinder`, `GCone`, `GSphere`,
  `GTorus`, `GLine`, `GCircle`, `GEllipse`, `GBSpline` — each constructed
  directly from its native FreeCAD surface/curve object, plus a
  `.from_values(...)` classmethod on
  `GPlane`/`GCylinder` for the rare case of building one from already-known
  values with no native face behind it); topology classes (`GEdge`, `GWire`,
  `GFace`, `GShell`, `GSolid`) that are **eagerly** built from their native
  equivalent and carry real behavior as methods (`gsolid.is_inside(point)`,
  `gface.value_at(u, v)`, `gsolid.find_interior_point()`, `gsolid.export_step(...)`,
  ...) instead of being passed to a separate backend object — with one
  deliberate exception: `GFace.wires()`/`.outer_wire()` are lazy
  (computed on first call, then cached), not eager, since nothing in
  GEOUNED actually needed them on most faces and `.outer_wire()` runs a
  real heuristic over every wire of the face — see "cut_face/cut_box take
  a GFace" below; free `Gmake_*`
  constructor functions (`Gmake_box`, `Gmake_cylinder`, `Gmake_half_space`,
  `Gmake_polygon_face`, ...) and free operation functions for anything
  combining more than one independent shape (`Gcut`, `Gcommon`, `Gfuse`,
  `Gsplit`, `Gin_contact`, `Gdistance`, `Gload_step`, `Gload_step_labels`,
  `Gexport_step`).
- `geo/__init__.py` — the single import point for the rest of GEOUNED:
  `from ...geo import GSolid, Gmake_cylinder, ...`.

**As of the `core.py::_set_geometry_bounding_box` fix (see "`core.py`'s
last native import closed out" below), zero files in `GEOUNED` carry a
direct `import Part`/`import FreeCAD` anymore** — `geo/_freecad_impl.py`
is now, in practice and not just by convention, the only file in the
whole codebase allowed to. `core.py` was the last holdout; before it,
`utils/meta_surfaces_utils.py`'s `import Part` was the previous-to-last
holdout,
previously documented here as deliberately deferred because narrowing its
`isinstance(e0.Curve, (Part.Circle, Part.Ellipse, Part.Hyperbola,
Part.Parabola))` check would silently change Hyperbola/Parabola behavior
— has since been closed out too: `planar_edges`/`edge_1D`/`spline_2D`
were rewritten to operate on `GEdge`/`GVector` throughout (the file's
`edges` parameters are `list[GEdge]` now, sourced from `GWire.Edges`, not
native edges), and the Hyperbola/Parabola case is handled explicitly
instead (`type(curve) in (GCircle, GEllipse)` narrows on purpose now,
with an explicit `elif curve is None: return False` branch for the
unsupported types — see "`GEdge`/`GWire` enrichment..." below for the
full account of why and how). Files documented in earlier passes
(`conversion/cell_definition_functions.py`, `utils/geometry_gu.py`,
`utils/geouned_classes.py`, `utils/build_shape_functions.py`) have since
been closed out completely:
- `cell_definition_functions.py`/`geometry_gu.py` — see "Plane/Cylinder/Cone
  unification" below for how `gen_plane_sphere`'s infinite `Part.Plane`
  was replaced (a finite plane clipped to a box centered on the sphere,
  2×radius +1% — verified numerically identical `distToShape` results
  against bounded face fragments, the only kind `same_faces` ever is in
  practice). No `Part.Vertex(...)` equivalent was needed for
  `same_wire`'s point-on-edge check either: it reused the native
  `edge.isInside(point, tolerance, True)` method already used elsewhere
  (`build_shape_functions.py::cut_face`), verified empirically against
  `distToShape` across 19 cases (line/circle/ellipse/BSpline curves,
  endpoints, the tolerance boundary, points beyond a curve's trim).
  `GEdge.is_inside(point, tolerance)` was added to `geo` as the general
  form of this. `same_wire` itself, along with `join_wires`/
  `merge_two_wires`/`common_vertexes`/`cut_wires`/`vertex_edge_index`
  and `ShellGu.set_outerWire`, was then deleted outright — confirmed dead
  code (the only call site was commented out with a note that it already
  errored, and nothing else referenced any of them).
- `geouned_classes.py`: `GeounedSurface.build_surface`'s
  `Box = FreeCAD.BoundBox(boundBox)` became `Box = to_gboundbox(boundBox)`
  (and each `Box.enlarge(10)` became `Box = Box.enlarged(10)`, `GBoundBox`
  being non-mutating) once it was confirmed every downstream consumer
  (`makePlane`/`makeCylinder`/`makeCone`, `makeMultiPlanes`,
  `build_complex_shape`/`myBox`/`Objects.py`'s `buildShape` methods)
  already normalizes via `to_gboundbox()` or plain attribute access,
  tolerating either native or `GBoundBox` input interchangeably.

- `utils/build_shape_functions.py`: fully closed out — see "convert at
  the origin: build_shape_functions.py's GVector-only rewrite" below.

`core.py`'s `self.geometry_bounding_box = FreeCAD.BoundBox(...)` in
`_set_geometry_bounding_box` — the very last holdout, previously kept
native because it's consumed well outside this migration's original
scope (`void.py`, `write_files.py`) — has since been closed out too; see
"`core.py`'s last native import closed out" below.

### Plane/Cylinder/Cone unification (`build_region/Objects.py`)

`build_shape_functions.py` (used directly by `GeounedSurface.build_surface`
for simple surfaces) and `build_region/Objects.py`'s `Plane`/`Cylinder`/
`Cone` classes (used via `get_cell_object`/`BuildDepth` to reconstruct
composite surfaces — RoundCorner/MultiRoundCorner/Can/TCone) used to each
have their own copy of "analytic surface + bounding box -> finite clipped
solid" for these 3 shapes. Now unified as 3 pure functions in
`build_region/Objects.py`, in `GVector`/`GBoundBox` space, with no
duplication left anywhere: `plane_polygon_from_box(normal, offset, box)`,
`cylinder_from_box(center, axis, radius, box)`,
`cone_from_box(apex, axis, tan, box)`. `build_shape_functions.py::makePlane`/
`makeCylinder`/`makeCone` are now thin wrappers around these; `Objects.py`'s
`Plane.buildShape`/`Cylinder.buildShape`/`Cone.buildShape` (`not dblsht`
branch) call them too.

Two real, pre-existing bugs were found and fixed while unifying, both
predating this migration entirely (confirmed via `git show` against the
last pre-migration commit) and both empirically verified before/after:
- **Cylinder margin**: the two duplicate implementations used different
  end-padding (`build_shape_functions.py`: fixed 5 units; `Objects.py`:
  10% of height) — not a bug exactly, but verified empirically (splitting
  a real solid against both tool variants, at tiny/normal/huge box
  scales) that the margin size doesn't affect the split result, only that
  it's nonzero. Unified on the 10%-proportional version.
  `Objects.Cylinder.buildShape` also now converts its box to `GBoundBox`
  up front (`Plane.buildShape` already did; `Cylinder.buildShape` used to
  work in raw native-box space throughout, an inconsistency now closed).
- **Cone direction (real bug)**: `makeCone` only scanned `dmax` (box
  corners projected onto `+axis` from apex) and returned `None` if
  `dmax <= 0`, uncaught by its only caller
  (`geouned_classes.py::build_surface` did `self.shape, self.shell =
  makeCone(...)`, an unhandled `TypeError` on `None`). Reproduced for
  real: a frustum built with the wider end on the `-axis` side of its
  (extrapolated) apex — `Part.makeCone(40, 20, 100)` — has
  `dmax <= 0` for its own solid's bounding box. `Objects.py`'s
  `Cone.buildShape` didn't crash here but was equally wrong: it always
  built forward along `+axis` regardless, so in the same case it would
  silently build a cone pointing *away* from the box, covering nothing.
  Root cause: OCC's native `Cone.SemiAngle` is *signed* exactly to record
  which direction the real material is in (verified empirically: the
  above frustum reports `SemiAngle < 0`, matching a sample point actually
  on the real face) — this sign already survives as far as
  `geouned_classes.py`'s `tan = math.tan(kne.Surf.SemiAngle)`, but got
  discarded (`abs()`) right before use. Fix: `cone_from_box` builds along
  `axis if tan >= 0 else -axis` (trusting the inherited sign, never
  re-derived by guessing from the box), sizes only by scanning in that
  already-correct direction, and returns `None` only when the box
  genuinely doesn't extend that way at all.
  `build_surface`'s Cone branch now guards the `None` case explicitly
  instead of blindly unpacking.

### `GPlane.intersect_plane` — plane-plane intersection in pure math

`makeMultiPlanes`'s `cut_face`/`cut_box` used to build a real infinite
`Part.Plane` purely to compute `face.Surface.intersect(plane)` against a
box face's (always planar) surface — a plane-plane intersection, which is
closed-form linear algebra with no CAD kernel needed. Added
`GPlane.intersect_plane(other) -> GLine | None` (`geo/_freecad_impl.py`)
and `GLine.from_values(position, direction)` (mirroring `GPlane`/
`GCylinder`'s existing `.from_values(...)` for a descriptor with no
native object behind it).

This was **not** a drop-in naive formula — empirically verified in stages:
- The textbook closed-form point formula (dividing by `|n1×n2|²`) matches
  native to floating-point precision for well-separated planes, but is
  genuinely numerically unstable approaching parallel: at ~1e-4 rad from
  parallel it already diverges from native by a relative ~1e-3, not just
  round-off noise (confirmed by re-deriving the point from OCC's own
  *stored* Axis/Position — after eliminating input round-trip error, the
  divergence persisted).
- A numerically-stabilized version (drop whichever coordinate axis the
  intersection direction is most aligned with, solve the remaining
  well-conditioned 2×2 system instead of the raw cross-product formula)
  extends the safe range but still diverges below roughly 1e-4 to 1e-6
  rad from parallel.
- Given this project's specific history with near-tangent configurations,
  `intersect_plane` is hybrid rather than trusting the stabilized formula
  everywhere: pure `GVector` math when `|n1×n2| >= 0.05` (~2.9 deg from
  parallel, a wide margin below the verified-safe ~0.01 rad boundary);
  below that, builds a transient native `Part.Plane`/`.intersect()` and
  wraps its result back into a `GLine`. Verified 42/42 against native
  across random well-separated pairs *and* near-parallel pairs down to
  1e-6 rad (the latter exercising the fallback branch, so those match
  exactly — they *are* native).

### `GLine.intersect_line` — line-vs-edge intersection, same treatment

`cut_face` then still needed the resulting `GLine` intersected against
each box-face edge to find where the cutting plane crosses the face's
boundary (`l.intersect(e.Curve)`). Box-face edges (built by
`makeBoxFaces`/`Gmake_polygon_face`) are always straight, so this reduces
to line-vs-line — and, since both lines always lie in the same (box
face's) plane by construction, to *coplanar* line-vs-line specifically.
Added `GLine.intersect_line(other) -> GVector | None`, same hybrid
treatment and same verified boundary as `intersect_plane` (pure math
matches native to floating-point precision for well-separated directions
and for skew — non-coplanar — pairs, which are unambiguous: the
coplanarity gap is either ~1e-15 or clearly nonzero, never borderline;
falls back to native below `|d1×d2| < 0.05`). Verified 46/46 against
native (coplanar random pairs, skew pairs, near-parallel pairs exercising
the fallback). No `Part.Line` of any kind remains in
`cut_face`/`cut_box`/`makeMultiPlanes`.

### `cut_face`/`cut_box` take a `GFace`, not a native face

Once `cut_face` no longer needed `Part.Plane`/`Part.Line`, its `face`
parameter itself was switched from a native `Part.Face` to a `GFace`
(`cut_box` now wraps each native box face via `GFace(f)` before calling
it). `GFace.Surface`/`.Edges`/`.Vertexes` are already eagerly classified,
so `cut_face` no longer needs its own `Gclassify_surface(face)`/
`Gclassify_curve(e)` calls, and point-membership uses `GEdge.is_inside(...)`
(the same method added earlier for `same_wire`) instead of the native
`e.isInside(...)`.

This surfaced that `GFace.Wires`/`GFace.OuterWire` — eagerly computed in
`GFace.__init__` alongside `Surface`/`Edges`/`Vertexes` — had **zero
callers anywhere in GEOUNED** (confirmed by grep: every existing
`.OuterWire.Edges` use in the codebase goes through `FaceGu`, the
separate native wrapper in `geometry_gu.py`, never through `GFace`).
`OuterWire` in particular runs `_pick_outer_wire`'s heuristic (compares
mean vertex-to-centroid distance across every wire of the face) on
*every* `GFace` ever constructed, for a value nothing read. Converted to
lazy, cached methods — `.wires()`/`.outer_wire()` — computed only on
first actual call. Since `GFace` is constructed pervasively throughout
GEOUNED (every `GSolid`/`GShell`'s `.Faces`), this wasn't just a
`cut_face`-local fix: the full `test_cadtocsg.py` suite dropped from
~180s to ~134s after this change alone.

### Convert at the origin: `build_shape_functions.py`'s GVector-only rewrite

Once `cut_face`/`cut_box` took a `GFace`, the rest of the file still mixed
native and `GVector` inputs, converting back and forth (`to_fc_vector`/
`to_gvector`) at whatever point inside the file happened to need one or
the other. Fully rewritten instead so every function *defined in this
file* accepts only `G*` types (`GVector`/`GPlane`/`GFace`/`GBoundBox`) and
contains zero `to_fc_vector`/`to_gvector` calls of its own — conversion is
pushed out to the call site that originates the native value, tracked by
running the test suite and following each resulting `TypeError` back to
its source rather than converting locally. `makePlane`/`makeCylinder`/
`makeCone` now take `GVector`s and return native shapes (their only native
consumers, `geouned_classes.py::build_surface`, convert once at the call);
`makeMultiPlanes` builds a `GPlane` at each iteration instead of a native
plane; `makeBoxFaces` returns `list[GFace]`, which made `cut_box` and the
final `Gmake_shell(...)` call in `makeMultiPlanes` stop needing per-call
`GFace(f)` wrapping; `sort_points` now accumulates in pure `GVector` space
(`.length` instead of `.Length`). Two more dead functions surfaced and
were deleted along the way: `check_sign`/`intersection` (a broken,
unused duplicate of the differently-signatured, actually-used
`check_sign` in `boolean_solids.py`) and `cylinder_cut_box` (zero callers
anywhere in `GEOUNED`), which in turn made `makeBoxFaces`'s
`isinstance(box[0], FreeCAD.Vector)` branch unreachable too. Only 2
`to_gvector` calls remain anywhere in the file, in `makeMultiPlanes`,
reading `PlaneParams.Axis`/`.Position` — the one native-holding boundary
this file can't push further out, since `PlaneParams` is a domain type
shared far beyond this file (see "Params class family" below).

### Dead-code cleanup: `basic_functions_part3.py` and 4 broken `__eq__` methods

`utils/basic_functions_part3.py` (183 lines) was a near-exact duplicate of
`basic_functions_part2.py`'s `is_same_plane`/`is_same_cylinder`/
`is_same_cone`/`is_same_sphere`/`is_same_torus`/`Fuzzy`/
`is_duplicate_in_list`, but the older, unmigrated, native-only version
(`.isEqual()`/`.Length` instead of `to_gvector()` + `.is_equal()`/
`.length`). Confirmed via exhaustive grep that nothing imports it anywhere
(only appears in the auto-generated `SOURCES.txt` packaging artifact) —
deleted outright. `basic_functions_part1.py`'s `CylinderOnlyParams`/
`ConeOnlyParams`/`SphereOnlyParams`/`TorusOnlyParams` each also carried a
broken, unreachable `__eq__` (compared `type(other)` against the wrong
wrapper class; `ConeOnlyParams`'s additionally referenced a nonexistent
`self.apex` lowercase typo) — confirmed via exhaustive search that the
only live `==` paths on `GeounedSurface`/its `.Surf` always compare
`Plane`-type params (triggering the correct `PlaneParams.__eq__`), so
these 4 were never actually reachable — deleted.

### The `Params` class family, `bVar`/`region`/`components`, and the two surface-numbering scopes

`GeounedSurface.Surf` is always one of ~16 "`*Params`" classes in
`basic_functions_part1.py`, in three informal tiers: pure geometric
descriptors (`PlaneParams`, `CylinderOnlyParams`, `ConeOnlyParams`,
`SphereOnlyParams`, `TorusOnlyParams` — largely redundant with `geo`'s
`GPlane`/`GCylinder`/`GCone`/`GSphere`/`GTorus`, not yet consolidated);
"basic surface + bounding plane(s)" (`CylinderParams`, `ConeParams`,
`SphereParams`, `TorusParams`); and composite/meta-surfaces combining
several of the above via AND/OR (`MultiPlanesParams`, `CanParams`,
`TConeParams`, `RoundCornerParams`, `MultiRoundCornerParams`,
`ReversedConeCylParams`) — the last group with inconsistent field naming
for the same underlying idea (`s1`/`s1_configuration` vs `p1`/
`p1_configuration` vs `Planes`+`Configuration`), not yet consolidated.

Every `GeounedSurface` has `.bVar` (a `BoolVariable`, the signed id this
surface is referred to by) and, for composite types, `.region` (a
`BoolSurface` wrapping a `BoolSequence` — the AND/OR boolean definition
over its components' `bVar`s). **These get computed/assigned in two
different phases with two genuinely different numbering scopes, and
conflating them would be wrong**:
- **Decompose phase** (`utils/functions.py` constructs the composite
  `GeounedSurface`, then `GeounedSurface.build_surface()` →
  `build_complex_shape()` → `get_cell_object()` builds its CAD shape to
  split the solid): `.bVar` here is a throwaway *local* id (sequential,
  assigned right at construction in `functions.py`) — its only job is to
  keep the AND/OR expression internally consistent for that one CAD
  build; which actual integers are used doesn't matter and nothing
  outside this one call depends on them.
- **Conversion phase** (`conversion/cell_definition.py`, walking every
  face of every decomposed solid element to reconstruct the whole
  solid's CSG expression): `Surfaces.add_can`/`add_tcone`/`add_plane`/...
  (`MetaSurfacesDict`, `geouned_classes.py`) **overwrite `.bVar` in place**
  on the same instances with the *global*, deduplicated canonical id —
  reusing an existing number if `is_same_plane`/`is_same_cylinder`/...
  (with tolerance) finds an existing geometric match anywhere in the
  model, incrementing `self.surfaceNumber` otherwise. Two geometrically
  identical planes bounding two different solid elements MUST end up
  with the same number here — that's the whole point of this phase.
  `.region` is computed for the first time here too.

Because of this split, `get_cell_object` (decompose-phase CAD building)
and `MetaSurfacesDict.Can_region`/`.TCone_region` (conversion-phase
global registration) **cannot** just have one read the other's already-
computed `.region` — at the time `get_cell_object` runs, conversion
hasn't happened yet and `.region` doesn't exist. What they *can* share
(and, before this pass, did not) is the AND/OR **rule** itself, since it
only depends on which ids and orientations/configurations are involved,
never on whether those ids are local or global. `round_corner_region`/
`multi_round_corner_region` (`basic_functions_part1.py`) already worked
this way — pure functions of `(ids, configuration) -> BoolSurface`,
called identically by both `get_cell_object` and
`MetaSurfacesDict.add_roundcorner`/`add_multiroundcorner`. **Can and
TCone did not**: `MetaSurfacesDict.Can_region`/`.TCone_region`
(geouned_classes.py) and `get_cell_object`'s `"Can"`/`"TCone"` branches
(build_region.py) each hand-rolled the identical AND/OR truth table
inline (same "AND Rev -> c p / AND Fwd -> -c :p / ..." comments, copied).
Extracted `can_region(cid, cyl_orientation, surf_list)` and
`tcone_region(cid, cone_orientation, surf_list)` as the same kind of pure
function (`basic_functions_part1.py`, next to `round_corner_region`);
`MetaSurfacesDict.Can_region`/`.TCone_region` now only resolve ids
(via `primitive_surfaces.add_*`, including the plane-sign-flip dance,
factored into a shared `_resolve_plane_id` helper) and call the pure
function; `get_cell_object` reads whatever is currently on `.bVar` and
calls the same pure function. Verified via `tests/geo` (107/107) and
`tests/test_cadtocsg.py` (50/50).

`GeounedSurface` also now has `.components: dict[abs(id), GeounedSurface]`
for Can/TCone — the numbering↔surface relation, materialized directly on
the object (built once in `Can_region`/`TCone_region`, alongside
`.region`) instead of being re-derived by walking `.Surf.X.Surf.Y` by
hand wherever it's needed. This let `boolean_solids.py::check_sign`'s
Can/TCone point-classification branches collapse from ~85 lines of
per-type `.Surf` walking into one generic block:
`surfSet = {id: check_sign(point, comp) > 0 for id, comp in
surf.components.items()}; surf.region.region.evaluate(surfSet)`. This
also fixed a latent, real bug: the old hand-walk matched
`si.Type == "cylinder"` (lowercase) against the real tag `"Cylinder"`
(capital C), so a Cylinder-type Can component's plane/cylinder ids were
silently dropped from `surfSet` during point classification — never
triggered before because nothing exercised it this way.

### RoundCorner/MultiRoundCorner: `.components`, and a real MultiRoundCorner bug found while adding it

RoundCorner already shared its AND/OR rule the right way (`round_corner_region`,
a pure function of ids -- same pattern `can_region`/`tcone_region` were
given above), so only `.components` needed adding: built in
`MetaSurfacesDict.get_roundCorner_region` (planes, then cylinder, then the
cylinder's own bounding plane -- the same order `check_sign`'s hand-written
version iterated in, so its incremental, short-circuiting
`multiDef.evaluate({single_id: value})` stays exactly as cheap as before).
`get_cell_object`'s `"RoundCorner"` branch was untouched -- it already
called the shared `round_corner_region`, no duplicate logic existed there.

`MetaSurfacesDict.get_overlap_rc` (~145 lines, merging two adjacent round
corners into one region) turned out to have **zero callers anywhere in
GEOUNED** -- confirmed by grep, and further confirmed dead by the fact
that `get_roundCorner_region`'s new `(region, components)` return shape
would have broken its `rc1_region * rc2_region`-style calls immediately
had anything actually invoked it. Deleted, along with the now-unused
`from .data_constants import mask` import it was the only consumer of.

Extending `.components` to MultiRoundCorner surfaced 3 real, pre-existing
bugs, all attribute-path mistakes consistent with this feature never
having been exercised end-to-end (`add_multiRoundCorner` even carried its
own comment admitting as much: *"will not return correct results for any
multi corner configuration"*):
- `add_multiRoundCorner`'s plane-sign-flip block wrote
  `cylplane.Axis = -cylplane.Axis` (should be `cylplane.Surf.Axis`) and
  `rc.Surf.Plane.bVar = pcid` (`RoundCornerParams` has no `.Plane`
  attribute, only `.Planes`/`.Cylinder` -- should be `cylplane.bVar`).
- `boolean_solids.py::check_sign`'s MultiRoundCorner branch read
  `rc.Surf.Plane`/`rc.Surf.Cylinder` directly off each corner -- the
  first is the same nonexistent-attribute mistake (guaranteed
  `AttributeError`), the second reads the Tier-2 `Cylinder` wrapper's
  `.bVar`, which never gets assigned (only the primitive
  `rc.Surf.Cylinder.Surf.Cylinder` does, in `add_multiRoundCorner`).
  Fixed to match `get_cell_object`'s already-correct
  `rc.Surf.Cylinder.Surf.Plane`/`.Surf.Cylinder` -- and, once `.components`
  existed for MultiRoundCorner too, this branch merged with RoundCorner's
  into one `elif surf.Type == "RoundCorner" or surf.Type ==
  "MultiRoundCorner":` block, both driven by `.components` identically.

**Important caveat**: this fix is *not* verified end-to-end against real
geometry. Instrumented `check_sign` directly and ran it across all 39
STEP files in the user's `RoundCorners` model set (13 of which do produce
a MultiRoundCorner surface, up to 6 in one file) -- `check_sign` was
never actually called, on any surface type, in any of the 39 files.
`Gsplit`'s normal CAD-based split path always resolved the cut before
ever needing the algebraic fallback `check_sign` provides. So the fix is
correct *relative to the established, working pattern used identically
in `get_cell_object`* and no longer crashes if the fallback path is ever
taken, but nobody has been able to confirm the *values* it produces are
geometrically right, because no available test model forces that code
path to run. If a case is ever found that does exercise it, that's the
first real end-to-end validation this branch will have had.

### `write/`: `mcnp_like/` + `openmc/` subpackages, `CommonInputWriter` mixin

`write/mcnp_format.py`/`serpent_format.py`/`phits_format.py`/`openmc_format.py`
(the 4 output-format writer classes) had 4 near-identical implementations
of several methods. Reorganized into `write/mcnp_like/` (MCNP, Serpent,
PHITS — the 3 MCNP-lineage text formats, sharing real card-syntax
conventions: full-line vs inline comment characters, `C`/`%`/`$`-style
banners) and `write/openmc/` (structurally different: XML/Python output,
no comment-card concept), per the user's explicit request to split by
that boundary rather than force all 4 into one hierarchy. `write/functions.py`
(already the shared home for `mcnp_surface`/`serpent_surface`/`phits_surface`/
`open_mc_surface`/`write_*_cell_def`/`CardLine`/etc. — sitting above both
new subpackages) gained 3 free functions used by *all 4* formats:
`get_cell_surf_summary`, `simplify_planes`, `sorted_surfaces`.

`write/mcnp_like/common_format.py`'s `CommonInputWriter` mixin (inherited
by `McnpInput`/`SerpentInput`/`PhitsInput`) adds the methods confirmed
duplicated *only* among the 3 MCNP-lineage formats on top of those 3 free
functions: `get_solid_cell_volume`, `write_cell_block`, `write_surface_block`,
`write_surfaces` (parametrized per-subclass by `_surface_formatter`/
`_format_name` class attributes), and `comment_format`/`comment_line`
(parametrized by `inline_comment_char`/`line_comment_char` — `$`/`C` for
MCNP, `%`/`%` for Serpent, `$`/`$` for PHITS). `OpenmcInput` does not
inherit this mixin (XML/Python output has no equivalent concept for most
of it) — it calls the 3 shared free functions directly instead.
`PhitsInput` keeps its historical `write_phits_surfaces`/
`write_phits_surface_block` method names (its own internal call sites,
and potentially external callers, use them) via one-line aliases to the
mixin's `write_surfaces`/`write_surface_block`; MCNP's extra
`prnt3PPlane` handling in `simplify_planes` is layered on top via
`super().simplify_planes(Surfaces)`.

**The `sorted_surfaces` unification is a real correction, per explicit
user confirmation** — the pre-existing MCNP version applied
`Surfaces.IndexOffset` when relabeling each surface's `bVar`
(`s.bVar = bsurf.copy(Surfaces.IndexOffset + abs(bsurf.value()))`);
Serpent/OpenMC/PHITS's versions skipped this relabeling entirely. The
user confirmed this was unintentional (only MCNP had received "the
adequate modifications") and that all 4 formats should behave like MCNP
here — so the free function now used everywhere is MCNP's original
version. Checked how much this actually changes today:
`MetaSurfacesDict` is always constructed with `offset=0` (`core.py`
never wires `settings.startSurf` through to it), so `IndexOffset` is
currently always 0 in practice — this fix corrects a currently-dormant
inconsistency, not a currently-visible bug; it will matter once/if
`IndexOffset` ever gets wired to a nonzero value.

Verified two ways: `tests/geo` (107/107) + `tests/test_cadtocsg.py`
(50/50), and a direct before/after byte-diff of all 5 generated output
files (`.mcnp`/`.serp`/`.inp`/`.xml`/`.py`) on two STEP files (a simple
one, and one producing a MultiRoundCorner surface) — identical except
for the non-deterministic `Creation Date` timestamp line, confirming
zero observable output change from the reorganization itself.

**Deferred, not yet done** (this is the live edge of the ongoing
`*Params`/`GeounedSurface` redesign — resume here):
- `settings.startSurf` is never wired to `MetaSurfacesDict`'s `IndexOffset`
  (`core.py:385` always constructs it with the default `offset=0`) --
  the `sorted_surfaces` unification above is currently a no-op in
  practice because of this. Connect them so `startSurf` actually takes
  effect.
- Tier-1 `*OnlyParams` vs `geo`'s `GPlane`/`GCylinder`/`GCone`/`GSphere`/
  `GTorus`: the storage-type duplication (native vs `GVector`) is closed
  (see below) -- but they're still two separate *class* definitions.
  Considered wrapping `*OnlyParams` around a `geo` descriptor via
  composition instead; rejected once traced through that nothing in
  `GEOUNED` ever needs to hand a `*OnlyParams` instance to a `geo`
  function expecting a real `GPlane`/`GCylinder`/etc (construction
  always happens by pulling the individual fields back out first) --
  so the extra indirection would have bought nothing. `geo`'s
  `GCone`/`GSphere`/`GTorus` did gain `.from_values(...)` classmethods
  either way (mirroring `GPlane`/`GCylinder`'s existing ones), for
  whenever that judgment call needs revisiting.

### Tier-1 `*OnlyParams` cleanup: dead `dimL`/`dimL1`/`dimL2`/`dimR` fields

Mapped every reader of the "extra" fields `CylinderOnlyParams`/
`ConeOnlyParams`/`PlaneParams` carry that `geo`'s `GCylinder`/`GCone`/
`GPlane` don't (`dimL`, `dimR`, `dimL1`, `dimL2`, `real`). Findings:
`CylinderOnlyParams.dimL`/`.real` and `ConeOnlyParams.dimL`/`.dimR`/
`.real` were never read anywhere in `GEOUNED/` outside their own class
definitions -- confirmed dead, deleted (along with the now-pointless
`real=True` constructor parameter on both classes, since the sole
caller of each, `geouned_classes.py`, always calls with one positional
arg). `PlaneParams.real` **is** live (read 4x in
`MetaSurfacesDict.add_plane`, as `stdtol=plane.Surf.real`, to choose
between `tolerances.pln_angle`/`pln_distance` and
`tolerances.add_pln_angle`/`add_pln_distance` -- `real=True` means the
plane came from an actual face in the model, `real=False` marks a
synthetic/auxiliary plane the code built itself, e.g. to close a
cylinder or cone) -- kept.

`PlaneParams.dimL1`/`.dimL2` were also live, but with only one reader:
`is_same_plane`'s relative-tolerance branch,
`tol = pln_distance * max(p2.dimL1, p2.dimL2)`. Deleting them required
a replacement scale, since `test_with_relative_tol_true` genuinely
exercises `relativeTol=True` (unlike the currently-dormant
`IndexOffset` case above). Every other `is_same_*` (cylinder/cone/
sphere/torus) already scales its relative tolerance by the surface's
own distance from the origin (`max(center1.length, center2.length)` or
equivalent) rather than by a face-extent measure -- `is_same_plane`
already computes exactly that value for a plane (`d1`/`d2`, the
signed distance from origin along the plane's own normal) as part of
the same-plane test itself, so the natural, pattern-matching
replacement is `tol = pln_distance * max(abs(d1), abs(d2))`, reusing
those local variables instead of `dimL1`/`dimL2`. Verified via
`tests/geo` (107/107) and `tests/test_cadtocsg.py` (50/50, including
`test_with_relative_tol_true`).

Constructor call sites (~30, across `decompose/`, `conversion/`,
`utils/functions.py`, `utils/meta_surfaces_utils.py`) were left
untouched -- they still pass the same 4-5 element tuples (many with
placeholder `1`/`1.0` values that were never geometrically meaningful
in the first place, per the mapping); `PlaneParams`/`CylinderOnlyParams`/
`ConeOnlyParams.__init__` just no longer read the now-irrelevant
positions. Not touched, and still dead: `Plane3PtsParams`
(`basic_functions_part1.py`) -- confirmed never instantiated anywhere,
same `dimL1`/`dimL2` pattern, but out of scope of this specific pass.

### Tier-1 `*OnlyParams`: native `FreeCAD.Vector` -> `GVector` storage

With the dead fields gone, converted `PlaneParams`/`CylinderOnlyParams`/
`ConeOnlyParams`/`SphereOnlyParams`/`TorusOnlyParams` to store `GVector`
directly instead of native `FreeCAD.Vector` via `_to_native_vector`
(deleted, along with its docstring's justification for existing --
"consumed by `write/*.py` with native-only assumptions like
`.isEqual()`" -- which an exhaustive grep confirmed doesn't correspond
to any surviving code path; same stale-blocker pattern already found
once this session with `basic_functions_part3.py`). Considered (and
rejected, per explicit user preference) wrapping each class around a
composed `geo` descriptor instead of just swapping the field type --
see the "Deferred" list above for why.

This is the first change in the whole migration where **static analysis
of "who reads these fields" wasn't sufficient** -- a grep-based survey
(by a sub-agent) concluded every consumer already tolerated `GVector`
(same operators: `.dot`, `.cross`, `.x/.y/.z`, scalar `*`, indexing).
Running the real test suite immediately surfaced 5 native-only call
sites the grep missed, because each reaches these fields through an
intermediate object rather than reading `.Surf.Axis`/`.Position`/etc
directly:
- `build_region.py::get_surface()` read `.Surf.Axis`/`.Position`/
  `.Center`/`.Apex` and packed them straight into `Objects.py`'s
  `Plane`/`Cylinder`/`Cone`/`Sphere`, whose own algebra
  (`splitFunction.py`'s `surface_side`/`btwPPlanes`, plus native
  `Matrix.multVec` in `.transform()`) was native-only at the time --
  fixed then by converting explicitly at `get_surface()` (`to_fc_vector`
  on each field). Superseded shortly after by the deeper fix below,
  which eliminates the native detour entirely instead of just moving
  where the conversion happens.
- `write/functions.py::simplify_planes` (added this session, during the
  writer-unification pass) reassigned `p.Surf.Axis = to_fc_vector(GVector(...))`
  -- mirroring the old, pre-GVector-migration behavior it was ported
  from -- now just `GVector(...)` directly.
- `utils/functions.py::convex_planes`: native in-place `.normalize()`
  (twice) -> `GVector.normalized()` (returns new, doesn't mutate);
  native `.Length` -> `GVector.length`; its `zaxis` parameter arrives
  native from its only caller (`cyl.Surface.Axis`, a real native
  surface) -- converted once via `to_gvector` at the top of the
  function so the rest of it is uniformly `GVector`.
- `utils/meta_surfaces_utils.py::commonEdge`-adjacent code (2 sites):
  `d.dot(adjPlane.Axis)` where `d` comes from native `face.valueAt(...)`
  (this file is one of the 2 remaining files allowed direct
  `import Part`/`FreeCAD`, so `d` staying native is correct) and
  `adjPlane.Axis` is now `GVector` -- fixed with `to_fc_vector` at the
  point of use, converting the `GVector` side rather than the native side.

Verified via `tests/geo` (107/107) and `tests/test_cadtocsg.py` (50/50)
-- but only after 3 rounds of "run the full suite, fix what it finds,
re-run" (50 -> 49 -> 2 -> 0 failures). **Lesson for the next
`*Params`/`GeounedSurface` step**: grep-based "who consumes this field"
surveys are necessary but not sufficient when the field passes through
an intermediate object (a `.params` tuple, a locally-derived variable)
before reaching the operation that cares about its type -- always
follow up with a real end-to-end run, not just static tracing.

### `build_region/Objects.py`: the third copy collapses into `geo`

The GVector-storage migration above led straight into the question that
had been bugging the user throughout: `build_region.py::get_surface()`
translates a `GeounedSurface` primitive into `Objects.py`'s own `Plane`/
`Cylinder`/`Cone`/`Sphere` classes -- a *third* parallel representation of
"a plane" (`geo.GPlane`, `basic_functions_part1.PlaneParams`, and this)
existing purely so `build_region/`'s CSG-cell-building machinery
(`get_cell_object`/`BuildDepth`/`SplitSolid`) had something to hold an
`id` + a per-cell-box-rebuildable CAD shape + point-in/out classification.
Traced why that third copy existed and whether it still needed to:

- **`Objects.py::Plane/Cylinder/Cone/Sphere.transform()`** -- the one
  thing that seemed to force these to be native (`FreeCAD.Matrix.multVec`)
  -- turned out to be **confirmed dead in CadToCsg**: grepped every
  constructor call and found `tr` is always the default `None`;
  `get_surface()` never passes one. `.buildShape()` was already a thin,
  fully-`GVector`-tolerant wrapper around `geo`'s own
  `plane_polygon_from_box`/`cylinder_from_box`/`cone_from_box`.
- **`splitFunction.py::surface_side`** (point-in/out classification for
  `SplitSolid`) turned out to be running the *exact same formulas*,
  natively, that `boolean_solids.check_sign_primitive` already runs in
  pure `GVector` for the main decomposition path -- confirmed
  formula-by-formula (plane: `axis.dot(point-position)`; cylinder:
  distance-from-axis vs radius, cross-product form here vs Pythagorean
  form there, same quantity; cone: `acos` of the axis/direction dot
  product vs `SemiAngle`). Also handled 7 surface types
  (`cone_elliptic`, `hyperboloid`, `ellipsoid`, `cylinder_elliptic`,
  `cylinder_hyperbolic`, `paraboloid`, generic `torus`, plus `box`) that
  `get_surface()` never actually constructs -- confirmed unreachable in
  this pipeline (leftover from a broader original toolkit).

Consolidated by moving the point-classification formulas to where they
conceptually belong -- `geo` itself, as `.is_inside(point: GVector) ->
bool` methods on `GPlane`/`GCylinder`/`GCone`/`GSphere` (same rationale
as `.intersect_plane`/`.intersect_line`: this is intrinsic analytic
geometry, not GEOUNED-CSG-specific) -- and `.transform(matrix)` methods
(taking a native `FreeCAD.Matrix`, consistent with `_freecad_impl.py`
being the FreeCAD-specific implementation file; a future `_occ_impl.py`
would accept whatever OCC's native transform type is under the same
method name). **`.transform()` is ported, not dropped, despite being
dead in CadToCsg today** -- explicit user call: `CsgToCad` (GEOReverse)
does need to move surfaces around, and this is the natural place for
that capability to live when that work starts.

`Objects.py`'s `Plane`/`Cylinder`/`Cone`/`Sphere` then collapsed into one
`CellSurface` class wrapping a `geo` descriptor directly (`.type` derived
from the wrapped descriptor's class, `.is_inside`/`.transform` delegating
straight to it, `.buildShape` dispatching on `.type` the same way the
old classes' `buildShape`s already did). The old classes' `truncated`
flag (explicit-end-planes cylinder/cone) and the double-sheet `Cone`
branch were **not** carried over -- both were already marked dead in
comments left by an earlier pass of this migration (`get_surface()`
never sets `truncated=True` or builds a double-sheet cone), and neither
has an equivalent in `geo`. `get_surface()` now builds `GPlane`/
`GCylinder`/`GCone`/`GSphere` directly via `.from_values(...)` -- no
translation step, no native detour. `surface_side` collapsed to `return
surf.is_inside(p)`; `btwPPlanes` (only used by the now-gone
truncated-cylinder/cone/box/cylinder_elliptic branches) deleted as a
consequence.

Verified via `tests/geo` (107/107), `tests/test_cadtocsg.py` (50/50),
and a re-scan of all 39 `RoundCorners` STEP files (same
per-file OK/MultiRoundCorner-count results as before -- this is the
code path that exercises `get_cell_object`/`BuildDepth`/`SplitSolid`
most heavily, so it was worth checking beyond the standard suite).

This closes essentially all of the "two/three parallel representations
of the same analytic surface" complaint for Plane/Cylinder/Cone/Sphere:
what's left is `geo`'s descriptor (the single geometric source of truth)
plus `basic_functions_part1`'s `*OnlyParams` (still a separate class,
per the earlier decision above, but now storage-identical and with no
duplicated behavior) plus `GeounedSurface` (the CSG-numbering/`bVar`/
`region`/`components` layer). `check_sign_primitive`
(`boolean_solids.py`) was deliberately **not** changed to call
`.is_inside()` -- it operates on `GeounedSurface`/`*OnlyParams` objects,
not `geo` descriptors directly, and per the "Option B, no wrapping"
decision above, building a transient `GPlane`/etc just to call one
method on a path this hot wasn't judged worth it. Its formulas remain a
second, independent copy of the same math -- a candidate for the
broader `components`/`definition` redesign below to close later, not
this pass.

**Update, later pass**: `check_sign_primitive`'s formula duplication above
was in fact closed out -- see "`check_sign_primitive` unified with `geo`'s
`is_inside` formulas, and a real Tier-2 dispatch bug fixed" below. The
Tier-2 dispatch design question (ignoring `.components`/the bounding
plane for Cylinder/Cone/Sphere/Torus) was confirmed with the user to be
intentional, not a bug -- these represent a single infinite analytic
surface, and the bounding plane is a CAD-reconstruction detail, not part
of the surface's own sign. What *was* a real bug in that same dispatch
(the Torus branch) is fixed, also described below.

### `check_sign_primitive` unified with `geo`'s `is_inside` formulas, and a real Tier-2 dispatch bug fixed

Closes the deferred item above. `check_sign_primitive` (`boolean_solids.py`)
duplicated `GPlane`/`GCylinder`/`GCone`/`GSphere.is_inside()`'s formulas
natively in `GeounedSurface`/`*OnlyParams` space. Since the Tier-1
`*OnlyParams` native `FreeCAD.Vector` -> `GVector` storage migration (see
above), `PlaneParams`/`CylinderOnlyParams`/`ConeOnlyParams`/`SphereOnlyParams`/
`TorusOnlyParams` are storage-identical to `geo`'s `GPlane`/`GCylinder`/
`GCone`/`GSphere`/`GTorus` (same field names, same `GVector` types) -- so
the formulas can be shared as free, duck-typed functions instead of
methods on either class family, avoiding the "Option B, no wrapping"
concern (no transient `GPlane`/etc construction on this hot path).

Added `is_inside_plane`/`is_inside_cylinder`/`is_inside_cone`/
`is_inside_sphere`/`is_inside_torus` to `geo/vector_geometry.py` (pure
functions, duck-typed on `.Axis`/`.Position`/`.Center`/`.Radius`/etc --
work identically on a `geo` descriptor or a Tier-1 `*OnlyParams`).
`GPlane.is_inside`/`GCylinder.is_inside`/`GCone.is_inside`/
`GSphere.is_inside` (`geo/_freecad_impl.py`) became one-line delegates to
these; `GTorus` gained `.is_inside()` too (didn't exist before).
`check_sign_primitive` collapsed to a dict dispatch:
```python
_IS_INSIDE_PRIMITIVE = {
    "Plane": vector_geometry.is_inside_plane,
    "CylinderOnly": vector_geometry.is_inside_cylinder,
    "SphereOnly": vector_geometry.is_inside_sphere,
    "ConeOnly": vector_geometry.is_inside_cone,
    "TorusOnly": vector_geometry.is_inside_torus,
}

def check_sign_primitive(point, surf):
    return 1 if _IS_INSIDE_PRIMITIVE[surf.Type](point, surf.Surf) else -1
```

While reviewing `check_sign`'s Tier-2 dispatch alongside this, found a
real, reachable bug in the Torus branch: `return check_sign(point,
tor=surf.Surf.Torus)` -- `tor=` is not a valid keyword argument of
`check_sign(solid_or_point, surf)`, so this raised `TypeError` on every
call. Fixed to `return check_sign(point, surf.Surf.Torus)`, matching the
sibling Cylinder/Cone/Sphere branches' own unkeyworded form.

### `.components` for every registered surface, not just the 4 composite types

The homogeneous design goal (below) called for *every* surface --
simple, open-with-plane, and composite alike -- to carry the same
`bVar`/`region`/`components` shape, not just Can/TCone/RoundCorner/
MultiRoundCorner. Checked what `MetaSurfacesDict.add_plane` (Tier-1,
standalone `Plane`) and `.add_cylinder`/`.add_cone`/`.add_sphere`/
`.add_torus` (Tier-2, an open 2nd-order surface + its closing plane(s))
already did: turned out **`.region` was already set uniformly** by all
of these (a plane's is the trivial single-variable `BoolSurface`) --
only `.components` was missing outside the 4 composite types. Added it
to all 5, mirroring the same pattern: `add_plane`'s is the degenerate
1-component case (`{abs(pid): plane}`, the plane referencing itself,
since a standalone plane *is* its own only component); the Tier-2
methods build `{abs(primitive_id): primitive, abs(plane_id): plane,
...}` alongside the existing region computation (cone additionally
keys its apex plane, if any; torus its 1-2 `UPlanes` and `VSurface`).

Deliberately did **not** use the `_resolve_plane_id` helper (shared by
Can/TCone) for the Tier-2 methods' plane-sign-flip step, even though the
inline pattern is identical -- `_resolve_plane_id` also mutates
`plane.Surf.Axis`/`.bVar` in place, which Can/TCone/RoundCorner need
(their own `get_cell_object`/`build_complex_shape` re-reads
`.Surf.Plane.Surf.Axis` later to build CAD) but Tier-2's `build_surface()`
never reads its bounding plane at all when building `.shape` (only the
open primitive) -- so that mutation would be a new, unrequested
side-effect for these 4 methods, not just a refactor. Kept the original
inline resolution logic unchanged and only added the `components`
bookkeeping alongside it.

Verified via `tests/geo` (107/107) and `tests/test_cadtocsg.py` (50/50).

With this, the "broader design goal" is functionally done for the
storage/bookkeeping layer: every surface that reaches `MetaSurfacesDict`
now has a uniform `bVar` + `region` (the boolean definition) +
`components` (the numbering<->surface relation) shape, whether it's a
bare plane, a cylinder-with-plane, or a RoundCorner. What's *not* done
(and wasn't asked for, this pass): consolidating the `.Type`-string
18-way dispatch in `GeounedSurface.build_surface()`/`write/*.py`'s
`mcnp_surface` etc into something that reads generically off
`components`/`region` instead -- `.Type` still exists and is still
required (card-format dispatch), it's just no longer the *only* way to
introspect a surface's structure.

Engine-swappability (the original motivation for the ABC in attempt 1) is
now achieved at the *module* level instead of via dependency injection: a
future `geo/_occ_impl.py` would define the same class/function names
against pythonocc-core, and `geo/__init__.py` would choose which
implementation to re-export — the rest of GEOUNED would not change.

Design points carried over from attempt 1 (still true, just relocated to
methods/free functions instead of ABC methods):
- Faces are eagerly classified into one of 5 analytic surface types
  (plane/cylinder/cone/sphere/torus) via `Gclassify_surface`; composite/
  meta-surfaces (RoundCorner, Can, TCone, MultiPlane...) are assembled by
  GEOUNED itself out of these via `Gcut`/`Gcommon`/`Gfuse`, never modeled
  in `geo`. Curve classification (`GEdge.Curve`, one of `GLine`/`GCircle`/
  `GEllipse`/`GBSpline`) is deliberately tolerant instead: it returns
  `None` for a degenerate edge or a real-but-unsupported curve type (e.g.
  `Part.Hyperbola`) rather than raising, because `GSolid`/`GFace` build
  eagerly and an incidental edge nothing downstream needs must not abort
  building the whole solid — only a face's `Surface` (what actually
  becomes an MCNP/OpenMC surface) is a hard failure.
- `Gsplit()` returns a `SplitResult` that must never silently come back
  empty; degenerate/tangency cases are resolved internally (tolerance
  retry, including a `scale_up_floor` branch mirroring the public
  `Options.scaleUp`/`Options.splitTolerance`) and reported via
  `degenerate_case_handled=True`. **Still NOT solved**: the original
  motivating tangency bug itself (a plane's intersection with a solid
  coinciding with a pre-existing tangency line) — `Gsplit` only ports
  GEOUNED's existing tolerance-scaling retry, the same limitation
  `FreeCADBackend.split()` had. The proper fix (face-adjacency graph
  excluding non-manifold edges, reconstructing solids per connected
  component) is still reserved for a future pyOCC implementation.
- `GSolid.faces_sharing_edge(edge)` exposes the non-manifold-edge
  diagnostic directly (edges shared by != 2 faces) — defined but not
  currently called anywhere in GEOUNED (confirmed via grep before
  simplifying the design around it).

### `FaceGu(GFace)`/`SolidGu(GSolid)`: merging the decompose-side native wrappers into `geo` via inheritance

`utils/geometry_gu.py`'s `FaceGu`/`SolidGu` were a *second*, independent
native-typed wrapper around a face/solid — pre-dating `geo` entirely,
used throughout `decompose/` (`SolidGu.Faces` is `tuple[FaceGu]`) for
chained native calls (`.valueAt`, `.tangentAt`, `.normalAt`,
`.distToShape`, `.isEqual`/`.isSame`, `tessellate(val, reset)`) that
GEOUNED's decomposition-side code depends on. Rather than rewrite every
call site to `GFace`'s snake_case/`GVector` API (hundreds of sites, high
risk), both classes now inherit from their `geo` counterpart instead:
`FaceGu(GFace)`/`SolidGu(GSolid)`, `__init__` calling
`super().__init__(x.__native__)` then layering only the decompose-side
extras on top (`FaceGu`: `.Index`, `.OuterWire` — computed lazily via
`set_outerWire()`, same `pick_outer_wire(self.wires())` heuristic `GFace`
itself uses, not reimplemented; `SolidGu`: `.TorusVParams`/
`.TorusUParams` torus-face-merge bookkeeping, `.tolerances`). This
reuses `GFace`/`GSolid`'s own classification/edge/vertex/boundbox
construction instead of duplicating it a second time — confirmed via the
full test suite (`tests/geo` + `tests/test_cadtocsg.py`) that nothing
outside these two classes depended on their old standalone-`object`
identity.

Two dead pieces of code surfaced and were deleted along the way:
`FaceGu`/`SolidGu`'s own `.__face__`/`.solid` attributes (superseded by
the inherited `.__native__` — every read site was mechanically renamed,
~10 call sites across `cell_definition_functions.py`, `functions.py`,
`geometry_gu.py` itself, `meta_surfaces_utils.py`); and
`geometry_gu.py::is_inverted` (a ~50-line function computing whether a
face's normal points inward, confirmed via grep to have zero callers
anywhere in `GEOUNED` — its only use, `SolidGu.__init__`'s
`self.inverted = is_inverted(solid)`, was itself dead, `.inverted` never
read downstream).

### Pushing `GSolid`/`GFace` wrapping to the origin: `GSolid.Solids`

Before this pass, code holding a compound (or a list of already-wrapped
solids) constantly unwrapped to native and rewrapped via `GSolid(s)` at
every call site that needed to iterate the pieces (`Gmake_compound([GSolid(s)
for s in m.Solids])`, `comsolid.exportStep(...)` on a bare native shape,
etc.) — three-way churn between "a `GSolid`", "its `.__native__`", and
"a fresh `GSolid(...)` wrapping one of its native sub-shapes". `GSolid.__init__`
now eagerly builds `.Solids: list[GSolid]` itself: for a real compound
(`len(native.Solids) > 1`) each native sub-solid is recursively wrapped
as its own `GSolid`; for a single/non-compound solid, `self.Solids` is
`[self]` (self-referencing, so `.Solids` is always a uniform
`list[GSolid]`, compound or not). This let every call site that used to
manually wrap/unwrap collapse to just passing the `GSolid` through:
`Gmake_compound(m.Solids)` (was `Gmake_compound([GSolid(s) for s in
m.Solids]).__native__`, `core.py`/`decom_one_generators.py`), `Solids =
Gload_step(filename)` (was `[g.__native__ for g in Gload_step(filename)]`,
`load_step.py`), `comsolid.refine().Solids`/`s.reverse()` (was
`GSolid(comsolid).refine().__shapes__`/`GSolid(s).reverse().__native__`,
`geouned_classes.py::GeounedSolid.__init__`), `bbox.union(s.BoundBox)`
(no `to_gboundbox()` needed, `s.BoundBox` already a `GBoundBox` since `s`
is now always a `GSolid`), `bbox = bbox.enlarged(10)` (was native
in-place `bbox.enlarge(10)`, `decom_one_generators.py::generic_split`,
confirming `solid.BoundBox` is `GBoundBox` throughout). `Gmake_compound`
itself already accepted `list[GSolid]` (unchanged) — the fix was at
every *caller*, not the constructor.

### `GWire`/`GEdge` enrichment, `pick_outer_wire`'s new `GWire`-based signature, and `Gclassify_curve`'s tolerance for an already-wrapped `GEdge`

`pick_outer_wire` used to take a native face and return a native
`Part.Wire` (`pick_outer_wire(native_face) -> Part.Wire`, reading
`native_face.Wires` itself). It now takes `list[GWire]` and returns a
`GWire` (`pick_outer_wire(wires: list[GWire]) -> GWire`), operating
purely in `GVector`/`GWire` space (`(v - center).length` instead of
`(v.Point - center).Length`) — callers now pass `face.wires()` (the
already-lazily-built list) instead of the native face itself. `GFace.outer_wire()`
and `FaceGu.set_outerWire()` both call it the same way now
(`pick_outer_wire(self.wires())`).

This has one important, non-local consequence: **anywhere code reaches
an edge via `.OuterWire.Edges` (a `GWire`'s `.Edges`, itself
`[GEdge(e) for e in native.OrderedEdges]`), that edge is now a `GEdge`,
not a native edge** — a cascading type change that touched a long tail
of call sites across `decom_utils_generator.py`
(`torus_bound_planes`/`cks_bound_planes`/`cks_edge_plane`/`spline_wires`),
`meta_surfaces_utils.py` (`commonEdge`/`commonEdgeFace`/`commonVertex`/
`eligible_plane`/`cyl_plane_region_conf`/`planar_edges`/`edge_1D`/
`spline_2D`/`region_sign`), and `functions.py::get_additional_corner_plane`.
Concretely: `.Vertexes[i]` is already a `GVector` now (a bare
`.Vertexes[i].Point` read, the old native pattern, raises
`AttributeError: 'GVector' object has no attribute 'Point'`); calling
`Gclassify_curve(e)` on one of these edges used to always misclassify to
`None` (`GEdge.Curve` is already the classified result, and a native
`.Curve` lookup on a `GEdge` finds `GEdge`'s own `.Curve` attribute,
whose type never matches any `Part.*` check). Fixed at the shared
boundary rather than chasing every call site individually (confirmed via
grep the pattern recurred ~10+ times, too many to fix with confidence
one by one — same "fix at the shared function" discipline as
`_resolve_plane_id`/`can_region`/`tcone_region` earlier in this
migration): `Gclassify_curve` now tolerates being called with an
already-wrapped `GEdge` and returns `.Curve` directly instead of
misclassifying it a second time. Every other call site was then fixed by
tracing real `AttributeError`/`TypeError` tracebacks from the full test
suite (never by static reading alone — the same discipline the "Tier-1
`*OnlyParams`" pass below already established): `.Vertexes[i].Point` →
`.Vertexes[i]`; `.Length`/`.normalize()` (native-style, capital `L`,
in-place) → `.length`/`.normalized()` (`GVector`-style, lowercase,
returns new) wherever the value flowed from a `GEdge`/`GWire` field;
`.valueAt`/`.derivative1At`/`.normalAt` → `.value_at`/`.derivative1_at`/
`.normal_at` (`GEdge`'s own methods) or, where a value needed to stay
native for arithmetic against another already-native value (e.g.
`cyl_plane_region_conf`, `spline_wires`'s `projection()` fallback branch
— both still fully native-style internally), explicit `to_fc_vector(...)`/
`edge.__native__`/`edge.Curve.__native__` unwraps at the point of use
instead of leaving the conversion implicit.

`meta_surfaces_utils.py::material_direction`'s signature was also
formalized as part of this: `material_direction(pos: GVector, face:
GFace | FaceGu, edge: GEdge)` (was untyped, and internally read
`edge.derivative1At(pe)`/`face.normalAt(u, v)` natively) — now
`edge.Curve.parameter(pos)`, `edge.derivative1_at(pe).normalized()`,
`face.Surface.parameter(pos)`, `face.normal_at(u, v).normalized()`
throughout. **`utils/functions.py` has its own, separate,
still-fully-native `material_direction`** (`edge.derivative1At(pe)`,
`dir.normalize()`, `face_in.normalAt(u,v)`) — used only by
`get_additional_corner_plane`, deliberately not unified with
`meta_surfaces_utils.py`'s version this pass (different signature,
different call-site expectations); callers must match whichever one
they import, and `get_additional_corner_plane` explicitly unwraps to
native (`cyl.__native__`, `e1.__native__`, `to_fc_vector(e1.Vertexes[0])`)
at its own call site since it uses the native version.

To support all of the above, `GPlane`/`GCylinder`/`GCone`/`GSphere`/
`GTorus` gained `.parameter(point) -> tuple[float, float]` (native
`.parameter()` passthrough via `to_fc_vector`, tolerant of a `GVector`
input); `GLine`/`GCircle`/`GEllipse`/`GBSpline` gained `.value(u)`
(point at parameter `u`) and `.parameter(point)` (inverse of `.value`);
`GFace` gained `.CenterOfMass` (eager `GVector`), `.isEqual`/`.isSame`
(delegating to native `.isEqual`/`.isSame`, absorbing what used to be
`FaceGu`-only), `tessellate(tolerance, reset=False)` (gained the
`reset` param), and `getUVNodes()` (a bare native `getUVNodes()`
passthrough replacing the old `get_uv_nodes(tolerance)`, which
re-tessellated internally as a side effect nothing actually needed —
its own test, `test_get_uv_nodes_matches_tessellate_point_count`, was
removed with it). `GWire` gained `.CenterOfMass` (native, **not**
converted via `to_gvector` — a known inconsistency, left as-is because
`GVector`'s own arithmetic already tolerates a native argument via
duck-typing, so nothing downstream actually breaks on it) and
`.OrderedVertexes` (`list[GVector]`).

### `GMatrix`: a neutral 4x4 matrix, for `GEdge`/`GWire.MatrixOfInertia`

`decom_utils_generator.py::spline_wires` needs the principal axis of a
wire/edge-set's inertia tensor (`get_axis_inertia`, reading
`mat.A11..mat.A33` off a native `FreeCAD.Matrix` returned by
`shape.MatrixOfInertia`) — another native value that used to force an
`edge.__native__.MatrixOfInertia`/`W.__native__.MatrixOfInertia` unwrap
at every call site once `edge`/`W` became `GEdge`/`GWire`. Added
`GMatrix` to `vector_geometry.py` (a neutral, frozen dataclass mirroring
`GVector`/`GBoundBox`'s existing pattern — 16 fields `A11..A44`, named to
match FreeCAD's own `Base.Matrix` attributes exactly, full 4x4 stored for
fidelity even though only the `A11..A33` rotation/inertia sub-block is
meaningful for this particular use) plus `to_gmatrix(matrix)` (a
tolerant converter, same shape as `to_gvector`/`to_gboundbox`). `GEdge`
and `GWire` both now eagerly carry `.MatrixOfInertia: GMatrix`, built at
construction time. Because `GMatrix`'s field names match native exactly,
`get_axis_inertia(mat)` itself needed **no changes** — it already only
reads `.A11..A33`, so it works unchanged on either a native `FreeCAD.Matrix`
or a `GMatrix`. `spline_wires` now reads `W.MatrixOfInertia`/
`e.MatrixOfInertia` directly instead of unwrapping to `.__native__` first.

### Debugging chain: real bugs found while stabilizing this pass

Besides the type-mismatch fixes above (expected fallout from the
`GWire`/`GEdge` signature changes, not independent bugs), two genuinely
pre-existing, previously-unreached bugs surfaced once the pipeline
started running further than before:
- **`BoolSequence.expand_regions_to_integer`** (`utils/boolean_function.py`)
  crashed (`TypeError: 'bool' object is not iterable`) on a cell whose
  `Definition` had simplified to a plain `bool` — a legitimate,
  well-established `BoolSequence` state (`self.elements = True/False`
  is set in over a dozen places in this class) that every *sibling*
  method already guards for (`join_operators`, `get_surfaces_numbers`,
  `get_complementary`) but `expand_regions_to_integer` didn't. Confirmed
  via `git stash` against the committed baseline that this is a real,
  previously-unreachable regression path, not a pre-existing bug on the
  old code path (the same STEP file, on the pre-migration native
  pipeline, doesn't produce a bool-valued cell at all) — simply never
  exercised before because earlier crashes elsewhere in the geometry
  pipeline always intercepted first. Fixed with the same one-line guard
  its siblings already use: `if type(self.elements) is bool: return`.
- **The MCNP writer already had a matching guard one level up**
  (`mcnp_format.py::write_cells`: `if type(cell.Definition.elements) is
  bool: log + skip this cell entirely`) that **OpenMC, Serpent, and
  PHITS's own cell-writers lacked** — `write/openmc/openmc_format.py`'s
  `write_xml_cells`/`write_py_cells`, `write/mcnp_like/serpent_format.py`'s
  `write_cells`, `write/mcnp_like/phits_format.py`'s `write_phits_cells`/
  `write_phits_cells_uni_void_def` all crashed identically the moment a
  real bool-valued cell reached them (which, before this pass, per the
  point above, never happened). Extended the exact same guard to all
  five. This is a real, pre-existing gap between the 4 output formats
  (matching the *shape* of the earlier-documented `sorted_surfaces`
  inconsistency in "`write/`: `mcnp_like/` + `openmc/` subpackages"
  above — MCNP quietly received a fix the other 3 formats didn't), not
  something introduced by this session's changes.

### Known bugs from the previous checkpoint — fixed

The two typos flagged at the previous checkpoint (`cell_definition_functions.py`'s
`gen_plane_cylinder`/`gen_plane_cone` calling the nonexistent `p1.is_qual(...)`
and `p2.sub(p1)`) are fixed: `is_qual` → `is_equal`, `p2.sub(p1)` → `(p2 - p1)`.

### `to_fc_vector`/`to_gvector` purged from all of `GEOUNED` — assume `GVector` everywhere

Explicit user directive: strip every `to_fc_vector`/`to_gvector` call out
of `GEOUNED` (everything except `geo/`, the backend, where they belong as
the actual conversion boundary), on the working assumption that by this
point in the migration every vector flowing through GEOUNED already *is*
a `GVector` and every object already *is* a `G*` type — "aún si sabemos
que el código puede cascar" (even knowing some of it will break). ~110
occurrences across 11 files, all removed; where the removal exposed
leftover native-style syntax on what's now known to be a `GVector`
(`.Length` → `.length`, `.normalize()` → `.normalized()`, `.sub()`/`.add()`
→ `-`/`+`, `.isEqual` → `.is_equal`), that was fixed too, function by
function, checking real callers each time rather than assuming.

Two structural pieces fell out of this pass:
- **`FaceGu.valueAt`/`.tangentAt` deleted** (`geometry_gu.py`) — these were
  thin native passthroughs shadowing `GFace`'s own already-inherited
  `.value_at()`/`.tangent_at()` (which return `GVector`, not a native
  `FreeCAD.Vector`). Every call site across `GEOUNED` that used to call
  `.valueAt(`/`.tangentAt(` on a `FaceGu`/`GFace` was switched to the
  snake_case form; confirmed via grep that no `.valueAt(`/`.tangentAt(`
  call on a `FaceGu` object remains anywhere (the only survivors are on
  confirmed-native objects: `edge.__native__.valueAt(...)`, and inside
  `projection()`'s legacy native body — see below).
- **`GEllipse` gained real `.XAxis`/`.YAxis`** (`geo/_freecad_impl.py`,
  matching `Part.Ellipse`'s own attribute names exactly) replacing a
  wrongly-named `.MajorAxis` field that actually held `native.XAxis` (a
  vector) — confirmed via grep that nothing outside
  `decom_utils_generator.py::projection()` ever read the old field, so
  renaming was safe. `projection()`'s ellipse branch was rewritten to use
  `.XAxis`/`.YAxis` (vectors) and `.MajorRadius`/`.MinorRadius` (the
  actual scalars — the old code read `.MajorAxis`/`.MinorAxis` as if they
  were scalars, which was simply wrong).

`decom_utils_generator.py::projection()` (the one clearly-flagged native
holdout from this pass) was converted to take a `GEdge` instead of a
native edge, per explicit walkthrough: `GEdge` already had
`.ParameterRange`; `type(Gclassify_curve(edge)) is GCircle` became
`type(edge.Curve) is GCircle` (no need to re-classify, `edge.Curve` is
already the classified result); every `edge.valueAt(...)` became
`edge.value_at(...)`. Also fixed in the same function: a genuinely
pre-existing bug where `dmin` was read in the ellipse branch's final
comparison (`abs(d0 - dmin) < abs(d0 - dmax)`, mirroring the circle
branch's use of the same variable) but never defined there — added the
same `dmin = (edge.value_at(pmin) - edge.Curve.Center).dot(axis)`
computation the circle branch already does, by direct analogy.

Confirmed-dead code deleted along the way (zero callers anywhere,
verified by grep before deleting, not just before this pass): `utils/
functions.py::get_additional_corner_plane_old`; `utils/geometry_gu.py::
innerWires`, `innerWires_org`, `line_projection`; `utils/
meta_surfaces_utils.py::get_edge`; `utils/basic_functions_part1.py::
is_in_edge`, `is_in_points`.

### Real bugs the sweep, and the subsequent full test run, surfaced

Removing a redundant conversion is safe by construction (the value was
already the type being asked for); every one of the bugs below was found
either while *tracing* a removal to confirm it was safe, or by then
actually running the test suite — never guessed:

- **`decom_one_generators.py::generic_split` — decomposition splitting
  was silently a no-op** (the most serious finding this session). `core.py`
  passes `main_split(Gmake_compound(m.Solids), ...)` — a `GSolid` — and
  `generic_split`'s own `solid` parameter is that `GSolid` throughout. But
  its body still did `Gsplit(GSolid(solid), ...)` — double-wrapping an
  already-`GSolid` value, which crashes inside `GSolid.__init__` (a `GEdge`
  reached through the re-parse is already-wrapped, and `v.Point` on its
  already-`GVector` vertices doesn't exist). That crash was caught by a
  broad `except Exception:`, whose fallback branch (`comsolid_solids =
  [solid]`) *also* left the value un-unwrapped, silently returning "this
  surface produced only 1 solid" — i.e., `Gsplit` never actually ran, and
  decomposition treated every candidate surface as a non-split, for as
  long as `core.py` has been passing `GSolid` here (i.e., since earlier
  in this same migration). Fixed: pass `solid` (already `GSolid`) directly
  to `Gsplit` instead of re-wrapping; fixed the `except` branch to
  properly unwrap (`solid.__native__`) so `comsolid_solids` stays
  native throughout, matching what `remove_solids` needs
  (`.removeSplitter()`/`.isValid()`, native-only); re-wrap `remove_solids`'
  native output back to `GSolid` at the one point `cleaned` needs to be
  `GSolid` again.
- **`void.py` — two separate native-`BoundBox` leaks**, both downstream
  of `core.py::_set_geometry_bounding_box`'s `self.geometry_bounding_box
  = FreeCAD.BoundBox(...)` (the one deliberately-deferred native holdout
  documented earlier in this file): `void_generation`'s `EnclosureBox =
  GeounedSolid(None, Box.__native__)` fed a *native* shape into
  `GeounedSolid.__init__`'s `else` branch, which is written for a `GSolid`
  (`comsolid.refine().Solids`, `comsolid.BoundBox`) — native.`.refine()`
  doesn't exist, so it silently fell into `except Exception: self.Solids
  = comsolid.Solids` (native list, not `GSolid`) and `self.BoundBox =
  comsolid.BoundBox` (native `FreeCAD.BoundBox`, not `GBoundBox`) — a
  `GeounedSolid` instance with different field *types* than every other
  one in the system, undetected because the exception was swallowed.
  Fixed by dropping the `.__native__` (`Box` was already `GSolid` three
  lines up). `set_graveyard_cell`'s `center = UniverseBox.Center` (native
  `FreeCAD.Vector`, from the same `UniverseBox`) flowed straight into
  `SphereOnlyParams` — which, post-sweep, no longer converts on the way
  in — surfacing much later as `AttributeError: 'Base.Vector' object has
  no attribute 'is_equal'` in the MCNP writer. Fixed with an explicit
  `to_gvector()` right at this one native-native boundary, matching the
  "convert exactly at the native leak, nowhere else" pattern used
  throughout this migration.
- **`get_box` (`utils/functions.py`) → `build_c_table_from_solids` →
  `split_solid_fast`/`split_s2_s1` (`utils/boolean_solids.py`) — the same
  double-wrap risk as `generic_split`, closed at the root.**
  `build_c_table_from_solids` had an explicit "accept both" shim
  (`box_native = Box.__native__ if type(Box) is GSolid else Box`) because
  `get_box` (used by `core.py`/`cell_definition.py`) returned native while
  `void_box_class.py` (calling `Gmake_box` directly) passed `GSolid` —
  but the *unconverted* `Box` (whichever type) was still being forwarded
  straight into `split_solid_fast`/`split_s2_s1`, which wrap it in a fresh
  `GSolid(solid)` — a live double-wrap risk whenever the `GSolid` branch
  was taken. Fixed by making `get_box` return `GSolid` too (dropping its
  own `.__native__|`), so both callers are consistent; the shim collapses
  to an unconditional `box_native = Box.__native__`; and it's
  `box_native` (always native, matching what `split_solid_fast`'s other
  code path — pieces from `comsolid_solids` — already always is), not
  `Box`, that gets passed onward. Found and fixed two more bugs in
  `split_s2_s1`'s `else` branch while tracing this: `check_sign(solid,
  surfaces[s2])` referenced a `solid` that was never defined anywhere in
  the function (fixed to `box_native`, matching `split_solid_fast`'s own
  identical `else: return check_sign(solid, surf)` fallback, which uses
  its own first argument the same way); and `res = check_sign(...), None`
  assigned a 2-tuple to `res` alone, leaving `splitRegions` undefined or
  stale from a previous loop iteration (fixed to `res, splitRegions =
  check_sign(...), None`, matching the sibling `if` branch's unpacking).
- **`decom_utils_generator.py::get_axis_inertia` leaked `numpy.float64`
  into `GVector`** — `numpy.linalg.eig`'s eigenvector components were
  passed straight into `GVector(...)` uncast. Downstream comparisons
  (`... > 0`) then produced `numpy.bool_` instead of Python `bool`,
  which `BoolSequence.clean()` doesn't recognize as boolean
  (`type(eVal) is not bool` → tries `.elements` on it → `AttributeError`).
  Fixed with an explicit `float(...)` cast on all three components.
- **`meta_surfaces_utils.py::region_sign`/`material_direction` didn't
  handle an edge whose curve `Gclassify_curve` can't model** (Hyperbola/
  Parabola — `edge.Curve is None`). `region_sign` used `e1.Curve.value(pe)`
  to get a point on the edge; switched to `e1.value_at(pe)` (`GEdge`'s own
  curve-classification-agnostic method — see the "GEdge/GWire enrichment"
  section above). `material_direction` used `edge.Curve.parameter(pos)` to
  go the other way (point → parameter); rather than special-case `None`
  inline, added `GEdge.parameter(point) -> float` to `geo/_freecad_impl.py`
  (going straight to the native curve, `self.__native__.Curve.parameter(...)`,
  bypassing GEOUNED's own classification entirely — matching every other
  `G*.parameter()` method's shape) and `material_direction` now just calls
  `edge.parameter(pos)` unconditionally. This is the one place this pass
  re-introduced a `to_fc_vector` call — but inside `geo/_freecad_impl.py`,
  the correct location for it, not back in `GEOUNED`.
- **`decom_utils_generator.py::cutting_face_number`**: `raise ("Spline
  surface detectected")` — raising a bare string is invalid Python
  (`TypeError: exceptions must derive from BaseException`); would have
  masked the real "spline surface detected" error with a different one.
  Fixed to `raise RuntimeError("Spline surface detected")` (typo fixed
  too).

Verified via the full suite after each fix, not just at the end: `tests/geo`
(106/106) and `tests/test_cadtocsg.py` (50/50) both green.

### More dead code, found while pushing point B (`.__native__` audit) — deleted

- **`build_region/Objects.py::FuseSolid`** — a third, byte-for-byte-identical
  copy of the same function already live in both `build_region.py` (used by
  `build_shape_functions.py`) and `splitFunction.py` (used internally by
  `joinBase`) — confirmed zero callers, deleted, along with the `GSolid`/
  `Gfuse`/`Gmake_compound` imports that became unused with it. The two
  *live* copies (`build_region.py`, `splitFunction.py` — also byte-for-byte
  identical, confirmed via `diff`) were merged: `build_region.py` already
  imports from `splitFunction.py` (no risk of a cycle the other way), so
  its own copy was deleted and `FuseSolid` added to the existing `from
  .splitFunction import ...` line instead.
- **`MetaSurfacesDict`/`SurfacesDict` (`geouned_classes.py`)** — both
  classes' constructors had a `surfaces=None` copy-constructor branch
  (`if surfaces is not None: ...`) that turned out to be completely dead:
  `MetaSurfacesDict` is constructed exactly once anywhere in `GEOUNED`
  (`core.py`, always without a `surfaces=` argument), and the *only* other
  place that ever called `SurfacesDict(some_dict)` with a real value was
  inside `MetaSurfacesDict`'s own now-deleted dead branch. Both classes'
  `__init__` collapsed to just the (previously `else`) live path, and the
  now-pointless `surfaces` parameter was dropped from both signatures.

### `settings.startSurf` finally wired to `MetaSurfacesDict`'s `IndexOffset`

Closes the "Deferred, not yet done" item from earlier in this file.
`core.py`'s `MetaSurfacesDict(...)` construction now passes
`offset=self.settings.startSurf - 1` (the `-1` because internal surface
numbering starts at 1, so `startSurf=1`, the default, must produce
`offset=0` — verified against `MetaSurfacesDict.surfaceNumber`'s own
`0`-then-pre-increment pattern). This alone wasn't sufficient: every
writer reads `Surfaces.primitive_surfaces.IndexOffset`
(`write/functions.py::sorted_surfaces`, called as
`self.sorted_surfaces(Surfaces.primitive_surfaces)` in all 4 formats) —
i.e. `SurfacesDict.IndexOffset`, a *different* object from
`MetaSurfacesDict.IndexOffset` — so `MetaSurfacesDict.__init__` also
needed to forward its own `offset` down into the `SurfacesDict(...)` it
constructs, which it previously didn't. `startSurf=1` (default) now
reproduces the exact previous behavior (`offset=0`, a no-op); other
values actually shift the numbering for the first time.

### Point C of the `.__native__` audit (`SolidGu`/`FaceGu`'s `super().__init__(x.__native__)`) — considered, left as-is

`SolidGu.__init__`/`FaceGu.__init__` unwrap an already-`GSolid`/`GFace`
argument back to native just to hand it to `GSolid.__init__`/`GFace.__init__`,
which then re-parses everything from scratch (re-classifies faces, edges,
vertices...) — real duplicated work, not just indirection. A backend-free
alternative was considered (`self.__dict__.update(x.__dict__)` instead of
re-parsing, adding only the `SolidGu`/`FaceGu`-specific fields on top) and
would work correctly, but was declined: the existing docstring already
established this cost as negligible (`SolidGu` is constructed twice in the
whole pipeline), and the `__dict__` trick trades an explicit, obvious
re-parse for an implicit dependency on `GSolid`/`GFace`'s internal field
layout — not worth it for a cost that's already known to not matter.

### `FaceGu.distToShape`'s native distance query moved into `geo`

Point 1 of a `.__native__`-audit punch list (every remaining `.__native__`
use in `GEOUNED` outside `geo/`, categorized): `FaceGu.distToShape`
(`geometry_gu.py`) turned out to have ~35 lines of dead manual
BoundBox/`.common()`/`e1.isSame(e2)` fallback logic — reached only if a
caller passed a shape with no `__native__` attribute, which, confirmed by
grep across all 5 live call sites, never happens (every caller passes a
`FaceGu`/`GFace`/`ShellGu`). That dead branch was deleted first, leaving
just the `ShellGu` recursion and a one-line native delegation. That
one line was then moved into `geo` proper: `GFace.distance_to(other) ->
float` (`_freecad_impl.py`, wrapping `self.__native__.distToShape(other.__native__)[0]`
— a real native boolean/distance query with no `GVector` equivalent, so
it belongs in the backend, not reimplemented). `FaceGu.distToShape` now
just calls `self.distance_to(shape)` for the non-`ShellGu` case; the
`ShellGu` recursion itself stays in `geometry_gu.py` since `ShellGu`
isn't a `geo` type. `cell_definition_functions.py`'s `gen_plane_sphere`,
which used to call `tmp_plane.__native__.distToShape(f.__native__)[0]`
(bypassing `FaceGu.distToShape` entirely, calling native directly since
neither operand needed `ShellGu`-awareness), was simplified to
`tmp_plane.distance_to(f)` the same way.

While the dead branch was still present, the user prototyped an
alternative implementation alongside it (`GFace.my_distToshape` — the
same BoundBox/`.common()`/`isSame` approach, revived as a real,
switched-in implementation rather than dead code) and wired
`FaceGu.distToShape` to call it instead of `distance_to`. It shipped
with a real bug: `if shape1 is shape1:` (comparing `shape1` to itself,
always `True`) instead of `if shape1 is shape2:` — meaning every
`distToShape` call anywhere in `GEOUNED` returned `0.0` unconditionally,
regardless of the actual two shapes involved. Caught before commit,
confirmed with the user as a typo, fixed to `is shape2`. Both
`distance_to` (native `distToShape`, now the one every live call site
actually uses) and `my_distToshape` (the BoundBox/common alternative)
exist side by side in `GFace` — the latter is not currently called from
anywhere in `GEOUNED` (kept as the user's own comparison/testing tool,
not wired back in).

### `core.py`'s last native import closed out

The very last direct `import Part`/`import FreeCAD` anywhere in
`GEOUNED` (`core.py::_set_geometry_bounding_box`'s `self.geometry_bounding_box
= FreeCAD.BoundBox(FreeCAD.Vector(...), FreeCAD.Vector(...))`) is gone —
replaced with `GBoundBox(xmin - padding, ..., zmax + padding)` (the
6-scalar constructor, matching what `xmin`/`xmax`/etc already were:
plain floats read off `GeounedSolid.optimalBoundingBox()`, itself
already `GBoundBox`-returning). `import FreeCAD` deleted from `core.py`
entirely — confirmed via grep it was the file's only use of `FreeCAD.*`.

This was deferred for a long time specifically because `UniverseBox`
(the parameter name `self.geometry_bounding_box` travels under once
passed to `void_generation`/`write_geometry`) is consumed in several
places outside this migration's original scope, so all of them were
traced before touching the source:
- `write_files.py` — only ever reads `.XMin`/`.XMax`/etc as plain
  scalars into output tuples. No change needed, works identically with
  either type.
- `void_box_class.py::VoidBox.__init__` — already tolerant (`box = Box
  if type(Box) is GBoundBox else to_gboundbox(Box)`), from earlier work
  this migration. No change needed.
- `void_functions.py::select_solids` — has a `"BoundBox" in str(Enclosure)`
  string-sniffing type check (distinguishing "this is the outer universe
  box" from "this is a real Enclosure object"). Fragile-looking but
  turns out to still work with `GBoundBox`: Python's default dataclass
  `__repr__`/`__str__` is `ClassName(field=value, ...)`, and `"GBoundBox"`
  itself contains the substring `"BoundBox"` — not touched, still correct.
- `void.py::set_graveyard_cell` — the `to_gvector(UniverseBox.Center)`
  conversion added earlier this session (back when `UniverseBox` was
  still native and this was a real native-leak fix) is now a redundant
  no-op, since `GBoundBox.Center` already returns `GVector` directly.
  Simplified back to `UniverseBox.Center`, and the now-unused
  `to_gvector` import dropped.
- `geouned_classes.py`'s `GeounedSolid.UniverseBox` field: unrelated
  despite the similar name — always `None`, confirmed dead (never
  assigned or read anywhere else) while tracing this.

Confirmed via grep after the fix: zero files anywhere in `GEOUNED`
(`geo/` and `GEOReverse/` excluded, as always) import `Part`/`FreeCAD`/
`BOPTools` directly anymore. This closes out, in full, the architectural
goal stated at the top of this file's "Current architecture" section.

### `check_sign` end-to-end verification, part 1: real scope corrected, one test artifact found (no real bug), `MultiRoundCorner`/`TCone` still unexercised

User request: build a real test/model to verify `check_sign` against
`MultiRoundCorner`, and more broadly think about verifying every complex
surface type this way.

**First, the scope of `check_sign` itself needed correcting** -- an
earlier pass of this file (see "RoundCorner/MultiRoundCorner: `.components`..."
above) described `check_sign` as a rare fallback "when `Gsplit`'s normal
CAD-based split path" fails. That's imprecise and the user pushed back on
it directly. The corrected picture, confirmed by reading the actual call
sites:
- **The real decomposition engine, with zero exceptions, is `Gsplit`** --
  used twice: (1) `decom_one_generators.py::generic_split` (lines ~31-34),
  splitting the main CAD solid by each identified candidate surface in
  turn; (2) one level below, inside composite-surface construction itself
  (`build_surface`'s complex-surface branch), since the only way found to
  build a Can/RoundCorner/etc.'s CAD shape is via boolean operations on
  simple surfaces -- also `Gsplit`, and the +/- side of its resulting
  fragments decides which piece becomes part of the reconstructed
  composite surface. If `Gsplit` fails or cuts wrong at either level, the
  whole conversion is compromised -- there is no fallback that saves it.
- **`check_sign` lives entirely outside both of those.** Its only caller
  is `build_c_table_from_solids` (`boolean_solids.py:240`), which builds
  a "Constraint Table" used purely to *simplify* an already-correctly-built
  cell's boolean expression after the fact (`conversion/cell_definition.py`'s
  `process_overlap`, itself only invoked from `noOverlapCell` when
  `options.forceNoOverlap` is set and two cells' CAD solids actually touch;
  also used by `void_box_class.py` for void-cell simplification). This is
  optional: the pipeline runs and produces a geometrically correct
  decomposition without it. Within it, `check_sign` covers three cases,
  in descending frequency: (1) a solid/fragment and a candidate surface
  that don't intersect at all -- by far the most common, since most
  surface pairs in a real model don't touch, so a full `Gsplit` isn't
  worth attempting, just "which side is the whole thing on"; (2) after a
  real, successful `Gsplit`, classifying each resulting fragment against
  the *next* surface being considered for the constraint table; (3) the
  literal fallback when `Gsplit` returns a degenerate 1-piece result.
  A wrong sign here degrades simplification (larger-than-necessary
  boolean expression) or, worse, an incorrect simplification when
  `forceNoOverlap`/void generation are active -- but never corrupts the
  base decomposition itself.

**Verification methodology**: for each composite surface (`RoundC`/
`FwdCan`/`RevCan`/`MultiRoundC`/`FwdTCone`/`RevTCone`) found in a model
after `build_solid_definition()`, build an independent CAD ground-truth
solid via the exact function `Gsplit`'s own composite-construction path
uses (`makeRoundCorner`/`makeCan`/`makeTCone`/`makeMultiRoundCorner` ->
`build_complex_shape` -> `get_cell_object`/`BuildDepth`/`SplitSolid` --
real boolean CAD surgery, sharing nothing computationally with
`check_sign`'s point-algebra except the surface's own analytic
parameters and, importantly, the *same* `can_region`/`tcone_region`/
`round_corner_region` pure-function AND/OR rule that `check_sign`
evaluates via `.region`/`.components` -- see "Can and TCone did not [share
the AND/OR rule]" above), then sample random points and compare
`check_sign(point, surf)` against the CAD solid's own `gsolid.is_inside(point)`.

**Real finding: a test-methodology bug, not a `check_sign` bug.** First
pass sampled points in `gsolid.BoundBox` (the *resulting* solid's own
bounding box) padded by an extra 20%. `RoundC`/`FwdCan` came back
~99-100% matching; `RevCan` came back only ~68-76% under either sign
calibration, looking like a real, reproducible orientation-specific bug.
Root cause, confirmed by instrumenting one case (`p1.stp`'s `RevCan`)
component-by-component: `cylinder_from_box`/`plane_polygon_from_box`
each already pad their own shape a bit past the construction `box`
(see "Cylinder margin" above), so `gsolid.BoundBox` can already exceed
`box`; adding another 20% on top pushed sample points **outside the box
the CAD reference solid was actually built from**. `check_sign` evaluates
the true, unbounded analytic surface (correctly -- it has no concept of
`box` at all); the CAD solid, clipped to `box`, simply has nothing built
out there, so `is_inside` returns `False` for reasons unrelated to
whether `check_sign`'s answer is right. `RevCan`'s region happens to be
geometrically "open" in some directions (confirmed against the user's
own description: material = outside-the-cylinder OR same-side-of-both-planes,
an inherently wide/sprawling combination, vs. `FwdCan`'s compact
AND-inside-cylinder-and-between-planes), so it was the type most exposed
by this padding artifact; `RoundC`'s two weakest-looking instances
(`part1.stp`, `part2.stp`) turned out to be the identical artifact.
Fixed by sampling strictly inside `box` (the same box passed to the
`make*` builder) instead of `gsolid.BoundBox` + padding -- confirmed
100/100 clean (`p1.stp` RevCan 300/300, `part1.stp`'s two `RoundC`
300/300 each) before re-running the full scan.

**Final, corrected results across all 52 STEP files in
`Test RoundCorners`**: every composite surface found matches its
independent CAD ground truth 300/300 -- 7 `RoundC` instances, 4 `RevCan`,
2 `FwdCan`, across 11 files. Convention confirmed: `check_sign(point,
surf) == 1` <-> `point` is on the material side.

**Still not achieved**: no `MultiRoundC` or `TCone` (Fwd or Rev) surface
appeared in any of the 52 files with this pipeline configuration
(`voidGen=False`, `simplify="no"`, `forceNoOverlap=False`, default
`skip_solids`) -- the original, explicit target of this verification was
never actually exercised. 21 of the 52 files additionally failed with
`core.py`'s `"no solid selected for translation"` (`self.meta_list`
empty after loading) -- investigated and confirmed **not a bug**:
`geo.Gload_step` genuinely returns 0 solids for every one of them, and
direct inspection of the raw STEP content confirms why -- each file has
exactly one `OPEN_SHELL` wrapping a single `ADVANCED_FACE`, zero
`MANIFOLD_SOLID_BREP`/`CLOSED_SHELL` entities. These are isolated
single-face exports (matching their names: `face0`-`face4`, `surf_0/1/3`,
`edge0`, `plane`, `ss`, `t`, `cc`, and, less obviously from the name
alone but confirmed identical in content, `piece2`/`bad_piece_1`/
`bad_rtn_1_1_0`/`bad_src_1`/`good_piece_2`/`good_rtn_1_1_0`/`good_src_1`)
-- almost certainly saved off during earlier debugging of one specific
problem face, not full-model fixtures. They cannot produce a
`MultiRoundCorner` under any settings, since there is no solid to
decompose. Verifying `check_sign` on `MultiRoundCorner`/`TCone` remains
open -- would need either a different STEP file set known to produce
one (an earlier, differently-configured scan of a 39-file version of
this same directory reportedly found 13 that did, per an earlier note in
this file -- not reproduced this session) or different `CadToCsg`
settings against the current 52-file set.

Diagnostic scripts (scratchpad only, not part of the repo):
`check_one_file.py`/`run_all_subprocess.py` (per-file subprocess scan,
crash-resilient against the project's still-unsolved native tangency
crash -- see "Motivating problem" at the top of this file), `diag_revcan.py`/
`diag_revcan2.py`/`diag_roundc_part1.py` (single-case component-by-component
instrumentation), `diag_no_solid.py`/`run_diag_no_solid.py` (the
zero-solids investigation).

### `check_sign` end-to-end verification, part 2: a wider fixture set, `MultiRoundCorner` finally validated, and a real `TCone_region` bug found and fixed

The 52-file `Test RoundCorners` set (part 1, above) turned out to be a
narrow/legacy fixture set -- the user pointed at a second, much richer
one: every subfolder of `Solidos/` except `Big_model_reserved` (large
models, slow to translate, deliberately excluded). 60 STEP files across
`Cans/`, `Enclosures/`, `Hollow_plates/`, `Multiplanes/`,
`Reversed_Cyl_Cones/`, `RoundCorners/` (41 files, `rc*`/`rrc*` naming),
`Torus/`, `trier/`. Re-ran the same `check_one_file.py`/
`run_all_subprocess.py` scan (already fixed to sample strictly inside the
construction `box`, per part 1) against this set.

**`MultiRoundCorner` validated for the first time this migration**: 17
instances across 13 files (`rc6/9/10/11/12/13/21/22/24`x2,
`rrc6/9/10/11/12/21/22/13`, `placathin.stp`) -- all 300/300. This closes
the original target of this whole verification effort. `RoundC`/`FwdCan`/
`RevCan` also came back clean across dozens more instances, consistent
with part 1's results, with 2 unrelated builder-side exceptions left
uninvestigated (`fwd_can_1.stp`/`rev_can_1.stp`, one `FwdCan` instance
each: `makeCan` raises `TypeError: 'bool' object is not iterable` --
noted, not yet root-caused).

**`RevTCone`: consistently and reproducibly wrong** in every file it
appeared in -- `placa.stp` (4 instances, 27-41/300), `placathin.stp` (4
instances, 31-42/300), `cyl_cone.stp` (1 instance, 157/300). Unlike
part 1's `RevCan` false alarm, this was **not** the box-padding artifact
(already fixed by this point) -- a real, reproducible discrepancy.
`FwdTCone` never appeared in any file, so it had no independent
cross-check at this point.

**Root cause, found by direct analogy with `Can`** (per the user's own
suggestion -- TCone's boolean logic "should be very similar or identical"
to Can's): `MetaSurfacesDict.TCone_region` (`geouned_classes.py:808`)
computed a single `configuration` value for *both* bounding planes purely
from the cone's orientation --
`configuration = "AND" if TCone.Orientation == "Forward" else "OR"` --
completely ignoring `TConeParams.p1_configuration`/`.p2_configuration`,
real per-plane data that has existed on the class since its definition.
`Can_region` never had this problem: it always read the real
`s1_configuration`/`s2_configuration`. Worse, `get_cell_object`'s
`"TCone"` branch (`build_region.py:97`, the function that builds the CAD
solid used as this verification's ground truth) *did* use the real
per-plane data (`geoObj.Surf.p1_configuration`/`.p2_configuration`) --
so `check_sign` (reading `.region`, built from `TCone_region`'s
fabricated value) and the CAD reference solid (built from the real data)
were silently evaluating two *different* boolean expressions for the
same physical surface. Confirmed empirically on `cyl_cone.stp`: real data
is `p1_configuration=p2_configuration="AND"`, but `TCone_region` was
substituting `"OR"` (cone `Orientation` is `"Reversed"`) -- matching
exactly the observed symptom (the CAD solid was a narrow AND-bounded
pocket along the cone's own axis, while `check_sign`'s wrongly-OR region
predicted material almost everywhere).

**Fix**: `TCone_region` now zips `(TCone.Surf.p1, TCone.Surf.p1_configuration)`/
`(TCone.Surf.p2, TCone.Surf.p2_configuration)` instead of computing a
single blanket value, mirroring `Can_region`'s existing pattern exactly.
Verified 300/300 on all 3 previously-failing files (`cyl_cone.stp`,
`placa.stp` x4, `placathin.stp` x4) and no regressions: `tests/geo` +
`tests/test_cadtocsg.py`, 156/156.

New diagnostic scripts (scratchpad only): `diag_revtcone.py` (component-
by-component instrumentation, same pattern as `diag_revcan.py`),
`diag_tcone_config.py` (the real-vs-hardcoded `p1_configuration` check
that pinpointed the bug).

### `check_sign` end-to-end verification, part 4: a real `can_region` sign bug for cone-plus-apex-plane Can components

Two `FwdCan` builder crashes noted in part 2 (`fwd_can_1.stp`/`rev_can_1.stp`,
one instance each: `makeCan` -> `get_cell_object` -> `region.to_integer()`
-> `TypeError: 'bool' object is not iterable`) turned out to be a real,
deeper bug, not the shallow missing-bool-guard issue it first looked like.

**First hypothesis, rejected**: `BoolSequence.to_integer()` lacks the
`isinstance(self.elements, bool)` guard its siblings (`copy()`,
`get_complementary()`) already have -- true, and it does crash on a
constant-`False` region. Patching just that was tried and reverted: the
user correctly pushed back that a `False` region here likely meant a
real sign bug upstream (the planes bounding a Can/cone probably wrong),
not a legitimately-empty region -- masking it with a guard would have
hidden a real geometry bug behind a silently-skipped surface.

**Root cause, confirmed against real geometry**: `fwd_can_1.stp`'s raw
solid is exactly 3 faces -- a cylinder and two cones sharing the same
axis and direction, one apex pointing into the material (protruding
cone, matches "FwdCan" naming) and one pointing out of it (recessing
cone) -- confirmed with a direct face dump. `decompose_solids()` splits
this into two sub-solids, cutting exactly at the first cone's apex
point; the small sub-solid (Volume ~7069, a real, non-negligible piece,
not a degenerate sliver) ends up bounded by the cylinder, that cone, and
the synthetic cut-plane -- which is geometrically the *same* plane the
Can-detection code (`build_can_params`, `utils/functions.py`) separately
computes as the cone's own `ApexPlane` (`cone_apex_plane()`, since the
apex lies on-axis). Taking a real interior point of that sub-solid
(`GSolid.find_interior_point()`, ground truth) and evaluating
`check_sign` against each individual component showed the contradiction
directly: `can_region`'s Cone-with-apex-plane-only branch, for a
*Reversed* cone (`basic_functions_part1.py`, was line 281), combined the
apex plane with the cone via **AND** (`BoolSurface(0,apid) *
BoolSurface(0,sid)`) -- but a real point known to be inside the physical
solid failed that AND's `apid` requirement outright, an unconditional,
point-independent contradiction (the expression reduces to `X AND NOT X`
purely from the signed ids involved, regardless of any point). Compared
against `MetaSurfacesDict.add_cone` (`geouned_classes.py:516-534`, the
already-validated Tier-2 standalone-Cone path, which handles the
identical "cone + its own ApexPlane" case): Forward combines via `*`
(AND) -- matches `can_region`'s Forward branch, no bug -- but **Reversed
combines via `+` (OR)**, not AND. `can_region`'s Reversed branch was
using AND where it needed OR.

Physical rule, confirmed directly by the user: for a cone + apex plane,
it's always (1) *inside* the cone -- AND with the apex plane, normal
along the cone's own axis direction, or (2) *outside* the cone -- OR
with the apex plane, normal opposite the axis. Forward orientation is
case (1), Reversed is case (2) -- so Reversed always combines with `+`,
never `*`, regardless of anything else going on in the expression.

**A second instance of the identical mistake was found by systematically
re-checking every branch of `can_region` that touches `apid`** (the
user's explicit ask, after the first fix, before accepting it: "verifica
si error de este tipo no se ha colado en otra rama de la logica"). Of
the 4 branches combining `apid`: `pid is None`/Forward (`*`, correct),
`pid is None`/Reversed (`*`, **the confirmed bug**, now `+`), both-present/
Forward (`*` in both AND and OR sub-cases, correct), both-present/Reversed/
AND (`+`, already correct) and both-present/Reversed/**OR** (was `*` --
inconsistent with its own AND-configured sibling one line above, which
already used `+`). Per the physical rule above (Reversed always uses OR
for `apid`, unconditionally), the OR-configured sub-case was fixed the
same way, from `BoolSurface(0, apid) * (BoolSurface(0, sid) *
BoolSurface(0, -pid))` to `BoolSurface(0, apid) + (BoolSurface(0, sid) *
BoolSurface(0, -pid))` (`basic_functions_part1.py`, was line 303) -- this
second one had no test data exercising it directly, fixed by the same
confirmed physical rule rather than by independent point-level
verification.

Verified: `fwd_can_1.stp`/`rev_can_1.stp` both 300/300 after the fix (no
`to_integer()` guard needed at all -- the region is a real, correctly-signed
`BoolSequence` now, never collapses to a bare bool for these cases); full
`tests/geo` + `tests/test_cadtocsg.py`, 156/156; full 60-file re-scan of
`Solidos/` (part 2/3's set), every composite surface still 300/300,
including the previously-crashing files.

### `check_sign` end-to-end verification, part 5: `MultiPlane` was never actually reachable -- `meta_surfaces.py::multiplane()` class-vs-instance bug

User request: verify `check_sign` for `MultiPlane`, the one tracked
composite type never seen in any of the 112 files scanned across parts
1-4. `Solidos/Multiplanes/multiplane_hollow.stp` -- named and foldered
specifically for this -- produced zero `MultiP` surfaces too (only 15
bare `Planes`), which the user immediately recognized as a familiar
symptom from developing this code: *"muchos planos se añaden al set
omit_solid... no detecta los multiplanes."*

Traced `generators.py::get_surfaces`'s exact call order (`next_Can` ->
`next_truncCone` -> `next_roundCorner` -> `exclude_no_cutting_planes` ->
`next_multiplanes` -> ...), instrumenting `omitfaces` after each stage
for `multiplane_hollow.stp`: Can/TCone/RoundCorner contributed nothing
(no cylinder/cone in this file), but `exclude_no_cutting_planes` ->
`external_plane()` marked 6 of the 15 planes as "external" before
`next_multiplanes` ever ran. That part matched the user's intuition
(planes getting excluded before multiplane detection sees them) but
turned out to be a red herring for *this* file: `external_plane()`'s own
`region_sign`-based AND/OR logic was working correctly on inspection.

**Real root cause, found by re-reading `meta_surfaces.py::multiplane()`
(the function `next_multiplanes` actually calls) line by line**: `Gclassify_curve(e)`
returns an *instance* (`GLine(curve)`, `_freecad_impl.py:513`), never the
bare class -- confirmed by grep, every other call site in the codebase
(`decom_utils_generator.py`, `meta_surfaces_utils.py`, ~10 sites) already
uses `type(Gclassify_curve(e)) is GLine`. `multiplane()` alone was still
using the bare `Gclassify_curve(e) is GLine` -- comparing an instance
against a class, which is *never* true. Every edge of every plane, in
every solid, ever passed through `multiplane()`, therefore hit `if
type_curve is not GLine: continue` unconditionally, so `multiplane()`
could never find an adjacent plane through any edge -- it always
returned just `[master_plane]` alone, and `next_multiplanes`'s own `if
len(mplanes) != 1: ...` guard then always skipped it. **This function
has been unable to produce a `MultiPlane` surface for any input at all**,
not just for this one file -- consistent with zero occurrences across
all 112 files scanned in parts 1-4, before this fix.

Fixed (`meta_surfaces.py:43`): `if type_curve is not GLine:` ->
`if type(type_curve) is not GLine:`, matching the established convention
everywhere else. (`multiplane_old`/`multiplane_loop`, two dead functions
in the same file with the identical bug and zero callers anywhere,
confirmed via grep -- left alone, out of scope, not wired into anything.)

Verified: `multiplane_hollow.stp` now produces `MultiP x2`, both 300/300.
Full `tests/geo` + `tests/test_cadtocsg.py`, 156/156, no regressions.
Re-ran the full 60-file `Solidos/` scan: **every result still 300/300**,
and `MultiP` now appears in 9 files that previously showed none --
including `combi_MP_RC.stp` and `RC_MP.stp`, whose names make clear they
were built specifically to exercise a MultiPlane+RoundCorner combination
that, before this fix, only ever showed the RoundCorner half. Several
files' `RoundC`/`RevCan`/`RevTCone` counts also shifted (e.g. `placa2.stp`:
`RoundC` 3->1, `RevCan` 4->6, `RevTCone` 0->2; `double_RC.stp`: `RoundC`
6->5, gained `MultiP` x3) -- expected fallout, not a regression: faces
that used to be swept into the wrong composite type (or left as bare
planes) for lack of a working MultiPlane path now group correctly, and
every resulting classification still validates 300/300 against
independently-built CAD ground truth. This is a materially more
significant fix than the sign bugs in parts 3-4 -- it affects real-model
face classification broadly, not just one meta-surface type's boolean
convention.

New diagnostic scripts (scratchpad only): `diag_multiplane_omit.py`
(the `omitfaces`-per-stage trace), `diag_multiplane_geom.py` (raw face
dump + `region_sign` check that ruled out `external_plane()` as the
cause for this file).

**`RevCC` (`ReversedConeCyl`) remains unverified -- and, per the user,
this scan methodology can never reach it.** Explicit clarification from
the user before stopping this session: a `ReversedConeCyl` is defined as
the *exterior* of several cylinders whose axes are nearly (not
necessarily exactly) parallel and which overlap each other. Unlike
`Can`/`TCone`/`RoundCorner`/`MultiRoundCorner`/`MultiPlane` -- all of
which get identified in `decompose/generators.py::get_surfaces` and used
to *cut* the main solid via `Gsplit` (the first-level decomposition this
whole verification effort scans for) -- a `RevCC` surface never cuts
anything. It only ever gets identified in the **conversion** module
(walking the faces of an *already-decomposed* solid element to
reconstruct its CSG expression -- see `add_reversedCC`,
`geouned_classes.py:1000`), describing an external bounding surface of a
cell after the fact. So scanning STEP files through
`decompose_solids()`/`build_solid_definition()` and looking for `RevCC`
in `geo.Surfaces` (the same method used for every other type in this
session) is structurally the wrong approach -- it will always show zero,
regardless of the STEP fixture set used. Verifying `check_sign` for
`RevCC` would need a different methodology, entered from the conversion
side rather than the decomposition side -- not attempted yet, picked up
next session.

## MCNP stochastic volume check: does the CSG translation match the CAD volume?

Different question from the `check_sign` verification above: that
verified the *algebra* GEOUNED writes for composite surfaces is
internally consistent with an independently-built CAD solid. This
verifies something more end-to-end -- that a *fully converted* MCNP
model, run for real, reproduces the *same volumes* as the original CAD
solids. GEOUNED finishing without raising an exception does not by
itself mean the CSG geometry equals the CAD geometry (e.g. a wrong
surface sign can silently produce a cell with the wrong shape but a
valid boolean expression).

**Mechanism** (already existed in the codebase, not new this session):
`CadToCsg.export_csg(..., volSDEF=True, volCARD=False)` writes an extra
MCNP block -- a photon source emitted isotropically inward from a
sphere enclosing the whole model, plus an F4 track-length tally on every
solid cell, each normalized (`SD4`) by that cell's CAD-computed volume.
Run in `MODE P` + `VOID` (no real physics), this is the standard MCNP
chord-length volume-estimation technique: **the expected tally result is
exactly 1.0** if the CSG cell's real volume matches CAD; a significant
deviation means the surfaces/signs written don't reconstruct the same
solid. Requires `settings.voidGen=True` (the sphere is built by void
generation) and the full `geo.run()` pipeline (decompose + build +
**void** -- `write_files.py`'s `Surfaces["Sph"][-1]` lookup raises
`IndexError` if `build_void()` was skipped).

**This session's run**: 110 STEP files converted to MCNP-only output
(mirroring the user's own `Test RoundCorners/myrun.py` settings --
`voidMat`/`dummyMat` for a placeholder void material, `minVoidSize=20`),
across `testing/inputSTEP` (repo's own pytest fixtures, excluding the
slow/pre-existing-failure `large/` subfolder) and `Solidos/` (excluding
`Big_model_reserved`), each run through the user's own `d1suned`
(an in-house MCNP variant) via WSL. Full pipeline, scripts, environment
gotchas, and results archived at
`\\wsl.localhost\Ubuntu-22.04\home\patrick\work\taller\SolidTestMCNP\scripts\`
(`README.md` there has the complete writeup) -- summarized here:

- **Environment gotchas worth remembering** (all specific to *automating*
  WSL from Windows, none of them problems with the user's own setup):
  `wsl.exe` invoked from Git Bash mangles `/home/...`-looking paths
  unless prefixed with `MSYS_NO_PATHCONV=1`; `.bashrc`'s standard "if not
  interactive, don't do anything" guard silently skips the user's `~/bin`
  PATH addition and `source /opt/intel/oneapi/setvars.sh` line whenever
  reached non-interactively (i.e. always, for automation) -- worked
  around by hardcoding the resulting `PATH`/`LD_LIBRARY_PATH` values
  directly rather than sourcing the vendor script; multi-line `wsl.exe
  -- bash -c '...'` inline strings silently lost variable expansion
  (`$i` came out empty) in a way a real script *file* did not, and
  sourcing `setvars.sh` *from within* a script file silently terminated
  the whole shell at that line for reasons not fully root-caused --
  worked around by always using a real `.sh` file, never sourcing the
  vendor script, and using plain `( cmd ) &` + periodic `wait` instead of
  `xargs -I{}` (which also silently processed zero directories through
  the same multi-layer quoting).
- **Results**: 261 solid-cell tally results across 111 run directories
  (110 real + 1 manual test). 92.3% within 2 sigma of 1.0; 5.7% between
  2-3 sigma (statistically expected noise at this sample size, ~4.3%
  predicted by chance alone -- not evidence of a problem); **5 real
  failures beyond 3 sigma**: `solidos_Reversed_Cyl_Cones_cyl_cone` (tally
  exactly 0.0 -- the same file used earlier this session to find the
  `TCone_region` bug, so worth re-checking carefully as recently-touched
  code), `repo_DoubleCylinder_placa3` (0.254, 535 sigma -- volume ~1/4 of
  CAD), `repo_Misc_PiezaDavid` (1.085, 39 sigma), `repo_SCDR_90` (0.919,
  38 sigma -- already marked `# fails (pre-existing)` in
  `tests/test_cadtocsg.py`'s own skip list, a known issue, not new), and
  `repo_Torus_solid1` (0.971, 3.84 sigma, borderline but real). One file
  (`solidos_Torus_2_degen_torii`) produced no tally section at all
  (d1suned exit 152) -- not yet investigated.

**Update, next session**: `cyl_cone`'s zero-tally failure investigated and
fixed -- see "`ReversedConeCylinder`'s AND/OR grouping..." below.
`placa3`/`PiezaDavid`/`Torus_solid1`/`2_degen_torii`'s exit-152 case
remain open. `SCDR_90` still lowest priority (known, pre-existing).

### `ReversedConeCylinder`'s AND/OR grouping was decided from an arbitrary, non-invariant direction -- found and fixed via the MCNP volume check

`cyl_cone.stp`'s zero-tally failure (above) traced to a real bug, found
by working entirely from the *.mcnp file's own surface/cell-definition
text (per the user's explicit request -- verify from the GEOUNED output
alone before touching Python internals). Parsing the raw MCNP surface
cards and cell-definition string directly (implementing the standard
P/PX/PY/PZ/C/Y/K/Y sense conventions by hand) and cross-checking against
`check_sign` at a real interior point of the actual CAD solid (found via
`GSolid.find_interior_point()`) surfaced a **unit mismatch** first --
GEOUNED writes MCNP surfaces in cm, but that interior point is in mm
(STEP/FreeCAD's native unit); comparing them directly gave nonsense
signs for several surfaces. Fixed the test methodology (scale the point
by 0.1) before drawing any conclusion -- after which `check_sign` and
the hand-rolled MCNP-file parser agreed on every surface but one.

**Real finding**: at that known-real point, exactly one surface (a
plane, part of the third of three `ReversedConeCylinder` groups in this
model) evaluates to the wrong sense. The model has 3 such groups (each a
cylinder+cone+cylinder chain with nearly-parallel axes, per how
`ReversedConeCylinder` gets identified in the first place -- see
`get_reversed_cone_cylinder`/`get_join_cone_cyl`); two of the three
correctly combine their 3 junction planes via OR (`OR[10,11,12]`,
`OR[16,17,15]`), the third combines the same shape of triple via a flat
AND (`AND[20,21,22]`) instead -- confirmed via the user's own domain
knowledge that all 3 groups should be OR, matching a real physical
feature. A quick numeric check (`convex_planes` applied to the actual
plane data of all 3 groups) confirmed all 9 planes individually satisfy
the intended physical invariant the user described (each plane's own
normal points away from its cylinder's axis) -- ruling out
`gen_plane_cylinder`/`gen_plane_cone` as the cause, contrary to the
user's own first guess.

**Root cause, found by instrumenting `get_join_cone_cyl` directly**
(`meta_surfaces_utils.py`): the AND/OR operator between adjacent
cylinder/cone pieces was decided *incrementally, per connection*, via
`operator = "AND" if d.dot(adjPlane.Axis) > 0 else "OR"`, where `d` is
derived from the *seed face's own UV-parameter traversal direction*
(`(pmax - pmin).normalized()` over that face's `ParameterRange`). This
is not a geometrically invariant reference: a STEP/CAD kernel's face
parametrization direction is implementation-defined, not guaranteed
consistent between multiple physical instances of "the same" feature.
Traced (via temporary debug prints, reverted after use) that the two
correct groups' seed faces both had `d` pointing roughly along -X, while
the broken group's `d` pointed roughly along -Z -- a real, confirmed
inconsistency, not a coincidence.

**The user's own stated design intent, once asked directly**: for a
group of planes whose normals lie roughly in a common plane (because the
cylinders/cones they bound have nearly-parallel axes), first confirm
their intersection/union forms a convex arrangement, then classify by a
single, group-wide test -- normals pointing outward from the group's own
centroid means OR, inward means AND. This is *exactly* what
`convex_planes` (`functions.py:525`) already computes and already uses
for `RoundCorner`/`MultiRoundCorner` (`build_roundC_params`) -- it just
wasn't being reused for `ReversedConeCylinder`. Verified numerically
before touching any code: calling `convex_planes` on the real plane data
of all 3 groups in `cyl_cone.stp` returns `convex=True,
orientation="Forward"` for **all three**, including the currently-broken
one -- i.e. all 3 groups get the *same* classification from
`convex_planes`, matching the user's confirmation that all 3 should be
OR. This fixed the `Forward`->`OR`, `Reversed`->`AND` calibration with
certainty before writing the fix.

**Fix**: `build_RCC_params` (`functions.py:259`) no longer walks
`.Connections` incrementally to build `PlaneSeq` -- it collects every
piece's own junction plane into one flat list and calls
`convex_planes(group_planes, cylcones[0].Surf.Axis)` once for the whole
group, building `PlaneSeq` as one OR-bracket (`orientation=="Forward"`)
or a flat AND list (`"Reversed"`) accordingly. Confirmed via direct
inspection (`len(rc)` is actually 3 per group here, not 2 as first
assumed from the plane count alone -- each of the 3 pieces contributes
its own junction plane, `AddPlanes` is empty for all 3 groups in this
file, so `PlaneSeq`'s own grouping is the *entire* determinant of the
observed AND/OR difference, not a confound from a separate always-OR
`AddPlanes` set as first suspected). `get_join_cone_cyl`
(`meta_surfaces_utils.py:275`) no longer computes the now-unused `d`/
`operator` at all -- the whole block computing it (previously lines
387-415) is deleted, along with the now-fully-unused `.Connections`
field on `reversedCCP` and the `parent_id` parameter it existed to
support (removed from `get_join_cone_cyl`'s signature and both call
sites, confirmed via grep to have zero other consumers).

Verified: `cyl_cone.stp`'s cell 1 F4 tally went from exactly `0.0` to
`0.996547 +/- 0.29%` (matching 1.0 within ~1.2 sigma). Full `tests/geo` +
`tests/test_cadtocsg.py`, 156/156, no regressions. Re-ran the entire
110-file MCNP volume-check batch: `cyl_cone` no longer appears among the
failures (5 -> 4), every other result unchanged (still 92.7%/5.7%/1.5%
in the 2-sigma/marginal/fail buckets) -- confirms the fix is both
correct and has no wider blast radius, despite touching shared
(`meta_surfaces_utils.py`, `functions.py`) code.

**Lost-particle check added to the analysis** (per explicit request,
`scripts/analyze_results.py`): scanned all 111 `outp` files for MCNP's
"N particles got lost" summary line (a *different* class of geometry
error than the volume-tally check -- a real gap in the CSG where no
cell is defined at all, `"no cell found in subroutine newcel"`,
independent of whether any given cell's own volume is correct). Only
one file has any: `solidos_Enclosures_w_encl` (`w_encl.stp`), 10 lost
particles (MCNP's default abort threshold, terminating that run early
at nps=884 of the requested 1e6 -- its own cell 1 tally still happened
to fall within 2 sigma despite the much-reduced statistics, so it
wasn't otherwise flagged). Not yet investigated -- a real, distinct
finding for next session, likely related to this file's `Enclosure`-
type solids specifically (the one folder in this fixture set built
around that feature).

## Code style preference

- User prefers speaking/planning in Spanish, but ALL code — including
  comments, docstrings, and variable/function names — must be written
  in English.
