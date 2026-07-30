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

As of the file-by-file closeout pass, 2 files in `GEOUNED` still carry a
direct `import Part`/`import FreeCAD`, each scoped to a small, itemized
set of deliberately deferred native uses with no faithful `geo` equivalent
(verified: everything else in those files — classification, construction,
vector arithmetic — now goes through `geo`). All are duck-typed native
*values* flowing through, not module-level dependencies leaking outward —
nothing outside these 2 files needs to `import Part`/`FreeCAD` itself to
consume them. Files documented in earlier passes (`conversion/
cell_definition_functions.py`, `utils/geometry_gu.py`,
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

- `core.py`: `self.geometry_bounding_box = FreeCAD.BoundBox(...)` in
  `_set_geometry_bounding_box` — consumed natively well outside this
  migration's scope (`void.py`, `write_files.py`,
  `geouned_classes.py`'s `self.UniverseBox`).
- `utils/meta_surfaces_utils.py`: one `isinstance(e0.Curve, (Part.Circle,
  Part.Ellipse, Part.Hyperbola, Part.Parabola))` in `planar_edges` —
  `Gclassify_curve` doesn't model Hyperbola/Parabola, so narrowing this
  to the 2 supported kinds would silently change behavior for the other 2.
- `utils/build_shape_functions.py`: fully closed out — see "convert at
  the origin: build_shape_functions.py's GVector-only rewrite" below.

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
- Tier-1 `*OnlyParams` vs `geo`'s `GPlane`/`GCylinder`/`GCone`/`GSphere`/
  `GTorus`: still two separate representations of the same analytic
  surfaces. Not yet consolidated.
- The broader design goal (stated by the user): a homogeneous
  `components`/`definition`/`bVar` representation covering *every*
  surface — simple and composite alike — so a simple surface is just the
  1-component, trivial-`definition` degenerate case of the same shape a
  Can/RoundCorner uses. `.Type` (the string tag) must be kept regardless
  of how this evolves — `write/*.py`'s `mcnp_surface` dispatches the MCNP
  card *format* on it, which is an orthogonal concern from the geometric
  boolean-composition redesign.

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

## Code style preference

- User prefers speaking/planning in Spanish, but ALL code — including
  comments, docstrings, and variable/function names — must be written
  in English.
