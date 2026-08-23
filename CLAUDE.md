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

**Correction, later same session**: `placa3`'s "535 sigma" figure above
was a stale-output-file artifact of this pipeline, not a real failure --
see "`placa3`'s ... volume failure was a stale-output-file artifact"
below for the full account. `PiezaDavid`/`Torus_solid1`/`2_degen_torii`
were independently re-checked and are confirmed real.

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

### `placa3`'s "535 sigma" volume failure was a stale-output-file artifact, not a real bug -- a d1suned/MCNP gotcha for this whole verification pipeline

Investigated `repo_DoubleCylinder_placa3` (0.254, 535 sigma from the
original batch) as the next-worst failure after the `cyl_cone` fix above.
Several controlled experiments were run first, each comparing a real,
GEOUNED-native transform of the solid (`GSolid.translate()`/`.rotate()`,
both thin wrappers around native `Part.Shape.translate()`/`.rotate()` --
a rigid `gp_Trsf` move, not a re-tessellation) followed by
`.export_step()` and a full reconversion:
- Rotating `placa3.step`'s cylinder/cone axis to align with `+Y` (moving
  it off the `GQ` quadric form onto an axis-aligned `K/Y` cone, same
  distance from origin, ~9487): tally went from the reported 0.254 to
  0.996.
- Translating `placa3.step`'s center of mass to the exact origin, no
  rotation (keeping the `GQ` form): tally went to 0.9998.
- A direct structural diff of the two resulting `.mcnp` files against the
  original confirmed the cell's boolean expression text and surface-type
  sequence were byte-identical -- only numeric surface parameters
  differed -- ruling out a decomposition/classification difference as
  the explanation.

This looked like a real distance-dependent `GQ`-precision bug (this
project's own `q_form_cone`/`q_form_cyl` compute far-from-origin
position-dependent coefficients that could plausibly suffer catastrophic
cancellation). **It wasn't.** Every attempt to reproduce it synthetically
failed: rotating/translating the already-verified-clean `cyl_cone.stp`
to placa3's *exact* axis, *exact* absolute center (not just matching
distance), and confirmed-identical radius (50.0) and semiangle (7.0000
deg) always gave a perfect tally (0.999-1.000), at every distance tested
from 50 to 50000 units. Checking placa3's own real axis/semiangle values
to full float precision (`0.6427876096867989`, `-0.7660444431187603`,
`SemiAngle=7.000000000022743`deg, `|Axis|` deviation from 1.0 exactly
`0.0`) showed no meaningful numerical noise either -- ruling out a
"Fusion-360-sourced imprecision" hypothesis too.

**The actual cause, found by diffing the two `.mcnp` files directly**:
`repo_DoubleCylinder_placa3/model.mcnp` (the original, "failing" run)
and a version reloaded/re-exported through a bare, transform-free STEP
round-trip (`solid = Gload_step(...)[0]; solid.export_step(...)`, then
reconverted) were **byte-identical** apart from two comment lines
(source filename, creation date) -- an impossible result if a
deterministic solver produced 0.254 for one and 0.9998 for the other.
The real explanation: `d1suned`/MCNP **refuses to overwrite an existing
`outp`/`mctal`/`runtpe`** -- if those files already exist when a run
starts, it silently writes to `outq`/`mctam`/`runtpf` instead. The "Re-ran
the entire 110-file MCNP volume-check batch" step mentioned above (after
the `cyl_cone`/`ReversedConeCylinder` fix) did exactly this for several
run directories, including `placa3` -- it produced a fresh, correct
`outq` (13:29, tally `0.999842 +/- 0.30%`) sitting right next to the
stale pre-fix `outp` (09:51, tally `0.254`) -- but `scripts/analyze_results.py`
only ever scans files literally named `outp`, so it kept silently
reporting the stale number, and every "confirmed" observation made this
session about `placa3` (round-trip healing, distance sensitivity, GQ
precision) was actually chasing a phantom -- **`placa3` was already
correct with the current code before any of this session's investigation
started.** A forced clean rerun (`rm -f outp runtpe mctal`, then
`d1suned` fresh) on the untouched original `model.mcnp` confirms this:
`0.999842 +/- 0.30%`, matching `outq` exactly.

**The other 3 previously-reported failures were checked for the same
stale-`outp`/live-`outq` split and are confirmed real, not artifacts**:
`repo_Misc_PiezaDavid`, `repo_SCDR_90`, `repo_Torus_solid1` each also
have an `outq` sibling from the same later batch rerun, but in all 3
cases `outq`'s tally is byte-identical to `outp`'s (`1.08460`, `0.91941`,
`0.97055` respectively, matching the original report) -- these files'
`model.mcnp` wasn't touched by the `ReversedConeCylinder` fix, so the
"rerun" reproduced the same (still genuinely wrong) result under a
different filename. `solidos_Torus_2_degen_torii` also still hits the
same `fatal error. f card cell 1 of bin 1 tally 4 not found` in both
`outp` and `outq` -- also confirmed real, unrelated to this artifact.

**Open follow-up, not yet done**: `scripts/analyze_results.py` needs to
either always force-clean (`rm -f outp mctal runtpe outq outr mctam mctan
runtpf runtpg`) before each rerun, or scan for and warn about `outq`/`outr`-style
siblings so a skipped cleanup step is never silently invisible again.
The corrected failure list going into next session is just **3 real
issues**: `PiezaDavid` (39 sigma), `Torus_solid1` (3.7 sigma, borderline),
and `2_degen_torii`'s fatal tally error -- `SCDR_90` remains the known,
pre-existing, lowest-priority one already marked in `tests/test_cadtocsg.py`.

**Genuine topological findings on `placa3.step`, kept on record as a
concrete pyOCC test case per explicit user request** -- these are real
properties of the file (confirmed present identically in both the
original and the STEP-round-tripped copy, so not an artifact of any
transform), independent of the stale-output confusion above, and exactly
the kind of silent-error category this project's eventual pyOCC
migration (face-adjacency graph, non-manifold-edge handling -- see
"Motivating problem" at the top of this file) is meant to address:
- **Two multi-wire planar faces**: face index 3 (3 wires, 27 edges) and
  face index 20 (3 wires, 25 edges) out of 24 total faces -- a plane
  with holes/islands rather than a single simple outer boundary. Not
  necessarily a defect on its own, but exactly the shape `GFace.wires()`/
  `.outer_wire()`'s heuristic (`pick_outer_wire`) has to disambiguate
  correctly.
- **One non-manifold edge, shared by only 1 face instead of 2**: a
  `BSplineCurve` of length 20.0, tolerance `1e-7`. `native.isValid()`
  still reports `True` for the whole solid despite this -- OCC tolerates
  it, but it is precisely the kind of signal `GSolid.faces_sharing_edge()`
  (added earlier this migration, currently uncalled anywhere in GEOUNED)
  exists to detect.
- **The `ReversedConeCylinder` chain's 9 segments use two slightly
  different axis vectors instead of one consistent axis**: 5 segments
  use `(0.6427876096867989, -0.7660444431187603, ~0)`, the other 4 use
  `(0.6427839377443012, -0.766047524229359, 0.0)` -- differing at the
  5th significant digit (~1e-5 relative), the same order of magnitude as
  this file's elevated BRep edge/vertex tolerance (up to `5.21e-05`,
  vs. a flat `1e-07` after STEP re-export). A real example of "near-but-
  not-exactly-parallel" axes as actually authored by upstream CAD
  tooling (this file's STEP header identifies it as from Autodesk Fusion
  360, `FILE_NAME('Open CASCADE Shape Model',...,'Fusion001',...)`, in a
  richer AP214 representation -- `PCURVE`/`SURFACE_CURVE`/
  `DEFINITIONAL_REPRESENTATION` entities that `cyl_cone.stp`'s simpler
  legacy-style export has none of) that any future is-this-the-same-axis
  comparison (`is_same_cylinder`, `get_join_cone_cyl`'s adjacency walk,
  `convex_planes`) must tolerate correctly, rather than silently treating
  as two different features.

### Hidden-surface decomposition experiment on SCDR_90.stp: manually reconstructing a RoundCorner, a real `Gsplit` under-separation bug, a `my_distToshape` crash fix, and a closing MCNP/d1suned verification that is NOT yet clean

Follow-up to the abandoned "cut-order optimization" idea floated in an
earlier claude.ai-chat-authored note (previously left in this exact spot
in the file, referencing a since-deleted experimental script
`src/geouned/tools/optimize_split.py` — removed, along with a stray
`opt_out.txt` debug-output file at the repo root, neither ever
integrated or committed). That script was reviewed and smoke-tested
(quick-mode heuristics showed no order-sensitivity for `SCDR_90.stp`),
but the user's actual direction turned out to be different: not
permuting `get_surfaces`'s own candidates, but *manually identifying
analytic surfaces that aren't present as real faces* in the STEP file
and using them as cutting tools — worked through end-to-end as a single
concrete example on `SCDR_90.stp` (`Solidos/convierte_bad_volume/`).

**The hidden surface, found by inspection**: two semicircular edges
(R=37mm) on `SCDR_90.stp`'s two flat end caps (Y=32.4997 and Y=67.4997),
same axis (Y), centers aligned along that axis (X=400.7668, Z=6.8842 for
both). The solid's actual wall there is a wider R=40 cylinder necking
down to R=37 via two short cone frustums right at each end — so the R=37
surface is genuinely hidden, implied only by the two end-cap rims, not
present as any real face.

**Building the RoundCorner shape — two wrong constructions before the
right one, each ruled out by direct volume/geometry checks, not
guessed**:
1. Cylinder (R=37, axis Y) trimmed by the 2 tangent planes at the arc's
   endpoints (`p1`/`p2`) ANDed with the cylinder alone — mathematically
   incapable of trimming anything: the 2 tangent points are *exactly*
   antipodal (combined arc length 116.2389mm = pi*37 to 5 significant
   figures), so the 2 tangent planes are parallel and tangent to a
   convex shape removes zero volume from it. Confirmed numerically
   (every sign combination reproduced either the full cylinder or
   nothing) before abandoning it.
2. `p1 AND p2 AND (NOT-cylinder OR pc)` (`pc` = the plane through all 4
   relevant vertices, i.e. through the axis and the tangent-point
   chord) — built a real, valid half-cylinder wedge, but the *wrong*
   shape entirely: far too small, missing essentially the whole
   surrounding "corner block."
3. **Correct, user-confirmed formula**:
   `region = box AND (cylinder OR pc) AND p1 AND p2`, where `p1`/`p2`
   are the tangent planes (each plane's material normal points toward
   the *other* tangent point) and `pc`'s material normal has a negative
   z component, using a box exactly the size of the working solid's own
   BoundBox (no padding). Verified: `Volume=378926.0424`, `isValid()`
   `True`, spans the full visible feature width — `Gsplit` against the
   real solid then gives a clean 2-piece split with volume conserved to
   0.003 out of 346082.

**Reusable recipe** for this kind of hidden-cylinder-plus-tangent-planes
feature, going forward: (1) find 2 same-axis, axis-aligned-center
partial-circle edges of matching radius; (2) at each edge's 2 endpoints,
the tangent plane there (material normal = radial direction, sign
pointing toward the *other* tangent point) gives `p1`/`p2`; (3) the
plane through all 4 relevant vertices (2 endpoints x 2 axial extremes)
— containing the axis and the tangent-point chord — is the additional
plane `pc`; get its sign from real data/the user, not from a
"push-a-boundary-point-inward" heuristic (that heuristic gave the wrong
sign the first time this was tried); (4)
`box AND (cylinder OR pc) AND p1 AND p2`, box = the exact working
BoundBox, no padding. Recall: `Gmake_half_space(plane)` keeps the
**-axis** side, while GEOUNED's own convention is a plane's `.Axis`
always points toward material — build cutting half-spaces with
`GPlane.from_values(position, -material_normal)`, never
`material_normal` directly.

**A second decomposition step surfaced a fresh, minimal, reproducible
instance of this whole migration's original motivating bug** (see
"Motivating problem" at the top of this file): cutting one of the
resulting pieces (`piece_0`, Vol=54556.92) by the `pc` plane alone
should give 3 fragments (confirmed independently, see below) but
`Gsplit`/`BOPTools.SplitAPI.slice` consistently returned only 2, across
every box size (10 to 200 units) and even a genuinely unbounded
half-space tool, and across every tolerance from 1e-4 to 1e-13 —
ruling out both "plane too small" and the usual tolerance-retry remedy.
Confirmed via direct point sampling that real material exists on both
sides of `pc` near the under-separated region, and that the specific
vertex involved (near X=363, Y=35.5, Z=-6, an R40-circle vertex on the
cone-transition faces) sits at signed distance **-0.0020mm** from `pc`
— i.e. essentially exactly on it. **Root cause, confirmed by bypassing
`Gsplit` entirely**: plain native boolean `.common()` on the identical
two half-spaces of `pc`, run separately per side, correctly returns 3
solids (2 on one side, 1 on the other) that sum to the same total volume
— `BOPTools.SplitAPI.slice` silently merges 2 of those into a single
output solid at the near-tangent vertex, something plain `.common()`/
`.cut()` on the same inputs does not do. **Not yet fixed in `Gsplit`
itself** (its docstring already promises `solids` is never silently
missing pieces — this is a concrete counterexample for the
`BOPTools.SplitAPI.slice` path specifically, distinct from the
tiny-tolerance-exception retry it already handles). A first proposed
fix ("fall back when `Gsplit` returns fewer solids than expected") was
rejected by the user for a correct reason: in normal GEOUNED operation
there is no ground truth for how many fragments a cut *should* produce
to compare against — only in this investigation was the right answer
(3) independently knowable. **Reframed goal**: detect the *precondition*
instead — recognize near-degenerate/"thin" planes or edges in the
source CAD that are themselves modeling artifacts and are exactly the
kind of near-exact tangency this bug needs to trigger — not started.
Regression fixture saved:
`Solidos/BadCADModel/SCDR_90_piece0_gsplit_tangent_bug.stp` (feed it
plus the `pc` plane, both fully characterized above, into `Gsplit` to
reproduce).

**A real, previously-undiscovered bug in `Gcommon`'s multi-tool
behavior** was also found while building the RoundCorner wedge:
`Gcommon(solid, [tool1, tool2])` (wraps native `shape.common(list)`)
does **not** compute `solid AND tool1 AND tool2` — it returned an empty
result for a case where an inclusion-exclusion bound
(`|A∩B| >= |A|+|B|-|U|`) proves the true intersection cannot be empty.
Fixed by chaining pairwise `Gcommon` calls instead of passing multiple
tools at once — not yet fixed inside `Gcommon` itself. Only 1 production
call site (`void_box_class.py:133`) and it already passes a single-
element list, so this bug is currently dormant in production, not live;
`Gfuse`/`Gcut` were not checked for the same pattern.

**Closing this out**: per explicit request, took the 5 final pieces this
manual decomposition produced (the RoundCorner wedge's 2 halves, the
`pc`-plane split of `piece_0` — worked around the `Gsplit` bug above via
per-side `.common()` — and the final split of one of those by a real
existing face's plane, `face[14]`, `Axis=(-0.8973,0,0.4415)`, found by
matching 3 given approximate vertex coordinates against the model's real
vertex list) and fed them **directly into GEOUNED's conversion phase,
skipping `decompose_solids()` entirely** (`meta_list = [GeounedSolid(0,
[gsolid1, ..., gsolid5])]`, then `build_solid_definition()` +
`build_void()` called directly — mirrors what `decompose_solids()`'s own
output would look like, since each already-cut piece is itself a
`GSolid`), then wrote MCNP with `volSDEF=True` and ran d1suned, per the
"MCNP stochastic volume check" method documented earlier in this file.

**A second real, blocking bug found and fixed en route**:
`GFace.my_distToshape` (`geo/_freecad_impl.py`) does
`try: shape1.common(shape2) except: shape2.common(shape1)` — but when 2
faces genuinely don't intersect, native `.common()` can raise
`ValueError: Null shape` on *both* orderings, and the bare `except` only
tried the second ordering without catching *its* failure too, crashing
`simple_solid_definition`'s multiplane detection outright the moment it
was exercised on a face combination `decompose_solids()`'s own output
never produces (this is live production code —
`FaceGu.distToShape`/`geometry_gu.py` calls `my_distToshape` directly,
not the more robust native `distance_to`). Fixed: wrapped the second
`.common()` call in its own try/except too, treating a double-failure
as "no intersection" (same outcome as an empty compound).

**Result: not a clean pass.** The combined 5-piece cell's tally came
back at 0.46524 (191 sigma from 1.0). Isolated by converting and running
d1suned on each of the 5 pieces *individually*:

| piece | true volume (mm^3) | tally | sigma | verdict |
|---|---|---|---|---|
| RoundCorner-wedge half (the bigger, "irreducible" one) | 291525.53 | 0.40156 | 187.0 | **wrong** |
| `pc`-split half (smaller) | 9264.41 | 0.99720 | 0.7 | correct |
| `pc`-split half (other) | 11597.17 | 0.08285 | 94.6 | **wrong** |
| `face[14]`-split half | 5275.33 | 0.99585 | 1.1 | correct |
| `face[14]`-split half (other) | 28420.01 | 0.99725 | 1.1 | correct |

3 of 5 pieces translate correctly; 2 do not, both involving the R=37
RoundCorner cylinder as a boundary face (the worse of the two also pulls
in 2 real `K/Y` cone surfaces from the model's own R40->R37 transition
faces). **Not yet root-caused** — working hypothesis, unconfirmed:
GEOUNED's automatic Can/RoundCorner/TCone face classifier
(`conversion/cell_definition.py`) expects a cylinder-to-bounding-plane
relationship to be a genuine tangency throughout (as every *real*
RoundCorner/Can feature is); this hand-built cut boundary has the R37
cylinder tangent to `p1`/`p2` but genuinely cut through (not tangent) by
`pc` — a combination the classifier may not have been designed for,
possibly leading it to pick the wrong AND/OR combination or orientation
when reconstructing these 2 pieces' own CSG definitions from their real
faces. Fixture files saved for continuing this directly (each is a
single already-decomposed solid — testing needs only a single-piece
`meta_list`, skip `decompose_solids()`, no need to redo the whole manual
chain):
`Solidos/convierte_bad_volume/SCDR_90_piece1_roundcorner_badvolume.stp`
and `.../SCDR_90_piece3_boolean_neg0_badvolume.stp`.

Diagnostic scripts (scratchpad only): `diag_placa3_topology.py` (face/
edge/vertex counts, multi-wire faces, non-manifold-edge detection via
`GSolid.faces_sharing_edge`-equivalent hashing, tolerance ranges),
`diag_placa3_surface_params.py` (per-segment RCC axis/center/apex/radius/
semiangle dump and pairwise diff, original vs. round-tripped),
`make_placa3_roundtrip.py` (the decisive zero-transform STEP round-trip
test), `rerun_placa3_original.sh` (forced clean rerun proving the
original was never actually broken).

### `PiezaDavid.stp`'s real volume failure: `is_closed_cylinder_cone` accepted a face that wasn't actually a closed 360° cylinder

Follow-up to the `PiezaDavid` deep-dive in the previous session (traced to
`build_can_params`'s Cylinder-branch plane-fitting on a non-planar BSpline
boundary, left unresolved -- see the paused investigation above). Picking
that back up surfaced a *different*, more fundamental bug one level
upstream: `get_can_surfaces` was identifying a Can on a cylinder face
(radius 15, axis Z, piece 0 of the decomposed solid) that visual
inspection confirmed **isn't a closed cylinder at all** -- "para que haya
una superficie can tiene que haber un cilindro cerrado (360º) y aqui no
hay" (user's own diagnosis, confirmed correct).

**Root cause**: `is_closed_cylinder_cone` (`meta_surfaces_utils.py`), for
the single-face (non-`ShellGu`) case, only checked whether the face's raw
UV bounding box (`face.ParameterRange`) spans a full `2*pi` in U. This is
necessary but not sufficient: a face whose boundary is a genuine, clean
closed loop always satisfies it, but so does a face whose boundary is
*jagged and irregular* over part of its circumference, as long as that
irregular part's U-projection still happens to span the missing angular
range. Confirmed exactly this for the offending face: its wire has a
clean half-circle at one Z level (2 `GCircle` arcs, U 90°→270°) closed by
2 seam-like `GLine` edges, but the *other* half (U 270°→90°, wrapping
through 0°) is not a matching circle at the same height -- it's 2
irregular `GBSpline` edges that drop to a completely different Z, left
over from a bad prior cut. The face's overall `ParameterRange` still
spans the full `2*pi` (each half contributes non-overlapping U), so the
old check passed it as "closed" even though it plainly isn't one.

**Fix, found after two rejected simpler ideas** (both discussed with and
rejected by the user first, per this project's established discipline
around `can_region`/`get_can_surfaces`-adjacent shared logic):
- *Rejected idea 1*: require the closing surface at each end to be a
  `GPlane`. Wrong -- a Can's end can legitimately be closed by a sphere,
  cylinder, or cone, not just a plane; only the *whole* end needs to be a
  single coherent surface.
- *Rejected idea 2*: require the shared boundary edges to be planar
  (`planar_edges`). Wrong -- a cylinder closed by a *perpendicular*
  cylinder of a different radius produces a genuinely non-planar (but
  still legitimate) intersection curve.
- *The actual fix*: replace the UV-bounding-box check with a real
  topological invariant, suggested by the user directly -- walk the
  face's outer wire in traversal order, and track the *unwrapped*
  cumulative angle swept around the cylinder's own axis (derived from
  `tangent x axis`'s radial component, `-r*(dtheta/ds)` -- see the
  derivation this was built from). Two refinements were needed before
  this was safe, both found by testing against real cases beyond the
  original bug, not just trusting the math:
  - *First attempt (monotonic-only)*: require the angle to never reverse
    sense while walking the wire. Wrong -- a completely normal closed
    cylinder capped at both ends (e.g. `ConeSphere.stp`, `drillsphere.stp`
    in the wider `Solidos/` corpus) legitimately reverses sense between
    its top and bottom rims, the same way a washer's outer and inner
    boundary wind oppositely in Green's theorem. Rejecting any reversal
    at all produced false negatives on real, valid Cans.
  - *Second attempt (each run must close before reversing, non-cyclic)*:
    partition the wire into maximal same-sense runs and require each to
    independently sum to a clean multiple of `2*pi` before the next
    reversal. Still wrong on one further real case (`tank.stp`, in the
    corpus above): a rim's `2*pi` sweep can be split across the *start*
    and *end* of the edge list (e.g. a rim represented as 2 arcs that
    happen to be the wire's first and last edges, with the opposite rim's
    edges in between) -- since the wire is a cycle, this is one
    continuous run, not two incomplete ones, but naive linear indexing
    treats them as separate and wrongly rejects. Fixed by rotating the
    sequence of edge sweeps to start right after a genuine direction
    change before partitioning into runs, so no run ever needs to be
    merged across the artificial start/end boundary.
  - The final, verified algorithm: `_is_closed_by_winding`
    (`meta_surfaces_utils.py`) samples each wire edge, computes its
    oriented angular sweep, and requires every maximal same-sense run
    (cyclically partitioned) to close to a clean multiple of `2*pi`. A
    single edge that's already a closed loop on its own (start==end
    vertex) is fast-pathed, but *only* after confirming its own sweep
    also closes cleanly -- an earlier version trusted vertex coincidence
    alone and was fooled by a genuinely degenerate face in
    `Cans/rev_can_1.stp` (a `GEllipse` edge that happens to close on
    itself, sitting in a wire that *also* contains an obviously-garbage
    `GBSpline` edge of length ~282030 with coordinates in the hundreds of
    thousands -- unrelated to this fix, and plausibly the actual cause of
    the long-unresolved `fwd_can_1.stp`/`rev_can_1.stp` "`makeCan` raises
    `TypeError`" bug noted in an earlier session).
  - **Extended to `GCone` and `ShellGu`** in the same session, per
    explicit user request ("hay que extenderlo a conos, y sobre todo a
    los shell"). Cone support was direct (`GCone` has no `XDir` the way
    `GCylinder` does, but the winding math never needed an absolute
    reference direction to begin with -- only *changes* in angle are
    ever measured -- so an arbitrary stable perpendicular to the axis,
    `_perpendicular_axis`, works for any surface type; `GCylinder` still
    prefers its own real `XDir` when it has one, purely to stay byte-for-
    byte on the exact frame this was validated against). `ShellGu`
    (several faces merged because they're the same analytic surface, e.g.
    a cylinder split into pieces by an earlier cut) generalizes the same
    invariant from "circulation around one face's wire" to "circulation
    around the union's outer boundary": `_boundary_edges_of_merged_faces`
    collects every edge from every merged face's own wires and keeps only
    the ones occurring an odd number of times (an edge shared between two
    of the merged faces, an internal seam where they join, occurs twice
    and is excluded); `_assemble_boundary_loops` chains the surviving
    edges into ordered closed loops by shared endpoints (there is no
    single native "wire" for a merged, non-contiguous boundary); the same
    `_loop_closes_full_turn` check then runs on each assembled loop,
    unchanged.
  - **A real granularity bug found while re-validating after the Cone/
    Shell extension**: sweeping the whole 60-file `Solidos/` corpus after
    the extension showed 6 files regressing (`FwdRevCan.stp`,
    `rev_can_1.stp`, `placa.stp`, `placathin.stp`, `rc20.stp`,
    `drillsphere.stp`) -- but isolating Cone vs. Shell (toggling each
    independently via monkeypatch) showed the *same* regression persisted
    even with both disabled, proving neither was the actual cause. Root
    cause, found by diffing against the original standalone validation
    script line by line: that script sampled and computed direction
    *within* each edge at fine granularity (many points per edge, deltas
    between consecutive sample points) before partitioning into
    same-sense runs; the ported source version had been simplified to
    compute one net sweep *per edge* first, then partition runs between
    edges -- losing sensitivity to a direction reversal that happens
    *partway through* a single curved (BSpline) edge rather than exactly
    at an edge boundary. Fixed by adding `_loop_sample_points` (flattens
    a loop's edges into one ordered list of individual sample points) and
    running the run-partition logic on that flat list, matching the
    validated script's granularity exactly. **Lesson**: a standalone
    validation script and its "ported" source version are only actually
    equivalent if verified to be -- re-run the full corpus regression
    after every refactor of already-validated logic, not just after
    genuinely new logic; a "faithful" port can silently change behavior
    through a granularity/aggregation-order difference that looks
    equivalent on paper.

**Verification**: tested against 4 hand-picked real cases (the broken
`PiezaDavid` face; `drillsphere.stp`'s sphere-closed cylinder; `ConeSphere.
stp`'s opposite-sense double rim; `rev_can_1.stp`'s degenerate face) at
each iteration, then a full differential regression across all 60 files
of the `Solidos/` corpus (Can/TCone/RoundCorner/MultiRoundCorner/
MultiPlane counts, baseline vs. patched, via `git stash`) after every
candidate change -- caught several wrong "improvements" before they were
ever considered final (see the granularity bug above, and the
`outer2_only` investigation below). Final state (`_is_closed_by_winding`
alone, before the `outer2_only` fix below): 0 diffs across the whole
corpus, `PiezaDavid.stp`'s false `RevCan` gone (`RoundC` unchanged at 4),
and the full `tests/geo` + `tests/test_cadtocsg.py` suite green (156/156).

### The washer-plane gap in `get_can_surfaces`: investigated, reverted twice, fixed on the third attempt

A separate, pre-existing issue surfaced while chasing the above: with the
winding fix in place, `tank.stp` (`testing/inputSTEP/Torus/`) crashed in
`isSameInterface` ("same Boolean Surface defined with oposition name").
Traced to `get_can_surfaces` silently rejecting a legitimate concentric
cylinder (radius 325) closed by *washer-shaped* (annular, 2-wire) planes,
because `commonEdge`'s `outer2_only=True` only searches the closing
plane's *outer* wire, missing the shared boundary when it's actually the
plane's *inner* wire (the hole). This specific crash turned out to be a
red herring -- fixing the cyclic-partition bug (above) independently made
it disappear -- but the underlying washer-plane gap was real and affected
more than just `tank.stp`.

**First attempt**: relax `outer2_only` to `False` unconditionally in
`get_can_surfaces`'s two `commonEdge` calls. Broke `drillsphere.stp`
(a previously-correct `RevCan`, lost). Reverted.

**Second attempt**: relax `outer2_only` only as a *fallback*, when the
strict (outer-wire-only) search finds nothing at all -- reasoning that
this narrows the change to exactly the previously-empty-result cases and
should leave every already-working case untouched. **Still broke
`drillsphere.stp`, identically.** This showed the mental model was wrong:
`get_Can` is called during *decomposition* itself, not just at
conversion-phase classification, and since `generic_split` stops at the
first candidate surface that actually splits the solid, *any* change to
which surfaces `get_Can` accepts -- even one that looks purely additive
-- can steer the whole decomposition down a different path for an
unrelated face elsewhere in the same solid. A local-looking, seemingly
safe fallback is not actually local. Reverted again.

**Third attempt, this time correct**: full 60-file differential
regression (Can/TCone/RoundCorner/MultiRoundCorner/MultiPlane counts) run
against *both* choices side by side -- `outer2_only=True` (strict) vs.
`outer2_only=False` (unconditional) -- rather than reasoning about it in
the abstract. Result: strict breaks 5 files (`FwdRevCan.stp`,
`rev_can_1.stp`, `placa.stp`, `placathin.stp`, `rc20.stp` -- all losing
real Cans/RoundCorners to the same washer-plane gap `tank.stp` had) while
relaxed breaks only 1 (`drillsphere.stp`). Net, relaxed is strictly
better across the corpus, and critically: **zero `MultiPlane` differences
either way** -- the exact regression class the user had warned about from
a past attempt at this same kind of change ("cuando lo cambie, despues
fueron los multiplane que no funcionaban como queria") did not recur here.
Applied `outer2_only=False` unconditionally at both `commonEdge` call
sites in `get_can_surfaces`. Final verified state: full `tests/geo` +
`tests/test_cadtocsg.py` suite green (156/156), and only 1 file
(`drillsphere.stp`) differs from the pre-session baseline across the full
60-file `Solidos/` corpus.

`drillsphere.stp`'s regression (one `RevCan` lost) is a known, accepted
trade-off, not yet root-caused to the same depth as everything else in
this section -- confirmed to be a genuine change (not a fluke: reproduced
identically across every algorithm variant tried), but *why* accepting
the washer-plane's inner-wire boundary elsewhere in the same solid steers
`generic_split` away from the surface that used to produce this Can was
not traced step by step the way the `PiezaDavid`/`get_can_surfaces`
investigation earlier in this file was. Whether it's a real geometric
regression or just a different, equally-valid decomposition (the kind of
"expected fallout, not a regression" outcome documented for the
`MultiPlane` fix earlier in this file, where several files' `RoundC`/
`RevCan` counts shifted but still validated 300/300 against independent
CAD ground truth) has not been checked via `check_sign`-style
verification -- flagged as the natural next step if this file's specific
behavior ever needs to be trusted further.

**Any future change in this area must re-run the full 60-file `Solidos/`
differential regression (Can/TCone/RoundCorner/MultiRoundCorner *and*
MultiPlane counts, all of them, compared both ways, not reasoned about
abstractly) before trusting any local-looking fix** -- a change that
looks correct, or even looks like a strict narrowing, for one target face
can silently steer decomposition elsewhere in ways no single-file check
will catch, and intuition about which direction (stricter vs. looser) is
"safer" has now been wrong twice in a row in this exact function.

### `drillsphere.stp`'s `outer2_only=False` regression: confirmed harmless via the MCNP volume check

Closes the "not yet root-caused" open question from the section above.
Ran `drillsphere.stp` through the full GEOUNED -> MCNP -> d1suned
stochastic volume check (the same `volSDEF=True` technique documented
under "MCNP stochastic volume check" earlier in this file): tally =
1.00055 +/- 0.20% (0.3 sigma from 1.0), SD4 matches the true CAD volume
exactly, zero lost particles. Confirms directly what was only inferred
before: losing this file's `RevCan` classification under
`outer2_only=False` does not corrupt the actual decomposed geometry --
the solid still gets built correctly through a different path, exactly
the kind of "different, equally-valid decomposition" outcome already
seen for the `MultiPlane` fix. No further action needed on this file.

### `GSolid.refine()` silently corrupting solids with an internal cavity -- `removeSplitter()` mutates its own receiver as a side effect

Found while building a small reusable test-fixture series (3 solids +
their complements, saved under `Solidos/trier/series_solid*`, per
explicit user request to keep them as regression fixtures): the
`solid2` complement (`universe - (half-cylinder fused with an inclined
cylinder stub)` -- a solid with a genuine internal cavity, the hole left
by the subtracted stub) reliably gave a d1suned volume tally of 0.981
(9.4 sigma off), and its `SD4` reference volume (the decomposed-piece
sum) didn't even match the true CAD volume (65075.3460 vs 62924.6537 mm^3).

Traced to `GeounedSolid.__init__`'s `comsolid.refine().Solids` call
(`geouned_classes.py:69`) -- `GSolid.refine()`
(`geo/_freecad_impl.py`) wraps native `Part.Shape.removeSplitter()`,
documented (both in GEOUNED and generally) as a purely cosmetic
simplification that never changes the enclosed volume. For this
cavity topology, that's false: `removeSplitter()`'s own return value has
volume 65075.3460 instead of the correct 62924.6537 -- a real ~3.4%
corruption, not float noise (confirmed deterministic across repeated
calls). The 3 cylindrical faces bounding the cavity also flip from the
correct `Reversed` to `Forward` as a direct consequence of this
corrupted topology -- not a separate orientation-reading bug, as first
suspected when the user asked why these faces read `Forward`.

**A second, nastier layer under this**: `removeSplitter()` also mutates
the shape it's called *on* as a side effect, not just the shape it
returns. Confirmed directly: `native.Volume` read immediately before and
after calling `native.removeSplitter()` differs (62924.6537 ->
62173.6484, a *third*, still-wrong value), even though the returned
object is provably distinct (`native is refined` -> `False`). This
invalidated the first attempted fix (compare `native.removeSplitter().Volume`
against `native.Volume`, fall back to `native` if they differ) -- by the
time the fallback path reads `native.Volume` again, `native` has already
been silently corrupted by the very `removeSplitter()` call being used
to decide whether to trust it.

**Fix**: `GSolid.refine()` now calls `removeSplitter()` on `native.copy()`,
never on `self.__native__` directly, so the pristine original is never at
risk regardless of which branch is taken; falls back to the untouched
original whenever the refined volume differs by more than float noise
(`1e-6` relative). Verified: `solid2_complement` (with the user's
independently-corrected CAD, saved over `Solidos/trier/series_solid2_complement.stp`)
now gives a d1suned tally of 0.996 (1.7 sigma), `SD4` matches true volume
exactly, and the 3 cavity faces correctly read `Reversed`. Full
`tests/geo` + `tests/test_cadtocsg.py` suite green after the fix (this
was checked in isolation, before the unrelated Can/TCone regressions
below existed).

### Can/TCone secondary-surface orientation rework -- in progress, 2 known regressions, committed as WIP

User-driven change (via debugger, iterated live in this session) to
`get_can_surfaces` (`meta_surfaces.py`) and `build_can_params`
(`functions.py`): `region_sign`'s AND/OR result is now normalized
against the main cylinder/cone's own orientation (`Forward` always paired
with `AND`, `Reversed` always paired with `OR` -- flipping `r` and
tracking whether a flip happened via a new `omit` flag threaded through
as a 3rd tuple element, `(s, r, omit)`, replacing the old `(s, r)` pairs).
`build_can_params` mirrors this: when `omit` is `False`, the secondary
surface's own plane normal (or, for Cylinder/Cone/Sphere secondaries, its
effective `Orientation`) gets flipped too, compensating so the physical
region stays the same while the *representation* becomes uniform.

Motivated directly by the `solid2`/`solid2_complement` investigation in
this session (`get_can_surfaces` picking a distant, unrelated real face
-- the half-cylinder's own symmetry-cut plane -- as if it were a local
Can end cap, then combining it via a plain `OR` that engulfed the whole
model) -- though that specific bug's root cause turned out to be
`refine()`'s corruption above, not this. This orientation-normalization
change is a separate, broader change to the same area, not yet fully
verified.

**2 known regressions, not yet fixed**: `tank.stp`
(`testing/inputSTEP/Torus/tank.stp`, now also copied to the new
`Solidos/no_convierte/` -- see below) and `Solidos/Cans/rev_can_1.stp`
both now crash with `RuntimeError: same Boolean Surface defined with
oposition name` (`isSameInterface`, `boolean_function.py:273`) during
conversion -- `add_forwardCan`/`add_cone` respectively. A separate,
already-identified-but-reverted issue in the same area (`get_tcone_surfaces`
producing the new 3-tuple while `build_tcone_params` still expected the
old 2-tuple, `ValueError: too many values to unpack`) was fixed and then
explicitly reverted at the user's request ("el erratum del tcone no es
la fuente del fallo") to keep investigating the real regression instead
-- that unpacking fix is *not* in this commit, `build_tcone_params` is
currently broken again the same way if a TCone's `get_tcone_surfaces`
path is ever exercised.

Committed as-is at explicit user instruction ("has el commit. veremos
los bugs despues") -- **this is a known-broken checkpoint, not a
verified fix**. Next session should resume by root-causing the
`isSameInterface` opposition-name error on `tank.stp`/`rev_can_1.stp`
before touching anything else in this area.

### Full-corpus MCNP volume re-check after this session's fixes, and 3 new `Solidos/` triage folders

Re-ran the full GEOUNED -> MCNP -> d1suned stochastic volume check (per
"MCNP stochastic volume check" above) against the *current* code state
(refine() fix in place; the in-progress, still-broken Can/TCone rework
also in place) across `testing/inputSTEP` + `Solidos/` (150 STEP files
found, 147 converted successfully). Result: 335 solid-cell tallies,
94.3% within 2 sigma (up from 92.3-92.7% in the prior full-corpus runs
documented earlier in this file), 4.5% marginal (2-3 sigma, consistent
with expected statistical noise at this sample size), 1.2% (4 cells)
real failures beyond 3 sigma:
- `repo_SCDR_90` (38.1 sigma) -- known, pre-existing, already marked
  `# fails (pre-existing)` in `tests/test_cadtocsg.py`.
- `repo_Torus_solid1` (3.84 sigma) -- known, already documented above as
  borderline-but-real.
- `solidos_PiezaDavid_pieces_piece_0` (9.8 sigma) -- **new**, not
  previously seen in any corpus scan this project has run. Not yet
  investigated.
- `claude_solid2_complement` (9.6 sigma) -- a stale leftover run
  directory from this session's own debugging, using the CAD *before*
  the user's fix; not a real corpus finding (the corrected file, run
  under its proper name `claude_series_solid2_complement`, gives 0.996 /
  1.7 sigma and does not appear in the failing list).

`solidos_Torus_2_degen_torii` (no tally section at all) and
`solidos_Enclosures_w_encl` (10 lost particles) remain open exactly as
before -- unchanged by this session's fixes, not investigated.

3 of the 150 files failed to convert at all (crashed, not a volume
issue): `repo_Torus_tank`, `solidos_Cans_rev_can_1` (both regressions
from the in-progress Can/TCone rework above) and
`solidos_BadCADModel_series_solid2_complement` (the deliberately-archived
bad CAD, expected to fail).

**New corpus-organization convention, per explicit user instruction**:
`Solidos/` now has 3 new triage subfolders, populated as this kind of
investigation finds new cases (going forward, save any newly-identified
case into the matching folder rather than leaving it loose):
- `Solidos/BadCADModel/` -- STEP files with a genuine CAD-level
  structural defect (not a GEOUNED bug) -- e.g. the original,
  user-corrected `series_solid2_complement.stp` cavity-topology case
  above.
- `Solidos/no_convierte/` -- STEP files where GEOUNED itself crashes
  during conversion. Currently: `tank.stp`, `series_solid2_complement.stp`
  (the archived bad-CAD copy), `rev_can_1.stp`.
- `Solidos/convierte_bad_volume/` -- STEP files that convert without
  crashing but whose d1suned volume tally comes back wrong (>3 sigma).
  Currently: `SCDR_90.stp`, `PiezaDavid_pieces_piece_0.stp`,
  `Torus_solid1.stp` (the 3 real, non-stale failures from the corpus
  re-check above).

### Can/cone sign fixes, following a new user-written spec (`configuracion_cans.txt`)

The user wrote a complete, from-scratch specification of the intended
Can AND/OR boolean rules (`C:\Users\Patrick\Work\Taller\GEOUNED_workshop\configuracion_cans.txt`)
-- a 2x2 grid of (main cylinder orientation) x (secondary surface `si`
AND/OR configuration): Fwd+AND and Rev+OR are the two "normal" matched
pairings (Sections 1/2 of the doc); Rev+AND is a real but special "open
mouth" case (Section 3, `si`'s own natural extension substitutes for a
missing closing surface, via a formula built from the *opposite*
orientation's normal formula); Fwd+OR is not a valid Can at all (Section
4, must reject and let `si` be tried as an ordinary simple cutting
surface instead). Comparing this spec line-by-line against the actual
code (`can_region()`, `get_can_surfaces()`, `build_can_params()`,
`MetaSurfacesDict.add_cone()`, `cone_apex_plane()`) surfaced 4 real,
independent bugs, all confirmed via real geometry (point-sampling against
`isInside()`, not just reasoning) before being applied:

1. **`can_region()`'s coaxial-cone-without-plane branch, Reversed case**
   used `+apid` (`OR[s, ap]`); the user confirmed the canonical rule for
   a cone + its own apex plane is *always* `AND[-s, ap]` or `OR[s, -ap]`
   -- never the other two sign combinations -- so this was a plain sign
   error, fixed to `-apid`.
2. **`get_can_surfaces()`'s Fwd+OR case** used to be silently force-
   normalized into the Rev+AND "open mouth" shape (flipping `r` and the
   secondary's reported orientation) instead of being rejected -- i.e.
   the code treated *both* mismatched pairings the same way, but per the
   spec only Rev+AND has a valid continuity formula; Fwd+OR must return
   `None, None` (this is almost certainly what silently built a
   nonsensical Can region and caused the `isSameInterface` "same Boolean
   Surface defined with oposition name" crashes on `tank.stp`/
   `Solidos/Cans/rev_can_1.stp` from the previous session's WIP
   checkpoint -- confirmed fixed for `rev_can_1.stp` specifically,
   `tank.stp` turned out to be a separate, unrelated torus issue, see
   below).
3. **`cone_apex_plane()`** used to flip its returned plane's normal based
   on `cone.Orientation` (`cone.Surface.Axis if Forward else
   -cone.Surface.Axis`) -- but the canonical convention (point 1 above)
   is unconditional: the apex plane's stored normal must always just be
   `cone.Surface.Axis`, full stop, with *all* orientation-dependent sign
   handling living in the formulas that consume it (`can_region()`,
   `add_cone()`). This function was double-applying the Reversed-case
   compensation on top of what the formulas already do, corrupting only
   the Reversed-cone case (verified: on `Solidos/Cans/rev_can_1.stp`'s
   two-cone Can, cone1 (Forward) had the two apex-plane sign conventions
   coincidentally agree, cone2 (Reversed) did not -- a 1.2% material
   "leak" between the two cones' regions, found and root-caused via 1000-
   point sampling against `isInside()`, precisely localized to the zone
   between the two cone apexes). Fixed to unconditionally return
   `cone.Surface.Axis`. Verified 1000/1000 after the fix (was 988/1000).
4. **`MetaSurfacesDict.add_cone()`'s Reversed branch** (the Tier-2
   standalone-cone conversion path, independent of `can_region()`) had
   its own copy of this same bug, structurally: `cone_region + pid`
   (missing the negation) instead of `cone_region + (-pid)`, discovered
   because fixing point 3 alone made this path's own Reversed-cone
   registration disagree with the Can's registration of the *same*
   physical cone (an `isSameInterface` opposition-name crash, since
   `rev_can_1.stp`'s Can literally reuses one of its own cone components
   independently elsewhere in the same solid). Fixed to match the same
   canonical convention.

All 4 fixes verified together: `tests/geo` + `tests/test_cadtocsg.py`
green (155/156, only the already-known-separate `tank.stp` failing).

**Two riskier, related ideas were tried and explicitly reverted** at the
user's direction, kept here as a record so they aren't retried blindly:
building the Can's CAD cutting surface (`get_cell_object`'s Can branch)
from the *complementary* region instead of the direct one (motivated by
`rev_can_1.stp`'s box-splitting construction leaking material outside
its intended box at most box sizes) and a companion change in
`generic_split` (`decom_one_generators.py`) to accept a single-piece
`Gsplit` result as a real, useful split when its volume measurably
differs from the input (distinguishing "carved an internal cavity" from
"tool didn't touch the solid"). Together these *did* make `rev_can_1.stp`
decompose into a clean 1-piece result with exact volume -- but chasing a
resulting regression (`DoubleCylinder/pieza.stp`, a Can that
`build_can_params` couldn't unpack: `get_can_surfaces` found only 1
secondary end instead of 2) traced back to an intermediate decomposition
piece where the *same* face is legitimately adjacent to *both* of a
cylinder's rim edges (`get_adjacent_cylknesurfFace`'s dedup-by-face-index
then collapses 2 real ends into 1) -- exported as
`Solidos/.../pieza_intermediate_piece.stp` (also sent to the user
directly) for inspection, and the user identified "a big problem" with
that piece on sight. Given that, both changes were reverted rather than
risk building on a piece that's suspect for reasons not yet understood --
this is *not* closed out, just deliberately shelved. Whether `rev_can_1.stp`
still needs one of these approaches, once the `pieza_intermediate_piece.stp`
concern is understood, is open for a future session.

### `SplitBase.base`: `Part.Solid` -> `GSolid`

Continuing the `.__native__`-audit theme from earlier in this file:
`build_region/splitFunction.py`'s `SplitBase.base` (the CAD shape a
cell-construction fragment carries through `BuildDepth`/`SplitSolid`/
`filterparts`/`joinBase`) used to be a raw native `Part.Solid`, with
`GSolid(...)`/`.__native__` wrap/unwrap dances scattered at nearly every
call site (`Gsplit(GSolid(base.base), ...)`, `GSolid(c).find_interior_point()`,
`FuseSolid`'s own internal `[GSolid(p) for p in parts]`, etc.). Converted
to carry `GSolid` throughout instead:
- `Objects.py::CellObj.makeBox()` returns `GSolid` directly (drops its
  own `.__native__`) -- its only caller (`BuildSolidParts`) seeds the
  very first `SplitBase` from it.
- `splitFunction.py::SplitSolid` no longer wraps/unwraps around `Gsplit`
  (`base.base` is already `GSolid`; `result.solids` is already
  `list[GSolid]`, no `[s.__native__ for s in ...]` conversion needed).
- `space_decomposition` operates on `list[GSolid]` directly -- including
  `c = c.reverse()` instead of the old native in-place `c.reverse()`,
  since `GSolid.reverse()` is copy-based, not in-place (a real semantic
  difference that had to be handled by reassignment, not just a
  mechanical type swap).
- `FuseSolid` takes and returns `GSolid` (see below for what else changed
  in it); `joinBase` and `build_shape_functions.py::build_complex_shape`
  (its only two callers) both already had `GSolid`-typed parts to feed it
  once `SplitBase.base` itself was converted.
- `build_complex_shape` -- the one place this migration meets code that
  *must* stay native (every downstream consumer of `makeCan`/`makeTCone`/
  etc: `GeounedSurface.shape`/`.shell`, `.exportStep()`, `Gsplit(...)`
  callers elsewhere in `GEOUNED`) -- now uses `gsolid.BoundBox`/
  `gsolid.Volume` (already `GBoundBox`/float, no conversion) for its own
  "did the surface actually cut the box" check, and converts to native
  (`gsolid.__native__`) only once, at the very end, right before
  returning. `myBox`/`to_gboundbox` needed no changes at all -- both
  already duck-type-accept a `GBoundBox` exactly like a native
  `FreeCAD.BoundBox` (same attribute names), confirmed rather than
  assumed.

Verified via `tests/geo` + `tests/test_cadtocsg.py`, 155/156 (same
`tank.stp` failure as before, unrelated).

### `FuseSolid` wasn't trying `ShapeFix_Shape` (`GSolid.fix()`) before giving up on a real fused solid

Found while the user was independently inspecting `build_complex_shape`
after the `SplitBase.base` migration above, having just manually fused
the same components in the FreeCAD GUI and gotten a real, valid fused
solid where GEOUNED's own pipeline was silently falling back to an
unfused `Gmake_compound` (a container of the original, possibly-
overlapping parts -- not a true union, and part of why `rev_can_1.stp`'s
decomposed-piece volumes didn't sum cleanly in earlier investigation
this session). Traced with a monkeypatched, instrumented `FuseSolid`
across `Solidos/RoundCorners/*.stp` + `Solidos/Cans/*.stp`: 3 real
compound-fallback occurrences, all on `rev_can_1.stp`, all with
`Gfuse()` raising nothing and `.refine()` raising nothing, but *both*
the raw fused shape and its `.refine()`'d version failing
`BRepCheck_Analyzer` (`.is_valid()` -> `False`) -- `removeSplitter()`
alone doesn't always repair a genuinely invalid boolean-fuse result.
Confirmed directly (captured the actual 4 real, individually-valid input
parts and replayed the fuse standalone) that `GSolid.fix()`
(`ShapeFix_Shape`, already existed in `geo` for a different purpose, just
never tried here) *does* repair it, at every tolerance tried, without
changing the volume at all. `FuseSolid` now tries `.fix(1e-6)` as a third
repair step (after `.refine()`, before giving up to `Gmake_compound`).
Confirmed this alone -- with none of the reverted complement-region/
`generic_split` changes from the section above -- already makes
`rev_can_1.stp` decompose into a clean 1-piece result with exact volume
(diff 0.0000 from the true CAD volume). Verified via `tests/geo` +
`tests/test_cadtocsg.py`, 155/156 (same `tank.stp` failure, unrelated).

Also found, alongside this (not yet fully written up, a companion fix
present in the working tree): `Gfuse()` itself now flattens a `Part.Compound`
input (one of `solids` being a compound rather than a single solid,
which can happen when `Gmake_compound` was itself the fallback for an
*earlier* unfusable group) into its constituent `Solids` before calling
native `.fuse()`, rather than passing the compound through as one opaque
shape.

### `tank.stp` is a separate, torus-related failure -- not a Can issue

Explicit user clarification after the Can rejection fix (Section 4
above) didn't make `tank.stp` pass: "el error debe venir de otra parte
puesto que mezcla toro y cilindros" -- confirmed this file mixes Torus
and Cylinder surfaces, and the `isSameInterface` crash on it is
unrelated to any of the Can fixes in this session. Not yet investigated;
`Solidos/Cans/rev_can_1.stp` (pure Can, no torus) was used as the
reproduction case for all 4 sign fixes above instead, specifically
*because* it doesn't have this confound.

### Closed: `pieza_intermediate_piece.stp` was a symptom of the reverted complement/generic_split changes, not a real bug

Resolved, not open after all. The `DoubleCylinder/pieza.stp` regression
above (`get_can_surfaces` finding only 1 secondary end instead of 2,
crashing `build_can_params`'s `cyl_in, sr1, sr2 = cs` unpack) only
happened while the (since-reverted) complement-region + `generic_split`
single-piece-real-change changes were active. Per the user directly:
the exported repro piece (`pieza_intermediate_piece.stp`) was itself an
*invalid* solid, produced only because those two changes together drove
decomposition into an incoherent state -- not a real, valid piece
exposing a genuine gap in `get_can_surfaces`. Confirmed: with those two
changes reverted (current committed state, `FuseSolid.fix()` fix in
their place instead), `DoubleCylinder/pieza.stp` runs cleanly end to end
(`geo.run()`, no crash) -- matches the 155/156 suite result, which
already included this file passing. No guard needed in
`get_can_surfaces()`; the exported repro file can be discarded.

### `tank.stp` root-caused and fixed: `isSameInterface`'s `.reverse`-agreement assertion was a false-positive guard, not a real invariant

Followed up on the "separate, torus-related failure" note above by
isolating `tank.stp`'s actual failing component via bisection over its
12 separate top-level solids (`skip_solids`, halving the set each time):
solids `[0-5]` and `[6-11]` each convert fine as a group; only `[6,7]`
together reproduce the crash; every one of the 12 solids converts fine
*individually*. So the failure is a genuine cross-solid interaction, not
a defect in any single component (and not actually torus-related after
all -- the torus mention in the earlier note was a red herring from
looking at the wrong part of the file).

Dumping both solids' faces showed why: solid[6] (a hollow tube) and
solid[7] (a solid cylinder it fits around) are two separate, physically
*adjacent* cells sharing the exact same cylindrical interface (radius
325, same axis/center) -- solid[6]'s material is outside that cylinder,
solid[7]'s is inside, both bounded by the *same* two axial planes. Both
independently get classified as a Can (cylinder + 2 end planes). Traced
into `MetaSurfacesDict.Can_region`/`add_forwardCan` and confirmed via
instrumentation: both Cans' own regions are individually correct and
self-consistent with the canonical characteristic-surface sign rule
(established earlier this file: a Forward main surface's id is negative
in its own region, Reversed is positive) --
```
Can_region(solid[6]): Orientation=Reversed cid=4  region=OR[4,-3,-2]   -> sign of 4 = +1  (consistent)
Can_region(solid[7]): Orientation=Forward  cid=4  region=AND[2,3,-4]   -> sign of 4 = -1  (consistent)
```
`OR[4,-3,-2]` and `AND[2,3,-4]` are exact structural complements of each
other (as they should be -- the two cells' Cans really are the same
physical cylindrical interface viewed from opposite sides) -- but both
have `.reverse=False` (neither was built via negation), which is exactly
the case `isSameInterface` treats as a contradiction and raises on:
```python
elif self.region == region2.region.get_complementary():
    if self.reverse == region2.reverse:
        raise RuntimeError(f"same Boolean Surface defined with oposition name : BooleanSurface {region2.__int__()}")
```

**Root cause, confirmed by design discussion with the user**: `.reverse`
is per-object *construction-history* bookkeeping (was this particular
`BoolSurface` built by negating a label at some point) -- it is not, and
was never meant to be, a property of the *relationship* between two
independently-built regions. Two genuinely different, physically
adjacent cells sharing a boundary with naturally opposite outward sense
will legitimately produce two independently-constructed, structurally
complementary regions that both happen to have `.reverse=False`, since
neither was built via negation. `isSameInterface`'s `.reverse`-agreement
check was using this bookkeeping as an invariant it never actually was.

**Design options considered and rejected before landing on the fix** (all
per direct exchange with the user): dropping the `.reverse` check
entirely -- rejected, the user didn't want to lose the real bug-catching
value it has in the common case; tracking each region's originating
solid to disambiguate -- rejected, since a solid that gets cut by a
later surface loses its connection to the original solid entirely, so
there's nothing stable to track; tagging `BoolSurface`/`BoolRegion`
objects with a "type" (Can/RoundCorner/...) -- rejected as unnecessary
once traced through: the type information the check would need
(`GeounedSurface.Type`, the characteristic surface, its real
`Orientation`) already exists one level up, on the `GeounedSurface` that
owns the region, at exactly the point each region gets finalized
(`Can_region`) -- no new type needs to live on the region itself, and
intermediate regions built while chaining `*`/`+` to construct a final
composite never need typing either, since nothing ever validates them
independently.

**Fix ("opcion A", the user's explicit choice)**: `isSameInterface`
doesn't decide anything itself anymore for the ambiguous case -- it gets
an `on_conflict="raise"` (default, preserves every existing call site's
behavior byte-for-byte) vs `"ignore"` parameter; a new free function,
`validate_characteristic_sign(region, surf_id, orientation, label)`
(`geouned_classes.py`, using a new `literal_sign(region, surf_id)`
utility in `boolean_function.py` that reuses
`BoolSequence.get_surfaces_numbers(negatives=True)`'s existing
signed-literal recursion rather than re-walking the tree), is called
once, right where `Can_region` finalizes a region -- if a region ever
fails its own canonical sign rule at construction time, that's a real
bug and raises immediately, before the region is ever compared against
anything else. Because every region reaching `add_forwardCan`/
`add_reverseCan`'s dedup loop -- both the newly-built one and every
already-registered one in `self["FwdCan"]`/`self["RevCan"]` -- has
already passed this check by construction, a `.reverse` conflict
encountered *there* can never be a real bug; both call sites now pass
`on_conflict="ignore"`, trusting the structural (same/complementary)
comparison result unconditionally.

**Verification**: a direct unit test confirms `validate_characteristic_sign`
still raises on a genuinely wrong sign (regression-guard intact, not just
silenced); `tank.stp`'s isolated `[6,7]` pair and the full 12-solid file
both convert cleanly now; the d1suned stochastic volume check (same
methodology as the "MCNP stochastic volume check" section above) gives
`tank.stp` a cell-1 tally of 0.988 +/- 0.78% (1.5 sigma from 1.0, SD4
matches true CAD volume exactly) -- correct, not just crash-free; full
`tests/geo` + `tests/test_cadtocsg.py`, 156/156; and a differential
regression across all 85 STEP files in `Solidos/` (Can/TCone/RoundCorner/
MultiRoundCorner/MultiPlane counts, git-stash before/after, same
methodology as the washer-plane fix earlier in this file) shows **zero
diffs** -- confirming the fix changes behavior only for the previously-
crashing ambiguous case and nothing else. `tank.stp` (and the earlier
`rev_can_1.stp`/`series_solid2_complement.stp`, resolved separately) are
removed from `Solidos/no_convierte/`, which is now empty.

Only `Can_region`/`add_forwardCan`/`add_reverseCan` were wired to the new
`on_conflict="ignore"` path -- `add_cylinder`/`add_cone` (Tier-2) already
build their region with the sign baked in correctly at construction
(`if cylinder.Orientation == "Forward": cid = -cid`, so
`validate_characteristic_sign` would be a no-op there) and `TCone_region`
has the identical structure to `Can_region` but wasn't touched this
pass -- both are natural, low-risk candidates to extend the same way if
a similar false-positive is ever hit for them.

### `PiezaDavid_pieces_piece_0.stp` removed from `convierte_bad_volume/`: corrupt solid, not a GEOUNED bug

The one previously-open item in `Solidos/convierte_bad_volume/`
(`solidos_PiezaDavid_pieces_piece_0`, 9.8-10 sigma, SD4 differing from
true CAD volume -- flagged "not yet investigated" in the corpus-recheck
notes above) is closed: per explicit user confirmation, this file is a
corrupt solid that doesn't represent a real solid, not a genuine GEOUNED
decomposition/volume bug. Deleted from `Solidos/convierte_bad_volume/`
outright (not moved to `BadCADModel/`, per the user's own instruction).
`convierte_bad_volume/` now contains only `SCDR_90.stp` (known,
pre-existing, already marked in `tests/test_cadtocsg.py`) and
`Torus_solid1.stp` (3.7-3.84 sigma, borderline-but-real, still open).

### `SCDR_90_piece1_roundcorner_badvolume.stp`: residual sliver faces from a boolean cut, and RoundCorner's own split-cylinder gap

Two related fixes, found while root-causing why this file (a piece the
user exported from `SCDR_90.stp` specifically to isolate its bad-volume
bug) converted without crashing but produced the wrong volume.

**Root cause 1 -- residual sliver faces block adjacency walking.** The
solid has 2 near-zero-area cone faces (`Area ~= 0.0024`, vs hundreds/
thousands for every real face) bridging what should be one continuous
radius-37 cylinder, split into 2 pieces by a boolean cut that grazed
tangentially instead of terminating cleanly -- a variant of this
project's original motivating tangency problem (see the top of this
file), but manifesting as a degenerate face artifact rather than a
non-manifold edge. Confirmed via real geometry: both slivers have edges
up to ~60mm long but a cone V-parameter range spanning only ~0.0001 --
a paper-thin strip, not a small patch.

Tried to *repair the CAD topology itself* first, empirically, before
touching GEOUNED: `Part.Shape.defeaturing(faces)` (wraps OCCT's
`BRepAlgoAPI_Defeaturing`, the semantically correct tool) silently
no-ops on singular/near-degenerate seed faces like these; `removeShape`
+`.fix()` collapses the whole solid to garbage; `removeShape`+`sewShape`
(at every tolerance from `1e-6` to `1.0`) produces a "valid" but
genuinely unclosed shell -- the gap needs real surface extension, not
edge-sewing; `removeSplitter()`, `.fix()` at any tolerance, and a full
STEP round-trip all leave the slivers untouched, since OCC's own
`isValid()` doesn't flag them as defective. None of FreeCAD/OCC's
available healing routes worked on this artifact class.

**Fix**: handle it in GEOUNED's own adjacency walking instead of trying
to heal the geometry. `other_face_edge` (`geometry_gu.py`) gained an
opt-in `skip_slivers` mode: when the face found across an edge has
`Area < Tolerances.min_area` (an existing tolerance, already used
elsewhere in `cell_definition.py` for a related purpose), treat it as
transparent and keep walking across its own other edges to find the
real neighboring feature, instead of stopping there or misclassifying.
Wired into `get_adjacent_cylplane`/`get_adjacent_cylknesurfFace`, the
two adjacency walkers Can/RoundCorner/TCone detection depend on; every
other caller of `other_face_edge` is unaffected (`skip_slivers=False`
by default).

**Root cause 2 -- RoundCorner never merged split cylinder pieces, unlike
Can.** Even after root cause 1, this file's RoundCorner still wasn't
detected: `get_roundcorner_surfaces` operated on whichever single
cylinder face happened to be the seed, so if one bounding corner plane
was only reachable from *the other* piece of the split cylinder, only 1
of the 2 required planes was ever found. `closed_cylinder_cone` already
solved exactly this problem for Can/TCone (merging same-surface
contiguous pieces into a `ShellGu` before checking closure) -- extracted
that merge step into a standalone `merge_same_surface_faces` helper and
applied it to RoundCorner too: `get_adjacent_cylplane` gained `ShellGu`
support (mirroring `get_adjacent_cylknesurf`'s existing shell/single-face
dispatch, pooling corner planes found from any merged piece).
`get_additional_corner_plane` and `build_roundC_params` needed the same
shell-awareness downstream (p1/p2 can each be reachable from a different
piece of the split cylinder) -- `get_roundcorner_surfaces` now carries
the merged shell alongside the original seed face through `rc_list` (a
5-tuple, was 4) specifically for this.

**Verification**: the file now registers `RoundC: 1` (was a bare `Cyl: 1`
-- an incorrect stand-in for the real fillet shape) and its d1suned
volume tally is `0.99883 +/- 0.23%` (0.5 sigma, SD4 matches true CAD
volume exactly) -- confirms the fix resolves the actual volume bug, not
just the classification symptom. `tests/geo` + `tests/test_cadtocsg.py`,
156/156 after each fix. An 87-file `Solidos/` corpus regression (Can/
TCone/RoundCorner/MultiRoundCorner/MultiPlane counts, before/after) shows
only 2 diffs across both fixes combined, both newly-detected composite
surfaces that were previously silently missed (`BadCADModel/
SCDR_90_piece0_gsplit_tangent_bug.stp` gains a `MultiRoundCorner`+
`MultiPlane`; this file gains its `RoundCorner`) -- no file loses a
detection, no new crashes. `SCDR_90_piece1_roundcorner_badvolume.stp`
removed from `Solidos/convierte_bad_volume/` (now correct);
`SCDR_90_piece3_boolean_neg0_badvolume.stp` (also exported from
`SCDR_90.stp`, same investigation) is still there, not yet root-caused.

### `SCDR_90_piece3_boolean_neg0_badvolume.stp`: root-caused down to two false leads, GEOUNED confirmed correct as-is

Closes out the investigation started above. The user manually inspected
this file's real geometry (STEP faces exported per-index with normals --
see the diagnostic scripts noted below) and worked out, independently,
that GEOUNED's output for this solid -- `RoundCorner` (`{0,3,15}` in the
original 19-face numbering) plus `ReversedConeCylinder` (`{10,11}`) plus
two bare `Cone` registrations (`{1,2}`, `{4,5}`) -- is already correct,
even though it doesn't match the single, clean 6-surface `AND`
expression (`-15 -3 -0 +10 -2 -4`) the user first derived and verified
300/300 against real points. Two false leads were chased and abandoned
before landing here, both reverted (working tree confirmed clean, no
diff against the last commit):

- **A "cone-bounded RoundCorner" extension to `get_roundcorner_surfaces`**
  (let the corner-plane chain walk pass through a real, non-sliver cone
  to reach a second cylinder of a different radius) -- built, verified to
  correctly find the cylinder beyond the cone, but explicitly rejected by
  the user: `RoundCorner`'s definition is strictly cylinder + two
  *planes* (normals perpendicular to the axis), chained cylinder->plane->
  cylinder only. "Cone bounded corner has no meaning" for this type.
  Reverted in full.
- **A "cone and cylinder must both be Reversed" gate on `get_join_cone_cyl`**
  -- motivated by the user's claim that `ReversedConeCylinder` (RevCC)
  requires a *pair* of Reversed cone+cylinder faces, and since this
  file's real cones are Forward-oriented, RevCC "cannot" legitimately
  appear here. Traced with the user's own explicit request to see it
  live under a debugger rather than through more scripted tracing.
  Turned out to be based on two independent misunderstandings, both
  resolved by the user directly: (1) `get_join_cone_cyl`'s `adjacent1`/
  `adjacent2` check is about the seed cylinder's own *angular* boundary
  (does it span a full 2*pi, and if not, what closes the gap) -- not an
  *axial* cone connection at all, so this whole file's `RevCC` trigger
  never actually touches the real cones in the first place; and (2) more
  fundamentally, RevCC's design only ever required *one* Reversed
  cylinder-or-cone face to trigger, not a matched pair -- the user's own
  prior assumption about the composite type's requirements was wrong,
  not GEOUNED's implementation. Reverted in full.

**Root cause of the whole "why doesn't this look like my clean formula"
puzzle, per the user's own final diagnosis**: two small, genuinely real
planes (not artifacts, contrary to an earlier read of this same
investigation) sit close enough to the main cylinder to look, on casual
visual inspection, like they could be the cylinder's own tangent
continuation -- but one of them is the actual surface GEOUNED's own
decomposition cuts along, splitting the solid into the two pieces
(`sub[0]`/`sub[1]`) that this whole investigation kept running into. The
CAD model itself is genuinely awkward here (small-but-real faces sitting
right at the boundary of what a human would call "the same surface,"
by design or by poor original authoring) -- not a defect GEOUNED
mis-handles, just one it represents through a more roundabout, but
verified-correct, combination of composites than the single clean
formula a human deriving it by eye would write down.

**User's own closing assessment**, worth preserving verbatim in spirit:
this version of GEOUNED is "quite robust" on this particular solid
despite its CAD representation being genuinely deficient -- the
conversion still produces a valid result even though the underlying
model is poorly built. Matching a human's simplified, single-expression
mental model of "what this shape obviously is" is explicitly named as a
longer-term aspiration ("un geouned ideal"), not a near-term gap to
close -- the current, more roundabout-but-correct composite combination
is accepted as-is for this file.

Diagnostic assets from this investigation (kept for reference, not
committed): `Solidos/RoundCorners/piece3_faces/` and
`piece3_faces_original/` (every face of the decomposed vs. original
solid, exported individually as STEP files, index-named, plus
`face_report.txt` with area/Axis/CenterOfMass/Apex/SemiAngle per face --
the tool this whole investigation was actually resolved with, once the
user could see the real geometry directly rather than only through
traced adjacency reports).

### `SCDR_90.stp`: a concrete, quantitatively-confirmed instance of the project's original motivating tangency problem

Follow-up to the earlier `SCDR_90_piece0_badvolume.stp`/`piece3` investigations. `SCDR_90.stp` itself (the un-decomposed original, 346082.46 mm^3) fails the d1suned volume check: `0.91941 +/- 0.23%` (35.0 sigma). Isolated per-component (5 decomposed pieces, each independently converted + d1suned-checked): pieces 2/3/4 are correct (within ~1.2 sigma); pieces 0 and 1 are both genuinely wrong (`0.95564`/16.4 sigma and `0.73891`/81.6 sigma respectively), both with `SD4` matching their own true CAD volume exactly (the decomposition *thinks* it has the right volume; the real MCNP surface reconstruction doesn't enclose that much space once particles are actually tracked through it).

Root-causing `piece0` went through two false leads before landing on the real cause, both instructive:

- **First hypothesis (wrong): a bare, unbounded Tier-2 cone with no angular/spatial scoping.** `piece0` has two cone pairs sharing the same axis line but opposite apexes (`idx=1/2`, apex y=104.5, axis -Y; `idx=4/5`, apex y=-4.5, axis +Y) plus a third cone face `idx=11` (apex y=-4.5, matching `idx=4/5` -- confirmed via `is_same_surface`, literally the same analytic cone, with `idx=5` directly touching `idx=11`, `distToShape=0.0`) that ends up correctly absorbed into a `MultiRoundCorner`. `idx=4/5` don't get merged with `idx=11` (likely because `idx=11` gets consumed by the round-corner detector before the fallback loop reaches `idx=4/5`) and end up registered as an independent, unbounded Tier-2 cone (`add_cone`, no apex plane, no bounding plane -- confirmed `cone_apex_plane` correctly returns `None` since the real apex sits outside the solid's own bounding box, so that part isn't broken). The *other* cone (`idx=1/2`) gets the identical bare treatment. Point-sampling (2000 points) found 28 failures, all with the identical violated condition (`check_sign(idx1/2's surface)` wrong, every other term correct) -- which looked at first like "an unbounded cone reaching into territory it doesn't own." Precise trigonometry confirmed the failing points are genuinely `~1.1 deg` outside `idx=1/2`'s own cone envelope, and the transition matches `idx=11`'s own envelope boundary exactly -- so the *initial* conclusion was that `idx=4/5`'s identical bare registration is silently safe only because it's mathematically identical to `idx=11` (a "lucky coincidence"), while `idx=1/2` has no such twin and genuinely conflicts. **This conclusion was wrong** (see below) -- flagged directly by the user, who insisted this was a cutting/decomposition problem, not a classification-scoping coincidence.

- **Second, confirmed-correct diagnosis: a real `Gsplit`/OCC boolean silent-cut failure, because the cutting tool is built from -- and therefore exactly coincides with -- a real face of the solid's own boundary.** Traced every `Gsplit` call across the full `SCDR_90.stp` decomposition tree (apex/axis identity logged per cone-tool call): the `idx=4/5`/`idx=11` cone (apex -4.5, axis +Y) *does* successfully split the original 322020.5 mm^3 solid into two pieces (188929.2 + 133090.7) at one point in the tree -- but every later re-attempt of either cone against any descendant piece (9 total cone-tool `Gsplit` calls found across the tree, all after that first success) returns exactly 1 piece, never 2. Directly reproduced the specific failure on `piece0`: a real OCC `.cut(cone_tool)` gives `0.0` volume, globally -- but a tiny local probe (a 0.01mm sphere placed exactly at one of the 28 failing points) confirms real material *is* there (nonzero `common()` with the true solid) and genuinely outside the cone tool (`isInside()` false, zero `common()` with the tool) -- a direct contradiction between the large-scale boolean result and local ground truth. Monte Carlo point-sampling (200000 points) estimated the "outside `idx=1/2`'s cone, but truly inside the solid" region at `4304 mm^3` (4.36% of the solid) -- matching the d1suned-measured deficit (`4387 mm^3`, 4.44%) almost exactly, confirming this region *is* the entire volume error, not a coincidental partial explanation.

  The mechanism: the `Gsplit` cutting tool for this candidate is built directly from `idx=1/2`'s own real apex/axis/semiangle -- i.e. the tool's surface is *exactly coincident* with a real face already forming part of the solid's own boundary. This is precisely the project's original motivating tangency problem (see the top of this file): a cutting surface whose intersection with the solid coincides exactly with a pre-existing boundary/tangency, causing `BOPTools.SplitAPI.slice` to silently fail to separate the two sides instead of erroring. Not something fixable by loosening/rescoping a classifier -- it's the same unsolved class of bug the eventual pyOCC face-adjacency-graph migration is meant to address.

**Lesson, stated directly by the user and worth internalizing**: when a `check_sign`/point-sampling investigation finds a small, geometrically-explicable discrepancy region, don't stop at "the classification is unbounded and this region happens to belong to a different, unrelated feature" -- that can look like a complete, satisfying explanation (and did, initially) while missing that the *reason* the region wasn't properly separated in the first place is a genuine boolean-cut failure. Cross-check with a real OCC boolean operation (and, when that also gives a suspicious all-or-nothing result like exact `0.0`, cross-check *that* with a tiny local probe) before concluding a scoping/classification story explains a wrong-volume finding.

`SCDR_90.stp` and `SCDR_90_piece0_badvolume.stp` moved to `Solidos/BadCADModel/` (genuine CAD-topology tangency issue, not a GEOUNED classification bug -- matches the established convention for that folder). `SCDR_90_piece1_badvolume.stp` (the worse of the two bad pieces, 81.6 sigma) was visually confirmed by the user to show the same pattern and moved to `BadCADModel/` too, without a full independent trace -- `convierte_bad_volume/` now contains only the unrelated, still-open `Torus_solid1.stp`.

### `Torus_solid1.stp` fixed: `merge_periodic_uv` under-reported the merged V-range when one piece nests inside another

Closes the last open item in `convierte_bad_volume/`. Real, confirmed bug (per the user's own hunch going in: "seguramente sera una mal colocacion de superficies adicionales -- el toro tiene 2").

The solid's torus is split into 2 face pieces (`is_same_surface` confirms same Center/Axis/MajorRadius/MinorRadius): a real piece (area 38.57) spanning V=[2.773, 5.112], and a tiny residual sliver (area 0.14) spanning V=[3.898, 4.346] -- entirely *nested inside* the real piece's own V-range, not a simple adjacent chain. `SolidGu.merge_periodic_uv` (`geometry_gu.py`), used to recombine a torus's split pieces into one true U/V extent before building the bounding planes for a non-closed torus, sorts the pieces by their own start value and then took `params[-1][1]` (the *last sorted-by-start* piece's own end value) as the merged maximum. That's only correct for a simple, non-overlapping chain of pieces -- when one piece is nested inside another, sorting by start doesn't imply sorted end, so the merged V-max silently came out as the *sliver's* own end (4.346) instead of the real piece's true end (5.112). The resulting V-bounding plane then cut off real material at the true end of the torus's own tube extent.

Fix: take the max end value across all pieces explicitly (`max(v1 for _, v1 in params)`) instead of assuming it's whichever piece sorts last by its own start. The periodic-wrap branch (pieces spanning the 0/2*pi boundary) was already correct and untouched -- confirmed separately, by hand, that this same file's U-range merge *does* hit that branch and computes correctly.

Verified: `Torus_solid1.stp`'s d1suned tally goes from `0.97055` (3.7 sigma) to `0.99672` (0.4 sigma) -- `SD4` matched the true CAD volume in both cases, confirming the bug was in the bounding surface geometry, not the reported volume. 156/156 tests/geo + test_cadtocsg.py; zero diffs across the `Solidos/` corpus regression. `convierte_bad_volume/` is now empty of real, unexplained failures -- every file that started this session's investigation there has been either fixed (`SCDR_90_piece1_roundcorner_badvolume.stp`, this file) or correctly re-filed as a genuine CAD defect (`BadCADModel/`).

### `round_corner_region`/`cyl_plane_region_conf` deep-dive on L1_S23.stp solid 174 — root cause narrowed to a rare near-tangent case, no fix landed, session ends with source fully reverted

Started from the L1_S23.stp full-model lost-particle investigation (see the
Big_model_reserved section preceding this one): isolating solid 174 alone
reproduced the loss (10 lost particles, tally=3.75/sigma=14.83) and the user
spotted, by eye, that its RoundCorner should be a flat AND `(1 2 3 -4)`, not
the actual `(-4:1 2 3)` (`p1 OR (pd AND cid AND p2)`) written by
`round_corner_region`'s `AND_p1_cyl and not AND_p2_cyl` / `AND_p1_pd and not
AND_p2_pd` / `not OR_bracket` branch (`basic_functions_part1.py`).

**First fix attempt (applied, then explicitly reverted by the user) --
`p1only_AND_p1pd_only_no_bracket`: OR to AND.** Validated 500/500 against
real CAD ground truth for solid 174 and via d1suned (0 lost particles,
tally 0.99247/sigma=1.93, was 10 lost/tally=3.75). But a systematic sweep of
every branch's formula against its `p1<->p2`-swap mirror (prompted by a
branch-coverage-visited-vs-unvisited analysis -- see below) found this fix
made `p1only_AND_p1pd_only_no_bracket`'s formula byte-identical to the
already-existing, separately-validated `both_AND_p1pd_p2pd` branch
(`p1*p2*pd*cid`) for a *different* flag combination. **The user rejected
this on principle**: "cada rama esta pensada para una configuracion
diferente. dos configuraciones distintas NO pueden dar lugar a la misma
expresion booleana" -- two genuinely different `(AND_p1_cyl, AND_p2_cyl,
AND_p1_pd, AND_p2_pd)` combinations collapsing to the identical formula is
itself evidence the *classification* is wrong, not that the formula needed
patching to match a different branch's answer. Reverted via
`git checkout -- basic_functions_part1.py`; **`round_corner_region` itself
is confirmed correct as originally written and was not touched further this
session.**

**Root cause, once redirected to classification**: `cyl_plane_region_conf`
(`meta_surfaces_utils.py`) derives each corner plane's material-pointing
normal (`n1`/`n2`) via a local heuristic (`pr1 = ac1.cross(p1_axis)`,
`pr2 = -ac1.cross(p2_axis)` -- note the asymmetric extra unary minus on the
p2 side, absent from p1's) rather than trusting the plane's own
`Surface.Axis` directly. For solid 174's corner specifically: real-face
export + direct `is_inside()` ground truth (`Solidos/solid/solid174_faces/`,
built the same way as the earlier SCDR_90 piece3 face-dump) confirmed both
p1 and p2's *raw, untouched* `Surface.Axis` already point toward material
correctly (`p1: Axis=(0,0,+1)`, material at `z>-977`; `p2: Axis=(0,0,-1)`,
material at `z<2439.95` -- both Orientation=`Reversed`) -- **`pr1`
correctly leaves `n1` unflipped; `pr2` incorrectly flips `n2`** to
`(0,0,+1)`, feeding wrong values into `AND_p2_cyl`/`AND_p2_pd` (both come
out `False` when they should be `True`), landing on the buggy branch
instead of the already-trusted `both_AND_p1pd_p2pd` (config=54, all 4
`AND_p*` flags `True`, confirmed both algebraically and by re-deriving
`AND_p1_cyl`/`AND_p2_cyl`/`AND_p1_pd`/`AND_p2_pd` from scratch with the
confirmed-correct `n1`/`n2` -- `diag_solid174_configuration.py`,
`compute_AND_cyl_with_correct_n.py`, scratchpad-only).

**Three attempted general fixes, all independently confirmed correct for
solid 174 (config -> 54, `both_AND_p1pd_p2pd`) and all independently
confirmed to *break* `RoundCorners/rc16.stp`** (10 lost particles via
d1suned, vs. baseline's clean 0 lost/tally=0.99423/sigma=1.15, in every
case) -- **none were applied to source**:
1. Remove the asymmetric negation directly in `AND_p2_pd`/`AND_p2_cyl`'s
   own `>0` tests (leaving `pr2`/`n2` untouched).
2. Remove `pr2`'s own extra unary minus (`pr2 = ac1.cross(p2_axis)`,
   matching `pr1`'s unnegated form) -- the most surgical-looking option,
   still broke `rc16.stp` identically.
3. Replace the whole `pr1`/`pr2` heuristic with the pre-migration
   convention the user recalled from when the FreeCAD interface lived
   directly inside GEOUNED: material is `-face.Surface.Axis` if
   `Orientation=="Forward"`, else `face.Surface.Axis` unchanged. This
   convention **is already implemented correctly elsewhere in the
   codebase** -- `conversion/cell_definition_functions.py::gen_plane`
   (`normal = face.Surface.Axis; if face.Orientation == "Forward": normal
   = -normal`) -- confirming `cyl_plane_region_conf` reinvented (and, for
   this one plane, got wrong) something already established and trusted
   everywhere else planes get classified. Algebraically identical to
   `-face.normal_at(u,v)` (FreeCAD's own Orientation-corrected outward
   normal).

**Critical methodology correction, found only after the 3rd attempt kept
failing `rc16.stp` despite being "obviously" the established convention**:
naively comparing raw `n1`/`n2` (or `gen_plane`'s formula) against absolute
`is_inside()` ground truth is only a valid test when `cylinder.Orientation
== "Reversed"` (`fwd_cyl=False`) -- i.e. solid 174's own case, where
`round_corner_region` applies no final complement. **All 7 of
`rc16.stp`'s round-corner cylinders are `Orientation="Forward"`
(`fwd_cyl=True`)** -- meaning `round_corner_region` takes the complement
of the *entire* built expression at the end
(`return -rc_region if fwd_cyl else rc_region`), so `n1`/`n2` are only
ever required to be correct *up to that eventual global complement*, not
in an absolute sense. Naive raw-normal ground-truth comparison (my first
pass) wrongly favored the `gen_plane`/`normal_at()` convention 8/8 on
`rc16.stp`; **redone properly** -- comparing the *final*, already-registered
`round_corner_region` output (`.region`/`.components`, which already
includes the complement) via `check_sign(point, roundC)` against real
`is_inside()`, sampling in a cylindrical tube around each corner's own
cylinder (`verify_rc16_final_expression.py`) -- the verdict flips
completely: **baseline (`pr1`/`pr2`) 1088/1800 (60.4%) vs. the
`gen_plane`-based fix 902/2400 (37.6%)**, i.e. the *existing* heuristic is
actually the better match once the complement is accounted for correctly.
This fully explains, retroactively, why the 3 general fixes broke
`rc16.stp`: they were solving the wrong (complement-blind) formulation of
the problem.

**Ruled out as the discriminating factor**: `cylinder.Orientation`
(Forward vs. Reversed) itself. `ac1`'s sign relative to the cylinder's own
real `Surface.Axis` was checked across all of `rc16.stp`'s (all-Forward)
corners and found to vary corner-by-corner (governed by the unrelated
`d2 > d1` "which end of the arc comes first" edge-traversal flip, *not*
by `Orientation`) while `pr1`/`pr2` still worked correctly every time
(`check_ac1_vs_orientation.py`) -- so `ac1`'s own sign ambiguity isn't the
cause either. A corpus-wide scan (excluding `Big_model_reserved`, per the
user's own request to ask before including those 5 slow files) found 124
round-corner calls with `cylinder.Orientation="Reversed"` across 50 files
(`find_reversed_cyl_corner.py`) -- **`RoundCorners/rc0.stp`, picked as a
small clean Reversed example, has `n1_old == n1_new` and `n2_old == n2_new`
exactly (zero disagreement) and both confirmed correct against real
`is_inside()` ground truth** (`check_rc0_reversed_corner.py`) -- i.e.
`pr1`/`pr2` also works perfectly fine on a *different* Reversed-cylinder
corner. **`fwd_cyl`/`Orientation` alone does not predict the bug.**

**Current leading hypothesis, not yet confirmed**: the bug is tied to a
genuinely near-degenerate/near-tangent geometric configuration specific to
solid 174's own corner, not a general sign-convention error reachable by
any blanket formula change. Concretely: `cross1 = n1.cross(nc1)` (used for
`AND_p1_cyl`) has `length=8.56e-4` for solid 174's p1 -- `nt1`/`n1` sit only
~0.049 deg from parallel, a genuine (not floating-point-noise) near-tangency
between the corner plane and the cylinder at that end, four orders of
magnitude above the existing `cross.length < 1e-8` degenerate-fallback
guard (so the guard never fires, but the heuristic is apparently still
fragile there) -- vs. `rc0.stp`/`rc16.stp`'s corners, which are all
well-conditioned (no near-tangency) and where `pr1`/`pr2` works cleanly.
Stability-tested directly: perturbing the *sampling point* used to
evaluate `nc1` (`cylinder.normal_at(u,v)`) by tiny amounts confirmed the
sign is fully stable throughout the entire *valid* domain of the cylinder
face (`u` from the seed value out to the opposite end) -- an earlier
apparent sign flip under negative perturbation was an artifact of
evaluating *outside* the real trimmed face, not a genuine instability
(`diag_robust_ac1.py`) -- so this isn't a simple "nudge the sampling point"
fix either; the near-tangency itself (not the point of evaluation) is the
suspect condition, not yet root-caused down to a specific formula term.

**Also confirmed, a genuine and unrelated finding worth keeping**:
`cyl_plane_region_conf(cylinder, ep1, ep2)` is **exactly swap-consistent**
under `ep1<->ep2` reordering -- calling it with the two corner planes
swapped reproduces precisely the naive `p1<->p2` relabeling of every flag,
bit for bit, for solid 174's real geometry (`test_swap_invariance.py`).
This confirms `p1only_*`/`p2only_*` branch pairs genuinely describe the
same physical situation under relabeling (useful context for the earlier,
separately-investigated-and-inconclusive `p2only_AND_p2pd_only_no_bracket`
mirror-symmetry question below) -- it's specifically `pr2`'s *absolute*
correctness against ground truth that's in question here, not any
inconsistency between the two argument orders.

**Branch-coverage side investigation** (before the above redirect):
instrumented `round_corner_region` and ran it across the full pytest suite
+ a 95-file `Solidos/` corpus scan (including `Big_model_reserved`, this
was *before* the user's ask-first request for that folder -- see
`feedback_ask_before_big_models` memory) -- 12/22 branches visited, 10
never hit (3 are the intentional `raise RuntimeError("should not
exist")` guards, expected never to fire; 7 are real, unexercised gaps,
most notably `p1only_AND_p1pd_only_OR_bracket` / its mirror
`p2only_AND_p2pd_only_OR_bracket`, the `OR_bracket=True` siblings of the
branch this whole investigation started from). A follow-up hypothesis that
`p2only_AND_p2pd_only_no_bracket` (visited, e.g. by `rc16.stp`) carries the
identical pre-fix OR-based bug as its mirror -- motivated by the same
formula pattern recurring -- was tested via 3000-point sampling on
`rc16.stp` and found genuinely **inconclusive** (buggy-formula 49.1%,
mirror-fix 33.1% over the whole padded bbox; a tightened local box gave
88.7% vs. 76.0%, i.e. the *current* code scored better, not worse) --
**not fixed, left as-is**; this is a separate, lower-priority thread from
the `pr1`/`pr2` investigation above and was not revisited once the
near-tangency hypothesis took over.

**State at end of session**: `git status` clean, zero diff against the
last commit (`cece308`) -- every fix attempt this session was tested only
via in-memory monkeypatching in scratchpad scripts, never landed in
`basic_functions_part1.py` or `meta_surfaces_utils.py`. `Solidos/solid/`
(new folder, created this session) holds `L1_S23_solid174.stp` and
`solid174_faces/` (8 individually-exported faces + `face_report.txt`) for
continuing the investigation without re-running the full L1_S23
decomposition. Diagnostic scripts (scratchpad only, not committed --
listed here since several encode non-obvious, reusable methodology):
`diag_solid174_configuration.py`/`diag_stability_solid174.py`/
`diag_robust_ac1.py` (per-vector instrumentation + perturbation stability
tests), `test_swap_invariance.py` (the `ep1<->ep2` swap-consistency
proof), `export_solid174_faces.py` (real-face dump, the methodology to
reuse for any future "which plane is really p1/p2" question),
`verify_p1_p2_material_side.py` (the `is_inside()`-based ground-truth
check that must only be trusted when `fwd_cyl=False`), `test_pr2_fix.py`/
`test_normalat_fix.py`/`convert_rc16_*_variants.py` (the 3 general-fix
attempts, each paired with a d1suned run), `verify_rc16_final_expression.py`
(the corrected, complement-aware final-expression ground-truth check --
**the template to reuse for any future correctness question about this
function**, since the naive raw-normal check is a proven trap),
`check_ac1_vs_orientation.py`/`find_reversed_cyl_corner.py`/
`check_rc0_reversed_corner.py` (ruling out `Orientation` as the
discriminant), `branch_coverage_round_corner.py` (the branch-visitation
instrumentation).

**Next session should resume here**: narrow the near-tangency hypothesis
down to a precise, testable condition (does `pr1`/`pr2` fail specifically
when `cross1.length` or `cross2.length` falls in some intermediate band
above the existing `1e-8` guard but below some larger threshold? does it
depend on *which* of `cross1`/`cross2` is small, given the asymmetric
`pr1`/`pr2` formulas?) using `rc16_pr2fix_variants`/`rc16_variants`/
`rc16_normalat_variants`'s already-exported MCNP models plus
`Solidos/solid/L1_S23_solid174.stp` as the two known reference points,
before attempting any further fix.

### Resolution: `cyl_plane_region_conf` used the single seed cylinder face for both ends' reference vectors, instead of each end's own real adjacent piece

Follow-up session, continuing directly from the near-tangency hypothesis
above. Root-caused and fixed; verified via pytest + full corpus diff +
d1suned; committed.

**How the near-tangency hypothesis resolved**: perturbation-testing
`nc1`/`ac1` (as done in the previous session) only checks *stability*, not
*correctness* -- and `ac1 = nt1 x nc1` is mathematically guaranteed equal
to the cylinder's own axis (up to sign) at *any* point on a true cylinder,
so it can never itself be corrupted by a nearby plane's tangency. The real
question was where `nc2`/`r2` (used only for `pr2`) were being evaluated,
not whether the near-tangent `cross1`/`AND_p1_cyl` computation was stable
(it already was, confirmed independently).

**Real root cause, found by inspecting solid 174's actual CAD (per the
user's direct suggestion to look at where materially the plane and
cylinder sides meet the shared edge, since a region whose only "material"
is the tangent edge itself was a live clue)**: the round-corner cylinder
is a true cylinder whose axis has zero Z-component, split (via
`merge_same_surface_faces`) into 2 same-surface 90-degree face pieces,
`face[0]` (u=[90,180] deg) and `face[3]` (u=[180,270] deg). At `face[0]`'s
own u=90 boundary the cylinder's radial direction is exactly `(0,0,-1)` --
genuinely, exactly tangent to p1 (a Z=const plane) by design, not by
floating-point noise (confirmed: `cylinder.Surface.Center.z - Radius`
equals p1's own z to full precision). Symmetrically, `cylinder.Center.z +
Radius` equals p2's z *exactly* too -- but that second tangency point
(radial direction `(0,0,+1)`) falls at u=270 deg, which is `face[3]`'s own
far boundary, **not** `face[0]`'s. `get_adjacent_cylplane`'s `ShellGu`
branch (`meta_surfaces_utils.py`) already searches every piece of a
merged shell individually and tags each found corner plane with *the
specific piece that actually touches it* -- confirmed directly:
`ep1`'s own cylinder piece is `face[0]` (`Index=0`, matching the seed),
but `ep2`'s is `face[3]` (`Index=3`, a *different* object) --
`ep1[0]`/`ep2[0]` (previously always discarded via `_, e1, _, _, p1 =
ep1`-style unpacking) already carried exactly the information needed.

`cyl_plane_region_conf(cylinder, ep1, ep2)`, however, used the single seed
`cylinder` argument's own `ParameterRange` for **both** ends: `r1`/`nc1`/
`nt1` from its `u1` boundary (correct, since `cylinder` *is* `face[0]`,
p1's real adjacent piece) and `r2`/`nc2` from its own `u2` boundary --
`face[0]`'s far end, which is just the *seam* between the two split
pieces (the u=180 deg point, an arbitrary trim boundary with no special
relationship to p2) -- instead of from `face[3]`'s own far boundary
(u=270 deg, the real second tangency point). `pr2`'s formula itself was
never wrong; it was evaluating a geometrically real quantity, just at the
wrong physical location on the wrong piece of a multi-piece cylinder.
This also explains why every attempted uniform sign-formula fix (pr2's
negation, `AND_p2_pd`/`AND_p2_cyl`'s negation, the full `gen_plane`/
`normal_at()` replacement -- all 3 from the section above) broke
`rc16.stp`: none of them addressed the actual bug (wrong reference point
on a split cylinder), so on ordinary un-split round corners (the vast
majority, including every one in `rc16.stp`/`rc0.stp`) they just
substituted a different, *also* wrong sign convention for a heuristic
that was already correct there.

**Fix** (`cyl_plane_region_conf`, `meta_surfaces_utils.py`): unpack
`cyl1`/`cyl2` from `ep1[0]`/`ep2[0]` (each end's own real adjacent
cylinder piece -- normally identical to the seed `cylinder`, but not
always) instead of discarding them; source `r1`/`nc1`/`nt1` from `cyl1`'s
own `ParameterRange` and `r2`/`nc2` from `cyl2`'s own -- specifically
`cyl2`'s **far** boundary (`u2b`, mirroring the original code's own
`u2`-as-far-boundary convention, just sourced from the correct piece).
`pr1`/`pr2`'s own formulas, `AND_p1_cyl`/`AND_p2_cyl`/`AND_p1_pd`/
`AND_p2_pd`'s tests, and the final `fwd_cyl`-based complement are all
completely untouched -- this is purely a "evaluate at the right point"
fix, not a sign-convention change, which is exactly why it doesn't
disturb any of the (many) already-correct ordinary cases.

**Verified**: solid 174 now computes `AND_p1_cyl=AND_p2_cyl=AND_p1_pd=
AND_p2_pd=True` (config 54, `both_AND_p1pd_p2pd` -- the same
already-trusted flat-AND branch/formula the user predicted from the very
start of this investigation, reached this time via a correctly-evaluated
classification instead of a patched formula). d1suned on the isolated
solid, end to end: `0` lost particles, tally `0.99247`/sigma `1.93` (was
10 lost/tally `3.75`/sigma `14.83`) -- matching, to 5 significant figures,
the result from the very first (later-reverted) direct-formula patch,
confirming both routes converge on the same physically correct answer.
Full `tests/geo` + `tests/test_cadtocsg.py`: 156/156. Full 95-file
`Solidos/` corpus diff (`git stash` before/after, real source change, not
monkeypatching): **exactly 1 file differs**,
`Big_model_reserved/TVA_final_allencl.stp` (`RoundC` 19 -> 17, every other
count and every other file byte-identical, zero new crashes) --
confirmed via d1suned that TVA's own pre-existing lost-particle count
(10, unrelated to this fix) is *unchanged* by the fix in either
direction. `rc16.stp` (the canary that broke on all 3 previous fix
attempts) is untouched by this one, confirmed both by the corpus diff
(absent from the 1-file diff list) and directly via d1suned (`0` lost
particles both before and after, identical tally).

### `TVA_final_allencl.stp`'s pre-existing lost-particle problem, part 1: enclosure solids duplicated into `meta_list` (real bug, confirmed NOT the cause)

Follow-up to the round-corner fix above: with the L1_S23 fix committed and verified, `TVA_final_allencl.stp` was the one remaining model with a pre-existing lost-particle count (10, unchanged by the round-corner fix in either direction -- confirmed identical before/after). User's own diagnosis going in: this model's STEP tree includes "Enclosure" solids -- CAD bodies that don't represent real physical components, only regions used to bound where void cells get generated -- and suspected the v2 migration mishandled separating them from real solids.

**Confirmed real, but not the cause of these particular lost particles.** `CadToCsg.load_step_file` already splits `Load.load_cad`'s output into `self.meta_list` (real solids) and `self.enclosure_list` (enclosure-only regions, identified via a `enclosure<N>_<M>_` label-or-parent-label regex in `loadfile/load_step.py`). For this file: `len(meta_list) == 52` and `len(enclosure_list) == 4` -- but those same 4 enclosure solids (huge volumes, ~15.7 to ~190 billion mm^3, `IsEnclosure=True`, labels like `enclosure1_0_`) are present in **both** lists, not removed from `meta_list` once correctly classified into `enclosure_list`. This means they get double-processed: once correctly as void-boundary regions, and once incorrectly as if they were real material cells, physically overlapping all the real geometry they're meant to only bound -- a genuine, real bug matching the user's suspicion.

**Ruled out as the cause of the 10 lost particles**, though, by direct test: converting with `skip_solids=[48,49,50,51]` (removing the 4 duplicated enclosure entries from `meta_list`, matching real `meta_list` indices confirmed via inspection, not guessed) reproduces the identical 10 lost particles, at the identical trajectories/surfaces (confirmed by comparing the `lost particle` trace blocks in `outp` line for line -- same surfaces 69/70/118/etc, same history numbers). The duplication is real and worth fixing on its own merits (each of those 4 enclosure solids is currently being needlessly decomposed and converted into real MCNP cells with enormous volumes, doubling work and risking exactly the kind of overlap the user described in principle) but is not what's causing these specific lost particles -- that turned out to be a separate, unrelated bug, described next.

### `TVA_final_allencl.stp`'s pre-existing lost-particle problem, part 2: a third, newly-confirmed instance of the already-known-broken Can/TCone secondary-surface orientation rework

With the enclosure-duplication path ruled out, traced the real cause from the raw `outp` lost-particle trace itself (per this session's established discipline: verify from GEOUNED's own written output before touching Python internals). All 10 lost particles cluster into 3 distinct trajectory patterns crossing 3 distinct surface groups -- one of them (5 of 10 particles) crosses `PZ 650`/`PZ -100` (the model's own top/bottom bounds) after travelling in a straight line for ~750 units with no intermediate surface crossing recorded, landing at a position (`X=-4374mm`) far *outside* even the real solid's own BoundBox (`XMin=-1940.5mm`) -- a direct sign that the cell's own written CSG expression, not just MCNP's tracking, doesn't actually bound the real material.

Confirmed by literal analytic evaluation of the exact written formula for that cell (`10 2 -100 (-76 62:-77:-60:14) (-78 62:17:-79:-60) (-62:-23) -23 60 -69 70`, the "Barrel upper left" solid): the formula genuinely evaluates `True` (inside) at points along the lost trajectory that the real CAD solid (`is_inside`) confirms are `False` (outside) -- a real leak in the written boolean expression, not a downstream MCNP tracking artifact. Root cause: clause `(-62:-23)` is a no-op, because `-23` (the model's own left/right `PX 0` split plane) is *also* independently, unconditionally required elsewhere in the same cell (the bare `-23` term) -- so `(-62 OR -23)` is trivially satisfied by `-23` alone regardless of `-62` (the real outer-wall cylinder, radius 193.68cm), meaning the outer wall is never actually enforced and the cell has no radial upper bound in that direction.

Traced further, via direct instrumentation of `can_region`'s real call arguments for this exact Can (`basic_functions_part1.py`, the pure AND/OR formula function): the outer-wall cylinder (component id 2, radius 1936.8mm) is passed into `can_region`'s `surf_list` with `orientation='Reversed'` -- but the real underlying CAD face for that exact radius/axis (`face[2]` on the real solid) has `Orientation='Forward'`, confirmed directly from the loaded STEP geometry. Feeding the wrong orientation flips which branch of `can_region`'s per-secondary-surface formula gets used (`sid * -pid` instead of the correct `-sid + -pid`), which is exactly what turns "material must be inside the outer wall" into "material must be outside the outer wall AND left of a certain plane" -- the leak.

**This is not a new bug** -- it's the *third* confirmed concrete failure of the "Can/TCone secondary-surface orientation rework" already flagged as an unresolved, "known-broken checkpoint" in this file's own earlier history (see that section above: committed as WIP at explicit user instruction, with `tank.stp`/`rev_can_1.stp` as the first 2 known regressions). The exact mechanism is in `build_can_params` (`utils/functions.py`, lines ~367-370):

```python
if omit:
    orientation = s.Orientation
else:
    orientation = "Reversed" if s.Orientation == "Forward" else "Forward"
```

When `omit` is `False`, the secondary surface's *reported* orientation is deliberately flipped (part of the same-session-documented attempt to normalize `region_sign`'s AND/OR result against the main cylinder's own orientation so the representation is uniform) -- for this Barrel Can, that flip turns the real, correct `Forward` into a wrongly-reported `Reversed`, which `can_region` then takes at face value, producing the leaking formula. Not fixed this session -- consolidating with the 2 already-known regressions as the next concrete target: `build_can_params`'s `omit=False` branch (and/or whatever upstream `region_sign`-based logic decides `omit` and the paired orientation flip in the first place) needs to be re-derived against real CAD ground truth the same way the round-corner fix above was, ideally using this Barrel case (small `omit=False` reproduction: `TVA_final_allencl.stp` solid index 9, "Barrel upper left", `skip_solids=[j for j in range(52) if j != 9]`) alongside the pre-existing `tank.stp`/`rev_can_1.stp` cases, rather than another blanket formula guess.

### `TVA_final_allencl.stp`'s lost-particle problem, part 3: `build_can_params`'s omit-flip was a red herring; the real bug was in `round_corner_region`

Follow-up to part 2's "Barrel upper left" finding (`build_can_params`'s
`omit=False` orientation-flip, `utils/functions.py` lines ~367-370,
flagged as the suspected cause). Tested directly: removed the flip
(Cylinder branch only, via monkeypatch first, then confirmed with a real
source edit) and reconverted both the isolated solid (index 9) and the
full 52-solid model. **The flip removal changed the written formula but
did not fix the lost particles** -- same 3 cells (10, 17, 21) still lost
particles identically before and after. Reverted the edit; `build_can_params`
was not the cause after all. (First isolated-solid d1suned attempts also
falsely looked "unaffected" by the patch -- traced to a stale-monkeypatch
bug in the test script, not a real finding; the *full*-model byte-diff of
the two variants' `.mcnp` output, done afterward, confirmed the patch
genuinely does change cell 10's formula, just not in a way that fixes
anything.)

The user then spotted the real bug by inspecting `TVA_solid16_cell17.stp`
(exported alongside solids 9/20, the other two failing cells, into the
new `Solidos/lost_particles/` folder) directly: the round corner's
`PX~0` plane sign looked wrong -- written `(-15:-2) 16 -17 4 -15`, should
be `(15:-2) 16 -17 4 -15`. Isolated-conversion reproduction confirmed the
exact same clause with small, readable surface numbers.

**First hypothesis (chased, then disproven): `simplify_planes` (`write/
functions.py`) breaks composite regions.** `simplify_planes` normalizes
PX/PY/PZ planes to a canonical positive-axis direction, flipping the
plane's shared `BoolVariable` reference (`bVar.change_ref()`) so every
*live* reference to that id reinterprets consistently. Traced plane 15
(here: `Axis=(-1,0,0)` pre-flip) through the full pipeline and confirmed
this flip *does* fire for it. Naive comparison of `round_corner_region`'s
raw, pre-flip return value (`OR[15,-2]`) against the post-flip *written*
text (`OR[-15,-2]`) looked like exactly the kind of "flip doesn't
propagate to an already-baked composite region" bug this session
already knows the shape of (`simplify_planes`'s docstring section
earlier in this file). **This was the wrong diagnosis** -- confirmed
empirically: `BoolSurface(0, p1id)` (how `round_corner_region` builds
its terms) actually stores the *live* `BoolVariable` object in the
`BoolSequence` tree (`BoolSurface.set_definition`'s `isinstance(...,
BoolVariable)` branch), not a frozen string/int -- so `change_ref()`
*does* propagate correctly. A direct 3000-point Monte Carlo against the
real decomposed solid (`GSolid.is_inside`, scaled mm->cm to match the
written MCNP surfaces) confirmed the *written*, post-flip formula
(`OR[-15,-2]`) is the one that's wrong (78.7% match) and the *pre-flip*
raw value (`OR[15,-2]`, i.e. what the user proposed, unflipped) is
correct (99.9% match) -- meaning the bug is upstream of `simplify_planes`
entirely, inside `round_corner_region` itself.

**Real root cause**: `round_corner_region`'s `if p1id == p2id:` fast path
(`basic_functions_part1.py`, used when a round corner has only one
distinct bounding plane instead of two) applies `fwd_cyl` *twice* for
this specific sub-case:

```python
if fwd_cyl:
    p1id = -p1id
    p2id = -p2id
if p1id == p2id:
    if AND_p1_cyl:
        rc_region = BoolSurface(0, p1id) * BoolSurface(0, cid)
    ...
return -rc_region if fwd_cyl else rc_region
```

Once *before* building the region (the pre-negation) and once again *after*
(the final `-rc_region` complement) -- for this branch specifically, the
two applications don't cancel out correctly, leaving `p1id`'s sign wrong
in the final result. Verified by hand (De Morgan expansion) and, more
importantly, empirically: **the AND sub-branch (`if AND_p1_cyl:`) is
broken -- 78.7% vs 99.9% on the TVA case -- but the sibling OR sub-branch
(`else:`) is not.** Checked the OR sub-branch directly against 2 other
real files that exercise the identical `p1id==p2id, fwd_cyl=True,
AND_p1_cyl=False` combination (`Solidos/trier/series_solid2_complement.stp`,
`series_solid2_halfcyl_plus_inclined.stp`, both pre-existing regression
fixtures from an earlier session): the *current* code already matches
CAD ground truth 100% (2000/2000) there, and the "candidate blanket fix"
(un-negating `p1id` for the whole `p1id==p2id` block, both sub-branches)
would have *broken* this already-correct OR case (only 21.4% match) --
exactly the kind of false generalization this project's `get_can_surfaces`/
`outer2_only` history already warned about (see "The washer-plane gap in
`get_can_surfaces`" above): a fix that looks like a strict, uniform
narrowing/widening of one condition can silently break an unrelated,
already-correct sibling case, so *always* re-verify both branches
independently rather than reasoning abstractly about symmetry.

**Fix, scoped to exactly the broken sub-case**: capture `p1id`'s value
*before* the `fwd_cyl` pre-negation (`p1id_raw`), and use that original,
un-negated value only inside `if p1id == p2id: if AND_p1_cyl:`. The
sibling `else` (OR) branch, and every other branch in the function
(`elif AND_p1_cyl and AND_p2_cyl:` etc., covering the general two-plane
case already validated extensively via `solid174`/`rc16`/the corpus's
`MultiRoundCorner` scan earlier in this file), are untouched.

**Verification**: `tests/geo` + `tests/test_cadtocsg.py` 156/156; a
zero-transform-except-the-fix corpus diff (Can/TCone/RoundC/MultiRoundC/
MultiP counts, `git stash` before/after) across 84 `Solidos/` files
(excluding `Big_model_reserved`) -- **0 diffs**, same single pre-existing,
unrelated `w_encl.stp` failure (`'list' object has no attribute 'level'`)
both before and after; and the full d1suned stochastic volume check on
the isolated, previously-failing cell (`TVA_solid16_cell17.stp`, with
real void generation, `volSDEF=True`): before the fix, 10 lost particles,
aborted at nps=18; after, **0 lost particles**, full 1,000,000-particle
run, tally `0.9984 +/- 0.31%` (0.52 sigma).

**Update**: cells 10 and 21 were re-checked directly (isolated conversion
+ d1suned, same methodology) and are fixed by this same
`round_corner_region` change -- cell 10 (`TVA_solid9_cell10.stp`): 10
lost -> 0 lost, tally `0.9974 +/- 0.25%` (1.04 sigma); cell 21
(`TVA_solid20_cell21.stp`): 10 lost -> 0 lost, tally `0.9961 +/- 0.27%`
(1.44 sigma). All three originally-failing TVA cells shared this one
root cause.

### `TVA_final_allencl.stp`'s lost-particle problem, part 4: a *different*, still-unresolved leak in cells 1/2 (RPV upper right/left), and a real, separate `distToShape` bug found and fixed along the way

With cells 10/17/21 confirmed fixed, converting the *full* TVA model
(52/48 solids, not just the 3 isolated cells) surfaced a **new** failure,
previously masked: cells 1/2 ("RPV upper right"/"RPV upper left") now
lose particles (10 lost, aborts at nps~103202) -- these cells' own
isolated conversions pass perfectly cleanly on their own (0 lost, tally
~1.00/0.999), so this is a cross-solid interaction, not a defect in
either cell alone.

**Root cause, narrowed via a long dialogue with the user, working from
real MCNP plotter screenshots and direct CAD queries** (not fixed this
session -- still open):
- The leaking clause is a `FwdCan` on "RPV upper left"'s own tiny
  decomposed corner fragment (`Solidos/lost_particles/TVA_can45_piece0.stp`,
  Volume~4.6e6mm^3): main cylinder cid=30 (R=395, small tilted corner
  cylinder), secondaries `s1=Plane(40, real, shared with Shoulder0041)`
  and `s2=Cylinder(2, R=2443.2 outer wall) + a synthetic edge-derived
  plane` (`cks_edge_plane`/`spline_wires`, since the cylinder30/cylinder2
  tangency curve is a genuine `GBSpline`, not a circle).
- The *same* cylinder cid=30 is *also* the main cylinder of a **RevCan**
  (found via `Shoulder0041`'s own Can detection), whose secondaries are
  `s1=Plane(40, same real plane)` and `s2=Plane(171, a *different* real
  plane, `Shoulder0041.face[5]`)` -- i.e. the Rev/Fwd pair does *not*
  share the same `s1`/`s2` pairing the user expected; the Reversed side
  correctly found a second *real* plane, the Forward side instead fell
  back to the cylinder+synthetic-plane construction.
- `Gin_contact`/`Gcommon`/`Gdistance` confirm piece0 and Pipe0041 are
  CAD-perfect tangent (0 common volume, 0 distance) at the shared
  R=2443.2 wall -- no real CAD gap or overlap anywhere. `piece0.face[3]`
  and `Pipe0041.face[6]` are the *exact same* analytic cylinder,
  bit-identical parameters.
- The MCNP plotter (`ip` command), run by the user directly, visually
  confirms a genuine geometric conflict (dashed/conflicting lines,
  MCNP's own "problem plane coincident" diagnostic) exactly at the
  piece0/Pipe0041 junction, involving the synthetic plane and the shared
  cylinder -- matching the diagnosis above.
- **Minimal reproduction confirmed**: `piece0` alone + `Pipe0041` alone
  (no decomposition needed at all, both are already-finished solids)
  reproduces the identical 10-lost-particle failure
  (`Solidos/lost_particles/TVA_piece0_plus_pipe0041.stp`). Further
  narrowed with the user's own `TVA_2_28.stp` (cells 2+28+34 together):
  2+34 alone passes cleanly; 2+28+34 together reproduces the leak --
  cell 28 (Pipe0041) is necessary but not sufficient, some 3-way
  interaction is involved, not fully root-caused.
- **Not yet fixed**: why `spline_wires`'s heuristic (BSpline-pole
  projection along the principal inertia axis, `0.51*span` margin
  offset) computes a position ~1600mm away from any real geometric
  feature for piece0's specific tangency edge, when the *same* function
  applied to Pipe0041's analogous edges lands correctly (either exactly
  on a real face, or legitimately synthetic-but-close). Picked up next
  session.

**Separate, real bug found and fixed while investigating the above --
unrelated to the actual leak, but a genuine correctness+performance
issue in its own right**: while checking why `Pipe0041`'s *own* other
R=395 corner (unrelated to piece0) failed to form a Can, traced
`get_can_surfaces` -> `commonEdge` -> `commonEdgeFace` ->
`FaceGu.distToShape` (`geometry_gu.py`) -> `GFace.my_distToshape`
(`_freecad_impl.py`, the BoundBox/`Common()`/edge-`isSame()` fast-path
alternative to the native `distToShape` query, documented earlier in
this file as "kept as the user's own comparison/testing tool, not wired
back in" -- that description was already stale, `FaceGu.distToShape`
*does* call it live). Two real bugs in it:
- `my_distToshape`'s final fallback (BoundBoxes overlap, but neither
  `Common()` nor edge `isSame()` can confirm contact) returned a
  **hardcoded `dist2Shape = 1.0`** instead of a real measurement --
  confirmed via direct comparison (`GFace.distance_to`, the reliable
  native query, gives `0.0` for the exact same pair) that this sentinel
  was firing on a genuinely *touching* pair, wrongly reporting a 1mm gap
  and causing `commonEdgeFace`'s `> 0` check to reject a real Can.
- `FaceGu.distToShape`'s own `ShellGu` branch (recursing over a merged
  shell's faces to find the minimum distance) initialized `distmin = 1`
  instead of `float("inf")` -- the same sentinel-as-real-value mistake,
  latent (would silently under-report a genuine gap larger than 1mm as
  exactly 1mm, or over-report a real sub-1mm gap has never been
  observed to matter in practice, but is a live correctness bug in
  general).

**Fix**: `my_distToshape`'s dead-end now calls `self.distance_to(other)`
(the reliable native query) instead of guessing `1.0`; `distToShape`'s
`ShellGu` branch now seeds `distmin` with `float("inf")`.
`FaceGu.distToShape`'s own dispatch (call `my_distToshape`, not
`distance_to`, in the non-`ShellGu` case) is otherwise unchanged --
**explicit user call**: a blanket switch to always calling the reliable
native `distance_to()` was tried first and confirmed correct, but caused
a severe performance regression (a single 2-solid file's decomposition
went from ~1.2s to ~483s) because far more Can candidates now succeed
(previously silently, incorrectly rejected) and trigger real, expensive
CAD construction. Falling back to the native query only in
`my_distToshape`'s one genuinely ambiguous branch keeps the fast path
for the overwhelming common case while fixing the correctness bug --
verified back to ~1.2s decomposition on the same file.

**Verification**: `tests/geo` + `tests/test_cadtocsg.py` 156/156 (twice,
once per fallback strategy tried). Confirmed via `TVA_2_28.stp` that this
fix is real and independent of the cells-1/2 leak above: it does fix
`Pipe0041`'s other R=395 corner's Can detection (previously silently
lost to the bogus 1mm gap), but the cells-2/28/34 d1suned result is
byte-identical before and after this fix (same nps=81164 abort, same
tallies to 5 significant figures) -- confirming these are two genuinely
independent bugs, not the same root cause.

### `get_can_surfaces` was checking `commonEdge` against the raw seed face instead of its own merged `cylinder_shell` -- a second, independent bug found via the same R=395 corner

Following up directly on the `distToShape` fix above -- the user kept
digging into why `Pipe0041`'s *other* R=395 corner (unrelated to
piece0/cid=30) still returned `None` from `get_can_surfaces` even after
that fix. Traced with the same instrumentation used throughout this
session (`closed_cylinder_cone`, `commonEdge`, phase-tagged prints):
`closed_cylinder_cone` correctly merges this cylinder's 3 split pieces
(Indexes 0, 4, 6, confirmed same Center/Axis/Radius -- a prior boolean
cut split one analytic surface into 3 face pieces) into a `ShellGu`
(`cylinder_shell`), and `region_sign(cylinder_shell, s)` already
correctly uses that merged shell -- but the two `commonEdge(...)` calls
right above it (`meta_surfaces.py`, checking whether each adjacent
surface actually shares an edge with the cylinder) still passed the raw
seed parameter `cylinder` (whichever single piece, e.g. Index=4,
happened to be the one `next_Can`/`get_Can` were iterating on), not
`cylinder_shell`. Confirmed directly: `commonEdgeFace(Index=4, the real
adjacent plane)` reports `distToShape=0.0` (genuinely touching) but
returns an **empty edge list** -- because the actual shared edge belongs
to a *different* piece of the same merged cylinder (Index=0 or 6, not
4), and `commonEdgeFace` only ever looks at the single face it's given,
never the other pieces of the group.

Before applying a fix, the user asked whether the caller (`get_Can`/
`next_Can`, both in `functions.py`/`generators.py`) should instead
pre-merge same-surface faces and pass a shell down to `get_can_surfaces`,
rather than patching inside the function. Checked: both callers use the
identical `for f in solidFaces: if isinstance(f.Surface, GCylinder): ...
get_can_surfaces(f, solidFaces)` pattern, passing a single raw face --
neither ever pre-merges. This confirms the intended design is "hand
`get_can_surfaces` any one face of the cylinder, it merges internally" --
matching what `closed_cylinder_cone` already does at the top of the
function -- so the fix stays fully inside `get_can_surfaces`, no caller
changes needed. Also confirmed the existing "omit every merged piece,
not just the seed" bookkeeping (`ShellGu.Indexes`, `closed_cylinder_cone`'s
`ck_index = set(ck_shell.Indexes)`, `get_Can`'s `canface_index.update(surfindex)`)
was already correct and would work automatically once `get_can_surfaces`
actually succeeds -- it just never got the chance to fire, since every
one of the 3 seed attempts independently failed on the same bug.

**Fix**: both `commonEdge(cylinder, s, ...)` calls now use `cylinder_shell`
instead of `cylinder`. This surfaced a second, related bug while
verifying: `commonEdge`'s own `ShellGu` branch returns a `(edges,
matching_face)` *tuple* (unlike the plain-face branch, which returns
just the edges list) -- `build_can_params` already knew to unpack this
(`if shell: edges, cyl = commonEdge(...) else: edges = commonEdge(...)`),
but the new `get_can_surfaces` call sites didn't, causing
`planar_edges()` to receive the whole tuple as if it were an edge list
(`AttributeError: 'list' object has no attribute 'Length'`, caught by
`tests/test_cadtocsg.py`, 2 failures). Fixed with the same
`is_shell = isinstance(cylinder_shell, ShellGu)` dispatch.

**Verification**: `tests/geo` + `tests/test_cadtocsg.py` 156/156 (after
fixing the tuple-unpacking regression); corpus diff across 84 `Solidos/`
files (excluding `Big_model_reserved`, same methodology as every other
fix this session) -- **0 diffs**, same single pre-existing `w_encl.stp`
failure both before and after. Confirmed via `TVA_2_28.stp` that
`Pipe0041`'s other R=395 corner now correctly forms its Can (previously
silently dropped). **Also confirmed independent of the cells-1/2/28/34
leak**: d1suned on the same file still loses 10 particles -- if
anything aborts *earlier* now (nps 6529 vs 81164 before), consistent
with the newly-recognized Can adding surface area to a model that still
carries the separate, unfixed `spline_wires` bug from the section
above. Real, verified, safe fix -- but does not move the open leak.

### `build_can_params`'s non-planar closing plane: replaced the empirical margin heuristic with an analytic two-contour calculation

The `cks_edge_plane`/`spline_wires`-based plane closing a Can's
non-planar (BSpline) secondary surface used a fixed `0.01 * d` shift
toward the secondary surface's own center -- an empirical guess that
found the original TVA_2_28.stp leak (piece0/Pipe0041's R=395 corner):
the shift could land inside the real tangency's own ambiguous zone and
cut real material, causing MCNP lost particles.

**Root geometric insight** (derived from a user-led back-and-forth,
starting from a hand-drawn diagram): the main Can cylinder and the
secondary surface (cylinder/cone/sphere) are two infinite analytic
surfaces meeting along real tangency contours. For each angle around the
main cylinder's own circumference, the line along its axis crosses the
secondary surface's boundary at up to two points -- a quadratic in the
axial parameter, solvable in closed form for all three secondary types.
Sweeping the angle traces one or two real contours; the free zone where
a plane can sit without cutting either "outside secondary" or "inside
secondary" real material lies strictly between the near contour's own
furthest-advanced point and the far contour's own furthest-back point,
projected onto the plane's normal. Verified against plane D's own real
geometry: the closed-form calculation reproduces the numeric search's
own boundary values (353.07/5175.19) to 2 decimal places, and a second,
far contour (~5175, previously invisible to the numeric search's smaller
step budget) was discovered this way.

**The plane's normal is not always the main cylinder's own axis**: for a
Cylinder/Cone secondary (which has its own axis), it's the direction
perpendicular to that axis within the plane containing both axes -- the
natural cross-cutting direction when the two axes are close to parallel
(verified with a synthetic near-parallel test case: normal came out
`(-0.9998, 0, 0.0175)`, nothing like the main axis). Only a Sphere
secondary (no axis of its own) uses the main axis directly.

**Position within the free zone**: per explicit user design, not simply
the midpoint (which could drift arbitrarily far from the real local
feature when the zone happens to be huge, as with plane D's second
contour). If the zone is narrow (half-width <= a threshold `T = 0.01 *
main_radius`), use the true midpoint (maximally safe on both sides).
If wide, use `min(0.001 * half_width, T)` past the near contour --
scales gently for moderately-wide zones, capped at `T` so it never
drifts far even for enormous zones. The discontinuity at the narrow/wide
boundary is intentional (explicit user confirmation), not smoothed.

Returns `None` -- rejecting the whole Can candidate, propagated up
through `build_can_params`/`next_Can`/`get_Can` -- if either contour
fails to close over the full angular range (no real roots at some angle,
or, for a cone secondary, a root landing on the wrong nappe): the main
cylinder isn't actually split into two disjoint pieces by the secondary
surface, so the Can premise doesn't hold at all, and no plane should be
guessed.

**A real, cross-product sign-ambiguity bug found and fixed during
verification**: `find_can_plane`'s normal comes from a cross product,
which has no preferred sign of its own -- unlike `cks_edge_plane`'s own
convention, always resolved via `material_direction` (a real geometric
signal: the face's own outward normal crossed with the tangent edge
direction, both orientation-corrected). Found via the full verification
pipeline (`SCDR.stp` crashing with a `laj overflow` MCNP fatal error;
`TVA_final_noencl.stp` cells 1/2/16/17 off by 5.7-8 sigma) -- both clean
with the old heuristic, both broken with the new analytic one. Root
cause confirmed by diffing the exported `.mcnp` surface cards directly:
2 of ~12 differing planes had their normal *fully flipped* (not just
repositioned) between old and new code, for the same physical corner.
Fixed by applying the identical `material_direction`-based sign check
`cks_edge_plane` already used, right after `find_can_plane` returns, in
`_closing_plane` (the new dispatcher that picks between `cks_edge_plane`
for planar/circular tangencies and `find_can_plane` for non-planar
ones): sample `material_direction` at the real tangency edge's own
midpoint, flip the returned normal if its dot product with that
direction is negative. Verified: `SCDR.stp` recovered its full clean
1,000,000-particle run (tally identical to the pre-analytic-rewrite
baseline, `0.99838`); `TVA_final_noencl.stp`'s 4 broken cells came back
to `0.32-0.96` sigma; the 86-file `Solidos/` corpus diff (excluding
`Big_model_reserved`) went from 1 difference (`series_solid3_complement.stp`'s
`RevCan` count, itself confirmed a different-but-valid decomposition via
d1suned) to **zero** once the sign fix landed -- meaning the sign bug
was silently steering that file's decomposition too, not just the two
volume-check failures that surfaced it.

**Full-corpus verification, this pass**: `tests/geo` +
`tests/test_cadtocsg.py` 156/156; the 86-file `Solidos/` differential
regression, zero diffs; the original `TVA_2_28.stp` leak still resolved
(0 lost particles, tallies `0.998`/`0.989`); a fresh translation +
d1suned pass across all 6 `Big_model_reserved` models (517 solid-cell
tallies total) -- 96.5% within 2 sigma, the only >3 sigma cases being
`TVA_final_allencl`'s already-documented, confirmed-pre-existing
cells-1/2 leak (identical lost-particle severity with the old code too)
and 2 marginal `SCDR.stp` cells (3.0-3.4 sigma, not investigated
further, plausible statistical noise at 153 cells sampled).

### `Solidos/Cans/pipe.stp`/`Tcan.stp`/`RevTcan.stp` and `Solidos/Big_one_cell/`: new user-provided fixtures, 3 more real bugs found

New corpus additions, per the user's own workshop: 3 small hand-built
Can/TCone examples (`pipe.stp`, `Tcan.stp`, `RevTcan.stp`, in `Solidos/Cans/`)
and 2 large real-world models (`FWTBM1.stp`, `modelCell_670000.stp`, in
the new `Solidos/Big_one_cell/` folder -- a solid component with many
pipe-like holes drilled through it, "como haber muchos RevCan", per the
user's own description). All 5 translated and checked with the same
d1suned stochastic volume methodology as everywhere else in this file:
the 3 `Cans/` fixtures all converted and ran clean (0 lost particles,
1,000,000-particle runs, 1.2-1.7 sigma); `FWTBM1.stp` converted and ran
clean too (0.39 sigma).

`modelCell_670000.stp` crashed, surfacing a real, pre-existing,
production-reachable bug unrelated to anything else this session:
`get_can_surfaces`'s "adjacent cylinder same radius and parallel"
branch (`meta_surfaces.py`) appended a 2-tuple `(s, None)` while
`build_can_params` always unpacks a 3-tuple `(s, r, omit)` --
`ValueError: not enough values to unpack`. Fixed to a 3-tuple (`omit`
value is never read on this branch, `True` only keeps the shape
consistent with every other entry). `tests/geo` + `test_cadtocsg.py`
still 156/156 after the fix -- this exact branch was apparently never
exercised by the existing corpus.

### `rev_pipe.stp`/`fwd_pipe.stp`: a minimal, deliberately-built pair of fixtures, and 3 more MultiRoundCorner-construction crashes fixed

`modelCell_670000.stp`'s *next* failure (past the tuple-unpacking fix)
was a deliberate safety guard (`_validated_material_direction`, added
earlier in this file's own history) rejecting a residual sliver face's
untrustworthy geometry -- too large/complex a model to debug directly,
per the user's own call ("hay que aislar el problema para no
perdernos"). The user built a minimal, deliberately-paired reproduction:
`Solidos/trier/rev_pipe.stp` (13 faces, the "reverse"/original solid --
the *big* block with 2 small pipe-junction corners carved out) and
`Solidos/trier/fwd_pipe.stp` (8 faces, the exact complementary piece --
just the 2 small carved-out corners, joined by a 4-plane "bridge").
`rev_pipe.stp` alone reproduces the crash deterministically; `fwd_pipe.stp`
converts cleanly (`FwdCan x2`, `MultiRoundC x1`) and, per the user's own
suggestion, served as an independent reference for what a *correct*,
uncorrupted construction of the same physical corner looks like --
though it turned out `fwd_pipe.stp` doesn't have an equivalent corner
for every failure found (see below), so this only partially panned out
as a direct oracle and instead mostly helped narrow down which
mechanism was broken.

Chasing `rev_pipe.stp`'s crash surfaced 3 more real, independent bugs in
`get_additional_corner_plane`/its helpers (`functions.py`,
`meta_surfaces_utils.py`) -- **all reachable through the same underlying
cause**: a round-corner cylinder split into pieces by an earlier cut,
whose split-boundary is razor-thin/near-degenerate (the corpus's
existing fixtures never had a case this extreme).

1. **The sliver-validation guard itself was masking its own fix.**
   `get_adjacent_cylplane`'s `skip_slivers` walk (`other_face_edge`)
   already finds the real, non-degenerate face beyond a residual sliver
   -- and *had always been returning it* (the 5-tuple's last element,
   `plane`) -- but `get_additional_corner_plane` discarded it (`_` in
   the unpacking) and evaluated `material_direction` on the sliver's own
   near-zero-area geometry instead, only cross-checking the sign against
   a *different* reference (the anchor's own edge). Per direct user
   confirmation ("sí usa la cara real en lugar de sliver en este caso"):
   fixed to evaluate on the real far face (`plane`) instead of the
   sliver (`near_face`) whenever a sliver walk actually happened
   (`near_face.Index != anchor.Index`) -- the whole
   `_validated_material_direction` wrapper (raise-on-disagreement) is no
   longer needed and was deleted, since there's no longer an untrustworthy
   value to validate.
2. **A second, genuinely different degeneracy, once the first was fixed**:
   two round-corner "wings" sharing the exact same tiny cylinder fragment
   (a wedge converging to a single point/cusp) gave `v1 = -v2` *exactly*
   -- `(v1+v2).normalized()` divides by zero. Per direct user
   confirmation, this is not a numerical fluke but a real, valid
   configuration for a **Reversed MultiRoundCorner**: every wing's
   material-pointing normal faces "outward", and two wings meeting at a
   cusp can legitimately point in exactly opposite outward directions
   there; since the corner's bounding planes are OR-combined, either
   direction alone is a correct choice (there is no well-defined
   bisector to average toward instead when they're antiparallel). Fixed
   by falling back to `v1` alone when `(v1+v2).length < 1e-6`.
3. **`region_sign` crashing on `Edges[0]` when `commonEdge` legitimately
   finds no shared edge**, for a pair `cutting_face_number` only
   speculatively considered adjacent (walking every face pair, not one
   pre-confirmed to share an edge). Root cause, confirmed by comparing
   against the *native* `distToShape` directly: `GFace.my_distToshape`'s
   BoundBox-fallback branch reported a genuinely-touching pair (native
   distance `0.0`) as `18.23` units apart, because one face is flat
   (zero-thickness bounding box) along an axis the other face's own
   bounding box doesn't span at all -- `Boxinter.YLength` came out
   *negative* (`-3.5`), correctly failing the "boxes intersect" check
   even though the real shapes do touch at a point. Same class of bug as
   the `dist2Shape=1.0` sentinel fixed earlier this session, reached via
   a different call path (`commonEdgeFace`'s own pre-check, not
   `FaceGu.distToShape`'s dead-end fallback). Fixed defensively at the
   `region_sign` level rather than trying to patch `my_distToshape`
   again: return `None` when `Edges` comes back empty/`None` -- checked
   all 6 existing call sites, every one already does `if sign ==
   "OR"`/`"AND"`, safe with `None`.
4. **`eligible_plane` crashing on `e.Vertexes[1]`** for a degenerate
   zero-length edge with only 1 vertex. Fixed to reject the plane
   (matching the function's own existing non-`GLine` rejection) instead
   of crashing.

**Verification**: `tests/geo` + `tests/test_cadtocsg.py` 156/156 after
each of the 4 fixes individually and combined; a full 86-file `Solidos/`
differential regression (excluding `Big_model_reserved`) -- **zero**
Can/TCone/RoundCorner/MultiRoundCorner/MultiPlane count differences from
the prior commit, confirming none of the 4 fixes changed anything for
the established corpus, only unblocked `rev_pipe.stp`'s previously-crashing
case. `rev_pipe.stp` now builds its `MultiRoundCorner` successfully
end-to-end (previously crashed on all 3 `functions.py`/`meta_surfaces_utils.py`
bugs in strict sequence -- fixing one only revealed the next).

**A `forward` build parameter was added alongside this** (user's own
addition, mostly independent of the 4 bug fixes): `GeounedSurface.build_surface`/
`makeCan`/`makeMultiRoundCorner`/`build_complex_shape` gained an optional
`forward` parameter -- when `True` and the surface's own `Orientation`
is `"Reversed"`, complements the cell's boolean definition before
building, producing the shape's Forward/complementary version instead
of its natural Reversed one (for comparing a composite surface's two
possible orientations against each other, e.g. against `fwd_pipe.stp`-style
independent references). `decompose/decom_one_generators.py`'s own call
site passes `forward=False` explicitly -- unchanged decomposition
behavior from before this parameter existed. **A real accidental
regression found and fixed while integrating this**: the edit that added
the `if forward: ...` block had *replaced* (not added alongside) the
pre-existing `rc.boundBox = myBox(Box, "Forward")` line in
`build_complex_shape` -- leaving `rc.boundBox` permanently `None` and
crashing `BuildDepth` (`subcell.boundBox.Box` on a `None`) for *every*
Can/MultiRoundCorner build, forward or not. (The literal string
`"Forward"` here is unrelated to the new `forward` boolean -- a
same-spelling coincidence between `myBox`'s own internal orientation
bookkeeping, used by `BuildDepth`'s box-splitting logic, and the new
parameter name.) Restored; confirmed `rev_pipe.stp`'s `MultiRoundCorner`
build works again immediately after.

### `rev_pipe.stp`, part 2: a real, reproducible instance of the project's original motivating non-manifold/tangency bug -- found, confirmed, not yet fixed

With `rev_pipe.stp` converting past the 4 crashes above, the user asked
whether `Gsplit`'s own result still under-separates the solid -- the
same class of problem as this file's own "Motivating problem" section.
Cutting the original 13-face solid by its own `MultiRoundCorner` tool
(`Gsplit`, and independently cross-checked with a plain native
`.cut()`, both agreeing exactly: 2 pieces, `504643.5350` + `7694.1331`,
volume conserved to float noise) leaves a **confirmed-invalid** big
piece: `piece.__native__.isValid()` is `False`, with **6 non-manifold
edges**, each shared by 3 faces instead of 2 (`removeSplitter()` and a
STEP round-trip -- the two techniques that "fixed" this class of bug
earlier for other files -- do *not* heal it here, unlike those earlier
cases).

Direct inspection of the 6 non-manifold edges (per-edge face dump: area,
surface type, shared vertices) shows a clean, consistent pattern: **4 of
the 6** involve a genuinely tiny sliver face (area ~0.0125, alongside 2
real substantial faces); the **other 2** (the two *longest* non-manifold
edges, both length 118.81, versus the 4 slivers' 30-or-less) involve one
real, substantial plane -- face 28, area `3564.32`, nothing degenerate
about it -- alongside 2 real faces of the main body.

Building a face-adjacency graph over `piece0`'s own 30 faces, excluding
all 6 non-manifold edges (`GSolid.faces_sharing_edge`'s existing
diagnostic, still not called from anywhere in production code, used
here manually), finds exactly **2 connected components**: a 27-face
main body and a 3-face group (`{28, 27, 29}` -- face 28 itself plus the
2 tiny slivers it happens to also touch). Neither raw group closes into
a valid solid on its own: the 27-face group's own shell is reported
`closed` but `isValid=False` (`.fix()` collapses it to **zero solids**
-- genuinely not a coherent volume without face 28 as its own proper
boundary); the 3-face group's shell is a valid but *open* (non-closed)
patch, `Part.makeSolid` fails outright. Conclusion, not yet acted on:
the correct reconstruction isn't a simple 2-way face partition -- face
28 is the real shared boundary between two solids and needs to be
*duplicated* (one copy per side, opposite orientation) to properly cap
both, the standard technique for splitting a non-manifold solid at a
shared face, one step beyond what `GSolid.faces_sharing_edge`'s existing
diagnostic alone provides.

**Not fixed** -- this is a live, concrete instance of the "still NOT
solved" problem named at the very top of this file (motivating the
entire pyOCC migration), now with a minimal, deliberately-reproducible
13-face fixture (`rev_pipe.stp`) and a fully characterized failure
mode (which 6 edges, which faces, why the 2 obvious repair attempts
don't work) ready for whenever that migration phase starts. Diagnostic
scripts (scratchpad only, not committed): `verify_gsplit_undercut.py`
(the `Gsplit`-vs-native-`.cut()`/`.common()` cross-check),
`verify_piece0_manifold.py` (non-manifold edge count + `removeSplitter`/
STEP-round-trip healing attempts), `reconstruct_piece0.py` /
`reconstruct_piece0_v2.py` (the face-adjacency-graph reconstruction and
per-non-manifold-edge face dump).

## pyOCC migration, Phase 1: engine-selection scaffolding + a positive `Gsplit` go/no-go result

Kicked off the actual pyOCC migration this file's top section has always
named as the eventual fix for the project's original motivating bug.
New branch `pyocc-migration` off `refactor_2.1_base`. Three decisions
confirmed with the user before starting: engine choice via an
environment variable, `GEOUNED_CAD_ENGINE` (`"freecad"`, the default, or
`"occ"`) -- not a `Settings` field, confirmed structurally impossible
without a much bigger refactor, since `geo/__init__.py`'s backend import
happens at `import geouned` time, well before any `Settings`/`CadToCsg`
instance could exist to carry the choice; migration order starts with
`Gsplit`/`GSolid` (highest risk, highest value -- it's the actual
function tied to the motivating bug), not the small analytic surface
classes; `GEOReverse` (CsgToCad) stays FreeCAD-only, permanently out of
scope (hard `.FCStd` export dependency, no pyOCC equivalent).

**Scaffolding**: `geo/__init__.py` now reads `GEOUNED_CAD_ENGINE` once
at module top and imports the same full name list from either
`._freecad_impl` (default) or a new `._occ_impl` -- every other file in
`GEOUNED` is unaffected either way, exactly the swappability this
package was designed for from the start. `geouned/__init__.py`'s
previously-unconditional `from .GEOReverse import *` (which does a hard
`import FreeCAD` at its own top) is now wrapped in the same
`try/except ImportError` pattern the existing conda-`freecad`-shim
import right above it already uses -- under `GEOUNED_CAD_ENGINE=occ` a
user may have no FreeCAD installed at all, so `CsgToCad`/`BoxSettings`
degrade to `None` with a logged warning instead of crashing `CadToCsg`
(GEOUNED) along with them.

**`geo/_occ_impl.py`** (new file) implements only what this validation
phase needs -- `kernel_version()`, `GVector`<->`gp_Pnt` conversion,
minimal `GSolid`/`GFace`/`GEdge` (`.Volume`/`.Faces`/`.Edges`/
`.is_valid()`/`.export_step()`, matching `_freecad_impl.py`'s own field
names where they overlap), `Gload_step`/`Gexport_step`, and `Gsplit`
itself -- everything else (`GPlane`/`GCylinder`/.../`Gcut`/`Gfuse`/...,
~30 names) is a `NotImplementedError` stub, kept only so
`geo/__init__.py`'s import list stays symmetric between both backends
rather than needing its own per-backend branching logic beyond the one
top-level `if`.

`Gsplit`'s pyOCC implementation: `BOPAlgo_Splitter` (`OCC.Core.BOPAlgo`,
the pyOCC equivalent of FreeCAD's `BOPTools.SplitAPI.slice` -- both
produce fragments from *both* sides of the cut, unlike a one-sided
`BRepAlgoAPI_Cut`), then per-resulting-solid `BRepCheck_Analyzer.IsValid()`;
any solid that comes back invalid goes through
`_repair_non_manifold_solid` -- builds an edge->faces adjacency map via
`TopTools_IndexedDataMapOfShapeListOfShape`/`topexp.MapShapesAndAncestors`,
finds edges shared by `!= 2` faces (non-manifold), union-finds the
solid's own faces into connected components excluding those edges, and
for any component missing a real face at a non-manifold edge, adds a
`BRepBuilderAPI_Copy` of a donor component's face there before sewing
(`BRepBuilderAPI_Sewing`) and solidifying (`BRepBuilderAPI_MakeSolid`)
each component independently -- the OCC-native version of the
duplicated-capping-face technique `reconstruct_piece0.py`/
`reconstruct_piece0_v2.py` prototyped by hand against FreeCAD earlier in
this project's history, never finished there.

**The actual go/no-go test, run against the exact `rev_pipe.stp` case
already fully characterized as broken on the FreeCAD side** (see the
section above this one: `BOPTools.SplitAPI.slice` returns a single
`isValid()==False` piece, Volume=504643.5350, 6 non-manifold edges, not
healed by `removeSplitter()` or a STEP round-trip): exported
`rev_pipe.stp`'s original 13-face solid (Volume=512337.6683) and its
`MultiRoundCorner` cutting tool as two independent STEP files from the
FreeCAD-capable default Python, then loaded both fresh into a
FreeCAD-free `pyoccenv` conda environment (pythonocc-core 7.9.0) and
called the new OCC `Gsplit` on them directly, with no shared state or
prior FreeCAD involvement in the cut itself. Result: **`BOPAlgo_Splitter`
correctly returns 3 valid solids on the raw, unrepaired first attempt**
(`degenerate_case_handled=False` -- the repair path was never even
needed): Volume 465264.5885 + 7694.1331 + 39378.9463 = 512337.6679,
matching the original volume to floating-point noise (diff 0.000389).
Confirms the mechanism directly: FreeCAD's single invalid
504643.5350-volume piece is exactly the *union* of the new pieces 0 and
2 (465264.5885 + 39378.9463 = 504643.5348) that `BOPAlgo_Splitter`
correctly keeps separate -- pyOCC resolves precisely the tangency-merge
FreeCAD's own splitter couldn't. `tests/geo` (106/106) confirmed
unaffected by the `geo/__init__.py` scaffolding change (default engine
still `"freecad"`, byte-identical behavior).

**Not yet validated**: the `_repair_non_manifold_solid` fallback path
itself -- this one test case never exercised it, since `BOPAlgo_Splitter`
didn't need repair here. Whether it actually works (on a case where OCC's
own splitter *does* leave an invalid result) is still open; the plan's
own framing already anticipated this ("if OCC's `BOPAlgo_Splitter` +
the face-adjacency reconstruction correctly separates this case, that's
strong evidence to continue" -- true here, but for a simpler reason than
the reconstruction logic being exercised and validated). Also not yet
done, per the plan's explicit "Not in this phase" list: full `GSolid`/
`GFace` port, a parallel `tests/geo/test_occ_impl.py`, the analytic
surface/curve descriptor classes, `Gclassify_surface`/`Gclassify_curve`
dispatch, a full `tests/test_cadtocsg.py` run under
`GEOUNED_CAD_ENGINE=occ`. Also found, not yet fixed: `utils/
meta_surfaces_utils.py` imports `GEdge`/`GFace` directly from
`geouned.geo._freecad_impl` instead of `geouned.geo` -- bypassing the
single-import-point convention this whole package's swappability
depends on, unconditionally requiring FreeCAD/`BOPTools` even under
`GEOUNED_CAD_ENGINE=occ`. Not a blocker for this validation slice
(tested `_occ_impl.py` standalone, bypassing `geouned/__init__.py`'s
full chain) but needs fixing before `GEOUNED_CAD_ENGINE=occ` can ever
support a real end-to-end run.

Diagnostic/setup scripts (scratchpad only, not committed):
`occ_import_helper.py` (standalone `_occ_impl.py` import bypassing the
`geouned` package init chain), `test_occ_simple_split.py` (box-cut-by-
plane sanity check before the real case), `export_rev_pipe_base_tool.py`
(FreeCAD-side STEP export of the base solid + tool, run with the default
Python), `test_occ_rev_pipe.py` (the actual go/no-go test, run in
`pyoccenv`).

## pyOCC migration, Phase 2: full `GSolid`/`GFace`/`GEdge`/`GWire` + analytic
descriptor port, `tests/geo/test_occ_impl.py`

Continued directly from Phase 1's positive go/no-go signal. Also fixed,
as a prerequisite: `utils/meta_surfaces_utils.py`'s `from
geouned.geo._freecad_impl import GEdge, GFace` (bypassing the
single-import-point convention, flagged but not fixed in Phase 1) now
imports from `geouned.geo` like everywhere else -- confirmed via a
direct test that `import geouned` under `GEOUNED_CAD_ENGINE=occ` now
succeeds end-to-end (degrading `GEOReverse`/`CsgToCad` to `None` with a
warning, as designed) in a completely FreeCAD-free environment
(`pyoccenv`), where it previously crashed on this one holdout import.

**`_occ_impl.py` now ports the full surface this migration's Phase 1
plan scoped as "not in this phase"**: all 5 analytic surface descriptors
(`GPlane`/`GCylinder`/`GCone`/`GSphere`/`GTorus`, including
`.from_values`/`.parameter`/`.is_inside`/`.transform`, plus `GPlane`'s
`.intersect_plane`/`GLine`'s `.intersect_line` with the same
hybrid-pure-math-then-native-fallback shape as `_freecad_impl.py`, using
`GeomAPI_IntSS`/`GeomAPI_ProjectPointOnCurve` as the native fallback);
all 4 curve descriptors (`GLine`/`GCircle`/`GEllipse`/`GBSpline`);
`Gclassify_surface`/`Gclassify_curve` (dispatching on
`BRepAdaptor_Surface`/`BRepAdaptor_Curve`'s own `GetType()`, mirroring
FreeCAD's `type(surface) is Part.Plane`-style dispatch); full
`GEdge`/`GWire`/`GFace`/`GShell`/`GSolid` (every method from
`_freecad_impl.py`'s version, including `find_interior_point`,
`faces_sharing_edge`, `fix`/`refine` -- using
`ShapeUpgrade_UnifySameDomain` as the `removeSplitter()` equivalent,
`ShapeFix_Shape` as the deeper repair fallback, same volume-invariance
guard as `refine()`'s FreeCAD version -- `reverse`/`translate`/`rotate`,
`tessellate`/`getUVNodes`, `my_distToshape`); every `Gmake_*` primitive
constructor (`Gmake_box`/`_cylinder`/`_cone`/`_sphere`/`_torus`/
`_half_space`/`_wire`/`_polygon_face`/`_shell`/`_compound`); `Gcut`/
`Gcommon`/`Gfuse` (via `BRepAlgoAPI_Cut`/`_Common`/`_Fuse`); `Gin_contact`/
`Gdistance` (via `BRepExtrema_DistShapeShape`, same BoundBox-prefilter
structure as the FreeCAD version).

Only 2 real gaps remain, both explicitly documented in the file's own
docstring rather than silently missing: `Gload_step_labels` (STEP
assembly/label-tree reading -- a separate concern from `Gload_step`'s
geometry-only read, needs `XCAFDoc`/`STEPCAFControl_Reader`, not yet
attempted) stays a `NotImplementedError` stub; `Gclassify_surface`'s
FreeCAD-side fallback for a `BSplineSurface` that's secretly a mislabeled
flat plane (`face.findPlane()`) has no pyOCC port, so that one case
returns `None` (unsupported surface) instead of a `GPlane` -- narrower
than FreeCAD, not wider (never silently misclassifies something FreeCAD
would reject too).

**`tests/geo/test_occ_impl.py`** (new, 39 tests) mirrors
`test_freecad_impl.py`'s coverage shape against real geometry with known
expected values (volumes, radii, semi-angles, intersection points, not
just "does it run") -- primitives, surface/curve classification,
`GPlane`/`GLine` intersection (well-conditioned, parallel, and skew
cases), face/wire/edge topology, half-spaces, all 3 boolean ops,
`Gsplit`, `Gin_contact`/`Gdistance`, `find_interior_point`/`fix`/
`refine`/`reverse`/`translate`/`rotate`, and a STEP round-trip. Guarded
the same way `test_freecad_impl.py` guards for FreeCAD
(`pytest.importorskip("OCC.Core.BRepPrimAPI", ...)`) plus an explicit
`GEOUNED_CAD_ENGINE` check, since `geouned.geo`'s backend choice is
resolved once at first import and cached in `sys.modules` -- if this
file's `os.environ.setdefault(...)` runs *after* something else in the
same test session already imported `geouned.geo` under the FreeCAD
engine, the module-level skip catches that instead of silently testing
the wrong backend. In practice this only ever matters for a single
process running both suites, which doesn't happen in this project's real
setup (FreeCAD and pythonocc-core live in mutually exclusive conda
environments -- confirmed directly: `test_freecad_impl.py` fails even to
*collect* under `pyoccenv`, `Module use of python311.dll conflicts with
this version of Python`, an OS/ABI-level DLL conflict pytest's
`importorskip` can't catch; `test_occ_impl.py` skips cleanly under the
default FreeCAD-capable Python, as designed). 106/106 `tests/geo`
(FreeCAD/default engine) confirmed unaffected throughout this whole
phase; the `rev_pipe.stp` go/no-go result (3 valid pieces,
`degenerate_case_handled=False`) re-verified byte-identical after
swapping in the full `GSolid`/`GFace` port in place of Phase 1's minimal
version.

**Not yet attempted**: a full `tests/test_cadtocsg.py` run under
`GEOUNED_CAD_ENGINE=occ` (the real decomposition/conversion/void/write
pipeline exercises a long tail of `geo` functionality beyond what
`test_occ_impl.py`'s unit-level coverage reaches -- e.g. void generation,
the writers, `Gload_step_labels`, and many call sites never exercised
by a synthetic box/cylinder/sphere/torus). This is the natural next
slice once picked back up, per the original Phase 1 plan's own framing.

## pyOCC migration, Phase 3: full `tests/test_cadtocsg.py` passes under
`GEOUNED_CAD_ENGINE=occ` (50/50, matching the FreeCAD baseline)

Closes the "not yet attempted" item above -- the actual integration
pipeline (`load_step_file` -> `decompose_solids` -> `build_solid_definition`
-> writers), run end to end against the same 50-file `testing/inputSTEP`
corpus `tests/test_cadtocsg.py` has always used to validate the FreeCAD
engine, now passes identically under the OCC engine. Reached by running
the suite repeatedly against `pyoccenv`, fixing exactly what each
traceback pointed at, and re-verifying `tests/geo` (FreeCAD, 106/106)
after every change -- the same empirical, one-gap-at-a-time discipline
this whole migration has used throughout. 6 real gaps found and fixed,
none of them guessed:

1. **`Gload_step_labels` was a stub, and load_cad calls it
   unconditionally** -- not the secondary/deferrable feature Phase 2
   assumed. Implemented via XCAF (`STEPCAFControl_Reader`, `XCAFDoc_
   DocumentTool.ShapeTool`, walking `IsAssembly`/`IsReference`/
   `IsSimpleShape` labels recursively). Verified positional alignment
   against FreeCAD's own output on 2 real fixtures (`BC.stp`, 1 solid;
   `tubos.stp`, 3 solids) -- node count, per-node `n_solids`, and
   `Gload_step`'s own solid order/volumes all had to line up, per
   `GLabelNode`'s documented contract. Found one real structural
   difference along the way: XCAF can represent several solids as
   sub-shapes of a single "simple shape" label (`tubos.stp`: 1 label,
   `n_solids=3`), where FreeCAD's `Import.insert()` splits that into 3
   separate auto-suffixed leaf objects -- matched by splitting into N
   `GLabelNode`s (sharing the same label/parent) on the OCC side too,
   preserving the positional-count contract exactly even though the
   exact label text can't be replicated without reverse-engineering
   FreeCAD's internal auto-suffix convention (a known, documented,
   non-blocking gap -- affects only the free-text comment for a
   multi-solid-per-label leaf, never indexing/counting).
2. **`solid.Shells[0]`, a real native-FreeCAD leak in 3 call sites**
   outside `geo/` (`build_shape_functions.py::build_complex_shape`,
   `geouned_classes.py::build_surface`'s Sphere and Torus branches) --
   added `Gfirst_shell(native_shape)` to both backends (native-in/
   native-out, matching the existing "convert at the boundary"
   convention these 3 call sites already followed) and swapped all 3.
   `build_shape_functions.py`'s own version also had a real, harmless
   pre-existing dead branch (`if len(solid.Shells) == 0: shell =
   solid.Shells[0] else: shell = solid.Shells[0]` -- both arms
   identical) collapsed away as part of the same edit.
3. **`decom_utils_generator.py::remove_solids`/`valid_solid` were
   entirely native-FreeCAD** (`.removeSplitter()`/`.isValid()`/`.Volume`/
   `.Area` called directly on raw native shapes), reached from
   `decom_one_generators.py::generic_split` by deliberately unwrapping
   `Gsplit`'s own `GSolid` results to native and re-wrapping after --
   real, working FreeCAD-only code with no OCC equivalent at all,
   unlike every other native leak found this migration (which were
   thin, mechanical unwraps). Both already-existing `GSolid` methods
   (`.refine()` -- the same removeSplitter-with-volume-invariance-guard
   `remove_solids` was hand-rolling -- and `.Volume`/`.Area`/`.is_valid()`)
   made the fix a real simplification, not just a port: rewrote both
   functions to operate on `list[GSolid]` throughout, deleted the
   native round-trip at the `generic_split` call site entirely, and
   dropped a confirmed-dead local variable (`compVol`, computed and
   never read) found while rewriting.
4. **`meta_surfaces_utils.py::edge_1D`/`spline_2D` called
   `edge.__native__.Curve.curvature(u)`/`.getKnots()` directly**,
   flagged in their own comments as having "no `geo` equivalent" --
   that was true until this pass. Added `GEdge.curvature(u)` (native
   `Part.Curve.curvature()` for FreeCAD; `GeomLProp_CLProps(curve, u, 2,
   tol).Curvature()` for OCC, verified against a real cylinder: radius-5
   circle gives curvature exactly 0.2, straight edges give 0) and
   `GEdge.knots()` (native `getKnots()` for FreeCAD;
   `Geom_BSplineCurve.DownCast(curve).Knot(i)` for OCC, verified to
   return the same *unique* knot values FreeCAD's version does, not the
   multiplicity-expanded sequence) to both backends. Both call sites
   simplified to `edge.curvature(...)`/`edge.knots()`, no native unwrap
   left in `meta_surfaces_utils.py` at either spot.
5. **A real, engine-independent periodicity bug in
   `get_join_cone_cyl`'s edge-matching loop**, found (not guessed) via
   direct instrumentation of a genuine OCC-only crash (`emin`/`emax`
   left unbound -- the search loop never matched any edge). Root cause:
   `d = abs(umin - u)` doesn't account for angular wraparound, so when
   `twoPimod` normalizes `Umin`/`Umax` to exactly `0.0` (a real,
   legitimate case: a merged face group whose combined angular span
   closes exactly at `2*pi`) but the actual candidate edges sit near
   `2*pi` rather than near `0`, the naive linear distance
   (`~2*pi - 0 ≈ 6.28`) can exceed the loop's own initial bound
   (`du = twoPi`), so the `if d < du` condition never fires for any
   edge. Fixed with the mathematically-strict generalization
   `d = min(d, twoPi - d)` (wraparound-aware angular distance -- always
   `<=` the naive linear distance, so it can only ever *find* a match
   the old code missed, never pick a worse one than before) in both of
   the function's two symmetric search loops (`emin` and `emax`).
   **This is a real, pre-existing bug reachable under FreeCAD too** --
   the OCC port's own slightly different (not wrong, just different --
   raw untrimmed STEP-authored range rather than a wrapped-into-[0,2*pi)
   one) `ParameterRange` convention for this specific merged face group
   is what happened to expose it on `BC.stp`, not something the port
   introduced. `tests/test_cadtocsg.py` (FreeCAD, 50/50) confirms zero
   regressions from this fix -- it only changes behavior in the
   wraparound case the old code got wrong.
6. **`GeomAPI_ProjectPointOnSurf` (backing every analytic descriptor's
   `.parameter()`, plus `GFace.parameter()`'s own separate copy of the
   same call -- found as a second, missed instance of the same pattern
   after the first fix didn't fully resolve the failure) can genuinely
   fail to converge** (`StdFail_NotDone`) -- confirmed two distinct real
   causes, not one: the default gradient-based algorithm
   (`Extrema_ExtAlgo_Grad`) failing where the more exhaustive
   `Extrema_ExtAlgo_Tree` succeeds (most cases), and a genuinely
   ill-posed query -- a point sitting exactly on a cylinder's own axis,
   confirmed via the real failing point
   (`GVector(x=0.0, y=0.0, z=-13.2287565553)` on a `GCylinder` centered
   on that same axis) -- where *no* algorithm can produce a unique
   answer, since every point on the circle at that height is exactly
   equidistant. Added a shared `_project_point_on_surface(point,
   geom_surface)` helper (deduplicating what had been 5 separate
   `GeomAPI_ProjectPointOnSurf` call sites -- one per analytic
   descriptor's own `.parameter()`, plus `GFace.parameter()`) with a
   3-tier fallback: `Extrema_ExtAlgo_Grad` (fast path) -> `Extrema_
   ExtAlgo_Tree` (recovers the real non-degenerate failures) -> `(0.0,
   0.0)` (a defined, arbitrary-but-deterministic answer for the
   inherently-undefined on-axis case, matching this migration's
   existing convention of never letting a genuinely ambiguous
   degenerate case crash the pipeline).

**Verification**: `tests/geo` (FreeCAD, 106/106) and
`tests/geo/test_occ_impl.py` (OCC, 39/39) confirmed after every one of
the 6 fixes above, individually; `tests/test_cadtocsg.py` run to
completion (not `-x`) after each round to catch every remaining gap in
one pass rather than one-at-a-time -- final state: **50/50 under both
`GEOUNED_CAD_ENGINE=freecad` (default) and `GEOUNED_CAD_ENGINE=occ`**,
confirming this migration has reached real functional parity across the
entire `testing/inputSTEP` corpus, not just the hand-picked `rev_pipe.stp`
go/no-go case Phase 1 validated.

**Still open, not attempted this pass** (per Phase 1's own "Not in this
phase" list, still unclaimed beyond `test_cadtocsg.py`): the
`Solidos/`-corpus-scale verification this project's FreeCAD-side history
relies on heavily (`check_sign` end-to-end verification, the MCNP
stochastic volume check, differential Can/TCone/RoundCorner/
MultiRoundCorner/MultiPlane corpus scans) has not been run under the OCC
engine at all -- `test_cadtocsg.py`'s 50-file corpus is `testing/inputSTEP`
only, a narrower, faster fixture set than `Solidos/`. `GEOReverse`
(CsgToCad) remains explicitly out of scope, unchanged. Void generation
and all 4 writers (MCNP/Serpent/PHITS/OpenMC) pass within
`test_cadtocsg.py`'s own assertions but haven't been separately
stress-tested the way the FreeCAD engine's void/writer code has been
throughout this project's history.

## pyOCC migration, Phase 4: `Solidos/` corpus verification (98/99 files),
7 more real bugs found and fixed, and a positive `rev_pipe.stp` full-pipeline
confirmation

Continued directly from Phase 3's `tests/test_cadtocsg.py` parity. Ran the
same `corpus_scan.py` methodology this project has used throughout its
FreeCAD-only history (load -> decompose -> build_solid_definition, count
each composite-surface type) against the full 99-file `Solidos/` corpus
(excluding `Big_model_reserved`, per established convention) under
`GEOUNED_CAD_ENGINE=occ`, diffed against a fresh FreeCAD baseline. Per
explicit user reminder mid-session: FreeCAD and pyOCC can legitimately cut
a solid into different intermediate pieces, so this phase's goal was
*parity of robustness* (no new crashes, `tests/test_cadtocsg.py` staying
50/50), not byte-identical decomposition -- a count difference alone,
without independent CAD-ground-truth verification, is not treated as a
bug.

**Environment/tooling gotcha, worth remembering**: passing a POSIX-style
path (`/c/Users/...`, as produced by Git Bash's own `find`) *embedded
inside a `python -c "..."` string* silently fails with `FileNotFoundError`
-- standard Windows Python's `open()` doesn't understand that path form.
It works fine as a separate shell *argument* to a script (`python
script.py "$SCRATCH/file"`), since Git Bash's MSYS layer auto-converts
paths only at that boundary, not inside an opaque string. Rebuilt the
corpus file list via PowerShell's `Get-ChildItem` instead (also needed
`encoding="utf-8-sig"` on the Python read side, since PowerShell's
`Set-Content -Encoding utf8` still emits a BOM in PowerShell 5.1).

**8 real bugs found and fixed this phase, all via the same discipline as
every prior phase -- real crash/hang first, then traced to a root cause,
never guessed**:

1. **`ShapeUpgrade_UnifySameDomain(..., True, True, True)` (the
   `removeSplitter()` equivalent inside `GSolid.refine()`/`.fix()`) can
   segfault the whole process** -- not just raise, not just run slow --
   on real, valid tangent geometry: confirmed deterministically on
   `Solidos/trier/ConeSphere.stp` (a tangent cone+sphere solid). Isolated
   to the `UnifyEdges` flag specifically (`UnifyFaces` alone gives an
   identical volume on the same case and returns promptly). **Tried
   disabling `UnifyEdges` globally, then reverted it**: doing so avoids
   the segfault but silently changes face topology broadly enough to
   regress 2 real cells in the authoritative `tests/test_cadtocsg.py`
   suite (`cylBox.stp`, `DoubleCylinder/pieza.stp` -- both lose a real
   Can secondary surface). Given that suite is the higher-priority,
   already-committed bar, `UnifyEdges` stays on; `ConeSphere.stp` remains
   a known, unresolved, narrow segfault under the OCC engine specifically
   -- documented in `GSolid.refine()`'s own docstring, not silently
   dropped.
2. **`TopTools_IndexedDataMapOfShapeListOfShape`/`TopTools_ListOfShape`
   don't have `.Extent()`** (both use `.Size()`) -- a bug in
   `_repair_non_manifold_solid`'s own code, written speculatively during
   Phase 1 and never actually exercised until this phase (Phase 1's
   `rev_pipe.stp` go/no-go test never needed the repair path, since
   `BOPAlgo_Splitter` succeeded raw). First fixed only the map's own
   `.Extent()` call (confirmed via direct API testing that a raw
   `TopTools_ListOfShape` legitimately doesn't have `.Extent()` either --
   an earlier isolated API check that "confirmed" `.Extent()` worked was
   itself testing the wrong object). Found and fixed via 2 real corpus
   files (`SCDR_90_piece0_gsplit_tangent_bug.stp`, `double_RC.stp`).
3. **A second, different degenerate case in `cyl_plane_region_conf`**
   (the same historically-fragile function documented at length earlier
   in this file, under "the L1_S23.stp solid 174" and "rc16.stp"
   sections): when a round corner's cylinder is split into 2 pieces
   meeting at a shared seam, the function's `r1`/`r2` reference-point
   sampling (`cyl1.value_at(u1, ...)`/`cyl2.value_at(u2b, ...)`) can land
   *both* points on the shared seam itself rather than each piece's own
   true far boundary, depending on which end of each piece's own
   `ParameterRange` happens to be the seam -- a real, reproducible
   `ZeroDivisionError` on `rc16.stp` (the function's own established
   canary file). Fixed with a self-correcting retry (try the default
   `(u1, u2b)` pairing; if the two points coincide, retry with `(u1b,
   u2)`) rather than guessing which end is "always" correct -- verified
   this doesn't change behavior for any already-working case (the retry
   only fires when the default pairing is already degenerate).
4. **`pick_outer_wire` divides by `len(vertices)` without checking it's
   nonzero** -- a face can legitimately have a wire with zero edges/
   vertices (confirmed on a decomposition fragment in `Solidos/Cans/
   pipe.stp`); fixed by skipping empty wires when picking the outer one
   (an edgeless wire can never meaningfully be "the outer boundary"
   anyway).
5. **`get_can_surfaces` could return an incomplete `surfaces` list (2
   elements instead of the required 3: `cylinder_shell` + exactly 2
   closing ends) without rejecting it**, crashing `build_can_params`'s
   unconditional `cyl_in, sr1, sr2 = cs` unpack. Confirmed via
   `DoubleCylinder/pieza.stp` (already in `tests/test_cadtocsg.py`'s own
   50-file corpus) that this is a real, generic robustness gap -- a Can
   without exactly 2 closing ends was never a valid Can in the first
   place, matching the function's own established "reject cleanly, don't
   crash" pattern used everywhere else in it. This is the concrete case
   the user's mid-session reminder anticipated: OCC's own decomposition
   produced a genuinely different intermediate fragment here (one whose
   cylindrical face has no real end caps in this particular piece) than
   FreeCAD's decomposition does for the same file -- not a bug in the
   fragment itself, just a shape `get_can_surfaces` needed to handle
   gracefully.
6. **`valid_solid` divides by `solid.Area` without checking it's
   nonzero** -- exactly the class of degenerate solid this function
   exists to reject (found immediately after fixing #5, on the very next
   fragment `DoubleCylinder/pieza.stp`'s own decomposition produces).
   Fixed with the same "reject, don't crash" guard.
7. **`build_RCC_params`'s `cylcones[0].Surf.Axis` access assumed a field
   shape that doesn't always hold**: `cylcones[0].Surf` can be a Tier-1
   `CylinderOnlyParams`/`ConeOnlyParams` (`.Axis` directly) or a Tier-2
   `CylinderParams`/`ConeParams` (basic surface + bounding plane(s),
   wrapping its own primitive one level down as a further
   `GeounedSurface` -- itself needing `.Surf.Axis`, not `.Axis`) --
   confirmed via direct instrumentation that which shape applies isn't
   reliably predicted by `cc.Type`'s exact string. Fixed by walking down
   (`.Cylinder`/`.Cone`, unwrapping a nested `GeounedSurface` via `.Surf`
   each time) until a direct `.Axis` is found, rather than assuming one
   fixed shape. Found via `Reversed_Cyl_Cones/cyl_cone.stp` and
   `cylcone_exact_placa3_pos.step` -- both files `get_reversed_cone_
   cylinder`/`build_RCC_params` is *never actually reached for* under
   FreeCAD's own decomposition of the same files (confirmed: FreeCAD
   succeeds on both with real `RevTCone`/`RoundC` counts, never touching
   this code path at all) -- another concrete instance of OCC's
   decomposition legitimately reaching a different, real code branch.
   Verified exact parity after the fix: both files now give
   `RevTCone=1, RoundC=1` under OCC, matching FreeCAD's own values
   precisely.

**Final diff, 98 files compared (`ConeSphere.stp` excluded, its segfault
already fully characterized in point 1)**:
- **8 files with composite-surface-count differences** (`SCDR_90_piece0_
  gsplit_tangent_bug.stp`, `placa.stp`, `PiezaDavid_pieces/piece_0.stp`,
  `TVA_solid0_cell1.stp`, `TVA_solid1_cell2.stp`, `double_RC.stp`,
  `series_solid2_halfcyl_plus_inclined.stp`, `series_solid3_sphere_
  plus_inclined.stp`) -- all attributable to legitimately different
  decomposition paths per the user's own explicit framing; not
  independently verified against CAD ground truth this pass (that deeper
  verification, `check_sign`-style, is out of scope here -- see "Still
  open" below).
- **1 OCC-only crash**: `SCDR_90.stp` (`Courbes non jointives`, the same
  `UnifyEdges` fragility family as `ConeSphere.stp` -- this file is
  already independently known-problematic on the FreeCAD side too, per
  this file's own extensive earlier "SCDR_90.stp" investigation history).
- **2 FreeCAD-only failures, both `rev_pipe.stp`** (`OCCError: FuseEdges
  : Fusion failed`) -- **OCC succeeds on both with a real, non-trivial
  result** (`RoundC=2, MultiRoundC=1`). This is the same file this whole
  migration's Phase 1 go/no-go test was built around (a confirmed
  FreeCAD non-manifold/tangency bug in `piece0`) -- this result extends
  that original, isolated `Gsplit`-only confirmation to the *full*
  decompose+convert pipeline: FreeCAD's own `build_solid_definition`
  path (via `FuseSolid`'s `.fix()` call, matching the `FuseEdges`
  signature) still trips on this file even after Phase 1's findings,
  while OCC's equivalent path does not.
- **2 files failing identically on both engines** (`modelCell_670000.stp`,
  `modelcell_cut1.stp`, different error text but same underlying
  already-known-problematic files) -- not a migration regression.

**Verification**: `tests/geo` (FreeCAD, 106/106) and `tests/geo/
test_occ_impl.py` (OCC, 39/39) confirmed after every one of the 7 code
fixes above; `tests/test_cadtocsg.py` re-run to full completion (not
`-x`) after each round -- final state **50/50 under both engines**,
confirmed twice in a row (once immediately after the `UnifyEdges` revert,
once again after the `build_RCC_params` fix) to be sure neither
introduced a fresh regression.

**Still open, not attempted this pass**: independent CAD-ground-truth
verification (`check_sign`-style, or the MCNP stochastic volume check)
of the 8 count-differing files above, to confirm each is a "different,
equally-valid decomposition" rather than a real classification miss --
this project's own established methodology for that kind of deeper
verification, not yet pointed at the OCC engine. `ConeSphere.stp`'s
segfault and `SCDR_90.stp`'s `UnifyEdges` crash remain unresolved (both
traced to the same root mechanism, deliberately not chased further given
the `tests/test_cadtocsg.py` regression risk already found once).
`GEOReverse` remains explicitly out of scope, unchanged.

## GEOReverse migration: removing the direct FreeCAD dependency, and real
FreeCAD/pyOCC symmetry with GEOUNED's own `geo` package

New branch off `pyocc-migration` (per user request, since this work needs
the current `geo` package). Explicit scope set by the user before
starting: mirror the exact same "remove `import FreeCAD`/`import Part`
from everywhere except one designated chokepoint" discipline this file's
own pyOCC-migration sections document at length for GEOUNED's forward
pipeline — but applied to `GEOReverse` (CsgToCad, the CSG→CAD reverse
pipeline), which had never been touched by any of that work. Also
explicit: GEOReverse relies heavily on split operations like GEOUNED;
GEOReverse has **no** meta-surface/composite-surface concept (no Can/
RoundCorner/TCone equivalent — every surface it builds is either a
primitive `geo` shape or one of 6 "exotic quadric" types GEOUNED never
produces); bugs found while migrating get fixed in a **separate, later**
pass, not silently while porting (the same "port bugs as-is, flag them"
discipline already established for `geo_quadrics.py` in an earlier
session — see below).

### Phase 0–2 (from an earlier session in this same effort): test
hardening, dead-code removal, `geo` extensions, the `_geo_bridge.py`
chokepoint

`tests/test_csgtocad.py`'s 2 tests used to assert only that output files
*exist*; extended with real per-solid volume assertions (same
`1e-6 * max(volume, 1.0)` tolerance style as `GSolid.refine()`'s own
guard) against a captured baseline. Deleted 4 confirmed-dead files
(`buildSolidCell_mod.py`, `buildSolidCell_org.py`, `Utils/BooleanSolids.py`,
`processInp.py` — zero references anywhere, verified by grep). Extended
`geo` itself with what GEOReverse's own algorithms need that GEOUNED's
forward pipeline never did: `GSolid.copy()`/`.transform_geometry(matrix)`,
`GBoundBox.transformed(matrix)`/`.contains_point(point, tolerance)`,
`Gmake_cone_frustum`/`Gmake_cone_double_sheet` (both engines), and
`GPlane.intersect_line` (companion to the already-existing
`GPlane.intersect_plane`, same hybrid-then-native-fallback verification
discipline). Built `GEOReverse/Modules/_geo_bridge.py`, GEOReverse's own
single-import-point module (mirroring `geo/__init__.py`'s role) — at the
time, deliberately hardcoded to `geo._freecad_impl` directly (bypassing
`geo/__init__.py`'s engine switch), reasoning that GEOReverse's hard
`.FCStd`-export dependency meant it could "never be pyOCC-only" — this
reasoning was later revisited and reversed, see "Full FreeCAD/pyOCC
symmetry" below.

### Phase 4: `Utils/boundBox.py` and `Objects.py`, the first real
integration bug, and the "accept both, normalize once" pattern reused

Converted `boundBox.py`'s internal algorithms (`myBox`, `solid_plane_box`,
`plane_intersect`/`plane_boundary`/`line_boundary`, `makePlane`) from
native `FreeCAD.Vector`/`FreeCAD.BoundBox`/`Part.Plane`/`Part.Line` to
`GVector`/`GBoundBox`/`GPlane`/`GLine`, reusing `GPlane.intersect_plane`/
`.intersect_line` instead of duplicating plane/line intersection math a
third time. This alone broke the pipeline immediately once tested for
real (not just in isolation) — `Objects.py` (not yet migrated at this
point) still built native `FreeCAD.BoundBox`/called native
`.getPoint()`/`.getEdge()` on whatever `boundBox.py` handed it, so the
now-`GBoundBox`-typed value it received didn't have those methods.
Traced via a debug reproduction (`geo.build_universe()` on
`cylinder_box.mcnp`, with `buildCAD.py`'s exception-swallowing `except:`
temporarily removed to see the real traceback — restored afterward)
rather than guessing. Fixed with the same "accept both, normalize once"
boundary shim already used throughout the GEOUNED-side migration
(`to_gboundbox()` at the one chokepoint every caller passes through,
`myBox.__init__` at the time) — later removed once every producer
became `GBoundBox`-native (see below).

Rewriting `Objects.py`'s 12 surface classes (`Plane`/`Sphere`/`Cylinder`/
`Cone`/`EllipticCone`/`Hyperboloid`/`Ellipsoid`/`EllipticCylinder`/
`HyperbolicCylinder`/`Paraboloid`/`Torus`/`Box`) to use `GVector`/numpy
instead of native `FreeCAD.Vector`/`FreeCAD.Matrix` surfaced a real,
pervasive, previously-latent bug: every one of these classes' `__init__`
did `if tr: self.transform(tr)`, where `tr` is a transform matrix that,
once `MCNPinput.py`/`XMLinput.py` were also converted, could genuinely be
a numpy array — and `bool(numpy_array)` raises `ValueError: truth value
of an array with more than one element is ambiguous` for anything but a
0/1-element array. All 12 sites fixed to `if tr is not None:`. The same
class of bug recurred in `MCNPinput.py`'s `substituteLikeCell`/
`setExplicitCellDefinition` (`if not c.TRCL:`/`if not c.TR:`, checking
whether a cell *has* a transform after conversion already ran) — fixed
with `isinstance(x, np.ndarray)` checks instead of bare truthiness,
preserving the exact original falsy-or-matrix semantics (a matrix is
never falsy) without ever calling `bool()` on a multi-element array.

Also deleted, once superseded: `Objects.py`'s own module-level
`makeHyperboloid`/`makeHyperbolicCylinder`/`makeEllipticCylinder`/
`makeEllipsoid`/`makeEllipticCone`/`makeParaboloid`/`ortoVect`/`FuseSolid`
(~250 lines) — all already reimplemented, verified, in `geo_quadrics.py`
(built in an earlier session) or the new shared `fuse_solids()` (below).
`Plane.buildShape` itself collapsed to a 3-line delegation to
`boundBox.py`'s own already-existing `makePlane(normal, position, box)`,
eliminating a second, hand-rolled copy of the same polygon-face
construction.

### `FuseSolid` consolidated into `_geo_bridge.py::fuse_solids` (the plan's
Phase 5, done early)

`FuseSolid` was byte-for-byte duplicated 3× (`Objects.py`,
`buildSolidCell.py`, `splitFunction.py`) — since converting all 3 call
sites to `GSolid` was happening anyway in this same pass, consolidated
into one shared `fuse_solids(parts: list[GSolid]) -> GSolid | None` in
`_geo_bridge.py`, using `Gfuse` + `GSolid.refine()` (not a raw, unguarded
`removeSplitter()`) — so every call site now gets `.refine()`'s existing
volume-invariance safety check for free, a real correctness improvement
over all 3 original copies, not just a refactor.

### `splitFunction.py`/`buildSolidCell.py`/`buildCAD.py`: `BOPTools.SplitAPI.slice`
→ `Gsplit`, native `.BoundBox`/`.Volume`/`.isValid()` → `GSolid`'s own

`SplitSolid`'s `BOPTools.SplitAPI.slice(...)` → `Gsplit(...)` — an
intentional behavior improvement flagged in the original plan (`Gsplit`
has internal tolerance-retry robustness the raw native call never had).
`point_inside`/`point_inside_org` (~90 lines of manual point-in-solid
search, `divide_box`) replaced by the already-existing, already-verified
`GSolid.find_interior_point()` — `point_inside_org` itself confirmed
dead (zero callers) and deleted outright, not just superseded.
`surface_side` (the algebraic point-classification fallback, mirroring
GEOUNED's own `check_sign`) needed only mechanical fixes (`.Length` →
`.length`, in-place `.normalize()` → reassigned `.normalized()`) since
its inputs are `GVector` throughout once `Objects.py`'s params became
`GVector`-native — not consolidated with `geo`'s own duplicate `is_inside_*`
formulas this pass (same "not worth it on a hot path" call GEOUNED's own
`check_sign_primitive` made previously). `buildCAD.py::interferencia`
(`BOPTools.SplitAPI.slice`/`.common()`) → `Gsplit`/`Gcommon` +
`fuse_solids`; `BuildUniverseCells`'s own `ContainerCell.CurrentTR`
truthiness checks fixed to `is not None` (same numpy-array class of bug).

### `core.py::export_cad`: engine-independent by design, `format` parameter

User-requested redesign: `export_cad(output_filename="", format="stp")`
— `format` accepts a `str` or `list` (a list exports one file per
format). `export_cad` itself validates the requested format(s) against
`_SUPPORTED_FORMATS[CAD_ENGINE]` before doing any work (clear `ValueError`
listing what's actually supported if not) and dispatches the real
writing through `_EXPORTERS[CAD_ENGINE]` — it contains no engine-specific
code of its own. **Real behavior change, not a bug**: `.FCStd` is no
longer written unconditionally (the original always wrote both a
`.stp`/`.step` *and* a `.FCStd`) — now only produced when `"fcstd"` is
explicitly requested. `tests/test_csgtocad.py` updated to request
`format=["stp", "fcstd"]` explicitly so both output paths stay covered.

### Full FreeCAD/pyOCC symmetry: `_geo_bridge.py` now really is
engine-switchable, mirroring `geo/__init__.py` exactly

User pushback on the Phase 2-era `_geo_bridge.py` design (which hardcoded
`geo._freecad_impl`): "if GEOUNED can be pyOCC-based, GEOReverse should
be able to too — everything should be symmetric FreeCAD/pyOCC." Traced
through what that actually requires, in 3 parts of very different size,
agreed with the user before implementing:

- **Part A (small, done)**: `_geo_bridge.py` changed from
  `from ...geo._freecad_impl import (...)` to `from ...geo import (...)`
  — since GEOReverse's own core pipeline (`Objects.py`/`buildSolidCell.py`/
  `splitFunction.py`/`buildCAD.py`) was, by this point, already fully
  routed through the bridge's re-exported names and touched no native
  type directly, this one import-line change makes the *entire*
  GEOReverse core follow `GEOUNED_CAD_ENGINE` automatically. Required
  filling 2 real gaps in `geo/__init__.py`'s export surface first:
  `GMatrix`/`to_gmatrix` (pure `vector_geometry.py` types, just never
  added to the top-level unconditional import block before) and
  `to_native_matrix` (GMatrix → native matrix; existed in
  `_freecad_impl.py`, missing from `_occ_impl.py` — added, returning a
  `gp_Trsf` built via `SetValues(...)` from the `GMatrix`'s own top-3-rows
  layout, matching how `_occ_impl.py`'s own `GSolid.transform_geometry`
  already expected a `gp_Trsf`, not a general 4×4). Also found and
  deleted `to_fc_boundbox` (renamed `to_native_boundbox` mid-session,
  then deleted outright) as confirmed dead — added to `_freecad_impl.py`
  in an earlier pass for a `Plane.buildShape` call site that no longer
  existed once that method was rewritten to delegate to `makePlane`.
- **Part B (small, done)**: `makeTree` (the FreeCAD document/label-tree
  builder — `App::Part`/`Part::FeaturePython`, no pyOCC equivalent at
  all) moved out of the now-fully-engine-agnostic `buildCAD.py` into a
  new `GEOReverse/Modules/_freecad_impl.py`, alongside `export_freecad`
  (built earlier this session for the `format=` redesign above). New
  sibling `GEOReverse/Modules/_occ_impl.py` (`export_occ`) is a stub —
  raises `NotImplementedError` with a docstring explaining the real gap:
  under pyOCC there's no document/label-tree concept, so a real
  implementation would need XCAF (`XCAFDoc_ShapeTool`/`ColorTool` for
  per-solid names, `STEPCAFControl_Writer` instead of the plain
  `STEPControl_Writer` `Gexport_step` already uses) — not attempted, since
  it needs a real pyOCC session to verify against known volumes/labels,
  same discipline as every other piece of this project's pyOCC work.
  `core.py` now dispatches through `_EXPORTERS`/`_SUPPORTED_FORMATS` built
  from both modules' own exports, with zero `FreeCAD`/`Import` imports of
  its own.
- **Part C (small, done)**: `geo_quadrics.py` (the 6 exotic quadric
  surfaces) restructured from one flat file into a package mirroring
  `geo/__init__.py`'s own dispatch pattern exactly:
  `geo_quadrics/__init__.py` (reads `CAD_ENGINE` from `_geo_bridge.py`,
  re-exports `Gmake_elliptic_cone`/`Gmake_hyperboloid`/`Gmake_ellipsoid`/
  `Gmake_elliptic_cylinder`/`Gmake_hyperbolic_cylinder`/`Gmake_paraboloid`/
  `Gmake_torus_elliptic` from whichever impl resolves), `_freecad_impl.py`
  (the existing dataclass-based implementation, unchanged behavior, now
  wrapped by 7 generic `Gmake_*` free functions — the *only* names the
  package re-exports; the dataclasses themselves are no longer imported
  by `Objects.py` directly), `_occ_impl.py` (a stub, same
  `_not_implemented(name)` pattern `geo/_occ_impl.py` itself already
  established for its own early Phase 1). `Objects.py`'s 6 call sites
  updated from the two-step `GClass.from_values(...).build_shape(...)`
  pattern to a single `Gmake_*(...)` call — the FreeCAD/pyOCC choice is
  now entirely internal to the `geo_quadrics` package, invisible to
  `Objects.py`.

Renamed `to_fc_vector`/`to_fc_matrix` → `to_native_vector`/`to_native_matrix`
throughout `geo` and GEOReverse (user request, for coherence): the old
"fc" prefix was already misleading on the pyOCC side (`_occ_impl.py`'s
own version returned a `gp_Pnt`, not a `FreeCAD.Vector`, with a docstring
apologizing for the mismatch) — the new name describes what the function
actually does on either backend.

### Real bug found via this restructuring: `geo/_freecad_impl.py`'s own
import order (`BOPTools` before `FreeCAD`)

Part A's `_geo_bridge.py` change removed that module's own `import FreeCAD`/
`import Part` (no longer needed once nothing in the file touched them
directly) — which immediately broke a bare `import geouned` with
`ModuleNotFoundError: No module named 'BOPTools'`. Root cause: `geo/
_freecad_impl.py`'s own top-level imports were ordered `import
BOPTools.SplitAPI` *before* `import FreeCAD` — harmless as long as
*something* elsewhere in the import chain happened to import `FreeCAD`
first (which is what actually appends FreeCAD's `Mod/*` subdirectories,
including the one providing `BOPTools`, to `sys.path` — confirmed
directly: `import FreeCAD` alone changes `sys.path` from a handful of
entries to ~30 new `Mod/*` paths). `_geo_bridge.py`'s own now-removed
`import FreeCAD` used to be exactly that "something," purely by
accident of file layout, not by design. `tests/geo`'s own test files
were never exposed to this because they explicitly `pytest.importorskip("FreeCAD")`
before importing anything `geouned`-side — a workaround for the same
latent bug, not evidence it didn't exist. Fixed at the actual root: `geo/
_freecad_impl.py`'s own import order swapped to `FreeCAD`/`Part` before
`BOPTools.SplitAPI`. A real, previously-undiscovered fragility in a file
that's been stable throughout this whole project's history — exposed
only once something changed *elsewhere* removed the accidental workaround.

### Verification

`tests/geo` (FreeCAD, 106/106, 1 pre-existing skip unrelated), `tests/
test_csgtocad.py` (2/2, both `mcnp`/`openmc_xml`, with the real volume
assertions), `tests/test_cadtocsg.py` (GEOUNED's own 50-file forward
regression, 50/50 — confirming the `geo` package additions this whole
effort needed have zero impact on GEOUNED itself) — re-run after every
real milestone throughout, not just at the end, matching this project's
own established discipline. `GEOUNED_CAD_ENGINE=occ` confirmed to reach
the new stub code paths correctly (dispatch resolves, raises the
intended `NotImplementedError` rather than crashing on an import error)
— full end-to-end pyOCC testing not possible from the default shell used
this session (no `pyoccenv` conda environment active here; matches this
project's own established split-environment setup documented earlier in
this file).

### Deferred, explicitly out of scope this pass

- Real pyOCC implementations for `geo_quadrics/_occ_impl.py` and
  `GEOReverse/Modules/_occ_impl.py` (export) — both stubs, both need a
  real `pyoccenv` session to build and verify against known geometry,
  per the user's own explicit instruction to leave them empty for now
  ("por ahora lo dejamos vacío, cuando pasamos realmente a pyocc habrá
  que crearlo").
- The bugs deliberately ported as-is and flagged (not fixed) in
  `geo_quadrics.py` during an earlier session — `GEllipsoid.build_shape()`/
  `GHyperboloid.build_shape(one_sheet=True)`'s native construction
  failures, the `is_inside()` pre-existing sign/typo bugs — untouched by
  this session's restructuring, still living in `geo_quadrics/_freecad_impl.py`
  under the same documented-but-not-fixed status.
- No attempt to independently verify GEOReverse's own output beyond the
  existing `test_csgtocad.py` fixture (`cylinder_box`) — broadening that
  fixture set was explicitly deferred back when `test_csgtocad.py` was
  first hardened, and remains deferred.

### `_geo_bridge.py` retired: GEOReverse now imports `geo` directly, matching
GEOUNED's own pattern

User pushback, next session: having every GEOReverse file funnel through
`_geo_bridge.py` (which mostly just re-exported `geo` names unchanged)
was a real asymmetry against GEOUNED's own style, where every file
imports `from ...geo import (...)` directly and keeps its own
file-local logic alongside that import — there is no equivalent
"GEOUNED_bridge.py" middleman on the forward-pipeline side.

Fix: deleted `_geo_bridge.py`; every former consumer (`Objects.py`,
`MCNPinput.py`, `XMLinput.py`, `buildCAD.py`, `splitFunction.py`,
`Utils/boundBox.py`, `geo_quadrics/__init__.py`, `core.py`) now imports
`geo` names directly, at whatever relative-import depth its own location
needs (`...geo` from `Modules/*.py`, `....geo` from `Modules/Utils/*.py`
and `Modules/geo_quadrics/*.py`, `..geo` from `core.py`). The handful of
names that were genuinely GEOReverse-specific (not `geo` re-exports) —
`to_np_matrix`/`to_gmatrix_from_np`/`transform_solid`/`matrix_multVec`/
`matrix_rotate_vec` (the numpy-transform bridge) and `fuse_solids` — moved
to a new `GEOReverse/Modules/matrix_utils.py`, which itself imports `geo`
directly rather than acting as a second middleman. `Utils/boundBox.py::makePlane`'s
own inline, deferred `from .._geo_bridge import Gmake_polygon_face` (no
real circular-import reason for it being local) was hoisted to the
file's top-level import alongside the rest.

Verified: `tests/geo` (106/106), `tests/test_csgtocad.py` (2/2),
`tests/test_cadtocsg.py` (50/50), plus a bare `import geouned` — all
unaffected, confirming this was a pure import-path reshuffle with zero
behavior change.

### `core.py`'s own remaining asymmetry closed: `cad_export/` package,
mirroring `geo_quadrics/` exactly

User caught one more instance of the same problem: `core.py` still
imported *both* `Modules/_freecad_impl.py` and `Modules/_occ_impl.py`
(the export-side pair) unconditionally at module load time, with its own
`_SUPPORTED_FORMATS`/`_EXPORTERS` dicts doing the `CAD_ENGINE` lookup at
*call* time instead of at *import* time. Harmless today only because
`_occ_impl.py` is a pure-Python stub with no `import OCC` of its own yet
-- the moment it gets a real implementation (needs `import OCC.Core...`
at module level, per that file's own docstring), `core.py` importing it
unconditionally would crash on a FreeCAD-only machine, exactly the
failure mode `geo/__init__.py`'s conditional-import dispatch already
exists to avoid.

Fix: moved `_freecad_impl.py`/`_occ_impl.py` into a new
`GEOReverse/Modules/cad_export/` package with an `__init__.py` dispatcher
-- byte-for-byte the same pattern as `geo_quadrics/__init__.py` (reads
`CAD_ENGINE`, imports *only* the resolved module's `SUPPORTED_FORMATS`/
`export_*`, re-exported under the generic name `export`). `core.py` now
does `from .Modules.cad_export import SUPPORTED_FORMATS, export as
export_cad_engine` and calls `export_cad_engine(...)` directly -- the
`_SUPPORTED_FORMATS`/`_EXPORTERS` dicts are gone entirely, since the
resolution already happened once, at import time, the same as every
other engine-switch point in this project.

Caught one real mistake while building this: `cad_export/__init__.py`'s
own `from ...geo import CAD_ENGINE` (3 dots) was one level too shallow --
`cad_export/` sits at the same depth as `geo_quadrics/`
(`GEOReverse/Modules/cad_export/`), so it needs 4 dots
(`from ....geo import CAD_ENGINE`) to match. Caught immediately by the
same `import geouned`/`tests/test_csgtocad.py` smoke test this whole
migration has leaned on throughout -- `CsgToCad` silently became `None`
(the `try/except ImportError` guard in `geouned/__init__.py` swallowed
the real `ModuleNotFoundError`), a good reminder that a "worked in
isolation" module can still break the guarded top-level import in a way
that only a real `import geouned` surfaces.

Verified: `tests/geo` (106/106), `tests/test_csgtocad.py` (2/2, only
after the depth fix), `tests/test_cadtocsg.py` (50/50).

### GEOReverse's own engine-specific code fully centralized: one
`_freecad_impl.py`/`_occ_impl.py` pair, matching `geo`'s own shape exactly

User's next request, direct: GEOUNED centralizes all its FreeCAD/pyOCC
code in one `_freecad_impl.py`/`_occ_impl.py` pair under `geo/`; GEOReverse
had the same *kind* of code (CAD export, the 6 exotic quadric surfaces)
scattered across two small packages (`cad_export/`, `geo_quadrics/`),
each with their own `_freecad_impl.py`/`_occ_impl.py`. Before doing this,
investigated `geo/_freecad_impl.py` itself for a parallel question --
whether it mixes GEOUNED-only functions in with what GEOReverse actually
uses. It does (confirmed by grepping every `from ...geo import` across
all of GEOReverse): roughly 20 names are genuinely shared
(`GVector`/`GBoundBox`/`GMatrix`/`GPlane`/`GLine`/`GSolid`/the primitive
`Gmake_*` constructors/`Gcommon`/`Gfuse`/`Gsplit`/the native-conversion
pair), against ~25 GEOUNED-only-by-current-usage (the classification
machinery -- `GCylinder`/`GCone`/`GSphere`/`GTorus`/`GCircle`/`GEllipse`/
`GBSpline`/`GEdge`/`GWire`/`GFace`/`GShell`/`Gclassify_surface`/
`_curve`/`pick_outer_wire`, plus `Gload_step`/`_labels`/`Gexport_step`/
`Gfirst_shell`/`Gmake_half_space`/`_shell`/`_wire`/`Gcut`/`Gin_contact`/
`Gdistance`/`kernel_version`). But this split is by *usage*, not
structure -- `Gexport_step`/`GFace` are already earmarked for GEOReverse's
own future pyOCC export, so physically separating `geo/_freecad_impl.py`
now would just drift stale the moment usage shifts, for no consumer
benefit (`geo/__init__.py` already is the one abstraction boundary either
side needs). Recommended against splitting that file; user agreed, scope
narrowed to GEOReverse's own side only.

Centralized `cad_export/`'s and `geo_quadrics/`'s FreeCAD implementations
into one `GEOReverse/Modules/_freecad_impl.py` (two clearly-banner-separated
sections, "CAD export" and "Exotic quadric surfaces" -- no naming
collisions between the two original files, confirmed before merging) and
their pyOCC stubs into one `Modules/_occ_impl.py`. Added `GEOReverse/
Modules/__init__.py` as the dispatcher -- byte-for-byte the same shape as
`geo/__init__.py` (reads `CAD_ENGINE`, imports only the resolved module's
names). `Modules/` had never had an `__init__.py` before (worked as an
implicit Python 3 namespace package) -- giving it one is safe here since
nothing it needs to import (`geo`, its own `_freecad_impl`/`_occ_impl`
siblings) creates a cycle with anything under `Modules/*` that would run
its `__init__.py` first. `core.py` now does `from .Modules import
SUPPORTED_FORMATS, export as export_cad_engine`; `Objects.py` (itself a
submodule of `Modules`) does `from . import Gmake_ellipsoid, ...` --
resolving to the package's own already-initialized namespace, not a
subpackage. Deleted `cad_export/`/`geo_quadrics/` entirely; fixed one
stale `geo_quadrics`/`_geo_bridge.py` reference left in `geouned/__init__.py`'s
own top-level comment from an earlier pass.

Verified: `tests/geo` (106/106), `tests/test_csgtocad.py` (2/2),
`tests/test_cadtocsg.py` (50/50), a bare `import geouned`, and the
`GEOUNED_CAD_ENGINE=occ` dispatch path resolving to the (still-stub)
`_occ_impl.py` correctly.

### GEOReverse genuinely functional under pyOCC: a real `pyoccenv` session,
a broken-MKL environment bug fixed, and the XCAF STEP export implemented

User had a `pyoccenv` conda environment already set up
(`C:\Users\Patrick\Apps\Conda\envs\pyoccenv\python.exe` -- non-standard
location, not on PATH, must be invoked by full path; see the `reference-
pyoccenv-path` memory) but this session's own Bash tool couldn't run it
at all (exit 127, no output, for both `-c` and script-file invocations)
-- same class of issue as FreeCAD's own documented Bash-segfaults-use-
PowerShell gotcha. **PowerShell works.**

**A real, pre-existing environment bug found and fixed before any of
GEOReverse's own code could be tested**: `numpy.linalg.eigh` alone (no
OCC involved at all) crashed the process with an access violation
(`0xC06D007F`) in this `pyoccenv`. Root cause, confirmed via `conda-meta`
inspection: numpy's LAPACK was linked against the environment's `_mkl`
build variant (`libblas`/`liblapack` 3.11.0-8, MKL 2026.1.0) --
suspected conflict between MKL's own threading layer and `occt`
7.9.3/`tbb` 2023.0.0 both live in the same process (`occt` depends on
TBB for parallelism; MKL can also route through TBB). `KMP_DUPLICATE_
LIB_OK=TRUE` did NOT fix it (ruled out the simple "duplicate OpenMP
runtime" case). Fixed by switching the environment's BLAS/LAPACK
provider from MKL to OpenBLAS (`conda install -n pyoccenv -c
conda-forge --override-channels "libblas=*=*openblas"
"liblapack=*=*openblas" "libcblas=*=*openblas"` -- needed
`--override-channels`, the bare command failed since it defaulted to
the `defaults` channel and none of these packages/build variants exist
there). Confirmed fixed directly (`eigh` after `import OCC`, real
values returned) before touching anything else. This was blocking
`MCNPinput.py::gq2cyl`'s own `numpy.linalg.eigh` call specifically --
i.e. any MCNP file with `GQ`/`SQ` (general/simplified-quadric) surfaces,
which `tests/csg_files/cylinder_box.mcnp` (this project's own main
GEOReverse fixture) has 2 of.

**With the environment fixed, confirmed the ENTIRE GEOReverse core
pipeline this session's earlier migration built (`Objects.py`/
`buildSolidCell.py`/`splitFunction.py`/`buildCAD.py`/`MCNPinput.py`/
`matrix_utils.py`) already works correctly against real pyOCC** --
`geo.build_universe()` on `cylinder_box.mcnp` under
`GEOUNED_CAD_ENGINE=occ` produced 4 real cells (`GSolid`s wrapping real
`TopoDS_Compound`/`TopoDS_Solid`), first genuine end-to-end validation
of this whole session's work against actual pyOCC, not just structural/
import-level checks.

**`Modules/_occ_impl.py::export_occ` implemented for real** (was a stub
raising `NotImplementedError`), using OCC's XCAF framework to replicate
`_freecad_impl.py::makeTree`'s exact naming/nesting
(`Universe_{U}_Container_{name}` -> `Material_{mat}_{U}{name}` ->
`Cell_{name}_{MAT}`), built and verified incrementally against the real
`pyoccenv` session rather than written blind (this project's own
established discipline throughout the whole pyOCC migration):
- `TDataStd_Name.Set(label, name)` needs a plain Python `str` -- passing
  an already-constructed `TCollection_ExtendedString` raises `TypeError:
  Wrong number or type of arguments` (SWIG overload resolution doesn't
  like an already-typed argument here, confirmed by testing both forms
  directly).
- `shape_tool.AddShape(shape, False)` on a `TopoDS_Compound` -- the
  common case, since most real `GSolid`s GEOReverse builds via
  `fuse_solids`'s fallback path are compounds, not single
  `TopoDS_Solid`s -- does NOT keep it as one label with N solids inside:
  `Gload_step_labels` reads it back as N separate same-named labels, one
  per solid. Confirmed via a direct write/read-back round trip. A real,
  harmless difference from the FreeCAD path (which keeps a compound as
  one `Part::FeaturePython`/one label) -- every solid stays correctly
  traceable to its cell/material by name, just at finer label
  granularity for compound cells. Not fixed (would need
  `AddComponent`-based manual sub-shape handling); documented in
  `_occ_impl.py`'s own docstring instead.
- The assembly-nesting pattern (`shape_tool.NewShape()` for each
  container level, `shape_tool.AddComponent(parent_label, child_label,
  TopLoc_Location())` to nest) verified with a real 2-level
  Universe->Material->Cell hierarchy, round-tripped through STEP and
  back via the already-verified `geo.Gload_step_labels`, confirming both
  names and full ancestor-chain nesting survive.

**Full end-to-end verification against the real fixture**: `geo.
build_universe()` + `geo.export_cad(format="stp")` under
`GEOUNED_CAD_ENGINE=occ` on both `cylinder_box.mcnp` (4 solids) and
`cylinder_box.xml` (5 solids) produced STEP files whose volumes match
the FreeCAD-engine baseline already recorded in `tests/test_csgtocad.py`
to ~1e-8 relative precision (cross-kernel noise between FreeCAD's own
OCCT build and pythonocc-core's, not a real discrepancy) -- and whose
XCAF labels correctly show the full `Cell_N_0 < Material_0_None0 <
Universe_0_Container_None < <barename>` chain.

**`tests/test_csgtocad.py` made engine-aware** so it's a real, permanent
regression test under either engine rather than a one-off manual check:
reads `CAD_ENGINE`, only requests `"fcstd"` (and only asserts the
`.FCStd` file exists) under `"freecad"`, and reads back the exported
`.stp` via `geo.Gload_step` (already engine-dispatched) instead of the
old FreeCAD-only `Part.Shape().read(...)`. Same `_EXPECTED_VOLUMES`
dict reused for both engines -- confirmed the existing `1e-6` tolerance
already comfortably covers the observed ~1e-8 cross-kernel volume
noise, no separate expected-value set needed. Verified: 2/2 under
`GEOUNED_CAD_ENGINE=occ` (real `pyoccenv` run) and 2/2 under the
default FreeCAD engine, both from the same test file.

**Still not implemented**: the 6 exotic quadric surfaces'
`Gmake_elliptic_cone`/`Gmake_hyperboloid`/etc. under `_occ_impl.py`
remain stubs (unrelated to CAD export, a separate follow-up phase per
this file's own earlier notes).

### GEOReverse STEP export: per-material color (occ engine only), and IGES ruled out

**IGES investigated, dropped.** User asked whether pyOCC could also
support `.igs` export (GEOReverse) / read (GEOUNED), as a quick extra
while searching for the original MCNP file for a stalled CAD round-trip
test (see below). Verified live against `pyoccenv` before writing
anything: `IGESControl_Writer`/`IGESCAFControl_Writer` do NOT preserve
solids through IGES in this OCCT build -- a test box round-tripped as 6
disconnected trimmed-planar-surface entities (IGES types 144/108/142/
102/110, grouped via a type-402 "Group" association), zero shell/solid
topology, confirmed by direct inspection of the raw `.igs` text. Tried
`write.iges.brep.mode=1` (a real, correctly-set static param, confirmed
via `Interface_Static.Items()` after constructing an `IGESControl_Writer`
to trigger the controller's own param registration) -- no Type 186
(Manifold Solid B-Rep) entity ever got written regardless. Tried every
`FromIGES.FixShape.*` solid/shell-reconstruction flag on read -- still
came back as a bare compound of faces. Matches IGES's real-world
reputation as fundamentally a surface/curve exchange format. User's
call: drop it entirely rather than build export-only or a
sewing-based-solid-reconstruction reader ("abortamos el import y export
a igs, era solo un extra si era fácil").

**Per-material color, implemented for the occ engine.** User request:
color every solid in a GEOReverse STEP export by its own `MAT` value, so
cells sharing a material are visually identifiable -- same color for the
same material, one standard default color when there's no material info
to distinguish by (0 or 1 distinct value), otherwise a real palette.
`Modules/_occ_impl.py::_material_colors`/`_build_tree`: colors go on via
`XCAFDoc_DocumentTool.ColorTool().SetColor(cell_label, Quantity_Color(r,
g, b, Quantity_TOC_RGB), XCAFDoc_ColorGen)`, same XCAF tree the STEP
export already builds for naming/nesting. Verified directly against a
real written `.stp`'s raw text (not guessed): `SetColor` on a label
writes real `STYLED_ITEM`/`COLOUR_RGB` (or `DRAUGHTING_PRE_DEFINED_COLOUR`
for an exact primary color) entities. **A real, separate binding quirk
found and left unfixed** (not on this feature's critical path): the
label-based read-side overload,
`color_tool.GetColor(label, XCAFDoc_ColorType, Quantity_Color&)`, raises
a SWIG `TypeError` ("wrong number or type of arguments") despite
matching one of the documented C++ prototypes exactly -- confirmed with a
minimal single-shape reproduction, not just the assembly case. The
shape-based overload, `GetColor(TopoDS_Shape const&, ...)`, works fine.
Nothing in this codebase currently reads colors back, so this was
flagged in `_occ_impl.py`'s own docstring rather than chased further.

Color choice: `_MATERIAL_PALETTE` is matplotlib/D3's "tab10" (10 entries)
-- the standard qualitative palette for maximally-distinguishable
categories across visualization tooling generally, picked over a
"realistic material" palette (steel-gray/copper/brass/...) because `MAT`
is an arbitrary MCNP material ID with no physical-material semantics
GEOUNED actually knows. `_DEFAULT_COLOR = (0.8, 0.8, 0.8)` for the
0-or-1-material case matches FreeCAD's own default `ShapeColor`, chosen
deliberately for continuity with this project's FreeCAD history.
**User follow-up, fixed same session**: the initial version cycled the
10-entry palette (`i % 10`) once material count exceeded 10, silently
colliding two different materials onto one color -- defeated the whole
point for any real model with more than 10 distinct materials (several
of this project's own fixtures have that many). Fixed with
`_extended_color`: golden-angle hue rotation
(`hue = (index * 0.618033988749895) % 1.0`, fixed
saturation/value=0.65/0.85 to stay visually consistent with tab10's own
tone), the standard technique for generating N incrementally-maximally-
distinct colors with no fixed upper bound. Verified with a synthetic
15-material case: 15 distinct colors requested, 15 distinct colors
actually found in the written STEP text, zero collisions.

Verified end to end (both the color feature and the overflow fix):
`tests/test_csgtocad.py` 2/2 under `GEOUNED_CAD_ENGINE=occ` (the fixture
itself only has one real material, `MAT=0`, on all 4 cells -- correctly
exercises the single-value default-gray branch, not the palette) and 2/2
under the default FreeCAD engine (untouched by this change);
`tests/geo/test_occ_impl.py` 39/39; a synthetic multi-material test built
directly against the real `export_occ` (not a mock) confirming 3 cells
across 3 distinct `MAT` values get exactly 3 distinct colors in the
output, with the two same-`MAT` cells sharing one.

**FreeCAD-side color, explicitly out of scope**: per-solid color for the
FreeCAD engine's own STEP export (`_freecad_impl.py::export_freecad`/
`makeTree`) was not attempted -- FreeCAD's shape-color API lives on
`Gui.ViewProvider`, which the user confirmed (from a prior, separate
investigation of their own) has no working headless path: it requires
actually opening the FreeCAD application, not just importing the `App`
module the way this project's whole FreeCAD backend does everywhere
else. Matches this project's existing GEOReverse-export docstring
framing (`.FCStd`/GUI-dependent things stay FreeCAD-only, `_occ_impl.py`
gets the pure-`App`-level features) -- color joins that list.

**Background/parallel work this session**: a real round-trip test
(GEOUNED occ forward + GEOReverse occ reverse) against
`Solidos/Big_model_reserved/hylife-v06.stp` was kicked off in the
background while the color feature above was being built, after an
earlier attempt on `divertor.step` was aborted -- that file turned out to
already be a GEOReverse-reconstructed CAD (not an original design file),
and the user explained such files are known to be geometrically "dirty"
and expected to fail if re-run through GEOUNED's own forward pipeline
(it did: a `ZeroDivisionError` in `cyl_plane_region_conf`, deliberately
NOT investigated per the user's own explicit instruction -- not a new
bug, just invalid input for this kind of test). `hylife-v06.stp`'s
result isn't known yet as of this note.

### Environment notes for next time

- `pyoccenv`'s python: `C:\Users\Patrick\Apps\Conda\envs\pyoccenv\
  python.exe` -- use the PowerShell tool to invoke it, not Bash (Bash
  gives exit 127 with zero output, even for trivial scripts -- not yet
  root-caused, just worked around).
- `conda` itself: `C:\Users\Patrick\Apps\Conda\Scripts\conda.exe` (also
  not on PATH in the Bash shell). `conda install -n pyoccenv ...` needs
  `-c conda-forge --override-channels` explicitly, or the solver
  defaults to the `defaults` channel and fails to find conda-forge-only
  packages (which is everything in this environment: `occt`,
  `pythonocc-core`, `mkl`, etc.).
- FreeCAD and pyOCC still can't coexist in one process (`ImportError:
  Module use of python311.dll conflicts with this version of Python` --
  FreeCAD bundles Python 3.11, `pyoccenv` is Python 3.12) -- always set
  `GEOUNED_CAD_ENGINE=occ` (and use `pyoccenv`'s own python.exe) for any
  pyOCC-side testing, never mix in the same invocation.

### `hylife-v06.stp` solid 17: a real pyOCC hang, root-caused with `py-spy`,
and a real bug found *because of* the fix

Follow-up to the abandoned `divertor.step` round-trip test (already-
GEOReverse-reconstructed CAD, known-dirty, correctly abandoned per the
user's own diagnosis -- see above). Retried the same GEOUNED-forward
round-trip idea against a real, original design file instead:
`Solidos/Big_model_reserved/hylife-v06.stp` (372 solids). Under
`GEOUNED_CAD_ENGINE=occ`, `decompose_solids()` reliably stalled at solid
index 17 -- confirmed genuinely stuck (not just slow) by watching real
elapsed time with no log progress for 15+ minutes, then confirmed the
process was still burning real CPU (`Get-Process` showing continuous CPU
accrual), ruling out a simple deadlock/idle hang.

**New tool for this investigation: `py-spy`**, installed into `pyoccenv`
(`pip install py-spy`) specifically to get a live Python stack trace of a
*running, not-yet-killed* process (`py-spy dump --pid N --locals`) --
this project's first use of live process introspection rather than
after-the-fact log/traceback analysis, and turned out to be decisive:
static reasoning about "where must this be stuck" was wrong more than
once this session (see below), while `py-spy` gave the real answer
directly, repeatedly, including real local-variable values (face
indices/areas/bounding boxes, UV node lists, actual native shape
pointers) at each hang.

**Isolating solid 17 wasn't as simple as re-exporting it once.**
Sequence of findings, each contradicting the previous hypothesis until
verified live:
- Exporting solid 17 alone to its own `.stp` and testing standalone:
  FreeCAD fails fast (~1s) with a genuinely different error
  (`Part.OCCError: Bnd_Box is void` inside `GSolid.refine()`); pyOCC
  succeeds in ~10s. Neither reproduces the original hang -- a real
  reminder (matching this project's own `placa3` precedent) that
  re-exporting a solid through STEP is not a perfectly faithful
  reproduction of "the same geometry read from the original file."
- Loading the *original* multi-solid file with `skip_solids` keeping
  only indices 0-16 (the "first 17", 0-indexed): both engines succeed
  cleanly. This *looked* contradictory (solid 17 should be in there --
  it isn't: indices 0-16 are 17 solids, but the actual hang is at index
  17, the 18th solid, one further than that slice reached) -- resolved
  by direct clarification with the user mid-session, not a real
  contradiction, just an off-by-one in how the slice was described.
- Per-solid instrumentation (print "STARTING solid N" *before* each
  `_decompose_target` call, not just "N completed") pinned the real
  index precisely: solid 17 (0-indexed), confirmed by watching it
  finish quickly for 0-16 and then never print "FINISHED" for 17.
- `py-spy dump` on the live full-model run: stuck inside
  `my_distToshape` (`geo/_occ_impl.py`) -> `BRepAlgoAPI_Common(shape1,
  shape2).Shape()` -- a real native OCC boolean-common construction,
  called from `contiguous_face`/`same_faces`/`merge_same_surface_faces`/
  `closed_cylinder_cone`/`get_can_surfaces` (Can-candidate adjacency
  detection). Locals showed `same_faces`'s own `Couples` list already
  had 28 entries accumulated (`i=0`, comparing against couple #28 when
  it hung) -- an O(n^2) pairwise face-adjacency walk over a face group
  with at least 29 members.
- Temporary debug prints in `contiguous_face` (reverted after use, this
  project's standard discipline) confirmed *why*: dozens of the compared
  faces have areas around 0.0008-0.01 -- residual sliver faces, the
  exact same artifact class already documented extensively in this file
  under "SCDR_90_piece1_roundcorner_badvolume.stp" -- but `same_faces`
  (used by `merge_same_surface_faces`, feeder to Can/RoundCorner
  detection) had never received the `skip_slivers` treatment
  `other_face_edge` already has for exactly this pattern.

**Fix 1 -- sliver filter in `merge_same_surface_faces`**
(`utils/meta_surfaces_utils.py`): exclude any candidate face with
`Area < Tolerances().min_area` from the same-analytic-surface group
*before* the O(n^2) `same_faces` walk, not just from individual
adjacency lookups the way `other_face_edge`'s `skip_slivers` already
did. A sliver's negligible area means dropping it from the merged
group's own membership doesn't change the group's real geometry
(closure angle, corner-plane adjacency) -- it was never going to be a
meaningfully "real" piece of the merged surface anyway. Verified:
`tests/geo` 156/156, `tests/test_cadtocsg.py` 89/89 (occ), no
regressions -- but did NOT fully resolve the hang; `py-spy` on a rerun
found a second, deeper hang.

**Fix 2 -- healing on load, `Gload_step` (occ)**: with fix 1 in place,
`py-spy` found the *next* hang genuinely inside `BOPAlgo_Splitter.Perform()`
itself (`Gsplit`'s own native split algorithm, nested at nested-split
`loop=2`) -- this time not an O(n^2) Python-level pattern at all, the
actual native OCC boolean-split algorithm stuck on this solid's real
geometry. The user's own hypothesis, stated directly and confirmed
before any code was written: "FreeCAD probably has a solid cleanup step
on load that OCC doesn't." Verified true, precisely: FreeCAD's
`Part.Shape().read()` implicitly heals on import; pyOCC's raw
`STEPControl_Reader` does not. A manual test -- `GSolid.fix(1e-6)`
(`ShapeFix_Shape`) applied to the loaded solid *before* decomposition --
resolved the hang completely (volume shift ~0.003%, real cleanup not
corruption; `decompose_solids()` then completed in ~23s). Notably,
`GeounedSolid.__init__` already calls `.refine()`
(`ShapeUpgrade_UnifySameDomain`) on every loaded solid unconditionally --
that alone was *not* sufficient, since `.refine()` has no `ShapeFix_Shape`
step of its own; only `.fix()` does. Wired permanently into
`geo/_occ_impl.py::Gload_step` (every solid healed immediately after
load, before any caller ever sees it) rather than left as a manual,
easy-to-forget step at each call site.

**Timing after the fix, same isolated solid**: FreeCAD 2.3s vs pyOCC
19-23s (~8-10x slower) -- the hang is gone, but a real, large constant-
factor gap remains, investigated next (per direct user pushback: "no
tiene sentido que tarde 8 veces mas... no puede haber tanta diferencia
entre versiones tan próximas de OCCT").

### Why pyOCC is still ~8-10x slower on this solid even after the hang fix:
confirmed root cause, not the one first guessed

**OCCT version difference confirmed but ruled out as the explanation**:
FreeCAD 1.1.1 bundles OCCT 7.8.1 (`FreeCAD.ConfigGet('OCC_VERSION')`);
`pyoccenv` uses OCCT 7.9.3 (`conda list -n pyoccenv | grep occt`) --
genuinely different versions, but the user correctly pushed back that a
minor-version gap this close wouldn't plausibly explain an 8-10x
regression on its own (would be a well-known, documented regression if
so) -- right call; the real cause is architectural, not a version bug.

**`py-spy`'s flamegraph mode** (`py-spy record -o out.svg --rate 200 --
python script.py`) used for the first time this session, on the healed
solid-17 decomposition (~4200 samples over ~20s): no single function
above ~3% of samples -- cost spread broadly across `Gsplit`,
`get_surfaces`, `GSolid`/`GFace`/`GEdge` construction, `makeCan`/
`build_surface`, `remove_solids`. This ruled out "one fixable hot path"
and pointed toward a distributed, per-call overhead difference instead.

**First hypothesis (partially right, not the full story): raw SWIG
binding overhead.** A direct, controlled micro-benchmark (same op count,
both engines): constructing/reading a point 200k times -- pyOCC (`gp_Pnt`)
0.44s vs FreeCAD (`FreeCAD.Vector`) 0.09s, ~4.8x gap. But box construction
(20k) was roughly equal, and exploring 6 faces + computing area (20k) was
actually *faster* under pyOCC (2.03s vs 2.92s) -- so "SWIG is just
slower" doesn't hold uniformly; the comparison itself was also flagged
by the user as not apples-to-apples (confirmed correct: `FreeCAD.Vector`
isn't a wrapped `gp_Pnt` at all, it's FreeCAD's own independent
lightweight `Base::Vector` C++ type, converted to/from real OCCT types
only at the topology boundary -- so this measured two different
implementations, not "the same operation via two bindings").

**Real, verified root cause: bulk vs per-element native calls, not a
binding speed difference in general.** Traced directly from source:
`_freecad_impl.py`'s `GBSpline.__init__` does
`self.Poles = [to_gvector(pole) for pole in native.getPoles()]` -- ONE
native call (`getPoles()`) that loops over every pole *inside* FreeCAD's
own hand-written C++ and hands back a ready Python list of already-
converted `FreeCAD.Vector`s. `_occ_impl.py`'s equivalent did
`[_to_gvector(geom_bspline.Pole(i)) for i in range(1, NbPoles()+1)]` --
N separate `.Pole(i)` SWIG calls from a Python loop, each returning a raw
`gp_Pnt` needing 3 more SWIG calls (`.X()/.Y()/.Z()`) to extract floats:
4N cross-language round trips instead of 1. Confirmed via `py-spy dump`
during a live hang investigation earlier in this same session (caught
mid-execution inside exactly this per-pole extraction loop) -- not
guessed. Tested whether pythonocc-core's own "bulk" alternative
(`Geom_BSplineCurve.Poles(array)`, filling a `TColgp_Array1OfPnt` in one
call) actually helps: **it doesn't** -- 0.39s vs 0.34s for the existing
loop (5000 reps, 25-pole curve) -- because reading the filled array back
out into Python floats still costs the same N `arr.Value(i)` + 3N
`.X()/.Y()/.Z()` round trips; the "bulk" call only saves the N
`.Pole(i)` calls, not the dominant per-element float-extraction cost.
**Conclusion, stated plainly for the user's own "es como si..." framing
and confirmed correct**: FreeCAD ships a hand-written, compiled C++
convenience layer between the raw OCCT API and what Python sees, doing
bulk conversions (array of native points -> Python list) entirely on the
C++ side; pythonocc-core, by design, is a much more literal SWIG
exposure of the *raw* OCCT API with no equivalent convenience layer --
every element-level access genuinely has to cross the Python/C++
boundary on its own, and that cost, multiplied by however many BSpline
poles/vertices/samples a solid's geometry has, is what dominates.

**Three options discussed for a systemic fix, one chosen**: (1) write a
custom C++ convenience layer, mirroring FreeCAD's own approach -- real,
but with the portability/build/maintenance cost the user explicitly
flagged as a concern, and now *two* engines' worth of it; (2) switch to
a different, less-raw OCCT binding (`OCP`, the pybind11-based binding
CadQuery/build123d use) -- a real, legitimate alternative to investigate,
but explicitly flagged as unverified: it's still a fairly direct OCCT
exposure architecturally, not confirmed to have FreeCAD-style bulk
convenience methods, and would need the same empirical verification
discipline as everything else in this project rather than being assumed
better for being newer; (3, chosen) **lazy evaluation** -- stop paying
the bulk-vs-per-element cost *at all* for data that's eagerly computed
but rarely read, the same technique already proven in this exact
project for `GFace.wires()`/`.outer_wire()` (real ~26% suite-time cut,
documented earlier in this file). Grepped every `.Poles` read in
`GEOUNED`: exactly one call site,
`decom_utils_generator.py::spline_wires` -- every other BSpline-classified
`GEdge` built anywhere in the pipeline was paying this cost for data
nothing ever reads.

**Implemented**: `GBSpline.Poles` became a lazy, cached `@property` in
*both* `_freecad_impl.py` and `_occ_impl.py` (symmetric change for
interface consistency between the two backends, even though the benefit
is pyOCC-specific -- FreeCAD's own `getPoles()` is already cheap
regardless of eager/lazy timing). Fully transparent to the one real call
site (`for p in edge.Curve.Poles:`) -- a property reads exactly like the
old plain attribute, no call-site changes needed anywhere, unlike the
`.wires()`/`.outer_wire()` precedent (which changed to explicit method
calls -- not needed here, since `.Poles` is genuinely data, not a
heuristic computation).

**A real, previously-dormant bug found *because* the `Gload_step`
healing fix (above) changed geometry slightly enough to trigger it** --
running the *full* `tests/test_cadtocsg.py` corpus (not just the
hylife-v06.stp reproduction) after landing the healing fix surfaced a
new failure on an unrelated fixture, `placa2.stp`
(`input_step_file5`), that had never failed before:
- First symptom: `UnboundLocalError: cannot access local variable
  'indmax'` in `gen_plane_cone`/`gen_plane_cylinder`
  (`meta_surfaces_utils.py`) -- a hand-rolled "if d < best-so-far" min-
  search loop that never initializes its index variable if the loop body
  never runs (an empty `UVNode_min`/`UVNode_max` -- `tessellate()`
  succeeding, no `RuntimeError`, but returning zero UV nodes on the
  now-slightly-different healed geometry). Fixed by treating an empty
  tessellation result the same as the already-handled
  `except RuntimeError` case (fall back to `ParameterRange`-derived
  corner UV values), in both `gen_plane_cylinder` and `gen_plane_cone`
  (byte-for-byte duplicated blocks).
- That surfaced a second, different failure at the *same* line on a
  full-suite rerun: not an empty list this time, but a loop that
  legitimately never finds any element satisfying `d < dmax`. Replaced
  the hand-rolled loop with `min(range(len(...)), key=...)` in both
  functions -- provably behavior-preserving (same distance metric, same
  first-occurrence-wins tie-break as a strict `<` loop) while
  guaranteeing *some* index always gets picked, since the list is
  already guaranteed non-empty by the first fix.
- That, in turn, surfaced a *third*, real geometric bug -- this one
  actually root-caused with live data rather than patched defensively,
  per the user's own explicit request to look at `gen_plane_cone`
  specifically once it kept surfacing new symptoms: a `ZeroDivisionError`
  in `dir2.cross(dir1).normalized()`, traced with temporary debug prints
  (reverted after use) to `ifacemin == ifacemax == 3` (both search targets
  on the *same* face) with `V1 == V2` exactly. Real cause: `gen_plane_cone`
  compared its wrapped node angles (`nd = twoPimod(node[0])`) against
  *unwrapped* `Umin`/`Umax`, while its sibling `gen_plane_cylinder`
  already wraps both sides of the identical comparison
  (`Uminr = twoPimod(Umin)`, `Umaxr = twoPimod(Umax)`) -- a real,
  previously-dormant omission, not an intentional difference: this
  particular face's native U-parameter range spans past `2*pi` (~7.12 to
  ~11.72 rad), and comparing that raw against an always-wrapped `nd`
  made both the "closest to Umin" and "closest to Umax" searches
  independently converge on the *same* wrong index, so the two sampled
  points ended up collinear with the cone's apex -- a zero-length cross
  product. Fixed by wrapping `Umin`/`Umax` in `gen_plane_cone` the same
  way its sibling already does -- verified against the real captured
  data (not just pattern-matched from the sibling function blindly, the
  kind of mistake this project's own history warns against repeatedly):
  the correctly-wrapped target values land near two genuinely different
  UV nodes (indices ~44 and ~0 in the captured list), no longer
  colliding.

**Final verification**: `tests/geo` 156/156 (FreeCAD), `tests/geo/
test_occ_impl.py` + `tests/test_cadtocsg.py` 89/89 (occ) -- both clean
after all four fixes (sliver filter, `Gload_step` healing, lazy
`GBSpline.Poles`, `gen_plane_cone`'s `twoPimod` wrap) landed together;
solid 17 (hylife-v06.stp), isolated, end to end: 19.5s, success, no
hang, no crash.

**Not yet done**: the ~8-10x pyOCC-vs-FreeCAD constant-factor gap on
this solid is reduced only insofar as the lazy-`Poles` fix removes one
real contributor (unmeasured how much, since the flamegraph showed
distributed cost across many operations, not dominated by pole
extraction alone) -- no attempt yet to quantify the *remaining* gap or
find further lazy-evaluation candidates the same way. `OCP` (option 2
above) not investigated at all. The full 372-solid `hylife-v06.stp`
round-trip (GEOUNED occ forward -> GEOReverse occ reverse, the original
motivating test for this whole investigation) was never actually
completed this session -- only solid 17's own decomposition was
isolated and fixed; picking the full-model run back up (with `minVoidSize=100`
per the user's own instruction for future runs, not the `20` used in
earlier attempts) is the natural next step.

## A third `geo` engine: OCP (pybind11), added alongside FreeCAD and
pythonocc-core -- not a replacement

Direct follow-up to the `_to_gvector`/binding-overhead investigation
above. Once the root cause (pythonocc-core's SWIG bindings costing more
per native call than FreeCAD's own hand-written convenience layer, or
than a pybind11-based binding) was confirmed and OCP was benchmarked as
faster than pythonocc-core on every operation tested -- see that
section's own comparison table -- the user made the call directly: yes,
port to OCP, but **do not delete or overwrite the existing pythonocc-core
implementation.** `geo` already runs FreeCAD and pythonocc-core side by
side with zero incompatibility (`GEOUNED_CAD_ENGINE=freecad`/`"occ"`);
OCP becomes a third, equally independent option --
`GEOUNED_CAD_ENGINE=ocp` -- not a replacement for `"occ"`.

**Status update, same session, after the `splitTolerance` fix below**:
pythonocc-core was initially kept as "legacy" (not actively
re-verified going forward). That framing is now retired -- once the
`splitTolerance` fix put `occ` and `ocp` within ~4% of each other
(1.23s vs 1.18s on the hylife-v06.stp solid-17 case, both now *faster*
than FreeCAD's 2.06s), the user's own call was to develop **both in
parallel going forward**, not treat one as primary: they're both fully
implemented already, cost about the same today, and it's plausible one
will end up handling some future case better than the other (matching
this session's own experience -- OCP consistently won or tied on raw
micro-benchmarks, but the real hylife bottleneck turned out to be
identical on both, an engine-independent tolerance default). Practical
effect: bug fixes and improvements found in one pyOCC-family backend
(e.g. the `GEdge` double-`_linear_props`-call fix below) should be
ported to the other too, not left as a `_occ_impl.py`-only or
`_ocp_impl.py`-only fix -- both `tests/geo/test_occ_impl.py` and
`tests/geo/test_ocp_impl.py` remain live, both-must-pass suites, not
one primary and one best-effort.

This was planned before implementation (`EnterPlanMode`, given the
scope -- a ~1600-line file to port plus a smaller GEOReverse one), after
2 Explore-agent passes cataloged every `OCC.Core` import and method call
across both `_occ_impl.py` files (confirmed: those two files are the
*only* places in the whole codebase that import `OCC.Core` at all --
everything else already goes through `geouned.geo`'s engine-agnostic
dispatch) and a `ocpenv` conda environment
(`python=3.12 ocp`, same OCCT 7.9.3 as `pyoccenv`) was created for live
verification.

### Phase 1: a real go/no-go validation before committing to the full port

Mirrored this whole migration's own original Phase-1 discipline
("prove the riskiest algorithm works on a real hard case before writing
1600 lines against a new binding"): re-derived `rev_pipe.stp`'s real
base solid + an actual solid-splitting tool (a `RoundCorner` candidate
surface, found by running `generic_split`'s real candidate-surface loop
under FreeCAD and exporting the first one that genuinely produced a
2-piece split -- not the specific `MultiRoundCorner` case referenced in
this file's much earlier Phase-1 notes, which no longer reproduces
identically after this session's many intervening fixes; volumes
459117.14 + 53220.47 = 512337.61 is the new FreeCAD ground truth for
this exact base/tool pair), then ran `BOPAlgo_Splitter` directly against
it under OCP.

**First attempt looked like a real failure, and wasn't one.** A minimal,
from-scratch `BOPAlgo_Splitter` script (not going through `geo` at all)
gave only 1 piece (53220 -- the smaller half only), marked invalid, with
real `BOPAlgo_AlertAcquiredSelfIntersection`/`AlertSolidBuilderUnusedFaces`
warnings. Ran the *identical* minimal script against pythonocc-core too
(same OCCT 7.9.3) -- **byte-identical wrong result** -- which correctly
redirected the investigation away from "is OCP broken" and onto "what is
my minimal script missing that the real `Gsplit` does." Two real gaps,
found by diffing against `Gsplit`'s actual source rather than guessing:
(1) the minimal script called `SetFuzzyValue(0.0)` unconditionally, while
real `Gsplit` only calls it `if tolerance:` -- turned out NOT to be the
fix (still wrong with the guard added) but worth noting since it looked
plausible; (2) the real fix: `Gload_step` heals every loaded solid
(`.fix(1e-6)`, this exact session's own earlier finding), and the
minimal script's own `load_solids()` skipped that entirely. Adding the
same `ShapeUpgrade_UnifySameDomain`+`ShapeFix_Shape` healing before the
split fixed it completely: 2 pieces, 459117.21 + 53220.47 = 512337.68,
matching FreeCAD to ~0.01%. Confirmed again through the real `Gsplit`
function once ported (not just the minimal script) -- identical result,
0.018s. **Lesson reinforced, not new**: always verify against the real,
already-working code path, not a hand-rolled reproduction that looks
equivalent -- this project has hit this exact trap before (`gen_plane_cone`
earlier this same session, `placa3`'s stale-outp artifact, others) and
will again.

### Phase 2: the full port, `geo/_ocp_impl.py`

Built as a close structural copy of `_occ_impl.py`, mechanically
retargeted to `OCP.*`, verified live (never guessed) against a series of
small scripts in `ocpenv` before writing the real file. Concrete,
confirmed API differences from pythonocc-core, all load-bearing and
documented in the new file's own module docstring:
- **The `_s`-suffix pattern is real but not universal.** OCP suffixes
  *static/namespace-style utility* methods with `_s`
  (`BRepGProp.SurfaceProperties_s`, `BRepTools.UVBounds_s`,
  `BRep_Tool.Pnt_s`/`.Range_s`/`.Curve_s`/`.Surface_s`/`.IsClosed_s`/
  `.Triangulation_s`, `TopExp.Vertices_s`/`.MapShapesAndAncestors_s`,
  `BRepBndLib.Add_s`/`.AddOptimal_s`, `XCAFApp_Application.GetApplication_s`,
  `XCAFDoc_DocumentTool.ShapeTool_s`/`.ColorTool_s`,
  `XCAFDoc_ShapeTool.IsReference_s`/`.GetReferredShape_s`/`.IsAssembly_s`/
  `.GetComponents_s`/`.IsSimpleShape_s`/`.GetShape_s`) -- but `TopoDS`'s
  shape-casting methods (`TopoDS.Face(x)`/`.Edge(x)`/`.Solid(x)`/etc) are
  *not* suffixed, matching pythonocc-core's own `topods.Face(x)` shape
  exactly, and `XCAFDoc_ShapeTool.GetFreeShapes` is *not* suffixed either
  despite being right next to several that are -- every individual call
  site had to be checked live (`dir(Class)`, then a real call), a blanket
  find-replace would have been wrong in both directions.
- **`BRep_Tool.Curve_s(edge, first, last)` does NOT return `(curve,
  first, last)` as one tuple** the way pythonocc-core's `BRep_Tool.Curve(edge)`
  does -- confirmed live that the `first`/`last` positional arguments are
  required but their *values* are discarded (passing `0.0, 0.0`
  placeholders is safe and returns the real curve). `first`/`last` must
  be read via a separate `BRep_Tool.Range_s(edge)` call, which *does*
  return a clean `(first, last)` tuple on its own. Restored the original
  3-tuple convenience as one new helper, `_edge_curve_and_range()`, so
  every call site that used to do
  `curve_and_range = BRep_Tool.Curve(edge)` needed only that one
  function swapped in, not individual rewrites.
- **No `.DownCast()` in OCP at all** (`Geom_BSplineCurve.DownCast(curve)`
  in the pythonocc-core version) -- confirmed live that pybind11 already
  returns the most-derived Python type directly (a curve that really is
  a `Geom_BSplineCurve` comes back typed as one), unlike SWIG, which
  always returns the statically-declared C++ return type and needs an
  explicit downcast. Every `DownCast` call became just using the value
  directly.
- **`TopExp.Vertices_s(edge, v1, v2)` and `BRep_Tool.Triangulation_s(face,
  loc)` DO mutate their output-parameter objects in place** (pass a
  freshly-constructed `TopoDS_Vertex()`/`TopLoc_Location()`, read it back
  after the call) -- the opposite convention from `Curve_s` above, so
  this had to be checked per-function too, not assumed consistent.
- **`ShapeUpgrade_UnifySameDomain`'s constructor argument order is
  swapped**: OCP is `(aShape, UnifyEdges, UnifyFaces, ConcatBSplines)`
  vs pythonocc-core's `(shape, unify_faces, unify_edges,
  concat_bsplines)`. Both real call sites (`GSolid.fix`, `.refine`) pass
  all-`True` either way (harmless today) but were ported with explicit
  keyword arguments so this can never silently break if that ever
  changes.
- **`kernel_version()` reads `OCP.__version__`**, not `OCC.VERSION` (OCP
  has no top-level `VERSION` attribute) -- confirmed `7.9.3.1`.

**Result: `tests/geo/test_ocp_impl.py` (a new file, copied from
`test_occ_impl.py` with only the engine name/import-guard changed) --
39/39 on the very first run, zero fixes needed.**

### Phase 3: the full `tests/test_cadtocsg.py` corpus surfaced the one
real gap -- `Gload_step_labels`'s XCAF walk

The 50-file corpus failed universally at first (`Load.load_cad` calls
`Gload_step_labels` unconditionally, matching this project's own
earlier-documented finding for pythonocc-core's port) -- but the *whole*
gap turned out to be confined to that one function's XCAF-specific API
calls, fixed one live traceback at a time, each a `_s`-suffix or
string-type correction from the list above:
`XCAFApp_Application.GetApplication_s()`; `TDocStd_Document`/
`app.NewDocument(...)` both need a real `TCollection_ExtendedString`,
not a plain Python `str` (confirmed the opposite of GEOReverse's own
`_occ_impl.py`-documented pythonocc-core quirk, which wanted a plain
`str` and rejected a pre-built `TCollection_ExtendedString` -- the two
bindings disagree in opposite directions on this exact call, so neither
convention could have been assumed from the other); **`TDF_Label` has no
`GetLabelName()` convenience method in OCP at all** (a pythonocc-core-only
addon, not standard OCCT) -- replaced with the real standard pattern,
finding the `TDataStd_Name` attribute directly
(`label.FindAttribute(TDataStd_Name.GetID_s(), name_attr)`) and reading
`name_attr.Get().ToExtString()` (confirmed live: pybind11 converts the
`TCollection_ExtendedString` result to a real Python `str`); the 6
`shape_tool.*` XCAF walk methods (`IsReference`/`GetReferredShape`/
`IsAssembly`/`GetComponents`/`IsSimpleShape`/`GetShape`) all needed the
`_s` suffix. **Result: 50/50 on the second full run**, after fixing only
that one function.

### Phase 4: `GEOReverse/Modules/_ocp_impl.py`

Same structural-copy-then-retarget approach, applied to the smaller
XCAF-export module (`_material_colors`/`_extended_color`/palette logic
carried over byte-for-byte unchanged -- pure Python, no native calls).
Applied every `_s`-suffix/`TCollection_ExtendedString` correction already
confirmed in Phase 3 up front, rather than rediscovering them one at a
time again. `tests/test_csgtocad.py` needed **zero changes at all**
(already fully engine-agnostic, dispatching off `CAD_ENGINE` generically)
-- but hit the *same* MKL/LAPACK `numpy.linalg.eigh` crash this file's
own "MCNP stochastic volume check" section already documented and fixed
for `pyoccenv`, this time in the freshly-created `ocpenv` (same root
cause: MKL-linked LAPACK conflicting with `occt`/`tbb` in-process;
identical fix: `conda install -n ocpenv -c conda-forge
--override-channels libblas=*=*openblas liblapack=*=*openblas
libcblas=*=*openblas`). **Result: 2/2 on the first run after the BLAS
fix**, zero code changes needed in `_ocp_impl.py` itself (the
instance-method calls on `shape_tool`/`color_tool` -- `NewShape`/
`AddComponent`/`AddShape`/`UpdateAssemblies`/`SetColor` -- turned out not
to need the `_s` suffix, correctly guessed by analogy with the
already-confirmed "instance methods don't get suffixed" pattern rather
than re-verified individually first).

### Dispatch: `geo/__init__.py` and `GEOReverse/Modules/__init__.py`

Both gained a third `elif _engine == "ocp": from ._ocp_impl import
(...)` branch, same 40/8-name import lists as the `"occ"` branch,
inserted between the existing `"occ"` branch and the FreeCAD `else` --
at the time this branch was added, the default engine was still
`"freecad"` when `GEOUNED_CAD_ENGINE` was unset, with `"ocp"` a new
explicit opt-in exactly like `"occ"` already was. `geo/__init__.py`'s
own module docstring (previously stale -- it still described
`_occ_impl.py` as "a Phase 1 validation slice... not yet suitable for
real use", long out of date by this point in the project's history) was
corrected at the same time to describe all three engines accurately and
note `"ocp"` as the now-recommended choice for new pyOCC-backed work.

**Update, later session**: the default itself was changed --
`geo/__init__.py`'s `_engine = os.environ.get("GEOUNED_CAD_ENGINE",
"ocp").strip().lower()` now defaults to `"ocp"`, not `"freecad"`, when
the env var is unset (confirmed live: `GEOUNED.geo.CAD_ENGINE` resolves
to `"ocp"` with zero environment configuration in a plain shell).
`"freecad"` and `"occ"` remain available as explicit opt-ins
(`GEOUNED_CAD_ENGINE=freecad` / `=occ`) exactly as before -- only the
unset-variable fallback changed. Every "default engine" reference
elsewhere in this file that predates this change (the pyOCC-migration
Phase 1-4 sections, the OCP go/no-go section, etc.) describes the state
*as of when it was written*, not the current default -- this note is
the authoritative current status.

### Final verification

`tests/geo/test_ocp_impl.py` 39/39, `tests/test_cadtocsg.py` 50/50,
`tests/test_csgtocad.py` 2/2 -- all under `GEOUNED_CAD_ENGINE=ocp` in
`ocpenv`. Re-ran every other engine's own suite too, confirming the
addition is purely additive: `tests/geo` 156/156 + `tests/test_cadtocsg.py`
50/50 under the default FreeCAD engine (2 skips now, one more than
before -- the new `test_ocp_impl.py` correctly self-skipping in a
FreeCAD-resolved process, matching `test_occ_impl.py`'s own established
pattern, not a regression); `tests/geo/test_occ_impl.py` 39/39 +
`tests/test_cadtocsg.py` 50/50 still passing under `GEOUNED_CAD_ENGINE=occ`
in `pyoccenv`, confirmed byte-for-byte untouched
(`git diff src/geouned/geo/_occ_impl.py
src/geouned/GEOReverse/Modules/_occ_impl.py` empty throughout).

**The headline number, reported honestly**: hylife-v06.stp solid 17, the
case that originally motivated this whole investigation -- FreeCAD 2.3s,
pythonocc-core 19.2s, **OCP 18.4s**. OCP is *not* a dramatic win here
(~4% faster than pythonocc-core, nowhere near the 2-6x gap the isolated
`_to_gvector`/point-conversion micro-benchmarks showed) -- consistent
with the earlier flamegraph finding that this solid's cost is spread
broadly across many different operation types (`Gsplit`, `get_surfaces`,
`GSolid`/`GFace`/`GEdge` construction, `makeCan`/`build_surface`,
`remove_solids`), not dominated by point/vector conversion alone, so a
faster binding for *that one specific operation* doesn't translate into
a proportional whole-pipeline speedup. OCP was still worth adding --
strictly faster or equal on every isolated benchmark, a real (if modest)
win here, and a technically cleaner/more actively-maintained binding
than SWIG going forward -- but the numbers don't support "OCP solves the
performance gap" as a claim. Closing the remaining ~8x gap to FreeCAD, if
that's ever pursued further, needs a different lever than the binding
swap alone (see the "reduce the number of calls" / lazy-evaluation
avenues discussed earlier in this same investigation).

### Environment: `ocpenv`

`C:\Users\Patrick\Apps\Conda\envs\ocpenv\python.exe` -- created this
session (`conda create -n ocpenv -c conda-forge --override-channels
python=3.12 ocp`, matching `pyoccenv`'s own OCCT 7.9.3). Same
PowerShell-not-Bash invocation gotcha as `pyoccenv` (untested whether
Bash fails identically here, but no reason to expect otherwise -- used
PowerShell throughout from the start this time). Needed the same
post-creation dependency installs `pyoccenv` also needed
(`tqdm`, `pytest`, plus an editable `pip install -e . --no-deps` of
`geouned` itself) and the identical MKL-to-OpenBLAS fix documented above.

## The ~8-10x hylife-v06.stp gap, root-caused: not the binding at all --
`splitTolerance`'s implicit-zero default, and an OCCT 7.9.x-vs-7.8.1
BOP sensitivity difference

Direct continuation of the OCP-evaluation section above -- the user
pushed back on "OCP is a modest win, gap not fully explained" and asked
for a genuine step-by-step comparison, not another micro-benchmark:
for *every* real split attempt during solid 17's decomposition, compare
the exact solid being cut, the exact cutting surface, and the time, side
by side between FreeCAD and pyOCC.

**Built a small, engine-agnostic instrumentation script** (monkeypatches
`decom_one_generators.generic_split`'s module-level `get_surfaces`/
`Gsplit` names, which Python resolves dynamically at call time -- so the
same patch works identically under either engine, no per-engine code
needed) that logs, per `Gsplit` call: which candidate surface type
triggered it, the base solid's volume/face count, the tool's volume, how
long the call took, and the resulting piece volumes. Ran it against the
identical `hylife_solid17.stp` under both engines.

**The real picture, finally concrete**: only 2 candidate surfaces are
ever tried (`generic_split` is called exactly once, `loop=0` -- never
recurses, since neither candidate ever produces a real split -- an
earlier hypothesis this session that recursion depth explained the cost
was wrong). Both are type `"Can"`.
- **FreeCAD**: candidate 1 -> 1 piece (no-op) in 1.24s; candidate 2 ->
  1 piece in 0.10s. Total: 2.11s. Final: solid stays whole (correctly --
  neither candidate is a real cut).
- **OCP, unmodified**: candidate 1 -> **15 pieces** in **17.2s**
  (`degenerate_case_handled=True`, the non-manifold repair path fired);
  candidate 2 -> 1 piece in 0.22s. Total: 18.3s. Final result: **still
  just 1 piece** -- `remove_solids` correctly discards all 15 fragments
  from candidate 1 as junk, so the *geometric answer* is identical to
  FreeCAD's, just reached at ~9x the cost.

**The 15 "pieces" tell the whole story**: sorted volumes
`[-7267.065, -0.003, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 3990139370.316]` -- 13 of 15 are genuinely near-zero volume,
one is negative (a reversed sliver), and one is the real bulk. OCCT
7.9.3's `BOPAlgo_Splitter` (used by both `occ` and `ocp` -- this is not
an OCP-specific issue, pythonocc-core would show the identical pattern,
same OCCT version) is finding a real, non-fictitious near-tangency in
this candidate tool that FreeCAD's bundled OCCT 7.8.1 doesn't detect at
all -- computing an actual (if junk) 15-fragment split plus running
non-manifold repair on it, all of which then gets thrown away. Directly
the same *class* of problem this whole file's own "Motivating problem"
section opens with (tangency-sensitive BOP behavior differing between
OCCT builds) -- just the opposite direction from the original bug (here
newer OCCT is *more* sensitive and does *extra*, wasted work, rather
than *less* sensitive and silently merging two solids that should split).

**Confirmed it's a tolerance question, not a deeper geometry mismatch**,
by testing the identical base+tool pair through `Gsplit` at explicit
fuzzy tolerances: 0.0/1e-9/1e-8/1e-7 all still gave 15 pieces (14-17s);
1e-6 gave 13 pieces (14.6s); 1e-5 gave 4 pieces (4.8s); **1e-4 gave
exactly 1 piece in 0.2s** -- faster than FreeCAD's own 1.24s for the
same conceptual result. A clean, monotonic transition, not a cliff or a
fluke.

**Root cause, plainly**: `Gsplit`'s own code only calls
`splitter.SetFuzzyValue(tolerance)` when `tolerance` is truthy -- and
`Options.splitTolerance` defaults to `0.0`. Neither engine has ever been
given an explicit fuzzy tolerance by GEOUNED itself; both have always
fallen back to whatever their own kernel's internal default is. FreeCAD
bundles OCCT 7.8.1; both `occ` and `ocp` link OCCT 7.9.3. The two
versions' own internal BOP defaults are not equally forgiving of
near-tangent geometry -- confirmed empirically, not from any OCCT
changelog (none consulted; the *effect* is what was verified, repeatedly,
against real geometry).

**Fix**: `Options.splitTolerance`'s default changed from a fixed `0.0`
to `None`, resolved inside `__init__` per the already-resolved
`geo.CAD_ENGINE`: `0.0` under `freecad` (unchanged), `1e-4` under `occ`/
`ocp` (new). An explicit value passed by the caller (e.g.
`tests/test_cadtocsg.py`'s own `splitTolerance=0`) is never touched --
this only changes what happens when the caller doesn't specify one at
all, matching the existing "sensible per-engine default, explicit
override always wins" pattern already used for `GEOUNED_CAD_ENGINE`
itself. `data_classes.py` (previously with zero dependency on `geo`)
gained one new import, `from ...geo import CAD_ENGINE` -- safe, no
circular-import risk, since `geo` never depends on `GEOUNED`.

**Verified**: `hylife_solid17.stp`'s full decomposition, `Options()`
with no explicit `splitTolerance` at all (the real-world call pattern,
not a hand-tuned test): **1.19s under `ocp`** (was 18.3s), matching the
manually-tuned result exactly. Full 50-file `tests/test_cadtocsg.py`
corpus re-run with `splitTolerance=1e-4` explicitly (a scratch copy of
the test, before landing the default-resolution change) -- 50/50, zero
regressions, and total corpus runtime *unchanged* (~60s either way --
this fixture set doesn't happen to contain hylife's specific pathology,
so the tolerance bump is free insurance for corpus files that don't need
it and a ~15x fix for the one real case found that does). After landing
the actual code change: all three engines' full suites re-verified
green -- FreeCAD 156/156 (2 skips), `occ` 89/89, `ocp` 91/91 (`test_ocp_impl.py`
39 + `test_cadtocsg.py` 50 + `test_csgtocad.py` 2).

**Not yet done**: only tested against `testing/inputSTEP`'s 50-file
corpus, not the larger/more varied `Solidos/` corpus this project's
history otherwise leans on heavily for exactly this kind of
tolerance-sensitive change -- the user's own third option
("verify against Solidos/ before trusting the default") wasn't needed to
land this (the fix is narrowly scoped, engine-conditional, and backward-
compatible for anyone already passing an explicit value), but would add
confidence if this default is ever revisited. `1e-4` itself was not
tuned/swept beyond the single confirmed-good point on this one case
(1e-5 still showed spurious fragments, 1e-4 was clean) -- a smaller
value closer to the transition, or a larger one for more headroom, was
not explored.

## `SCDR_90_piece0_badvolume.stp`: the original motivating tangency bug,
reproduced fresh on a minimal fixture, pyOCC confirmed NOT to fix it,
and a promising-but-unfinished analytic-reconstruction lead

Direct follow-up to the `SCDR_90.stp` investigation earlier in this
file (the section ending "...the same class of bug the eventual pyOCC
migration is meant to address"). User's own question, now that `geo`
has 3 real, working engines: does `Gsplit` under `occ`/`ocp` actually
solve this specific, already-identified tangency case, or not?

**Reproduction**: `Solidos/BadCAD_decomposition/SCDR_90_piece0_badvolume.stp`
(98886.17 mm^3) -- already-decomposed piece from the earlier investigation.
`generic_split`'s first real candidate that should cut it is a `ConeOnly`
surface (candidate #5 in call order, after 2 `Can`-adjacent planes fail
harmlessly). Traced with the same `Gsplit`-wrapping instrumentation used
throughout this file: **`Gsplit` returns exactly 1 solid, volume
unchanged, under FreeCAD AND under `ocp` (`BOPAlgo_Splitter`, OCCT
7.9.3)** -- identical failure on both engines. This directly answers the
question: swapping to pyOCC's own splitter does **not**, by itself, fix
this class of case. `degenerate_case_handled=False` in both -- `Gsplit`
doesn't even recognize this as an ambiguous/retry-worthy split; it
returns a confident, wrong answer.

**Root cause, confirmed precisely** (not assumed): the cutting cone tool
is geometrically **identical** to a real face already on the solid's own
boundary -- faces 1 and 2 of the base solid are literal fragments of the
exact same infinite cone (same Axis, Apex distance 0.000000, SemiAngle
diff 0.000000) the tool is built from. `Gcommon(solid, [tool])` reports
the *entire* solid volume as common (98886.17, matching base exactly);
`Gcut(solid, [tool])` reports 0. Both are wrong: a 5000-point Monte Carlo
sample of real material (`base.is_inside`) against the tool's own
`is_inside()` found 64/1397 real-material points (4.6%) genuinely
**outside** the tool -- a direct, real contradiction with `Gcommon`'s
"fully contained" claim. This is the same failure mode as the original
`SCDR_90.stp` finding (`Gcommon`/`Gcut` on the *whole* model, not this
isolated piece) but now reproduced on a minimal, single-solid fixture
and confirmed identical under a fresh OCCT major version (7.9.3 vs
FreeCAD's bundled 7.8.1) -- so this is not an OCCT-version-specific bug,
it's inherent to how BOP kernels of this whole generation handle an
exactly-coincident cutting surface.

**A second, genuinely new piece of the puzzle, from the user's own
manual FreeCAD-GUI investigation**: attempting the same cut interactively
in FreeCAD, the kernel *can* compute the intersection curve between the
cutting cone and every ordinary planar face bounding the solid, but
*cannot* compute the intersection curve between the cutting cone and
**a different real cone face** also on the solid's boundary -- the face
produced by the *earlier* decomposition step that created this exact
piece in the first place (a coaxial "symmetric cone" cut, not repeated
in this investigation but baked into this fixture's own geometry).
Confirmed by inspecting the solid's own faces: a second cone family
exists (apex Y=-4.4997..., vs. the cutting tool's own apex Y=104.4997),
sharing the *exact same axis line* and, critically, the **same SemiAngle**
(0.785398163397 rad = 45 deg exactly) as the cutting cone, just pointing
in the opposite axial direction. Two coaxial cones with equal half-angle
pointing toward each other have an intersection that is NOT a generically
hard freeform curve -- it is a single, exact circle (elementary
trigonometry: for apex-to-apex axial distance D and shared semi-angle
theta, the crossing radius is D/2, at the axial midpoint between the two
apexes). Computed directly for this case: center at the axial midpoint
(Y=49.9997, matching the axis exactly), radius 54.5. This is very
plausibly *the specific curve OCCT's own general surface-surface
intersection algorithm is failing to resolve numerically* -- an
analytically exact case that a general numerical BOP solver can still
stumble on when the two input surfaces are this exactly coincident/
symmetric, matching this whole file's running theme (exact coincidence,
not near-miss, is what breaks these algorithms).

**First reconstruction attempt (bypassing OCCT's boolean engine via face
partitioning) failed -- superseded, see below.** Added `Gmake_solid(shell)
-> GSolid | None` to all 3 `geo` backends this session (`Part.makeSolid` /
`BRepBuilderAPI_MakeSolid`, mirroring the already-existing `Gmake_shell`).
Classifying each of the solid's own faces into an "inside the tool"/
"outside the tool" group (probing off each face via `GFace.normal_at` +
tiny offset, with a tessellation-sampled fallback since **this solid's
own center of mass is itself outside its material** -- confirmed
genuinely non-convex/non-star-shaped, so a centroid-direction fallback is
unsafe here) and capping each group with the shared coincident cone
face(s) worked for classification, but the resulting solids came out
**invalid** (one with negative volume -- an orientation/sewing mistake).
Paused at this point in an earlier pass of this session; picked back up
and resolved differently, described next.

### Resolution: closed-form circle + a single well-conditioned face
pre-split, retried through the *ordinary* BOP splitter

The user's own question reframed the fix: rather than reconstructing the
whole cut face and sewing two full solids by hand (the failed attempt
above), what if only the *one* genuinely degenerate piece -- the circular
arc where the tool's cone crosses the other, coaxial cone -- gets
resolved analytically, and everything else is left to OCCT's own,
already-working machinery?

Recipe, verified end-to-end in scratchpad before being ported to
production:
1. The circle is exact and closed-form: center = midpoint of the two
   cones' apexes, radius = half their distance, axis = the shared axis
   (already established above).
2. The circle's real, *bounded* arc (not the full 360 degrees) is found
   generically -- not by guessing a fixed face/edge topology -- by
   picking any one point on the theoretical circle, reading its own V
   parameter on the second cone's real surface (`ShapeAnalysis_Surface.
   ValueOfUV`), then walking that face's outer-wire edges and bisecting
   for where each edge's own V crosses that same value. This works for
   any edge curve type (line, circle, BSpline...) and needs no assumption
   about how many boundary edges the face has -- confirmed against this
   fixture's real geometry to sub-micron agreement with the earlier
   hand-picked reference points.
3. **The key simplification, found by testing the idea directly rather
   than assuming it wouldn't work**: instead of building the whole new
   cut face and re-trimming every neighboring face by hand, split *only*
   the second cone's own face at this one arc (`BRepAlgoAPI_Splitter`,
   ordinary and well-conditioned -- no degenerate curve needs discovering
   here, the arc is already known), rebuild `base` as a new solid with
   that one face replaced by its two pieces (`Gmake_shell`/`Gmake_solid`),
   and retry the *plain* `BOPAlgo_Splitter` on this presplit solid against
   the *original, unperturbed* tool. Once the arc already exists as real
   topology, the tool no longer needs to discover it via the degenerate
   solver, and the ordinary 3D split succeeds on its own -- confirmed:
   2 valid solids, volumes summing to the original to ~2e-7 relative
   precision, with zero perturbation of the tool at all.

Two real gotchas, both load-bearing for the production port: (a) an edge
built directly in a surface's own (U,V) parameter space needs
`BRepLib.BuildCurve3d_s(edge)` (`breplib.BuildCurve3d(edge)` under
pythonocc-core) forced before being used as a splitting tool -- otherwise
`BRepAlgoAPI_Splitter` crashes the *process* natively (exit code 5, no
Python traceback), not a catchable exception; (b) an edge lying on two
different surfaces (both cones, here) needs a *separate* edge instance
built with its own pcurve for each surface it's tested against -- the
same 3D edge built for one surface's parametrization is invalid on the
other.

**Also tried and confirmed unhelpful, ruling out the two obvious
alternatives before committing to this approach**: `GeomAPI_IntSS`
called directly on the two cone surfaces (bypassing `BOPAlgo_Splitter`'s
own internal dispatch) "succeeds" (`IsDone=True`) but silently returns a
*wrong* curve -- confined to a single meridian plane, oscillating between
the two apexes, touching the real circle only at 2 isolated points --
confirming the degeneracy is inherent to the coaxial-equal-angle
configuration itself, not specific to which OCCT routine is asked to
resolve it. Patching OCCT's own C++ solver was explicitly ruled out of
scope (would mean maintaining a permanent OCCT fork).

### Ported to production: `Gsplit`'s coaxial-cone fallback

Added to `geo/_occ_impl.py` and `geo/_ocp_impl.py` (FreeCAD not
attempted -- no equivalent for the raw `Geom2d_Line`/`ShapeAnalysis_
Surface.ValueOfUV`/`BRepLib.BuildCurve3d_s` calls this needs; left as
documented future work). New pure-math predicate
`vector_geometry.is_coaxial_cone_pair(cone1, cone2, ...)` -- same axis
*line* (parallel or anti-parallel direction both count, per a real
correction from the user: this fixture's own tool/other-cone axes point
in opposite directions) and same `|SemiAngle|`, but a *different* apex
(the complement of the already-existing `is_same_cone_surface`, which
requires matching apex and axis direction both).

Per explicit user redesign (the initial version tried the generic BOP
split first and only fell back to this on a suspicious "1 unchanged
solid" result): `Gsplit` now checks *up front* whether `tool` has a cone
face at all (`_find_cone_face`, a cheap loop, true for only a small
fraction of real calls) -- if so, `_try_coaxial_cone_split` is tried
*before* the generic path, avoiding wastefully running `BOPAlgo_Splitter`
once on geometry already known to defeat it and then again after the
presplit fix. `_try_coaxial_cone_split` groups `base`'s own cone faces by
`is_coaxial_cone_pair`-candidacy (further grouped by shared exact
surface, since a solid can have the "other cone" split into several
fragments by an earlier cut), and for each candidate runs the recipe
above. Finding a coaxial-cone pair is a **candidate, not a guarantee** --
confirmed by the user with a real counterexample (the first cut of the
un-decomposed `SCDR_90.stp` has this kind of coincidental, irrelevant
match elsewhere in the model, where the generic split already works
fine) -- so every step that doesn't pan out (not exactly 2 arc crossings,
the face doesn't actually split, the presplit solid doesn't rebuild
validly, or the retried split doesn't produce a volume-conserving
multi-solid result within `1e-6 * max(|base.Volume|, 1.0)`, matching
`GSolid.refine()`'s own existing tolerance) falls through silently to
today's unchanged-solid behavior -- this fix can only ever help or be a
no-op, never make an already-working case worse.

Verified: `tests/geo` (156/156 FreeCAD, 39/39 occ, 39/39 ocp) and
`tests/test_cadtocsg.py` (50/50 under all 3 engines); a 100-file
`Solidos/` corpus differential scan (excluding `Big_model_reserved`) --
**only the 2 targeted files change** (`SCDR_90_piece0_badvolume.stp`
1->2 pieces, `SCDR_90_piece1_badvolume.stp` 1->2 pieces; the full,
un-decomposed `SCDR_90.stp` itself goes from 5 to 7 pieces as a direct
consequence), every other file byte-for-byte identical including the 2
already-known-broken files (`Chapuza_mal_construido/origSolid_45.stp`,
`trier/ConeSphere.stp`, both fail identically before/after); a real MCNP
stochastic volume check (`volSDEF=True`, via d1suned) on both fixed
pieces in isolation: `piece0` 0.28 sigma, `piece1` 1.35 sigma -- clean.

### `SCDR_90_piece4_lost_particles.stp`: a second, unrelated, real bug
found by isolating d1suned per decomposed piece of the full `SCDR_90.stp`

With the coaxial-cone fix landed, decomposing the *full* `SCDR_90.stp`
(now 7 pieces, was 5) and running the MCNP stochastic volume check
end-to-end hit d1suned's lost-particle abort (10 lost, "no cell found in
subroutine newcel") within the first ~500 histories, at any requested
NPS -- confirmed, by running the *pre-fix* 5-piece version through the
identical check, that this lost-particle gap is **pre-existing and
unrelated to the coaxial-cone fix**: both the 5-piece and 7-piece
versions lose exactly 10 particles at nearly identical history numbers.

Per the user's own direction ("haz un chequeo uno por uno de cada
componente"), isolated each of the 7 decomposed pieces individually
(`meta_list = [GeounedSolid(0, [piece])]`, skipping `decompose_solids()`,
the same technique used throughout this file's history) and ran d1suned
on each: 6 of 7 clean (0 lost particles, 0.04-1.6 sigma), and **piece4**
(the largest, Volume=132923.39) reproduced the exact symptom in
isolation (10 lost particles). Exported as
`Solidos/lost_particles/SCDR_90_piece4_lost_particles.stp`.

**Root cause, found by inspecting piece4's own written boolean
definition directly** (`geo.Surfaces["RevCC"]`, `.Surf.PlaneSeq`/
`.AddPlanes`): piece4's CSG is `many_AND_planes AND multiplane AND
(reversed_cone AND (plane2 OR plane4 OR plane5))` -- a
`ReversedConeCylinder` (RevCC) whose one cone segment closes against 3
"additional" boundary planes. Of the 3, `plane4` is synthetic (no
matching real face on the solid -- confirmed by tolerance-matching every
candidate plane's Axis/Position against piece4's own faces; the other 2
*do* match real faces). Per the user's direct correction, mid-
investigation, of two side-hypotheses that turned out to be red herrings
(the plane's own sign/position, and a `check_sign` dispatch gap for
`"ReversedConeCylinder"` -- real, confirmed, but not the cause: RevCC
never cuts anything during decomposition, so `check_sign` never needing
to classify one is by design, not an oversight): 20000-point Monte Carlo
sampling of piece4's real geometry (`is_inside()`) against the written
expression showed the OR-combination of the 3 additional planes produces
45 false positives (points wrongly classified as material) out of ~19960
relevant samples; switching that single OR to AND (per the user's exact
instruction: "la combinación del RevCC con el plano tiene que ser AND")
eliminated all 45, at the cost of 6 new false negatives (99.78% ->
99.97% match) -- a real, substantial improvement, confirmed empirically
before being applied, not guessed.

**Fix**: `MetaSurfacesDict.add_reversedCC` (`geouned_classes.py`) -- the
loop building `plane_region` from `reversedCC.Surf.AddPlanes` used
`BoolSurface.add` (OR) unconditionally; changed to `BoolSurface.mult`
(AND), one line. `PlaneSeq`'s own existing Forward/Reversed AND-vs-OR
logic (`build_RCC_params`) is untouched -- this was specifically about
`AddPlanes` always defaulting to OR regardless of the group's own
orientation.

Verified: piece4 in isolation, d1suned, 0 lost particles (was 10), tally
0.99507 +/- 0.30% (1.65 sigma, was un-measurable -- aborted before any
useful statistics). Full `tests/geo` + `tests/test_cadtocsg.py`, all 3
engines, green. A second 100-file `Solidos/` corpus differential scan --
**zero count differences anywhere** (this fix only changes AND/OR
*structure*, invisible to a composite-surface-count scan); the one
already-broken file (`modelCell_670000.stp`) merely flips which failure
mode it hits first (timeout vs. Python recursion limit), still broken
either way, not a new regression.

**Closing verification**: the full, un-decomposed `SCDR_90.stp` (both
fixes applied), MCNP stochastic volume check, 1e8 histories: **zero lost
particles** (previously aborted every attempt within ~500 histories) and
a tally of 0.999346 +/- 0.02% (**~3.3 sigma** -- right at this project's
own >3-sigma "real failure" threshold, but a night-and-day change from
the original, long-documented 35-sigma failure at 0.91941 -- volume
accuracy went from ~92% to ~99.93% of the true CAD volume). Whatever
tiny residual this represents is a plausible candidate for a future
session, but the two real, confirmed bugs behind the original failure
(the coaxial-cone `Gsplit` degeneracy and the RevCC `AddPlanes` OR/AND
bug) are both fixed and verified.

### `Big_one_cell/modelCell_670000.stp` piece59: `convex_planes` was a red
herring, the real bug is a razor-thin AND/OR sign in `cyl_plane_region_conf`

User request: investigate why `Solidos/Big_one_cell/modelCell_670000.stp`'s
piece59 (3 structurally identical R=6 round-corner cylinders) shows up as
`MultiRoundC:1, RevCC:1` instead of `RoundC:3` -- i.e. why 2 of the 3
cylinders get wrongly merged into one MultiRoundCorner (with an OR-combined
plane pair) while the 3rd is left as a standalone RevCC, when the correct
answer (confirmed by the user from the raw geometry, independent of any
code) is 3 separate RoundCorners.

**First hypothesis, built and fully validated, then reverted at the user's
explicit request once it turned out to be the wrong target**: `convex_planes`
(`functions.py`) -- the function deciding whether a MultiRoundCorner's own
group of bounding planes forms a coherent AND-combined convex corner or an
OR-combined open one -- was a rotational-consistency heuristic (sort plane
positions angularly, check axes turn one consistent way), confirmed wrong
in general (returns "convex" for provably unbounded plane sets). Built and
validated, end to end, a real general-purpose replacement: a Sutherland-
Hodgman convex-polyhedron-from-halfspaces clipper
(`convex_polyhedron_from_halfspaces`, initially added to `geo/vector_geometry.py`
as pure math with no GEOUNED coupling, per the user's explicit correction
that the *general* function -- not a GEOUNED-specific inlined copy -- was
what they'd asked for), verified against synthetic cases (cube, octant,
tetrahedron, all exact) and 20 randomized 6-plane cases cross-checked
against real CAD (OCP `BRepAlgoAPI_Common`, 14/14 bounded cases exact once
the comparison box was sized large enough not to itself clip the true
polyhedron -- the same "too-small comparison box" pitfall documented
elsewhere in this file for the earlier `RevCC` random-case validation).

**Then the user pointed out the real, structural defect this whole
sub-investigation had missed**: `convex_planes` was never actually reaching
its own new (correct) logic for piece59 at all -- `build_roundC_params`'s
own plane list, after deduplication, collapses to only 2 unique planes for
this file (not 3+), tripping a `len(plane_list) < 3` early-return shortcut
that always returns `True` unconditionally. So the whole `convex_planes`
investigation, however correct in isolation, was chasing the wrong target.
**Per explicit user instruction, this entire piece of work was reverted**
(`git checkout`/manual revert back to the original rotational-heuristic
`convex_planes`, and the new `vector_geometry.py` function removed) --
"volvamos al inicio de este problema."

**The real root cause, found by tracing `get_roundcorner_surfaces`
directly**: the 3rd cylinder (face Index 2) is rejected *before* any
grouping/convexity logic ever runs, at `cyl_plane_region_conf`'s own
`AND_p1_cyl`/`AND_p2_cyl` sign test (`basic_functions_part1.py`... no,
`meta_surfaces_utils.py`). This is a razor-thin case: `cross1 =
n1.cross(nc1)` (the cross product deciding the AND/OR sign between the
cylinder and its own corner plane) comes out with magnitude ~4.5e-05 for
this specific plane pairing -- comfortably *above* the function's existing
degenerate-tangency guard (`cross1.length < 1e-8`, which would trust a
default `True`) but still numerically meaningless: at this near-exact
tangency, the plane's own "inside/outside" status as a function of angle
around the cylinder is a sinusoid that *touches* zero at an extremum
rather than *crossing* it transversally (confirmed directly: sweeping the
angle ±30 degrees around the sampled tangency point showed the plane
stayed on one side for the entire arc except a single point, ruling out
any nearby first-order sign transition to lock onto).

**User-guided fix**: rather than trying to patch the analytic sign formula
further (this exact function's fragility around near-tangency has its own
long, already-documented history in this file -- L1_S23 solid174, rc16),
add a real-geometry point-sampling fallback, triggered only when
`cross1`/`cross2`'s magnitude falls in the newly-identified fragile band
(`1e-8` to `1e-3`, comfortably bracketing the observed ~4.5e-05 without
touching the already-robust ~0.85-0.95 magnitudes seen on well-conditioned
pairs elsewhere in the same file): `_and_or_by_material_sampling`
(`meta_surfaces_utils.py`) samples ~600 random points in a local box
around the cylinder (outside its radius, on the material side of the
*other* corner plane, whose own sign resolution doesn't depend on the
fragile cross product at all), and compares real solid membership
(`solid.is_inside`) conditioned on whether each point is also on the
material side of the plane being tested -- an AND relationship shows near-
zero material when that condition fails; an OR relationship shows
comparable material whether it holds or not. Validated directly against
the user's own manually-derived ground truth for piece59's 3 corners (6
cylinder-plane pairs, AND/OR dictated by the user from the real geometry
independently of any code) -- the sampling test matched all 6, including
the 4 that the analytic formula alone got wrong (3 wrong, 1 right by
chance) and the 2 well-conditioned ones the analytic formula already had
right. Required threading the enclosing solid (`SolidGu`/`GSolid`, needed
for `is_inside`) down through `get_roundCorner`/`next_roundCorner`/
`get_roundcorner_surfaces`/`cyl_plane_region_conf` -- all 4 already had it
available at their own call sites, just weren't passing it through.

Verified: piece59 now gives `RoundC:3, MultiRoundC:0, RevCC:0` (matching
the user's own independent determination); `tests/geo` (78 under ocp) +
`tests/test_cadtocsg.py` (50 under ocp) green; a real d1suned run
(`volSDEF=True`, full void generation) on the actual translated piece59:
**0 lost particles**, tally `0.9888 +/- 0.65%` (~1.7 sigma), all 10
statistical checks passed.

### Regression discovered mid-corpus-scan on `cyl_cone.stp`: an earlier
same-session fix (`_is_straight_edge`) was fundamentally wrong for the
general near-parallel case, replaced with a real topological criterion

The 482-file `Solidos/` differential corpus scan run after the piece59 fix
(0 new failures, 52 files with expected composite-surface-count shifts)
included `Reversed_Cyl_Cones/cyl_cone.stp` -- the very fixture this file's
own "ReversedConeCylinder's AND/OR grouping" section (above) had already
used to validate a real, committed fix. The user immediately flagged this:
"¡pero esto ya lo había arreglado!" -- prompting an MCNP re-check that
confirmed a real regression: cell 1's tally, previously `0.996547`, now
came back exactly `0.0` (the identical symptom the original fix had
resolved).

**Root cause, isolated by selectively disabling each of this session's own
prior changes in turn**: not the `cyl_plane_region_conf` sampling fallback
(disabling it left `RevCC` unchanged at the wrong count) -- it was
`_is_straight_edge`, a guard added *earlier in this same session* (for an
unrelated fix, chaining a `hylife-v06.stp` cylinder pair with genuinely
perpendicular axes into one wrongly-merged RevCC group) that required the
edge joining two chain segments to be a straight line or a collinear-pole
BSpline. Forcing it to always return `True` restored `cyl_cone.stp`'s
correct `RevCC:4`.

**Why the premise was wrong, established empirically rather than assumed**
-- a deliberate real-CAD experiment, at the user's own direction, building
two overlapping cylinders (R=2, axes 2cm apart, H=8) and rotating one axis
relative to the other around two different perpendicular pivots:
- Rotating so the two axes stay *coplanar* (intersecting when extended):
  the shared boundary is `GLine` only at exactly 0 degrees; for *any*
  nonzero tilt (1 to 90 degrees tested) it's an exact `GEllipse` -- OCC
  computes cylinder-cylinder intersections as exact conics when the axes
  are coplanar, never a BSpline approximation. (A genuine topological
  transition was also found and fully characterized here, between 28 and
  30 degrees for H=8 -- the growing ellipse starts intersecting the
  cylinders' own flat end caps; confirmed as a pure finite-height artifact,
  not a property of the underlying analytic intersection, by showing the
  critical angle scales as `~230/H` degrees across H=8/16/32/64.)
- Rotating so the two axes go *skew* (the general, most realistic case for
  real near-parallel RevCC data, since two independently-placed axes in a
  real 3D model are essentially never exactly coplanar): the shared
  boundary is a genuine `GBSpline` for *every* nonzero angle tested (1 to
  90 degrees) -- confirming the real tangency/intersection curve between
  two near-parallel (not coaxial) cylinder or cylinder/cone faces is
  intrinsically curved, never straight, in the case that actually matters.

So `_is_straight_edge` rejected every legitimate skew-axis chain junction
it was ever tested against (`cyl_cone.stp`'s own real chain, confirmed via
direct topology inspection: the edge joining its cylinder and cone pieces
is a real `GBSpline`) while never being the thing that actually
distinguished `hylife-v06.stp`'s genuine bad match from a good one in the
first place.

**Finding the real discriminator -- three more hypotheses tried and
confirmed NOT to work, checked directly against both cases' real face
topology (not synthetic geometry) before landing on the one that does**:
1. Exact vertex convergence -- do the two faces' own "other" boundary
   edges (the ones touching each endpoint of the shared edge) meet at a
   common far vertex? Fails on `cyl_cone.stp`'s own genuine chain too
   (real trim boundaries from independent CAD history don't coincide even
   for a legitimate pair) -- confirmed by direct inspection of the real
   edge/vertex data, not assumption.
2. Same-neighbor-face -- do the two faces' own other-edges lead to the
   same real neighboring face? Matches on *both* the good (`cyl_cone.stp`)
   and the bad (`hylife-v06.stp`) case alike -- not discriminating.
3. (User's own diagnosis, found by inspecting the bad case directly rather
   than guessing further): on `hylife-v06.stp`'s bad match specifically,
   one of the two faces (the R=5170 cylinder) has only 2 edges total for
   the whole face -- meaning the edge touching one endpoint of the shared
   boundary and the edge touching the other endpoint are literally the
   *same* edge (a degenerate 2-edge "bigon" face), not two distinct edges
   as an ordinary quad-shaped chain-junction face would have. Confirmed
   directly: `cyl_cone.stp`'s own genuine chain has 2 *distinct* edges at
   each end on both faces; `hylife-v06.stp`'s bad match has the same edge
   at both ends on the R=5170 face.

**Fix**: `_is_straight_edge` replaced by `_valid_chain_junction`
(`meta_surfaces_utils.py`) -- for each of the two candidate faces, find
the edge (other than the shared one) touching each of the shared edge's 2
endpoints; reject the junction if, on either face, those two edges turn
out to be the same edge object (`is_same`). Purely topological, no curve-
shape or tolerance-sensitive position comparison involved.

**A second, independent real bug found and fixed while root-causing this**:
the `emin`/`emax` nearest-boundary-edge search in `get_join_cone_cyl` compares
each candidate edge's own midpoint parameter `u` against the group's own
`umin`/`umax` (already reduced into `[0, 2*pi)` via `twoPimod`) using a
"wraparound-aware" correction, `d = min(d, twoPi - d)` -- but `u` itself
was never reduced the same way first, and a face's own raw `ParameterRange`
can genuinely exceed `2*pi` (confirmed live: `cyl_cone.stp`'s own face9,
`ParameterRange=(5.284, 9.281, ...)`, the second bound past `2*pi`). When
`u` isn't reduced, `d` can itself exceed `2*pi`, making `twoPi - d` go
*negative* -- and since `min()` then picks that negative pseudo-distance
as the smallest value in the comparison regardless of the true angular
distance, the search silently locks onto the wrong candidate edge
(confirmed numerically on this exact face: the true closest edge, at true
distance 0.012 rad, lost to an unrelated one that produced a bogus -1.14
"distance"). This is what caused `get_join_cone_cyl`'s own adjacency
search to be asymmetric on one of `cyl_cone.stp`'s 3 chains: cylinder-to-
cone found the connection correctly, but cone-to-cylinder (searching from
the cone's own, past-`2*pi` parameter range) silently landed on an
unrelated real plane instead, fragmenting a 3-element chain into 2. Fixed
by reducing `u` via `twoPimod(u)` before computing `d`, in both the
`umin` and `umax` search loops.

**A third, separate gap found by the user testing a different real
file** (`Solidos/lost_particles/modelcell_cut1_piece51_lost_particles.stp`):
two genuinely perpendicular-axis cylinders passed `_valid_chain_junction`'s
topological test (both had 2 distinct edges per end) but still aren't a
real chain member -- confirming the topological test alone isn't
sufficient on its own. Fixed by adding a direct floor on the axis
alignment: `abs(face.Surface.Axis.dot(adjacent.Surface.Axis)) > 0.1`
(permissive -- rejects only the last ~6 degrees approaching exactly
perpendicular, not a tight "must be near-parallel" bound), alongside
`_valid_chain_junction`, not replacing it.

**Verification, each fix isolated and confirmed via real d1suned runs
against the specific fixture that motivated it**: `cyl_cone.stp` (the
`_valid_chain_junction` + `twoPimod` fixes together) -- `RevCC` back to
the correct 4 (was 9 immediately after the regression, briefly 4-but-still-
wrong-content once only `_valid_chain_junction` was fixed, matching the
historical formula byte-for-byte only once the `twoPimod` fix landed too)
-- tally `0.996542 +/- 0.29%`, exactly reproducing the originally-documented
`0.996547`, 0 lost particles. `modelcell_cut1_piece51_lost_particles.stp`
(the axis-dot-product floor) -- `RevCC` from 1 wrongly-merged group to 2
correctly-separate ones, tally `0.998693 +/- 0.45%`, 0 lost particles --
this was one of the original 6 lost-particle files identified at the very
start of this investigation. Full `tests/geo` (78 ocp / 156+2skip freecad)
+ `tests/test_cadtocsg.py` (50 ocp / 50 freecad) green throughout. A final
482-file `Solidos/` differential scan (excluding `Big_model_reserved`)
against the pre-session git HEAD: 0 new failures (one file,
`modelCell_670000.stp`, needed longer than the scan driver's own 180s
per-file timeout -- confirmed via a direct, untimed rerun to be a genuine
~1.85x slowdown from the added real-geometry checks, not a hang: 261s,
correct/consistent result) and 39 files with composite-surface-count
shifts, all consistent with RevCC chains now grouping/ungrouping
correctly.

**Deferred, not yet done**: the `AdjacentMultiplanePlanes` OR-escape
mechanism (limiting a RevCC's own additional plane to its local side when
a real MultiPlane component plane borders it -- see the section above)
was built and wired up for RevCC only. The user explicitly flagged, right
before the commit closing this session's work, that the same mechanism
needs extending to MultiRoundCorner too -- not yet started; a memory note
(`project_mrc_adjacent_multiplane_pending.md`) tracks this for the next
session.

### Open-items audit, 2026-08-21: compiled from the file's full history, then
re-verified point by point against the current code/live runs

User request, right after the RevCC chain-continuation fixes above: compile
every "not yet done"/"deferred"/"still open" item scattered across this
file's entire history into one list, then -- per explicit user pushback
("creo que hay cosas que ya hemos solucionado") -- actually re-check each
one against the current codebase or a live run rather than trusting the
(often stale) prose each item was originally written under. Several turned
out to already be fixed, silently, as a side effect of later unrelated
work; one turned out to be only partially fixed; none were made worse.
This section is the authoritative status as of this date -- prefer it over
any single older section's own "not yet done" framing above.

**Confirmed fixed (contradicts earlier "not yet done" text in this
file)**:
- **`SCDR_90.stp`'s `UnifyEdges` crash** (flagged repeatedly through the
  pyOCC migration Phase 4 section and later) -- re-ran directly under
  `ocp`: decomposes and translates cleanly (`RevCC:4`), no crash. Likely
  fixed as a side effect of the coaxial-cone `Gsplit` fallback or one of
  the many later `meta_surfaces_utils.py` changes; not root-caused to a
  specific commit, just confirmed no longer reproducing.
- **`Solidos/Enclosures/w_encl.stp`'s 10 lost particles** -- re-ran
  end-to-end (convert + d1suned, `volSDEF=True`, full void generation):
  **0 lost particles**, full 1,000,000-history run, tally `0.9994 +/-
  0.18%`. Was previously the one file in a 111-file batch with any lost
  particles at all (see the "MCNP stochastic volume check" section far
  above) -- now clean.
- **`Gcommon`'s multi-tool bug** (`solid.common([tool1, tool2])` not
  computing the true intersection -- see the "SCDR_90 hidden-surface
  decomposition" section) -- re-read the current implementation directly:
  **`geo/_occ_impl.py` and `geo/_ocp_impl.py`'s own `Gcommon` already
  chain `BRepAlgoAPI_Common` pairwise, one tool at a time** (`for tool in
  tools: result = BRepAlgoAPI_Common(result, tool.__native__).Shape()`) --
  the exact fix this file's own earlier section proposed but never
  applied, evidently added independently while building these two
  backends, with nothing in this file ever connecting the two facts.
  **`geo/_freecad_impl.py`'s own `Gcommon` still has the original bug**
  (`solid.__native__.common([tool.__native__ for tool in tools])`, the
  whole list passed to native `.common()` at once) -- confirmed by direct
  code reading, not yet fixed there specifically.
- **`Gsplit`'s general non-manifold-solid reconstruction** (the
  face-adjacency-graph-plus-duplicated-capping-face technique, named at
  the very top of this file as the eventual proper fix for the project's
  original motivating bug, and separately flagged mid-file as "reserved
  for a future pyOCC implementation, not started") -- **is not just
  started but fully implemented and already wired into `Gsplit`**:
  `_repair_non_manifold_solid` exists in both `geo/_occ_impl.py` and
  `geo/_ocp_impl.py`, doing exactly the documented technique (union-find
  over faces excluding non-manifold edges, `BRepBuilderAPI_Copy` to cap
  each component missing a boundary there). What this file's own pyOCC
  Phase 1-4 sections got right is that it had never been *validated*
  against a real case needing it (`rev_pipe.stp`'s go/no-go test never
  exercised the repair path, since `BOPAlgo_Splitter` succeeded raw on
  that specific base/tool pair). Re-run today with `Gsplit` instrumented
  to log every call: **the repair path fires 3 times decomposing
  `Solidos/RoundCorners/rev_pipe.stp`**, `degenerate_case_handled=True`
  each time. The immediate per-call output includes some genuinely
  garbage fragments (zero and even negative volume -- the technique isn't
  clean on this input) but the *pipeline's* own downstream `remove_solids`
  filtering already discards those, and the final decomposition (5 real
  solids) sums to `512336.16`, matching the file's own documented true
  volume (`512337.6683`) to ~0.0003% -- a correct, volume-conserving
  result. So `rev_pipe.stp`'s own non-manifold case (documented at length,
  "found, confirmed, not yet fixed") is, in practice, already resolved --
  just never re-checked after the repair function was written.

**Confirmed still broken, exactly as documented**:
- **`ConeSphere.stp`'s segfault** (`UnifyEdges` inside `GSolid.refine()`/
  `.fix()`, under `ocp`) -- re-ran directly: still crashes (native exit
  code 5). `SCDR_90.stp` shared the same documented root cause and no
  longer crashes (see above) -- these two files' fates have now diverged
  under whatever changed since, worth another look if `ConeSphere.stp` is
  ever revisited specifically.
- **Enclosure solids duplicated into `meta_list`** (`TVA_final_allencl.stp`,
  confirmed real but not the cause of that file's own lost particles,
  which were separately fixed) -- confirmed by direct code reading:
  `loadfile/load_step.py`'s own call to the already-existing fix,
  `LF.remove_enclosure(meta_list)`, is still commented out (one line,
  `# LF.remove_enclosure(meta_list)`) -- the function itself is complete
  and correct, simply never invoked.
- **The 6 exotic quadric surfaces under OCC/OCP**
  (`Gmake_elliptic_cone`/`Gmake_hyperboloid`/`Gmake_ellipsoid`/
  `Gmake_elliptic_cylinder`/`Gmake_hyperbolic_cylinder`/`Gmake_paraboloid`/
  `Gmake_torus_elliptic`) -- confirmed still `_not_implemented(...)` stubs
  in `GEOReverse/Modules/_occ_impl.py`.

**Changed symptom, not actually fixed**:
- **`Solidos/Torus/2_degen_torii.stp`** -- previously a hard MCNP fatal
  error (`exit 152`, no tally section written at all). Re-ran end to end:
  the fatal crash is gone (`d1suned` completes, `RC=0`, tally section
  exists), but the run now hits the *ordinary* lost-particle abort (10
  lost, "no cell found in subroutine newcel", stopping at `nps=18` instead
  of the requested count) -- a real, still-open geometry gap, just a
  different and more tractable one than before.

**Not re-verified this pass (would need a live run/deeper trace not done
today)**:
- `TVA_final_allencl.stp`'s cells 1/2 (RPV upper right/left) -- the code
  for the fix this file documents (`vector_geometry.find_can_plane`,
  wired into `_closing_plane`) is confirmed present and in active use, and
  a later section in this same file already reports the specific tally
  numbers improving after it landed -- almost certainly closed, just not
  re-run today to confirm the exact number again.
- `check_sign` verification of RevCC from the conversion side (a
  different methodology than every other composite type's, per the
  decomposition-side scan this project used everywhere else -- never
  attempted).
- A full `Solidos/` corpus differential scan specifically under the raw
  `occ` (pythonocc-core/SWIG) engine, as opposed to `ocp` (now the
  default) or `freecad` -- extensive scans exist under the other two, none
  recorded under `occ` specifically since it stopped being the primary
  focus.
- `Gload_step_labels`'s FreeCAD-style auto-suffix naming for multiple
  solids sharing one XCAF label under `occ`/`ocp` -- documented as a
  narrow, non-blocking gap; not re-checked.

**Still pending, unrelated to the above (already tracked separately)**:
`AdjacentMultiplanePlanes` needs extending from RevCC to MultiRoundCorner
too -- see the section immediately above and
`project_mrc_adjacent_multiplane_pending.md`. `gen_plane_cylinder`/
`gen_plane_cone` (`meta_surfaces_utils.py`) still operate on a single raw
face (`Faces[ifacemin]`/`Faces[ifacemax]`) rather than a merged
same-surface shell when a RevCC segment's own cylinder/cone is split into
several contiguous pieces -- confirmed today to not be the cause of
`cyl_cone.stp`'s regression (none of its segments are actually split), but
the architectural gap is real and matches the exact pattern already fixed
once before in `cyl_plane_region_conf` -- no reproduction case found yet.

**Added to the pending list, 2026-08-21 (user request, not yet started)**:
- Re-run the full RevCC-on-irreducible-solids corpus scan (the same
  `Solidos/RevCC_corpus_scan` set this whole session's fixes were tested
  against piece by piece) as one complete pass now that all 3
  `get_join_cone_cyl` fixes (the `cyl_plane_region_conf` sampling
  fallback, `_valid_chain_junction`, the `twoPimod` wraparound fix, and
  the axis-dot-product floor) are committed together -- today's
  verification was per-fixture, not a fresh full sweep with everything
  landed at once.
- Reorganize the `Solidos/` STEP test fixture tree -- too many files
  accumulated across many sessions, almost certainly with real
  duplicates (the same physical case exported under different names in
  different triage folders) -- per the user's own observation while the
  482-file corpus scan was running: "son muchos sólidos y seguro que
  muchos serán duplicados."

## RevCC corpus re-run session (2026-08-22): folder reorg, 3 corrupt fixtures
retired, 3 independent real bugs fixed, and `add_reversedCC`'s own conceptual
redesign

Picked up from the pending list's first item: re-ran the full RevCC corpus
(the batch of already-decomposed irreducible solids, standard `convert_one.py`
settings, no special instrumentation) end to end through GEOUNED + d1suned.
283 files converted (283/283 OK), 283/283 d1suned runs completed (no crashes).
Result: 95.8% within 2σ, 1.1% marginal (2-3σ), 3.2% (9 tallies) real failures
beyond 3σ; 6 files with lost particles. Investigated one by one; every real
finding below is a *separate*, independently-verified bug -- not one fix
explaining everything.

**Workshop folder convention established, per explicit user request** (see
[[reference_workshop_folder_layout]] memory): `Solidos/` holds STP files
only; `Solidos/test_models/` is now the curated regression-fixture set (used
for this and future differential corpus scans, superseding the older ad-hoc
`RevCC_corpus_scan/` collection for that purpose); `../calculos_CLAUDE/`
(sibling of `Solidos/`, i.e. `GEOUNED_workshop/calculos_CLAUDE/`) holds every
calculation run (MCNP conversion, d1suned) -- never STP files. All of this
session's scratch STEP exports and diagnostic scripts live under
`calculos_CLAUDE/hylife113_angle_test/` and `calculos_CLAUDE/*_test/`.

**3 fixtures confirmed genuinely corrupt CAD, moved to `Solidos/BadCAD_decomposition/`**
(not GEOUNED bugs, per the same standing convention documented earlier in
this file): `Big_one_cell__modelcell_cut1__solid0_piece5__revcc2.stp` and its
duplicate (10 lost particles each, badly-cut decomposition fragment);
`DoubleCylinder__pieza__solid0_piece1__revcc1.stp` (5 lost particles, tally
52.458 -- badly-cut fragment); `Big_model_reserved__TVA_final_allencl__solid10_piece1__revcc3.stp`
(tally exactly 0.0) -- confirmed a genuinely degenerate sliver, 15,019 mm³
volume spread over a 3.87m × 3.87m × **22 micron** BoundBox (Volume/Area
ratio 0.011mm, well past `valid_solid`'s own `1e-3` rejection threshold in
absolute terms but the face itself is real, not a boolean-split artifact --
matching the family of `Solidos/BadCAD_decomposition/` fixtures already
documented earlier in this file).

**4 duplicate STP pairs found and moved to `Solidos/RevCC_corpus_scan/duplicates_removed/`**
(same physical piece, exported twice under different names/folders -- e.g.
from a `_noencl` vs `_allencl` variant of the same parent model, or a
`_v2`/`_all_pieces` re-export): confirmed via **direct comparison of the
written MCNP cell/surface cards** (per explicit user instruction -- not
volume alone), not just numeric volume/BoundBox similarity. 3 of the 4 pairs
are byte-for-byte identical cell/surface text (only header/timestamp/STEP-
translator-subversion differ); the 4th (`modelcell_cut1` piece70 vs
`all_pieces` piece_30) looked different at first glance but is the *same*
physical cell with surfaces 1↔2/3↔5/4↔6 simply permuted (confirmed by
cross-checking that the permutation is self-consistent across both the
surface list and the cell's own boolean signs) -- a numbering-order artifact
from a slightly different face-processing order between the two source
exports, not a real geometric difference.

### Bug 1: `_repair_non_manifold_solid`'s own reconstruction was never
validated -- confirmed via `modelcell_cut1_piece70`

The face-adjacency-graph non-manifold repair (`geo/_occ_impl.py` and
`_ocp_impl.py`, both engines identically affected) never checked its own
output before trusting it -- unlike `_try_coaxial_cone_split`'s already-
established "every piece must be BRepCheck_Analyzer-valid AND the summed
volume must match the input" safety net. Confirmed live on `piece70`
(`Big_one_cell__modelcell_cut1__solid0_piece70__revcc2.stp`): the very first
`Gsplit` call (base solid cut by a single plane) produced a raw, invalid
BOP fragment; the repair "fixed" it into **3 pieces summing to 3045.9 --
~30% more volume than the 2330.1 input**, and 2 of those 3 pieces were
*themselves* still topologically invalid. This is the concrete mechanism
behind a class of decomposition artifact this project has repeatedly hit
(a real, correct-looking piece plus a spurious extra fragment). Fixed with
the exact same discipline `_try_coaxial_cone_split` already established:
after `_repair_non_manifold_solid` runs, verify every resulting piece is
independently valid AND that their summed volume matches the pre-repair
invalid solid's own volume (1e-6 relative tolerance, matching
`GSolid.refine()`'s own guard) -- if either check fails, discard the
reconstruction and keep the original, unrepaired (still-invalid) solid
instead of fabricating volume. Discussed directly with the user whether a
partial accept (keep only the individually-valid pieces from the repair)
would be safer than all-or-nothing revert: no -- confirmed on this exact
fixture that *all 3* repaired pieces were invalid, so partial-accept would
have kept zero volume (worse than reverting to the original). The
volume-conservation check on the *whole* reconstructed set, not per-piece
validity alone, is the right invariant in general (a partial accept can
silently drop real material even when the kept pieces are individually
valid).

Verified: `piece70`'s own decomposition no longer fabricates the spurious
extra fragment (now correctly stays as 1 unresolved piece for that specific
cut, matching the "no worse than before" design goal -- the underlying
inability to cleanly split this exact plane cut is a separate, still-open
tangency question, but the silent corruption is gone). `tests/geo` (all 3
engines) and `tests/test_cadtocsg.py` unaffected.

### Bug 2: `convex_planes`'s own angular sort had a dead sign test, and
`build_roundC_params` was starving it of points -- confirmed via
`Big_one_cell/modelCell_670000.stp` piece59's sibling, `piece52`

Same file (`modelCell_670000.stp`), same physical feature (3 R=6mm
round-corner cylinders) as the already-fixed `piece59` -- but `piece52`
still wrongly merged them into one `MultiRoundC` (flat `AND` of all 9
components, matching zero of 300 real interior points -- tally=0.0 in the
corpus batch) instead of 3 separate `RoundC` like `piece59` gives.

Two independent bugs, found by tracing the exact code path, not guessed:

1. **`convex_planes`'s own convexity/turning-consistency test had a dead
   sign check**: `if cross.dot(ref) < 0: sina = -sina` -- but `cross =
   ref.cross(rp)` is *always* perpendicular to `ref` by construction, so
   `cross.dot(ref)` is exactly `0.0` every time (confirmed numerically:
   `0.0000000000` to 10 decimal places on real data), never negative. This
   silently broke the angular sort the convexity check depends on (`sina`
   never flips sign, so `atan2` never distinguishes clockwise from
   counter-clockwise around the group). Should have been `cross.dot(zaxis)`
   (the same reference the function's own turning-sign test a few lines
   below already uses) -- fixed.
2. **Even with the sign fixed, 3 points alone can never fail the turning-
   consistency test** (too few pairwise turns to detect a real
   inconsistency) -- confirmed directly: re-running `convex_planes` on
   `piece52`'s real 3 shared corner planes still gave `convex=True` after
   fix 1 alone. `build_roundC_params` was passing only the shared corner
   planes (`plane_list`, deduplicated -- 3 for this file) into
   `convex_planes`, discarding each cylinder's own additional closing
   plane (`gpa`, already computed a few lines earlier for a different
   purpose and silently dropped on the floor here). Extending the
   convexity/orientation *test's own* input to `plane_list + gpa`s (6
   points total for this file; `plane_list` itself, used for the group's
   real top-level AND/OR terms, is untouched) makes `convex_planes`
   correctly return `False` -- confirmed live.

With both fixes, `piece52` gives `RoundC: 4` (this file's real corner count,
different from `piece59`'s 3 -- not a discrepancy, just a different real
solid) and matches **300/300 real interior + 300/300 real exterior points**
against direct CAD ground truth (`solid.is_inside`). d1suned on the isolated
piece: tally `0.99892 ± 0.50%` (was `0.0` exactly), 0 lost particles.

### Bug 3: `get_adjacent_cylplane`'s curved-edge walk had no axial-extreme
check -- confirmed via `Big_model_reserved/TVA_final_allencl.stp` solid8
piece0

`_find_adjacent_multiplane_planes` (feeding `ReversedConeCylParams.AdjacentMultiplanePlanes`,
the RevCC/MultiPlane-boundary escape mechanism documented earlier in this
file) calls `get_adjacent_cylplane(..., cornerPlanes=False)`, which walks
*every* curved boundary edge of a cylinder/cone segment's own OuterWire and
accepts whatever real `GPlane` it finds across each one -- with no check
that the edge is actually at the surface's own axial extreme (V=Vmin or
V=Vmax). A hole, notch, or unrelated mid-height feature also leaves a
curved boundary edge, and a real plane found across *that* one is not a
legitimate closing plane for the segment as a whole. Confirmed live: this
RevCC's own cylinder segment spans Z=[-999.98, 6500.0] (its real axial
extent), but the unfiltered walk picked up a genuinely real, unrelated
plane sitting at **Z=4000** -- squarely inside the segment's own span, not
at either end -- as if it were a closing boundary. Combined via `AND` at
the top level of the RevCC's own `AdjacentMultiplanePlanes` mechanism, this
wrongly chopped off most of the real solid's own axial extent.

Fixed by adding an optional `axial_bounds=(vmin, vmax)` parameter to
`get_adjacent_cylplane` -- when given, only accepts a curved edge whose own
sampled V-parameter lands within a small tolerance of one of the two
bounds (computed, for a merged multi-piece shell, as the true min/max
across every piece's own `ParameterRange`, not each piece's local range,
which would wrongly treat internal seams between pieces as if they were
real ends). Threaded through only at `_find_adjacent_multiplane_planes`'s
own call site -- the two *other* `cornerPlanes=False`/`True` callers
(`get_can_surfaces`'s own end-cap search, `multiplane()`'s corner-plane
search) default `axial_bounds=None`, unchanged behavior, since this defect
is specific to searching for a segment's own *axial* closing plane, not
the *corner* planes those other callers look for.

Verified: `TVA_final_allencl` solid8 piece0's `mp_planes` search now
correctly returns 0 candidates (the Z=4000 plane excluded, and no real
end-cap plane exists for this segment at its own true extremes) instead of
the wrong Z=4000 one. d1suned: tally `0.99582 ± 0.32%` (was `2.17844`,
σ=346.6), 0 lost particles.

### Bug 4 (the deep one): `add_reversedCC`'s own `plane_region` -- a real
conceptual defect, not a sign bug, found via extensive live derivation with
the user and finally fixed by the user directly

Confirmed via `Big_model_reserved/hylife-v06.stp` solid113/114 piece0 and
`inputSTEP/dientes3.stp` solid0 piece2 -- all three share the same RevCC
topology: 2 cone segments + 1 cylinder segment, each with its own
"additional plane" (`pk1`, `pk2`, `pc` respectively). All three originally
gave real, large tally deviations (1.119σ≈52, 1.1215σ≈53, 1.368σ≈51).

**What was ruled out first, methodically, before finding the real cause**:
- `check_sign` had no dispatch branch for `"ReversedConeCylinder"` at all
  (fell through to an implicit `None`) -- added one (mirroring Can/TCone's
  own `.components`/`.region.evaluate()` pattern, with `.components`
  correspondingly populated in `_reversedCC_component`/`add_reversedCC`)
  purely to make this verifiable at all. Confirmed the RevCC's own internal
  region, evaluated this way, already matched real geometry correctly at
  every sampled point -- the bug was NOT in the composite surface's own
  formula being wrong in isolation.
- Hand-derived, with the user, a from-scratch "recursive line-segment
  decomposition" algorithm for combining N boundary planes via nested
  AND/OR based on local convexity (mirroring `generic_split`'s own real
  3D recursive-cut algorithm, applied to a 2D projection of the RevCC's
  own additional planes) -- a genuinely rigorous derivation (confirmed via
  a concrete "step" test case, `M = (LA AND LB) OR (LC AND -LB)`,
  matching ground truth 5/5 test points) that correctly resolved the
  "sequential chaining is order-dependent and therefore wrong" flaw a
  simpler first attempt had. Started implementing this against the real
  RevCC data (projecting the additional planes onto a plane perpendicular
  to the chain's own shared axis, finding real 2-point segments via
  where this plane crosses each segment's own real face edges) and it
  *did* produce a valid, real solid once the right projection height and
  face-to-CylCone identity mapping were found -- but was abandoned
  ("cambio de estrategia, esto no nos lleva a ningún sitio") once it
  became clear the full general algorithm wasn't going to be needed.
- **"Apply GEOUNED to itself"**: built a box, ran the real `Gsplit`
  (matching `generic_split`'s own recursive splitting exactly) using
  `pc`/`pk1`/`pk2` as the only 3 candidate cutting tools, and checked each
  resulting piece's own material status via `Gcommon` against the real
  solid (not a single point sample -- the full boolean intersection).
  Found empirically: material = `pk1 OR pk2` (with `pc` not appearing at
  all), verified to 0.0005% volume match. Cross-verified independently:
  flipping the *sign* of `pk1`/`pk2` (their raw, uncorrected
  `gen_plane_cone`-computed axis has no material-direction correction the
  way `find_can_plane`/`cks_edge_plane` already apply -- a real, separate,
  not-yet-fixed defect in `gen_plane_cone`/`gen_plane_cylinder`) while
  keeping the *original* `plane_region = OR[pc,pk1,pk2]` structure
  produced the exact same tally (`0.99938`) via a completely different
  code path -- strong independent confirmation.
- A GEOReverse (CsgToCad) round-trip of the *original, unfixed* cell 1
  found a separate, real, still-unresolved discrepancy: the reconstructed
  CAD volume was 1.401× the true solid's volume, while d1suned's own
  tally for the same unfixed file was only 1.119× -- the two don't agree,
  and the reconstructed solid's own BoundBox (Z:[3981.7, 4465.6])
  genuinely exceeds the cell's own explicit bounding planes 6/7
  (Z:[4050.8, 4396.5]). Not pursued further once the user found the real
  fix via the boolean-formula route instead -- flagged here as a distinct,
  open GEOReverse-side question for a future session.

**The real fix, found by the user directly, after all of the above**: the
original code paired each segment's own primitive term with its own
*individual* closing plane (`terms_region = AND[(s_i OR -p_i) for each
segment i]`, `plane_region = OR[p_1, ..., p_n]`,
`reversedCC_region = plane_region * terms_region`). The fix instead pairs
every segment's own primitive term with the *global* union of every
closing plane (`AND[(s_i OR -plane_region) for each segment i]`) --
algebraically, since the final expression is already AND'd with
`plane_region` itself, this simplifies to
**`plane_region AND (s_1 AND s_2 AND ... AND s_n)`**: material requires
being on the correct side of *at least one* closing plane (OR, unchanged),
AND satisfying *every* segment's own primitive surface simultaneously
(AND) -- a materially different, and evidently correct, physical
statement from the original per-segment pairing. `geouned_classes.py`:

```python
plane_region = None
surf_components = []
for cc in cylcones:
    s_region, p_region = self._reversedCC_component(cc)
    plane_region = BoolSurface.add(plane_region, p_region)
    surf_components.append(s_region)

surf_region = None
for s_region in surf_components:
    surf_region = BoolSurface.mult(surf_region, s_region + (-plane_region))

surf_region.region.simplify(None)
reversedCC_region = plane_region * surf_region
```

(The exploratory `.components`/`check_sign` RevCC-verification scaffolding
added earlier in this same investigation was reverted once no longer
needed -- `boolean_solids.py` and `geouned_classes.py`'s `.components`
tracking are both back to their pre-session state; only the `plane_region`
fix above remains.)

Verified via the real end-to-end pipeline (`decompose_solids` →
`build_solid_definition` → `build_void` → `export_csg` → d1suned, standard
settings) on all 3 originally-failing solids: `hylife-v06` solid113
`0.99938 ± 0.24%`, solid114 `1.00220 ± 0.24%`, `dientes3` piece2
`1.00332 ± 0.81%` -- all 0 lost particles, all within ~1σ of 1.0 (were
51-53σ).

### Combined verification and a large, expected differential (not yet
independently isolated)

All 3 bugs (piece70's repair-validation, piece52's `convex_planes`, TVA
solid8's `get_adjacent_cylplane`) plus the `add_reversedCC` fix landed
together. `tests/geo` (156+2skip FreeCAD, 78+1skip occ, 39 ocp) and
`tests/test_cadtocsg.py` (50/50, all 3 engines) all green.

A differential corpus scan across the new `Solidos/test_models/` set (109
files, excluding `Big_model_reserved`, Can/TCone/RoundCorner/
MultiRoundCorner/MultiPlane/RevCC counts, `git stash` before/after) found
**24 files differ** -- but the great majority (everything under
`RoundCorners/`, e.g. `rc10.stp`/`rrc10.stp`/etc.) show `RevCC` completely
unchanged (0→0) and only `RoundC`/`MultiRoundC` shift, in a very
consistent `MultiRoundC:1 → RoundC:2`-shaped pattern -- matching exactly
the *already independently 300/300-validated* `convex_planes`/
`build_roundC_params` fix (bug 2 above) spreading to files far beyond the
one that originally motivated it, not a side effect of the `add_reversedCC`
fix. This is the first time that fix's full corpus-wide blast radius has
been observed (previously validated only on `piece52`/`piece59`
specifically) -- consistent with, but not yet independently confirmed
against, real CAD ground truth the way `piece52` itself was. **Not done
this session**: isolating bug 2's and bug 4's own differential contributions
separately (re-run the scan with only one fix active at a time) to confirm
neither is masking a problem in the other -- flagged as the natural next
step if this corpus's behavior ever needs deeper trust beyond the 3
specific d1suned-verified solids above.

## `rc9.stp` MultiRoundCorner investigation: 3 real bugs found and fixed --
`is_same_plane_surface`'s antiparallel-axis bug (foundational), `convex_planes`'s
unsorted-negative-angle bug, and a genuinely-invalid "same face closes both
ends" RoundCorner premise

Follow-up session (2026-08-23) to the RevCC corpus re-run above. Picking up
the pending "isolate Bug 2's differential contribution" item led instead to
a fresh, real bug: `Solidos/test_models/RoundCorners/rc9.stp` (a small,
deliberately simple 2-cylinder "stadium" shape: 2 round posts connected by
2 flat parallel walls) gave tally=2.83 (3x the real volume) via d1suned --
a genuinely new finding, not a re-surfacing of any previously-documented
issue.

### Duplicate found and moved: `rrc9.stp`

`rc9.stp` and `rrc9.stp` (both in `RoundCorners/`) turned out to be
byte-identical MCNP output except for the source-face comment
(`/Fusion0021` vs `/Slice.0011`) -- confirmed the same duplication pattern
already established for the `RevCC_corpus_scan/duplicates_removed/` set
earlier in this project's history. Moved to a new
`Solidos/test_models/RoundCorners/duplicates_removed/rrc9.stp`, `rc9.stp`
kept as canonical.

### Bug 1: same real face closing both ends of a RoundCorner's cylinder is
a genuinely invalid premise, not a fixable sign case

`Solidos/working_solids/rc1_decomp.stp` (a fragment the user isolated by
hand from a different model, `rc1`) surfaced the *other* end of the
already-documented "p1 and p2 identical" special case (see the RoundCorner
theoretical definition, `composite_surface_definitions.md`): the doc
already distinguished "2 disjoint faces of the same coincident plane"
(valid) from "genuinely only 1 real face closing both ends" -- but the code
never actually enforced that distinction. `cyl_plane_region_conf`'s
`AND_p1_pd`/`AND_p2_pd` came out as *unconditional logical opposites*
whenever `p1 is p2` (the literal same Python face object, not just
geometrically coincident) -- not tangency noise, an exact 0-degree angular
singularity: when the same face closes both ends, the "additional plane"
`pd` (through both touching edges) *is* that same face, so testing p1/p2's
relation to `pd` is testing the face against itself.

Two fix attempts tried and reverted first (kept as a caution): skipping the
degeneracy-rejection check when `p1 is p2` -- rejected by the user as
"displacing the problem," not fixing the invalid premise itself; and
forcing `AND_p2_pd`'s own sign to agree with `AND_p1_pd` when `n1==n2` --
**broke `Big_one_cell/modelCell_670000_solid0_piece59` outright** (10 lost
particles via d1suned, confirmed a previously-fixed real p1!=p2 RoundCorner
case where that exact sign convention is load-bearing) -- reverted
immediately.

**Actual fix, per explicit user direction ("lo tiene que poner en
get_adjacent_plane")**: `get_adjacent_cylplane`'s `cornerPlanes=True`
branch (`meta_surfaces_utils.py`) now deduplicates its own found corner
planes by real face `Index` before returning -- when both of a cylinder's
corner edges close against the *same* real face, that collapses to a
single entry, and the caller's own pre-existing `len(adjacent_planes) != 2`
check rejects it naturally, no new rejection logic needed downstream.
Verified: `rc1_decomp.stp` falls back to ordinary per-face reconstruction
(d1suned tally 1.0165 +/- 1.74%, was misclassified before); `piece59` fully
unaffected; full `tests/geo` + `tests/test_cadtocsg.py` 128/128 under ocp.
Committed as `2722ad6`.

### The "4 configurations" worked example -- RoundCorner's `p1_cyl` valid
range, dictated and saved to memory

Investigating `rc9.stp` itself (p1 != p2, a real 2-plane corner, unrelated
to Bug 1 above) surfaced a second candidate issue: `AND_p1_cyl`/`AND_p2_cyl`
came out wrong (OR when real material sampling showed AND) at a
*well-conditioned* cross product (`0.484`, nowhere near the existing
`1e-3` degenerate-tangency threshold) -- a different class of bug than the
already-fixed piece59 near-tangency case. The user walked through a
from-scratch geometric derivation (a unit circle, 4 concrete half-plane
configurations hinged at an arc endpoint) to pin down exactly when a
round-corner bounding plane's own hinge-angle configuration is degenerate
-- fully dictated and saved to `composite_surface_definitions.md`'s
RoundCorner section (config 4: the half-plane's own chosen ray direction
travels toward the plane's *other* circle-crossing point, and that second
crossing lies *inside* the round corner's own arc -- both conditions must
hold together). A code fix attempt based on this (`_plane_recrosses_arc`
in `meta_surfaces_utils.py`, rejecting when both p1 and p2 hit this
pattern) was built, verified against a hand-derived criterion, and **wired
in -- then found to make `rc9.stp` *worse* (introduced 10 lost particles
that weren't there before)** once tested end-to-end, because rejecting a
decomposition-time *candidate* surface can silently steer
`generic_split` down a different cutting path entirely (the same risk
class as the `get_can_surfaces`/`outer2_only` precedent documented earlier
in this file) -- reverted in full, including the now-dead
`_plane_recrosses_arc` helper.

**This whole thread turned out to be chasing a symptom, not the root
cause** -- see Bug 3 below, which independently fixed the *actual*
degeneracy this investigation was working around. The 4-configuration
derivation and its degeneracy criterion remain saved in memory as a
correct, dictated piece of theory (confirmed by the user, "sí es
correcto") even though the specific code fix built from it was reverted.

### Bug 2 (the real root cause): `is_same_plane_surface`'s antiparallel-axis
offset comparison

Direct user request: verify the pristine, undecomposed `rc9.stp` (before
any cut) classifies correctly. It does -- `get_roundCorner` on the raw
solid finds **2 independent, valid RoundCorners** (`Configuration=7`,
i.e. `fwd_cyl + AND_p1_cyl + AND_p2_cyl`, all-AND -- the natural
classification for a post bounded by 2 parallel walls, per the MRC
"Definability condition": both real corners here independently confirm
the *same* AND relationship with both walls, exactly the precondition an
MRC needs). But `build_roundC_params`'s own `is_same_surface(p1.Surface,
p2.Surface)` check -- meant to detect the legitimate "p1==p2 geometrically"
special case -- came back **True** for `rc9.stp`'s 2 real, genuinely
different, parallel walls (3.5 units apart, antiparallel normals), forcing
`gpa` (the additional/closing plane) to `None` for both corners and
corrupting `multi_round_corner_region`'s formula downstream (the
originally-observed 2.83x-volume over-inclusion, a real, standalone bug
independent of anything this file's earlier RevCC sessions covered).

Root cause: `is_same_plane_surface` (`geo/vector_geometry.py`) --
```python
if abs(plane_1.Axis.dot(plane_2.Axis)) < 0.99999:
    return False
return abs(plane_1.Axis.dot(plane_1.Position) - plane_2.Axis.dot(plane_2.Position)) <= 1e-5
```
Each plane's own offset is measured *along its own axis* -- correct for
same-direction axes, but when the two axes are *antiparallel* (still a
legitimate "same infinite plane" case, e.g. the same real plane reached
via opposite Face Orientations, which the function's own docstring
explicitly says should count as equal), the two offsets are measured in
*opposite* directions and must be compared via `d1 == -d2`, not
`d1 == d2`. For `rc9.stp`'s 2 real, different, parallel walls (y=+1.75
axis=(0,-1,0), y=-1.75 axis=(0,1,0)): `d1 = axis1.dot(pos1) = -1.75`,
`d2 = axis2.dot(pos2) = -1.75` -- equal by coincidence of the antiparallel
convention, wrongly matching. **Fix**: branch on the sign of the axis dot
product -- `d1 == d2` when parallel (unchanged), `d1 == -d2` when
antiparallel. Single, foundational function (only one definition in the
whole codebase, shared by all 3 engines via `geo/vector_geometry.py`).

Verified: `rc9.stp` tally 0.999274 +/- 0.61%, 0 lost particles (was 2.83,
before any of this session's other fixes were even in place). `piece59`
unaffected (0.98880 +/- 0.65%, byte-identical). Full `tests/geo` +
`tests/test_cadtocsg.py` green under both ocp (128/128) and freecad
(156/156). A 108-file `Solidos/test_models/` differential scan (excluding
`Big_model_reserved`) found **exactly 2 diffs, both `rc9.stp`/its moved
duplicate**, going from `MultiRoundC:1` to `RoundC:2` -- zero regressions
elsewhere in the corpus.

### Bug 3: `convex_planes`'s angle sort silently corrupted by unnormalized
negative `atan2` output

While root-causing why `rc9.stp`'s cylinder1 face was arriving at
conversion time *already fragmented* into 3 pieces (a 241.6-volume bulk
piece missing its own real x=+0.968 touching edge, plus 2 small 3.27-volume
slivers) -- confirmed via `decompose_solids()` alone (before any
conversion-phase code runs) that `rc9.stp`, despite the pristine-solid
check above showing it's a valid, irreducible 2-RoundCorner shape, was
genuinely being **cut into 5 pieces** by `generic_split` -- the user found
and fixed the real cause directly, in `convex_planes` (`functions.py`):
```python
angles.append((math.atan2(sina, cosa), i))
```
`math.atan2` returns values in `(-pi, pi]` -- mixing negative and positive
angles before the subsequent `angles.sort()`, which assumes a single,
monotonically-comparable circular ordering starting from a consistent
reference. An unnormalized negative angle sorts *before* all positive
ones instead of at its true position further around the circle, corrupting
the convexity/orientation classification this function feeds into
`ReversedConeCylinder`'s AND/OR grouping and `MultiRoundCorner`'s own
plane-list convexity test (both already documented at length elsewhere in
this file) -- and, per this session's finding, apparently also feeding
into whatever candidate-surface classification led `generic_split` to
accept an incorrect cutting candidate for `rc9.stp` specifically. **Fix**:
normalize into `[0, 2*pi)` before appending --
```python
angle = math.atan2(sina, cosa)
while angle < 0:
    angle += 2 * math.pi
angles.append((angle, i))
```
Verified: `decompose_solids()` alone now leaves `rc9.stp` as a single,
whole, 1-piece solid (matching the pristine-solid classification exactly,
cylinder1's face keeping both real x=+0.968 edges intact) -- the
fragmentation is gone entirely. Final tally after all 3 fixes combined:
unchanged from Bug 2 alone, 0.999274 +/- 0.61%, 0 lost particles -- Bug 2
was already sufficient to fix `rc9.stp`'s own tally, but Bug 3 fixes the
*decomposition* itself (the shape should never have been split to begin
with) and very plausibly explains other, unrelated `convex_planes`-driven
regressions elsewhere in the corpus not specifically chased down this
session.

A full 109-file `Solidos/test_models/` batch conversion + d1suned rerun
(all 3 fixes combined, per explicit user request, no differential
composite-count scan needed this time) confirmed: 107/109 convert (same 2
pre-existing, already-documented failures -- `ConeSphere.stp` segfault,
`multiplane_add_plane_cone`'s zero-division bug); `rc9.stp` and its moved
duplicate both give the fixed 0.999 tally; every previously-marginal
(2-3 sigma) file is byte-for-byte the same set as before these fixes,
confirming no new regressions among previously-working files. Two files'
*failure mode* changed without being a regression in the "previously fine,
now broken" sense: `Big_complex_cell/modelCell_670000.stp` previously
failed to convert at all, now converts (slowly, ~230s) but loses 10
particles at runtime -- partial progress, not a regression, since there
was no tally to compare against before; `modelcell_cut1.stp` previously
gave a bad tally (45.5 sigma, a real pre-existing bug), now loses 10
particles instead -- same underlying complex-geometry issue, different
symptom. Both remain open, not investigated further this session. One
real, transient methodology trap hit and resolved during this same
verification: a stale `RoundCorners__rrc9` output directory (from before
`rrc9.stp` was moved into `duplicates_removed/`) was re-scanned by
d1suned's own directory-sweep script and briefly looked like a real
regression (tally back to 2.83) -- confirmed as leftover stale data (not
the actual, freshly-converted `RoundCorners__duplicates_removed__rrc9`,
which correctly shows the fixed 0.999 tally) and deleted, matching this
project's own established `placa3`-stale-`outp` precedent.

Diagnostic scripts this session (scratchpad only, not committed):
`check_rc9_configs.py`/`check_rc9_cross_magnitude.py` (the material-sampling
verification of `AND_p1_cyl`/`AND_p2_cyl` against real geometry),
`verify_config4_criterion.py` (the geometric "other circle crossing"
criterion check, part of the reverted Bug-1-adjacent thread),
`check_rc1_n1_n2.py`/`instrument_cyl_plane_region_conf.py` (the p1==p2
n1/n2 sign-resolution trace), `check_rc9_original_config.py` (the
pristine-solid, uncontaminated `get_roundCorner` call that first revealed
`is_same_plane_surface`'s bug via a correct `gpa` position),
`check_post_decompose_wire.py`/`trace_get_adjacent_cylplane.py` (the
5-piece-fragmentation trace that led to Bug 3), `check_refine_corruption.py`
(ruling out `GSolid.refine()` as the fragmentation's cause).

## Two crashes found while resolving "GEOUNED doesn't finish" cases

Follow-up session, working through the standing pending-list item of STEP
files where GEOUNED itself crashes rather than mis-converts.

### `Mixed/multiplane_add_plane_cone.stp`: `gen_plane_cone`'s cone-apex
degeneracy

Crashed with `ZeroDivisionError` inside `gen_plane_cone`
(`meta_surfaces_utils.py`). Root cause: this function finds each of 2
UV-node candidates by matching on U only (ignoring V), and one of this
file's 2 cone faces has `ParameterRange` with `Vmin=0.0` -- i.e. the face's
own V=0 boundary IS the apex. When the matched node happens to land at
V=0, `(V1 - apex)` is a genuine zero vector, so `.normalized()` divides by
zero. Fixed by detecting the apex-degenerate case directly (`(V1 -
apex).length < 1e-7`) and, when it fires, re-sampling the same face at a
tiny nonzero V fraction (`1e-3 * vmax`) instead of the apex itself --
applied symmetrically to both `V1`/`ifacemin` and `V2`/`ifacemax`, since
either candidate can independently land on an apex. Verified via d1suned:
tally `0.9912 +/- 0.45%`, 0 lost particles.

### `Mixed/ConeSphere.stp`: a genuine native process crash (access
violation) in `ShapeUpgrade_UnifySameDomain`, opposite crash-triggering
flags between OCP and pythonocc-core on identical OCCT 7.9.3 geometry

Crashed the whole process (no Python traceback, exit code matching
`0xC0000005`) under both pyOCC engines -- first noticed as a load-time
crash (`Gload_step`'s own healing call), then, once worked around there, a
*second*, structurally identical crash resurfaced at decomposition time
(`GSolid.refine()`, called unconditionally on every loaded solid from
`GeounedSolid.__init__`). A genuine access violation cannot be caught by
Python `try`/`except` -- confirmed directly, matching this project's own
established understanding of the difference between this class of crash
and the `StdFail_NotDone`/`Standard_Failure`/`RuntimeError` cases
`refine()`'s existing exception handling already does catch.

**First isolated which `ShapeUpgrade_UnifySameDomain` flag actually
triggers it, per engine, by direct construction-flag sweep (keyword and
positional both cross-checked) against the file's own loaded solid --
not assumed from either engine's own pre-existing docstring, both of
which turned out to describe the wrong flag**:

- **Under `ocp` (pybind11)**: `UnifyFaces=True` crashes (whether alone or
  combined with `UnifyEdges`); `UnifyEdges=True` alone (`UnifyFaces=False`)
  completes cleanly, identical volume, still valid.
- **Under `occ` (pythonocc-core/SWIG)**: the exact opposite --
  `unify_edges=True` crashes (whether alone or combined);
  `unify_faces=True` alone (`unify_edges=False`) completes cleanly,
  identical volume, still valid.

**The two bindings' native crash behavior on identical OCCT 7.9.3
geometry is not symmetric.** This contradicts what each engine's own
`refine()`/`fix()` docstring previously claimed (`_ocp_impl.py`'s said
"isolated to the UnifyEdges flag specifically (UnifyFaces alone... returns
promptly)"; `_occ_impl.py`'s said UnifyEdges hangs and was kept on,
implying UnifyFaces was safe) -- both were either stale (an earlier OCCT
build behaved differently) or a keyword/positional mixup (both
constructors' own argument order is confirmed swapped between the two
bindings, an easy place to get flipped, and already flagged as a gotcha in
`_ocp_impl.py`'s own module docstring). Trust the fresh, direct
per-engine verification over either old docstring claim.

**Fix, applied independently per engine** (`GSolid.fix()` and
`GSolid.refine()`, both files): disable the crash-triggering flag for that
engine specifically, keep the other one on. Re-verified against the
*specific* precedent regression this kind of change has broken before
(`tests/test_cadtocsg.py`'s `cylBox.stp`/`DoubleCylinder/pieza.stp`, which
lost a real Can secondary surface the one time `UnifyEdges` was disabled
under `occ` previously) -- **no regression found this time**, on either
file, under either engine (`cylBox.stp`: `FwdCan:3, RoundC:1` unchanged;
`pieza.stp`: `MultiP:1, RevCan:3, RoundC:1, RevCC:1` unchanged) -- the
disabled flag this session is the *other* one from what regressed before
in each case, so this is a different, narrower change than the one that
previously failed, not a retry of the same thing.

`_ocp_impl.py`'s `Gload_step` had grown a separate, narrower `_heal_on_load`
helper (`ShapeFix_Shape` only, deliberately bypassing `GSolid.fix()`
entirely) as a first, more conservative attempt at working around the
load-time crash, built before the exact crash-triggering flag had been
isolated. Once `fix()` itself no longer crashes on this file,
`_heal_on_load` became redundant (and strictly weaker -- it skipped the
`UnifyEdges` healing step too) -- removed, `Gload_step` now calls
`GSolid(...).fix(1e-6)` directly, matching `_occ_impl.py`'s own equivalent
exactly (engine symmetry restored).

Verified: `tests/geo/test_ocp_impl.py` + `tests/test_cadtocsg.py` (89/89,
`ocp`) and `tests/geo/test_occ_impl.py` + `tests/test_cadtocsg.py` (89/89,
`occ`); the real end-to-end pipeline (load -> decompose -> build -> void ->
export) on `ConeSphere.stp` under both engines, no crash, `OK`; a real
d1suned MCNP stochastic volume check on the `ocp`-converted output: tally
`0.9949 +/- 0.28%`, 0 lost particles -- confirms the fix produces
geometrically correct output, not just a crash-free run.

Diagnostic scripts this session (scratchpad only, not committed):
`test_unifyedges_variants.py`/`test_unify_minimal.py`/`test_unify_positional.py`
(the `ocp` flag-isolation sweep), `test_occ_conesphere_unify.py`/
`test_occ_load_conesphere.py`/`test_occ_fix_isolated.py`/`test_occ_fix_variants.py`
(the identical sweep under `occ`, which revealed the opposite-flag
finding), `check_unifyfaces_regression.py`/`check_occ_regression.py` (the
`cylBox.stp`/`pieza.stp` precedent-regression re-check, per engine),
`test_refine_unifyedges_only.py`/`test_ocp_fix_at_load.py` (volume/validity
confirmation of the fixed flag combination).

**Not yet done**: FreeCAD engine not affected (uses `Part.Shape.removeSplitter()`,
a completely different code path) and not re-checked here, matching this
project's own convention that these two pyOCC-family fixes don't
necessarily transfer to FreeCAD. No sweep of *other* files for the same
class of crash under either engine -- this session only confirms
`ConeSphere.stp` specifically; a `Solidos/` corpus-wide scan for other
files that might hit the now-still-enabled flag (`UnifyEdges` under `ocp`,
`UnifyFaces` under `occ`) on some other real geometry was not attempted.

## `Cans/pipe.stp` fixed: `get_can_surfaces`'s same-radius-cylinder branch
required `is_parallel`, making it dead code for a genuine "broken cylinder"

`pipe.stp` is a single tube bent through 2 kinks: 3 straight cylinder
segments of the *same radius* whose axes meet at an angle at each kink
(not parallel -- the user's own definition of a "cilindro quebrado"),
closed at the 2 true open ends by flat planes. d1suned confirmed ~27.6
sigma off before this fix.

**Methodology, user-directed step by step, not guessed**: first built
`Cans/pipe1.stp` (same solid, same volume, `BRepAlgoAPI_Splitter` used to
split *only* the central cylinder face -- not the solid -- at the plane
X=-9600, producing 2 face pieces of the same analytic cylinder) as a
deliberate test fixture, to check `closed_cylinder_cone`'s own merging
behavior with a known-in-advance "this must merge" case before touching
any Can-detection code.

1. `closed_cylinder_cone` on `pipe.stp`'s own 3 real (unsplit) segments:
   confirmed no merging happens between segments (different axes, real
   kink) -- each stays its own single-face result, as expected.
2. `closed_cylinder_cone` on `pipe1.stp`'s artificially-split central
   segment: confirmed it *does* correctly re-merge the 2 pieces back into
   one `ShellGu` (same radius, same axis *line* -- `is_same_cylinder`'s
   real criterion, not just parallel direction) regardless of which piece
   is used as the seed.
3. `get_adjacent_cylknesurf` on that merged shell: found the 2 real
   kink-neighbor cylinder faces (not the 2 end-cap planes, which turned
   out to be one hop further away -- correcting an initial assumption
   about `pipe.stp`'s own topology: the cylinders touch each other
   *directly* at each kink via a real curved (elliptical) edge, with no
   separating plane in between; the 2 planes only close the 2 true open
   ends of the whole bent tube).
4. Direct trace of `get_can_surfaces`'s own per-neighbor branch
   (`meta_surfaces.py:148`) on those 2 real kink-neighbors: **`same_rad`
   was always `True`, `is_parallel` was always `False`** -- confirmed the
   user's own prediction live, not just by reading the code: since
   `get_adjacent_cylknesurf` already excludes anything `is_same_surface`
   to the shell's own surface (same radius + same axis *line*), the only
   way a face reaching this `if` could ever have `is_parallel==True` is a
   separate, laterally-offset (non-collinear) parallel cylinder of the
   same radius -- never a real kink neighbor, whose axis is by definition
   not parallel. So the branch meant to handle "adjacent same-radius
   cylinder, treat as pass-through" was unreachable dead code for exactly
   the "broken cylinder" scenario it needed to cover, and execution fell
   through to the generic `commonEdge`/`region_sign`-based closing-surface
   path instead -- treating each kink neighbor as if it were an ordinary
   Can end cap rather than a continuation of the same composite body.

**Fix** (`meta_surfaces.py`, `get_can_surfaces`): dropped the
`is_parallel(...)` condition from the `if` at line 148, keeping only the
radius check -- per the user's own direct instruction, once the dead-code
diagnosis above was confirmed live. Every face reaching this branch has
already passed `get_adjacent_cylknesurf`'s `is_same_surface` exclusion, so
same-radius alone is now sufficient to route it into the pass-through
(`(s, None, True)`, `omit=True`) mechanism -- whether the neighbor is a
laterally-offset parallel cylinder (the branch's original, narrower
target) or a genuine kinked-axis neighbor (the case this fix actually
unblocks).

**Verified**: `get_can_surfaces` on `pipe1.stp` now returns both kink
neighbors via the pass-through mechanism instead of falling through.
d1suned on `pipe.stp`: tally `0.99185 +/- 0.55%` (was ~27.6 sigma), 0 lost
particles. `tests/geo/test_ocp_impl.py` + `tests/test_cadtocsg.py`, 89/89
under `ocp`. A 109-file differential corpus scan (`Solidos/test_models`,
excluding `Big_model_reserved`, composite-surface counts, `git stash`
before/after) -- **only the 2 targeted files differ, 0 regressions
elsewhere**: `pipe.stp` `FwdCan:2 -> FwdCan:3`, `pipe1.stp` `{Cyl:2,
FwdCan:2, RevCC:1} -> FwdCan:3` -- and `pipe.stp`/`pipe1.stp` now give the
*identical* result, confirming the artificial face split (a pure
topology-only change, zero volume/geometry difference) no longer perturbs
the classification at all, as it should.

**Not yet done this pass**: re-verification under `occ`/`freecad` engines
(only `ocp` checked); `Big_one_cell/modelCell_670000.stp`'s own "converts
but loses 10 particles" symptom was flagged earlier this file as
possibly a `RevCC`-adjacent case -- not re-checked against this fix.

## `SCDR_90_piece2.stp` sliver investigation: `CharacteristicWidth`, a
robust general sliver detector -- implemented, wired everywhere found so
far, but a real regression on a sibling file is NOT yet fixed. Session
ends here with uncommitted changes -- resume by fixing the regression
before committing.

Follow-up to the `Cans/pipe.stp` fix above (same session). User-directed
investigation of `Solidos/test_models/Decomposed/SCDR_90_piece2.stp`
(pending list: d1suned tally ~23.5 sigma off / intermittently 10 lost
particles), working test-first per the user's own established discipline
this whole session.

### Dead end 1: `min_area` was never actually wired into the candidate-plane
generator at all

Traced a specific unexplained cutting plane (Position/Axis given
directly by the user) to a real face: `face[12]`, a genuine STEP face,
Area=1.9262mm^2 -- a residual boolean-cut sliver, not a modeling error.
Raising `Tolerances.min_area` (even to 3.0, comfortably above 1.9262) had
**zero effect** on the actual translation. Root cause: `order_plane_face`
(`decompose/decom_utils_generator.py`, the function that ranks/selects
which real plane faces become decomposition cutting candidates) never
read `min_area` -- or *any* area threshold -- at all; `plane_generator`
computed `tolerances.min_area` but never passed it through. Fixed:
`order_plane_face(Faces, omitfaces, min_area=None, min_face_width=None)`
now actually excludes faces below the threshold; wired from
`plane_generator`. Verified: `min_area=3.0` alone now gives
`SCDR_90_piece2.stp` a clean d1suned tally (`0.999918 +/- 0.34%`, was
losing particles) -- but the *default* `min_area=0.01` still doesn't
reach this sliver's own 1.9262mm^2 area, so this alone doesn't fix the
file without manual tuning.

### Dead end 2: raw area/compactness don't generalize across the corpus

User's own idea: since 8.46mm^2 (a real face) and 1.93mm^2 (the sliver)
are the same order of magnitude, maybe an "aspect ratio" per face (area
vs. a shape-compactness measure) would separate them where raw area
can't. Added `GFace.Compactness = Area / RG_max^2` (`RG_max` = largest
principal radius of gyration, via `BRepGProp.SurfaceProperties_s(...).PrincipalProperties()`
-- computed once, piggybacking on the existing Area/CenterOfMass call, no
extra native pass needed) to `geo/_ocp_impl.py`. Worked cleanly on this
one file (real faces: Compactness 0.585-4.62; slivers: 6e-7-0.019) --
but a full 109-file/1733-face corpus scan found real, large-area,
legitimately elongated faces (e.g. `RoundCorners/TVA_solid16_cell17.stp`
face[12], Area=14575.8mm^2, a genuine 5.6mm x 2602.8mm structural
plate edge -- confirmed by direct export+inspection, user verified it by
eye) with Compactness as low as 0.026 -- indistinguishable from a sliver
by this measure alone, since Compactness measures shape elongation, not
physical scale. A universal Compactness threshold would misclassify real
thin panels.

### Dead end 3: isolated-edge-length clustering (per-solid noise floor)

Comparing the sliver's own edge lengths against the *rest of the same
solid's* edge-length distribution (not the whole corpus) showed a real
pattern: the sliver's edges (0.055mm) sit in an isolated log-space
cluster, bracketed by large gaps on both sides, disconnected from both
the solid's own sub-micron noise floor and its real-feature range; a
real face's short edge (e.g. TVA's 5.6mm) sits at the *bottom of a
continuous* distribution instead. Implemented a per-solid clustering +
gap-detection algorithm and tested 3 refinements (bare isolation, shortest-
edge-only, isolated-edge-count) -- all produced false positives: a real,
large cone/plane face directly touching the sliver has the sliver's own
tiny shared boundary edge as its own "shortest edge," getting flagged
even though the face itself is completely legitimate. Abandoned as too
complex/fragile relative to what it bought.

### The fix that worked: `CharacteristicWidth`

User's refined idea: combine a characteristic *length* with Compactness,
rather than using either alone. Derived analytically: for a rectangle of
length L and width W, `RG_max == L/sqrt(12)` exactly, so
`W == Area/(RG_max*sqrt(12)) == sqrt(Area*Compactness/12)` -- and this
generalizes correctly to a *curved* sliver too (RG_max captures the
spread along whatever shape the face follows, straight or curved).
Verified this recovers the sliver's true width (0.055034mm, matching its
own real ParameterRange-derived dimension exactly) and TVA's real
panel's true width (5.6mm, exact) from Area+Compactness alone -- no
per-surface-type angle-to-arclength conversion needed, unlike a naive
ParameterRange-based approach.

**Corpus-wide validation** (109 files, 1733 faces, reusing the same scan
data as the Compactness dead end): sorted by `CharacteristicWidth`, there
is a single, clean gap of >0.5 log10-decades between `0.0550342mm`
(`SCDR_90_piece2.stp` face[12], the sliver -- the *largest* width among
every known sliver in the whole corpus) and `0.192768mm`
(`Torus_solid1.stp` face[7] -- the *smallest* width among every other
face in the whole corpus, real or not yet independently confirmed).
**Not a single face out of 1733, across every file, falls in that gap.**
0.1mm (the user's own original suggestion) sits in the middle. Not a
strict mathematical guarantee in all cases (14/1733 faces, all real,
substantial, *curved* faces like sphere caps, have `width` up to 23%
*larger* than `sqrt(Area)` would suggest under a flat-rectangle model --
confirmed this deviation is always in the safe direction, toward *larger*
computed width, never smaller) but empirically the cleanest, most robust
signal found this session, and the only one that survived corpus-wide
testing.

### Implementation (ocp engine only so far)

- `geo/_ocp_impl.py::GFace.__init__` -- added `Compactness` and
  `CharacteristicWidth` as eager fields (both derived from the single
  already-computed `PrincipalProperties()` call).
- `GEOUNED/utils/data_classes.py::Tolerances` -- new `min_face_width`
  field, default `0.1` (mm). Deliberately kept as a *separate*,
  independent tolerance from `min_area`, not a replacement -- the two
  are complementary (confirmed via the same corpus scan: `width` isn't
  strictly bounded by `sqrt(Area)` for all shapes, so neither threshold
  strictly subsumes the other; `min_area` still independently catches a
  tiny-in-both-dimensions compact fragment, `min_face_width` catches a
  large-area-but-narrow sliver `min_area` alone would miss).
- `decompose/decom_utils_generator.py::order_plane_face` -- excludes on
  `min_area` OR `min_face_width` (both apply to plane candidates).
- `utils/meta_surfaces_utils.py::eligible_plane` -- **also fixed a
  real, independent pre-existing bug found along the way**: it always
  called a bare `Tolerances()` (the class default) instead of accepting
  the caller's actual configured tolerances object, silently ignoring
  any `min_area` the user had set via `CadToCsg(tolerances=...)`. Now
  takes `tolerances=None` and threads the real object through; also
  gained the same `min_face_width` check.
- `utils/meta_surfaces.py::multiplane`, `decompose/generators.py::next_multiplanes`,
  `utils/functions.py::get_multiplanes` (the CONVERSION-phase MultiPlane
  path, called from `conversion/cell_definition.py`) -- all now thread
  `tolerances` through to `eligible_plane`/the recursive `multiplane`
  call, instead of relying on the previously-broken hardcoded default.
- `conversion/cell_definition.py::simple_solid_definition`'s own
  per-face reconstruction loop -- already had a `min_area` check
  (`face.Area < Surfaces.tolerances.min_area`, logged and skipped) but,
  same story, no width check; added `face.CharacteristicWidth <
  Surfaces.tolerances.min_face_width`, **not restricted to `GPlane`**
  (an early version wrongly gated this to planes only -- corrected per
  direct user feedback: a thin sliver Cylinder/Cone/Sphere/Torus patch is
  exactly the same class of artifact, and `CharacteristicWidth` is
  already computed generically for every surface type).
- `decompose/generators.py::cylinder_generator`/`cone_generator`/
  `sphere_generator`/`torus_generator` -- **also had zero area/width
  filtering of any kind** for their own decomposition-candidate role
  (parallel gap to `plane_generator`'s, found by the same "does this
  apply to every face-type loop" question, per direct user instruction);
  all 4 now take `tolerances=None` and skip a face below
  `min_face_width`, wired from `get_surfaces`.

### Verification: 2 files fixed/improved, 1 file regressed -- NOT resolved

- `Decomposed/SCDR_90_piece2.stp`: **fixed**, tally `0.999918 +/- 0.34%`
  (was losing particles / ~23.5 sigma), with *default* tolerances (no
  manual `min_area` tuning needed) -- the original goal, achieved. 15
  surfaces written (was 17 -- the sliver's own 2 spurious extra planes,
  leaked in via the conversion-phase per-face loop's own missing width
  check, are gone).
- `Complex_cell/SCDR_90.stp` (the full, un-decomposed original this
  fixture family derives from -- long documented history earlier in this
  file, historically 0.999346/~3.3 sigma right at this project's own
  "real failure" threshold): **improved**, tally `0.998417 +/- 0.23%`
  (~0.69 sigma) -- a real, independently-confirmed improvement, not
  assumed from the piece2 fix alone.
- `Mixed/SCDR_90_hollow.stp` (a related SCDR_90 variant, same face-area
  fingerprint as `piece2` in the corpus scan): **regressed**. Confirmed
  via direct before/after d1suned on the *identical* file, isolating this
  session's changes with `git stash`/`stash pop` (not assumed): before,
  `0.998967 +/- 0.37%` (clean, ~0.28 sigma); after, `1.49022 +/- 0.32%`
  (badly wrong, ~150+ sigma). Composite-surface counts shifted
  `MultiP:2,RoundC:2,RevCC:1` -> `MultiP:1,RoundC:4,RevCC:3` -- a materially
  different reconstruction, not just a numeric drift. A raw MCNP-text
  diff (same technique used for `piece2`'s own root-cause) shows a more
  complex pattern than `piece2`'s "one known sliver plane leaks back in"
  -- several *new* plane surfaces with normals not matching any single
  already-identified sliver, and others disappearing -- suggesting the
  fix changed which candidate surfaces get discovered during RevCC/
  MultiPlane chain detection in a way that's *wrong* for this file,
  unlike `piece2`/`SCDR_90.stp` where the same kind of change was
  *correct*. **Not root-caused.** This is exactly the "decomposition
  candidate-list change can silently steer down a different, wrong
  path elsewhere" risk this project's own history warns about
  repeatedly (`get_can_surfaces`/`outer2_only`, the RevCC corpus
  sessions, etc.) -- confirmed to have actually happened here, not just
  a theoretical risk this time.

**109-file corpus differential scan** (`Solidos/test_models`, excluding
`Big_model_reserved`, composite-surface counts, `git stash`/`stash pop`):
exactly 4 files differ -- the 3 above plus
`Big_complex_cell/modelCell_670000.stp` (`RoundC: 41 -> 40`, one fewer --
not independently verified this session; this file is already known
lost-particle-broken regardless of this change, per the pending list
below, so this specific count shift's own correctness is unconfirmed).

**Update: the `SCDR_90_hollow.stp` regression named below is resolved** --
see "`SCDR_90_hollow.stp` piece5 resolved" further down this file for the
full account (a real, separate `Gsplit` degeneracy, not a bug in the
`CharacteristicWidth` work itself). The working tree was left uncommitted
at the time this note was written; by the time the fix below landed, more
files had accumulated (`utils/meta_surfaces_utils.py`,
`geo/vector_geometry.py` in addition to the ones listed here) -- see
`git status` for the current, authoritative set rather than trusting this
list.

### Not yet done (beyond the regression itself)

- `CharacteristicWidth`/`Compactness` exist only in `geo/_ocp_impl.py` --
  not yet ported to `_occ_impl.py` or `_freecad_impl.py`. Per this
  project's own "prove it works, then propagate" convention, porting
  should wait until the `SCDR_90_hollow.stp` regression is resolved and
  the `ocp`-engine behavior is fully trusted.
- `tests/geo/test_ocp_impl.py` + `tests/test_cadtocsg.py` (89/89) were
  green after every step this session, but this only exercises
  `testing/inputSTEP`'s 50-file corpus, not `Solidos/test_models` (where
  the regression was actually found) -- the 109-file scan is the real
  regression net here, not the committed test suite.
- `Big_complex_cell/modelcell_cut1.stp`, `Enclosures/w_encl.stp` cells 4-5,
  `Big_complex_cell/modelCell_670000.stp`'s own lost-particles issue --
  still open from the prior pending list. `Mixed/multiplane_add_plane_cyl.stp`
  is no longer open -- see "`_find_adjacent_multiplane_planes`... " further
  down this file (improved, not fully resolved: 4.5 sigma -> 1.8 sigma).

## `SCDR_90_hollow.stp` piece5 resolved: a real cone-cylinder coaxial
degeneracy `_try_coaxial_cone_split` never searched for, plus a retry
tolerance that was too tight for its own presplit -- found and fixed
with extensive live user guidance, corpus-verified, tests green

Direct continuation of the `SCDR_90_piece2.stp` sliver investigation
above (same session, later). `SCDR_90_hollow.stp` piece5 (one of 8
irreducible decomposed pieces, Volume=4511.6013mm^3) gave a clean-looking
but wrong d1suned tally (0.740239 +/- 0.52%, ~26% material missing) --
unaffected by any of the `CharacteristicWidth` work. **User's own
diagnosis, stated directly and confirmed correct end to end**: piece5 was
never actually irreducible -- a real `Gsplit` call during decomposition
silently failed to separate it, the same class of bug as this file's own
"Motivating problem" and the already-existing coaxial-cone `Gsplit`
fallback (`_try_coaxial_cone_split`, `geo/_occ_impl.py`/`_ocp_impl.py`)
was built to catch -- but didn't, here.

**Investigation, in the order it actually happened** (kept because each
wrong turn is instructive, matching this file's own established
discipline):
- First isolated a genuine *verification-script* bug, not a geometry bug:
  point-sampling `check_sign` against piece5's own top-level surfaces
  while assuming every term must be positive gave `both_hits=0`
  (apparently zero overlap between predicted CSG and real material) --
  contradicting d1suned's own clean-looking tally. **User caught this
  directly** ("ni puede ser tan distinta tu estimacion del CSG con
  d1suned"). Root cause: piece5's real cell definition
  (`3 -5 -6 3 7 (-1:-2 4)`) requires the two K/Y cones **negated**
  (`-5`, `-6`) -- respecting the real signed literals gave
  `predicted/real ~= 0.7567`, matching d1suned almost exactly and
  confirming the deficit is real, not a script artifact.
- Verified the K/Y cards' own trailing sheet values (`-1`/`+1`) were
  correctly, individually assigned to the right physical cone (matched
  by exact apex-coordinate coincidence to 6+ significant digits) --
  **not swapped**, ruling out that specific hypothesis cleanly.
- `Gsplit` call-tracing (monkeypatching `geo.Gsplit`, matching this
  file's own established technique) during `decompose_solids()` found
  piece5's own ancestor (base_vol=4511.6013, byte-identical to piece5's
  own final volume) rejected 4 separate cone-tool candidates as
  "unchanged" (`degenerate_case_handled=False`) -- but a nearly-identical
  sibling solid elsewhere in the same decomposition tree
  (base_vol=4511.5953, differing by only 0.006mm^3) *did* successfully
  split via the exact same class of tool
  (`degenerate_case_handled=True`, giving 3344.11/1167.49) -- a strong
  early signal that piece5's own deficit (~1172mm^3) matched that
  sibling's own smaller piece almost exactly.
- `_try_coaxial_cone_split` **was** already firing on piece5's own
  cone1/cone2 candidates but returning `None` every time. Two real,
  separate misdiagnoses were chased before the correct one, both
  confirmed wrong via direct plain-boolean (`BRepAlgoAPI_Common`/`Cut`)
  ground-truth checks rather than assumption:
  1. First suspected the coaxial-cone search itself found the *wrong*
     candidate circle on a mis-fragmented `other_cone` face (v0 on the
     cone's own UV parametrization looked inconsistent with global Y at
     first glance) -- this was a **false lead**: the mid_point/radius
     computed by the existing formula (`(apex1+apex2)/2`,
     `|apex1-apex2|/2`) was, in fact, already exactly correct (Y=49.9997,
     R=54.5) -- the confusion was mistakenly assuming the surface's own
     UV "V" parameter equals global Y directly, which it doesn't.
  2. Then extended the fix to search coaxial **cone-cylinder** pairs too
     (a real, separate degeneracy class: a cone reaching a coaxial
     cylinder's own radius at exactly one height) -- added
     `is_coaxial_cone_cylinder_pair` (`geo/vector_geometry.py`) and
     `_group_coaxial_cylinder_faces` (`_ocp_impl.py`), wired into
     `_try_coaxial_cone_split` as additional candidates alongside the
     existing cone-cone search. This correctly found and split the *real*
     circle where the R=40 cylinder, cone1, and cone2 all coincide
     (Y=64.4997) -- but this turned out to be a **second false lead**:
     that circle already existed as a real edge on both the cylinder and
     cone2 faces (confirmed live: `n split_pieces=1`, nothing to split --
     the boundary was already exactly there), and the user's own manual
     FreeCAD cut confirmed directly that the only edge OCCT's kernel
     failed to draw was specifically **the edge between the cutting cone
     and the other cone surface** -- i.e. the original cone-cone
     candidate, not cone-cylinder. (`is_coaxial_cone_cylinder_pair`/
     `_group_coaxial_cylinder_faces` are kept in the codebase regardless
     -- a real, independently correct predicate/grouping for a real
     degeneracy class, just not the one active in this specific fixture.)
- **Real root cause, confirmed via user-supplied ground-truth vertices**:
  the user gave 5 real coordinates from their own inspection of the
  actual solid and a parallel manual FreeCAD cut. 3 (`v1`, `v2`, `v3`)
  matched real existing vertices to within 0.03-0.20mm; the other 2
  (`v4`=(450.56,50,-15.27), `v5`=(448.69,50,-19.07), explicitly called
  "puntos" not "vertices" by the user) matched **no** existing vertex --
  but were confirmed to lie almost exactly on the tool cone's own surface
  (44.9995-45.0002 deg vs a 45.0000 deg SemiAngle). Re-running the
  *original*, unmodified cone-cone candidate search (the one already in
  the codebase before this session, mid_point=(400.767,49.9997,6.884),
  radius=54.5) found its own 2 crossings at **exactly** v4/v5 -- the true
  circle really was the cone-cone one all along, and the crossing points
  the algorithm already correctly finds on it are precisely the 2 real
  points the user separately gave from their own inspection.
- **Why the presplit + retry still failed even on the correct circle**:
  the presplit (splitting cone2's own fragment at the v4-v5 arc) built a
  valid solid, but `_raw_bop_split(presplit, tool, tolerance)` still
  returned exactly 1 unchanged solid at the caller's own `tolerance`
  (0.0) and every value up to 0.05. **Sweeping much wider retry
  tolerances directly** (not guessed -- tried a real range) found the
  actual threshold: **0.1 to 2.0** cleanly separates the presplit into 2
  solids (3344.14/1166.41, matching the sibling's 3344.11/1167.49 to
  within ~1mm^3) -- `tolerance=0.05` fails, `tolerance=0.1` succeeds, a
  real, sharp cliff, not a gradual improvement. The existing code only
  ever retried at the caller's own (typically near-zero) `tolerance` --
  never escalated it -- so a presplit that's topologically valid but not
  numerically *exact* enough for OCCT's own coincidence detection at
  tight tolerance was silently discarded every time.

**Fix, `_try_coaxial_cone_split` (`geo/_ocp_impl.py`)**: the retry step
now tries an escalating tolerance ladder
(`tolerance, 1e-6, 1e-4, 1e-2, 0.1, 0.5, 1.0`, skipping any value below
the caller's own `tolerance`) instead of a single fixed attempt, keeping
the first that produces >=2 valid solids passing the volume-conservation
check. The volume-conservation tolerance itself is loosened from the
existing `1e-6` relative bound to `1e-3` relative **specifically when a
nonzero fuzzy retry tolerance was needed** (confirmed live: the real
deviation at `tolerance=0.1` is ~2.3e-4 relative, comfortably inside
`1e-3` with margin, while a still-tight `1e-6` bound would have wrongly
rejected the correct result) -- an exact (`tolerance=0.0`) retry keeps
the original tight `1e-6` bound unchanged, so nothing about the
already-validated `rev_pipe.stp`/`SCDR_90_piece0_badvolume.stp` go/no-go
cases from this fix's own original introduction is loosened.

**2 real regressions found and fixed while corpus-verifying** (both
confirmed via a 109-file `Solidos/test_models` differential scan,
excluding `Big_model_reserved`):
- `_split_face_at_v_line` crashed (`Standard_ConstructionError:
  Geom2d_TrimmedCurve::U1 == U2`) on 4 files
  (`Cans/fwd_can_0.stp`/`fwd_can_1.stp`/`rev_can_0.stp`/`rev_can_1.stp`)
  once the new cone-cylinder candidate search started reaching a
  periodic surface where the two candidate crossings coincided once
  reduced to the same U parameter (e.g. a 2*pi wraparound) -- a real,
  previously-unreachable degenerate input this function's own
  `Geom2d_TrimmedCurve` construction didn't guard against. Fixed with a
  `u_hi - u_lo < 1e-9` guard, returning `[native_face]` unchanged (the
  function's own already-documented "didn't actually separate anything"
  outcome) instead of crashing -- applies to cone-cone candidates too,
  not just the new cone-cylinder ones, though only the latter reached it
  in practice this session.
- `gen_plane_cylinder` (`utils/meta_surfaces_utils.py`) crashed
  (`ZeroDivisionError` inside `GVector.normalized()`) on
  `Big_complex_cell/modelcell_cut1.stp` -- an indirect consequence (a
  different decomposition path reached this pre-existing fragility for
  the first time, not a bug in this session's own new code) of piece5's
  fix changing decomposition ordering elsewhere in the corpus. Root
  cause: `(V2 - V1).cross(axis)` is exactly zero when the closest-UV-node
  search picks the same point for both `ifacemin`/`ifacemax` ends -- this
  function's own header comment already flags it as a known-simplified
  approximation ("Tolerance in this function are not the general once /
  function should be reviewed"). Fixed with a length guard falling back
  to `_perpendicular_axis(axis)` (the same arbitrary-but-deterministic
  fallback pattern already used elsewhere in this file for a
  similarly-undefined direction) instead of crashing.

**Verification**: the fixed `SCDR_90_hollow.stp` now decomposes piece5
into 2 real pieces (3344.14/1166.41, was 1 piece at 4511.60) --
`decompose_solids()` alone confirms this, matching the sibling branch's
own split almost exactly. Full end-to-end d1suned stochastic volume
check on the complete, re-generated model (`volSDEF=True`, full void
generation, standard settings): **tally 0.998967 +/- 0.37%** (was
0.740239, ~35 sigma off) -- **0 lost particles**. `tests/geo` +
`tests/test_cadtocsg.py`, 128/128 under `ocp`. A 109-file
`Solidos/test_models` differential corpus scan (composite-surface-count
crash/success comparison, excluding `Big_model_reserved`) went from 6
failures (4 real regressions from this fix's own first cut, fixed above;
1 pre-existing `ZeroDivisionError` also fixed above; 1 pre-existing slow
file, `Big_complex_cell/modelCell_670000.stp`, exceeding this scan's own
90s per-file timeout -- already documented elsewhere in this file as
genuinely slow, ~230s, not a crash) down to **108/109**, with the one
remaining "failure" being exactly that same known-slow file.

**Not yet done**: `CharacteristicWidth`/`Compactness` (from the
`SCDR_90_piece2.stp` investigation immediately above) remain
`_ocp_impl.py`-only, not yet ported to `_occ_impl.py`/`_freecad_impl.py`
-- this session's fixes are `ocp`-only too (`is_coaxial_cone_cylinder_pair`
lives in the shared `vector_geometry.py` and is available to `_occ_impl.py`
already, but `_group_coaxial_cylinder_faces`/the retry-tolerance-ladder
change were only made in `_ocp_impl.py`). A full "before vs after"
composite-surface-count differential (not just crash/success) was not
completed for this specific fix -- the `git stash` attempt to build a
clean before-baseline accidentally stashed the *entire* `_ocp_impl.py`
file (including the earlier, still-uncommitted `CharacteristicWidth`
work), producing a systematically broken (not a valid) baseline; this was
recognized and abandoned in favor of the crash/success-only comparison
above, which is what's actually verified.

### A real, severe cross-engine crash found by finally testing FreeCAD/occ
against this session's `CharacteristicWidth` work -- fixed with defensive
`getattr` guards, not a full port

The 128/128 `ocp` test result above was the only engine checked
end-to-end for most of this session -- running `tests/test_cadtocsg.py`
under `GEOUNED_CAD_ENGINE=freecad` for the first time since
`CharacteristicWidth` was introduced (prompted by finishing the piece5
fix and wanting to confirm all 3 engines before considering this done)
found **47/117 tests failing** with `AttributeError: 'FaceGu' object has
no attribute 'CharacteristicWidth'` -- every one of the 6 real call sites
added this session (`decompose/generators.py`'s 4 `*_generator`
functions, `conversion/cell_definition.py`'s per-face loop,
`utils/meta_surfaces_utils.py::eligible_plane`,
`decompose/decom_utils_generator.py::order_plane_face`) assumed
`CharacteristicWidth` exists on every `GFace`/`FaceGu`, which was only
ever true under `ocp`. The "Not yet done" note above already flagged
`CharacteristicWidth` as `ocp`-only, but had understated the actual
consequence -- not "an unavailable optimization elsewhere," a **hard
crash** on any FreeCAD conversion touching a plane/cylinder/cone/sphere/
torus candidate at all (i.e. nearly every real file).

Rather than porting the full `PrincipalProperties`/radius-of-gyration
machinery to `_freecad_impl.py`/`_occ_impl.py` right now (a real, bounded
but nontrivial piece of work, and explicitly out of scope for finishing
the piece5 fix), each of the 6 call sites was changed to
`getattr(face, "CharacteristicWidth", float("inf"))` instead of a direct
attribute read -- on an engine that doesn't have the field, the check
degrades to "never trigger" (matching the pre-`CharacteristicWidth`
behavior exactly, `min_area` alone still applies), never a crash; on
`ocp`, `getattr` resolves to the real value, so `ocp`'s own already-
verified behavior (0.999918 tally on `SCDR_90_piece2.stp`, etc.) is
completely unchanged. Verified: **freecad 117/117**, **occ 89/89**
(pythonocc-core -- confirmed to have hit the identical crash class before
the fix, same root cause, same guard resolves it), **ocp 128/128** --
all 3 engines green together for the first time since
`CharacteristicWidth` was introduced. Porting the real computation to the
other 2 engines (so `min_face_width` actually protects them too, not just
`ocp`) remains open, now correctly scoped as "a missing feature on 2
engines," not "a live crash on 2 engines."

**Update, same session -- ported for real, per explicit user instruction
("lo del CharacteristicWidth sí a ambos freecad y occ")**:
`Compactness`/`CharacteristicWidth` are now computed on every engine's
own `GFace`, not just `getattr`-guarded away:
- `_occ_impl.py` (pythonocc-core): byte-for-byte the same code as `ocp`'s
  own `GFace.__init__` -- `props.PrincipalProperties().RadiusOfGyration()`
  is an identical API call on both bindings (already piggybacking on the
  same `_surface_props(native)` GProp call this engine's `GFace` already
  made for `Area`/`CenterOfMass`).
- `_freecad_impl.py`: FreeCAD's `Part` API has no direct
  `PrincipalProperties()` equivalent, so this one genuinely needed a
  different technique -- `native.MatrixOfInertia` (the same area-based
  inertia tensor `GEdge`/`GWire.MatrixOfInertia` already use, from
  earlier in this migration) diagonalized by hand via
  `numpy.linalg.eigvalsh` (mirroring `decom_utils_generator.py::
  get_axis_inertia`'s own already-established eigendecomposition
  pattern for edges), giving the same 3 principal moments OCCT's own
  `PrincipalProperties()` computes internally; radius of gyration per
  axis is `sqrt(moment / Area)`. **Verified, not assumed**: computed
  `CharacteristicWidth` for `Solidos/test_models/Decomposed/
  SCDR_90_piece2.stp`'s own already-known sliver face (face index 12)
  under all 3 engines directly via `geo.Gload_step` -- **all three give
  the byte-identical value, `0.055034196945107326`** -- confirming the
  hand-diagonalized FreeCAD route is not just plausible but numerically
  exact, matching this migration's own established "verify against a
  real, known value across engines" discipline rather than trusting the
  math by inspection alone. `FaceGu` (`utils/geometry_gu.py`) needed no
  separate change -- it inherits from `GFace` and already calls
  `super().__init__(x.__native__)`, so it picks up the new fields
  automatically, closing the exact crash this whole section started
  from. The `getattr(..., float("inf"))` guards at the 6 call sites
  are left in place as a harmless defensive fallback (now always
  resolving to the real value on all 3 engines) rather than reverted
  back to direct attribute access. `_try_coaxial_cone_split`'s own
  extension (the coaxial cone/cylinder search, tolerance-escalation
  retry) was ported to `_occ_impl.py` too (mirroring `ocp`'s fix
  exactly, pythonocc-core naming conventions) but **deliberately not**
  to `_freecad_impl.py` -- confirmed with the user directly that this
  one stays occ/ocp-only, per the original scoping decision (FreeCAD's
  `Part` API has no equivalent for the raw `Geom2d_Line`/
  `ShapeAnalysis_Surface.ValueOfUV`/`BRepLib.BuildCurve3d_s` primitives
  this specific technique needs).

**Update**: the full 3-engine suite was run, per the user's own "todos al
final" instruction -- freecad 119/119, occ 87/89, ocp 126/128, with the
4 failures (2 per pyOCC engine) confirmed pre-existing and unrelated to
this session (see "Closed later the same day" above, in the pending-tasks
section, for the full trace/`git stash` verification).

## Pending tasks, 2026-08-23 (consolidated)

**Explicit user priority ordering, stated directly at the end of this
session**: finish cleaning up known bugs in `GEOUNED` (the forward
CAD-to-CSG pipeline) *before* picking `GEOReverse` (CsgToCad) issues back
up -- e.g. the `test_cylbox_convertion` failure and the `hylife-v06.stp`
round-trip discrepancy, both `GEOReverse`-side, are deliberately left
open rather than chased further right now.

Compiled from every open item scattered across this file's history plus
this session's own findings, superseding the 2026-08-21 audit above where
they overlap. Not independently re-verified item by item this pass (that
full-discipline re-check is itself listed as a pending item, below) --
treat as a compiled index, not a guarantee every line still reproduces.

**Closed since the 2026-08-21 audit (no longer pending)**:
- `ConeSphere.stp`'s native segfault (`ShapeUpgrade_UnifySameDomain`) --
  fixed both engines, this session (see "Two crashes found" above).
- `multiplane_add_plane_cone.stp`'s `ZeroDivisionError` -- fixed, this
  session.
- The full RevCC-on-irreducible-solids corpus re-run -- done, 2026-08-22
  (283 files, 4 real bugs found and fixed: `_repair_non_manifold_solid`'s
  own unvalidated reconstruction, `convex_planes`'s dead sign test +
  `build_roundC_params` point-starving, `get_adjacent_cylplane`'s missing
  axial-extreme check, `add_reversedCC`'s `plane_region` redesign).
- `rc9.stp`'s `MultiRoundCorner` misclassification -- fixed, 2026-08-23 (3
  bugs: `get_adjacent_cylplane` dedup, `is_same_plane_surface` antiparallel
  bug, `convex_planes` unnormalized-angle bug).
- `Cans/pipe.stp`'s ~27.6 sigma tally -- fixed, 2026-08-23 (see "`Cans/pipe.stp`
  fixed" above: `get_can_surfaces`'s same-radius-cylinder branch required
  `is_parallel`, dead code for a genuine kinked "broken cylinder").

**Resolved since this list was written** -- `SCDR_90_hollow.stp`'s own
regression (`CharacteristicWidth`-based sliver filtering breaking it,
0.999 -> 1.49) turned out to be a real, separate, pre-existing `Gsplit`
degeneracy (piece5 was never actually irreducible), unrelated to
`CharacteristicWidth` itself -- see "`SCDR_90_hollow.stp` piece5
resolved" further down this file for the full fix (`_try_coaxial_cone_split`'s
retry now escalates its own fuzzy tolerance instead of trying only the
caller's, which was too tight). Full tally now 0.998967, was 0.740239.
The working tree is still uncommitted (more files now than the 8 listed
when this note was first written -- see `git status`) -- porting
`CharacteristicWidth`/this session's other `ocp`-only fixes to
`occ`/`freecad` remains open, not blocked by a regression anymore.

**Closed later the same day (2026-08-23, evening session)** -- see the
"`SCDR_90_hollow.stp` piece5 resolved" and "A real, severe cross-engine
crash..." sections further down this file for the full accounts:
- `SCDR_90_hollow.stp` piece5's ~26% missing volume -- root-caused (a
  real `Gsplit` coaxial-cone-pair degeneracy the existing
  `_try_coaxial_cone_split` fallback detected but whose retry never
  escalated its own fuzzy tolerance past the caller's near-zero default)
  and fixed. Full-model d1suned tally: 0.740239 -> 0.998967.
- The `CharacteristicWidth`/`Compactness` work (from the `SCDR_90_piece2.stp`
  investigation, same day) was found to hard-crash the `freecad` and
  `occ` engines entirely (47/117 and equivalent tests failing,
  `AttributeError`) the first time either was actually tested against it
  -- fixed two ways: immediate `getattr(..., float("inf"))` guards at
  all 6 real call sites (never crash, degrade to "check doesn't apply"),
  then, per explicit user instruction, a real port of the computation
  itself to both engines (`_freecad_impl.py` via `MatrixOfInertia` +
  hand eigendecomposition, `_occ_impl.py` byte-identical to `ocp`'s own
  `PrincipalProperties()` call) -- verified to give the byte-identical
  value on all 3 engines for a known sliver face.
- `_try_coaxial_cone_split`'s own fix (cone/cylinder coaxial detection +
  tolerance-escalating retry) was also ported to `_occ_impl.py`
  (pythonocc-core) -- confirmed with the user this one does **not** go to
  `_freecad_impl.py` (no equivalent for the raw `Geom2d_Line`/
  `ShapeAnalysis_Surface`/`BRepLib.BuildCurve3d_s` primitives it needs).
- `docs/users_guide/execution_settings/cad2csg/python_cadtocsg_{cli,api}_usage.rst`
  and `tests/config_cadtocsg_complete_defaults.json` updated with the new
  `min_face_width` `Tolerances` field (the `Tolerances` class docstring
  itself, and therefore `docs/python_api.rst`'s autodoc page, already had
  it from when the field was first added).
- Full 3-engine test suite run (`tests/geo` + `tests/test_cadtocsg.py` +
  `tests/test_csgtocad.py`) after all of the above: **freecad 119/119,
  occ 87/89, ocp 126/128** -- the 2 failures on each pyOCC engine are
  `test_csgtocad.py::test_cylbox_convertion[mcnp]`/`[openmc_xml]`
  (`GEOReverse`/`CsgToCad` producing 2/1 solids in the final exported
  STEP where 4/5 are expected). **Confirmed, via `git stash` of this
  entire session's `src/` changes, to be a pre-existing failure --
  reproduces byte-for-byte identically (same "assert 2 == 4"/"assert
  1 == 5", same merged volume) on the pre-session baseline.** Traced far
  enough to rule out this session's own `Gsplit` changes as the cause: a
  live `Gsplit`-call trace of the same `cylinder_box.mcnp` reconstruction
  shows every tool used is `GCylinder`/`GPlane`/`GSphere` -- never a cone
  -- so `_try_coaxial_cone_split` (which only engages when `_find_cone_face(tool)`
  finds one) never even runs here. Not otherwise root-caused this
  session -- a real, pre-existing `GEOReverse`-side bug (or an OCCT-
  version-sensitivity difference between FreeCAD's bundled 7.8.1 and the
  pyOCC engines' 7.9.3, matching this file's own recurring theme) that
  was simply never caught before because `tests/test_csgtocad.py` had
  apparently not been run against `occ` at all, and not recently against
  `ocp`, prior to this session's own end-to-end verification pass. Flagged
  as a new, standalone pending item below.

**New, not yet written up anywhere else in this file** (found during this
session's `Solidos/test_models` batch conversion + d1suned run, before the
rc9.stp deep dive -- characterized/triaged but left unfixed when the
session redirected to rc9.stp, then to the 2 crashes above):
- `Big_complex_cell/modelcell_cut1.stp` -- was a bad-tally failure
  (~45.5 sigma, per the 2026-08-21 audit's own "Changed symptom" note),
  now loses particles at runtime instead. Not root-caused either way.
- ~~`Mixed/multiplane_add_plane_cyl.stp` -- d1suned tally ~4.5 sigma off~~
  -- **investigated and improved, 2026-08-23 (later session)**. Turned out
  unrelated to `gen_plane_cone`'s already-fixed apex degeneracy -- the real
  cause was `_find_adjacent_multiplane_planes` (`meta_surfaces_utils.py`)
  finding **zero** adjacent MultiPlane planes for either of this file's 2
  RevCC segments, when it should have found real ones. Root cause,
  confirmed live: the old implementation delegated to
  `get_adjacent_cylplane(..., cornerPlanes=False, axial_bounds=...)`,
  which only accepts a *curved* boundary edge whose own midpoint sits
  within a `1e-3 * range`-scaled tolerance of the segment's true V-extreme
  -- correct for a cylinder/cone cut by a *perpendicular* plane (circular
  rim, midpoint exactly at the extreme), but this file's cylinder is cut
  by a *non-perpendicular* plane, giving an *elliptical* rim whose own
  midpoint sits ~0.05-0.06 off the true extreme against a tolerance of
  only ~0.003 -- silently excluding both of the file's real end-cap edges.
  **Fixed per explicit user-directed strategy change** ("vamos a invertir
  la busquedad"): rather than first guessing which edges could plausibly
  border a closing plane via a shape/position heuristic and only then
  checking whether the result happens to match a known MultiPlane
  component, `_find_adjacent_multiplane_planes` now walks *every* edge of
  the segment's own shell directly and checks whether its real neighbor
  (via `other_face_edge`, sliver-tolerant) is already one of `multiplanes`'
  own component planes -- no shape/position heuristic to get subtly wrong,
  since the candidate set is already fully known in advance.
  `get_adjacent_cylplane` itself is untouched (still used by `meta_surfaces.py`
  for Can/RoundCorner detection) -- only this one caller was rewired to
  stop using it. Verified: d1suned tally 0.980107 (±0.44%, ~4.5 sigma) ->
  **0.991981 (±0.44%, ~1.8 sigma)** -- a real, substantial improvement,
  though not perfectly at 1.0 (a residual ~1.8 sigma gap remains, not
  investigated further this pass). `tests/geo` + `tests/test_cadtocsg.py`
  128/128 under `ocp`; a 109-file `Solidos/test_models` differential
  corpus scan (composite-surface counts, excluding `Big_model_reserved`)
  shows **zero differences** anywhere -- the fix only improves this one
  file's own RevCC accuracy, without changing classification counts
  anywhere else in the corpus.
- `Enclosures/w_encl.stp` cells 4-5 -- zero tally, tied to the
  already-documented, still-unfixed enclosure-duplication bug
  (`LF.remove_enclosure(meta_list)` commented out in `load_step.py`) --
  note this is a *different* symptom from the same file's own
  already-confirmed-fixed 10-lost-particles case documented in the
  2026-08-21 audit above; the two may not be the same finding.
- `Big_complex_cell/modelCell_670000.stp` -- now converts (slow, ~230s)
  but loses 10 particles at runtime. Not investigated.

**Carried forward from the 2026-08-21 audit, still open (not re-verified
this pass)**:
- `AdjacentMultiplanePlanes` needs extending from RevCC to
  MultiRoundCorner too (`project_mrc_adjacent_multiplane_pending.md`).
- ~~`gen_plane_cylinder`/`gen_plane_cone` still operate on a single raw face
  rather than a merged same-surface shell~~ -- **investigated and closed,
  2026-08-23 (later session), NOT a bug.** `get_join_cone_cyl`'s own
  `ifacemin = face_index[UValmin.index(Umin)]` (and `ifacemax` likewise)
  looked, by direct reading, like an indexing mismatch: `UValmin`/`UValmax`
  are built by iterating `sameface_index` (the real, contiguity-filtered
  merge group from `same_faces()`), so the position `UValmin.index(Umin)`
  finds is a position *within* `sameface_index`, not `face_index` (the
  wider, unfiltered candidate list `same_faces()` started from) --
  indexing into the wrong list whenever `same_faces()` actually excludes a
  candidate. Confirmed live on `Solidos/test_models/Reversed_Cyl_Cones/cyl_cone.stp`
  (already a trusted, d1suned-verified-clean fixture, tally 0.996547):
  `face_index=[0,20,22]` but `sameface_index=[0,22]` (face 20 excluded as
  non-contiguous with seed face 0), and the "buggy" line does pick face 20
  instead of 22. **Changing it to `sameface_index[...]` (the seemingly
  "correct" fix) breaks the file outright -- 10 lost particles, tally
  0.908±14%** -- confirmed twice, independently, on 2 different affected
  seed calls (`seed=0`: `ifacemin` 20 vs 22; `seed=4`: `ifacemin` 12 vs
  14; both regress independently when "fixed" alone). Root cause of why
  the "buggy" choice is actually necessary, found by tracing execution
  directly rather than reasoning from the code: `ifacemin`/`ifacemax`
  aren't just used to build the bounding plane's own V1/V2 reference
  points -- they're also used to select *which face's own boundary edges*
  get searched to find `adjacent1`/`adjacent2`, the faces the recursive
  chain-following (`new_adjacent1 = get_join_cone_cyl(adjacent1, ...)`)
  continues into. For `cyl_cone.stp`'s seed=0: `ifacemin=20` (the
  "wrong"/non-contiguous face) is exactly the face whose own edge search
  finds `adjacent1=2`, a real, valid chain continuation
  (`chain-continue=True`) -- `ifacemin=22` (the "correct"/contiguous face)
  would search a *different* face's edges instead, finding whatever
  adjacent face is really next to face 22, which does not continue this
  same real chain. So the wider, unfiltered `face_index` list is the
  *intentionally* correct one to index into here: `same_faces()`'s own
  contiguity filter is right for deciding which faces belong to the
  literal merged same-surface patch (used correctly, elsewhere, for
  `omitFaces.update(sameface_index)`), but the specific face used to
  *continue the chain* needs the wider candidate set, since the true next
  segment can be reached through a same-cylinder-identity face that isn't
  itself part of the immediate contiguous patch. **Explicit user
  decision, once this was understood: leave the code exactly as it is.**
  No fix applied; item closed, not because it's provably optimal, but
  because two independent, direct regression tests confirm the current
  behavior is what the model actually needs, and no alternative was found
  that doesn't break it.
- Isolating the RevCC-corpus differential scan's bug-2 vs bug-4
  contributions separately (109-file `Solidos/test_models` scan, 24 files
  differed after the 2026-08-22 fixes landed together) -- not yet split
  apart to confirm neither fix is masking a problem in the other.
- `check_sign` verification of RevCC from the conversion side (never
  attempted -- decomposition-side scanning structurally can't reach it,
  per this file's own earlier explanation).
- A full `Solidos/` corpus differential scan specifically under the raw
  `occ` (pythonocc-core) engine (as opposed to `ocp`, now the default, or
  `freecad`) -- never run.
- `Gload_step_labels`'s FreeCAD-style auto-suffix naming gap for multiple
  solids sharing one XCAF label, under `occ`/`ocp` -- narrow, non-blocking.
- Enclosure solids duplicated into `meta_list` -- the fix
  (`LF.remove_enclosure`) exists and is correct but its call is still
  commented out in `load_step.py`.
- The 6 exotic quadric surfaces (`Gmake_elliptic_cone`/`Gmake_hyperboloid`/
  etc.) remain `_not_implemented(...)` stubs in `GEOReverse`'s pyOCC
  backends.
- `Solidos/Torus/2_degen_torii.stp` -- the hard MCNP fatal error is gone,
  but it now hits the ordinary 10-lost-particle abort instead. Still open.
- `Solidos/` STEP fixture tree reorganization/dedup -- only partially
  done (`test_models/` + a couple of `duplicates_removed/` moves); the
  user's own "son muchos sólidos y seguro que muchos serán duplicados"
  concern is still largely unaddressed.
- The GEOReverse (CsgToCad) round-trip discrepancy on `hylife-v06.stp`
  cell 1 (reconstructed CAD volume 1.401x the true solid vs. d1suned's own
  1.119x tally on the same unfixed file -- the two don't agree) --
  surfaced during the `add_reversedCC` investigation, not pursued once the
  real fix was found via the boolean-formula route instead. Still open.
- `tests/test_csgtocad.py::test_cylbox_convertion[mcnp]`/`[openmc_xml]`
  fail under both `occ` and `ocp` (`GEOReverse` producing 2/1 solids in
  the exported STEP instead of the expected 4/5) -- confirmed
  pre-existing (reproduces identically with this entire session's `src/`
  changes stashed out) and confirmed unrelated to this session's own
  `Gsplit` coaxial-cone work (a live trace shows no cone tool is ever
  used reconstructing `cylinder_box.mcnp`, so `_try_coaxial_cone_split`
  never engages). `freecad` passes both tests cleanly. New this session
  (2026-08-23 evening), not carried forward from an earlier audit.
  **Narrowed further, same session**: `cylinder_box.mcnp`'s "Solid Cells:
  1" header means only cell 1 is real material (cells 2/3/4 are the
  auto-generated void/graveyard-in/graveyard cells) -- cell 1's own
  definition is a single boolean expression combining 6 `:`(OR)-separated
  sub-clauses, so the 4 `_EXPECTED_VOLUMES["mcnp"]` entries
  (1520814.9834 + 3864483.442 + 20092792.2374 = 25478090.66, plus the
  graveyard's 7.999999999974521e18) represent cell 1's own real material
  as **3 genuinely separate/disjoint solid pieces** -- confirmed by the
  coincidence that 25478090.66 matches, to ~0.5 out of 25 million, a
  value (`25478091.187395196`) already seen mid-construction in an
  unrelated `Gsplit` trace (the sphere-cut fragment kept after removing
  the graveyard). `interferencia` (`GEOReverse/Modules/buildCAD.py`,
  the function whose own `fuse_solids(cellParts)` call was the first,
  natural suspect for "3 things got merged into fewer") was **ruled out
  directly, not by inspection alone** -- live-traced and confirmed it
  never even fires for this file: `ContainerCell.shape` (the root
  universe's own container cell) is `None` at the top level, so
  `BuildUniverseCells`'s own `if universeCut and ContainerCell.shape:`
  guard short-circuits before `interferencia` is ever called; it only
  matters for *nested* sub-universes, which this file has none of. The
  real merge must be happening inside `Objects.py`'s own
  `CellObj.buildShape` (building cell 1's 6-OR-term definition into a
  single CAD shape) -- not yet traced into. Per the test's own header
  comment, `_EXPECTED_VOLUMES` was captured from FreeCAD's *own*,
  pre-migration reconstruction of this exact file, so this may be a
  genuine algorithmic difference introduced somewhere across this
  project's whole multi-session pyOCC migration (how many separate
  solids a multi-OR-term cell's construction keeps vs. fuses), not
  necessarily a bug in any one recent change. **Explicit user decision:
  stop here for now** -- `GEOReverse` investigation is deliberately
  deferred until `GEOUNED`'s own forward pipeline is fully clean of
  known bugs; this item stays open, picked up only once that's true.
- `Mixed/multiplane_add_plane_cyl.stp`'s own residual ~1.8 sigma gap
  (was ~4.5 sigma, fixed by `_find_adjacent_multiplane_planes`'s inverted
  search -- see that section above) -- not investigated further this
  session; the fix landed a real, verified improvement, but the file
  isn't perfectly at tally 1.0 yet.

## Session close, 2026-08-23 (evening) -- GEOUNED-side status for the
next session

Per the user's own explicit priority ordering (stated twice this
session): finish `GEOUNED` (the forward CAD-to-CSG pipeline) before
picking `GEOReverse` back up. As of this commit, the concrete `GEOUNED`-side
punch list (excluding anything already marked `GEOReverse`-only above) is:
`Big_complex_cell/modelcell_cut1.stp` (lost particles, not root-caused),
`Enclosures/w_encl.stp` cells 4-5 (zero tally, tied to the still-commented-out
`LF.remove_enclosure` call), `Big_complex_cell/modelCell_670000.stp`
(lost particles, not investigated), `Solidos/Torus/2_degen_torii.stp`
(lost-particle abort, was a harder fatal error before), `multiplane_add_plane_cyl.stp`'s
own residual ~1.8 sigma (just above), `AdjacentMultiplanePlanes` needing
the same RevCC-to-MultiRoundCorner extension flagged since 2026-08-21, and
the still-unexercised `occ`-engine-specific full corpus scan. Everything
in this session's own work (piece5/coaxial-cone fix, `CharacteristicWidth`
cross-engine port, the `gen_plane_cylinder`/`gen_plane_cone` investigation
closed as not-a-bug, and this last `_find_adjacent_multiplane_planes` fix)
is committed and pushed to `origin/georeverse-migration` -- working tree
clean.

## Code style preference

- User prefers speaking/planning in Spanish, but ALL code — including
  comments, docstrings, and variable/function names — must be written
  in English.
