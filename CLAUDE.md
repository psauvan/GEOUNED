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
  ...) instead of being passed to a separate backend object; free `Gmake_*`
  constructor functions (`Gmake_box`, `Gmake_cylinder`, `Gmake_half_space`,
  `Gmake_polygon_face`, ...) and free operation functions for anything
  combining more than one independent shape (`Gcut`, `Gcommon`, `Gfuse`,
  `Gsplit`, `Gin_contact`, `Gdistance`, `Gload_step`, `Gload_step_labels`,
  `Gexport_step`).
- `geo/__init__.py` — the single import point for the rest of GEOUNED:
  `from ...geo import GSolid, Gmake_cylinder, ...`.

As of the file-by-file closeout pass, 6 files in `GEOUNED` still carry a
direct `import Part`/`import FreeCAD`, each scoped to a small, itemized
set of deliberately deferred native uses with no faithful `geo` equivalent
(verified: everything else in those files — classification, construction,
vector arithmetic — now goes through `geo`). All are duck-typed native
*values* flowing through, not module-level dependencies leaking outward —
nothing outside these 6 files needs to `import Part`/`FreeCAD` itself to
consume them.
- `utils/geouned_classes.py`: `Box = FreeCAD.BoundBox(boundBox)` in
  `GeounedSurface.build_surface` — the box is threaded natively into
  `build_shape_functions.py`'s `Plane`/`Cylinder`/`Cone`/`MultiPlane`/
  `Can`/`TCone`/`RoundCorner`/`MultiRoundCorner` branches, which need
  `Box.getEdge(i)`/`Box.getPoint(i)` (no `GBoundBox` equivalent).
- `core.py`: `self.geometry_bounding_box = FreeCAD.BoundBox(...)` in
  `_set_geometry_bounding_box` — consumed natively well outside this
  migration's scope (`void.py`, `write_files.py`,
  `geouned_classes.py`'s `self.UniverseBox`).
- `conversion/cell_definition_functions.py`: `Part.Plane(sphere_center,
  normal).toShape()` in `gen_plane_sphere` — an actual infinite analytic
  plane used for `distToShape`; `Gmake_half_space` is a *box-clipped*
  (1e6-extent) approximation, not faithful enough to swap in.
- `utils/geometry_gu.py`: `Part.Vertex(pos1)`/`Part.Vertex(pos2)` in
  `same_wire` — no `geo` constructor for a lone vertex.
- `utils/meta_surfaces_utils.py`: one `isinstance(e0.Curve, (Part.Circle,
  Part.Ellipse, Part.Hyperbola, Part.Parabola))` in `planar_edges` —
  `Gclassify_curve` doesn't model Hyperbola/Parabola, so narrowing this
  to the 2 supported kinds would silently change behavior for the other 2.
- `utils/build_shape_functions.py` (biggest file, still the most native):
  `Part.Plane(...)` + the `cut_face`/`cut_box` analytic-intersection
  machinery (same infinite-plane gap as above), `Part.makeSolid(shell)`
  (no `Gmake_solid`), `Part.makeCone(...)` (kept native to avoid an
  avoidable `atan`/`tan` round-trip vs. `Gmake_cone`'s `half_angle` API),
  and the `box: FreeCAD.BoundBox` parameter threaded through from
  `geouned_classes.py` above.

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
