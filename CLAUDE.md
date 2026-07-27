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

## Migration plan (in progress, in this order)

1. **Code cleanup first, before touching geometry dependencies:**
   - Standardize style: PEP 8 naming, `ruff`/`black` for lint+format,
     `mypy`/`pyright` for type hints (note: found at least one mutable
     default argument bug pattern, `skip_solids=[]`, to fix during cleanup).
   - Reorganize folder structure: split large modules by responsibility
     (io / decomposition / surfaces / export), consolidate config into
     `pyproject.toml`, add `pre-commit`.
   - Build bridge/adapter layer (see below) BEFORE swapping the backend,
     so the geometry-engine dependency is isolated to one place.
2. **Introduce a `GeometryBackend` abstract interface** (Adapter /
   Ports & Adapters pattern) so the rest of GEOUNED never imports
   `Part`, `FreeCAD`, or `OCC.Core` directly — only neutral wrapper
   types and this interface. See `geometry_backend_interface.py`
   (attached alongside this file / already in the repo if committed).
   Key design points:
   - `GSolid`/`GFace`/`GEdge`/`GVertex` are opaque wrappers carrying a
     `native` object + a reference to the backend that produced it.
   - `classify_surface()` is the highest-value method: translates a
     face into neutral `SurfaceType` + params (plane/cylinder/cone/
     sphere/torus) — this is where most of today's FreeCAD-specific
     logic in the decomposition module likely lives.
   - `split()` returns a `SplitResult` that must never silently return
     an uncut solid; the backend is responsible for resolving
     degenerate/tangency cases internally (this is where the fix for
     the motivating problem above should live, encapsulated).
   - `faces_sharing_edge()` exposes the non-manifold-edge diagnostic
     directly (edges shared by != 2 faces).
3. **Implement `FreeCADBackend`** against the current `Part`/`FreeCAD`
   API, validating the interface covers everything `decomposition/`
   actually needs.
4. **Implement `OCCBackend`** against pythonocc-core, with the
   degenerate-split handling built in.
5. Run existing test suite (`tests/`, `testing/`, CI via `ci.yml`)
   parametrized over both backends to catch numerical divergences
   before removing the FreeCAD backend.

## Code style preference

- User prefers speaking/planning in Spanish, but ALL code — including
  comments, docstrings, and variable/function names — must be written
  in English.

## Artifacts already produced in the planning conversation

- `geometry_backend_interface.py`: full draft of the `GeometryBackend`
  ABC plus neutral dataclasses (`GSolid`, `GFace`, `GEdge`, `GVertex`,
  `GVector`, `SurfaceType`, `PlaneParams`/`CylinderParams`/`ConeParams`/
  `SphereParams`/`TorusParams`, `SurfaceGeometry`, `SplitResult`). Not
  yet validated against the real `decomposition/` module — next step
  is sketching `FreeCADBackend` against it to check coverage.
