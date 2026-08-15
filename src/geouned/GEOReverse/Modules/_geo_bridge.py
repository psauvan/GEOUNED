"""
GEOReverse/Modules/_geo_bridge.py

The single point of import for the rest of GEOReverse's core geometry
pipeline: `from ._geo_bridge import GVector, GSolid, Gmake_cylinder, ...`.
Mirrors the role `geo/__init__.py` plays for GEOUNED -- and, as of this
module, genuinely delegates to it (`from ...geo import (...)`) rather
than hardcoding a backend, so GEOReverse's core follows the same
`GEOUNED_CAD_ENGINE` environment variable GEOUNED's forward pipeline
does. `CAD_ENGINE` below records which one actually resolved, for
`core.py::export_cad`'s own per-engine dispatch/validation.

Two things still don't follow the engine switch, both deliberately, both
narrower than "GEOReverse is FreeCAD-only" used to be:
- `geo_quadrics/` (the 6 exotic quadric surfaces GEOUNED's own
  decomposition never produces) is its own small package mirroring this
  same `__init__.py`-dispatch pattern (`geo_quadrics/_freecad_impl.py` +
  `geo_quadrics/_occ_impl.py`) -- but the pyOCC side is currently an
  empty stub (every `Gmake_*` raises `NotImplementedError`), since a real
  port needs pyOCC equivalents of `Part.Ellipse`/`Part.Hyperbola`/
  `.revolve()`/`Part.makeLoft` that haven't been built yet. That package
  imports `FreeCAD`/`Part` directly in its own `_freecad_impl.py`, the
  same way `geo/_freecad_impl.py` does -- not through this module.
- `core.py::export_cad`'s `.FCStd` output has no pyOCC equivalent at
  all (no such document concept) -- handled by that function's own
  per-engine `_SUPPORTED_FORMATS` validation, not by anything here.

GEOReverse represents its own affine transforms (MCNP TRn cards,
universe nesting) as plain numpy 4x4 arrays -- its own working matrix
type, chosen for the battle-tested composition/inversion numpy already
gives for free (`@`, `numpy.linalg.inv`). The *only* neutral types that
ever cross between GEOReverse and `geo` are `GVector` and `GMatrix`;
numpy arrays never convert directly to/from a native type --
`to_np_matrix`/`to_gmatrix_from_np` below always go through `GMatrix`
(via `geo`'s own `to_gmatrix`/`to_native_matrix`, both already
engine-dispatched by `geo/__init__.py`), never around it. Native
geometry types (`FreeCAD.Matrix`/`FreeCAD.Vector`, or their pyOCC
equivalents) should never appear anywhere in GEOReverse outside this one
file, `geo_quadrics/_freecad_impl.py`/`_occ_impl.py`, and `_freecad_impl.py`/
`_occ_impl.py` (the export-side pair, used only by `core.py`).
"""

import numpy as np

from ...geo import (
    CAD_ENGINE,
    GBoundBox,
    GBSpline,
    GCircle,
    GCone,
    GCylinder,
    GEdge,
    GEllipse,
    GFace,
    GLine,
    GMatrix,
    GPlane,
    GShape,
    GShell,
    GSolid,
    GSphere,
    GTorus,
    GVector,
    GWire,
    SplitResult,
    Gclassify_curve,
    Gclassify_surface,
    Gcommon,
    Gcut,
    Gdistance,
    Gexport_step,
    Gfirst_shell,
    Gfuse,
    Gin_contact,
    Gload_step,
    Gload_step_labels,
    Gmake_box,
    Gmake_compound,
    Gmake_cone,
    Gmake_cone_double_sheet,
    Gmake_cone_frustum,
    Gmake_cylinder,
    Gmake_half_space,
    Gmake_polygon_face,
    Gmake_shell,
    Gmake_sphere,
    Gmake_torus,
    Gmake_wire,
    Gsplit,
    kernel_version,
    pick_outer_wire,
    to_native_matrix,
    to_native_vector,
    to_gboundbox,
    to_gmatrix,
    to_gvector,
)

IDENTITY_MATRIX = np.eye(4)


def to_np_matrix(matrix: GMatrix) -> np.ndarray:
    """GMatrix -> 4x4 row-major numpy array (GEOReverse's own transform
    representation). Never takes a native FreeCAD.Matrix directly --
    callers holding one convert it via `geo`'s own `to_gmatrix` first."""
    return np.array(
        [
            [matrix.A11, matrix.A12, matrix.A13, matrix.A14],
            [matrix.A21, matrix.A22, matrix.A23, matrix.A24],
            [matrix.A31, matrix.A32, matrix.A33, matrix.A34],
            [matrix.A41, matrix.A42, matrix.A43, matrix.A44],
        ]
    )


def to_gmatrix_from_np(matrix: np.ndarray) -> GMatrix:
    """Inverse of `to_np_matrix` -- 4x4 numpy array -> GMatrix."""
    a = [float(v) for v in matrix.flatten()]
    return GMatrix(*a)


def transform_solid(solid: GSolid, matrix: np.ndarray) -> GSolid:
    """Apply a numpy affine transform to a `GSolid`. Routes through
    `GMatrix` (`to_gmatrix_from_np` then `to_native_matrix`) since
    `GSolid.transform_geometry` has no GMatrix-accepting form of its own --
    there is no direct numpy -> native shortcut anywhere in this module."""
    return solid.transform_geometry(to_native_matrix(to_gmatrix_from_np(matrix)))


def matrix_multVec(matrix: np.ndarray, v: GVector) -> GVector:
    """Full affine transform of a position (rotation + translation) --
    numpy equivalent of native `FreeCAD.Matrix.multVec`. Pure GVector/numpy
    math, no native detour needed."""
    r = matrix[:3, :3] @ np.array([v.x, v.y, v.z]) + matrix[:3, 3]
    return GVector(float(r[0]), float(r[1]), float(r[2]))


def matrix_rotate_vec(matrix: np.ndarray, v: GVector) -> GVector:
    """Rotation-only transform of a direction (no translation) -- numpy
    equivalent of native `FreeCAD.Matrix.submatrix(3).multVec`."""
    r = matrix[:3, :3] @ np.array([v.x, v.y, v.z])
    return GVector(float(r[0]), float(r[1]), float(r[2]))


def fuse_solids(parts: list) -> GSolid | None:
    """Boolean-union `parts` (a list of `GSolid`) into one solid,
    tolerating a failed/invalid fuse by falling back to an unfused
    compound. The single shared implementation for what used to be 3
    byte-for-byte-identical copies (`Objects.py`, `buildSolidCell.py`,
    `splitFunction.py`) -- consolidated here per the migration plan's
    Phase 5. Uses `GSolid.refine()` (not a raw, unguarded
    `removeSplitter()`) so the fused result gets that method's existing
    volume-invariance safety check for free."""
    if len(parts) == 0:
        return None
    if len(parts) == 1:
        solid = parts[0]
    else:
        try:
            fused = Gfuse(parts)
        except Exception:
            fused = None

        if fused is not None:
            try:
                refined = fused.refine()
            except Exception:
                refined = fused

            if refined.is_valid():
                solid = refined
            elif fused.is_valid():
                solid = fused
            else:
                solid = Gmake_compound(parts)
        else:
            solid = Gmake_compound(parts)

    if solid.Volume < 0:
        solid = solid.reverse()
    return solid
