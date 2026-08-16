"""
GEOReverse/Modules/matrix_utils.py

GEOReverse-specific extras that don't belong inside `geo` itself, kept in
their own module rather than a `geo`-re-exporting middleman -- every
other GEOReverse file imports `geo` names directly
(`from ...geo import GSolid, Gsplit, ...`), the same pattern GEOUNED's
own forward pipeline already uses everywhere.

GEOReverse represents its own affine transforms (MCNP TRn cards,
universe nesting) as plain numpy 4x4 arrays -- its own working matrix
type, chosen for the battle-tested composition/inversion numpy already
gives for free (`@`, `numpy.linalg.inv`). The *only* neutral type that
ever crosses between this representation and `geo` is `GMatrix`; numpy
arrays never convert directly to/from a native type -- `to_np_matrix`/
`to_gmatrix_from_np` below always go through `GMatrix` (via `geo`'s own
`to_gmatrix`/`to_native_matrix`, both already engine-dispatched by
`geo/__init__.py`), never around it.

`fuse_solids` is likewise a GEOReverse policy (how to combine solids
robustly, with a safe fallback), not a `geo` primitive -- consolidates
what used to be 3 byte-for-byte-identical copies (`Objects.py`,
`buildSolidCell.py`, `splitFunction.py`).
"""

import numpy as np

from ...geo import GMatrix, GSolid, GVector, Gfuse, Gmake_compound, to_native_matrix

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
    compound. Uses `GSolid.refine()` (not a raw, unguarded
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
