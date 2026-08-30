"""
geo/freecad/_native_utils.py

Native-conversion helpers and kernel provenance -- split out of the
former monolithic _freecad_impl.py (2026-08-30), grouped by function
alongside its occ/ocp siblings of the same name.
"""

from __future__ import annotations

import FreeCAD

from ..vector_geometry import GMatrix, GVector


def to_native_vector(vector: GVector) -> FreeCAD.Vector:
    """Write-side half of the transitional pair with `vector_geometry.to_gvector` --
    materializes a neutral GVector back into a native FreeCAD.Vector, needed only
    where geo code calls a native Part/FreeCAD function directly."""
    return FreeCAD.Vector(vector.x, vector.y, vector.z)


def to_native_matrix(matrix: GMatrix) -> FreeCAD.Matrix:
    """Write-side half of the transitional pair with `vector_geometry.to_gmatrix` --
    materializes a neutral GMatrix back into a native FreeCAD.Matrix, needed only
    where geo code calls a native Part/FreeCAD function directly (e.g.
    `GSolid.transform_geometry`, which has no GMatrix-accepting overload)."""
    return FreeCAD.Matrix(
        matrix.A11,
        matrix.A12,
        matrix.A13,
        matrix.A14,
        matrix.A21,
        matrix.A22,
        matrix.A23,
        matrix.A24,
        matrix.A31,
        matrix.A32,
        matrix.A33,
        matrix.A34,
        matrix.A41,
        matrix.A42,
        matrix.A43,
        matrix.A44,
    )


def kernel_version() -> str:
    """
    Version string of the underlying geometry kernel/application (e.g.
    FreeCAD's own version), for provenance notes in output file headers.
    Purely informational -- callers must not parse or branch on the format.
    """
    return "{V[0]}.{V[1]}.{V[2]}".format(V=FreeCAD.Version())
