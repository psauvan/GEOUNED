"""
geo/ocp/_native_utils.py

Native-conversion helpers and kernel provenance -- split out of the
former monolithic _ocp_impl.py (2026-08-30), grouped by function
alongside its freecad/occ siblings of the same name.
"""

from __future__ import annotations

import OCP
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.GProp import GProp_GProps
from OCP.gp import (
    gp_Dir,
    gp_Pnt,
    gp_Trsf,
    gp_Vec,
)
from OCP.ShapeFix import ShapeFix_Shape
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from ..vector_geometry import (
    GBoundBox,
    GMatrix,
    GVector,
    to_gvector,
)


def _unimplemented(name):
    def _raise(*args, **kwargs):
        raise NotImplementedError(f"geo.ocp.{name} is not implemented yet.")

    _raise.__name__ = name
    return _raise


def kernel_version() -> str:
    return OCP.__version__


def to_native_vector(vector: GVector) -> gp_Pnt:
    """Name kept for cross-backend API compatibility -- despite the
    name, this returns an OCP gp_Pnt, not a FreeCAD.Vector."""
    return gp_Pnt(vector.x, vector.y, vector.z)


def to_native_matrix(matrix: GMatrix) -> gp_Trsf:
    """Name kept for cross-backend API compatibility with
    _freecad_impl.py's own to_native_matrix -- despite the name, this returns
    an OCP gp_Trsf, not a FreeCAD.Matrix. Assumes `matrix` represents a
    rigid transform (rotation + translation, no scale/shear): true for
    every matrix GEOReverse ever builds (MCNP TRn cards), the only real
    consumer of this function today."""
    trsf = gp_Trsf()
    trsf.SetValues(
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
    )
    return trsf


def _to_gvector(pnt) -> GVector:
    """Internal: native gp_Pnt/gp_Dir/gp_Vec -> GVector. Not exported --
    the public `to_gvector` (vector_geometry.py) duck-types on .x/.y/.z
    attributes, which these OCP types don't have (they use .X()/.Y()/.Z()
    methods instead)."""
    return GVector(pnt.X(), pnt.Y(), pnt.Z())


def _orientation_str(shape) -> str:
    """TopAbs_Orientation (FORWARD=0/REVERSED=1/...) -> the "Forward"/
    "Reversed" strings GEOUNED compares against everywhere, matching
    FreeCAD's own native .Orientation string attribute."""
    return "Reversed" if shape.Orientation() == 1 else "Forward"


def _to_gmatrix_3x3(mat) -> GMatrix:
    """A gp_Mat (3x3, .Value(i,j) 1-indexed) -> GMatrix. Only A11..A33
    are ever meaningful for GEdge/GWire.MatrixOfInertia (see GMatrix's
    own docstring) -- the rest of the 4x4 is filled with the
    translation-free identity convention (A44=1, all else 0)."""
    return GMatrix(
        mat.Value(1, 1),
        mat.Value(1, 2),
        mat.Value(1, 3),
        0.0,
        mat.Value(2, 1),
        mat.Value(2, 2),
        mat.Value(2, 3),
        0.0,
        mat.Value(3, 1),
        mat.Value(3, 2),
        mat.Value(3, 3),
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )


def _linear_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    BRepGProp.LinearProperties_s(shape, props)
    return props


def _surface_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, props)
    return props


def _volume_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props


def _edge_curve_and_range(native_edge):
    """Restores pythonocc-core's BRep_Tool.Curve(edge) -> (curve, first,
    last) convenience (or None if the edge is degenerate/has no 3D
    curve) -- OCP's BRep_Tool.Curve_s needs dummy first/last args (their
    values are discarded, verified live) and BRep_Tool.Range_s is a
    separate call that returns the real (first, last) tuple on its own."""
    curve = BRep_Tool.Curve_s(native_edge, 0.0, 0.0)
    if curve is None:
        return None
    first, last = BRep_Tool.Range_s(native_edge)
    return curve, first, last


def _project_point_on_surface(point: GVector, geom_surface) -> tuple[float, float]:
    """(u, v) of the nearest point on `geom_surface` to `point`. The
    default GeomAPI_ProjectPointOnSurf algorithm (Extrema_ExtAlgo_Grad,
    gradient-based) occasionally fails to converge (StdFail_NotDone) on
    a real, well-formed surface/point pair -- confirmed on a real case
    in PiezaDavid.stp -- so this retries with the more exhaustive
    Extrema_ExtAlgo_Tree before giving up. Confirmed via that same case:
    even Extrema_ExtAlgo_Tree can genuinely fail for a point sitting
    exactly on a cylinder's own axis (r=0) -- there the "nearest point"
    isn't mathematically unique (every point on the circle at that
    height is equidistant), not a numerical fluke, so no algorithm can
    do better; falls back to (0.0, 0.0), an arbitrary-but-deterministic
    answer for what is an inherently undefined query at that exact point."""
    from OCP.Extrema import Extrema_ExtAlgo_Tree

    proj = GeomAPI_ProjectPointOnSurf(to_native_vector(point), geom_surface)
    if proj.IsDone():
        return proj.LowerDistanceParameters()
    proj = GeomAPI_ProjectPointOnSurf(to_native_vector(point), geom_surface, Extrema_ExtAlgo_Tree)
    if proj.IsDone():
        return proj.LowerDistanceParameters()
    return (0.0, 0.0)


def _bnd_box(shape) -> GBoundBox:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return GBoundBox(xmin, ymin, zmin, xmax, ymax, zmax)


def _native_fix(native, tolerance: float):
    """Native-in/native-out extraction of `GSolid.fix()`'s own body --
    see that method's docstring for the full, hard-won history of why
    every step here is exactly what it is (UnifyEdges-only-when-already-
    valid, UnifyFaces' own confirmed native-crash risk kept on
    unconditionally per explicit user decision, the fallback repair
    running on the original `native` rather than UnifyEdges' own
    possibly-corrupted output). `GSolid.fix()` itself is a thin wrapper
    around this (`GSolid(_native_fix(self.__native__, tolerance))`) --
    single source of this fragile logic, not two copies to keep in sync,
    while still letting `Gload_and_process_step` call it directly on the
    raw loaded shape, before that loop builds its own `GSolid`."""
    if BRepCheck_Analyzer(native).IsValid():
        unify = ShapeUpgrade_UnifySameDomain(native, UnifyEdges=True, UnifyFaces=True, ConcatBSplines=True)
        unify.Build()
        unified = unify.Shape()
        if BRepCheck_Analyzer(unified).IsValid():
            return unified
    fixer = ShapeFix_Shape(BRepBuilderAPI_Copy(native).Shape())
    fixer.SetPrecision(tolerance)
    fixer.Perform()
    return fixer.Shape()
