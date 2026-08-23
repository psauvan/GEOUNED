"""
geo/_occ_impl.py

pythonocc-core (pyOCC) implementation of the `geo` package -- a sibling
to `_freecad_impl.py`, selected instead of it by `geo/__init__.py` when
GEOUNED_CAD_ENGINE=occ. Mirrors `_freecad_impl.py`'s class/function
names and field names (matching FreeCAD's own attribute names, e.g.
GPlane.Position/.Axis/.XDir) so the rest of GEOUNED does not change
regardless of which backend is selected.

`GEOReverse` (CsgToCad) is out of scope for this file entirely -- it
keeps importing FreeCAD/Part directly, unconditionally, regardless of
GEOUNED_CAD_ENGINE (see geouned/__init__.py).

Known gaps (see CLAUDE.md's pyOCC migration section for the full
history): `Gload_step_labels` (STEP assembly/label-tree reading, a
separate concern from `Gload_step`'s pure geometry read) is not yet
implemented; `Gclassify_surface`'s BSplineSurface-secretly-a-plane
fallback (FreeCAD's face.findPlane()) has no pyOCC port yet, so a
mislabeled flat face returns None here instead of a GPlane.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import OCC
from OCC.Core.BOPAlgo import BOPAlgo_Splitter
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Builder, BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Splitter
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Copy,
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_Transform,
)
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepLib import breplib
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
    BRepPrimAPI_MakeTorus,
)
from OCC.Core.BRepTools import breptools
from OCC.Core.BRepTopAdaptor import BRepTopAdaptor_FClass2d
from OCC.Core.Geom2d import Geom2d_Line, Geom2d_TrimmedCurve
from OCC.Core.GeomAbs import (
    GeomAbs_Circle,
    GeomAbs_Cone,
    GeomAbs_Cylinder,
    GeomAbs_Ellipse,
    GeomAbs_Line,
    GeomAbs_Plane,
    GeomAbs_Sphere,
    GeomAbs_Torus,
)
from OCC.Core.GeomAPI import GeomAPI_ProjectPointOnCurve, GeomAPI_ProjectPointOnSurf
from OCC.Core.GeomLProp import GeomLProp_CLProps, GeomLProp_SLProps
from OCC.Core.GProp import GProp_GProps
from OCC.Core.gp import gp_Ax1, gp_Ax2, gp_Ax3, gp_Dir, gp_Dir2d, gp_Pnt, gp_Pnt2d, gp_Trsf, gp_Vec
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
from OCC.Core.ShapeFix import ShapeFix_Shape
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_IN, TopAbs_SOLID, TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shell, TopoDS_Vertex, topods
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape, TopTools_ListOfShape

from .vector_geometry import (
    GBoundBox,
    GLabelNode,
    GMatrix,
    GVector,
    cylinder_tangent_at,
    cylinder_value_at,
    is_coaxial_cone_pair,
    is_inside_cone,
    is_inside_cylinder,
    is_inside_plane,
    is_inside_sphere,
    is_inside_torus,
    plane_tangent_at,
    plane_value_at,
    to_gvector,
)


def _unimplemented(name):
    def _raise(*args, **kwargs):
        raise NotImplementedError(f"geo._occ_impl.{name} is not implemented yet.")

    _raise.__name__ = name
    return _raise


def kernel_version() -> str:
    return OCC.VERSION


def to_native_vector(vector: GVector) -> gp_Pnt:
    """Name kept for cross-backend API compatibility -- despite the
    name, this returns a pyOCC gp_Pnt, not a FreeCAD.Vector."""
    return gp_Pnt(vector.x, vector.y, vector.z)


def to_native_matrix(matrix: GMatrix) -> gp_Trsf:
    """Name kept for cross-backend API compatibility with
    _freecad_impl.py's own to_native_matrix -- despite the name, this returns
    a pyOCC gp_Trsf, not a FreeCAD.Matrix. Assumes `matrix` represents a
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
    attributes, which these pyOCC types don't have (they use .X()/.Y()/.Z()
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
    brepgprop.LinearProperties(shape, props)
    return props


def _surface_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    brepgprop.SurfaceProperties(shape, props)
    return props


def _volume_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    return props


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
    from OCC.Core.Extrema import Extrema_ExtAlgo_Tree

    proj = GeomAPI_ProjectPointOnSurf(to_native_vector(point), geom_surface)
    if proj.IsDone():
        return proj.LowerDistanceParameters()
    proj = GeomAPI_ProjectPointOnSurf(to_native_vector(point), geom_surface, Extrema_ExtAlgo_Tree)
    if proj.IsDone():
        return proj.LowerDistanceParameters()
    return (0.0, 0.0)


def _bnd_box(shape) -> GBoundBox:
    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return GBoundBox(xmin, ymin, zmin, xmax, ymax, zmax)


# ---------------------------------------------------------------------------
# Analytic surface descriptors (wrap a native face's classified surface, not
# the face itself; the backend only knows about these 5 -- composite
# meta-surfaces (RoundCorner, Can, TCone, MultiPlane...) are assembled by
# GEOUNED itself out of these via Gcut/Gcommon/Gfuse, not modeled here)
# ---------------------------------------------------------------------------


class GPlane:
    """Field names match FreeCAD's own `Part.Plane` attribute names."""

    def __init__(self, gp_pln, geom_surface=None):
        ax3 = gp_pln.Position()
        self.Position = _to_gvector(gp_pln.Location())
        self.Axis = _to_gvector(ax3.Direction())
        self.XDir = _to_gvector(ax3.XDirection())
        self.__native__ = geom_surface

    @classmethod
    def from_values(cls, position: GVector, axis: GVector, xdir: GVector | None = None) -> "GPlane":
        plane = cls.__new__(cls)
        plane.Position = position
        plane.Axis = axis
        plane.XDir = xdir
        plane.__native__ = None
        return plane

    def parameter(self, point: GVector) -> tuple[float, float]:
        return _project_point_on_surface(point, self.__native__)

    def value_at(self, u: float, v: float) -> GVector:
        return plane_value_at(self, u, v)

    def tangent_at(self, u: float, v: float) -> tuple[GVector, GVector]:
        return plane_tangent_at(self, u, v)

    def is_inside(self, point: GVector) -> bool:
        return is_inside_plane(point, self)

    def transform(self, matrix: gp_Trsf) -> "GPlane":
        position = _to_gvector(to_native_vector(self.Position).Transformed(matrix))
        axis_pnt = gp_Pnt(self.Axis.x, self.Axis.y, self.Axis.z).Transformed(matrix)
        origin = gp_Pnt(0, 0, 0).Transformed(matrix)
        axis = GVector(axis_pnt.X() - origin.X(), axis_pnt.Y() - origin.Y(), axis_pnt.Z() - origin.Z()).normalized()
        return GPlane.from_values(position, axis)

    def intersect_plane(self, other: "GPlane") -> "GLine | None":
        """Pure GVector math for the well-conditioned case (see
        _freecad_impl.py's GPlane.intersect_plane docstring for the
        verified boundary this mirrors); below that this falls back to
        a transient native intersection via the two Geom_Plane surfaces
        when available, otherwise returns None (a native fallback
        requires both planes to carry a real `.__native__` Geom_Surface,
        which values-constructed planes don't)."""
        n1, n2 = self.Axis, other.Axis
        d = n1.cross(n2)
        dl = d.length
        if dl < 1e-10:
            return None

        if dl < 0.05:
            if self.__native__ is None or other.__native__ is None:
                return None
            from OCC.Core.GeomAPI import GeomAPI_IntSS

            intersector = GeomAPI_IntSS(self.__native__, other.__native__, 1e-7)
            if not intersector.IsDone() or intersector.NbLines() == 0:
                return None
            line_curve = intersector.Line(1)
            pnt = gp_Pnt()
            tangent = gp_Vec()
            line_curve.D1(0.0, pnt, tangent)
            return GLine.from_values(_to_gvector(pnt), _to_gvector(tangent).normalized())

        direction = d.normalized()
        d1 = n1.dot(self.Position)
        d2 = n2.dot(other.Position)
        comps = (d.x, d.y, d.z)
        ax = max(range(3), key=lambda i: abs(comps[i]))
        i, j = [k for k in range(3) if k != ax]
        n1c, n2c = (n1.x, n1.y, n1.z), (n2.x, n2.y, n2.z)
        det = n1c[i] * n2c[j] - n1c[j] * n2c[i]
        xi = (d1 * n2c[j] - d2 * n1c[j]) / det
        xj = (n1c[i] * d2 - n2c[i] * d1) / det
        point_c = [0.0, 0.0, 0.0]
        point_c[ax] = 0.0
        point_c[i] = xi
        point_c[j] = xj
        point = GVector(point_c[0], point_c[1], point_c[2])
        return GLine.from_values(point, direction)

    def intersect_line(self, line: "GLine") -> GVector | None:
        """See _freecad_impl.py's own docstring -- pure GVector math, no native fallback needed."""
        denom = self.Axis.dot(line.Direction)
        if abs(denom) < 1e-12:
            return None
        t = self.Axis.dot(self.Position - line.Position) / denom
        return line.Position + line.Direction * t


class GCylinder:
    def __init__(self, gp_cyl, geom_surface=None):
        ax3 = gp_cyl.Position()
        self.Center = _to_gvector(gp_cyl.Location())
        self.Axis = _to_gvector(ax3.Direction())
        self.Radius = gp_cyl.Radius()
        self.XDir = _to_gvector(ax3.XDirection())
        self.__native__ = geom_surface

    @classmethod
    def from_values(cls, center: GVector, axis: GVector, radius: float, xdir: GVector | None = None) -> "GCylinder":
        cylinder = cls.__new__(cls)
        cylinder.Center = center
        cylinder.Axis = axis
        cylinder.Radius = radius
        cylinder.XDir = xdir
        cylinder.__native__ = None
        return cylinder

    def parameter(self, point: GVector) -> tuple[float, float]:
        return _project_point_on_surface(point, self.__native__)

    def value_at(self, u: float, v: float) -> GVector:
        return cylinder_value_at(self, u, v)

    def tangent_at(self, u: float, v: float) -> tuple[GVector, GVector]:
        return cylinder_tangent_at(self, u, v)

    def is_inside(self, point: GVector) -> bool:
        return is_inside_cylinder(point, self)

    def transform(self, matrix: gp_Trsf) -> "GCylinder":
        center_pnt = to_native_vector(self.Center).Transformed(matrix)
        axis_pnt = gp_Pnt(self.Axis.x, self.Axis.y, self.Axis.z).Transformed(matrix)
        origin = gp_Pnt(0, 0, 0).Transformed(matrix)
        axis = GVector(axis_pnt.X() - origin.X(), axis_pnt.Y() - origin.Y(), axis_pnt.Z() - origin.Z())
        return GCylinder.from_values(_to_gvector(center_pnt), axis, self.Radius)


class GCone:
    def __init__(self, gp_cone, geom_surface=None):
        ax3 = gp_cone.Position()
        self.Apex = _to_gvector(gp_cone.Apex())
        self.Axis = _to_gvector(ax3.Direction())
        self.SemiAngle = gp_cone.SemiAngle()
        self.Radius = gp_cone.RefRadius()
        self.__native__ = geom_surface

    @classmethod
    def from_values(cls, apex: GVector, axis: GVector, semi_angle: float, radius: float | None = None) -> "GCone":
        cone = cls.__new__(cls)
        cone.Apex = apex
        cone.Axis = axis
        cone.SemiAngle = semi_angle
        cone.Radius = radius
        cone.__native__ = None
        return cone

    def parameter(self, point: GVector) -> tuple[float, float]:
        return _project_point_on_surface(point, self.__native__)

    def is_inside(self, point: GVector) -> bool:
        return is_inside_cone(point, self)

    def transform(self, matrix: gp_Trsf) -> "GCone":
        apex_pnt = to_native_vector(self.Apex).Transformed(matrix)
        axis_pnt = gp_Pnt(self.Axis.x, self.Axis.y, self.Axis.z).Transformed(matrix)
        origin = gp_Pnt(0, 0, 0).Transformed(matrix)
        axis = GVector(axis_pnt.X() - origin.X(), axis_pnt.Y() - origin.Y(), axis_pnt.Z() - origin.Z())
        return GCone.from_values(_to_gvector(apex_pnt), axis, self.SemiAngle, self.Radius)


class GSphere:
    def __init__(self, gp_sph, geom_surface=None):
        self.Center = _to_gvector(gp_sph.Location())
        self.Radius = gp_sph.Radius()
        self.__native__ = geom_surface

    @classmethod
    def from_values(cls, center: GVector, radius: float) -> "GSphere":
        sphere = cls.__new__(cls)
        sphere.Center = center
        sphere.Radius = radius
        sphere.__native__ = None
        return sphere

    def parameter(self, point: GVector) -> tuple[float, float]:
        return _project_point_on_surface(point, self.__native__)

    def is_inside(self, point: GVector) -> bool:
        return is_inside_sphere(point, self)

    def transform(self, matrix: gp_Trsf) -> "GSphere":
        center_pnt = to_native_vector(self.Center).Transformed(matrix)
        return GSphere.from_values(_to_gvector(center_pnt), self.Radius)


class GTorus:
    def __init__(self, gp_tor, geom_surface=None):
        ax3 = gp_tor.Position()
        self.Center = _to_gvector(gp_tor.Location())
        self.Axis = _to_gvector(ax3.Direction())
        self.MajorRadius = gp_tor.MajorRadius()
        self.MinorRadius = gp_tor.MinorRadius()
        self.__native__ = geom_surface

    @classmethod
    def from_values(cls, center: GVector, axis: GVector, major_radius: float, minor_radius: float) -> "GTorus":
        torus = cls.__new__(cls)
        torus.Center = center
        torus.Axis = axis
        torus.MajorRadius = major_radius
        torus.MinorRadius = minor_radius
        torus.__native__ = None
        return torus

    def parameter(self, point: GVector) -> tuple[float, float]:
        return _project_point_on_surface(point, self.__native__)

    def is_inside(self, point: GVector) -> bool:
        return is_inside_torus(point, self)


def Gclassify_surface(native_face):
    """See _freecad_impl.py's Gclassify_surface docstring -- same
    contract (None for a surface type GEOUNED can't model). The
    BSplineSurface-secretly-a-plane fallback (FreeCAD's face.findPlane())
    has no pyOCC port yet -- a mislabeled flat face returns None here."""
    adaptor = BRepAdaptor_Surface(native_face, True)
    kind = adaptor.GetType()
    if kind == GeomAbs_Plane:
        return GPlane(adaptor.Plane(), BRep_Tool.Surface(native_face))
    if kind == GeomAbs_Cylinder:
        return GCylinder(adaptor.Cylinder(), BRep_Tool.Surface(native_face))
    if kind == GeomAbs_Cone:
        return GCone(adaptor.Cone(), BRep_Tool.Surface(native_face))
    if kind == GeomAbs_Sphere:
        return GSphere(adaptor.Sphere(), BRep_Tool.Surface(native_face))
    if kind == GeomAbs_Torus:
        return GTorus(adaptor.Torus(), BRep_Tool.Surface(native_face))
    return None


# ---------------------------------------------------------------------------
# Analytic curve descriptors (wrap a native edge's classified curve)
# ---------------------------------------------------------------------------


class GLine:
    def __init__(self, gp_lin, geom_curve=None):
        self.Position = _to_gvector(gp_lin.Location())
        self.Direction = _to_gvector(gp_lin.Direction())
        self.__native__ = geom_curve

    @classmethod
    def from_values(cls, position: GVector, direction: GVector) -> "GLine":
        line = cls.__new__(cls)
        line.Position = position
        line.Direction = direction
        line.__native__ = None
        return line

    def intersect_line(self, other: "GLine") -> GVector | None:
        """See _freecad_impl.py's GLine.intersect_line docstring for the
        verified boundary this mirrors. Pure GVector math for the
        well-conditioned case; below that, falls back to native only
        when both lines have a real backing curve (from_values-built
        lines have none), otherwise returns None rather than risking a
        silently-wrong point."""
        d1, d2 = self.Direction, other.Direction
        cr = d1.cross(d2)
        crl = cr.length
        if crl < 1e-10:
            return None

        w = other.Position - self.Position
        scale_ref = max(self.Position.length, other.Position.length, 1.0)

        if crl < 0.05:
            if self.__native__ is None or other.__native__ is None:
                return None
            proj = GeomAPI_ProjectPointOnCurve(to_native_vector(self.Position), other.__native__)
            if proj.NbPoints() == 0:
                return None
            return _to_gvector(proj.NearestPoint())

        if abs(w.dot(cr)) / crl > 1e-6 * scale_ref:
            return None

        t = (w.cross(d2)).dot(cr) / (crl * crl)
        return self.Position + d1 * t

    def value(self, u: float) -> GVector:
        return self.Position + self.Direction * u

    def parameter(self, point: GVector) -> float:
        return (point - self.Position).dot(self.Direction)


class GCircle:
    def __init__(self, gp_circ, geom_curve=None):
        self.Center = _to_gvector(gp_circ.Location())
        self.Axis = _to_gvector(gp_circ.Position().Direction())
        self.Radius = gp_circ.Radius()
        self.__native__ = geom_curve

    def value(self, u: float) -> GVector:
        return _to_gvector(self.__native__.Value(u))

    def parameter(self, point: GVector) -> float:
        proj = GeomAPI_ProjectPointOnCurve(to_native_vector(point), self.__native__)
        return proj.LowerDistanceParameter()


class GEllipse:
    """Field names match FreeCAD's own `Part.Ellipse` attribute names."""

    def __init__(self, gp_elips, geom_curve=None):
        self.Center = _to_gvector(gp_elips.Location())
        self.Axis = _to_gvector(gp_elips.Position().Direction())
        self.XAxis = _to_gvector(gp_elips.XAxis().Direction())
        self.YAxis = _to_gvector(gp_elips.YAxis().Direction())
        self.MajorRadius = gp_elips.MajorRadius()
        self.MinorRadius = gp_elips.MinorRadius()
        self.__native__ = geom_curve

    def value(self, u: float) -> GVector:
        return _to_gvector(self.__native__.Value(u))

    def parameter(self, point: GVector) -> float:
        proj = GeomAPI_ProjectPointOnCurve(to_native_vector(point), self.__native__)
        return proj.LowerDistanceParameter()


class GBSpline:
    def __init__(self, geom_curve):
        self.__native__ = geom_curve
        self._poles = None

    @property
    def Poles(self) -> list[GVector]:
        """Lazy, cached. Real, measured cost on this backend specifically
        (2026-08-16, hylife-v06.stp): pyOCC's Geom_BSplineCurve has no
        bulk pole->Python-list conversion the way FreeCAD's getPoles()
        does (confirmed empirically -- even the nominally "bulk"
        Poles(array) overload doesn't help, since reading each gp_Pnt
        back out of the filled array still costs the same per-element
        Python<->C++ round trip as calling .Pole(i) directly). Eagerly
        building this for every BSpline-classified edge in GSolid's
        constructor -- most of which (only decom_utils_generator.py::
        spline_wires reads .Poles at all) never need it -- was measured
        contributing real, avoidable overhead to solid decomposition on
        BSpline-heavy geometry."""
        if self._poles is None:
            from OCC.Core.Geom import Geom_BSplineCurve

            geom_bspline = Geom_BSplineCurve.DownCast(self.__native__)
            self._poles = [_to_gvector(geom_bspline.Pole(i)) for i in range(1, geom_bspline.NbPoles() + 1)]
        return self._poles

    def value(self, u: float) -> GVector:
        return _to_gvector(self.__native__.Value(u))

    def parameter(self, point: GVector) -> float:
        proj = GeomAPI_ProjectPointOnCurve(to_native_vector(point), self.__native__)
        return proj.LowerDistanceParameter()


def Gclassify_curve(native_edge):
    """See _freecad_impl.py's Gclassify_curve docstring -- same
    contract (None for a degenerate edge or a real-but-unsupported
    curve type such as Hyperbola/Parabola)."""
    if isinstance(native_edge, GEdge):
        return native_edge.Curve
    curve_and_range = BRep_Tool.Curve(native_edge)
    if curve_and_range is None or curve_and_range[0] is None:
        return None
    geom_curve = curve_and_range[0]
    adaptor = BRepAdaptor_Curve(native_edge)
    kind = adaptor.GetType()
    if kind == GeomAbs_Line:
        return GLine(adaptor.Line(), geom_curve)
    if kind == GeomAbs_Circle:
        return GCircle(adaptor.Circle(), geom_curve)
    if kind == GeomAbs_Ellipse:
        return GEllipse(adaptor.Ellipse(), geom_curve)
    from OCC.Core.GeomAbs import GeomAbs_BSplineCurve

    if kind == GeomAbs_BSplineCurve:
        return GBSpline(geom_curve)
    return None


# ---------------------------------------------------------------------------
# Neutral topology types -- eagerly built from their native equivalent.
# ---------------------------------------------------------------------------


class GEdge:
    def __init__(self, native):
        self.__native__ = native
        self.Curve = Gclassify_curve(native)
        vertexes = []
        vexp = TopExp_Explorer(native, TopAbs_VERTEX)
        while vexp.More():
            vertexes.append(_to_gvector(BRep_Tool.Pnt(topods.Vertex(vexp.Current()))))
            vexp.Next()
        self.Vertexes = vertexes
        first, last = BRep_Tool.Range(native)
        self.ParameterRange = (first, last)
        self.Orientation = _orientation_str(native)
        edge_props = _linear_props(native)
        self.Length = edge_props.Mass()
        self.MatrixOfInertia = _to_gmatrix_3x3(edge_props.MatrixOfInertia())

    def value_at(self, u: float) -> GVector:
        adaptor = BRepAdaptor_Curve(self.__native__)
        return _to_gvector(adaptor.Value(u))

    def derivative1_at(self, u: float) -> GVector:
        adaptor = BRepAdaptor_Curve(self.__native__)
        pnt = gp_Pnt()
        deriv = gp_Vec()
        adaptor.D1(u, pnt, deriv)
        return _to_gvector(deriv)

    def normal_at(self, u: float) -> GVector:
        adaptor = BRepAdaptor_Curve(self.__native__)
        pnt = gp_Pnt()
        d1 = gp_Vec()
        d2 = gp_Vec()
        adaptor.D2(u, pnt, d1, d2)
        return _to_gvector(d2)

    def parameter(self, point: GVector) -> float:
        curve_and_range = BRep_Tool.Curve(self.__native__)
        proj = GeomAPI_ProjectPointOnCurve(to_native_vector(point), curve_and_range[0])
        return proj.LowerDistanceParameter()

    def curvature(self, u: float) -> float:
        """Curvature of the edge's curve at parametric coordinate `u`
        (0 for a straight line)."""
        curve_and_range = BRep_Tool.Curve(self.__native__)
        props = GeomLProp_CLProps(curve_and_range[0], u, 2, 1e-6)
        return props.Curvature()

    def knots(self) -> list[float]:
        """Unique knot values of the edge's curve (only meaningful for a
        BSpline curve -- see Gclassify_curve/GBSpline)."""
        from OCC.Core.Geom import Geom_BSplineCurve

        curve_and_range = BRep_Tool.Curve(self.__native__)
        bspline = Geom_BSplineCurve.DownCast(curve_and_range[0])
        return [bspline.Knot(i) for i in range(1, bspline.NbKnots() + 1)]

    def is_same(self, other: "GEdge") -> bool:
        return self.__native__.IsSame(other.__native__)

    def is_inside(self, point: GVector, tolerance: float) -> bool:
        curve_and_range = BRep_Tool.Curve(self.__native__)
        geom_curve, first, last = curve_and_range
        proj = GeomAPI_ProjectPointOnCurve(to_native_vector(point), geom_curve)
        if proj.NbPoints() == 0:
            return False
        if proj.LowerDistance() > tolerance:
            return False
        u = proj.LowerDistanceParameter()
        return (first - tolerance) <= u <= (last + tolerance)

    def distance_to(self, other: "GEdge") -> float:
        """Minimum distance between this edge and `other` (0 if they
        touch or overlap). Native curve-curve extrema query -- no
        GVector equivalent, and much cheaper than a face-level boolean
        since there's no surface/BOP algorithm involved."""
        return BRepExtrema_DistShapeShape(self.__native__, other.__native__).Value()

    def my_distToshape(self, other: "GEdge") -> float:
        """BoundBox-prefilter fast path, mirroring GFace.my_distToshape's
        shape -- but simpler: two 1D curves have no "interior"/volume to
        intersect, so there's no boolean-common step here, only "boxes
        overlap -> ask the real distance" or "boxes clearly separate ->
        the (necessarily coarser, but sufficient for a tolerance
        comparison) BoundBox-center distance," skipping the native call
        entirely for pairs that are obviously far apart."""
        shape1 = self.__native__
        shape2 = other.__native__
        box1 = _bnd_box(shape1)
        box2 = _bnd_box(shape2)
        intersect = (
            min(box1.XMax, box2.XMax) - max(box1.XMin, box2.XMin) > -1e-6
            and min(box1.YMax, box2.YMax) - max(box1.YMin, box2.YMin) > -1e-6
            and min(box1.ZMax, box2.ZMax) - max(box1.ZMin, box2.ZMin) > -1e-6
        )
        if intersect:
            return self.distance_to(other)
        c1 = GVector((box1.XMin + box1.XMax) / 2, (box1.YMin + box1.YMax) / 2, (box1.ZMin + box1.ZMax) / 2)
        c2 = GVector((box2.XMin + box2.XMax) / 2, (box2.YMin + box2.YMax) / 2, (box2.ZMin + box2.ZMax) / 2)
        return (c2 - c1).length

    def export_step(self, filename: str) -> None:
        _export_shapes_step([self.__native__], filename)


class GWire:
    def __init__(self, native):
        self.__native__ = native
        props = _linear_props(native)
        self.CenterOfMass = _to_gvector(props.CentreOfMass())
        from OCC.Core.BRepTools import BRepTools_WireExplorer

        edges = []
        vertices = []
        wexp = BRepTools_WireExplorer(native)
        while wexp.More():
            edges.append(GEdge(topods.Edge(wexp.Current())))
            vertices.append(_to_gvector(BRep_Tool.Pnt(wexp.CurrentVertex())))
            wexp.Next()
        self.Edges = edges
        # BRepTools_WireExplorer.CurrentVertex() is the vertex the current
        # edge starts FROM in wire-traversal order -- one per edge, matching
        # FreeCAD's native Wire.OrderedVertexes convention for a CLOSED wire
        # exactly (N vertices for N edges). For an OPEN wire, FreeCAD's
        # convention has N+1 (also including the last edge's own trailing
        # endpoint, never visited as a "current" vertex by the explorer) --
        # topexp.Vertices gives that endpoint respecting the same
        # traversal-consistent orientation CurrentVertex() already used.
        if edges and not BRep_Tool.IsClosed(native):
            v1, v2 = TopoDS_Vertex(), TopoDS_Vertex()
            topexp.Vertices(edges[-1].__native__, v1, v2)
            vertices.append(_to_gvector(BRep_Tool.Pnt(v2)))
        self.OrderedVertexes = vertices
        self.MatrixOfInertia = _to_gmatrix_3x3(props.MatrixOfInertia())


def pick_outer_wire(wires: list[GWire]) -> "GWire":
    """Same heuristic as _freecad_impl.py's pick_outer_wire (largest
    mean vertex-to-centroid distance) -- pure GVector/GWire math, no
    backend-specific code, ported unchanged."""
    if len(wires) == 1:
        return wires[0]
    best_wire = None
    best_extension = 0.0
    for wire in wires:
        vertices = wire.OrderedVertexes
        if not vertices:
            # a degenerate (edgeless) wire can never be the meaningful
            # outer boundary -- skip rather than divide by zero
            continue
        center = wire.CenterOfMass
        extension = sum((v - center).length for v in vertices) / len(vertices)
        if extension > best_extension:
            best_extension = extension
            best_wire = wire
    return best_wire


class GFace:
    def __init__(self, native):
        self.__native__ = native
        self.Surface = Gclassify_surface(native)
        edges = []
        eexp = TopExp_Explorer(native, TopAbs_EDGE)
        while eexp.More():
            edges.append(GEdge(topods.Edge(eexp.Current())))
            eexp.Next()
        self.Edges = edges
        vertexes = []
        vexp = TopExp_Explorer(native, TopAbs_VERTEX)
        while vexp.More():
            vertexes.append(_to_gvector(BRep_Tool.Pnt(topods.Vertex(vexp.Current()))))
            vexp.Next()
        self.Vertexes = vertexes
        self.BoundBox = _bnd_box(native)
        umin, umax, vmin, vmax = breptools.UVBounds(native)
        self.ParameterRange = (umin, umax, vmin, vmax)
        self.Orientation = _orientation_str(native)
        props = _surface_props(native)
        self.Area = props.Mass()
        self.CenterOfMass = _to_gvector(props.CentreOfMass())

        self.index: int | None = None
        self.__wires__: "list[GWire] | None" = None
        self.__outer_wire__: "GWire | None" = None

    def wires(self) -> "list[GWire]":
        if self.__wires__ is None:
            from OCC.Core.TopAbs import TopAbs_WIRE

            ws = []
            wexp = TopExp_Explorer(self.__native__, TopAbs_WIRE)
            while wexp.More():
                ws.append(GWire(topods.Wire(wexp.Current())))
                wexp.Next()
            self.__wires__ = ws
        return self.__wires__

    def outer_wire(self) -> "GWire":
        if self.__outer_wire__ is None:
            self.__outer_wire__ = pick_outer_wire(self.wires())
        return self.__outer_wire__

    def isEqual(self, face: "GFace") -> bool:
        return self.__native__.IsEqual(face.__native__)

    def isSame(self, face: "GFace") -> bool:
        return self.__native__.IsSame(face.__native__)

    def value_at(self, u: float, v: float) -> GVector:
        surf = BRep_Tool.Surface(self.__native__)
        props = GeomLProp_SLProps(surf, u, v, 1, 1e-6)
        return _to_gvector(props.Value())

    def normal_at(self, u: float, v: float) -> GVector:
        surf = BRep_Tool.Surface(self.__native__)
        props = GeomLProp_SLProps(surf, u, v, 1, 1e-6)
        n = _to_gvector(props.Normal())
        return -n if self.__native__.Orientation() == 1 else n

    def tangent_at(self, u: float, v: float) -> tuple[GVector, GVector]:
        surf = BRep_Tool.Surface(self.__native__)
        props = GeomLProp_SLProps(surf, u, v, 1, 1e-6)
        return _to_gvector(props.D1U()), _to_gvector(props.D1V())

    def parameter(self, point: GVector) -> tuple[float, float]:
        surf = BRep_Tool.Surface(self.__native__)
        return _project_point_on_surface(point, surf)

    def is_part_of_domain(self, u: float, v: float) -> bool:
        classifier = BRepTopAdaptor_FClass2d(self.__native__, 1e-7)
        return classifier.Perform(gp_Pnt2d(u, v)) != 1  # != TopAbs_OUT

    def tessellate(self, tolerance: float, reset: bool = False) -> list[GVector]:
        if reset:
            breptools.Clean(self.__native__)
        BRepMesh_IncrementalMesh(self.__native__, tolerance)
        loc = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(self.__native__, loc)
        if triangulation is None:
            return []
        trsf = loc.Transformation()
        points = []
        for i in range(1, triangulation.NbNodes() + 1):
            pnt = triangulation.Node(i).Transformed(trsf)
            points.append(_to_gvector(pnt))
        return points

    def getUVNodes(self):
        BRepMesh_IncrementalMesh(self.__native__, 0.1)
        loc = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(self.__native__, loc)
        if triangulation is None or not triangulation.HasUVNodes():
            return []
        return [(triangulation.UVNode(i).X(), triangulation.UVNode(i).Y()) for i in range(1, triangulation.NbNodes() + 1)]

    def orientation_outward(self, solid: "GSolid") -> bool:
        umin, umax, vmin, vmax = self.ParameterRange
        u = (umin + umax) / 2.0
        v = (vmin + vmax) / 2.0
        point = self.value_at(u, v)
        normal = self.normal_at(u, v)
        probe = point + normal * 1e-6
        classifier = BRepClass3d_SolidClassifier(solid.__native__)
        classifier.Perform(to_native_vector(probe), 1e-7)
        return classifier.State() != TopAbs_IN

    def export_step(self, filename: str) -> None:
        _export_shapes_step([self.__native__], filename)

    def distance_to(self, other: "GFace") -> float:
        return BRepExtrema_DistShapeShape(self.__native__, other.__native__).Value()

    def my_distToshape(self, other: "GFace") -> float:
        """Mirrors _freecad_impl.py's GFace.my_distToshape (BoundBox
        overlap + boolean-common/edge-identity fast path, falling back
        to the reliable native distance_to() in the remaining ambiguous
        case)."""
        shape1 = self.__native__
        shape2 = other.__native__

        if shape1.IsSame(shape2):
            return 0.0

        box1 = _bnd_box(shape1)
        box2 = _bnd_box(shape2)
        intersect = (
            min(box1.XMax, box2.XMax) - max(box1.XMin, box2.XMin) > -1e-6
            and min(box1.YMax, box2.YMax) - max(box1.YMin, box2.YMin) > -1e-6
            and min(box1.ZMax, box2.ZMax) - max(box1.ZMin, box2.ZMin) > -1e-6
        )
        if intersect:
            try:
                common = BRepAlgoAPI_Common(shape1, shape2).Shape()
                has_content = not common.IsNull()
                if has_content:
                    props = _volume_props(common)
                    has_content = abs(props.Mass()) > 1e-8
                    if not has_content:
                        exp = TopExp_Explorer(common, TopAbs_FACE)
                        has_content = exp.More()
                        if not has_content:
                            exp = TopExp_Explorer(common, TopAbs_EDGE)
                            has_content = exp.More()
            except Exception:
                has_content = False

            if has_content:
                return 0.0

            same = False
            e1exp = TopExp_Explorer(shape1, TopAbs_EDGE)
            while e1exp.More() and not same:
                e1 = topods.Edge(e1exp.Current())
                e2exp = TopExp_Explorer(shape2, TopAbs_EDGE)
                while e2exp.More():
                    e2 = topods.Edge(e2exp.Current())
                    if e1.IsSame(e2):
                        same = True
                        break
                    e2exp.Next()
                e1exp.Next()
            if same:
                return 0.0
            return self.distance_to(other)

        c1 = GVector((box1.XMin + box1.XMax) / 2, (box1.YMin + box1.YMax) / 2, (box1.ZMin + box1.ZMax) / 2)
        c2 = GVector((box2.XMin + box2.XMax) / 2, (box2.YMin + box2.YMax) / 2, (box2.ZMin + box2.ZMax) / 2)
        return (c2 - c1).length


class GShell:
    def __init__(self, native, faces: list[GFace] | None = None):
        self.__native__ = native
        if faces is not None:
            self.Faces = faces
        else:
            fs = []
            fexp = TopExp_Explorer(native, TopAbs_FACE)
            while fexp.More():
                fs.append(GFace(topods.Face(fexp.Current())))
                fexp.Next()
            self.Faces = fs
        self.Orientation = _orientation_str(native)

    def export_step(self, filename: str) -> None:
        _export_shapes_step([self.__native__], filename)


class GSolid:
    def __init__(self, native):
        self.Solids = []
        faces = []
        fexp = TopExp_Explorer(native, TopAbs_FACE)
        while fexp.More():
            faces.append(GFace(topods.Face(fexp.Current())))
            fexp.Next()
        self.Faces = faces
        for index, face in enumerate(self.Faces):
            face.index = index
        edges = []
        eexp = TopExp_Explorer(native, TopAbs_EDGE)
        while eexp.More():
            edges.append(GEdge(topods.Edge(eexp.Current())))
            eexp.Next()
        self.Edges = edges
        vertexes = []
        vexp = TopExp_Explorer(native, TopAbs_VERTEX)
        while vexp.More():
            vertexes.append(_to_gvector(BRep_Tool.Pnt(topods.Vertex(vexp.Current()))))
            vexp.Next()
        self.Vertexes = vertexes
        self.BoundBox = _bnd_box(native)
        self.Orientation = _orientation_str(native)
        self.Area = _surface_props(native).Mass()
        self.Volume = _volume_props(native).Mass()

        native_solids = []
        sexp = TopExp_Explorer(native, TopAbs_SOLID)
        while sexp.More():
            native_solids.append(topods.Solid(sexp.Current()))
            sexp.Next()
        if len(native_solids) > 1:
            for s in native_solids:
                self.Solids.append(GSolid(s))
        else:
            self.Solids.append(self)

        self.__native__ = native
        self.__shapes__ = native_solids if native_solids else [native]

    def is_inside(self, point: GVector, tolerance: float = 0.0) -> bool:
        classifier = BRepClass3d_SolidClassifier(self.__native__)
        classifier.Perform(to_native_vector(point), tolerance if tolerance > 0 else 1e-7)
        return classifier.State() == TopAbs_IN

    def optimal_bounding_box(self, use_triangulation: bool = True) -> GBoundBox:
        box = Bnd_Box()
        if use_triangulation:
            brepbndlib.AddOptimal(self.__native__, box)
        else:
            brepbndlib.Add(self.__native__, box)
        xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
        return GBoundBox(xmin, ymin, zmin, xmax, ymax, zmax)

    def center_of_mass(self) -> GVector:
        return _to_gvector(_volume_props(self.__native__).CentreOfMass())

    def find_interior_point(self) -> GVector | None:
        """Same algorithm as _freecad_impl.py's GSolid.find_interior_point
        (center of mass first, then probe inward from each face along
        its inward normal) -- see that docstring for why this strategy
        was chosen over sampling/octree alternatives."""
        native = self.__native__
        first_shape = self.__shapes__[0]
        point = _volume_props(first_shape).CentreOfMass()
        classifier = BRepClass3d_SolidClassifier(native)
        classifier.Perform(point, 1e-7)
        if classifier.State() == TopAbs_IN:
            return _to_gvector(point)

        volume = abs(_volume_props(native).Mass())
        length = 0.5 * volume**0.33333
        fexp = TopExp_Explorer(native, TopAbs_FACE)
        while fexp.More():
            face = topods.Face(fexp.Current())
            umin, umax, vmin, vmax = breptools.UVBounds(face)
            u = 0.5 * (umin + umax)
            v = 0.5 * (vmin + vmax)
            classifier2d = BRepTopAdaptor_FClass2d(face, 1e-7)
            if classifier2d.Perform(gp_Pnt2d(u, v)) != 1:  # inside the trimmed domain
                surf = BRep_Tool.Surface(face)
                props = GeomLProp_SLProps(surf, u, v, 1, 1e-6)
                pos = props.Value()
                normal = props.Normal()
                if face.Orientation() == 1:
                    normal = normal.Reversed()
                normal = normal.Reversed()  # inward
                d = length
                for _ in range(12):
                    d = d * 0.5
                    probe = gp_Pnt(pos.X() + d * normal.X(), pos.Y() + d * normal.Y(), pos.Z() + d * normal.Z())
                    classifier.Perform(probe, 1e-7)
                    if classifier.State() == TopAbs_IN:
                        return _to_gvector(probe)
            fexp.Next()
        return None

    def faces_sharing_edge(self, edge: GEdge) -> list[GFace]:
        matches = []
        fexp = TopExp_Explorer(self.__native__, TopAbs_FACE)
        while fexp.More():
            face = topods.Face(fexp.Current())
            eexp = TopExp_Explorer(face, TopAbs_EDGE)
            found = False
            while eexp.More() and not found:
                if topods.Edge(eexp.Current()).IsSame(edge.__native__):
                    matches.append(face)
                    found = True
                eexp.Next()
            fexp.Next()
        return [GFace(f) for f in matches]

    def is_valid(self) -> bool:
        return BRepCheck_Analyzer(self.__native__).IsValid()

    def fix(self, tolerance: float) -> "GSolid":
        # UnifyEdges is only attempted when the input is already known-valid
        # -- on an already-invalid solid it's a confirmed native crash/hang
        # risk (see remove_solids._refine_if_valid's docstring for the real
        # reproduction), not just a wasted simplification pass. ShapeFix_Shape
        # is the actual repair step either way.
        #
        # UnifyEdges' own *output* can itself be invalid even when its input
        # was valid (confirmed live, 2026-08-19, rev_pipe.stp's raw loaded
        # solid: BRepCheck_Analyzer says valid=True going in, valid=False
        # coming out of UnifyEdges alone) -- so the fallback ShapeFix_Shape
        # repair below must run on the original, untouched `native`, never on
        # UnifyEdges' own (possibly corrupted) result. An earlier version of
        # this method reassigned `native` to UnifyEdges' output before this
        # check, so the fallback repair silently worked on already-broken
        # input and could never recover -- confirmed empirically: ShapeFix_
        # Shape on the corrupted UnifyEdges output stayed invalid, while the
        # identical call on the original raw solid came back valid.
        native = self.__native__
        if BRepCheck_Analyzer(native).IsValid():
            # unify_edges=True is a confirmed native-crash source (access
            # violation, not a catchable Python exception) under this
            # engine specifically: reproduced live, 2026-08-23, on
            # Solidos/test_models/Mixed/ConeSphere.stp's own valid,
            # loaded solid (this crashes right here, at load time, since
            # Gload_step calls .fix() unconditionally on every solid).
            # unify_edges=False/unify_faces=True completes cleanly on the
            # identical input. NOTE this is the OPPOSITE flag from the
            # _ocp_impl.py fix for the same symptom on the same file --
            # confirmed empirically, not assumed: under OCP (pybind11)
            # it's unify_faces=True that crashes and unify_edges=True
            # alone is safe; under this engine (pythonocc-core/SWIG) it's
            # the reverse. The two bindings' own native crash behavior on
            # identical OCCT 7.9.3 geometry is not symmetric -- do not
            # assume a fix ported from one engine applies unchanged to
            # the other; always re-verify per engine. Kept off here for
            # the same reason unify_edges was previously conditioned on
            # validity: no Python-level guard can recover from this once
            # triggered.
            unify = ShapeUpgrade_UnifySameDomain(native, True, False, True)
            unify.Build()
            unified = unify.Shape()
            if BRepCheck_Analyzer(unified).IsValid():
                return GSolid(unified)
        fixer = ShapeFix_Shape(BRepBuilderAPI_Copy(native).Shape())
        fixer.SetPrecision(tolerance)
        fixer.Perform()
        return GSolid(fixer.Shape())

    def reverse(self) -> "GSolid":
        return GSolid(BRepBuilderAPI_Copy(self.__native__).Shape().Reversed())

    def refine(self) -> "GSolid":
        """See _freecad_impl.py's GSolid.refine docstring -- same
        volume-invariance guard, using ShapeUpgrade_UnifySameDomain as
        the removeSplitter() equivalent.

        unify_edges=True is a confirmed native-crash source under this
        engine (access violation, not a catchable Python exception -- no
        try/except below can recover from it, unlike the RuntimeError
        case further down): reproduced live, 2026-08-23, on
        Solidos/test_models/Mixed/ConeSphere.stp's own valid, loaded
        solid, deterministically -- matching this file's own long-
        documented "KNOWN GAP" below, previously described as a hang
        rather than a crash (not re-checked which is accurate; the
        practical effect -- the process never returns control -- is the
        same either way). unify_edges=False/unify_faces=True completes
        cleanly on the identical input (same volume, still valid), and
        was re-verified this session to NOT reproduce the previously-
        documented cylBox.stp/DoubleCylinder-pieza.stp Can-secondary-
        surface regression that an earlier attempt at this same change
        hit (see fix()'s own docstring above for the direct comparison
        against _ocp_impl.py, where the crash-triggering flag is the
        opposite one). Kept off here unconditionally, matching fix()'s
        own identical change.

        A second, distinct failure mode of the same underlying fragility
        -- unify.Build() outright raising a native OCCT StdFail_NotDone
        failure rather than hanging or silently mis-refining -- was hit
        on real geometry in hylife-v06.stp (a >300-solid real model,
        under the ocp engine specifically, though the code path is
        identical here). Caught and treated the same way as the
        volume-mismatch case: fall back to the untouched original shape.
        pythonocc-core (unlike OCP) doesn't expose StdFail_NotDone as a
        distinct importable Python class -- confirmed empirically that
        every native OCCT Standard_Failure surfaces here as a plain
        RuntimeError, with the real exception name only embedded in the
        message text -- so this catches RuntimeError broadly; the only
        OCCT call in this block is unify.Build() itself, so there's
        nothing else in scope this could mask."""
        native = self.__native__
        original_volume = _volume_props(native).Mass()
        copy = BRepBuilderAPI_Copy(native).Shape()
        unify = ShapeUpgrade_UnifySameDomain(copy, True, False, True)
        try:
            unify.Build()
            refined = unify.Shape()
            refined_volume = _volume_props(refined).Mass()
        except RuntimeError:
            return GSolid(native)
        if abs(refined_volume - original_volume) > 1e-6 * max(abs(original_volume), 1.0):
            return GSolid(native)
        return GSolid(refined)

    def translate(self, vector: GVector) -> "GSolid":
        trsf = gp_Trsf()
        trsf.SetTranslation(gp_Vec(vector.x, vector.y, vector.z))
        return GSolid(BRepBuilderAPI_Transform(self.__native__, trsf, True).Shape())

    def rotate(self, axis_point: GVector, axis_dir: GVector, angle_rad: float) -> "GSolid":
        trsf = gp_Trsf()
        axis = gp_Ax1(to_native_vector(axis_point), gp_Dir(axis_dir.x, axis_dir.y, axis_dir.z))
        trsf.SetRotation(axis, angle_rad)
        return GSolid(BRepBuilderAPI_Transform(self.__native__, trsf, True).Shape())

    def export_step(self, filename: str) -> None:
        _export_shapes_step([self.__native__], filename)

    def copy(self) -> "GSolid":
        return GSolid(BRepBuilderAPI_Copy(self.__native__).Shape())

    def transform_geometry(self, matrix) -> "GSolid":
        """Apply a rigid affine transform (a native gp_Trsf). See
        _freecad_impl.py's own docstring -- not currently used anywhere
        (GEOReverse, this method's only real consumer, is FreeCAD-only),
        kept for backend symmetry."""
        return GSolid(BRepBuilderAPI_Transform(self.__native__, matrix, True).Shape())


GShape = GSolid | GFace | GEdge | GShell


@dataclass(frozen=True)
class SplitResult:
    solids: list[GSolid]
    degenerate_case_handled: bool = False
    notes: str = ""


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------


def _export_shapes_step(native_shapes: list, filename: str) -> None:
    writer = STEPControl_Writer()
    for shape in native_shapes:
        writer.Transfer(shape, STEPControl_AsIs)
    status = writer.Write(filename)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP export failed for {filename} (status={status})")


def Gfirst_shell(native_shape):
    """The first shell of a native solid shape. See _freecad_impl.py's
    Gfirst_shell docstring for why this exists and where it's called
    from."""
    from OCC.Core.TopAbs import TopAbs_SHELL

    explorer = TopExp_Explorer(native_shape, TopAbs_SHELL)
    return topods.Shell(explorer.Current())


def Gload_step(filename: str) -> list[GSolid]:
    """Loads a STEP file's solids and heals each one (GSolid.fix(1e-6),
    ShapeFix_Shape) before returning it.

    FreeCAD's own STEP importer (Part.Shape.read, _freecad_impl.py's
    Gload_step) does this kind of cleanup implicitly as part of its
    translation pipeline; pyOCC's raw STEPControl_Reader does not -- a
    solid loaded this way can carry small tolerance/topology issues
    invisible to BRepCheck_Analyzer.IsValid() but severe enough to make
    later native BOP calls (BOPAlgo_Splitter, BRepAlgoAPI_Common, even
    the "reliable" BRepExtrema_DistShapeShape fallback) pathologically
    slow or hang outright on otherwise simple, valid-looking geometry.
    Confirmed live (2026-08-16, hylife-v06.stp solid 17 -- a simple
    solid FreeCAD converts without any issue): unhealed, decompose_solids()
    never returned (traced via py-spy to 3 different native hangs across
    repeated attempts); healed via .fix(1e-6) right after load (volume
    shift ~0.003%, real cleanup not corruption), the exact same solid
    decomposes cleanly in ~23s. GeounedSolid.__init__ already calls
    .refine() (ShapeUpgrade_UnifySameDomain) on every loaded solid, but
    that alone was not sufficient here -- refine() has no ShapeFix_Shape
    step of its own; only .fix() does, which is why the healing has to
    happen here, not left to rely on that later call."""
    reader = STEPControl_Reader()
    status = reader.ReadFile(filename)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed for {filename} (status={status})")
    reader.TransferRoots()
    shape = reader.OneShape()
    solids = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        solids.append(GSolid(topods.Solid(explorer.Current())).fix(1e-6))
        explorer.Next()
    return solids


def Gload_step_labels(filename: str) -> list[GLabelNode]:
    """
    Parse the STEP file's assembly tree via XCAF and return one
    `GLabelNode` per solid-bearing leaf, in the same order as
    `Gload_step`'s solids -- see `GLabelNode`'s own docstring for the
    positional-alignment contract this must satisfy. Only XCAF "simple
    shape" leaf labels are ever appended to the returned list (an
    assembly label's own conceptual shape is just a grouping container,
    matching FreeCAD's `Import.insert()`-based version, which only ever
    sees `Part::Feature` leaf objects, never the group/assembly
    containers STEP's importer builds around them) -- but assembly
    labels are still walked and given their own `GLabelNode`, used as
    the `parent` of their children, exactly like the FreeCAD version's
    non-solid-bearing ancestor nodes.
    """
    from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
    from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.XCAFApp import XCAFApp_Application
    from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

    app = XCAFApp_Application.GetApplication()
    doc = TDocStd_Document("XmlXCAF")
    app.NewDocument("XmlXCAF", doc)

    reader = STEPCAFControl_Reader()
    reader.SetColorMode(False)
    reader.SetNameMode(True)
    reader.SetLayerMode(False)
    reader.SetMatMode(False)
    status = reader.ReadFile(filename)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed for {filename} (status={status})")
    reader.Transfer(doc)

    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    nodes: list[GLabelNode] = []

    def count_solids(shape) -> int:
        n = 0
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        while exp.More():
            n += 1
            exp.Next()
        return n

    def walk(label, parent_node):
        name = label.GetLabelName()
        if shape_tool.IsReference(label):
            referred = TDF_Label()
            shape_tool.GetReferredShape(label, referred)
            walk(referred, parent_node)
            return
        if shape_tool.IsAssembly(label):
            node = GLabelNode(label=name, parent=parent_node, n_solids=0)
            comps = TDF_LabelSequence()
            shape_tool.GetComponents(label, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), node)
        elif shape_tool.IsSimpleShape(label):
            n_solids = count_solids(shape_tool.GetShape(label))
            if n_solids == 0:
                return
            if n_solids == 1:
                nodes.append(GLabelNode(label=name, parent=parent_node, n_solids=1))
            else:
                # A single XCAF "simple shape" label whose own geometry is a
                # compound of several solids -- unlike FreeCAD's Import.insert(),
                # which splits this into several separate Part::Feature leaf
                # objects (with auto-suffixed names), XCAF keeps it as one
                # label. Split into one node per solid here too, to preserve
                # the positional-alignment contract with Gload_step's own
                # per-solid TopExp_Explorer walk -- sharing the same label/
                # parent across all of them rather than guessing at FreeCAD's
                # internal auto-suffix naming convention.
                for _ in range(n_solids):
                    nodes.append(GLabelNode(label=name, parent=parent_node, n_solids=1))

    free_labels = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_labels)
    for i in range(1, free_labels.Length() + 1):
        walk(free_labels.Value(i), None)

    return nodes


def Gexport_step(shapes: list[GShape], filename: str) -> None:
    _export_shapes_step([shape.__native__ for shape in shapes], filename)


# ---------------------------------------------------------------------------
# Primitive construction
# ---------------------------------------------------------------------------


def Gmake_box(xmin: float, ymin: float, zmin: float, xmax: float, ymax: float, zmax: float) -> GSolid:
    box = BRepPrimAPI_MakeBox(gp_Pnt(xmin, ymin, zmin), gp_Pnt(xmax, ymax, zmax)).Shape()
    return GSolid(box)


def Gmake_cylinder(point: GVector, axis: GVector, radius: float, height: float) -> GSolid:
    ax2 = gp_Ax2(to_native_vector(point), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeCylinder(ax2, radius, height).Shape()
    return GSolid(native)


def Gmake_cone(apex: GVector, axis: GVector, half_angle: float, height: float) -> GSolid:
    base_radius = height * math.tan(abs(half_angle))
    ax2 = gp_Ax2(to_native_vector(apex), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeCone(ax2, 0.0, base_radius, height).Shape()
    return GSolid(native)


def Gmake_cone_frustum(point: GVector, axis: GVector, radius1: float, radius2: float, height: float) -> GSolid:
    """Truncated cone (frustum): radius1 at `point`, radius2 at `point + height*axis`. See _freecad_impl.py's own docstring for why this is a separate function from Gmake_cone."""
    ax2 = gp_Ax2(to_native_vector(point), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeCone(ax2, radius1, radius2, height).Shape()
    return GSolid(native)


def Gmake_cone_double_sheet(apex: GVector, axis: GVector, half_angle: float, length: float) -> GSolid:
    """Both nappes of an infinite cone, fused into one solid. See _freecad_impl.py's own docstring."""
    sheet1 = Gmake_cone(apex, axis, half_angle, length)
    sheet2 = Gmake_cone(apex, -axis, half_angle, length)
    fused = Gfuse([sheet1, sheet2])
    return fused.refine()


def Gmake_sphere(center: GVector, radius: float) -> GSolid:
    native = BRepPrimAPI_MakeSphere(to_native_vector(center), radius).Shape()
    return GSolid(native)


def Gmake_torus(center: GVector, axis: GVector, major_radius: float, minor_radius: float) -> GSolid:
    ax2 = gp_Ax2(to_native_vector(center), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeTorus(ax2, major_radius, minor_radius).Shape()
    return GSolid(native)


def Gmake_half_space(plane: GPlane) -> GSolid:
    """Half-space bounded by an infinite plane (internally clipped to a
    huge working box, matching _freecad_impl.py's own box-based approach
    rather than pyOCC's face+reference-point BRepPrimAPI_MakeHalfSpace,
    to keep this function's signature -- a GPlane, no reference point --
    identical across backends)."""
    extent = 1.0e6
    box = BRepPrimAPI_MakeBox(gp_Pnt(-extent / 2.0, -extent / 2.0, 0.0), gp_Pnt(extent / 2.0, extent / 2.0, extent)).Shape()
    normal = plane.Axis.normalized()
    target = gp_Dir(normal.x, normal.y, normal.z)
    ax3_from = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    ax3_to = gp_Ax3(to_native_vector(plane.Position), target)
    trsf = gp_Trsf()
    trsf.SetTransformation(ax3_to, ax3_from)
    moved = BRepBuilderAPI_Transform(box, trsf, True).Shape()
    return GSolid(moved)


def Gmake_wire(edges: list[GEdge]) -> GWire:
    maker = BRepBuilderAPI_MakeWire()
    for edge in edges:
        maker.Add(edge.__native__)
    return GWire(maker.Wire())


def Gmake_polygon_face(points: list[GVector]) -> GFace:
    poly = BRepBuilderAPI_MakePolygon()
    for p in points:
        poly.Add(to_native_vector(p))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return GFace(face)


def Gmake_shell(faces: list[GFace]) -> GShell:
    sewer = BRepBuilderAPI_Sewing(1e-6)
    for face in faces:
        sewer.Add(face.__native__)
    sewer.Perform()
    sewed = sewer.SewedShape()
    builder = BRep_Builder()
    shell = TopoDS_Shell()
    builder.MakeShell(shell)
    fexp = TopExp_Explorer(sewed, TopAbs_FACE)
    if fexp.More():
        while fexp.More():
            builder.Add(shell, topods.Face(fexp.Current()))
            fexp.Next()
    else:
        for face in faces:
            builder.Add(shell, face.__native__)
    return GShell(shell, faces=faces)


def Gmake_solid(shell: GShell) -> "GSolid | None":
    """Close a watertight shell into a real solid (a genuine enclosed
    volume), for callers that built a shell face-by-face (e.g. clipping a
    box by successive cutting planes) and need a proper GSolid out of it
    -- as opposed to Gmake_shell's own callers, which only need the faces
    addressable as a group, not a valid volume. Returns None if the shell
    isn't actually closed/well-formed enough to bound a solid."""
    try:
        solid_maker = BRepBuilderAPI_MakeSolid(shell.__native__)
        if solid_maker.IsDone():
            return GSolid(solid_maker.Solid())
    except Exception:
        pass
    return None


def Gmake_compound(shapes: list[GSolid]) -> GSolid:
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for s in shapes:
        builder.Add(compound, s.__native__)
    return GSolid(compound)


# ---------------------------------------------------------------------------
# Boolean / split operations
# ---------------------------------------------------------------------------


def _exploded_solids(native_result):
    solids = []
    explorer = TopExp_Explorer(native_result, TopAbs_SOLID)
    while explorer.More():
        solids.append(topods.Solid(explorer.Current()))
        explorer.Next()
    return solids


def Gcut(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    result = solid.__native__
    for tool in tools:
        result = BRepAlgoAPI_Cut(result, tool.__native__).Shape()
    return [GSolid(s) for s in _exploded_solids(result)]


def Gcommon(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    result = solid.__native__
    for tool in tools:
        result = BRepAlgoAPI_Common(result, tool.__native__).Shape()
    return [GSolid(s) for s in _exploded_solids(result)]


def Gfuse(solids: list[GSolid]) -> GSolid:
    shapes = []
    for gsolid in solids:
        sub_solids = _exploded_solids(gsolid.__native__)
        if len(sub_solids) > 1:
            shapes.extend(sub_solids)
        else:
            shapes.append(gsolid.__native__)
    fused = shapes[0]
    for s in shapes[1:]:
        fused = BRepAlgoAPI_Fuse(fused, s).Shape()
    return GSolid(fused)


def _edge_face_map(native_solid) -> TopTools_IndexedDataMapOfShapeListOfShape:
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(native_solid, TopAbs_EDGE, TopAbs_FACE, m)
    return m


def _repair_non_manifold_solid(native_solid) -> list:
    """Attempt to split a non-manifold TopoDS_Solid (confirmed invalid
    via BRepCheck_Analyzer) into its real connected components: build a
    face-adjacency graph over the solid's own faces, excluding edges
    shared by != 2 faces, find connected components via union-find, and
    for each component missing a proper boundary at a non-manifold edge,
    duplicate the real face found there (via BRepBuilderAPI_Copy) so
    both sides get their own capping copy. Returns a list of native
    TopoDS_Solid -- may be a single-element list containing the
    original, unrepaired solid if reconstruction doesn't succeed.
    """
    faces = []
    explorer = TopExp_Explorer(native_solid, TopAbs_FACE)
    while explorer.More():
        faces.append(topods.Face(explorer.Current()))
        explorer.Next()
    n = len(faces)

    edge_map = _edge_face_map(native_solid)
    non_manifold_edge_keys = {i for i in range(1, edge_map.Size() + 1) if edge_map.FindFromIndex(i).Size() != 2}

    def face_index(face):
        for i, f in enumerate(faces):
            if f.IsSame(face):
                return i
        return None

    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(1, edge_map.Size() + 1):
        if i in non_manifold_edge_keys:
            continue
        face_list = edge_map.FindFromIndex(i)
        if face_list.Size() == 2:
            it = iter(face_list)
            f1 = next(it)
            f2 = next(it)
            i1, i2 = face_index(f1), face_index(f2)
            if i1 is not None and i2 is not None:
                union(i1, i2)

    components: dict[int, list] = {}
    for i in range(n):
        components.setdefault(find(i), []).append(i)

    if len(components) < 2:
        return [native_solid]

    comp_extra_faces: dict[int, list] = {root: [] for root in components}
    for i in non_manifold_edge_keys:
        face_list = edge_map.FindFromIndex(i)
        by_component: dict[int, list] = {}
        for f in face_list:
            idx = face_index(f)
            if idx is None:
                continue
            by_component.setdefault(find(idx), []).append(f)
        for root in components:
            if root not in by_component:
                donor_root, donor_faces = next(iter(by_component.items()))
                comp_extra_faces.setdefault(root, []).append(donor_faces[0])

    results = []
    for root, idxs in components.items():
        comp_faces = [faces[i] for i in idxs]
        for donor_face in comp_extra_faces.get(root, []):
            comp_faces.append(BRepBuilderAPI_Copy(donor_face).Shape())

        sewer = BRepBuilderAPI_Sewing(1e-6)
        for f in comp_faces:
            sewer.Add(f)
        sewer.Perform()
        sewed = sewer.SewedShape()

        builder = BRep_Builder()
        shell_explorer = TopExp_Explorer(sewed, TopAbs_FACE)
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        seen_any = False
        while shell_explorer.More():
            builder.Add(shell, topods.Face(shell_explorer.Current()))
            seen_any = True
            shell_explorer.Next()
        if not seen_any:
            continue

        try:
            solid_maker = BRepBuilderAPI_MakeSolid(shell)
            if solid_maker.IsDone():
                results.append(solid_maker.Solid())
        except Exception:
            pass

    if not results:
        return [native_solid]
    return results


def _raw_bop_split(base_native, tool_native, tolerance: float) -> tuple[list, bool, bool]:
    """The actual BOPAlgo_Splitter call plus non-manifold repair, factored
    out of Gsplit so `_try_coaxial_cone_split`'s own internal retry (on a
    presplit copy of `base`) can reuse it directly without recursing back
    through Gsplit's own coaxial-cone fallback. Returns (native_solids,
    repaired_any, tool_missed_entirely) -- the third value distinguishes
    "BOP found literally nothing" (tool genuinely doesn't touch base) from
    "BOP found exactly one, unchanged solid" (the silent no-op symptom the
    coaxial-cone fallback targets); the two need different handling."""
    splitter = BOPAlgo_Splitter()
    splitter.AddArgument(base_native)
    splitter.AddTool(tool_native)
    if tolerance:
        splitter.SetFuzzyValue(tolerance)
    splitter.Perform()
    raw_solids = _exploded_solids(splitter.Shape())

    if not raw_solids:
        return [base_native], False, True

    repaired_any = False
    final_native_solids = []
    for s in raw_solids:
        if BRepCheck_Analyzer(s).IsValid():
            final_native_solids.append(s)
            continue
        repaired = _repair_non_manifold_solid(s)
        changed = len(repaired) > 1 or (len(repaired) == 1 and not repaired[0].IsEqual(s))
        if changed:
            # Never trust the face-adjacency-graph reconstruction blindly:
            # every piece must be a genuinely valid solid AND their summed
            # volume must match the invalid input's own volume (same
            # discipline as _try_coaxial_cone_split's own safety net) --
            # otherwise the reconstruction can silently invent or lose
            # material. Confirmed on a real fixture (modelcell_cut1
            # piece70): a single-plane cut's invalid fragment got
            # "repaired" into 3 pieces summing to ~30% more volume than
            # the original, 2 of them themselves still invalid.
            all_valid = all(BRepCheck_Analyzer(r).IsValid() for r in repaired)
            if all_valid:
                original_volume = abs(_volume_props(s).Mass())
                repaired_volume = sum(abs(_volume_props(r).Mass()) for r in repaired)
                volume_ok = abs(repaired_volume - original_volume) <= 1e-6 * max(original_volume, 1.0)
            else:
                volume_ok = False
            if not (all_valid and volume_ok):
                repaired = [s]
                changed = False
        if changed:
            repaired_any = True
        final_native_solids.extend(repaired)
    return final_native_solids, repaired_any, False


def _find_cone_face(shape) -> "GFace | None":
    """First face of `shape` (anything with a `.Faces` list of GFace, e.g.
    a GSolid) whose analytic surface is a GCone, or None if it has none."""
    for f in shape.Faces:
        if isinstance(f.Surface, GCone):
            return f
    return None


def _group_coaxial_cone_faces(base_faces: "list[GFace]", tool_cone: "GCone") -> "list[list[GFace]]":
    """Groups of `base_faces` whose own cone surface is coaxial with, and
    shares the same |SemiAngle| as, `tool_cone` (see
    vector_geometry.is_coaxial_cone_pair) -- each group sharing one exact
    (Apex, Axis, SemiAngle) among its own members, i.e. real fragments of
    the *same* second cone (a solid can have that cone split into several
    faces by an earlier cut)."""
    groups: list[list[GFace]] = []
    for f in base_faces:
        s = f.Surface
        if not isinstance(s, GCone):
            continue
        if not is_coaxial_cone_pair(tool_cone, s):
            continue
        for group in groups:
            gs = group[0].Surface
            if (
                abs(gs.SemiAngle - s.SemiAngle) < 1e-6
                and abs(gs.Axis.dot(s.Axis)) > 1.0 - 1e-5
                and (gs.Apex - s.Apex).length < 1e-5
            ):
                group.append(f)
                break
        else:
            groups.append([f])
    return groups


def _cone_v_value(point: GVector, native_cone_surf) -> float:
    return ShapeAnalysis_Surface(native_cone_surf).ValueOfUV(to_native_vector(point), 1e-6).Y()


def _find_v_crossings(face: "GFace", native_cone_surf, v0: float, samples: int = 64) -> "list[GVector]":
    """Points where `face`'s own outer-wire boundary crosses the constant
    V=v0 line on `native_cone_surf` (the surface `face` itself lies on) --
    i.e. where an analytically-known circle at that fixed V (see
    `_try_coaxial_cone_split`) crosses the face's *real* trimmed boundary.
    Samples each boundary edge and bisects across any sign change of
    (V - v0); works for any edge curve type (line, circle, BSpline...) and
    makes no assumption about how many boundary edges the face has."""
    crossings = []
    for edge in face.outer_wire().Edges:
        umin, umax = edge.ParameterRange
        prev_t = umin
        prev_v = _cone_v_value(edge.value_at(prev_t), native_cone_surf)
        for i in range(1, samples + 1):
            t = umin + (umax - umin) * i / samples
            v = _cone_v_value(edge.value_at(t), native_cone_surf)
            if (prev_v - v0) * (v - v0) < 0:
                lo, hi, lo_v = prev_t, t, prev_v
                for _ in range(60):
                    mid = (lo + hi) / 2.0
                    mid_v = _cone_v_value(edge.value_at(mid), native_cone_surf)
                    if (lo_v - v0) * (mid_v - v0) <= 0:
                        hi = mid
                    else:
                        lo, lo_v = mid, mid_v
                crossings.append(edge.value_at((lo + hi) / 2.0))
            prev_t, prev_v = t, v
    return crossings


def _split_face_at_v_line(native_face, native_cone_surf, point_a: GVector, point_b: GVector) -> list:
    """Split `native_face` (lying on `native_cone_surf`) at the constant-V
    line between `point_a`/`point_b` (both already confirmed to sit on
    that surface). The edge is built directly in the surface's own (U,V)
    space and needs breplib.BuildCurve3d before use as a splitting tool --
    otherwise BRepAlgoAPI_Splitter crashes the process natively rather
    than raising (confirmed 2026-08-18, under OCP; not independently
    re-confirmed under pythonocc-core, but the underlying edge -- lacking
    a 3D curve until this call -- is the same regardless of binding).
    Returns the resulting native faces (a 1-element list if the split
    didn't actually separate anything)."""
    sas = ShapeAnalysis_Surface(native_cone_surf)
    uv_a = sas.ValueOfUV(to_native_vector(point_a), 1e-6)
    uv_b = sas.ValueOfUV(to_native_vector(point_b), 1e-6)
    v_common = (uv_a.Y() + uv_b.Y()) / 2.0
    line2d = Geom2d_Line(gp_Pnt2d(0.0, v_common), gp_Dir2d(1.0, 0.0))
    u_lo, u_hi = sorted([uv_a.X(), uv_b.X()])
    edge = BRepBuilderAPI_MakeEdge(Geom2d_TrimmedCurve(line2d, u_lo, u_hi), native_cone_surf).Edge()
    breplib.BuildCurve3d(edge)

    splitter = BRepAlgoAPI_Splitter()
    args = TopTools_ListOfShape()
    args.Append(native_face)
    tools = TopTools_ListOfShape()
    tools.Append(edge)
    splitter.SetArguments(args)
    splitter.SetTools(tools)
    splitter.Build()
    if not splitter.IsDone():
        return [native_face]
    pieces = []
    exp = TopExp_Explorer(splitter.Shape(), TopAbs_FACE)
    while exp.More():
        pieces.append(topods.Face(exp.Current()))
        exp.Next()
    return pieces if pieces else [native_face]


def _try_coaxial_cone_split(base: "GSolid", tool: "GSolid", tolerance: float) -> "list[GSolid] | None":
    """Checked *before* the generic split is even attempted, whenever
    `tool` is a cone -- avoids wastefully running BOPAlgo_Splitter once on
    geometry already known to defeat it, then again after the presplit
    fix (see Gsplit). Targets a specific, real degeneracy: `tool`'s own
    cutting surface is a cone that is coaxial
    with, and shares the same semi-angle as, a *different* cone already on
    `base`'s own boundary (see vector_geometry.is_coaxial_cone_pair). Two
    coaxial cones with equal semi-angle intersect in an exact circle,
    which is a genuinely degenerate case for OCCT's own quadric-quadric
    solver (confirmed 2026-08-18 against a real fixture,
    Solidos/BadCAD_decomposition/SCDR_90_piece0_badvolume.stp:
    BOPAlgo_Splitter silently returns the unsplit solid at every tolerance
    from 0.1 to 1e-22; GeomAPI_IntSS "succeeds" but returns a wrong curve,
    confined to one meridian plane, oscillating between the two apexes,
    rather than the real circle). No parameter exposed by OCP or
    pythonocc-core resolves this; patching OCCT's own C++ solver was
    explicitly ruled out (would mean maintaining a permanent OCCT fork).

    Rather than reconstructing the whole cut face by hand, this resolves
    only the one genuinely degenerate piece -- the circular arc where the
    tool's cone crosses the other cone -- in closed form (center = midpoint
    of the two apexes, radius = half their distance), splits just that one
    real face of `base` at the arc (an ordinary, well-conditioned
    face-level operation, not a 3D solid-level one), and retries the
    *normal* BOP split on the resulting solid: once the arc already exists
    as real topology, the tool no longer needs to discover it via the
    degenerate solver, and the ordinary 3D split succeeds on its own.

    Finding a coaxial-cone pair on `base` is a candidate, not a guarantee
    -- the pair may be unrelated to this particular cut (a real
    counterexample, given directly by the user: the first cut attempted
    while decomposing the un-decomposed
    Solidos/BadCAD_decomposition/SCDR_90.stp, which this fixture was itself
    cut from, has this kind of coincidental match elsewhere in the model
    and the *generic* split already works correctly there). Returns None
    whenever the candidate doesn't pan out at any step (not exactly 2 arc
    crossings found, the face doesn't actually split, the presplit solid
    doesn't rebuild validly, or the retried split still doesn't produce a
    volume-conserving multi-solid result) -- every one of these is an
    ordinary, silent "not applicable here" outcome, never an error; the
    caller falls through to today's existing unchanged-solid behavior.
    """
    tool_cone_face = _find_cone_face(tool)
    if tool_cone_face is None:
        return None
    tool_cone = tool_cone_face.Surface

    for group in _group_coaxial_cone_faces(base.Faces, tool_cone):
        for other_face in group:
            other_cone = other_face.Surface
            mid_point = (tool_cone.Apex + other_cone.Apex) * 0.5
            radius = (tool_cone.Apex - other_cone.Apex).length / 2.0
            axis = tool_cone.Axis.normalized()

            ref = GVector(1, 0, 0)
            if abs(ref.dot(axis)) > 0.9:
                ref = GVector(0, 1, 0)
            u_dir = (ref - axis * ref.dot(axis)).normalized()
            probe_point = mid_point + u_dir * radius

            native_other_surf = BRep_Tool.Surface(other_face.__native__)
            v0 = _cone_v_value(probe_point, native_other_surf)

            crossings = _find_v_crossings(other_face, native_other_surf, v0)
            if len(crossings) != 2:
                continue

            split_pieces = _split_face_at_v_line(other_face.__native__, native_other_surf, crossings[0], crossings[1])
            if len(split_pieces) < 2:
                continue

            new_faces = [f for f in base.Faces if f is not other_face]
            new_faces += [GFace(p) for p in split_pieces]

            try:
                presplit = Gmake_solid(Gmake_shell(new_faces))
            except Exception:
                presplit = None
            if presplit is None:
                continue

            retry_native_solids, _, _ = _raw_bop_split(presplit.__native__, tool.__native__, tolerance)
            if len(retry_native_solids) < 2:
                continue
            retry_solids = [GSolid(s) for s in retry_native_solids]
            total_volume = sum(s.Volume for s in retry_solids)
            if abs(total_volume - base.Volume) > 1e-6 * max(abs(base.Volume), 1.0):
                continue
            if not all(s.is_valid() for s in retry_solids):
                continue
            return retry_solids

    return None


def Gsplit(
    base: GSolid, tool: GShape, tolerance: float, scale: float = 0.1, scale_up_floor: float | None = None
) -> SplitResult:
    if _find_cone_face(tool) is not None:
        fixed = _try_coaxial_cone_split(base, tool, tolerance)
        if fixed is not None:
            return SplitResult(
                solids=fixed,
                degenerate_case_handled=True,
                notes="coaxial cone degeneracy resolved analytically",
            )

    final_native_solids, repaired_any, tool_missed_entirely = _raw_bop_split(base.__native__, tool.__native__, tolerance)

    if tool_missed_entirely:
        return SplitResult(
            solids=[base], degenerate_case_handled=True, notes="tool did not intersect solid; returning it unchanged"
        )

    solids = [GSolid(s) for s in final_native_solids]
    return SplitResult(
        solids=solids,
        degenerate_case_handled=repaired_any,
        notes="non-manifold repair applied" if repaired_any else "",
    )


# ---------------------------------------------------------------------------
# Spatial queries between two independent shapes
# ---------------------------------------------------------------------------


def Gin_contact(shape_a: GShape, shape_b: GShape, tolerance: float) -> bool:
    native_a = shape_a.__native__
    native_b = shape_b.__native__

    box_a = _bnd_box(native_a)
    box_b = _bnd_box(native_b)
    if not (
        min(box_a.XMax, box_b.XMax) - max(box_a.XMin, box_b.XMin) > -tolerance
        and min(box_a.YMax, box_b.YMax) - max(box_a.YMin, box_b.YMin) > -tolerance
        and min(box_a.ZMax, box_b.ZMax) - max(box_a.ZMin, box_b.ZMin) > -tolerance
    ):
        return False

    try:
        return BRepExtrema_DistShapeShape(native_a, native_b).Value() < tolerance
    except Exception:
        pass

    try:
        common = BRepAlgoAPI_Common(native_a, native_b).Shape()
        if common.IsNull():
            return False
        props = _volume_props(common)
        if abs(props.Mass()) > 1e-8:
            return True
        for kind in (TopAbs_FACE, TopAbs_EDGE):
            if TopExp_Explorer(common, kind).More():
                return True
        return False
    except Exception:
        return False


def Gdistance(shape_a: GShape, shape_b: GShape) -> float:
    return BRepExtrema_DistShapeShape(shape_a.__native__, shape_b.__native__).Value()
