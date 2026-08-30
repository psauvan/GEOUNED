"""
geo/occ/topology.py

Analytic surface descriptors (GPlane/GCylinder/GCone/GSphere/GTorus),
curve descriptors (GLine/GCircle/GEllipse/GBSpline), and the neutral
topology classes (GEdge/GWire/GFace/GShell/GSolid) -- kept as ONE file
rather than split further: a real dependency analysis (2026-08-30, see
CLAUDE.md) found these are genuinely mutually recursive. `GShape` (the
type alias combining GSolid/GFace/GEdge/GShell) lives here too, even
though it sits physically elsewhere in the original file, since it
belongs with the classes it names.
"""

from __future__ import annotations

import OCC

from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import (
    BRepAdaptor_Curve,
    BRepAdaptor_Surface,
)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Copy,
    BRepBuilderAPI_Transform,
)
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.BRepTopAdaptor import BRepTopAdaptor_FClass2d
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
from OCC.Core.GeomAPI import GeomAPI_ProjectPointOnCurve
from OCC.Core.GeomLProp import (
    GeomLProp_CLProps,
    GeomLProp_SLProps,
)
from OCC.Core.gp import (
    gp_Ax1,
    gp_Dir,
    gp_Pnt,
    gp_Pnt2d,
    gp_Trsf,
    gp_Vec,
)
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_IN,
    TopAbs_SOLID,
    TopAbs_VERTEX,
)
from OCC.Core.TopExp import (
    TopExp_Explorer,
    topexp,
)
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import (
    TopoDS_Vertex,
    topods,
)
from ..vector_geometry import (
    GBoundBox,
    GVector,
)
from ..surface_geometry import (
    cylinder_tangent_at,
    cylinder_value_at,
    is_inside_cone,
    is_inside_cylinder,
    is_inside_plane,
    is_inside_sphere,
    is_inside_torus,
    plane_tangent_at,
    plane_value_at,
    torus_sheet_sign,
)
from ._native_utils import (
    _bnd_box,
    _linear_props,
    _native_fix,
    _orientation_str,
    _project_point_on_surface,
    _surface_props,
    _to_gmatrix_3x3,
    _to_gvector,
    _volume_props,
    to_native_vector,
)


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
        # See _freecad_impl.py's GTorus.__init__ for the full rationale
        # (self-intersecting torus, two sheets, a_sign disambiguation).
        self.Degenerated = self.MinorRadius > self.MajorRadius
        self.a_sign = 1

    @classmethod
    def from_values(cls, center: GVector, axis: GVector, major_radius: float, minor_radius: float) -> "GTorus":
        torus = cls.__new__(cls)
        torus.Center = center
        torus.Axis = axis
        torus.MajorRadius = major_radius
        torus.MinorRadius = minor_radius
        torus.__native__ = None
        torus.Degenerated = minor_radius > major_radius
        torus.a_sign = 1
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
        torus = GTorus(adaptor.Torus(), BRep_Tool.Surface(native_face))
        # torus.Degenerated already set by GTorus.__init__; refine
        # a_sign (default 1) from a real vertex of the originating face.
        if torus.Degenerated:
            vexp = TopExp_Explorer(native_face, TopAbs_VERTEX)
            vertex = _to_gvector(BRep_Tool.Pnt(topods.Vertex(vexp.Current())))
            torus.a_sign = torus_sheet_sign(vertex, torus)
        return torus
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
        from .io import _export_shapes_step  # lazy: io.py imports this module too

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
        # Ported from the `ocp` engine's own GFace (see its docstring for
        # the full corpus-verified rationale/thresholds) -- a surface-
        # type-agnostic "how compact is this face's own shape" measure,
        # piggybacking on the SurfaceProperties call already made above.
        # Area / RG_max^2, where RG_max is the largest of the 3 principal
        # radii of gyration: a compact shape has this ~O(1), a thin sliver
        # (even a curved one) has negligible area for its own spatial
        # extent, so this ratio comes out orders of magnitude smaller.
        principal = props.PrincipalProperties()
        rg_max = max(principal.RadiusOfGyration())
        self.Compactness = self.Area / (rg_max * rg_max) if rg_max > 1e-9 else float("inf")
        # The face's own true short physical dimension ("width"): for a
        # rectangle of length L and width W, RG_max == L/sqrt(12) exactly,
        # so W == Area/(RG_max*sqrt(12)) -- generalizes correctly to a
        # curved/annular sliver too. Used by Tolerances.min_face_width.
        self.CharacteristicWidth = self.Area / (rg_max * 3.4641016151377544) if rg_max > 1e-9 else 0.0

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
        from .io import _export_shapes_step  # lazy: io.py imports this module too

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
        from .io import _export_shapes_step  # lazy: io.py imports this module too

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
        # Thin wrapper -- see _native_fix's own docstring for the full,
        # hard-won history of every step in this repair (UnifyEdges-only-
        # when-already-valid, unify_edges' confirmed native-crash risk
        # kept on unconditionally per explicit user decision -- matching
        # _ocp_impl.py's own symmetric reversal, even though this
        # specific engine's crash-triggering flag is unify_edges, not
        # ocp's unify_faces, see this file's own module docstring -- the
        # fallback repair running on the original native shape rather
        # than UnifyEdges' own possibly-corrupted output). Extracted to a
        # module-level, native-in/native-out function so
        # Gload_and_process_step/Gcheck_and_repair can call it without
        # ever constructing a GSolid first -- one copy of this fragile
        # logic, not two to keep in sync.
        return GSolid(_native_fix(self.__native__, tolerance))

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
        same either way).

        Disabling unify_edges here (2026-08-23) was reverted the next
        day (2026-08-24), matching fix()'s own reversal -- see its
        docstring for the full account. unify_edges=True is restored
        unconditionally; ConeSphere.stp is accepted as a known,
        unresolved crash/hang under this engine.

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
        unify = ShapeUpgrade_UnifySameDomain(copy, True, True, True)
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
        from .io import _export_shapes_step  # lazy: io.py imports this module too

        _export_shapes_step([self.__native__], filename)

    def copy(self) -> "GSolid":
        return GSolid(BRepBuilderAPI_Copy(self.__native__).Shape())

    def transform_geometry(self, matrix) -> "GSolid":
        """Apply a rigid affine transform (a native gp_Trsf). See
        _freecad_impl.py's own docstring -- not currently used anywhere
        (GEOReverse, this method's only real consumer, is FreeCAD-only),
        kept for backend symmetry."""
        return GSolid(BRepBuilderAPI_Transform(self.__native__, matrix, True).Shape())


# A shape-like argument accepted by generic spatial queries (Gin_contact...).
GShape = GSolid | GFace | GEdge | GShell
