"""
geo/_freecad_impl.py

FreeCAD/Part implementation of the `geo` package -- the ONLY file in
GEOUNED allowed to `import Part`/`FreeCAD`/`BOPTools`. Everything else
imports the GSolid/GFace/.../Gmake_*/Gsplit/... names from `geo`
(re-exported by `geo/__init__.py`), never Part/FreeCAD directly.

This replaces the earlier `GeometryBackend` ABC + dependency-injected
`FreeCADBackend` design: instead of `_backend.method(gsolid, ...)`, the
neutral classes themselves carry native + behavior (`gsolid.method(...)`).
Engine-swappability (the original motivation for the ABC) is now achieved
at the module level instead: a future `_occ_impl.py` would define the
same class/function names against pythonOCC, and `geo/__init__.py` would
choose which one to re-export.

Known limitation: `Gsplit()` here only ports GEOUNED's existing
tolerance-scaling retry, which handles the kernel raising an exception at
very small tolerances. It does NOT solve the silent-uncut-solid tangency
bug described in the project's motivating problem (a plane's intersection
with a solid coinciding with a pre-existing tangency line) -- today
GEOUNED only works around that case via a STEP export/import round-trip
elsewhere in the pipeline. The proper fix (face-adjacency graph excluding
non-manifold edges, reconstructing solids per connected component) is
reserved for a future OCC implementation, per the migration plan.
"""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass

import BOPTools.SplitAPI
import FreeCAD
import Part
from FreeCAD import Import

from .vector_geometry import (
    GBoundBox,
    GLabelNode,
    GMatrix,
    GVector,
    cylinder_tangent_at,
    cylinder_value_at,
    is_inside_cone,
    is_inside_cylinder,
    is_inside_plane,
    is_inside_sphere,
    is_inside_torus,
    plane_tangent_at,
    plane_value_at,
    to_gboundbox,
    to_gmatrix,
    to_gvector,
)


def to_fc_vector(vector: GVector) -> FreeCAD.Vector:
    """Write-side half of the transitional pair with `vector_geometry.to_gvector` --
    materializes a neutral GVector back into a native FreeCAD.Vector, needed only
    where geo code calls a native Part/FreeCAD function directly."""
    return FreeCAD.Vector(vector.x, vector.y, vector.z)


def kernel_version() -> str:
    """
    Version string of the underlying geometry kernel/application (e.g.
    FreeCAD's own version), for provenance notes in output file headers.
    Purely informational -- callers must not parse or branch on the format.
    """
    return "{V[0]}.{V[1]}.{V[2]}".format(V=FreeCAD.Version())


# ---------------------------------------------------------------------------
# Analytic surface descriptors (wrap a native face's Surface geometry, not
# the face itself; the backend only knows about these 5 -- composite
# meta-surfaces (RoundCorner, Can, TCone, MultiPlane...) are assembled by
# GEOUNED itself out of these via Gcut/Gcommon/Gfuse, not modeled here)
# ---------------------------------------------------------------------------


class GPlane:
    """Field names match FreeCAD's own `Part.Plane` attribute names."""

    def __init__(self, native):
        self.Position = to_gvector(native.Position)
        self.Axis = to_gvector(native.Axis)
        # Reference "u=0" direction (OCCT's XDirection), needed to evaluate
        # value_at/tangent_at analytically so (u, v) agrees with the
        # native face's own parametrization.
        self.XDir = to_gvector(native.Rotation.multVec(FreeCAD.Vector(1, 0, 0)))
        self.__native__ = native

    @classmethod
    def from_values(cls, position: GVector, axis: GVector, xdir: GVector | None = None) -> "GPlane":
        """
        Build a GPlane from already-known values, with no native face/
        surface backing it (e.g. reconstructed from previously-classified
        data rather than classified fresh off a face). `value_at`/
        `tangent_at` raise unless `xdir` is given.
        """
        plane = cls.__new__(cls)
        plane.Position = position
        plane.Axis = axis
        plane.XDir = xdir
        plane.__native__ = None
        return plane

    def parameter(self, point: GVector) -> tuple[float, float]:
        """Parametric coordinates (u, v) of the nearest point on the plane to `point`."""
        return self.__native__.parameter(to_fc_vector(point))

    def value_at(self, u: float, v: float) -> GVector:
        return plane_value_at(self, u, v)

    def tangent_at(self, u: float, v: float) -> tuple[GVector, GVector]:
        return plane_tangent_at(self, u, v)

    def is_inside(self, point: GVector) -> bool:
        """Which side of the (infinite) plane `point` is on -- True on the
        side `Axis` points toward. Delegates to `vector_geometry.is_inside_plane`,
        shared with `boolean_solids.check_sign_primitive`'s Plane branch
        (which calls it directly on a PlaneParams, not a GPlane -- both
        store the same fields, so the same formula works on either)."""
        return is_inside_plane(point, self)

    def transform(self, matrix) -> "GPlane":
        """Apply a native FreeCAD.Matrix affine transform, returning a new
        GPlane (this one is not mutated). Not currently exercised by
        CadToCsg (the corresponding Objects.py::Plane.transform() this
        replaces was confirmed dead there) -- kept/ported for CsgToCad
        (GEOReverse), which does need to move surfaces around."""
        position = to_gvector(matrix.multVec(to_fc_vector(self.Position)))
        axis = to_gvector(matrix.submatrix(3).multVec(to_fc_vector(self.Axis))).normalized()
        return GPlane.from_values(position, axis)

    def intersect_plane(self, other: "GPlane") -> "GLine | None":
        """
        Intersection line of this (infinite) plane with `other`. Returns
        None if the planes are parallel (or coincident).

        Pure GVector math for the well-conditioned case: verified against
        Part.Plane.intersect() across random plane pairs down to ~0.01 rad
        (~0.6 deg) from parallel, matching to within floating-point noise
        (relative perpendicular deviation ~1e-8 or better). Below that
        angle the point-on-line computation becomes genuinely
        ill-conditioned even with the numerically-stabilized pivot used
        here (verified: relative deviation grows to ~1e-3 by 1e-4 rad) --
        that regime falls back to a transient native Part.Plane/
        Part.Plane.intersect() instead of risking a silently-wrong point,
        with a wide safety margin below the verified-good boundary.
        """
        n1, n2 = self.Axis, other.Axis
        d = n1.cross(n2)
        dl = d.length
        if dl < 1e-10:
            return None  # parallel or coincident

        if dl < 0.05:  # well below the verified-safe 0.01 rad boundary
            native1 = Part.Plane(to_fc_vector(self.Position), to_fc_vector(n1))
            native2 = Part.Plane(to_fc_vector(other.Position), to_fc_vector(n2))
            lines = native1.intersect(native2)
            if not lines:
                return None
            l = lines[0]
            return GLine.from_values(to_gvector(l.Location), to_gvector(l.Direction))

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


class GCylinder:
    def __init__(self, native):
        self.Center = to_gvector(native.Center)
        self.Axis = to_gvector(native.Axis)
        self.Radius = native.Radius
        self.XDir = to_gvector(native.Rotation.multVec(FreeCAD.Vector(1, 0, 0)))
        self.__native__ = native

    @classmethod
    def from_values(cls, center: GVector, axis: GVector, radius: float, xdir: GVector | None = None) -> "GCylinder":
        """Build a GCylinder from already-known values, with no native face/surface backing it. `value_at`/`tangent_at` raise unless `xdir` is given."""
        cylinder = cls.__new__(cls)
        cylinder.Center = center
        cylinder.Axis = axis
        cylinder.Radius = radius
        cylinder.XDir = xdir
        cylinder.__native__ = None
        return cylinder

    def parameter(self, point: GVector) -> tuple[float, float]:
        """Parametric coordinates (u, v) of the nearest point on the cylinder to `point`."""
        return self.__native__.parameter(to_fc_vector(point))

    def value_at(self, u: float, v: float) -> GVector:
        return cylinder_value_at(self, u, v)

    def tangent_at(self, u: float, v: float) -> tuple[GVector, GVector]:
        return cylinder_tangent_at(self, u, v)

    def is_inside(self, point: GVector) -> bool:
        """True if `point` is outside the (infinite) cylinder. See
        GPlane.is_inside for the shared-formula rationale."""
        return is_inside_cylinder(point, self)

    def transform(self, matrix) -> "GCylinder":
        """Apply a native FreeCAD.Matrix affine transform, returning a new
        GCylinder. See GPlane.transform for why this isn't exercised by
        CadToCsg today but is kept for CsgToCad (GEOReverse)."""
        center = to_gvector(matrix.multVec(to_fc_vector(self.Center)))
        axis = to_gvector(matrix.submatrix(3).multVec(to_fc_vector(self.Axis)))
        return GCylinder.from_values(center, axis, self.Radius)


class GCone:
    def __init__(self, native):
        self.Apex = to_gvector(native.Apex)
        self.Axis = to_gvector(native.Axis)
        self.SemiAngle = native.SemiAngle  # radians
        self.Radius = native.Radius  # radius of the reference circle at the face's v=0
        self.__native__ = native

    @classmethod
    def from_values(cls, apex: GVector, axis: GVector, semi_angle: float, radius: float | None = None) -> "GCone":
        """Build a GCone from already-known values, with no native face/
        surface backing it. `radius` (the reference circle at v=0) has no
        meaning without a native face's own parametrization, so it's left
        None unless the caller actually has it."""
        cone = cls.__new__(cls)
        cone.Apex = apex
        cone.Axis = axis
        cone.SemiAngle = semi_angle
        cone.Radius = radius
        cone.__native__ = None
        return cone

    def parameter(self, point: GVector) -> tuple[float, float]:
        """Parametric coordinates (u, v) of the nearest point on the cone to `point`."""
        return self.__native__.parameter(to_fc_vector(point))

    def is_inside(self, point: GVector) -> bool:
        """True if `point` is outside the (infinite, single-sheet) cone.
        See GPlane.is_inside for the shared-formula rationale."""
        return is_inside_cone(point, self)

    def transform(self, matrix) -> "GCone":
        """Apply a native FreeCAD.Matrix affine transform, returning a new
        GCone. See GPlane.transform for why this isn't exercised by
        CadToCsg today but is kept for CsgToCad (GEOReverse)."""
        apex = to_gvector(matrix.multVec(to_fc_vector(self.Apex)))
        axis = to_gvector(matrix.submatrix(3).multVec(to_fc_vector(self.Axis)))
        return GCone.from_values(apex, axis, self.SemiAngle, self.Radius)


class GSphere:
    def __init__(self, native):
        self.Center = to_gvector(native.Center)
        self.Radius = native.Radius
        self.__native__ = native

    @classmethod
    def from_values(cls, center: GVector, radius: float) -> "GSphere":
        """Build a GSphere from already-known values, with no native face/surface backing it."""
        sphere = cls.__new__(cls)
        sphere.Center = center
        sphere.Radius = radius
        sphere.__native__ = None
        return sphere

    def parameter(self, point: GVector) -> tuple[float, float]:
        """Parametric coordinates (u, v) of the nearest point on the sphere to `point`."""
        return self.__native__.parameter(to_fc_vector(point))

    def is_inside(self, point: GVector) -> bool:
        """True if `point` is outside the sphere. See GPlane.is_inside
        for the shared-formula rationale."""
        return is_inside_sphere(point, self)

    def transform(self, matrix) -> "GSphere":
        """Apply a native FreeCAD.Matrix affine transform, returning a new
        GSphere. See GPlane.transform for why this isn't exercised by
        CadToCsg today but is kept for CsgToCad (GEOReverse)."""
        center = to_gvector(matrix.multVec(to_fc_vector(self.Center)))
        return GSphere.from_values(center, self.Radius)


class GTorus:
    def __init__(self, native):
        self.Center = to_gvector(native.Center)
        self.Axis = to_gvector(native.Axis)
        self.MajorRadius = native.MajorRadius
        self.MinorRadius = native.MinorRadius
        self.__native__ = native

    @classmethod
    def from_values(cls, center: GVector, axis: GVector, major_radius: float, minor_radius: float) -> "GTorus":
        """Build a GTorus from already-known values, with no native face/surface backing it."""
        torus = cls.__new__(cls)
        torus.Center = center
        torus.Axis = axis
        torus.MajorRadius = major_radius
        torus.MinorRadius = minor_radius
        torus.__native__ = None
        return torus

    def parameter(self, point: GVector) -> tuple[float, float]:
        """Parametric coordinates (u, v) of the nearest point on the torus to `point`."""
        return self.__native__.parameter(to_fc_vector(point))

    def is_inside(self, point: GVector) -> bool:
        """True if `point` is outside the torus. See GPlane.is_inside
        for the shared-formula rationale."""
        return is_inside_torus(point, self)


def Gclassify_surface(native_face):
    """
    Determine the underlying surface type of a face (plane/cylinder/cone/
    sphere/torus) and build its GEOUNED descriptor. Returns None for a
    surface type GEOUNED can't model (a genuine BSplineSurface, a
    SurfaceOfRevolution/SurfaceOfExtrusion, ...) -- e.g. loading a STEP
    file with a spline-surfaced solid must not crash just from GSolid/
    GFace eagerly walking its faces (confirmed by testing a real loft:
    `GSolid(loft_with_bspline_sides)` raised before this was made
    tolerant). Detecting "this face's surface isn't one we support" and
    deciding what to do about it (skip the solid, warn, stop) is a
    GEOUNED-level policy decision, not something to fail on here -- the
    hard failure for a genuinely unsupported surface happens later, only
    if GEOUNED actually tries to write it out as an MCNP/OpenMC surface.
    """
    surface = native_face.Surface
    kind = type(surface)
    if kind is Part.Plane:
        return GPlane(surface)
    if kind is Part.Cylinder:
        return GCylinder(surface)
    if kind is Part.Cone:
        return GCone(surface)
    if kind is Part.Sphere:
        return GSphere(surface)
    if kind is Part.Toroid:
        return GTorus(surface)
    if kind is Part.BSplineSurface:
        # the only acceptable case for a BSplineSurface is one that's
        # geometrically just a mislabeled plane (some CAD exports do this
        # for flat faces) -- otherwise it's a real spline, unsupported.
        plane = native_face.findPlane()
        if plane is not None:
            return GPlane(plane)
        return None
    return None


# ---------------------------------------------------------------------------
# Analytic curve descriptors (wrap a native edge's Curve geometry)
# ---------------------------------------------------------------------------


class GLine:
    def __init__(self, native):
        self.Position = to_gvector(native.Location)
        self.Direction = to_gvector(native.Direction)
        self.__native__ = native

    @classmethod
    def from_values(cls, position: GVector, direction: GVector) -> "GLine":
        """Build a GLine from already-known values, with no native edge/curve backing it (e.g. a plane-plane intersection line)."""
        line = cls.__new__(cls)
        line.Position = position
        line.Direction = direction
        line.__native__ = None
        return line

    def intersect_line(self, other: "GLine") -> GVector | None:
        """
        Intersection point of this (infinite) line with `other`. Returns
        None if the lines are parallel, or skew (not coplanar).

        Pure GVector math for the well-conditioned case, verified against
        Part.Line.intersect() across random coplanar line pairs (skew
        pairs verified separately: the coplanarity gap is either ~1e-15
        or clearly nonzero, never ambiguous). Falls back to a transient
        native Part.Line/Part.Line.intersect() below ~0.01 rad from
        parallel, mirroring GPlane.intersect_plane's verified boundary --
        the underlying instability has the same shape (both divide by a
        |cross product|^2-scale term).
        """
        d1, d2 = self.Direction, other.Direction
        cr = d1.cross(d2)
        crl = cr.length
        if crl < 1e-10:
            return None  # parallel

        w = other.Position - self.Position
        scale_ref = max(self.Position.length, other.Position.length, 1.0)

        if crl < 0.05:
            native1 = Part.Line(to_fc_vector(self.Position), to_fc_vector(self.Position + d1))
            native2 = Part.Line(to_fc_vector(other.Position), to_fc_vector(other.Position + d2))
            pts = native1.intersect(native2)
            if not pts:
                return None
            return to_gvector(pts[0].toShape().Point)

        if abs(w.dot(cr)) / crl > 1e-6 * scale_ref:
            return None  # skew lines, no true intersection

        t = (w.cross(d2)).dot(cr) / (crl * crl)
        return self.Position + d1 * t

    def value(self, u: float) -> GVector:
        """Point on the line at parametric coordinate `u` (distance along the line from `Position`)."""
        return to_gvector(self.__native__.value(u))

    def parameter(self, point: GVector) -> float:
        """Parametric coordinate `u` (distance along the line from `Position`) of the nearest point on the line to `point`."""
        return self.__native__.parameter(to_fc_vector(point))


class GCircle:
    def __init__(self, native):
        self.Center = to_gvector(native.Center)
        self.Axis = to_gvector(native.Axis)
        self.Radius = native.Radius
        self.__native__ = native

    def value(self, u: float) -> GVector:
        """Point on the circle at parametric coordinate `u` (radians)."""
        return to_gvector(self.__native__.value(u))

    def parameter(self, point: GVector) -> float:
        """Parametric coordinate `u` (radians) of the nearest point on the circle to `point`."""
        return self.__native__.parameter(to_fc_vector(point))


class GEllipse:
    """Field names match FreeCAD's own `Part.Ellipse` attribute names."""

    def __init__(self, native):
        self.Center = to_gvector(native.Center)
        self.Axis = to_gvector(native.Axis)
        self.XAxis = to_gvector(native.XAxis)
        self.YAxis = to_gvector(native.YAxis)
        self.MajorRadius = native.MajorRadius
        self.MinorRadius = native.MinorRadius
        self.__native__ = native

    def value(self, u: float) -> GVector:
        """Point on the ellipse at parametric coordinate `u` (radians)."""
        return to_gvector(self.__native__.value(u))

    def parameter(self, point: GVector) -> float:
        """Parametric coordinate `u` (radians) of the nearest point on the ellipse to `point`."""
        return self.__native__.parameter(to_fc_vector(point))


class GBSpline:
    def __init__(self, native):
        self.Poles = [to_gvector(pole) for pole in native.getPoles()]
        self.__native__ = native

    def value(self, u: float) -> GVector:
        """Point on the B-spline at parametric coordinate `u` (radians)."""
        return to_gvector(self.__native__.value(u))

    def parameter(self, point: GVector) -> float:
        """Parametric coordinate `u` of the nearest point on the B-spline to `point`."""
        return self.__native__.parameter(to_fc_vector(point))


def Gclassify_curve(native_edge):
    """
    Determine the underlying curve type of an edge (line/circle/ellipse/
    bspline) and build its GEOUNED descriptor. Returns None both for a
    degenerate edge (e.g. the zero-length "pole" edges of a sphere or a
    cone's apex -- FreeCAD itself raises TypeError accessing `.Curve` on
    one) and for a real but unsupported curve type (e.g. a Hyperbola,
    which some STEP imports produce on an incidental edge that nothing
    downstream actually needs classified). Unlike faces (`Gclassify_surface`,
    which does raise): GSolid/GFace build eagerly, so an edge whose curve
    GEOUNED can't model must not abort building the *solid* just because
    that one edge was walked -- only classifying a face's Surface (the
    thing that actually becomes an MCNP/OpenMC surface) is a hard failure.

    Tolerates being called with an already-wrapped GEdge (e.g. from a
    GWire's `.Edges`, itself built by `pick_outer_wire`) instead of a
    native edge -- GEdge.Curve is already the classified result from its
    own construction, so it's returned directly rather than misclassified
    a second time (a native `.Curve` lookup on a GEdge would find GEdge's
    own `.Curve` attribute, whose type never matches any `Part.*` check
    below, silently returning None regardless of the real curve type).
    """
    if isinstance(native_edge, GEdge):
        return native_edge.Curve
    try:
        curve = native_edge.Curve
    except TypeError:
        return None
    kind = type(curve)
    if kind is Part.Line:
        return GLine(curve)
    if kind is Part.Circle:
        return GCircle(curve)
    if kind is Part.Ellipse:
        return GEllipse(curve)
    if kind is Part.BSplineCurve:
        return GBSpline(curve)
    return None


# ---------------------------------------------------------------------------
# Neutral topology types -- eagerly built from their native equivalent,
# whether read from a STEP file or constructed internally by GEOUNED.
# ---------------------------------------------------------------------------


class GEdge:
    def __init__(self, native):
        self.__native__ = native
        self.Curve = Gclassify_curve(native)
        self.Vertexes = [to_gvector(v.Point) for v in native.Vertexes]
        self.ParameterRange = native.ParameterRange
        self.Orientation = native.Orientation
        self.Length = native.Length
        self.MatrixOfInertia = to_gmatrix(native.MatrixOfInertia)

    def value_at(self, u: float) -> GVector:
        return to_gvector(self.__native__.valueAt(u))

    def derivative1_at(self, u: float) -> GVector:
        """
        First derivative of the edge's curve at parametric coordinate `u`.
        NOT unit-normalized -- e.g. on a circle of radius r this has
        length r, not 1.
        """
        return to_gvector(self.__native__.derivative1At(u))

    def normal_at(self, u: float) -> GVector:
        """Curve normal at parametric coordinate `u`. Only meaningful for a curved edge -- undefined (raises) for a straight line."""
        return to_gvector(self.__native__.normalAt(u))

    def parameter(self, point: GVector) -> float:
        """
        Parametric coordinate of the nearest point on the edge's curve to
        `point`. Goes straight to the native curve rather than through
        `.Curve` (which is None for a curve type Gclassify_curve doesn't
        model, e.g. Hyperbola/Parabola) -- `.parameter()` itself is a basic
        native curve operation, available regardless of classification.
        """
        return self.__native__.Curve.parameter(to_fc_vector(point))

    def is_same(self, other: "GEdge") -> bool:
        """
        True if `self` and `other` are the same underlying topological
        edge -- identity, not geometric coincidence (two distinct edges
        that happen to trace the same curve are NOT "same").
        """
        return self.__native__.isSame(other.__native__)

    def is_inside(self, point: GVector, tolerance: float) -> bool:
        """
        True if `point` lies on this (trimmed) edge within `tolerance`.
        Equivalent to `distToShape` against a single-point vertex shape,
        but works directly off the edge -- verified empirically against
        `distToShape` across line/circle/ellipse/BSpline edges, including
        endpoints, the tolerance boundary, and points beyond a curve's
        trim on its periodic/infinite extension.
        """
        return self.__native__.isInside(to_fc_vector(point), tolerance, True)

    def export_step(self, filename: str) -> None:
        Part.makeCompound([self.__native__]).exportStep(filename)


class GWire:
    def __init__(self, native):
        self.__native__ = native
        # in wire traversal order (edge[i] and edge[i+1] share a vertex)
        self.CenterOfMass = native.CenterOfMass
        self.OrderedVertexes = [to_gvector(v.Point) for v in native.OrderedVertexes]
        self.Edges = [GEdge(e) for e in native.OrderedEdges]
        self.MatrixOfInertia = to_gmatrix(native.MatrixOfInertia)


def pick_outer_wire(wires: list[GWire]) -> "GWire":
    """
    GEOUNED's own heuristic (largest mean vertex-to-centroid distance
    among the face's wires), not FreeCAD's native `Face.OuterWire` --
    the native attribute picks the wrong wire for some faces (e.g. a
    face with a through-hole). Public (used both by `GFace.outer_wire()`
    below and directly by `geometry_gu.py::FaceGu`, which needs the outer
    wire natively -- no reason for that file to reimplement the same
    algorithm a second time).
    """

    if len(wires) == 1:
        return wires[0]
    best_wire = None
    best_extension = 0.0
    for wire in wires:
        vertices = wire.OrderedVertexes
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
        self.Edges = [GEdge(e) for e in native.Edges]
        self.Vertexes = [to_gvector(v.Point) for v in native.Vertexes]
        self.BoundBox = to_gboundbox(native.BoundBox)
        self.ParameterRange = native.ParameterRange
        self.Orientation = native.Orientation
        self.Area = native.Area
        self.CenterOfMass = to_gvector(native.CenterOfMass)

        # assigned later by whoever built the face list this face came
        # from (its position within the parent solid's face list, e.g.
        # for "is this the same face" adjacency checks); no meaningful
        # value until then
        self.index: int | None = None
        # Wires/OuterWire are NOT part of GFace's eager enrichment (unlike
        # Surface/Edges/Vertexes above): OuterWire in particular runs
        # pick_outer_wire's heuristic over every wire of the face, which
        # is wasted work for the common case (most callers only ever touch
        # Surface/Edges/Vertexes) -- computed lazily instead, on the first
        # actual call to wires()/outer_wire(), and cached after that.
        self.__wires__: "list[GWire] | None" = None
        self.__outer_wire__: "GWire | None" = None

    def wires(self) -> "list[GWire]":
        """All wires bounding this face. Computed lazily and cached."""
        if self.__wires__ is None:
            self.__wires__ = [GWire(w) for w in self.__native__.Wires]
        return self.__wires__

    def outer_wire(self) -> "GWire":
        """
        The face's outer wire, via GEOUNED's own largest-mean-vertex-distance
        heuristic (`pick_outer_wire`) -- not FreeCAD's native `Face.OuterWire`,
        which picks the wrong wire for some faces (e.g. one with a through-hole).
        Computed lazily and cached.
        """
        if self.__outer_wire__ is None:
            self.__outer_wire__ = pick_outer_wire(self.wires())
        return self.__outer_wire__

    def isEqual(self, face: "GFace") -> bool:
        return self.__native__.isEqual(face.__native__)

    def isSame(self, face: "GFace") -> bool:
        return self.__native__.isSame(face.__native__)

    def value_at(self, u: float, v: float) -> GVector:
        return to_gvector(self.__native__.valueAt(u, v))

    def normal_at(self, u: float, v: float) -> GVector:
        return to_gvector(self.__native__.normalAt(u, v))

    def tangent_at(self, u: float, v: float) -> tuple[GVector, GVector]:
        d_u, d_v = self.__native__.tangentAt(u, v)
        return to_gvector(d_u), to_gvector(d_v)

    def parameter(self, point: GVector) -> tuple[float, float]:
        """(u, v) parametric coordinates of `point`, assumed to lie on the face. Inverse of `value_at`."""
        return self.__native__.Surface.parameter(to_fc_vector(point))

    def is_part_of_domain(self, u: float, v: float) -> bool:
        """True if (u, v) lies within the face's actual trimmed boundary, not just its parameter-range rectangle."""
        return self.__native__.isPartOfDomain(u, v)

    def tessellate(self, tolerance: float, reset: bool = False) -> list[GVector]:
        vertices, _facets = self.__native__.tessellate(tolerance, reset)
        return [to_gvector(v) for v in vertices]

    def getUVNodes(self):
        return self.__native__.getUVNodes()

    def orientation_outward(self, solid: "GSolid") -> bool:
        """True if this face's normal, as oriented in `solid`, points outward from the material."""
        u_min, u_max, v_min, v_max = self.ParameterRange
        u = (u_min + u_max) / 2.0
        v = (v_min + v_max) / 2.0
        point = self.__native__.valueAt(u, v)
        normal = self.__native__.normalAt(u, v)
        probe = point + normal * 1e-6
        return not solid.__native__.isInside(probe, 1e-7, False)

    def export_step(self, filename: str) -> None:
        Part.makeCompound([self.__native__]).exportStep(filename)

    def distance_to(self, other: "GFace") -> float:
        """Minimum distance between this face and `other` (0 if they touch or overlap). Native boolean/distance query, no GVector equivalent."""
        return self.__native__.distToShape(other.__native__)[0]

    def my_distToshape(self, other: "GFace") -> float:
        """alternative to distance_to, for testing -- same as distance_to. Doesn't call native distToShape but use boundbox distances and Common object of to check if solids touch eah other"""

        shape1 = self.__native__
        shape2 = other.__native__

        if shape1 is shape2:
            return 0.0
        else:
            Boxinter = shape1.BoundBox.intersected(shape2.BoundBox)
            intersect = Boxinter.XLength > -1e-6 and Boxinter.YLength > -1e-6 and Boxinter.ZLength > -1e-6
            if intersect:
                try:
                    inter = shape1.common(shape2)
                except:
                    inter = shape2.common(shape1)

                if abs(inter.Volume) > 1e-8 or len(inter.Solids) > 0 or len(inter.Faces) > 0 or len(inter.Edges) > 0:
                    dist2Shape = 0.0
                else:
                    same = False
                    for e1 in shape1.Edges:
                        if same:
                            break
                        for e2 in shape2.Edges:
                            if e1.isSame(e2):
                                dist2Shape = 0.0
                                same = True
                                break
                    if not same:
                        dist2Shape = 1.0
            else:
                c1 = shape1.BoundBox.Center
                c2 = shape2.BoundBox.Center
                d = c2 - c1
                dist2Shape = d.Length
            return dist2Shape


class GShell:
    """
    A connected group of faces, sitting between GFace and GSolid in the
    topology hierarchy. GEOUNED builds these itself (e.g. grouping faces
    that share the same analytic surface but got split into separate
    topological faces by a seam or a tangency line) via `Gmake_shell`,
    not by classification off an existing solid.
    """

    def __init__(self, native, faces: list[GFace] | None = None):
        self.__native__ = native
        self.Faces = faces if faces is not None else [GFace(f) for f in native.Faces]
        self.Orientation = native.Orientation

    def export_step(self, filename: str) -> None:
        Part.makeCompound([self.__native__]).exportStep(filename)


class GSolid:
    def __init__(self, native):
        self.Solids = []
        self.Faces = [GFace(f) for f in native.Faces]
        for index, face in enumerate(self.Faces):
            face.index = index
        self.Edges = [GEdge(e) for e in native.Edges]
        self.Vertexes = [to_gvector(v.Point) for v in native.Vertexes]
        self.BoundBox = to_gboundbox(native.BoundBox)
        self.Orientation = native.Orientation
        self.Area = native.Area
        self.Volume = native.Volume

        if len(native.Solids) > 1:
            for s in native.Solids:
                solid = GSolid(s)
                self.Solids.append(solid)
        else:
            self.Solids.append(self)

        self.__native__ = native
        # individual native solid pieces of (possibly compound) `native`
        # -- e.g. find_interior_point/export need to iterate these
        self.__shapes__ = native.Solids

    def is_inside(self, point: GVector, tolerance: float = 0.0) -> bool:
        """True if `point` lies inside the solid (equivalent to Part.Shape.isInside())."""
        return self.__native__.isInside(to_fc_vector(point), tolerance, False)

    def optimal_bounding_box(self, use_triangulation: bool = True) -> GBoundBox:
        """
        More accurate axis-aligned bounding box than `.BoundBox` (computed
        from the actual shape rather than the kernel's fast estimate), at
        higher cost. Still axis-aligned, not oriented -- same shape as
        `.BoundBox`, just tighter for curved surfaces.
        `use_triangulation=False` trades some accuracy for speed.
        """
        return to_gboundbox(self.__native__.optimalBoundingBox(use_triangulation))

    def center_of_mass(self) -> GVector:
        return to_gvector(self.__native__.CenterOfMass)

    def find_interior_point(self) -> GVector | None:
        """
        A point strictly inside the solid's volume, or None if none could
        be found (near-zero-volume/degenerate solid).

        Tries the center of mass first, then probes inward from each face
        along its inward normal. Benchmarked against two alternative
        strategies (sampling along solid-vertex segments; recursive
        bounding-box octree subdivision) that GEOUNED used to implement at
        several call sites: both are slower AND, for solids of revolution
        with few/no real vertices (a thin torus/fillet -- exactly what
        RoundCorner/TCone produce), the octree fallback can fail outright
        within its fixed subdivision depth. Face-normal probing has
        neither weakness, so it is the only strategy implemented here.
        """
        native = self.__native__
        point = self.__shapes__[0].CenterOfMass
        if native.isInside(point, 0.0, False):
            return to_gvector(point)

        length = 0.5 * abs(native.Volume) ** 0.33333
        for face in native.Faces:
            u_min, u_max, v_min, v_max = face.ParameterRange
            u = 0.5 * (u_min + u_max)
            v = 0.5 * (v_min + v_max)
            if face.isPartOfDomain(u, v):
                normal = -face.normalAt(u, v)
                pos = face.valueAt(u, v)
                d = length
                for _ in range(12):
                    d = d * 0.5
                    point = pos + d * normal
                    if native.isInside(point, 0.0, False):
                        return to_gvector(point)
        return None

    def faces_sharing_edge(self, edge: GEdge) -> list[GFace]:
        """
        Faces of this solid sharing `edge`. A 'normal' edge in a closed
        solid is shared by 2 faces; >2 is the signal of a non-manifold
        condition (the tangency case that motivated this migration).
        """
        matches = []
        for face in self.__native__.Faces:
            for candidate in face.Edges:
                if candidate.isSame(edge.__native__):
                    matches.append(face)
                    break
        return [GFace(f) for f in matches]

    def is_valid(self) -> bool:
        """Equivalent to BRepCheck_Analyzer / shape.isValid()."""
        return self.__native__.isValid()

    def fix(self, tolerance: float) -> "GSolid":
        """Attempts to repair an invalid solid or one with degenerate topology (equivalent to ShapeFix_Shape / Part.Shape.fix())."""
        shape = self.__native__.removeSplitter()
        if not shape.isValid():
            shape = shape.copy()
            shape.fix(tolerance, tolerance, tolerance)
        return GSolid(shape)

    def reverse(self) -> "GSolid":
        """A new GSolid with flipped orientation (equivalent to `Part.Shape.reverse()`). Needed after a boolean operation reports a negative volume."""
        reversed_shape = self.__native__.copy()
        reversed_shape.reverse()
        return GSolid(reversed_shape)

    def refine(self) -> "GSolid":
        """
        Remove redundant edges/faces left by a boolean operation between
        coplanar/tangent surfaces (equivalent to `Part.Shape.removeSplitter()`).
        Meant to be a purely cosmetic simplification that never changes the
        enclosed volume -- but on some tangent/cavity topologies OCC's
        `removeSplitter()` silently corrupts the shape instead (confirmed on
        a solid-with-cavity boolean-cut result, where it inflated the volume
        by ~3.4% and flipped face orientations on the cavity's boundary).
        Falls back to the un-refined solid whenever the volume moved by more
        than floating-point noise, since a real simplification is supposed
        to be volume-invariant by definition. `removeSplitter()` mutates the
        shape it's called on as a side effect (confirmed empirically: even
        though it returns a distinct object, the *receiver*'s own `.Volume`
        changes too, to a third, still-wrong value) -- so it's called on a
        copy, never on `self.__native__` directly, keeping the fallback
        genuinely pristine.
        """
        native = self.__native__
        original_volume = native.Volume
        refined = native.copy().removeSplitter()
        if abs(refined.Volume - original_volume) > 1e-6 * max(abs(original_volume), 1.0):
            return GSolid(native)
        return GSolid(refined)

    def translate(self, vector: GVector) -> "GSolid":
        shape = self.__native__.copy()
        shape.translate(to_fc_vector(vector))
        return GSolid(shape)

    def rotate(self, axis_point: GVector, axis_dir: GVector, angle_rad: float) -> "GSolid":
        shape = self.__native__.copy()
        shape.rotate(to_fc_vector(axis_point), to_fc_vector(axis_dir), math.degrees(angle_rad))
        return GSolid(shape)

    def export_step(self, filename: str) -> None:
        """Export all of this solid's shapes to a single STEP file."""
        self.__native__.exportStep(filename)


# A shape-like argument accepted by generic spatial queries (Gin_contact...).
GShape = GSolid | GFace | GEdge | GShell


# ---------------------------------------------------------------------------
# Result of operations that can fail/degenerate
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SplitResult:
    """
    Result of a Gsplit operation.

    `solids` must never be empty for a valid operation: if a degenerate
    case is detected (tangency, coincident edges...) Gsplit must resolve
    it internally (retry at a different tolerance, fall back to the
    unchanged solid, etc.) and report it via `degenerate_case_handled=True`,
    NEVER silently return nothing.
    """

    solids: list[GSolid]
    degenerate_case_handled: bool = False
    notes: str = ""


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------


def Gload_step(filename: str) -> list[GSolid]:
    """
    Load a STEP file and return the list of top-level solids, with every
    transformation from the file's assembly/placement hierarchy already
    applied (baked into each solid's own geometry) -- callers never need
    to apply a separate placement themselves.
    """
    shape = Part.Shape()
    shape.read(filename)
    return [GSolid(solid) for solid in shape.Solids]


def Gload_step_labels(filename: str) -> list[GLabelNode]:
    """
    Parse the same STEP file's assembly tree and return one `GLabelNode`
    per solid-bearing leaf, in the same order as `Gload_step`'s solids
    (see `GLabelNode`'s docstring for exactly how they line up). This is
    a separate read from `Gload_step`, not a by-product of it: extracting
    labels/hierarchy needs the file's assembly-tree structure (via
    `Import.insert`, which builds FreeCAD's own Part::Feature/Label/InList
    document tree), a different concern from -- and a different reader
    than -- resolving each solid's final, transformed geometry
    (`Part.Shape.read`, used by `Gload_step`, returns geometry only, no
    labels at all).
    """
    doc = FreeCAD.newDocument(uuid.uuid4().hex)
    try:
        Import.insert(filename, doc.Name)

        nodes: dict[str, GLabelNode] = {}

        def build_node(elem) -> GLabelNode:
            if elem.Name in nodes:
                return nodes[elem.Name]
            parent = build_node(elem.InList[0]) if elem.InList else None
            n_solids = 0
            if elem.TypeId == "Part::Feature" and elem.Shape.Solids:
                n_solids = len(elem.Shape.Solids)
            node = GLabelNode(label=elem.Label, parent=parent, n_solids=n_solids)
            nodes[elem.Name] = node
            return node

        return [build_node(elem) for elem in doc.Objects if elem.TypeId == "Part::Feature" and elem.Shape.Solids]
    finally:
        FreeCAD.closeDocument(doc.Name)


def Gexport_step(shapes: list[GShape], filename: str) -> None:
    """Export a list of shapes (any mix of GSolid/GFace/GEdge/GShell) to a single STEP file."""
    compound = Part.makeCompound([shape.__native__ for shape in shapes])
    compound.exportStep(filename)


# ---------------------------------------------------------------------------
# Primitive construction
# (used both by CsgToCad/GEOReverse and by GEOUNED's own meta-surface
# assembly, which builds RoundCorner/Can/TCone/MultiPlane/... by
# cutting/fusing these analytic primitives itself)
# ---------------------------------------------------------------------------


def Gmake_box(xmin: float, ymin: float, zmin: float, xmax: float, ymax: float, zmax: float) -> GSolid:
    box = Part.makeBox(xmax - xmin, ymax - ymin, zmax - zmin, FreeCAD.Vector(xmin, ymin, zmin))
    return GSolid(box)


def Gmake_cylinder(point: GVector, axis: GVector, radius: float, height: float) -> GSolid:
    native = Part.makeCylinder(radius, height, to_fc_vector(point), to_fc_vector(axis))
    return GSolid(native)


def Gmake_cone(apex: GVector, axis: GVector, half_angle: float, height: float) -> GSolid:
    base_radius = height * math.tan(abs(half_angle))
    native = Part.makeCone(0.0, base_radius, height, to_fc_vector(apex), to_fc_vector(axis))
    return GSolid(native)


def Gmake_sphere(center: GVector, radius: float) -> GSolid:
    native = Part.makeSphere(radius, to_fc_vector(center))
    return GSolid(native)


def Gmake_torus(center: GVector, axis: GVector, major_radius: float, minor_radius: float) -> GSolid:
    native = Part.makeTorus(major_radius, minor_radius, to_fc_vector(center), to_fc_vector(axis))
    return GSolid(native)


def Gmake_half_space(plane: GPlane) -> GSolid:
    """Half-space bounded by an infinite plane (internally clipped to a working box). Needed to reconstruct CSG cells defined by the intersection of half-spaces."""
    extent = 1.0e6
    box = Part.makeBox(extent, extent, extent, FreeCAD.Vector(-extent / 2.0, -extent / 2.0, -extent))
    normal = to_fc_vector(plane.Axis.normalized())
    box.Placement = FreeCAD.Placement(
        to_fc_vector(plane.Position),
        FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), normal),
    )
    return GSolid(box)


def Gmake_wire(edges: list[GEdge]) -> GWire:
    """
    Build a wire from an ordered sequence of edges. GEOUNED performs wire
    joining/merging logic itself (e.g. splicing wires that share
    vertices) on top of this primitive; this function does not need to
    know about that higher-level logic.
    """
    native = Part.Wire([edge.__native__ for edge in edges])
    return GWire(native)


def Gmake_polygon_face(points: list[GVector]) -> GFace:
    """
    Build a single planar face bounded by the closed polygon through
    `points`, in order. Used to reconstruct a bounded plane face from the
    points where an infinite plane crosses a bounding box.
    """
    native = Part.Face(Part.makePolygon([to_fc_vector(p) for p in points], True))
    return GFace(native)


def Gmake_shell(faces: list[GFace]) -> GShell:
    """
    Build a shell from a group of faces. Unlike classifying an existing
    solid's topology, this is GEOUNED assembling its own face groupings
    (e.g. faces sharing one analytic surface split by a seam) into a
    single addressable shape, primarily so Gin_contact/Gdistance can be
    called on the group as a whole.
    """
    native = Part.makeShell([face.__native__ for face in faces])
    return GShell(native, faces=faces)


def Gmake_compound(shapes: list[GSolid]) -> GSolid:
    """Group `shapes` into a single compound shape, with no boolean operation applied (they may overlap or be disjoint). Used as a last-resort fallback when Gfuse fails or produces an invalid result."""
    return GSolid(Part.makeCompound([s.__native__ for s in shapes]))


# ---------------------------------------------------------------------------
# Boolean / split operations
# (free functions rather than methods since they combine two-or-more
# independent shapes -- there is no single natural "self")
# ---------------------------------------------------------------------------


def Gcut(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    """Subtract `tools` from `solid`. May return >1 solid if fragmented."""
    result = solid.__native__.cut([tool.__native__ for tool in tools])
    return [GSolid(s) for s in result.Solids]


def Gcommon(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    """Boolean intersection."""
    result = solid.__native__.common([tool.__native__ for tool in tools])
    return [GSolid(s) for s in result.Solids]


def Gfuse(solids: list[GSolid]) -> GSolid:
    """Boolean union."""
    shapes = []
    for gsolid in solids:
        if type(gsolid.__native__) is Part.Compound:
            shapes.extend(gsolid.__native__.Solids)
        else:
            shapes.append(gsolid.__native__)
    fused = shapes[0].fuse(shapes[1:]) if len(shapes) > 1 else shapes[0]
    return GSolid(fused)


def Gsplit(
    base: GSolid,
    tool: GShape,
    tolerance: float,
    scale: float = 0.1,
    scale_up_floor: float | None = None,
) -> SplitResult:
    """
    Cut `base` with a surface/solid `tool` (typically a plane) and return
    ALL resulting fragments. Replaces `BOPTools.SplitAPI.slice` at every
    GEOUNED call site: this function is responsible for internally
    resolving degenerate cases (tangencies, tool not intersecting the
    solid, kernel exceptions at small tolerances) -- callers must NOT
    implement their own retry/offset logic on top of this.

    `scale_up_floor` mirrors GEOUNED's public `Options.scaleUp`/
    `Options.splitTolerance`: when `tolerance` drops below 1e-12 and
    `scale_up_floor` is given, retry upward starting from that floor
    instead of just attempting the tiny tolerance as-is. Below 1e-12 with
    no floor, and at `tolerance >= 0.1`, there is no retry at all --
    those are the two cases where shrinking further is not expected to
    help.
    """
    tools = [tool.__native__]

    if tolerance >= 0.1:
        compound = BOPTools.SplitAPI.slice(base.__native__, tools, "Split", tolerance=tolerance)
    elif tolerance < 1e-12:
        if scale_up_floor is not None:
            floor = 1e-13 if scale_up_floor == 0 else scale_up_floor
            return Gsplit(base, tool, floor / scale, scale=1.0 / scale, scale_up_floor=scale_up_floor)
        compound = BOPTools.SplitAPI.slice(base.__native__, tools, "Split", tolerance=tolerance)
    else:
        try:
            compound = BOPTools.SplitAPI.slice(base.__native__, tools, "Split", tolerance=tolerance)
        except Exception:
            retried = Gsplit(base, tool, tolerance * scale, scale, scale_up_floor)
            return SplitResult(
                solids=retried.solids,
                degenerate_case_handled=True,
                notes=f"retried at tolerance={tolerance * scale}",
            )

    if not compound.Solids:
        # tool doesn't intersect solid at all (e.g. a cutting plane
        # entirely outside the solid's extent) -- slice() reports this as
        # an empty compound rather than raising. Not a fragmentation, so
        # fall back to the solid unchanged instead of reporting "no
        # solids".
        return SplitResult(
            solids=[base],
            degenerate_case_handled=True,
            notes="tool did not intersect solid; returning it unchanged",
        )
    return SplitResult(solids=[GSolid(s) for s in compound.Solids])


# ---------------------------------------------------------------------------
# Spatial queries between two independent shapes
# ---------------------------------------------------------------------------


def Gin_contact(shape_a: GShape, shape_b: GShape, tolerance: float) -> bool:
    """
    True if shape_a and shape_b share at least one point in the
    volumetric sense (touching or overlapping within tolerance; e.g. two
    concentric spherical shells count as in contact). Accepts any
    combination of GSolid/GFace/GEdge/GShell.

    GEOUNED never needs the actual distance value, only this boolean --
    so this function owns all robustness workarounds internally
    (bounding-box pre-filtering, boolean-common fallback, degenerate/
    slow-kernel cases) rather than each call site reimplementing them.
    """
    native_a = shape_a.__native__
    native_b = shape_b.__native__

    box_intersection = native_a.BoundBox.intersected(native_b.BoundBox)
    if not (
        box_intersection.XLength > -tolerance
        and box_intersection.YLength > -tolerance
        and box_intersection.ZLength > -tolerance
    ):
        return False

    try:
        return native_a.distToShape(native_b)[0] < tolerance
    except Exception:
        pass

    if hasattr(native_a, "Volume") and hasattr(native_b, "Volume"):
        common = native_a.common(native_b)
        return abs(common.Volume) > 1e-8 or bool(common.Solids) or bool(common.Faces) or bool(common.Edges)
    return False


def Gdistance(shape_a: GShape, shape_b: GShape) -> float:
    """
    Minimum distance between shape_a and shape_b (0.0 if touching or
    overlapping). Unlike `Gin_contact`, this is a thin wrapper with no
    extra robustness layer -- use `Gin_contact` instead wherever only the
    boolean is actually needed.
    """
    return shape_a.__native__.distToShape(shape_b.__native__)[0]
