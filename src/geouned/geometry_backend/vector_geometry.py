"""
geometry_backend/vector_geometry.py

Geometric predicates built purely on `GVector` arithmetic. Backend-
agnostic: these never touch a `GeometryBackend`, `Part`, `FreeCAD`, or
`OCC.Core` object. This is the layer above the adapter interface where
GEOUNED's own reasoning about parallelism, colinearity, and plane
membership lives (previously scattered across `basic_functions_part1.py`
as free functions taking FreeCAD.Vector directly).
"""

from __future__ import annotations

import math

from .geometry_backend_interface import GBoundBox, GCone, GCylinder, GPlane, GSphere, GTorus, GVector


def to_gvector(vector) -> GVector:
    """
    Convert any vector-like object exposing `.x`/`.y`/`.z` into a neutral
    GVector. Read-side half of a transitional pair with a backend's
    `to_fc_vector`/`to_occ_vector`: convert to GVector as soon as a
    vector is read from a native object, operate on GVector thereafter,
    convert back only right before a call still needing a native arg.
    """
    return GVector(vector.x, vector.y, vector.z)


def to_gboundbox(box) -> GBoundBox:
    """Convert any box-like object exposing `.XMin`/.../`.ZMax` (e.g. a native `FreeCAD.BoundBox`) into a neutral GBoundBox."""
    return GBoundBox(box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)


def is_same_value(v1: float, v2: float, tolerance: float = 1e-6) -> bool:
    return abs(v1 - v2) < tolerance


def is_opposite(vector_1: GVector, vector_2: GVector, tolerance: float = 1e-3) -> bool:
    return vector_1.angle_to(-vector_2) < tolerance


def is_parallel(vector_1: GVector, vector_2: GVector, tolerance: float = 1e-3) -> bool:
    angle = vector_1.angle_to(vector_2)
    return angle < tolerance or is_same_value(angle, math.pi, tolerance)


def is_in_line(point: GVector, direction: GVector, point_on_line: GVector, tolerance: float = 1e-6) -> bool:
    to_point = point - point_on_line
    return is_parallel(direction, to_point) or to_point.length < tolerance


def is_in_plane(point: GVector, plane: GPlane, tolerance: float = 1e-7) -> bool:
    return abs(plane.Axis.dot(point - plane.Position)) < tolerance


def sign_plane(point: GVector, plane: GPlane) -> int:
    return 1 if plane.Axis.dot(point - plane.Position) >= 0.0 else -1


def is_same_plane_surface(plane_1: GPlane, plane_2: GPlane) -> bool:
    """
    True if two planes are the same infinite analytic plane (same axis
    direction -- either way -- and same offset from the origin). Direct
    port of the former `PlaneGu.isSameSurface`, kept at the same fixed
    tolerances: this is a decomposition-time "is this literally the same
    underlying surface" check, distinct from `is_same_plane` in
    `basic_functions_part2.py`, which compares already-built output
    surfaces with user-configurable tolerances for a different purpose
    (surface-list deduplication).
    """
    if abs(plane_1.Axis.dot(plane_2.Axis)) < 0.99999:
        return False
    return abs(plane_1.Axis.dot(plane_1.Position) - plane_2.Axis.dot(plane_2.Position)) <= 1e-5


def is_parallel_plane_surface(plane_1: GPlane, plane_2: GPlane) -> bool:
    """Direct port of the former `PlaneGu.isParallel`, same fixed tolerance."""
    return abs(plane_1.Axis.dot(plane_2.Axis)) > 0.99999


def is_same_cylinder_surface(cylinder_1: GCylinder, cylinder_2: GCylinder) -> bool:
    """Direct port of the former `CylinderGu.isSameSurface`, same fixed tolerances."""
    if abs(cylinder_1.Radius - cylinder_2.Radius) > 1e-5:
        return False
    if (cylinder_1.Center - cylinder_2.Center).length > 1e-5:
        return False
    return abs(cylinder_1.Axis.dot(cylinder_2.Axis)) >= 0.99999


def is_same_cone_surface(cone_1: GCone, cone_2: GCone) -> bool:
    """Direct port of the former `ConeGu.isSameSurface`, same fixed tolerances."""
    if abs(cone_1.SemiAngle - cone_2.SemiAngle) > 1e-5:
        return False
    if (cone_1.Apex - cone_2.Apex).length > 1e-5:
        return False
    return abs(cone_1.Axis.dot(cone_2.Axis)) >= 0.99999


def is_same_sphere_surface(sphere_1: GSphere, sphere_2: GSphere) -> bool:
    """Direct port of the former `SphereGu.isSameSurface`, same fixed tolerance."""
    if abs(sphere_1.Radius - sphere_2.Radius) > 1e-5:
        return False
    return (sphere_1.Center - sphere_2.Center).length <= 1e-5


def is_same_torus_surface(torus_1: GTorus, torus_2: GTorus) -> bool:
    """Direct port of the former `TorusGu.isSameSurface`, same fixed tolerances."""
    if abs(torus_1.MajorRadius - torus_2.MajorRadius) > 1e-5:
        return False
    if abs(torus_1.MinorRadius - torus_2.MinorRadius) > 1e-5:
        return False
    if (torus_1.Center - torus_2.Center).length > 1e-5:
        return False
    return abs(torus_1.Axis.dot(torus_2.Axis)) >= 0.99999


def _require_x_dir(surface: GPlane | GCylinder) -> GVector:
    if surface.XDir is None:
        raise ValueError(
            f"{type(surface).__name__}.XDir is required to evaluate value_at/tangent_at analytically "
            "(this instance was reconstructed without a native face, so its (u, v) reference "
            "direction is unknown)"
        )
    return surface.XDir


def plane_value_at(plane: GPlane, u: float, v: float) -> GVector:
    """
    Point on the infinite plane at parametric coordinates (u, v), matching
    FreeCAD/OCCT's own Plane parametrization exactly (Position + u*XDir +
    v*YDir, YDir = Axis x XDir) -- not just a geometrically-equivalent
    plane with an arbitrary (u, v) origin/orientation.
    """
    x_dir = _require_x_dir(plane)
    y_dir = plane.Axis.cross(x_dir)
    return plane.Position + x_dir * u + y_dir * v


def plane_tangent_at(plane: GPlane, u: float, v: float) -> tuple[GVector, GVector]:
    """Unit tangent directions (d/du, d/dv) on the plane -- constant everywhere, same convention as `plane_value_at`."""
    x_dir = _require_x_dir(plane)
    y_dir = plane.Axis.cross(x_dir)
    return x_dir, y_dir


def cylinder_value_at(cylinder: GCylinder, u: float, v: float) -> GVector:
    """
    Point on the infinite cylinder at parametric coordinates (u, v),
    matching FreeCAD/OCCT's own Cylinder parametrization exactly (u is the
    angle from XDir toward YDir = Axis x XDir, v is the distance along
    Axis) -- not just a geometrically-equivalent cylinder with an
    arbitrary angular origin.
    """
    x_dir = _require_x_dir(cylinder)
    y_dir = cylinder.Axis.cross(x_dir)
    radial = x_dir * math.cos(u) + y_dir * math.sin(u)
    return cylinder.Center + radial * cylinder.Radius + cylinder.Axis * v


def cylinder_tangent_at(cylinder: GCylinder, u: float, v: float) -> tuple[GVector, GVector]:
    """Unit tangent directions (d/du, d/dv) on the cylinder, same convention as `cylinder_value_at`."""
    x_dir = _require_x_dir(cylinder)
    y_dir = cylinder.Axis.cross(x_dir)
    tangent_u = y_dir * math.cos(u) - x_dir * math.sin(u)
    return tangent_u, cylinder.Axis
