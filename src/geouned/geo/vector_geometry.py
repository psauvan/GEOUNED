"""
geo/vector_geometry.py

Pure-math foundation of the `geo` package: no `Part`/`FreeCAD`/`OCC.Core`
import anywhere in this file. `GVector`/`GBoundBox`/`GLabelNode` are plain
data, and the predicates below (is_same_plane_surface, is_parallel...)
operate only on GVector arithmetic and the GPlane/GCylinder/... surface
descriptors' fields -- never on a native shape. This is what a future
`_occ_impl.py` would import unchanged, alongside `_freecad_impl.py`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Neutral vector type
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GVector:
    """Neutral vector/point. Avoids passing FreeCAD.Vector or gp_Pnt directly."""
    x: float
    y: float
    z: float

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __add__(self, other: "GVector") -> "GVector":
        return GVector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "GVector") -> "GVector":
        return GVector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "GVector":
        return GVector(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> "GVector":
        return GVector(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> "GVector":
        return GVector(-self.x, -self.y, -self.z)

    def __getitem__(self, index: int) -> float:
        return (self.x, self.y, self.z)[index]

    def dot(self, other: "GVector") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "GVector") -> "GVector":
        return GVector(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    @property
    def length(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> "GVector":
        length = self.length
        return GVector(self.x / length, self.y / length, self.z / length)

    def is_equal(self, other: "GVector", tolerance: float = 1e-6) -> bool:
        return (self - other).length < tolerance

    def angle_to(self, other: "GVector") -> float:
        """Angle in radians, in [0, pi]. Robust near 0 and pi (atan2-based)."""
        return math.atan2(self.cross(other).length, self.dot(other))


def to_gvector(vector) -> GVector:
    """Convert any vector-like object exposing `.x`/`.y`/`.z` (e.g. a native FreeCAD.Vector) into a neutral GVector."""
    return GVector(vector.x, vector.y, vector.z)


# ---------------------------------------------------------------------------
# Neutral 4x4 affine matrix
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GMatrix:
    """Neutral 4x4 affine matrix. Field names match FreeCAD's own `Base.Matrix`
    attribute names (A11..A44). Used e.g. for `GEdge`/`GWire.MatrixOfInertia`
    (only the A11..A33 rotation/inertia sub-block is meaningful there, but the
    full 4x4 is stored for fidelity with the native type)."""
    A11: float; A12: float; A13: float; A14: float
    A21: float; A22: float; A23: float; A24: float
    A31: float; A32: float; A33: float; A34: float
    A41: float; A42: float; A43: float; A44: float


def to_gmatrix(matrix) -> GMatrix:
    """Convert a native FreeCAD.Matrix (or anything exposing the same A11..A44 attributes) into a neutral GMatrix."""
    return GMatrix(
        matrix.A11, matrix.A12, matrix.A13, matrix.A14,
        matrix.A21, matrix.A22, matrix.A23, matrix.A24,
        matrix.A31, matrix.A32, matrix.A33, matrix.A34,
        matrix.A41, matrix.A42, matrix.A43, matrix.A44,
    )


# ---------------------------------------------------------------------------
# Neutral axis-aligned bounding box
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GBoundBox:
    """Field names match FreeCAD's own `Base.BoundBox` attribute names."""
    XMin: float
    YMin: float
    ZMin: float
    XMax: float
    YMax: float
    ZMax: float

    @property
    def XLength(self) -> float:
        return self.XMax - self.XMin

    @property
    def YLength(self) -> float:
        return self.YMax - self.YMin

    @property
    def ZLength(self) -> float:
        return self.ZMax - self.ZMin

    @property
    def Center(self) -> GVector:
        return GVector(
            0.5 * (self.XMin + self.XMax),
            0.5 * (self.YMin + self.YMax),
            0.5 * (self.ZMin + self.ZMax),
        )

    @property
    def DiagonalLength(self) -> float:
        return math.sqrt(self.XLength**2 + self.YLength**2 + self.ZLength**2)

    def is_valid(self) -> bool:
        return self.XMin <= self.XMax and self.YMin <= self.YMax and self.ZMin <= self.ZMax

    def intersects(self, other: "GBoundBox") -> bool:
        """True if the two boxes overlap or touch (inclusive at the boundary), matching FreeCAD's own `BoundBox.intersect`."""
        return (
            self.XMin <= other.XMax and self.XMax >= other.XMin
            and self.YMin <= other.YMax and self.YMax >= other.YMin
            and self.ZMin <= other.ZMax and self.ZMax >= other.ZMin
        )

    def enlarged(self, d: float) -> "GBoundBox":
        """A new box expanded by `d` on every side (matches FreeCAD's `BoundBox.enlarge`, but non-mutating)."""
        return GBoundBox(
            self.XMin - d, self.YMin - d, self.ZMin - d,
            self.XMax + d, self.YMax + d, self.ZMax + d,
        )

    def union(self, other: "GBoundBox") -> "GBoundBox":
        """Smallest box containing both `self` and `other` (matches FreeCAD's `BoundBox.add`, but non-mutating)."""
        return GBoundBox(
            min(self.XMin, other.XMin), min(self.YMin, other.YMin), min(self.ZMin, other.ZMin),
            max(self.XMax, other.XMax), max(self.YMax, other.YMax), max(self.ZMax, other.ZMax),
        )

    def intersected(self, other: "GBoundBox") -> "GBoundBox":
        """
        Componentwise overlap with `other` (matches FreeCAD's own
        `BoundBox.intersected`). May come back invalid -- check
        `is_valid()` -- if the two boxes don't actually overlap on some
        axis; this is not an error, callers are expected to check.
        """
        return GBoundBox(
            max(self.XMin, other.XMin), max(self.YMin, other.YMin), max(self.ZMin, other.ZMin),
            min(self.XMax, other.XMax), min(self.YMax, other.YMax), min(self.ZMax, other.ZMax),
        )

    def get_point(self, i: int) -> GVector:
        """
        Corner `i` (0-7) of the box. Numbering matches FreeCAD's own
        `BoundBox.getPoint` exactly (verified empirically): 0-3 are the
        ZMax face going around, 4-7 the ZMin face in the same order.
        """
        points = (
            (self.XMin, self.YMin, self.ZMax),
            (self.XMax, self.YMin, self.ZMax),
            (self.XMax, self.YMax, self.ZMax),
            (self.XMin, self.YMax, self.ZMax),
            (self.XMin, self.YMin, self.ZMin),
            (self.XMax, self.YMin, self.ZMin),
            (self.XMax, self.YMax, self.ZMin),
            (self.XMin, self.YMax, self.ZMin),
        )
        return GVector(*points[i])

    def get_edge(self, i: int) -> tuple[GVector, GVector]:
        """Edge `i` (0-11) of the box, as its two endpoints. Numbering matches FreeCAD's own `BoundBox.getEdge`."""
        if i < 4:
            return self.get_point(i), self.get_point((i + 1) % 4)
        if i < 8:
            j = i - 4
            return self.get_point(4 + j), self.get_point(4 + (j + 1) % 4)
        j = i - 8
        return self.get_point(j), self.get_point(j + 4)


def to_gboundbox(box) -> GBoundBox:
    """Convert any box-like object exposing `.XMin`/.../`.ZMax` (e.g. a native `FreeCAD.BoundBox`) into a neutral GBoundBox."""
    return GBoundBox(box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)


# ---------------------------------------------------------------------------
# STEP assembly-tree label node (pure data, no native reference)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GLabelNode:
    """
    One node of a STEP file's assembly/label tree, as read by
    `Gload_step_labels`. Only solid-bearing leaves are returned in that
    list, but `parent` still walks up through every ancestor (including
    non-solid-bearing group/assembly nodes) so callers can reconstruct a
    full label path, exactly as GEOUNED's own comment/material/dilution/
    enclosure parsing needs.

    Positionally aligned with `Gload_step`: node[i]'s `n_solids` solids
    are `Gload_step(...)`'s next `n_solids` entries, in order, once all
    nodes up to `i` have been consumed.
    """
    label: str
    parent: "GLabelNode | None"
    n_solids: int


# ---------------------------------------------------------------------------
# Geometric predicates on GVector / surface descriptors (GPlane, GCylinder...)
# ---------------------------------------------------------------------------

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


def is_in_plane(point: GVector, plane, tolerance: float = 1e-7) -> bool:
    return abs(plane.Axis.dot(point - plane.Position)) < tolerance


def sign_plane(point: GVector, plane) -> int:
    return 1 if plane.Axis.dot(point - plane.Position) >= 0.0 else -1


def is_same_plane_surface(plane_1, plane_2) -> bool:
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


def is_parallel_plane_surface(plane_1, plane_2) -> bool:
    """Direct port of the former `PlaneGu.isParallel`, same fixed tolerance."""
    return abs(plane_1.Axis.dot(plane_2.Axis)) > 0.99999


def is_same_cylinder_surface(cylinder_1, cylinder_2) -> bool:
    """Direct port of the former `CylinderGu.isSameSurface`, same fixed tolerances."""
    if abs(cylinder_1.Radius - cylinder_2.Radius) > 1e-5:
        return False
    if (cylinder_1.Center - cylinder_2.Center).length > 1e-5:
        return False
    return abs(cylinder_1.Axis.dot(cylinder_2.Axis)) >= 0.99999


def is_same_cone_surface(cone_1, cone_2) -> bool:
    """Direct port of the former `ConeGu.isSameSurface`, same fixed tolerances."""
    if abs(cone_1.SemiAngle - cone_2.SemiAngle) > 1e-5:
        return False
    if (cone_1.Apex - cone_2.Apex).length > 1e-5:
        return False
    return abs(cone_1.Axis.dot(cone_2.Axis)) >= 0.99999


def is_same_sphere_surface(sphere_1, sphere_2) -> bool:
    """Direct port of the former `SphereGu.isSameSurface`, same fixed tolerance."""
    if abs(sphere_1.Radius - sphere_2.Radius) > 1e-5:
        return False
    return (sphere_1.Center - sphere_2.Center).length <= 1e-5


def is_same_torus_surface(torus_1, torus_2) -> bool:
    """Direct port of the former `TorusGu.isSameSurface`, same fixed tolerances."""
    if abs(torus_1.MajorRadius - torus_2.MajorRadius) > 1e-5:
        return False
    if abs(torus_1.MinorRadius - torus_2.MinorRadius) > 1e-5:
        return False
    if (torus_1.Center - torus_2.Center).length > 1e-5:
        return False
    return abs(torus_1.Axis.dot(torus_2.Axis)) >= 0.99999


def _require_x_dir(surface) -> GVector:
    if surface.XDir is None:
        raise ValueError(
            f"{type(surface).__name__}.XDir is required to evaluate value_at/tangent_at analytically "
            "(this instance was reconstructed without a native face, so its (u, v) reference "
            "direction is unknown)"
        )
    return surface.XDir


def plane_value_at(plane, u: float, v: float) -> GVector:
    """
    Point on the infinite plane at parametric coordinates (u, v), matching
    FreeCAD/OCCT's own Plane parametrization exactly (Position + u*XDir +
    v*YDir, YDir = Axis x XDir) -- not just a geometrically-equivalent
    plane with an arbitrary (u, v) origin/orientation.
    """
    x_dir = _require_x_dir(plane)
    y_dir = plane.Axis.cross(x_dir)
    return plane.Position + x_dir * u + y_dir * v


def plane_tangent_at(plane, u: float, v: float) -> tuple[GVector, GVector]:
    """Unit tangent directions (d/du, d/dv) on the plane -- constant everywhere, same convention as `plane_value_at`."""
    x_dir = _require_x_dir(plane)
    y_dir = plane.Axis.cross(x_dir)
    return x_dir, y_dir


def cylinder_value_at(cylinder, u: float, v: float) -> GVector:
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


def cylinder_tangent_at(cylinder, u: float, v: float) -> tuple[GVector, GVector]:
    """Unit tangent directions (d/du, d/dv) on the cylinder, same convention as `cylinder_value_at`."""
    x_dir = _require_x_dir(cylinder)
    y_dir = cylinder.Axis.cross(x_dir)
    tangent_u = y_dir * math.cos(u) - x_dir * math.sin(u)
    return tangent_u, cylinder.Axis
