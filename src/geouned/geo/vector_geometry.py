"""
geo/vector_geometry.py

Pure-math foundation of the `geo` package: no `Part`/`FreeCAD`/`OCC.Core`
import anywhere in this file, and no dependency on any other file in this
package either. `GVector`/`GMatrix`/`GBoundBox` are plain, neutral data
types -- this is what a future backend would import unchanged, and what
every other file in `geo` (`surface_geometry.py`, `solid_defects.py`, the
3 `_*_impl.py` backends) builds on top of.

Sibling files, split out of what used to be one large `vector_geometry.py`
(2026-08-28, per direct user request -- the original file had grown to
mix several unrelated layers):
  - `surface_geometry.py` -- geometric predicates on the analytic surface
    descriptors (`GPlane`/`GCylinder`/`GCone`/`GSphere`/`GTorus`), operating
    only on `GVector` arithmetic and those descriptors' fields, never on a
    native shape. This is what was originally meant by "the geometric
    predicates created at the start of this migration."
  - `solid_defects.py` -- load-time CAD-defect detection/repair-support on
    a whole `GSolid` (short edges, split-ring faces, near-coincident
    surface pairs) -- everything that runs when a solid is first read from
    a STEP file, before any GEOUNED-level classification happens.
  - `io_utils.py` -- process-level/loading utilities unrelated to vector
    math: `suppress_native_stdout`, and `GLabelNode` (the STEP assembly/
    label-tree node type `Gload_step_labels` returns -- moved here rather
    than kept alongside `GVector`/`GBoundBox`, since it's STEP-loading
    data, not a geometric quantity).
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

    A11: float
    A12: float
    A13: float
    A14: float
    A21: float
    A22: float
    A23: float
    A24: float
    A31: float
    A32: float
    A33: float
    A34: float
    A41: float
    A42: float
    A43: float
    A44: float


def to_gmatrix(matrix) -> GMatrix:
    """Convert a native FreeCAD.Matrix (or anything exposing the same A11..A44 attributes) into a neutral GMatrix."""
    return GMatrix(
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
            self.XMin <= other.XMax
            and self.XMax >= other.XMin
            and self.YMin <= other.YMax
            and self.YMax >= other.YMin
            and self.ZMin <= other.ZMax
            and self.ZMax >= other.ZMin
        )

    def enlarged(self, d: float) -> "GBoundBox":
        """A new box expanded by `d` on every side (matches FreeCAD's `BoundBox.enlarge`, but non-mutating)."""
        return GBoundBox(
            self.XMin - d,
            self.YMin - d,
            self.ZMin - d,
            self.XMax + d,
            self.YMax + d,
            self.ZMax + d,
        )

    def union(self, other: "GBoundBox") -> "GBoundBox":
        """Smallest box containing both `self` and `other` (matches FreeCAD's `BoundBox.add`, but non-mutating)."""
        return GBoundBox(
            min(self.XMin, other.XMin),
            min(self.YMin, other.YMin),
            min(self.ZMin, other.ZMin),
            max(self.XMax, other.XMax),
            max(self.YMax, other.YMax),
            max(self.ZMax, other.ZMax),
        )

    def intersected(self, other: "GBoundBox") -> "GBoundBox":
        """
        Componentwise overlap with `other` (matches FreeCAD's own
        `BoundBox.intersected`). May come back invalid -- check
        `is_valid()` -- if the two boxes don't actually overlap on some
        axis; this is not an error, callers are expected to check.
        """
        return GBoundBox(
            max(self.XMin, other.XMin),
            max(self.YMin, other.YMin),
            max(self.ZMin, other.ZMin),
            min(self.XMax, other.XMax),
            min(self.YMax, other.YMax),
            min(self.ZMax, other.ZMax),
        )

    def contains_point(self, point: GVector, tolerance: float = 0.0) -> bool:
        """True if `point` lies inside or on the boundary of the box (matches FreeCAD's own `BoundBox.isInside`)."""
        return (
            self.XMin - tolerance <= point.x <= self.XMax + tolerance
            and self.YMin - tolerance <= point.y <= self.YMax + tolerance
            and self.ZMin - tolerance <= point.z <= self.ZMax + tolerance
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

    def transformed(self, matrix) -> "GBoundBox":
        """
        New box enclosing this one after applying an affine `matrix`
        (matches FreeCAD's own `BoundBox.transformed`, but non-mutating) --
        since a general affine transform can rotate an axis-aligned box off
        -axis, this transforms all 8 corners and returns their own new
        axis-aligned bounding box, not a simple componentwise remap.

        Duck-typed on `.A11`..`.A44` (row-major affine, translation in the
        4th column: `x' = A11*x + A12*y + A13*z + A14`, etc.) -- works with
        either a native `FreeCAD.Matrix` or a `GMatrix`, same as
        `to_gmatrix`'s own tolerance for either.
        """
        corners = [_affine_transform_point(matrix, self.get_point(i)) for i in range(8)]
        xs = [c.x for c in corners]
        ys = [c.y for c in corners]
        zs = [c.z for c in corners]
        return GBoundBox(min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def _affine_transform_point(matrix, point: GVector) -> GVector:
    return GVector(
        matrix.A11 * point.x + matrix.A12 * point.y + matrix.A13 * point.z + matrix.A14,
        matrix.A21 * point.x + matrix.A22 * point.y + matrix.A23 * point.z + matrix.A24,
        matrix.A31 * point.x + matrix.A32 * point.y + matrix.A33 * point.z + matrix.A34,
    )


def to_gboundbox(box) -> GBoundBox:
    """Convert any box-like object exposing `.XMin`/.../`.ZMax` (e.g. a native `FreeCAD.BoundBox`) into a neutral GBoundBox."""
    return GBoundBox(box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)
