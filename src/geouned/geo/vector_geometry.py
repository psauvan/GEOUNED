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

import contextlib
import math
import os
import sys
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

    Each plane's own offset (`Axis.dot(Position)`) is measured along its
    *own* axis -- when the two axes are antiparallel (opposite direction,
    still the same infinite plane, e.g. the same real plane reached via
    two different Face Orientations), those two offsets are measured in
    opposite directions and must be compared via `d1 == -d2`, not
    `d1 == d2` (confirmed as a real bug via `Solidos/test_models/
    RoundCorners/rc9.stp`, 2026-08-23: two genuinely different, parallel
    planes 3.5 units apart with antiparallel axes have equal-magnitude
    offsets of the same sign, which `d1 == d2` alone wrongly matched --
    `MetaSurfacesDict`/`build_roundC_params` then treated one cylinder's
    own 2 distinct bounding planes as a single coincident one, losing its
    real additional/closing plane entirely).
    """
    axis_dot = plane_1.Axis.dot(plane_2.Axis)
    if abs(axis_dot) < 0.99999:
        return False
    d1 = plane_1.Axis.dot(plane_1.Position)
    d2 = plane_2.Axis.dot(plane_2.Position)
    if axis_dot > 0:
        return abs(d1 - d2) <= 1e-5
    else:
        return abs(d1 + d2) <= 1e-5


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


def is_coaxial_cone_pair(
    cone_1, cone_2, angle_tol: float = 1e-6, axis_dot_tol: float = 1e-5, apex_line_tol: float = 1e-5
) -> bool:
    """True when two cones share the same axis *line* (their axis vectors
    may be parallel or anti-parallel -- direction is not constrained) and
    the same `SemiAngle`, but sit at *different* apexes along that line.

    This is the exact "two coaxial cones pointing toward or away from each
    other" configuration whose analytic intersection degenerates into a
    single circle instead of a generic space curve, which is what makes it
    a hard case for a general quadric-quadric BOP solver (see `Gsplit`'s
    coaxial-cone fallback in `_occ_impl.py`/`_ocp_impl.py`).

    The complement of `is_same_cone_surface` (which additionally requires
    a matching apex *and* matching axis direction, i.e. literally the same
    infinite cone): this predicate explicitly requires the apex to differ
    and tolerates either axis direction along the shared line.
    """
    if abs(cone_1.SemiAngle - cone_2.SemiAngle) > angle_tol:
        return False
    if abs(cone_1.Axis.dot(cone_2.Axis)) < 1.0 - axis_dot_tol:
        return False
    apex_offset = cone_2.Apex - cone_1.Apex
    if apex_offset.length < apex_line_tol:
        return False  # same apex -> the same cone entirely, not a pair
    along = apex_offset.dot(cone_1.Axis)
    radial = (apex_offset - cone_1.Axis * along).length
    return radial < apex_line_tol


def is_coaxial_cone_cylinder_pair(cone, cylinder, radial_tol: float = 1e-5, semiangle_min: float = 1e-6) -> bool:
    """True when `cone` and `cylinder` share the same axis *line* (either
    axis direction) and `cone`'s SemiAngle is non-degenerate (not exactly
    0 or 90 degrees), so the cone genuinely reaches `cylinder.Radius` at
    exactly one height along its own axis -- an analytic certainty for
    any coaxial cone/cylinder pair, not a coincidence.

    This is the cone/cylinder counterpart of `is_coaxial_cone_pair`: when
    the real solid's own boundary is also *tangent* along that one circle
    (e.g. a cone built to blend smoothly into a cylinder of the same
    radius), the circle is the same kind of degenerate quadric-quadric
    intersection a general BOP solver struggles with -- confirmed live on
    a real fixture where a cutting cone, a cylinder, and a second cone all
    shared one exact circle simultaneously (`Gsplit`'s coaxial-cone
    fallback in `_occ_impl.py`/`_ocp_impl.py` only searched for a second
    *cone*, missing this case entirely).
    """
    if abs(math.tan(cone.SemiAngle)) < semiangle_min:
        return False
    if abs(cone.Axis.dot(cylinder.Axis)) < 1.0 - 1e-5:
        return False
    offset = cylinder.Center - cone.Apex
    along = offset.dot(cone.Axis)
    radial = (offset - cone.Axis * along).length
    return radial < radial_tol


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


# ---------------------------------------------------------------------------
# Point-classification predicates ("is `point` outside this analytic
# surface"). Free functions, duck-typed on .Axis/.Position/.Center/etc,
# so they work identically whether called on a `geo` descriptor (GPlane,
# GCylinder, ...) or on GEOUNED's own Tier-1 *OnlyParams (PlaneParams,
# CylinderOnlyParams, ...) -- both store the same fields under the same
# names since the Tier-1 GVector-storage migration. This is what lets
# `boolean_solids.check_sign_primitive` (a decomposition-time hot path,
# operating on *OnlyParams) share these formulas with `GPlane.is_inside`
# etc instead of duplicating them, with no transient object construction
# on either side.
# ---------------------------------------------------------------------------


def is_inside_plane(point: GVector, plane) -> bool:
    return plane.Axis.dot(point - plane.Position) > 0


def is_inside_cylinder(point: GVector, cylinder) -> bool:
    r = point - cylinder.Center
    z = cylinder.Axis.dot(r)
    return (r.length * r.length - z * z) > cylinder.Radius * cylinder.Radius


def is_inside_cone(point: GVector, cone) -> bool:
    r = (point - cone.Apex).normalized()
    # rounded to 15 decimals before acos: guards against a just-past-1.0
    # float rounding error (acos would otherwise raise on a point exactly
    # on the axis).
    z = round(cone.Axis.dot(r), 15)
    alpha = math.acos(z)
    return alpha > cone.SemiAngle


def is_inside_sphere(point: GVector, sphere) -> bool:
    return (point - sphere.Center).length > sphere.Radius


def is_inside_torus(point: GVector, torus) -> bool:
    r = point - torus.Center
    h = r.dot(torus.Axis)
    rho = r - h * torus.Axis
    rp = math.sqrt((rho.length - torus.MajorRadius) ** 2 + h**2)
    return rp > torus.MinorRadius


def torus_sheet_sign(vertex: GVector, torus, tol: float = 1e-8) -> int:
    """For a self-intersecting (degenerate: MinorRadius > MajorRadius)
    torus, +1 if `vertex` lies on the ordinary outer sheet, -1 if on the
    pinched, self-intersecting inner sheet -- these are two genuinely
    distinct subsets of the point set satisfying the torus's own implicit
    equation (folded through the axis where the naive tube radius would
    go negative), not just two ways of naming the same surface. Always +1
    for a non-degenerate torus, where the two sheets coincide.

    Derivation: `radial` is the true (always non-negative) radial
    direction of `vertex` itself, read directly off its real 3D position
    -- not derived from the torus's own (u, v) parametrization, which is
    exactly what folds over and becomes ambiguous on the inner sheet.
    `radial * MajorRadius` is therefore the point on the torus's own
    major (central) circle at `vertex`'s true azimuth; the true tube
    radius at that azimuth is the distance from there to `vertex`. On the
    outer sheet this distance is always MinorRadius (matching the
    standard parametrization); on the inner sheet it isn't, since the
    inner sheet's parametrization has its major-circle offset applied in
    the *opposite* azimuthal direction from where the point actually
    sits.
    """
    r = vertex - torus.Center
    outer = r.length > math.sqrt(torus.MinorRadius**2 - torus.MajorRadius**2)
    return 1 if outer else -1


MIN_SLIVER_EDGE_LENGTH = 1.0e-3
"""Absolute floor (mm) for find_short_edges' own threshold -- per direct
user instruction: the effective threshold must never drop below this,
even for a solid whose own BoundBox diagonal is small enough that
`rel_tol * diagonal` alone would push it below the model's own working
geometric tolerance (e.g. a tiny decomposed piece), which would make the
detector unable to catch even a genuinely near-zero-length degenerate
edge there."""

DEGENERATE_EDGE_LENGTH_FLOOR = 1.0e-9
"""Lower floor (mm): an edge shorter than this is treated as a
legitimate OCCT *degenerate* edge (a pole singularity on a closed
sphere/cone, where the surface parametrization collapses to a single
point) rather than a genuine CAD defect -- confirmed live, 2026-08-27,
`testing/inputSTEP/Torus/face2.stp` and `tank.stp`: both real,
long-working fixtures have real sphere faces whose own pole edges
measure ~7.7e-15mm (floating-point noise around a mathematically exact
zero, not a real gap), which find_short_edges' own detection wrongly
flagged as corrupted before this floor was added -- a real false
positive that broke `tests/test_cadtocsg.py` outright (the new
"stop"-by-default behavior halted on 2 previously-clean files). Per
direct user instruction, set well below the smallest genuine defect
found anywhere in this project's own corpus work (~4e-4mm, Decomposed/
modelcell_cut1_v2_piece66.stp's own real sliver) while still staying
comfortably above the ~1e-15 floating-point noise floor -- 1e-9 keeps
6 orders of magnitude of margin on the noise side and 5 on the real-
defect side."""


def find_short_edges(solid, rel_tol: float = 1e-4) -> list:
    """Faces of `solid` (a GSolid) touching at least one edge whose own
    length is pathologically small relative to the solid's overall scale
    (edge.Length / solid.BoundBox.DiagonalLength < rel_tol) -- a purely
    topological signature of a degenerate/spurious feature (a residual
    boolean-cut artifact, an accidental sliver from a CAD export),
    independent of surface type or parameters. Duck-typed on
    `solid.Faces` (each a GFace with `.Edges`, each a GEdge with
    `.Length`) and `solid.BoundBox.DiagonalLength` -- identical across
    all 3 engines, no native calls needed.

    Confirmed live on a real fixture (`Solidos/working_solids/
    "beltline left.stp"`): a visually-obvious spurious plane, invisible
    to both `BRepCheck_Analyzer` (reports the solid fully valid) and to
    `GFace.CharacteristicWidth` (the plane's own width is unremarkable,
    ~1mm) -- is caught immediately this way: its own boundary edges
    connecting to the model's real geometry measure 0.888mm, and its
    neighboring sliver face's edges measure 0.029mm, both several orders
    of magnitude below the model's own ~7246mm diagonal, while every
    other edge in the model is in the thousands-of-mm range. Default
    `rel_tol=1e-4` (0.01% of the model's own scale, per direct user
    instruction: real models can be meter-scale with legitimate
    millimeter-scale details, which a looser 1e-3 default risks flagging
    as false positives) -- comfortably below both tiers above; a
    109-file corpus scan (Solidos/test_models, raw solids and
    decomposed pieces alike) found zero false positives at this value,
    including on a fixture with independently-documented real, legitimate
    ~0.026mm-wide faces (Decomposed/modelcell_cut1_v2_piece66.stp) --
    confirmed to correctly distinguish that real feature from a
    genuinely separate, much smaller (0.0004mm-edge) sliver on the same
    piece. Unlike `Tolerances.min_face_width`, this needs no per-solid
    `scaled()` accommodation -- being already relative to each solid's
    own BoundBox, it doesn't suffer the "small decomposed piece" failure
    mode that motivated `scaled()` in the first place.

    NOTE on repair, not detection: this function is deliberately simple
    and fast -- it returns every face touching a short edge, not a
    minimal "just the spurious cluster" set (which would need real
    topological reasoning: on "beltline left.stp" specifically, this
    returns 8 faces, including 2 legitimate mirror-symmetry-cut planes
    and 2 legitimate end caps that merely happen to touch the same short
    edges as the 4 real defect faces -- confirmed live, no candidate
    graph-based refinement tried gave a general, reliably-correct
    minimal set for this fixture). Per explicit user direction, this is
    accepted: detecting a genuine CAD defect is more valuable than
    perfectly auto-repairing it (a false "translation failure" wrongly
    blamed on GEOUNED is worse than an honest "this solid could not be
    auto-repaired, fix the CAD" -- see geo.Gdefeature's own docstring for
    how repair is attempted and safely abandoned when it doesn't
    converge cleanly).

    Returns each offending face once (never duplicated, even if it has
    several short edges); empty list if none found. The effective
    threshold is `max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)` -- never
    below the 1e-3mm absolute floor, per direct user instruction (guards
    the small-solid case where `diag` alone would otherwise push the
    relative threshold below any meaningful working tolerance). An edge
    shorter than DEGENERATE_EDGE_LENGTH_FLOOR is never flagged, however
    small the effective threshold gets -- see that constant's own
    docstring for why (a legitimate OCCT pole-degenerate edge, not a
    defect)."""
    diag = solid.BoundBox.DiagonalLength
    if diag <= 0.0:
        return []
    threshold = max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)
    flagged = []
    for face in solid.Faces:
        for edge in face.Edges:
            if DEGENERATE_EDGE_LENGTH_FLOOR <= edge.Length < threshold:
                flagged.append(face)
                break
    return flagged


def _solve_quadratic(a: float, b: float, c: float) -> tuple[float, float] | None:
    """Real roots of a*t^2 + b*t + c = 0, ordered (smaller, larger).
    None if there are 0 real roots, or if the equation degenerates to
    non-quadratic (a ~ 0) -- the caller needs a genuine entry/exit pair,
    not a single crossing."""
    if abs(a) < 1e-9:
        return None
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    r1 = (-b - sq) / (2.0 * a)
    r2 = (-b + sq) / (2.0 * a)
    return (r1, r2) if r1 <= r2 else (r2, r1)


def find_can_plane(
    main_center: GVector,
    main_axis: GVector,
    main_radius: float,
    kind: str,
    secondary,
    n_angles: int = 720,
    narrow_wide_threshold: float | None = None,
) -> tuple[GVector, GVector] | None:
    """Compute the disambiguating plane for a Can/RoundCorner-style
    composite surface: a main cylinder split into two disjoint pieces by
    a secondary surface (`kind` one of "cylinder"/"cone"/"sphere",
    `secondary` a GCylinder/GCone/GSphere-like descriptor).

    The two (infinite) analytic surfaces meet along two closed tangency
    contours on the main cylinder -- for each angle phi around the main
    cylinder's own circumference, the line along its axis crosses the
    secondary surface's boundary at up to two points (a quadratic in the
    axial parameter t, for all three secondary types). The near contour
    is where each such line *enters* the secondary surface, the far
    contour where it *exits*; the free zone -- where a plane can sit
    without cutting either the "outside secondary" or "inside secondary"
    real material -- lies strictly between the near contour's own
    furthest-advanced point and the far contour's own furthest-back
    point, projected onto the plane's normal.

    Returns None if either contour fails to close over the full angular
    range (no real roots, or -- for a cone secondary -- a root on the
    wrong nappe -- at some phi): the main cylinder isn't actually split
    into two disjoint pieces by this secondary surface, so the premise
    for a Can doesn't hold at all.

    The plane's normal is not always main_axis: for a Cylinder/Cone
    secondary (which has its own axis), it's the direction perpendicular
    to the secondary's axis within the plane containing both axes --
    the natural cross-cutting direction where the two axes are close to
    parallel, not a cut transverse to the main cylinder's own length.
    Only a Sphere secondary (no axis of its own) uses main_axis directly.

    Once the free zone is found, its position is either the midpoint (if
    narrow) or a small step past the near contour capped at
    narrow_wide_threshold (if wide, so the plane never drifts far from
    the real local feature just because the analytic zone is huge) --
    see the docstring-adjacent discussion in functions.py::build_can_params
    for why both regimes are needed.
    """
    A = main_axis.normalized()
    e2 = A.cross(GVector(1.0, 0.0, 0.0))
    if e2.length < 1e-6:
        e2 = A.cross(GVector(0.0, 1.0, 0.0))
    e2 = e2.normalized()
    e1 = A.cross(e2).normalized()

    if kind == "cylinder":
        secondary_axis = secondary.Axis.normalized()
        a_dot_as = A.dot(secondary_axis)
        radius2 = secondary.Radius * secondary.Radius
        reference = secondary.Center

        def coeffs(u: GVector) -> tuple[float, float, float]:
            a = 1.0 - a_dot_as * a_dot_as
            u_a = u.dot(A)
            u_as = u.dot(secondary_axis)
            b = 2.0 * (u_a - u_as * a_dot_as)
            c = u.dot(u) - u_as * u_as - radius2
            return a, b, c

        def root_ok(u: GVector, t: float) -> bool:
            return True

    elif kind == "cone":
        secondary_axis = secondary.Axis.normalized()
        a_dot_ac = A.dot(secondary_axis)
        cos2 = math.cos(secondary.SemiAngle) ** 2
        reference = secondary.Apex

        def coeffs(u: GVector) -> tuple[float, float, float]:
            p_ = u.dot(secondary_axis)
            a = a_dot_ac * a_dot_ac - cos2
            b = 2.0 * (p_ * a_dot_ac - cos2 * u.dot(A))
            c = p_ * p_ - cos2 * u.dot(u)
            return a, b, c

        def root_ok(u: GVector, t: float) -> bool:
            # single-nappe cone: only the side u.dot(axis) + t*(A.dot(axis)) >= 0
            # (matching is_inside_cone's own convention) is physically real.
            return (u.dot(secondary_axis) + t * a_dot_ac) >= 0.0

    elif kind == "sphere":
        secondary_axis = None
        radius2 = secondary.Radius * secondary.Radius
        reference = secondary.Center

        def coeffs(u: GVector) -> tuple[float, float, float]:
            a = 1.0
            b = 2.0 * u.dot(A)
            c = u.dot(u) - radius2
            return a, b, c

        def root_ok(u: GVector, t: float) -> bool:
            return True

    else:
        raise ValueError(f"find_can_plane: unknown secondary kind {kind!r}")

    if secondary_axis is None:
        normal = A
    else:
        n_common = A.cross(secondary_axis)
        if n_common.length < 1e-9:
            normal = A  # axes (near-)parallel -- common plane undefined
        else:
            normal = secondary_axis.cross(n_common.normalized()).normalized()

    best_near = (-math.inf, None)  # (projection onto normal, point)
    best_far = (math.inf, None)
    for i in range(n_angles):
        phi = 2.0 * math.pi * i / n_angles
        offset = e1 * (main_radius * math.cos(phi)) + e2 * (main_radius * math.sin(phi))
        u = (main_center - reference) + offset

        roots = _solve_quadratic(*coeffs(u))
        if roots is None:
            return None
        t_near, t_far = roots
        if not (root_ok(u, t_near) and root_ok(u, t_far)):
            return None

        p_near = main_center + A * t_near + offset
        p_far = main_center + A * t_far + offset
        proj_near = p_near.dot(normal)
        proj_far = p_far.dot(normal)
        if proj_near > best_near[0]:
            best_near = (proj_near, p_near)
        if proj_far < best_far[0]:
            best_far = (proj_far, p_far)

    proj_near, point_near = best_near
    proj_far, point_far = best_far
    if proj_far <= proj_near:
        return None

    if narrow_wide_threshold is None:
        narrow_wide_threshold = 0.01 * main_radius

    half_width = 0.5 * (proj_far - proj_near)
    if half_width <= narrow_wide_threshold:
        offset_amount = half_width
    else:
        offset_amount = min(0.001 * half_width, narrow_wide_threshold)

    position = point_near + normal * offset_amount
    return position, normal


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


# ---------------------------------------------------------------------------
# I/O helper -- not geometry math, but shared across all 3 backends since
# none of them expose a clean way to silence this natively
# ---------------------------------------------------------------------------


@contextlib.contextmanager
def suppress_native_stdout():
    """Silences C/C++-level writes to stdout for the duration of the block
    -- e.g. OCCT's STEPControl_Writer (used by every export_step()/
    Gexport_step() in all 3 backends, including FreeCAD's own
    Part.Shape.exportStep(), which wraps the identical OCCT writer), which
    prints its own "Statistics on Transfer (Write)" banner directly via
    std::cout, unconditionally, with no verbosity/quiet flag exposed
    anywhere -- confirmed live (2026-08-27): none of Interface_Static's
    known parameter names ("write.step.verbosity", "write.verbosity",
    "write.step.trace", ...) exist, so there is no OCCT-side switch to
    flip instead.

    contextlib.redirect_stdout has no effect on this kind of write --
    it only reroutes Python's own sys.stdout object, not the OS file
    descriptor a native library's std::cout is bound to. This redirects
    the real file descriptor (fd 1) instead, so it silences a native
    library's own direct writes too, not just Python's print(). Restores
    the original fd unconditionally, even if the block raises.
    """
    sys.stdout.flush()
    saved_fd = os.dup(1)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull_fd, 1)
        yield
    finally:
        sys.stdout.flush()
        os.dup2(saved_fd, 1)
        os.close(devnull_fd)
        os.close(saved_fd)
