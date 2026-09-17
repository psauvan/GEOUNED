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


def arbitrary_perpendicular(axis: GVector) -> GVector:
    """An arbitrary, deterministic unit vector perpendicular to `axis`,
    picked by crossing `axis` with whichever world axis its own largest
    component is aligned with (avoids a near-zero cross product).

    Pure `GVector` math, moved here from `GEOReverse/Modules/_freecad_impl.py`'s
    `ortoVect` (2026-09-12) once confirmed it had zero native dependency of
    its own -- used by that file's exotic-quadric shape construction
    (`GParaboloid.build_shape`, `_make_torus_elliptic_native`). NOT the
    same formula as `GEOUNED/utils/meta_surfaces_utils.py::_perpendicular_axis`
    (a separate, independently-validated "some stable perpendicular"
    heuristic used by the winding-closure check) -- the two solve the same
    kind of problem for unrelated call sites and were never verified to be
    interchangeable, so they are kept as two distinct functions rather than
    merged; do not swap one in for the other without re-verifying the
    specific caller."""
    vmax = 0.0
    v_orto = None
    if abs(axis.x) > vmax:
        v_orto = GVector(0, 1, 0)
        vmax = abs(axis.x)
    if abs(axis.y) > vmax:
        v_orto = GVector(0, 0, 1)
        vmax = abs(axis.y)
    if abs(axis.z) > vmax:
        v_orto = GVector(1, 0, 0)
        vmax = abs(axis.z)
    if v_orto is None:
        return None
    return axis.cross(v_orto).normalized()


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
# myBox: axis-aligned box approximation of a boolean cell's material region
# ---------------------------------------------------------------------------
#
# Shared 2026-09-17 between GEOUNED's `build_region/` (constructs the small
# solid a composite meta-surface -- RoundCorner/Can/TCone/MultiRoundCorner --
# itself represents, from its own 2-4 primitive components) and GEOReverse's
# `CAD/buildSolidCell.py`/`Utils/boundBox.py` (reconstructs an arbitrary
# MCNP/OpenMC cell's solid from its boolean surface definition, where
# `solid_plane_box.build_box_depth()` walks the definition tree combining
# per-surface myBox instances via the same AND=`.mult()`/OR=`.add()` this
# class provides). Pure box arithmetic over `GBoundBox`, zero native-kernel
# dependency -- previously two independently-maintained copies (one in each
# pipeline) that had silently drifted: GEOReverse's copy had a real,
# corpus-affecting bug (see `.add()`/`.mult()` docstrings below) that was
# found and fixed via a systematic empirical audit; GEOUNED's own copy
# (`box_intersect`/`plane_region`, deleted in the same move that added this
# shared class) turned out to have the exact same bug, just never exercised
# in practice (GEOUNED's only live call site for `.mult()`,
# `build_region.py::filterparts`, always constructs both operands with
# `orientation="Forward"`, the one branch that was already correct in both
# old copies).


def _box_volume(box: "GBoundBox") -> float:
    return box.XLength * box.YLength * box.ZLength


class myBox:
    """`Orientation="Forward"` + `Box=X` means the region's material is
    INSIDE `X`; `Orientation="Reversed"` + `Box=X` means material is
    OUTSIDE `X` (the "hole"). `Box=None` + `Forward` = empty; `Box=None` +
    `Reversed` = the whole universe."""

    def __init__(self, boundBox=None, orientation=None):
        self.Volume = 0
        if type(boundBox) is myBox:
            self.Box = boundBox.Box
            self.Orientation = boundBox.Orientation
            self.Volume = boundBox.Volume
        else:
            if boundBox is not None:
                boundBox = to_gboundbox(boundBox)
                if boundBox.XLength <= 1e-12 or boundBox.YLength <= 1e-12 or boundBox.ZLength <= 1e-12:
                    self.Box = None
                else:
                    self.Box = boundBox
                    self.Volume = _box_volume(boundBox)
            else:
                self.Box = None
            self.Orientation = orientation
        if self.Orientation is None:
            raise TypeError("myBox orientation cannot be None")

    def add(self, box):
        """Non-mutating GBoundBox equivalent of FreeCAD.BoundBox's own
        in-place `.add()` -- computes self = self OR box.

        Fixed 2026-09-17 (found via a systematic empirical audit,
        methodology and full derivation in CLAUDE.md's own "myBox.add()/
        .mult() arithmetic" entry -- the audit found 20/32 tested
        Forward/Reversed configurations UNSAFE, i.e. silently EXCLUDING
        real material, not just imprecise): a `myBox` with
        Orientation="Forward" means "material is INSIDE Box"; Orientation=
        "Reversed" means "material is OUTSIDE Box" (Box=None + Reversed =
        the whole universe; Box=None + Forward = empty). The previous
        code, once both operands had a real Box, always computed
        `self.Box.union(box.Box)` regardless of orientation -- correct
        only for Forward OR Forward. For exactly one Reversed operand
        (`A + notB`), the true result is `notB` restricted to `B \\ A`'s
        own bounding box -- generally not a single box at all, so this
        keeps only the one case that IS exact and safe (the two boxes
        don't overlap at all, so `B \\ A == B`) and falls back to the
        always-safe "universe" (Box=None) otherwise, rather than the old
        `union(A,B)`, which could claim strictly LESS material than the
        true `notB \\ A` region (confirmed empirically: e.g. A, B disjoint
        gave Reversed+union(A,B), wrongly excluding all of A -- the old
        formula's real safety violation, not just a looseness one). For
        two Reversed operands (`notA + notB`), De Morgan gives
        `not(A and B)`, i.e. Reversed with Box = the *intersection* of A
        and B (empty when disjoint) -- the old code's plain
        `union(A,B)` was `not(A or B)` instead, the AND case's own
        answer, not this one's."""
        if self.Box is None:
            if self.Orientation == "Forward":
                self.Box = box.Box
                self.Orientation = box.Orientation
                self.Volume = box.Volume
        elif box.Box is None:
            if box.Orientation == "Reversed":
                self.Box = None
                self.Orientation = "Reversed"
                self.Volume = 0
        else:
            if self.Orientation == box.Orientation:
                if self.Orientation == "Forward":
                    self.Box = self.Box.union(box.Box)
                else:
                    inter = self.Box.intersected(box.Box)
                    self.Box = inter if inter.is_valid() else None
            else:
                fwd_box = self.Box if self.Orientation == "Forward" else box.Box
                rev_box = box.Box if self.Orientation == "Forward" else self.Box
                overlap = fwd_box.intersected(rev_box)
                self.Box = None if overlap.is_valid() else rev_box
                self.Orientation = "Reversed"
            self.Volume = _box_volume(self.Box) if self.Box is not None else 0

    def mult(self, box):
        """Non-mutating GBoundBox equivalent of the original's in-place
        `.add()` in the AND branch -- computes self = self AND box.

        Fixed 2026-09-17, same audit as `add()` above: for exactly one
        Reversed operand (`A * notB`, i.e. `A \\ B`), the true result is
        generally not a single box either, but here there's always a
        SAFE, simple, exact-when-disjoint choice needing no case split at
        all: `A \\ B` is always a *subset* of `A` itself, so keeping the
        Forward operand's own Box completely unchanged (discarding the
        Reversed operand's Box entirely) is always a safe upper bound,
        exact whenever the two don't overlap. The old code instead
        computed `self.Box.intersected(box.Box)` regardless of
        orientation here -- the AND-of-two-Forward-boxes formula, wrong
        for this case (confirmed empirically unsafe: e.g. A, B disjoint
        gave Forward+None (empty!) for `A * notB`, when the true answer
        is all of A).

        `notA * notB` (both Reversed, De Morgan: `not(A or B)`, i.e.
        Reversed with Box = A union B) needed its own separate fix, found
        by the same audit: unlike an intersection of two axis-aligned
        boxes (always itself exactly one axis-aligned box, or empty), a
        UNION of two boxes is only exactly one box when they combine with
        no gap relative to their own combined bounding box (e.g. two
        boxes sharing a full common range on one axis, or one containing
        the other) -- otherwise the bounding box of A union B is a real
        over-approximation of the true excluded region, unsafe here
        (a Reversed box's own Box represents what's excluded, so an
        oversized one wrongly excludes real material -- confirmed
        empirically: two disjoint boxes gave a bounding "union" box that
        wrongly claimed the empty gap between them as excluded too, and
        an L-shaped pair sharing only a corner did the same for the
        gap in their own combined bounding box's far corner). Checked via
        the standard inclusion-exclusion identity (no gap exists iff the
        bounding box's own volume equals `vol(A) + vol(B) - vol(A∩B)`
        exactly); the safe fallback otherwise is the larger of the two
        boxes alone (always a subset of A union B, so always safe, just
        not always tight)."""
        if self.Orientation is None:
            self.Box = box.Box
            self.Orientation = box.Orientation
            self.Volume = box.Volume
        elif self.Box is None:
            if self.Orientation == "Reversed":
                self.Box = box.Box
                self.Orientation = box.Orientation
                self.Volume = box.Volume
        elif box.Box is None:
            if box.Orientation == "Forward":
                self.Box = None
                self.Orientation = "Forward"
                self.Volume = 0
        else:
            if self.Orientation == box.Orientation:
                if self.Orientation == "Reversed":
                    union_box = self.Box.union(box.Box)
                    inter = self.Box.intersected(box.Box)
                    inter_vol = _box_volume(inter) if inter.is_valid() else 0.0
                    self_vol = _box_volume(self.Box)
                    box_vol = _box_volume(box.Box)
                    union_vol = _box_volume(union_box)
                    if abs(union_vol - (self_vol + box_vol - inter_vol)) < 1e-6 * max(union_vol, 1.0):
                        self.Box = union_box
                    else:
                        self.Box = self.Box if self_vol >= box_vol else box.Box
                else:
                    inter = self.Box.intersected(box.Box)
                    self.Box = inter if inter.is_valid() else None
            else:
                if self.Orientation != "Forward":
                    self.Box = box.Box
                self.Orientation = "Forward"
            self.Volume = _box_volume(self.Box) if self.Box is not None else 0

    def sameBox(self, box):
        if self.Box is None or box.Box is None:
            if self.Box is None and box.Box is None:
                return self.Orientation == box.Orientation
            else:
                return False

        for i in range(6):
            p1 = self.Box.get_point(i)
            p2 = box.Box.get_point(i)
            if (p1 - p2).length > 1e-6:
                return False
        return True


# ---------------------------------------------------------------------------
# Angular arc utilities
# ---------------------------------------------------------------------------


def arc_extent(pairs: list[tuple[float, float]], tol: float = 1e-5) -> tuple[float, int, float, int]:
    """
    Given a list of (a0, a1) angle pairs (a0 < a1, radians) that together
    trace exactly one open circular arc (total angular extent < 2*pi),
    find the arc's two free ends.

    Each pair may sit anywhere on the real line -- angles are not
    required to already lie in [0, 2*pi). Between any two pairs
    (a0,a1)/(b0,b1) the only allowed relations are a chained overlap
    (a0<=b0<=a1<=b1), a nesting (a0<=b0<b1<=a1), or a real gap (a1<b0)
    that some other pair in the list bridges -- possibly by wrapping
    through 0/2*pi.

    Returns (angle_min, index_min, angle_max, index_max): the original,
    unmodified endpoint values (and their pair's index in `pairs`) that
    bound the arc. Walking from angle_min with increasing angle (wrapping
    through 0/2*pi if needed) reaches angle_max after covering exactly
    the arc -- angle_min is not necessarily numerically smaller than
    angle_max (e.g. angle_min=5.6, angle_max=2: the arc passes through
    0, equivalent to angle_min == 5.6 - 2*pi once unwrapped).
    """
    n = len(pairs)
    if n == 0:
        raise ValueError("arc_extent: empty pair list")

    two_pi = 2.0 * math.pi

    # Canonicalize each pair into a common frame: shift the whole pair
    # (rigidly, same shift on both ends) so its own start lands in
    # [0, 2*pi) -- this only changes which representative of the pair's
    # position on the circle we compare with, never its real extent.
    c0 = [0.0] * n
    c1 = [0.0] * n
    for i, (a0, a1) in enumerate(pairs):
        start = a0 - two_pi * math.floor(a0 / two_pi)
        c0[i] = start
        c1[i] = start + (a1 - a0)

    order = sorted(range(n), key=lambda i: c0[i])

    # Standard sweep-merge in the canonical frame. Because the whole set
    # forms one arc on the circle, and cutting a circle at one point
    # (here, the 0/2*pi boundary) can split a single arc into at most 2
    # pieces, this can only ever produce 1 or 2 groups.
    groups = []
    for i in order:
        if groups and c0[i] <= groups[-1]["end_val"] + tol:
            if c1[i] > groups[-1]["end_val"]:
                groups[-1]["end_val"] = c1[i]
                groups[-1]["end_idx"] = i
        else:
            groups.append(
                {
                    "start_val": c0[i],
                    "start_idx": i,
                    "end_val": c1[i],
                    "end_idx": i,
                }
            )

    if len(groups) == 1:
        g = groups[0]
        return pairs[g["start_idx"]][0], g["start_idx"], pairs[g["end_idx"]][1], g["end_idx"]

    if len(groups) == 2:
        # groups is sorted by start_val ascending: groups[0] is the piece
        # nearest 0 (the arc's tail after wrapping), groups[-1] is the
        # piece nearest 2*pi (the arc's true start, continuing through
        # the wrap into groups[0]).
        first, last = groups[0], groups[-1]
        if last["end_val"] < first["start_val"] + two_pi - tol:
            raise ValueError(
                "arc_extent: pairs do not stitch into a single arc across " "the 0/2*pi boundary (gap between the two groups)"
            )
        return pairs[last["start_idx"]][0], last["start_idx"], pairs[first["end_idx"]][1], first["end_idx"]

    raise ValueError(f"arc_extent: pairs split into {len(groups)} disconnected groups, not a single arc")
