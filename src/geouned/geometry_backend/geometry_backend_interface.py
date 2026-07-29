"""
geometry_backend/interface.py

Neutral interface (Adapter / Ports & Adapters pattern) to decouple GEOUNED
from its underlying geometry engine (FreeCAD Part / pythonOCC).

Design principle:
- The rest of GEOUNED (decomposition, surfaces, io...) NEVER imports
  `Part`, `FreeCAD`, or `OCC.Core` directly.
- It only knows these neutral types (GSolid, GFace, GEdge, GWire, GVertex,
  GPlane/GCylinder/..., GLine/GCircle/...) and the `GeometryBackend`
  interface.
- Each concrete backend (FreeCADBackend, OCCBackend) acts as a translator
  between these neutral types and the real objects of its library.
- The backend only knows about the 5 analytic surface types (plane,
  cylinder, cone, sphere, torus) and the 4 analytic curve types (line,
  circle, ellipse, bspline). Composite/meta-surfaces (RoundCorner, Can,
  TCone, MultiPlane, ReversedConeCyl...) are NOT a backend concept: they
  are built by GEOUNED itself, by combining backend-constructed analytic
  primitives with boolean operations (cut/common/fuse). The backend never
  needs to know a meta-surface exists.

`GFace`/`GEdge`/`GVertex` carry an opaque `native` reference to the
backend's real object, plus the backend that produced it -- but unlike a
flat wrapper, `get_faces()`/`get_edges()`/`get_vertices()` populate them
eagerly with their classified neutral geometry (`.Surface`/`.Curve`),
topology (`.Edges`/`.Vertexes`/`.OuterWire`) and parameter range, all in
one pass. This mirrors GEOUNED's own `FaceGu`/`SolidGu` caching (done
"to release memory" -- repeated native attribute access is a measured
perf problem) but with neutral, GVector-based data instead of native
types, and it means GEOUNED code never needs `isinstance(x, Part.Shape)`
or a second round-trip into the backend just to read a face's axis.

`GVector` is the exception carrying real behavior: a plain value type
with its own arithmetic (dot, cross, length, normalize...), independent
of any backend. Higher-level geometric predicates built on top of it
(parallel, opposite, in-line, in-plane...) live in `vector_geometry.py`,
not in this module.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Sequence, Union


# ---------------------------------------------------------------------------
# Neutral vector type (pure math, no backend dependency)
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


@dataclass(frozen=True)
class GBoundBox:
    """
    Neutral axis-aligned bounding box. Pure math, no backend dependency --
    same philosophy as GVector: constructible directly by GEOUNED (e.g.
    splitting a box in half) as well as returned by the backend (e.g.
    `bounding_box`/`optimal_bounding_box`). Field names match FreeCAD's
    own `Base.BoundBox` attribute names.
    """
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


# ---------------------------------------------------------------------------
# Analytic surface types
# (the 5 the backend must know about; composite meta-surfaces are
# assembled by GEOUNED from these, not modeled here. Field names match
# FreeCAD's own Part.Plane/Cylinder/Cone/Sphere/Toroid attribute names to
# minimize churn at the many call sites that already read `.Axis`,
# `.Center`, `.Position`, `.Radius` etc. Type is discriminated with
# `type(face.Surface) is GPlane`, same convention GEOUNED's own
# PlaneGu/CylinderGu/... already used.)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GPlane:
    Position: GVector
    Axis: GVector
    # Reference "u=0" direction (OCCT's XDirection), needed to evaluate
    # value_at/tangent_at analytically so (u, v) agrees with the native
    # face's own parametrization. None when the plane was reconstructed
    # from already-processed data (e.g. legacy PlaneParams) rather than
    # classified directly off a native face -- value_at/tangent_at raise
    # in that case rather than fabricate an arbitrary direction.
    XDir: GVector | None = None


@dataclass(frozen=True)
class GCylinder:
    Center: GVector
    Axis: GVector
    Radius: float
    # Same XDirection caveat as GPlane.XDir above.
    XDir: GVector | None = None


@dataclass(frozen=True)
class GCone:
    Apex: GVector
    Axis: GVector
    SemiAngle: float  # radians
    Radius: float  # radius of the reference circle at the face's v=0


@dataclass(frozen=True)
class GSphere:
    Center: GVector
    Radius: float


@dataclass(frozen=True)
class GTorus:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float


GSurface = Union[GPlane, GCylinder, GCone, GSphere, GTorus]


# ---------------------------------------------------------------------------
# Analytic curve types
# (the 4 the backend must know about, needed for wire/edge handling in
# decomposition and GEOReverse. Same FreeCAD-matching-name convention as
# the surface types above.)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GLine:
    Position: GVector
    Direction: GVector


@dataclass(frozen=True)
class GCircle:
    Center: GVector
    Axis: GVector
    Radius: float


@dataclass(frozen=True)
class GEllipse:
    Center: GVector
    Axis: GVector
    MajorAxis: GVector
    MajorRadius: float
    MinorRadius: float


@dataclass(frozen=True)
class GBSpline:
    Poles: list[GVector]


GCurve = Union[GLine, GCircle, GEllipse, GBSpline]


# ---------------------------------------------------------------------------
# Neutral topology types
# ---------------------------------------------------------------------------

class _ExportableShape:
    """
    Mixin for every neutral type that carries a real backend shape (a
    `native` + the `backend` that produced it): a debugging convenience to
    dump just that one shape to STEP and open it in a CAD viewer, without
    reaching for `backend.export_step([...], ...)` and a one-element list
    every time.
    """
    def export_step(self, filename: str) -> None:
        self.backend.export_step([self], filename)


@dataclass(frozen=True)
class GSolid(_ExportableShape):
    """Opaque wrapper around a backend-native solid."""
    native: Any
    backend: "GeometryBackend"
    Orientation: str
    BoundBox: GBoundBox


@dataclass(frozen=True)
class GVertex:
    native: Any
    backend: "GeometryBackend"
    Point: GVector


@dataclass(frozen=True)
class GEdge(_ExportableShape):
    native: Any
    backend: "GeometryBackend"
    Curve: GCurve
    Vertexes: tuple[GVertex, ...]
    ParameterRange: tuple[float, float]
    Orientation: str


@dataclass(frozen=True)
class GWire:
    native: Any
    backend: "GeometryBackend"


@dataclass
class GFace(_ExportableShape):
    """
    Not frozen, unlike the other neutral types: `index` is assigned by
    GEOUNED after construction (a face's position within its parent
    solid's face list, e.g. for "is this the same face" adjacency
    checks) and has no meaningful value until then.
    """
    native: Any
    backend: "GeometryBackend"
    Surface: GSurface
    Edges: tuple[GEdge, ...]
    OuterWire: GWire
    ParameterRange: tuple[float, float, float, float]
    Orientation: str
    index: int | None = None


@dataclass(frozen=True)
class GShell(_ExportableShape):
    """
    Opaque wrapper around a backend-native shell: a connected group of
    faces, sitting between GFace and GSolid in the topology hierarchy.
    GEOUNED builds these itself (e.g. grouping faces that share the same
    analytic surface but got split into separate topological faces by a
    seam or a tangency line) via `make_shell`, not by classification off
    an existing solid.
    """
    native: Any
    backend: "GeometryBackend"
    Faces: tuple[GFace, ...]
    Orientation: str


# A shape-like argument accepted by generic spatial queries (in_contact...).
GShape = Union[GSolid, GFace, GEdge, GShell]


# ---------------------------------------------------------------------------
# Result of operations that can fail/degenerate
# (this is where we encapsulate the tangency problem discussed earlier)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SplitResult:
    """
    Result of a split/cut operation.

    `solids` must never be empty for a valid operation: if the backend
    detects a degenerate case (tangency, coincident edges...) it must
    resolve it internally (micro-offset, ShapeFix, etc.) and report it
    via `degenerate_case_handled=True`, NEVER silently return the
    original, uncut solid.
    """
    solids: list[GSolid]
    degenerate_case_handled: bool = False
    notes: str = ""


@dataclass(frozen=True)
class GLabelNode:
    """
    One node of a STEP file's assembly/label tree, as read by
    `load_step_labels`. Only solid-bearing leaves are returned in that
    list, but `parent` still walks up through every ancestor (including
    non-solid-bearing group/assembly nodes) so callers can reconstruct a
    full label path, exactly as GEOUNED's own comment/material/dilution/
    enclosure parsing needs.

    Positionally aligned with `load_step`: node[i]'s `n_solids` solids are
    `load_step(...)`'s next `n_solids` entries, in order, once all nodes
    up to `i` have been consumed.
    """
    label: str
    parent: "GLabelNode | None"
    n_solids: int


# ---------------------------------------------------------------------------
# Main interface
# ---------------------------------------------------------------------------

class GeometryBackend(ABC):
    """
    Contract that any geometry engine used by GEOUNED must fulfil.

    Conventions:
    - All lengths in mm (consistent with the rest of GEOUNED).
    - Angles returned by the backend are in radians; conversion to
      degrees (when needed for MCNP) happens in the `surfaces/` layer,
      not here.
    - No method should print or log directly: raise typed exceptions
      (see `errors.py`, not included here) and let the calling layer
      decide how to report it.
    """

    # -- Metadata ---------------------------------------------------------

    @abstractmethod
    def kernel_version(self) -> str:
        """
        Version string of the underlying geometry kernel/application (e.g.
        FreeCAD's own version), for provenance notes in output file
        headers. Purely informational -- callers must not parse or branch
        on the format.
        """

    # -- I/O ------------------------------------------------------------

    @abstractmethod
    def load_step(self, filename: str) -> list[GSolid]:
        """
        Load a STEP file and return the list of top-level solids, with
        every transformation from the file's assembly/placement hierarchy
        already applied (baked into each solid's own geometry) -- callers
        never need to apply a separate placement themselves.
        """

    @abstractmethod
    def load_step_labels(self, filename: str) -> list[GLabelNode]:
        """
        Parse the same STEP file's assembly tree and return one
        `GLabelNode` per solid-bearing leaf, in the same order as
        `load_step`'s solids (see `GLabelNode`'s docstring for exactly how
        they line up). This is a separate read from `load_step`, not a
        by-product of it: extracting labels/hierarchy needs the file's
        assembly-tree structure, which is a different concern from -- and
        may use a different underlying reader than -- resolving each
        solid's final, transformed geometry.
        """

    @abstractmethod
    def export_step(self, shapes: Sequence[GShape], filename: str) -> None:
        """
        Export a list of shapes to a STEP file. Accepts any mix of
        GSolid/GFace/GEdge/GShell -- most callers export solids, but a
        single face/edge/shell is valid too (e.g. for debugging).
        """

    # -- Primitive construction -------------------------------------------
    # (used both by CsgToCad/GEOReverse and by GEOUNED's own meta-surface
    # assembly, which builds RoundCorner/Can/TCone/MultiPlane/... by
    # cutting/fusing these analytic primitives itself)

    @abstractmethod
    def make_box(
        self, xmin: float, ymin: float, zmin: float,
        xmax: float, ymax: float, zmax: float,
    ) -> GSolid: ...

    @abstractmethod
    def make_cylinder(
        self, base_point: GVector, axis_dir: GVector,
        radius: float, height: float,
    ) -> GSolid: ...

    @abstractmethod
    def make_cone(
        self, apex: GVector, axis_dir: GVector,
        half_angle: float, height: float,
    ) -> GSolid: ...

    @abstractmethod
    def make_sphere(self, center: GVector, radius: float) -> GSolid: ...

    @abstractmethod
    def make_torus(
        self, center: GVector, axis_dir: GVector,
        major_radius: float, minor_radius: float,
    ) -> GSolid: ...

    @abstractmethod
    def make_half_space(self, plane: GPlane) -> GSolid:
        """
        Half-space bounded by an infinite plane (internally clipped to a
        working box by the backend). Needed to reconstruct CSG cells
        defined by the intersection of half-spaces.
        """

    @abstractmethod
    def make_wire(self, edges: Sequence[GEdge]) -> GWire:
        """
        Build a wire from an ordered sequence of edges. GEOUNED performs
        wire joining/merging logic itself (e.g. splicing wires that
        share vertices) on top of this primitive and `in_contact`/
        `get_vertices`; the backend does not need to know about that
        higher-level logic.
        """

    @abstractmethod
    def make_polygon_face(self, points: Sequence[GVector]) -> GFace:
        """
        Build a single planar face bounded by the closed polygon through
        `points`, in order. Used to reconstruct a bounded plane face from
        the points where an infinite plane crosses a bounding box (GEOUNED
        computes the intersection points and their ordering; this is
        purely the "build a face from an ordered boundary" primitive).
        """

    @abstractmethod
    def make_compound(self, shapes: Sequence[GSolid]) -> GSolid:
        """
        Group `shapes` into a single compound shape, with no boolean
        operation applied (they may overlap or be disjoint). Used as a
        last-resort fallback when `fuse` fails or produces an invalid
        result.
        """

    @abstractmethod
    def reverse(self, solid: GSolid) -> GSolid:
        """
        Flip `solid`'s orientation (equivalent to `Part.Shape.reverse()`).
        Needed after a boolean operation reports a negative volume.
        """

    @abstractmethod
    def refine(self, solid: GSolid) -> GSolid:
        """
        Remove redundant edges/faces left by a boolean operation between
        coplanar/tangent surfaces (equivalent to `Part.Shape.removeSplitter()`).
        Purely cosmetic simplification -- never changes the enclosed volume.
        """

    @abstractmethod
    def make_shell(self, faces: Sequence[GFace]) -> GShell:
        """
        Build a shell from a group of faces. Unlike `get_faces`, this is
        not a classification of an existing solid's topology -- GEOUNED
        calls this to assemble its own face groupings (e.g. faces sharing
        one analytic surface split by a seam) into a single addressable
        shape, primarily so `in_contact`/`distance` can be called on the
        group as a whole.
        """

    # -- Boolean operations -------------------------------------------------

    @abstractmethod
    def cut(self, solid: GSolid, tools: Sequence[GSolid]) -> list[GSolid]:
        """Subtract `tools` from `solid`. May return >1 solid if fragmented."""

    @abstractmethod
    def common(self, solid: GSolid, tools: Sequence[GSolid]) -> list[GSolid]:
        """Boolean intersection."""

    @abstractmethod
    def fuse(self, solids: Sequence[GSolid]) -> GSolid:
        """Boolean union."""

    @abstractmethod
    def split(
        self, solid: GSolid, tool: GFace | GSolid, tolerance: float,
    ) -> SplitResult:
        """
        Cut `solid` with a surface/solid `tool` (typically a plane) and
        return ALL resulting fragments.

        This is the method that replaces `BOPTools.SplitAPI.slice` in
        the FreeCAD backend, and `BOPAlgo_Splitter` /
        `BRepAlgoAPI_Splitter` in the OCC backend.

        The backend is responsible for internally resolving degenerate
        cases (tangencies, coincident edges) -- see `SplitResult`. The
        GEOUNED decomposition layer must NOT implement retry/offset
        logic: that lives here.
        """

    # -- Topological traversal --------------------------------------------
    # (get_faces/get_edges/get_vertices return fully enriched objects:
    # classified Surface/Curve, nested topology, and parameter range are
    # all populated in one pass, not lazily re-derived per call)

    @abstractmethod
    def get_faces(self, solid: GSolid) -> list[GFace]: ...

    @abstractmethod
    def get_edges(self, face: GFace) -> list[GEdge]: ...

    @abstractmethod
    def get_outer_wire(self, face: GFace) -> GWire:
        """
        Outer boundary wire of a face. NOT necessarily the native
        kernel's own "outer wire" concept, which can pick the wrong wire
        for some faces -- the backend must apply GEOUNED's own heuristic
        (largest mean vertex-to-centroid distance among the face's
        wires) rather than trusting a native `OuterWire`-equivalent
        attribute blindly.
        """

    @abstractmethod
    def get_wire_edges(self, wire: GWire) -> list[GEdge]:
        """
        Edges of `wire`, in wire traversal order (edge[i] and edge[i+1]
        share a vertex). Callers rely on this ordering to splice/cut
        wires by edge index.
        """

    @abstractmethod
    def get_vertices(self, edge: GEdge) -> list[GVertex]: ...

    @abstractmethod
    def get_solid_vertices(self, solid: GSolid) -> list[GVertex]:
        """All vertices of `solid` (every vertex of every face), not just one edge's."""

    @abstractmethod
    def faces_sharing_edge(self, solid: GSolid, edge: GEdge) -> list[GFace]:
        """
        Number and list of faces sharing a given edge.
        A 'normal' edge in a closed solid is shared by 2 faces; >2 is
        the signal of a non-manifold condition (the tangency case we saw).
        """

    @abstractmethod
    def is_same_edge(self, edge_1: GEdge, edge_2: GEdge) -> bool:
        """
        True if `edge_1` and `edge_2` are the same underlying topological
        edge -- identity, not geometric coincidence (two distinct edges
        that happen to trace the same curve are NOT "same").
        """

    @abstractmethod
    def is_same_vertex(self, vertex_1: GVertex, vertex_2: GVertex) -> bool:
        """Same topological-identity distinction as `is_same_edge`, for vertices."""

    # -- Surface and curve classification -----------------------------
    # (the real core of the CAD -> CSG pipeline; restricted to the 5
    # analytic surface types and 4 analytic curve types -- composite
    # meta-surfaces are assembled above this layer, in GEOUNED itself.
    # These are the building blocks `get_faces`/`get_edges` use to
    # populate `GFace.Surface`/`GEdge.Curve`; call directly only if you
    # already hold a GFace/GEdge and need to re-classify it.)

    @abstractmethod
    def classify_surface(self, face: GFace) -> GSurface:
        """
        Determine the underlying surface type of a face
        (plane/cylinder/cone/sphere/torus) and extract its native
        geometric parameters, already converted to the neutral types
        (GPlane, GCylinder, ...).
        """

    @abstractmethod
    def classify_edge(self, edge: GEdge) -> GCurve:
        """
        Determine the underlying curve type of an edge
        (line/circle/ellipse/bspline) and extract its native geometric
        parameters, already converted to the neutral types.
        """

    @abstractmethod
    def face_orientation_outward(self, solid: GSolid, face: GFace) -> bool:
        """
        True if the face's normal, as oriented in the solid, points
        outward from the material. Needed to fix the sign (+/-) of the
        CSG surface in MCNP/OpenMC.
        """

    # -- Face parametric queries ---------------------------------------------
    # (only meaningful for the 5 analytic surface types above; not for
    # composite/meta-surfaces, which have no single (u, v) domain)

    @abstractmethod
    def parameter_range(self, face: GFace) -> tuple[float, float, float, float]:
        """Returns (u_min, u_max, v_min, v_max), the valid domain for `face`."""

    @abstractmethod
    def is_part_of_domain(self, face: GFace, u: float, v: float) -> bool:
        """
        True if (u, v) lies within `face`'s actual trimmed boundary, not just
        its parameter-range rectangle (`parameter_range` may report a wider
        domain than the face's real, possibly holed/cut, boundary).
        """

    @abstractmethod
    def face_value_at(self, face: GFace, u: float, v: float) -> GVector:
        """Point on `face` at parametric coordinates (u, v)."""

    @abstractmethod
    def face_parameter_at(self, face: GFace, point: GVector) -> tuple[float, float]:
        """(u, v) parametric coordinates of `point`, assumed to lie on `face`. Inverse of `face_value_at`."""

    @abstractmethod
    def face_normal_at(self, face: GFace, u: float, v: float) -> GVector:
        """Surface normal on `face` at parametric coordinates (u, v)."""

    @abstractmethod
    def face_tangent_at(self, face: GFace, u: float, v: float) -> tuple[GVector, GVector]:
        """Unit tangent directions (d/du, d/dv) on `face` at parametric coordinates (u, v)."""

    @abstractmethod
    def tessellate(self, face: GFace, tolerance: float) -> list[GVector]:
        """Sampled points approximating `face`, within `tolerance`."""

    @abstractmethod
    def face_get_uv_nodes(self, face: GFace, tolerance: float) -> list[tuple[float, float]]:
        """
        (u, v) parametric coordinates of each tessellation vertex, in the
        same order as `tessellate(face, tolerance)` -- lets a caller match
        a sampled 3D point back to where it sits in the face's parameter
        domain. Re-tessellates internally at `tolerance`; does not require
        `tessellate` to have been called first.
        """

    # -- Edge parametric queries -----------------------------------------
    # (only meaningful for the 4 analytic curve types above)

    @abstractmethod
    def edge_parameter_range(self, edge: GEdge) -> tuple[float, float]:
        """Returns (u_min, u_max), the valid domain for `edge`."""

    @abstractmethod
    def edge_value_at(self, edge: GEdge, u: float) -> GVector:
        """Point on `edge` at parametric coordinate `u`."""

    @abstractmethod
    def edge_derivative1_at(self, edge: GEdge, u: float) -> GVector:
        """
        First derivative of `edge`'s curve at parametric coordinate `u`.
        NOT unit-normalized -- e.g. on a circle of radius r this has
        length r, not 1. Distinct from `face_tangent_at`, which does
        return unit vectors; callers that need a direction only must
        normalize this themselves.
        """

    @abstractmethod
    def edge_normal_at(self, edge: GEdge, u: float) -> GVector:
        """
        Curve normal at parametric coordinate `u`. Only meaningful for a
        curved edge (circle/ellipse/bspline) -- undefined (the backend may
        raise) for a straight line, which has no curvature.
        """

    @abstractmethod
    def edge_length(self, edge: GEdge) -> float:
        """Arc length of `edge`."""

    # -- Geometric properties -------------------------------------------------

    @abstractmethod
    def volume(self, solid: GSolid) -> float: ...

    @abstractmethod
    def area(self, face: GFace) -> float: ...

    @abstractmethod
    def bounding_box(self, solid: GSolid) -> GBoundBox:
        """Fast axis-aligned bounding box (may overestimate for curved surfaces -- see `optimal_bounding_box`)."""

    @abstractmethod
    def optimal_bounding_box(self, solid: GSolid, use_triangulation: bool = True) -> GBoundBox:
        """
        More accurate axis-aligned bounding box than `bounding_box`
        (computed from the actual shape rather than the kernel's fast
        estimate), at higher cost. Still axis-aligned, not oriented --
        same shape as `bounding_box`, just tighter for curved surfaces.
        `use_triangulation=False` trades some accuracy for speed.
        """

    @abstractmethod
    def center_of_mass(self, solid: GSolid) -> GVector: ...

    # -- Spatial queries -------------------------------------------------------

    @abstractmethod
    def is_inside(self, solid: GSolid, point: GVector, tolerance: float) -> bool:
        """
        True if `point` lies inside `solid`. `solid` must be a closed
        volume (equivalent to Part.Shape.isInside()).
        """

    @abstractmethod
    def find_interior_point(self, solid: GSolid) -> GVector | None:
        """
        A point strictly inside `solid`'s volume, or None if none could be
        found (near-zero-volume/degenerate solid).

        Tries the center of mass first, then probes inward from each face
        along its inward normal. Benchmarked against two alternative
        strategies (sampling along solid-vertex segments; recursive
        bounding-box octree subdivision) that GEOUNED used to implement at
        several call sites: both are slower AND, for solids of revolution
        with few/no real vertices (a thin torus/fillet -- exactly what
        RoundCorner/TCone produce), the octree fallback can fail outright
        within its fixed subdivision depth. Face-normal probing has neither
        weakness, so it is the only strategy exposed here.
        """

    @abstractmethod
    def in_contact(self, shape_a: GShape, shape_b: GShape, tolerance: float) -> bool:
        """
        True if shape_a and shape_b share at least one point in the
        volumetric sense (touching or overlapping within tolerance; e.g.
        two concentric spherical shells count as in contact). Accepts
        any combination of GSolid/GFace/GEdge.

        GEOUNED never needs the actual distance value, only this
        boolean -- so the backend owns all robustness workarounds
        internally (bounding-box pre-filtering, boolean-common fallback,
        degenerate/slow-kernel cases) rather than each call site
        reimplementing them.
        """

    @abstractmethod
    def distance(self, shape_a: GShape, shape_b: GShape) -> float:
        """
        Minimum distance between shape_a and shape_b (0.0 if touching or
        overlapping). Accepts any combination of GSolid/GFace/GEdge. Unlike
        `in_contact`, this is a thin wrapper with no extra robustness
        layer -- use `in_contact` instead wherever only the boolean is
        actually needed.
        """

    # -- Validation / diagnostics --------------------------------------------

    @abstractmethod
    def is_valid(self, solid: GSolid) -> bool:
        """Equivalent to BRepCheck_Analyzer / shape.isValid()."""

    @abstractmethod
    def fix_shape(self, solid: GSolid, tolerance: float) -> GSolid:
        """
        Attempts to repair an invalid solid or one with degenerate
        topology (equivalent to ShapeFix_Shape / Part.Shape.fix()).
        """

    # -- Transformations -------------------------------------------------------

    @abstractmethod
    def translate(self, solid: GSolid, vector: GVector) -> GSolid: ...

    @abstractmethod
    def rotate(
        self, solid: GSolid, axis_point: GVector, axis_dir: GVector,
        angle_rad: float,
    ) -> GSolid: ...
