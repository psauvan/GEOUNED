"""
geometry_backend/interface.py

Neutral interface (Adapter / Ports & Adapters pattern) to decouple GEOUNED
from its underlying geometry engine (FreeCAD Part / pythonOCC).

Design principle:
- The rest of GEOUNED (decomposition, surfaces, io...) NEVER imports
  `Part`, `FreeCAD`, or `OCC.Core` directly.
- It only knows these neutral types (GSolid, GFace, GEdge, GWire, GVertex,
  SurfaceGeometry, EdgeGeometry...) and the `GeometryBackend` interface.
- Each concrete backend (FreeCADBackend, OCCBackend) acts as a translator
  between these neutral types and the real objects of its library.
- The backend only knows about the 5 analytic surface types (plane,
  cylinder, cone, sphere, torus) and the 4 analytic curve types (line,
  circle, ellipse, bspline). Composite/meta-surfaces (RoundCorner, Can,
  TCone, MultiPlane, ReversedConeCyl...) are NOT a backend concept: they
  are built by GEOUNED itself, by combining backend-constructed analytic
  primitives with boolean operations (cut/common/fuse). The backend never
  needs to know a meta-surface exists.

The GSolid/GFace/GEdge/GWire/GVertex types are lightweight wrappers that
only carry an opaque reference (`native`) to the backend's real object,
plus the backend that produced it. GEOUNED code must never do
`isinstance(x, Part.Shape)` or similar: if it needs to do something with
the object, it asks the backend to do it.

`GVector` is the one exception: it is a plain value type with its own
arithmetic (dot, cross, length, normalize...) and does not depend on the
backend at all. Higher-level geometric predicates built on top of it
(parallel, opposite, in-line, in-plane...) live in `vector_geometry.py`,
not in this ABC.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
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

    def __neg__(self) -> "GVector":
        return GVector(-self.x, -self.y, -self.z)

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


# ---------------------------------------------------------------------------
# Neutral types (opaque wrappers)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GSolid:
    """Opaque wrapper around a backend-native solid."""
    native: Any
    backend: "GeometryBackend"


@dataclass(frozen=True)
class GFace:
    native: Any
    backend: "GeometryBackend"


@dataclass(frozen=True)
class GWire:
    native: Any
    backend: "GeometryBackend"


@dataclass(frozen=True)
class GEdge:
    native: Any
    backend: "GeometryBackend"


@dataclass(frozen=True)
class GVertex:
    native: Any
    backend: "GeometryBackend"


# A shape-like argument accepted by generic spatial queries (in_contact...).
GShape = Union[GSolid, GFace, GEdge]


# ---------------------------------------------------------------------------
# Surface classification and parameters
# (the 5 analytic surface types the backend must know about; composite
# meta-surfaces are assembled by GEOUNED from these, not modeled here)
# ---------------------------------------------------------------------------

class SurfaceType(Enum):
    PLANE = auto()
    CYLINDER = auto()
    CONE = auto()
    SPHERE = auto()
    TORUS = auto()
    UNKNOWN = auto()


@dataclass(frozen=True)
class PlaneParams:
    point: GVector
    normal: GVector


@dataclass(frozen=True)
class CylinderParams:
    axis_point: GVector
    axis_dir: GVector
    radius: float


@dataclass(frozen=True)
class ConeParams:
    apex: GVector
    axis_dir: GVector
    half_angle: float  # radians


@dataclass(frozen=True)
class SphereParams:
    center: GVector
    radius: float


@dataclass(frozen=True)
class TorusParams:
    center: GVector
    axis_dir: GVector
    major_radius: float
    minor_radius: float


# Simple discriminated union: the caller checks `surface_type` and casts
# `params` to the matching type. (If preferred, this can instead be
# modeled with Python 3.10+ structural pattern matching, or subclasses.)
@dataclass(frozen=True)
class SurfaceGeometry:
    surface_type: SurfaceType
    params: PlaneParams | CylinderParams | ConeParams | SphereParams | TorusParams | None


# ---------------------------------------------------------------------------
# Curve classification and parameters
# (the 4 analytic curve types the backend must know about, needed for
# wire/edge handling in decomposition and GEOReverse)
# ---------------------------------------------------------------------------

class CurveType(Enum):
    LINE = auto()
    CIRCLE = auto()
    ELLIPSE = auto()
    BSPLINE = auto()
    UNKNOWN = auto()


@dataclass(frozen=True)
class LineParams:
    point: GVector
    direction: GVector


@dataclass(frozen=True)
class CircleParams:
    center: GVector
    axis_dir: GVector
    radius: float


@dataclass(frozen=True)
class EllipseParams:
    center: GVector
    axis_dir: GVector
    major_axis_dir: GVector
    major_radius: float
    minor_radius: float


@dataclass(frozen=True)
class BSplineParams:
    poles: list[GVector]


@dataclass(frozen=True)
class EdgeGeometry:
    curve_type: CurveType
    params: LineParams | CircleParams | EllipseParams | BSplineParams | None


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

    # -- I/O ------------------------------------------------------------

    @abstractmethod
    def load_step(self, filename: str) -> list[GSolid]:
        """Load a STEP file and return the list of top-level solids."""

    @abstractmethod
    def export_step(self, solids: Sequence[GSolid], filename: str) -> None:
        """Export a list of solids to a STEP file."""

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
    def make_half_space(self, plane: PlaneParams) -> GSolid:
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

    @abstractmethod
    def get_faces(self, solid: GSolid) -> list[GFace]: ...

    @abstractmethod
    def get_edges(self, face: GFace) -> list[GEdge]: ...

    @abstractmethod
    def get_outer_wire(self, face: GFace) -> GWire:
        """Outer boundary wire of a face."""

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
    def get_vertex_point(self, vertex: GVertex) -> GVector: ...

    @abstractmethod
    def faces_sharing_edge(self, solid: GSolid, edge: GEdge) -> list[GFace]:
        """
        Number and list of faces sharing a given edge.
        A 'normal' edge in a closed solid is shared by 2 faces; >2 is
        the signal of a non-manifold condition (the tangency case we saw).
        """

    # -- Surface and curve classification -----------------------------
    # (the real core of the CAD -> CSG pipeline; restricted to the 5
    # analytic surface types and 4 analytic curve types -- composite
    # meta-surfaces are assembled above this layer, in GEOUNED itself)

    @abstractmethod
    def classify_surface(self, face: GFace) -> SurfaceGeometry:
        """
        Determine the underlying surface type of a face
        (plane/cylinder/cone/sphere/torus) and extract its native
        geometric parameters, already converted to the neutral types
        (PlaneParams, CylinderParams, ...).
        """

    @abstractmethod
    def classify_edge(self, edge: GEdge) -> EdgeGeometry:
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
    def face_value_at(self, face: GFace, u: float, v: float) -> GVector:
        """Point on `face` at parametric coordinates (u, v)."""

    @abstractmethod
    def face_normal_at(self, face: GFace, u: float, v: float) -> GVector:
        """Surface normal on `face` at parametric coordinates (u, v)."""

    @abstractmethod
    def tessellate(self, face: GFace, tolerance: float) -> list[GVector]:
        """Sampled points approximating `face`, within `tolerance`."""

    # -- Edge parametric queries -----------------------------------------
    # (only meaningful for the 4 analytic curve types above)

    @abstractmethod
    def edge_parameter_range(self, edge: GEdge) -> tuple[float, float]:
        """Returns (u_min, u_max), the valid domain for `edge`."""

    @abstractmethod
    def edge_value_at(self, edge: GEdge, u: float) -> GVector:
        """Point on `edge` at parametric coordinate `u`."""

    # -- Geometric properties -------------------------------------------------

    @abstractmethod
    def volume(self, solid: GSolid) -> float: ...

    @abstractmethod
    def area(self, face: GFace) -> float: ...

    @abstractmethod
    def bounding_box(
        self, solid: GSolid,
    ) -> tuple[float, float, float, float, float, float]:
        """Returns (xmin, ymin, zmin, xmax, ymax, zmax)."""

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
