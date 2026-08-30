"""
geo/freecad/primitives.py

Primitive construction (Gmake_box/_cylinder/_cone/.../_compound) --
used both by CsgToCad/GEOReverse and by GEOUNED's own meta-surface
assembly, which builds RoundCorner/Can/TCone/MultiPlane/... by
cutting/fusing these analytic primitives itself.
"""

from __future__ import annotations

import math

import FreeCAD
import Part

from .topology import GEdge, GFace, GPlane, GShell, GSolid, GWire
from ._native_utils import to_native_vector
from .boolean import Gfuse
from ..vector_geometry import GVector


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
    native = Part.makeCylinder(radius, height, to_native_vector(point), to_native_vector(axis))
    return GSolid(native)


def Gmake_cone(apex: GVector, axis: GVector, half_angle: float, height: float) -> GSolid:
    base_radius = height * math.tan(abs(half_angle))
    native = Part.makeCone(0.0, base_radius, height, to_native_vector(apex), to_native_vector(axis))
    return GSolid(native)


def Gmake_cone_frustum(point: GVector, axis: GVector, radius1: float, radius2: float, height: float) -> GSolid:
    """
    Truncated cone (frustum): `radius1` at `point`, `radius2` at
    `point + height*axis` -- unlike `Gmake_cone`, neither end needs to be
    zero (a real apex). Needed by CsgToCad (GEOReverse) for MCNP's
    truncated-cone surface form, which `Gmake_cone`'s single-apex shape has
    no way to represent.
    """
    native = Part.makeCone(radius1, radius2, height, to_native_vector(point), to_native_vector(axis))
    return GSolid(native)


def Gmake_cone_double_sheet(apex: GVector, axis: GVector, half_angle: float, length: float) -> GSolid:
    """
    Both nappes of an infinite (quadric) cone, built as one fused solid --
    `Gmake_cone` only ever builds a single nappe. `length` bounds how far
    each nappe extends from `apex` along `+axis`/`-axis`, same convention
    as `Gmake_cone`'s own `height`. Needed by CsgToCad (GEOReverse) for
    MCNP's double-sheet quadric cone form.
    """
    sheet1 = Gmake_cone(apex, axis, half_angle, length)
    sheet2 = Gmake_cone(apex, -axis, half_angle, length)
    fused = Gfuse([sheet1, sheet2])
    return fused.refine()


def Gmake_sphere(center: GVector, radius: float) -> GSolid:
    native = Part.makeSphere(radius, to_native_vector(center))
    return GSolid(native)


def Gmake_torus(center: GVector, axis: GVector, major_radius: float, minor_radius: float) -> GSolid:
    native = Part.makeTorus(major_radius, minor_radius, to_native_vector(center), to_native_vector(axis))
    return GSolid(native)


def Gmake_half_space(plane: GPlane) -> GSolid:
    """Half-space bounded by an infinite plane (internally clipped to a working box). Needed to reconstruct CSG cells defined by the intersection of half-spaces."""
    extent = 1.0e6
    box = Part.makeBox(extent, extent, extent, FreeCAD.Vector(-extent / 2.0, -extent / 2.0, -extent))
    normal = to_native_vector(plane.Axis.normalized())
    box.Placement = FreeCAD.Placement(
        to_native_vector(plane.Position),
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
    native = Part.Face(Part.makePolygon([to_native_vector(p) for p in points], True))
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


def Gmake_solid(shell: GShell) -> "GSolid | None":
    """Close a watertight shell into a real solid (a genuine enclosed
    volume), for callers that built a shell face-by-face (e.g. clipping a
    box by successive cutting planes) and need a proper GSolid out of it
    -- as opposed to Gmake_shell's own callers, which only need the faces
    addressable as a group, not a valid volume. Returns None if the shell
    isn't actually closed/well-formed enough to bound a solid (the
    caller's own responsibility to fall back sensibly, matching the
    None-for-degenerate-input convention used elsewhere in this module,
    e.g. Gmake_polygon_face's own plane_polygon_from_box caller)."""
    try:
        return GSolid(Part.makeSolid(shell.__native__))
    except Exception:
        return None


def Gmake_compound(shapes: list[GSolid]) -> GSolid:
    """Group `shapes` into a single compound shape, with no boolean operation applied (they may overlap or be disjoint). Used as a last-resort fallback when Gfuse fails or produces an invalid result."""
    return GSolid(Part.makeCompound([s.__native__ for s in shapes]))
