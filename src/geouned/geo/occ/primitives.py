"""
geo/occ/primitives.py

Primitive construction (Gmake_box/_cylinder/_cone/.../_compound).
"""

from __future__ import annotations

import math

from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_Transform,
)
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
    BRepPrimAPI_MakeTorus,
)
from OCC.Core.gp import (
    gp_Ax2,
    gp_Ax3,
    gp_Dir,
    gp_Pnt,
    gp_Trsf,
)
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import (
    TopoDS_Compound,
    TopoDS_Shell,
    topods,
)
from ..vector_geometry import GVector
from .topology import GEdge, GFace, GPlane, GShell, GSolid, GWire
from ._native_utils import to_native_vector
from .boolean import Gfuse


# ---------------------------------------------------------------------------
# Primitive construction
# ---------------------------------------------------------------------------


def Gmake_box(xmin: float, ymin: float, zmin: float, xmax: float, ymax: float, zmax: float) -> GSolid:
    box = BRepPrimAPI_MakeBox(gp_Pnt(xmin, ymin, zmin), gp_Pnt(xmax, ymax, zmax)).Shape()
    return GSolid(box)


def Gmake_cylinder(point: GVector, axis: GVector, radius: float, height: float) -> GSolid:
    ax2 = gp_Ax2(to_native_vector(point), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeCylinder(ax2, radius, height).Shape()
    return GSolid(native)


def Gmake_cone(apex: GVector, axis: GVector, half_angle: float, height: float) -> GSolid:
    base_radius = height * math.tan(abs(half_angle))
    ax2 = gp_Ax2(to_native_vector(apex), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeCone(ax2, 0.0, base_radius, height).Shape()
    return GSolid(native)


def Gmake_cone_frustum(point: GVector, axis: GVector, radius1: float, radius2: float, height: float) -> GSolid:
    """Truncated cone (frustum): radius1 at `point`, radius2 at `point + height*axis`. See _freecad_impl.py's own docstring for why this is a separate function from Gmake_cone."""
    ax2 = gp_Ax2(to_native_vector(point), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeCone(ax2, radius1, radius2, height).Shape()
    return GSolid(native)


def Gmake_cone_double_sheet(apex: GVector, axis: GVector, half_angle: float, length: float) -> GSolid:
    """Both nappes of an infinite cone, fused into one solid. See _freecad_impl.py's own docstring."""
    sheet1 = Gmake_cone(apex, axis, half_angle, length)
    sheet2 = Gmake_cone(apex, -axis, half_angle, length)
    fused = Gfuse([sheet1, sheet2])
    return fused.refine()


def Gmake_sphere(center: GVector, radius: float) -> GSolid:
    native = BRepPrimAPI_MakeSphere(to_native_vector(center), radius).Shape()
    return GSolid(native)


def Gmake_torus(center: GVector, axis: GVector, major_radius: float, minor_radius: float) -> GSolid:
    ax2 = gp_Ax2(to_native_vector(center), gp_Dir(axis.x, axis.y, axis.z))
    native = BRepPrimAPI_MakeTorus(ax2, major_radius, minor_radius).Shape()
    return GSolid(native)


def Gmake_half_space(plane: GPlane) -> GSolid:
    """Half-space bounded by an infinite plane (internally clipped to a
    huge working box, matching _freecad_impl.py's own box-based approach
    rather than pyOCC's face+reference-point BRepPrimAPI_MakeHalfSpace,
    to keep this function's signature -- a GPlane, no reference point --
    identical across backends)."""
    extent = 1.0e6
    box = BRepPrimAPI_MakeBox(gp_Pnt(-extent / 2.0, -extent / 2.0, 0.0), gp_Pnt(extent / 2.0, extent / 2.0, extent)).Shape()
    normal = plane.Axis.normalized()
    target = gp_Dir(normal.x, normal.y, normal.z)
    ax3_from = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    ax3_to = gp_Ax3(to_native_vector(plane.Position), target)
    trsf = gp_Trsf()
    trsf.SetTransformation(ax3_to, ax3_from)
    moved = BRepBuilderAPI_Transform(box, trsf, True).Shape()
    return GSolid(moved)


def Gmake_wire(edges: list[GEdge]) -> GWire:
    maker = BRepBuilderAPI_MakeWire()
    for edge in edges:
        maker.Add(edge.__native__)
    return GWire(maker.Wire())


def Gmake_polygon_face(points: list[GVector]) -> GFace:
    poly = BRepBuilderAPI_MakePolygon()
    for p in points:
        poly.Add(to_native_vector(p))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return GFace(face)


def Gmake_shell(faces: list[GFace]) -> GShell:
    sewer = BRepBuilderAPI_Sewing(1e-6)
    for face in faces:
        sewer.Add(face.__native__)
    sewer.Perform()
    sewed = sewer.SewedShape()
    builder = BRep_Builder()
    shell = TopoDS_Shell()
    builder.MakeShell(shell)
    fexp = TopExp_Explorer(sewed, TopAbs_FACE)
    if fexp.More():
        while fexp.More():
            builder.Add(shell, topods.Face(fexp.Current()))
            fexp.Next()
    else:
        for face in faces:
            builder.Add(shell, face.__native__)
    return GShell(shell, faces=faces)


def Gmake_solid(shell: GShell) -> "GSolid | None":
    """Close a watertight shell into a real solid (a genuine enclosed
    volume), for callers that built a shell face-by-face (e.g. clipping a
    box by successive cutting planes) and need a proper GSolid out of it
    -- as opposed to Gmake_shell's own callers, which only need the faces
    addressable as a group, not a valid volume. Returns None if the shell
    isn't actually closed/well-formed enough to bound a solid."""
    try:
        solid_maker = BRepBuilderAPI_MakeSolid(shell.__native__)
        if solid_maker.IsDone():
            return GSolid(solid_maker.Solid())
    except Exception:
        pass
    return None


def Gmake_compound(shapes: list[GSolid]) -> GSolid:
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for s in shapes:
        builder.Add(compound, s.__native__)
    return GSolid(compound)
