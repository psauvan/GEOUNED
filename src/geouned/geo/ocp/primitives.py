"""
geo/ocp/primitives.py

Primitive construction (Gmake_box/_cylinder/_cone/.../_compound).
"""

from __future__ import annotations

import math

from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_Transform,
)
from OCP.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeRevol,
    BRepPrimAPI_MakeSphere,
    BRepPrimAPI_MakeTorus,
)
from OCP.Geom import Geom_Ellipse
from OCP.gp import (
    gp_Ax1,
    gp_Ax2,
    gp_Ax3,
    gp_Dir,
    gp_Pnt,
    gp_Trsf,
    gp_Vec,
)
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import (
    TopoDS,
    TopoDS_Compound,
    TopoDS_Shell,
)
from ..vector_geometry import GVector, arbitrary_perpendicular
from .topology import GEdge, GFace, GPlane, GShell, GSolid, GWire
from ._native_utils import to_native_vector
from .boolean import Gfuse


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


# `Gmake_torus_elliptic` -- circular or elliptic torus, including the
# degenerate case where the tube self-intersects the revolution axis and
# only one of the two resulting sheets ("outer" or "inner") is wanted.
# Moved here from `GEOReverse/Modules/engine_dependency/_ocp_impl.py`
# (2026-09-13), where it was first implemented and verified
# (`tests/test_georeverse_ocp_impl.py`) -- GEOUNED's own forward pipeline
# needs the same degenerate-sheet selection capability (see CLAUDE.md's
# "Known open items" -> GEOUNED), so it now lives here as the single
# shared implementation, re-exported back into GEOReverse rather than
# duplicated. Mirrors `geo/occ/primitives.py::Gmake_torus_elliptic`
# exactly -- see that file's own comment block for the full
# construction-technique writeup; only the OCP module paths and the
# `TopoDS.Shell(...)` cast (no `_s`-suffixed free function needed here)
# differ from the pythonocc-core version.
def Gmake_torus_elliptic(center, axis, major_radius, minor_radius_a, minor_radius_b, outer: "bool | None" = None) -> GSolid:
    """`major_radius` (MCNP's own `R`) is the tube center's distance from
    `center` along the perpendicular direction `arbitrary_perpendicular(axis)`
    picks. `minor_radius_a`/`minor_radius_b` are the tube's own elliptical
    cross-section radii: `minor_radius_a` along the *same* (radial)
    direction as `major_radius` itself, `minor_radius_b` along the torus
    axis direction -- `minor_radius_a > minor_radius_b` gives a flattened
    ("oblate") torus, `minor_radius_a < minor_radius_b` gives a torus
    elongated along its own axis, and `minor_radius_a == minor_radius_b`
    is the plain circular-section torus (`geo.Gmake_torus`'s own case).

    `outer` (only meaningful in the degenerate case, ignored otherwise)
    defaults to `None`, meaning: derive it from the *sign* of
    `major_radius` itself (`>= 0` -> outer, `< 0` -> inner) -- matching
    the round-trip convention already established on the forward
    (`CadToCsg`) side: `geo.surface_geometry.torus_sheet_sign` classifies
    a real degenerate-torus face's sheet as `GTorus.a_sign` (+1 outer, -1
    inner), and `GEOUNED/write/functions.py` writes it back out to
    MCNP/OpenMC/etc by negating the major radius for the inner sheet
    (`radMaj *= surf.a_sign`) rather than adding a format field that
    doesn't exist -- so a real MCNP/OpenMC file's own signed `R` already
    carries this distinction, and the caller doesn't need to compute
    anything extra: it can keep passing `Ra` straight through, sign
    included. Pass an explicit `True`/`False` to override. The magnitude
    `abs(major_radius)` is what's actually used for every geometric
    computation below -- the sign only ever selects `outer`."""
    R = abs(major_radius)
    if outer is None:
        outer = major_radius >= 0

    native_center = to_native_vector(center)
    z_dir = gp_Dir(axis.x, axis.y, axis.z)
    x_axis = arbitrary_perpendicular(axis)
    x_dir = gp_Dir(x_axis.x, x_axis.y, x_axis.z)

    # minor_radius_a pairs with the radial direction (x_dir, the same
    # direction major_radius/R itself is measured along), minor_radius_b
    # pairs with the torus axis direction (z_dir). Geom_Ellipse requires
    # its own MajorRadius >= MinorRadius, so swap for the API call while
    # tracking which physical direction each radius actually belongs to.
    ellipse_major_r, ellipse_minor_r = minor_radius_a, minor_radius_b
    ellipse_major_dir, ellipse_minor_dir = x_dir, z_dir
    if ellipse_major_r < ellipse_minor_r:
        ellipse_major_r, ellipse_minor_r = ellipse_minor_r, ellipse_major_r
        ellipse_major_dir, ellipse_minor_dir = ellipse_minor_dir, ellipse_major_dir

    e_center = to_native_vector(center + x_axis * R)

    normal = gp_Dir(
        gp_Vec(ellipse_major_dir.X(), ellipse_major_dir.Y(), ellipse_major_dir.Z()).Crossed(
            gp_Vec(ellipse_minor_dir.X(), ellipse_minor_dir.Y(), ellipse_minor_dir.Z())
        )
    )
    ax2 = gp_Ax2(e_center, normal, ellipse_major_dir)
    ellipse = Geom_Ellipse(ax2, ellipse_major_r, ellipse_minor_r)

    if R < minor_radius_a:
        pz = minor_radius_b * math.sqrt(1.0 - (R / minor_radius_a) ** 2)
        pz1 = to_native_vector(center - axis * pz)
        pz2 = to_native_vector(center + axis * pz)

        def _param(point: gp_Pnt) -> float:
            rel = GVector(point.X() - e_center.X(), point.Y() - e_center.Y(), point.Z() - e_center.Z())
            u = rel.dot(GVector(ellipse_major_dir.X(), ellipse_major_dir.Y(), ellipse_major_dir.Z()))
            v = rel.dot(GVector(ellipse_minor_dir.X(), ellipse_minor_dir.Y(), ellipse_minor_dir.Z()))
            return math.atan2(v / ellipse_minor_r, u / ellipse_major_r)

        p1 = _param(pz1) % (2.0 * math.pi)
        p2 = _param(pz2) % (2.0 * math.pi)
        if p2 < p1:
            p1, p2 = p2, p1
        if (p2 - p1) > math.pi:
            arc_long, arc_short = (p1, p2), (p2, p1 + 2.0 * math.pi)
        else:
            arc_short, arc_long = (p1, p2), (p2, p1 + 2.0 * math.pi)
        t_start, t_end = arc_long if outer else arc_short

        edge = BRepBuilderAPI_MakeEdge(ellipse, t_start, t_end).Edge()
    else:
        edge = BRepBuilderAPI_MakeEdge(ellipse).Edge()

    wire = BRepBuilderAPI_MakeWire(edge).Wire()
    shell = BRepPrimAPI_MakeRevol(wire, gp_Ax1(native_center, z_dir), 2.0 * math.pi).Shape()
    solid = GSolid(BRepBuilderAPI_MakeSolid(TopoDS.Shell(shell)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


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
            builder.Add(shell, TopoDS.Face(fexp.Current()))
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
