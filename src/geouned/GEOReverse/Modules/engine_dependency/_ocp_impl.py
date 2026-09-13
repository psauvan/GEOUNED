"""
GEOReverse/Modules/_ocp_impl.py

The OCP counterpart to `_freecad_impl.py`/`_occ_impl.py`, covering the
same two unrelated concerns: CAD export (implemented) and the 6 exotic
quadric surfaces (still a stub). `Modules/__init__.py` resolves here
when `CAD_ENGINE == "ocp"`. Mirrors `_occ_impl.py`'s structure and
naming exactly (see that file's own docstring for the full account of
*why* GEOReverse has an OCP backend at all -- a live benchmark found
pythonocc-core's SWIG bindings measurably slower per native call than
OCP's pybind11 ones).

**CAD export** (`SUPPORTED_FORMATS`, `export_ocp`): builds the same
Universe/Material/Cell XCAF label tree `_occ_impl.py::export_occ` does,
via OCP's own XCAF API -- which differs from pythonocc-core's in several
confirmed, load-bearing ways (see `geo/_ocp_impl.py`'s own docstring for
the general pattern; specifics for the XCAF calls used here):
- `TDataStd_Name.Set` needs a static-method `_s` suffix
  (`TDataStd_Name.Set_s`) and a real `TCollection_ExtendedString`, not a
  plain Python `str` (the reverse of pythonocc-core's own quirk, which
  wanted a plain str and rejected a pre-built `TCollection_ExtendedString`).
- `XCAFDoc_DocumentTool.ShapeTool`/`.ColorTool` need the `_s` suffix
  (`.ShapeTool_s`/`.ColorTool_s`).
- `XCAFApp_Application.GetApplication` needs the `_s` suffix
  (`.GetApplication_s`).
- `TDocStd_Document(...)`/`XCAFApp_Application.NewDocument(...)` both
  need a real `TCollection_ExtendedString`, not a plain `str`.
`.fcstd` has no OCP equivalent at all and stays unsupported
(`SUPPORTED_FORMATS` does NOT include "fcstd").

**Per-solid color by material**: shares `cad_export_shared.material_colors`
(and its palette/golden-angle machinery) verbatim with `_occ_impl.py` --
only the XCAF calls that apply the color differ in the ways above. Not
yet independently re-verified end to end against a real written STEP
file's raw text under OCP the way it was for pythonocc-core (see
`_occ_impl.py`'s own docstring) -- flagged for whenever that's checked.

**Exotic quadric surfaces**: all 7 are now implemented, mirroring
`_occ_impl.py` exactly -- `Gmake_ellipsoid`, `Gmake_elliptic_cylinder`,
`Gmake_torus_elliptic` (the last one now shared with GEOUNED, imported
from `geo`), `Gmake_elliptic_cone`, `Gmake_hyperboloid`,
`Gmake_hyperbolic_cylinder` and `Gmake_paraboloid` (see `_occ_impl.py`'s
own module-level comment blocks for the construction techniques).
"""

import math
from dataclasses import dataclass

from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCP.Geom import Geom_Circle, Geom_Ellipse, Geom_Hyperbola, Geom_Parabola
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_DocumentTool

from ....geo import GSolid, GVector, Gfuse, Gmake_compound, Gmake_torus_elliptic, arbitrary_perpendicular, to_native_vector
from ..Utils.cad_export_shared import cell_label_name, material_colors, material_label_name, universe_label_name

SUPPORTED_FORMATS = {"stp", "step"}


def _build_tree(shape_tool, color_tool, mat_colors, CADCells, parent_label):
    """Mirrors `_occ_impl.py::_build_tree` exactly, one XCAF label per
    FreeCAD `App::Part`/`Part::FeaturePython` object there: a
    `Universe_{U}_Container_{name}` assembly label, containing a
    `Material_{mat}_{U}{name}` assembly label per distinct material,
    each containing one `Cell_{name}_{MAT}`-named shape label per solid
    cell -- recursing into nested universes exactly where `makeTree`
    does (inline, as they're encountered), with material grouping
    collected across the whole list and added afterward, same as there.
    Each cell's shape label is colored by its own material, via
    `mat_colors` (see `cad_export_shared.material_colors`). Label text
    itself comes from `cad_export_shared`'s naming functions, shared
    with every other backend."""
    label, universeCADCells = CADCells
    universe_label = shape_tool.NewShape()
    TDataStd_Name.Set_s(universe_label, TCollection_ExtendedString(universe_label_name(label[0], label[1])))
    shape_tool.AddComponent(parent_label, universe_label, TopLoc_Location())

    mat_groups = {}
    for c in universeCADCells:
        if isinstance(c, (tuple, list)):
            _build_tree(shape_tool, color_tool, mat_colors, c, universe_label)
        else:
            mat_groups.setdefault(c.MAT, []).append(c)

    for mat, cells in mat_groups.items():
        mat_label = shape_tool.NewShape()
        TDataStd_Name.Set_s(mat_label, TCollection_ExtendedString(material_label_name(mat, label[0], label[1])))
        shape_tool.AddComponent(universe_label, mat_label, TopLoc_Location())
        color = Quantity_Color(*mat_colors[mat], Quantity_TOC_RGB)
        for c in cells:
            cell_label = shape_tool.AddShape(c.shape.__native__, False)
            TDataStd_Name.Set_s(cell_label, TCollection_ExtendedString(cell_label_name(c.name, c.MAT)))
            shape_tool.AddComponent(mat_label, cell_label, TopLoc_Location())
            color_tool.SetColor(cell_label, color, XCAFDoc_ColorGen)

    return universe_label


def export_ocp(buildCAD_list, formats, output_filename, barename):
    """Builds the XCAF document tree via `_build_tree` and writes each
    requested format (only "stp"/"step" -- see `SUPPORTED_FORMATS`)."""
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    app.NewDocument(TCollection_ExtendedString("XmlXCAF"), doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    top_label = shape_tool.NewShape()
    TDataStd_Name.Set_s(top_label, TCollection_ExtendedString(barename))

    mat_colors = material_colors(buildCAD_list)
    for CAD in buildCAD_list:
        _build_tree(shape_tool, color_tool, mat_colors, CAD, top_label)

    shape_tool.UpdateAssemblies()

    for fmt in formats:
        if fmt in ("stp", "step"):
            writer = STEPCAFControl_Writer()
            writer.Transfer(doc)
            filename = f"{output_filename}.{fmt}"
            status = writer.Write(filename)
            if status != IFSelect_RetDone:
                raise RuntimeError(f"STEP export failed for {filename} (status={status})")


# ---------------------------------------------------------------------------
# GEllipsoid -- mirrors _occ_impl.py::GEllipsoid exactly (same construction
# technique, same verification); only the OCP module paths and the
# TopoDS.Shell(...) cast (no `_s` suffix needed for that one, unlike
# TDataStd_Name.Set_s/etc. above -- see this file's own docstring) differ
# from the pythonocc-core version. See _occ_impl.py's own comment block
# right above its GEllipsoid for the full construction-technique writeup.
# ---------------------------------------------------------------------------


@dataclass
class GEllipsoid:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis) -> "GEllipsoid":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis)

    def is_inside(self, point: GVector) -> bool:
        """Fixed 2026-09-14 -- mirrors `_occ_impl.py::GEllipsoid.is_inside`
        exactly; see that file's own docstring for the two real bugs the
        old ported-as-is version had (a Center-double-subtraction and a
        swapped axial/radial radius pairing in both branches) and the
        independent verification."""
        r = point - self.Center
        rx = r.dot(self.Axis)
        perp = r - rx * self.Axis
        ry = perp.length

        if (self.Axis - self.MajorAxis).length < 1e-5:
            axial_radius, radial_radius = self.MajorRadius, self.MinorRadius
        else:
            axial_radius, radial_radius = self.MinorRadius, self.MajorRadius

        radical = 1 - (rx / axial_radius) ** 2
        if radical > 0:
            y = radial_radius * math.sqrt(radical)
            return ry < y
        return False

    def build_shape(self) -> GSolid:
        return _make_ellipsoid_native(self)


def _revolve_half_ellipse_to_solid(center, axis, rev_radius, perp_axis, perp_radius) -> GSolid:
    """See _occ_impl.py::_revolve_half_ellipse_to_solid's own docstring --
    identical technique, only the OCP TopoDS.Shell(...) cast differs."""
    native_center = to_native_vector(center)
    axis_dir = gp_Dir(axis.x, axis.y, axis.z)
    perp_dir = gp_Dir(perp_axis.x, perp_axis.y, perp_axis.z)

    if rev_radius >= perp_radius:
        big_dir, big_r = axis_dir, rev_radius
        small_dir, small_r = perp_dir, perp_radius
        u1, u2 = 0.0, math.pi
    else:
        big_dir, big_r = perp_dir, perp_radius
        small_dir, small_r = axis_dir, rev_radius
        u1, u2 = -math.pi / 2.0, math.pi / 2.0

    normal = gp_Dir(gp_Vec(big_dir.X(), big_dir.Y(), big_dir.Z()).Crossed(gp_Vec(small_dir.X(), small_dir.Y(), small_dir.Z())))
    ax2 = gp_Ax2(native_center, normal, big_dir)
    ellipse = Geom_Ellipse(ax2, big_r, small_r)
    edge = BRepBuilderAPI_MakeEdge(ellipse, u1, u2).Edge()
    wire = BRepBuilderAPI_MakeWire(edge).Wire()

    shell = BRepPrimAPI_MakeRevol(wire, gp_Ax1(native_center, axis_dir), 2.0 * math.pi).Shape()
    solid = GSolid(BRepBuilderAPI_MakeSolid(TopoDS.Shell(shell)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def _make_ellipsoid_native(surf: "GEllipsoid") -> GSolid:
    if (surf.Axis - surf.MinorAxis).length < 1e-5:
        rev_radius, perp_axis, perp_radius = surf.MinorRadius, surf.MajorAxis, surf.MajorRadius
    else:
        rev_radius, perp_axis, perp_radius = surf.MajorRadius, surf.MinorAxis, surf.MinorRadius
    return _revolve_half_ellipse_to_solid(surf.Center, surf.Axis, rev_radius, perp_axis, perp_radius)


# `GEllipticCone` -- mirrors `_occ_impl.py::GEllipticCone` exactly (same
# construction technique -- ruled loft from the apex vertex to the base
# ellipse wire at axial distance `length`, same MCNP GQ/SQ RefRadius
# scaling convention, same DoubleSheet fuse); only the OCP module paths
# differ from the pythonocc-core version. See `_occ_impl.py`'s own
# comment block for the full construction-technique writeup.
@dataclass
class GEllipticCone:
    Apex: GVector
    Axis: GVector
    RefRadius: float
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector
    DoubleSheet: bool = False

    @classmethod
    def from_values(
        cls, apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet=False
    ) -> "GEllipticCone":
        return cls(apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet)

    def is_inside(self, point: GVector) -> bool:
        """Ported as-is from `_freecad_impl.py::GEllipticCone.is_inside`
        (pure GVector math, identical on every engine)."""
        r = point - self.Apex
        x = r.dot(self.MajorAxis)
        y = r.dot(self.MinorAxis)
        z = r.dot(self.Axis)
        if self.DoubleSheet:
            z = abs(z)
        return (x / self.MajorRadius) ** 2 + (y / self.MinorRadius) ** 2 - z / self.RefRadius < 0

    def build_shape(self, length: float) -> GSolid:
        if not self.DoubleSheet:
            return _make_elliptic_cone_native(self, length, forward=True)
        sheet1 = _make_elliptic_cone_native(self, length, forward=True)
        sheet2 = _make_elliptic_cone_native(self, length, forward=False)
        fused = Gfuse([sheet1, sheet2])
        return fused.refine()


def _make_elliptic_cone_native(surf: "GEllipticCone", length: float, forward: bool) -> GSolid:
    axis_vec = surf.Axis if forward else -surf.Axis
    axis_dir = gp_Dir(axis_vec.x, axis_vec.y, axis_vec.z)
    xdir = gp_Dir(surf.MajorAxis.x, surf.MajorAxis.y, surf.MajorAxis.z)

    major_r = surf.MajorRadius / surf.RefRadius * length
    minor_r = surf.MinorRadius / surf.RefRadius * length
    ellipse_center = to_native_vector(surf.Apex + axis_vec * length)
    wire = _make_ellipse_wire(ellipse_center, axis_dir, xdir, major_r, minor_r)

    apex_vertex = BRepBuilderAPI_MakeVertex(to_native_vector(surf.Apex)).Vertex()

    lofter = BRepOffsetAPI_ThruSections(True, True)  # isSolid=True, ruled=True
    lofter.AddVertex(apex_vertex)
    lofter.AddWire(wire)
    lofter.Build()
    solid = GSolid(lofter.Shape())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def Gmake_elliptic_cone(
    apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet, length
) -> GSolid:
    return GEllipticCone.from_values(
        apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet
    ).build_shape(length)


# `GHyperboloid` -- mirrors `_occ_impl.py::GHyperboloid` exactly (same
# construction technique -- one branch revolved around MajorAxis from its
# on-axis vertex to a capped rim at axial distance `length`; `OneSheet`
# default `True` builds only that branch, `False` also mirrors it through
# Center for the second branch and assembles both as a compound, since
# the two sheets never touch); only the OCP module paths differ. See
# `_occ_impl.py`'s own comment block for the full construction-technique
# writeup, including its own `is_inside` fix (2026-09-14, same design here).


def _make_hyperboloid_sheet(surf: "GHyperboloid", length: float, major_axis: GVector) -> GSolid:
    center_native = to_native_vector(surf.Center)
    major_dir = gp_Dir(major_axis.x, major_axis.y, major_axis.z)
    minor_dir = gp_Dir(surf.MinorAxis.x, surf.MinorAxis.y, surf.MinorAxis.z)
    normal = gp_Dir(gp_Vec(major_dir.X(), major_dir.Y(), major_dir.Z()).Crossed(gp_Vec(minor_dir.X(), minor_dir.Y(), minor_dir.Z())))
    ax2 = gp_Ax2(center_native, normal, major_dir)
    hyperbola = Geom_Hyperbola(ax2, surf.MajorRadius, surf.MinorRadius)

    t_end = math.acosh(length / surf.MajorRadius)
    edge = BRepBuilderAPI_MakeEdge(hyperbola, 0.0, t_end).Edge()
    wire = BRepBuilderAPI_MakeWire(edge).Wire()
    shell = BRepPrimAPI_MakeRevol(wire, gp_Ax1(center_native, major_dir), 2.0 * math.pi).Shape()

    rim_radius = surf.MinorRadius * math.sqrt((length / surf.MajorRadius) ** 2 - 1.0)
    rim_center = to_native_vector(surf.Center + major_axis * length)
    cap = _make_disc_face(rim_center, major_dir, rim_radius)

    sewer = BRepBuilderAPI_Sewing(1e-6)
    sewer.Add(shell)
    sewer.Add(cap)
    sewer.Perform()
    sewn = sewer.SewedShape()

    solid = GSolid(BRepBuilderAPI_MakeSolid(TopoDS.Shell(sewn)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def _make_disc_face(center: gp_Pnt, normal_dir: gp_Dir, radius: float):
    """Shared by `GHyperboloid`'s rim cap and `GHyperbolicCylinder`'s two
    rim caps below -- a plain planar disc, normal `normal_dir`, radius
    `radius`, centered at `center`."""
    circle = Geom_Circle(gp_Ax2(center, normal_dir), radius)
    wire = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(circle).Edge()).Wire()
    return BRepBuilderAPI_MakeFace(wire).Face()


@dataclass
class GHyperboloid:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector
    OneSheet: bool = True

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet=True) -> "GHyperboloid":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet)

    def is_inside(self, point: GVector) -> bool:
        """Fixed and redesigned 2026-09-14 -- mirrors
        `_occ_impl.py::GHyperboloid.is_inside` exactly: a point is inside
        the (two-sheet) hyperboloid's own quadric region exactly when it
        is NOT inside the same-parameters `GHyperbolicCylinder` (both
        surfaces come from the same hyperbola, revolved around opposite
        axes, so their regions are complementary); `OneSheet=True` adds
        one more check -- the sign of the axial coordinate along
        `MajorAxis` -- to tell which of the two disjoint sheets a point
        is near. See `_occ_impl.py`'s own docstring for the full
        derivation and the independent verification."""
        cylinder_equivalent = GHyperbolicCylinder(
            self.Center, self.Axis, self.MajorRadius, self.MinorRadius, self.MajorAxis, self.MinorAxis
        )
        inside_quadric = not cylinder_equivalent.is_inside(point)
        if not self.OneSheet:
            return inside_quadric
        rx = (point - self.Center).dot(self.MajorAxis)
        return inside_quadric and rx > 0

    def build_shape(self, length: float) -> GSolid:
        sheet1 = _make_hyperboloid_sheet(self, length, self.MajorAxis)
        if self.OneSheet:
            return sheet1
        sheet2 = _make_hyperboloid_sheet(self, length, -self.MajorAxis)
        return Gmake_compound([sheet1, sheet2])


def Gmake_hyperboloid(center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet, length) -> GSolid:
    return GHyperboloid.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet).build_shape(
        length
    )


# `GHyperbolicCylinder` -- mirrors `_occ_impl.py::GHyperbolicCylinder`
# exactly (revolve around MinorAxis instead of MajorAxis -- always one
# connected "hourglass" sheet, waist at `center` itself, both ends
# capped since neither sits on the revolution axis; supersedes
# `_freecad_impl.py`'s own extrude-based technique with a genuinely
# different surface); only the OCP module paths differ. `is_inside` is
# fixed to match (2026-09-14, see its own docstring). See
# `_occ_impl.py`'s own comment block for the full writeup.


def _make_hyperbolic_cylinder_native(surf: "GHyperbolicCylinder", height: float) -> GSolid:
    center_native = to_native_vector(surf.Center)
    major_dir = gp_Dir(surf.MajorAxis.x, surf.MajorAxis.y, surf.MajorAxis.z)
    minor_dir = gp_Dir(surf.MinorAxis.x, surf.MinorAxis.y, surf.MinorAxis.z)
    normal = gp_Dir(gp_Vec(major_dir.X(), major_dir.Y(), major_dir.Z()).Crossed(gp_Vec(minor_dir.X(), minor_dir.Y(), minor_dir.Z())))
    ax2 = gp_Ax2(center_native, normal, major_dir)
    hyperbola = Geom_Hyperbola(ax2, surf.MajorRadius, surf.MinorRadius)

    t_end = math.asinh(height / surf.MinorRadius)
    edge = BRepBuilderAPI_MakeEdge(hyperbola, 0.0, t_end).Edge()
    wire = BRepBuilderAPI_MakeWire(edge).Wire()
    shell = BRepPrimAPI_MakeRevol(wire, gp_Ax1(center_native, minor_dir), 2.0 * math.pi).Shape()

    cap0 = _make_disc_face(center_native, minor_dir, surf.MajorRadius)
    rim_radius = surf.MajorRadius * math.sqrt(1.0 + (height / surf.MinorRadius) ** 2)
    rim_center = to_native_vector(surf.Center + surf.MinorAxis * height)
    cap1 = _make_disc_face(rim_center, minor_dir, rim_radius)

    sewer = BRepBuilderAPI_Sewing(1e-6)
    sewer.Add(shell)
    sewer.Add(cap0)
    sewer.Add(cap1)
    sewer.Perform()
    sewn = sewer.SewedShape()

    solid = GSolid(BRepBuilderAPI_MakeSolid(TopoDS.Shell(sewn)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


@dataclass
class GHyperbolicCylinder:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis) -> "GHyperbolicCylinder":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis)

    def is_inside(self, point: GVector) -> bool:
        """Fixed 2026-09-14 to match the revolve-based `build_shape` above
        -- mirrors `_occ_impl.py::GHyperbolicCylinder.is_inside` exactly;
        see that file's own docstring for the full derivation and the
        independent verification."""
        r = point - self.Center
        v = r.dot(self.MinorAxis)
        perp = r - v * self.MinorAxis
        d = perp.length
        y = self.MajorRadius * math.sqrt(1.0 + (v / self.MinorRadius) ** 2)
        return d < y

    def build_shape(self, height: float) -> GSolid:
        return _make_hyperbolic_cylinder_native(self, height)


def Gmake_hyperbolic_cylinder(center, axis, major_radius, minor_radius, major_axis, minor_axis, height) -> GSolid:
    return GHyperbolicCylinder.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape(
        height
    )


def Gmake_ellipsoid(center, axis, major_radius, minor_radius, major_axis, minor_axis) -> GSolid:
    return GEllipsoid.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape()


# ---------------------------------------------------------------------------
# GEllipticCylinder -- mirrors _occ_impl.py::GEllipticCylinder exactly (same
# construction technique, same `center` = extrusion start-point convention,
# same verification); only the OCP module paths and the TopoDS.Face/Shell
# casts (no `_s` suffix needed for those, see this file's own docstring)
# differ from the pythonocc-core version. See _occ_impl.py's own comment
# block for the full construction-technique writeup.
# ---------------------------------------------------------------------------


@dataclass
class GEllipticCylinder:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis) -> "GEllipticCylinder":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis)

    def is_inside(self, point: GVector) -> bool:
        r = point - self.Center
        x = r.dot(self.MajorAxis)
        y = r.dot(self.MinorAxis)
        return (x / self.MajorRadius) ** 2 + (y / self.MinorRadius) ** 2 - 1 < 0

    def build_shape(self, height: float) -> GSolid:
        return _make_elliptic_cylinder_native(self, height)


def _make_ellipse_wire(center: gp_Pnt, axis_dir: gp_Dir, xdir: gp_Dir, major_r: float, minor_r: float):
    ax2 = gp_Ax2(center, axis_dir, xdir)
    ellipse = Geom_Ellipse(ax2, major_r, minor_r)
    edge = BRepBuilderAPI_MakeEdge(ellipse).Edge()
    return BRepBuilderAPI_MakeWire(edge).Wire()


def _make_elliptic_cylinder_native(surf: "GEllipticCylinder", height: float) -> GSolid:
    axis_dir = gp_Dir(surf.Axis.x, surf.Axis.y, surf.Axis.z)
    xdir = gp_Dir(surf.MajorAxis.x, surf.MajorAxis.y, surf.MajorAxis.z)
    p1 = to_native_vector(surf.Center)
    p2 = to_native_vector(surf.Center + surf.Axis * height)

    wire1 = _make_ellipse_wire(p1, axis_dir, xdir, surf.MajorRadius, surf.MinorRadius)
    wire2 = _make_ellipse_wire(p2, axis_dir, xdir, surf.MajorRadius, surf.MinorRadius)

    lofter = BRepOffsetAPI_ThruSections(False, True)  # isSolid=False, ruled=True
    lofter.AddWire(wire1)
    lofter.AddWire(wire2)
    lofter.Build()
    lateral = lofter.Shape()

    cap1 = BRepBuilderAPI_MakeFace(wire1).Face()
    cap2 = BRepBuilderAPI_MakeFace(wire2).Face()

    sewer = BRepBuilderAPI_Sewing(1e-6)
    exp = TopExp_Explorer(lateral, TopAbs_FACE)
    while exp.More():
        sewer.Add(TopoDS.Face(exp.Current()))
        exp.Next()
    sewer.Add(cap1)
    sewer.Add(cap2)
    sewer.Perform()
    sewn = sewer.SewedShape()

    solid = GSolid(BRepBuilderAPI_MakeSolid(TopoDS.Shell(sewn)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def Gmake_elliptic_cylinder(center, axis, major_radius, minor_radius, major_axis, minor_axis, height) -> GSolid:
    return GEllipticCylinder.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape(height)


# `GParaboloid` -- mirrors `_occ_impl.py::GParaboloid` exactly (same
# construction technique -- one branch revolved around Axis from its
# on-axis vertex to a capped rim at axial distance `length`; always a
# single sheet, no OneSheet flag); only the OCP module paths differ. See
# `_occ_impl.py`'s own comment block for the full construction-technique
# writeup.


@dataclass
class GParaboloid:
    Center: GVector
    Axis: GVector
    Focal: float

    @classmethod
    def from_values(cls, center, axis, focal) -> "GParaboloid":
        return cls(center, axis, focal)

    def is_inside(self, point: GVector) -> bool:
        """Ported as-is from `_freecad_impl.py::GParaboloid.is_inside`
        (pure GVector math, identical on every engine) -- confirmed
        correct (no Center-mixing bug) by the same independent
        ground-truth verification used for the other surfaces."""
        r = point - self.Center
        x = r.dot(self.Axis)
        if x < 0:
            return False
        perp = r - x * self.Axis
        d = perp.length
        y = math.sqrt(4 * self.Focal * x)
        return d < y

    def build_shape(self, length: float) -> "GSolid | None":
        return _make_paraboloid_native(self, length)


def _make_paraboloid_native(surf: "GParaboloid", length: float) -> "GSolid | None":
    if length <= 0:
        return None

    center_native = to_native_vector(surf.Center)
    axis_dir = gp_Dir(surf.Axis.x, surf.Axis.y, surf.Axis.z)
    perp_axis = arbitrary_perpendicular(surf.Axis)
    perp_dir = gp_Dir(perp_axis.x, perp_axis.y, perp_axis.z)
    normal = gp_Dir(gp_Vec(axis_dir.X(), axis_dir.Y(), axis_dir.Z()).Crossed(gp_Vec(perp_dir.X(), perp_dir.Y(), perp_dir.Z())))
    ax2 = gp_Ax2(center_native, normal, axis_dir)
    parabola = Geom_Parabola(ax2, surf.Focal)

    u_end = math.sqrt(4.0 * surf.Focal * length)
    edge = BRepBuilderAPI_MakeEdge(parabola, 0.0, u_end).Edge()
    wire = BRepBuilderAPI_MakeWire(edge).Wire()
    shell = BRepPrimAPI_MakeRevol(wire, gp_Ax1(center_native, axis_dir), 2.0 * math.pi).Shape()

    rim_center = to_native_vector(surf.Center + surf.Axis * length)
    cap = _make_disc_face(rim_center, axis_dir, u_end)

    sewer = BRepBuilderAPI_Sewing(1e-6)
    sewer.Add(shell)
    sewer.Add(cap)
    sewer.Perform()
    sewn = sewer.SewedShape()

    solid = GSolid(BRepBuilderAPI_MakeSolid(TopoDS.Shell(sewn)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def Gmake_paraboloid(center, axis, focal, length) -> "GSolid | None":
    return GParaboloid.from_values(center, axis, focal).build_shape(length)


# `Gmake_torus_elliptic` (circular or elliptic torus, including the
# degenerate inner/outer sheet selection) moved to `geo/ocp/torus_elliptic.py`
# (2026-09-13) -- GEOUNED's own forward pipeline needs the same capability
# (see CLAUDE.md's "Known open items" -> GEOUNED), so it now lives in `geo`
# as the single shared implementation and is re-exported here (see the
# `from ....geo import ...` line above) rather than duplicated.
