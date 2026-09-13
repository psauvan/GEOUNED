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

**Exotic quadric surfaces**: `Gmake_ellipsoid`, `Gmake_elliptic_cylinder`
and `Gmake_torus_elliptic` (the last one now shared with GEOUNED, imported
from `geo`) are implemented, mirroring `_occ_impl.py` exactly.
`Gmake_elliptic_cone`, `Gmake_hyperboloid` and `Gmake_hyperbolic_cylinder`
are still NOT implemented here either -- same gap as `_occ_impl.py`, see
that file's own docstring for what would be needed.
"""

import math
from dataclasses import dataclass

from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCP.Geom import Geom_Ellipse
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

from ....geo import GSolid, GVector, Gmake_torus_elliptic, arbitrary_perpendicular, to_native_vector
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


def _not_implemented(name):
    def _raise(*args, **kwargs):
        raise NotImplementedError(
            f"{name} has no OCP implementation yet -- exotic quadric surfaces "
            "(ellipsoid/hyperboloid/elliptic cone/elliptic or hyperbolic cylinder/paraboloid/elliptic "
            "torus) are still FreeCAD-only. See this module's own docstring for what's needed to add one."
        )

    _raise.__name__ = name
    return _raise


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
        """Ported as-is from _freecad_impl.py::GEllipsoid.is_inside -- see
        that method's own docstring for the pre-existing bug it carries
        forward unfixed."""
        r = point - self.Center
        rx = r.dot(self.Axis)
        ry_vec = r - (rx * self.Axis + self.Center)
        ry = ry_vec.length

        if (self.Axis - self.MinorAxis).length < 1e-5:
            rad_x, rad_y = self.MajorRadius, self.MinorRadius
        else:
            rad_y = self.MinorRadius  # pre-existing bug: rad_x left undefined, ported as-is
            rad_x = rad_y

        radical = 1 - (rx / rad_x) ** 2
        if radical > 0:
            y = rad_y * math.sqrt(radical)
            return ry - y < 0
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


def Gmake_elliptic_cone(*args, **kwargs):
    return _not_implemented("Gmake_elliptic_cone")(*args, **kwargs)


def Gmake_hyperboloid(*args, **kwargs):
    return _not_implemented("Gmake_hyperboloid")(*args, **kwargs)


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


def Gmake_hyperbolic_cylinder(*args, **kwargs):
    return _not_implemented("Gmake_hyperbolic_cylinder")(*args, **kwargs)


Gmake_paraboloid = _not_implemented("Gmake_paraboloid")


# `Gmake_torus_elliptic` (circular or elliptic torus, including the
# degenerate inner/outer sheet selection) moved to `geo/ocp/torus_elliptic.py`
# (2026-09-13) -- GEOUNED's own forward pipeline needs the same capability
# (see CLAUDE.md's "Known open items" -> GEOUNED), so it now lives in `geo`
# as the single shared implementation and is re-exported here (see the
# `from ....geo import ...` line above) rather than duplicated.
