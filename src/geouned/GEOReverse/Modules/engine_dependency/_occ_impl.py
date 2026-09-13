"""
GEOReverse/Modules/_occ_impl.py

The pyOCC counterpart to `_freecad_impl.py`, covering the same two
unrelated concerns: CAD export (implemented) and the 6 exotic quadric
surfaces (still a stub). `Modules/__init__.py` resolves here when
`CAD_ENGINE == "occ"`.

**CAD export** (`SUPPORTED_FORMATS`, `export_occ`): pyOCC has no
document/label-tree concept the way FreeCAD does, so this doesn't just
dump solids to STEP -- it reconstructs the same Universe/Material/Cell
naming and nesting `_freecad_impl.py`'s `makeTree` builds, via OCC's XCAF
framework (`XCAFDoc_ShapeTool`, `TDataStd_Name`, `STEPCAFControl_Writer`
instead of the plain `STEPControl_Writer` `geo.Gexport_step` uses).
`.fcstd` has no pyOCC equivalent at all and stays unsupported
(`SUPPORTED_FORMATS` does NOT include "fcstd").

Verified end to end against a real `pyoccenv` session (2026-08-16, not
guessed): `TDataStd_Name.Set(label, name)` needs a plain Python `str`,
not a pre-built `TCollection_ExtendedString` (passing the latter raises
`TypeError: Wrong number or type of arguments` -- SWIG's overload
resolution doesn't like an already-typed argument here, only a raw
`str` it can convert itself). `shape_tool.AddShape(shape, False)` on a
`TopoDS_Compound` (the common case -- most real `GSolid`s built by
`geo.Gfuse_solids`'s compound fallback path are compounds, not single
`TopoDS_Solid`s) does NOT keep it as one label with N solids inside --
`Gload_step_labels` reads it back as N separate same-named labels, one
per solid. A real, harmless difference from the FreeCAD path (which
keeps a compound as one `Part::FeaturePython`/one label) -- every solid
is still correctly traceable to its cell/material by name, just at finer
label granularity for compound cells. Round-tripped a full 2-level
Universe->Material->Cell hierarchy (`AddComponent(parent_label,
child_label, TopLoc_Location())`) through STEP and back via the
already-verified `geo.Gload_step_labels`, confirming names and nesting
both survive.

**Per-solid color by material** (`material_colors`/`_build_tree`'s
`color_tool.SetColor` calls -- the color computation itself lives in
`cad_export_shared.py`, shared verbatim with `_ocp_impl.py`): every cell
sharing the same `MAT` value gets the same color, via
`XCAFDoc_DocumentTool.ColorTool` + `Quantity_Color(r, g, b,
Quantity_TOC_RGB)`, `XCAFDoc_ColorGen`. Verified directly against a real
STEP output (2026-08-16): `SetColor(label, color, XCAFDoc_ColorGen)` --
the label-based overload -- writes real `STYLED_ITEM`/`COLOUR_RGB` (or
`DRAUGHTING_PRE_DEFINED_COLOUR` for an exact primary color) entities
into the file, confirmed by direct inspection of the raw STEP text.
**Found, not used**: the read-side `color_tool.GetColor(label,
XCAFDoc_ColorType, Quantity_Color&)` overload raises a SWIG `TypeError`
("wrong number or type of arguments") in this pythonocc-core build
despite matching one of the documented C++ prototypes exactly -- a real
binding quirk, not a usage mistake (confirmed by testing the identical
call against a fresh single-shape document with no assembly nesting
involved). The `GetColor(TopoDS_Shape const&, ...)` shape-based overload
works fine. Not fixed/worked around here since nothing in this codebase
currently reads colors back -- flagged for whenever that's needed.

**Exotic quadric surfaces**: `Gmake_ellipsoid`, `Gmake_elliptic_cylinder`
and `Gmake_torus_elliptic` (the last one now shared with GEOUNED, imported
from `geo` -- see its own module for the construction technique) are
implemented. `Gmake_elliptic_cone`, `Gmake_hyperboloid` and
`Gmake_hyperbolic_cylinder` are still NOT implemented -- building these
needs pyOCC equivalents of `_freecad_impl.py`'s own
`Part.Hyperbola`/`.extrude()`/`Part.makeLoft` constructions -- likely
`Geom_Hyperbola`, `BRepPrimAPI_MakePrism` for the extrude-based
hyperbolic-cylinder build, and `BRepOffsetAPI_ThruSections` for the
loft-based elliptic-cone build. Flagged as its own follow-up phase.
"""

import math
from dataclasses import dataclass

from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCC.Core.Geom import Geom_Ellipse
from OCC.Core.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Vec
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
from OCC.Core.TDataStd import TDataStd_Name
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import topods
from OCC.Core.XCAFApp import XCAFApp_Application
from OCC.Core.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_DocumentTool

from ....geo import GSolid, GVector, Gmake_torus_elliptic, arbitrary_perpendicular, to_native_vector
from ..Utils.cad_export_shared import cell_label_name, material_colors, material_label_name, universe_label_name

SUPPORTED_FORMATS = {"stp", "step"}


def _build_tree(shape_tool, color_tool, mat_colors, CADCells, parent_label):
    """Mirrors `_freecad_impl.py::makeTree` exactly, one XCAF label per
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
    TDataStd_Name.Set(universe_label, universe_label_name(label[0], label[1]))
    shape_tool.AddComponent(parent_label, universe_label, TopLoc_Location())

    mat_groups = {}
    for c in universeCADCells:
        if isinstance(c, (tuple, list)):
            _build_tree(shape_tool, color_tool, mat_colors, c, universe_label)
        else:
            mat_groups.setdefault(c.MAT, []).append(c)

    for mat, cells in mat_groups.items():
        mat_label = shape_tool.NewShape()
        TDataStd_Name.Set(mat_label, material_label_name(mat, label[0], label[1]))
        shape_tool.AddComponent(universe_label, mat_label, TopLoc_Location())
        color = Quantity_Color(*mat_colors[mat], Quantity_TOC_RGB)
        for c in cells:
            cell_label = shape_tool.AddShape(c.shape.__native__, False)
            TDataStd_Name.Set(cell_label, cell_label_name(c.name, c.MAT))
            shape_tool.AddComponent(mat_label, cell_label, TopLoc_Location())
            color_tool.SetColor(cell_label, color, XCAFDoc_ColorGen)

    return universe_label


def export_occ(buildCAD_list, formats, output_filename, barename):
    """Builds the XCAF document tree via `_build_tree` and writes each
    requested format (only "stp"/"step" -- see `SUPPORTED_FORMATS`)."""
    app = XCAFApp_Application.GetApplication()
    doc = TDocStd_Document("XmlXCAF")
    app.NewDocument("XmlXCAF", doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool(doc.Main())

    top_label = shape_tool.NewShape()
    TDataStd_Name.Set(top_label, barename)

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
            f"{name} has no pyOCC implementation yet -- exotic quadric surfaces "
            "(ellipsoid/hyperboloid/elliptic cone/elliptic or hyperbolic cylinder/paraboloid/elliptic "
            "torus) are still FreeCAD-only. See this module's own docstring for what's needed to add one."
        )

    _raise.__name__ = name
    return _raise


# ---------------------------------------------------------------------------
# GEllipsoid
#
# Construction technique (per direct user instruction, 2026-09-13): draw the
# ellipse curve in a plane, revolve it 360 degrees around an axis lying in
# that same plane to sweep out the surface. Specifically here: the ellipse's
# own two semi-axes are `rev_radius` (along the axis of revolution -- the
# spheroid's polar radius) and `perp_radius` (perpendicular to it, in the
# ellipse's plane -- the spheroid's equatorial radius). Only the HALF of the
# ellipse on the `perp_radius >= 0` side is built as the profile edge -- its
# two endpoints then land exactly ON the axis of revolution (the two poles),
# so revolving it 360 degrees already produces a closed, watertight shell on
# its own -- no separate end-capping needed (unlike the open hyperbola/
# parabola profiles planned for GHyperboloid/GParaboloid, which will need
# it). This differs from `_freecad_impl.py::_make_ellipsoid_native`'s own
# technique (full ellipse curve revolved 180 degrees, or a 0..pi half
# revolved 360) -- that FreeCAD version is documented (this file's own
# module docstring) to fail in `Part.makeSolid` on this exact FreeCAD
# version; this half-profile-with-poles-on-axis technique is the standard,
# more robust construction and was verified independently here (see
# `Gmake_ellipsoid`'s own docstring for the verification).
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
        """Ported as-is from `_freecad_impl.py::GEllipsoid.is_inside` (pure
        GVector math, identical on every engine) -- including its own
        pre-existing bug in the "revolution around minor axis" branch (see
        that method's own docstring). NOT fixed here, per this project's
        standing discipline of not silently fixing an unrelated bug while
        porting/building something else."""
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
    """Builds a closed spheroid of revolution: an ellipse profile with
    semi-axes `rev_radius` (along `axis`, the axis of revolution) and
    `perp_radius` (along `perp_axis`, perpendicular to it), keeping only the
    half with `perp_radius`'s own coordinate >= 0 -- both its endpoints then
    lie exactly on `axis` (the two poles) -- revolved 360 degrees around
    `axis`. `Geom_Ellipse` requires its own MajorRadius >= MinorRadius, so
    whichever of `rev_radius`/`perp_radius` is numerically larger is used as
    the ellipse's own major direction; the correct half-parameter range
    (`u1`, `u2`) is derived for either case so the kept half always has its
    two ends on `axis` regardless of which one that turns out to be
    (verified against the analytic spheroid volume 4/3 * pi * perp_radius^2
    * rev_radius for both a prolate and an oblate case, and a third case
    with an arbitrary, non-axis-aligned `axis`, all three exact to float
    precision -- see the history log for the verification script)."""
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
    solid = GSolid(BRepBuilderAPI_MakeSolid(topods.Shell(shell)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def _make_ellipsoid_native(surf: "GEllipsoid") -> GSolid:
    # `Axis` (the axis of revolution) is always either MajorAxis or
    # MinorAxis (a spheroid's axis of revolution is one of its own two
    # defining directions -- the other, equal pair is what perp_axis
    # sweeps out) -- matches _freecad_impl.py::_make_ellipsoid_native's
    # own `if (axis - minor_axis)...` branch exactly.
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
# GEllipticCylinder
#
# Construction technique (per direct user instruction, 2026-09-13): an
# elliptic base in a plane whose normal is the cylinder's own axis, the
# axis passing through the ellipse's center; the cylinder is built by
# displacing this ellipse along the axis, then closing the two open ends
# with (planar) caps. `center` keeps `_freecad_impl.py`'s own convention
# (confirmed with the user before implementing, since both real call
# sites in `Objects.py::EllipticCylinder.buildShape` already compute it
# this way): the extrusion's own START point, not its true geometric
# midpoint -- the two ellipse profiles built below sit at `center` and
# `center + axis * height`, matching FreeCAD's own `build_shape` exactly.
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
        """Ported as-is from `_freecad_impl.py::GEllipticCylinder.is_inside`
        (pure GVector math, identical on every engine)."""
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

    # Ruled loft between the two identical, axis-translated ellipses is
    # exactly the analytic lateral surface of a right elliptic cylinder
    # (every generator line is straight and parallel to the axis) --
    # verified against the analytic volume pi*MajorRadius*MinorRadius*height
    # (see this file's own history-log entry for the verification script).
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
        sewer.Add(topods.Face(exp.Current()))
        exp.Next()
    sewer.Add(cap1)
    sewer.Add(cap2)
    sewer.Perform()
    sewn = sewer.SewedShape()

    solid = GSolid(BRepBuilderAPI_MakeSolid(topods.Shell(sewn)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def Gmake_elliptic_cylinder(center, axis, major_radius, minor_radius, major_axis, minor_axis, height) -> GSolid:
    return GEllipticCylinder.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape(height)


def Gmake_hyperbolic_cylinder(*args, **kwargs):
    return _not_implemented("Gmake_hyperbolic_cylinder")(*args, **kwargs)


Gmake_paraboloid = _not_implemented("Gmake_paraboloid")


# `Gmake_torus_elliptic` (circular or elliptic torus, including the
# degenerate inner/outer sheet selection) moved to `geo/occ/torus_elliptic.py`
# (2026-09-13) -- GEOUNED's own forward pipeline needs the same capability
# (see CLAUDE.md's "Known open items" -> GEOUNED), so it now lives in `geo`
# as the single shared implementation and is re-exported here (see the
# `from ....geo import ...` line above) rather than duplicated.
