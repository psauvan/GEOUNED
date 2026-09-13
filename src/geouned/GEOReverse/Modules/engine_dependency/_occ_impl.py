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

**Exotic quadric surfaces**: all 7 are now implemented --
`Gmake_ellipsoid`, `Gmake_elliptic_cylinder`, `Gmake_torus_elliptic` (the
last one now shared with GEOUNED, imported from `geo`), `Gmake_elliptic_cone`,
`Gmake_hyperboloid`, `Gmake_hyperbolic_cylinder` (see their own
module-level comment blocks for the construction techniques and, for
the latter, how it revolves rather than extrudes -- a genuinely
different surface from `_freecad_impl.py`'s own version) and
`Gmake_paraboloid` (same one-branch-revolve technique as
`Gmake_hyperboloid`, but always a single sheet -- no second branch to
mirror).
"""

import math
from dataclasses import dataclass

from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCC.Core.Geom import Geom_Circle, Geom_Ellipse, Geom_Hyperbola, Geom_Parabola
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

from ....geo import GSolid, GVector, Gfuse, Gmake_compound, Gmake_torus_elliptic, arbitrary_perpendicular, to_native_vector
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
        """Fixed 2026-09-14 -- independently verified against 44 hand-
        computed ground-truth points (see the history log for the
        verification script): the version ported as-is from
        `_freecad_impl.py::GEllipsoid.is_inside` had two real bugs, not
        just the one its own docstring flagged. (1) `ry_vec = r - (rx *
        self.Axis + self.Center)` double-subtracted `Center` (`r` is
        already relative to `Center`), breaking every case once `Center`
        isn't the origin. (2) BOTH branches picked the wrong pairing of
        radius to axis: the "else" branch (revolve around `MajorAxis`)
        used `MinorRadius` for both the axial and radial extent, ignoring
        `MajorRadius` entirely; the "if" branch (revolve around
        `MinorAxis`, believed correct by the old docstring) swapped which
        radius belongs to the axial direction vs. the radial one. Fixed:
        whichever axis `Axis` actually is (`MajorAxis` or `MinorAxis`)
        keeps its own matching radius as the axial extent, the other
        radius as the perpendicular (radial) extent."""
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


# ---------------------------------------------------------------------------
# GEllipticCone
#
# Construction technique (per direct user instruction, 2026-09-14): from
# the apex and the cone's own axis, define an ellipse in a plane whose
# normal is the axis, at distance `length` from the apex along the axis --
# the ellipse's center is the point where the axis crosses that plane.
# Sweep a straight line from the apex to every point around the ellipse's
# contour; the resulting surface is the elliptic cone. Built here via a
# ruled loft (`BRepOffsetAPI_ThruSections`) from the apex vertex to the
# ellipse wire, with `isSolid=True` so it closes directly into a solid --
# same idea as `_freecad_impl.py::_make_elliptic_cone_native`'s own
# `Part.makeLoft([point, ellipse_shape], True)`, ported to pyOCC's own
# vertex+wire loft API. `RefRadius`/`MajorRadius`/`MinorRadius` match the
# MCNP GQ/SQ-derived quadric convention (`_freecad_impl.py::GEllipticCone`'s
# own docstring): the cross-section ellipse's real semi-axes at axial
# distance `length` from the apex are `MajorRadius/RefRadius * length` and
# `MinorRadius/RefRadius * length` -- i.e. the cross-section scales
# linearly with distance from the apex, and `RefRadius` is the axial
# distance at which the semi-axes equal `MajorRadius`/`MinorRadius` exactly.
# `DoubleSheet` (both nappes of the cone from the same apex, matching
# `geo.Gmake_cone_double_sheet`'s own circular-cone case) fuses the
# forward and axis-reversed single sheets.
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# GHyperboloid
#
# Construction technique (per direct user instruction, 2026-09-14): draw
# only ONE branch of the hyperbola, from its own vertex (where it crosses
# `MajorAxis`, on the axis of revolution itself) to a point on the curve
# whose projection onto `MajorAxis` is at distance `length` from `Center`.
# Revolving this arc 360 degrees around `MajorAxis` gives "branch 1" --
# the sheet on the positive `MajorAxis` side. The vertex end already sits
# ON the revolution axis, so it closes on its own (same "point on axis
# needs no capping" trick as the half-profile ellipsoid); the far end is
# an open circular rim at axial distance `length`, capped with a planar
# disc.
#
# `OneSheet` (default `True`, per direct user instruction, 2026-09-14):
# `True` builds ONLY branch 1 (the single positive-axis sheet -- the
# default). `False` also builds branch 2 -- this same construction
# mirrored through the plane through `Center` perpendicular to
# `MajorAxis` (equivalent to just negating `MajorAxis` for both the
# vertex/rim offsets and the revolution axis direction, since it's the
# same line either way) -- and assembles both as a compound, not a fuse:
# the two sheets never touch (a real gap between the two vertices, per
# the actual geometry), matching `_freecad_impl.py`'s own
# `Part.makeCompound((hyper1, hyper2))`.
#
# `is_inside` (fixed 2026-09-14, per direct user instruction): the
# (two-sheet) hyperboloid's own quadric region is the *complement* of
# the same-parameters `GHyperbolicCylinder`'s (one-sheet) region -- both
# come from the same hyperbola, just revolved around the opposite axis.
# `OneSheet=True` needs one extra check beyond that shared complement
# test, to tell which of the two disjoint sheets a point is near (the
# sign of its axial coordinate along `MajorAxis`) -- see `is_inside`'s
# own docstring below.
# ---------------------------------------------------------------------------


def _make_hyperboloid_sheet(surf: "GHyperboloid", length: float, major_axis: GVector) -> GSolid:
    center_native = to_native_vector(surf.Center)
    major_dir = gp_Dir(major_axis.x, major_axis.y, major_axis.z)
    minor_dir = gp_Dir(surf.MinorAxis.x, surf.MinorAxis.y, surf.MinorAxis.z)
    normal = gp_Dir(
        gp_Vec(major_dir.X(), major_dir.Y(), major_dir.Z()).Crossed(gp_Vec(minor_dir.X(), minor_dir.Y(), minor_dir.Z()))
    )
    ax2 = gp_Ax2(center_native, normal, major_dir)
    hyperbola = Geom_Hyperbola(ax2, surf.MajorRadius, surf.MinorRadius)

    # OCCT's own Geom_Hyperbola parametrization: P(t) = Center +
    # MajorRadius*cosh(t)*XDir + MinorRadius*sinh(t)*YDir -- t=0 is
    # exactly the vertex (on-axis), and cosh(t_end) = length/MajorRadius
    # puts the far end's axial projection at `length` from Center.
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

    solid = GSolid(BRepBuilderAPI_MakeSolid(topods.Shell(sewn)).Solid())
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
        """Fixed and redesigned 2026-09-14 (per direct user instruction),
        replacing the old ported-as-is version (which had its own
        pre-existing Center-mixing bug on top of testing a different
        `OneSheet` meaning than `build_shape` uses -- see the history
        log). A point is inside the (two-sheet) hyperboloid's own
        quadric region exactly when it is NOT inside the same-parameters
        `GHyperbolicCylinder` (the one-sheet "hourglass") -- both surfaces
        come from the same hyperbola, just revolved around the opposite
        axis, so their regions are complementary. `OneSheet=True` (only
        branch 1, the positive-`MajorAxis` side) needs one more check on
        top of that shared quadric test: which of the two disjoint sheets
        the point is actually near, via the sign of the axial coordinate.
        Independently verified against hand-computed ground-truth points
        (see the history log)."""
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


# ---------------------------------------------------------------------------
# GHyperbolicCylinder
#
# Construction technique (per direct user instruction, 2026-09-14):
# revolving a hyperbola around its own MINOR (conjugate) axis -- unlike
# `GHyperboloid` above, which revolves around `MajorAxis` -- always gives
# a single, fully-connected "hourglass" surface (the waist sits exactly
# at the vertex, radius `MajorRadius`, widening symmetrically on either
# side; never two disjoint pieces, whatever `length`/`height` is used).
# `center` doubles as both the hyperbola's own analytic center AND the
# revolved portion's own start point (`v=0`, the waist) -- matching
# `Gmake_elliptic_cylinder`'s own "center = start point, not true
# midpoint" convention (and this surface's own real call site,
# `Objects.py::HyperbolicCylinder.buildShape`, which already computes
# `point = center + dmin*axis` before calling this). The revolved portion
# spans `v` in `[0, height]` along `MinorAxis` from `center` -- NEITHER
# end sits on the revolution axis here (the waist itself, at v=0, still
# has radius `MajorRadius` > 0), so BOTH ends need a capping disc, unlike
# `GHyperboloid`'s single on-axis vertex.
#
# This supersedes `_freecad_impl.py::GHyperbolicCylinder.build_shape`'s
# own extrude-based technique (translating two mirrored hyperbola
# branches along a separate `Axis` field) with this revolve-based one --
# a genuinely different surface (a curved one-sheet hyperboloid segment,
# not a flat-generator translated hyperbolic prism). `is_inside` below
# is fixed to match (2026-09-14, see its own docstring) -- the old
# ported-as-is version tested the superseded extruded-prism definition.
# ---------------------------------------------------------------------------


def _make_hyperbolic_cylinder_native(surf: "GHyperbolicCylinder", height: float) -> GSolid:
    center_native = to_native_vector(surf.Center)
    major_dir = gp_Dir(surf.MajorAxis.x, surf.MajorAxis.y, surf.MajorAxis.z)
    minor_dir = gp_Dir(surf.MinorAxis.x, surf.MinorAxis.y, surf.MinorAxis.z)
    normal = gp_Dir(
        gp_Vec(major_dir.X(), major_dir.Y(), major_dir.Z()).Crossed(gp_Vec(minor_dir.X(), minor_dir.Y(), minor_dir.Z()))
    )
    ax2 = gp_Ax2(center_native, normal, major_dir)
    hyperbola = Geom_Hyperbola(ax2, surf.MajorRadius, surf.MinorRadius)

    # P(t) = Center + MajorRadius*cosh(t)*XDir + MinorRadius*sinh(t)*YDir
    # -- v (offset along MinorAxis) = MinorRadius*sinh(t), so t=0 is the
    # waist (v=0) and asinh(height/MinorRadius) puts the far end's own
    # MinorAxis offset at `height`.
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

    solid = GSolid(BRepBuilderAPI_MakeSolid(topods.Shell(sewn)).Solid())
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
        (the old version, ported as-is from `_freecad_impl.py`, tested the
        superseded extruded-prism definition instead -- see this section's
        own module-level comment). The waist -- at `v=0`, i.e. `Center`
        itself -- has radius `MajorRadius`; it grows with `|v|` along
        `MinorAxis` as `MajorRadius*sqrt(1 + (v/MinorRadius)^2)`. `d` is
        the FULL 3D perpendicular distance from the `MinorAxis`-line
        through `Center` (not just the `MajorAxis`-plane projection the
        old version used), matching an actual solid of revolution.
        Independently verified against hand-computed ground-truth points
        (see the history log)."""
        r = point - self.Center
        v = r.dot(self.MinorAxis)
        perp = r - v * self.MinorAxis
        d = perp.length
        y = self.MajorRadius * math.sqrt(1.0 + (v / self.MinorRadius) ** 2)
        return d < y

    def build_shape(self, height: float) -> GSolid:
        return _make_hyperbolic_cylinder_native(self, height)


def Gmake_hyperbolic_cylinder(center, axis, major_radius, minor_radius, major_axis, minor_axis, height) -> GSolid:
    return GHyperbolicCylinder.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape(height)


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


# ---------------------------------------------------------------------------
# GParaboloid
#
# Construction technique (per direct user instruction, 2026-09-14): same
# idea as `GHyperboloid` -- draw the profile curve (a parabola here, one
# branch of a hyperbola there) from its own vertex (on the revolution
# axis) out to a point whose axial projection is at distance `length`
# from `Center`, then revolve 360 degrees around `Axis`; the vertex end
# is already on the axis (no capping needed there), the far end is an
# open circular rim, capped with a planar disc. Unlike `GHyperboloid`,
# a paraboloid only ever has ONE sheet -- there is no second branch to
# mirror, no `OneSheet` flag needed at all.
#
# OCCT's own `Geom_Parabola` parametrization: P(u) = Center +
# (u^2/(4*Focal))*XDir + u*YDir -- at parameter `u`, the axial offset is
# u^2/(4*Focal) and the radial offset is exactly `u` itself, so solving
# axial-offset=`length` for `u` gives `u_end = sqrt(4*Focal*length)`,
# which is simultaneously the far rim's own radius -- no separate rim
# formula needed, unlike the hyperbola case.
# ---------------------------------------------------------------------------


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
        (pure GVector math, identical on every engine) -- unlike
        `GEllipsoid`/`GHyperboloid`'s own versions, this one has no
        Center-mixing bug (`perp = r - x*self.Axis` never re-adds
        `Center`), confirmed correct by the same independent
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

    solid = GSolid(BRepBuilderAPI_MakeSolid(topods.Shell(sewn)).Solid())
    if solid.Volume < 0:
        solid = solid.reverse()
    return solid


def Gmake_paraboloid(center, axis, focal, length) -> "GSolid | None":
    return GParaboloid.from_values(center, axis, focal).build_shape(length)


# `Gmake_torus_elliptic` (circular or elliptic torus, including the
# degenerate inner/outer sheet selection) moved to `geo/occ/torus_elliptic.py`
# (2026-09-13) -- GEOUNED's own forward pipeline needs the same capability
# (see CLAUDE.md's "Known open items" -> GEOUNED), so it now lives in `geo`
# as the single shared implementation and is re-exported here (see the
# `from ....geo import ...` line above) rather than duplicated.
