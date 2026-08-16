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
GEOReverse's own `fuse_solids` fallback path are compounds, not single
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

**Per-solid color by material** (`_material_colors`/`_build_tree`'s
`color_tool.SetColor` calls): every cell sharing the same `MAT` value
gets the same color, via `XCAFDoc_DocumentTool.ColorTool` +
`Quantity_Color(r, g, b, Quantity_TOC_RGB)`, `XCAFDoc_ColorGen`. Verified
directly against a real STEP output (2026-08-16): `SetColor(label, color,
XCAFDoc_ColorGen)` -- the label-based overload -- writes real
`STYLED_ITEM`/`COLOUR_RGB` (or `DRAUGHTING_PRE_DEFINED_COLOUR` for an
exact primary color) entities into the file, confirmed by direct
inspection of the raw STEP text. **Found, not used**: the read-side
`color_tool.GetColor(label, XCAFDoc_ColorType, Quantity_Color&)` overload
raises a SWIG `TypeError` ("wrong number or type of arguments") in this
pythonocc-core build despite matching one of the documented C++
prototypes exactly -- a real binding quirk, not a usage mistake (confirmed
by testing the identical call against a fresh single-shape document with
no assembly nesting involved). The `GetColor(TopoDS_Shape const&, ...)`
shape-based overload works fine. Not fixed/worked around here since
nothing in this codebase currently reads colors back -- flagged for
whenever that's needed.

**Exotic quadric surfaces** (`Gmake_elliptic_cone`, `Gmake_hyperboloid`,
`Gmake_ellipsoid`, `Gmake_elliptic_cylinder`, `Gmake_hyperbolic_cylinder`,
`Gmake_paraboloid`, `Gmake_torus_elliptic`): still NOT implemented --
building these needs pyOCC equivalents of `_freecad_impl.py`'s own
`Part.Ellipse`/`Part.Hyperbola`/`.revolve()`/`.extrude()`/`Part.makeLoft`
constructions -- likely `Geom_Ellipse`/`Geom_Hyperbola`,
`BRepPrimAPI_MakeRevol` for the revolve-based builds,
`BRepPrimAPI_MakePrism` for the extrude-based hyperbolic-cylinder build,
and `BRepOffsetAPI_ThruSections` for the loft-based
elliptic-cone/-cylinder builds. Flagged as its own follow-up phase.
"""

import colorsys

from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
from OCC.Core.TDataStd import TDataStd_Name
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.XCAFApp import XCAFApp_Application
from OCC.Core.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_DocumentTool

SUPPORTED_FORMATS = {"stp", "step"}

# matplotlib/D3 "tab10" -- the standard qualitative palette used across
# visualization tooling (matplotlib's own default, D3's category10) to
# give each of up to 10 categories a maximally distinguishable color.
_MATERIAL_PALETTE = [
    (0.121569, 0.466667, 0.705882),  # blue
    (1.000000, 0.498039, 0.054902),  # orange
    (0.172549, 0.627451, 0.172549),  # green
    (0.839216, 0.152941, 0.156863),  # red
    (0.580392, 0.403922, 0.741176),  # purple
    (0.549020, 0.337255, 0.294118),  # brown
    (0.890196, 0.466667, 0.760784),  # pink
    (0.498039, 0.498039, 0.498039),  # gray
    (0.737255, 0.741176, 0.133333),  # olive
    (0.090196, 0.745098, 0.811765),  # cyan
]
# standard neutral CAD part color (matches FreeCAD's own default ShapeColor),
# used when there's no material info to distinguish by, or only one value.
_DEFAULT_COLOR = (0.8, 0.8, 0.8)

# golden-angle hue rotation (the standard trick for generating N
# incrementally-maximally-distinct colors, e.g. used by most "N distinct
# colors" generators): once a model has more distinct materials than
# _MATERIAL_PALETTE has entries, cycling the fixed palette would silently
# collide two different materials onto the same color, defeating the whole
# point -- generate further colors procedurally instead, so every material
# always gets its own, never a reused one.
_GOLDEN_ANGLE = 0.618033988749895


def _extended_color(index):
    hue = (index * _GOLDEN_ANGLE) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.65, 0.85)


def _collect_materials(CADCells, seen):
    _, universeCADCells = CADCells
    for c in universeCADCells:
        if isinstance(c, (tuple, list)):
            _collect_materials(c, seen)
        else:
            seen.add(c.MAT)


def _material_colors(buildCAD_list):
    """One color per distinct cell.MAT value across the whole export, so
    every solid sharing the same material gets the same color -- and every
    material always gets its own color, never one shared with a different
    material (see `_extended_color` for the >10-materials case). Falls
    back to a single standard CAD gray when there's no real material info
    to distinguish by (zero or one distinct value)."""
    seen = set()
    for CAD in buildCAD_list:
        _collect_materials(CAD, seen)
    if len(seen) <= 1:
        return {mat: _DEFAULT_COLOR for mat in seen}
    ordered = sorted(seen, key=lambda m: (m is None, m))
    colors = {}
    for i, mat in enumerate(ordered):
        if i < len(_MATERIAL_PALETTE):
            colors[mat] = _MATERIAL_PALETTE[i]
        else:
            colors[mat] = _extended_color(i)
    return colors


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
    `mat_colors` (see `_material_colors`)."""
    label, universeCADCells = CADCells
    universe_label = shape_tool.NewShape()
    TDataStd_Name.Set(universe_label, f"Universe_{label[1]}_Container_{label[0]}")
    shape_tool.AddComponent(parent_label, universe_label, TopLoc_Location())

    mat_groups = {}
    for c in universeCADCells:
        if isinstance(c, (tuple, list)):
            _build_tree(shape_tool, color_tool, mat_colors, c, universe_label)
        else:
            mat_groups.setdefault(c.MAT, []).append(c)

    for mat, cells in mat_groups.items():
        mat_label = shape_tool.NewShape()
        TDataStd_Name.Set(mat_label, f"Material_{mat}_{label[0]}{label[1]}")
        shape_tool.AddComponent(universe_label, mat_label, TopLoc_Location())
        color = Quantity_Color(*mat_colors[mat], Quantity_TOC_RGB)
        for c in cells:
            cell_label = shape_tool.AddShape(c.shape.__native__, False)
            TDataStd_Name.Set(cell_label, f"Cell_{c.name}_{c.MAT}")
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

    mat_colors = _material_colors(buildCAD_list)
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


Gmake_elliptic_cone = _not_implemented("Gmake_elliptic_cone")
Gmake_hyperboloid = _not_implemented("Gmake_hyperboloid")
Gmake_ellipsoid = _not_implemented("Gmake_ellipsoid")
Gmake_elliptic_cylinder = _not_implemented("Gmake_elliptic_cylinder")
Gmake_hyperbolic_cylinder = _not_implemented("Gmake_hyperbolic_cylinder")
Gmake_paraboloid = _not_implemented("Gmake_paraboloid")
Gmake_torus_elliptic = _not_implemented("Gmake_torus_elliptic")
