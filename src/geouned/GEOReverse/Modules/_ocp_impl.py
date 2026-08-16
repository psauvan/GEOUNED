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

**Per-solid color by material**: identical logic to `_occ_impl.py`
(`_material_colors`/`_extended_color`/`_MATERIAL_PALETTE`/`_DEFAULT_COLOR`,
verbatim) -- only the XCAF calls that apply the color differ in the ways
above. Not yet independently re-verified end to end against a real
written STEP file's raw text under OCP the way it was for pythonocc-core
(see `_occ_impl.py`'s own docstring) -- flagged for whenever that's
checked.

**Exotic quadric surfaces**: still NOT implemented here either -- same
gap as `_occ_impl.py`, see that file's own docstring for what would be
needed.
"""

import colorsys

from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_DocumentTool

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
    """Mirrors `_occ_impl.py::_build_tree` exactly, one XCAF label per
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
    TDataStd_Name.Set_s(universe_label, TCollection_ExtendedString(f"Universe_{label[1]}_Container_{label[0]}"))
    shape_tool.AddComponent(parent_label, universe_label, TopLoc_Location())

    mat_groups = {}
    for c in universeCADCells:
        if isinstance(c, (tuple, list)):
            _build_tree(shape_tool, color_tool, mat_colors, c, universe_label)
        else:
            mat_groups.setdefault(c.MAT, []).append(c)

    for mat, cells in mat_groups.items():
        mat_label = shape_tool.NewShape()
        TDataStd_Name.Set_s(mat_label, TCollection_ExtendedString(f"Material_{mat}_{label[0]}{label[1]}"))
        shape_tool.AddComponent(universe_label, mat_label, TopLoc_Location())
        color = Quantity_Color(*mat_colors[mat], Quantity_TOC_RGB)
        for c in cells:
            cell_label = shape_tool.AddShape(c.shape.__native__, False)
            TDataStd_Name.Set_s(cell_label, TCollection_ExtendedString(f"Cell_{c.name}_{c.MAT}"))
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
            f"{name} has no OCP implementation yet -- exotic quadric surfaces "
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
