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

**Exotic quadric surfaces**: still NOT implemented here either -- same
gap as `_occ_impl.py`, see that file's own docstring for what would be
needed.
"""

from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_DocumentTool

from .cad_export_shared import cell_label_name, material_colors, material_label_name, universe_label_name

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


Gmake_elliptic_cone = _not_implemented("Gmake_elliptic_cone")
Gmake_hyperboloid = _not_implemented("Gmake_hyperboloid")
Gmake_ellipsoid = _not_implemented("Gmake_ellipsoid")
Gmake_elliptic_cylinder = _not_implemented("Gmake_elliptic_cylinder")
Gmake_hyperbolic_cylinder = _not_implemented("Gmake_hyperbolic_cylinder")
Gmake_paraboloid = _not_implemented("Gmake_paraboloid")
Gmake_torus_elliptic = _not_implemented("Gmake_torus_elliptic")
