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

from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
from OCC.Core.TDataStd import TDataStd_Name
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.XCAFApp import XCAFApp_Application
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

SUPPORTED_FORMATS = {"stp", "step"}


def _build_tree(shape_tool, CADCells, parent_label):
    """Mirrors `_freecad_impl.py::makeTree` exactly, one XCAF label per
    FreeCAD `App::Part`/`Part::FeaturePython` object there: a
    `Universe_{U}_Container_{name}` assembly label, containing a
    `Material_{mat}_{U}{name}` assembly label per distinct material,
    each containing one `Cell_{name}_{MAT}`-named shape label per solid
    cell -- recursing into nested universes exactly where `makeTree`
    does (inline, as they're encountered), with material grouping
    collected across the whole list and added afterward, same as there."""
    label, universeCADCells = CADCells
    universe_label = shape_tool.NewShape()
    TDataStd_Name.Set(universe_label, f"Universe_{label[1]}_Container_{label[0]}")
    shape_tool.AddComponent(parent_label, universe_label, TopLoc_Location())

    mat_groups = {}
    for c in universeCADCells:
        if isinstance(c, (tuple, list)):
            _build_tree(shape_tool, c, universe_label)
        else:
            mat_groups.setdefault(c.MAT, []).append(c)

    for mat, cells in mat_groups.items():
        mat_label = shape_tool.NewShape()
        TDataStd_Name.Set(mat_label, f"Material_{mat}_{label[0]}{label[1]}")
        shape_tool.AddComponent(universe_label, mat_label, TopLoc_Location())
        for c in cells:
            cell_label = shape_tool.AddShape(c.shape.__native__, False)
            TDataStd_Name.Set(cell_label, f"Cell_{c.name}_{c.MAT}")
            shape_tool.AddComponent(mat_label, cell_label, TopLoc_Location())

    return universe_label


def export_occ(buildCAD_list, formats, output_filename, barename):
    """Builds the XCAF document tree via `_build_tree` and writes each
    requested format (only "stp"/"step" -- see `SUPPORTED_FORMATS`)."""
    app = XCAFApp_Application.GetApplication()
    doc = TDocStd_Document("XmlXCAF")
    app.NewDocument("XmlXCAF", doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())

    top_label = shape_tool.NewShape()
    TDataStd_Name.Set(top_label, barename)

    for CAD in buildCAD_list:
        _build_tree(shape_tool, CAD, top_label)

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
