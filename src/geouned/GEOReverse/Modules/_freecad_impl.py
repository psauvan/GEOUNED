"""
GEOReverse/Modules/_freecad_impl.py

The FreeCAD-engine-specific implementation of CAD export -- mirrors the
role `geo/_freecad_impl.py` plays for `geo` itself, and
`geo_quadrics/_freecad_impl.py` for that package. `core.py::export_cad`
is CAD-engine-independent and only ever calls `export_freecad` indirectly,
through the `_EXPORTERS` dispatch table -- it never imports `FreeCAD`/
`Import` itself. `_occ_impl.py` is this module's sibling for the pyOCC
case (currently a stub -- see its own docstring for why).
"""

import FreeCAD
import Import

SUPPORTED_FORMATS = {"stp", "step", "fcstd"}


def makeTree(CADdoc, CADCells):
    """Builds the FreeCAD document tree (Universe/Material `App::Part`
    grouping, one `Part::FeaturePython` per solid cell) that `export_freecad`
    writes out. FreeCAD-document-specific through and through (no
    pyOCC equivalent -- pyOCC has no document/label-tree concept), so this
    lives here rather than in `buildCAD.py`, which is otherwise
    CAD-engine-agnostic."""

    label, universeCADCells = CADCells
    groupObj = CADdoc.addObject("App::Part", "Materials")

    groupObj.Label = f"Universe_{label[1]}_Container_{label[0]}"

    CADObj = {}
    for i, c in enumerate(universeCADCells):
        if isinstance(c, (tuple, list)):
            groupObj.addObject(makeTree(CADdoc, c))
        else:
            featObj = CADdoc.addObject("Part::FeaturePython", f"solid{i}")
            featObj.Label = f"Cell_{c.name}_{c.MAT}"
            featObj.Shape = c.shape.__native__
            if c.MAT not in CADObj.keys():
                CADObj[c.MAT] = [featObj]
            else:
                CADObj[c.MAT].append(featObj)

    for mat, matGroup in CADObj.items():
        groupMatObj = CADdoc.addObject("App::Part", "Materials")
        groupMatObj.Label = f"Material_{mat}_{label[0]}{label[1]}"
        groupMatObj.addObjects(matGroup)
        groupObj.addObject(groupMatObj)

    return groupObj


def export_freecad(buildCAD_list, formats, output_filename, barename):
    """Builds the FreeCAD document tree via `makeTree` and writes each
    requested format."""
    CADdoc = FreeCAD.newDocument("converted_with_geouned")

    CADobj = CADdoc.addObject("App::Part", "Universes")
    CADobj.Label = barename

    for CAD in buildCAD_list:
        CADobj.addObject(makeTree(CADdoc, CAD))

    for fmt in formats:
        if fmt in ("stp", "step"):
            Import.export(CADdoc.Objects[0:1], f"{output_filename}.{fmt}")
        elif fmt == "fcstd":
            CADdoc.saveAs(f"{output_filename}.FCStd")
