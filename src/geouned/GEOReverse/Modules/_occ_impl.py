"""
GEOReverse/Modules/_occ_impl.py

pyOCC-engine-specific implementation of CAD export -- sibling to
`_freecad_impl.py`, NOT YET IMPLEMENTED. `core.py::export_cad` resolves
here (via `_EXPORTERS["occ"]`) when `CAD_ENGINE == "occ"`.

Unlike `geo`'s own STEP export (`Gexport_step`, already engine-agnostic
and usable as-is once GEOReverse's `GSolid`s are genuinely pyOCC
native), this file still needs real work: `_freecad_impl.py`'s
`export_freecad` doesn't just dump solids to STEP, it first builds a
FreeCAD *document* tree (`makeTree` -- Universe/Material `App::Part`
grouping, per-cell labels) that both the `.stp`/`.step` export (via
`Import.export`, which walks that tree) and the FreeCAD-only `.FCStd`
output depend on. pyOCC has no document/label-tree concept at all, so a
real implementation here needs to reconstruct at least the label/grouping
part some other way -- most likely via OCC's XCAF framework
(`XCAFDoc_ShapeTool`/`XCAFDoc_ColorTool` names+colors per solid,
written through `STEPCAFControl_Writer` instead of the plain
`STEPControl_Writer` `Gexport_step` uses) -- and `.fcstd` simply stays
unsupported under this engine (already handled by `core.py`'s own
`_SUPPORTED_FORMATS["occ"]`, which should NOT include "fcstd").

Not attempted yet -- flagged as its own follow-up phase.
"""

SUPPORTED_FORMATS = {"stp", "step"}


def export_occ(buildCAD_list, formats, output_filename, barename):
    raise NotImplementedError(
        "export_occ has no pyOCC implementation yet -- CAD export under GEOUNED_CAD_ENGINE=occ "
        "is not supported. See this module's own docstring for what's needed to add it."
    )
