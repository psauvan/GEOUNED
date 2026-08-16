"""
GEOReverse/Modules/cad_export/_occ_impl.py

pyOCC-engine-specific implementation of CAD export -- sibling to
`_freecad_impl.py`, NOT YET IMPLEMENTED. `cad_export/__init__.py`
resolves here when `CAD_ENGINE == "occ"`.

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
unsupported under this engine (already handled by `cad_export/__init__.py`'s
own `SUPPORTED_FORMATS`, which should NOT include "fcstd").

Not attempted yet -- flagged as its own follow-up phase. Once this needs
a real `import OCC...` at module level, `cad_export/__init__.py`'s
conditional import is what keeps that from crashing a FreeCAD-only
machine -- unlike the old core.py, which imported both engines'
implementations unconditionally.
"""

SUPPORTED_FORMATS = {"stp", "step"}


def export_occ(buildCAD_list, formats, output_filename, barename):
    raise NotImplementedError(
        "export_occ has no pyOCC implementation yet -- CAD export under GEOUNED_CAD_ENGINE=occ "
        "is not supported. See this module's own docstring for what's needed to add it."
    )
