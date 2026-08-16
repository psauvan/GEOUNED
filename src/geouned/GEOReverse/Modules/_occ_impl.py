"""
GEOReverse/Modules/_occ_impl.py

The pyOCC counterpart to `_freecad_impl.py`, covering the same two
unrelated concerns (CAD export, the 6 exotic quadric surfaces) --
NEITHER is implemented yet. `Modules/__init__.py` resolves here when
`CAD_ENGINE == "occ"`.

**CAD export** (`SUPPORTED_FORMATS`, `export_occ`): unlike `geo`'s own
STEP export (`Gexport_step`, already engine-agnostic and usable as-is
once GEOReverse's `GSolid`s are genuinely pyOCC native), this still needs
real work: `_freecad_impl.py`'s `export_freecad` doesn't just dump solids
to STEP, it first builds a FreeCAD *document* tree (`makeTree` --
Universe/Material `App::Part` grouping, per-cell labels) that both the
`.stp`/`.step` export (via `Import.export`, which walks that tree) and
the FreeCAD-only `.FCStd` output depend on. pyOCC has no document/
label-tree concept at all, so a real implementation needs to reconstruct
at least the label/grouping part some other way -- most likely via OCC's
XCAF framework (`XCAFDoc_ShapeTool`/`XCAFDoc_ColorTool` names+colors per
solid, written through `STEPCAFControl_Writer` instead of the plain
`STEPControl_Writer` `Gexport_step` uses) -- and `.fcstd` simply stays
unsupported under this engine (`SUPPORTED_FORMATS` below does NOT include
"fcstd").

**Exotic quadric surfaces** (`Gmake_elliptic_cone`, `Gmake_hyperboloid`,
`Gmake_ellipsoid`, `Gmake_elliptic_cylinder`, `Gmake_hyperbolic_cylinder`,
`Gmake_paraboloid`, `Gmake_torus_elliptic`): building these needs pyOCC
equivalents of `_freecad_impl.py`'s own `Part.Ellipse`/`Part.Hyperbola`/
`.revolve()`/`.extrude()`/`Part.makeLoft` constructions -- likely
`Geom_Ellipse`/`Geom_Hyperbola`, `BRepPrimAPI_MakeRevol` for the
revolve-based builds, `BRepPrimAPI_MakePrism` for the extrude-based
hyperbolic-cylinder build, and `BRepOffsetAPI_ThruSections` for the
loft-based elliptic-cone/-cylinder builds.

Neither piece attempted yet -- flagged as its own follow-up phase, not
something to guess at without a real pyOCC session to verify each
construction against known volumes/geometry, matching every other piece
of this project's pyOCC migration (see CLAUDE.md's own history: every
`geo` addition was verified against real geometry before being trusted,
never written blind). Once either needs a real `import OCC...` at module
level, `Modules/__init__.py`'s conditional import is what keeps that
from crashing a FreeCAD-only machine -- unlike the old `core.py`, which
used to import both engines' implementations unconditionally.
"""

SUPPORTED_FORMATS = {"stp", "step"}


def export_occ(buildCAD_list, formats, output_filename, barename):
    raise NotImplementedError(
        "export_occ has no pyOCC implementation yet -- CAD export under GEOUNED_CAD_ENGINE=occ "
        "is not supported. See this module's own docstring for what's needed to add it."
    )


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
