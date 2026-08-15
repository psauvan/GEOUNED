"""
GEOReverse/Modules/geo_quadrics/_occ_impl.py

pyOCC implementation of the 6 exotic quadric surfaces (elliptic cone,
hyperboloid, ellipsoid, elliptic cylinder, hyperbolic cylinder,
paraboloid) -- NOT YET IMPLEMENTED. Every `Gmake_*` function below raises
`NotImplementedError` until this is built; `geo_quadrics/__init__.py`
resolves here when `CAD_ENGINE == "occ"`.

Building this needs pyOCC equivalents of `_freecad_impl.py`'s own
`Part.Ellipse`/`Part.Hyperbola`/`.revolve()`/`.extrude()`/`Part.makeLoft`
constructions -- likely `Geom_Ellipse`/`Geom_Hyperbola`,
`BRepPrimAPI_MakeRevol` for the revolve-based builds, `BRepPrimAPI_MakePrism`
for the extrude-based hyperbolic-cylinder build, and
`BRepOffsetAPI_ThruSections` for the loft-based elliptic-cone/-cylinder
builds. Not attempted yet -- flagged as its own follow-up phase, not
something to guess at without a real pyOCC session to verify each
construction against known volumes, matching every other piece of this
project's pyOCC migration (see CLAUDE.md's own history: every `geo`
addition was verified against real geometry before being trusted, never
written blind).
"""


def _not_implemented(name):
    def _raise(*args, **kwargs):
        raise NotImplementedError(
            f"geo_quadrics.{name} has no pyOCC implementation yet -- exotic quadric surfaces "
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
