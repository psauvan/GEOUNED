"""
GEOReverse/Modules/geo_quadrics/__init__.py

Single point of import for `Objects.py`'s use of the 6 exotic quadric
surfaces: `from .geo_quadrics import Gmake_ellipsoid, Gmake_hyperboloid,
...`. Mirrors `geo/__init__.py`'s own engine-switch pattern exactly,
reading the same `CAD_ENGINE` `geo` itself resolved -- not a second,
independent switch.
"""

from ....geo import CAD_ENGINE

if CAD_ENGINE == "occ":
    from ._occ_impl import (
        Gmake_elliptic_cone,
        Gmake_elliptic_cylinder,
        Gmake_ellipsoid,
        Gmake_hyperbolic_cylinder,
        Gmake_hyperboloid,
        Gmake_paraboloid,
        Gmake_torus_elliptic,
    )
else:
    from ._freecad_impl import (
        Gmake_elliptic_cone,
        Gmake_elliptic_cylinder,
        Gmake_ellipsoid,
        Gmake_hyperbolic_cylinder,
        Gmake_hyperboloid,
        Gmake_paraboloid,
        Gmake_torus_elliptic,
    )
