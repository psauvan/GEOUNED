"""
GEOReverse/Modules/__init__.py

Single point of import for GEOReverse's own engine-specific code:
`from .Modules import SUPPORTED_FORMATS, export, Gmake_ellipsoid, ...`.
Mirrors `geo/__init__.py`'s own dispatch pattern exactly -- reads the
same `CAD_ENGINE` `geo` itself resolved, then imports *only* the
matching `_freecad_impl.py`/`_occ_impl.py`, never both. `core.py` and
`Objects.py` are this package's own consumers; neither imports
`_freecad_impl.py`/`_occ_impl.py` directly.
"""

from ...geo import CAD_ENGINE

if CAD_ENGINE == "occ":
    from ._occ_impl import (
        SUPPORTED_FORMATS,
        export_occ as export,
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
        SUPPORTED_FORMATS,
        export_freecad as export,
        Gmake_elliptic_cone,
        Gmake_elliptic_cylinder,
        Gmake_ellipsoid,
        Gmake_hyperbolic_cylinder,
        Gmake_hyperboloid,
        Gmake_paraboloid,
        Gmake_torus_elliptic,
    )
