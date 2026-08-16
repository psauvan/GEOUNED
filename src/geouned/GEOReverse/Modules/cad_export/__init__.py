"""
GEOReverse/Modules/cad_export/__init__.py

Single point of import for `core.py`'s CAD export step:
`from .cad_export import SUPPORTED_FORMATS, export`. Mirrors
`geo_quadrics/__init__.py`'s own `CAD_ENGINE`-dispatch pattern exactly --
`core.py` never imports `_freecad_impl.py`/`_occ_impl.py` directly, and
only the resolved engine's own module is ever actually imported. Matters
once `_occ_impl.py` gets a real pyOCC implementation that needs
`import OCC...` at module level -- importing both unconditionally (as
`core.py` used to) would crash on a FreeCAD-only machine the moment that
happens.
"""

from ....geo import CAD_ENGINE

if CAD_ENGINE == "occ":
    from ._occ_impl import SUPPORTED_FORMATS, export_occ as export
else:
    from ._freecad_impl import SUPPORTED_FORMATS, export_freecad as export
