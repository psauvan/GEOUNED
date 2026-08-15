import logging
from importlib.metadata import version

logger = logging.getLogger("general_logger")

# this try except attempts to import freecad (lowercase) which is the conda
# package name for FreeCAD (mixed case) upon import the conda package appends
# the sys path for Conda installed FreeCAD, consequently FreeCAD can then be
# found by subsequent import statements through out the code base
try:
    import freecad
except ImportError:
    pass

# GEOReverse (CsgToCad)'s core geometry pipeline now follows GEOUNED_CAD_ENGINE
# the same way GEOUNED's own forward pipeline does (see GEOReverse/Modules/
# _geo_bridge.py) -- but its 6 exotic quadric surfaces (geo_quadrics/) and its
# CAD export step (_freecad_impl.py/_occ_impl.py) are still FreeCAD-only in
# practice: the pyOCC side of both is a stub pending a real port. Under
# GEOUNED_CAD_ENGINE=occ a user may not have FreeCAD installed at all, so this
# import must degrade gracefully instead of crashing CadToCsg (GEOUNED) along
# with it.
try:
    from .GEOReverse import *
except ImportError as e:
    logger.warning(f"GEOReverse (CsgToCad) unavailable -- FreeCAD import failed: {e}")
    CsgToCad = None
    BoxSettings = None

from .GEOUNED import *

# __version__ = version("geouned")
__version__ = "0.0.0"

__all__ = ["__version__"]
