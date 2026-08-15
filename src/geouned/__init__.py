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

# GEOReverse (CsgToCad) always imports FreeCAD/Part directly and has a hard
# .FCStd export dependency with no pyOCC equivalent -- it stays FreeCAD-only
# regardless of GEOUNED_CAD_ENGINE (see geo/__init__.py). Under
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
