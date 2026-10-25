import logging
from importlib.metadata import version

logger = logging.getLogger("general_logger")

# GEOReverse (CsgToCad)'s core geometry pipeline follows GEOUNED_CAD_ENGINE
# the same way GEOUNED's own forward pipeline does (see
# GEOReverse/Modules/__init__.py) -- no FreeCAD dependency anywhere in
# either pipeline any more.
from .GEOReverse import *

from .GEOUNED import *

# __version__ = version("geouned")
__version__ = "0.0.0"

__all__ = ["__version__"]
