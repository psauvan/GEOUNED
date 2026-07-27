"""
Bootstraps imports for geometry_backend tests.

Two independent problems are solved here:
1. `geouned.geometry_backend` only exists under `src/` in this working
   tree, not in whatever `geouned` release happens to be pip-installed
   -- so `src` must be first on `sys.path` for these tests specifically
   (scoped to this directory only, via pytest's per-directory conftest).
2. On this machine, FreeCAD 1.1 is installed at the path below. Its
   `bin` directory holds `FreeCAD.pyd` and the DLLs it depends on;
   `lib` holds `Part.pyd`/`BOPTools` and depends on those same DLLs.
   Since Python 3.8, extension modules no longer search PATH for
   dependent DLLs -- both directories must be registered explicitly via
   `os.add_dll_directory`, or importing `Part` fails with a bare
   "DLL load failed" error that gives no indication it's a PATH issue.
"""

import os
import sys
from pathlib import Path

_SRC_DIR = Path(__file__).resolve().parents[2] / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

_FREECAD_ROOT = Path(r"C:\Program Files\FreeCAD 1.1")
_FREECAD_BIN = _FREECAD_ROOT / "bin"
_FREECAD_LIB = _FREECAD_ROOT / "lib"

if _FREECAD_BIN.is_dir() and _FREECAD_LIB.is_dir():
    os.add_dll_directory(str(_FREECAD_BIN))
    os.add_dll_directory(str(_FREECAD_LIB))
    if str(_FREECAD_LIB) not in sys.path:
        sys.path.append(str(_FREECAD_LIB))
