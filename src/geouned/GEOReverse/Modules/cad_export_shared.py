"""
GEOReverse/Modules/cad_export_shared.py

Pure-Python pieces of CAD export previously duplicated -- some across
all 3 backends, some just between `_occ_impl.py`/`_ocp_impl.py` --
factored out once confirmed byte-identical (2026-09-12). No native CAD
import here at all: every backend still owns the part that's genuinely
engine-specific (how a label gets *named* vs. how the API call that
*applies* that name/color actually differs).

- Universe/Material/Cell label-name builders: the exact same f-string
  convention `_freecad_impl.py::makeTree` and `_occ_impl.py`/
  `_ocp_impl.py::_build_tree` all independently built (verified
  byte-identical across all 3 before extracting) -- kept here once so
  the 3 backends can't silently drift out of sync with each other again.
- Per-material color assignment (`material_colors`): verbatim duplicate
  between `_occ_impl.py` and `_ocp_impl.py` only -- `freecad`'s own
  export has no color support at all (FreeCAD's shape-color API lives
  on `Gui.ViewProvider`, which needs a running GUI; confirmed no
  headless path exists, see the `freecad_headless_no_viewprovider`
  project note). Only the native `XCAFDoc_ColorTool.SetColor` calls that
  *apply* these values differ between occ/ocp -- the color values
  themselves never touched a native type.
"""

import colorsys

# ---------------------------------------------------------------------------
# Universe/Material/Cell label naming
# ---------------------------------------------------------------------------


def universe_label_name(name, universe_id) -> str:
    return f"Universe_{universe_id}_Container_{name}"


def material_label_name(mat, name, universe_id) -> str:
    return f"Material_{mat}_{name}{universe_id}"


def cell_label_name(cell_name, mat) -> str:
    return f"Cell_{cell_name}_{mat}"


# ---------------------------------------------------------------------------
# Per-material color assignment (occ/ocp only)
# ---------------------------------------------------------------------------

# matplotlib/D3 "tab10" -- the standard qualitative palette used across
# visualization tooling (matplotlib's own default, D3's category10) to
# give each of up to 10 categories a maximally distinguishable color.
_MATERIAL_PALETTE = [
    (0.121569, 0.466667, 0.705882),  # blue
    (1.000000, 0.498039, 0.054902),  # orange
    (0.172549, 0.627451, 0.172549),  # green
    (0.839216, 0.152941, 0.156863),  # red
    (0.580392, 0.403922, 0.741176),  # purple
    (0.549020, 0.337255, 0.294118),  # brown
    (0.890196, 0.466667, 0.760784),  # pink
    (0.498039, 0.498039, 0.498039),  # gray
    (0.737255, 0.741176, 0.133333),  # olive
    (0.090196, 0.745098, 0.811765),  # cyan
]
# standard neutral CAD part color (matches FreeCAD's own default ShapeColor),
# used when there's no material info to distinguish by, or only one value.
_DEFAULT_COLOR = (0.8, 0.8, 0.8)

# golden-angle hue rotation (the standard trick for generating N
# incrementally-maximally-distinct colors, e.g. used by most "N distinct
# colors" generators): once a model has more distinct materials than
# _MATERIAL_PALETTE has entries, cycling the fixed palette would silently
# collide two different materials onto the same color, defeating the whole
# point -- generate further colors procedurally instead, so every material
# always gets its own, never a reused one.
_GOLDEN_ANGLE = 0.618033988749895


def _extended_color(index):
    hue = (index * _GOLDEN_ANGLE) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.65, 0.85)


def _collect_materials(CADCells, seen):
    _, universeCADCells = CADCells
    for c in universeCADCells:
        if isinstance(c, (tuple, list)):
            _collect_materials(c, seen)
        else:
            seen.add(c.MAT)


def material_colors(buildCAD_list):
    """One color per distinct cell.MAT value across the whole export, so
    every solid sharing the same material gets the same color -- and every
    material always gets its own color, never one shared with a different
    material (see `_extended_color` for the >10-materials case). Falls
    back to a single standard CAD gray when there's no real material info
    to distinguish by (zero or one distinct value)."""
    seen = set()
    for CAD in buildCAD_list:
        _collect_materials(CAD, seen)
    if len(seen) <= 1:
        return {mat: _DEFAULT_COLOR for mat in seen}
    ordered = sorted(seen, key=lambda m: (m is None, m))
    colors = {}
    for i, mat in enumerate(ordered):
        if i < len(_MATERIAL_PALETTE):
            colors[mat] = _MATERIAL_PALETTE[i]
        else:
            colors[mat] = _extended_color(i)
    return colors
