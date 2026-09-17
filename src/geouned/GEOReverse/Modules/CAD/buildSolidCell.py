from ..data_class import Options
from ....GEOUNED.utils.data_classes import Tolerances
from ....geo import BuildDepth, getPart
from .splitFunction import surface_side


def BuildSolid(cell):
    # BuildDepth/BuildSolidParts/filterparts/getPart/SplitBase/joinBase/
    # SplitSolid moved to `geo.solid_ops`, 2026-09-17/18 -- shared with
    # GEOUNED's own, previously near-identical copy (`build_region/
    # build_region.py`'s BuildDepth/BuildSolidParts/filterparts/getPart,
    # `build_region/splitFunction.py`'s SplitBase/joinBase/SplitSolid, both
    # now deleted). See CLAUDE.md's "build_region/ vs
    # CAD/buildSolidCell.py+splitFunction.py unification" entry. `Options.
    # splitTolerance` (a bare float, GEOReverse's own global tuning knob)
    # is converted into a real `Tolerances` instance once, here, at this
    # pipeline's own single entry point into the shared cascade -- it used
    # to be re-read and re-wrapped on every call, deep inside `SplitSolid`
    # itself. `surface_side` (this module's own, exotic-quadric-aware
    # point classification, unchanged) is passed through as the shared
    # cascade's pluggable `classify` callable.
    cell.cleanUndefined()
    tolerances = Tolerances(split_tolerance=Options.splitTolerance)
    celParts = BuildDepth(cell, base=None, tolerances=tolerances, classify=surface_side)
    celParts = getPart(celParts)
    shapeParts = []
    for i, s in enumerate(celParts):
        shapeParts.append(s.base)
    return shapeParts
