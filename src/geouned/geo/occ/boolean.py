"""
geo/occ/boolean.py

Boolean operations (Gcut/Gcommon/Gfuse) and the shared _exploded_solids
helper (also used by repair.py's Gheal_topology and split_repair.py).
"""

from __future__ import annotations

from OCC.Core.BRepAlgoAPI import (
    BRepAlgoAPI_Common,
    BRepAlgoAPI_Cut,
    BRepAlgoAPI_Fuse,
)
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from .topology import GSolid


# ---------------------------------------------------------------------------
# Boolean / split operations
# ---------------------------------------------------------------------------


def _exploded_solids(native_result):
    solids = []
    explorer = TopExp_Explorer(native_result, TopAbs_SOLID)
    while explorer.More():
        solids.append(topods.Solid(explorer.Current()))
        explorer.Next()
    return solids


def Gcut(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    result = solid.__native__
    for tool in tools:
        result = BRepAlgoAPI_Cut(result, tool.__native__).Shape()
    return [GSolid(s) for s in _exploded_solids(result)]


def Gcommon(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    result = solid.__native__
    for tool in tools:
        result = BRepAlgoAPI_Common(result, tool.__native__).Shape()
    return [GSolid(s) for s in _exploded_solids(result)]


def Gfuse(solids: list[GSolid]) -> GSolid:
    shapes = []
    for gsolid in solids:
        sub_solids = _exploded_solids(gsolid.__native__)
        if len(sub_solids) > 1:
            shapes.extend(sub_solids)
        else:
            shapes.append(gsolid.__native__)
    fused = shapes[0]
    for s in shapes[1:]:
        fused = BRepAlgoAPI_Fuse(fused, s).Shape()
    return GSolid(fused)
