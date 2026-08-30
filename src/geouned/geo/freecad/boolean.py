"""
geo/freecad/boolean.py

Boolean operations (Gcut/Gcommon/Gfuse) -- free functions rather than
methods since they combine two-or-more independent shapes, so there is
no single natural "self". Gsplit and its own cascade live in split.py
instead, next to this file, not in it -- Gsplit is a large, separately
evolving feature, not a simple boolean.
"""

from __future__ import annotations

import Part

from .topology import GSolid


# ---------------------------------------------------------------------------
# Boolean / split operations
# (free functions rather than methods since they combine two-or-more
# independent shapes -- there is no single natural "self")
# ---------------------------------------------------------------------------


def Gcut(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    """Subtract `tools` from `solid`. May return >1 solid if fragmented."""
    result = solid.__native__.cut([tool.__native__ for tool in tools])
    return [GSolid(s) for s in result.Solids]


def Gcommon(solid: GSolid, tools: list[GSolid]) -> list[GSolid]:
    """Boolean intersection."""
    result = solid.__native__.common([tool.__native__ for tool in tools])
    return [GSolid(s) for s in result.Solids]


def Gfuse(solids: list[GSolid]) -> GSolid:
    """Boolean union."""
    shapes = []
    for gsolid in solids:
        if type(gsolid.__native__) is Part.Compound:
            shapes.extend(gsolid.__native__.Solids)
        else:
            shapes.append(gsolid.__native__)
    fused = shapes[0].fuse(shapes[1:]) if len(shapes) > 1 else shapes[0]
    return GSolid(fused)
