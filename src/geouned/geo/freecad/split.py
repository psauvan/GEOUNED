"""
geo/freecad/split.py

SplitResult and Gsplit -- FreeCAD's own tolerance-retry cascade over
BOPTools.SplitAPI.slice (does NOT solve the project's original
motivating silent-uncut-solid tangency bug -- see this file's own
Gsplit docstring, and CLAUDE.md's "Motivating problem" section).
"""

from __future__ import annotations

from dataclasses import dataclass

import Part
import BOPTools.SplitAPI

from .topology import GShape, GSolid


# ---------------------------------------------------------------------------
# Result of operations that can fail/degenerate
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SplitResult:
    """
    Result of a Gsplit operation.

    `solids` must never be empty for a valid operation: if a degenerate
    case is detected (tangency, coincident edges...) Gsplit must resolve
    it internally (retry at a different tolerance, fall back to the
    unchanged solid, etc.) and report it via `degenerate_case_handled=True`,
    NEVER silently return nothing.
    """

    solids: list[GSolid]
    degenerate_case_handled: bool = False
    notes: str = ""


def Gsplit(
    base: GSolid,
    tool: GShape,
    tolerances,
) -> SplitResult:

    scale = 0.1
    split_tolerance = tolerances.split_tolerance
    scale_up_floor = tolerances.scale_up_floor
    scale = tolerances.scale
    return recursive_freecad_Gsplit(base, tool, split_tolerance, scale, scale_up_floor) 

def recursive_freecad_Gsplit(
    base: GSolid,
    tool: GShape,
    tolerance: float,
    scale: float = 0.1,
    scale_up_floor: float | None = None,
) -> SplitResult:
    """
    Cut `base` with a surface/solid `tool` (typically a plane) and return
    ALL resulting fragments. Replaces `BOPTools.SplitAPI.slice` at every
    GEOUNED call site: this function is responsible for internally
    resolving degenerate cases (tangencies, tool not intersecting the
    solid, kernel exceptions at small tolerances) -- callers must NOT
    implement their own retry/offset logic on top of this.

    `scale_up_floor` mirrors GEOUNED's public `Options.scaleUp`/
    `Options.splitTolerance`: when `tolerance` drops below 1e-12 and
    `scale_up_floor` is given, retry upward starting from that floor
    instead of just attempting the tiny tolerance as-is. Below 1e-12 with
    no floor, and at `tolerance >= 0.1`, there is no retry at all --
    those are the two cases where shrinking further is not expected to
    help.
    """
    tools = [tool.__native__]

    if tolerance >= 0.1:
        compound = BOPTools.SplitAPI.slice(base.__native__, tools, "Split", tolerance=tolerance)
    elif tolerance < 1e-12:
        if scale_up_floor is not None:
            floor = 1e-13 if scale_up_floor == 0 else scale_up_floor
            return recursive_freecad_Gsplit(base, tool, floor / scale, scale=1.0 / scale, scale_up_floor=scale_up_floor)
        compound = BOPTools.SplitAPI.slice(base.__native__, tools, "Split", tolerance=tolerance)
    else:
        try:
            compound = BOPTools.SplitAPI.slice(base.__native__, tools, "Split", tolerance=tolerance)
        except Exception:
            retried = recursive_freecad_Gsplit(base, tool, tolerance * scale, scale, scale_up_floor)
            return SplitResult(
                solids=retried.solids,
                degenerate_case_handled=True,
                notes=f"retried at tolerance={tolerance * scale}",
            )

    return check_out_solids(base, compound.Solids)


def check_out_solids(original, split_solids):
    if not split_solids:
        # tool doesn't intersect solid at all (e.g. a cutting plane
        # entirely outside the solid's extent) -- slice() reports this as
        # an empty compound rather than raising. Not a fragmentation, so
        # fall back to the solid unchanged instead of reporting "no
        # solids".
        return SplitResult(
            solids=[original],
            degenerate_case_handled=True,
            notes="tool did not intersect solid; returning it unchanged",
        )

    if sum(s.Volume for s in split_solids) < 1e-3:
        return SplitResult(
            solids=[original],
            degenerate_case_handled=True,
            notes="tool did not intersect solid; returning it unchanged",
        )
    elif len(split_solids) == 1:
        return SplitResult(solids=[original])
    else:
        cleaned = remove_solids(split_solids, original.Volume)
        if len(cleaned) < len(split_solids):
            return SplitResult(
                solids=[GSolid(s) for s in cleaned],
                degenerate_case_handled=True,
                notes="tool did not intersect solid; returning it unchanged",
            )
        else:    
            return SplitResult(solids=[GSolid(s) for s in cleaned])


def remove_solids(Solids: list, Volume) -> list:
    # `Solids` here are native Part.Solid (straight from BOPTools.SplitAPI.
    # slice()'s own compound, via check_out_solids) -- valid_solid/
    # _refine_if_valid are GSolid-typed (per their own signatures), so wrap
    # on the way in and unwrap on the way out, matching check_out_solids'
    # own expectation that `cleaned` stays native.
    Solids_Clean = []
    for solid in Solids:
        if not valid_solid(GSolid(solid), Volume):
            continue
        Solids_Clean.append(solid)

    return [_refine_if_valid(GSolid(sol)).__native__ for sol in Solids_Clean]


def valid_solid(solid: GSolid, Volume) -> bool:
    if solid.Volume < 0:
        return False
    Vol_tol = 1e-2
    Vol_area_ratio = 1e-3
    if solid.Area == 0 or abs(solid.Volume / solid.Area) < Vol_area_ratio:
        return False
    if abs(solid.Volume) < Vol_tol:
        return False
    return True


def _refine_if_valid(solid: GSolid) -> GSolid:
    # refine() (ShapeUpgrade_UnifySameDomain/removeSplitter) is a cosmetic
    # simplification of an already-valid solid, not a repair tool -- on a
    # solid that's already topologically invalid (BRepCheck_Analyzer), its
    # UnifyEdges step is a confirmed, previously-documented crash/hang
    # risk (see GSolid.refine()'s own docstring, the ConeSphere.stp case
    # under occ/ocp) that no amount of Python try/except can catch, since
    # it's a native process crash, not a raised exception. Confirmed live
    # (2026-08-19, Solidos/Big_one_cell/modelcell_cut1.stp under ocp): a
    # BOP-produced fragment that's already invalid before refine() ever
    # runs reliably segfaults the process inside refine()'s own UnifyEdges
    # call.
    #
    # A real repair attempt via .fix() (ShapeFix_Shape) was tried here
    # twice, both reverted. The first attempt crashed even earlier than
    # refine() itself did; that specific crash traced back to a real bug in
    # .fix() (fixed 2026-08-19: it used to reassign its own `native`
    # variable to UnifyEdges' own possibly-corrupted output before checking
    # that output's validity, so its ShapeFix_Shape fallback silently
    # repaired the *corrupted* intermediate instead of the true original
    # input). With that fixed, calling .fix() *after* a full decomposition
    # had already completed (on the final, already-produced invalid
    # fragments, as a separate manual pass) worked cleanly with no crash on
    # this exact modelcell_cut1.stp reproduction. But wiring the fixed
    # .fix() into this function -- called *during* decomposition, on
    # intermediate fragments that then flow into further Gsplit calls, not
    # just on final output -- reproduced a crash again on the same file,
    # this time STATUS_STACK_OVERFLOW (0xC00000FD) rather than the original
    # UnifyEdges access violation. So repairing a fragment mid-decomposition
    # and feeding the repaired result back into further cuts is its own,
    # separately confirmed, still-unresolved crash risk -- distinct from
    # (and not fixed by) the .fix() bug fix above. Reverted back to the
    # simple, confirmed-safe form: leave an already-invalid solid untouched
    # rather than risk repairing it here. GSolid.fix() itself remains a
    # real, safe repair tool for use *after* decomposition is complete (on
    # final output only), just not at this specific, mid-pipeline call site.
    return solid.refine() if solid.is_valid() else solid
