"""
geo/ocp/split.py

SplitResult and Gsplit's own orchestration (_raw_bop_split,
remove_tools_from_raw_solids, check_changed_ok) -- the coaxial-cone
fallback lives in split_coaxial_cone.py, the non-manifold-repair
helpers in split_repair.py, both imported from here.
"""

from __future__ import annotations

import math

from dataclasses import dataclass

from OCP.BOPAlgo import BOPAlgo_Splitter
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from ..solid_defects import find_sliver_faces, valid_solid
from .topology import GShape, GSolid
from ._native_utils import _volume_props
from .boolean import _exploded_solids
from .split_repair import _separate_edge_joined_components, _repair_non_manifold_solid
from .split_coaxial_cone import _find_cone_face, _try_coaxial_cone_split
from .repair import Gsliver_heal, Gheal_topology, Gmerge_coplanar_planes, Gclose_open_solid


@dataclass(frozen=True)
class SplitResult:
    solids: list[GSolid]
    degenerate_case_handled: bool = False
    notes: str = ""
    # A candidate that was BRepCheck-valid and volume-plausible but never
    # actually resolved to a real TopoDS_Solid even after
    # `_resolve_solid_candidate`'s own repair attempt -- each entry is
    # that dropped candidate's own |Volume|. Empty in the common case.
    # Kept as a separate, typed field (not folded into `notes`) so a
    # caller can react to *this specific* event without string-matching
    # `notes`, which also carries routine, expected chatter ("cut did
    # not yield >= 2 sane solids") that is NOT worth surfacing as a
    # warning on its own.
    dropped_no_solid: tuple = ()


def _raw_bop_split(base_native, tool_native, split_tolerance, tolerances) -> tuple[list, bool]:
    """The actual BOPAlgo_Splitter call plus non-manifold repair, factored
    out of Gsplit so `_try_coaxial_cone_split`'s own internal retry (on a
    presplit copy of `base`) can reuse it directly without recursing back
    through Gsplit's own coaxial-cone fallback. Returns (native_solids,
    repaired_any). If BOP finds nothing at all, or every repair attempt
    fails to produce more than one real piece, returns ([base_native],
    False) -- the tool did not usefully split the solid."""
    splitter = BOPAlgo_Splitter()
    splitter.AddArgument(base_native)
    splitter.AddTool(tool_native)
    if split_tolerance > 0:
        splitter.SetFuzzyValue(split_tolerance)
    splitter.Perform()
    raw_solids = _exploded_solids(splitter.Shape())

    if not raw_solids:
        return [base_native], False

    raw_solids = remove_tools_from_raw_solids(raw_solids, base_native, tool_native)

    repaired_any = False
    final_native_solids = []
    for s in raw_solids:
        # Phantom cut ("corte fantasma"): the solid is really >= 2 regions
        # touching only along edges -- return them separated, no capping.
        # Runs on every BOPAlgo output, not just the invalid ones: a
        # phantom-merge can report IsValid()==True. Returns None (fast --
        # no non-manifold edges) for anything that is not this case.
        separated = _separate_edge_joined_components(s)
        if separated is not None:
            repaired_any = True
            final_native_solids.extend(separated)
            continue
        if BRepCheck_Analyzer(s).IsValid():
            faceSliver = find_sliver_faces(GSolid(s), tolerances.min_face_width)
            if not faceSliver:
                final_native_solids.append(s)
                continue
        else:
            # Open-solid repair (a doubled BOPAlgo seam, or a thin missing
            # strip left directly by the split -- see open_solid_repair.py's
            # own docstring for both known causes): cheap and narrowly
            # scoped, so try it before the heavier non-manifold-graph
            # reconstruction. Self-gated (watertight + BRepCheck-valid +
            # volume-conserving), so on any unmatched/unhealed case it
            # returns None and falls through to the existing cascade
            # unchanged.
            Gopened = Gclose_open_solid(GSolid(s), tolerances)
            if Gopened is not None:
                repaired_any = True
                final_native_solids.append(Gopened.__native__)
                continue

        repaired = _repair_non_manifold_solid(s, tolerances.fix_tolerance)
        repaired, same_solid, change_ok = check_changed_ok(s, repaired, tolerances.volume_tolerance)

        if same_solid or not change_ok:
            Grepaired = Gsliver_heal(GSolid(s), tolerances)
            if Grepaired is None:
                repaired = [s]
            else:
                repaired = [Grepaired.__native__]
            repaired, same_solid, change_ok = check_changed_ok(s, repaired, tolerances.volume_tolerance)

        # Last resort: a fragment still BRepCheck-invalid after non-manifold
        # + sliver repair. An in-memory STEP serialize->deserialize
        # (Gheal_topology) rebuilds every face from its clean analytic
        # description -- the only thing found to fix the
        # BRepCheck_InvalidImbricationOfWires class of defect a *failed*
        # BOPAlgo split leaves behind (ShapeFix_Shape / UnifySameDomain do
        # not). Self-gated: Gheal_topology accepts its own result only if
        # it is BRepCheck-valid and volume-conserving to
        # MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE (deliberately looser than the
        # other heal gates -- a failed split inflates the fragment's volume
        # and the rebuild corrects it), else returns None -> keep the
        # fragment as-is. Gated on `not IsValid()` AND a volume that could
        # plausibly be a real piece (Step 1's valid_solid drops a
        # near-zero fragment anyway -- no point paying a STEP round-trip
        # on it) so the common path pays nothing; the healed fragment then
        # flows into generic_split's recursion like any other, so a
        # full-corpus + modelcell_cut1.stp (STACK_OVERFLOW canary) check
        # is required before trusting this.
        if (
            (same_solid or not change_ok)
            and len(repaired) == 1
            and abs(_volume_props(repaired[0]).Mass()) > tolerances.min_solid_volume
            and not BRepCheck_Analyzer(repaired[0]).IsValid()
        ):
            Ghealed = Gheal_topology(GSolid(repaired[0]))
            if Ghealed is not None:
                repaired = [Ghealed.__native__]
                change_ok = True

        if change_ok:
            repaired_any = True

        final_native_solids.extend(repaired)
    if len(final_native_solids) > 1:
        return final_native_solids, repaired_any
    else:
        return [base_native], False


def remove_tools_from_raw_solids(raw_solids, base_native, tool_native):
    """Sometimes the tool solid is returned in the split results, must be removed
    from split solid list"""

    if len(raw_solids) < 2:
        return raw_solids

    tool_volume = _volume_props(tool_native).Mass()
    base_volume = _volume_props(base_native).Mass()
    in_volume = base_volume + tool_volume
    out_volume = sum(_volume_props(x).Mass() for x in raw_solids)
    if abs(out_volume - in_volume) < 1e-5 * in_volume and abs(tool_volume) > 1e-5:
        base_components = []
        tool_CM = _volume_props(tool_native).CentreOfMass()
        for s in raw_solids:
            s_volume = _volume_props(s).Mass()
            if abs(s_volume - tool_volume) < 1e-5 * abs(s_volume):
                sol_CM = _volume_props(s).CentreOfMass()
                d2 = tool_CM.SquareDistance(sol_CM)
                if math.sqrt(d2) < 1e-6:
                    continue
            else:
                base_components.append(s)
        return base_components
    else:
        return raw_solids


def check_changed_ok(original, repaired, volume_tolerance):
    # Never trust the face-adjacency-graph reconstruction blindly:
    # every piece must be a genuinely valid solid AND their summed
    # volume must match the invalid input's own volume (same
    # discipline as _try_coaxial_cone_split's own safety net) --
    # otherwise the reconstruction can silently invent or lose
    # material. Confirmed on a real fixture (modelcell_cut1
    # piece70): a single-plane cut's invalid fragment got
    # "repaired" into 3 pieces summing to ~30% more volume than
    # the original, 2 of them themselves still invalid.

    not_sane_solid = len(repaired) > 1 or (len(repaired) == 1 and not repaired[0].IsEqual(original))
    change_ok = None
    if not_sane_solid:
        all_valid = all(BRepCheck_Analyzer(r).IsValid() for r in repaired)
        if all_valid:
            original_volume = abs(_volume_props(original).Mass())
            repaired_volume = sum(abs(_volume_props(r).Mass()) for r in repaired)
            volume_ok = abs(repaired_volume - original_volume) <= volume_tolerance * max(original_volume, 1.0)
        else:
            volume_ok = False
        change_ok = all_valid and volume_ok

    return repaired, not not_sane_solid, change_ok


def _shells(native) -> list:
    shells = []
    explorer = TopExp_Explorer(native, TopAbs_SHELL)
    while explorer.More():
        shells.append(TopoDS.Shell(explorer.Current()))
        explorer.Next()
    return shells


def _resolve_solid_candidate(g: GSolid, tolerances) -> "GSolid | None":
    """Backstop against a candidate that is BRepCheck-valid (and reports
    a plausible .Volume) but never actually resolves to any real
    TopoDS_Solid -- confirmed live (2026-09-11, rev_pipe.stp): a repair
    step earlier in the cascade (Gsliver_heal's own ShapeUpgrade_
    UnifySameDomain step) can leave a candidate as a bare TopoDS_Shell/
    Compound that stays valid and volume-correct but was never wrapped
    as a solid. `GSolid.Solids`'s own recursive TopAbs_SOLID discovery
    then silently drops it several layers up (generic_split's own
    compounding), with no crash anywhere -- see CLAUDE.md.

    Gsliver_heal's own internal fix (2026-09-11) already prevents this
    for that one specific origin; this is the general backstop, so any
    OTHER repair step producing the same symptom is still caught here
    rather than silently accepted.

    Tried in order: (1) already resolves to >= 1 real solid -- returned
    unchanged; (2) Gclose_open_solid, for a genuinely-open (free-edge)
    case that reaches this point through some other path; (3) if the
    shape holds exactly one TopoDS_Shell, wrap it via
    BRepBuilderAPI_MakeSolid, accepted only if valid and volume-
    conserving against `g`'s own already-computed Volume. Returns None
    (the caller then drops the candidate and reports it via
    SplitResult.notes) for anything else -- deliberately narrow, same
    "detect first, repair only when fast and reliable" convention as
    every other repair in this cascade."""
    if _exploded_solids(g.__native__):
        return g

    opened = Gclose_open_solid(g, tolerances)
    if opened is not None and _exploded_solids(opened.__native__):
        return opened

    shells = _shells(g.__native__)
    if len(shells) != 1:
        return None
    try:
        maker = BRepBuilderAPI_MakeSolid(shells[0])
        if not maker.IsDone():
            return None
        solid = maker.Solid()
    except Exception:
        return None
    if not BRepCheck_Analyzer(solid).IsValid():
        return None
    result = GSolid(solid)
    if abs(result.Volume - g.Volume) > tolerances.volume_tolerance * max(abs(g.Volume), 1.0):
        return None
    return result


def _finalize_split(candidates, base: GSolid, tolerances, repaired_any: bool, notes: str = "") -> SplitResult:
    """Apply the "Gsplit returns only sane solids" contract to a raw list
    of candidate fragments (`list[GSolid]`), shared by every Gsplit return
    path.

    A fragment is kept only if it is BRepCheck-valid AND
    `solid_defects.valid_solid` accepts it (positive volume, not a thin
    sliver by Volume/Area, above the absolute degeneracy floor) AND its
    volume clears `min_solid_volume` AND `_resolve_solid_candidate`
    confirms (repairing if needed) that it genuinely resolves to a real
    TopoDS_Solid. `_raw_bop_split` already tries hard to emit only valid
    pieces (non-manifold / sliver / STEP-round-trip repair, Step 2); the
    `is_valid()` check here is the backstop -- an invalid fragment no
    repair could rescue is not something to hand back. If fewer than 2
    fragments survive, the tool grazed `base` rather than genuinely
    dividing it (a sliver + the bulk, a near-tangent BOP weld, a tool
    that missed, an unhealable invalid fragment) -- return `base`
    unchanged (it came in valid) so `generic_split` treats it as "no
    split" and keeps the solid whole. This restores the pre-ef0077c
    behaviour that `decom_utils_generator.remove_solids` provided
    (deleted when Gsplit's signature was unified); `_raw_bop_split`'s
    own `len <= 1 -> [base_native]` fallback and the freecad
    `check_out_solids` convention already work this way."""

    candidates = [Gmerge_coplanar_planes(s) for s in candidates]

    resolved = []
    unresolved_volumes = []
    for g in candidates:
        r = _resolve_solid_candidate(g, tolerances)
        if r is None:
            unresolved_volumes.append(abs(g.Volume))
        else:
            resolved.append(r)

    sane = [g for g in resolved if g.is_valid() and valid_solid(g) and abs(g.Volume) > tolerances.min_solid_volume]

    extra_notes = []
    if unresolved_volumes:
        vols = ", ".join(f"{v:.2f}" for v in unresolved_volumes)
        extra_notes.append(
            f"{len(unresolved_volumes)} candidate(s) (volumes: {vols}) never resolved to a real "
            "TopoDS_Solid even after repair and were dropped"
        )

    if len(sane) < 2:
        return SplitResult(
            solids=[base],
            degenerate_case_handled=repaired_any or bool(unresolved_volumes),
            notes="; ".join(["cut did not yield >= 2 sane solids; base unchanged"] + extra_notes),
            dropped_no_solid=tuple(unresolved_volumes),
        )
    dropped = len(resolved) - len(sane)
    all_notes = [n for n in (notes,) if n]
    if dropped:
        all_notes.append(f"dropped {dropped} degenerate fragment(s)")
    all_notes.extend(extra_notes)
    return SplitResult(
        solids=sane,
        degenerate_case_handled=repaired_any or dropped > 0 or bool(unresolved_volumes),
        notes="; ".join(all_notes),
        dropped_no_solid=tuple(unresolved_volumes),
    )


def Gsplit(base: GSolid, tool: GShape, tolerances) -> SplitResult:

    if _find_cone_face(tool) is not None:
        tolerance_floor = tolerances.scale_up_floor
        fixed = _try_coaxial_cone_split(base, tool, tolerance_floor, tolerances)
        if fixed is not None:
            return _finalize_split(fixed, base, tolerances, True, notes="coaxial cone degeneracy resolved analytically")

    final_native_solids, repaired_any = _raw_bop_split(base.__native__, tool.__native__, tolerances.split_tolerance, tolerances)

    candidates = [GSolid(s) for s in final_native_solids]
    return _finalize_split(
        candidates,
        base,
        tolerances,
        repaired_any,
        notes="non-manifold repair applied" if repaired_any else "",
    )
