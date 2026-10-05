#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging

from .generators import get_surfaces
from ...geo import (
    CAD_ENGINE,
    GCompound,
    GSolid,
    Gheal_topology,
    Gmake_compound,
    Gsolid_max_tolerance,
    Gsolid_nonmanifold_edge_count,
    Gsplit,
    Gtrim_overlapping_coplanar_faces,
    is_sliver_solid,
    merge_sliver_pieces,
)
from ...geo.constants import SPLIT_CANDIDATE_VOLUME_REL_TOL, TOLERANCE_WELD_FLOOR
from ...geo.solid_defects import find_overlapping_coplanar_faces
from ...geo.volume_utils import volume_within

logger = logging.getLogger("general_logger")


def split_surfaces(solid, options, tolerances):

    solid_components = generic_split(solid, options, tolerances)

    # `.fix()` can raise a real native crash (Standard_Failure: "Courbes non
    # jointives", the same ShapeUpgrade_UnifySameDomain crash class already
    # documented for Mixed/ConeSphere.stp) on a piece that never needed
    # fixing in the first place -- confirmed live on Complex_cell/SCDR_90.stp.
    # Keep the original, unfixed piece on that failure rather than let one
    # auxiliary repair attempt abort an otherwise-fine decomposition.
    fixed_solids = []
    for s in solid_components:
        try:
            s = s.fix(tolerances.fix_tolerance)
        except Exception:
            pass
        # A boolean cut landing exactly tangent to a pre-existing surface
        # can leave a residual "phantom" face duplicating part of a real
        # neighboring face -- `generic_split`'s own recursive cuts can
        # create this defect fresh, on a piece that never had it at load
        # time (confirmed live, struct_plate2.stp's own piece8: 0 defects
        # on the original, undecomposed solid; the defect only appears
        # after decomposition splits it), so the load-time repair cascade
        # (`Gcheck_and_repair`) never gets a chance to catch it -- it has
        # to be checked again here, on each piece this function itself
        # produces. Same crash-safety discipline as `.fix()` just above:
        # never let an auxiliary repair attempt abort an otherwise-fine
        # decomposition, and keep the original piece if the repair isn't
        # possible (`Gtrim_overlapping_coplanar_faces` already returns
        # `None` rather than raising in that case, but the whole check is
        # still wrapped defensively).
        try:
            if find_overlapping_coplanar_faces(s):
                repaired = Gtrim_overlapping_coplanar_faces(s)
                if repaired is not None:
                    s = repaired
        except Exception:
            pass
        fixed_solids.append(s)

    # A thin sliver that survived Gsplit's own repair cascade (see
    # merge_sliver_pieces's own docstring, geo/solid_ops.py) is its own
    # genuinely separate, valid solid -- Gsliver_heal declines to touch
    # it (not a redundant coplanar face WITHIN one piece), so it is still
    # present here, in the FULL list of this solid's final irreducible
    # pieces. Eliminate it now, by fusing it into whichever real
    # neighbor it is still in contact with, rather than letting it become
    # its own separate cell with a face no MCNP/OpenMC boolean definition
    # can ever meaningfully bound.
    fixed_solids = merge_sliver_pieces(fixed_solids, tolerances)

    comp = GCompound(fixed_solids)

    volratio = (comp.Volume - solid.Volume) / solid.Volume
    if volratio > tolerances.volume_tolerance:
        logger.warning(f"Lost {volratio * 100:6.2f}% of the original volume")

    # A fragment Gsplit accepted as "sane" (BRepCheck-valid + a real
    # volume, via _finalize_split's own filter) can still fail to
    # resolve to a genuine TopoDS_Solid: a repair step inside
    # _raw_bop_split's cascade (confirmed: Gsliver_heal's own
    # ShapeUpgrade_UnifySameDomain step) can leave a fragment as a bare
    # TopoDS_Compound/Shell that is still topologically valid and still
    # reports a correct .Volume (computed shape-type-agnostically) -- but
    # contains zero real solid leaves. Detected here via a one-shot,
    # throwaway native wrap+re-extract of EACH piece ALONE (never several
    # touching pieces together, which is the exact scenario GCompound's
    # own docstring documents as unreliable) -- so this probe only ever
    # asks "does this single fragment's own native shape contain a real
    # TopAbs_SOLID", never anything aggregate, and is never used for the
    # actual returned pieces/volumes (those come from `fixed_solids`/`comp`
    # above) -- a fragment that fails this resolves to zero real solid
    # leaves and will NOT be part of this cell's boolean expression, a
    # real, currently-unrepaired gap (see CLAUDE.md).
    dropped = [f for f in fixed_solids if not Gmake_compound([f]).Solids]
    if dropped:
        dropped_volume = sum(abs(f.Volume) for f in dropped)
        fragment_volumes = ", ".join(f"{abs(f.Volume):.2f}" for f in fixed_solids)
        logger.warning(
            f"generic_split produced {len(fixed_solids)} fragment(s) "
            f"(volumes: {fragment_volumes}) but {len(dropped)} did not resolve to "
            f"a real solid -- {dropped_volume:.2f} of volume silently dropped and "
            "will NOT be considered when building this solid's boolean expression."
        )

    return comp


def generic_split(solid, options, tolerances, loop=0, healed=False):
    # Heal a BRepCheck-invalid decomposition fragment before splitting it
    # further (or returning it as a final piece). BOPAlgo_Splitter can
    # leave a cut cylinder/cone wedge with one V-boundary edge's pcurve a
    # full 2*pi period off: the face's BRepTools::UVBounds then spans the
    # whole period (IsUClosed()==True) with ~one period of phantom
    # material glued on, so its ParameterRange contradicts its own
    # small-angle arc edges and Can/RoundCorner/RevCC detection treats it
    # as a closed cylinder. Only ShapeFix_Shape (GSolid.fix() on an
    # invalid solid) resolves it -- removeSplitter/UnifySameDomain
    # (.refine(), already applied by GeounedSolid.__init__) does not, and
    # _raw_bop_split's own repair result gets discarded whenever it
    # collapses to a single solid. Confirmed on RevCC_regression/
    # Big_one_cell__modelCell_670000__solid0_piece59: the first decomposed
    # sub-solid's R=6 round-corner cylinder went from a 2*pi U range
    # (piece vol 6849, split total overshooting the input by exactly the
    # 754mm^3 phantom region) to a ~75 deg wedge (vol 6095, split total
    # matching the input). Only fires on a genuinely invalid piece, so
    # the normal path is untouched.
    if not solid.is_valid():
        fixed = solid.fix(tolerances.fix_tolerance)
        if fixed.is_valid():
            solid = fixed

    bbox = solid.BoundBox
    bbox = bbox.enlarged(10)

    comsolid_solids = [solid]
    omitfaces = set()

    # BOPAlgo_Splitter.Perform() can corrupt `solid`'s own native shape as a
    # side effect of merely attempting an intersection, even when the
    # candidate tool ultimately fails to split it at all (still 1 output
    # piece) -- since Gsplit's own `base` argument is passed by reference
    # straight into BOPAlgo_Splitter.AddArgument, this corruption is
    # cumulative and order-dependent across the WHOLE candidate-surface
    # search below, not scoped to one failed attempt. Confirmed live on
    # Mixed/multiplane_add_plane_cyl.stp: an earlier, failed Plane candidate
    # raised the base fragment's own tolerance 5x (1.2e-7 -> 6.0e-7), and a
    # LATER candidate that would have split it correctly (3 pieces summing
    # exactly to the input volume, on a pristine copy) instead
    # under-separated it (2 pieces, losing a whole 276 mm^3 fragment) once
    # tried against the now-degraded shape -- the volume-conservation guard
    # below correctly rejected that wrong split, but no OTHER candidate ever
    # got a chance to try against a clean base, so the fragment was left
    # permanently unsplit. A tolerance reset alone (Gsolid_set_tolerance)
    # was tried and confirmed INSUFFICIENT: instrumented directly, face/
    # edge/vertex counts and every vertex position stayed bit-identical
    # across repeated "failed" attempts even with the tolerance reset
    # applied each time, yet the base's own `is_valid()` (BRepCheck_Analyzer)
    # flipped True -> False -- BOPAlgo_Splitter had corrupted the shape's
    # internal parametrization consistency (pcurve/SameParameter state), not
    # its tolerance or geometry, and that is what made the later candidate
    # misbehave. Fixed by resetting BOTH the tolerance AND re-running the
    # same validity fix already used once at the top of this function,
    # before every new candidate is tried, not just once at the top.
    # freecad has no equivalent tolerance-reset primitive
    # (Gsolid_set_tolerance) yet -- occ/ocp only, matching every other
    # native-tolerance-inspection feature in this cascade; its own `.fix()`
    # re-run still applies there since it's engine-agnostic.
    if CAD_ENGINE != "freecad":
        from ...geo import Gsolid_set_tolerance

        original_tolerance = Gsolid_max_tolerance(solid)
    else:
        original_tolerance = None

    # Candidate-surface detection (cutting planes, MultiPlane/Can/TCone/
    # RoundCorner masters, ...) must judge a face's own min_area/
    # min_face_width against THIS solid's own scale, exactly like
    # cell_definition.py::simple_solid_definition's own scaled_tolerances
    # -- otherwise the same face can be read as a genuine feature at the
    # cell-definition stage (scaled) but as a sliver here (unscaled),
    # confirmed live on Solidos/working_solids/clamp.stp's solid0: two
    # real, 0.081mm-wide corner-forming planes (CharacteristicWidth
    # 0.081115) sit strictly between the default min_face_width=0.1 and
    # this solid's own scaled 0.068 -- eligible_plane/order_plane_face
    # read them as slivers here (unscaled tolerances), while
    # cell_definition.py's own get_multiplanes (scaled) correctly reads
    # them as real geometry. The `Gsplit`/`build_surface`/`solid.fix()`
    # calls below keep the ORIGINAL, unscaled `tolerances` -- those are
    # kernel-operation precision (split/fix tolerance), not surface-size
    # classification, and scaling them down for a small fragment is not
    # the concern this scaling exists for.
    scaled_tolerances = tolerances.scaled(solid.Volume)

    new_split = False
    # The first candidate that successfully splits `solid` is not
    # necessarily the BEST one: a candidate tried early (per the
    # established cylinder/cone-before-plane ordering) can "win" by
    # carving off only sliver-sized debris from what is still, in
    # substance, a single real piece, while a LATER candidate -- tried
    # only if this one is rejected -- can give a genuinely clean,
    # multi-piece split instead. Confirmed live, 2026-10-05, on
    # Solidos/working_solids/clamp.stp's solid0: an R=5.25 cylinder
    # candidate "succeeds" first, splitting off two 0.0837 mm^3 wedges
    # that merge_sliver_pieces (see its own docstring) then fuses right
    # back -- but a real, 0.081 mm-wide corner-forming plane, tried much
    # later (ranked low by cutting_face_number), gives a genuinely clean
    # 3-piece split (188.74 + 62.95 + 62.95 mm^3, none of them slivers)
    # that was never reached because the cylinder candidate already broke
    # this loop. Per direct user instruction, merge_sliver_pieces exists
    # for the case the SOURCE solid itself has a genuine, unavoidable
    # geometric imprecision that forces a sliver fragment no matter which
    # candidate is used (see struct_plate3.stp, CLAUDE.md) -- it is a
    # last resort, not something a candidate should be allowed to trigger
    # just by being tried first. `fallback_solids` remembers the first
    # sliver-producing split found (in case nothing better ever turns
    # up); only a split leaving >=2 genuinely non-sliver pieces is
    # accepted immediately.
    fallback_solids = None
    for surf in get_surfaces(solid, omitfaces, scaled_tolerances, options, meta_surface=options.meta_surfaces):
        if original_tolerance is not None:
            Gsolid_set_tolerance(solid, original_tolerance)
        if not solid.is_valid():
            fixed = solid.fix(tolerances.fix_tolerance)
            if fixed.is_valid():
                solid = fixed
        try:
            # build_surface (Can/RoundCorner/... construction, via
            # get_cell_object) can raise -- e.g. round_corner_region's own
            # "this configuration should not exist" sanity-check assertion
            # on a genuinely degenerate near-tangency configuration -- not
            # just return shape=None. Both outcomes mean the same thing to
            # this loop ("this candidate surface could not be built, try
            # the next one"), so both are handled the same way: log and
            # move on, never let one candidate's failure abort the whole
            # decomposition. Confirmed reachable in practice once
            # large_cell_plane_split started feeding many-face solids
            # through this loop -- their synthetic cutting-plane faces
            # create new corner configurations this classifier was never
            # exercised against before.
            surf.build_surface(bbox, tolerances, forward=True)
            if surf.shape is None:
                logger.info(f"Cannot build {surf.Type} surface.")
                continue
            result = Gsplit(
                solid,
                GSolid(surf.shape),
                tolerances,
            )
            comsolid_solids = result.solids
            if result.dropped_no_solid:
                vols = ", ".join(f"{v:.2f}" for v in result.dropped_no_solid)
                logger.warning(
                    f"Gsplit: {len(result.dropped_no_solid)} candidate fragment(s) (volumes: {vols}) never "
                    "resolved to a real solid and were dropped -- this material will NOT be part of any "
                    "cell's boolean expression."
                )
        except Exception as e:
            comsolid_solids = [solid]
            logger.info(f"Failed to build or split base with {surf.Type} surface: {e}")

        if len(comsolid_solids) > 1:
            # A "successful" split (>=2 sane solids returned) is not
            # necessarily a CORRECT one -- confirmed live, 2026-09-12,
            # Big_complex_cell/modelCell_670000.stp: a MultiPlane candidate
            # against a heavily-recut fragment (many nested large_cell_
            # plane_split cuts deep) returned 2 real, valid, tiny slivers
            # (volumes 1204/1475) while BOPAlgo_Splitter silently merged
            # the other ~3.88 million mm^3 of real material away entirely
            # -- a raw split, not filtered by anything downstream, since
            # both kept pieces are genuinely valid and above every size
            # threshold. Re-running the identical (unhealed) base+tool
            # pair confirmed this deterministically: _raw_bop_split itself
            # only ever finds 2 pieces on the live, tolerance-accumulated
            # geometry, but does find all 4 real pieces (matching the base
            # volume exactly) once the SAME base is healed via a STEP
            # round-trip first -- the exact class of BOPAlgo tolerance-weld
            # this file's own STEP-heal retry below already exists to catch,
            # just never triggered here because a (silently wrong) split
            # DID technically happen. Guard against this directly: verify
            # the pieces actually tile the base's own volume before trusting
            # the split at all.
            piece_sum = sum(abs(p.Volume) for p in comsolid_solids)
            orig_vol = abs(solid.Volume)
            if orig_vol > 0 and not volume_within(piece_sum, orig_vol, SPLIT_CANDIDATE_VOLUME_REL_TOL):
                logger.warning(
                    f"Gsplit with a {surf.Type} surface produced {len(comsolid_solids)} piece(s) summing to "
                    f"{piece_sum:.2f}, but the base fragment's own volume is {orig_vol:.2f} -- a likely "
                    "BOPAlgo under-separation (real material silently merged away) rather than a genuine "
                    "split. Discarding this candidate and trying the next one."
                )
                comsolid_solids = [solid]
                continue

            non_sliver_pieces = [p for p in comsolid_solids if not is_sliver_solid(p)]
            if len(non_sliver_pieces) <= 1:
                # At most one of the returned pieces is genuinely real --
                # the rest are the same class of thin-wedge artifact
                # merge_sliver_pieces exists to fuse back later. Keep this
                # as a fallback (only used if the whole candidate search
                # below never finds anything better), but don't treat it
                # as a win: trying the next candidate instead.
                if fallback_solids is None:
                    fallback_solids = comsolid_solids
                comsolid_solids = [solid]
                continue

            new_split = True
            break

    # No candidate surface separated this fragment. A near-tangent BOPAlgo
    # weld can leave a split result BRepCheck-valid yet joined at a
    # junction it papered over rather than resolved -- the signature is
    # (a) edge/vertex tolerances inflated far above split_tolerance (e.g.
    # 0.075 mm vs 1e-4) AND (b) non-manifold edges (shared by != 2 faces).
    # A STEP round-trip (Gheal_topology) rebuilds every face from its clean
    # analytic description, dropping the inflated tolerance and the
    # tolerance-artifact free edges so an ordinary split then separates it;
    # .fix()/.refine() do not. Try it once, only when the fragment is
    # genuinely stuck and carries both parts of that signature -- degenerate
    # tori (huge tolerance, 0 free edges) and sew-repaired shells do not
    # qualify, so the common path pays nothing. Confirmed on
    # RevCC_regression/Big_one_cell__modelCell_670000__solid0_piece52: a
    # 5406 mm^3 fused wedge -> [816.5, 4590.0].
    if not new_split and not healed:
        tol_floor = max(50.0 * tolerances.split_tolerance, TOLERANCE_WELD_FLOOR)
        if Gsolid_max_tolerance(solid) > tol_floor and Gsolid_nonmanifold_edge_count(solid) >= 1:
            rebuilt = Gheal_topology(solid)
            if rebuilt is not None:
                try:
                    # The heal is opportunistic: if the rebuilt solid trips
                    # anything downstream, keep the un-healed fragment --
                    # no worse off than not having tried.
                    return generic_split(rebuilt, options, tolerances, loop, healed=True)
                except Exception:
                    logger.info("STEP round-trip heal retry failed; keeping the original fragment")

    if not new_split and fallback_solids is not None:
        # Nothing cleaner ever turned up (and the tolerance-weld heal
        # retry above either didn't apply or didn't help either) -- this
        # solid genuinely needs a sliver-producing split to decompose at
        # all, matching struct_plate3.stp's own real, unavoidable
        # source-CAD imprecision (see merge_sliver_pieces's own docstring).
        # Only now is that candidate accepted.
        comsolid_solids = fallback_solids
        new_split = True

    if new_split:
        components = []
        for part in comsolid_solids:
            subcomp = generic_split(part, options, tolerances, loop + 1)
            components.extend(subcomp)
    else:
        # NOTE: an earlier version of this branch unconditionally re-ran
        # .fix() on every piece here, based on an initial (wrong) diagnosis
        # of the RoundCorners/rrc23.stp volume-corruption bug -- the real
        # cause turned out to be in split_surfaces's own Gmake_compound
        # wrap/re-extract step (see that function's own comment), not here.
        # Removed: confirmed live on Complex_cell/SCDR_90.stp that an
        # unconditional .fix() at every recursive leaf can itself trigger a
        # real native crash (Standard_Failure: "Courbes non jointives",
        # the same ShapeUpgrade_UnifySameDomain crash class already
        # documented for Mixed/ConeSphere.stp) on a piece that never needed
        # fixing in the first place -- strictly more crash-prone than the
        # single, narrowly-scoped fix in split_surfaces, for no benefit.
        components = comsolid_solids
    return components


def main_split(solids, options, tolerances):
    """decompose in basic solids a list of solids from CAD.

    `solids` is a plain list of GSolid (the top-level input solids, usually
    just one) -- never a native compound: see GCompound's own docstring for
    why this whole pipeline avoids native TopoDS_Compound wrap/re-extract
    cycles for anything other than a one-shot STEP export.
    """
    solid_parts = []

    for solid in solids:
        piece = split_surfaces(solid, options, tolerances)
        solid_parts.extend(piece.Solids)

    return GCompound(solid_parts)
