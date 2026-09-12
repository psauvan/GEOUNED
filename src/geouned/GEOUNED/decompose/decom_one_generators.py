#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging

from .generators import get_surfaces
from ...geo import (
    GSolid,
    Gheal_topology,
    Gmake_compound,
    Gsolid_max_tolerance,
    Gsolid_nonmanifold_edge_count,
    Gsplit,
)

logger = logging.getLogger("general_logger")


def split_surfaces(solid, options, tolerances):

    solid_components = generic_split(solid, options, tolerances)
    comp = Gmake_compound(solid_components)

    volratio = (comp.Volume - solid.Volume) / solid.Volume
    if volratio > 0.001:
        logger.warning(f"Lost {volratio * 100:6.2f}% of the original volume")

    # A fragment Gsplit accepted as "sane" (BRepCheck-valid + a real
    # volume, via _finalize_split's own filter) can still fail to
    # resolve to a genuine TopoDS_Solid once wrapped into `comp`: a
    # repair step inside _raw_bop_split's cascade (confirmed: Gsliver_
    # heal's own ShapeUpgrade_UnifySameDomain step) can leave a fragment
    # as a bare TopoDS_Compound/Shell that is still topologically valid
    # and still reports a correct .Volume (computed shape-type-
    # agnostically) -- but contains zero real solid leaves. Such a
    # fragment silently disappears from `comp.Solids` (which only counts
    # TopAbs_SOLID nodes, at any depth) with no crash and, critically,
    # no volume-vs-input mismatch -- the `volratio` check above cannot
    # see it, since `comp.Volume` already counts its volume regardless
    # of the wrapper type. Detect it here by comparing fragment counts
    # before vs after `comp` re-parses its own solid content, and warn
    # with every fragment's own volume so the missing one can be
    # identified -- this solid will NOT be part of this cell's boolean
    # expression, a real, currently-unrepaired gap (see CLAUDE.md).
    if len(comp.Solids) != len(solid_components):
        dropped_volume = sum(abs(f.Volume) for f in solid_components) - sum(abs(s.Volume) for s in comp.Solids)
        fragment_volumes = ", ".join(f"{abs(f.Volume):.2f}" for f in solid_components)
        logger.warning(
            f"generic_split produced {len(solid_components)} fragment(s) "
            f"(volumes: {fragment_volumes}) but only {len(comp.Solids)} resolved to "
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

    new_split = False
    for surf in get_surfaces(solid, omitfaces, tolerances, options):
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
            if orig_vol > 0 and abs(piece_sum - orig_vol) > max(1.0e-4 * orig_vol, tolerances.min_solid_volume):
                logger.warning(
                    f"Gsplit with a {surf.Type} surface produced {len(comsolid_solids)} piece(s) summing to "
                    f"{piece_sum:.2f}, but the base fragment's own volume is {orig_vol:.2f} -- a likely "
                    "BOPAlgo under-separation (real material silently merged away) rather than a genuine "
                    "split. Discarding this candidate and trying the next one."
                )
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
        tol_floor = max(50.0 * tolerances.split_tolerance, 1.0e-3)
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

    if new_split:
        components = []
        for part in comsolid_solids:
            subcomp = generic_split(part, options, tolerances, loop + 1)
            components.extend(subcomp)
    else:
        components = comsolid_solids
    return components


def main_split(solidShape, options, tolerances):
    """decompose in basic solids a solid from CAD."""
    solid_parts = []

    for solid in solidShape.Solids:
        piece = split_surfaces(solid, options, tolerances)
        solid_parts.append(piece)

    return Gmake_compound(solid_parts)
