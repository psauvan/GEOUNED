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
        logger.info("Lost {volratio*100:6.2f}% of the original volume")
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
    for surf in get_surfaces(solid, omitfaces, tolerances):
        surf.build_surface(bbox, tolerances, forward=True)
        try:
            result = Gsplit(
                solid,
                GSolid(surf.shape),
                tolerances,
            )
            comsolid_solids = result.solids
        except Exception:
            comsolid_solids = [solid]
            logger.info("Failed split base with {surf.shape.Faces[0].Surface} surface")

        if len(comsolid_solids) > 1:
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
