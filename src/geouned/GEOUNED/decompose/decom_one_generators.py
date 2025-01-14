#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging
import Part

from .decom_utils_generator import remove_solids
from .generators import get_surfaces
from ..utils.split_function import split_bop

logger = logging.getLogger("general_logger")


def split_surfaces(solid, options, tolerances):

    solid_components = generic_split(solid, options, tolerances)
    comp = Part.makeCompound(solid_components)
    volratio = (comp.Volume - solid.Volume) / solid.Volume
    if volratio > 0.001:
        logger.info("Lost {volratio*100:6.2f}% of the original volume")
    return comp


def generic_split(solid, options, tolerances):

    stop_loop = False
    bbox = solid.BoundBox
    bbox.enlarge(10)
    cleaned = [solid]
    for surf in get_surfaces(solid, tolerances):
        surf.build_surface(bbox)
        try:
            comsolid = split_bop(solid, [surf.shape], options.splitTolerance, options)
        except:
            comsolid = solid
            logger.info("Failed split base with {surf.shape.Faces[0].Surface} surface")

        if not comsolid.Solids:
            comsolid = solid
        cleaned = remove_solids(solid, comsolid.Solids)
        if len(cleaned) > 1:
            stop_loop = True
            break

    if stop_loop:
        components = []
        for part in cleaned:
            subcomp = generic_split(part, options, tolerances)
            components.extend(subcomp)
    else:
        components = cleaned
    return components


def main_split(solidShape, options, tolerances):
    """decompose in basic solids a solid from CAD."""
    solid_parts = []

    for solid in solidShape.Solids:
        piece = split_surfaces(solid, options, tolerances)
        solid_parts.append(piece)

    return Part.makeCompound(solid_parts)
