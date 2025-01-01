#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging
import Part

from .decom_utils import extract_surfaces, remove_solids, get_surfaces
from ..utils import functions as UF

logger = logging.getLogger("general_logger")


def split_surfaces(solid, options, tolerances, numeric_format):

    surfaces = get_surfaces(solid, options, tolerances, numeric_format)

    solid_components = generic_split(solid, surfaces, options, tolerances, numeric_format)
    comp = Part.makeCompound(solid_components)
    volratio = (comp.Volume - solid.Volume) / solid.Volume
    if volratio > 0.001:
        logger.info("Lost {volratio*100:6.2f}% of the original volume")
    return comp, 0


def generic_split(solid, surfaces: list, options, tolerances, numeric_format):

    stop_loop = False
    bbox = solid.BoundBox
    bbox.enlarge(10)
    for surf in surfaces:
        surf.build_surface(bbox)
        try:
            comsolid = UF.split_bop(solid, [surf.shape], options.splitTolerance, options)
        except:
            logger.info("Failed split base with {surf.shape.Faces[0].Surface} surface")

        if not comsolid.Solids:
            comsolid = solid
        cleaned, err = remove_solids(comsolid.Solids)
        if len(cleaned) > 1:
            stop_loop = True
            break

    if stop_loop:
        components = []
        for part in cleaned:
            new_surfaces = get_surfaces(part, options, tolerances, numeric_format)
            subcomp = generic_split(part, new_surfaces, options, tolerances, numeric_format)
            components.extend(subcomp)
    else:
        components = cleaned
    return components


def main_split(solidShape, options, tolerances, numeric_format):
    """decompose in basic solids a solid from CAD."""
    solid_parts = []

    for solid in solidShape.Solids:
        piece, err = split_surfaces(solid, options, tolerances, numeric_format)
        solid_parts.append(piece)

    return Part.makeCompound(solid_parts), err
