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
    import FreeCAD
    def vf(v):
        return f'({v.x:6.3f} {v.y:6.3f} {v.z:6.3f})'
    
    bbox = solid.BoundBox
    for i,f in enumerate(solid.Faces):
        f.exportStep(f'f_{i}.stp')
        print(f'face {i}')
        for e in f.Edges:
            if e.Length < 1e-5 :
                continue
            p0,p1 = e.ParameterRange
            pe = 0.5*(p1+p0)
            pos = e.Curve.value(pe)
            dir = e.derivative1At(pe)
            if e.Orientation == "Reversed":
                dir = -dir
            if e.curvatureAt(pe) < 1e-5:
                u,v = f.Surface.parameter(pos)
                normale = f.Surface.normal(u,v)
                if f.Orientation == "Reversed" : 
                    normale = -normale
            else:    
                normale = e.normalAt(pe)
            matvec = dir.cross(normale)
            print(vf(pos),vf(matvec))
            #print(vf(normale),vf(normalf))

    exit()        

    bbox.enlarge(10)
    cleaned = [solid]
    omitfaces = set()

    for surf in get_surfaces(solid, omitfaces, tolerances):
        surf.build_surface(bbox)
        try:
            comsolid = split_bop(solid, [surf.shape], options.splitTolerance, options)
        except:
            comsolid = solid
            logger.info("Failed split base with {surf.shape.Faces[0].Surface} surface")

        if not comsolid.Solids:
            cleaned = solid.Solids
        elif comsolid.Volume == 0:
            cleaned = solid.Solids
        elif len(comsolid.Solids) == 1:
            cleaned = [solid]
        else:
            cleaned = remove_solids(comsolid.Solids, solid.Volume)
        if len(cleaned) > 1:
            new_split = True
            break
    else:
        # loop exit normally (without break)
        new_split = False

    if new_split:
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
