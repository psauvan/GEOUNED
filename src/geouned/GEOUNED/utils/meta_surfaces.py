import Part
import math

from .data_classes import Options,Tolerances, NumericFormat
from .basic_functions_part2 import is_same_plane
from .meta_surfaces_utils import other_face_edge, region_sign, get_adjacent_cylplane, get_join_cone_cyl

twoPi = 2*math.pi
halfPi = 0.5*math.pi

def multiplane_loop(adjacents, multi_list, planes):
    for p in adjacents:
        new_adjacents = multiplane(p, planes)
        for ap in reversed(new_adjacents):
            if ap in multi_list:
                new_adjacents.remove(ap)
        multi_list.extend(new_adjacents)
        multiplane_loop(new_adjacents, multi_list, planes)

def multiplane(p, planes):
    """Found planes adjacent to "p". Region delimited by plane is concanve."""
    Edges = p.OuterWire.Edges
    addplane = [p]
    for e in Edges:
        try:
            type_curve = type(e.Curve)
        except:
            type_curve = None
        if type_curve is not Part.Line:
            continue

        adjacent_plane = other_face_edge(e, p, planes, outer_only=True)
        if adjacent_plane is not None:
            sign = region_sign(p, adjacent_plane)
            if sign == "OR":
                addplane.append(adjacent_plane)
    return addplane


def get_fwdcan_surfaces(cylinder, solidFaces):
    adjacent_planes = get_adjacent_cylplane(cylinder, solidFaces, cornerPlanes=False)

    #for p in adjacent_planes:
    #    r = region_sign(p, cylinder)
    #    if r == "AND":
    #        plane_list.append(p)
    
    if len(adjacent_planes) > 0:
        p1s = adjacent_planes[0:1]
        p2s = []
        r1 = adjacent_planes[0].Surface.Position
        axis = adjacent_planes[0].Surface.Axis
        for p in adjacent_planes[1:]:
            d = p.Surface.Position - r1
            if d.Length < 1e-5:
                p1s.append(p) 
            else:
                d.normalize()
                if abs(axis.dot(d)) < 1e-5:
                    p1s.append(p)
                else :
                    p2s.append(p)
        
        umin,umax,vmin,vmax = cylinder.ParameterRange
        angle = umax-umin
        if abs(angle-twoPi)< 1e-5:
            if p2s :
                return (p1s,p2s),cylinder
            else:
                return (p1s,),cylinder
        else:
            return [],None
    else:
        return [], None
    

def get_revcan_surfaces(cylinder, solidFaces):
    adjacent_planes = get_adjacent_cylplane(cylinder, solidFaces, cornerPlanes=False)
    if len(adjacent_planes) not in (1, 2):
        return None, None

    surfaces = []
    faceindex = set()
    p1 = adjacent_planes[0]
    r1 = region_sign(p1, cylinder)
    if r1 == "OR":
        surfaces.append(p1)
        faceindex.add(p1.Index)

    if len(adjacent_planes) == 2:
        p2 = adjacent_planes[1]
        r2 = region_sign(p2, cylinder)
        if r2 == "OR":
            surfaces.append(p2)
            faceindex.add(p2.Index)

    if len(surfaces) > 0:
        surfaces.append(cylinder)
        faceindex.add(cylinder.Index)
        return surfaces, faceindex
    else:
        return None, None


def get_roundcorner_surfaces(cylinder, Faces):

    adjacent_planes = get_adjacent_cylplane(cylinder, Faces)
    if len(adjacent_planes) != 2:
        return None, None

    p1, p2 = adjacent_planes
    r1,a1 = region_sign(p1, cylinder,outAngle=True)
    r2,a2 = region_sign(p2, cylinder,outAngle=True)
    if r1 != r2 or r1 == "OR":
        return None, None
    if a1 < halfPi+0.1 or a2 < halfPi +0.1 : 
        return None,None
    
    face_index = {cylinder.Index, p1.Index, p2.Index}
    faces = ((cylinder, p1, p2), r1)
    return faces, face_index


def get_revConeCyl_surfaces(face, Faces, omitFaces):
        revCC = get_join_cone_cyl(face,Faces,omitFaces,Tolerances())
        if len(revCC) == 1:
            omitFaces.difference_update(revCC[0].surf_index)
            return None
        elif len(revCC) == 2:
            p1 = revCC[0].Surf.Surf.Plane.Surf
            p2 = revCC[1].Surf.Surf.Plane.Surf
            if is_same_plane(p1,p2, Options(), Tolerances(), NumericFormat()):
                for rcc in revCC:
                    omitFaces.difference_update(rcc.surf_index)
                return None
            else:
                return revCC    
        else:
            return revCC

