import Part
import math

from .data_classes import Options, Tolerances, NumericFormat
from .basic_functions_part2 import is_same_plane
from .geometry_gu import PlaneGu, other_face_edge
from .meta_surfaces_utils import (
    region_sign,
    get_adjacent_cylplane,
    get_adjacent_cylsurf,
    get_join_cone_cyl,
    closed_cylinder,
)

twoPi = 2 * math.pi
halfPi = 0.5 * math.pi


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

    # for p in adjacent_planes:
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
                else:
                    p2s.append(p)

        umin, umax, vmin, vmax = cylinder.ParameterRange
        angle = umax - umin
        if abs(angle - twoPi) < 1e-5:
            if p2s:
                return (p1s, p2s), cylinder
            else:
                return (p1s,), cylinder
        else:
            return [], None
    else:
        return [], None


def get_can_surfaces(cylinder, solidFaces):
    cylinder_shell, faceindex, closed = closed_cylinder(cylinder, solidFaces)
    if not closed:
        return None, None

    ext_faces = get_adjacent_cylsurf(cylinder_shell, solidFaces)

    surfaces = [cylinder_shell]

    for s, outer in ext_faces:
        r = region_sign(cylinder_shell, s)
        surfaces.append((s, r))
        if outer:  # current face is edge is an outer edge of the adjacent face
            faceindex.add(s.Index)

    return surfaces, faceindex


def get_roundcorner_surfaces(cylinder, Faces):

    adjacent_planes = get_adjacent_cylplane(cylinder, Faces)
    if len(adjacent_planes) != 2:
        return None, None

    p1, p2 = adjacent_planes
    r1, a1 = region_sign(p1, cylinder, outAngle=True)
    r2, a2 = region_sign(p2, cylinder, outAngle=True)

    if r1 != r2 or r1 == "OR":
        return None, None
    if abs(a1 - math.pi) > 0.1 or abs(a2 - math.pi) > 0.1:
        return None, None

    face_index = {cylinder.Index, p1.Index, p2.Index}
    faces = ((cylinder, p1, p2), r1)
    return faces, face_index


def get_revConeCyl_surfaces(face, Faces, multifaces, omitFaces):
    return get_join_cone_cyl(face, -1, Faces, multifaces, omitFaces, Tolerances())
