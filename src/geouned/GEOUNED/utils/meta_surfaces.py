import Part

from .data_classes import Tolerances
from .data_constants import twoPi, mask
from .basic_functions_part2 import is_parallel, is_same_cylinder
from .geometry_gu import CylinderGu, TorusGu, other_face_edge
from .meta_surfaces_utils import (
    cyl_plane_region_conf,
    region_sign,
    get_adjacent_cylplane,
    get_adjacent_cylsurf,
    get_join_cone_cyl,
    closed_cylinder,
    most_outer_faces,
    commonEdge,
    planar_edges,
)


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
    cyl_value = 1 if cylinder_shell.Orientation == "Reversed" else -1

    for s in ext_faces:
        if type(s.Surface) is CylinderGu:
            if abs(s.Surface.Radius - cylinder.Surface.Radius) < 1e-6:
                edges = commonEdge(cylinder, s, outer1_only=True, outer2_only=True)
                if edges is not None:
                    if planar_edges(edges):
                        surfaces.append((s, None))
                        continue
        elif type(s.Surface) is TorusGu:
            return None, None

        r = region_sign(cylinder_shell, s)
        surfaces.append((s, r))
        s_value = 1 if r == "AND" else -1
        if s_value != cyl_value:
            faceindex.add(s.Index)  # will not split with adjacent surface

    if len(ext_faces) > 2:
        ext_faces = most_outer_faces(cylinder, ext_faces)
        for s in reversed(surfaces[1:]):
            if s[0] not in ext_faces:
                surfaces.remove(s)

    return surfaces, faceindex


def get_roundcorner_surfaces(cylinder, Faces, cylinders_set):

    rc_list = []
    face_index = set()

    adjacent_planes = get_adjacent_cylplane(cylinder, Faces, cornerPlanes=True)
    if len(adjacent_planes) != 2:
        return None, None

    ep1, ep2 = adjacent_planes
    p1, p2 = ep1[1], ep2[1]

    configuration = cyl_plane_region_conf(cylinder, ep1, ep2)
    fwd_corner = configuration & mask.fwd_corner == mask.fwd_corner

    face_index.update({cylinder.Index, p1.Index, p2.Index})
    rc_list.append((cylinder, p1, p2, (configuration, fwd_corner)))

    for newplane in (p1, p2):
        for edge in newplane.OuterWire.Edges:
            f = other_face_edge(edge, newplane, Faces)
            if type(f.Surface) != CylinderGu:
                continue
            if f.Index in cylinders_set:
                continue
            if not is_parallel(f.Surface.Axis, cylinder.Surface.Axis):
                continue

            cylinders_set.add(f.Index)
            if is_same_cylinder(f.Surface, cylinder.Surface):
                continue

            rc, newindex = get_roundcorner_surfaces(f, Faces, cylinders_set)
            if rc is None:
                cylinders_set.remove(f.Index)
                continue

            # rc[0][3][1] fwd_corner value of new round corner
            if fwd_corner != rc[0][3][1]:
                cylinders_set.remove(f.Index)
                continue

            rc_list.extend(rc)
            face_index.update(newindex)
            break

    return rc_list, face_index


def get_revConeCyl_surfaces(face, Faces, multifaces, omitFaces):
    return get_join_cone_cyl(face, -1, Faces, multifaces, omitFaces, Tolerances())
