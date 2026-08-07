from .data_classes import Tolerances
from .data_constants import twoPi, mask
from .basic_functions_part2 import is_parallel, is_same_cylinder
from .geometry_gu import other_face_edge
from ...geo import GLine, GPlane, GCylinder, GTorus, Gclassify_curve
from .meta_surfaces_utils import (
    cyl_plane_region_conf,
    region_sign,
    get_adjacent_cylplane,
    get_adjacent_cylknesurf,
    get_join_cone_cyl,
    closed_cylinder_cone,
    most_outer_faces,
    commonEdge,
    planar_edges,
    same_curve,
    eligible_plane,
)


def multiplane_loop(adjacents, multi_list, planes):
    for p in adjacents:
        new_adjacents = multiplane(p, planes)
        for ap in reversed(new_adjacents):
            if ap in multi_list:
                new_adjacents.remove(ap)
        multi_list.extend(new_adjacents)
        multiplane_loop(new_adjacents, multi_list, planes)


def multiplane(master_plane, planes, plane_index):
    """Found planes adjacent to "p". Region delimited by plane is concanve."""
    Edges = master_plane.OuterWire.Edges

    if master_plane.Index not in plane_index:
        multiplane_list = [master_plane]
        plane_index.add(master_plane.Index)
    else:
        multiplane_list = []

    addplane = []
    for e in Edges:
        type_curve = Gclassify_curve(e)
        if type(type_curve) is not GLine:
            continue

        adjacent_plane = other_face_edge(e, master_plane, planes, outer_only=True)
        if adjacent_plane is not None:
            if adjacent_plane.Index in plane_index:
                continue
            sign = region_sign(master_plane, adjacent_plane)
            if sign == "OR":
                addplane.append(adjacent_plane)
                plane_index.add(adjacent_plane.Index)

    multiplane_list.extend(addplane)
    for p in addplane:
        if not eligible_plane(p):
            continue
        multiplane_list.extend(multiplane(p, planes, plane_index))

    return multiplane_list


def multiplane_old(p, planes):
    """Found planes adjacent to "p". Region delimited by plane is concanve."""
    Edges = p.OuterWire.Edges
    addplane = [p]
    for e in Edges:
        type_curve = Gclassify_curve(e)
        if type_curve is not GLine:
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
            if d.length < 1e-5:
                p1s.append(p)
            else:
                d = d.normalized()
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
    cylinder_shell, faceindex, closed = closed_cylinder_cone(cylinder, solidFaces)
    if not closed:
        return None, None

    ext_faces = get_adjacent_cylknesurf(cylinder_shell, solidFaces)
    surfaces = [cylinder_shell]
    cyl_value = 1 if cylinder_shell.Orientation == "Reversed" else -1

    for s in ext_faces:
        if type(s.Surface) is GCylinder:
            if abs(s.Surface.Radius - cylinder.Surface.Radius) < 1e-6 and is_parallel(
                s.Surface.Axis, cylinder.Surface.Axis, Tolerances().angle
            ):
                edges = commonEdge(cylinder, s, outer1_only=True, outer2_only=False)
                if edges is not None:
                    if planar_edges(edges):
                        surfaces.append((s, None)) # adjacent cylinder has same radius and is parallel to cylinder.
                        continue
        elif type(s.Surface) is GTorus:
            return None, None

        # A Can's end can legitimately be closed by any surface type
        # (plane, sphere, cylinder, cone -- even a non-planar boundary,
        # e.g. a cylinder perpendicular to this one), but the whole end
        # must be one clean boundary with that single surface, not
        # several edges left over from an irregular cut. same_curve
        # checks curve identity rather than planarity, so it accepts a
        # genuinely non-planar intersection while still rejecting a
        # jumble of unrelated edges.
        edges = commonEdge(cylinder, s, outer1_only=True, outer2_only=False)
        if edges is None or not same_curve(edges):
            return None, None

        r = region_sign(cylinder_shell, s)
        omit = True
        if r == "OR" and cylinder.Orientation == "Forward":
            omit = False
            r = "AND"   
        elif r == "AND" and cylinder.Orientation == "Reversed":
            omit = False    
            r = "OR"
        surfaces.append((s, r, omit))

        if omit:
            faceindex.add(s.Index)

    if len(ext_faces) > 2:
        ext_faces, remove_index = most_outer_faces(cylinder, ext_faces)
        if remove_index:
            # remove_index is non-empty when one end has faces belonging
            # to neither extreme face's own surface -- i.e. that end is
            # not a clean single-surface closure (e.g. an irregular cut
            # leaving several unrelated small faces), so this cylinder
            # isn't a real Can. Without this check, most_outer_faces'
            # pruning silently discards those mismatched faces and picks
            # one of the remaining ones as if it cleanly closed the end.
            return None, None
        for s in reversed(surfaces[1:]):
            if s[0] not in ext_faces:
                surfaces.remove(s)

    return surfaces, faceindex


def get_tcone_surfaces(cone, solidFaces):
    cone_shell, faceindex, closed = closed_cylinder_cone(cone, solidFaces)
    if not closed:
        return None, None

    ext_faces = get_adjacent_cylknesurf(cone_shell, solidFaces)
    if len(ext_faces) == 1:
        return None, None
    surfaces = [cone_shell]
    kne_value = 1 if cone_shell.Orientation == "Reversed" else -1

    for s in ext_faces:
        if type(s.Surface) is not GPlane:
            return None, None

        r = region_sign(cone_shell, s)
        surfaces.append((s, r, True))
        # s_value = 1 if r == "AND" else -1
        # if s_value != kne_value:
        #    faceindex.add(s.Index)  # will not split with adjacent surface
        faceindex.add(s.Index)  # same check as get_can_surfces

    if len(ext_faces) > 2:
        ext_faces, remove_index = most_outer_faces(cone, ext_faces)
        for s in reversed(surfaces[1:]):
            if s[0] not in ext_faces:
                surfaces.remove(s)
                if s[0].Index in remove_index:
                    faceindex.remove(s[0].Index)

    if len(ext_faces) != 2:
        return None, None
    else:
        return surfaces, faceindex


def get_roundcorner_surfaces(cylinder, Faces, cylinders_set, level=0):

    rc_list = []
    face_index = set()

    adjacent_planes = get_adjacent_cylplane(cylinder, Faces, cornerPlanes=True)
    if len(adjacent_planes) != 2:
        return None, None

    ep1, ep2 = adjacent_planes
    p1, p2 = ep1[1], ep2[1]

    configuration = cyl_plane_region_conf(cylinder, ep1, ep2)
    # check if not degenerated round corner
    # if degenerated discard it
    AND_cyl_p1 = configuration & mask.p1_cyl == mask.p1_cyl
    AND_cyl_p2 = configuration & mask.p2_cyl == mask.p2_cyl
    if not AND_cyl_p1:
        AND_p1_pd = configuration & mask.p1_pd == mask.p1_pd
        if AND_p1_pd:
            return None, None
    if not AND_cyl_p2:
        AND_p2_pd = configuration & mask.p2_pd == mask.p2_pd
        if AND_p2_pd:
            return None, None

    fwd_cyl = configuration & mask.fwd_cyl == mask.fwd_cyl

    face_index.update({cylinder.Index, p1.Index, p2.Index})
    rc_list.append((cylinder, p1, p2, (configuration, fwd_cyl)))

    for newplane in (p1, p2):
        for edge in newplane.OuterWire.Edges:
            f = other_face_edge(edge, newplane, Faces)
            if type(f.Surface) != GCylinder:
                continue
            if f.Index in cylinders_set:
                continue
            if not is_parallel(f.Surface.Axis, cylinder.Surface.Axis):
                continue

            cylinders_set.add(f.Index)
            if is_same_cylinder(f.Surface, cylinder.Surface):
                continue

            rc, newindex = get_roundcorner_surfaces(f, Faces, cylinders_set, level + 1)
            if rc is None:
                cylinders_set.remove(f.Index)
                continue

            rc_list.extend(rc)
            face_index.update(newindex)
            break

    return rc_list, face_index


def get_revConeCyl_surfaces(face, Faces, multifaces, omitFaces):
    return get_join_cone_cyl(face, Faces, multifaces, omitFaces, Tolerances())
