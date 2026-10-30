import math

from .data_constants import twoPi, mask
from ..utils.basic_functions_part1 import twoPimod
from .geometry_gu import other_face_edge
from ...geo import GLine, GPlane, GCylinder, GTorus, Gclassify_curve
from .geometry_gu import ShellFaceGu, is_same_surface
from .meta_surfaces_utils import (
    cyl_plane_region_conf,
    region_sign,
    get_adjacent_cylplane,
    get_adjacent_cylknesurf,
    get_join_cone_cyl,
    closed_cylinder_cone,
    merge_same_surface_faces,
    most_outer_faces,
    commonEdge,
    planar_edges,
    same_curve,
    eligible_plane,
    get_additional_corner_plane,
)
from ...geo.constants import PARAM_ANGLE_TOL, POINT_POINT_TOL
from ...geo.surface_geometry import axes_parallel, axes_perpendicular
from ...geo.surface_geometry import is_same_cylinder_surface


def multiplane_loop(adjacents, multi_list, planes):
    for p in adjacents:
        new_adjacents = multiplane(p, planes)
        for ap in reversed(new_adjacents):
            if ap in multi_list:
                new_adjacents.remove(ap)
        multi_list.extend(new_adjacents)
        multiplane_loop(new_adjacents, multi_list, planes)


def multiplane(master_plane, planes, plane_index, tolerances=None):
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

        # skip_slivers=True: a residual near-zero-area sliver face bridging
        # master_plane to its real neighbor (confirmed live on
        # Solidos/Big_one_cell/modelcell_cut1.stp's piece 36 -- a genuine
        # 5-real-plane solid where one sliver, Area=0.0021, was being read
        # as a 6th "plane" with a material-pointing normal that doesn't
        # actually point into real material) must not be treated as the
        # real adjacent plane itself: walk across it to the real face
        # beyond, matching get_adjacent_cylplane/get_adjacent_cylknesurfFace's
        # own established use of this same parameter.
        result = other_face_edge(e, master_plane, planes, outer_only=True, skip_slivers=True, tolerances=tolerances)
        if result is not None:
            _, _, adjacent_plane = result
            if adjacent_plane.Index in plane_index:
                continue
            sign = region_sign(master_plane, adjacent_plane, tolerances=tolerances)
            if sign == "OR":
                addplane.append(adjacent_plane)
                plane_index.add(adjacent_plane.Index)

    multiplane_list.extend(addplane)
    for p in addplane:
        if not eligible_plane(p, tolerances):
            continue
        multiplane_list.extend(multiplane(p, planes, plane_index, tolerances))

    return multiplane_list


def multiplane_old(p, planes, *, tolerances):
    """Found planes adjacent to "p". Region delimited by plane is concanve."""
    Edges = p.OuterWire.Edges
    addplane = [p]
    for e in Edges:
        type_curve = Gclassify_curve(e)
        if type_curve is not GLine:
            continue

        adjacent_plane = other_face_edge(e, p, planes, outer_only=True)
        if adjacent_plane is not None:
            sign = region_sign(p, adjacent_plane, tolerances=tolerances)
            if sign == "OR":
                addplane.append(adjacent_plane)
    return addplane


def get_fwdcan_surfaces(cylinder, solidFaces, *, tolerances):
    adjacent_planes = get_adjacent_cylplane(cylinder, solidFaces, cornerPlanes=False, tolerances=tolerances)

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
            if d.length < POINT_POINT_TOL:
                p1s.append(p)
            else:
                d = d.normalized()
                if axes_perpendicular(axis, d, tolerances.angle):
                    p1s.append(p)
                else:
                    p2s.append(p)

        umin, umax, vmin, vmax = cylinder.ParameterRange
        angle = umax - umin
        if abs(angle - twoPi) < PARAM_ANGLE_TOL:
            if p2s:
                return (p1s, p2s), cylinder
            else:
                return (p1s,), cylinder
        else:
            return [], None
    else:
        return [], None


def get_can_surfaces(cylinder, solidFaces, *, tolerances):
    cylinder_shell, faceindex, closed = closed_cylinder_cone(cylinder, solidFaces, tolerances=tolerances)
    if not closed:
        return None, None

    ext_faces = get_adjacent_cylknesurf(cylinder_shell, solidFaces, tolerances=tolerances)
    surfaces = [cylinder_shell]
    cyl_value = 1 if cylinder_shell.Orientation == "Reversed" else -1
    # commonEdge's own ShellGu branch returns (edges, matching_face)
    # instead of a bare edges list -- account for that here (same
    # shell/no-shell dispatch build_can_params already uses) since
    # cylinder_shell can be either, depending on whether the seed's
    # analytic surface is split into several same-surface pieces.
    is_shell = isinstance(cylinder_shell, ShellFaceGu)

    for s in ext_faces:
        if type(s.Surface) is GCylinder:
            if abs(s.Surface.Radius - cylinder.Surface.Radius) < tolerances.value:
                # Same-radius adjacent cylinder -- deliberately NOT also
                # requiring is_parallel(s.Axis, cylinder.Axis) here: any s
                # reaching this point already passed get_adjacent_cylknesurf's
                # own is_same_surface exclusion, which (for a cylinder) means
                # same radius + same axis *line*, not just parallel direction.
                # So an is_parallel check here could only ever pass for a
                # separate, laterally-offset (non-collinear) parallel
                # cylinder of the same radius -- never for a "broken
                # cylinder" (2 straight segments of the same radius meeting
                # at a real kink, where the segments' axes are NOT parallel
                # by definition), which is exactly the case this branch is
                # meant to handle. Confirmed live, 2026-08-23, on
                # Solidos/test_models/Cans/pipe1.stp (3 straight segments,
                # same radius, kinked at 2 joints): the real kink-neighbor
                # cylinder always has is_parallel=False, so the old
                # is_parallel-gated condition was unreachable dead code for
                # this scenario, falling through to the generic
                # commonEdge/region_sign path below instead.
                result = commonEdge(cylinder_shell, s, outer1_only=True, outer2_only=False)
                edges = result[0] if is_shell else result
                if edges is not None:
                    if planar_edges(edges, tolerances=tolerances):
                        # adjacent cylinder has same radius (parallel or,
                        # for a broken-cylinder kink, at an angle).
                        # build_can_params's r is None branch never reads omit -- True
                        # here only keeps this a 3-tuple like every other entry.
                        surfaces.append((s, None, True))
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
        # jumble of unrelated edges. Uses cylinder_shell (the merged
        # same-surface shell built above), not the raw seed `cylinder`
        # -- a same-surface cylinder split into several pieces (e.g.
        # Pipe0041's own R=395 corner, split into 3) can have its real
        # closing edge belong to a *different* piece than the one that
        # happened to be passed in as the seed; commonEdge's own ShellGu
        # branch already searches every piece, it just wasn't being
        # given the shell here.
        result = commonEdge(cylinder_shell, s, outer1_only=True, outer2_only=False)
        edges = result[0] if is_shell else result
        if edges is None or not same_curve(edges, tolerances=tolerances):
            return None, None

        r = region_sign(cylinder_shell, s, tolerances=tolerances)
        if r == "OR" and cylinder.Orientation == "Forward":
            # Forward cylinder + OR is not a valid Can configuration at
            # all (no combination of AND/OR-with-continuity can represent
            # it) -- reject the whole Can. The caller (next_Can) still
            # needs `s` to be tried as an ordinary simple cutting surface
            # before the main cylinder itself, since region_sign's OR
            # result means `s` cuts more cleanly here than `cylinder`
            # does.
            return None, None

        omit = True
        if r == "AND" and cylinder.Orientation == "Reversed":
            # the Can's "mouth" is open on this end (no real closing
            # surface AND-bounds the main cylinder here) -- `s`'s own
            # natural extension stands in for the missing closure, using
            # the opposite-orientation continuity formula (can_region()
            # reproduces it automatically once `r`/orientation are
            # swapped like this, reusing the same Forward/AND-Reversed/OR
            # formulas already validated for a real closing surface).
            omit = False
            r = "OR"
        surfaces.append((s, r, omit))

        if omit:
            faceindex.add(s.Index)

    if len(ext_faces) > 2:
        ext_faces, remove_index = most_outer_faces(cylinder, ext_faces, tolerances=tolerances)
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

    if len(surfaces) != 3:
        # A real Can always needs exactly 2 closing ends (cylinder_shell
        # + 2 secondary surfaces) -- build_can_params unconditionally
        # unpacks 3 values. Fewer than that means this piece's own
        # topology doesn't actually offer 2 distinct closures (e.g. an
        # intermediate decomposition fragment whose cylindrical face has
        # no real end caps at all) -- not a valid Can, reject cleanly
        # rather than crashing on unpack. Confirmed reachable with a
        # legitimately different (not wrong) decomposition path: the two
        # geometry backends this project supports don't always split a
        # solid into the same intermediate pieces.
        return None, None

    return surfaces, faceindex


def get_tcone_surfaces(cone, solidFaces, *, tolerances):
    cone_shell, faceindex, closed = closed_cylinder_cone(cone, solidFaces, tolerances=tolerances)
    if not closed:
        return None, None

    ext_faces = get_adjacent_cylknesurf(cone_shell, solidFaces, tolerances=tolerances)
    if len(ext_faces) == 1:
        return None, None
    surfaces = [cone_shell]
    kne_value = 1 if cone_shell.Orientation == "Reversed" else -1

    for s in ext_faces:
        if type(s.Surface) is not GPlane:
            return None, None

        r = region_sign(cone_shell, s, tolerances=tolerances)
        if r == "OR" and cone.Orientation == "Forward":
            # Forward cone + OR is not a valid TCone configuration at all
            # (no combination of AND/OR-with-continuity can represent it)
            # -- reject the whole TCone, mirroring get_can_surfaces's own
            # identical rejection for a Forward cylinder.
            return None, None

        omit = True
        if r == "AND" and cone.Orientation == "Reversed":
            # The TCone's "mouth" is open on this end (no real closing
            # surface AND-bounds the main cone here) -- same open-mouth
            # continuity conversion as get_can_surfaces (see its own
            # comment for the full reasoning): `s`'s own natural extension
            # stands in for the missing closure, using the
            # opposite-orientation continuity formula. build_tcone_params
            # already has the matching `if not omit: normal = -normal`
            # branch to realize this (byte-identical to build_can_params's
            # own), it just was never reached before this fix, since this
            # function never produced omit=False nor converted AND to OR.
            omit = False
            r = "OR"
        surfaces.append((s, r, omit))

        if omit:
            faceindex.add(s.Index)

    if len(ext_faces) > 2:
        ext_faces, remove_index = most_outer_faces(cone, ext_faces, tolerances=tolerances)
        for s in reversed(surfaces[1:]):
            if s[0] not in ext_faces:
                surfaces.remove(s)
                if s[0].Index in remove_index:
                    # s[0].Index was only ever added to faceindex when its
                    # own `omit` was True (see the open-mouth continuity
                    # branch above) -- discard rather than remove so a
                    # never-added (omit=False) face doesn't raise KeyError.
                    faceindex.discard(s[0].Index)

    if len(ext_faces) != 2:
        return None, None
    else:
        return surfaces, faceindex


def _farthest_corner_plane_pair(cyl_shell, adjacent_planes):
    """Among more than 2 corner-plane candidates `get_adjacent_cylplane`
    already found (and already filtered for a normal orthogonal to the
    cylinder's own axis -- that part of the selection is correct as is,
    this only covers the >2 case it used to reject outright), pick the
    pair whose own touching edges (`item[1]`) are farthest apart --
    measured as the arc they draw over the real, trimmed cylindrical
    FACE (`cyl_shell.Surface.parameter`, the one common reference frame
    `ShellFaceGu._U_parameter_faces` already uses for a multi-piece
    shell's own overall extent), not a straight-line 3D distance, and
    not each candidate's own, possibly differently-originated, local
    surface parametrization (two real face pieces of the exact same
    analytic cylinder can have a different U origin -- the documented
    frame-mismatch class of bug this project already fixed once for
    `gen_plane_cone`/`gen_plane_cylinder`, see CLAUDE.md).

    Each candidate's own azimuthal position is unwrapped relative to
    the first one (add/subtract a full turn until within half a turn of
    it) before comparing -- safe here because the round corner's own
    cylinder is never a fully closed face (the caller already rejected
    that case earlier), so every candidate's real position lies within
    one contiguous, less-than-a-full-turn span, with no genuine
    ambiguity about which way around the branch cut sits."""
    radius = cyl_shell.Surface.Radius
    positions = []
    u_ref = None
    for item in adjacent_planes:
        point = item[1].Vertexes[0]
        u = cyl_shell.Surface.parameter(point)[0]
        if u_ref is None:
            u_ref = u
        else:
            while u - u_ref > math.pi:
                u -= twoPi
            while u - u_ref < -math.pi:
                u += twoPi
        positions.append(u)

    best_pair = None
    best_arc = -1.0
    for i in range(len(adjacent_planes)):
        for j in range(i + 1, len(adjacent_planes)):
            arc = abs(positions[i] - positions[j]) * radius
            if arc > best_arc:
                best_arc = arc
                best_pair = (adjacent_planes[i], adjacent_planes[j])
    return best_pair


def _edge_mid_point(edge):
    """Point at the middle of edge's own parameter range -- for a straight
    (GLine) edge this is just its geometric midpoint."""
    first, last = edge.ParameterRange
    return edge.value_at(0.5 * (first + last))


def _edge_point_distance(edge, point):
    """Real 3D distance from `point` to the bounded edge `edge`: the
    orthogonal projection when it falls within edge's own parameter
    range, else the nearest of its two endpoints.
    GeomAPI_ProjectPointOnCurve (behind GEdge.parameter) only searches
    for an extremum within the curve's own trimmed range and raises
    StdFail_NotDone when none exists there -- confirmed live, 2026-10-30,
    Misc/PiezaDavid.stp, for a point whose true orthogonal foot falls
    beyond one of edge's own ends."""
    first, last = edge.ParameterRange
    try:
        closest = edge.value_at(edge.parameter(point))
    except Exception:
        v0 = edge.value_at(first)
        v1 = edge.value_at(last)
        d0 = (point - v0).length
        d1 = (point - v1).length
        closest = v0 if d0 <= d1 else v1
    return (point - closest).length


def _farthest_plane_point_from_edge(plane_face, edge):
    """Among the real vertices of plane_face's own OuterWire boundary
    (not plane_face.ParameterRange's own corners -- plane_face's real
    trimmed shape need not be a rectangle, so a ParameterRange corner can
    fall outside the real face entirely, confirmed live, 2026-10-30,
    RoundCorners/shed_part.stp), the one whose real 3D distance to
    `edge` is maximum."""
    best_point = None
    best_dist = -1.0
    for point in plane_face.OuterWire.OrderedVertexes:
        dist = _edge_point_distance(edge, point)
        if dist > best_dist:
            best_dist = dist
            best_point = point
    return best_point


def _project_onto_axis_plane(point, axis, plane_point):
    """Project `point` onto the plane through `plane_point` perpendicular
    to `axis`."""
    diff = point - plane_point
    return point - diff.dot(axis) * axis


def get_roundcorner_surfaces(cylinder, Faces, cylinders_set, level=0, solid=None, *, tolerances):

    rc_list = []
    face_index = set()

    # A boolean cut can split the round corner's own cylinder into several
    # contiguous pieces (e.g. a residual-cut artifact bridging two pieces,
    # or a genuine multi-piece split) -- each piece's own edges may only
    # reach one of the two bounding corner planes. Merge same-surface
    # contiguous pieces into one ShellGu first (mirrors closed_cylinder_cone's
    # treatment of Can/TCone) so the corner-plane search sees the whole
    # feature's boundary, not just whichever piece `cylinder` happens to be.
    cyl_shell = merge_same_surface_faces(cylinder, Faces, tolerances=tolerances)
    if type(cyl_shell) is ShellFaceGu:
        umin, umax, _, _ = cyl_shell.U_parameter_range
    else:
        umin, umax, _, _ = cyl_shell.ParameterRange
    closed = twoPimod(abs(umax - umin)) == 0.0
    if closed:
        return None, None

    adjacent_planes = get_adjacent_cylplane(cyl_shell, Faces, cornerPlanes=True, tolerances=tolerances)
    if len(adjacent_planes) < 2:
        return None, None
    elif len(adjacent_planes) == 2:
        ep1, ep2 = adjacent_planes
    else:
        ep1, ep2 = _farthest_corner_plane_pair(cyl_shell, adjacent_planes)
    p1, p2 = ep1[3], ep2[3]
    e1, e2 = ep1[1], ep2[1]

    # cyl_plane_region_conf still takes the single seed `cylinder` (not the
    # shell) -- same pattern get_can_surfaces uses for its own per-endpoint
    # computations (commonEdge/most_outer_faces against the original face,
    # only the closure/adjacency search itself uses the merged shell).
    configuration, switched = cyl_plane_region_conf(cyl_shell, ep1, ep2, solid=solid, tolerances=tolerances)
    if configuration is None:
        return None, None
    # check if not degenerated round corner
    # if degenerated discard it

    if switched:
        p1, p2 = p2, p1
        e1, e2 = e2, e1

    # 4 points, all lying in one common plane perpendicular to the
    # cylinder's own axis (the exact height along the axis is arbitrary --
    # here the height of e1/e2's own midpoints): the point of e1, the
    # point of e2, and, for each corner plane, the point of that plane
    # farthest from its own tangent edge, projected onto that same
    # perpendicular plane.
    axis = cyl_shell.Surface.Axis
    point_e1 = _edge_mid_point(e1)
    point_e2 = _edge_mid_point(e2)
    point_p1 = _project_onto_axis_plane(_farthest_plane_point_from_edge(p1, e1), axis, point_e1)
    point_p2 = _project_onto_axis_plane(_farthest_plane_point_from_edge(p2, e2), axis, point_e2)
    corner_points = (point_p1, point_e1, point_e2, point_p2)

    """    
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
    """
    fwd_cyl = configuration & mask.fwd_cyl == mask.fwd_cyl

    if type(cyl_shell) is ShellFaceGu:
        face_index.update(cyl_shell.Indexes)
        cylinders_set.update(cyl_shell.Indexes)
    else:
        face_index.add(cylinder.Index)
    face_index.update({p1.Index, p2.Index})

    if is_same_surface(p1.Surface, p2.Surface, tolerances=tolerances):
        gpa = None
    else:
        # ep1/ep2 (not cyl): if the round corner's own cylinder was
        # split into several contiguous pieces, p1/p2 may each only be
        # reachable from a different piece, possibly through a residual
        # sliver bridging them -- get_additional_corner_plane needs the
        # exact touching edge/face get_adjacent_cylplane already found,
        # not cyl's own (possibly non-touching) edge.
        gpa = get_additional_corner_plane(cyl_shell)
    rc_list.append((cylinder, p1, p2, gpa, corner_points, (configuration, fwd_cyl)))

    for newplane in (p1, p2):
        for edge in newplane.OuterWire.Edges:
            # skip_slivers: the next round corner's own cylinder may be
            # bridged to this plane by a residual sliver (same reasoning as
            # the corner-plane search above) -- walk through it instead of
            # stopping there and silently failing to chain into a
            # MultiRoundCorner.
            result = other_face_edge(edge, newplane, Faces, skip_slivers=True, tolerances=tolerances)
            if result is None:
                continue
            _, _, f = result
            if type(f.Surface) != GCylinder:
                continue
            if f.Index in cylinders_set:
                continue
            if not axes_parallel(f.Surface.Axis, cylinder.Surface.Axis, tolerances.cyl_angle):
                continue

            cylinders_set.add(f.Index)
            if is_same_cylinder_surface(f.Surface, cylinder.Surface, tolerances):
                continue

            rc, newindex = get_roundcorner_surfaces(f, Faces, cylinders_set, level + 1, solid=solid, tolerances=tolerances)
            if rc is None:
                cylinders_set.remove(f.Index)
                continue

            rc_list.extend(rc)
            face_index.update(newindex)
            break

    return rc_list, face_index


def get_revConeCyl_surfaces(revcc_shell, Faces, multifaces, omitFaces, tolerances):
    return get_join_cone_cyl(revcc_shell, Faces, multifaces, omitFaces, tolerances)
