#
# Set of useful functions used in different parts of the code
#
import logging
import math

logger = logging.getLogger("general_logger")

from .boolean_function import BoolVariable
from .geometry_gu import ShellGu, is_same_surface
from .geouned_classes import GeounedSurface
from .data_classes import NumericFormat, Options, Tolerances
from .meta_surfaces import multiplane, get_can_surfaces, get_tcone_surfaces, get_roundcorner_surfaces, get_revConeCyl_surfaces
from .meta_surfaces_utils import commonEdge, commonVertex, no_convex, planar_edges, eligible_plane, material_direction
from ..decompose.decom_utils_generator import cks_edge_plane
from ..conversion.cell_definition_functions import cone_apex_plane
from .basic_functions_part2 import is_same_plane
from ...geo import GPlane, GCylinder, GCone, GSphere, Gmake_box, vector_geometry
from .basic_functions_part1 import shapes_in_contact


def get_box(comp, enlargeBox):
    # comp is always a GeounedSolid here, whose BoundBox is a GBoundBox
    box = comp.BoundBox.enlarged(enlargeBox)
    return Gmake_box(box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)


def get_multiplanes(solidFaces, omit_faces_set=None, tolerances=None):
    """identify and return all multiplanes in the solid."""

    if omit_faces_set is None:
        omit_faces_set = set()
        one_value_return = False
    else:
        one_value_return = True

    planes = []
    for f in solidFaces:
        if isinstance(f.Surface, GPlane):
            planes.append(f)

    multiplane_list = []
    multiplane_objects = []

    for p in planes:
        if p.Index in omit_faces_set:
            continue
        if not eligible_plane(p, tolerances):
            continue
        mp_plane_index = set()
        mplanes = multiplane(p, planes, mp_plane_index, tolerances)
        if len(mplanes) != 1:
            if no_convex(mplanes):
                mp_params = build_multip_params(mplanes)
                mp = GeounedSurface(("MultiPlane", mp_params))
                if mp.Surf.PlaneNumber < 2:
                    continue
                for pp in mplanes:
                    omit_faces_set.add(pp.Index)
                multiplane_list.append(mplanes)
                multiplane_objects.append(mp)

    if one_value_return:
        return multiplane_objects
    else:
        return multiplane_objects, omit_faces_set


def get_Can(solidFaces, canface_index=None):
    """identify and return all can type in the solid."""

    if canface_index is None:
        canface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    can_list = []
    for f in solidFaces:
        if isinstance(f.Surface, GCylinder):
            if f.Index in canface_index:
                continue
            cs, surfindex = get_can_surfaces(f, solidFaces)
            if cs is not None:
                params = build_can_params(cs)
                if params is not None:
                    gc = GeounedSurface(("Can", params, f.Orientation))
                    can_list.append(gc)
                    canface_index.update(surfindex)

    if one_value_return:
        return can_list
    else:
        return can_list, canface_index


def get_TCone(solidFaces, Tconeface_index=None):
    """identify and return all can type in the solid."""

    if Tconeface_index is None:
        Tconeface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    tcone_list = []
    for f in solidFaces:
        if isinstance(f.Surface, GCone):
            if f.Index in Tconeface_index:
                continue
            cs, surfindex = get_tcone_surfaces(f, solidFaces)
            if cs is not None:
                gc = GeounedSurface(("TCone", build_tcone_params(cs), f.Orientation))
                tcone_list.append(gc)
                Tconeface_index.update(surfindex)

    if one_value_return:
        return tcone_list
    else:
        return tcone_list, Tconeface_index


def get_roundCorner(solidFaces, cornerface_index=None, solid=None):
    """identify and return all roundcorner type in the solid."""
    if cornerface_index is None:
        cornerface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    corner_list = []
    for f in solidFaces:
        if isinstance(f.Surface, GCylinder):
            if f.Index in cornerface_index:
                continue
            rc, surfindex = get_roundcorner_surfaces(f, solidFaces, {f.Index}, solid=solid)
            if rc is not None:
                cornerface_index.update(surfindex)
                rc_list, plane_list, multi_round, orientation = build_roundC_params(rc)
                if not multi_round:
                    corner_list.extend(rc_list)
                else:
                    gc = GeounedSurface(("MultiRoundCorner", (rc_list, plane_list, orientation)))
                    corner_list.append(gc)

    if one_value_return:
        return corner_list
    else:
        return corner_list, cornerface_index


def get_reversed_cone_cylinder(solidFaces, multiplanes, tolerances, conecylface_index=None):
    if conecylface_index is None:
        conecylface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    conecyl_list = []
    for f in solidFaces:
        if f.Index in conecylface_index:
            continue
        if isinstance(f.Surface, (GCylinder, GCone)):
            if f.Orientation == "Reversed":
                rcc = get_revConeCyl_surfaces(f, solidFaces, multiplanes, conecylface_index, tolerances)
                if rcc:
                    gc = GeounedSurface(("ReversedConeCylinder", build_RCC_params(rcc)))
                    conecyl_list.append(gc)

    if one_value_return:
        return conecyl_list
    else:
        return conecyl_list, conecylface_index


def build_roundC_params(rc_list):

    roundcorner_list = []
    plane_list = []
    extra_planes = []
    var_id = 0

    for cyl, p1, p2, config_orientation, ep1, ep2 in rc_list:
        config, fwd_corner = config_orientation
        cylOnly = GeounedSurface(("CylinderOnly", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))
        var_id += 1
        cylOnly.bVar = BoolVariable(var_id)
        if is_same_surface(p1.Surface, p2.Surface):
            gpa = None
        else:
            # ep1/ep2 (not cyl): if the round corner's own cylinder was
            # split into several contiguous pieces, p1/p2 may each only be
            # reachable from a different piece, possibly through a residual
            # sliver bridging them -- get_additional_corner_plane needs the
            # exact touching edge/face get_adjacent_cylplane already found,
            # not cyl's own (possibly non-touching) edge.
            gpa = get_additional_corner_plane(ep1, ep2)
            if gpa in plane_list:
                index = plane_list.index(gpa)
                gpa.bVar = plane_list[index].bVar
            else:
                var_id += 1
                gpa.bVar = BoolVariable(var_id)
            extra_planes.append(gpa)
        gcyl = GeounedSurface(("Cylinder", (cylOnly, gpa), cyl.Orientation))

        p1Axis = p1.Surface.Axis if p1.Orientation == "Reversed" else -p1.Surface.Axis
        p2Axis = p2.Surface.Axis if p2.Orientation == "Reversed" else -p2.Surface.Axis

        gp1 = GeounedSurface(("Plane", (p1.CenterOfMass, p1Axis, 1.0, 1.0)))
        gp2 = GeounedSurface(("Plane", (p2.CenterOfMass, p2Axis, 1.0, 1.0)))

        if gp1 in plane_list:
            index = plane_list.index(gp1)
            gp1.bVar = plane_list[index].bVar
        else:
            var_id += 1
            gp1.bVar = BoolVariable(var_id)

        if gp1 != gp2:
            if gp2 in plane_list:
                index = plane_list.index(gp2)
                gp2.bVar = plane_list[index].bVar
            else:
                var_id += 1
                gp2.bVar = BoolVariable(var_id)
            plane_list.extend((gp1, gp2))
        else:
            plane_list.append(gp1)
        params = (gcyl, (gp1, gp2), config)

        orientation = "Forward" if fwd_corner else "Reversed"
        rc = GeounedSurface(("RoundCorner", params, orientation))
        roundcorner_list.append(rc)

    multi_round = False
    orientation = None
    if len(plane_list) > 2:
        i = 0
        multi_round = True
        while i < len(plane_list) - 1:
            pi = plane_list[i]
            n = len(plane_list) - 1
            for j, pj in enumerate(reversed(plane_list[i + 1 :])):
                if pi == pj:
                    del plane_list[n - j]
            i += 1

        if len(plane_list) > 1:
            # convex_planes needs every plane actually bounding the group,
            # not just the shared corner planes (plane_list): each
            # cylinder's own closing plane (gpa/extra_planes, already used
            # per-corner as `pcid` in multi_round_corner_region) also
            # constrains the group's real shape. With only the 2-3 shared
            # corner planes, the convexity/turning-consistency check can be
            # too under-determined to ever fail (confirmed on a real
            # fixture: 3 shared planes alone always passed as "convex",
            # while the same 3 planes plus their 3 per-cylinder closing
            # planes correctly failed) -- plane_list itself (the group's
            # own top-level AND/OR terms) is left untouched, only the
            # convexity/orientation *test* sees the richer set.
            convexity_planes = list(plane_list)
            for p in extra_planes:
                if not any(p == q for q in convexity_planes):
                    convexity_planes.append(p)
            multi_round, orientation = convex_planes(convexity_planes, cyl.Surface.Axis)

        if multi_round:
            if len(plane_list) > 1:
                cylinder_list = []
                # for rc in roundcorner_list:
                #    cylinder_list.append(rc.Surf.Cylinder)
                # roundcorner_list = cylinder_list
                center = plane_list[0].Surf.Position
                for p in plane_list[1:]:
                    center = center + p.Surf.Position
                center = center / len(plane_list)
                dotvalue = p.Surf.Axis.dot(p.Surf.Position - center)
                if abs(dotvalue) < 1e-5:  # aligned planes
                    orientation = rc.Surf.Cylinder.Orientation
                else:
                    orientation = "Reversed" if dotvalue > 0 else "Forward"
            else:
                # Deduplication above can collapse plane_list down to a
                # single shared plane -- e.g. several independent,
                # non-contiguous cylinders (not merge-able by
                # merge_same_surface_faces, since they don't actually
                # touch each other) that each individually bound against
                # the same physical wall. There's no second plane to
                # derive a group-relative orientation from (convex_planes
                # itself needs >= 2 points), so fall back to the
                # last-processed corner's own already-computed cylinder
                # orientation, exactly like the "aligned planes" case
                # above already does.
                orientation = rc.Surf.Cylinder.Orientation
    params = (roundcorner_list, plane_list, multi_round, orientation)
    return params


def build_RCC_params(rc):
    # Each chain segment (cylinder or cone) becomes a Tier-2 "Cylinder"/
    # "Cone" GeounedSurface, bundling its own additional plane directly
    # (and, for a cone, its own ApexPlane) -- MetaSurfacesDict.add_reversedCC
    # reads these straight off each element to build the RevCC's boolean
    # region (n=1: AND[s,p]; n>1: AND[OR[all p_i], AND(OR[s_i,-p_i])]),
    # so no separate plane-grouping/orientation bookkeeping is needed here
    # any more.
    #
    # mp_planes (from get_join_cone_cyl's _find_adjacent_multiplane_planes,
    # a real curved-edge topological-adjacency walk, not a coincident-point
    # heuristic) lists the real MultiPlane component planes physically
    # bordering each segment's own cylinder/cone -- since the RevCC's
    # cylinders/cones are all near-coaxial, the same real multiplane plane
    # can genuinely be found adjacent to more than one segment.
    # _find_adjacent_multiplane_planes always returns the same GeounedSurface
    # object reference for the same real match, so a plain identity check is
    # enough to dedupe, no geometric comparison (is_same_plane) needed here.
    cylcones = []
    adjacent_mp_planes = []
    for cc in rc:
        if cc.Type == "Cylinder":
            cylOnly, plane, mp_planes = cc.Params
            gcylcone = GeounedSurface(("Cylinder", (cylOnly, plane)))
        else:
            cone, apexPlane, plane, mp_planes = cc.Params
            gcylcone = GeounedSurface(("Cone", (cone, apexPlane, plane)))
        cylcones.append(gcylcone)
        for mpp in mp_planes:
            if not any(mpp is existing for existing in adjacent_mp_planes):
                adjacent_mp_planes.append(mpp)

    return cylcones, adjacent_mp_planes


def _closing_plane(cyl, edges, kind, secondary):
    """The plane closing a Can/RoundCorner-style secondary surface off
    against cyl's own boundary. A real circular/elliptical tangency
    (planar_edges) already has an exact plane through its curve's own
    center (cks_edge_plane) -- kept as-is. A non-planar (generally
    BSpline) tangency has no such center; the plane there is computed
    analytically from the two real surfaces themselves, not guessed
    from the tangency curve's shape -- see
    vector_geometry.find_can_plane. Returns None if that analytic
    computation finds the main cylinder isn't actually split into two
    disjoint pieces by the secondary surface -- the caller must treat
    that as "this isn't a valid Can", not fall back to a guess."""
    if planar_edges(edges):
        return cks_edge_plane(cyl, edges)
    result = vector_geometry.find_can_plane(cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, kind, secondary)
    if result is None:
        return None
    position, normal = result

    # find_can_plane's normal comes from a cross product, which has no
    # preferred sign of its own -- cks_edge_plane's own convention
    # ("positive plane direction toward material", enforced there via
    # the identical material_direction-based check) has to be applied
    # here too, since find_can_plane never sees a real edge/face to
    # derive it from.
    edge = edges[0]
    p0, p1 = edge.ParameterRange
    pos = edge.value_at(0.5 * (p0 + p1))
    vect, _ = material_direction(pos, cyl, edge)
    if normal.dot(vect) < 0:
        normal = -normal

    return GeounedSurface(("Plane", (position, normal, 1.0, 1.0)))


def build_can_params(cs):
    cyl_in, sr1, sr2 = cs
    shell = type(cyl_in) is ShellGu
    if not shell:
        cyl = cyl_in
    else:
        cyl = cyl_in.Faces[0]

    bsurf = []
    sid = 0
    for s, r, omit in (sr1, sr2):
        if type(s.Surface) is GPlane:
            normal = -s.Surface.Axis if s.Orientation == "Forward" else s.Surface.Axis
            if r == "OR":
                normal = -normal  # plane axis toward cylinder center
            if not omit:
                normal = -normal  # virtual can surface
            gs = GeounedSurface(("Plane", (s.Surface.Position, normal, 1.0, 1.0)))
            sid += 1
            gs.bVar = BoolVariable(sid)

        elif type(s.Surface) is GCylinder:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            if r is None:
                # adjacent cylinder has same radius and is parallel to cylinder --
                # this is a continuation of the same analytic surface, not a
                # split-by-a-different-surface Can, so the tangency-curve-only
                # plane (cks_edge_plane) is still the right tool here.
                pa = cks_edge_plane(cyl, edges)
                if pa is not None:
                    sid += 1
                    pa.bVar = BoolVariable(sid)
                r = "AND" if s.Orientation == "Forward" else "OR"
                gs = GeounedSurface(("Plane", (pa.Surf.Position, pa.Surf.Axis, 1.0, 1.0)))
                gs.bVar = pa.bVar
            else:
                pa = _closing_plane(cyl, edges, "cylinder", s.Surface)
                if pa is None:
                    return None
                sid += 1
                pa.bVar = BoolVariable(sid)

                cylOnly = GeounedSurface(("CylinderOnly", (s.Surface.Center, s.Surface.Axis, s.Surface.Radius, 1.0, 1.0)))
                sid += 1
                cylOnly.bVar = BoolVariable(sid)

                if omit:
                    orientation = s.Orientation
                else:
                    orientation = "Reversed" if s.Orientation == "Forward" else "Forward"

                gs = GeounedSurface(("Cylinder", (cylOnly, pa), orientation))

        elif type(s.Surface) is GCone:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            coneOnly = GeounedSurface(("ConeOnly", (s.Surface.Apex, s.Surface.Axis, s.Surface.SemiAngle, 1.0, 1.0)))
            sid += 1
            coneOnly.bVar = BoolVariable(sid)

            # apex distance from cylinder axis
            cp = s.Surface.Apex - cyl.Surface.Center
            a = cyl.Surface.Axis
            alpha = cp.dot(a)
            sqr = cp.dot(cp) - alpha * alpha
            if abs(sqr) < 1e-8:
                adist = 0
            else:
                adist = math.sqrt(sqr)

            if adist < cyl.Surface.Radius:
                apexPlane = cone_apex_plane(s, Tolerances())
                if apexPlane is not None:
                    sid += 1
                    apexPlane.bVar = BoolVariable(sid)
                pa = None
            else:
                apexPlane = None
                pa = _closing_plane(cyl, edges, "cone", s.Surface)
                if pa is None:
                    return None
                sid += 1
                pa.bVar = BoolVariable(sid)

            if omit:
                orientation = s.Orientation
            else:
                orientation = "Reversed" if s.Orientation == "Forward" else "Forward"

            gs = GeounedSurface(("Cone", (coneOnly, apexPlane, pa), orientation))

        elif type(s.Surface) is GSphere:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)
            sphOnly = GeounedSurface(("SphereOnly", (s.Surface.Center, s.Surface.Radius)))
            sid += 1
            sphOnly.bVar = BoolVariable(sid)

            pa = _closing_plane(cyl, edges, "sphere", s.Surface)
            if pa is None:
                return None
            sid += 1
            pa.bVar = BoolVariable(sid)

            if omit:
                orientation = s.Orientation
            else:
                orientation = "Reversed" if s.Orientation == "Forward" else "Forward"
            gs = GeounedSurface(("Sphere", (sphOnly, pa), orientation))

        bsurf.append((gs, r))

    cylOnly = GeounedSurface(("CylinderOnly", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))
    sid += 1
    cylOnly.bVar = BoolVariable(sid)
    gcyl = GeounedSurface(("Cylinder", (cylOnly, None), cyl.Orientation))

    return (gcyl, bsurf[0], bsurf[1], cyl.Orientation)


def build_tcone_params(ks):
    kne_in, p1, p2 = ks
    shell = type(kne_in) is ShellGu
    if not shell:
        kne = kne_in
    else:
        kne = kne_in.Faces[0]

    bsurf = []
    sid = 0
    for s, r, omit in (p1, p2):
        normal = -s.Surface.Axis if s.Orientation == "Forward" else s.Surface.Axis
        if r == "OR":
            normal = -normal  # plane axis toward cone center
        if not omit:
            normal = -normal  # virtual can surface
        gs = GeounedSurface(("Plane", (s.Surface.Position, normal, 1.0, 1.0)))
        sid += 1
        gs.bVar = BoolVariable(sid)
        bsurf.append((gs, r))

    coneOnly = GeounedSurface(("ConeOnly", (kne.Surface.Apex, kne.Surface.Axis, kne.Surface.SemiAngle, 1.0, 1.0)))
    sid += 1
    coneOnly.bVar = BoolVariable(sid)
    gcone = GeounedSurface(("Cone", (coneOnly, None, None), kne.Orientation))
    return (gcone, bsurf[0], bsurf[1])


def build_multip_params(plane_list):

    planeparams = []
    edges = []
    vertexes = []

    for p in plane_list:
        # plane = PlaneGu(p)
        # planeparams.append((plane.Position, plane.Axis, plane.dim1, plane.dim2))
        normal = -p.Surface.Axis if p.Orientation == "Forward" else p.Surface.Axis
        gp = GeounedSurface(("Plane", (p.Surface.Position, normal, 1.0, 1.0)))
        same = False
        for pp in planeparams:
            if is_same_plane(pp.Surf, gp.Surf, Options(), Tolerances(), NumericFormat()):
                same = True
                break
        if not same:
            planeparams.append(gp)

    ajdacent_planes = [[] for i in range(len(plane_list))]
    for i, p1 in enumerate(plane_list):
        for j, p2 in enumerate(plane_list[i + 1 :]):
            Edges = commonEdge(p1, p2)
            if Edges:
                e = Edges[0]
                edges.append(e)
                ajdacent_planes[i].append((j, e))
                ajdacent_planes[j].append((i, e))

    vertex_list = []
    for i, e1 in enumerate(edges):

        for e2 in edges[i + 1 :]:
            vertex_list.extend(commonVertex(e1, e2))

    vertexes = []
    while len(vertex_list) > 0:
        v = vertex_list.pop()
        n = 0
        for vi in reversed(vertex_list):
            if v == vi:
                n += 1
                vertex_list.remove(vi)
        if n > 0:
            vertexes.append((v, n + 1))

    return (planeparams, edges, vertexes)


def convex_planes(plane_list, zaxis):
    center = plane_list[0].Surf.Position
    for p in plane_list[1:]:
        center = center + p.Surf.Position
    center = center / len(plane_list)

    ref = (plane_list[0].Surf.Position - center).normalized()
    orientation = "Forward" if plane_list[0].Surf.Axis.dot(ref) > 0 else "Reversed"

    if len(plane_list) < 3:
        return True, orientation

    angles = []
    for i, p in enumerate(plane_list[1:]):
        rp = (p.Surf.Position - center).normalized()
        cosa = ref.dot(rp)
        cross = ref.cross(rp)
        sina = cross.length
        if cross.dot(zaxis) < 0:
            sina = -sina
        angle = math.atan2(sina, cosa)
        while angle < 0:
            angle += 2 * math.pi
        angles.append((angle, i))

    angles.sort()

    p1 = ref
    p0 = plane_list[angles[-1][1] + 1].Surf.Axis
    signref = zaxis.dot(p0.cross(p1))

    p0 = p1
    convex = True
    for a, i in angles:
        p1 = plane_list[i + 1].Surf.Axis
        sign = zaxis.dot(p0.cross(p1))
        if signref * sign < 0:
            convex = False
            break
        p0 = p1

    return convex, orientation


def get_additional_corner_plane(ep1, ep2):
    # ep1/ep2 come from get_adjacent_cylplane's cornerPlanes=True search:
    # (anchor, cyl_edge, touching_edge, near_face, plane). near_face/plane
    # are where `plane` actually touches -- anchor itself (near_face is
    # anchor) when found directly, or a residual sliver bridging them (see
    # get_roundcorner_surfaces' merge_same_surface_faces + skip_slivers)
    # otherwise, in which case `plane` is the real, non-degenerate face the
    # sliver walk found beyond it. material_direction needs real, trustworthy
    # geometry -- evaluate on `plane` (already found, previously discarded
    # here) instead of on the sliver's own near-zero-area geometry, which
    # can have an arbitrary/wrong normal from its own degenerate
    # triangulation.
    #
    # NOTE 2026-08-19: two rewrites were tried here and both reverted, kept
    # as a record so they aren't retried blindly:
    #  1) an axis-based construction (supporting-plane normal taken from the
    #     anchor's own contour, via 2 parallel straight edges or the
    #     contour's inertia-tensor principal axis) -- caused a confirmed
    #     infinite-recursion regression on cylBox.stp.
    #  2) a "coplanar with e1/e2" construction (e1/e2 being the touching
    #     edges where p1/p2 meet the cylinder -- real generatrix lines, so
    #     parallel to each other; normal = cross(shared_direction,
    #     connecting_vector), closed-form, no sampling) -- this one *did*
    #     fix cylBox.stp and origSolid_0.stp, but caused a different,
    #     confirmed infinite-recursion regression elsewhere in
    #     Solidos/Big_one_cell/modelcell_cut1.stp's own decomposition tree
    #     (a "CylinderOnly" candidate stuck at a constant volume across
    #     960+ recursion levels) -- i.e. it changes which candidate
    #     surfaces succeed/fail deep in the decomposition tree for OTHER,
    #     unrelated corners in the same solid, not just the corner it's
    #     computed for.
    # Both attempts were motivated by a real, confirmed-wrong plane on
    # modelcell_cut1.stp piece 66 (v1/v2 averaging pulled in an unrelated
    # third face's own material direction, per direct user diagnosis) --
    # that problem is still open. Direct numeric testing (sampling the wire
    # and checking axis.dot(point - sample) for both candidates) confirmed
    # BOTH the axis-based and the v1+v2 direction are valid supporting
    # planes of the wire (no sample crosses to the negative side for
    # either) -- i.e. "rests on the contour, nothing crosses it" does not
    # uniquely determine the correct plane. Fixing piece 66 needs either a
    # sharper criterion that provably can't perturb an unrelated corner's
    # own candidate search elsewhere in the same solid, or a targeted fix
    # to piece 66's own ep1/ep2 selection (which edge/face
    # get_adjacent_cylplane picks) rather than a blanket formula change --
    # not resumed yet.
    anchor1, cyl_edge1, e1, near1, plane1 = ep1
    anchor2, cyl_edge2, e2, near2, plane2 = ep2

    pos1 = e1.Vertexes[0]
    pos2 = e2.Vertexes[0]
    face1 = anchor1 if near1.Index == anchor1.Index else plane1
    face2 = anchor2 if near2.Index == anchor2.Index else plane2
    v1, n1 = material_direction(pos1, face1, e1)
    v2, n2 = material_direction(pos2, face2, e2)
    point = 0.5 * (pos1 + pos2)
    combined = v1 + v2
    if combined.length < 1e-2:
        # v1/v2 (near-)exactly opposed: a real, valid configuration for a
        # Reversed MultiRoundCorner (every wing's material-pointing normal
        # faces "outward", and two wings meeting at a cusp can legitimately
        # point in exactly opposite outward directions there) -- the
        # corner's bounding planes are OR-combined, so either direction
        # alone is a correct choice; there's no well-defined bisector to
        # average toward instead.
        #
        # The threshold was originally 1e-6 (only the *exactly* zero-length
        # case), but a genuinely near-cusp corner rarely lands on exact
        # floating-point cancellation -- confirmed live, 2026-08-23,
        # Solidos/test_models/Mixed/SCDR_90_hollow.stp's own piece4 (a real
        # R=37mm round corner): v1=(0.323,0,-0.947), v2=(-0.324,0,0.946)
        # are antiparallel to within ~0.086 degrees, giving
        # combined.length=0.00154 -- comfortably above the old 1e-6 guard,
        # so it fell through to `combined.normalized()`, whose *direction*
        # is dominated by that tiny near-cancellation residual (essentially
        # numerical noise, not a meaningful bisector) rather than any real
        # geometric signal. Raised to 1e-2 (~0.57 degrees from exactly
        # opposed) for comfortable margin over the observed case while
        # staying well below any genuine, well-conditioned corner angle.
        #
        # v1 vs v2, direct user correction: for piece4's real chain (this
        # R37 corner sits adjacent to a real R40 corner in the same
        # MultiRoundCorner), the additional plane's normal must be
        # consistent with the neighboring corner's own additional plane,
        # not an arbitrary pick -- confirmed v2 is the one that matches
        # (v2=(-0.324,0,0.946) vs the R40 corner's own additional-plane
        # axis=(-0.322,0,0.947), while v1 is its near-exact negation).
        paxis = v2
    else:
        paxis = combined.normalized()
    return GeounedSurface(("Plane", (point, paxis, 1.0, 1.0, False)))
