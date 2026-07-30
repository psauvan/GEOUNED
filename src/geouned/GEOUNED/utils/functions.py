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
from .meta_surfaces_utils import commonEdge, commonVertex, no_convex, planar_edges, eligible_plane
from ..decompose.decom_utils_generator import cks_edge_plane
from ..conversion.cell_definition_functions import cone_apex_plane
from .basic_functions_part2 import is_same_plane
from ...geo import GPlane, GCylinder, GCone, GSphere, Gmake_box, to_fc_vector, to_gvector
from .basic_functions_part1 import shapes_in_contact


def get_box(comp, enlargeBox):
    # comp is always a GeounedSolid here, whose BoundBox is a GBoundBox
    box = comp.BoundBox.enlarged(enlargeBox)
    return Gmake_box(box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax).__native__


def get_multiplanes(solidFaces, omit_faces_set=None):
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
        if not eligible_plane(p):
            continue
        mp_plane_index = set()
        mplanes = multiplane(p, planes, mp_plane_index)
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
                gc = GeounedSurface(("Can", build_can_params(cs), f.Orientation))
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


def get_roundCorner(solidFaces, cornerface_index=None):
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
            rc, surfindex = get_roundcorner_surfaces(f, solidFaces, {f.Index})
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


def get_reversed_cone_cylinder(solidFaces, multiplanes, conecylface_index=None):
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
                rcc = get_revConeCyl_surfaces(f, solidFaces, multiplanes, conecylface_index)
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
    var_id = 0

    for cyl, p1, p2, config_orientation in rc_list:
        config, fwd_corner = config_orientation
        cylOnly = GeounedSurface(("CylinderOnly", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))
        var_id += 1
        cylOnly.bVar = BoolVariable(var_id)
        if is_same_surface(p1.Surface, p2.Surface):
            gpa = None
        else:
            gpa = get_additional_corner_plane(cyl, p1, p2)
            if gpa in plane_list:
                index = plane_list.index(gpa)
                gpa.bVar = plane_list[index].bVar
            else:
                var_id += 1
                gpa.bVar = BoolVariable(var_id)
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

        multi_round, orientation = convex_planes(plane_list, cyl.Surface.Axis)

        if multi_round:
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
    params = (roundcorner_list, plane_list, multi_round, orientation)
    return params


def build_RCC_params(rc):
    cylcones = []
    plane_dict = dict()
    add_planes = []
    init = None
    for cc in rc:
        if cc.Type == "Cylinder":
            gcylcone, plane, addP = cc.Params
        else:
            cone, apexPlane, plane, addP = cc.Params
            gcylcone = GeounedSurface(("Cone", (cone, apexPlane, None), "Reversed"))

        add_planes.extend(addP)
        if len(cc.Connections) == 1:
            init = cc.Index
        plane_dict[cc.Index] = (cc.Connections, plane)
        cylcones.append(gcylcone)

    if len(rc) == 1:
        loop = False
        init = tuple(plane_dict.keys())[0]
        planeSeq = [plane_dict[init][1]]
    else:
        loop = True
        if init is None:
            init = tuple(plane_dict.keys())[0]
        nextip, operator = plane_dict[init][0][0]
        ip = init
        if operator == "OR":
            gp = plane_dict[init][1]
            ORPlanes = [gp]
            planeSeq = []
        else:
            ORPlanes = []
            gp = plane_dict[init][1]
            planeSeq = [gp]

    while loop:
        connect = plane_dict[nextip][0]
        if len(connect) == 1:
            ip = nextip
            nextip, nextop = connect[0]
            loop = False
        else:
            next1, op1 = connect[0]
            next2, op2 = connect[1]
            if ip != next1:
                ip = nextip
                nextip = next1
                nextop = op1
            else:
                ip = nextip
                nextip = next2
                nextop = op2
            if nextip == init:
                loop = False

        gp = plane_dict[ip][1]
        if operator == "OR":
            ORPlanes.append(gp)
            if not loop:
                planeSeq.append(ORPlanes)
        else:
            if ORPlanes:
                planeSeq.append(ORPlanes)
                ORPlanes = []
            planeSeq.append(gp)
        operator = nextop
    params = (cylcones, planeSeq, add_planes)
    return params


def build_can_params(cs):
    cyl_in, sr1, sr2 = cs
    shell = type(cyl_in) is ShellGu
    if not shell:
        cyl = cyl_in
    else:
        cyl = cyl_in.Faces[0]

    bsurf = []
    sid = 0
    for s, r in (sr1, sr2):

        if type(s.Surface) is GPlane:
            normal = -s.Surface.Axis if s.Orientation == "Forward" else s.Surface.Axis
            if r == "OR":
                normal = -normal  # plane axis toward cylinder center
            gs = GeounedSurface(("Plane", (s.Surface.Position, normal, 1.0, 1.0)))
            sid += 1
            gs.bVar = BoolVariable(sid)

        elif type(s.Surface) is GCylinder:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            pa = cks_edge_plane(cyl, edges)
            if pa is not None:
                sid += 1
                pa.bVar = BoolVariable(sid)

            if r is None:
                r = "AND" if s.Orientation == "Forward" else "OR"
                gs = GeounedSurface(("Plane", (pa.Surf.Position, pa.Surf.Axis, 1.0, 1.0)))
                gs.bVar = pa.bVar
            else:
                cylOnly = GeounedSurface(("CylinderOnly", (s.Surface.Center, s.Surface.Axis, s.Surface.Radius, 1.0, 1.0)))
                sid += 1
                cylOnly.bVar = BoolVariable(sid)
                if not planar_edges(edges):
                    # move sligtly the plane position toward boundary surface center
                    cr = cylOnly.Surf.Center - pa.Surf.Position
                    d = cr - cr.dot(cylOnly.Surf.Axis) * cylOnly.Surf.Axis
                    pa.Surf.Position = pa.Surf.Position + 0.01 * d
                gs = GeounedSurface(("Cylinder", (cylOnly, pa), s.Orientation))

        elif type(s.Surface) is GCone:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            coneOnly = GeounedSurface(("ConeOnly", (s.Surface.Apex, s.Surface.Axis, s.Surface.SemiAngle, 1.0, 1.0)))
            sid += 1
            coneOnly.bVar = BoolVariable(sid)

            # apex distance from cylinder axis
            cp = s.Surface.Apex - to_fc_vector(cyl.Surface.Center)
            a = to_fc_vector(cyl.Surface.Axis)
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
                pa = cks_edge_plane(cyl, edges)
                apexPlane = None
                if pa is not None:
                    sid += 1
                    pa.bVar = BoolVariable(sid)
                if not planar_edges(edges):
                    # move sligtly the plane position toward boundary surface center
                    cr = coneOnly.Surf.Apex - pa.Surf.Position
                    d = cr - cr.dot(coneOnly.Surf.Axis) * coneOnly.Surf.Axis
                    pa.Surf.Position = pa.Surf.Position + 0.01 * d

            gs = GeounedSurface(("Cone", (coneOnly, apexPlane, pa), s.Orientation))

        elif type(s.Surface) is GSphere:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)
            sphOnly = GeounedSurface(("SphereOnly", (s.Surface.Center, s.Surface.Radius)))
            sid += 1
            sphOnly.bVar = BoolVariable(sid)

            pa = cks_edge_plane(cyl, edges)
            if pa is not None:
                sid += 1
                pa.bVar = BoolVariable(sid)

            if not planar_edges(edges):
                # move sligtly the plane position toward boundary surface center
                d = sphOnly.Surf.Center - pa.Surf.Position
                pa.Surf.Position = pa.Surf.Position + 0.01 * d

            gs = GeounedSurface(("Sphere", (sphOnly, pa), s.Orientation))

        bsurf.append((gs, r))

    cylOnly = GeounedSurface(("CylinderOnly", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))
    sid += 1
    cylOnly.bVar = BoolVariable(sid)
    gcyl = GeounedSurface(("Cylinder", (cylOnly, None), cyl.Orientation))

    return (gcyl, bsurf[0], bsurf[1])


def build_tcone_params(ks):
    kne_in, p1, p2 = ks
    shell = type(kne_in) is ShellGu
    if not shell:
        kne = kne_in
    else:
        kne = kne_in.Faces[0]

    bsurf = []
    sid = 0
    for s, r in (p1, p2):
        normal = -s.Surface.Axis if s.Orientation == "Forward" else s.Surface.Axis
        if r == "OR":
            normal = -normal  # plane axis toward cone center
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
    zaxis = to_gvector(zaxis)

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
        if cross.dot(ref) < 0:
            sina = -sina
        angles.append((math.atan2(sina, cosa), i))

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


def material_direction(pos, face_in, edge):

    pe = edge.Curve.parameter(pos)
    dir = edge.derivative1At(pe)
    dir.normalize()
    if edge.Orientation == "Reversed":
        dir = -dir
    u, v = face_in.Surface.parameter(pos)
    normalf = face_in.normalAt(u, v)
    normalf.normalize()
    matvec = normalf.cross(dir)

    return matvec, normalf


def get_additional_corner_plane(cyl, p1, p2):
    Edges1 = commonEdge(cyl, p1)
    Edges2 = commonEdge(cyl, p2)
    e1 = Edges1[0]
    e2 = Edges2[0]
    p1 = e1.Vertexes[0].Point
    p2 = e2.Vertexes[0].Point
    v1, n1 = material_direction(e1.Vertexes[0].Point, cyl.__face__, e1)
    v2, n2 = material_direction(e2.Vertexes[0].Point, cyl.__face__, e2)
    point = 0.5 * (p1 + p2)
    paxis = v1 + v2
    paxis.normalize()
    return GeounedSurface(("Plane", (point, paxis, 1.0, 1.0, False)))


def get_additional_corner_plane_old(cyl, p1, p2):
    Edges1 = commonEdge(cyl, p1)
    Edges2 = commonEdge(cyl, p2)
    e1 = Edges1[0]
    e2 = Edges2[0]
    point11 = e1.Vertexes[0].Point
    point12 = e1.Vertexes[1].Point
    p1, p2 = e2.ParameterRange
    point21 = e2.valueAt(p1)
    point22 = e2.valueAt(p2)
    v21 = point21 - point11
    v22 = point22 - point11
    cyl_axis = to_fc_vector(cyl.Surface.Axis)
    dt1 = abs(cyl_axis.dot(v21))
    dt2 = abs(cyl_axis.dot(v22))
    vect = v21 if dt1 < dt2 else v22
    paxis = vect.cross(cyl_axis)
    paxis.normalize()
    umin, umax, vmin, vmax = cyl.ParameterRange
    surfpoint = cyl.valueAt(0.5 * (umin + umax), 0.5 * (vmin + vmax))
    dir = surfpoint - to_fc_vector(cyl.Surface.Center)
    dir.normalize()

    if dir.dot(paxis) < 0:
        paxis = -paxis  # normal plane toward existing cylinder surface
    eps = 1e-7 * cyl.Surface.Radius  # used to avoid lost particles with possible complementary region
    point = 0.25 * (point11 + point12 + point21 + point22) + eps * paxis

    return GeounedSurface(("Plane", (point, paxis, 1.0, 1.0, False)))
