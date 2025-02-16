#
# Set of useful functions used in different parts of the code
#
import logging

import FreeCAD
import Part
import math

logger = logging.getLogger("general_logger")

from .geometry_gu import ShellGu, PlaneGu, CylinderGu, ConeGu, SphereGu
from .geouned_classes import GeounedSurface
from .data_classes import NumericFormat, Options, Tolerances
from .meta_surfaces import multiplane_loop, get_can_surfaces, get_roundcorner_surfaces, get_revConeCyl_surfaces
from .meta_surfaces_utils import commonEdge, commonVertex, no_convex, planar_edges
from ..decompose.decom_utils_generator import cyl_edge_plane
from ..conversion.cell_definition_functions import cone_apex_plane
from .basic_functions_part2 import is_same_plane


def get_box(comp, enlargeBox):
    bb = FreeCAD.BoundBox(comp.BoundBox)
    bb.enlarge(enlargeBox)
    xMin, yMin, zMin = bb.XMin, bb.YMin, bb.ZMin
    xLength, yLength, zLength = bb.XLength, bb.YLength, bb.ZLength

    return Part.makeBox(
        xLength,
        yLength,
        zLength,
        FreeCAD.Vector(xMin, yMin, zMin),
        FreeCAD.Vector(0, 0, 1),
    )


def get_multiplanes(solidFaces, plane_index_set=None):
    """identify and return all multiplanes in the solid."""

    if plane_index_set is None:
        plane_index_set = set()
        one_value_return = False
    else:
        one_value_return = True

    planes = []
    for f in solidFaces:
        if isinstance(f.Surface, PlaneGu):
            planes.append(f)

    multiplane_list = []
    multiplane_objects = []

    for p in planes:
        loop = False
        for mp in multiplane_list:
            if p in mp:
                loop = True
                break
        if loop:
            continue
        mplanes = [p]
        multiplane_loop([p], mplanes, planes)
        if len(mplanes) != 1:
            if no_convex(mplanes):
                mp_params = build_multip_params(mplanes)
                mp = GeounedSurface(("MultiPlane", mp_params))
                if mp.Surf.PlaneNumber < 2:
                    continue
                for pp in mplanes:
                    plane_index_set.add(pp.Index)
                multiplane_list.append(mplanes)
                multiplane_objects.append(mp)

    if one_value_return:
        return multiplane_objects
    else:
        return multiplane_objects, plane_index_set


def get_reverseCan(solidFaces, canface_index=None):
    """identify and return all can type in the solid."""

    if canface_index is None:
        canface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    can_list = []
    for f in solidFaces:
        if isinstance(f.Surface, CylinderGu):
            if f.Index in canface_index:
                continue
            if f.Orientation == "Reversed":
                cs, surfindex = get_can_surfaces(f, solidFaces)
                if cs is not None:
                    gc = GeounedSurface(("Can", build_can_params(cs)))
                    can_list.append(gc)
                    canface_index.update(surfindex)

    if one_value_return:
        return can_list
    else:
        return can_list, canface_index


def get_roundCorner(solidFaces, cornerface_index=None):
    """identify and return all roundcorner type in the solid."""
    if cornerface_index is None:
        cornerface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    corner_list = []
    for f in solidFaces:
        if isinstance(f.Surface, CylinderGu):
            if f.Index in cornerface_index:
                continue
            if f.Orientation == "Forward":
                rc, surfindex = get_roundcorner_surfaces(f, solidFaces)
                if rc is not None:
                    gc = GeounedSurface(("RoundCorner", build_roundC_params(rc)))
                    cornerface_index.update(surfindex)
                    corner_list.append(gc)

    if one_value_return:
        return corner_list
    else:
        return corner_list, cornerface_index


def get_reversed_cone_cylinder(solidFaces, multiplanes, conecylface_index=None):
    """identify and return all roundcorner type in the solid."""
    if conecylface_index is None:
        conecylface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    conecyl_list = []
    for f in solidFaces:
        if f.Index in conecylface_index:
            continue
        if isinstance(f.Surface, (CylinderGu, ConeGu)):
            if f.Orientation == "Reversed":
                rcc = get_revConeCyl_surfaces(f, solidFaces, multiplanes, conecylface_index)
                if rcc:
                    gc = GeounedSurface(("ReversedConeCylinder", build_RCC_params(rcc)))
                    conecyl_list.append(gc)

    if one_value_return:
        return conecyl_list
    else:
        return conecyl_list, conecylface_index


def build_roundC_params(rc):
    cyl, p1, p2 = rc[0]
    configuration = rc[1]

    gcyl = GeounedSurface(("CylinderOnly", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))
    pos_orientation = "Reversed" if configuration == "AND" else "Forward"
    p1Axis = p1.Surface.Axis if p1.Orientation == pos_orientation else -p1.Surface.Axis
    p2Axis = p2.Surface.Axis if p2.Orientation == pos_orientation else -p2.Surface.Axis

    gp1 = GeounedSurface(("Plane", (p1.Surface.Position, p1Axis, 1.0, 1.0)))
    gp2 = GeounedSurface(("Plane", (p2.Surface.Position, p2Axis, 1.0, 1.0)))

    gpa = get_additional_corner_plane(cyl, p1, p2)

    params = ((gcyl, gpa), (gp1, gp2), configuration)
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
            gcylcone = GeounedSurface(("Cone", (cone, apexPlane, None, []), "Reversed"))

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
    for s, r in (sr1, sr2):

        if type(s.Surface) is PlaneGu:
            normal = -s.Surface.Axis if s.Orientation == "Forward" else s.Surface.Axis
            if r == "OR":
                normal = -normal  # plane axis toward cylinder center
            gs = GeounedSurface(("Plane", (s.Surface.Position, normal, 1.0, 1.0)))
            pa = None

        elif type(s.Surface) is CylinderGu:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            if planar_edges(edges) and False:
                gs = cyl_edge_plane(cyl, edges)
            else:
                cylOnly = GeounedSurface(("CylinderOnly", (s.Surface.Center, s.Surface.Axis, s.Surface.Radius, 1.0, 1.0)))
                pa = cyl_edge_plane(cyl, edges)
                gs = GeounedSurface(("Cylinder", (cylOnly, pa, None), s.Orientation))

        elif type(s.Surface) is ConeGu:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            if planar_edges(edges) and False:
                gs = cyl_edge_plane(cyl, edges)
            else:
                coneOnly = GeounedSurface(("ConeOnly", (s.Surface.Apex, s.Surface.Axis, s.Surface.SemiAngle, 1.0, 1.0)))
                pa = cyl_edge_plane(cyl, edges)
                apexPlane = cone_apex_plane(s, Tolerances())
                gs = GeounedSurface(("Cone", (coneOnly, apexPlane, pa, None), s.Orientation))

        elif type(s.Surface) is SphereGu:
            if shell:
                edges, cyl = commonEdge(cyl_in, s, outer1_only=True, outer2_only=False)
            else:
                edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)

            edges = commonEdge(cyl, s, outer1_only=True, outer2_only=False)
            sphOnly = GeounedSurface(("SphereOnly", (s.Surface.Center, s.Surface.Radius)))
            pa = cyl_edge_plane(cyl, edges)
            gs = GeounedSurface(("Sphere", (sphOnly, pa), s.Orientation))

        bsurf.append((gs, r))

    cylOnly = GeounedSurface(("CylinderOnly", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))
    gcyl = GeounedSurface(("Cylinder", (cylOnly, None, None), cyl.Orientation))

    return (gcyl, bsurf[0], bsurf[1])


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
            if v.Point == vi.Point:
                n += 1
                vertex_list.remove(vi)
        if n > 0:
            vertexes.append((v, n + 1))

    return (planeparams, edges, vertexes)


def get_additional_corner_plane(cyl, p1, p2):
    Edges1 = commonEdge(cyl, p1)
    Edges2 = commonEdge(cyl, p2)
    e1 = Edges1[0]
    e2 = Edges2[0]
    point1 = e1.Vertexes[0].Point
    point21 = e2.Vertexes[0].Point
    point22 = e2.Vertexes[1].Point
    v21 = point21 - point1
    v22 = point22 - point1
    dt1 = abs(cyl.Surface.Axis.dot(v21))
    dt2 = abs(cyl.Surface.Axis.dot(v22))
    vect = v21 if dt1 < dt2 else v22
    paxis = vect.cross(cyl.Surface.Axis)
    paxis.normalize()
    umin, umax, vmin, vmax = cyl.ParameterRange
    surfpoint = cyl.valueAt(0.5 * (umin + umax), 0.5 * (vmin + vmax))
    dir = surfpoint - cyl.Surface.Center
    dir.normalize()

    if dir.dot(paxis) > 0:
        paxis = -paxis  # normal plane toward negative cylinder surface
    eps = 1e-7 * cyl.Surface.Radius  # used to avoid lost particles with possible complementary region
    point = point1 + eps * paxis

    return GeounedSurface(("Plane", (point, paxis, 1.0, 1.0)))
