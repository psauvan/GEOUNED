#
# Set of useful functions used in different parts of the code
#
import logging

import FreeCAD
import Part

logger = logging.getLogger("general_logger")

from .geometry_gu import PlaneGu, CylinderGu
from .geouned_classes import GeounedSurface
from .data_classes import NumericFormat, Options, Tolerances
from .meta_surfaces import commonVertex, commonEdge, multiplane_loop, no_convex, get_revcan_surfaces, get_roundcorner_surfaces

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


def get_multiplanes(solid, plane_index_set=None):
    """identify and return all multiplanes in the solid."""

    if plane_index_set is None:
        plane_index_set = set()
        one_value_return = False
    else:
        one_value_return = True

    planes = []
    for f in solid.Faces:
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
                for pp in mplanes:
                    plane_index_set.add(pp.Index)
                multiplane_list.append(mplanes)
                multiplane_objects.append(mp)

    if one_value_return:
        return multiplane_objects
    else:
        return multiplane_objects, plane_index_set


def get_reverseCan(solid, canface_index=None):
    """identify and return all can type in the solid."""

    if canface_index is None:
        canface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    can_list = []
    for f in solid.Faces:
        if isinstance(f.Surface, CylinderGu):
            if f.Index in canface_index:
                continue
            if f.Orientation == "Reversed":
                cs, surfindex = get_revcan_surfaces(f, solid)
                if cs is not None:
                    gc = GeounedSurface(("ReverseCan", build_revcan_params(cs)))
                    can_list.append(gc)
                    canface_index.update(surfindex)

    if one_value_return:
        return can_list
    else:
        return can_list, canface_index


def get_roundCorner(solid, cornerface_index=None):
    """identify and return all roundcorner type in the solid."""
    if cornerface_index is None:
        cornerface_index = set()
        one_value_return = False
    else:
        one_value_return = True

    corner_list = []
    for f in solid.Faces:
        if isinstance(f.Surface, CylinderGu):
            if f.Index in cornerface_index:
                continue
            if f.Orientation == "Forward":
                rc, surfindex = get_roundcorner_surfaces(f, solid)
                if rc is not None:
                    gc = GeounedSurface(("RoundCorner", build_roundC_params(rc)))
                    cornerface_index.update(surfindex)
                    corner_list.append(gc)

    if one_value_return:
        return corner_list
    else:
        return corner_list, cornerface_index


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


def build_revcan_params(cs):
    cyl,p1 = reversed(cs[-2:])
    if isinstance(cyl, GeounedSurface):
        gcyl = cyl
    else:
        gcyl = GeounedSurface(("CylinderOnly", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))

    if isinstance(p1, GeounedSurface):
        gp1 = p1
    else:
        gp1 = GeounedSurface(("Plane", (p1.Surface.Position, p1.Surface.Axis, 1.0, 1.0)))

    params = [gcyl, gp1]
    if len(cs) == 3:
        p2 = cs[-3]
        if isinstance(p2, GeounedSurface):
            gp2 = p2
        else:
            gp2 = GeounedSurface(("Plane", (p2.Surface.Position, p2.Surface.Axis, 1.0, 1.0)))
        params.append(gp2)
    return params


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
            e = commonEdge(p1, p2)
            if e is not None:
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
    e1 = commonEdge(cyl, p1)
    e2 = commonEdge(cyl, p2)
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
        paxis = -paxis
    eps = 1e-7 * cyl.Surface.Radius  # used to avoid lost particles with possible complementary region
    point = point1 + eps * paxis

    return GeounedSurface(("Plane", (point, paxis, 1.0, 1.0)))
