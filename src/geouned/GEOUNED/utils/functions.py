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
from .meta_surfaces import commonVertex, commonEdge, multiplane_loop, no_convex, get_revcan_surfaces

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


def get_multiplanes(solid):
    """identify and return all multiplanes in the solid."""
    planes = []
    for f in solid.Faces:
        if isinstance(f.Surface, PlaneGu):
            planes.append(f)

    multiplane_list = []
    multiplane_objects = []
    plane_index_set = set()
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
    return multiplane_objects, plane_index_set


def get_reverseCan(solid):
    """identify and return all can type in the solid."""

    can_list = []
    canface_index = set()
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

    return can_list, canface_index


def build_revcan_params(cs):
    cyl, p1 = cs[0:2]
    if isinstance(cyl, GeounedSurface):
        gcyl = cyl
    else:
        gcyl = GeounedSurface(("Cylinder", (cyl.Surface.Center, cyl.Surface.Axis, cyl.Surface.Radius, 1.0, 1.0)))

    if isinstance(p1, GeounedSurface):
        gp1 = p1
    else:
        gp1 = GeounedSurface(("Plane", (p1.Surface.Position, p1.Surface.Axis, 1.0, 1.0)))

    params = [gcyl, gp1]
    if len(cs) == 3:
        p2 = cs[2]
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
