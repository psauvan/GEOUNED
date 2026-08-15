import numpy

from .build_region.build_region import BuildDepth, get_cell_object, getPart, FuseSolid
from .build_region.Objects import myBox, plane_polygon_from_box, cylinder_from_box, cone_from_box
from ...geo import (
    GBoundBox,
    GPlane,
    GCylinder,
    GCone,
    GFace,
    GVector,
    Gmake_shell,
    Gmake_polygon_face,
)


def makePlane(normal: GVector, position: GVector, box: GBoundBox):
    p0 = normal.dot(position)
    face = plane_polygon_from_box(normal, p0, box)
    return face.__native__ if face is not None else None  # None: Plane does not cross box


def makeCylinder(center: GVector, axis: GVector, radius: float, box: GBoundBox):
    gsolid = cylinder_from_box(center, axis, radius, box)
    Cylinder = gsolid.__native__
    shell = next(f.__native__ for f in gsolid.Faces if type(f.Surface) is GCylinder)
    return (Cylinder, shell)


def makeCone(axis: GVector, apex: GVector, tan: float, box: GBoundBox):
    gsolid = cone_from_box(apex, axis, tan, box)
    if gsolid is None:
        return None
    cone = gsolid.__native__
    shell = next(f.__native__ for f in gsolid.Faces if type(f.Surface) is GCone)
    return (cone, shell)


def makeMultiPlanes(plane_list: list, vertex_list: list, box: GBoundBox, multibuild=True):
    """build CAD object (FreeCAD Shell) of the multiplane surface"""
    boxlim = (box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)
    cutfaces = makeBoxFaces(boxlim)

    for p in plane_list:
        axis = p.Surf.Axis
        if multibuild:
            axis = -axis  # for mutliplane shape construction planes direction must be inverted
        plane = GPlane.from_values(p.Surf.Position, axis)
        newbox_points = cut_box(cutfaces, plane)
        cutfaces = makeBoxFaces(newbox_points)
    if multibuild:
        plane_points = remove_box_faces(newbox_points, cutfaces, boxlim)
        fix_points(plane_points, vertex_list)
    else:
        plane_points = newbox_points

    if len(plane_points) == 0:
        return None  # multiplane doesn't cross box
    else:
        return Gmake_shell(makeBoxFaces(plane_points)).__native__


def makeRoundCorner(roundCorner, Box):
    return build_complex_shape(roundCorner, Box)


def makeMultiRoundCorner(multiRoundCorner, Box, forward=False):
    return build_complex_shape(multiRoundCorner, Box, forward=forward)


def makeCan(can, Box, forward=False):
    return build_complex_shape(can, Box, forward=forward)


def makeTCone(tcone, Box):
    return build_complex_shape(tcone, Box)


def build_complex_shape(surface, Box, forward=False):
    rc = get_cell_object(surface)
    rc.boundBox = myBox(Box, "Forward")
    if forward:
        # if forward is True, the forward shape of the surface is built instead of the reversed shape.
        if surface.Orientation == "Reversed":
            rc.definition = rc.definition.get_complementary()
    for s in rc.surfaces.values():
        s.buildShape(Box)
    celparts = BuildDepth(rc, None)
    celparts = getPart(celparts)
    if len(celparts) == 0:
        return (None, None)

    shapeParts = []
    for i, s in enumerate(celparts):
        shapeParts.append(s.base)

    gsolid = FuseSolid(shapeParts)
    if rc.boundBox.sameBox(myBox(gsolid.BoundBox)) and rc.boundBox.Volume == gsolid.Volume:
        # surface shape doesn't cut box
        return (None, None)

    # build_complex_shape's own callers (makeCan/makeTCone/makeRoundCorner/
    # makeMultiRoundCorner, and everything downstream of them:
    # GeounedSurface.shape/.shell, .exportStep(), Gsplit(...) callers) all
    # expect native Part shapes, so convert here, at the boundary, rather
    # than pushing this change any further out.
    solid = gsolid.__native__
    if len(solid.Shells) == 0:
        shell = solid.Shells[0]
    else:
        shell = solid.Shells[0]  # not sure if for Reversed MultiRoundConer inner shells is the index 0 shell
    return (solid, shell)


def makeBoxFaces(box: list):
    """Build faces of a box. Returns a list of GFace."""
    if isinstance(box[0], (int, float)):
        xmin, ymin, zmin, xmax, ymax, zmax = box
        v0 = GVector(xmin, ymin, zmin)
        v1 = GVector(xmin, ymax, zmin)
        v2 = GVector(xmin, ymax, zmax)
        v3 = GVector(xmin, ymin, zmax)
        v4 = GVector(xmax, ymin, zmin)
        v5 = GVector(xmax, ymax, zmin)
        v6 = GVector(xmax, ymax, zmax)
        v7 = GVector(xmax, ymin, zmax)
        face1 = (v0, v1, v2, v3)
        face2 = (v7, v6, v5, v4)
        face3 = (v0, v3, v7, v4)
        face4 = (v5, v6, v2, v1)
        face5 = (v4, v5, v1, v0)
        face6 = (v6, v7, v3, v2)

        faces_points = (face1, face2, face3, face4, face5, face6)

    else:
        faces_points = box

    faces = []
    for f in faces_points:
        if len(f) < 3:
            continue
        faces.append(Gmake_polygon_face(f))
    return faces


def cut_face(gface: GFace, plane: GPlane):
    """Cut the "face" with the "plane". Remaining part is portion in "plane" normal direction."""
    gline = gface.Surface.intersect_plane(plane)  # faces here are always planar (built by makeBoxFaces)
    inter = []
    if gline is not None:
        for e in gface.Edges:
            edge_line = e.Curve  # edges here are always straight (built by makeBoxFaces)
            if abs(abs(gline.Direction.dot(edge_line.Direction)) - 1) < 1e-6:
                point = None  # if e and line are parallel: no point or infinity
            else:
                point = gline.intersect_line(edge_line)

            if point is not None and e.is_inside(point, 1e-8):
                inter.append(point)

    newpoints = inter[:]
    for v in gface.Vertexes:
        if plane.Axis.dot(v - plane.Position) > 0:
            newpoints.append(v)
    if len(newpoints) == 0:
        return None, None
    else:
        sorted_points = sort_points(newpoints, gface.Surface.Axis)
        return sorted_points, inter


def cut_box(faces: list, plane: GPlane):
    """Cut the box make of planar faces with "plane" """
    updatedfaces = []
    newface_points = []
    for f in faces:
        newface, newpoints = cut_face(f, plane)
        if newface is None:
            continue
        updatedfaces.append(newface)
        newface_points.extend(newpoints)

    fix_same_points(newface_points)
    sorted_points = sort_points(newface_points, plane.Axis)
    updatedfaces.append(sorted_points)

    return updatedfaces


def sort_points(point_list: list, normal: GVector):
    """Sort the points of the polygon face in anti-clock wise with respect vector "normal"."""
    if len(point_list) == 0:
        return []

    s = GVector(0, 0, 0)
    for v in point_list:
        s = s + v
    s = s / len(point_list)

    vtxvec = []
    for v in point_list:
        vtxvec.append(v - s)

    X0 = vtxvec[0]
    Y0 = normal.cross(X0)

    orden = []
    for i, v in enumerate(vtxvec):
        phi = numpy.arctan2(v.dot(Y0), v.dot(X0))
        orden.append((phi, i))
    orden.sort()

    points = list(point_list[p[1]] for p in orden)
    return points


def remove_box_faces(point_face_list: list, faces: list, boxlim: list):
    """Remove the remaing initial BoundBox faces from the multplane faces produced"""
    tol = 1e-8
    plane_points = []
    for i, gface in enumerate(faces):
        axis = gface.Surface.Axis
        position = gface.Surface.Position
        if abs(axis.dot(GVector(1, 0, 0)) - 1) < tol and abs(boxlim[0] - position.x) < tol:
            continue
        elif abs(axis.dot(GVector(-1, 0, 0)) - 1) < tol and abs(boxlim[3] - position.x) < tol:
            continue
        elif abs(axis.dot(GVector(0, 1, 0)) - 1) < tol and abs(boxlim[1] - position.y) < tol:
            continue
        elif abs(axis.dot(GVector(0, -1, 0)) - 1) < tol and abs(boxlim[4] - position.y) < tol:
            continue
        elif abs(axis.dot(GVector(0, 0, 1)) - 1) < tol and abs(boxlim[2] - position.z) < tol:
            continue
        elif abs(axis.dot(GVector(0, 0, -1)) - 1) < tol and abs(boxlim[5] - position.z) < tol:
            continue
        plane_points.append(point_face_list[i])

    return plane_points


def fix_same_points(points_inplane: list):
    """Replace all point separated by distance < tol, by the same point.
    Replace all vertexes point in multiplane surface by the original vertex point.
    """
    tol = 1e-8
    remove = []
    for i, p1 in enumerate(points_inplane):
        if i in remove:
            continue
        for p2 in points_inplane[i + 1 :]:
            r = p1 - p2
            if r.length < tol:
                remove.append(i)

    for ind in reversed(remove):
        del points_inplane[ind]


def fix_points(point_plane_list: list, vertex_list: list):
    """Replace all point separated by distance < tol, by the same point.
    Replace all vertexes point in multiplane surface by the original vertex point.
    """
    tol = 1e-8
    for i, current_plane in enumerate(point_plane_list):
        for point in current_plane:
            for planepts in point_plane_list[i + 1 :]:
                for j in range(len(planepts)):
                    r = point - planepts[j]
                    if r.length < tol:
                        planepts[j] = point

    for v, ord in vertex_list:
        for planepts in point_plane_list:
            for i in range(len(planepts)):
                r = v - planepts[i]
                if r.length < tol:
                    planepts[i] = v
