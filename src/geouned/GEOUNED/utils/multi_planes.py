import FreeCAD
import Part
import numpy


class plane_index:
    def __init__(self, plane, index, orientation):
        self.plane = plane
        self.index = index
        self.orientation = orientation


def multiplane_loop(adjacents, multi_list, planes):
    for p in adjacents:
        new_adjacents = multiplane(p, planes)
        for ap in reversed(new_adjacents):
            if ap in multi_list:
                new_adjacents.remove(ap)
        multi_list.extend(new_adjacents)
        multiplane_loop(new_adjacents, multi_list, planes)


def no_convex(mplane_list):
    planes = mplane_list[:]
    while len(planes) > 1:
        p = planes.pop()
        Edges = p.plane.OuterWire.Edges
        for e in Edges:
            try:
                type_curve = type(e.Curve)
            except:
                type_curve = None
            if type_curve is not Part.Line:
                continue
            adjacent_plane = other_face_edge(e, p, planes, outer_only=True)
            if adjacent_plane is not None:
                sign = region_sign(p.plane, adjacent_plane.plane, e)
                if sign == "AND":
                    return False
    return True


def convex_wire(p):
    Edges = p.OuterWire.Edges
    for e in Edges:
        if type(e.Curve) != Part.Line:
            return []

    normal = p.Surface.Axis
    v0 = Edges[0].Curve.Direction.cross(p.Surface.Axis)
    if Edges[0].Orientation == "Forward":
        v0 = -v0

    for e in Edges[1:]:
        v1 = e.Curve.Direction.cross(p.Surface.Axis)
        if e.Orientation == "Forward":
            v1 = -v1
        if normal.dot(v0.cross(v1)) < 0:
            return []
        v0 = v1

    return Edges


def multiplane(p, planes):
    """Found planes adjacent to "p". Region delimited by plane is concanve."""
    Edges = p.plane.OuterWire.Edges
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
            sign = region_sign(p.plane, adjacent_plane.plane, e)
            if sign == "OR":
                addplane.append(adjacent_plane)
    return addplane


def region_sign(p1, p2, e1):
    normal1 = p1.Surface.Axis
    direction = e1.Curve.Direction
    if e1.Orientation == "Forward":
        direction = -direction
    vect = direction.cross(normal1)  # toward inside the face. For vect, p1 face orientation doesn't matter.

    normal2 = p2.Surface.Axis
    if p2.Orientation == "Forward":
        normal2 = -normal2

    return "OR" if vect.dot(normal2) < 0 else "AND"


def other_face_edge(current_edge, current_face, Faces, outer_only=False):
    for face in Faces:
        if face.plane.isSame(current_face.plane):
            continue

        Edges = face.plane.OuterWire.Edges if outer_only else face.plane.Edges
        for edge in Edges:
            if current_edge.isSame(edge):
                return face


def commonVertex(e1, e2):
    if e1.distToShape(e2)[0] > 0:
        return []

    common = []
    if e1.Vertexes[0].Point == e2.Vertexes[0].Point:
        common.append(e1.Vertexes[0])
    elif e1.Vertexes[0].Point == e2.Vertexes[1].Point:
        common.append(e1.Vertexes[0])

    if e1.Vertexes[1].Point == e2.Vertexes[0].Point:
        common.append(e1.Vertexes[1])
    elif e1.Vertexes[1].Point == e2.Vertexes[1].Point:
        common.append(e1.Vertexes[1])

    return common


def commonEdge(face1, face2, outer_only=True):
    if face1.distToShape(face2)[0] > 0:
        return None

    Edges1 = face1.OuterWire.Edges if outer_only else face1.Edges
    Edges2 = face2.OuterWire.Edges if outer_only else face2.Edges
    for e1 in Edges1:
        for e2 in Edges2:
            if e1.isSame(e2):
                return e1
    return None


def makeBoxFaces(box: list):
    """Build faces of a box."""
    if isinstance(box[0], (int, float)):
        xmin, ymin, zmin, xmax, ymax, zmax = box
        v0 = FreeCAD.Vector(xmin, ymin, zmin)
        v1 = FreeCAD.Vector(xmin, ymax, zmin)
        v2 = FreeCAD.Vector(xmin, ymax, zmax)
        v3 = FreeCAD.Vector(xmin, ymin, zmax)
        v4 = FreeCAD.Vector(xmax, ymin, zmin)
        v5 = FreeCAD.Vector(xmax, ymax, zmin)
        v6 = FreeCAD.Vector(xmax, ymax, zmax)
        v7 = FreeCAD.Vector(xmax, ymin, zmax)
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
        faces.append(Part.Face(Part.makePolygon(f, True)))
    return faces


def cut_face(face: Part.Face, plane: Part.Plane):
    """Cut the "face" with the "plane". Remaining part is portion in "plane" normal direction."""
    line = face.Surface.intersect(plane)
    inter = []
    if len(line) > 0:
        l = line[0]
        for e in face.Edges:
            pt = l.intersect(e.Curve)
            if len(pt) > 0:
                point = pt[0].toShape().Point
                if e.isInside(point, 1e-8, True):
                    inter.append(point)

    newpoints = inter[:]
    for v in face.Vertexes:
        if plane.Axis.dot(v.Point - plane.Position) > 0:
            newpoints.append(v.Point)
    if len(newpoints) == 0:
        return None, None
    else:
        sorted = sort_points(newpoints, face.Surface.Axis)
        return sorted, inter


def cut_box(faces: list, plane: Part.Plane):
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
    sorted = sort_points(newface_points, plane.Axis)
    updatedfaces.append(sorted)

    return updatedfaces


def sort_points(point_list: list, normal: FreeCAD.Vector):
    """Sort the points of the polygon face in anti-clock wise with respect vector "normal"."""
    if len(point_list) == 0:
        return []

    s = FreeCAD.Vector((0, 0, 0))
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
    for i, face in enumerate(faces):
        if abs(face.Surface.Axis.dot(FreeCAD.Vector(1, 0, 0)) - 1) < tol and abs(boxlim[0] - face.Surface.Position.x) < tol:
            continue
        elif abs(face.Surface.Axis.dot(FreeCAD.Vector(-1, 0, 0)) - 1) < tol and abs(boxlim[3] - face.Surface.Position.x) < tol:
            continue
        elif abs(face.Surface.Axis.dot(FreeCAD.Vector(0, 1, 0)) - 1) < tol and abs(boxlim[1] - face.Surface.Position.y) < tol:
            continue
        elif abs(face.Surface.Axis.dot(FreeCAD.Vector(0, -1, 0)) - 1) < tol and abs(boxlim[4] - face.Surface.Position.y) < tol:
            continue
        elif abs(face.Surface.Axis.dot(FreeCAD.Vector(0, 0, 1)) - 1) < tol and abs(boxlim[2] - face.Surface.Position.z) < tol:
            continue
        elif abs(face.Surface.Axis.dot(FreeCAD.Vector(0, 0, -1)) - 1) < tol and abs(boxlim[5] - face.Surface.Position.z) < tol:
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
            if r.Length < tol:
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
                    if r.Length < tol:
                        planepts[j] = point

    for v, ord in vertex_list:
        for planepts in point_plane_list:
            for i in range(len(planepts)):
                r = v.Point - planepts[i]
                if r.Length < tol:
                    planepts[i] = v.Point


def makeMultiPlanes(plane_list: list, vertex_list: list, box: FreeCAD.BoundBox):
    """build CAD object (FreeCAD Shell) of the multiplane surface"""
    boxlim = (box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)
    cutfaces = makeBoxFaces(boxlim)
    for p in plane_list:
        plane = Part.Plane(p.Surf.Position, -p.Surf.Axis)  # for mutliplane shape construction planes direction must be inverted
        newbox_points = cut_box(cutfaces, plane)
        cutfaces = makeBoxFaces(newbox_points)

    plane_points = remove_box_faces(newbox_points, cutfaces, boxlim)
    fix_points(plane_points, vertex_list)

    return Part.makeShell(makeBoxFaces(plane_points))
