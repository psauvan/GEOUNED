import FreeCAD
import Part
import numpy
from .geometry_gu import PlaneGu
from .functions import GeounedSurface


def get_metaplanes(solid, universe_box):
    planes = []
    for f in solid.Faces:
        if type(f.Surface) is Part.Plane:
            planes.append(f)

    metaplane_list = []
    for p in planes:
        loop = False
        for mp in metaplane_list:
            if p in mp:
                loop = True
        if loop:
            continue
        metap = [p]
        meta_loop([p], metap, planes)
        if len(metap) != 1:
            if no_convex(metap):
                mp_params = build_metap_params(metap)
                mp = GeounedSurface(
                    ("MetaPlane", mp_params),
                    universe_box,
                    Face="Build",
                )
                metaplane_list.append(mp)
    return metaplane_list


def meta_loop(adjacents, metalist, planes):
    for p in adjacents:
        new_adjacents = metaplane(p, planes)
        for ap in reversed(new_adjacents):
            if ap in metalist:
                new_adjacents.remove(ap)
        metalist.extend(new_adjacents)
        meta_loop(new_adjacents, metalist, planes)


def no_convex(meta_plane_list):
    planes = meta_plane_list[:]
    while len(planes) > 1:
        p = planes.pop()
        Edges = p.OuterWire.Edges
        for e in Edges:
            try:
                type_curve = type(e.Curve)
            except:
                type_curve = None
            if type_curve is not Part.Line:
                continue
            adjacent_plane = other_face_edge(e, p, planes, outer_only=True)
            if adjacent_plane is not None:
                sign = region_sign(p, adjacent_plane, e)
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


def metaplane(p, planes):
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
            sign = region_sign(p, adjacent_plane, e)
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
        if face.isSame(current_face):
            continue

        Edges = face.OuterWire.Edges if outer_only else face.Edges
        for edge in Edges:
            if current_edge.isSame(edge):
                return face


def build_metap_params(plane_list):

    planeparams = []
    edges = []
    vertexes = []

    for p in plane_list:
        plane = PlaneGu(p)
        planeparams.append((plane.Position, plane.Axis, plane.dim1, plane.dim2))

    ajdacent_planes = [[] for i in range(len(plane_list))]
    for i, p1 in enumerate(plane_list):
        for j, p2 in enumerate(plane[i + 1 :]):
            e = commonEdge(p1, p2)
            if e is not None:
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


def commonVertex(e1, e2):
    if e1.distToShape(e2) > 0:
        return []

    common = []
    if e1.Vertexes[0] == e2.Vertexes[0]:
        common.append(e1.Vertexes[0])
    elif e1.Vertexes[0] == e2.Vertexes[1]:
        common.append(e1.Vertexes[0])

    if e1.Vertexes[1] == e2.Vertexes[0]:
        common.append(e1.Vertexes[1])
    elif e1.Vertexes[1] == e2.Vertexes[1]:
        common.append(e1.Vertexes[1])

    return common


def commonEdge(face1, face2, outer_only=True):
    if face1.distToShape[0] > 0:
        return None

    Edges1 = face1.OuterWire.Edges if outer_only else face1.Edges
    Edges2 = face2.OuterWire.Edges if outer_only else face2.Edges
    for e1 in Edges1:
        for e2 in Edges2:
            if e1.isSame(e2):
                return e1
    return None


def makeBoxFaces(box):
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
        faces.append(Part.Face(Part.makePolygon(f, True)))
    return faces


def cut_face(face, plane):
    line = face.Surface.intersect(plane)
    inter = []
    if len(line) > 0:
        l = line[0]
        for e in face.Edges:
            pt = l.intersect(e.Curve)
            if len(pt) > 0:
                point = pt[0].toShape().Point
                if e.isInside(point, 1e-12, True):
                    inter.append(point)

    newpoints = inter[:]
    for v in face.Vertexes:
        if v.Point.dot(plane.Axis) > 0:
            newpoints.append(v.Point)
    if len(newpoints) == 0:
        return None, None
    else:
        sorted = sort_points(newpoints, face.Surface.Axis)
        return sorted, inter


def cut_box(faces, plane):
    updatedfaces = []
    newface_points = []
    for f in faces:
        newface, newpoints = cut_face(f, plane)
        if newface is None:
            continue
        updatedfaces.append(newface)
        newface_points.extend(newpoints)

    sorted = sort_points(newface_points, plane.Axis)
    updatedfaces.append(sorted)

    return updatedfaces


def sort_points(point_list, normal):

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

    points = tuple(point_list[p[1]] for p in orden)
    return points


def remove_box_faces(point_face_list, faces, boxlim):
    plane_points = []
    for i, face in enumerate(faces):
        if abs(face.Surface.Axis.dot(FreeCAD.Vector(1, 0, 0)) - 1) < 1e-12 and abs(boxlim[0] - face.Surface.Position.X) < 1e-12:
            continue
        elif (
            abs(face.Surface.Axis.dot(FreeCAD.Vector(-1, 0, 0)) - 1) < 1e-12
            and abs(boxlim[3] - face.Surface.Position.X) < 1e-12
        ):
            continue
        elif (
            abs(face.Surface.Axis.dot(FreeCAD.Vector(0, 1, 0)) - 1) < 1e-12 and abs(boxlim[1] - face.Surface.Position.Y) < 1e-12
        ):
            continue
        elif (
            abs(face.Surface.Axis.dot(FreeCAD.Vector(0, -1, 0)) - 1) < 1e-12
            and abs(boxlim[4] - face.Surface.Position.Y) < 1e-12
        ):
            continue
        elif (
            abs(face.Surface.Axis.dot(FreeCAD.Vector(0, 0, 1)) - 1) < 1e-12 and abs(boxlim[2] - face.Surface.Position.Z) < 1e-12
        ):
            continue
        elif (
            abs(face.Surface.Axis.dot(FreeCAD.Vector(0, 0, -1)) - 1) < 1e-12
            and abs(boxlim[5] - face.Surface.Position.Z) < 1e-12
        ):
            continue
        plane_points.append(point_face_list[i])

    return plane_points


def fix_points(point_plane_list, vertex_list):
    tol = 1e-8
    for i, current_plane in enumerate(point_plane_list):
        for point in current_plane:
            for planepts in point_plane_list[i + 1 :]:
                for j in range(len(planepts)):
                    r = point - planepts[j]
                    if r.Length < tol:
                        planepts[j] = point

    for v in vertex_list:
        for planpts in point_plane_list:
            for i in range(len(planpts)):
                r = v.Point - planepts[i]
                if r.Length < tol:
                    planpts[i] = v.Point


def makeMetaPlanes(plane_list, vertex_list, box):
    boxlim = (box.Xmin, box.YMin, box.Zmin, box.XMax, box.YMax, box.ZMax)
    cutfaces = makeBoxFaces(boxlim)
    for p in plane_list:
        plane = Part.Plane(p.Position, p.Axis)
        newbox_points = cut_box(cutfaces, plane)
        cutfaces = makeBoxFaces(newbox_points)

    plane_points = remove_box_faces(newbox_points, cutfaces, boxlim)
    fix_points(plane_points, vertex_list)
    metaFaces = Part.Shell(makeBoxFaces(plane_points))

    return metaFaces
