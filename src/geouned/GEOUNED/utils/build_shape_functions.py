import Part
import FreeCAD
import numpy

from .split_function import split_bop
from .data_classes import Options


def makePlane(normal, position, Box):

    p0 = normal.dot(position)

    pointEdge = []
    for i in range(12):
        edge = Box.getEdge(i)
        p1 = normal.dot(edge[0])
        p2 = normal.dot(edge[1])
        d0 = p0 - p1
        d1 = p2 - p1
        if d1 != 0:
            a = d0 / d1
            if a >= 0 and a <= 1:
                pointEdge.append(edge[0] + a * (edge[1] - edge[0]))

    if len(pointEdge) == 0:
        return None  # Plane does not cross box

    s = FreeCAD.Vector((0, 0, 0))
    for v in pointEdge:
        s = s + v
    s = s / len(pointEdge)

    vtxvec = []
    for v in pointEdge:
        vtxvec.append(v - s)

    X0 = vtxvec[0]
    Y0 = normal.cross(X0)

    orden = []
    for i, v in enumerate(vtxvec):
        phi = numpy.arctan2(v.dot(Y0), v.dot(X0))
        orden.append((phi, i))
    orden.sort()

    return Part.Face(Part.makePolygon([pointEdge[p[1]] for p in orden], True))


def makeCylinder(axis, center, radius, Box):
    dmin = axis.dot((Box.getPoint(0) - center))
    dmax = dmin
    for i in range(1, 8):
        d = axis.dot((Box.getPoint(i) - center))
        dmin = min(d, dmin)
        dmax = max(d, dmax)

    center = center + (dmin - 5) * axis
    length = dmax - dmin + 10
    return Part.makeCylinder(radius, length, center, axis, 360.0)


def makeCone(axis, apex, tan, Box):
    dmax = axis.dot((Box.getPoint(0) - apex))
    for i in range(1, 8):
        d = axis.dot((Box.getPoint(i) - apex))
        dmax = max(d, dmax)

    if dmax > 0:
        length = dmax + 10
        rad = tan * length
        return Part.makeCone(0.0, rad, length, apex, axis, 360.0)
    else:
        return None


def makeMultiPlanes(plane_list: list, vertex_list: list, box: FreeCAD.BoundBox, multibuild=True):
    """build CAD object (FreeCAD Shell) of the multiplane surface"""
    boxlim = (box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)
    cutfaces = makeBoxFaces(boxlim)

    for p in plane_list:
        axis = (
            -p.Surf.Axis if multibuild else p.Surf.Axis
        )  # for mutliplane shape construction planes direction must be inverted
        plane = Part.Plane(p.Surf.Position, axis)
        newbox_points = cut_box(cutfaces, plane)
        cutfaces = makeBoxFaces(newbox_points)
    if multibuild:
        plane_points = remove_box_faces(newbox_points, cutfaces, boxlim)
        fix_points(plane_points, vertex_list)
    else:
        plane_points = newbox_points

    return Part.makeShell(makeBoxFaces(plane_points))


def makeRoundCorner(cylinder, addPlane, planes, config, Box):
    cylinder.build_surface(Box)
    addplane_part = Part.makeSolid(makeMultiPlanes([addPlane], [], Box, False))
    plane_part = Part.makeSolid(makeMultiPlanes(planes, [], Box, False))

    cyl_region = cylinder.shape.fuse(addplane_part)
    if config == "AND":
        solid = intersection(cyl_region, plane_part)
    else:
        solid = cyl_region.fuse(plane_part)
    return solid.removeSplitter()


def intersection(sol1, sol2):
    d1 = sol1.cut(sol2)
    d2 = sol2.cut(sol1)
    d12 = d1.fuse(d2)
    return sol1.cut(d12)


def makeCylinderCan(cylinder, plane_list, Box):

    radius = cylinder.Surf.Radius
    axis = cylinder.Surf.Axis
    center = cylinder.Surf.Center
    plane1 = plane_list[0]
    normal1 = plane1.Surf.Axis
    pos1 = plane1.Surf.Position
    if len(plane_list) == 2:
        plane2 = plane_list[1]
        normal2 = plane2.Surf.Axis
        pos2 = plane2.Surf.Position

    options = Options()

    g1 = (pos1 - center).dot(normal1) / normal1.dot(axis)
    pt1 = center + g1 * axis

    axisnorm = axis.dot(normal1)
    orto = abs(axisnorm) > 1 - 1e-8
    if len(plane_list) == 2 and orto:
        orto = abs(axis.dot(normal2)) > 1 - 1e-8
    
    dmin = axis.dot(Box.getPoint(0) - pt1)
    dmax = dmin
    for i in range(1, 8):
        d = axis.dot(Box.getPoint(i) - pt1)
        dmin = min(d, dmin)
        dmax = max(d, dmax)
    if not orto:
        height = dmax - dmin
        dmin -= 0.1 * height
        dmax += 0.1 * height
        height = dmax - dmin

        point = pt1 + dmin * axis
        cylshape = Part.makeCylinder(radius, height, point, axis, 360)
        planeshape1 = makePlane(normal1, pt1, Box)

        if len(plane_list) == 1:
            comsolid = split_bop(cylshape, [planeshape1], options.splitTolerance, options)
            s1, s2 = comsolid.Solids
            v1 = s1.CenterOfMass - pt1
            if v1.dot(normal1) > 0:
                return s2
            else:
                return s1
        else:
            g2 = (pos2 - center).dot(normal2) / normal2.dot(axis)
            pt2 = center + g2 * axis
            planeshape2 = makePlane(normal2, pt2, Box)
            comsolid = split_bop(cylshape, [planeshape1, planeshape2], options.splitTolerance, options)
            s1, s2, s3 = comsolid.Solids
            v1 = s1.CenterOfMass - pt1
            v2 = s1.CenterOfMass - pt2
            if v1.dot(v2) < 0:
                return s1
            v1 = s2.CenterOfMass - pt1
            v2 = s2.CenterOfMass - pt2
            if v1.dot(v2) < 0:
                return s2
            return s3

    else:
        if len(plane_list) == 1:
            if axisnorm > 0:
                height = -dmin
                axis = -axis
            else:
                height = dmax
            return Part.makeCylinder(radius, height, pt1, axis, 360)
        else:
            g2 = (plane2.Surf.Position - center).dot(plane2.Surf.Axis) / plane2.Surf.Axis.dot(axis)
            if g1 > g2:
                axis = -axis
            return Part.makeCylinder(radius, abs(g2 - g1), pt1, axis, 360)


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
            if abs(abs(l.Direction.dot(e.Curve.Direction))-1) < 1e-6:
                pt = [] # if e and line are parallel not point or infinity
            else:    
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
