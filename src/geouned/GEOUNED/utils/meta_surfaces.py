import FreeCAD
import Part
import math
import numpy

from .geometry_gu import PlaneGu, CylinderGu, TorusGu
from .geouned_classes import GeounedSurface
from .data_classes import Tolerances

twoPi = 2 * math.pi


def multiplane_loop(adjacents, multi_list, planes):
    for p in adjacents:
        new_adjacents = multiplane(p, planes)
        for ap in reversed(new_adjacents):
            if ap in multi_list:
                new_adjacents.remove(ap)
        multi_list.extend(new_adjacents)
        multiplane_loop(new_adjacents, multi_list, planes)


def no_convex_full(mplane_list):
    """keep planes only all planes are no convex each other"""
    planes = mplane_list[:]
    for p in planes:
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
                sign = region_sign(p, adjacent_plane)
                if sign == "AND":
                    return False
    return True


def no_convex(mplane_list):
    """keep part of no complex plane set"""
    planes = mplane_list[:]
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
                sign = region_sign(p, adjacent_plane)
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
            sign = region_sign(p, adjacent_plane)
            if sign == "OR":
                addplane.append(adjacent_plane)
    return addplane


def region_sign(p1, s2):
    normal1 = p1.Surface.Axis
    e1 = commonEdge(p1, s2, outer_only=False)

    if isinstance(e1.Curve, Part.Line):
        direction = e1.Curve.Direction

        if isinstance(s2.Surface, PlaneGu):
            arc = 0
            normal2 = s2.Surface.Axis
            if s2.Orientation == "Reversed":
                normal2 = -normal2
        else:
            umin, umax, vmin, vmax = s2.Surface.face.ParameterRange
            arc = abs(umax - umin)
            pos = e1.Vertexes[0].Point
            u, v = u, v = s2.Surface.face.Surface.parameter(pos)

            normal2 = s2.Surface.face.normalAt(u, v)
    else:
        arc = 0
        pos = e1.Vertexes[0].Point
        pe = e1.Curve.parameter(pos)
        direction = e1.derivative1At(pe)
        direction.normalize()

        u, v = s2.Surface.face.Surface.parameter(pos)
        normal2 = s2.Surface.face.normalAt(u, v)

    vect = direction.cross(normal1)
    dprod = vect.dot(normal2)
    if abs(dprod) < 1e-4:
        return "AND" if abs(dprod) < arc else "OR"
    else:
        if p1.Orientation != e1.Orientation:
            dprod = -dprod
        return "OR" if dprod < 0 else "AND"


def other_face_edge(current_edge, current_face, Faces, outer_only=False):
    for face in Faces:
        if face.Index == current_face.Index:
            continue

        Edges = face.OuterWire.Edges if outer_only else face.Edges
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


def get_revcan_surfaces(cylinder, solid):

    same_cylinder_faces, cylindex = get_adjacent_cylinder_faces(cylinder, solid.Faces)
    if not is_closed_cylinder(same_cylinder_faces):
        return None, None
    feLow, feHigh = get_side_edges(same_cylinder_faces)

    lowPlane, indexlow = get_closing_plane(feLow, solid.Faces, "low")
    highPlane, indexhigh = get_closing_plane(feHigh, solid.Faces, "high")

    surfaces = [cylinder]
    if lowPlane:
        surfaces.append(lowPlane)
        cylindex.update(indexlow)
    if highPlane:
        surfaces.append(highPlane)
        cylindex.update(indexhigh)

    if len(surfaces) > 1:
        return surfaces, cylindex
    else:
        return None, None


def get_roundcorner_surfaces(cylinder, solid):

    adjacent_planes = get_adjacent_cylplane(cylinder, solid.Faces)
    if len(adjacent_planes) != 2:
        return None, None

    p1, p2 = adjacent_planes
    r1 = region_sign(p1, cylinder)
    r2 = region_sign(p2, cylinder)
    if r1 != r2:
        return None, None
    face_index = {cylinder.Index, p1.Index, p2.Index}
    faces = ((cylinder, p1, p2), r1)
    return faces, face_index


def get_adjacent_cylplane(cyl, Faces):
    planes = []
    for e in cyl.OuterWire.Edges:
        if not isinstance(e.Curve, Part.Line):
            continue
        otherface = other_face_edge(e, cyl, Faces, outer_only=True)
        if otherface is None : 
            continue
        if isinstance(otherface.Surface, PlaneGu):
            if abs(otherface.Surface.Axis.dot(cyl.Surface.Axis)) < 1.0e-5:
                planes.append(otherface)

    delindex = []
    for i, p1 in enumerate(planes):
        for j, p2 in enumerate(planes[i + 1 :]):
            if p1.isSame(p2):
                delindex.append(j + i + 1)

    delindex.sort()
    delindex.reverse()
    for i in delindex:
        del planes[i]

    return planes


def get_adjacent_cylinder_faces(cylinder, Faces):
    sameCyl = [cylinder]
    for face in Faces:
        if isinstance(face, CylinderGu):
            if cylinder.Surf.issameCylinder(face.Surf):
                sameCyl.append(face)

    joinCyl = [cylinder]
    cylindex = {
        cylinder.Index,
    }
    get_adjacent_cylface_loop(joinCyl, cylindex, sameCyl)

    faces = []
    for c in sameCyl:
        if c.Index in cylindex:
            faces.append(c)
    return faces, cylindex


def get_adjacent_cylface_loop(joinfaces, cylindex, sameCyl):
    for cyl in joinfaces:
        adjacent = get_ajacent_cylface(cyl, cylindex, sameCyl)
        if adjacent:
            cylindex.update({a.Index for a in adjacent})
            newFacesindex = get_adjacent_cylface_loop(adjacent, cylindex, sameCyl)
            cylindex.update(newFacesindex)


def get_ajacent_cylface(cyl, cylindex, sameCyl):
    adjacent = []
    for c in sameCyl:
        if c.Index in cylindex:
            continue
        if is_adjacent_cylinder(cyl, c):
            adjacent.append(c)
    return adjacent


def is_adjacent_cylinder(c1, c2):
    for e1 in c1.Edges:
        for e2 in c2.Edges:
            if e1.issame(e2):
                return True
    return False


def is_closed_cylinder(face_list):

    Umin, Umax, vmin, vmax = face_list[0].ParameterRange
    for f in face_list[1:]:
        umin, umax, vmin, vmax = f.ParameterRange
        Umin = min(Umin, umin)
        Umax = min(Umax, umax)
    return abs(Umax - Umin) % twoPi < 1e-3


def get_side_edges(cylinder_faces):

    origin = cylinder_faces[0].Surface.Center
    axis = cylinder_faces[0].Surface.Axis
    sideLow = (1e15, None)
    sideHigh = (-1e15, None)

    for ic, cyl in enumerate(cylinder_faces):
        for ie, e in enumerate(cyl.OuterWire.Edges):
            if isinstance(e.Curve, Part.Line):
                continue
            D = axis.dot(e.CenterOfMass - origin)
            if D < sideLow[0]:
                sideLow = (D, (ic, ie))
            if D > sideHigh[0]:
                sideHigh = (D, (ic, ie))

    facelow = cylinder_faces[sideLow[1][0]]
    facehigh = cylinder_faces[sideHigh[1][0]]
    edgelow = facelow.OuterWire.Edges[sideLow[1][1]]
    edgehigh = facehigh.OuterWire.Edges[sideHigh[1][1]]

    return (facelow, edgelow, axis), (facehigh, edgehigh, axis)


def get_closing_plane(face_edge, Faces, lowSide):
    cylface, edge, axis = face_edge
    in_cylinder = face_in_cylinder(edge, cylface)
    otherface = other_face_edge(edge, cylface, Faces, outer_only=False)

    if isinstance(otherface.Surface, PlaneGu):
        if not in_cylinder:
            return None, None
        if lowSide:
            if otherface.Surface.Axis.dot(axis) > 0:
                otherface.Surface.reverse()
        else:
            if otherface.Surface.Axis.dot(axis) < 0:
                otherface.Surface.reverse()

        return otherface, {cylface.Index, otherface.Index}
    else:
        planes = cyl_bound_planes(cylface, Faces, lowSide, Edges=(edge,))
        plane = None
        index = None

        if planes:
            if not convex_face_cyl(cylface, edge, otherface):
                plane = planes[0]
            else:
                if in_cylinder:
                    plane = planes[0]
                    index = otherface.Index
        if plane:
            if lowSide:
                if plane.Surf.Axis.dot(axis) > 0:
                    plane.Surf.Axis = -plane.Surf.Axis
            else:
                if plane.Surf.Axis.dot(axis) < 0:
                    plane.Surf.Axis = -plane.Surf.Axis
        if index:
            faceindex = {cylface.Index, index}
        else:
            faceindex = {cylface.Index}
        return plane, faceindex


def face_in_cylinder(edge, face):
    if isinstance(edge.Curve, Part.BSplineCurve):
        return edge.Curve.getD0(0).dot(face.Surface.Axis) < 0
    else:
        return edge.Curve.Axis.dot(face.Surface.Axis) < 0


def convex_face_cyl(cyl, edge, otherface):
    v1 = cyl.CenterOfMass - edge.CenterOfMass
    v2 = otherface.CenterOfMass - edge.CenterOfMass
    return v1.dot(v2) < 0


def cyl_bound_planes(face, solidFaces, lowSide, Edges=None):

    zaxis = face.Surface.Axis
    if Edges is None:
        Edges = face.OuterWire.Edges
    planes = []
    for e in Edges:
        try:
            curve = str(e.Curve)
        except:
            curve = "none"

        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is not None:
            if type(adjacent_face.Surface) is TorusGu:
                continue  # doesn't create plane if other face is a torus
            if face.Surface.isSameSurface(adjacent_face.Surface):
                continue  # doesn't create plane if other face has same surface

            if curve[0:6] == "Circle":
                dir = e.Curve.Axis
                center = e.Curve.Center
                dim1 = e.Curve.Radius
                dim2 = e.Curve.Radius
                if dir.dot(zaxis) > 0 and lowSide:
                    dir = -dir
                elif dir.dot(zaxis) < 0 and not lowSide:
                    dir = -dir
                plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
                planes.append(plane)

            elif curve == "<Ellipse object>":
                dir = e.Curve.Axis
                center = e.Curve.Center
                dim1 = e.Curve.MinorRadius
                dim2 = e.Curve.MajorRadius
                if dir.dot(zaxis) > 0 and lowSide:
                    dir = -dir
                elif dir.dot(zaxis) < 0 and not lowSide:
                    dir = -dir

                plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
                planes.append(plane)

            elif curve == "<BSplineCurve object>":
                planeParams = plane_spline_curve(e, face.Surface.Axis, lowSide)
                if planeParams is not None:
                    plane = GeounedSurface(("Plane", planeParams))
                    planes.append(plane)

    return planes


def plane_spline_curve(edge, zaxis, lowSide):

    majoraxis = get_axis_inertia(edge.MatrixOfInertia)
    curve_2d = True
    knots = edge.Curve.getKnots()
    poles = edge.Curve.getPoles()
    for k in knots:
        # check if derivative orthogonal to curve normal vector
        normal_k = edge.derivative1At(k).cross(edge.normalAt(k))
        normal_k.normalize()
        if abs(normal_k.dot(majoraxis)) > Tolerances().value:
            curve_2d = False
            break

    if curve_2d:
        return (edge.valueAt(0), majoraxis, 1, 1)
    else:
        rmin = (1e15, None)
        rmax = (-1e15, None)
        for p in poles:
            r = majoraxis.dot(p)
            if rmin[0] > r:
                rmin = (r, p)
            if rmax[0] < r:
                rmax = (r, p)

        rmin = rmin[1]
        rmax = rmax[1]
        d = 0.01 * abs(majoraxis.dot(rmax - rmin))
        vec = majoraxis
        if majoraxis.dot(zaxis) > 0:
            if lowSide:
                vec = -vec
                point = rmin + d * vec
            else:
                point = rmax + d * vec
        else:
            if lowSide:
                point = rmax + d * vec
            else:
                vec = -vec
                point = rmin + d * vec

        return (point, vec, 1, 1)


def get_axis_inertia(mat):
    inertialMat = numpy.array(((mat.A11, mat.A12, mat.A13), (mat.A21, mat.A22, mat.A23), (mat.A31, mat.A32, mat.A33)))
    eigval, evect = numpy.linalg.eig(inertialMat)

    return FreeCAD.Vector(evect.T[numpy.argmax(eigval)])
