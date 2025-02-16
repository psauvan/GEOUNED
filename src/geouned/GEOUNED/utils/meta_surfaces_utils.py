import FreeCAD
import Part
import math
import numpy

from collections import OrderedDict

from .geometry_gu import ShellGu, PlaneGu, CylinderGu, ConeGu, TorusGu, FaceGu, other_face_edge
from .geouned_classes import GeounedSurface
from .data_classes import Tolerances
from ..utils.basic_functions_part1 import is_in_line, is_parallel
from ..conversion.cell_definition_functions import gen_cone, gen_cylinder, cone_apex_plane

twoPi = 2 * math.pi


class reversedCCP:
    def __init__(self, surfType, params):
        self.Type = surfType
        self.Surf_index = set()
        self.Params = params
        self.Connections = []
        self.Index = None


def remove_twice_parallel(mplanes):
    plane_list = []
    omit = set()
    for i, p1 in enumerate(mplanes):
        if p1.Index in omit:
            continue
        parallel = []
        for p2 in mplanes[i + 1 :]:
            if p2.Index in omit:
                continue
            if p1.Surface.isParallel(p2.Surface):
                parallel.append(p2)
                omit.add(p2.Index)
        if len(parallel) > 1:
            parallel.append(p1)
            omit.add(p1.Index)
            plane_list.append(parallel)

    for parallel in plane_list:
        p0 = parallel[0]
        dmin = 0
        dmax = 0
        pmin = p0
        pmax = p0
        for i, p in enumerate(parallel[1:]):
            d = p0.Surface.Axis.dot(p.Surface.Position - p0.Surface.Position)
            if d > dmax:
                dmax = d
                pmax = p
            elif d < dmin:
                dmin = d
                pmin = p

        if dmax - dmin < 1e-5:
            continue

        for p in reversed(parallel):
            if p.Surface.isSameSurface(pmin.Surface):
                parallel.remove(p)
            elif p.Surface.isSameSurface(pmax.Surface):
                parallel.remove(p)

        for p in parallel:
            mplanes.remove(p)


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


def get_adjacent_cylplane(cyl, Faces, cornerPlanes=True):
    planes = []

    if cornerPlanes:
        for e in cyl.OuterWire.Edges:
            if not isinstance(e.Curve, Part.Line):
                continue
            otherface = other_face_edge(e, cyl, Faces, outer_only=True)
            if otherface is None:
                continue
            if isinstance(otherface.Surface, PlaneGu):
                if abs(otherface.Surface.Axis.dot(cyl.Surface.Axis)) < 1.0e-5:
                    planes.append(otherface)
    else:
        for e in cyl.OuterWire.Edges:
            if isinstance(e.Curve, Part.Line):
                continue
            otherface = other_face_edge(e, cyl, Faces, outer_only=False)
            if otherface is None:
                continue
            if isinstance(otherface.Surface, PlaneGu):
                planes.append(otherface)

    delindex = set()
    for i, p1 in enumerate(planes):
        for j, p2 in enumerate(planes[i + 1 :]):
            if p1.isSame(p2):
                delindex.add(j + i + 1)

    delindex = list(delindex)
    delindex.sort()
    delindex.reverse()
    for i in delindex:
        del planes[i]

    return planes


def get_adjacent_cylsurf(cyl, Faces):
    if type(cyl) is ShellGu:
        adjacent = []
        adjIndexes = set()
        surface = cyl.Faces[0].Surface
        for f in cyl.Faces:
            adj = get_adjacent_cylsurfFace(f, Faces)
            for af in adj:
                if af.Surface.isSameSurface(surface):
                    continue
                if af.Index not in adjIndexes:
                    adjIndexes.add(af.Index)
                    adjacent.append(af)

        if len(adjacent) > 2:
            adjacent = most_outer_faces(cyl, adjacent)
        return adjacent

    else:
        return get_adjacent_cylsurfFace(cyl, Faces)


def get_adjacent_cylsurfFace(cyl, Faces):
    adjfaces = []

    other_index = set()
    for e in cyl.OuterWire.Edges:

        if isinstance(e.Curve, Part.Line):
            continue
        otherface = other_face_edge(e, cyl, Faces, outer_only=False)
        if otherface is None:
            continue
        if otherface.Index in other_index:
            continue

        other_index.add(otherface.Index)
        adjfaces.append(otherface)

    delindex = set()
    for i, s1 in enumerate(adjfaces):
        for j, s2 in enumerate(adjfaces[i + 1 :]):
            if s1.isSame(s2):
                delindex.add(j + i + 1)

    delindex = list(delindex)
    delindex.sort()
    delindex.reverse()
    for i in delindex:
        del adjfaces[i]

    return adjfaces


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


def is_closed_cylinder(shape):
    if type(shape) is not ShellGu:
        umin, umax, vmin, vmax = shape.ParameterRange
        return umax - umin > twoPi - 1e-5

    Urange = []
    for f in shape.Faces:
        umin, umax, vmin, vmax = f.ParameterRange
        umin, umax = twoPimod(umin), twoPimod(umax)
        if umin > umax:
            Urange.append((umin - twoPi, umax))
        else:
            Urange.append((umin, umax))

    Urange.sort()
    Umin, Umax = Urange[0]
    if Umin < 0:
        Umin0 = Umin + twoPi
    else:
        Umin0 = Umin

    angle = Umax - Umin
    for umin, umax in Urange[1:]:
        if umax <= Umax:
            continue
        elif umin - Umax < 1e-5:
            angle += umax - Umax
            Umax = umax
            if angle > twoPi - 1e-5:
                return True
        else:
            return False
    return angle > twoPi - 1e-5


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


def face_in_cylinder(edge, face):
    if isinstance(edge.Curve, Part.BSplineCurve):
        return edge.Curve.getD0(0).dot(face.Surface.Axis) < 0
    else:
        return edge.Curve.Axis.dot(face.Surface.Axis) < 0


def convex_face_cyl(cyl, edge, otherface):
    v1 = cyl.CenterOfMass - edge.CenterOfMass
    v2 = otherface.CenterOfMass - edge.CenterOfMass
    return v1.dot(v2) < 0


def get_join_cone_cyl(face, parent_id, GUFaces, multiplanes, omitFaces, tolerances):
    face_index = [face.Index]
    faces = [face]
    joined_faces = []

    for face2 in GUFaces:
        if face2.Index in omitFaces:
            continue
        if type(face2.Surface) != type(face.Surface):
            continue
        if isinstance(face2.Surface, CylinderGu) and face2.Index != face.Index:
            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and abs(face2.Surface.Radius - face.Surface.Radius) < 1e-5
                and is_in_line(face2.Surface.Center, face.Surface.Axis, face.Surface.Center)
            ):
                face_index.append(face2.Index)
                faces.append(face2)
        elif isinstance(face2.Surface, ConeGu) and face2.Index != face.Index:
            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and abs(face2.Surface.SemiAngle - face.Surface.SemiAngle) < 1.0e-5
                and face2.Surface.Apex.sub(face.Surface.Apex).Length < 1e-5
            ):
                face_index.append(face2.Index)
                faces.append(face2)

    sameface_index = [face.Index]  # la face de entrada

    for k in same_faces(faces, tolerances):
        sameface_index.append(face_index[k])

    AngleRange = 0.0
    Uval, UValmin, UValmax = [], [], []
    for index in sameface_index:
        Range = GUFaces[index].ParameterRange
        AngleRange = AngleRange + abs(Range[1] - Range[0])
        Uval.append(Range[0:2])
        UValmin.append(Range[0])
        UValmax.append(Range[1])
    if twoPimod(AngleRange) == 0:
        return []

    omitFaces.update(sameface_index)
    Umin, Umax = sort_range(Uval)

    ifacemin = face_index[UValmin.index(Umin)]
    ifacemax = face_index[UValmax.index(Umax)]

    du = twoPi
    umin = twoPimod(Umin)
    for e in GUFaces[ifacemin].OuterWire.Edges:
        pnt = 0.5 * (e.Vertexes[0].Point + e.Vertexes[-1].Point)
        u, v = GUFaces[ifacemin].__face__.Surface.parameter(pnt)
        if abs(umin - u) < du:
            du = abs(umin - u)
            emin = e

    du = twoPi
    umax = twoPimod(Umax)
    for e in GUFaces[ifacemax].OuterWire.Edges:
        pnt = 0.5 * (e.Vertexes[0].Point + e.Vertexes[-1].Point)
        u, v = GUFaces[ifacemax].__face__.Surface.parameter(pnt)
        if abs(umax - u) < du:
            du = abs(umax - u)
            emax = e

    adjacent1 = other_face_edge(emin, GUFaces[ifacemin], GUFaces)
    adjacent2 = other_face_edge(emax, GUFaces[ifacemax], GUFaces)

    normal1 = None
    normal2 = None
    new_adjacent1 = []
    new_adjacent2 = []

    if adjacent1 is not None:
        if isinstance(adjacent1.Surface, (ConeGu, CylinderGu)):
            if adjacent1.Index not in omitFaces and adjacent1.Orientation == "Reversed":
                new_adjacent1 = get_join_cone_cyl(adjacent1, face.Index, GUFaces, multiplanes, omitFaces, tolerances)
        elif multiplanes:
            if isinstance(adjacent1.Surface, PlaneGu):
                normal1 = adjacent1.Surface.Axis if adjacent1.Orientation == "Forward" else -adjacent1.Surface.Axis

    if adjacent2 is not None:
        if isinstance(adjacent2.Surface, (ConeGu, CylinderGu)):
            if adjacent2.Index not in omitFaces and adjacent2.Orientation == "Reversed":
                new_adjacent2 = get_join_cone_cyl(adjacent2, face.Index, GUFaces, multiplanes, omitFaces, tolerances)
        elif multiplanes:
            if isinstance(adjacent2.Surface, PlaneGu):
                normal2 = adjacent2.Surface.Axis if adjacent2.Orientation == "Forward" else -adjacent2.Surface.Axis

    if type(face.Surface) is CylinderGu:
        cylOnly = gen_cylinder(face)
        cylcone_plane, add_planes = gen_plane_cylinder(ifacemin, ifacemax, Umin, Umax, GUFaces, normal1, normal2)

        facein = reversedCCP("Cylinder", (cylOnly, cylcone_plane, add_planes))
        facein.Surf_index.update(sameface_index)
        facein.Index = face.Index
        if parent_id >= 0:
            facein.Connections.append([parent_id, None])

    else:
        coneOnly = gen_cone(face)
        apexPlane = cone_apex_plane(face, Tolerances())
        cylcone_plane, add_planes = gen_plane_cone(ifacemin, ifacemax, Umin, Umax, GUFaces, normal1, normal2)

        facein = reversedCCP("Cone", (coneOnly, apexPlane, cylcone_plane, add_planes))
        facein.Surf_index.update(sameface_index)
        facein.Index = face.Index
        if parent_id >= 0:
            facein.Connections.append([parent_id, None])

    if new_adjacent1 or new_adjacent2:
        umin, umax, vmin, vmax = face.ParameterRange
        v = 0.5 * (vmin + vmax)
        pmin = face.valueAt(umin, v)
        pmax = face.valueAt(umax, v)
        d = pmax - pmin
        d.normalize()

        for adj in new_adjacent1:
            parents = [x[0] for x in adj.Connections]
            if face.Index in parents:
                i = parents.index(face.Index)
                adjPlane = adj.Params[1].Surf
                operator = "AND" if d.dot(adjPlane.Axis) > 0 else "OR"
                adj.Connections[i][1] = operator
                facein.Connections.append([adj.Index, operator])
        joined_faces.extend(new_adjacent1)

        d = -d
        for adj in new_adjacent2:
            parents = [x[0] for x in adj.Connections]
            if face.Index in parents:
                i = parents.index(face.Index)
                adjPlane = adj.Params[1].Surf
                operator = "AND" if d.dot(adjPlane.Axis) > 0 else "OR"
                adj.Connections[i][1] = operator
                facein.Connections.append([adj.Index, operator])
        joined_faces.extend(new_adjacent2)

    joined_faces.append(facein)
    return joined_faces


# Tolerance in this function are not the general once
# function should be reviewed
def gen_plane_cylinder(ifacemin, ifacemax, Umin, Umax, Faces, normal1=None, normal2=None):

    if ifacemin == ifacemax:
        face2 = Faces[ifacemin]
        try:
            face2.tessellate(0.1)
            UVNode_min = face2.getUVNodes()
        except RuntimeError:
            PR = face2.ParameterRange
            UVNode1 = (PR[0], PR[2])
            UVNode2 = (PR[1], PR[3])
            UVNode_min = (UVNode1, UVNode2)
        UVNode_max = UVNode_min
    else:
        face2min = Faces[ifacemin]
        try:
            face2min.tessellate(0.1)
            UVNode_min = face2min.getUVNodes()
        except RuntimeError:
            PR = face2min.ParameterRange
            UVNode_min = ((PR[0], PR[2]),)

        face2max = Faces[ifacemax]
        try:
            face2max.tessellate(0.1)
            UVNode_max = face2max.getUVNodes()
        except RuntimeError:
            PR = face2max.ParameterRange
            UVNode_max = ((PR[1], PR[3]),)

    dmin = twoPi
    Uminr = twoPimod(Umin)
    Umaxr = twoPimod(Umax)
    for i, node in enumerate(UVNode_min):
        nd = twoPimod(node[0])
        d = abs(nd - Uminr)
        if d < dmin:
            dmin = d
            indmin = i

    dmax = twoPi
    for i, node in enumerate(UVNode_max):
        nd = twoPimod(node[0])
        d = abs(nd - Umaxr)
        if d < dmax:
            dmax = d
            indmax = i

    V1 = Faces[ifacemin].valueAt(UVNode_min[indmin][0], UVNode_min[indmin][1])
    V2 = Faces[ifacemax].valueAt(UVNode_max[indmax][0], UVNode_max[indmax][1])

    axis = Faces[ifacemin].Surface.Axis
    normal = V2.sub(V1).cross(axis)
    normal.normalize()

    plane = GeounedSurface(("Plane", (V1, normal, 1, 1)))

    add_planes = []
    if normal1:
        plane1 = GeounedSurface(("Plane", (V1, normal1, 1, 1)))
        add_planes.append(plane1)

    if normal2:
        plane2 = GeounedSurface(("Plane", (V2, normal2, 1, 1)))
        add_planes.append(plane2)

    return plane, add_planes


# Tolerance in this function are not the general once
# function should be reviewed
def gen_plane_cone(ifacemin, ifacemax, Umin, Umax, Faces, normal1=None, normal2=None):

    if ifacemin == ifacemax:
        face2 = Faces[ifacemin]
        try:
            face2.tessellate(0.1)
            UVNode_min = face2.getUVNodes()
        except RuntimeError:
            PR = face2.ParameterRange
            UVNode1 = (PR[0], PR[2])
            UVNode2 = (PR[1], PR[3])
            UVNode_min = (UVNode1, UVNode2)
        UVNode_max = UVNode_min
    else:
        face2min = Faces[ifacemin]
        try:
            face2min.tessellate(0.1)
            UVNode_min = face2min.getUVNodes()
        except RuntimeError:
            PR = face2min.ParameterRange
            UVNode_min = ((PR[0], PR[2]),)

        face2max = Faces[ifacemax]
        try:
            face2max.tessellate(0.1)
            UVNode_max = face2max.getUVNodes()
        except RuntimeError:
            PR = face2max.ParameterRange
            UVNode_max = ((PR[1], PR[3]),)

    dmin = twoPi
    for i, node in enumerate(UVNode_min):
        nd = twoPimod(node[0])
        d = abs(nd - Umin)
        if d < dmin:
            dmin = d
            indmin = i

    dmax = twoPi
    for i, node in enumerate(UVNode_max):
        nd = twoPimod(node[0])
        d = abs(nd - Umax)
        if d < dmax:
            dmax = d
            indmax = i

    V1 = Faces[ifacemin].valueAt(UVNode_min[indmin][0], UVNode_min[indmin][1])
    V2 = Faces[ifacemax].valueAt(UVNode_max[indmax][0], UVNode_max[indmax][1])

    dir1 = V1 - Faces[ifacemin].Surface.Apex
    dir2 = V2 - Faces[ifacemin].Surface.Apex
    dir1.normalize()
    dir2.normalize()
    normal = dir2.cross(dir1)
    normal.normalize()

    plane = GeounedSurface(("Plane", (Faces[ifacemin].Surface.Apex, normal, 1, 1)))

    add_planes = []
    if normal1:
        plane1 = GeounedSurface(("Plane", (V1, normal1, 1, 1)))
        add_planes.append(plane1)

    if normal2:
        plane2 = GeounedSurface(("Plane", (V2, normal2, 1, 1)))
        add_planes.append(plane2)

    return plane, add_planes


def get_edge(v1, face, normal, axis):
    for edge in face.Edges:
        if type(edge.Curve) == Part.Line:
            vect = v1 - edge.Curve.Location
            if vect.Length < 1e-8:
                return edge
            vect.normalize()
            if abs(abs(vect.dot(edge.Curve.Direction) - 1)) < 1e-5:
                return edge

        elif type(edge.Curve) == Part.BSplineCurve:
            if v1.sub(edge.Vertexes[0].Point).Length < 1e-5 or v1.sub(edge.Vertexes[1].Point).Length < 1e-5:
                vect = edge.Vertexes[0].Point - edge.Vertexes[1].Point
                vect.normalize()
                if abs(vect.dot(normal)) < 1e-5 and abs(vect.dot(axis)) < 1e-5:
                    continue
                else:
                    return edge
        else:
            if abs(abs(edge.Curve.Axis.dot(face.Surface.Axis)) - 1) < 1e-5:
                continue
            else:
                return edge


def sort_range(Urange):
    workRange = Urange[1:]
    current = Urange[0]
    for r in reversed(workRange):
        joined = join_range(current, r)
        if joined is None:
            continue
        current = joined
        workRange.remove(r)
    if len(workRange) == 0:
        return current
    elif len(workRange) == 1:
        joined = join_range(current, workRange[0])
        if joined is None:
            return adjust_range(current, workRange[0])
        else:
            return joined
    else:
        workRange.append(current)
        sorted = sort_range(workRange)
        return sorted


def join_range(U0, U1):
    if (U0[0] - U1[0] < 1e-5) and (-1e-5 < U0[1] - U1[0]):
        if U1[1] > U0[1]:
            return (U0[0], U1[1])
        else:
            return U0
    elif (U0[0] - U1[1] < 1e-5) and (-1e-5 < U0[1] - U1[1]):
        if U1[0] < U0[0]:
            return (U1[0], U0[1])
        else:
            return U0
    elif (U1[0] < U0[0]) and (U0[1] < U1[1]):
        return U1

    elif (U0[0] < U1[0]) and (U1[1] < U0[1]):
        return U0
    else:
        return None


def adjust_range(U0, U1):

    V0 = [twoPimod(x) for x in U0]
    V1 = [twoPimod(x) for x in U1]

    if abs(V0[0] - V1[1]) < 1e-5:
        imin = 1  # U1[0]
        imax = 0  # U0[1]
    elif abs(V1[0] - V0[1]) < 1e-5:
        imin = 0  # U0[0]
        imax = 1  # U1[1]
    elif V1[1] < V0[0]:
        imin = 0  # U0[0]
        imax = 1  # U1[1]
    else:
        imin = 1  # U1[0]
        imax = 0  # U1[0]

    mat = (U0, U1)
    return (mat[imin][0], mat[imax][1])


#   Check if to faces are joint
def contiguous_face(face1, face2, tolerances):
    return face1.distToShape(face2)[0] < tolerances.distance


def same_faces(Faces, tolerances):
    Connection = OrderedDict()
    if len(Faces) == 1:
        return []

    for i, face1 in enumerate(Faces):
        Couples = []
        if not Faces[i + 1 :]:
            continue
        for j, face2 in enumerate(Faces[i + 1 :]):
            if contiguous_face(face1, face2, tolerances):

                Couples.append(i + 1 + j)

        Connection[i] = Couples

    lista = Connection[0]
    Connection.popitem(0)

    if len(Connection) == 0:  # solo los elementos de la lista de la superficie 0
        return lista

    if not lista:  # ninguna face está conecta conecta con la superficie 0
        return lista

    for elem in Connection:
        if elem in lista:  # que la key esta en lista implica que sus dependencias estan
            lista.extend(Connection[elem])
        else:
            for elem2 in Connection[elem]:
                if elem2 in lista:  # si una de sus dependencias esta en lista lo esta la clave
                    lista.append(elem)

    return list(set(lista))


def closed_circle_edge(planes):
    angle = 0
    for p in planes:
        umin, umax = p.edge.ParameterRange
        angle += umax - umin
    return abs(angle - 2 * math.pi) < 1e-5


def most_outer_faces(cyl, faces):

    if type(cyl) is ShellGu:
        umin, umax, Vmin, Vmax = cyl.Faces[0].ParameterRange
        for f in cyl.Faces[1:]:
            umin, umax, vmin, vmax = f.ParameterRange
            Vmin = min(Vmin, vmin)
            Vmax = max(Vmax, vmax)
        cylSurf = cyl.Faces[0].Surface
        center = cylSurf.Center + cylSurf.Axis * cylSurf.Axis.dot(cyl.CenterOfMass - cylSurf.center)
        rmin = cylSurf.Axis.dot(cylSurf.value(0, Vmin) - center)
        rmax = cylSurf.Axis.dot(cylSurf.value(0, Vmax) - center)
    else:
        umin, umax, vmin, vmax = cyl.ParameterRange
        center = cyl.Surface.Center
        rmin = cyl.Surface.Axis.dot(cyl.valueAt(0, vmin) - center)
        rmax = cyl.Surface.Axis.dot(cyl.valueAt(0, vmax) - center)

    dmin = dmax = abs(rmax - rmin)
    for f in faces:
        d = cyl.Surface.Axis.dot(f.CenterOfMass - center)
        if abs(d - rmin) < dmin:
            fmin = f
            dmin = abs(d - rmin)
        if abs(d - rmax) < dmax:
            fmax = f
            dmax = abs(d - rmax)

    return (fmin, fmax)


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


def commonEdge(face1, face2, outer1_only=True, outer2_only=True):
    if type(face1) is ShellGu:
        for face in face1.Faces:
            edges = commonEdgeFace(face, face2, outer1_only, outer2_only)
            if edges is not None:
                return edges, face
        return None, None
    else:
        return commonEdgeFace(face1, face2, outer1_only, outer2_only)


def commonEdgeFace(face1, face2, outer1_only=True, outer2_only=True):
    if face1.distToShape(face2)[0] > 0:
        return None

    edges = []
    Edges1 = face1.OuterWire.Edges if outer1_only else face1.Edges
    Edges2 = face2.OuterWire.Edges if outer2_only else face2.Edges
    for e1 in Edges1:
        for e2 in Edges2:
            if e1.isSame(e2):
                edges.append(e1)
    return edges


def region_sign(s1_in, s2, outAngle=False):
    if type(s1_in) is ShellGu:
        Edges, s1 = commonEdge(s1_in, s2, outer1_only=False, outer2_only=False)
    else:
        Edges = commonEdge(s1_in, s2, outer1_only=False, outer2_only=False)
        s1 = s1_in

    e1 = Edges[0]
    p0, p1 = e1.ParameterRange
    pe1 = 0.5 * (p0 + p1)
    pos = e1.valueAt(pe1)

    if isinstance(s1.Surface, PlaneGu):
        normal1 = s1.Surface.Axis if s1.Orientation == "Forward" else -s1.Surface.Axis
    else:
        u, v = s1.Surface.face.Surface.parameter(pos)
        normal1 = s1.Surface.face.normalAt(u, v)

    if isinstance(e1.Curve, Part.Line):
        direction = e1.Curve.Direction

        if isinstance(s2.Surface, PlaneGu):
            arc = 0
            normal2 = s2.Surface.Axis if s2.Orientation == "Forward" else -s2.Surface.Axis
        else:
            umin, umax, vmin, vmax = s2.Surface.face.ParameterRange
            arc = abs(umax - umin)
            u, v = s2.Surface.face.Surface.parameter(pos)
            normal2 = s2.Surface.face.normalAt(u, v)
    else:
        arc = 0
        direction = e1.derivative1At(pe1)
        direction.normalize()

        u, v = s2.Surface.face.Surface.parameter(pos)
        normal2 = s2.Surface.face.normalAt(u, v)

    vect = direction.cross(normal1)
    if e1.Orientation == "Reversed":
        vect = -vect

    dprod = vect.dot(normal2)
    if abs(dprod) < 1e-4:
        operator = "AND" if abs(dprod) < arc else "OR"
        if outAngle:
            return operator, angle(normal1, normal2, operator)
        else:
            return operator

    else:
        operator = "OR" if dprod < 0 else "AND"
        if outAngle:
            return operator, angle(normal1, normal2, operator)
        else:
            return operator


def angle(v1, v2, operator):
    d = v1.dot(v2) / (v1.Length * v2.Length)
    a = math.acos(max(-1, min(1, d)))
    if operator == "AND":
        return math.pi - a
    else:
        return math.pi + a


def closed_cylinder(cylinder, solidFaces):
    Cylinders = [cylinder]
    for cyl in solidFaces:
        if cyl.Index == cylinder.Index:
            continue
        if cylinder.Surface.isSameSurface(cyl.Surface):
            Cylinders.append(cyl)

    if len(Cylinders) > 1:
        sameIndex = same_faces(
            Cylinders, Tolerances()
        )  # return all face connected (direct or indirectly ) to first face (cylinder)
        sameIndex.insert(0, 0)
        sameCyl = [Cylinders[i] for i in sameIndex]
        if len(sameCyl) > 1:
            cyl_shell = ShellGu(sameCyl)
            cyl_index = set(cyl_shell.Indexes)
        else:
            cyl_shell = cylinder
            cyl_index = {cylinder.Index}
    else:
        cyl_shell = cylinder
        cyl_index = {cylinder.Index}

    return cyl_shell, cyl_index, is_closed_cylinder(cyl_shell)


def planar_edges(edges):

    e0 = edges[0]
    if type(e0.Curve) is Part.BSplineCurve:
        d0 = e0.derivative1At(0)
        if d0.Length < 1e-5:
            dir0 = e0.Vertexes[1].Point - e0.Vertexes[0].Point
            dir0.normalize()
            center0 = 0.5 * (e0.Vertexes[1].Point + e0.Vertexes[0].Point)
        elif spline_2D(e0):
            dir0 = e0.derivative1At(0).cross(e0.normalAt(0))
            dir0.normalize()
            center0 = 0.5 * (e0.Vertexes[1].Point + e0.Vertexes[0].Point)
        else:
            return False
    elif isinstance(e0.Curve, (Part.Circle, Part.Ellipse, Part.Hyperbola, Part.Parabola)):
        dir0 = e0.Curve.Axis
        center0 = e0.Curve.Center
    else:  # should be a line
        dir0 = e0.Curve.Direction
        center0 = e0.Curve.Location

    if len(edges) == 1:
        return True

    for ei in edges[1:]:
        if type(ei.Curve) is Part.BSplineCurve:
            di = ei.derivative1At(0)
            if di.Length < 1e-5:
                dir = ei.Vertexes[1].Point - ei.Vertexes[0].Point
                dir.normalize()
                center = 0.5 * (ei.Vertexes[1].Point + ei.Vertexes[0].Point)
            elif spline_2D(ei):
                dir = ei.derivative1At(0).cross(ei.normalAt(0))
                dir.normalize()
                center = 0.5 * (ei.Vertexes[1].Point + ei.Vertexes[0].Point)
            else:
                return False
        elif isinstance(ei.Curve, (Part.Circle, Part.Ellipse)):
            dir = ei.Curve.Axis
            center = ei.Curve.Center
        else:  # should be a line
            dir = ei.Curve.direction
            center = ei.Curve.location

        if not is_parallel(dir0, dir, Tolerances().angle):
            return False
        if abs(dir0.dot(center - center0)) > 1e-5:
            return False

    return True


def spline_1D(edge):
    knots = edge.Curve.getKnots()
    return edge.Curve.curvature(knots[0]) < 1e-6


def spline_2D(edge):
    knots = edge.Curve.getKnots()

    if edge.Curve.curvature(knots[0]) < 1e-6:
        return False  # straight line

    d0 = edge.derivative1At(knots[0])
    if d0.Length < 1e-5:
        return False

    norm_0 = d0.cross(edge.normalAt(knots[0]))
    norm_0.normalize()

    for k in knots[1:]:
        # check if derivative orthogonal to curve normal vector
        dk = edge.derivative1At(k)
        normal_k = dk.cross(edge.normalAt(k))
        normal_k.normalize()
        if abs(normal_k.dot(norm_0)) > Tolerances().value:
            return False
    return True


def twoPimod(x):
    x = x % twoPi
    if x < 1e-5:
        return 0.0
    elif twoPi - x < 1e-5:
        return 0.0
    else:
        return x
