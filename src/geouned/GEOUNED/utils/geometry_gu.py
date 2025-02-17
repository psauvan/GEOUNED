#
#  definition of GEOUNED objects to release memory
#
#  GEOUNED SurfacesGU, SolidsGU, PlaneGU, etc.. objects are created because FreeCAD create a new object
#  each time an attribute of FreeCAD object is called. This leads to code crash with memory failure
#  when attribues are call large amount of times. Like it is in this code.

import logging
import math

import FreeCAD
import Part

from .basic_functions_part1 import is_same_value
from .basic_functions_part2 import is_same_torus

logger = logging.getLogger("general_logger")


class face_index:
    def __init__(self, face, index, orientation):
        self.face = face
        self.index = index
        self.orientation = orientation


# SURFACES
class SurfacesGu(object):
    """GEOUNED surface class"""

    def __init__(self, face):
        self.face = face
        self.Surface = self.face.Surface
        self.type = str(self.Surface)

    def __str__(self):
        """str(Surface) is done for the classification of the surface.
        Surface_GU saves this information in self.type"""
        return self.type


class PlaneGu(SurfacesGu):
    """GEOUNED Plane Class"""

    def __init__(self, face, plane3Pts=False, BSpline_Plane=None):
        SurfacesGu.__init__(self, face)
        self.pointDef = plane3Pts
        if BSpline_Plane is None:
            self.Axis = face.Surface.Axis
            self.Position = face.Surface.Position
            if plane3Pts:
                self.Points = tuple(v.Point for v in face.Vertexes)
                d1 = self.Points[0] - self.Points[1]
                d2 = self.Points[0] - self.Points[2]
                d3 = self.Points[1] - self.Points[2]
                self.dim1 = max(d1.Length, d2.Length, d3.Length)
                self.dim2 = min(d1.Length, d2.Length, d3.Length)
            else:
                self.dim1 = face.ParameterRange[1] - face.ParameterRange[0]
                self.dim2 = face.ParameterRange[3] - face.ParameterRange[2]
        else:
            self.Axis = BSpline_Plane.Axis
            self.Position = BSpline_Plane.Position
            self.dim1 = 1
            self.dim2 = 1

    def isSameSurface(self, surface):
        if type(surface) is not PlaneGu:
            return False
        if abs(self.Axis.dot(surface.Axis)) < 0.99999:
            return False
        if abs(self.Axis.dot(self.Position) - surface.Axis.dot(surface.Position)) > 1e-5:
            return False
        return True

    def isParallel(self, surface):
        if type(surface) is not PlaneGu:
            return False
        return abs(self.Axis.dot(surface.Axis)) > 0.99999

    def reverse(self):
        self.Axis = -self.Axis


class CylinderGu(SurfacesGu):
    """GEOUNED Cylinder Class"""

    def __init__(self, face):
        SurfacesGu.__init__(self, face)
        self.Axis = face.Surface.Axis
        self.Radius = face.Surface.Radius
        self.Center = face.Surface.Center
        self.dimL = face.ParameterRange[3] - face.ParameterRange[2]

    def isSameSurface(self, surface):
        if type(surface) is not CylinderGu:
            return False
        if abs(self.Radius - surface.Radius) > 1e-5:
            return False
        d = self.Center - surface.Center
        if d.Length > 1e-5:
            return False
        if abs(self.Axis.dot(surface.Axis)) < 0.99999:
            return False
        return True


class ConeGu(SurfacesGu):
    """GEOUNED Cone Class"""

    def __init__(self, face):
        SurfacesGu.__init__(self, face)
        self.Axis = face.Surface.Axis
        self.Apex = face.Surface.Apex
        self.SemiAngle = face.Surface.SemiAngle
        self.dimL = face.ParameterRange[3] - face.ParameterRange[2]
        self.dimR = face.Surface.Radius
        self.Radius = face.Surface.Radius

    def isSameSurface(self, surface):
        if type(surface) is not ConeGu:
            return False
        if abs(self.SemiAngle - surface.SemiAngle) > 1e-5:
            return False
        d = self.Apex - surface.Apex
        if d.Length > 1e-5:
            return False
        if abs(self.Axis.dot(surface.Axis)) < 0.99999:
            return False
        return True


class SphereGu(SurfacesGu):
    """GEOUNED Sphere Class"""

    def __init__(self, face):
        SurfacesGu.__init__(self, face)
        self.type = self.type[0:6]
        self.Center = face.Surface.Center
        self.Radius = face.Surface.Radius

    def isSameSurface(self, surface):
        if type(surface) is not SphereGu:
            return False
        if abs(self.Radius - surface.Radius) > 1e-5:
            return False
        d = self.Center - surface.Center
        if d.Length > 1e-5:
            return False
        return True


class TorusGu(SurfacesGu):
    """GEOUNED Torus Class"""

    def __init__(self, face):
        SurfacesGu.__init__(self, face)
        self.Center = face.Surface.Center
        self.Axis = face.Surface.Axis
        self.MajorRadius = face.Surface.MajorRadius
        self.MinorRadius = face.Surface.MinorRadius

    def isSameSurface(self, surface):
        if type(surface) is not TorusGu:
            return False
        if abs(self.MajorRadius - surface.MajorRadius) > 1e-5:
            return False
        if abs(self.MinorRadius - surface.MinorRadius) > 1e-5:
            return False
        d = self.Center - surface.Center
        if d.Length > 1e-5:
            return False
        if abs(self.Axis.dot(surface.Axis)) < 0.99999:
            return False
        return True


class SolidGu:
    """GEOUNED Solid Class"""

    def __init__(self, solid, tolerances, plane3Pts=False):
        self.solid = solid
        faces = define_list_face_gu(solid.Faces, plane3Pts)
        self.Faces = faces
        self.tolerances = tolerances
        self.Solids = solid.Solids
        self.BoundBox = solid.BoundBox
        self.Edges = solid.Edges
        self.TorusVParams = {}
        self.TorusUParams = {}
        self.inverted = is_inverted(solid)

        for i, face in enumerate(self.Faces):
            face.set_index(i)

        for i, face in enumerate(self.Faces):
            face.set_outerWire(self.Faces)

        toroidIndex = []
        for i, face in enumerate(self.Faces):
            if isinstance(face.Surface, TorusGu):
                toroidIndex.append(i)

        if len(toroidIndex) != 0:
            tFaces = self.same_torus_surf(toroidIndex)
            for i, tSet in enumerate(tFaces):
                URange = self.merge_periodic_uv("U", tSet)
                VRange = self.merge_periodic_uv("V", tSet)
                for t in tSet:
                    self.TorusVParams[t] = (i, VRange)
                    self.TorusUParams[t] = (i, URange)

    def same_torus_surf(self, torusList):
        """group as a single face all the neighbour faces of the same torus"""
        sameTorusFace = []
        temp = torusList[:]
        while len(temp) > 0:
            i = temp[0]
            current = [i]
            for j in temp[1:]:
                if is_same_torus(
                    self.Faces[i].Surface,
                    self.Faces[j].Surface,
                    dtol=self.tolerances.tor_distance,
                    atol=self.tolerances.tor_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    current.append(j)
            for c in current:
                temp.remove(c)
            sameTorusFace.append(current)

        return self.separate_surfaces(sameTorusFace)

    def separate_surfaces(self, faceList):
        """group all faces in faceList forming a continuous surface"""
        sameSurfaces = []
        for tset in faceList:
            temp = tset[:]
            while len(temp) > 0:
                i = 0
                current = [temp[0]]
                removeList = [temp[0]]
                while len(temp) > 0 and i < len(current):
                    for tindex in temp:
                        if self.Faces[current[i]].distToShape(self.Faces[tindex])[0] < self.tolerances.distance:
                            if tindex not in current:
                                current.append(tindex)
                                removeList.append(tindex)
                    i += 1
                    for c in removeList:
                        temp.remove(c)
                    removeList = []

                sameSurfaces.append(current)
        return sameSurfaces

    # TODO check if this function is used as it appears to be nut used in the code
    def merge_no_periodic_uv(self, parameter, faceList):
        if parameter == "U":
            i1 = 0
            i2 = 2
        elif parameter == "V":
            i1 = 2
            i2 = 4

        v_min, v_max = self.Faces[faceList[0]].ParameterRange[i1:i2]
        for face in faceList[1:]:
            V0, V1 = self.Faces[face].ParameterRange[i1:i2]
            v_min = min(v_min, V0)
            v_max = max(v_max, V1)
        mergedParams = (False, (v_min, v_max))

        return mergedParams

    def merge_periodic_uv(self, parameter, faceList):
        two_pi = 2.0 * math.pi
        if parameter == "U":
            i1 = 0
            i2 = 2
        elif parameter == "V":
            i1 = 2
            i2 = 4

        params = []
        arcLength = 0.0
        for face in faceList:
            V0, V1 = self.Faces[face].ParameterRange[i1:i2]
            arcLength += V1 - V0
            params.append((V0, V1))

        params.sort()
        V0 = params[0][0]
        V1 = params[-1][1]
        if arcLength >= two_pi * (1.0 - self.tolerances.relativePrecision):
            mergedParams = (True, (V0, V0 + two_pi))
        else:
            if is_same_value(V0, 0.0, self.tolerances.relativePrecision) and is_same_value(
                V1, two_pi, self.tolerances.relativePrecision
            ):
                for i in range(len(params) - 1):
                    if not is_same_value(
                        params[i][1],
                        params[i + 1][0],
                        self.tolerances.relativePrecision,
                    ):
                        break
                v_min = params[i + 1][0] - two_pi
                v_max = params[i][1]
            else:
                v_min = params[0][0]
                v_max = params[-1][1]
            mergedParams = (False, (v_min, v_max))

        return mergedParams


# FACES
class FaceGu(object):
    """GEOUNED Face Class"""

    def __init__(self, face, Plane3Pts=False):
        # GEOUNED based atributes

        self.__face__ = face
        self.Index = None
        self.Surface = define_surface(face, Plane3Pts)  # Define the appropiate GU Surface of the face

        # FreeCAD based Atributes
        self.Area = face.Area
        self.CenterOfMass = face.CenterOfMass
        self.ParameterRange = face.ParameterRange
        self.Orientation = face.Orientation
        self.Edges = face.Edges
        self.Vertexes = face.Vertexes
        self.OuterWire = None
        return

    def set_index(self, index):
        self.Index = index

    def set_outerWire(self, Faces):
        self.OuterWire = set_outerWire(self.__face__.Wires, self, Faces)
        #self.OuterWire = self.__face__.OuterWire

    def tessellate(self, val, reset=False):
        res = self.__face__.tessellate(val, reset)
        return res

    def getUVNodes(self):
        return self.__face__.getUVNodes()

    def isEqual(self, face):
        return self.__face__.isEqual(face.__face__)

    def isSame(self, face):
        return self.__face__.isSame(face.__face__)

    def valueAt(self, u, v):
        return self.__face__.valueAt(u, v)

    def distToShape(self, shape):
        shape1 = self.__face__
        if isinstance(shape, Part.Shape):
            shape2 = shape
        elif isinstance(shape, ShellGu):
            distmin = 1
            for f in shape.Faces:
                d = self.distToShape(f)
                distmin = min(distmin, d[0])
            return (distmin,)
        else:
            shape2 = shape.__face__

        if shape1 is shape2:
            return (0,)
        else:
            try:
                dist2Shape = shape1.distToShape(shape2)
            except:
                dist2Shape = shape2.distToShape(shape1)
            return dist2Shape


class ShellGu:
    def __init__(self, faces):
        self.__shell__ = Part.makeShell(faces)
        self.Faces = faces
        self.Edges = []
        self.Indexes = []
        self.Area = 0
        self.CenterOfMass = FreeCAD.Vector(0, 0, 0)
        for f in faces:
            self.Edges.extend(f.Edges)
            self.Indexes.append(f.Index)
            self.Area += f.Area
            self.CenterOfMass += f.Area * f.CenterOfMass
        self.Orientation = faces[0].Orientation
        # self.set_outerWire() #produce error and no used anymore

    def distToShape(self, shape):
        distmin = 1
        for f in shape.Faces:
            d = f.distToShape(shape)
            distmin = min(distmin, d[0])
        return (distmin,)

    def set_outerWire(self):
        wires = []
        for f in self.Faces:
            wires.append(f.OuterWire)
        self.OuterWire = join_wires(wires)


# Aux functions
def define_list_face_gu(face_list, plane3Pts=False):
    """Return the list of the  corresponding Face_GU  object of a FaceList"""
    return tuple(FaceGu(face, plane3Pts) for face in face_list)


def define_surface(face, plane3Pts):

    kind_surf = type(face.Surface)
    if kind_surf is Part.Plane:
        Surf_GU = PlaneGu(face, plane3Pts)
    elif kind_surf is Part.Cylinder:
        Surf_GU = CylinderGu(face)
    elif kind_surf is Part.Cone:
        Surf_GU = ConeGu(face)
    elif kind_surf is Part.Sphere:
        Surf_GU = SphereGu(face)
    elif kind_surf is Part.Toroid:
        Surf_GU = TorusGu(face)
    elif kind_surf is Part.BSplineSurface:
        Surf_GU = BSplineGu(face)
    else:
        logger.info(f"bad Surface type {kind_surf}")
        Surf_GU = None
    return Surf_GU


def is_inverted(solid):

    face = solid.Faces[0]

    # u=(face.Surface.bounds()[0]+face.Surface.bounds()[1])/2.0 # entre 0 y 2pi si es completo
    # v=face.Surface.bounds()[0]+(face.Surface.bounds()[3]-face.Surface.bounds()[2])/3.0 # a lo largo del eje
    parameter_range = face.ParameterRange
    u = (parameter_range[1] + parameter_range[0]) / 2.0
    v = (parameter_range[3] + parameter_range[2]) / 2.0

    if isinstance(face.Surface, Part.Cylinder):
        dist1 = face.Surface.value(u, v).distanceToLine(face.Surface.Center, face.Surface.Axis)
        dist2 = (
            face.Surface.value(u, v)
            .add(face.Surface.normal(u, v).multiply(1.0e-6))
            .distanceToLine(face.Surface.Center, face.Surface.Axis)
        )
        if (dist2 - dist1) < 0.0:
            # The normal of the cylinder is going inside
            return True

    elif isinstance(face.Surface, Part.Cone):
        dist1 = face.Surface.value(u, v).distanceToLine(face.Surface.Apex, face.Surface.Axis)
        dist2 = (
            face.Surface.value(u, v)
            .add(face.Surface.normal(u, v).multiply(1.0e-6))
            .distanceToLine(face.Surface.Apex, face.Surface.Axis)
        )
        if (dist2 - dist1) < 0.0:
            # The normal of the cylinder is going inside
            return True
    # MIO
    elif isinstance(face.Surface, Part.Sphere):
        # radii = point - center
        radii = face.Surface.value(u, v).add(face.Surface.Center.multiply(-1))
        radii_b = face.Surface.value(u, v).add(face.Surface.normal(u, v).multiply(1.0e-6)).add(face.Surface.Center.multiply(-1))
        # radii_b  = radii.add( face.Surface.normal(u,v).multiply(1.0e-6) )
        if (radii_b.Length - radii.Length) < 0.0:
            # An increasing of the radii vector in the normal direction decreases the radii: oposite normal direction
            return True

    elif isinstance(face.Surface, Part.Plane):
        dist1 = face.CenterOfMass.distanceToPoint(solid.BoundBox.Center)
        dist2 = face.CenterOfMass.add(face.normalAt(u, v).multiply(1.0e-6)).distanceToPoint(solid.BoundBox.Center)
        point2 = face.CenterOfMass.add(face.normalAt(u, v).multiply(1.0e-6))
        if solid.isInside(point2, 1e-7, False):
            return True

    return False


def BSplineGu(face):

    plane = face.findPlane()
    if plane is None:
        return None
    else:
        return PlaneGu(face, BSpline_Plane=plane)


def set_outerWire(wires, face, Faces):
    if len(wires) == 1:
        return wires[0]

    for w in wires:
        if not innerWires(w, face, Faces):
            return w
    print("outer wire not found")


def innerWires(wire, face, Faces):
    for i, x0 in enumerate(wire.OrderedVertexes):
        for x1 in wire.OrderedVertexes[i + 1 :]:
            dx = x0.Point - x1.Point
            if dx.Length < 1e-5:
                return False

    if len(Faces) == 0:
        return True
    for edge in wire.Edges:
        adjface = other_face_edge(edge, face, Faces)
        pos = edge.Vertexes[0].Point
        u, v = face.__face__.Surface.parameter(pos)
        normal = face.__face__.normalAt(u, v)

        pe = edge.Curve.parameter(pos)

        if type(edge.Curve) is Part.Line:
            direction = edge.Curve.Direction
        else:
            direction = edge.derivative1At(pe)
        if edge.Orientation == "Reversed":
            direction = -direction
        direction.normalize()

        vect = direction.cross(normal)
        u, v = adjface.__face__.Surface.parameter(pos)
        vect2 = adjface.__face__.normalAt(u, v)
        scalar = vect.dot(vect2)
        if abs(scalar) < 1e-5:
            continue
        elif scalar > 0:
            return False
    else:
        return True


def other_face_edge(current_edge, current_face, Faces, outer_only=False):
    for face in Faces:
        if face.Index == current_face.Index:
            continue

        Edges = face.OuterWire.Edges if outer_only else face.Edges
        for edge in Edges:
            if current_edge.isSame(edge):
                return face


def join_wires(wireList):
    if len(wireList) == 0:
        return None
    elif len(wireList) == 1:
        return wireList[0]
    elif len(wireList) == 2:
        merged = merge_two_wires(*wireList)
        if not merged:
            return wireList[0]
        else:
            return merged
    else:
        noneList = []
        joined = False
        w1 = wireList[0]
        for w2 in wireList[1:]:
            joined_W = merge_two_wires(w1, w2)
            if joined_W is None:
                noneList.append(w2)
            else:
                joined = True
                w1 = joined_W

        if joined and len(noneList) > 0:
            noneList.insert(0, w1)
            return join_wires(noneList)
        else:
            return w1


def merge_two_wires(wire1, wire2):
    if wire1.distToShape(wire2)[0] > 1e-5:
        return None
    cmon_vertexes = common_vertexes(wire1, wire2)
    if len(cmon_vertexes[0]) == 0:
        return None

    parts1 = cut_wires(cmon_vertexes[0], wire1.OrderedEdges)
    parts2 = cut_wires(cmon_vertexes[1], wire2.OrderedEdges)
    joined = []
    addSame = False
    i = 0
    for w1, w2 in zip(parts1, parts2):
        if same_wire(w1, w2):
            if not addSame and i != 0:
                joined.extend(w1)
                addSame = True
        else:
            joined.extend(w1 + w2)
        i += 1
    return Part.Wire(joined)


def common_vertexes(w1, w2):
    common1 = []
    common2 = []
    for v1 in w1.OrderedVertexes:
        for v2 in w2.Vertexes:
            d = v1.Point - v2.Point
            if d.Length < 1e-6:
                common1.append(v1)
                common2.append(v2)
    return (common1, common2)


def cut_wires(cmon_vertexes, wire):
    vertex_pos, increase = vertex_edge_index(cmon_vertexes, wire)
    parts = []
    nval = len(vertex_pos)
    if increase:
        for i0 in range(len(vertex_pos)):
            i1 = (i0 + 1) % nval
            e0 = max(vertex_pos[i0])
            e1 = min(vertex_pos[i1])
            if e0 <= e1:
                if e1 + 1 == len(wire):
                    parts.append(wire[e0:])
                else:
                    parts.append(wire[e0 : e1 + 1])
            else:
                parts.append(wire[e0:] + wire[0 : e1 + 1])
    else:
        for i0 in range(len(vertex_pos)):
            i1 = (i0 + 1) % nval
            e0 = max(vertex_pos[i1])
            e1 = min(vertex_pos[i0])
            if e0 <= e1:
                if e1 + 1 == len(wire):
                    parts.append(list(reversed(wire[e0:])))
                else:
                    parts.append(list(reversed(wire[e0 : e1 + 1])))
            else:
                parts.append(list(reversed(wire[0 : e1 + 1])) + list(reversed(wire[e0:])))
    return parts


def vertex_edge_index(cmon_vertexes, wireEdges):
    position = []
    for v in cmon_vertexes:
        ind = []
        for i, e in enumerate(wireEdges):
            if v.isSame(e.Vertexes[0]) or v.isSame(e.Vertexes[1]):
                ind.append(i)
                if len(ind) == 2:
                    break
        position.append(ind)

    seq = [x[0] for x in position]
    vmin, vmax = min(seq), max(seq)
    if seq[0] > seq[1]:
        if seq[0] == vmax and seq[1] == vmin:
            increase = True
        else:
            increase = False
    else:
        if seq[0] == vmin and seq[1] == vmax:
            increase = False
        else:
            increase = True

    for x in position:
        if x[0] == 0:
            if x[1] == len(wireEdges) - 1:
                x[1] = -1
            break
    return position, increase


def same_wire(w1, w2):
    e1 = w1[0]
    e2 = w2[0]
    p1min, p1max = e1.ParameterRange
    p1 = 0.5 * (p1min + p1max)
    pos1 = e1.valueAt(p1)
    v1 = Part.Vertex(pos1)
    if e2.distToShape(v1)[0] < 1e-5:
        return True
    else:
        p2min, p2max = e2.ParameterRange
        p2 = 0.5 * (p2min + p2max)
        pos2 = e2.valueAt(p2)
        v2 = Part.Vertex(pos2)
        return e1.distToShape(v2)[0] < 1e-5
