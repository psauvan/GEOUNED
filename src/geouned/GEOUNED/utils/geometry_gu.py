#
#  definition of GEOUNED objects to release memory
#
#  GEOUNED SurfacesGU, SolidsGU, PlaneGU, etc.. objects are created because FreeCAD create a new object
#  each time an attribute of FreeCAD object is called. This leads to code crash with memory failure
#  when attribues are call large amount of times. Like it is in this code.

import logging
import math

import Part

from .basic_functions_part1 import is_same_value
from .basic_functions_part2 import is_same_torus
from ..utils.data_constants import twoPi
from ...geometry_backend import vector_geometry
from ...geometry_backend.vector_geometry import to_gvector
from ...geometry_backend.freecad_backend import FreeCADBackend
from ...geometry_backend.geometry_backend_interface import GPlane, GCylinder, GCone, GSphere, GTorus, GFace

logger = logging.getLogger("general_logger")

_backend = FreeCADBackend()

_SAME_SURFACE_PREDICATE = {
    GPlane: vector_geometry.is_same_plane_surface,
    GCylinder: vector_geometry.is_same_cylinder_surface,
    GCone: vector_geometry.is_same_cone_surface,
    GSphere: vector_geometry.is_same_sphere_surface,
    GTorus: vector_geometry.is_same_torus_surface,
}


def is_same_surface(surface_1, surface_2):
    """Dispatches to the neutral-type predicate for the 5 analytic surface types."""
    if type(surface_1) is not type(surface_2):
        return False
    return _SAME_SURFACE_PREDICATE[type(surface_1)](surface_1, surface_2)


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


class SolidGu:
    """GEOUNED Solid Class"""

    def __init__(self, solid, tolerances):
        self.solid = solid
        faces = define_list_face_gu(solid.Faces)
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
            face.set_outerWire()

        toroidIndex = []
        for i, face in enumerate(self.Faces):
            if isinstance(face.Surface, GTorus):
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

    def __init__(self, face):
        # GEOUNED based atributes

        self.__face__ = face
        self.Index = None
        self.Surface = define_surface(face)  # Define the appropiate GU Surface of the face

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

    def set_outerWire(self):
        self.OuterWire = set_outerWire(self.__face__.Wires, self)
        # self.OuterWire = self.__face__.OuterWire

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

    def tangentAt(self, u, v):
        return self.__face__.tangentAt(u, v)

    def parameter(self, point):
        return self.__face__.Surface.parameter(point)

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
            return shape1.distToShape(shape2)

        if shape1 is shape2:
            return (0,)
        else:
            Boxinter = shape1.BoundBox.intersected(shape2.BoundBox)
            intersect = Boxinter.XLength > -1e-6 and Boxinter.YLength > -1e-6 and Boxinter.ZLength > -1e-6
            if intersect:
                try:
                    # dist2Shape = shape1.distToShape(shape2)
                    inter = shape1.common(shape2)
                except:
                    # dist2Shape = shape2.distToShape(shape1)
                    inter = shape2.common(shape1)

                if abs(inter.Volume) > 1e-8 or len(inter.Solids) > 0 or len(inter.Faces) > 0 or len(inter.Edges) > 0:
                    dist2Shape = (0.0,)
                else:
                    same = False
                    for e1 in shape1.Edges:
                        if same:
                            break
                        for e2 in shape2.Edges:
                            if e1.isSame(e2):
                                dist2Shape = (0,)
                                same = True
                                break
                    if not same:
                        dist2Shape = (1.0,)
            else:
                c1 = shape1.BoundBox.Center
                c2 = shape2.BoundBox.Center
                d = c2 - c1
                dist2Shape = (d.Length,)
            #            dts = shape1.distToShape(shape2)[0]
            #            if (dist2Shape[0] == 0 and dts > 1e-8) or (dts < 1e-8 and dist2Shape[0] > 0 ):
            #                print ('vamos a ver')
            return dist2Shape


class ShellGu:
    def __init__(self, faces):
        self.Faces = faces
        self.__shell__ = self.makeShell()
        self.Indexes = [f.Index for f in faces]
        self.Orientation = faces[0].Orientation
        # self.set_outerWire() #produce error and no used anymore

    def makeShell(self):
        if type(self.Faces[0]) is FaceGu:
            native_faces = [f.__face__ for f in self.Faces]
        else:
            native_faces = self.Faces
        # Surface/Edges/OuterWire deliberately left unclassified: make_shell
        # only reads .native, and eagerly classifying here (_build_gface)
        # would force edge curve-classification on every boundary edge,
        # which raises for curve types the backend doesn't model (e.g. a
        # trimmed conic section's Part.Hyperbola) -- topology-only shell
        # construction has no reason to require that.
        gfaces = [
            GFace(
                native=nf,
                backend=_backend,
                Surface=None,
                Edges=(),
                OuterWire=None,
                ParameterRange=nf.ParameterRange,
                Orientation=nf.Orientation,
            )
            for nf in native_faces
        ]
        return _backend.make_shell(gfaces)

    def set_outerWire(self):
        wires = []
        for f in self.Faces:
            wires.append(f.OuterWire)
        self.OuterWire = join_wires(wires)


# Aux functions
def define_list_face_gu(face_list):
    """Return the list of the  corresponding Face_GU  object of a FaceList"""
    return tuple(FaceGu(face) for face in face_list)


_NATIVE_SURFACE_KINDS = (Part.Plane, Part.Cylinder, Part.Cone, Part.Sphere, Part.Toroid, Part.BSplineSurface)


def define_surface(face):
    # Part.BSplineSurface is included: transport codes don't support BSpline
    # surfaces at all, the only acceptable case is one that's geometrically
    # just a mislabeled plane, which is what the backend raises on if it
    # isn't -- see FreeCADBackend._classify_native_surface's docstring.
    kind_surf = type(face.Surface)
    if kind_surf in _NATIVE_SURFACE_KINDS:
        return _backend._classify_native_surface(face)
    logger.info(f"bad Surface type {kind_surf}")
    return None


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


def set_outerWire(wires, face):
    if len(wires) == 1:
        return wires[0]

    dist = 0
    outWire = None
    for w in wires:
        ext = wire_extension(w)
        if ext > dist:
            outWire = w
            dist = ext
    return outWire


def wire_extension(wire):
    center = wire.CenterOfMass
    dist = 0
    for x in wire.OrderedVertexes:
        dist += (x.Point - center).Length
    return dist / len(wire.OrderedVertexes)


def innerWires(wire, face):
    for i, x0 in enumerate(wire.OrderedVertexes):
        for x1 in wire.OrderedVertexes[i + 1 :]:
            dx = x0.Point - x1.Point
            if dx.Length < 1e-5:
                return False

    positions = []
    vect = []

    length = 0
    u_sum = 0
    v_sum = 0
    umin, umax, vmin, vmax = face.__face__.ParameterRange
    for edge in wire.Edges:
        pmin, pmax = edge.ParameterRange
        pe = 0.5 * (pmin + pmax)
        pos = edge.valueAt(pe)
        u, v = face.parameter(pos)
        if u < umin:
            u += twoPi
        elif u > umax:
            u -= twoPi
        if v < vmin:
            v += twoPi
        elif v > vmax:
            v -= twoPi

        normal = face.__face__.Surface.normal(u, v)
        u_sum += u * edge.Length
        v_sum += v * edge.Length
        length += edge.Length

        if type(edge.Curve) is Part.Line:
            direction = edge.Curve.Direction
        else:
            direction = edge.Curve.tangent(pe)[0]

        direction.normalize()
        if edge.Orientation == "Forward":
            direction = -direction

        vect.append(direction.cross(normal))
        positions.append(pos)

    if len(wire.Edges) == 1:
        center = wire.CenterOfMass
    else:
        umean = u_sum / length
        vmean = v_sum / length
        center = face.__face__.Surface.value(umean, vmean)

    ssum = 0
    i = 0
    for v, p in zip(vect, positions):
        dir = p - center
        dir.normalize()
        i += 1
        ssum += v.dot(dir)
    return ssum < 0


def innerWires_org(wire, face, Faces):
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
        u, v = face.parameter(pos)
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
        u, v = adjface.parameter(pos)
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
    return None


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


def line_projection(p1, v1, p2, v2):
    """return the point of the projection of the line with point p2 and axis v2
    on line (p1,v1)"""

    p1 = to_gvector(p1)
    p2 = to_gvector(p2)
    x = to_gvector(v1).normalized()
    y = to_gvector(v2).normalized()

    alpha = p1.dot(x)
    beta = p2.dot(x)
    gamma = p1.dot(y)
    delta = p2.dot(y)
    c = x.dot(y)
    if abs(c) > 1 - 1e-6:
        return None  # v1 and v2 parallel
    else:
        xm = (-alpha + beta + c * (gamma - delta)) / (1 - c * c)
        return p1 + xm * x
