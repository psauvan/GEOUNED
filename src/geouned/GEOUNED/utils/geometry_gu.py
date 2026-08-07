#
#  definition of GEOUNED objects to release memory
#
#  GEOUNED SurfacesGU, SolidsGU, PlaneGU, etc.. objects are created because FreeCAD create a new object
#  each time an attribute of FreeCAD object is called. This leads to code crash with memory failure
#  when attribues are call large amount of times. Like it is in this code.

import logging
import math

from .basic_functions_part1 import is_same_value
from .basic_functions_part2 import is_same_torus
from ...geo import vector_geometry
from ...geo import (
    GCone,
    GCylinder,
    GFace,
    GPlane,
    GSolid,
    GSphere,
    GTorus,
    Gclassify_surface,
    Gmake_shell,
    pick_outer_wire,
)

logger = logging.getLogger("general_logger")

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


class SolidGu(GSolid):
    """GEOUNED Solid Class -- adds decomposition-specific torus-face-merge
    bookkeeping (TorusVParams/TorusUParams) and tolerances on top of
    GSolid, and exposes .Faces as FaceGu (native-typed) instead of GFace.
    GSolid's own .Faces/.Edges/.Vertexes/.BoundBox/.Solids/etc are built by
    super().__init__() but otherwise unused here -- SolidGu is only ever
    constructed twice in the whole pipeline (once per solid being
    decomposed), so the extra construction cost is negligible."""

    def __init__(self, solid, tolerances):
        super().__init__(solid.__native__)
        self.Faces = define_list_face_gu(solid.Faces)
        self.tolerances = tolerances
        self.TorusVParams = {}
        self.TorusUParams = {}

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
class FaceGu(GFace):
    """GEOUNED Face Class -- a thin, native-typed compatibility layer over
    GFace, used by the decomposition-side face-analysis code (SolidGu/
    ShellGu and everything downstream of them), which still consumes
    native FreeCAD types (chained .sub/.cross/.dot/.normalize/.isEqual,
    tessellate(val, reset), etc.) rather than GVector. Reuses GFace's
    classification/edge/vertex/boundbox construction instead of
    duplicating it a second time; overrides Edges/Vertexes back to native
    (FaceGu always exposed those as native, not GEdge/GVector)."""

    def __init__(self, face):
        super().__init__(face.__native__)

        # GEOUNED based atributes
        self.Index = None

        # FreeCAD based Atributes
        self.CenterOfMass = face.CenterOfMass
        self.Edges = face.Edges
        self.Vertexes = face.Vertexes
        self.OuterWire = None
        return

    def set_index(self, index):
        self.Index = index

    def set_outerWire(self):
        self.OuterWire = pick_outer_wire(self.wires())

    def parameter(self, point):
        return self.Surface.parameter(point)

    def distToShape(self, shape):
        # every caller passes a ShellGu or something wrapping a native
        # shape (FaceGu/GFace/GSolid) -- confirmed via grep, no live call
        # site ever passes a raw native shape here. The actual native
        # distance query lives in geo (GFace.distance_to) -- ShellGu
        # itself isn't a geo type, so the recursion over its Faces stays
        # here.
        if isinstance(shape, ShellGu):
            distmin = 1
            for f in shape.Faces:
                d = self.distToShape(f)
                distmin = min(distmin, d[0])
            return (distmin,)
        else:
            return (self.my_distToshape(shape),)


class ShellGu:
    def __init__(self, faces):
        self.Faces = faces
        self.__shell__ = self.makeShell()
        self.Indexes = [f.Index for f in faces]
        self.Orientation = faces[0].Orientation

    def makeShell(self):
        # self.Faces is always FaceGu (its only caller, closed_cylinder_cone,
        # always passes a SolidGu.Faces subset) -- FaceGu already IS a GFace
        # via inheritance, so no unwrap-to-native/rebuild is needed here.
        return Gmake_shell(self.Faces)


# Aux functions
def define_list_face_gu(face_list):
    """Return the list of the  corresponding Face_GU  object of a FaceList"""
    return tuple(FaceGu(face) for face in face_list)


def define_surface(face, surface=None):
    # Gclassify_surface itself returns None for a surface type GEOUNED can't
    # model (a genuine BSplineSurface, SurfaceOfRevolution/Extrusion, ...) --
    # see its docstring in geo/_freecad_impl.py. `surface`, if given, is an
    # already-classified result (e.g. GFace.__init__'s, reused by FaceGu to
    # avoid classifying the same face twice) -- only the "log if unclassifiable"
    # check runs again, not the classification itself.
    if surface is None:
        surface = Gclassify_surface(face)
    if surface is None:
        logger.info(f"bad Surface type {type(face.Surface)}")
    return surface


def other_face_edge(current_edge, current_face, Faces, outer_only=False):
    for face in Faces:
        if face.Index == current_face.Index:
            continue

        Edges = face.OuterWire.Edges if outer_only else face.Edges
        for edge in Edges:
            if current_edge.is_same(edge):
                return face
    return None
