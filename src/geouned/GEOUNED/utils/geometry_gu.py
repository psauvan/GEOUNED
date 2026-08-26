#
#  definition of GEOUNED objects to release memory
#
#  GEOUNED SurfacesGU, SolidsGU, PlaneGU, etc.. objects are created because FreeCAD create a new object
#  each time an attribute of FreeCAD object is called. This leads to code crash with memory failure
#  when attribues are call large amount of times. Like it is in this code.

import logging
import math

from .data_constants import twoPi
from .basic_functions_part1 import is_same_value, twoPimod
from .basic_functions_part2 import is_same_torus
from .data_classes import Tolerances
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
                    check_a_sign=True,  # never merge a self-intersecting torus's two distinct sheets into one face group
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
                # params is sorted by V0 ascending, so params[0][0] is always
                # the true minimum V0 -- but sorting by V0 does not imply
                # sorted V1, so params[-1][1] is only the true maximum V1
                # when the pieces form a simple, non-nested chain. When one
                # piece's own range is fully nested inside another's (e.g. a
                # tiny residual sliver piece sitting within a larger piece's
                # own V-span), params[-1][1] can under-report the real
                # merged extent -- take the max explicitly instead.
                v_min = params[0][0]
                v_max = max(v1 for _, v1 in params)
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
        if isinstance(shape, ShellFaceGu):
            distmin = float("inf")
            for f in shape.Faces:
                d = self.distToShape(f)
                distmin = min(distmin, d[0])
            return (distmin,)
        else:
            return (self.my_distToshape(shape),)


class ShellFaceGu:
    def __init__(self, faces):
        self.Faces = faces
        if not self._check_same_surface_type():
            logger.info("ShellFaceGu: faces are not of the same type")
            raise RuntimeError("ShellFaceGu: faces are not of the same type")
        self.Surface = Gclassify_surface(faces[0].__native__)  # all faces are the same type, so just classify the first one

        self.__shell__ = self.makeShell()
        self.Indexes = {f.Index for f in faces}
        self.Orientation = faces[0].Orientation
        self.CenterOfMass = self._get_center_of_mass()
        self.U_parameter_range = self._U_parameter_faces()

    def _U_parameter_faces(self):
        AngleRange = 0.0
        Uval, UValmin, UValmax = [], [], []
        for f in self.Faces:
            Range = f.ParameterRange
            AngleRange = AngleRange + abs(Range[1] - Range[0])
            Uval.append(Range[0:2])
            UValmin.append(Range[0])
            UValmax.append(Range[1])

        if twoPimod(AngleRange) == 0:
            return 0, twoPi, 0, 0

        Umin, Umax = sort_range(Uval)
        ifacemin = UValmin.index(Umin)
        ifacemax = UValmax.index(Umax)
        return Umin, Umax, ifacemin, ifacemax

    def _check_same_surface_type(self):
        if len(self.Faces) == 0:
            return False
        first_type = type(self.Faces[0].Surface)
        for face in self.Faces[1:]:
            if type(face.Surface) != first_type:
                return False
        return True

    def makeShell(self):
        # self.Faces is always FaceGu (its only caller, closed_cylinder_cone,
        # always passes a SolidGu.Faces subset) -- FaceGu already IS a GFace
        # via inheritance, so no unwrap-to-native/rebuild is needed here.
        return Gmake_shell(self.Faces)

    def _get_center_of_mass(self):
        # every caller passes a ShellGu or something wrapping a native
        # shape (FaceGu/GFace/GSolid) -- confirmed via grep, no live call
        # site ever passes a raw native shape here. The actual native
        # distance query lives in geo (GFace.distance_to) -- ShellGu
        # itself isn't a geo type, so the recursion over its Faces stays
        # here.
        com = vector_geometry.GVector(0, 0, 0)
        area = 0.0
        for f in self.Faces:
            area += f.Area
            com += f.Area * f.CenterOfMass
        return com / (area * len(self.Faces))


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


def other_face_edge(
    current_edge, current_face, Faces, outer_only=False, skip_slivers=False, _min_area=None, _min_face_width=None, _visited=None
):
    # skip_slivers=False preserves the original behavior for every existing
    # caller: returns just the found face. A caller that's walking adjacency
    # to find a real neighboring feature (get_adjacent_cylplane,
    # get_adjacent_cylknesurfFace) can pass skip_slivers=True to treat a
    # residual sliver face (area below tolerances.min_area, OR
    # CharacteristicWidth below tolerances.min_face_width -- a degenerate
    # near-zero-area or large-area-but-narrow patch left over from a boolean
    # cut that grazed tangentially instead of terminating cleanly) as
    # transparent: instead of stopping there, keep walking across the
    # sliver's own other edges to find the real face beyond it. Both checks
    # are needed: area alone misses a narrow-but-large sliver (the original
    # motivation for CharacteristicWidth, see SCDR_90_piece2.stp), and width
    # alone would need the same guard the other direction -- confirmed live,
    # 2026-08-24, SCDR_90_hollow.stp: a real cone-to-MultiPlane match was
    # anchored on a face with Area=0.0151 (just above the 0.01 default
    # min_area, so not skipped) but CharacteristicWidth=0.0778 (below the
    # 0.1 default min_face_width) -- a genuine sliver by width, missed by
    # area alone.
    #
    # In skip_slivers mode the return shape changes to a 3-tuple
    # (touching_edge, near_face, far_face): far_face is the real face found;
    # touching_edge is the edge where far_face actually borders whatever's
    # immediately next to it (near_face) -- which is current_face itself
    # when found directly (touching_edge is then just current_edge, and
    # near_face/touching_edge together are identical to the pre-skip_slivers
    # behavior), or the sliver's own far edge/the sliver itself when found
    # by walking through one or more slivers. A caller that only cares about
    # far_face (get_adjacent_cylknesurfFace's plain face lists, or
    # get_adjacent_cylplane's cornerPlanes=False branch) can just take the
    # 3rd element; one that needs to evaluate something (e.g.
    # material_direction) exactly at the real boundary -- rather than at
    # current_face's own, possibly non-touching, edge -- needs all three.
    for face in Faces:
        if face.Index == current_face.Index:
            continue

        Edges = face.OuterWire.Edges if outer_only else face.Edges
        for edge in Edges:
            if current_edge.is_same(edge):
                if not skip_slivers:
                    return face
                area_threshold = _min_area if _min_area is not None else Tolerances().min_area
                width_threshold = _min_face_width if _min_face_width is not None else Tolerances().min_face_width
                width = getattr(face, "CharacteristicWidth", float("inf"))
                if face.Area >= area_threshold and width >= width_threshold:
                    return current_edge, current_face, face
                visited = set(_visited) if _visited else set()
                visited.add(current_face.Index)
                if face.Index in visited:
                    return None  # already visited -- avoid a sliver cycle
                visited.add(face.Index)
                for e2 in face.OuterWire.Edges if outer_only else face.Edges:
                    if e2.is_same(current_edge):
                        continue
                    found = other_face_edge(e2, face, Faces, outer_only, skip_slivers, area_threshold, width_threshold, visited)
                    if found is not None:
                        return found
                return None
    return None


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
