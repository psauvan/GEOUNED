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
from ...geo import vector_geometry, surface_geometry
from ...geo import (
    GCone,
    GCylinder,
    GEllipticCylinder,
    GFace,
    GPlane,
    GSolid,
    GSphere,
    GTorus,
    Gclassify_surface,
    Gmake_shell,
    pick_outer_wire,
)
from ...geo.constants import PARAM_ANGLE_TOL
from ...geo.constants import NUMERIC_TOL

logger = logging.getLogger("general_logger")


def faces_touch(face1, face2) -> bool:
    """True if `face1` and `face2` are within `NUMERIC_TOL` of each other (touching or overlapping) -- faces of the
    SAME solid, so the intrinsic constant applies (see `geo.constants.NUMERIC_TOL`'s own docstring). Shared by
    `SolidGu.separate_surfaces` (grouping same-torus face fragments into connected pieces) and
    `meta_surfaces_utils.commonEdgeFace` (the same "are these two faces even close" pre-filter before it looks for a
    shared boundary edge) -- both already went through the identical `FaceGu.distToShape` -> `GFace.my_distToshape`
    chain before this was factored out, so this is a pure deduplication, not a behaviour change."""
    return face1.distToShape(face2)[0] < NUMERIC_TOL

_SAME_SURFACE_PREDICATE = {
    GPlane: surface_geometry.is_same_plane_surface,
    GCylinder: surface_geometry.is_same_cylinder_surface,
    GCone: surface_geometry.is_same_cone_surface,
    GSphere: surface_geometry.is_same_sphere_surface,
    GTorus: surface_geometry.is_same_torus_surface,
    GEllipticCylinder: surface_geometry.is_same_elliptic_cylinder_surface,
}


def is_same_surface(surface_1, surface_2, tolerances):
    """Dispatches to the neutral-type predicate for the 5 analytic surface types."""
    if type(surface_1) is not type(surface_2):
        return False
    return _SAME_SURFACE_PREDICATE[type(surface_1)](surface_1, surface_2, tolerances)


def _reprojected_range(face, other, value_index, p0, p1):
    """Re-express `face`'s own raw (p0, p1) range (along the parameter
    selected by `value_index`, 0=U/1=V) in `other`'s own native frame, or
    `None` if `face` has fewer than 2 vertices with distinguishable own
    values (degenerate, should not happen for a real face), or if
    `face`/`other` don't expose real vertex/surface data at all (a
    lightweight test double standing in for a face already in a common
    frame, needing no reprojection)."""
    try:
        return _reprojected_range_unchecked(face, other, value_index, p0, p1)
    except AttributeError:
        return None


def _reprojected_range_unchecked(face, other, value_index, p0, p1):
    """The real implementation of `_reprojected_range`, assuming `face`/
    `other` both expose `.Vertexes` and `.Surface.parameter(...)`.

    `face` and `other` are two fragments of the exact SAME physical,
    already-identified analytic surface (same a_sign/sheet, confirmed by
    the caller via `is_same_torus_surface(..., check_a_sign=True)`), so
    ANY of `face`'s own real vertices lies exactly on `other`'s own
    surface equation too -- no shared edge is needed to relate the two.

    Two faces of one physical torus can carry ANTIPARALLEL native axes
    (two independently-built native surface instances with opposite-sign
    frames), not merely a different U/V origin -- confirmed live,
    2026-10-28, `Mixed/base_all8_to_cylinder.stp`. This makes the map
    from `face`'s own parameter to `other`'s own parameter a REFLECTION
    (`other_val = C - own_val`) rather than a plain SHIFT (`other_val =
    own_val + C`) -- confirmed live on the same fixture: a single-point
    shift happened to give the right answer for one face pair's V range
    by coincidence (one face's own tiny residual arc nested inside the
    other's), but silently under-reported the true combined U range for
    the exact same pair (missing ~1.5 rad of real extent) because the
    true U map is a reflection, not a shift.

    Determined empirically from the 2 of `face`'s own vertices whose own
    values (along `value_index`) are farthest apart (the best-
    conditioned pair, away from any accidental near-duplicate), never
    assumed from the axes alone and never from an arbitrary INTERIOR
    point: for a genuinely self-intersecting (degenerate, MinorRadius >
    MajorRadius) torus, an interior point can satisfy the implicit
    surface equation at more than one (u, v), so native point inversion
    can silently land on the wrong, only-algebraically-equivalent branch;
    a real VERTEX has no such ambiguity (it is where the face's own
    boundary curves meet, not a point picked by this function)."""
    vertices = list(face.Vertexes)
    if len(vertices) < 2:
        return None
    own_values = [face.Surface.parameter(v)[value_index] for v in vertices]
    i_min = min(range(len(vertices)), key=lambda i: own_values[i])
    i_max = max(range(len(vertices)), key=lambda i: own_values[i])
    if i_min == i_max:
        return None
    own_a, own_b = own_values[i_min], own_values[i_max]
    ref_a = other.Surface.parameter(vertices[i_min])[value_index]
    ref_b = other.Surface.parameter(vertices[i_max])[value_index]

    def unwrapped_gap(x, y):
        d = x - y
        two_pi = 2.0 * math.pi
        while d > math.pi:
            d -= two_pi
        while d < -math.pi:
            d += two_pi
        return abs(d)

    shift_a, shift_b = ref_a - own_a, ref_b - own_b
    reflect_a, reflect_b = ref_a + own_a, ref_b + own_b
    if unwrapped_gap(shift_a, shift_b) <= unwrapped_gap(reflect_a, reflect_b):
        shift = shift_a
        while shift > math.pi:
            shift -= 2.0 * math.pi
        while shift < -math.pi:
            shift += 2.0 * math.pi
        return p0 + shift, p1 + shift
    else:
        const = reflect_a
        while const > 2.0 * math.pi + math.pi:
            const -= 2.0 * math.pi
        while const < -math.pi:
            const += 2.0 * math.pi
        # a reflection reverses order: own's own min maps to other's own max
        return const - p1, const - p0


def merge_periodic_uv(parameter, faces):
    """Merged parameter range, along "U" or "V", of `faces` (pieces of ONE periodic surface).

    Returns `(closed, (v_min, v_max))`: `closed` is True when the pieces add up to a full turn (2*pi), in which case
    the range is `(V0, V0 + 2*pi)`. "Adds up to a full turn" and "starts at 0 / ends at 2*pi / touches its neighbour"
    are decided with NUMERIC_TOL: the parameters come from the same solid's own faces, so they carry the same numbers.

    `faces[0]` itself always keeps its own raw ParameterRange untouched
    (no native round-trip at all). Every OTHER face's own range is
    re-expressed in `faces[0]`'s own native frame via `_reprojected_range`
    (using `face`'s own real vertices, reprojected through `faces[0]`'s
    own surface equation -- never through an arbitrary interior point;
    see that function's own docstring for why, and for the shift-vs-
    reflection distinction it resolves). A face with fewer than 2
    vertices with distinguishable own values (should not happen for a
    real face) keeps its own raw range untouched, same as `faces[0]` --
    silently trusting raw values here is safer than guessing a transform
    from an unreliable reprojection."""
    two_pi = 2.0 * math.pi
    if parameter == "U":
        own_i1, own_i2 = 0, 2
    elif parameter == "V":
        own_i1, own_i2 = 2, 4
    else:
        raise ValueError(f"parameter must be 'U' or 'V', not {parameter!r}")
    value_index = 0 if parameter == "U" else 1

    reference_face = faces[0]
    params = []
    arcLength = 0.0
    ref_V0 = None
    for i, face in enumerate(faces):
        p0, p1 = face.ParameterRange[own_i1:own_i2]
        if i == 0:
            V0, V1 = p0, p1
        else:
            reprojected = _reprojected_range(face, reference_face, value_index, p0, p1)
            V0, V1 = reprojected if reprojected is not None else (p0, p1)
            if V1 < V0:
                V0, V1 = V1, V0
        # Even a correctly-anchored (or, for i==0, deliberately untouched)
        # pair can land on a DIFFERENT, only periodically-equivalent
        # representative of the SAME real arc as an earlier pair --
        # confirmed live, 2026-10-28, the real fixture this function was
        # rewritten for: face4's own raw range and face5's own shifted
        # range turned out to be the IDENTICAL real arc, just one whole
        # period apart (6.226..6.283 vs -0.057..~0) -- sorting and
        # min/max-ing them as-is reads that as two DISTANT arcs needing
        # to span the WHOLE gap between them (a bogus ~6.34 rad result)
        # instead of recognizing they already overlap almost entirely.
        # Unwrapping every pair's own V0 to within half a period of the
        # very first pair's own V0 keeps the whole list in one mutually
        # consistent frame, so two equivalent-but-offset pairs collapse
        # back onto each other before the sort/min/max below ever runs.
        if ref_V0 is None:
            ref_V0 = V0
        else:
            while V0 - ref_V0 > math.pi:
                V0 -= two_pi
                V1 -= two_pi
            while V0 - ref_V0 < -math.pi:
                V0 += two_pi
                V1 += two_pi
        arcLength += V1 - V0
        params.append((V0, V1))

    params.sort()
    V0 = params[0][0]
    V1 = params[-1][1]
    if arcLength >= two_pi * (1.0 - NUMERIC_TOL):
        mergedParams = (True, (V0, V0 + two_pi))
    else:
        if is_same_value(V0, 0.0, NUMERIC_TOL) and is_same_value(V1, two_pi, NUMERIC_TOL):
            for i in range(len(params) - 1):
                if not is_same_value(params[i][1], params[i + 1][0], NUMERIC_TOL):
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
                if surface_geometry.is_same_torus_surface(
                    self.Faces[i].Surface,
                    self.Faces[j].Surface,
                    self.tolerances,
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
                        if faces_touch(self.Faces[current[i]], self.Faces[tindex]):
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
        return merge_periodic_uv(parameter, [self.Faces[face] for face in faceList])


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
        if isinstance(self.Surface, GPlane):
            self.U_parameter_range = (0, 0, 0, 0)
        else:
            self.U_parameter_range = self._U_parameter_faces()

    def _U_parameter_faces(self):
        # Each face carries its own native surface parametrisation, and
        # OCCT can give two faces of the SAME analytic cylinder/cone/torus
        # different U origins (different XDirection on the ax3). Collecting
        # each face's raw ParameterRange[0:2] then mixes frames, so ranges
        # that are physically contiguous look disjoint and arc_extent
        # fails (RoundCorners/rrc23.stp). Re-express every face's U
        # interval in ONE common frame -- self.Surface, classified from
        # Faces[0] -- by projecting a real boundary point back through
        # self.Surface.parameter().
        #
        # Reproject via _reprojected_range (shared with merge_periodic_uv's
        # own, already-validated shift-vs-reflection logic) instead of a
        # separate, ad-hoc "reproject u0/u1, unwrap u1 to within half a
        # period of u0" scheme -- that scheme (tried first, reverted
        # 2026-10-30) assumed a face's own real angular width never
        # exceeds half a turn, which is false in general: confirmed live,
        # Hollow_plates/placa2.stp, a real cone face spanning ~249
        # degrees (> pi) got its own reprojected u1 silently snapped to
        # the WRONG periodic branch (the "short way around" from u0,
        # ~110 degrees), truncating its true extent and making arc_extent
        # reject a genuinely contiguous pair as a "gap". _reprojected_range
        # instead determines the SHIFT-vs-REFLECTION mapping from 2 of the
        # face's own real vertices (never from u0/u1 directly), then
        # applies it to the exact, known (u0, u1) -- the true width is
        # never assumed to be small.
        #
        # self.Faces[0] itself is the EXCEPTION: self.Surface was
        # classified FROM it, so its own raw (u0, u1) is already
        # expressed in that exact frame by construction -- reprojecting
        # it anyway round-trips it through native point inversion for no
        # reason, and that round-trip can land on a different, only
        # periodically-equivalent representative right at a periodic
        # boundary (confirmed live in the sibling merge_periodic_uv fix,
        # same file: a raw 0.0 came back as 2*pi), silently corrupting
        # the one face this function must get exactly right.
        Uval = []
        for i, f in enumerate(self.Faces):
            u0, u1, v0, v1 = f.ParameterRange
            if i == 0:
                Uval.append((u0, u1))
                continue
            reprojected = _reprojected_range(f, self.Faces[0], 0, u0, u1)
            cu0, cu1 = reprojected if reprojected is not None else (u0, u1)
            Uval.append((min(cu0, cu1), max(cu0, cu1)))

        Umin, ifacemin, Umax, ifacemax = vector_geometry.arc_extent(Uval)

        if abs(Umin - Umax) < PARAM_ANGLE_TOL:
            return 0, twoPi, 0, 0
        else:
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
    current_edge, current_face, Faces, outer_only=False, skip_slivers=False, *, tolerances=None, _visited=None
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
                if tolerances is None:
                    raise ValueError("other_face_edge(skip_slivers=True) needs the run's tolerances")
                area_threshold = tolerances.min_area
                width_threshold = tolerances.min_face_width
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
                    found = other_face_edge(e2, face, Faces, outer_only, skip_slivers, tolerances=tolerances, _visited=visited)
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
    elif len(workRange) < len(Urange) - 1:
        # made real progress this pass (fewer ranges remain than entered,
        # i.e. join_range succeeded at least once) -- safe to keep
        # recursing, a later pass may converge further.
        workRange.append(current)
        sorted = sort_range(workRange)
        return sorted
    else:
        # No progress at all: none of the >= 2 remaining ranges could be
        # joined to `current`, even once. This is a genuinely disconnected
        # group -- e.g. two pieces that only touch modulo 2*pi (join_range
        # has no wraparound awareness) plus a third piece with a real gap
        # to both (confirmed live on Solidos/test_models/Mixed/
        # SCDR_90_hollow.stp, where 3 real face fragments of one cone are
        # topologically connected -- same_faces() correctly groups them --
        # but their own U-ranges can't be linearly ordered). Recursing
        # further would never shrink `workRange` (every future pass sees
        # the identical, unjoinable set), so this used to be an unbounded
        # recursion (RecursionError) with no caller ever able to guard
        # against it -- a class of face grouping that only became reachable
        # once ShellFaceGu started computing U_parameter_range eagerly for
        # every merge_same_surface_faces() result (Can/TCone/RoundCorner/
        # RevCC alike), not just the one narrow RevCC-chain call site that
        # used to be sort_range's only caller. Fold every remaining range
        # into `current` via the already-trusted adjust_range fallback
        # (never returns None, so this is guaranteed to terminate) instead
        # of inventing new merge semantics -- the result is a best-effort
        # bounding range, not a claim that this group is truly contiguous.
        for r in workRange:
            current = adjust_range(current, r)
        return current


def join_range(U0, U1):
    if (U0[0] - U1[0] < PARAM_ANGLE_TOL) and (-PARAM_ANGLE_TOL < U0[1] - U1[0]):
        if U1[1] > U0[1]:
            return (U0[0], U1[1])
        else:
            return U0
    elif (U0[0] - U1[1] < PARAM_ANGLE_TOL) and (-PARAM_ANGLE_TOL < U0[1] - U1[1]):
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

    if abs(V0[0] - V1[1]) < PARAM_ANGLE_TOL:
        imin = 1  # U1[0]
        imax = 0  # U0[1]
    elif abs(V1[0] - V0[1]) < PARAM_ANGLE_TOL:
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
