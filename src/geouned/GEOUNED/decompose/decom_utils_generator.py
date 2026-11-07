#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging
import math
import numpy

from geouned.geo.vector_geometry import GMatrix

from ..utils.data_constants import twoPi
from ..utils.geouned_classes import GeounedSurface
from ..utils.geometry_gu import other_face_edge, is_same_surface, ShellFaceGu
from ...geo import (
    GPlane,
    GCylinder,
    GCone,
    GSolid,
    GSphere,
    GTorus,
    GLine,
    GCircle,
    GEllipse,
    GBSpline,
    GVector,
    Gclassify_curve,
    Gmake_wire,
)
from ..utils.basic_functions_part1 import (
    is_same_value,
)
from ..utils.meta_surfaces_utils import material_direction, region_sign, planar_edges, edge_1D
from ...geo.constants import CKS_COAXIAL_AXIS_DIST_TOL, NUMERIC_DOUBLE_TOL, NUMERIC_TOL, PARAM_ANGLE_TOL, POINT_POINT_TOL
from ...geo.surface_geometry import axes_parallel

logger = logging.getLogger("general_logger")


def curved_edge_plane(face, e):
    """The plane through `e`, a single CIRCLE/ELLIPSE/planar-BSpline
    boundary edge of `face` (any of cylinder/cone/sphere/torus) --
    built directly from the edge's own curve geometry (center/axis/
    radii), never from `face`'s own material direction (unlike
    cks_edge_plane, which both relies on that and hardcodes dim1=dim2=1
    instead of the edge's own real size -- a design tuned for
    cks_bound_planes' own narrower role, not verified safe for every
    other caller). `None` when `e`'s own curve type isn't one of these
    three, or (Circle only, torus faces only) when the circle's own
    axis is parallel to the torus's own axis -- a "latitude" boundary
    circle (fixed U-parameter, varying V around the tube), whose own
    plane would cut across the tube at a fixed axial height rather than
    bound the face where it actually ends; only a "meridian" boundary
    circle (fixed V, the tube's own cross-section at one azimuth, axis
    perpendicular to the torus axis) is a real face-ending bound. No
    analogous exclusion exists for cylinder/cone/sphere, whose own
    circular boundaries are always a genuine end cap."""
    curve = Gclassify_curve(e)

    if type(curve) is GCircle:
        dir = curve.Axis
        center = curve.Center
        dim1 = curve.Radius
        dim2 = curve.Radius
        return GeounedSurface(("Plane", (center, dir, dim1, dim2)))

    if type(curve) is GEllipse:
        dir = curve.Axis
        center = curve.Center
        dim1 = curve.MinorRadius
        dim2 = curve.MajorRadius
        return GeounedSurface(("Plane", (center, dir, dim1, dim2)))

    if type(curve) is GBSpline:
        planeParams = spline_wires((e,), face)
        if planeParams is not None:
            return GeounedSurface(("Plane", planeParams))

    return None


def line_edge_tangent_plane(face, adjacent_face, e):
    """The cutting plane for a near-tangent pair whose shared boundary
    edge is a straight LINE -- a generatrix, which planar_edges always
    rejects on its own (a bare line has no unique supporting plane), so
    this is a separate, direct construction instead of curved_edge_
    plane's Circle/Ellipse/BSpline-only logic.

    Per direct user design, derived case by case from the only 3 real
    possibilities for two DIFFERENT, near-tangent surfaces to share a
    straight-line boundary (a cone-cone pair might also do so in some
    degenerate configuration, but no confirmed real case exists -- not
    handled, `None`):
    - cylinder-plane / cone-plane: the cutting plane contains the
      shared edge and its own normal is perpendicular to the adjacent
      plane's own normal -- i.e. normal_cut = cross(edge_direction,
      adjacent_plane_normal). NOT the adjacent plane itself reused
      directly (a first, wrong attempt at this derivation) -- that
      plane IS already one of the solid's own real boundary faces, so
      reusing it as the cutting tool does not resolve the tangency
      degeneracy at all, it just relocates it to the plane's own side.
    - cylinder-cylinder: 2 cylinders sharing a straight generatrix
      tangentially must have PARALLEL axes; the cutting plane contains
      the shared edge and is perpendicular to the plane spanned by the
      two axes -- i.e. its own normal lies within that axes-plane,
      perpendicular to the (shared) axis direction, which is exactly
      the direction connecting the two cylinder centers once their
      common axis component is removed.
    `None` for any other pair, or a degenerate configuration (e.g.
    coincident cylinder axes, leaving no well-defined radial
    direction)."""
    if e.Length < POINT_POINT_TOL:
        return None  # degenerate edge -- no well-defined direction

    adj_surf = adjacent_face.Surface
    if type(adj_surf) is GPlane:
        first, last = e.ParameterRange
        v0 = e.value_at(first)
        v1 = e.value_at(last)
        diff = v1 - v0
        if diff.length < POINT_POINT_TOL:
            # a closed/periodic edge: its two PARAMETER endpoints land on
            # the same 3D point even though the edge itself has real
            # length (e.Length, checked above, does not catch this) --
            # no well-defined chord direction to build the cutting plane
            # from.
            return None
        e_dir = diff.normalized()
        normal = e_dir.cross(adj_surf.Axis)
        if normal.length < NUMERIC_TOL:
            return None  # degenerate: edge direction parallel to the plane's own normal
        normal = normal.normalized()
        position = e.value_at(0.5 * (first + last))
        return GeounedSurface(("Plane", (position, normal, 1, 1)))

    face_surf = face.Surface
    if type(face_surf) is GCylinder and type(adj_surf) is GCylinder:
        axis_dir = face_surf.Axis
        diff = adj_surf.Center - face_surf.Center
        radial = diff - diff.dot(axis_dir) * axis_dir
        if radial.length < NUMERIC_TOL:
            return None  # coincident axes -- no well-defined radial direction
        radial = radial.normalized()
        first, last = e.ParameterRange
        position = e.value_at(0.5 * (first + last))
        return GeounedSurface(("Plane", (position, radial, 1, 1)))

    return None


def is_line_edge(e):
    """True if `e`'s own curve is a straight line -- either a genuine
    GLine, or a GBSpline that is geometrically straight (zero curvature
    along its length, the same degenerate-BSpline-as-a-line signature
    meta_surfaces_utils.edge_1D already tests for a different caller).
    The curve TYPE is checked first (cheap, exact) so a real curved
    BSpline never pays for the curvature probe at all."""
    curve = Gclassify_curve(e)
    if type(curve) is GLine:
        return True
    if type(curve) is GBSpline:
        return edge_1D(e)
    return False


def torus_bound_planes(solidFaces, face, *, tolerances):
    params = face.ParameterRange
    planes = []
    if is_same_value(params[1] - params[0], twoPi, PARAM_ANGLE_TOL):
        return planes

    Edges = face.OuterWire.Edges

    for e in Edges:
        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is not None:
            if is_same_surface(face.Surface, adjacent_face.Surface, tolerances=tolerances):
                continue  # doesn't create plane if other face has same surface

        plane = curved_edge_plane(face, e)
        if plane is not None:
            planes.append(plane)

    return planes


def cks_bound_planes(solidFaces, face, omitfaces, Edges=None, *, tolerances):
    #face can be a cylinder/cone/sphere/torus. Edges must by planar but not a straight line.

    if Edges is None:
        Edges = face.OuterWire.Edges
    planes = []

    for e in Edges:
        if not planar_edges([e], tolerances=tolerances):
            continue
        if is_line_edge(e):
            continue
        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is not None:
            if type(adjacent_face.Surface) is GPlane and adjacent_face.Index in omitfaces:
                continue
            if type(adjacent_face.Surface) is GTorus:
                continue  # doesn't create plane if other face is a torus
            if is_same_surface(face.Surface, adjacent_face.Surface, tolerances=tolerances):
                continue  # doesn't create plane if other face has same surface
            if (type(face.Surface) is GCone or type(face.Surface) is GCylinder) and (
                type(adjacent_face.Surface) is GCone or type(adjacent_face.Surface) is GCylinder
            ):

                if type(face.Surface) is GCone:
                    p1 = face.Surface.Apex
                else:
                    p1 = face.Surface.Center

                if type(adjacent_face.Surface) is GCone:
                    p2 = adjacent_face.Surface.Apex
                else:
                    p2 = adjacent_face.Surface.Center

                axis1 = face.Surface.Axis
                axis2 = adjacent_face.Surface.Axis

                # Calculate the distance between the two axes: a plane is
                # only built when they are close enough to be the same
                # line (a genuine coaxial cone/cylinder tangent-
                # continuation seam, e.g. a Can/TCone's own closing
                # circle) -- two genuinely distinct, non-coaxial axes
                # give up here (their own real edge, if planar at all,
                # isn't this kind of closing seam). CKS_COAXIAL_AXIS_
                # DIST_TOL (not POINT_POINT_TOL, which is tuned for a
                # much smaller role, exact point coincidence) -- see its
                # own docstring in geo/constants.py for the corpus
                # measurement this value is based on.
                cross = axis1.cross(axis2)
                if cross.length > NUMERIC_DOUBLE_TOL:
                    dist = abs(cross.dot(p1 - p2)) / cross.length
                    if dist > CKS_COAXIAL_AXIS_DIST_TOL:
                        continue  # not coaxial enough -- a real corner, no plane here
                else:
                    # if the axes are parallel, check the distance between the two points
                    dist = (p1 - p2).length
                    if dist > CKS_COAXIAL_AXIS_DIST_TOL:
                        continue  # not coaxial enough -- a real corner, no plane here

            plane = cks_edge_plane(face, [e])
            if plane is not None:
                planes.append(plane)
    if len(planes) > 2 and type(face.Surface) is not GSphere:
        planes = most_outer_planes(face.Surface.Axis, planes)
    return planes


def tangent_cut_plane(face, solidFaces, *, tolerances, roundCorner=False):
    """If `face` (a cylinder/cone/sphere/torus candidate about to be
    yielded as a cutting surface by cylinder_generator/cone_generator/
    sphere_generator/torus_generator) is near-tangent to an adjacent,
    DIFFERENT-surface face along one of its own boundary edges, AND
    that shared edge itself lies within a well-defined plane
    (planar_edges), return the plane through that edge -- the caller
    should yield THIS instead of `face`'s own normal surface candidate.
    `None` when no edge qualifies (no near-tangent neighbor, or the
    tangent edge isn't planar) -- the caller should yield its own
    normal candidate as usual.

    Tangency is decided by comparing each face's own outward normal
    (material_direction's own convention, both oriented consistently
    away from material -- see GFace.normal_at) at the shared edge's own
    midpoint: cos(theta) = dot(normal1, normal2); theta <
    tolerances.tangent_angle means the two surfaces continue into each
    other smoothly (no real corner), so cutting along the curved
    surface itself right at that line risks a degenerate, sliver-
    producing or non-separating result -- confirmed live, 2026-10-28,
    Mixed/base_all8_to_cylinder.stp: a cylinder tangent to a torus at
    EXACTLY theta=0 left generic_split's own candidate search unable to
    split that fragment at all (BOPAlgo_Splitter repeatedly falling back
    to the uncut original once every curved-surface candidate produced
    only a degenerate sliver).

    tolerances.tangent_angle is deliberately a generous (5 degrees,
    per direct user estimate of the angle above which a cut is already
    safe), not a tight corpus-measured value: per direct user design,
    substituting the plane is never itself a source of error -- at
    worst it fails to separate anything, exactly like any other
    rejected candidate (generic_split's own search just moves on to the
    next one, since neither this function nor its callers ever touch
    omitfaces) -- it only risks a less optimal decomposition (more
    pieces than strictly necessary). Robustness is explicitly preferred
    over optimal granularity for this decision.

    The plane itself is built by curved_edge_plane (Circle/Ellipse/
    planar-BSpline edges) or line_edge_tangent_plane (a straight-line
    edge, handled by its own dedicated cylinder/cone/plane-specific
    construction -- see that function's own docstring) -- never
    cks_edge_plane, which was designed for cks_bound_planes' own
    narrower role (a hardcoded dim1=dim2=1, a sign flip from `face`'s
    own material_direction, an un-trimmed edge.Curve.value(0) sample)
    and isn't verified safe for this different one.

    `face` may also be a `ShellFaceGu` -- several contiguous fragments of
    the same analytic surface, merged by merge_same_surface_faces when a
    boolean cut left a feature split into pieces (confirmed live,
    Mixed/base_all8_to_cylinder.stp). A ShellFaceGu has no single
    OuterWire of its own (see geometry_gu.ShellFaceGu), so the search
    fans out over each real fragment in turn, the same pattern as
    get_adjacent_cylplane's own is_shell branch: any edge shared BETWEEN
    two fragments of the merged shell is skipped naturally by the
    is_same_surface check below (both fragments carry the identical
    surface), so no extra filtering is needed here -- the first fragment
    with a genuine tangent neighbor wins."""
    if type(face) is ShellFaceGu:
        for f in face.Faces:
            plane = tangent_cut_plane(f, solidFaces, tolerances=tolerances, roundCorner=roundCorner)
            if plane is not None:
                return plane
        return None

    for e in face.OuterWire.Edges:
        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is None:
            continue
        if is_same_surface(face.Surface, adjacent_face.Surface, tolerances=tolerances):
            continue

        curve = Gclassify_curve(e)
        if roundCorner and type(adjacent_face.Surface) is GPlane:
            continue  
        
        if type(curve) is GLine or type(adjacent_face.Surface) is GPlane:
            plane = line_edge_tangent_plane(face, adjacent_face, e)
        elif planar_edges([e], tolerances=tolerances):
            plane = curved_edge_plane(face, e)
        else:
            plane = None  # tangent, but this edge has no supported, well-defined plane
        if plane is None:
            continue

        first, last = e.ParameterRange
        pos = e.value_at(0.5 * (first + last))
        u1, v1 = face.Surface.parameter(pos)
        normal1 = face.normal_at(u1, v1)
        u2, v2 = adjacent_face.Surface.parameter(pos)
        normal2 = adjacent_face.normal_at(u2, v2)
        cos_theta = normal1.dot(normal2)
        if cos_theta <= math.cos(tolerances.tangent_angle):
            continue  # not tangent enough -- a real corner, cut by the curved surface itself

        return plane
    return None


def torus_tangent_cut_surface(face, solidFaces, *, tolerances):
    """Like tangent_cut_plane, but for a TORUS candidate about to be
    yielded by torus_generator specifically: when the near-tangent
    neighbor found along one of `face`'s own boundary edges is a
    PLANE, a plane cannot be the cutting surface here -- it would only
    graze the torus along that one tangent circle, never cleanly
    separating material, which left generic_split's own recursive
    search unable to converge (confirmed live, 2026-10-30,
    Torus/example.stp's own solid index 6: an unbounded chain of
    ever-finer near-tangent-plane substitutions -- nfaces +2 and
    candidate count +4 per recursion level, for 25+ levels -- eventually
    overflowing the native call stack). The surface that genuinely
    separates "inside this radius" from "outside it" is a cylinder
    coaxial with the torus's own axis, at the tangent edge's own
    (circular) radius.

    For any OTHER near-tangent neighbor (e.g. a torus tangent to a
    cylinder, Mixed/base_all8_to_cylinder.stp -- the original fixture
    tangent_cut_plane was built for), this defers entirely to
    tangent_cut_plane's own, unmodified plane-based logic."""
    for e in face.OuterWire.Edges:
        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is None:
            continue
        if is_same_surface(face.Surface, adjacent_face.Surface, tolerances=tolerances):
            continue

        curve = Gclassify_curve(e)
        if type(curve) is GLine or type(adjacent_face.Surface) is GPlane:
            plane = line_edge_tangent_plane(face, adjacent_face, e)
        elif planar_edges([e], tolerances=tolerances):
            plane = curved_edge_plane(face, e)
        else:
            plane = None  # tangent, but this edge has no supported, well-defined plane
        if plane is None:
            continue


        first, last = e.ParameterRange
        pos = e.value_at(0.5 * (first + last))
        u1, v1 = face.Surface.parameter(pos)
        normal1 = face.normal_at(u1, v1)
        u2, v2 = adjacent_face.Surface.parameter(pos)
        normal2 = adjacent_face.normal_at(u2, v2)
        cos_theta = normal1.dot(normal2)
        if cos_theta <= math.cos(tolerances.tangent_angle):
            continue  # not tangent enough -- a real corner, cut by the torus itself

        if type(adjacent_face.Surface) is not GPlane:
            return plane
        else:
            torus = face.Surface
            # Prefer the shared edge's own circle radius (exact to the
            # edge's real geometry) over a radial distance recomputed from
            # one sampled point -- but the edge need not be a circle (e.g.
            # a BSpline residue of an earlier cut), in which case there is
            # no curve.Radius to read and the radial distance from the
            # torus axis at the sampled point is the only option left.
            radius = curve.Radius if type(curve) is GCircle else None
            if radius is None:
                diff = pos - torus.Center
                radial = diff - diff.dot(torus.Axis) * torus.Axis
                radius = radial.length
                if radius < NUMERIC_TOL:
                    continue  # degenerate -- tangent point sits on the torus's own axis
            return GeounedSurface(("CylinderOnly", (torus.Center, torus.Axis, radius, 1)))


def cks_edge_plane(face, edges, pc=None):

    planeParams = None
    spline = False
    for edge in edges:
        if type(Gclassify_curve(edge)) is GBSpline:
            spline = True
            break

    if spline:
        planeParams = spline_wires(edges, face, pc)
    else:
        edge = edges[0]
        curve = Gclassify_curve(edge)
        if type(curve) in (GCircle, GEllipse):
            pos = edge.Curve.value(0)
            center = curve.Center
            dir = curve.Axis
            vect, normalf = material_direction(pos, face, edge)
            if dir.dot(vect) < 0:
                dir = -dir
            planeParams = [center, dir, 1, 1, False]

    if planeParams is not None:
        return GeounedSurface(("Plane", planeParams))


def spline_wires(edges, face, pc=None):

    zaxis = face.Surface.Axis
    try:
        W = Gmake_wire(list(edges))
        majoraxis = get_axis_inertia(W.MatrixOfInertia)
    except:
        majoraxis = GVector(0, 0, 0)
        for e in edges:
            majoraxis = majoraxis + get_axis_inertia(e.MatrixOfInertia)
        majoraxis = majoraxis.normalized()

    edge = edges[0]
    p0, p1 = edge.ParameterRange
    pe = 0.5 * (p0 + p1)
    pos = edge.Curve.value(pe)
    vect, normalf = material_direction(pos, face, edge)

    lowSide = zaxis.dot(vect) > 0

    rmin = (1e15, None)
    rmax = (-1e15, None)

    for edge in edges:
        curve = Gclassify_curve(edge)

        if type(curve) is GBSpline:
            for p in edge.Curve.Poles:
                r = majoraxis.dot(p)
                if rmin[0] > r:
                    rmin = (r, p)
                if rmax[0] < r:
                    rmax = (r, p)
        elif type(curve) is GLine:
            for v in edge.Vertexes:
                r = majoraxis.dot(v)
                if rmin[0] > r:
                    rmin = (r, v)
                if rmax[0] < r:
                    rmax = (r, v)
        else:
            p0, p1 = projection(edge, majoraxis)
            for pi in (p0, p1):
                p = edge.value_at(pi)
                r = majoraxis.dot(p)
                if rmin[0] > r:
                    rmin = (r, p)
                if rmax[0] < r:
                    rmax = (r, p)

        rmin = rmin[1]
        rmax = rmax[1]
        if pc is not None:
            centerDir = pc - W.CenterOfMass
            centerDir.normalize()
            if centerDir.dot(majoraxis) > 0:
                point = rmax
            else:
                point = rmin
            d = 0.01 * abs(majoraxis.dot(rmax - rmin))  # 1% of the real span along majoraxis
        else:
            point = 0.5 * (rmin + rmax)
            d = 0.51 * abs(majoraxis.dot(rmax - rmin))

        vec = majoraxis
        if majoraxis.dot(zaxis) > 0:
            if not lowSide:
                vec = -vec
        else:
            if lowSide:
                vec = -vec
        if pc is not None:
            point += d * centerDir
        else:
            point -= d * vec

        return [point, vec, 1, 1, False]  # positive plane directiontoward the center of the cylinder


def get_axis_inertia(mat: GMatrix):
    # The inertia tensor is always symmetric -- eigh (not eig) both avoids
    # the complex-dtype/ComplexWarning noise eig produces on a matrix it
    # doesn't know is symmetric, and is the numerically appropriate solver
    # for this case.
    inertialMat = numpy.array(((mat.A11, mat.A12, mat.A13), (mat.A21, mat.A22, mat.A23), (mat.A31, mat.A32, mat.A33)))
    eigval, evect = numpy.linalg.eigh(inertialMat)
    principal = evect.T[numpy.argmax(eigval)]

    return GVector(float(principal[0]), float(principal[1]), float(principal[2]))


def external_plane(plane, Faces, *, tolerances):
    Edges = plane.Edges
    for e in Edges:
        adjacent_face = other_face_edge(e, plane, Faces)
        if adjacent_face is None:
            continue
        if isinstance(
            adjacent_face.Surface, GPlane
        ):  # if not plane not sure current plane will not cut other part of the solid
            if region_sign(plane, adjacent_face, tolerances=tolerances) == "OR":
                return False
        else:
            return False
    return True


def exclude_no_cutting_planes(Faces, omit=None, *, tolerances):
    if omit is None:
        omit = set()
        return_set = True
    else:
        return_set = False
    for f in Faces:
        if f.Index in omit:
            continue
        if isinstance(f.Surface, GPlane):
            if external_plane(f, Faces, tolerances=tolerances):
                omit.add(f.Index)

    return omit if return_set else None


def cutting_face_number(f, Faces, omitfaces, *, tolerances):
    Edges = f.Edges
    ncut = 0
    for e in Edges:
        adjacent_face = other_face_edge(e, f, Faces)
        if adjacent_face is None:
            continue
        if adjacent_face.Index in omitfaces:
            continue
        if isinstance(adjacent_face.Surface, GPlane):
            ncut += 1
        elif adjacent_face.Surface is None:
            # unsupported (spline / hyperbola / ...) neighbouring surface:
            # decomposition cannot classify it. (Was preceded by a stray
            # `adjacent_face.__native__.exportStep("Spline_surface.stp")`
            # debug dump -- FreeCAD-only API, so under pyOCC it raised
            # AttributeError and masked this RuntimeError; removed.)
            raise RuntimeError("Spline surface detected")
        elif region_sign(f, adjacent_face, tolerances=tolerances) == "OR":
            ncut += 1
    return ncut


def order_plane_face(Faces, omitfaces, min_area=None, min_face_width=None, *, tolerances):
    # A residual sliver plane (a real face, but a near-zero-area boolean-
    # cut artifact, not a genuine feature -- see Tolerances.min_area's own
    # docstring) was never excluded here: plane_generator's own cutting-
    # plane candidate list had no area filter of any kind, so every sliver
    # plane in a solid was always tried as a real cutting surface during
    # decomposition, regardless of how min_area was set (confirmed live,
    # Solidos/test_models/Decomposed/SCDR_90_piece2.stp, 2026-08-23: a
    # 1.9262mm^2 sliver plane was found to be the source of an
    # unrecognized cutting-plane candidate; raising min_area had zero
    # effect on the actual translation because nothing here ever read it).
    #
    # min_area alone isn't a reliable sliver signal on its own, though: a
    # real, large-area, genuinely thin-and-long panel/wall face (e.g.
    # Solidos/test_models/RoundCorners/TVA_solid16_cell17.stp's own
    # face[12], area=14575.8mm^2, a real 5.6mm x 2602.8mm plate edge) can
    # have a raw Area comparable to -- or bigger than -- a real sliver's
    # (SCDR_90_piece2.stp's own sliver is only 1.93mm^2), while being a
    # completely legitimate feature. min_face_width (GFace.CharacteristicWidth
    # -- see its own docstring) is the corpus-verified robust signal:
    # it recovers the face's true physical short dimension regardless of
    # surface curvature, so a genuinely thin-but-long real panel (width
    # 5.6mm) is never confused with an actual boolean-cut sliver (width
    # 0.055mm) the way raw Area can be.
    for f in Faces:
        if f.Index in omitfaces:
            continue
        if not isinstance(f.Surface, GPlane):
            continue
        if min_area is not None and f.Area < min_area:
            omitfaces.add(f.Index)
            continue
        if min_face_width is not None and getattr(f, "CharacteristicWidth", float("inf")) < min_face_width:
            omitfaces.add(f.Index)

    counts = []
    face_dict = {}
    for f in Faces:
        if f.Index in omitfaces:
            continue
        if not isinstance(f.Surface, GPlane):
            continue
        ncut = cutting_face_number(f, Faces, omitfaces, tolerances=tolerances)
        counts.append((ncut, f.Index))
        face_dict[f.Index] = f

    counts.sort(reverse=True)
    return tuple(face_dict[x[1]] for x in counts)


def omit_isolated_planes(Faces, omitfaces, *, tolerances):
    for f in Faces:
        if f.Index in omitfaces:
            continue
        if not isinstance(f.Surface, GPlane):
            continue
        if f.OuterWire is None:
            # a face whose only wire(s) are degenerate (zero-length/
            # edgeless) has no meaningful outer boundary to walk --
            # pick_outer_wire() returns None for this case rather than
            # crashing; nothing useful can be inferred about isolation
            # from such a face, so skip it.
            continue

        for e in f.OuterWire.Edges:
            adjacent_face = other_face_edge(e, f, Faces)
            if adjacent_face is None:
                continue
            if type(adjacent_face.Surface) is GPlane:
                if axes_parallel(adjacent_face.Surface.Axis, f.Surface.Axis, tolerances.pln_angle):
                    if adjacent_face.Index not in omitfaces:
                        omitfaces.add(f.Index)
                        break


def projection(edge, axis):
    pmin, pmax = edge.ParameterRange
    if type(edge.Curve) is GCircle:
        vmin = edge.value_at(pmin) - edge.Curve.Center
        v1 = edge.Curve.Axis.cross(vmin)
        dmin = vmin.dot(axis)
        d1 = v1.dot(axis)
        if dmin == 0:
            p0 = 0.5 * math.pi + pmin
            p1 = p0 + math.pi
        else:
            p0 = math.atan(d1 / dmin)
            p1 = p0 + math.pi
        if p1 < pmax:
            return p0, p1
        elif p0 < pmax:
            v0 = edge.value_at(p0) - edge.Curve.Center
            d0 = v0.dot(axis)
            vmax = edge.value_at(pmax) - edge.Curve.Center
            dmax = vmax.dot(axis)
            if abs(d0 - dmin) < abs(d0 - dmax):
                return p0, pmax
            else:
                return p0, pmin
        else:
            return pmin, pmax
    else:
        vx = edge.Curve.XAxis
        vy = edge.Curve.YAxis
        a = edge.Curve.MajorRadius
        b = edge.Curve.MinorRadius
        dx = vx.dot(axis)
        dy = vy.dot(axis)
        dmin = (edge.value_at(pmin) - edge.Curve.Center).dot(axis)
        if dx == 0:
            p0 = 0.5 * math.pi + pmin
            p1 = p0 + math.pi
        else:
            p0 = math.atan(b * dy / a * dx)
            p1 = p0 + math.pi
        if p1 < pmax:
            return p0, p1
        elif p0 < pmax:
            v0 = edge.value_at(p0) - edge.Curve.Center
            d0 = v0.dot(axis)
            vmax = edge.value_at(pmax) - edge.Curve.Center
            dmax = vmax.dot(axis)
            if abs(d0 - dmin) < abs(d0 - dmax):
                return p0, pmax
            else:
                return p0, pmin
        else:
            return pmin, pmax


def most_outer_planes(axis, planes):

    if len(planes) < 3:
        return planes

    distances = []
    for i, p in enumerate(planes):
        proj = axis.dot(p.Surf.Position)
        distances.append((proj, i))

    distances.sort()
    i0 = distances[0][1]
    i1 = distances[-1][1]
    return (planes[i0], planes[i1])
