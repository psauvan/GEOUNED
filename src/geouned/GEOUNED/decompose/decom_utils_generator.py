#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging
import math
import numpy

from geouned.geo.vector_geometry import GMatrix

from ..utils.data_constants import twoPi
from ..utils.geouned_classes import GeounedSurface
from ..utils.geometry_gu import other_face_edge, is_same_surface
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
    Gheal_topology,
    Gmake_wire,
)
from ..utils.basic_functions_part1 import (
    is_parallel,
    is_same_value,
)
from ..utils.meta_surfaces_utils import material_direction, region_sign, planar_edges

logger = logging.getLogger("general_logger")


def torus_bound_planes(solidFaces, face, tolerances):
    params = face.ParameterRange
    planes = []
    if is_same_value(params[1] - params[0], twoPi, tolerances.value):
        return planes

    Edges = face.OuterWire.Edges

    for e in Edges:
        curve = Gclassify_curve(e)

        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is not None:
            if is_same_surface(face.Surface, adjacent_face.Surface):
                continue  # doesn't create plane if other face has same surface

        if type(curve) is GCircle:
            dir = curve.Axis
            if not is_parallel(dir, face.Surface.Axis, tolerances.angle):
                center = curve.Center
                dim1 = curve.Radius
                dim2 = curve.Radius
                plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
                planes.append(plane)

        elif type(curve) is GEllipse:
            dir = curve.Axis
            center = curve.Center
            dim1 = curve.MinorRadius
            dim2 = curve.MajorRadius
            plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
            planes.append(plane)

        elif type(curve) is GBSpline:
            planeParams = spline_wires((e,), face)
            if planeParams is not None:
                plane = GeounedSurface(("Plane", planeParams))
                planes.append(plane)

    return planes


def cks_bound_planes(solidFaces, face, omitfaces, Edges=None):

    if Edges is None:
        Edges = face.OuterWire.Edges
    planes = []

    for e in Edges:
        if not planar_edges([e]):
            continue
        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is not None:
            if type(adjacent_face.Surface) is GPlane and adjacent_face.Index in omitfaces:
                continue
            if type(adjacent_face.Surface) is GTorus:
                continue  # doesn't create plane if other face is a torus
            if is_same_surface(face.Surface, adjacent_face.Surface):
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

                # calculate distance between the two axes and if it is less than a tolerance, do not create a plane
                cross = axis1.cross(axis2)
                if cross.length > 1e-6:
                    dist = abs(cross.dot(p1 - p2)) / cross.length
                    if dist > 1e-3:
                        continue  # doesn't create plane if the axes are not close enough
                else:
                    # if the axes are parallel, check the distance between the two points
                    dist = (p1 - p2).length
                    if dist > 1e-3:
                        continue  # doesn't create plane if the axes are not close enough

            plane = cks_edge_plane(face, [e])
            if plane is not None:
                planes.append(plane)
    if len(planes) > 2 and type(face.Surface) is not GSphere:
        planes = most_outer_planes(face.Surface.Axis, planes)
    return planes


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
            d = 0.01 * abs(majoraxis.dot(rmax - rmin))
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


def valid_solid(solid: GSolid, Volume) -> bool:
    if solid.Volume < 0:
        return False
    Vol_tol = 1e-2
    Vol_area_ratio = 1e-3
    if solid.Area == 0 or abs(solid.Volume / solid.Area) < Vol_area_ratio:
        return False
    if abs(solid.Volume) < Vol_tol:
        return False
    return True


def _refine_if_valid(solid: GSolid) -> GSolid:
    # refine() (ShapeUpgrade_UnifySameDomain/removeSplitter) is a cosmetic
    # simplification of an already-valid solid, not a repair tool -- on a
    # solid that's already topologically invalid (BRepCheck_Analyzer), its
    # UnifyEdges step is a confirmed, previously-documented crash/hang
    # risk (see GSolid.refine()'s own docstring, the ConeSphere.stp case
    # under occ/ocp) that no amount of Python try/except can catch, since
    # it's a native process crash, not a raised exception. Confirmed live
    # (2026-08-19, Solidos/Big_one_cell/modelcell_cut1.stp under ocp): a
    # BOP-produced fragment that's already invalid before refine() ever
    # runs reliably segfaults the process inside refine()'s own UnifyEdges
    # call.
    #
    # A real repair attempt via .fix() (ShapeFix_Shape) was tried here
    # twice, both reverted. The first attempt crashed even earlier than
    # refine() itself did; that specific crash traced back to a real bug in
    # .fix() (fixed 2026-08-19: it used to reassign its own `native`
    # variable to UnifyEdges' own possibly-corrupted output before checking
    # that output's validity, so its ShapeFix_Shape fallback silently
    # repaired the *corrupted* intermediate instead of the true original
    # input). With that fixed, calling .fix() *after* a full decomposition
    # had already completed (on the final, already-produced invalid
    # fragments, as a separate manual pass) worked cleanly with no crash on
    # this exact modelcell_cut1.stp reproduction. But wiring the fixed
    # .fix() into this function -- called *during* decomposition, on
    # intermediate fragments that then flow into further Gsplit calls, not
    # just on final output -- reproduced a crash again on the same file,
    # this time STATUS_STACK_OVERFLOW (0xC00000FD) rather than the original
    # UnifyEdges access violation. So repairing a fragment mid-decomposition
    # and feeding the repaired result back into further cuts is its own,
    # separately confirmed, still-unresolved crash risk -- distinct from
    # (and not fixed by) the .fix() bug fix above. Reverted back to the
    # simple, confirmed-safe form: leave an already-invalid solid untouched
    # rather than risk repairing it here. GSolid.fix() itself remains a
    # real, safe repair tool for use *after* decomposition is complete (on
    # final output only), just not at this specific, mid-pipeline call site.
    return solid.refine() if solid.is_valid() else solid


def remove_solids(Solids: list[GSolid], Volume) -> list[GSolid]:

    if len(Solids) == 1:
        return [_refine_if_valid(Solids[0])]

    Solids_Clean = []
    for solid in Solids:
        if not valid_solid(solid, Volume):
            logger.warning(f"remove_solids degenerated solids are produced bad dimensions")
            continue
        if not solid.is_valid():
            # A failed / degenerate BOP split can hand back a fragment with
            # real geometry (so valid_solid passes it) but broken topology
            # -- e.g. a face with BRepCheck_InvalidImbricationOfWires, which
            # ShapeFix/refine/fix cannot repair (see Gheal_topology's own
            # docstring + reference_cad_defect_recipes.md Recipe 3). Feeding
            # such a fragment forward can crash later face-analysis code
            # (get_adjacent_cylplane on a face whose wires() came back
            # empty). Try the in-memory STEP serialize->deserialize rebuild;
            # use it only if it comes back valid and volume-conserved.
            # If it can't heal (a non-manifold _repair_non_manifold_solid
            # remnant etc.), keep the original fragment unchanged -- the
            # pre-existing behaviour, since dropping it here loses real
            # volume (confirmed: dropping rev_pipe.stp's un-healable
            # fragment sends its d1suned tally 0.998 -> 0.952). The
            # `cyl.OuterWire is None` guard in get_adjacent_cylplane keeps
            # such a survivor from crashing downstream.
            healed = Gheal_topology(solid)
            if healed is not None:
                solid = healed
        Solids_Clean.append(solid)

    return [_refine_if_valid(sol) for sol in Solids_Clean]


def external_plane(plane, Faces):
    Edges = plane.Edges
    for e in Edges:
        adjacent_face = other_face_edge(e, plane, Faces)
        if adjacent_face is None:
            continue
        if isinstance(
            adjacent_face.Surface, GPlane
        ):  # if not plane not sure current plane will not cut other part of the solid
            if region_sign(plane, adjacent_face) == "OR":
                return False
        else:
            return False
    return True


def exclude_no_cutting_planes(Faces, omit=None):
    if omit is None:
        omit = set()
        return_set = True
    else:
        return_set = False
    for f in Faces:
        if f.Index in omit:
            continue
        if isinstance(f.Surface, GPlane):
            if external_plane(f, Faces):
                omit.add(f.Index)

    return omit if return_set else None


def cutting_face_number(f, Faces, omitfaces):
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
            adjacent_face.__native__.exportStep("Spline_surface.stp")
            raise RuntimeError("Spline surface detected")
        elif region_sign(f, adjacent_face) == "OR":
            ncut += 1
    return ncut


def order_plane_face(Faces, omitfaces, min_area=None, min_face_width=None):
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
        ncut = cutting_face_number(f, Faces, omitfaces)
        counts.append((ncut, f.Index))
        face_dict[f.Index] = f

    counts.sort(reverse=True)
    return tuple(face_dict[x[1]] for x in counts)


def omit_isolated_planes(Faces, omitfaces):
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
                if abs(abs(adjacent_face.Surface.Axis.dot(f.Surface.Axis)) - 1) < 1e-5:
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
