#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging
import math
import numpy

import FreeCAD
import Part

from ..utils.data_classes import Tolerances
from ..utils.geouned_classes import GeounedSurface
from ..utils.geometry_gu import PlaneGu, TorusGu, other_face_edge
from ..utils.basic_functions_part1 import (
    is_parallel,
    is_same_value,
)
from ..utils.meta_surfaces_utils import region_sign, spline_2D, spline_1D

logger = logging.getLogger("general_logger")
twoPi = math.pi * 2


def gen_plane(pos, normal, diag):
    plane = Part.makePlane(diag, diag, pos, normal)
    vec_on_plane = plane.Vertexes[3].Point.sub(plane.Vertexes[0].Point)
    new_pos = plane.Vertexes[0].Point.sub(vec_on_plane)
    plane_center = Part.makePlane(2.0 * diag, 2.0 * diag, new_pos, normal)
    return plane_center


def cyl_bound_planes_first_version(solidFaces, face):
    Edges = face.OuterWire.Edges
    planes = []
    for e in Edges:
        try:
            curve = str(e.Curve)
        except:
            curve = "none"

        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is None:
            continue

        if type(adjacent_face.Surface) is TorusGu:
            continue  # doesn't create plane if other face is a torus
        if type(adjacent_face.Surface) is PlaneGu:
            continue  # doesn't create plane if other face is a Plane
        if face.Surface.isSameSurface(adjacent_face.Surface):
            continue  # doesn't create plane if other face has same surface

        if curve[0:6] == "Circle":
            if e.Curve.Radius < 1e-6:
                continue
            dir = e.Curve.Axis
            center = e.Curve.Center
            dim1 = e.Curve.Radius
            dim2 = e.Curve.Radius
            plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
            planes.append(plane)

        elif curve == "<Ellipse object>":
            if e.Curve.MinorRadius < 1e-6 or e.Curve.MajorRadius < 1e-6:
                continue
            dir = e.Curve.Axis
            center = e.Curve.Center
            dim1 = e.Curve.MinorRadius
            dim2 = e.Curve.MajorRadius
            plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
            planes.append(plane)

    return planes


def torus_bound_planes(solidFaces, face, tolerances):
    params = face.ParameterRange
    planes = []
    if is_same_value(params[1] - params[0], twoPi, tolerances.value):
        return planes

    Edges = face.OuterWire.Edges

    for e in Edges:
        try:
            curve = str(e.Curve)
        except:
            curve = "none"

        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is not None:
            if face.Surface.isSameSurface(adjacent_face.Surface):
                continue  # doesn't create plane if other face has same surface

        if curve[0:6] == "Circle":
            dir = e.Curve.Axis
            if not is_parallel(dir, face.Surface.Axis, tolerances.angle):
                center = e.Curve.Center
                dim1 = e.Curve.Radius
                dim2 = e.Curve.Radius
                plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
                planes.append(plane)

        elif curve == "<Ellipse object>":
            dir = e.Curve.Axis
            center = e.Curve.Center
            dim1 = e.Curve.MinorRadius
            dim2 = e.Curve.MajorRadius
            plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
            planes.append(plane)

        elif curve == "<BSplineCurve object>":
            planeParams = spline_wires(e, face)
            if planeParams is not None:
                plane = GeounedSurface(("Plane", planeParams))
                planes.append(plane)

    return planes


def cyl_bound_planes(solidFaces, face, Edges=None):

    if Edges is None:
        Edges = face.OuterWire.Edges
    planes = []

    for e in Edges:
        adjacent_face = other_face_edge(e, face, solidFaces)
        if adjacent_face is not None:
            if type(adjacent_face.Surface) is TorusGu:
                continue  # doesn't create plane if other face is a torus
            if face.Surface.isSameSurface(adjacent_face.Surface):
                continue  # doesn't create plane if other face has same surface
            plane = cyl_edge_plane(face, [e])
            if plane is not None:
                planes.append(plane)

    return planes


def cyl_edge_plane(face, edges):

    planeParams = None
    spline = False
    for edge in edges:
        if type(edge.Curve) is Part.BSplineCurve:
            spline = True
            break

    if spline:
        planeParams = spline_wires(edges, face)
    else:
        edge = edges[0]
        if type(edge.Curve) is Part.Circle:
            dir = edge.Curve.Axis
            center = edge.Curve.Center
            dim1 = edge.Curve.Radius
            dim2 = edge.Curve.Radius
            pos = edge.Curve.value(0)
            u, v = face.__face__.Surface.parameter(pos)
            normal = face.__face__.Surface.normal(u, v)  # always in the same direction F or R
            direction = edge.derivative1At(0)
            direction.normalize()
            vect = direction.cross(normal)
            if edge.Orientation == "Reversed":
                vect = -vect
            if dir.dot(vect) > 0:
                dir = -dir
            planeParams = [center, dir, dim1, dim2]  # toward the center of the cylinder

        elif type(edge.Curve) is Part.Ellipse:
            dir = edge.Curve.Axis
            center = edge.Curve.Center
            dim1 = edge.Curve.MinorRadius
            dim2 = edge.Curve.MajorRadius
            pos = edge.Curve.value(0)
            u, v = face.__face__.Surface.parameter(pos)
            normal = face.__face__.Surface.normal(u, v)
            direction = edge.derivative1At(0)
            direction.normalize()
            vect = direction.cross(normal)
            if edge.Orientation == "Reversed":
                vect = -vect
            if dir.dot(vect) > 0:
                dir = -dir
            planeParams = [center, dir, dim1, dim2]  # toward the center of the cylinder

    if planeParams is not None:
        return GeounedSurface(("Plane", planeParams))


def spline_wires(edges, face):

    zaxis = face.Surface.Axis
    W = Part.Wire(edges)
    majoraxis = get_axis_inertia(W.MatrixOfInertia)

    edge = edges[0]
    p0, p1 = edge.ParameterRange
    pe = 0.5 * (p0 + p1)
    pos = edge.Curve.value(pe)
    u, v = face.__face__.Surface.parameter(pos)
    normal = face.__face__.Surface.normal(u, v)  # Always in the same direction
    direction = edge.derivative1At(pe)
    direction.normalize()

    vect = direction.cross(normal)  # vector forward direction Vs Material
    if edge.Orientation == "Reversed":
        vect = -vect
    lowSide = zaxis.dot(vect) < 0

    rmin = (1e15, None)
    rmax = (-1e15, None)

    for edge in edges:

        if type(edge.Curve) is Part.BSplineCurve:
            for p in edge.Curve.getPoles():
                r = majoraxis.dot(p)
                if rmin[0] > r:
                    rmin = (r, p)
                if rmax[0] < r:
                    rmax = (r, p)
        elif type(edge.Curve) is Part.Line:
            for v in edge.Vertexes:
                r = majoraxis.dot(v.Point)
                if rmin[0] > r:
                    rmin = (r, v.Point)
                if rmax[0] < r:
                    rmax = (r, v.Point)
        else:
            p0, p1 = projection(edge, majoraxis)
            for pi in (p0, p1):
                p = edge.valueAt(pi)
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
                point = rmin - d * vec
            else:
                point = rmax + d * vec
                vec = -vec
        else:
            if lowSide:
                point = rmax + d * vec
                vec = -vec
            else:
                point = rmin - d * vec

        return [point, vec, 1, 1]  # toward the center of the cylinder


def get_axis_inertia(mat):
    inertialMat = numpy.array(((mat.A11, mat.A12, mat.A13), (mat.A21, mat.A22, mat.A23), (mat.A31, mat.A32, mat.A33)))
    eigval, evect = numpy.linalg.eig(inertialMat)

    return FreeCAD.Vector(evect.T[numpy.argmax(eigval)])


def valid_solid(solid, Volume):
    if solid.Volume < 0:
        return False
    Vol_tol = 1e-2
    Vol_area_ratio = 1e-3
    if abs(solid.Volume / solid.Area) < Vol_area_ratio:
        return False
    if abs(solid.Volume) < Vol_tol:
        return False
    return True


def remove_solids(Solids, Volume):

    if len(Solids) == 1:
        try:
            Solids[0] = Solids[0].removeSplitter()
        except:
            pass
        return Solids

    compVol = 0
    for sol in Solids:
        compVol += sol.Volume

    Solids_Clean = []
    for solid in Solids:
        if not valid_solid(solid, Volume):
            logger.warning(f"remove_solids degenerated solids are produced bad dimensions")
            continue
        Solids_Clean.append(solid)

    for i, sol in enumerate(Solids_Clean):
        try:
            Solids_Clean[i] = sol.removeSplitter()
        except:
            Solids_Clean[i] = sol
    return Solids_Clean


def external_plane(plane, Faces):
    Edges = plane.Edges
    for e in Edges:
        adjacent_face = other_face_edge(e, plane, Faces)
        if adjacent_face is None:
            continue
        if isinstance(
            adjacent_face.Surface, PlaneGu
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
        if isinstance(f.Surface, PlaneGu):
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
        if isinstance(adjacent_face.Surface, PlaneGu):
            ncut += 1
        elif region_sign(f, adjacent_face) == "OR":
            ncut += 1
    return ncut


def order_plane_face(Faces, omitfaces):
    counts = []
    face_dict = {}
    for f in Faces:
        if f.Index in omitfaces:
            continue
        if not isinstance(f.Surface, PlaneGu):
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
        if not isinstance(f.Surface, PlaneGu):
            continue

        for e in f.OuterWire.Edges:
            adjacent_face = other_face_edge(e, f, Faces)
            if adjacent_face is None:
                continue
            if type(adjacent_face.Surface) is PlaneGu:
                if abs(abs(adjacent_face.Surface.Axis.dot(f.Surface.Axis)) - 1) < 1e-5:
                    if adjacent_face.Index not in omitfaces:
                        omitfaces.add(f.Index)
                        break


def projection(edge, axis):
    pmin, pmax = edge.ParameterRange
    if type(edge.Curve) == Part.Circle:
        vmin = edge.valueAt(pmin) - edge.Curve.Center
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
            v0 = edge.valueAt(p0) - edge.Curve.Center
            d0 = v0.dot(axis)
            vmax = edge.valueAt(pmax) - edge.Curve.Center
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
        a = edge.Curve.MajorAxis
        b = edge.Curve.MinorAxis
        dx = vx.dot(axis)
        dy = vy.dot(axis)
        if dx == 0:
            p0 = 0.5 * math.pi + pmin
            p1 = p0 + math.pi
        else:
            p0 = math.atan(b * dy / a * dx)
            p1 = p0 + math.pi
        if p1 < pmax:
            return p0, p1
        elif p0 < pmax:
            v0 = edge.valueAt(p0) - edge.Curve.Center
            d0 = v0.dot(axis)
            vmax = edge.valueAt(pmax) - edge.Curve.Center
            dmax = vmax.dot(axis)
            if abs(d0 - dmin) < abs(d0 - dmax):
                return p0, pmax
            else:
                return p0, pmin
        else:
            return pmin, pmax
