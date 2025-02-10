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
            planeParams = plane_spline_curve(e, face)
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

    try:
        curve = str(edges[0].Curve)
    except:
        curve = "none"

    planeParams = None
    if curve[0:6] == "Circle":
        dir = edges[0].Curve.Axis
        center = edges[0].Curve.Center
        dim1 = edges[0].Curve.Radius
        dim2 = edges[0].Curve.Radius
        pos = edges[0].Curve.value(0)
        u, v = face.__face__.Surface.parameter(pos)
        normal = face.__face__.normalAt(u, v)
        direction = edges[0].derivative1At(0)
        direction.normalize()
        vect = direction.cross(normal)
        if edges[0].Orientation == "Reversed":
            vect = -vect
        if dir.dot(vect) > 0:
            dir = -dir
        planeParams = [center, dir, dim1, dim2]

    elif curve == "<Ellipse object>":
        dir = edges[0].Curve.Axis
        center = edges[0].Curve.Center
        dim1 = edges[0].Curve.MinorRadius
        dim2 = edges[0].Curve.MajorRadius
        pos = edges[0].Curve.value(0)
        u, v = face.__face__.Surface.parameter(pos)
        normal = face.__face__.normalAt(u, v)
        direction = edges[0].derivative1At(0)
        direction.normalize()
        vect = direction.cross(normal)
        if edges[0].Orientation == "Reversed":
            vect = -vect
        if dir.dot(vect) > 0:
            dir = -dir
        planeParams = [center, dir, dim1, dim2]

    elif curve == "<BSplineCurve object>":
        planeParams = plane_spline_curve(edges, face)

    if planeParams is not None:
        if face.Orientation == "Reversed":
            planeParams[1] = -planeParams[1]
        return GeounedSurface(("Plane", planeParams))


def plane_spline_curve(edges, face):
    if spline_1D(edges[0]):
        return None

    zaxis = face.Surface.Axis

    pos = edges[0].Curve.value(0)
    u, v = face.__face__.Surface.parameter(pos)
    normal = face.__face__.normalAt(u, v)
    direction = edges[0].derivative1At(0)
    direction.normalize()

    vect = direction.cross(normal)  # vector forward direction Vs Material
    if edges[0].Orientation == "Reversed":
        vect = -vect

    lowSide = zaxis.dot(vect) < 0
    W = Part.Wire(edges)
    majoraxis = get_axis_inertia(W.MatrixOfInertia)

    if spline_2D(edges[0]):
        return (edges[0].valueAt(0), majoraxis, 1, 1)
    else:
        rmin = (1e15, None)
        rmax = (-1e15, None)
        for e in edges:
            for p in e.Curve.getPoles():
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

        return [point, vec, 1, 1]


def get_axis_inertia(mat):
    inertialMat = numpy.array(((mat.A11, mat.A12, mat.A13), (mat.A21, mat.A22, mat.A23), (mat.A31, mat.A32, mat.A33)))
    eigval, evect = numpy.linalg.eig(inertialMat)

    return FreeCAD.Vector(evect.T[numpy.argmax(eigval)])


def valid_solid(solid):
    Vol_tol = 1e-2
    Vol_area_ratio = 1e-3
    if abs(solid.Volume / solid.Area) < Vol_area_ratio:
        return False
    if abs(solid.Volume) < Vol_tol:
        return False
    return True


def remove_solids(Solids):

    if len(Solids) == 1:
        Solids[0] = Solids[0].removeSplitter()
        return Solids

    Solids_Clean = []
    for solid in Solids:
        if not valid_solid(solid):
            logger.warning(f"remove_solids degenerated solids are produced bad dimensions")
            continue
        Solids_Clean.append(solid.removeSplitter())

    return Solids_Clean


def external_plane(plane, Faces):
    Edges = plane.Edges
    for e in Edges:
        adjacent_face = other_face_edge(e, plane, Faces)
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
