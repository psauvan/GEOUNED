import math
import logging

import FreeCAD
import Part

from ..utils.basic_functions_part1 import (
    is_in_line,
    is_opposite,
    is_parallel,
    is_same_value,
)
from ..utils.basic_functions_part2 import is_same_plane
from ..utils.geouned_classes import GeounedSurface
from ..utils.geometry_gu import PlaneGu

logger = logging.getLogger("general_logger")


def gen_plane(face, orientation):
    normal = face.Surface.Axis
    if orientation == "Forward":
        normal = -normal
    pos = face.CenterOfMass
    return GeounedSurface(("Plane", (pos, normal, 1, 1)))


def gen_cylinder(face):
    Axis = face.Surface.Axis
    Center = face.Surface.Center
    Radius = face.Surface.Radius
    return GeounedSurface(("CylinderOnly", (Center, Axis, Radius, 1)))


def gen_cone(face):
    Axis = face.Surface.Axis
    Apex = face.Surface.Apex
    SemiAngle = face.Surface.SemiAngle
    return GeounedSurface(("ConeOnly", (Apex, Axis, SemiAngle, 1, 1)))


def gen_sphere(face):
    Center = face.Surface.Center
    Radius = face.Surface.Radius
    return GeounedSurface(("SphereOnly", (Center, Radius)))


def gen_torus(face, tolerances):
    Center = face.Surface.Center
    Axis = face.Surface.Axis
    MajorRadius = face.Surface.MajorRadius
    MinorRadius = face.Surface.MinorRadius
    if (
        is_parallel(Axis, FreeCAD.Vector(1, 0, 0), tolerances.angle)
        or is_parallel(Axis, FreeCAD.Vector(0, 1, 0), tolerances.angle)
        or is_parallel(Axis, FreeCAD.Vector(0, 0, 1), tolerances.angle)
    ):
        return GeounedSurface(("TorusOnly", (Center, Axis, MajorRadius, MinorRadius)))
    else:
        return None


def cone_apex_plane(cone, orientation, tolerances):
    if (
        is_parallel(cone.Surface.Axis, FreeCAD.Vector(1, 0, 0), tolerances.angle)
        or is_parallel(cone.Surface.Axis, FreeCAD.Vector(0, 1, 0), tolerances.angle)
        or is_parallel(cone.Surface.Axis, FreeCAD.Vector(0, 0, 1), tolerances.angle)
    ):
        return None

    normal = cone.Surface.Axis if orientation == "Forward" else -cone.Surface.Axis
    return GeounedSurface(("Plane", (cone.Surface.Apex, normal, 1, 1)))


def V_torus_surfaces(face, v_params, Surfaces):
    if is_parallel(face.Surface.Axis, FreeCAD.Vector(1, 0, 0), Surfaces.tolerances.tor_angle):
        axis = FreeCAD.Vector(1, 0, 0)
    elif is_parallel(face.Surface.Axis, FreeCAD.Vector(0, 1, 0), Surfaces.tolerances.tor_angle):
        axis = FreeCAD.Vector(0, 1, 0)
    elif is_parallel(face.Surface.Axis, FreeCAD.Vector(0, 0, 1), Surfaces.tolerances.tor_angle):
        axis = FreeCAD.Vector(0, 0, 1)

    p1 = face.valueAt(0.0, v_params[0]) - face.Surface.Center
    z1 = p1.dot(axis)
    d1 = p1.cross(axis).Length

    p2 = face.valueAt(0.0, v_params[1]) - face.Surface.Center
    z2 = p2.dot(axis)
    d2 = p2.cross(axis).Length

    if is_same_value(z1, z2, Surfaces.tolerances.distance):
        center = face.Surface.Center + z1 * axis
        v_mid = (v_params[0] + v_params[1]) * 0.5
        p_mid = face.valueAt(0, v_mid) - face.Surface.Center
        if p_mid.dot(axis) < z1:
            axis = -axis
        return GeounedSurface(("Plane", (center, axis, 1, 1))),None

    elif is_same_value(d1, d2, Surfaces.tolerances.distance) or Surfaces.options.force_cylinder:
        radius = min(d1, d2)
        center = face.Surface.Center
        if is_same_value(d1, face.Surface.MajorRadius, Surfaces.tolerances.distance):
            v_mid = (v_params[0] + v_params[1]) * 0.5
            p_mid = face.valueAt(0, v_mid) - center
            if p_mid.cross(axis).Length < face.Surface.MajorRadius:
                in_surf = True
            v_mid = (v_params[0] + v_params[1]) * 0.5
            p_mid = face.valueAt(0, v_mid) - center
            if p_mid.cross(axis).Length < face.Surface.MajorRadius:
                in_surf = True
                radius = max(d1, d2)
            else:
                in_surf = False
        else:
            if d1 < face.Surface.MajorRadius:
                orientation = "Forward"
                radius = max(d1, d2)
            else:
                orientation = "Reversed"
        return GeounedSurface(("CylinderOnly", (center, axis, radius, 1))) ,orientation
    else:
        za = (z2 * d1 - z1 * d2) / (d1 - d2)
        apex = face.Surface.Center + za * axis
        semi_angle = abs(math.atan(d1 / (z1 - za)))

        cone_axis = axis if (z1 - za) > 0.0 else -axis
        cone = GeounedSurface(("ConeOnly", (apex, cone_axis, semi_angle, 1, 1)))

        v_mid = (v_params[0] + v_params[1]) * 0.5
        p_mid = face.valueAt(0, v_mid) - face.Surface.Center
        z_mid = p_mid.dot(axis)
        d_mid = p_mid.cross(axis).Length

        d_cone = d1 * (z_mid - za) / (z1 - za)
        in_surf = True if d_mid < d_cone else False

        if in_surf:
            orientation = "Forward"
        else:    
            orientation = "Reversed"
            
        #apexPlane = cone_apex_plane(cone, orientation, Surfaces.tolerances)  #apex plane not produced because torus axis along x,y,z
        return cone, orientation
 

def U_torus_planes(face, u_params, Surfaces):

    if is_parallel(face.Surface.Axis, FreeCAD.Vector(1, 0, 0), Surfaces.tolerances.tor_angle):
        axis = FreeCAD.Vector(1, 0, 0)
    elif is_parallel(face.Surface.Axis, FreeCAD.Vector(0, 1, 0), Surfaces.tolerances.tor_angle):
        axis = FreeCAD.Vector(0, 1, 0)
    elif is_parallel(face.Surface.Axis, FreeCAD.Vector(0, 0, 1), Surfaces.tolerances.tor_angle):
        axis = FreeCAD.Vector(0, 0, 1)

    center = face.Surface.Center
    p1 = face.valueAt(u_params[0], 0.0)
    p2 = face.valueAt(u_params[1], 0.0)
    pmid = face.valueAt(0.5 * (u_params[0] + u_params[1]), 0.0)

    if is_same_value(abs(u_params[1] - u_params[0]), math.pi, Surfaces.tolerances.value):
        d = axis.cross(p2 - p1)
        d.normalize()
        if d.dot(pmid - center) < 0:
            d = -d

        return (GeounedSurface(("Plane", (center, d, 1, 1))), )

    elif u_params[1] - u_params[0] < math.pi:
        d = axis.cross(p2 - p1)
        d.normalize()
        if d.dot(pmid - center) < 0:
            d = -d

        return (GeounedSurface(("Plane", (center, d, 1, 1))), )


    else:
        d1 = axis.cross(p1)
        d1.normalize()
        if d1.dot(pmid - center) < 0:
            d1 = -d1

        d2 = axis.cross(p2)
        d2.normalize()
        if d2.dot(pmid - center) < 0:
            d2 = -d2

        plane1 = GeounedSurface(("Plane", (center, d1, 1, 1)))
        plane2 = GeounedSurface(("Plane", (center, d2, 1, 1)))
        return (plane1, plane2)


def gen_plane_sphere(face, solidFaces):
    same_faces = []
    same_faces.append(face)

    for f in solidFaces:
        if f.isEqual(face) or str(f.Surface) != "Sphere":
            continue
        if f.Surface.Center == face.Surface.Center and f.Surface.Radius == face.Surface.Radius:
            # print 'Warning: coincident sphere faces are the same'
            for f2 in same_faces:
                if f.__face__.distToShape(f2.__face__)[0] < 1e-6:
                    same_faces.append(f)
                    break

    # print same_faces
    normal = FreeCAD.Vector(0, 0, 0)
    for f in same_faces:
        normal += f.Area * (f.CenterOfMass - face.Surface.Center)
    normal.normalize()
    tmp_plane = Part.Plane(face.Surface.Center, normal).toShape()

    dmin = 2 * face.Surface.Radius
    for f in same_faces:
        dist = tmp_plane.distToShape(f.__face__)[0]
        dmin = min(dmin, dist)

    if dmin > 1e-6:
        center = face.Surface.Center + 0.95 * dmin * normal
        plane = GeounedSurface(("Plane", (center, normal, 1, 1)))
    else:
        return None


def gen_plane_cylinder(face, solidFaces, tolerances):

    surf = face.Surface
    rad = surf.Radius

    if str(surf) != "<Cylinder object>":
        return None

    my_index = face.Index
    face_index = [my_index]

    for face2 in solidFaces:
        if face2.Area < tolerances.min_area:
            logger.warning(
                f"surface {str(surf)} removed from cell definition. Face area < Min area ({face2.Area} < {tolerances.min_area})"
            )
            continue
        if str(face2.Surface) == "<Cylinder object>" and face2.Index != face.Index:
            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and face2.Surface.Radius == rad
                and is_in_line(face2.Surface.Center, face.Surface.Axis, face.Surface.Center)
            ):
                # print 'Warning: coincident cylinder faces are the same'
                face_index.append(face2.Index)

    u_min, u_max = get_u_value_boundary(solidFaces, face_index, my_index)
    if u_min is None:
        return None

    u_1, i1 = u_min
    u_2, i2 = u_max

    v_1 = solidFaces[i1].ParameterRange[2]
    v_2 = solidFaces[i2].ParameterRange[2]

    p1 = solidFaces[i1].valueAt(u_1, v_1)
    p2 = solidFaces[i2].valueAt(u_2, v_2)


    if p1.isEqual(p2, 1e-5):
        logger.error("Error in the additional place definition")
        return None

    normal = p2.sub(p1).cross(face.Surface.Axis)
    normal.normalize()
    if normal.dot(face.CenterOfMass - p1) < 0:
        normal = -normal

    return GeounedSurface(("Plane", (p1, normal, 1, 1)))


def gen_plane_cone(face, solidFaces, tolerances):

    Surf = face.Surface
    if str(Surf) != "<Cone object>":
        return None

    myIndex = solidFaces.index(face)
    face_index = [myIndex]

    for face2 in solidFaces:
        if face2.Area < tolerances.min_area:
            logger.warning(
                f"{str(Surf)} surface removed from cell definition. Face area < Min area ({face2.Area} < {tolerances.min_area})"
            )
            continue
        if str(face2.Surface) == "<Cone object>" and not (face2.isEqual(face)):
            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and face2.Surface.Apex.isEqual(face.Surface.Apex, 1e-5)
                and (face2.Surface.SemiAngle - face.Surface.SemiAngle) < 1e-6
            ):
                face_index.append(face2.Index)

    u_min, u_max = get_u_value_boundary(solidFaces, face_index, myIndex)
    if u_min is None:
        return None

    u_1, i1 = u_min
    u_2, i2 = u_max

    v_1 = solidFaces[i1].ParameterRange[2]
    v_2 = solidFaces[i2].ParameterRange[2]

    p1 = solidFaces[i1].valueAt(u_1, v_1)
    p2 = solidFaces[i2].valueAt(u_2, v_2)

    if p1.isEqual(p2, 1e-5):
        logger.error("in the additional place definition")
        return None

    v1 = p1 - face.Surface.Apex
    v2 = p2 - face.Surface.Apex
    normal = v1.cross(v2)
    normal.normalize()
    if normal.dot(face.CenterOfMass - face.Surface.Apex) < 0:
        normal = -normal

    return GeounedSurface(("Plane", (face.Surface.Apex, normal, 1, 1)))


def get_u_value_boundary(solidFaces, face_index, my_index):

    face_u_ranges, closed_face = get_closed_ranges(solidFaces, face_index)
    if closed_face:
        return None, None

    for face_u_range in face_u_ranges:
        if my_index in face_u_range[2]:
            u_min, u_max = face_u_range[0:2]
            return u_min, u_max


def get_closed_ranges(solidFaces, face_index):

    u_nodes = []
    for index in face_index:
        URange = solidFaces[index].ParameterRange
        u_nodes.append((URange[0], index))
        u_nodes.append((URange[1], index))
    u_nodes.sort()

    closed_range = get_intervals(u_nodes)

    a_min = closed_range[0][0][0]
    a_max = closed_range[-1][1][0]

    if abs(a_max - a_min - 2.0 * math.pi) < 1e-2:
        if len(closed_range) == 1:
            closed_face = True
        else:
            endPoint = (closed_range[-1][0][0] - 2 * math.pi, closed_range[-1][0][1])
            closed_range[0][0] = endPoint
            closed_range[0][2].update(closed_range[-1][2])
            del closed_range[-1]

            if len(closed_range) == 1:
                if abs(closed_range[0][1][0] - closed_range[0][0][0] - 2.0 * math.pi) < 1e-2:
                    closed_face = True
                else:
                    closed_face = False
            else:
                closed_face = False
    else:
        closed_face = False
    return closed_range, closed_face


def get_intervals(u_nodes):
    closed_ranges = []
    pos_min = dict()
    pos_max = dict()
    for i, node in enumerate(u_nodes):
        if node[1] not in pos_min.keys():
            pos_min[node[1]] = i
        else:
            pos_max[node[1]] = i

    u_min = u_nodes[0]
    i_pos = pos_max[u_min[1]]

    while True:
        x = u_nodes[i_pos]
        end = True
        for i in range(i_pos + 1, len(u_nodes)):
            mxt_int = u_nodes[i][1]
            if (
                u_nodes[pos_min[mxt_int]][0] - x[0]
            ) < 1e-5:  # x pos is > min boundary of the next inteval inside precision 1e-5
                i_pos = pos_max[mxt_int]
                end = False
                break

        if end:
            u_max = x
            closed_ranges.append([u_min, u_max])
            i_pos += 1
            if i_pos < len(u_nodes):
                u_min = u_nodes[i_pos]
                i_pos = pos_max[u_min[1]]
            else:
                break

    for closed_range in closed_ranges:
        index = set()
        xmin = closed_range[0][0]
        xmax = closed_range[1][0]
        for interval in u_nodes:
            x = interval[0]
            if (xmin - x) < 1.0e-5 and (x - xmax) < 1.0e-5:
                index.add(interval[1])
        closed_range.append(index)

    return closed_ranges

def omit_multiplane_repeated_planes(mp_region,Surfaces,Faces):
    repeated_planes = set()
    planes = mp_region.region.get_surfaces_numbers()
    for p in planes :
        pg = Surfaces.primitive_surfaces.get_surface(p)
        for face in Faces:
            if not isinstance(face,PlaneGu):
                continue
            if is_same_plane(face.Surface, pg.Surf, Surfaces.options, Surfaces.tolerances, Surfaces.numeric_format):
                repeated_planes.add(face.Index)
    return repeated_planes                


   