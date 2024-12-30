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
from ..utils.boolean_function import BoolRegion
from ..utils.functions import GeounedSurface

logger = logging.getLogger("general_logger")

def auxillary_plane(plane,Surfaces):
    pid,exist = Surfaces.primitive_surfaces.add_plane(plane,True)  
    if exist:
        p = Surfaces.primitive_surfaces.get_surface(pid)     
        if is_opposite(plane.Surf.Axis, p.Surf.Axis, Surfaces.tolerances.pln_angle):
            pid = -pid  
    return BoolRegion(0,str(pid))        

def gen_plane(face, orientation):
    normal = face.Surface.Axis
    if orientation == "Forward":
            normal = -normal
    pos = face.CenterOfMass
    return GeounedSurface(("Plane", (pos, normal, 1, 1)), None)
    
def gen_cylinder(face):
    Axis = face.Surface.Axis
    Center = face.Surface.Center
    Radius = face.Surface.Radius
    return GeounedSurface(("Cylinder", (Center, Axis, Radius, 1)), None)    

def gen_cone(face):
    Axis = face.Surface.Axis
    Apex = face.Surface.Apex
    SemiAngle = face.Surface.SemiAngle
    return GeounedSurface(("Cone", (Apex, Axis, SemiAngle, 1, 1)), None)

def gen_sphere(face):
    Center = face.Surface.Center
    Radius = face.Surface.Radius
    return GeounedSurface(("Sphere", (Center, Radius)), None)

def gen_torus(face,tolerances):
    Center = face.Surface.Center
    Axis = face.Surface.Axis
    MajorRadius = face.Surface.MajorRadius
    MinorRadius = face.Surface.MinorRadius
    if (
        is_parallel(Axis, FreeCAD.Vector(1, 0, 0), tolerances.angle) or
        is_parallel(Axis, FreeCAD.Vector(0, 1, 0), tolerances.angle) or
        is_parallel(Axis, FreeCAD.Vector(0, 0, 1), tolerances.angle)
        ):
        return GeounedSurface(("Torus", (Center, Axis, MajorRadius, MinorRadius)), None)
    else:
        return None
    
def cone_apex_plane(cone,orientation,tolerances):
    if (
        is_parallel(cone.Surf.Axis, FreeCAD.Vector(1, 0, 0), tolerances.angle)
        or is_parallel(cone.Surf.Axis, FreeCAD.Vector(0, 1, 0), tolerances.angle)
        or is_parallel(cone.Surf.Axis, FreeCAD.Vector(0, 0, 1), tolerances.angle)
    ):
        return None
    
    normal = cone.Axis if orientation == "Forward" else -cone.Axis
    return GeounedSurface(("Plane", (cone.Apex, normal, 1, 1)),None)


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
        plane = GeounedSurface(("Plane", (center, axis, 1, 1)),None)
        return auxillary_plane(plane, Surfaces)

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
                in_surf = True
                radius = max(d1, d2)
            else:
                in_surf = False

        cylinder = GeounedSurface(("Cylinder", (center, axis, radius, 1)),None)
        pid,exist = Surfaces.primitive_surfaces.add_cylinder(cylinder,True)
        if in_surf :
            pid = -pid

        return BoolRegion(0,str(pid))

    else:
        surf_type = "Cone"
        za = (z2 * d1 - z1 * d2) / (d1 - d2)
        apex = face.Surface.Center + za * axis
        semi_angle = abs(math.atan(d1 / (z1 - za)))

        cone_axis = axis if (z1 - za) > 0.0 else -axis

        v_mid = (v_params[0] + v_params[1]) * 0.5
        p_mid = face.valueAt(0, v_mid) - face.Surface.Center
        z_mid = p_mid.dot(axis)
        d_mid = p_mid.cross(axis).Length

        d_cone = d1 * (z_mid - za) / (z1 - za)
        in_surf = True if d_mid < d_cone else False

        cone = GeounedSurface(("Cone", (apex, cone_axis, semi_angle, 1, 1)),None)
        pid,exist = Surfaces.primitive_surfaces.add_cone(cone)   

        if in_surf :
            pid = -pid
            aux_plane_region = cone_apex_plane(cone,"Forward",Surfaces.tolerances)
            aux_cone_region = BoolRegion(0,str(pid))
            if aux_plane_region:
                return aux_cone_region * aux_plane_region
            else:
                return aux_cone_region
        else:    
            aux_plane_region = cone_apex_plane(cone,"Reversed",Surfaces.tolerances)
            aux_cone_region = BoolRegion(0,str(pid))
            if aux_plane_region:
                return aux_cone_region + aux_plane_region
            else:
                return aux_cone_region

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

        plane = GeounedSurface(("Plane", (center, d, 1, 1)),None)
        return auxillary_plane(plane, Surfaces)
        

    elif u_params[1] - u_params[0] < math.pi:
        d = axis.cross(p2 - p1)
        d.normalize()
        if d.dot(pmid - center) < 0:
            d = -d

        plane = GeounedSurface(("Plane", (center, d, 1, 1)),None)
        return auxillary_plane(plane, Surfaces)

    else:
        d1 = axis.cross(p1)
        d1.normalize()
        if d1.dot(pmid - center) < 0:
            d1 = -d1

        d2 = axis.cross(p2)
        d2.normalize()
        if d2.dot(pmid - center) < 0:
            d2 = -d2

        plane1 = GeounedSurface(("Plane", (center, d1, 1, 1)),None)
        plane2 = GeounedSurface(("Plane", (center, d2, 1, 1)),None)
        return auxillary_plane(plane1, Surfaces) + auxillary_plane(plane2, Surfaces)


def is_inverted(solid):

    face = solid.Faces[0]

    # u=(face.Surface.bounds()[0]+face.Surface.bounds()[1])/2.0 # entre 0 y 2pi si es completo
    # v=face.Surface.bounds()[0]+(face.Surface.bounds()[3]-face.Surface.bounds()[2])/3.0 # a lo largo del eje
    parameter_range = face.ParameterRange
    u = (parameter_range[1] + parameter_range[0]) / 2.0
    v = (parameter_range[3] + parameter_range[2]) / 2.0

    if isinstance(face.Surface,Part.Cylinder) :
        dist1 = face.Surface.value(u, v).distanceToLine(face.Surface.Center, face.Surface.Axis)
        dist2 = (
            face.Surface.value(u, v)
            .add(face.Surface.normal(u, v).multiply(1.0e-6))
            .distanceToLine(face.Surface.Center, face.Surface.Axis)
        )
        if (dist2 - dist1) < 0.0:
            # The normal of the cylinder is going inside
            return True
        
    elif isinstance(face.Surface,Part.Cone):
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
    elif isinstance(face.Surface,Part.Sphere):
        # radii = point - center
        radii = face.Surface.value(u, v).add(face.Surface.Center.multiply(-1))
        radii_b = face.Surface.value(u, v).add(face.Surface.normal(u, v).multiply(1.0e-6)).add(face.Surface.Center.multiply(-1))
        # radii_b  = radii.add( face.Surface.normal(u,v).multiply(1.0e-6) )
        if (radii_b.Length - radii.Length) < 0.0:
            # An increasing of the radii vector in the normal direction decreases the radii: oposite normal direction
            return True

    elif isinstance(face.Surface,Part.Plane):
        dist1 = face.CenterOfMass.distanceToPoint(solid.BoundBox.Center)
        dist2 = face.CenterOfMass.add(face.normalAt(u, v).multiply(1.0e-6)).distanceToPoint(solid.BoundBox.Center)
        point2 = face.CenterOfMass.add(face.normalAt(u, v).multiply(1.0e-6))
        if solid.isInside(point2, 1e-7, False):
            return True

    return False   

def gen_plane_sphere(face, solid, Surfaces):
    same_faces = []
    same_faces.append(face)

    for f in solid.Faces:
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
        plane = GeounedSurface(("Plane", (center, normal, 1, 1)),None)
        return auxillary_plane(plane, Surfaces)
    else:
        return None


def gen_plane_cylinder(face, solid, Surfaces):

    surf = face.Surface
    rad = surf.Radius

    if str(surf) != "<Cylinder object>":
        return None

    my_index = solid.Faces.index(face)
    face_index = [my_index]

    for i, face2 in enumerate(solid.Faces):
        if face2.Area < Surfaces.tolerances.min_area:
            logger.warning(
                f"surface {str(surf)} removed from cell definition. Face area < Min area ({face2.Area} < {Surfaces.tolerances.min_area})"
            )
            continue
        if str(face2.Surface) == "<Cylinder object>" and not (face2.isEqual(face)):
            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and face2.Surface.Radius == rad
                and is_in_line(face2.Surface.Center, face.Surface.Axis, face.Surface.Center)
            ):
                # print 'Warning: coincident cylinder faces are the same'
                face_index.append(i)

    u_min, u_max = get_u_value_boundary(solid, face_index, my_index)
    if u_min is None:
        return None

    u_1, i1 = u_min
    u_2, i2 = u_max

    v_1 = solid.Faces[i1].ParameterRange[2]
    v_2 = solid.Faces[i2].ParameterRange[2]

    p1 = solid.Faces[i1].valueAt(u_1, v_1)
    p2 = solid.Faces[i2].valueAt(u_2, v_2)

    if p1.isEqual(p2, 1e-5):
        logger.error("Error in the additional place definition")
        return None

    normal = p2.sub(p1).cross(face.Surface.Axis)
    normal.normalize()
    if normal.dot(face.CenterOfMass-p1) < 0 :
        normal = -normal

    plane = GeounedSurface(("Plane", (p1, normal, 1, 1)),None)
    return auxillary_plane(plane, Surfaces)

def gen_plane_cone(face, solid, Surfaces):

    Surf = face.Surface
    if str(Surf) != "<Cone object>":
        return None

    myIndex = solid.Faces.index(face)
    face_index = [myIndex]

    for i, face2 in enumerate(solid.Faces):
        if face2.Area < Surfaces.tolerances.min_area:
            logger.warning(
                f"{str(Surf)} surface removed from cell definition. Face area < Min area ({face2.Area} < {Surfaces.tolerances.min_area})"
            )
            continue
        if str(face2.Surface) == "<Cone object>" and not (face2.isEqual(face)):
            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and face2.Surface.Apex.isEqual(face.Surface.Apex, 1e-5)
                and (face2.Surface.SemiAngle - face.Surface.SemiAngle) < 1e-6
            ):
                face_index.append(i)

    u_min, u_max = get_u_value_boundary(solid, face_index, myIndex)
    if u_min is None:
        return None

    u_1, i1 = u_min
    u_2, i2 = u_max

    v_1 = solid.Faces[i1].ParameterRange[2]
    v_2 = solid.Faces[i2].ParameterRange[2]

    p1 = solid.Faces[i1].valueAt(u_1, v_1)
    p2 = solid.Faces[i2].valueAt(u_2, v_2)

    if p1.isEqual(p2, 1e-5):
        logger.error("in the additional place definition")
        return None

    v1 = p1-face.Surface.Apex
    v2 = p2-face.Surface.Apex
    normal = v1.cross(v2)
    normal.normalize()
    if normal.dot(face.CenterOfMass-face.Surface.Apex) < 0:
        normal = -normal

    plane = GeounedSurface(("Plane", (face.Surface.Apex, normal, 1, 1)),None)
    return auxillary_plane(plane, Surfaces)

def get_u_value_boundary(solid, face_index, my_index):

    face_u_ranges, closed_face = get_closed_ranges(solid, face_index)
    if closed_face:
        return None, None

    for face_u_range in face_u_ranges:
        if my_index in face_u_range[2]:
            u_min, u_max = face_u_range[0:2]
            return u_min, u_max  
        
def get_closed_ranges(solid, face_index):

    u_nodes = []
    for index in face_index:
        URange = solid.Faces[index].ParameterRange
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