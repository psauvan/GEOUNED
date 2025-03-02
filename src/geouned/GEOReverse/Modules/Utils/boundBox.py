import FreeCAD
import Part
import math
from .booleanFunction import BoolSequence

twoPi = math.pi * 2


class solid_plane:
    def __init__(self, NTcell=None):
        if NTcell is None:
            self.Planes = None
            self.definition = None
        else:    
            plane_dict, surf_to_plane_dict = quadric_to_plane(NTcell.surfaces)
            self.planes = plane_dict
            self.definition = plane_definition(NTcell.definition.copy(), surf_to_plane_dict)
        
    def isInside(self, point):
        surf_value = dict()
        for p_index, p in self.planes.items():
            normal, pointPlane = p.Axis, p.Position
            pt = FreeCAD.Vector(point.X, point.Y, point.Z)
            r = pt - pointPlane
            dot = normal.dot(r)
            if abs(dot) < 1e-2:
                surf_value[p_index] = None
            else:
                surf_value[p_index] = dot > 0  # assume inside even outside close to th surface
        inside = self.definition.evaluate(surf_value)
        if inside is None:
            return True  # if result is not means point on surface => inside
        else:
            return inside

    def get_boundBox(self):
        if self.definition.operator == "OR" and self.definition.level > 0:
            bBox = FreeCAD.BoundBox()
            for definition in self.definition.elements:
                compsol = solid_plane()
                compsol.definition = definition
                comp_planes=dict()
                for p in compsol.definition.get_surfaces_numbers():
                    comp_planes[p] = self.planes[p]
                compsol.planes = comp_planes
                bBox.add(compsol.get_component_boundBox())
            return bBox
        else:
            return self.get_component_boundBox()    
    
    def get_component_boundBox(self):
        axis_list = ("x", "y", "z")
        point_list = plane_intersect(tuple(self.planes.values()))

        box_lim = []
        for axis in axis_list:
            s_point = sort_point(point_list, axis)
            for point in s_point:
                if self.isInside(point):
                    box_lim.append(pointaxis(point, axis))
                    break
            s_point = reversed(remove_points(s_point, pointaxis(point, axis), axis, True))

            for point in s_point:
                if self.isInside(point):
                    box_lim.append(pointaxis(point, axis))
                    break
            point_list = remove_points(s_point, pointaxis(point, axis), axis, False)

        return FreeCAD.BoundBox(box_lim[0], box_lim[2], box_lim[4], box_lim[1], box_lim[3], box_lim[5])


def quadric_to_plane(surfaces):

    surf_planes_dict = dict()
    planes = dict()
    surf_index = list(surfaces.keys())
    surf_index.sort()
    next_index = surf_index[-1] + 1

    for s_index, s in surfaces.items():
        if s.type == "plane":
            normal, d = s.params
            position = normal * d
            planes[s_index] = Part.Plane(position, normal)
        else:
            surf_planes = convert_to_planes(s)
            p_index = []
            for p in surf_planes:
                planes[next_index] = p
                p_index.append(next_index)
                next_index += 1
            surf_planes_dict[s_index] = p_index
    return planes, surf_planes_dict


def convert_to_planes(s):
    if s.type == "cylinder":
        return cylinder_to_planes(s)
    elif s.type == "cone":
        return cone_to_planes(s)
    elif s.type == "sphere":
        return sphere_to_planes(s)
    elif s.type == "torus":
        return torus_to_planes(s)


def get_orto_axis(axis):
    x = FreeCAD.Vector(1, 0, 0)
    z = FreeCAD.Vector(0, 0, 1)
    vx = axis.cross(x)
    vz = axis.cross(z)
    if vx.Length < vz.Length:
        v = vz
    else:
        v = vx
    v.normalize()
    w = v.cross(axis)
    w.normalize()

    return v, w


def cylinder_to_planes(cyl):
    center, axis, radius = cyl.params
    x, y = get_orto_axis(axis)
    r1 = center + x * radius
    r2 = center - x * radius
    r3 = center + y * radius
    r4 = center - y * radius

    p1 = Part.Plane(r1, -x)
    p2 = Part.Plane(r2, x)
    p3 = Part.Plane(r3, -y)
    p4 = Part.Plane(r4, y)
    return (p1, p2, p3, p4)


def cone_to_planes(cone):
    apex, axis, t, dbl = cone.params
    sa = math.atan(t)
    nface = 4
    x, y = get_orto_axis(axis)
    cs = math.cos(sa)
    ss = math.sin(sa)
    dphi = twoPi / nface
    phi = 0
    cplanes = []
    for i in range(nface):
        rho = x * math.cos(phi) + y * math.sin(phi)
        ni = -axis * ss + rho * cs
        pi = Part.Plane(apex, -ni)
        cplanes.append(pi)
        phi += dphi
    pa = Part.Plane(apex, axis)
    cplanes.append(pa)
    return cplanes


def sphere_to_planes(sphere):
    center, radius = sphere.params
    x = FreeCAD.Vector(1, 0, 0)
    y = FreeCAD.Vector(0, 1, 0)
    z = FreeCAD.Vector(0, 0, 1)

    r1 = center + x * radius
    r2 = center - x * radius
    r3 = center + y * radius
    r4 = center - y * radius
    r5 = center + z * radius
    r6 = center - z * radius

    p1 = Part.Plane(r1, -x)
    p2 = Part.Plane(r2, x)
    p3 = Part.Plane(r3, -y)
    p4 = Part.Plane(r4, y)
    p5 = Part.Plane(r5, -z)
    p6 = Part.Plane(r6, z)
    return (p1, p2, p3, p4, p5, p6)


def torus_to_planes(torus):
    center, axis, majorRadius, minor1, minor2 = torus.params
    minorRadius = max(minor1, minor2)

    x = FreeCAD.Vector(1, 0, 0)
    y = FreeCAD.Vector(0, 1, 0)
    z = FreeCAD.Vector(0, 0, 1)

    dist = majorRadius + minorRadius
    if abs(abs(axis.dot(x)) - 1) < 1e-5:
        r1 = center + x * minorRadius
        r2 = center - x * minorRadius
        r3 = center + y * dist
        r4 = center - y * dist
        r5 = center + z * dist
        r6 = center - z * dist
    elif abs(abs(axis.dot(y)) - 1) < 1e-5:
        r1 = center + x * dist
        r2 = center - x * dist
        r3 = center + y * minorRadius
        r4 = center - y * minorRadius
        r5 = center + z * dist
        r6 = center - z * dist
    elif abs(abs(axis.dot(z)) - 1) < 1e-5:
        r1 = center + x * dist
        r2 = center - x * dist
        r3 = center + y * dist
        r4 = center - y * dist
        r5 = center + z * minorRadius
        r6 = center - z * minorRadius

    p1 = Part.Plane(r1, -x)
    p2 = Part.Plane(r2, x)
    p3 = Part.Plane(r3, -y)
    p4 = Part.Plane(r4, y)
    p5 = Part.Plane(r5, -z)
    p6 = Part.Plane(r6, z)
    return (p1, p2, p3, p4, p5, p6)


def plane_definition(seq, surf_index):
    for s, planes in surf_index.items():
        pm = BoolSequence(" ".join((str(p) for p in planes)))
        pp = BoolSequence(":".join((str(-p) for p in planes)))

        change_surf(seq, -s, pm)
        change_surf(seq, s, pp)

    return seq


def change_surf(seq, old, new):
    for i, e in enumerate(seq.elements):
        if type(e) is BoolSequence:
            change_surf(e, old, new)
        else:
            if e == old:
                seq.elements[i] = new
    seq.join_operators()


def plane_intersect(plane_list):
    point_list = []
    for i, p1 in enumerate(plane_list[0:-2]):
        j = i + 1
        for p2 in plane_list[i + 1 : -1]:
            line = p1.intersect(p2)
            if len(line) == 0:
                continue
            line = line[0]
            for p3 in plane_list[j + 1 :]:
                inter = line.intersect(p3)
                if len(inter[0]) == 0:
                    continue
                point_list.append(inter[0][0])
            j += 1
    return point_list


def sort_point(point_list, axis):
    axis_points = []
    if axis == "x":
        for i, point in enumerate(point_list):
            axis_points.append((point.X, i))
    elif axis == "y":
        for i, point in enumerate(point_list):
            axis_points.append((point.Y, i))
    elif axis == "z":
        for i, point in enumerate(point_list):
            axis_points.append((point.Z, i))
    else:
        print("bad axis name")

    axis_points.sort()
    sorted_points = (point_list[x[1]] for x in axis_points)
    return sorted_points


def remove_points(point_list, value, axis, lower):
    if axis == "x":
        if lower:
            kept_points = filter(lambda p: p.X >= value, point_list)
        else:
            kept_points = filter(lambda p: p.X <= value, point_list)
    elif axis == "y":
        if lower:
            kept_points = filter(lambda p: p.Y >= value, point_list)
        else:
            kept_points = filter(lambda p: p.Y <= value, point_list)
    elif axis == "z":
        if lower:
            kept_points = filter(lambda p: p.Z >= value, point_list)
        else:
            kept_points = filter(lambda p: p.Z <= value, point_list)
    else:
        print("bad axis name")
    return list(kept_points)


def pointaxis(p, axis):
    if axis == "x":
        return p.X
    elif axis == "y":
        return p.Y
    elif axis == "z":
        return p.Z
