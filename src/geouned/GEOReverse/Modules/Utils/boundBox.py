import FreeCAD
import Part
import math
import numpy
from .booleanFunction import BoolSequence
from ..data_class import BoxSettings

twoPi = math.pi * 2


class solid_plane_box:
    def __init__(self, NTCell = None, outbox=None):
        if NTCell is None:
            self.Planes = None
            self.definition = None
            self.surf_to_plane = None
            self.insolid_tolerance = BoxSettings().insolid_tolerance
            self.universe_radius = BoxSettings().universe_radius
            self.max_solid_length = BoxSettings().max_solid_length
            self.universe_center = FreeCAD.Vector(0, 0, 0)
        else:
            plane_dict, surf_to_plane_dict = quadric_to_plane(NTCell.definition,NTCell.surfaces)
            self.planes = plane_dict
            self.surf_to_plane = surf_to_plane_dict
            self.definition = plane_definition(NTCell.definition.copy(), surf_to_plane_dict)
            self.insolid_tolerance = NTCell.settings.insolid_tolerance
            self.universe_radius = NTCell.settings.universe_radius
            self.max_solid_length = NTCell.settings.max_solid_length
            self.universe_center = FreeCAD.Vector(0, 0, 0)

        if outbox:
            self.universe_radius = outbox.DiagonalLength * 0.5
            self.universe_center = outbox.Center

    def export_surf_planes(self, box):

        surf = set(self.surf_to_plane.keys())
        surf_planes = set()
        for s in surf:
            planes = []
            for p in self.surf_to_plane[s]:
                if p not in self.surf_to_plane.keys():
                    break
                normal, position = self.planes[p].Axis, self.planes[p].Position
                planes.append(makePlane(normal, position, box))
                surf_planes.add(p)
            else:
                compsurf = Part.Compound(planes)
                if compsurf is not None:
                    compsurf.exportStep(f"psurf_{s}.stp")

        all_planes = set(self.planes.keys())
        for p in all_planes - surf_planes:
            normal, position = self.planes[p].Axis, self.planes[p].Position
            compsurf = makePlane(normal, position, box)
            if compsurf is not None:
                compsurf.exportStep(f"psurf_{p}.stp")

    def isInside(self, point):
        surf_value = dict()
        for p_index, p in self.planes.items():
            normal, pointPlane = p.Axis, p.Position
            pt = FreeCAD.Vector(point.x, point.y, point.z)
            r = pt - pointPlane
            dot = normal.dot(r)
            if abs(dot) < self.insolid_tolerance:
                surf_value[p_index] = None  # undefined value for point close to the surface
            else:
                surf_value[p_index] = dot > 0
        inside = self.definition.evaluate(surf_value)

        # if point close to the surface assume inside the solid independently if inside or outside
        return True if inside is None else inside

    def get_boundBox(self, enlarge=0):
        if self.definition.operator == "OR" and self.definition.level > 0:
            bBox = FreeCAD.BoundBox()
            for definition in self.definition.elements:
                compsol = solid_plane_box()
                compsol.universe_center = self.universe_center
                compsol.universe_radius = self.universe_radius
                compsol.max_solid_length = self.max_solid_length
                compsol.definition = definition
                comp_planes = dict()
                for p in compsol.definition.get_surfaces_numbers():
                    if p in self.planes.keys():
                        comp_planes[p] = self.planes[p]
                compsol.planes = comp_planes
                compsol.surf_to_plane = self.surf_to_plane
                compBox = compsol.get_component_boundBox()
                if compBox is not None:
                    bBox.add(compBox)
        elif self.definition.operator == "AND" and self.definition.level > 0:
            bBox = None
            for definition in self.definition.elements:
                compsol = solid_plane_box()
                compsol.universe_center = self.universe_center
                compsol.universe_radius = self.universe_radius
                compsol.max_solid_length = self.max_solid_length
                compsol.definition = definition
                comp_planes = dict()
                for p in compsol.definition.get_surfaces_numbers():
                    if p in self.planes.keys():
                        comp_planes[p] = self.planes[p]
                compsol.planes = comp_planes
                compsol.surf_to_plane = self.surf_to_plane
                compBox = compsol.get_component_boundBox()
                if compBox is not None:
                    if bBox is not None:
                        bBox.intersected(compBox)
                    else:
                        bBox = compBox    
        else:
            bBox = self.get_component_boundBox()
            if bBox is None:
                bBox = FreeCAD.BoundBox()

        if enlarge > 0:
            dx = (bBox.XMax - bBox.XMin) * 0.5 * (1 + enlarge)
            dy = (bBox.YMax - bBox.YMin) * 0.5 * (1 + enlarge)
            dz = (bBox.ZMax - bBox.ZMin) * 0.5 * (1 + enlarge)
            x0 = 0.5 * (bBox.XMax + bBox.XMin)
            y0 = 0.5 * (bBox.YMax + bBox.YMin)
            z0 = 0.5 * (bBox.ZMax + bBox.ZMin)
            return FreeCAD.BoundBox(x0 - dx, y0 - dy, z0 - dz, x0 + dx, y0 + dy, z0 + dz)
        else:
            return bBox

    def get_component_boundBox(self):
        axis_list = ("x", "y", "z")
        point_list = plane_intersect(tuple(self.planes.values()), self.universe_radius, self.universe_center)
#        inertia_matrix(point_list)
        box_lim = []
        if point_list == []:
            return None
        
        for axis in axis_list:
            s_point = sort_point(point_list, axis)
            if s_point == []:
                return None
            for point in s_point:
                if self.isInside(point):
                    box_lim.append(pointaxis(point, axis))
                    break      
            
            s_point = remove_points(s_point, pointaxis(point, axis), axis, True, self.max_solid_length)
            if s_point == []:
                return None
            for point in s_point:
                if self.isInside(point):
                    box_lim.append(pointaxis(point, axis))
                    break
            point_list = remove_points(s_point, pointaxis(point, axis), axis, False)

        if len(box_lim) == 6:
            return FreeCAD.BoundBox(box_lim[0], box_lim[2], box_lim[4], box_lim[1], box_lim[3], box_lim[5])
        else:
            return None


def quadric_to_plane(cellDef,surfaces):

    surf_planes_dict = dict()
    planes = dict()

    surf_index = cellDef.signedSurfaces()
    next=list({abs(s) for s in surf_index})
    next.sort()
    next_index = next[-1] + 1
    apex = []

    for s_index in surf_index:
        pos = s_index > 0
        s_index = abs(s_index)
        s = surfaces[s_index]
        if s.type == "plane":
            normal, d = s.params
            position = normal * d
            planes[s_index] = Part.Plane(position, normal)
        else:
            surf_planes = convert_to_planes(s,pos)
            if s.type == "cone":
                apex.append(s.params[0])
                dbl = s.params[3]
                p_index = []
                for p in surf_planes:
                    planes[next_index] = p
                    p_index.append(next_index)
                    next_index += 1
                
                if dbl :
                    surf_planes_dict[s_index] = ('dblcone',p_index)
                else:
                    surf_planes_dict[s_index] = p_index    

            elif s.type == "torus":
                extplanes, inplanes = surf_planes
                p_ext = []
                p_in = []
                for p in extplanes:
                    planes[next_index] = p
                    p_ext.append(next_index)
                    next_index += 1
                for p in inplanes:
                    planes[next_index] = p
                    p_in.append(next_index)
                    next_index += 1
                surf_planes_dict[s_index] = ('torus',p_ext, p_in)
            else:
                p_index = []
                for p in surf_planes:
                    planes[next_index] = p
                    p_index.append(next_index)
                    next_index += 1
                surf_planes_dict[s_index] = p_index
    return planes, surf_planes_dict


def convert_to_planes(s,pos):
    if s.type == "cylinder":
        return cylinder_to_planes(s,pos)
    elif s.type == "cone":
        return cone_to_planes(s,pos)
    elif s.type == "sphere":
        return sphere_to_planes(s,pos)
    elif s.type == "torus":
        return torus_to_planes(s,pos)
    elif s.type == "paraboloid":
        return []  # I have to think how approximate paraboloid with planes
    else:
        print(f'{s.type} not implemented for boundbox'  )
        return []


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


def cylinder_to_planes(cyl,pos):
    center, axis, radius = cyl.params
    if pos : radius = radius * 0.70710678
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


def cone_to_planes(cone,pos):
    apex, axis, t, dbl = cone.params
    if pos : t = t * 0.70710678
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


def sphere_to_planes(sphere,pos):
    center, radius = sphere.params
    if pos : radius = radius * 0.70710678
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


def torus_to_planes(torus,pos):
    center, axis, majorRadius, minorR, minorA = torus.params

    x = FreeCAD.Vector(1, 0, 0)
    y = FreeCAD.Vector(0, 1, 0)
    z = FreeCAD.Vector(0, 0, 1)

    if pos:
        dist = (majorRadius + minorR) * 0.70710678
        difR = majorRadius - minorR
    else:     
        dist = majorRadius + minorR
        difR = (majorRadius - minorR) * 0.70710678


    if abs(abs(axis.dot(x)) - 1) < 1e-5:
        r1 = center + x * minorA
        r2 = center - x * minorA
        r3 = center + y * dist
        r4 = center - y * dist
        r5 = center + z * dist
        r6 = center - z * dist
        if difR > 0:
            r7 = center + y * difR
            r8 = center - y * difR
            r9 = center + z * difR
            r10 = center - z * difR
            p7 = Part.Plane(r7, y)
            p8 = Part.Plane(r8, -y)
            p9 = Part.Plane(r9, z)
            p10 = Part.Plane(r10, -z)
    elif abs(abs(axis.dot(y)) - 1) < 1e-5:
        r1 = center + x * dist
        r2 = center - x * dist
        r3 = center + y * minorA
        r4 = center - y * minorA
        r5 = center + z * dist
        r6 = center - z * dist
        if difR > 0:
            r7 = center + x * difR
            r8 = center - x * difR
            r9 = center + z * difR
            r10 = center - z * difR
            p7 = Part.Plane(r7, x)
            p8 = Part.Plane(r8, -x)
            p9 = Part.Plane(r9, z)
            p10 = Part.Plane(r10, -z)
    elif abs(abs(axis.dot(z)) - 1) < 1e-5:
        r1 = center + x * dist
        r2 = center - x * dist
        r3 = center + y * dist
        r4 = center - y * dist
        r5 = center + z * minorA
        r6 = center - z * minorA
        if difR > 0:
            r7 = center + x * difR
            r8 = center - x * difR
            r9 = center + y * difR
            r10 = center - y * difR
            p7 = Part.Plane(r7, x)
            p8 = Part.Plane(r8, -x)
            p9 = Part.Plane(r9, y)
            p10 = Part.Plane(r10, -y)

    p1 = Part.Plane(r1, -x)
    p2 = Part.Plane(r2, x)
    p3 = Part.Plane(r3, -y)
    p4 = Part.Plane(r4, y)
    p5 = Part.Plane(r5, -z)
    p6 = Part.Plane(r6, z)
    external_planes = (p1, p2, p3, p4, p5, p6)
    if difR > 0:
        central_planes = (p7, p8, p9, p10)
    else:
        central_planes = tuple()
    return (external_planes, central_planes)

def plane_definition(seq, surf_index):
    for s, planes in surf_index.items():
        if len(planes) == 0:
            continue
        if type(planes[0]) is str:
            if planes[0] == 'torus':
                extplanes, inplanes = planes[1:3]
                extm = BoolSequence(" ".join((str(p) for p in extplanes)))
                if len(inplanes) > 0:
                    inm = BoolSequence(":".join((str(p) for p in inplanes)))
                    pm = BoolSequence(operator="AND")
                    pm.append(extm, inm)
                else:
                    pm = extm
            elif planes[0] == 'dblcone':
                cplanes = planes[1]
                cone1 = BoolSequence(" ".join((str(p) for p in cplanes)))
                cone2 = BoolSequence(" ".join((str(-p) for p in cplanes)))
                pm = BoolSequence(operator='OR')
                pm.append(cone1,cone2)
        else:
            pm = BoolSequence(" ".join((str(p) for p in planes)))

        pp = pm.get_complementary()
        change_surf(seq, -s, pm)
        change_surf(seq, s, pp)

    seq.join_operators()
    return seq


def change_surf(seq, old, new):
    for i, e in enumerate(seq.elements):
        if type(e) is BoolSequence:
            change_surf(e, old, new)
        else:
            if e == old:
                if new is None:
                    seq.elements[i] = seq.operator == "AND"
                else:    
                    seq.elements[i] = new
    seq.clean()                
    seq.join_operators()


def plane_intersect(plane_list, u_radius, u_center):
    point_list = []
    origin = u_center.Length == 0

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
                p = inter[0][0]
                p = FreeCAD.Vector(p.X, p.Y, p.Z)
                d = p if origin else p - u_center
                if d.Length < u_radius:
                    point_list.append(p)
            j += 1
    return point_list


def sort_point(point_list, axis):
    axis_points = []
    if axis == "x":
        for i, point in enumerate(point_list):
            axis_points.append((point.x, i))
    elif axis == "y":
        for i, point in enumerate(point_list):
            axis_points.append((point.y, i))
    elif axis == "z":
        for i, point in enumerate(point_list):
            axis_points.append((point.z, i))
    else:
        print("bad axis name")

    axis_points.sort()
    sorted_points = list(point_list[x[1]] for x in axis_points)    
    removed = remove_close_points(list(sorted_points))
    return removed

def remove_close_points(sorted_list):
    if len(sorted_list)<2:
        return sorted_list
    new_points=[]
    p = sorted_list.pop()
    new_points.append(p)
    while len(sorted_list) > 0:
        nextp = sorted_list.pop()
        dp = p-nextp
        while (dp.Length < 0.1 ):
            if len(sorted_list) > 0:
                nextp = sorted_list.pop()
                dp = p-nextp
            else:
                break
        else:
            p = nextp        
            new_points.append(p)
    new_points.reverse()        
    return new_points    
        

def remove_points(point_list, value, axis, lower, Lmax = None):
    kept_points = []
    if axis == "x":
        if lower:
            upper_value = value + Lmax
            for p in point_list[::-1]:
                if p.x < value:
                    break
                if p.x < upper_value:
                    kept_points.append(p)
        else:
            for p in point_list[::-1]:
                if p.x > value:
                    break
                kept_points.append(p)
    elif axis == "y":
        if lower:
            upper_value = value + Lmax
            for p in point_list[::-1]:
                if p.y < value:
                    break
                if p.y < upper_value:
                    kept_points.append(p)
        else:
            for p in point_list[::-1]:
                if p.y > value:
                    break
                kept_points.append(p)
    elif axis == "z":
        if lower:
            upper_value = value + Lmax
            for p in point_list[::-1]:
                if p.z < value:
                    break
                if p.z < upper_value:
                    kept_points.append(p)
        else:
            for p in point_list[::-1]:
                if p.z > value:
                    break
                kept_points.append(p)
    else:
        print("bad axis name")
    return kept_points


def pointaxis(p, axis):
    if axis == "x":
        return p.x
    elif axis == "y":
        return p.y
    elif axis == "z":
        return p.z


def makePlane(normal, position, Box):

    p0 = normal.dot(position)

    pointEdge = []
    for i in range(12):
        edge = Box.getEdge(i)
        p1 = normal.dot(edge[0])
        p2 = normal.dot(edge[1])
        d0 = p0 - p1
        d1 = p2 - p1
        if d1 != 0:
            a = d0 / d1
            if a >= 0 and a <= 1:
                pointEdge.append(edge[0] + a * (edge[1] - edge[0]))

    if len(pointEdge) == 0:
        return None  # Plane does not cross box

    s = FreeCAD.Vector((0, 0, 0))
    for v in pointEdge:
        s = s + v
    s = s / len(pointEdge)

    vtxvec = []
    for v in pointEdge:
        vtxvec.append(v - s)

    X0 = vtxvec[0]
    Y0 = normal.cross(X0)

    orden = []
    for i, v in enumerate(vtxvec):
        phi = numpy.arctan2(v.dot(Y0), v.dot(X0))
        orden.append((phi, i))
    orden.sort()

    return Part.Face(Part.makePolygon([pointEdge[p[1]] for p in orden], True))

def inertia_matrix(points):
    npoints = len(points)
    numpy_points = numpy.ndarray((npoints,3))
    for i,p in enumerate(points):
        numpy_points[i] = numpy.array((p.x,p.y,p.z))

    x0 = numpy.sum(numpy_points[:,0])/npoints
    y0 = numpy.sum(numpy_points[:,1])/npoints
    z0 = numpy.sum(numpy_points[:,2])/npoints
    Sxx = numpy.sum(numpy_points[:,0]*numpy_points[:,0])
    Syy = numpy.sum(numpy_points[:,1]*numpy_points[:,1])
    Szz = numpy.sum(numpy_points[:,2]*numpy_points[:,2])
    Sxy = numpy.sum(numpy_points[:,0]*numpy_points[:,1])
    Sxz = numpy.sum(numpy_points[:,0]*numpy_points[:,2])
    Syz = numpy.sum(numpy_points[:,1]*numpy_points[:,2])

    Ixx = Sxx/npoints - x0*x0
    Iyy = Syy/npoints - y0*y0
    Izz = Szz/npoints - z0*z0
    Ixy = Sxy/npoints - x0*y0
    Ixz = Sxz/npoints - x0*z0
    Iyz = Syz/npoints - y0*z0

    inertia_matrix = numpy.array(((Ixx,Ixy,Ixz),
                                  (Ixy,Iyy,Iyz),
                                  (Ixz,Iyz,Izz)))
    eigvalue,vectors = numpy.linalg.eig(inertia_matrix)
    return