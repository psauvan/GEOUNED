#
# Set of useful functions used in different parts of the code
#
import math

import FreeCAD


def is_same_value(v1, v2, tolerance=1e-6):
    return abs(v1 - v2) < tolerance


def is_opposite(vector_1, vector_2, tolerance=1e-6):
    return abs(vector_1.getAngle(-vector_2)) < tolerance


def is_parallel(vector_1, vector_2, tolerance=1e-6):
    angle = abs(vector_1.getAngle(vector_2))
    return angle < tolerance or is_same_value(angle, math.pi, tolerance)


def is_in_line(point, dir, pnt_line, tolerance=1e-6):
    r12 = point - pnt_line
    return is_parallel(dir, r12) or (r12.Length < tolerance)


# TODO check this function is used in the code
def is_in_points(point, points, tolerance=1e-5):
    if len(points) > 0:
        for p in points:
            if point.isEqual(p, tolerance):
                return True
    return False


# TODO check this function is used in the code
def is_in_edge(edge1, edge2, tolerance=1e-8):
    ver1 = edge1.Vertexes
    ver2 = edge2.Vertexes
    con1 = ver1[0].Point.isEqual(ver2[0].Point, tolerance) or ver1[0].Point.isEqual(ver2[1].Point, tolerance)
    con2 = ver1[1].Point.isEqual(ver2[0].Point, tolerance) or ver1[1].Point.isEqual(ver2[1].Point, tolerance)
    return con1 and con2


def is_in_plane(point, plane, d_tolerance=1e-7):
    return abs(point.distanceToPlane(plane.Surf.Position, plane.Surf.Axis)) < d_tolerance


def is_in_tolerance(val, tol, fuzzy_low, fuzzy_high):
    if abs(val) < fuzzy_low:
        return True, False  # 1) isintolerance 2) fuzzy
    elif abs(val) < tol:
        return True, True
    elif abs(val) > fuzzy_high:
        return False, False
    else:
        return False, True


def sign_plane(point, plane):
    value = plane.Surf.Axis.dot(point - plane.Surf.Position)
    if value >= 0.0:
        sign = 1
    else:
        sign = -1
    return sign


def points_to_coeffs(points):
    p1, p2, p3 = points[0:3]
    scf = (p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z)

    # mcnp implementation to convert 3 point plane to
    # plane parameters

    tpp = [0] * 4
    for i in range(1, 4):
        j = i % 3 + 1
        k = 6 - i - j
        k -= 1
        j -= 1
        tpp[i - 1] = (
            scf[j] * (scf[k + 3] - scf[k + 6]) + scf[j + 3] * (scf[k + 6] - scf[k]) + scf[j + 6] * (scf[k] - scf[k + 3])
        )
        tpp[3] += scf[i - 1] * (scf[j + 3] * scf[k + 6] - scf[j + 6] * scf[k + 3])

    xm = 0
    coeff = [0] * 4
    for i in range(1, 5):
        if xm == 0 and tpp[4 - i] != 0:
            xm = 1 / tpp[4 - i]
        coeff[4 - i] = tpp[4 - i] * xm

    axis = FreeCAD.Vector(coeff[0:3])
    distance = coeff[3] / axis.Length
    axis.normalize()

    return axis, distance


class Plane3PtsParams:
    def __init__(self, params, real=True):
        self.Position = params[0]
        self.Axis = params[1]
        self.dimL1 = params[2]
        self.dimL2 = params[3]
        self.Points = params[4]
        self.real = real
        self.pointDef = True

    def __str__(self):
        #      outstr = '''Plane :
        #    Point 1  : {P1[0]}  {P1[1]}  {P1[2]}
        #    Point 2  : {P2[0]}  {P2[1]}  {P2[2]}
        #    Point 3  : {P3[0]}  {P3[1]}  {P3[2]} '''.format(P1=self.Points[0], P2=self.Points[1], P3=self.Points[2])
        pos = self.Axis.dot(self.Position)
        outstr = f"""Plane :
    Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
    Position : {pos}  """
        return outstr


class PlaneParams:
    def __init__(self, params, real=True):
        self.Position = params[0]
        self.Axis = params[1]
        self.dimL1 = params[2]
        self.dimL2 = params[3]
        self.real = real
        self.pointDef = False

    def __str__(self):
        pos = self.Axis.dot(self.Position)
        outstr = f"""Plane :
    Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
    Position : {pos}  """
        return outstr


class DiskParams:
    def __init__(self, params, real=True):
        if params[0] == "circle":
            self.Type = "circle"
            self.Center = params[1]
            self.Axis = params[2]
            self.Radius = params[3]
        else:
            self.Type = "ellipse"
            self.Center = params[1]
            self.Axis = params[2]
            self.majAxis = params[3]
            self.minAxis = params[4]

    def __str__(self):
        if self.Type == "circle":
            outstr = f"""Circular disk :
        Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
        Center   : {self.Center.x}  {self.Center.y}  {self.Center.z}
        Radius   : {self.Radius} """
        else:
            outstr = f"""Elliptic disk :
        Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
        Center   : {self.Center.x}  {self.Center.y}  {self.Center.z}
        Min/Maj Radius   : {self.minRadius} {self.majRadius} """

        return outstr


class CylinderOnlyParams:
    def __init__(self, params, real=True):
        self.Center = params[0]
        self.Axis = params[1]
        self.Radius = params[2]
        self.dimL = params[3]
        self.real = real

    def __str__(self):
        outstr = f"""Cylinder :
    Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
    Center   : {self.Center.x}  {self.Center.y}  {self.Center.z}
    Radius   : {self.Radius}  """
        return outstr


class ConeOnlyParams:
    def __init__(self, params, real=True):
        self.Apex = params[0]
        self.Axis = params[1]
        self.SemiAngle = params[2]
        self.dimL = params[3]
        self.dimR = params[4]
        self.real = real

    def __str__(self):
        outstr = f"""Cone :
    Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
    Center   : {self.Apex.x}  {self.Apex.y}  {self.Apex.z}
    SemiAngle: {self.SemiAngle}  """
        return outstr


class SphereOnlyParams:
    def __init__(self, params):
        self.Center = params[0]
        self.Radius = params[1]

    def __str__(self):
        outstr = f"""Sphere :
    Center   : {self.Center.x}  {self.Center.y}  {self.Center.z}
    Radius   : {self.Radius}  """
        return outstr


class TorusOnlyParams:
    def __init__(self, params):
        self.Center = params[0]
        self.Axis = params[1]
        self.MajorRadius = params[2]
        self.MinorRadius = params[3]

    def __str__(self):
        outstr = f"""Torus :
    Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
    Center   : {self.Center.x}  {self.Center.y}  {self.Center.z}
    MajorRadius: {self.MajorRadius}
    MinorRadius: {self.MinorRadius} """
        return outstr


class MultiPlanesParams:
    def __init__(self, params):
        self.PlaneNumber = len(params[0])
        self.Edges = params[1]
        self.Vertexes = params[2]
        self.Planes = params[0][:]

    def __str__(self):
        outstr = f"""Multiplane :\n"""
        for p in self.Planes:
            outstr += f"{p.__str__()} \n"
        return outstr


class ReverseCanParams:
    def __init__(self, params):
        self.PlaneNumber = len(params) - 1
        self.Planes = params[1:]
        self.Cylinder = params[0]

    def __str__(self):
        outstr = f"""ReverseCan :\n"""
        outstr += f"{self.Cylinder.__str__()} \n"
        for p in self.Planes:
            outstr += f"{p.__str__()} \n"
        return outstr


class ForwardCanParams:
    def __init__(self, params):
        self.PlaneNumber = len(params) - 1
        self.Planes = params[1:]
        self.Cylinder = params[0]

    def __str__(self):
        outstr = f"""ForwardCan :\n"""
        outstr += f"{self.Cylinder.__str__()} \n"
        for p in self.Planes:
            outstr += f"{p.__str__()} \n"
        return outstr


class RoundCornerParams:
    def __init__(self, params):
        self.Planes = params[1]
        self.Cylinder = params[0][0]
        self.AddPlane = params[0][1]
        self.Configuration = params[2]

    def __str__(self):
        outstr = f"""RoundCorner :\n"""
        outstr += f"{self.Configuration} \n"
        outstr += f"{self.Cylinder.__str__()} \n"
        outstr += f"{self.AddPlane.__str__()} \n"
        for p in self.Planes:
            outstr += f"{p.__str__()} \n"
        return outstr


class ReversedConeCylParams:
    def __init__(self, params):
        self.CylCones = []
        for cc in params:
            self.CylCones.append(cc.Surf)

    def __str__(self):
        outstr = f"""ReversedCylCone :\n"""
        for cc in self.CylCones:
            outstr += f"{cc.__str__()} \n"
        return outstr


class SphereParams:
    def __init__(self, params):
        self.Sphere = params[0]
        self.Plane = params[1]
        self.Orientation = params[2]

    def __str__(self):
        outstr = f"""Sphere :\n"""
        outstr += f"{self.Orientation} \n"
        outstr += f"{self.Sphere.__str__()} \n"
        if self.Plane:
            outstr += f"{self.Plane.__str__()} \n"
        return outstr


class CylinderParams:
    def __init__(self, params):
        self.Cylinder = params[0]
        self.Plane = params[1]
        self.addPlanes = params[2]
        self.Orientation = params[3]

    def __str__(self):
        outstr = f"""Cylinder :\n"""
        outstr += f"{self.Orientation} \n"
        outstr += f"{self.Cylinder.__str__()} \n"
        if self.Plane:
            outstr += f"{self.Plane.__str__()} \n"
        return outstr


class ConeParams:
    def __init__(self, params):
        self.Cone = params[0]
        self.ApexPlane = params[1]
        self.Plane = params[2]
        self.addPlanes = params[3]
        self.Orientation = params[4]

    def __str__(self):
        outstr = f"""Cone :\n"""
        outstr += f"{self.Orientation} \n"
        outstr += f"{self.Cone.__str__()} \n"
        if self.ApexPlane:
            outstr += f"{self.ApexPlane.__str__()} \n"
        if self.Plane:
            outstr += f"{self.Plane.__str__()} \n"
        return outstr


class TorusParams:
    def __init__(self, params):
        self.Torus = params[0]
        self.UPlanes = params[1]
        self.VSurface = params[2]
        self.Orientation = params[3]
        self.SOrientation = params[4]

    def __str__(self):
        outstr = f"""Cone :\n"""
        outstr += f"{self.Orientation} \n"
        outstr += f"{self.Torus.__str__()} \n"
        if self.UPlane:
            outstr += f"{self.UPlane.__str__()} \n"
        if self.VSurface:
            outstr += f"{self.VSurfacePlane.__str__()} \n"
        return outstr
