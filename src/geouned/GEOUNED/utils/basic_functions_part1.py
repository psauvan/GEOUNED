#
# Set of useful functions used in different parts of the code
#
import math

import FreeCAD
from .data_constants import mask
from .boolean_function import BoolSurface


def is_same_value(v1, v2, tolerance=1e-6):
    return abs(v1 - v2) < tolerance


def is_opposite(vector_1, vector_2, tolerance=1e-3):
    return abs(vector_1.getAngle(-vector_2)) < tolerance


def is_parallel(vector_1, vector_2, tolerance=1e-3):
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


def round_corner_region(p1id, p2id, cid, pid, configuration):
    # p1,p2,c,pc are boolVariable objects
    # p1,p2,c,pc are planes and cylinder indexes
    # p1,p2,pc index correspond to normal vector pointing toward material
    # pc index pointing toward cylinder arc

    fwd_cyl = configuration & mask.fwd_cyl == mask.fwd_cyl
    AND_p1_cyl = configuration & mask.p1_cyl == mask.p1_cyl
    AND_p2_cyl = configuration & mask.p2_cyl == mask.p2_cyl
    AND_p1_pd = configuration & mask.p1_pd == mask.p1_pd
    AND_p2_pd = configuration & mask.p2_pd == mask.p2_pd
    OR_bracket = configuration & mask.p1_p2 == mask.p1_p2
    same_p1_pd = configuration & mask.same_p1_pd == mask.same_p1_pd
    same_p2_pd = configuration & mask.same_p2_pd == mask.same_p2_pd

    if fwd_cyl:
        p1id = -p1id
        p2id = -p2id

    if p1id == p2id:
        if AND_p1_cyl:
            rc_region = BoolSurface(0, p1id) * BoolSurface(0, cid)
        else:
            rc_region = BoolSurface(0, p1id) + BoolSurface(0, cid)
    elif AND_p1_cyl and AND_p2_cyl:
        if same_p1_pd and same_p2_pd:
            rc_region = BoolSurface(0, p1id) * BoolSurface(0, cid)
        elif same_p1_pd or same_p2_pd:
            if OR_bracket:
                rc_region = (BoolSurface(0, p1id) + BoolSurface(0, p2id)) * BoolSurface(0, cid)
            else:
                rc_region = BoolSurface(0, p1id) * BoolSurface(0, cid) * BoolSurface(0, p2id)
        elif AND_p1_pd and AND_p2_pd:
            rc_region = BoolSurface(0, p1id) * BoolSurface(0, p2id) * BoolSurface(0, pid) * BoolSurface(0, cid)
        elif not AND_p1_pd and not AND_p2_pd:
            rc_region = BoolSurface(0, cid) * (BoolSurface(0, p1id) + BoolSurface(0, p2id) + BoolSurface(0, pid))
        elif not AND_p1_pd and AND_p2_pd:
            if OR_bracket:
                rc_region = (BoolSurface(0, p1id) + BoolSurface(0, pid)) * BoolSurface(0, p2id) * BoolSurface(0, cid)
            else:
                rc_region = (BoolSurface(0, p1id) + (BoolSurface(0, pid) * BoolSurface(0, p2id))) * BoolSurface(0, cid)
        else:
            if OR_bracket:
                rc_region = (BoolSurface(0, p2id) + BoolSurface(0, pid)) * BoolSurface(0, p1id) * BoolSurface(0, cid)
            else:
                rc_region = (BoolSurface(0, p2id) + (BoolSurface(0, pid) * BoolSurface(0, p1id))) * BoolSurface(0, cid)

    elif not AND_p1_cyl and not AND_p2_cyl:
        if same_p1_pd and same_p2_pd:
            rc_region = BoolSurface(0, p1id) + BoolSurface(0, cid)
        elif not AND_p1_pd and not AND_p2_pd:
            rc_region = BoolSurface(0, p1id) + BoolSurface(0, p2id) + (BoolSurface(0, pid) * BoolSurface(0, cid))
        else:
            errorlog = f"""error this configuration should not exist for roundCorner.
 AND_p1_cyl : {AND_p1_cyl}
 AND_p2_cyl : {AND_p2_cyl}
 AND_p1_pd : {AND_p1_pd}
 AND_p1_pd : {AND_p2_pd}"""
            raise RuntimeError(errorlog)

    elif AND_p1_cyl and not AND_p2_cyl:
        if not AND_p1_pd and not AND_p2_pd:
            rc_region = ((BoolSurface(0, p1id) + BoolSurface(0, pid)) * BoolSurface(0, cid)) + BoolSurface(0, p2id)
        elif AND_p1_pd and not AND_p2_pd:
            if OR_bracket:
                rc_region = BoolSurface(0, p1id) * (BoolSurface(0, p2id) + (BoolSurface(0, pid) * BoolSurface(0, cid)))
            else:
                rc_region = (BoolSurface(0, p1id) * BoolSurface(0, pid) * BoolSurface(0, cid)) + BoolSurface(0, p2id)
        else:
            errorlog = f"""error this configuration should not exist for roundCorner.
 AND_p1_cyl : {AND_p1_cyl}
 AND_p2_cyl : {AND_p2_cyl}
 AND_p1_pd : {AND_p1_pd}
 AND_p1_pd : {AND_p2_pd}"""
            raise RuntimeError(errorlog)
    else:
        if not AND_p1_pd and not AND_p2_pd:
            rc_region = BoolSurface(0, p1id) + (BoolSurface(0, cid) * (BoolSurface(0, pid) + BoolSurface(0, p2id)))
        elif not AND_p1_pd and AND_p2_pd:
            if OR_bracket:
                rc_region = (BoolSurface(0, p1id) + (BoolSurface(0, cid) * BoolSurface(0, pid))) * BoolSurface(0, p2id)
            else:
                rc_region = BoolSurface(0, p1id) + (BoolSurface(0, pid) * BoolSurface(0, cid) * BoolSurface(0, p2id))
        else:
            errorlog = f"""error this configuration should not exist for roundCorner.
 AND_p1_cyl : {AND_p1_cyl}
 AND_p2_cyl : {AND_p2_cyl}
 AND_p1_pd : {AND_p1_pd}
 AND_p1_pd : {AND_p2_pd}"""
            raise RuntimeError(errorlog)

    return -rc_region if fwd_cyl else rc_region


def multi_round_corner_region(mRoundC, overlap):
    multi_rc_region = None
    if overlap:
        rc1 = mRoundC.Surf.Corners[0]
        rc2 = mRoundC.Surf.Corners[1]
        cyl1 = rc1.Surf.Cylinder.Surf.Cylinder
        cyl2 = rc2.Surf.Cylinder.Surf.Cylinder

        cyl1.Orientation = rc1.Surf.Cylinder.Orientation
        cyl2.Orientation = rc2.Surf.Cylinder.Orientation

        pc1 = rc1.Surf.Cylinder.Surf.Plane
        pc2 = rc2.Surf.Cylinder.Surf.Plane
        p11, p12 = rc1.Surf.Planes
        p21, p22 = rc2.Surf.Planes

        c1id = cyl1.bVar
        c2id = cyl2.bVar
        pc1id = pc1.bVar
        pc2id = pc2.bVar

        AND1_p1_cyl = not (rc1.Surf.Configuration & mask.p1_cyl == mask.p1_cyl)
        AND1_p2_cyl = not (rc1.Surf.Configuration & mask.p2_cyl == mask.p2_cyl)
        AND2_p1_cyl = not (rc2.Surf.Configuration & mask.p1_cyl == mask.p1_cyl)
        AND2_p2_cyl = not (rc2.Surf.Configuration & mask.p2_cyl == mask.p2_cyl)

        if AND1_p1_cyl and AND1_p2_cyl and AND2_p1_cyl and AND2_p2_cyl:
            rc1_region = round_corner_region(p11.bVar, p12.bVar, c1id, pc1id, rc1.Surf.Configuration)
            rc2_region = round_corner_region(p21.bVar, p22.bVar, c2id, pc2id, rc2.Surf.Configuration)
            multi_rc_region = rc1_region * rc2_region if mRoundC.Orientation == "Forward" else rc1_region + rc2_region
        elif AND1_p1_cyl and AND1_p2_cyl:
            rc2_region = round_corner_region(p21.bVar, p22.bVar, c2id, pc2id, rc2.Surf.Configuration)
            comp1 = (
                -BoolSurface(0, c1id) - BoolSurface(0, pc1id)
                if cyl1.Orientation == "Forward"
                else BoolSurface(0, c1id) * BoolSurface(0, pc1id)
            )
            multi_rc_region = rc2_region * comp1 if mRoundC.Orientation == "Forward" else rc2_region + comp1
        elif AND2_p1_cyl and AND2_p2_cyl:
            rc1_region = round_corner_region(p11.bVar, p12.bVar, c1id, pc1id, rc1.Surf.Configuration)
            comp2 = (
                -BoolSurface(0, c2id) - BoolSurface(0, pc2id)
                if cyl2.Orientation == "Forward"
                else BoolSurface(0, c2id) * BoolSurface(0, pc2id)
            )
            multi_rc_region = rc1_region * comp2 if mRoundC.Orientation == "Forward" else rc1_region + comp2
        elif not AND1_p1_cyl and not AND1_p2_cyl:
            rc2_region = round_corner_region(p21.bVar, p22.bVar, c2id, pc2id, rc2.Surf.Configuration)
            if mRoundC.Orientation == "Forward":
                multi_rc_region = -BoolSurface(0, pc1id) * rc2_region + BoolSurface(0, -c1id)
            else:
                multi_rc_region = rc2_region * (BoolSurface(0, c1id) - BoolSurface(0, pc1id))
        elif not AND2_p1_cyl and not AND2_p2_cyl:
            rc1_region = round_corner_region(p11.bVar, p12.bVar, c1id, pc1id, rc1.Surf.Configuration)
            if mRoundC.Orientation == "Forward":
                multi_rc_region = -BoolSurface(0, pc2id) * rc1_region + BoolSurface(0, -c2id)
            else:
                multi_rc_region = rc1_region * (BoolSurface(0, c2id) - BoolSurface(0, pc2id))
        else:
            rc1_region = round_corner_region(p11.bVar, p12.bVar, c1id, pc1id, rc1.Surf.Configuration)
            p2surf = BoolSurface(0, pc2id)
            if mRoundC.Orientation == "Forward":
                multi_rc_region = (-p2surf * rc1_region) + (p2surf * BoolSurface(0, -c2id))
            else:
                multi_rc_region = (-p2surf * rc1_region) + (p2surf * BoolSurface(0, c2id))

    else:
        if mRoundC.Orientation == "Forward":
            for plane in mRoundC.Surf.Planes:
                pid = BoolSurface(0, plane.bVar)
                multi_rc_region = BoolSurface.mult(multi_rc_region, pid)

            or_comp = []
            for rc in mRoundC.Surf.Corners:
                cylinder = rc.Surf.Cylinder.Surf.Cylinder
                cid = BoolSurface(0, cylinder.bVar)
                if rc.Surf.Cylinder.Surf.Plane is not None:
                    pcid = BoolSurface(0, rc.Surf.Cylinder.Surf.Plane.bVar)
                else:
                    pcid = None

                if rc.Orientation == "Forward":
                    if pcid is not None:
                        multi_rc_region = BoolSurface.mult(multi_rc_region, -pcid)
                        or_comp.append((-cid, pcid))
                    else:
                        or_comp.append((-cid,))
                else:
                    multi_rc_region = BoolSurface.mult(multi_rc_region, pcid) * cid

            for cp in or_comp:
                if len(cp) == 1:
                    multi_rc_region = multi_rc_region + cp[0]
                else:
                    multi_rc_region = multi_rc_region + (cp[0] * cp[1])
        else:
            for plane in mRoundC.Surf.Planes:
                pid = BoolSurface(0, plane.bVar)
                multi_rc_region = BoolSurface.add(multi_rc_region, pid)

            and_comp = []
            for rc in mRoundC.Surf.Corners:
                cylinder = rc.Surf.Cylinder.Surf.Cylinder
                cid = BoolSurface(0, cylinder.bVar)
                if rc.Surf.Plane is not None:
                    pcid = BoolSurface(0, rc.Surf.Cylinder.Surf.Plane.bVar)
                else:
                    pcid = None

                if rc.Orientation == "Forward":
                    if pcid is not None:
                        multi_rc_region = BoolSurface.add(multi_rc_region, -pcid) - cid
                    else:
                        multi_rc_region = multi_rc_region - cid
                else:
                    multi_rc_region = BoolSurface.add(multi_rc_region, pcid)
                    and_comp.append((cid, -pcid))
            for c, p in and_comp:
                multi_rc_region = multi_rc_region * (c + p)

    return multi_rc_region


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
    def __init__(self, params):
        self.Position = params[0]
        self.Axis = params[1]
        self.dimL1 = params[2]
        self.dimL2 = params[3]
        if len(params) > 4:
            self.real = params[4]
        else:
            self.real = True
        self.pointDef = False

    def __eq__(self, p2):
        if type(p2) is not PlaneParams:
            return False
        r = self.Position - p2.Position
        if abs(r.dot(self.Axis)) > 1e-6:
            return False

        d = self.Axis.dot(p2.Axis)
        return abs(d - 1) < 1e-6

    def __str__(self):
        pos = self.Axis.dot(self.Position)
        outstr = f"""Plane :
    Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
    Position : {pos}  """
        return outstr


class CylinderOnlyParams:
    def __init__(self, params, real=True):
        self.Center = params[0]
        self.Axis = params[1]
        self.Radius = params[2]
        self.dimL = params[3]
        self.real = real

    def __eq__(self, c2):
        if type(c2) is not CylinderParams:
            return False

        if abs(self.Radius - c2.Radius) > 1.0e-8:
            return False

        r = self.Center - c2.Center
        if r.Length > 1e-8:
            return False

        d = self.Axis.dot(c2.Axis)
        if abs(d - 1) > 1e-8:
            return False
        else:
            return True

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

    def __eq__(self, c2):
        if type(c2) is not ConeParams:
            return False

        if abs(self.SemiAngle - c2.SemiAngle) > 1.0e-8:
            return False

        r = self.apex - c2.Apex
        if r.Length > 1e-8:
            return False

        d = self.Axis.dot(c2.Axis)
        if abs(d - 1) > 1e-8:
            return False
        else:
            return True

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

    def __eq__(self, s2):
        if type(s2) is not SphereParams:
            return False

        if abs(self.Radius - s2.Radius) > 1.0e-8:
            return False

        r = self.Center - s2.Center
        if r.Length > 1e-8:
            return False
        else:
            return True

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

    def __eq__(self, t2):
        if type(t2) is not TorusParams:
            return False

        if abs(self.MajorRadius - t2.MajorRadius) > 1.0e-8:
            return False

        if abs(self.MinorRadius - t2.MinorRadius) > 1.0e-8:
            return False

        r = self.Center - t2.Center
        if r.Length > 1e-8:
            return False

        d = self.Axis.dot(t2.Axis)
        if abs(d - 1) > 1e-8:
            return False
        else:
            return True

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

    def __eq__(self, mp):
        if self.PlaneNumber != mp.PlaneNumber:
            return False
        eq_count = 0
        for p1 in self.Planes:
            for p2 in mp.Planes:
                if p1 == p2:
                    eq_count += 1
                    break
        return eq_count == self.PlaneNumber

    def __str__(self):
        outstr = f"""Multiplane :\n"""
        for p in self.Planes:
            outstr += f"{p.__str__()} \n"
        return outstr


class CanParams:
    def __init__(self, params):
        self.Cylinder = params[0]
        self.s1 = params[1][0]
        self.s1_configuration = params[1][1]
        self.s2 = params[2][0]
        self.s2_configuration = params[2][1]


class TConeParams:
    def __init__(self, params):
        self.Cone = params[0]
        self.p1 = params[1][0]
        self.p1_configuration = params[1][1]
        self.p2 = params[2][0]
        self.p2_configuration = params[2][1]


class RoundCornerParams:
    def __init__(self, params):
        self.Configuration = params[2]
        self.Planes = params[1]
        self.Cylinder = params[0]


class MultiRoundCornerParams:
    def __init__(self, params):
        self.Orientation = params[2]
        self.Planes = params[1]
        self.Corners = params[0]


class ReversedConeCylParams:
    def __init__(self, params):
        self.CylCones = params[0]
        self.PlaneSeq = params[1]
        self.AddPlanes = params[2]


class SphereParams:
    def __init__(self, params):
        self.Sphere = params[0]
        self.Plane = params[1]

    def __str__(self):
        outstr = f"""Sphere :\n"""
        outstr += f"{self.Sphere.__str__()} \n"
        if self.Plane:
            outstr += f"{self.Plane.__str__()} \n"
        return outstr


class CylinderParams:
    def __init__(self, params):
        self.Cylinder = params[0]
        self.Plane = params[1]

    def __str__(self):
        outstr = f"""Cylinder :\n"""
        outstr += f"{self.Cylinder.__str__()} \n"
        if self.Plane:
            outstr += f"{self.Plane.__str__()} \n"
        return outstr


class ConeParams:
    def __init__(self, params):
        self.Cone = params[0]
        self.ApexPlane = params[1]
        self.Plane = params[2]

    def __str__(self):
        outstr = f"""Cone :\n"""
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
        self.SOrientation = params[3]
