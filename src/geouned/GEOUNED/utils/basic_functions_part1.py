#
# Set of useful functions used in different parts of the code
#
import math

from .data_constants import mask, twoPi
from .boolean_function import BoolSurface
from ...geo import surface_geometry
from ...geo import GPlane, GSolid, GVector, Gin_contact

# The functions below are thin adapters over `surface_geometry.py` (the
# backend-agnostic predicate layer). Callers throughout GEOUNED are
# expected to already pass GVector.


def is_same_value(v1, v2, tolerance=1e-6):
    return surface_geometry.is_same_value(v1, v2, tolerance)


def is_opposite(vector_1, vector_2, tolerance=1e-3):
    return surface_geometry.is_opposite(vector_1, vector_2, tolerance)


def is_parallel(vector_1, vector_2, tolerance=1e-3):
    return surface_geometry.is_parallel(vector_1, vector_2, tolerance)


def is_in_line(point, dir, pnt_line, tolerance=1e-6):
    return surface_geometry.is_in_line(point, dir, pnt_line, tolerance)


def is_in_plane(point, plane, d_tolerance=1e-7):
    plane_params = GPlane.from_values(plane.Surf.Position, plane.Surf.Axis)
    return surface_geometry.is_in_plane(point, plane_params, d_tolerance)


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
    plane_params = GPlane.from_values(plane.Surf.Position, plane.Surf.Axis)
    return surface_geometry.sign_plane(point, plane_params)


def shapes_in_contact(shape1, shape2, tolerance=1e-6):
    if shape1 is shape2:
        return True
    return Gin_contact(
        GSolid(shape1),
        GSolid(shape2),
        tolerance,
    )


def twoPimod(x):
    x = x % twoPi
    if x < 1e-5:
        return 0.0
    elif twoPi - x < 1e-5:
        return 0.0
    else:
        return x


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

    axis = GVector(*coeff[0:3])
    distance = coeff[3] / axis.length
    axis = axis.normalized()

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

    p1id_raw = p1id

    if fwd_cyl:
        p1id = -p1id
        p2id = -p2id

    if p1id == p2id:
        if AND_p1_cyl:
            # Single-corner-plane AND case: the fwd_cyl pre-negation above
            # combined with the final -rc_region complement below
            # double-applies fwd_cyl to p1id here (verified wrong against
            # real CAD ground truth: TVA_final_allencl.stp's "Barrel upper
            # left" RoundCorner, 78.7% match with the pre-negated p1id vs
            # 99.9% using the original, un-negated one). The sibling OR
            # branch just below does NOT have this problem (verified 100%
            # correct on Solidos/trier/series_solid2_complement.stp) so it
            # deliberately keeps using the pre-negated p1id.
            rc_region = BoolSurface(0, p1id_raw) * BoolSurface(0, cid)
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


def multi_round_corner_region(mRoundC):

    planes_region = None
    revCyl_region = None
    fwdCyl_region = None

    if mRoundC.Orientation == "Forward":
        for plane in mRoundC.Surf.Planes:
            pid = BoolSurface(0, plane.bVar)
            planes_region = BoolSurface.mult(planes_region, pid)

        for rc in mRoundC.Surf.Corners:
            cylinder = rc.Surf.Cylinder.Surf.Cylinder
            cid = BoolSurface(0, cylinder.bVar)
            if rc.Surf.Cylinder.Surf.Plane is not None:
                pcid = BoolSurface(0, rc.Surf.Cylinder.Surf.Plane.bVar)
            else:
                pcid = BoolSurface(0, rc.Surf.Planes[0].bVar)
                if rc.Surf.Cylinder.Orientation == "Forward":
                    pcid = -pcid

            if rc.Surf.Cylinder.Orientation == "Forward":
                planes_region = BoolSurface.mult(planes_region, -pcid)
                fwdCyl_region = BoolSurface.add(fwdCyl_region, -cid * pcid)
            else:
                planes_region = BoolSurface.mult(planes_region, pcid)
                revCyl_region = BoolSurface.mult(revCyl_region, cid)

        multi_rc_region = (planes_region + fwdCyl_region) * revCyl_region
    else:
        for plane in mRoundC.Surf.Planes:
            pid = BoolSurface(0, plane.bVar)
            planes_region = BoolSurface.add(planes_region, pid)

        for rc in mRoundC.Surf.Corners:
            cylinder = rc.Surf.Cylinder.Surf.Cylinder
            cid = BoolSurface(0, cylinder.bVar)
            if rc.Surf.Cylinder.Surf.Plane is not None:
                pcid = BoolSurface(0, rc.Surf.Cylinder.Surf.Plane.bVar)
            else:
                pcid = BoolSurface(0, rc.Surf.Planes[0].bVar)
                if rc.Surf.Cylinder.Orientation == "Forward":
                    pcid = -pcid

            if rc.Surf.Cylinder.Orientation == "Forward":
                planes_region = BoolSurface.add(planes_region, -pcid)
                fwdCyl_region = BoolSurface.add(fwdCyl_region, -cid)
            else:
                planes_region = BoolSurface.add(planes_region, pcid)
                revCyl_region = BoolSurface.mult(revCyl_region, -pcid + cid)

        multi_rc_region = (planes_region * revCyl_region) + fwdCyl_region

    return multi_rc_region


def can_region(cid, cyl_orientation, surf_list):
    """
    Boolean AND/OR definition of a Can meta-surface (a cylinder bounded by
    up to two secondary surfaces s1/s2, each a Plane, Cylinder, Cone or
    Sphere), built purely from already-resolved signed ids -- shared by
    the two places that need this same region: `MetaSurfacesDict.Can_region`
    (global, deduplicated ids, registered via `primitive_surfaces.add_*`)
    and `get_cell_object` (local, decomposition-time ids read straight off
    `.bVar`). Only the id *scope* differs between the two callers; the
    combination rule itself must not be duplicated.

    cid: id of the cylinder component (sign convention: cyl_orientation
        decides the sign actually used, same as round_corner_region).
    cyl_orientation: "Forward" or "Reversed", orientation of the cylinder.
    surf_list: iterable of (kind, sid, pid, apid, orientation, configuration)
        kind: "Plane" | "Cylinder" | "Cone" | "Sphere"
        sid: id of the secondary analytic surface; None for kind=="Plane"
        pid: id of the plane bounding the secondary surface (or the plane
            itself when kind=="Plane"); None if there is none
        apid: id of the cone's apex plane; only meaningful for kind=="Cone"
        orientation: "Forward"/"Reversed" of the secondary surface; not
            used for kind=="Plane"
        configuration: "AND" or "OR", how this surface combines with cid
    """
    region = BoolSurface(0, cid) if cyl_orientation == "Reversed" else BoolSurface(0, -cid)

    for kind, sid, pid, apid, orientation, configuration in surf_list:
        if kind == "Plane":
            si_region = BoolSurface(0, pid) if configuration == "AND" else BoolSurface(0, -pid)
        elif pid is None:
            if apid is None:
                si_region = BoolSurface(0, -sid) if orientation == "Forward" else BoolSurface(0, sid)
            elif orientation == "Forward":
                si_region = BoolSurface(0, apid) * BoolSurface(0, -sid)
            else:
                si_region = BoolSurface(0, -apid) + BoolSurface(0, sid)
        elif apid is None:
            if orientation == "Forward":
                if configuration == "AND":
                    si_region = BoolSurface(0, -sid) + BoolSurface(0, pid)
                else:
                    si_region = BoolSurface(0, -sid) + BoolSurface(0, -pid)
            else:
                if configuration == "AND":
                    si_region = BoolSurface(0, sid) * BoolSurface(0, pid)
                else:
                    si_region = BoolSurface(0, sid) * BoolSurface(0, -pid)
        else:
            if orientation == "Forward":
                if configuration == "AND":
                    si_region = BoolSurface(0, apid) * (BoolSurface(0, -sid) + BoolSurface(0, pid))
                else:
                    si_region = BoolSurface(0, apid) * (BoolSurface(0, -sid) + BoolSurface(0, -pid))
            else:
                if configuration == "AND":
                    si_region = BoolSurface(0, apid) + (BoolSurface(0, sid) * BoolSurface(0, pid))
                else:
                    si_region = BoolSurface(0, apid) + (BoolSurface(0, sid) * BoolSurface(0, -pid))

        region = region * si_region if configuration == "AND" else region + si_region

    return region


def tcone_region(cid, cone_orientation, surf_list):
    """
    Boolean AND/OR definition of a TCone meta-surface (a cone bounded by
    two planes p1/p2), built purely from already-resolved signed ids --
    same rationale and shared-by-two-callers reasoning as `can_region`.

    cid: id of the cone component.
    cone_orientation: "Forward" or "Reversed", orientation of the cone.
    surf_list: iterable of (pid, configuration), one per bounding plane;
        configuration is "AND" or "OR".
    """
    region = BoolSurface(0, -cid) if cone_orientation == "Forward" else BoolSurface(0, cid)

    for pid, configuration in surf_list:
        si_region = BoolSurface(0, pid) if configuration == "AND" else BoolSurface(0, -pid)
        region = region * si_region if configuration == "AND" else region + si_region

    return region


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
    def __init__(self, params):
        self.Center = params[0]
        self.Axis = params[1]
        self.Radius = params[2]

    def __str__(self):
        outstr = f"""Cylinder :
    Axis     : {self.Axis.x}  {self.Axis.y}  {self.Axis.z} 
    Center   : {self.Center.x}  {self.Center.y}  {self.Center.z}
    Radius   : {self.Radius}  """
        return outstr


class ConeOnlyParams:
    def __init__(self, params):
        self.Apex = params[0]
        self.Axis = params[1]
        self.SemiAngle = params[2]

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
        # A self-intersecting (degenerate) torus has two geometrically
        # distinct sheets sharing the same (Center, Axis, MajorRadius,
        # MinorRadius) -- see geo/surface_geometry.py's torus_sheet_sign
        # docstring. Callers that already know which sheet a face
        # belongs to (its GTorus descriptor's own .a_sign, computed once
        # at classification time from a real face vertex) pass it as an
        # optional 5th tuple element; every other caller gets the safe
        # default (1, "outer sheet"), matching a non-degenerate torus.
        self.Degenerated = self.MinorRadius > self.MajorRadius
        self.a_sign = params[4] if len(params) > 4 else 1

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
        # params[0] is a plain list of Tier-2 "Cylinder"/"Cone" GeounedSurface
        # objects, each already bundling its own additional plane (and, for
        # a Cone, its own ApexPlane) -- see MetaSurfacesDict.add_reversedCC
        # for how these combine into the RevCC's boolean region.
        #
        # params[1] (AdjacentMultiplanePlanes) is at most 2 real-face Plane
        # GeounedSurfaces (one per end of the whole chain) identifying which,
        # if any, of the chain's own 2 ends actually borders a MultiPlane --
        # a MultiPlane can make the irreducible solid non-convex, which is
        # exactly the configuration where the RevCC's own additional plane
        # (correct only locally) must not act as an unrestricted global cut.
        self.CylCones = params[0]
        self.AdjacentMultiplanePlanes = params[1]


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
