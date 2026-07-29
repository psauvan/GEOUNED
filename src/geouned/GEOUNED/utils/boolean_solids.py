#
#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#
import logging
import math

from .boolean_function import BoolSequence, BoolSurface
from .geouned_classes import GeounedSurface
from ...geo import GSolid, GVector, Gdistance, Gsplit, to_gvector

BoolVals = (None, True, False)
primitives_surfaces = ("Plane", "CylinderOnly", "SphereOnly", "ConeOnly", "TorusOnly")

logger = logging.getLogger("general_logger")


class CTelement:
    def __init__(self, val=None, S1=None, S2=None):
        self.diagonal = False
        self.S1 = S1
        self.S2 = S2
        if val is None:
            self.val = None
        else:
            if type(val) is int:
                self.diagonal = True
                self.type = None
            else:
                self.type = val.count(0)
            self.val = val

    def get_transpose(self):
        if self.diagonal:
            return self.val
        return CTelement((self.val[0], self.val[3], self.val[2], self.val[1]), self.S2, self.S1)

    def get_dependence(self):
        if self.diagonal:
            if self.val == 0:
                return True, False
            elif self.val == -1:
                return "Null", False
            else:
                return True, "Null"

        Ones = sum(self.val)
        if Ones == 4:
            return None, None
        elif Ones == 3:
            ind = self.val.index(0)
            if ind == 0:
                return False, None
            elif ind == 1:
                return True, None
            elif ind == 2:
                return None, True
            else:
                return None, False
        elif Ones == 2:
            ind1 = self.val.index(0)
            ind2 = self.val.index(0, ind1 + 1)
            if ind1 == 0 and ind2 == 1:
                return "Null", None
            elif ind1 == 2 and ind2 == 3:
                return None, "Null"
            elif ind1 == 0 and ind2 == 3:
                return False, False
            else:
                return True, True
        elif Ones == 1:
            ind = self.val.index(1)
            if ind == 0:
                return True, "Null"
            elif ind == 1:
                return False, "Null"
            elif ind == 2:
                return "Null", False
            else:
                return "Null", True
        else:
            return "Null", "Null"


class ConstraintTable(dict):

    def __init__(self):
        self.diagonal = None

    def __str__(self):

        varName = list(self.keys())

        # Constraint Table only diagonal terms
        if len(self[varName[0]]) == 1:
            line = ""
            for name in varName:
                element = self[name][name]
                line += f" {name:4d} : {element.val}\n"
            return line

        outstr = "  "
        for name in varName:
            outstr = outstr + f" {name:3d}"
        outstr = outstr + "\n"

        for name1 in varName:
            line = f" {name1:3d} "
            linenot = f"~{name1:3d} "
            for name2 in varName:
                elmt = self[name1][name2]
                if elmt.diagonal:
                    line += f" {elmt.val:>2d} "
                    linenot += "    "
                else:
                    line += f" {elmt.val[0]}{elmt.val[1]} "
                    linenot += f" {elmt.val[3]}{elmt.val[2]} "
            outstr += line + "\n"
            outstr += linenot + "\n"
        return outstr

    def add_element(self, k1, k2, val):

        if k1 in self.keys():
            self[k1][k2] = val
        else:
            self[k1] = {k2: val}

    def fill_missing_elements(self):
        keys = list(self.keys())
        missing = []
        for i, k1 in enumerate(keys):
            for k2 in keys[i + 1 :]:
                if k2 in self[k1].keys():
                    elmt = self[k1][k2]
                    self[k2][k1] = elmt.get_transpose()
                else:
                    missing.append((k1, k2))

        for k1, k2 in missing:
            diag1 = self[k1][k1]
            diag2 = self[k2][k2]
            new = combine_diag_elements(diag1, diag2)
            new.S1 = k1
            new.S2 = k2
            self[k1][k2] = new
            self[k2][k1] = new.get_transpose()

    def get_out_surfaces(self):
        out = []
        for k in self.keys():
            if self[k][k].val != 0:
                out.append(k)
        return out

    def get_constraint_set(self, valname):
        trueSet = {}
        falseSet = {}
        TNull = False
        FNull = False
        for k in self.keys():
            TValue, FValue = self[valname][k].get_dependence()
            if TValue == "Null":
                TNull = True
                TValue = None
            if FValue == "Null":
                FNull = True
                FValue = None
            if TValue is not None:
                trueSet[k] = TValue
            if FValue is not None:
                falseSet[k] = FValue

        if TNull:
            trueSet = None
        if FNull:
            falseSet = None
        return trueSet, falseSet

    def solid_in_box(self, Seq):  #  Sequence of the cell
        surfs = tuple(Seq.get_surfaces_numbers())
        if self.diagonal:
            seqValues = dict()
            for s in surfs:
                val = BoolVals[self[s][s].val]
                if val is not None:
                    seqValues[s] = val
            # if evaluate   None  : Box intersection Cell != 0      Part of the cell in the box
            #               True  : Box intersection Cell == Box    Cell cover the full region of the box. Void cell doesn't exist
            #               False : Box intersection Cell == 0      Cell out of the box
            res = Seq.evaluate(seqValues)
            return res if type(res) is bool else None

        else:
            trueSet, falseSet = self.get_constraint_set(surfs[0])
            if trueSet is not None:
                trueVal = Seq.evaluate(trueSet)
                trueVal = trueVal if type(trueVal) is bool else None
                if trueVal is None:
                    return None
                if falseSet is not None:
                    falseVal = Seq.evaluate(falseSet)
                    falseVal = falseVal if type(falseVal) is bool else None
                    if falseVal is None:
                        return None  # Part of the cell in the box
                    if trueVal == falseVal:
                        return trueVal  # True Cover full cell, False not in the box
                    else:
                        return None  # Part of the cell in the box
                else:
                    return trueVal
            elif falseSet is not None:
                falseVal = Seq.evaluate(falseSet)
                return falseVal if type(falseVal) is bool else None
            else:
                logger.info("Bad trouble surfaces is on none side of the box!!")
                return False


def combine_diag_elements(d1, d2):
    if d1.val == 0 and d2.val == 0:
        return CTelement((1, 1, 1, 1))
    elif d1.val == 1 and d2.val == 0:
        return CTelement((1, 1, 0, 0))
    elif d1.val == -1 and d2.val == 0:
        return CTelement((0, 0, 1, 1))
    elif d1.val == 0 and d2.val == 1:
        return CTelement((1, 0, 0, 1))
    elif d1.val == 0 and d2.val == -1:
        return CTelement((0, 1, 1, 0))
    elif d1.val == 1 and d2.val == 1:
        return CTelement((1, 0, 0, 0))
    elif d1.val == 1 and d2.val == -1:
        return CTelement((0, 1, 0, 0))
    elif d1.val == -1 and d2.val == 1:
        return CTelement((0, 0, 0, 1))
    elif d1.val == -1 and d2.val == -1:
        return CTelement((0, 0, 1, 0))


def build_c_table_from_solids(Box, SurfInfo, simplification_mode, options, omit_surfaces=set()):

    # Box is a GSolid when it comes from the (already migrated) void
    # pipeline, or a native Part.Shape when it comes from callers not
    # migrated yet (cell_definition.py/core.py's get_box) -- accept both.
    box_native = Box.__native__ if type(Box) is GSolid else Box

    if type(SurfInfo) is dict:
        surfaces = SurfInfo
        surfaceList = tuple(surfaces.keys())
    elif type(SurfInfo) is tuple:
        surfaceList, surfaces = SurfInfo
    else:
        surfaces = SurfInfo.Surfaces
        surfaceList = SurfInfo.surfaceList

    if type(surfaces[surfaceList[0]]) is GeounedSurface:
        for s in surfaceList:
            ss = surfaces[s]
            ss.build_surface(box_native.BoundBox)
    else:
        for s in surfaceList:
            surfaces[s].buildShape(box_native.BoundBox)

    CTable = ConstraintTable()
    if simplification_mode == "diag":
        CTable.diagonal = True
    else:
        CTable.diagonal = False

    for i, s1 in enumerate(surfaceList):
        if s1 not in omit_surfaces:
            res, splitRegions = split_solid_fast(Box, surfaces[s1], True, options)
        else:
            res = 0

        CTable.add_element(s1, s1, CTelement(res, s1, s1))
        if simplification_mode == "diag":
            continue

        if s1 in omit_surfaces:
            val = (1, 1, 1, 1)
            for s2 in surfaceList[i + 1 :]:
                CTable.add_element(s1, s2, CTelement(val, s1, s2))
            continue

        if splitRegions is None:
            if res == 0:
                # case that s1 cut the box but the split fails and return only one solids
                # the following function try to split box with surface s2 instead surface s1
                split_s2_s1((i, s1), Box, CTable, surfaceList, surfaces, omit_surfaces, options)
            continue  # loop, no region to be split by s2

        for s2 in surfaceList[i + 1 :]:
            if s2 in omit_surfaces:
                CTable.add_element(s1, s2, CTelement((1, 1, 1, 1), s1, s2))
                continue

            posS1, negS1 = splitRegions

            pos0 = None
            for solid in posS1:
                pos = split_solid_fast(solid, surfaces[s2], False, options)
                if pos == (1, 1):
                    break  # s2 intersect S1 Region
                if pos0 is None:
                    pos0 = pos
                else:
                    if pos != pos0:  # s1 regions are on both side of s2
                        pos = (1, 1)
                        break

            neg0 = None
            for solid in negS1:
                neg = split_solid_fast(solid, surfaces[s2], False, options)
                if neg == (1, 1):
                    break  # s2 intersect S1 Region
                if neg0 is None:
                    neg0 = neg
                else:
                    if neg != neg0:  # s1 regions are on both side of s2
                        neg = (1, 1)
                        break

            val = (pos[0], pos[1], neg[1], neg[0])
            CTable.add_element(s1, s2, CTelement(val, s1, s2))

    # if some surfaces don't cross the box some elements in Constraint table are not filled
    if simplification_mode != "diag":
        CTable.fill_missing_elements()
    return CTable


def split_s2_s1(s1tuple, Box, CTable, surfaceList, surfaces, omit_surfaces, options):
    i1, s1 = s1tuple
    for s2 in surfaceList[i1 + 1 :]:
        if surfaces[s2].shape:
            res, splitRegions = split_solid_fast(Box, surfaces[s2], True, options)
        else:
            res = check_sign(solid, surfaces[s2]), None

        if s2 in omit_surfaces:
            CTable.add_element(s1, s2, CTelement((1, 1, 1, 1), s1, s2))
            continue

        if splitRegions is None:
            continue

        posS2, negS2 = splitRegions
        pos0 = None
        for solid in posS2:
            pos = split_solid_fast(solid, surfaces[s1], False, options)
            if pos == (1, 1):
                break  # s1 intersect S2 Region
            if pos0 is None:
                pos0 = pos
            else:
                if pos != pos0:  # s2 regions are on both side of s1
                    pos = (1, 1)
                    break

        neg0 = None
        for solid in negS2:
            neg = split_solid_fast(solid, surfaces[s1], False, options)
            if neg == (1, 1):
                break  # s2 intersect S1 Region
            if neg0 is None:
                neg0 = neg
            else:
                if neg != neg0:  # s1 regions are on both side of s2
                    neg = (1, 1)
                    break

        val = (pos[0], pos[1], neg[1], neg[0])
        CTable.add_element(s1, s2, CTelement(val, s1, s2).get_transpose())


def remove_extra_surfaces(CellSeq, CTable):
    # checking is make on solid cell definition to be removed from void cell
    outSurfaces = set(CTable.get_out_surfaces())
    newDef = BoolSequence(operator="OR")

    # Loop over all compound solids of the metaSolid

    if CellSeq.level == 0 and len(CellSeq.elements) == 1:
        return CellSeq

    if CellSeq.operator == "AND":
        newSeq = BoolSequence(operator="AND")
        newSeq.append(CellSeq.copy())
        CellSeq.assign(newSeq)

    for subCell in CellSeq.elements:
        nullcell = False
        if type(subCell) is not BoolSurface:
            subCell.check()
        if type(subCell.elements) is bool:
            chk = not subCell.elements
        else:
            chk = None

        if chk is False:  # the cell doesn't exist
            nullcell = True
        elif chk is True:  # the cell describe the full universe
            newDef.elements = True
            newDef.level = -1
            return newDef

        # if subcell has finite volume check it intersection with the box
        if not nullcell:
            res = CTable.solid_in_box(subCell)
            if res is None:
                # subcell intersect the box
                # get the surfaces of the solids out of the box
                # get reduced definition

                # if subcell lev!= 0 remove surface operation is not valid
                if subCell.level == 0:
                    removeSurf = outSurfaces & subCell.get_surfaces_numbers()
                    for s in removeSurf:
                        val = True if CTable[s][s].val > 0 else False
                        subCell.substitute(s, val)

                if type(subCell.elements) is bool:
                    if subCell.elements is False:  #  cell does not intersect void box
                        continue
                    else:  # cell cover fully void box
                        newDef.elements = True
                        newDef.level = -1
                        return newDef
                else:
                    newDef.append(subCell)

            elif res is True:
                # subcell cover the full box region Void cell doesn't exist
                newDef.elements = True
                newDef.level = -1
                return newDef

    newDef.clean()

    return newDef


def split_solid_fast(solid, surf, box, options):

    if box:
        if surf.shape:
            result = Gsplit(
                GSolid(solid), GSolid(surf.shape), options.splitTolerance,
                scale_up_floor=options.splitTolerance if options.scaleUp else None,
            )
            comsolid_solids = [s.__native__ for s in result.solids]
        else:
            return check_sign(solid, surf), None

        if len(comsolid_solids) <= 1:
            if len(comsolid_solids) == 1:
                res = split_solid_fast(solid, surf, False, options)
                if res == (1, 1):
                    return 0, None
                else:
                    return check_sign(solid, surf), None
            else:
                return check_sign(solid, surf), None
        # sgn = check_sign(solid,surf)   # if "box" and single object => the box is not split, surface s1 out of the box.
        # if sgn == 1 :                 # The sign is the side of surface s1 where the box is located
        #    # return ((1,0),(0,0)),None  # return the diagonal element of the Constraint Table for s1
        #    return ((1,0),(0,0)),None  # return the diagonal element of the Constraint Table for s1
        # else:
        #    return ((0,0),(0,1)),None

        else:
            posSol = []
            negSol = []
            for s in comsolid_solids:
                sgn = check_sign(s, surf)
                if sgn == 1:
                    posSol.append(s)
                else:
                    negSol.append(s)
            return 0, (
                posSol,
                negSol,
            )  # return the diagonal element of the Constraint Table for s1, and solids to be split by s2
            # return ((1,0),(0,1)), (posSol,negSol) # return the diagonal element of the Constraint Table for s1, and solids to be split by s2

    else:
        # "not box" => return the position of the +/- region of s1 (the solid) with respect s2
        if surf.shell:
            dist = Gdistance(GSolid(solid), GSolid(surf.shell))
            if dist > 1e-6:
                # chech if surf and solid don't intersect actually (native call: distToShape's
                # positive-distance report can be a false negative for a degenerate/tangent
                # contact, e.g. touching along a zero-area line -- common()'s Area is the
                # authoritative check in that case, not covered by the generic `in_contact`)
                cc = solid.common(surf.shell)
                if abs(cc.Area) > 0:
                    dist = 0
        else:
            dist = 1.0
            # volume = 0
        if dist > 1e-6:  # face doesn't intersect solid
            # if volume < 1e-6:  # face doesn't intersect solid

            sgn = check_sign(solid, surf)
            if sgn == 1:
                return (
                    1,
                    0,
                )  # values of s2 and -s2   "0" means this region doesn't exist
            else:
                return (0, 1)
        else:
            return (1, 1)  # values of s2 and -s2


# find one point inside a solid (region)
def point_inside(solid):
    gsolid = GSolid(solid)
    point = gsolid.find_interior_point()
    if point is None:
        logger.info(f"Solid not found in bounding Box (Volume : {gsolid.Volume})")
    return point


def check_sign(solid_or_point, surf):

    if type(solid_or_point) is GVector:
        point = solid_or_point
    else:
        point = point_inside(solid_or_point)

    if surf.Type in primitives_surfaces:
        return check_sign_primitive(point, surf)

    elif surf.Type == "Cylinder":
        return check_sign(point, surf.Surf.Cylinder)

    elif surf.Type == "Cone":
        return check_sign(point, surf.Surf.Cone)

    elif surf.Type == "Sphere":
        return check_sign(point, surf.Surf.Sphere)

    elif surf.Type == "Torus":
        return check_sign(point, tor=surf.Surf.Torus)

    elif surf.Type == "MultiPlane":
        for plane in surf.Surf.Planes:
            p_sign = check_sign(point, plane)
            if p_sign == 1:
                return 1
        return -1

    elif surf.Type == "Can" or surf.Type == "TCone":
        if surf.Type == "Can":
            can_surfaces = [surf.Surf.Cylinder.Surf.Cylinder]
            s12 = (surf.Surf.s1, surf.Surf.s2)
            for si in s12:
                if si.Type == "Plane":
                    can_surfaces.append(si)
                elif si.Type == "cylinder":
                    can_surfaces.append(si.Surf.Plane)
                    can_surfaces.append(si.Surf.Cylinder)
                elif si.Type == "Cone":
                    can_surfaces.append(si.Surf.Cone)
                    if si.Surf.ApexPlane is not None:
                        can_surfaces.append(si.Surf.ApexPlane)
                    if si.Surf.Plane is not None:
                        can_surfaces.append(si.Surf.Plane)

                elif si.Type == "Sphere":
                    can_surfaces.append(si.Surf.Plane)
                    can_surfaces.append(si.Surf.Sphere)

            surfSet = dict()
            for s in can_surfaces:
                surfSet[s.bVar] = check_sign(point, s) > 0

            inside = surf.region.region.evaluate(surfSet)
            return 1 if inside else -1

        else:
            tcone_surfaces = (surf.Surf.Cone.Surf.Cone, surf.Surf.p1, surf.Surf.p2)
            surfSet = dict()
            for s in tcone_surfaces:
                surfSet[s.bVar] = check_sign(point, s) > 0

            inside = surf.region.region.evaluate(surfSet)
            return 1 if inside else -1

    elif surf.Type == "RoundCorner":
        multiDef = surf.region.region.copy()
        p1, p2 = surf.Surf.Planes
        if p1 == p2:
            planes = (p1,)
        else:
            planes = (p1, p2)
        for plane in planes:
            value = check_sign(point, plane) > 0
            multiDef = multiDef.evaluate({plane.bVar: value})
            if type(multiDef) is bool:
                return 1 if multiDef else -1

        cyl = surf.Surf.Cylinder.Surf.Cylinder
        value = check_sign(point, cyl) > 0
        multiDef = multiDef.evaluate({cyl.bVar: value})
        if type(multiDef) is bool:
            return 1 if multiDef else -1

        cplane = surf.Surf.Cylinder.Surf.Plane
        if cplane is not None:
            value = check_sign(point, cplane) > 0
            multiDef = multiDef.evaluate({cplane.bVar: value})
            if type(multiDef) is bool:
                return 1 if multiDef else -1

    elif surf.Type == "MultiRoundCorner":

        multiDef = surf.region.region.copy()
        for plane in surf.Surf.Planes:
            value = check_sign(point, plane) > 0
            multiDef = multiDef.evaluate({plane.bVar: value})
            if type(multiDef) is bool:
                return 1 if multiDef else -1

        for rc in surf.Surf.Corners:
            plane = rc.Surf.Plane
            if plane is not None:
                value = check_sign(point, plane) > 0
                multiDef = multiDef.evaluate({plane.bVar: value})
                if type(multiDef) is bool:
                    return 1 if multiDef else -1

            cyl = rc.Surf.Cylinder
            value = check_sign(point, cyl) > 0
            multiDef = multiDef.evaluate({cyl.bVar: value})
            if type(multiDef) is bool:
                return 1 if multiDef else -1


def check_sign_primitive(point, surf):

    if surf.Type == "Plane":
        r = point - to_gvector(surf.Surf.Position)
        if to_gvector(surf.Surf.Axis).dot(r) > 0:
            return 1
        else:
            return -1

    elif surf.Type == "CylinderOnly":
        r = point - to_gvector(surf.Surf.Center)
        L2 = r.length * r.length
        z = to_gvector(surf.Surf.Axis).dot(r)
        z2 = z * z
        R2 = surf.Surf.Radius * surf.Surf.Radius
        if L2 - z2 > R2:
            return 1
        else:
            return -1

    elif surf.Type == "SphereOnly":
        r = point - to_gvector(surf.Surf.Center)
        if r.length > surf.Surf.Radius:
            return 1
        else:
            return -1

    elif surf.Type == "ConeOnly":
        r = (point - to_gvector(surf.Surf.Apex)).normalized()
        z = round(to_gvector(surf.Surf.Axis).dot(r), 15)
        alpha = math.acos(z)

        if alpha > surf.Surf.SemiAngle:
            return 1
        else:
            return -1

    elif surf.Type == "TorusOnly":
        axis = to_gvector(surf.Surf.Axis)
        r = point - to_gvector(surf.Surf.Center)
        h = r.dot(axis)
        rho = r - h * axis

        rp = math.sqrt((rho.length - surf.Surf.MajorRadius) ** 2 + h**2)
        if rp > surf.Surf.MinorRadius:
            return 1
        else:
            return -1


def get_kne_planes(Surfaces):

    kne_planes = set()
    for kne in Surfaces["Cone"]:
        for index in kne.region.get_surfaces_numbers():
            if Surfaces.primitive_surfaces.get_surface(index).Type == "Plane":
                kne_planes.add(index)
    # return kne_planes
    return set()
