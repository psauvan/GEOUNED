"""File with the VoidBox class"""

import logging

from ..conversion import cell_definition as Conv
from ..decompose.decom_one_generators import main_split
from ..utils.boolean_function import BoolSequence, BoolVariable, BoolSurface
from ..utils.boolean_solids import build_c_table_from_solids, remove_extra_surfaces, get_kne_planes
from ..utils.geouned_classes import GeounedSolid, GeounedSurface
from ...geo import GBoundBox, GSolid, GVector, Gcommon, Gdistance, Gmake_box, to_gboundbox

logger = logging.getLogger("general_logger")


class VoidBox:
    def __init__(self, MetaSolids, Box, EnclosureCAD=None, Definition=None):

        self.Objects = []
        box = Box if type(Box) is GBoundBox else to_gboundbox(Box)
        if EnclosureCAD is None:
            self.BoundBox = box
            self.PieceEnclosure = None
            self.isEnclosure = False
            self.Definition = None
        else:
            self.BoundBox = box
            self.PieceEnclosure = EnclosureCAD if type(EnclosureCAD) is GSolid else GSolid(EnclosureCAD)
            self.isEnclosure = True
            self.Definition = Definition

        for m in MetaSolids:
            if not m.BoundBox:
                continue
            if m.BoundBox.is_valid():
                if self.BoundBox.intersects(m.BoundBox):
                    Obj = self.copy_meta(m)
                    self.remove_extra_comp(Obj, self.BoundBox)
                    self.Objects.append(Obj)
        return

    def split(self, minSize=200):

        dims = [self.BoundBox.XLength, self.BoundBox.YLength, self.BoundBox.ZLength]
        coord = ["X", "Y", "Z"]
        for i in range(2, -1, -1):
            if 0.5 * dims[i] < minSize:
                del dims[i]
                del coord[i]

        if len(dims) == 0:
            return None

        ip = dims.index(max(dims))
        #    print ('dims : {} {}'.format(coord[ip],0.5*dims[ip]))
        if coord[ip] == "X":
            pos = self.BoundBox.XMin + 0.5 * self.BoundBox.XLength
            X1Min = self.BoundBox.XMin
            X1Max = pos
            X2Min = pos
            X2Max = self.BoundBox.XMax
            Y1Min = self.BoundBox.YMin
            Y1Max = self.BoundBox.YMax
            Y2Min = Y1Min
            Y2Max = Y1Max
            Z1Min = self.BoundBox.ZMin
            Z1Max = self.BoundBox.ZMax
            Z2Min = Z1Min
            Z2Max = Z1Max
        elif coord[ip] == "Y":
            pos = self.BoundBox.YMin + 0.5 * self.BoundBox.YLength
            X1Min = self.BoundBox.XMin
            X1Max = self.BoundBox.XMax
            X2Min = X1Min
            X2Max = X1Max
            Y1Min = self.BoundBox.YMin
            Y1Max = pos
            Y2Min = pos
            Y2Max = self.BoundBox.YMax
            Z1Min = self.BoundBox.ZMin
            Z1Max = self.BoundBox.ZMax
            Z2Min = Z1Min
            Z2Max = Z1Max
        else:
            pos = self.BoundBox.ZMin + 0.5 * self.BoundBox.ZLength
            X1Min = self.BoundBox.XMin
            X1Max = self.BoundBox.XMax
            X2Min = X1Min
            X2Max = X1Max

            Y1Min = self.BoundBox.YMin
            Y1Max = self.BoundBox.YMax
            Y2Min = Y1Min
            Y2Max = Y1Max

            Z1Min = self.BoundBox.ZMin
            Z1Max = pos
            Z2Min = pos
            Z2Max = self.BoundBox.ZMax

        box1 = GBoundBox(X1Min, Y1Min, Z1Min, X1Max, Y1Max, Z1Max)
        box2 = GBoundBox(X2Min, Y2Min, Z2Min, X2Max, Y2Max, Z2Max)

        if self.PieceEnclosure is None:
            Space1 = VoidBox(self.Objects, box1)
            Space2 = VoidBox(self.Objects, box2)
            VoidBoxTuple = (Space1, Space2)
        else:
            Space1 = self.piece_enclosure_split(box1)
            Space2 = self.piece_enclosure_split(box2)
            VoidBoxTuple = (Space1, Space2)
            if Space1 is None:
                VoidBoxTuple = (Space2,)
            if Space2 is None:
                VoidBoxTuple = (Space1,)
            if Space1 is None and Space2 is None:
                VoidBoxTuple = ()

        return VoidBoxTuple

    def piece_enclosure_split(self, Box, Tolerance=1.0e-13):
        """This function creates a box-shaped solid with the new limits of given bounding box and
        it is intersected with the piece of nested enclosure to create the new void cell.
        If the limited region does not intersect with the piece, no void cell is created.
        """

        cube = Gmake_box(Box.XMin, Box.YMin, Box.ZMin, Box.XMax, Box.YMax, Box.ZMax)
        dist = Gdistance(cube, self.PieceEnclosure)
        try:
            if abs(dist / Box.DiagonalLength) > Tolerance:
                return None
        except ZeroDivisionError:
            return None
        common_solids = Gcommon(cube, [self.PieceEnclosure])
        cube_volume = cube.Volume
        common_volume = sum(s.Volume for s in common_solids)
        try:
            reldif = (cube_volume - common_volume) / cube_volume
        except ZeroDivisionError:
            return None
        if abs(reldif) <= Tolerance:
            return VoidBox(self.Objects, Box, cube, self.Definition)
        elif common_solids:
            return VoidBox(self.Objects, Box, common_solids[0], self.Definition)
        else:
            return None

    def refine(self):
        Cube = Gmake_box(
            self.BoundBox.XMin,
            self.BoundBox.YMin,
            self.BoundBox.ZMin,
            self.BoundBox.XMax,
            self.BoundBox.YMax,
            self.BoundBox.ZMax,
        )

        for m in self.Objects:
            self.remove_extra_comp(m, Cube, mode="dist")
        return

    def get_void_complementary(self, Surfaces, options, simplify="no"):

        bBox = self.BoundBox
        if self.PieceEnclosure is None:
            boxDef = BoolSequence(operator="AND")
            enclosure = False
            d = options.enlargeBox

        else:
            boxDef = self.Definition.copy()
            if boxDef.operator == "OR":
                ANDDef = BoolSequence(operator="AND")
                ANDDef.append(boxDef)
                boxDef = ANDDef
            enclosure = True
            d = max(options.enlargeBox, 2)

        for p in self.get_bound_planes():
            plane_region = Surfaces.add_plane(p, False)
            if boxDef.base_type is BoolVariable:
                boxDef.append(plane_region.region)
            else:
                boxDef.append(plane_region)

        Box = Gmake_box(
            bBox.XMin - d,
            bBox.YMin - d,
            bBox.ZMin - d,
            bBox.XMax + d,
            bBox.YMax + d,
            bBox.ZMax + d,
        )

        voidSolidDef = BoolSequence(operator="OR")

        cellIn = []
        for m in self.Objects:
            voidSolidDef.append(m.Definition)
            if m.IsEnclosure:
                continue
            cellIn.append(m.__id__)

        # voidSolidDef.join_operators()

        if not voidSolidDef.elements:
            return (
                boxDef,
                None,
            )  #  Cell to get complementary are null => Void is only box definition

        # join all basic solids into one big meta Object
        # CAD solid representation is not needed because
        # here we are working with surfaces and void box
        voidSolidDef.same_level()
        complementary = BoolSequence(operator="AND")
        complementary.append(boxDef)
        if (
            simplify != "no" and False
        ):  # temporary removed because remove_extra_surface not modified to handle correctly surface regions
            surfList = voidSolidDef.get_regions()

            if enclosure:
                surfList.update(boxDef.get_regions())
            else:
                for s in boxDef.elements:
                    val = s > 0
                    voidSolidDef.substitute(abs(s), val)
                voidSolidDef.clean()
            if type(voidSolidDef.elements) is bool:
                res = voidSolidDef.elements
            else:
                res = None

            if enclosure or res is None:
                surfaceDict = {}
                for i in surfList:
                    surfaceDict[i] = Surfaces.get_surface(i)
                CTable = build_c_table_from_solids(Box, surfaceDict, simplify, options=options)
            else:
                if res is True:
                    return None, None
                else:
                    return boxDef, None

            newTemp = BoolSequence(operator="OR")

            if voidSolidDef.level == 0:
                if len(voidSolidDef.elements) == 1:
                    voidSolidDef.operator = "AND"
                cellVoid = BoolSequence(operator="OR")
                cellVoid.append(voidSolidDef)
                voidSolidDef = cellVoid

            for solDef in voidSolidDef.elements:
                newSolid = remove_extra_surfaces(solDef, CTable)
                if type(newSolid.elements) is not bool:
                    newTemp.append(newSolid)
                elif newSolid.elements is True:
                    return None, None

            voidSolidDef = newTemp
        else:
            if voidSolidDef.level == 0:
                if len(voidSolidDef.elements) == 1:
                    voidSolidDef.operator = "AND"
                cellVoid = BoolSequence(operator="OR")
                cellVoid.append(voidSolidDef)
                voidSolidDef = cellVoid

        if voidSolidDef.elements is True:
            return None, None
        elif voidSolidDef.elements is False or voidSolidDef.elements == []:
            return boxDef, None

        if voidSolidDef.level == 0:
            compSeq = voidSolidDef.get_complementary()
        else:
            if voidSolidDef.level == 1 and voidSolidDef.operator == "AND":
                compSeq = BoolSequence(operator="OR")
            else:
                compSeq = BoolSequence(operator="AND")

            for comp in voidSolidDef.elements:
                if simplify == "no":
                    comp.check()
                    if type(comp.elements) is bool:
                        chk = comp.elements
                    else:
                        chk = None

                    # solid in cover full Void cell volume  => Void cell doesn't exist
                    if chk is True:
                        logger.warning("void Cell should not exist")
                        return None, None

                    # solid cell is not in void cell Void cell volume  => doesn't contribute to void definition
                    elif chk is False:
                        continue

                pmoc = comp.get_complementary()
                compSeq.append(pmoc)

        # if compSeq.base_type is BoolVariable and complementary.base_type is BoolRegion:
        #    complementary.expand_regions_to_boolVar()
        complementary.expand_regions_to_boolVar()

        if simplify == "full":
            # compSeq.simplify(CTable)
            compSeq.expand_regions_to_boolVar()
            surfaceDict = {}

            if enclosure:
                complementary.append(compSeq)
                primitive_surfs = complementary.get_surfaces_numbers()
                for i in primitive_surfs:
                    surfaceDict[i] = Surfaces.get_primitive_surface(i)

                kne_planes = get_kne_planes(Surfaces)
                kne_planes = kne_planes.intersection(primitive_surfs)

                CTable = build_c_table_from_solids(Box, surfaceDict, simplify, options=options, omit_surfaces=kne_planes)
                complementary.simplify(CTable)
            else:
                primitive_surfs = compSeq.get_surfaces_numbers()
                for i in primitive_surfs:
                    surfaceDict[i] = Surfaces.get_primitive_surface(i)

                kne_planes = get_kne_planes(Surfaces)
                kne_planes = kne_planes.intersection(primitive_surfs)

                CTable = build_c_table_from_solids(Box, surfaceDict, simplify, options=options, omit_surfaces=kne_planes)
                compSeq.simplify(CTable)
                complementary.append(compSeq)
        else:
            compSeq.expand_regions_to_boolVar()
            compSeq.simplify(None)
            complementary.simplify(None, outOp="AND")
            complementary.append(compSeq)

        complementary.clean()
        complementary.level_update()

        if type(complementary.elements) is bool:
            return None, None
        else:
            return complementary, cellIn

    def get_numbers(self):
        ns = 0
        nb = 0

        for m in self.Objects:
            ns += len(m.Surfaces)
            nb += len(m.Definition.elements) if m.Definition.level > 0 else 1

        return ns, nb

    def get_bound_planes(self):
        Xmid = 0.5 * (self.BoundBox.XMin + self.BoundBox.XMax)
        Ymid = 0.5 * (self.BoundBox.YMin + self.BoundBox.YMax)
        Zmid = 0.5 * (self.BoundBox.ZMin + self.BoundBox.ZMax)
        LX = self.BoundBox.ZMin + self.BoundBox.XLength
        LY = self.BoundBox.ZMin + self.BoundBox.YLength
        LZ = self.BoundBox.ZMin + self.BoundBox.ZLength
        PXMin = GeounedSurface(
            (
                "Plane",
                (
                    GVector(self.BoundBox.XMin, Ymid, Zmid),
                    GVector(1, 0, 0),
                    LY,
                    LZ,
                ),
            ),
            self.BoundBox,
        )
        PXMax = GeounedSurface(
            (
                "Plane",
                (
                    GVector(self.BoundBox.XMax, Ymid, Zmid),
                    GVector(-1, 0, 0),
                    LY,
                    LZ,
                ),
            ),
            self.BoundBox,
        )
        PYMin = GeounedSurface(
            (
                "Plane",
                (
                    GVector(Xmid, self.BoundBox.YMin, Zmid),
                    GVector(0, 1, 0),
                    LZ,
                    LX,
                ),
            ),
            self.BoundBox,
        )
        PYMax = GeounedSurface(
            (
                "Plane",
                (
                    GVector(Xmid, self.BoundBox.YMax, Zmid),
                    GVector(0, -1, 0),
                    LZ,
                    LX,
                ),
            ),
            self.BoundBox,
        )
        PZMin = GeounedSurface(
            (
                "Plane",
                (
                    GVector(Xmid, Ymid, self.BoundBox.ZMin),
                    GVector(0, 0, 1),
                    LX,
                    LY,
                ),
            ),
            self.BoundBox,
        )
        PZMax = GeounedSurface(
            (
                "Plane",
                (
                    GVector(Xmid, Ymid, self.BoundBox.ZMax),
                    GVector(0, 0, -1),
                    LX,
                    LY,
                ),
            ),
            self.BoundBox,
        )

        return (PXMin, PXMax, PYMin, PYMax, PZMin, PZMax)

    def remove_extra_comp(self, Obj, Box, mode="box"):
        reducedSol = []
        reducedDef = BoolSequence(operator="OR")
        if not Obj.Solids:
            return
        # Compare Solid BoundBox (here Box is a GBoundBox)
        if mode == "box":
            for i, sol in enumerate(Obj.Solids):
                if to_gboundbox(sol.BoundBox).is_valid():
                    if Box.intersects(to_gboundbox(sol.BoundBox)):
                        reducedSol.append(sol)
                        reducedDef.append(Obj.Definition.elements[i])

        # Compare solid using distance (here Box is a GSolid Cube)
        else:
            for i, sol in enumerate(Obj.Solids):
                dist = Gdistance(Box, GSolid(sol))
                if dist == 0:
                    reducedSol.append(sol)
                    reducedDef.append(Obj.Definition.elements[i])

        if len(reducedSol) < len(Obj.Solids):
            Obj.update_solids(reducedSol)
            Obj.set_definition(reducedDef)
        return

    def copy_meta(self, m):
        solidsCopy = m.Solids[:]
        facesCopy = m.Faces[:]
        Meta = GeounedSolid(m.__id__, solidsCopy)
        Meta.set_definition(m.Definition.copy())
        Meta.set_faces(facesCopy)
        if m.IsEnclosure:
            Meta.IsEnclosure = True
            Meta.EnclosureID = m.EnclosureID
        return Meta
