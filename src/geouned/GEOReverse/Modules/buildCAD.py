import BOPTools.SplitAPI
from tqdm import tqdm
import FreeCAD

from .buildSolidCell import FuseSolid
from .Objects import CadCell
from .Utils.booleanFunction import BoolSequence
from .Utils.boundBox import solid_plane_box


def interferencia(container, cell, mode="slice"):

    if mode == "common":
        return cell.shape.common(container.shape)

    Base = cell.shape
    Tool = (container.shape,)

    solids = BOPTools.SplitAPI.slice(Base, Tool, "Split", tolerance=0).Solids
    cellParts = []
    for s in solids:
        if container.shape.isInside(s.CenterOfMass, 0.0, False):
            cellParts.append(s)

    if not cellParts:
        return cell.shape
    else:
        return FuseSolid(cellParts)


def AssignSurfaceToCell(UniverseCells, modelSurfaces):

    for Uid, uniCells in UniverseCells.items():
        for cId, c in uniCells.items():
            c.setSurfaces(modelSurfaces)


def get_universe_containers(levels, Universes):
    Ucontainer = {}
    for lev in range(1, len(levels)):
        for U, name in levels[lev]:
            UFILL = Universes[U][name].FILL
            if UFILL in Ucontainer.keys():
                Ucontainer[UFILL].append((U, name, lev))
            else:
                Ucontainer[UFILL] = [(U, name, lev)]
    return Ucontainer


def BuildUniverseCells(startInfo, ContainerCell, AllUniverses, universeCut=True):

    CADUniverse = []
    Ustart, levelMax = startInfo
    Universe = AllUniverses[Ustart]

    print(f"Build Universe {ContainerCell.FILL} in container cell {ContainerCell.name}")
    fails = []
    #for NTcell in tqdm(Universe.values(), desc="build cell"):
    for NTcell in Universe.values():
        if NTcell.shape:
            buildShape = False
            if ContainerCell.CurrentTR:
                cell = NTcell.copy()
                cell.transformSolid(ContainerCell.CurrentTR)
            else:
                cell = NTcell
        else:
            CTRF = None
            buildShape = True

        if buildShape:
            
            print(NTcell.name)
            if type(NTcell.definition) is not BoolSequence:
                NTcell.definition = BoolSequence(NTcell.definition.str)

            if ContainerCell.shape is not None:
                external_box = ContainerCell.shape.BoundBox
                if ContainerCell.CurrentTR:
                    external_box = external_box.transformed(ContainerCell.CurrentTR.inverse())
            else:
                external_box = None

            solid_box = solid_plane_box(NTcell, outbox=external_box)
            bBox = solid_box.get_boundBox(0.1)
            if bBox.XLength < 1e-6 or bBox.YLength < 1e-6 or bBox.ZLength < 1e-6:
                if external_box is not None:
                    bBox = external_box
                else:
                    NTcell.shape = None
                    print(f"Cell {NTcell.name} BoundBox is null")
                    fails.append(NTcell.name)
                    continue

            debug = False
            if debug:
                NTcell.buildShape(bBox, simplify=False)
            else:
                try:
                    NTcell.buildShape(bBox, simplify=False)
                except:
                    print(f"fail converting cell {NTcell.name}")
                    fails.append(NTcell.name)

            if NTcell.shape is None:
                continue

            cell = NTcell.copy()
            if ContainerCell.CurrentTR:
                cell.transformSolid(ContainerCell.CurrentTR)

        if universeCut and ContainerCell.shape:
            cell.shape = interferencia(ContainerCell, cell)

        if not cell.FILL or ContainerCell.level + 1 == levelMax:
            CADUniverse.append(cell)
        else:
            if ContainerCell.CurrentTR:
                cell.CurrentTR = ContainerCell.CurrentTR.multiply(cell.TRFL)
            cell.level = ContainerCell.level + 1
            univ, ff = BuildUniverseCells((cell.FILL, levelMax), cell, AllUniverses, universeCut=universeCut)
            CADUniverse.append(univ)
            fails.extend(ff)

    return ((ContainerCell.name, Ustart), CADUniverse), fails


def makeTree(CADdoc, CADCells):
    
    label, universeCADCells = CADCells
    groupObj = CADdoc.addObject("App::Part", "Materials")

    groupObj.Label = f"Universe_{label[1]}_Container_{label[0]}"

    CADObj = {}
    for i, c in enumerate(universeCADCells):
        if isinstance(c,(tuple,list)):
            groupObj.addObject(makeTree(CADdoc, c))
        else:
            featObj = CADdoc.addObject("Part::FeaturePython", f"solid{i}")
            featObj.Label = f"Cell_{c.name}_{c.MAT}"
            featObj.Shape = c.shape
            if c.MAT not in CADObj.keys():
                CADObj[c.MAT] = [featObj]
            else:
                CADObj[c.MAT].append(featObj)

    for mat, matGroup in CADObj.items():
        groupMatObj = CADdoc.addObject("App::Part", "Materials")
        groupMatObj.Label = f"Material_{mat}_{label[0]}{label[1]}"
        groupMatObj.addObjects(matGroup)
        groupObj.addObject(groupMatObj)

    return groupObj
