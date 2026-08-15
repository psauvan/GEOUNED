import numpy as np
from tqdm import tqdm

from ._geo_bridge import Gcommon, Gsplit, fuse_solids, to_gmatrix_from_np
from .Utils.booleanFunction import BoolSequence
from .Utils.boundBox import myBox


def interferencia(container, cell, mode="slice"):

    if mode == "common":
        return fuse_solids(Gcommon(cell.shape, [container.shape]))

    solids = Gsplit(cell.shape, container.shape, tolerance=0).solids

    cellParts = [s for s in solids if container.shape.is_inside(s.center_of_mass())]

    if not cellParts:
        return cell.shape
    else:
        return fuse_solids(cellParts)


def AssignSurfaceToCell(UniverseCells, modelSurfaces):

    for Uid, uniCells in UniverseCells.items():
        for c in uniCells.values():
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

    if ContainerCell.name is not None:
        print(f"Build Universe {ContainerCell.FILL} in container cell {ContainerCell.name}")
    else:
        print(f"Build Universe {ContainerCell.FILL}")
    fails = []
    for NTcell in tqdm(Universe.values(), desc="build cell"):

        if NTcell.shape:
            buildShape = False
            if ContainerCell.CurrentTR is not None:
                cell = NTcell.copy()
                cell.transformSolid(ContainerCell.CurrentTR)
            else:
                cell = NTcell
        else:
            CTRF = None
            buildShape = True

        if buildShape:
            if type(NTcell.definition) is not BoolSequence:
                NTcell.definition = BoolSequence(NTcell.definition.str)

            if ContainerCell.shape is not None:
                external_box = myBox(ContainerCell.shape.BoundBox, "Forward")
                if ContainerCell.CurrentTR is not None:
                    inv = np.linalg.inv(ContainerCell.CurrentTR)
                    external_box.Box = external_box.Box.transformed(to_gmatrix_from_np(inv))
            else:
                external_box = ContainerCell.externalBox

            try:
                NTcell.build_BoundBox(external_box, enlarge=0.2)
                if NTcell.boundBox.Orientation == "Forward" and NTcell.boundBox.Box is None:
                    NTcell.shape = None
                else:
                    if NTcell.boundBox.Orientation == "Forward":
                        NTcell.externalBox = NTcell.boundBox
                    NTcell.buildShape(simplify=False)
            except:
                fails.append(NTcell.name)
                continue

            if NTcell.shape is None:
                continue

            cell = NTcell.copy()
            if ContainerCell.CurrentTR is not None:
                cell.transformSolid(ContainerCell.CurrentTR)

        if universeCut and ContainerCell.shape:
            cell.shape = interferencia(ContainerCell, cell)

        if not cell.FILL or ContainerCell.level + 1 > levelMax:
            CADUniverse.append(cell)
        else:
            if ContainerCell.CurrentTR is not None:
                cell.CurrentTR = ContainerCell.CurrentTR @ cell.TRFL
            cell.level = ContainerCell.level + 1
            univ, ff = BuildUniverseCells((cell.FILL, levelMax), cell, AllUniverses, universeCut=universeCut)
            CADUniverse.append(univ)
            fails.extend(ff)

    return ((ContainerCell.name, Ustart), CADUniverse), fails
