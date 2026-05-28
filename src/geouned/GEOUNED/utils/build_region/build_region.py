import Part

from .splitFunction import SplitBase, SplitSolid, joinBase
from .Objects import CellObj, Plane, Cylinder, myBox
from ..boolean_function import BoolSurface, BoolSequence
from ..basic_functions_part1 import round_corner_region

def get_cell_object(geoObj):

    cell = CellObj()

    if geoObj.Type == "RoundCorner":
        cid = geoObj.Surf.Cylinder.Surf.Cylinder.bVar
        plane1, plane2 = geoObj.Surf.Planes
        if plane1 == plane2:
            p1id = plane1.bVar
            cell.surfaces[abs(p1id)] = get_surface(abs(p1id), plane1)
            cell.surfaces[abs(cid)] = get_surface(abs(cid), geoObj.Surf.Cylinder.Surf.Cylinder)
            region = round_corner_region(p1id, p1id, cid, 0, geoObj.Surf.Configuration)
        else:
            p1id = plane1.bVar
            p2id = plane2.bVar
            cell.surfaces[abs(p1id)] = get_surface(abs(p1id), plane1)
            cell.surfaces[abs(p2id)] = get_surface(abs(p2id), plane2)
            cell.surfaces[abs(cid)] = get_surface(abs(cid), geoObj.Surf.Cylinder.Surf.Cylinder)
            if geoObj.Surf.Cylinder.Surf.Plane is not None:
                pcid = geoObj.Surf.Cylinder.Surf.Plane.bVar
                cell.surfaces[abs(pcid)] = get_surface(abs(pcid), geoObj.Surf.Cylinder.Surf.Plane)
            else:
                pcid = 0    
            
            region = round_corner_region(p1id, p2id, cid, pcid, geoObj.Surf.Configuration)   

    elif geoObj.Type == "MultiRoundCorner":
        region = None
        Fcyl = []
        Rcyl = []
        if geoObj.Orientation == "Forward":
            for plane in geoObj.Surf.Planes:
                pid = plane.bVar
                region = BoolSurface.mult(region, BoolSurface(0, pid))
                cell.surfaces[abs(pid)] = get_surface(abs(pid), plane)

            for rc in geoObj.Surf.Corners:
                cylinder = rc.Surf.Cylinder
                plane = rc.Surf.Plane
                if plane is not None:
                    pid = plane.bVar
                    cell.surfaces[abs(pid)] = get_surface(abs(pid), plane)
                    if rc.Orientation == "Forward":
                        region = region * (-BoolSurface(0, pid))
                    else:
                        region = region * BoolSurface(0, pid)
 
                cid = cylinder.bVar
                cell.surfaces[abs(cid)] = get_surface(abs(cid), cylinder)
                if rc.Orientation == "Forward":
                    Fcyl.append(-BoolSurface(0, cid))
                else:
                    Rcyl.append(BoolSurface(0, cid))
            for cyl in Rcyl:
                region = region * cyl
            for cyl in Fcyl:
                region = region + cyl
        else:
            for plane in geoObj.Surf.Planes:
                pid = plane.bVar
                region = BoolSurface.add(region, BoolSurface(0, pid))
                cell.surfaces[abs(pid)] = get_surface(abs(pid), plane)

            for rc in geoObj.Surf.Corners:
                cylinder = rc.Surf.Cylinder
                plane = rc.Surf.Plane
                if plane is not None:
                    pid = plane.bVar
                    cell.surfaces[abs(pid)] = get_surface(abs(pid), plane)
                    if rc.Orientation == "Forward":
                        region = region - BoolSurface(0, pid)
                    else:
                        region = region + BoolSurface(0, pid)

                cid = cylinder.bVar
                cell.surfaces[abs(cid)] = get_surface(abs(cid), cylinder)
                if rc.Orientation == "Forward":
                    Fcyl.append(-BoolSurface(0, cid))
                else:
                    Rcyl.append(BoolSurface(0, cid))

            for cyl in Fcyl:
                region = region + cyl
            for cyl in Rcyl:
                region = region * cyl

    cell.definition = region.to_integer()

    return cell


def get_surface(id, surf):

    if surf.Type == "Plane":
        params = (surf.Surf.Axis, surf.Surf.Axis.dot(surf.Surf.Position))
        return Plane(id, id, params)
    elif surf.Type == "CylinderOnly":
        params = (surf.Surf.Center, surf.Surf.Axis, surf.Surf.Radius)
        return Cylinder(id, id, params)


def getPart(slist):
    sol = []
    for s in slist:
        if type(s) is list:
            sol.extend(getPart(s))
        else:
            sol.append(s)
    return sol


def BuildDepth(cell, base):
    cell.definition.group_single()
    if cell.definition.level == 0:
        # if base is None build solid from cell boundBox
        # else base is build solid split by cell surfaces
        base, cut = BuildSolidParts(cell, base)
        return base

    if type(base) is not list:
        base = [base]
    newBase = []

    for CS in base:
        if type(cell.definition.elements) is not bool:
            if cell.definition.level == 0:
                tmp = BoolSequence(operator=cell.definition.operator)
                tmp.append(cell.definition)
                cell.definition = tmp

            if cell.definition.operator == "AND":
                part = CS
                for e in cell.definition.elements:
                    subcell = cell.getSubCell(e)
                    keep = []
                    if part is not None:
                        # subcell.build_BoundBox(cell.externalBox, enlarge=10)
                        if subcell.boundBox.Box is None:
                            if subcell.boundBox.Orientation == "Reversed":
                                continue
                            else:
                                part = []
                                break

                        part, keep = filterparts(part, subcell)
                        if len(part) == 0:
                            if len(keep) == 0:
                                break
                            else:
                                part = keep
                                continue
                    part = BuildDepth(subcell, part)
                    part.extend(keep)
                newBase.extend(part)
            else:
                cellParts = []
                for e in cell.definition.elements:
                    subcell = cell.getSubCell(e)
                    if CS is not None:
                        # subcell.build_BoundBox(cell.externalBox, enlarge=10)
                        if subcell.boundBox.Box is None:
                            if subcell.boundBox.Orientation == "Reversed":
                                if type(CS) is SplitBase:
                                    cellParts.append(CS)
                                else:
                                    cellParts.extend(CS)
                            continue
                        part, keep = filterparts(CS, subcell)
                        cellParts.extend(keep)
                        if len(part) == 0:
                            continue
                    else:
                        part = CS
                    part = BuildDepth(subcell, part)
                    cellParts.extend(part)

                # newBase.extend(cellParts)
                JB = joinBase(cellParts)
                if JB.base is not None:
                    newBase.append(JB)

        elif cell.definition.elements:
            newBase.append(CS)

    return newBase


def BuildSolidParts(cell, base):

    # part if several base in input
    if isinstance(base, (list, tuple)):
        fullPart = []
        cutPart = []

        for b in base:
            fullList, cutList = BuildSolidParts(cell, b)
            fullPart.extend(fullList)
            cutPart.extend(cutList)

        # if len(fullPart) > 1:
        #     fullPart = [joinBase(fullPart)]
        # if len(cutPart) > 1:
        #     cutPart = [joinBase(cutPart)]

        return fullPart, cutPart

    if base:
        boundBox = base.base.BoundBox
        if boundBox.XLength < 1e-6 or boundBox.YLength < 1e-6 or boundBox.ZLength < 1e-6:
            return [], []
    else:
        boundBox = cell.boundBox

    surfaces = tuple(cell.surfaces.values())

    if base is None:
        cellBox = cell.makeBox()
        if cellBox is None:
            return [], []
        base = SplitBase(cellBox, orientation="Forward")

    planes = []
    others = []
    for s in surfaces:
        if s.type == "plane":
            planes.append(s)
        else:
            others.append(s)

    cut = base
    full = []
    for p in planes:
        newf, cut = SplitSolid(cut, (p,), cell, tolerance=1e-12)
        full.extend(newf)
        if len(cut) == 0:
            break

    for surf in others:
        newf, cut = SplitSolid(cut, (surf,), cell, tolerance=1e-12)
        full.extend(newf)
        if len(cut) == 0:
            break

    if type(cut) is SplitBase:
        cut = [cut]

    # if len(full) > 1:
    #    full = [joinBase(full)]
    # if len(cut) > 1:
    #    cut = [joinBase(cut)]

    return full, cut


def filterparts(parts, cell):
    process_part = []
    keep_part = []
    cellBox = cell.boundBox
    built = False
    if type(parts) is SplitBase:
        parts = (parts,)
    for p in parts:
        if p is None:
            process_part.append(p)
            continue
        cBox = myBox(cellBox.Box, "Forward")
        pbb = p.base.BoundBox

        pBox = myBox(pbb, "Forward")
        cBox.mult(pBox)
        if cBox.Box is None:
            if p.orientation == "Forward":
                if cellBox.Orientation == "Reversed":
                    keep_part.append(p)
            else:
                if cellBox.Orientation == "Reversed":
                    # process_part.append(p)
                    keep_part.append(p)
                    if not built:
                        built = True
                        cellpart = BuildDepth(cell, None)
                        keep_part.extend(cellpart)
        else:
            process_part.append(p)
    return process_part, keep_part


def FuseSolid(parts):
    if (len(parts)) <= 1:
        if parts:
            solid = parts[0]
        else:
            return None
    else:
        try:
            fused = parts[0].fuse(parts[1:])
        except:
            fused = None

        if fused is not None:
            try:
                refinedfused = fused.removeSplitter()
            except:
                refinedfused = fused

            if refinedfused.isValid():
                solid = refinedfused
            else:
                if fused.isValid():
                    solid = fused
                else:
                    solid = Part.makeCompound(parts)
        else:
            solid = Part.makeCompound(parts)

    if solid.Volume < 0:
        solid.reverse()
    return solid
