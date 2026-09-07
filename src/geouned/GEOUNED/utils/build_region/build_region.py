from .splitFunction import SplitBase, SplitSolid, joinBase
from .Objects import CellObj, CellSurface, myBox
from ..boolean_function import BoolSequence
from ..basic_functions_part1 import round_corner_region, multi_round_corner_region, can_region, tcone_region
from ....geo import GCone, GCylinder, GPlane, GSphere


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

        for plane in geoObj.Surf.Planes:
            pid = plane.bVar
            cell.surfaces[abs(pid)] = get_surface(abs(pid), plane)

        for rc in geoObj.Surf.Corners:
            cylinder = rc.Surf.Cylinder.Surf.Cylinder
            cid = cylinder.bVar
            cell.surfaces[abs(cid)] = get_surface(abs(cid), cylinder)

            plane = rc.Surf.Cylinder.Surf.Plane
            if plane is not None:
                pid = plane.bVar
                cell.surfaces[abs(pid)] = get_surface(abs(pid), plane)

        region = multi_round_corner_region(geoObj)

    elif geoObj.Type == "Can":
        cyl = geoObj.Surf.Cylinder.Surf.Cylinder
        s12_surf = ((geoObj.Surf.s1, geoObj.Surf.s1_configuration), (geoObj.Surf.s2, geoObj.Surf.s2_configuration))

        cid = cyl.bVar
        cell.surfaces[abs(cid)] = get_surface(abs(cid), cyl)

        # Boolean AND/OR rule itself lives in can_region() (shared with
        # MetaSurfacesDict.Can_region, which builds the same region from
        # globally-deduplicated ids -- see basic_functions_part1.py). Here
        # the ids are whatever is currently on `.bVar` (decomposition-time,
        # local numbering); only the surface-number -> surface bookkeeping
        # (cell.surfaces) is this function's own responsibility.
        surf_list = []
        for si, ri in s12_surf:
            if si.Type == "Plane":
                pid = si.bVar
                cell.surfaces[abs(pid)] = get_surface(abs(pid), si)
                surf_list.append(("Plane", None, pid, None, None, ri))
                continue

            if si.Type == "Cylinder":
                surf = si.Surf.Cylinder
            elif si.Type == "Cone":
                surf = si.Surf.Cone
            elif si.Type == "Sphere":
                surf = si.Surf.Sphere

            sid = surf.bVar
            cell.surfaces[abs(sid)] = get_surface(abs(sid), surf)

            plane = si.Surf.Plane
            pid = plane.bVar if plane is not None else None
            if plane is not None:
                cell.surfaces[abs(pid)] = get_surface(abs(pid), plane)

            apexplane = si.Surf.ApexPlane if si.Type == "Cone" else None
            apid = apexplane.bVar if apexplane is not None else None
            if apexplane is not None:
                cell.surfaces[abs(apid)] = get_surface(abs(apid), apexplane)

            surf_list.append((si.Type, sid, pid, apid, si.Orientation, ri))

        region = can_region(cid, geoObj.Surf.Cylinder.Orientation, surf_list)

    elif geoObj.Type == "TCone":
        kne = geoObj.Surf.Cone.Surf.Cone
        s12_surf = ((geoObj.Surf.p1, geoObj.Surf.p1_configuration), (geoObj.Surf.p2, geoObj.Surf.p2_configuration))

        cid = kne.bVar
        cell.surfaces[abs(cid)] = get_surface(abs(cid), kne)

        surf_list = []
        for pi, ri in s12_surf:
            pid = pi.bVar
            cell.surfaces[abs(pid)] = get_surface(abs(pid), pi)
            surf_list.append((pid, ri))

        region = tcone_region(cid, geoObj.Surf.Cone.Orientation, surf_list)

    cell.definition = region.to_integer()

    return cell


def get_surface(id, surf):
    # surf.Surf.{Axis,Position,Center,Apex,Radius,SemiAngle} are already
    # GVector (basic_functions_part1.py's *OnlyParams/PlaneParams) --
    # build the geo descriptor directly, no native detour.
    if surf.Type == "Plane":
        descriptor = GPlane.from_values(surf.Surf.Position, surf.Surf.Axis)
    elif surf.Type == "CylinderOnly":
        descriptor = GCylinder.from_values(surf.Surf.Center, surf.Surf.Axis, surf.Surf.Radius)
    elif surf.Type == "ConeOnly":
        descriptor = GCone.from_values(surf.Surf.Apex, surf.Surf.Axis, surf.Surf.SemiAngle)
    elif surf.Type == "SphereOnly":
        descriptor = GSphere.from_values(surf.Surf.Center, surf.Surf.Radius)
    else:
        return None
    return CellSurface(id, id, descriptor)


def getPart(slist):
    sol = []
    for s in slist:
        if type(s) is list:
            sol.extend(getPart(s))
        else:
            sol.append(s)
    return sol


def BuildDepth(cell, base, tolerances):
    cell.definition.group_single()
    if cell.definition.level == 0:
        # if base is None build solid from cell boundBox
        # else base is build solid split by cell surfaces
        base, cut = BuildSolidParts(cell, base, tolerances)
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

                        part, keep = filterparts(part, subcell, tolerances)
                        if len(part) == 0:
                            if len(keep) == 0:
                                break
                            else:
                                part = keep
                                continue
                    part = BuildDepth(subcell, part, tolerances)
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
                        part, keep = filterparts(CS, subcell, tolerances)
                        cellParts.extend(keep)
                        if len(part) == 0:
                            continue
                    else:
                        part = CS
                    part = BuildDepth(subcell, part, tolerances)
                    cellParts.extend(part)

                # newBase.extend(cellParts)
                JB = joinBase(cellParts, tolerances)
                if JB.base is not None:
                    newBase.append(JB)

        elif cell.definition.elements:
            newBase.append(CS)

    return newBase


def BuildSolidParts(cell, base, tolerances):

    # part if several base in input
    if isinstance(base, (list, tuple)):
        fullPart = []
        cutPart = []

        for b in base:
            fullList, cutList = BuildSolidParts(cell, b, tolerances)
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
    split_tolerance = tolerances.split_tolerance
    for p in planes:
        newf, cut = SplitSolid(cut, (p,), cell, split_tolerance, tolerances)
        full.extend(newf)
        if len(cut) == 0:
            break

    for surf in others:
        newf, cut = SplitSolid(cut, (surf,), cell, split_tolerance, tolerances)
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


def filterparts(parts, cell, tolerances):
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
                        cellpart = BuildDepth(cell, None, tolerances)
                        keep_part.extend(cellpart)
        else:
            process_part.append(p)
    return process_part, keep_part
