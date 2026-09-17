from .Objects import CellObj, CellSurface
from ..basic_functions_part1 import round_corner_region, multi_round_corner_region, can_region, tcone_region
from ....geo import GCone, GCylinder, GPlane, GSphere

# The split-cascade functions that used to live here (BuildDepth,
# BuildSolidParts, filterparts, getPart, plus SplitBase/joinBase/SplitSolid
# from the sibling splitFunction.py, deleted) moved to `geo.solid_ops`,
# 2026-09-17/18: shared with GEOReverse's own, previously near-identical
# copy in `CAD/buildSolidCell.py`/`CAD/splitFunction.py` -- see CLAUDE.md's
# "build_region/ vs CAD/buildSolidCell.py+splitFunction.py unification"
# entry. `build_shape_functions.py::build_complex_shape` (the only caller
# of that cascade for this pipeline) imports them from `geo` directly.
# Only `get_cell_object`/`get_surface` are genuinely GEOUNED-specific
# (translating a composite meta-surface's own GeounedSurface tree into the
# small CellObj/CellSurface the shared cascade operates on) and stay here.


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
