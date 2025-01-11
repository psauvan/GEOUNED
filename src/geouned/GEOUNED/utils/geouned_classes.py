#
# Set of useful functions used in different parts of the code
#
import logging
import math

import FreeCAD
import Part

logger = logging.getLogger("general_logger")

from .basic_functions_part1 import (
    Plane3PtsParams,
    PlaneParams,
    ConeOnlyParams,
    CylinderOnlyParams,
    SphereOnlyParams,
    TorusOnlyParams,
    ConeParams,
    CylinderParams,
    SphereParams,
    TorusParams,

    MultiPlanesParams,
    ReverseCanParams,
    RoundCornerParams,
)
from .basic_functions_part2 import is_same_plane, is_same_cylinder, is_same_cone, is_same_sphere, is_same_torus

from .data_classes import NumericFormat, Options, Tolerances
from .boolean_function import BoolRegion
from .build_shape_functions import makePlane, makeCylinder, makeCone, makeMultiPlanes, makeReverseCan, makeRoundCorner
from .basic_functions_part1 import is_parallel, is_opposite

from .data_classes import NumericFormat, Options, Tolerances


class GeounedSolid:
    def __init__(self, id, comsolid=None):
        refine = True
        if not comsolid:
            self.Solids = None
            self.Volume = None
            self.BoundBox = None
        elif type(comsolid) is list:
            self.Solids = comsolid
            vol = 0
            self.BoundBox = FreeCAD.BoundBox()
            for s in comsolid:
                vol += s.Volume
                self.BoundBox.add(s.BoundBox)
            self.Volume = vol
        else:
            if refine:
                try:
                    self.Solids = comsolid.removeSplitter().Solids
                except:
                    self.Solids = comsolid.Solids

                for s in self.Solids:
                    if s.Volume < 0:
                        s.reverse()
            else:
                self.Solids = comsolid.Solids
            self.Volume = comsolid.Volume
            self.BoundBox = comsolid.BoundBox

        self.__id__ = id
        self.label = None
        self.Definition = []
        self.Faces = []
        self.Edges = []
        self.Comments = ""
        self.Density = 0
        self.Dilution = 1
        self.Material = 0
        self.Surfaces = []
        self.Rho = None
        self.MatInfo = None
        self.CellType = "solid"  # other types : 'void', 'enclosure', 'fill'
        self.Universe = 0
        self.Void = False
        self.IsEnclosure = False
        self.EnclosureID = None
        self.ParentEnclosureID = None
        self.SonEnclosures = []
        self.enclosure_list = None
        self.CADSolid = None
        self.UniverseBox = None
        self.NullCell = True

    def update_solids(self, solidList):
        self.Solids = solidList
        vol = 0
        self.BoundBox = FreeCAD.BoundBox()
        for s in solidList:
            vol += s.Volume
            self.BoundBox.add(s.BoundBox)

    def set_cad_solid(self):
        self.CADSolid = Part.makeCompound(self.Solids)
        self.Volume = self.CADSolid.Volume
        self.BoundBox = self.CADSolid.BoundBox

    def optimalBoundingBox(self):
        if self.CADSolid is None:
            self.set_cad_solid()
        return self.CADSolid.optimalBoundingBox()

    def set_definition(self, definition, simplify=False):

        if definition is None:
            self.NullCell = True
            return

        self.NullCell = False
        self.Definition = definition

        if not self.Void:
            if not self.Definition.elements:
                self.NullCell = True
                return

        self.Surfaces = tuple(self.Definition.get_surfaces_numbers())

    def set_faces(self, faces):
        self.Faces = faces

    def set_comments(self, comments):
        self.Comments = comments

    def set_material(self, material, rho=None, info=None):

        self.Material = material
        self.Rho = rho
        self.MatInfo = info

        if rho is not None:
            self.Density = self.Dilution * rho
        else:
            if material != 0:
                self.Density = None

    def set_dilution(self, dilution):
        self.Dilution = dilution
        if self.Rho is not None:
            self.Density = self.Rho * dilution

    def check_intersection(self, solid, dtolerance=1.0e-6, vtolerance=1e-10):
        """Check if solid intersect with current solid.
        return : -2 solid fully embedded in self.CADSolid ;
                 -1 self.CADSolid fully embedded in solid ;
                  0 self.CADSolid intersect solid ;
                  1 self.CADSolid and solid fully disjoint"""

        dist = 1e12
        for sol1 in self.CADSolid.Solids:
            for sol2 in solid.Solids:
                try:
                    distShape = sol1.distToShape(sol2)[0]
                except:
                    logger.info("Failed solid1.distToshape(solid2), try with inverted solids")
                    distShape = sol2.distToShape(sol1)[0]
                    logger.info(f"inverted disToShape OK {distShape}")
                dist = min(dist, distShape)
                if dist == 0:
                    break

        if dist > dtolerance:
            return 1

        common = self.CADSolid.common(solid)
        if abs(common.Volume) < vtolerance:
            return 1
        if abs(self.CADSolid.Volume - common.Volume) / common.Volume < vtolerance:
            return -1
        elif abs(solid.Volume - common.Volume) / common.Volume < vtolerance:
            return -2
        else:
            return 0


class GeounedSurface:

    def __init__(self, params, Face=None):

        self.Index = 0
        self.region = None
        if params[0] == "Plane":
            self.Type = "Plane"
            self.Surf = PlaneParams(params[1])  # plane point defined as the shortest distance to origin
        elif params[0] == "Plane3Pts":
            self.Type = "Plane"
            self.Surf = Plane3PtsParams(params[1])  # plane point defined with 3 points
        elif params[0] == "CylinderOnly":
            self.Type = params[0]
            self.Surf = CylinderOnlyParams(params[1])
        elif params[0] == "ConeOnly":
            self.Type = params[0]
            self.Surf = ConeOnlyParams(params[1])
        elif params[0] == "SphereOnly":
            self.Type = params[0]
            self.Surf = SphereOnlyParams(params[1])
        elif params[0] == "TorusOnly":
            self.Type = params[0]
            self.Surf = TorusOnlyParams(params[1])    
        elif params[0] == "Cylinder":
            self.Type = params[0]
            self.Surf = CylinderParams(params[1])
        elif params[0] == "Cone":
            self.Type = params[0]
            self.Surf = ConeParams(params[1])
        elif params[0] == "Sphere":
            self.Type = params[0]
            self.Surf = SphereParams(params[1])
        elif params[0] == "Torus":
            self.Type = params[0]
            self.Surf = TorusParams(params[1])
        elif params[0] == "MultiPlane":
            self.Type = params[0]
            self.Surf = MultiPlanesParams(params[1])
        elif params[0] == "ReverseCan":
            self.Type = params[0]
            self.Surf = ReverseCanParams(params[1])
        elif params[0] == "RoundCorner":
            self.Type = params[0]
            self.Surf = RoundCornerParams(params[1])

        self.shape = Face

        return

    def build_surface(self, boundBox):

        Box = FreeCAD.BoundBox(boundBox)
        if self.Type == "Plane":
            Box.enlarge(10)
            self.shape = makePlane(self.Surf.Axis, self.Surf.Position, Box)

        elif self.Type == "CylinderOnly":
            self.shape = makeCylinder(self.Surf.Axis, self.Surf.Center, self.Surf.Radius, Box)

        elif self.Type == "ConeOnly":
            tan = math.tan(self.Surf.SemiAngle)
            self.shape = makeCone(self.Surf.Axis, self.Surf.Apex, tan, Box)

        elif self.Type == "SphereOnly":
            rad = self.Surf.Radius
            pnt = self.Surf.Center
            self.shape = Part.makeSphere(rad, pnt).Faces[0]
            return

        elif self.Type == "TorusOnly":
            axis = self.Surf.Axis
            center = self.Surf.Center
            majorR = self.Surf.MajorRadius
            minorR = self.Surf.MinorRadius

            torus = Part.makeTorus(majorR, minorR, center, axis)
            self.shape = torus.Faces[0]
            return

        elif self.Type == "MultiPlane":
            Box.enlarge(10)
            planes = self.Surf.Planes
            vertexes = self.Surf.Vertexes
            multiplane = makeMultiPlanes(planes, vertexes, Box)
            self.shape = multiplane

        elif self.Type == "ReverseCan":
            Box.enlarge(10)
            self.shape = makeReverseCan(self.Surf.Cylinder, self.Surf.Planes, Box)

        elif self.Type == "RoundCorner":
            Box.enlarge(10)
            self.shape = makeRoundCorner(self.Surf.Cylinder, self.Surf.AddPlane, self.Surf.Planes, self.Surf.Configuration, Box)

        else:
            logger.error(f"Cannot build {self.Type} shape")
            return


class MetaSurfIndex:
    def __init__(self, index, surfaces):
        self.index = index
        self.surfaces = surfaces
        self.single_surface = isinstance(surfaces, int)


class MetaSurfacesDict(dict):


    def __init__(
        self,
        surfaces=None,
        offset: int = 0,
        options: Options = Options(),
        tolerances: Tolerances = Tolerances(),
        numeric_format: NumericFormat = NumericFormat(),
    ):

        self.IndexOffset = offset
        self.options = options
        self.tolerances = tolerances
        self.numeric_format = numeric_format

        self.__nickname__ = {
                "Plane":"Planes",
                "Cylinder":"Cyl",
                "Cone":"Cone",
                "Sphere":"Sph",
                "Torus":"Tor",
                "MultiPlane":"MultiP",
                "ReverseCan":"revCan",
                "RoundCorner":"RoundC"}

        surfname = ["Planes", "Cyl", "Cone", "Sph", "Tor", "MultiP", "RevCan", "RoundC"]
        for name in surfname:
            self[name] = []

        self.__surfIndex__ = dict()

        if surfaces is not None:
            for key in surfaces.keys():
                self[key] = surfaces[key][:]
                self.__surfIndex__[key] = surfaces.__surfIndex__[key][:]
            self.surfaceNumber = surfaces.surfaceNumber
            self.options = surfaces.options
            self.tolerances = surfaces.tolerances
            self.numeric_format = surfaces.numeric_format
            self.__last_obj__ = (surfaces.__last_obj__[0], surfaces.__last_obj__[1])
            self.primitive_surfaces = SurfacesDict(surfaces.primitive_surfaces)
        else:
            self.primitive_surfaces = SurfacesDict(
                options=self.options, tolerances=self.tolerances, numeric_format=self.numeric_format
            )
            self.surfaceNumber = 0
            self.__last_obj__ = ("", -1)
            for key in surfname:
                self.__surfIndex__[key] = []
        return

    def change_sign(self,psurf):
        
        for ind in psurf.surfIndex:
            surf = self.get_surface(ind)      
            surf.region.chg_surf_sign(psurf.Index)

    def get_surface(self, pindex):

        for key,values in self.__surfIndex__.items():
            if pindex not in values:
                continue
            sindex = values.index(pindex)
            return self[key][sindex]
        return None

    def get_primitive_surface(self, index):
        return self.primitive_surfaces.get_surface(index)

    def del_surface(self, index):
        self.primitive_surfaces.del_surface(index)

    def extend(self, surface):
        self.primitive_surfaces.extend(surface)

    def chg_surf_sign(self,surf):
        for ind in surf.surfIndex:
            region = self.get_primitive_surface(ind)
            region.chg_surf_sign(surf)     

    def add_plane(self, plane, fuzzy):
        pid, exist = self.primitive_surfaces.add_plane(plane, fuzzy)
        same_dir = True
        if exist:
            p_in = self.get_primitive_surface(pid)
            same_dir = not is_opposite(plane.Surf.Axis, p_in.Surf.Axis)

            found = False
            for i, p in enumerate(self["Planes"]):
                if pid in p.region.surfaces:
                    self.__last_obj__ = ("Planes", i)
                    found = True
                    break
            if found:
                p_region = p.region
                return -p_region if (p_region.reverse == same_dir) else p_region
                # Equivalent to :
                # if p.reverse and same_dir :
                #     return -p
                # if p.reverse and not same_dir:
                #    return p
                # if not p.reverse and same_dir :
                #    return p
                # if not p.reverse and not same_dir :
                #    return -p
        else:
            plane.surfIndex = []
            p_in = plane
            

        self.surfaceNumber += 1
        p_in.surfIndex.append(self.surfaceNumber)

        if not same_dir:
            pid = -pid
        plane.region = BoolRegion(self.surfaceNumber, str(pid), reverse=not same_dir)
        self["Planes"].append(plane)
        self.__surfIndex__["Planes"].append(plane.region.__int__())
        return plane.region

    def add_cylinder(self, cylinder, fuzzy=False):
        used_surfaces = []
        cid, exist_c = self.primitive_surfaces.add_cylinder(cylinder.Surf.Cylinder)
        if exist_c:
            used_surfaces.append(self.get_primitive_surface(cid))
        else:   
            cylinder.Surf.Cylinder.surfIndex = []
            used_surfaces.append(cylinder.Surf.Cylinder)

        if cylinder.Surf.Orientation == "Forward":
            cid = -cid
            reverse = False
        else:
            reverse = True
        cyl_surfaces = str(cid)
    
        if cylinder.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(cylinder.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(cylinder.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                cylinder.Surf.Plane.surfIndex = []
                used_surfaces.append(cylinder.Surf.Plane)

            cyl_surfaces += f' {pid}'
        cylinder_region = BoolRegion(0, cyl_surfaces, reverse = reverse)

        add_cyl = True
        for i, cyl in enumerate(self["Cyl"]):
            if cyl.region == cylinder_region:
                add_cyl = False
                self.__last_obj__ = ("Cyl", i)
                break

        if add_cyl:
            self.surfaceNumber += 1
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            cylinder.region = cylinder_region.copy(self.surfaceNumber)
            self["Cyl"].append(cylinder)
            self.__surfIndex__["Cyl"].append(cylinder.region.__int__())
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            return cylinder.region
        else:
            return cyl.region

    def add_cone(self, cone):
        used_surfaces = []
        cid, exist_c = self.primitive_surfaces.add_cone(cone.Surf.Cone)
        if exist_c:
            used_surfaces.append(self.get_primitive_surface(cid))
        else:   
            cone.Surf.Cone.surfIndex = []
            used_surfaces.append(cone.Surf.Cone)

        if cone.Surf.Orientation == "Forward":
            cid = -cid
            reverse = False
        else:
            reverse = True
        cone_surfaces = str(cid)
    
        if cone.Surf.ApexPlane:
            pid, exist_p = self.primitive_surfaces.add_plane(cone.Surf.ApexPlane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(cone.Surf.ApexPlane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                cone.Surf.ApexPlane.surfIndex = []
                used_surfaces.append(cone.Surf.ApexPlane) 

            if cone.Surf.Orientation == "Forward" :
                cone_surfaces += f' {pid}'
            else:    
                cone_surfaces += f':{pid}'

        if cone.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(cone.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(cone.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                cone.Surf.Plane.surfIndex = []
                used_surfaces.append(cone.Surf.Plane) 
            cone_surfaces = f'{pid} ({cone_surfaces})'        
        
        cone_region = BoolRegion(0, cone_surfaces, reverse = reverse)

        add_cone = True
        for i, kne in enumerate(self["Cone"]):
            if kne.region == cone_region:
                add_cone = False
                self.__last_obj__ = ("Cone", i)
                break

        if add_cone:
            self.surfaceNumber += 1
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            cone.region = cone_region.copy(self.surfaceNumber)
            self.__last_obj__ = ("Cone", len(self["Cone"]))
            self["Cone"].append(cone)
            self.__surfIndex__["Cone"].append(cone.region.__int__())
            return cone.region
        else:
            return kne.region
        
    def add_sphere(self, sphere):
        used_surfaces = []
        sid, exist_s = self.primitive_surfaces.add_sphere(sphere.Surf.Sphere)
        if exist_s:
            used_surfaces.append(self.get_primitive_surface(sid))
        else:   
            sphere.Surf.Sphere.surfIndex = []
            used_surfaces.append(sphere.Surf.Sphere)

        if sphere.Surf.Orientation == "Forward":
            sid = -sid
            reverse = False
        else:
            reverse = True

        sph_surfaces = str(sid)    
        if sphere.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(sphere.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(sphere.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                sphere.Surf.Plane.surfIndex = []
                used_surfaces.append(sphere.Surf.Plane)

            sph_surfaces += f' {pid}'
        sphere_region = BoolRegion(0, sph_surfaces, reverse = reverse)

        add_sph = True
        for i, sph in enumerate(self["Sph"]):
            if sph.region == sphere_region:
                add_sph = False
                self.__last_obj__ = ("Sph", i)
                break

        if add_sph:
            self.surfaceNumber += 1
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            sphere.region = sphere_region.copy(self.surfaceNumber)
            self.__last_obj__ = ("Sph", len(self["Sph"]))
            self["Sph"].append(sphere)
            self.__surfIndex__["Sph"].append(sphere.region.__int__())
            return sphere.region
        else:
            return sph.region
    
    def add_torus(self, torus):
        used_surfaces = []
        tid, exist_t = self.primitive_surfaces.add_torus(torus.Surf.Torus)
        if exist_t:
            used_surfaces.append(self.get_primitive_surface(tid))
        else:   
            torus.Surf.Torus.surfIndex = []
            used_surfaces.append(torus.Surf.Torus)

        if torus.Surf.Orientation == "Forward":
            tid = -tid
            reverse = False
        else:
            reverse = True
        torus_surfaces = str(tid)
        
        psurf = []
        for tp in torus.Surf.UPlanes:
            pid, exist_p = self.primitive_surfaces.add_plane(tp, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(tp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                tp.surfIndex = []
                used_surfaces.append(tp)

            psurf.append(str(pid))

        if len(psurf) == 2 :
            torus_surfaces += f' ({":".join(psurf)})'
        elif len(psurf) == 1 :    
            torus_surfaces += f' {psurf[0]}'

        if torus.Surf.VSurface:
            if torus.Surf.VSurface.Type == "Plane":
                sid, exist_s = self.primitive_surfaces.add_plane(torus.Surf.VSurface, True)
            elif torus.Surf.VSurface.Type == "Cylinder":
                sid, exist_s = self.primitive_surfaces.add_cylinder(torus.Surf.VSurface, True)
            else:
                sid, exist_s = self.primitive_surfaces.add_cone(torus.Surf.VSurface)

            if exist_s:
                surf = self.get_primitive_surface(sid)
                used_surfaces.append(surf)
                if torus.Surf.VSurface.Type == "Plane":
                    if is_opposite(torus.Surf.VSurface.Surf.Axis, surf.Surf.Axis, self.tolerances.pln_angle):
                        sid = -sid
                else:
                    if torus.Surf.SOrientation == "Forward":
                        sid = -sid        
            else:        
                torus.Surf.VSurface.surfIndex =  []
                used_surfaces.append(torus.Surf.Vsurface)
            
            torus_surfaces += f' {sid}'
                
        torus_region = BoolRegion(0, torus_surfaces, reverse = reverse)

        add_torus = True
        for i, tor in enumerate(self["Tor"]):
            if tor.region == torus_region:
                add_torus = False
                self.__last_obj__ = ("Tor", i)
                break

        if add_torus:
            self.surfaceNumber += 1
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            torus.region = torus_region.copy(self.surfaceNumber)
            self.__last_obj__ = ("Tor", len(self["Tor"]))
            self["Tor"].append(torus)
            self.__surfIndex__["Tor"].append(torus.region.__int__())
            return torus.region
        else:
            return tor.region

    def add_multiPlane(self, multiP):
        multiP_surfaces = []
        used_surfaces = []
        for mp in multiP.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(mp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(mp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                mp.surfIndex = []
                used_surfaces.append(mp)        
            multiP_surfaces.append(str(pid))
        multiP_region = BoolRegion(0, ":".join(multiP_surfaces))

        add_multiP = True
        for i, mp in enumerate(self["MultiP"]):
            if mp.region == multiP_region:
                add_multiP = False
                self.__last_obj__ = ("MultiP", i)
                break

        if add_multiP:
            self.surfaceNumber += 1
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            multiP.region = multiP_region.copy(self.surfaceNumber)
            self.__last_obj__ = ("MultiP", len(self["MultiP"]))
            self["MultiP"].append(multiP)
            self.__surfIndex__["MultiP"].append(multiP.region.__int__())
            return multiP.region
        else:
            return mp.region

    def add_reverseCan(self, reverseCan):
        used_surfaces = []
        pid, exist = self.primitive_surfaces.add_cylinder(reverseCan.Surf.Cylinder, True)
        if exist:
            used_surfaces.append(self.get_primitive_surface(pid))
        else:   
            reverseCan.Surf.Cylinder.surfIndex = []
            used_surfaces.append(reverseCan.Surf.Cylinder)

        reverseCan_surfaces = [str(pid)]
        for cp in reverseCan.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(cp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(cp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                cp.surfIndex = []
                used_surfaces.append(cp)

            reverseCan_surfaces.append(str(pid))
        reverseCan_region = BoolRegion(0, ":".join(reverseCan_surfaces))

        add_can = True
        for i, cs in enumerate(self["RevCan"]):
            if cs.region == reverseCan_region:
                add_can = False
                self.__last_obj__ = ("RevCan", i)
                break

        if add_can:
            self.surfaceNumber += 1
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            reverseCan.region = reverseCan_region.copy(self.surfaceNumber)
            self.__last_obj__ = ("RevCan", len(self["RevCan"]))
            self["RevCan"].append(reverseCan)
            self.__surfIndex__["RevCan"].append(reverseCan.region.__int__())
            return reverseCan.region
        else:
            return cs.region

    def add_roundCorner(self, roundC):
        used_surfaces = []
        cid, exist_c = self.primitive_surfaces.add_cylinder(roundC.Surf.Cylinder, True)
        if exist_c:
            used_surfaces.append(self.get_primitive_surface(cid))
        else:   
            roundC.Surf.Cylinder.surfIndex = []
            used_surfaces.append(roundC.Surf.Cylinder)

        pid, exist_p = self.primitive_surfaces.add_plane(roundC.Surf.AddPlane, True)
        if exist_p:
            p = self.get_primitive_surface(pid)
            used_surfaces.append(p)
            if is_opposite(roundC.Surf.AddPlane.Surf.Axis, roundC.Surf.AddPlane.Surf.Axis, self.tolerances.pln_angle):
                pid = -pid
        else:
            roundC.Surf.AddPlane.surfIndex = []
            used_surfaces.append(roundC.Surf.AddPlane)

        roundC_surfaces = [f"({-cid}:{pid})"]

        for cp in roundC.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(cp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                used_surfaces.append(p)
                if is_opposite(cp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            else:
                cp.surfIndex = []
                used_surfaces.append(cp)        
            roundC_surfaces.append(str(pid))

        if roundC.Surf.Configuration == "AND":
            op = " "
        else:
            op = ":"
        roundC_region = BoolRegion(0, op.join(roundC_surfaces))

        add_corner = True
        for i, rc in enumerate(self["RoundC"]):
            if rc.region == roundC_region:
                add_corner = False
                self.__last_obj__ = ("RoundC", i)
                break

        if add_corner:
            self.surfaceNumber += 1
            for s in used_surfaces:
                s.surfIndex.append(self.surfaceNumber)
            roundC.region = roundC_region.copy(self.surfaceNumber)
            self.__last_obj__ = ("RoundC", len(self["RoundC"]))
            self["RoundC"].append(roundC)
            self.__surfIndex__["RoundC"].append(roundC.region.__int__())
            return roundC.region
        else:
            return rc.region


class SurfacesDict(dict):
    def __init__(
        self,
        surfaces=None,
        offset: int = 0,
        options: Options = Options(),
        tolerances: Tolerances = Tolerances(),
        numeric_format: NumericFormat = NumericFormat(),
    ):

        self.IndexOffset = offset
        self.options = options
        self.tolerances = tolerances
        self.numeric_format = numeric_format

        surfname = ["PX", "PY", "PZ", "P", "Cyl", "Cone", "Sph", "Tor"]
        for name in surfname:
            self[name] = []

        self.__surfIndex__ = dict()

        if surfaces is not None:
            for key in surfaces.keys():
                self[key] = surfaces[key][:]
                self.__surfIndex__[key] = surfaces.__surfIndex__[key][:]
            self.surfaceNumber = surfaces.surfaceNumber
            self.metaSurfaceNumber = surfaces.metaSurfaceNumber
            self.options = surfaces.options
            self.tolerances = surfaces.tolerances
            self.numeric_format = surfaces.numeric_format
            self.__last_obj__ = (surfaces.__last_obj__[0], surfaces.__last_obj__[1])
        else:
            self.surfaceNumber = 0
            self.metaSurfaceNumber = 0
            self.__last_obj__ = ("", -1)
            for key in surfname:
                self.__surfIndex__[key] = []
        return

    def __str__(self):
        for key in self.keys():
            logger.info(f"{key}, {self[key]}")
        return ""

    def get_surface(self, index):

        lastKey = self.__last_obj__[0]
        lastInd = self.__last_obj__[1]
        if lastKey != "":
            if len(self[lastKey]) > 0:
                if self[lastKey][lastInd].Index == index:
                    return self[lastKey][lastInd]

        for key, values in self.__surfIndex__.items():
            if index not in values:
                continue
            i = values.index(index)
            self.__last_obj__ = (key, i)
            return self[key][i]

        logger.info(f"Index {index} not found in Surfaces")
        return None

    def del_surface(self, index):
        self.get_surface(index)
        self.__surfIndex__[self.__last_obj__[0]].remove(index)
        del self[self.__last_obj__[0]][self.__last_obj__[1]]
        return

    def extend(self, surface):
        for Pkey in ["PX", "PY", "PZ", "P"]:
            for s in surface[Pkey]:
                self.add_plane(s, False)
        for s in surface["Cyl"]:
            self.add_cylinder(s, False)
        for s in surface["Cone"]:
            self.add_cone(s)
        for s in surface["Sph"]:
            self.add_sphere(s)
        for s in surface["Tor"]:
            self.add_torus(s)

    def add_plane(self, plane, fuzzy):
        ex = FreeCAD.Vector(1, 0, 0)
        ey = FreeCAD.Vector(0, 1, 0)
        ez = FreeCAD.Vector(0, 0, 1)

        if is_parallel(plane.Surf.Axis, ex, self.tolerances.pln_angle):
            add_plane = True
            for i, p in enumerate(self["PX"]):
                if is_same_plane(
                    plane.Surf,
                    p.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                    fuzzy=(fuzzy, p.Index),
                ):
                    add_plane = False
                    index = p.Index
                    self.__last_obj__ = ("PX", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.Index = self.surfaceNumber + self.IndexOffset
                self.__last_obj__ = ("PX", len(self["PX"]))
                self["PX"].append(plane)
                self.__surfIndex__["PX"].append(plane.Index)

        elif is_parallel(plane.Surf.Axis, ey, self.tolerances.pln_angle):
            add_plane = True
            for i, p in enumerate(self["PY"]):
                if is_same_plane(
                    plane.Surf,
                    p.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                    fuzzy=(fuzzy, p.Index),
                ):
                    add_plane = False
                    index = p.Index
                    self.__last_obj__ = ("PY", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.Index = self.surfaceNumber + self.IndexOffset
                self.__last_obj__ = ("PY", len(self["PY"]))
                self["PY"].append(plane)
                self.__surfIndex__["PY"].append(plane.Index)

        elif is_parallel(plane.Surf.Axis, ez, self.tolerances.pln_angle):
            add_plane = True
            for i, p in enumerate(self["PZ"]):
                if is_same_plane(
                    plane.Surf,
                    p.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                    fuzzy=(fuzzy, p.Index),
                ):
                    add_plane = False
                    index = p.Index
                    self.__last_obj__ = ("PZ", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.Index = self.surfaceNumber + self.IndexOffset
                self.__last_obj__ = ("PZ", len(self["PZ"]))
                self["PZ"].append(plane)
                self.__surfIndex__["PZ"].append(plane.Index)

        else:
            add_plane = True
            for i, p in enumerate(self["P"]):
                if is_same_plane(
                    plane.Surf,
                    p.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                    fuzzy=(fuzzy, p.Index),
                ):
                    add_plane = False
                    index = p.Index
                    self.__last_obj__ = ("P", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.Index = self.surfaceNumber + self.IndexOffset
                self.__last_obj__ = ("P", len(self["P"]))
                self["P"].append(plane)
                self.__surfIndex__["P"].append(plane.Index)

        if add_plane:
            return plane.Index, False
        else:
            return index, True

    def add_cylinder(self, cyl, fuzzy=False):
        addCyl = True
        for i, c in enumerate(self["Cyl"]):
            if is_same_cylinder(
                cyl.Surf,
                c.Surf,
                options=self.options,
                tolerances=self.tolerances,
                numeric_format=self.numeric_format,
                fuzzy=(fuzzy, c.Index),
            ):
                addCyl = False
                index = c.Index
                self.__last_obj__ = ("Cyl", i)
                break

        if addCyl:
            self.surfaceNumber += 1
            cyl.Index = self.surfaceNumber + self.IndexOffset
            self.__last_obj__ = ("Cyl", len(self["Cyl"]))
            self["Cyl"].append(cyl)
            self.__surfIndex__["Cyl"].append(cyl.Index)
            return cyl.Index, False
        else:
            return index, True

    def add_cone(self, cone):
        cone_added = True
        for i, c in enumerate(self["Cone"]):
            if is_same_cone(
                cone.Surf,
                c.Surf,
                dtol=self.tolerances.kne_distance,
                atol=self.tolerances.kne_angle,
                rel_tol=self.tolerances.relativeTol,
            ):
                cone_added = False
                index = c.Index
                self.__last_obj__ = ("Cone", i)
                break
        if cone_added:
            self.surfaceNumber += 1
            cone.Index = self.surfaceNumber + self.IndexOffset
            self.__last_obj__ = ("Cone", len(self["Cone"]))
            self["Cone"].append(cone)
            self.__surfIndex__["Cone"].append(cone.Index)
            return cone.Index, False
        else:
            return index, True

    def add_sphere(self, sph):
        sphere_added = True
        for i, s in enumerate(self["Sph"]):
            if is_same_sphere(
                sph.Surf,
                s.Surf,
                self.tolerances.sph_distance,
                rel_tol=self.tolerances.relativeTol,
            ):
                sphere_added = False
                index = s.Index
                self.__last_obj__ = ("Sph", i)
                break
        if sphere_added:
            self.surfaceNumber += 1
            sph.Index = self.surfaceNumber + self.IndexOffset
            self.__last_obj__ = ("Sph", len(self["Sph"]))
            self["Sph"].append(sph)
            self.__surfIndex__["Sph"].append(sph.Index)
            return sph.Index, False
        else:
            return index, True

    def add_torus(self, tor):
        add_torus = True
        for i, s in enumerate(self["Tor"]):
            if is_same_torus(
                tor.Surf,
                s.Surf,
                dtol=self.tolerances.tor_distance,
                atol=self.tolerances.tor_angle,
                rel_tol=self.tolerances.relativeTol,
            ):
                add_torus = False
                index = s.Index
                self.__last_obj__ = ("Tor", i)
                break
        if add_torus:
            self.surfaceNumber += 1
            tor.Index = self.surfaceNumber + self.IndexOffset
            self.__last_obj__ = ("Tor", len(self["Tor"]))
            self["Tor"].append(tor)
            self.__surfIndex__["Tor"].append(tor.Index)
            return tor.Index, False
        else:
            return index, True

    def get_id(self, facein):

        if facein.Type == "Plane":
            if is_parallel(facein.Surf.Axis, FreeCAD.Vector(1, 0, 0), self.tolerances.pln_angle):
                p = "PX"
            elif is_parallel(facein.Surf.Axis, FreeCAD.Vector(0, 1, 0), self.tolerances.pln_angle):
                p = "PY"
            elif is_parallel(facein.Surf.Axis, FreeCAD.Vector(0, 0, 1), self.tolerances.pln_angle):
                p = "PZ"
            else:
                p = "P"

            for s in self[p]:
                if is_same_plane(
                    facein.Surf,
                    s.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                ):
                    return s.Index

        elif facein.Type == "Cylinder":
            for s in self["Cyl"]:
                if is_same_cylinder(
                    facein.Surf,
                    s.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                ):
                    return s.Index

        elif facein.Type == "Cone":
            for s in self["Cone"]:
                if is_same_cone(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.kne_distance,
                    atol=self.tolerances.kne_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.Index

        elif facein.Type == "Sphere":
            for s in self["Sph"]:
                if is_same_sphere(facein.Surf, s.Surf, self.tolerances.sph_distance, rel_tol=self.tolerances.relativeTol):
                    return s.Index

        elif facein.Type == "Torus":
            for s in self["Tor"]:
                if is_same_torus(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.tor_distance,
                    atol=self.tolerances.tor_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.Index

        return 0
