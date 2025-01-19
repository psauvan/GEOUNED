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

        self.Index0 = 0
        self.bVar = None
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
            "Plane": "Planes",
            "Cylinder": "Cyl",
            "Cone": "Cone",
            "Sphere": "Sph",
            "Torus": "Tor",
            "MultiPlane": "MultiP",
            "ReverseCan": "revCan",
            "RoundCorner": "RoundC",
        }

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

    def get_surface(self, pindex):

        for key, values in self.__surfIndex__.items():
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

    def add_plane(self, plane, fuzzy):
        pid, exist = self.primitive_surfaces.add_plane(plane, fuzzy)
        same_dir = True
        add_plane = True

        if exist:
            p_in = self.get_primitive_surface(pid)
            same_dir = not is_opposite(plane.Surf.Axis, p_in.Surf.Axis)
            if not same_dir:
                pid = -pid

            for p_region in self["Planes"]:
                boundary = p_region.isSameInterface(pid)
                if abs(boundary) == 1:
                    add_plane = False
                    break

        if add_plane:
            self.surfaceNumber += 1
            newregion = pid.copy(self.surfaceNumber)
            self["Planes"].append(newregion)
        else:
            newregion = p_region if boundary > 0 else -p_region

        return newregion

    def add_cylinder(self, cylinder, fuzzy=False):

        cid, exist_c = self.primitive_surfaces.add_cylinder(cylinder.Surf.Cylinder)
        if cylinder.Surf.Orientation == "Forward":
            cid = -cid
        cylinder_region = cid

        if cylinder.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(cylinder.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(cylinder.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            cylinder_region = cylinder_region * pid

        add_cyl = True
        for cyl_region in self["Cyl"]:
            boundary = cylinder_region.isSameInterface(cyl_region)
            if abs(boundary) == 1:
                add_cyl = False
                break

        if add_cyl:
            self.surfaceNumber += 1
            newregion = cylinder_region.copy(self.surfaceNumber)
            self["Cyl"].append(newregion)
        else:
            newregion = cyl_region if boundary > 0 else -cyl_region

        return newregion

    def add_cone(self, cone):
        cid, exist_c = self.primitive_surfaces.add_cone(cone.Surf.Cone)

        if cone.Surf.Orientation == "Forward":
            cid = -cid
        cone_region = cid

        if cone.Surf.ApexPlane:
            pid, exist_p = self.primitive_surfaces.add_plane(cone.Surf.ApexPlane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(cone.Surf.ApexPlane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid

            if cone.Surf.Orientation == "Forward":
                cone_region = cone_region * pid
            else:
                cone_region = cone_region + pid

        if cone.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(cone.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(cone.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            cone_region = cone_region * pid

        add_cone = True
        for kne_region in self["Cone"]:
            boundary = cone_region.isSameInterface(kne_region)
            if abs(boundary) == 1:
                add_cone = False
                break

        if add_cone:
            self.surfaceNumber += 1
            newregion = cone_region.copy(self.surfaceNumber)
            self["Cone"].append(newregion)
        else:
            newregion = kne_region if boundary > 0 else -kne_region
        return newregion

    def add_sphere(self, sphere):
        sid, exist_s = self.primitive_surfaces.add_sphere(sphere.Surf.Sphere)

        if sphere.Surf.Orientation == "Forward":
            sid = -sid

        sphere_region = sid
        if sphere.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(sphere.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(sphere.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            sphere_region = sphere_region * pid

        add_sph = True
        for sph_region in self["Sph"]:
            boundary = sphere_region.isSameInterface(sph_region)
            if abs(boundary) == 1:
                add_sph = False
                break

        if add_sph:
            self.surfaceNumber += 1
            newregion = sphere_region.copy(self.surfaceNumber)
            self["Sph"].append(newregion)
        else:
            newregion = sph_region if boundary > 0 else -sph_region
        return newregion

    def add_torus(self, torus):
        tid, exist_t = self.primitive_surfaces.add_torus(torus.Surf.Torus)

        if torus.Surf.Orientation == "Forward":
            tid = -tid

        torus_region = tid

        psurf = []
        for tp in torus.Surf.UPlanes:
            pid, exist_p = self.primitive_surfaces.add_plane(tp, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(tp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            psurf.append(pid)

        if len(psurf) == 2:
            torus_region = torus_region * (psurf[0] + psurf[1])
        elif len(psurf) == 1:
            torus_region = torus_region * psurf[0]

        if torus.Surf.VSurface:
            if torus.Surf.VSurface.Type == "Plane":
                sid, exist_s = self.primitive_surfaces.add_plane(torus.Surf.VSurface, True)
            elif torus.Surf.VSurface.Type == "Cylinder":
                sid, exist_s = self.primitive_surfaces.add_cylinder(torus.Surf.VSurface, True)
            else:
                sid, exist_s = self.primitive_surfaces.add_cone(torus.Surf.VSurface)

            if exist_s:
                surf = self.get_primitive_surface(sid)
                if torus.Surf.VSurface.Type == "Plane":
                    if is_opposite(torus.Surf.VSurface.Surf.Axis, surf.Surf.Axis, self.tolerances.pln_angle):
                        sid = -sid
                else:
                    if torus.Surf.SOrientation == "Forward":
                        sid = -sid

            torus_region = torus_region * sid

        add_torus = True
        for tor_region in self["Tor"]:
            boundary = torus_region.isSameInterface(tor_region)
            if abs(boundary) == 1:
                add_torus = False
                break

        if add_torus:
            self.surfaceNumber += 1
            newregion = torus_region.copy(self.surfaceNumber)
            self["Tor"].append(newregion)
        else:
            newregion = tor_region if boundary > 0 else -tor_region
        return newregion

    def add_multiPlane(self, multiP):

        multiP_region = None
        for mp in multiP.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(mp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                if is_opposite(mp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid

            multiP_region = BoolRegion.add(multiP_region, pid)

        add_multiP = True
        for mp_region in self["MultiP"]:
            boundary = multiP_region.isSameInterface(mp_region)
            if abs(boundary) == 1:
                add_multiP = False
                break

        if add_multiP:
            self.surfaceNumber += 1
            newregion = multiP_region.copy(self.surfaceNumber)
            self["MultiP"].append(newregion)
        else:
            newregion = mp_region if boundary > 0 else -mp_region
        return newregion

    def add_reverseCan(self, reverseCan):
        pid, exist = self.primitive_surfaces.add_cylinder(reverseCan.Surf.Cylinder, True)
        reverseCan_region = pid

        for cp in reverseCan.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(cp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                if is_opposite(cp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid

            reverseCan_region = reverseCan_region + pid

        add_can = True
        for cs_region in self["RevCan"]:
            boundary = reverseCan_region.isSameInterface(cs_region)
            if abs(boundary) == 1:
                add_can = False
                break

        if add_can:
            self.surfaceNumber += 1
            newregion = reverseCan_region.copy(self.surfaceNumber)
            self["RevCan"].append(newregion)
        else:
            newregion = cs_region if boundary > 0 else -cs_region
        return newregion

    def add_roundCorner(self, roundC):
        cid, exist_c = self.primitive_surfaces.add_cylinder(roundC.Surf.Cylinder, True)

        pid, exist_p = self.primitive_surfaces.add_plane(roundC.Surf.AddPlane, True)
        if exist_p:
            p = self.get_primitive_surface(pid)
            if is_opposite(roundC.Surf.AddPlane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                pid = -pid

        roundC_region = -cid + pid

        for cp in roundC.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(cp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                if is_opposite(cp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid

            if roundC.Surf.Configuration == "AND":
                roundC_region = roundC_region * pid
            else:
                roundC_region = roundC_region + pid

        add_corner = True
        for rc_region in self["RoundC"]:
            boundary = roundC_region.isSameInterface(rc_region)
            if abs(boundary) == 1:
                add_corner = False
                break

        if add_corner:
            self.surfaceNumber += 1
            newregion = roundC_region.copy(self.surfaceNumber)
            self["RoundC"].append(newregion)
        else:
            newregion = rc_region if boundary > 0 else -rc_region
        return newregion


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
                if self[lastKey][lastInd].bVar == index:
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
                    fuzzy=(fuzzy, p.bVar.__int__()),
                ):
                    add_plane = False
                    bVar = p.bVar
                    self.__last_obj__ = ("PX", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("PX", len(self["PX"]))
                self["PX"].append(plane)
                self.__surfIndex__["PX"].append(plane.bVar.__int__())

        elif is_parallel(plane.Surf.Axis, ey, self.tolerances.pln_angle):
            add_plane = True
            for i, p in enumerate(self["PY"]):
                if is_same_plane(
                    plane.Surf,
                    p.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                    fuzzy=(fuzzy, p.bVar.__int__()),
                ):
                    add_plane = False
                    bVar = p.bVar
                    self.__last_obj__ = ("PY", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("PY", len(self["PY"]))
                self["PY"].append(plane)
                self.__surfIndex__["PY"].append(plane.bVar.__int__())

        elif is_parallel(plane.Surf.Axis, ez, self.tolerances.pln_angle):
            add_plane = True
            for i, p in enumerate(self["PZ"]):
                if is_same_plane(
                    plane.Surf,
                    p.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                    fuzzy=(fuzzy, p.bVar.__int__()),
                ):
                    add_plane = False
                    bVar = p.bVar
                    self.__last_obj__ = ("PZ", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("PZ", len(self["PZ"]))
                self["PZ"].append(plane)
                self.__surfIndex__["PZ"].append(plane.bVar.__int__())

        else:
            add_plane = True
            for i, p in enumerate(self["P"]):
                if is_same_plane(
                    plane.Surf,
                    p.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                    fuzzy=(fuzzy, p.bVar.__int__()),
                ):
                    add_plane = False
                    bVar = p.bVar
                    self.__last_obj__ = ("P", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("P", len(self["P"]))
                self["P"].append(plane)
                self.__surfIndex__["P"].append(plane.bVar.__int__())

        if add_plane:
            return plane.bVar, False
        else:
            return bVar, True

    def add_cylinder(self, cyl, fuzzy=False):
        addCyl = True
        for i, c in enumerate(self["Cyl"]):
            if is_same_cylinder(
                cyl.Surf,
                c.Surf,
                options=self.options,
                tolerances=self.tolerances,
                numeric_format=self.numeric_format,
                fuzzy=(fuzzy, c.bVar.__int__()),
            ):
                addCyl = False
                bVar = c.bVar
                self.__last_obj__ = ("Cyl", i)
                break

        if addCyl:
            self.surfaceNumber += 1
            cyl.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Cyl", len(self["Cyl"]))
            self["Cyl"].append(cyl)
            self.__surfIndex__["Cyl"].append(cyl.bVar.__int__())
            return cyl.bVar, False
        else:
            return bVar, True

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
                bVar = c.bVar
                self.__last_obj__ = ("Cone", i)
                break
        if cone_added:
            self.surfaceNumber += 1
            cone.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Cone", len(self["Cone"]))
            self["Cone"].append(cone)
            self.__surfIndex__["Cone"].append(cone.bVar.__int__())
            return cone.bVar, False
        else:
            return bVar, True

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
                bVar = s.bVar
                self.__last_obj__ = ("Sph", i)
                break
        if sphere_added:
            self.surfaceNumber += 1
            sph.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Sph", len(self["Sph"]))
            self["Sph"].append(sph)
            self.__surfIndex__["Sph"].append(sph.bVar.__int__())
            return sph.bVar, False
        else:
            return bVar, True

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
                bVar = s.bVar
                self.__last_obj__ = ("Tor", i)
                break
        if add_torus:
            self.surfaceNumber += 1
            tor.bVar = BoolRegion(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Tor", len(self["Tor"]))
            self["Tor"].append(tor)
            self.__surfIndex__["Tor"].append(tor.bVar.__int__())
            return tor.bVar, False
        else:
            return bVar, True

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
                    return s.bVar.__int__()

        elif facein.Type == "Cylinder":
            for s in self["Cyl"]:
                if is_same_cylinder(
                    facein.Surf,
                    s.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                ):
                    return s.bVar.__int__()

        elif facein.Type == "Cone":
            for s in self["Cone"]:
                if is_same_cone(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.kne_distance,
                    atol=self.tolerances.kne_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.bVar.__int__()

        elif facein.Type == "Sphere":
            for s in self["Sph"]:
                if is_same_sphere(facein.Surf, s.Surf, self.tolerances.sph_distance, rel_tol=self.tolerances.relativeTol):
                    return s.bVar.__int__()

        elif facein.Type == "Torus":
            for s in self["Tor"]:
                if is_same_torus(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.tor_distance,
                    atol=self.tolerances.tor_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.bVar.__int__()

        return 0
