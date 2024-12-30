#
# Set of useful functions used in different parts of the code
#
import logging
import math

import BOPTools.SplitAPI
import FreeCAD
import numpy as np
import Part

logger = logging.getLogger("general_logger")

from .basic_functions_part1 import (
    ConeParams,
    CylinderParams,
    Plane3PtsParams,
    PlaneParams,
    SphereParams,
    TorusParams,
    MultiPlanesParams,
)
from . import basic_functions_part2 as BF

from .geometry_gu import PlaneGu

from .data_classes import NumericFormat, Options, Tolerances
from .multi_planes import plane_index
from .boolean_function import BoolRegion
from .multi_planes import (
    commonVertex, 
    commonEdge, 
    multiplane_loop,
    no_convex, 
    makeMultiPlanes,
    ) 
from .basic_functions_part1 import is_parallel, is_opposite


def get_box(comp, enlargeBox):
    bb = FreeCAD.BoundBox(comp.BoundBox)
    bb.enlarge(enlargeBox)
    xMin, yMin, zMin = bb.XMin, bb.YMin, bb.ZMin
    xLength, yLength, zLength = bb.XLength, bb.YLength, bb.ZLength

    return Part.makeBox(
        xLength,
        yLength,
        zLength,
        FreeCAD.Vector(xMin, yMin, zMin),
        FreeCAD.Vector(0, 0, 1),
    )


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

    def __init__(self, params, boundBox, Face=None):

        self.Index = 0
        self.__boundBox__ = boundBox
        if params[0] == "Plane":
            self.Type = "Plane"
            self.Surf = PlaneParams(params[1])  # plane point defined as the shortest distance to origin
        elif params[0] == "Plane3Pts":
            self.Type = "Plane"
            self.Surf = Plane3PtsParams(params[1])  # plane point defined with 3 points
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

        self.shape = Face

        if type(Face) is str:
            if Face == "Build":
                self.build_surface()

        if self.shape == "Build":
            raise ValueError(f"stop {params} {boundBox}")
        return

    def build_surface(self):

        if self.Type == "Plane":

            normal = self.Surf.Axis
            p0 = normal.dot(self.Surf.Position)
            Box = FreeCAD.BoundBox(self.__boundBox__)
            Box.enlarge(10)

            pointEdge = []
            for i in range(12):
                edge = Box.getEdge(i)
                p1 = normal.dot(edge[0])
                p2 = normal.dot(edge[1])
                d0 = p0 - p1
                d1 = p2 - p1
                if d1 != 0:
                    a = d0 / d1
                    if a >= 0 and a <= 1:
                        pointEdge.append(edge[0] + a * (edge[1] - edge[0]))

            if len(pointEdge) == 0:
                self.shape = None  # Plane does not cross box
                return

            s = FreeCAD.Vector((0, 0, 0))
            for v in pointEdge:
                s = s + v
            s = s / len(pointEdge)

            vtxvec = []
            for v in pointEdge:
                vtxvec.append(v - s)

            X0 = vtxvec[0]
            Y0 = normal.cross(X0)

            orden = []
            for i, v in enumerate(vtxvec):
                phi = np.arctan2(v.dot(Y0), v.dot(X0))
                orden.append((phi, i))
            orden.sort()

            self.shape = Part.Face(Part.makePolygon([pointEdge[p[1]] for p in orden], True))

            return

        elif self.Type == "Cylinder":
            dir = self.Surf.Axis
            pnt = self.Surf.Center
            rad = self.Surf.Radius

            dmin = dir.dot((self.__boundBox__.getPoint(0) - pnt))
            dmax = dmin
            for i in range(1, 8):
                d = dir.dot((self.__boundBox__.getPoint(i) - pnt))
                dmin = min(d, dmin)
                dmax = max(d, dmax)

            pnt = pnt + (dmin - 5) * dir
            length = dmax - dmin + 10
            self.shape = Part.makeCylinder(rad, length, pnt, dir, 360.0)
            return

        elif self.Type == "Cone":
            #         dir  = self.Surf.Axis
            #         apex = self.Surf.Apex
            #         tan = math.tan(self.Surf.SemiAngle)
            #         rad = tan*diag # New radius far away to cover all geometry

            axis = self.Surf.Axis
            apex = self.Surf.Apex
            tan = math.tan(self.Surf.SemiAngle)

            dmax = axis.dot((self.__boundBox__.getPoint(0) - apex))
            for i in range(1, 8):
                d = axis.dot((self.__boundBox__.getPoint(i) - apex))
                dmax = max(d, dmax)

            if dmax > 0:
                length = dmax + 10
                rad = tan * length
                self.shape = Part.makeCone(0.0, rad, length, apex, axis, 360.0)
            else:
                self.shape = None
            return

        elif self.Type == "Sphere":
            rad = self.Surf.Radius
            pnt = self.Surf.Center
            self.shape = Part.makeSphere(rad, pnt).Faces[0]
            return

        elif self.Type == "Torus":
            axis = self.Surf.Axis
            center = self.Surf.Center
            majorR = self.Surf.MajorRadius
            minorR = self.Surf.MinorRadius

            torus = Part.makeTorus(majorR, minorR, center, axis)
            self.shape = torus.Faces[0]
            return
        
        elif self.type == "MultiPlane":
            planes = self.Surf.Planes
            vertexes = self.Surf.Vertexes
            metaplanes = makeMultiPlanes(planes, vertexes, self.__boundBox__)
            self.shape = metaplanes

        else:
            logger.error(f"Type {self.Type} is not defined")
            return

class MetaSurfIndex:
    def __init__(self,index,surfaces):
        self.index = index
        self.surfaces = surfaces
        self.single_surface = isinstance(surfaces,int)

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
        

        surfname = ["Planes", "Cyl", "Cone", "Sph", "Tor","MultiP","OpenCyl"]
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
            self.primitive_surfaces = SurfacesDict(options=self.options,tolerances=self.tolerances,numeric_format=self.numeric_format)
            self.surfaceNumber = 0
            self.__last_obj__ = ("", -1)
            for key in surfname:
                self.__surfIndex__[key] = []
        return

    def get_surface(self, index):
        return self.primitive_surfaces.get_surface(index)

    def del_surface(self, index):
        self.primitive_surfaces.del_surface(index)

    def extend(self, surface):
        self.primitive_surfaces.extend(surface)

    def add_plane(self, plane, fuzzy):
        pid,exist = self.primitive_surfaces.add_plane(plane,fuzzy)
        same_dir = True
        if exist :
            p_in = self.primitive_surfaces.get_surface(pid)
            same_dir = not is_opposite(plane.Surf.Axis,p_in.Surf.Axis)

            found = False
            for i, p in enumerate(self["Planes"]):
                if pid in p.surfaces:
                    self.__last_obj__ = ("Planes", i)
                    found = True
                    break
            if found :
                    return -p if (p.reverse == same_dir) else p
                    # Equivalent to : 
                    # if p.reverse and same_dir :
                    #     return -p
                    # if p.reverse and not same_dir:
                    #    return p 
                    # if not p.reverse and same_dir :
                    #    return p
                    # if not p.reverse and not same_dir :
                    #    return -p
    
        self.surfaceNumber += 1
        if not same_dir :
            pid = -pid
        plane = BoolRegion(self.surfaceNumber,str(pid), reverse= not same_dir )
        self.__last_obj__ = ("Planes", len(self["Planes"]))
        self["Planes"].append(plane)
        self.__surfIndex__["Planes"].append(plane.__int__())
        return plane

    def add_cylinder(self, cyl, orientation, fuzzy=False):
        pid,exist = self.primitive_surfaces.add_cylinder(cyl,fuzzy)
        if exist :
            found = False
            for i, cyl in enumerate(self["Cyl"]):
                if pid in cyl.surfaces:
                    self.__last_obj__ = ("Cyl", i)
                    found = True
                    break
            if found : return -cyl if (cyl.reverse == (orientation == "Forward")) else cyl
    
        self.surfaceNumber += 1
        if orientation == "Forward":
            pid = -pid
            reverse = False
        else:
            reverse = True     
        cylinder = BoolRegion(self.surfaceNumber,str(pid),reverse=reverse)
        self.__last_obj__ = ("Cyl", len(self["Cyl"]))
        self["Cyl"].append(cylinder)
        self.__surfIndex__["Cyl"].append(cylinder.__int__())
        return cylinder

    def add_cone(self, cone, orientation):
        pid,exist = self.primitive_surfaces.add_cone(cone)
        if exist :
            found = False
            for i, kne in enumerate(self["Cone"]):
                if pid in kne.surfaces:
                    self.__last_obj__ = ("Cone", i)
                    found = True
                    break
            if found : return -kne if (kne.reverse == (orientation == "Forward")) else kne
    
        self.surfaceNumber += 1
        if orientation == "Forward":
            pid = -pid
            reverse = False
        else:
            reverse = True     
        kne = BoolRegion(self.surfaceNumber,str(pid),reverse=reverse)
        self.__last_obj__ = ("Cone", len(self["Cone"]))
        self["Cone"].append(kne)
        self.__surfIndex__["Cone"].append(kne.__int__())
        return kne

    def add_sphere(self, sphere, orientation):
        pid,exist = self.primitive_surfaces.add_sphere(sphere)
        if exist :
            found = False
            for i, sph in enumerate(self["Sph"]):
                if pid in sph.surfaces:
                    self.__last_obj__ = ("Sph", i)
                    found = True
                    break
            if found :
                return -sph if (sph.reverse == (orientation == "Forward")) else sph 
    
        self.surfaceNumber += 1
        if orientation == "Forward":
            pid = -pid
            reverse = False
        else:
            reverse = True     
        
        sph = BoolRegion(self.surfaceNumber,str(pid),reverse=reverse)
        self.__last_obj__ = ("Sph", len(self["Sph"]))
        self["Sph"].append(sph)
        self.__surfIndex__["Sph"].append(sph.__int__())
        return sph

    def add_torus(self, torus, orientation):
        pid,exist = self.primitive_surfaces.add_torus(torus)
        if exist :
            found = False
            for i, tor in enumerate(self["Tor"]):
                if pid in tor.surfaces:
                    self.__last_obj__ = ("Tor", i)
                    found = True
                    break
            if found : 
                return -tor if (tor.reverse == (orientation == "Forward")) else tor
        
        self.surfaceNumber += 1
        if orientation == "Forward":
            pid = -pid
            reverse = False
        else:
            reverse = True     

        tor = BoolRegion(self.surfaceNumber,str(pid),reverse=reverse)
        self.__last_obj__ = ("Tor", len(self["Tor"]))
        self["Tor"].append(tor)
        self.__surfIndex__["Tor"].append(tor.__int__())
        return tor

    def add_multiPlane(self, multiP):
        multiP_surfaces = []
        for mp in multiP.Surf.Planes:
            pid,exist = self.primitive_surfaces.add_plane(mp,True)
            if exist:
                p = self.primitive_surfaces.get_surface(pid)     
                if is_opposite(mp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid 
            multiP_surfaces.append(str(pid))
        multiP_region = BoolRegion(0,':'.join(multiP_surfaces))    
        
        add_multiP = True
        for i, mp in enumerate(self["MultiP"]):
            if mp == multiP_region:
                add_multiP = False
                self.__last_obj__ = ("MultiP", i)
                break
            
        if add_multiP:
            self.surfaceNumber += 1
            multiP_region = multiP_region.copy(self.surfaceNumber)
            self.__last_obj__ = ("MultiP", len(self["MultiP"]))
            self["MultiP"].append(multiP_region)
            self.__surfIndex__["MultiP"].append(multiP_region.__int__())
            return multiP_region
        else:  
            return mp

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
                if BF.is_same_plane(
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
                if BF.is_same_plane(
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
                if BF.is_same_plane(
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
                if BF.is_same_plane(
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
            if BF.is_same_cylinder(
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
            if BF.is_same_cone(
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
            if BF.is_same_sphere(
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
            if BF.is_same_torus(
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
        
    def get_id(self,facein):
        
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
                if BF.is_same_plane(
                    facein.Surf,
                    s.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                ):
                    return s.Index

        elif facein.Type == "Cylinder":
            for s in self["Cyl"]:
                if BF.is_same_cylinder(
                    facein.Surf,
                    s.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                ):
                    return s.Index

        elif facein.Type == "Cone":
            for s in self["Cone"]:
                if BF.is_same_cone(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.kne_distance,
                    atol=self.tolerances.kne_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.Index

        elif facein.Type == "Sphere":
            for s in self["Sph"]:
                if BF.is_same_sphere(facein.Surf, s.Surf, self.tolerances.sph_distance, rel_tol=self.tolerances.relativeTol):
                    return s.Index

        elif facein.Type == "Torus":
            for s in self["Tor"]:
                if BF.is_same_torus(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.tor_distance,
                    atol=self.tolerances.tor_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.Index

        return 0

def split_bop(solid, tools, tolerance, options, scale=0.1):

    if tolerance >= 0.1:
        compSolid = BOPTools.SplitAPI.slice(solid, tools, "Split", tolerance=tolerance)

    elif tolerance < 1e-12:
        if options.scaleUp:
            tol = 1e-13 if options.splitTolerance == 0 else options.splitTolerance
            compSolid = split_bop(solid, tools, tol / scale, options, 1.0 / scale)
        else:
            compSolid = BOPTools.SplitAPI.slice(solid, tools, "Split", tolerance=tolerance)

    else:
        try:
            compSolid = BOPTools.SplitAPI.slice(solid, tools, "Split", tolerance=tolerance)
        except:
            compSolid = split_bop(solid, tools, tolerance * scale, options, scale)

    return compSolid

def get_multiplanes(solid, BoundBox = None):
    """ identify and return all multiplanes in the solid.
    """
    planes = []
    for i,f in enumerate(solid.Faces):
        if isinstance(f.Surface, PlaneGu) :
            planes.append(plane_index(f,i))

    multiplane_list = []
    multiplane_objects = []
    plane_index_set = set()
    for p in planes:
        loop = False
        for mp in multiplane_list:
            if p in mp:
                loop = True
                break
        if loop:
            continue
        mplanes = [p]
        multiplane_loop([p], mplanes, planes)
        if len(mplanes) != 1:
            if no_convex(mplanes):
                mp_params = build_multip_params(mplanes)
                mp = GeounedSurface(
                    ("MultiPlane", mp_params),
                    BoundBox,
                    Face= 'Build' if BoundBox is not None else None,
                )
                for pp in mplanes:
                    plane_index_set.add(pp.index)
                multiplane_list.append(mplanes)
                multiplane_objects.append(mp)
    return multiplane_objects,plane_index_set

def build_multip_params(plane_list):

    planeparams = []
    edges = []
    vertexes = []

    for p in plane_list:
        #plane = PlaneGu(p)
        #planeparams.append((plane.Position, plane.Axis, plane.dim1, plane.dim2))
        normal = p.plane.Surface.Axis if p.plane.Orientation == "Forward" else -p.plane.Surface.Axis
        gp = GeounedSurface(("Plane", (p.plane.Surface.Position, normal, 1., 1.)), None,None)
        planeparams.append(gp)

    
    ajdacent_planes = [[] for i in range(len(plane_list))]
    for i, p1 in enumerate(plane_list):
        for j, p2 in enumerate(plane_list[i + 1 :]):
            e = commonEdge(p1.plane, p2.plane)
            if e is not None:
                edges.append(e)
                ajdacent_planes[i].append((j, e))
                ajdacent_planes[j].append((i, e))

    vertex_list = []
    for i, e1 in enumerate(edges):

        for e2 in edges[i + 1 :]:
            vertex_list.extend(commonVertex(e1, e2))

    vertexes = []
    while len(vertex_list) > 0:
        v = vertex_list.pop()
        n = 0
        for vi in reversed(vertex_list):
            if v.Point == vi.Point:
                n += 1
                vertex_list.remove(vi)
        if n > 0:
            vertexes.append((v, n + 1))

    return (planeparams, edges, vertexes)
