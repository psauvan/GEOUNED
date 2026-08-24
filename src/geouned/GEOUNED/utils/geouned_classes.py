#
# Set of useful functions used in different parts of the code
#
import logging
import math

logger = logging.getLogger("general_logger")

from .basic_functions_part1 import (
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
    RoundCornerParams,
    MultiRoundCornerParams,
    ReversedConeCylParams,
    CanParams,
    TConeParams,
)
from .basic_functions_part1 import round_corner_region, multi_round_corner_region, can_region, tcone_region
from .basic_functions_part2 import is_same_plane, is_same_cylinder, is_same_cone, is_same_sphere, is_same_torus

from .data_classes import NumericFormat, Options, Tolerances
from .boolean_function import BoolSurface, BoolVariable, literal_sign
from .build_shape_functions import (
    makePlane,
    makeCylinder,
    makeCone,
    makeMultiPlanes,
    makeCan,
    makeTCone,
    makeRoundCorner,
    makeMultiRoundCorner,
)
from .basic_functions_part1 import is_parallel, is_opposite
from ...geo import (
    GBoundBox,
    GSolid,
    GVector,
    Gcommon,
    Gdistance,
    Gfirst_shell,
    Gmake_compound,
    Gmake_sphere,
    Gmake_torus,
    to_gboundbox,
)


def _empty_boundbox():
    """Seed for accumulating a GBoundBox via .union() -- an inverted-infinite box, same trick as FreeCAD's own empty `BoundBox()`."""
    return GBoundBox(math.inf, math.inf, math.inf, -math.inf, -math.inf, -math.inf)


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
            bbox = _empty_boundbox()
            for s in comsolid:
                vol += s.Volume
                bbox = bbox.union(s.BoundBox)
            self.BoundBox = bbox
            self.Volume = vol
        else:
            if refine:
                try:
                    self.Solids = comsolid.refine().Solids
                except Exception:
                    self.Solids = comsolid.Solids

                for i, s in enumerate(self.Solids):
                    if s.Volume < 0:
                        self.Solids[i] = s.reverse()
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
        bbox = _empty_boundbox()
        for s in solidList:
            vol += s.Volume
            bbox = bbox.union(s.BoundBox)
        self.BoundBox = bbox

    def set_cad_solid(self):
        if self.Solids is not None:
            gcompound = Gmake_compound(self.Solids)
            self.CADSolid = gcompound.__native__
            self.Volume = gcompound.Volume
            self.BoundBox = gcompound.BoundBox

    def optimalBoundingBox(self):
        if self.CADSolid is None:
            self.set_cad_solid()
        return GSolid(self.CADSolid).optimal_bounding_box()

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

        # self.CADSolid/solid are both native shapes (set_cad_solid() stores
        # gcompound.__native__ deliberately, since callers elsewhere need a
        # real native shape too) -- wrap once here rather than calling
        # native-only conveniences (.Solids/.distToShape/.common/.Volume)
        # directly, since only FreeCAD's own Part.Shape has those; OCC/OCP's
        # native TopoDS_Compound has none of them. Gdistance already finds
        # the true minimum distance between two arbitrary (possibly
        # multi-solid) shapes on its own, so the manual per-sub-solid
        # nested-loop distToShape scan below is no longer needed either.
        g1 = GSolid(self.CADSolid)
        g2 = GSolid(solid)

        # A cheap early-out for the genuinely-disjoint case, WITHOUT the
        # bug the previous Gdistance-based version had: Gdistance measures
        # *surface-to-surface* distance, which is large whenever one solid
        # is fully nested inside the other with room to spare (e.g. a small
        # object floating inside a much bigger enclosure, not touching its
        # walls) -- a real, common, legitimate case this function's own
        # docstring explicitly promises to detect (-1/-2). Confirmed live,
        # 2026-08-24, Solidos/test_models/Enclosures/w_encl.stp: a real
        # sphere fully embedded in its own enclosure (common_volume equals
        # the sphere's own volume exactly) still measured Gdistance=40.74mm
        # (its own surface, not touching the enclosure's walls), so the old
        # `Gdistance > dtolerance -> return 1` check wrongly classified it
        # as disjoint before ever reaching the volume-based check below --
        # assignEnclosure then never recognized this solid as belonging to
        # the enclosure at all, and the enclosure's own generated void cell
        # never learned to exclude it, leaving a real geometric overlap
        # (confirmed via a d1suned run: the sphere's own cell got exactly
        # 0.0 tally, all track length silently absorbed by the void cell
        # that wrongly also claims that same space).
        # BoundBox.intersects() is the mathematically sound version of the
        # same "can these possibly overlap" pre-filter: two disjoint
        # BoundBoxes make overlap impossible regardless of surface
        # distance, but a fully-nested BoundBox still reports intersecting
        # (matching FreeCAD's own BoundBox.intersect semantics), so it
        # never produces this false negative.
        if not g1.BoundBox.intersects(g2.BoundBox):
            return 1

        common_volume = sum(c.Volume for c in Gcommon(g1, [g2]))
        if abs(common_volume) < vtolerance:
            return 1
        if abs(g1.Volume - common_volume) / common_volume < vtolerance:
            return -1
        elif abs(g2.Volume - common_volume) / common_volume < vtolerance:
            return -2
        else:
            return 0


class GeounedSurface:

    def __init__(self, params, Face=None):
        self.Index0 = 0
        self.bVar = None
        self.region = None
        self.components = None  # dict[abs(id), GeounedSurface]: numbering<->surface relation, set by MetaSurfacesDict.add_* for every registered surface (Plane's is the trivial 1-component case; Cylinder/Cone/Sphere/Torus and Can/TCone/RoundCorner/MultiRoundCorner hold their real components)
        if params[0] == "Plane":
            self.Type = "Plane"
            self.Surf = PlaneParams(params[1])  # plane point defined as the shortest distance to origin
            if len(params) > 2:
                self.Orientation = params[2]
            else:
                self.Orientation = "Reversed"
        elif params[0] == "CylinderOnly":
            self.Type = params[0]
            self.Surf = CylinderOnlyParams(params[1])
            self.Orientation = None
        elif params[0] == "ConeOnly":
            self.Type = params[0]
            self.Surf = ConeOnlyParams(params[1])
            self.Orientation = None
        elif params[0] == "SphereOnly":
            self.Type = params[0]
            self.Surf = SphereOnlyParams(params[1])
            self.Orientation = None
        elif params[0] == "TorusOnly":
            self.Type = params[0]
            self.Surf = TorusOnlyParams(params[1])
            self.Orientation = None
        elif params[0] == "Cylinder":
            self.Type = params[0]
            self.Surf = CylinderParams(params[1])
            if len(params) > 2:
                self.Orientation = params[2]
            else:
                self.Orientation = None
        elif params[0] == "Cone":
            self.Type = params[0]
            self.Surf = ConeParams(params[1])
            if len(params) > 2:
                self.Orientation = params[2]
            else:
                self.Orientation = None
        elif params[0] == "Sphere":
            self.Type = params[0]
            self.Surf = SphereParams(params[1])
            if len(params) > 2:
                self.Orientation = params[2]
            else:
                self.Orientation = None
        elif params[0] == "Torus":
            self.Type = params[0]
            self.Surf = TorusParams(params[1])
            if len(params) > 2:
                self.Orientation = params[2]
            else:
                self.Orientation = None
        elif params[0] == "MultiPlane":
            self.Type = params[0]
            self.Surf = MultiPlanesParams(params[1])
            self.Orientation = "Reversed"
        elif params[0] == "Can":
            self.Type = params[0]
            self.Surf = CanParams(params[1])
            if len(params) > 2:
                self.Orientation = params[2]
            else:
                self.Orientation = None
        elif params[0] == "TCone":
            self.Type = params[0]
            self.Surf = TConeParams(params[1])
            if len(params) > 2:
                self.Orientation = params[2]
            else:
                self.Orientation = None
        elif params[0] == "RoundCorner":
            self.Type = params[0]
            self.Surf = RoundCornerParams(params[1])
            self.Orientation = params[2]
        elif params[0] == "MultiRoundCorner":
            self.Type = params[0]
            self.Surf = MultiRoundCornerParams(params[1])
            self.Orientation = params[1][2]
        elif params[0] == "ReversedConeCylinder":
            self.Type = params[0]
            self.Surf = ReversedConeCylParams(params[1])
            self.Orientation = "Reversed"
        else:
            print(f"type {params[0]} not found")

        self.shape = Face
        return

    def __eq__(self, s2):
        if type(self.Surf) != type(s2.Surf):
            return False
        else:
            return self.Surf == s2.Surf

    def build_surface(self, boundBox, forward=False):

        Box = to_gboundbox(boundBox)
        if self.Type == "Plane":
            Box = Box.enlarged(10)
            self.shape = makePlane(self.Surf.Axis, self.Surf.Position, Box)
            self.shell = self.shape

        elif self.Type == "Cylinder" or self.Type == "CylinderOnly":
            cyl = self.Surf.Cylinder if self.Type == "Cylinder" else self
            self.shape, self.shell = makeCylinder(cyl.Surf.Center, cyl.Surf.Axis, cyl.Surf.Radius, Box)

        elif self.Type == "Cone" or self.Type == "ConeOnly":
            kne = self.Surf.Cone if self.Type == "Cone" else self
            tan = math.tan(kne.Surf.SemiAngle)
            result = makeCone(kne.Surf.Axis, kne.Surf.Apex, tan, Box)
            if result is None:
                self.shape = None
                self.shell = None
            else:
                self.shape, self.shell = result

        elif self.Type == "Sphere" or self.Type == "SphereOnly":
            sph = self.Surf.Sphere if self.Type == "Sphere" else self
            rad = sph.Surf.Radius
            pnt = sph.Surf.Center
            self.shape = Gmake_sphere(pnt, rad).__native__
            self.shell = Gfirst_shell(self.shape)
            return

        elif self.Type == "Torus" or self.Type == "TorusOnly":
            tor = self.Surf.Torus if self.Type == "Torus" else self
            axis = tor.Surf.Axis
            center = tor.Surf.Center
            majorR = tor.Surf.MajorRadius
            minorR = tor.Surf.MinorRadius

            torus_solid = Gmake_torus(center, axis, majorR, minorR)
            self.shape = torus_solid.Faces[0].__native__
            self.shell = Gfirst_shell(torus_solid.__native__)
            return

        elif self.Type == "MultiPlane":
            Box = Box.enlarged(10)
            planes = self.Surf.Planes
            vertexes = self.Surf.Vertexes
            result = makeMultiPlanes(planes, vertexes, Box)
            if result is None:
                self.shape = None
                self.shell = None
            else:
                self.shape, self.shell = result

        elif self.Type == "Can":
            self.shape, self.shell = makeCan(self, Box, forward=forward)

        elif self.Type == "TCone":
            self.shape, self.shell = makeTCone(self, Box)

        elif self.Type == "RoundCorner":
            Box = Box.enlarged(10)
            self.shape, self.shell = makeRoundCorner(self, Box)

        elif self.Type == "MultiRoundCorner":
            Box = Box.enlarged(10)
            self.shape, self.shell = makeMultiRoundCorner(self, Box, forward=forward)

        elif self.Type == "ReversedConeCylinder":
            # No need to build shape since this shape not used in decomposition
            pass

        else:
            logger.error(f"Cannot build {self.Type} shape")
            return


class MetaSurfIndex:
    def __init__(self, index, surfaces):
        self.index = index
        self.surfaces = surfaces
        self.single_surface = isinstance(surfaces, int)


def validate_characteristic_sign(region, surf_id, orientation, label):
    """Self-consistency check shared by every composite/Tier-2 region (Can,
    TCone, Cylinder, Cone, ...): the characteristic (main) surface's own id
    must appear negative in `region` when its real Orientation is Forward,
    positive when Reversed -- the canonical rule confirmed empirically
    across can_region/add_cone/add_cylinder. Called right where a region is
    finalized, so a violation here is a genuine construction bug, never a
    legitimate cross-region relationship (that's what isSameInterface's
    on_conflict="ignore" mode exists for instead)."""
    sign = literal_sign(region, surf_id)
    expected = -1 if orientation == "Forward" else 1
    if sign != expected:
        raise RuntimeError(
            f"{label}: characteristic surface {surf_id} (Orientation={orientation}) "
            f"has sign {sign} in region {region}, expected {expected}"
        )


class MetaSurfacesDict(dict):

    def __init__(
        self,
        offset: int = 0,
        options: Options = Options(),
        tolerances: Tolerances = Tolerances(),
        numeric_format: NumericFormat = NumericFormat(),
    ):

        self.IndexOffset = offset
        self.options = options
        self.tolerances = tolerances
        self.numeric_format = numeric_format

        surfname = [
            "Planes",
            "Cyl",
            "Cone",
            "Sph",
            "Tor",
            "MultiP",
            "FwdCan",
            "RevCan",
            "FwdTCone",
            "RevTCone",
            "RoundC",
            "MultiRoundC",
            "RevCC",
        ]
        for name in surfname:
            self[name] = []

        self.__surfIndex__ = dict()

        self.primitive_surfaces = SurfacesDict(
            offset=offset, options=self.options, tolerances=self.tolerances, numeric_format=self.numeric_format
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

    # def add_surface(self, surf, fuzzy=False):
    #    if surf.Type == "Plane":
    #       return self.add_plane(surf, fuzzy)
    #    elif surf.Type == "Cylinder":
    #        return self.add_cylinder(surf, fuzzy)
    #    elif surf.Type == "Cone":
    #        return self.add_cone(surf)
    #    elif surf.Type == "Sphere":
    #        return self.add_cphere(surf)
    #    elif surf.Type == "Torus":
    #        return self.add_torus(surf)

    def add_plane(self, plane, fuzzy):
        pid, exist = self.primitive_surfaces.add_plane(plane, fuzzy)
        same_dir = True
        add_plane = True

        if exist:
            p_in = self.get_primitive_surface(pid)
            same_dir = not is_opposite(plane.Surf.Axis, p_in.Surf.Axis)
            if not same_dir:
                pid = -pid

            for p_surf in self["Planes"]:
                if abs(p_surf.region.region.elements[0]) == abs(pid):
                    add_plane = False
                    same_sign = pid == int(math.copysign(pid, p_surf.region.region.elements[0]))
                    break

        if add_plane:
            self.surfaceNumber += 1
            newregion = BoolSurface(self.surfaceNumber, pid)
            plane.region = newregion
            plane.components = {abs(pid): plane}
            self["Planes"].append(plane)
            self.__surfIndex__["Planes"].append(plane.region.__int__())
        else:
            newregion = p_surf.region if same_sign else -p_surf.region

        return newregion

    def add_cylinder(self, cylinder, fuzzy=False):
        cid, exist_c = self.primitive_surfaces.add_cylinder(cylinder.Surf.Cylinder)
        characteristic_id = cid
        if cylinder.Orientation == "Forward":
            cid = -cid
        cylinder_region = BoolSurface(0, cid)
        components = {abs(cid): cylinder.Surf.Cylinder}

        if cylinder.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(cylinder.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(cylinder.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            cylinder_region = cylinder_region * BoolSurface(0, pid)
            components[abs(pid)] = cylinder.Surf.Plane

        validate_characteristic_sign(cylinder_region.region, int(characteristic_id), cylinder.Orientation, "add_cylinder")

        add_cyl = True
        for cyl_surf in self["Cyl"]:
            # Both cylinder_region and cyl_surf.region were already
            # validated self-consistent at their own construction time
            # (above), so a .reverse conflict here is never a bug -- it's
            # two distinct, adjacent cylinders legitimately sharing the
            # same real surface with opposite sense (same reasoning as
            # Can_region/add_sphere).
            boundary = cylinder_region.isSameInterface(cyl_surf.region, on_conflict="ignore")
            if abs(boundary) == 1:
                add_cyl = False
                break

        if add_cyl:
            self.surfaceNumber += 1
            newregion = cylinder_region.copy(self.surfaceNumber)
            cylinder.region = newregion
            cylinder.components = components
            self["Cyl"].append(cylinder)
            self.__surfIndex__["Cyl"].append(cylinder.region.__int__())
        else:
            newregion = cyl_surf.region if boundary > 0 else -cyl_surf.region

        return newregion

    def add_cone(self, cone):
        cid, exist_c = self.primitive_surfaces.add_cone(cone.Surf.Cone)
        characteristic_id = cid

        if cone.Orientation == "Forward":
            cid = -cid
        cone_region = BoolSurface(0, cid)
        components = {abs(cid): cone.Surf.Cone}

        if cone.Surf.ApexPlane:
            pid, exist_p = self.primitive_surfaces.add_plane(cone.Surf.ApexPlane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(cone.Surf.ApexPlane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid

            if cone.Orientation == "Forward":
                cone_region = cone_region * BoolSurface(0, pid)
            else:
                cone_region = cone_region + BoolSurface(0, -pid)
            components[abs(pid)] = cone.Surf.ApexPlane

        if cone.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(cone.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(cone.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            cone_region = cone_region * BoolSurface(0, pid)
            components[abs(pid)] = cone.Surf.Plane

        validate_characteristic_sign(cone_region.region, int(characteristic_id), cone.Orientation, "add_cone")

        add_cone = True
        for kne_surf in self["Cone"]:
            # Both cone_region and kne_surf.region were already validated
            # self-consistent at their own construction time (above), so a
            # .reverse conflict here is never a bug -- it's two distinct,
            # adjacent cones legitimately sharing the same real surface
            # with opposite sense (same reasoning as Can_region/add_sphere).
            boundary = cone_region.isSameInterface(kne_surf.region, on_conflict="ignore")
            if abs(boundary) == 1:
                add_cone = False
                break

        if add_cone:
            self.surfaceNumber += 1
            newregion = cone_region.copy(self.surfaceNumber)
            cone.region = newregion
            cone.components = components
            self["Cone"].append(cone)
            self.__surfIndex__["Cone"].append(cone.region.__int__())
        else:
            newregion = kne_surf.region if boundary > 0 else -kne_surf.region
        return newregion

    def add_sphere(self, sphere):
        sid, exist_s = self.primitive_surfaces.add_sphere(sphere.Surf.Sphere)
        characteristic_id = sid

        if sphere.Orientation == "Forward":
            sid = -sid

        sphere_region = BoolSurface(0, sid)
        components = {abs(sid): sphere.Surf.Sphere}
        if sphere.Surf.Plane:
            pid, exist_p = self.primitive_surfaces.add_plane(sphere.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(sphere.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            sphere_region = sphere_region * BoolSurface(0, pid)
            components[abs(pid)] = sphere.Surf.Plane

        validate_characteristic_sign(sphere_region.region, int(characteristic_id), sphere.Orientation, "add_sphere")

        add_sph = True
        for sph_surf in self["Sph"]:
            # Both sphere_region and sph_surf.region were already validated
            # self-consistent at their own construction time (above), so a
            # .reverse conflict here is never a bug -- it's two distinct,
            # adjacent spheres legitimately sharing the same real surface
            # with opposite sense (same reasoning as Can_region).
            boundary = sphere_region.isSameInterface(sph_surf.region, on_conflict="ignore")
            if abs(boundary) == 1:
                add_sph = False
                break

        if add_sph:
            self.surfaceNumber += 1
            newregion = sphere_region.copy(self.surfaceNumber)
            sphere.region = newregion
            sphere.components = components
            self["Sph"].append(sphere)
            self.__surfIndex__["Sph"].append(sphere.region.__int__())
        else:
            newregion = sph_surf.region if boundary > 0 else -sph_surf.region
        return newregion

    def add_torus(self, torus):
        tid, exist_t = self.primitive_surfaces.add_torus(torus.Surf.Torus)
        characteristic_id = tid

        if torus.Orientation == "Forward":
            tid = -tid

        torus_region = BoolSurface(0, tid)
        components = {abs(tid): torus.Surf.Torus}

        psurf = []
        for tp in torus.Surf.UPlanes:
            pid, exist_p = self.primitive_surfaces.add_plane(tp, True)
            if exist_p:
                p = self.get_primitive_surface(pid)
                if is_opposite(tp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            psurf.append(pid)
            components[abs(pid)] = tp

        if len(psurf) == 2:
            torus_region = torus_region * (BoolSurface(0, psurf[0]) + BoolSurface(0, psurf[1]))
        elif len(psurf) == 1:
            torus_region = torus_region * BoolSurface(0, psurf[0])

        if torus.Surf.VSurface:
            if torus.Surf.VSurface.Type == "Plane":
                sid, exist_s = self.primitive_surfaces.add_plane(torus.Surf.VSurface, True)
            elif torus.Surf.VSurface.Type == "CylinderOnly":
                sid, exist_s = self.primitive_surfaces.add_cylinder(torus.Surf.VSurface, True)
                if torus.Surf.SOrientation == "Forward":
                    sid = -sid
            else:
                sid, exist_s = self.primitive_surfaces.add_cone(torus.Surf.VSurface)
                if torus.Surf.SOrientation == "Forward":
                    sid = -sid

            if exist_s:
                surf = self.get_primitive_surface(sid)
                if torus.Surf.VSurface.Type == "Plane":
                    if is_opposite(torus.Surf.VSurface.Surf.Axis, surf.Surf.Axis, self.tolerances.pln_angle):
                        sid = -sid

            torus_region = torus_region * BoolSurface(0, sid)
            components[abs(sid)] = torus.Surf.VSurface

        validate_characteristic_sign(torus_region.region, int(characteristic_id), torus.Orientation, "add_torus")

        add_torus = True
        for tor_surf in self["Tor"]:
            # Both torus_region and tor_surf.region were already validated
            # self-consistent at their own construction time (above), so a
            # .reverse conflict here is never a bug -- it's two distinct,
            # adjacent tori legitimately sharing the same real surface with
            # opposite sense (same reasoning as Can_region/add_sphere).
            boundary = torus_region.isSameInterface(tor_surf.region, on_conflict="ignore")
            if abs(boundary) == 1:
                add_torus = False
                break

        if add_torus:
            self.surfaceNumber += 1
            newregion = torus_region.copy(self.surfaceNumber)
            torus.region = newregion
            torus.components = components
            self["Tor"].append(torus)
            self.__surfIndex__["Tor"].append(torus.region.__int__())
        else:
            newregion = tor_surf.region if boundary > 0 else -tor_surf.region
        return newregion

    def add_multiPlane(self, multiP):

        multiP_region = None
        for mp in multiP.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(mp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                if is_opposite(mp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid

            multiP_region = BoolSurface.add(multiP_region, BoolSurface(0, pid))

        add_multiP = True
        for mp_surf in self["MultiP"]:
            # A MultiPlane is built purely from real plane orientations
            # (region_sign/is_opposite, per plane), with no separate
            # Fwd/Rev registration split -- but two distinct, adjacent
            # MultiPlanes can still legitimately share one real plane with
            # opposite sense, the same "structurally complementary,
            # neither built by negation" false positive already confirmed
            # for Can_region/TCone_region/MultiRoundCorner (tank.stp,
            # modelCell_670000.stp). on_conflict="ignore" trusts the
            # structural comparison the same way.
            boundary = multiP_region.isSameInterface(mp_surf.region, on_conflict="ignore")
            if abs(boundary) == 1:
                add_multiP = False
                break

        if add_multiP:
            self.surfaceNumber += 1
            newregion = multiP_region.copy(self.surfaceNumber)
            multiP.region = newregion
            self["MultiP"].append(multiP)
            self.__surfIndex__["MultiP"].append(multiP.region.__int__())
        else:
            newregion = mp_surf.region if boundary > 0 else -mp_surf.region
        return newregion

    # Reversed and Forward Can region are defined as follow:
    # assumed additional plane always oriented toward the cylinder (fwd or rev orientation)
    #  - FR AND : -C P S
    #  - FF AND : -C (P: -S)
    #  - RF OR  : -C : -P: -S
    #  - RR OR  : -C : -P   S
    #  - RF AND : same as RR OR
    #  - RR AND : same as RF OR
    # if only plane assume S = True in the previous expressions

    def _resolve_plane_id(self, plane):
        """Register `plane` in the global primitive-surfaces dedup registry
        and return its signed id, flipping the sign (and the plane's own
        Axis/bVar, in place) if it turns out to be the opposite-facing
        duplicate of an already-registered plane. Shared by Can_region and
        TCone_region -- both build their solid from these ids, so the sign
        must be kept consistent with whichever plane instance ends up
        registered."""
        pid, exist = self.primitive_surfaces.add_surface(plane, True)
        if exist:
            p = self.get_primitive_surface(pid)
            if is_opposite(plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                pid = -pid
                # change plane axis because Can/TCone shape is build with solid definition based on Surfaces dict reference
                plane.Surf.Axis = -plane.Surf.Axis
                plane.bVar = pid
        return pid

    def Can_region(self, FRCan):
        cylCan = FRCan.Surf.Cylinder
        cid, exist = self.primitive_surfaces.add_cylinder(cylCan.Surf.Cylinder, True)

        raw_surf_list = []
        if FRCan.Surf.s1 is not None:
            raw_surf_list.append((FRCan.Surf.s1, FRCan.Surf.s1_configuration))
        if FRCan.Surf.s2 is not None:
            raw_surf_list.append((FRCan.Surf.s2, FRCan.Surf.s2_configuration))

        components = {abs(cid): cylCan.Surf.Cylinder}
        surf_list = []
        for si, configuration in raw_surf_list:
            if si.Type == "Plane":
                pid = self._resolve_plane_id(si)
                components[abs(pid)] = si
                surf_list.append(("Plane", None, pid, None, None, configuration))
                continue

            plane = si.Surf.Plane
            aplane = None
            if si.Type == "Cylinder":
                surf = si.Surf.Cylinder
            elif si.Type == "Cone":
                surf = si.Surf.Cone
                aplane = si.Surf.ApexPlane
            elif si.Type == "Sphere":
                surf = si.Surf.Sphere

            sid, exist = self.primitive_surfaces.add_surface(surf, True)
            components[abs(sid)] = surf
            pid = self._resolve_plane_id(plane) if plane is not None else None
            if plane is not None:
                components[abs(pid)] = plane
            apid = self._resolve_plane_id(aplane) if aplane is not None else None
            if aplane is not None:
                components[abs(apid)] = aplane
            surf_list.append((si.Type, sid, pid, apid, si.Orientation, configuration))

        region = can_region(cid, cylCan.Orientation, surf_list)
        validate_characteristic_sign(region.region, int(cid), cylCan.Orientation, "Can_region")
        return region, components

    def add_forwardCan(self, forwardCan):
        fwd_region, components = self.Can_region(forwardCan)

        add_can = True
        for kind in ("FwdCan", "RevCan"):
            if not add_can:
                break
            for cs_surf in self[kind]:
                # Both fwd_region and cs_surf.region were already validated
                # self-consistent at Can_region time, so a .reverse conflict
                # here is never a bug -- it's two distinct, adjacent Cans
                # legitimately sharing the same real surface with opposite
                # sense. Trust the structural match either way.
                boundary = fwd_region.isSameInterface(cs_surf.region, on_conflict="ignore")
                if abs(boundary) == 1:
                    add_can = False
                    break

        if add_can:
            self.surfaceNumber += 1
            newregion = fwd_region.copy(self.surfaceNumber)
            forwardCan.region = newregion
            forwardCan.components = components
            self["FwdCan"].append(forwardCan)
            self.__surfIndex__["FwdCan"].append(forwardCan.region.__int__())
        else:
            newregion = cs_surf.region if boundary > 0 else -cs_surf.region
        return newregion

    def add_reverseCan(self, reverseCan):
        rev_region, components = self.Can_region(reverseCan)

        add_can = True
        for kind in ("RevCan", "FwdCan"):
            if not add_can:
                break
            for cs_surf in self[kind]:
                # See add_forwardCan: both regions are already known
                # self-consistent, so a .reverse conflict here is never a
                # bug -- trust the structural match either way.
                boundary = rev_region.isSameInterface(cs_surf.region, on_conflict="ignore")
                if abs(boundary) == 1:
                    add_can = False
                    break

        if add_can:
            self.surfaceNumber += 1
            newregion = rev_region.copy(self.surfaceNumber)
            reverseCan.region = newregion
            reverseCan.components = components
            self["RevCan"].append(reverseCan)
            self.__surfIndex__["RevCan"].append(reverseCan.region.__int__())
        else:
            newregion = cs_surf.region if boundary > 0 else -cs_surf.region
        return newregion

    def TCone_region(self, TCone):
        kneCan = TCone.Surf.Cone
        cid, exist = self.primitive_surfaces.add_cone(kneCan.Surf.Cone)

        components = {abs(cid): kneCan.Surf.Cone}
        surf_list = []
        for pi, configuration in (
            (TCone.Surf.p1, TCone.Surf.p1_configuration),
            (TCone.Surf.p2, TCone.Surf.p2_configuration),
        ):
            pid = self._resolve_plane_id(pi)
            components[abs(pid)] = pi
            surf_list.append((pid, configuration))

        region = tcone_region(cid, TCone.Orientation, surf_list)
        validate_characteristic_sign(region.region, int(cid), TCone.Orientation, "TCone_region")
        return region, components

    def add_forwardTCone(self, forwardTCone):
        fwd_region, components = self.TCone_region(forwardTCone)

        add_kne = True
        for kind in ("FwdTCone", "RevTCone"):
            if not add_kne:
                break
            for cs_surf in self[kind]:
                # Both fwd_region and cs_surf.region were already validated
                # self-consistent at TCone_region time, so a .reverse
                # conflict here is never a bug -- it's two distinct,
                # adjacent TCones legitimately sharing the same real
                # surface with opposite sense (same reasoning as
                # Can_region).
                boundary = fwd_region.isSameInterface(cs_surf.region, on_conflict="ignore")
                if abs(boundary) == 1:
                    add_kne = False
                    break

        if add_kne:
            self.surfaceNumber += 1
            newregion = fwd_region.copy(self.surfaceNumber)
            forwardTCone.region = newregion
            forwardTCone.components = components
            self["FwdTCone"].append(forwardTCone)
            self.__surfIndex__["FwdTCone"].append(forwardTCone.region.__int__())
        else:
            newregion = cs_surf.region if boundary > 0 else -cs_surf.region
        return newregion

    def add_reverseTCone(self, reverseTCone):
        rev_region, components = self.TCone_region(reverseTCone)

        add_kne = True
        for kind in ("RevTCone", "FwdTCone"):
            if not add_kne:
                break
            for cs_surf in self[kind]:
                # Both rev_region and cs_surf.region were already validated
                # self-consistent at TCone_region time, so a .reverse
                # conflict here is never a bug -- same reasoning as
                # add_forwardTCone/Can_region.
                boundary = rev_region.isSameInterface(cs_surf.region, on_conflict="ignore")
                if abs(boundary) == 1:
                    add_kne = False
                    break

        if add_kne:
            self.surfaceNumber += 1
            newregion = rev_region.copy(self.surfaceNumber)
            reverseTCone.region = newregion
            reverseTCone.components = components
            self["RevTCone"].append(reverseTCone)
            self.__surfIndex__["RevTCone"].append(reverseTCone.region.__int__())
        else:
            newregion = cs_surf.region if boundary > 0 else -cs_surf.region
        return newregion

    def add_multiRoundCorner(self, mRoundC):
        resolved_planes = set()
        for plane in mRoundC.Surf.Planes:
            pid, exist = self.primitive_surfaces.add_plane(plane, True)
            if exist:
                p = self.get_primitive_surface(pid)
                if is_opposite(plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
                    # change plane axis because MultiRoundCorner shape is build with solid definition based on Surfaces dict reference
                    plane.Surf.Axis = -plane.Surf.Axis
                    plane.bVar = pid
            resolved_planes.add(id(plane))

        components = {abs(plane.bVar): plane for plane in mRoundC.Surf.Planes}

        for rc in mRoundC.Surf.Corners:
            cid, exist_c = self.primitive_surfaces.add_cylinder(rc.Surf.Cylinder.Surf.Cylinder, True)
            rc.Surf.Cylinder.Surf.Cylinder.bVar = cid
            components[abs(cid)] = rc.Surf.Cylinder.Surf.Cylinder

            if rc.Surf.Cylinder.Surf.Plane is not None:
                pcid, exist_p = self.primitive_surfaces.add_plane(rc.Surf.Cylinder.Surf.Plane, True)
                cylplane = rc.Surf.Cylinder.Surf.Plane
            else:
                cylplane = rc.Surf.Planes[0]
                if id(cylplane) in resolved_planes:
                    # cylplane is the same object already resolved in the
                    # Planes-loop above (its own Surf.Axis was already
                    # mutated to the correct, sign-corrected direction there
                    # -- re-registering it here would compare that
                    # already-corrected axis against the primitive a second
                    # time, trivially finding it "not opposite" and silently
                    # discarding the correction). Reuse the already-resolved
                    # bVar instead of re-deriving the sign.
                    pcid, exist_p = cylplane.bVar, False
                else:
                    pcid, exist_p = self.primitive_surfaces.add_plane(cylplane, True)

            if exist_p:
                p = self.get_primitive_surface(abs(pcid))
                if is_opposite(cylplane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pcid = -pcid
                    # change plane axis because MultiRoundCorner shape is build with solid definition based on Surfaces dict reference
                    cylplane.Surf.Axis = -cylplane.Surf.Axis
                    cylplane.bVar = pcid
            components[abs(pcid)] = cylplane

        multi_rc_region = multi_round_corner_region(mRoundC)

        add_mcorner = True
        for rc_surf in self["MultiRoundC"]:
            # multi_round_corner_region() is a trusted, independently
            # verified pure function (300/300 against real CAD ground
            # truth across 17 instances in 13 files, per this project's
            # own check_sign verification history) -- so a .reverse
            # conflict here, like Can_region/TCone_region's own
            # on_conflict="ignore" sites, is never a bug: it's two
            # distinct, adjacent MultiRoundCorners legitimately sharing
            # the same real surface with opposite sense. Confirmed
            # reproducible without this guard on
            # Solidos/Big_one_cell/modelCell_670000.stp (same class of
            # false positive as the tank.stp/Can_region case).
            boundary = multi_rc_region.isSameInterface(rc_surf.region, on_conflict="ignore")
            if abs(boundary) == 1:
                add_mcorner = False
                break

        if add_mcorner:
            self.surfaceNumber += 1
            newregion = multi_rc_region.copy(self.surfaceNumber)
            mRoundC.region = newregion
            mRoundC.components = components
            self["MultiRoundC"].append(mRoundC)
            self.__surfIndex__["MultiRoundC"].append(mRoundC.region.__int__())
        else:
            # boundary == -1 means multi_rc_region is the structural
            # complement of the already-registered rc_surf.region (two
            # distinct, adjacent MultiRoundCorners sharing one real surface
            # with opposite sense, e.g. comp_multiRC.step's two cells) --
            # reusing rc_surf.region un-negated here was a real bug: every
            # sibling add_* method (add_forwardCan/add_reverseCan/
            # add_forwardTCone/add_reverseTCone/add_roundCorner/
            # add_multiPlane) already negates on this exact condition.
            newregion = rc_surf.region if boundary > 0 else -rc_surf.region
        return newregion

    def get_roundCorner_region(self, roundC):
        config = roundC.Surf.Configuration
        cylinder = roundC.Surf.Cylinder
        planes = roundC.Surf.Planes

        cid, exist_c = self.primitive_surfaces.add_cylinder(cylinder.Surf.Cylinder, True)
        cylinder.Surf.Cylinder.bVar = cid
        if cylinder.Surf.Plane is not None:
            pcid, exist_p = self.primitive_surfaces.add_plane(cylinder.Surf.Plane, True)
            if exist_p:
                p = self.get_primitive_surface(pcid)
                if is_opposite(cylinder.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pcid = -pcid
                    # change plane axis because Round corner shape is build with solid definition based on Surfaces dict reference
                    cylinder.Surf.Plane.Surf.Axis = -cylinder.Surf.Plane.Surf.Axis
                    cylinder.Surf.Plane.bVar = pcid
        else:
            pcid = None

        p1, p2 = planes
        p1id, exist = self.primitive_surfaces.add_plane(p1, True)
        if exist:
            p = self.get_primitive_surface(p1id)
            if is_opposite(p1.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                p1id = -p1id
                # change plane axis because Round corner shape is build with solid definition based on Surfaces dict reference
                p1.Surf.Axis = -p1.Surf.Axis
                p1.bVar = p1id

        if p1 != p2:
            p2id, exist = self.primitive_surfaces.add_plane(p2, True)
            if exist:
                p = self.get_primitive_surface(p2id)
                if is_opposite(p2.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    p2id = -p2id
                    # change plane axis because Round corner shape is build with solid definition based on Surfaces dict reference
                    p2.Surf.Axis = -p2.Surf.Axis
                    p2.bVar = p2id
        else:
            p2id = p1id

        # Same order check_sign iterates components in (planes, then
        # cylinder, then cylinder's plane) -- kept so the incremental,
        # short-circuiting evaluate() there is unaffected by this change.
        components = {abs(p1id): p1}
        if p2id != p1id:
            components[abs(p2id)] = p2
        components[abs(cid)] = cylinder.Surf.Cylinder
        if cylinder.Surf.Plane is not None:
            components[abs(pcid)] = cylinder.Surf.Plane

        return round_corner_region(p1id, p2id, cid, pcid, config), components

    def add_roundCorner(self, roundC):
        roundC_region, components = self.get_roundCorner_region(roundC)

        add_corner = True
        for rc_surf in self["RoundC"]:
            # round_corner_region() is the same kind of trusted, pure,
            # independently verified region-builder as
            # multi_round_corner_region() (300/300 against real CAD ground
            # truth, per this project's check_sign verification history) --
            # same false-positive class as Can_region/TCone_region/
            # MultiRoundCorner: two distinct, adjacent RoundCorners can
            # legitimately share one real surface with opposite sense.
            boundary = roundC_region.isSameInterface(rc_surf.region, on_conflict="ignore")
            if abs(boundary) == 1:
                add_corner = False
                break

        if add_corner:
            self.surfaceNumber += 1
            newregion = roundC_region.copy(self.surfaceNumber)
            roundC.region = newregion
            roundC.components = components
            self["RoundC"].append(roundC)
            self.__surfIndex__["RoundC"].append(roundC.region.__int__())
        else:
            newregion = rc_surf.region if boundary > 0 else -rc_surf.region
        return newregion

    def _reversedCC_component(self, cc):
        """Resolve one RevCC chain segment (a Tier-2 "Cylinder"/"Cone"
        GeounedSurface) into (s_region, p_region): `s_region` is the
        segment's own surface term -- for a cone with an ApexPlane this is
        first built as OR[s, -ap] (the ApexPlane's normal negated) before
        going any further, matching a bare cylinder/cone otherwise; `p_region`
        is the segment's own additional plane, registered independently
        (never shared/summed across segments here -- each chain segment
        gets exactly one independent plane, per the RevCC definition).
        Also returns `components`, the numbering<->surface relation for
        every id this segment references -- same role as Can/TCone/
        RoundCorner's own `.components`, needed for `check_sign`."""
        components = {}
        if cc.Type == "Cylinder":
            sid, exist = self.primitive_surfaces.add_cylinder(cc.Surf.Cylinder, True)
            cc.Surf.Cylinder.bVar = sid
            s_region = BoolSurface(0, sid)
            components[abs(sid)] = cc.Surf.Cylinder
        else:
            cid, exist = self.primitive_surfaces.add_cone(cc.Surf.Cone)
            cc.Surf.Cone.bVar = cid
            s_region = BoolSurface(0, cid)
            components[abs(cid)] = cc.Surf.Cone
            if cc.Surf.ApexPlane:
                apid, exist = self.primitive_surfaces.add_plane(cc.Surf.ApexPlane, True)
                if exist:
                    p = self.get_primitive_surface(apid)
                    if is_opposite(cc.Surf.ApexPlane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                        apid = -apid
                s_region = s_region + (-BoolSurface(0, apid))
                components[abs(apid)] = cc.Surf.ApexPlane

        pid, exist = self.primitive_surfaces.add_plane(cc.Surf.Plane, True)
        if exist:
            p = self.get_primitive_surface(pid)
            if is_opposite(cc.Surf.Plane.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                pid = -pid
        p_region = BoolSurface(0, pid)
        components[abs(pid)] = cc.Surf.Plane
        return s_region, p_region, components

    def add_reversedCC(self, reversedCC):
        cylcones = reversedCC.Surf.CylCones
        components = {}

        if len(cylcones) == 1:
            s_region, p_region, comp = self._reversedCC_component(cylcones[0])
            components.update(comp)
            reversedCC_region = s_region * p_region
        else:
            plane_region = None

            surf_components = []
            for cc in cylcones:
                s_region, p_region, comp = self._reversedCC_component(cc)
                components.update(comp)
                plane_region = BoolSurface.add(plane_region, p_region)
                surf_components.append(s_region)

            surf_region = None
            for s_region in surf_components:
                surf_region = BoolSurface.mult(surf_region, s_region + (-plane_region))

            surf_region.region.simplify(None)
            reversedCC_region = plane_region * surf_region

        # A MultiPlane can make the irreducible solid non-convex -- exactly
        # the configuration where the RevCC's own additional plane, correct
        # only locally near its own cylinder/cone, must not act as an
        # unrestricted global cut. For each real MultiPlane component plane
        # found sitting on this RevCC's own boundary (AdjacentMultiplanePlanes,
        # at most one per chain end), OR it in -- direction opposite to its
        # own stored (material-pointing) axis -- so the RevCC's restriction
        # only applies on its own local side of that boundary, and stops
        # applying (reads True, non-restrictive) beyond it.
        for mpp in reversedCC.Surf.AdjacentMultiplanePlanes:
            pid, exist = self.primitive_surfaces.add_plane(mpp, True)
            if exist:
                p = self.get_primitive_surface(pid)
                if is_opposite(mpp.Surf.Axis, p.Surf.Axis, self.tolerances.pln_angle):
                    pid = -pid
            components[abs(pid)] = mpp
            reversedCC_region = reversedCC_region + (-BoolSurface(0, pid))

        add_cc = True
        for cs_surf in self["RevCC"]:
            boundary = reversedCC_region.isSameInterface(cs_surf.region)
            if abs(boundary) == 1:
                add_cc = False
                break

        if add_cc:
            self.surfaceNumber += 1
            newregion = reversedCC_region.copy(self.surfaceNumber)
            reversedCC.region = newregion
            reversedCC.components = components
            self["RevCC"].append(reversedCC)
            self.__surfIndex__["RevCC"].append(reversedCC.region.__int__())
        else:
            newregion = cs_surf.region if boundary > 0 else -cs_surf.region
        return newregion


class SurfacesDict(dict):
    def __init__(
        self,
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

    def get_sorted_surfaces(self):
        bsurf_list = []
        for surftype in self.__surfIndex__.values():
            for bsurf in surftype:
                bsurf_list.append((abs(bsurf.value()), bsurf))
        bsurf_list.sort()
        index, surfIndex = zip(*bsurf_list)
        return surfIndex

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

    def add_surface(self, surface, fuzzy=False):
        if surface.Type == "Plane":
            return self.add_plane(surface, fuzzy)
        elif surface.Type == "CylinderOnly":
            return self.add_cylinder(surface, fuzzy)
        elif surface.Type == "Cylinder":
            return self.add_cylinder(surface.Surf.Cylinder, fuzzy)
        elif surface.Type == "ConeOnly":
            return self.add_cone(surface)
        elif surface.Type == "Cone":
            return self.add_cone(surface.Surf.Cone)
        elif surface.Type == "SphereOnly":
            return self.add_sphere(surface)
        elif surface.Type == "Sphere":
            return self.add_sphere(surface.Surf.Sphere)
        elif surface.Type == "TorusOnly":
            return self.add_torus(surface)
        elif surface.Type == "Torus":
            return self.add_torus(surface.Surf.Torus)

    def add_plane(self, plane, fuzzy):
        ex = GVector(1, 0, 0)
        ey = GVector(0, 1, 0)
        ez = GVector(0, 0, 1)

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
                    stdtol=plane.Surf.real,
                ):
                    add_plane = False
                    bVar = p.bVar
                    plane.bVar = bVar
                    self.__last_obj__ = ("PX", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("PX", len(self["PX"]))
                self["PX"].append(plane)
                self.__surfIndex__["PX"].append(plane.bVar)

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
                    stdtol=plane.Surf.real,
                ):
                    add_plane = False
                    bVar = p.bVar
                    plane.bVar = bVar
                    self.__last_obj__ = ("PY", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("PY", len(self["PY"]))
                self["PY"].append(plane)
                self.__surfIndex__["PY"].append(plane.bVar)

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
                    stdtol=plane.Surf.real,
                ):
                    add_plane = False
                    bVar = p.bVar
                    plane.bVar = bVar
                    self.__last_obj__ = ("PZ", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("PZ", len(self["PZ"]))
                self["PZ"].append(plane)
                self.__surfIndex__["PZ"].append(plane.bVar)

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
                    stdtol=plane.Surf.real,
                ):
                    add_plane = False
                    bVar = p.bVar
                    plane.bVar = bVar
                    self.__last_obj__ = ("P", i)
                    break
            if add_plane:
                self.surfaceNumber += 1
                plane.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
                self.__last_obj__ = ("P", len(self["P"]))
                self["P"].append(plane)
                self.__surfIndex__["P"].append(plane.bVar)

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
                cyl.bVar = bVar
                self.__last_obj__ = ("Cyl", i)
                break

        if addCyl:
            self.surfaceNumber += 1
            cyl.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Cyl", len(self["Cyl"]))
            self["Cyl"].append(cyl)
            self.__surfIndex__["Cyl"].append(cyl.bVar)
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
                cone.bVar = bVar
                self.__last_obj__ = ("Cone", i)
                break
        if cone_added:
            self.surfaceNumber += 1
            cone.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Cone", len(self["Cone"]))
            self["Cone"].append(cone)
            self.__surfIndex__["Cone"].append(cone.bVar)
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
                sph.bVar = bVar
                self.__last_obj__ = ("Sph", i)
                break
        if sphere_added:
            self.surfaceNumber += 1
            sph.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Sph", len(self["Sph"]))
            self["Sph"].append(sph)
            self.__surfIndex__["Sph"].append(sph.bVar)
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
                tor.bVar = bVar
                self.__last_obj__ = ("Tor", i)
                break
        if add_torus:
            self.surfaceNumber += 1
            tor.bVar = BoolVariable(self.surfaceNumber + self.IndexOffset)
            self.__last_obj__ = ("Tor", len(self["Tor"]))
            self["Tor"].append(tor)
            self.__surfIndex__["Tor"].append(tor.bVar)
            return tor.bVar, False
        else:
            return bVar, True

    def get_id(self, facein):

        if facein.Type == "Plane":
            if is_parallel(facein.Surf.Axis, GVector(1, 0, 0), self.tolerances.pln_angle):
                p = "PX"
            elif is_parallel(facein.Surf.Axis, GVector(0, 1, 0), self.tolerances.pln_angle):
                p = "PY"
            elif is_parallel(facein.Surf.Axis, GVector(0, 0, 1), self.tolerances.pln_angle):
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
                    return s.bVar

        elif facein.Type == "Cylinder":
            for s in self["Cyl"]:
                if is_same_cylinder(
                    facein.Surf,
                    s.Surf,
                    options=self.options,
                    tolerances=self.tolerances,
                    numeric_format=self.numeric_format,
                ):
                    return s.bVar

        elif facein.Type == "Cone":
            for s in self["Cone"]:
                if is_same_cone(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.kne_distance,
                    atol=self.tolerances.kne_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.bVar

        elif facein.Type == "Sphere":
            for s in self["Sph"]:
                if is_same_sphere(facein.Surf, s.Surf, self.tolerances.sph_distance, rel_tol=self.tolerances.relativeTol):
                    return s.bVar

        elif facein.Type == "Torus":
            for s in self["Tor"]:
                if is_same_torus(
                    facein.Surf,
                    s.Surf,
                    dtol=self.tolerances.tor_distance,
                    atol=self.tolerances.tor_angle,
                    rel_tol=self.tolerances.relativeTol,
                ):
                    return s.bVar

        return 0
