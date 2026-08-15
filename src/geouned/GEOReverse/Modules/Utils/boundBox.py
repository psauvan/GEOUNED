import math
import numpy
import typing

from .._geo_bridge import GVector, GBoundBox, GPlane, GLine
from .booleanFunction import BoolSequence

twoPi = math.pi * 2


class BoxSettings:
    """Parameters used in the solids boundbox generation. Optimized dimensions can reduce
    the translation time.

    Args:
        universe_radius (float, optional): Maximum radius of the CAD universe.
            Solids with coordinates x^2+y^2+z*2 > universe_radius^2 will be cut or not represented.
            Units mm. Defaults to 1.0e6.
        insolid_tolerance (float, optional): Maximum distance from the nearest
            surface of the solid, for which a point outside the solid is assumed
            inside the solid. Used only for boundbox generation. Units mm.
            Defaults to 1.
        box_dimensions (None,tuple,list, optional): dimensions of the universe box in which solids
            will be converted to CAD. Dimensions are (Xmin, Ymin, Zmin, Xmax, Ymax, Zmax) of the box.
            If no box dimensions is provided, the universe dimension is given by the universe_radius parameter.
            Defaul to None.
    """

    def __init__(
        self,
        universe_radius: float = 1.0e6,  # units mm
        insolid_tolerance: float = 1,  # units mm
        box_dimensions: typing.Union[None, list, tuple] = None,
    ):

        self.universe_radius = universe_radius
        self.insolid_tolerance = insolid_tolerance
        self.box_dimensions = box_dimensions
        self.set_universe_box()

    @property
    def universe_radius(self):
        return self._universe_radius

    @universe_radius.setter
    def universe_radius(self, universe_radius: float):
        if not isinstance(universe_radius, (float, int)):
            raise TypeError(f"geoReverse.Settings.universe_radius should be a float, not a {type(universe_radius)}")
        self._universe_radius = universe_radius

    @property
    def insolid_tolerance(self):
        return self._insolid_tolerance

    @insolid_tolerance.setter
    def insolid_tolerance(self, insolid_tolerance: float):
        if not isinstance(insolid_tolerance, (float, int)):
            raise TypeError(f"geoReverse.Settings.insolid_tolerance should be a float, not a {type(insolid_tolerance)}")
        self._insolid_tolerance = insolid_tolerance

    @property
    def box_dimensions(self):
        return self._box_dimensions

    @box_dimensions.setter
    def box_dimensions(self, box_dimensions: typing.Union[None, list, tuple]):
        if box_dimensions is None:
            self._box_dimensions = None
        else:
            if not isinstance(box_dimensions, (list, tuple)):
                raise TypeError(f"geoReverse.Settings.box_dimensions should be a list or tuple, not a {type(box_dimensions)}")
            for x in box_dimensions:
                if not isinstance(x, (float, int)):
                    raise TypeError(f"geoReverse.Settings.box_dimensions elements should be floats, not a {type(x)}")

            for i in range(3):
                vmin, vmax = box_dimensions[i], box_dimensions[i + 3]
                if vmin >= vmax:
                    raise TypeError(
                        f"geoReverse.Settings.box_dimensions bad box limits. Limits should be (Xmin, Ymin, Zmin, Xmax, Ymax, Zmax)."
                    )

            self._box_dimensions = box_dimensions

    @property
    def universe_box(self):
        return self._universe_box

    def set_universe_box(self):
        if self.box_dimensions is None:
            self._universe_box = myBox(
                GBoundBox(
                    -self.universe_radius,
                    -self.universe_radius,
                    -self.universe_radius,
                    self.universe_radius,
                    self.universe_radius,
                    self.universe_radius,
                ),
                "Forward",
            )
        else:
            self._universe_box = myBox(GBoundBox(*self.box_dimensions), "Forward")
            radius = max(map(abs, self.box_dimensions))
            self.universe_radius = radius


class myBox:
    def __init__(self, boundBox=None, orientation=None):

        if type(boundBox) is myBox:
            self.Box = boundBox.Box
            self.Orientation = boundBox.Orientation
        else:
            if boundBox is not None:
                if boundBox.XLength <= 1e-12:
                    self.Box = None
                elif boundBox.YLength <= 1e-12:
                    self.Box = None
                elif boundBox.ZLength <= 1e-12:
                    self.Box = None
                else:
                    self.Box = boundBox
            else:
                self.Box = None
            self.Orientation = orientation
        if self.Orientation is None:
            raise TypeError("myBox orientation cannot by None type")

    def add(self, box):
        """Non-mutating GBoundBox equivalent of FreeCAD.BoundBox's own
        in-place `.add()` -- reassigns `self.Box` to the union instead."""
        if self.Box is None:
            if self.Orientation == "Forward":
                self.Box = box.Box
                self.Orientation = box.Orientation
        elif box.Box is None:
            if box.Orientation == "Reversed":
                self.Box = None
                self.Orientation = "Reversed"
        else:
            self.Box = self.Box.union(box.Box)
            if self.Orientation != box.Orientation:
                self.Orientation = "Reversed"

    def mult(self, box):
        """Non-mutating GBoundBox equivalent of the original's in-place `.add()` in the AND branch."""
        if self.Orientation is None:
            self.Box = box.Box
            self.Orientation = box.Orientation
        elif self.Box is None:
            if self.Orientation == "Reversed":
                self.Box = box.Box
                self.Orientation = box.Orientation
        elif box.Box is None:
            if box.Orientation == "Forward":
                self.Box = None
                self.Orientation = "Forward"
        else:
            if self.Orientation == "Reversed" or box.Orientation == "Reversed":
                self.Box = self.Box.union(box.Box)
            else:
                inter = self.Box.intersected(box.Box)
                if inter.is_valid():
                    self.Box = inter
                else:
                    self.Box = None
            if self.Orientation != box.Orientation:
                self.Orientation = "Forward"

    def sameBox(self, box):
        if self.Box is None or box.Box is None:
            if self.Box is None and box.Box is None:
                return self.Orientation == box.Orientation
            else:
                return False

        for i in range(6):
            p1 = self.Box.get_point(i)
            p2 = box.Box.get_point(i)
            if (p1 - p2).length > 1e-6:
                return False
        return True


class solid_plane_box:
    def __init__(self, NTCell=None, outbox=None):
        if NTCell is None:
            settings = BoxSettings()
            self.planes = None
            self.definition = None
            self.surfaces = None
            self.surf_to_plane = None
            self.insolid_tolerance = settings.insolid_tolerance
            self.universe_box = settings.universe_radius
            self.orientation = None
        else:
            test_orientation = "Forward"
            self.insolid_tolerance = NTCell.settings.insolid_tolerance
            self.universe_box = NTCell.settings.universe_box
            self.surfaces = NTCell.surfaces
            plane_dict, surf_to_plane_dict = quadric_to_plane(NTCell.definition, NTCell.surfaces, test_orientation)
            self.planes = plane_dict
            self.surf_to_plane = surf_to_plane_dict
            self.definition = plane_definition(NTCell.definition.copy(), surf_to_plane_dict, test_orientation)
            self.orientation = self.get_box_orientation()

            if test_orientation != self.orientation:
                plane_dict, surf_to_plane_dict = quadric_to_plane(NTCell.definition, NTCell.surfaces, self.orientation)
                self.planes = plane_dict
                self.surf_to_plane = surf_to_plane_dict
                self.definition = plane_definition(NTCell.definition.copy(), surf_to_plane_dict, self.orientation)

        if outbox:
            self.outBox = outbox
        else:
            self.outBox = self.universe_box

    def export_surf_planes(self, box):
        """Debug helper -- exports each surface's approximating planes as a
        STEP file. Not currently ported (needs geo's Gmake_polygon_face /
        Gexport_step, plus a working makePlane call site below) -- left
        as a stub raising clearly rather than silently doing nothing,
        since nothing in CsgToCad's own pipeline calls this (grep-confirmed
        zero callers outside this file's own definition)."""
        raise NotImplementedError("solid_plane_box.export_surf_planes: debug-only STEP export, not migrated (no callers)")

    def isInside(self, point, boundary_inside=True):
        surf_value = dict()
        for p_index, p in self.planes.items():
            normal, pointPlane = p.Axis, p.Position
            r = point - pointPlane
            dot = normal.dot(r)
            if abs(dot) < self.insolid_tolerance:
                surf_value[p_index] = None  # undefined value for point close to the surface
            else:
                surf_value[p_index] = dot > 0
        inside = self.definition.evaluate(surf_value)

        # if point close to the surface assume inside the solid independently if inside or outside
        # inside if point close to boundary
        forward_inside = boundary_inside if inside is None else inside
        return forward_inside if boundary_inside else not forward_inside

    def copy(self, newdefinition=None, rebuild=False):
        cpsol = solid_plane_box()
        cpsol.insolid_tolerance = self.insolid_tolerance
        cpsol.universe_box = self.universe_box
        cpsol.outBox = self.outBox
        cpsol.orientation = self.orientation

        if newdefinition is not None:
            if rebuild:
                plane_dict, surf_to_plane_dict = quadric_to_plane(newdefinition, self.surfaces, self.orientation)
                cpsol.planes = plane_dict
                cpsol.surf_to_plane = surf_to_plane_dict
                cpsol.definition = plane_definition(newdefinition.copy(), surf_to_plane_dict, self.orientation)
            else:
                cpsol.surf_to_plane = self.surf_to_plane
                cpsol.definition = newdefinition.copy()
                cpsol.planes = dict()
                for p in newdefinition.get_surfaces_numbers():
                    if p in self.planes.keys():
                        cpsol.planes[p] = self.planes[p]
        else:
            cpsol.surf_to_plane = self.surf_to_plane
            cpsol.definition = self.definition.copy()
            cpsol.planes = self.planes
        return cpsol

    def get_boundBox(self, enlarge=0):
        mBox = self.build_box_depth()
        bBox = mBox.Box
        if bBox is not None and enlarge > 0:
            dx = (bBox.XMax - bBox.XMin) * enlarge
            dy = (bBox.YMax - bBox.YMin) * enlarge
            dz = (bBox.ZMax - bBox.ZMin) * enlarge
            bBox = GBoundBox(bBox.XMin - dx, bBox.YMin - dy, bBox.ZMin - dz, bBox.XMax + dx, bBox.YMax + dy, bBox.ZMax + dz)
            mBox.Box = bBox
        return mBox

    def build_box_depth(self):

        if self.definition.level == 0:
            return self.get_component_boundBox()
        else:
            box_list = []
            if type(self.definition.elements) is bool:
                return myBox(None, "Reversed") if self.definition.elements else myBox(None, "Forward")
            for c in self.definition.elements:
                cbox = self.copy(c)
                box = cbox.build_box_depth()
                box_list.append(box)

            fullBox = myBox(box_list[0])

            if self.definition.operator == "AND":
                for box in box_list[1:]:
                    fullBox.mult(box)
                    if fullBox.Box is None and fullBox.Orientation == "Forward":
                        break
            else:
                for box in box_list[1:]:
                    fullBox.add(box)
                    if fullBox.Box is None and fullBox.Orientation == "Reversed":
                        break

            return fullBox

    def get_component_boundBox(self, cutBoundary=False):
        axis_list = ("x", "y", "z")

        orientation = self.get_box_orientation()
        if not cutBoundary:
            if orientation == "Undefined":
                cutBoundary = True
                orientation = "Forward"
        else:
            if orientation == "Undefined":
                orientation = "Forward"

        planes_inter = tuple(self.planes[x] for x in self.definition.get_surfaces_numbers())
        point_list = plane_intersect(planes_inter, self.outBox.Box, cutBoundary)
        box_lim = []
        if len(point_list) < 6:
            if not cutBoundary:
                return self.get_component_boundBox(True)
            else:
                return myBox(None, orientation)
        else:
            for axis in axis_list:
                s_point = sort_point(point_list, axis)
                if s_point == []:
                    return None
                for point in s_point:
                    if self.isInside(point, orientation == "Forward"):
                        box_lim.append(pointaxis(point, axis))
                        break

                s_point = remove_points(s_point, pointaxis(point, axis), axis, True)
                if s_point == []:
                    return None
                for point in s_point:
                    if self.isInside(point, orientation == "Forward"):
                        box_lim.append(pointaxis(point, axis))
                        break
                point_list = remove_points(s_point, pointaxis(point, axis), axis, False)

            if len(box_lim) < 6:
                if cutBoundary:
                    return myBox(None, orientation)
                else:
                    return self.get_component_boundBox(True)
            else:
                box = GBoundBox(box_lim[0], box_lim[2], box_lim[4], box_lim[1], box_lim[3], box_lim[5])
                if box.XLength < 1e-12 or box.YLength < 1e-12 or box.ZLength < 1e-12:
                    if cutBoundary:
                        return myBox(None, orientation)
                    else:
                        return self.get_component_boundBox(True)
                else:
                    return myBox(box, orientation)

    def get_box_orientation(self):
        ninside = 0
        universeBox = self.universe_box.Box
        for i in range(8):
            p = universeBox.get_point(i)
            if self.isInside(p, True):
                ninside += 1
        if ninside == 8:
            return "Reversed"
        elif ninside == 0:
            return "Forward"
        else:
            return "Undefined"


def quadric_to_plane(cellDef, surfaces, orientation):

    surf_planes_dict = dict()
    planes = dict()

    surf_index = cellDef.signedSurfaces()
    next = list({abs(s) for s in surf_index})
    next.sort()
    next_index = next[-1] + 1
    apex = []

    if orientation == "Reversed":
        fwd = False
    elif orientation == "Forward":
        fwd = True
    else:
        fwd = None

    for s_index in surf_index:
        s_label = abs(s_index)
        s = surfaces[s_label]
        if s.type == "plane":
            normal, d = s.params
            position = normal * d
            planes[s_label] = GPlane.from_values(position, normal)
        else:
            if fwd is None:
                pos = None
            else:
                pos = (s_index > 0) == fwd
            surf_planes = convert_to_planes(s, pos)
            if s.type == "cone":
                apex.append(s.params[0])
                dbl = s.params[3]
                p_index = []
                for p in surf_planes:
                    planes[next_index] = p
                    p_index.append(next_index)
                    next_index += 1

                if dbl:
                    surf_planes_dict[s_label] = ("dblcone", p_index)
                else:
                    surf_planes_dict[s_label] = ("cone", p_index)

            elif s.type == "torus":
                extplanes, inplanes = surf_planes
                p_ext = []
                p_in = []
                for p in extplanes:
                    planes[next_index] = p
                    p_ext.append(next_index)
                    next_index += 1
                for p in inplanes:
                    planes[next_index] = p
                    p_in.append(next_index)
                    next_index += 1
                surf_planes_dict[s_label] = ("torus", p_ext, p_in)
            else:
                p_index = []
                for p in surf_planes:
                    planes[next_index] = p
                    p_index.append(next_index)
                    next_index += 1
                surf_planes_dict[s_label] = p_index
    return planes, surf_planes_dict


def convert_to_planes(s, pos):
    if s.type == "cylinder":
        return cylinder_to_planes(s, pos)
    elif s.type == "cone":
        return cone_to_planes(s, pos)
    elif s.type == "sphere":
        return sphere_to_planes(s, pos)
    elif s.type == "torus":
        return torus_to_planes(s, pos)
    elif s.type == "paraboloid":
        return parabola_to_planes(s, pos)
    elif s.type == "box":
        return box_to_planes(s)
    else:
        print(f"{s.type} not implemented for boundbox")
        return []


def get_orto_axis(axis):
    x = GVector(1, 0, 0)
    z = GVector(0, 0, 1)
    vx = axis.cross(x)
    vz = axis.cross(z)
    if vx.length < vz.length:
        v = vz
    else:
        v = vx
    v = v.normalized()
    w = v.cross(axis)
    w = w.normalized()

    return v, w


def cylinder_to_planes(cyl, pos):
    center, axis, radius = cyl.params
    if pos is None:
        radius = radius * 0.8535533906
    elif pos:
        radius = radius * 0.70710678

    x, y = get_orto_axis(axis)
    r1 = center + x * radius
    r2 = center - x * radius
    r3 = center + y * radius
    r4 = center - y * radius

    p1 = GPlane.from_values(r1, -x)
    p2 = GPlane.from_values(r2, x)
    p3 = GPlane.from_values(r3, -y)
    p4 = GPlane.from_values(r4, y)
    return (p1, p2, p3, p4)


def cone_to_planes(cone, pos):
    apex, axis, t, dbl = cone.params
    if pos is None:
        t = t * 0.8535533906
    elif pos:
        t = t * 0.70710678
    sa = math.atan(t)
    nface = 4
    x, y = get_orto_axis(axis)
    cs = math.cos(sa)
    ss = math.sin(sa)
    dphi = twoPi / nface
    phi = 0
    cplanes = []
    for i in range(nface):
        rho = x * math.cos(phi) + y * math.sin(phi)
        ni = -axis * ss + rho * cs
        pi = GPlane.from_values(apex, -ni)
        cplanes.append(pi)
        phi += dphi

    pa = GPlane.from_values(apex, axis)
    cplanes.append(pa)

    return cplanes


def sphere_to_planes(sphere, pos):
    center, radius = sphere.params
    if pos is None:
        radius = radius * 0.8535533906
    elif pos:
        radius = radius * 0.70710678

    x = GVector(1, 0, 0)
    y = GVector(0, 1, 0)
    z = GVector(0, 0, 1)

    r1 = center + x * radius
    r2 = center - x * radius
    r3 = center + y * radius
    r4 = center - y * radius
    r5 = center + z * radius
    r6 = center - z * radius

    p1 = GPlane.from_values(r1, -x)
    p2 = GPlane.from_values(r2, x)
    p3 = GPlane.from_values(r3, -y)
    p4 = GPlane.from_values(r4, y)
    p5 = GPlane.from_values(r5, -z)
    p6 = GPlane.from_values(r6, z)
    return (p1, p2, p3, p4, p5, p6)


def torus_to_planes(torus, pos):
    center, axis, majorRadius, minorR, minorA = torus.params

    x = GVector(1, 0, 0)
    y = GVector(0, 1, 0)
    z = GVector(0, 0, 1)

    if pos is None:
        dist = (majorRadius + minorR) * 0.8535533906
        difR = (majorRadius - minorR) * 0.8535533906
    elif pos:
        dist = (majorRadius + minorR) * 0.70710678
        difR = majorRadius - minorR
    else:
        dist = majorRadius + minorR
        difR = (majorRadius - minorR) * 0.70710678

    if abs(abs(axis.dot(x)) - 1) < 1e-5:
        r1 = center + x * minorA
        r2 = center - x * minorA
        r3 = center + y * dist
        r4 = center - y * dist
        r5 = center + z * dist
        r6 = center - z * dist
        if difR > 0:
            r7 = center + y * difR
            r8 = center - y * difR
            r9 = center + z * difR
            r10 = center - z * difR
            p7 = GPlane.from_values(r7, y)
            p8 = GPlane.from_values(r8, -y)
            p9 = GPlane.from_values(r9, z)
            p10 = GPlane.from_values(r10, -z)
    elif abs(abs(axis.dot(y)) - 1) < 1e-5:
        r1 = center + x * dist
        r2 = center - x * dist
        r3 = center + y * minorA
        r4 = center - y * minorA
        r5 = center + z * dist
        r6 = center - z * dist
        if difR > 0:
            r7 = center + x * difR
            r8 = center - x * difR
            r9 = center + z * difR
            r10 = center - z * difR
            p7 = GPlane.from_values(r7, x)
            p8 = GPlane.from_values(r8, -x)
            p9 = GPlane.from_values(r9, z)
            p10 = GPlane.from_values(r10, -z)
    elif abs(abs(axis.dot(z)) - 1) < 1e-5:
        r1 = center + x * dist
        r2 = center - x * dist
        r3 = center + y * dist
        r4 = center - y * dist
        r5 = center + z * minorA
        r6 = center - z * minorA
        if difR > 0:
            r7 = center + x * difR
            r8 = center - x * difR
            r9 = center + y * difR
            r10 = center - y * difR
            p7 = GPlane.from_values(r7, x)
            p8 = GPlane.from_values(r8, -x)
            p9 = GPlane.from_values(r9, y)
            p10 = GPlane.from_values(r10, -y)

    p1 = GPlane.from_values(r1, -x)
    p2 = GPlane.from_values(r2, x)
    p3 = GPlane.from_values(r3, -y)
    p4 = GPlane.from_values(r4, y)
    p5 = GPlane.from_values(r5, -z)
    p6 = GPlane.from_values(r6, z)
    external_planes = (p1, p2, p3, p4, p5, p6)
    if difR > 0:
        central_planes = (p7, p8, p9, p10)
    else:
        central_planes = tuple()
    return (external_planes, central_planes)


def box_to_planes(box):

    org, vec1, vec2, vec3 = box.params[:]
    p1 = GPlane.from_values(org, vec1)
    p2 = GPlane.from_values(org, vec2)
    p3 = GPlane.from_values(org, vec3)
    p4 = GPlane.from_values(org + vec1, -vec1)
    p5 = GPlane.from_values(org + vec2, -vec2)
    p6 = GPlane.from_values(org + vec3, -vec3)

    return (p1, p2, p3, p4, p5, p6)


def parabola_to_planes(parabola, pos):
    # parabola approximated by plane tanget to the curve
    # plane separation such that distance from plane to curve < a*x0 (a parameter < 1, x0 absica of tangent point )
    # the sequence of tangent points is xn+1 = xn * (1 - sqrt(2*a))
    # initial point x0 is calculated suche that after n iteration the last y = xn^2 / (4*focal) is < rmax, where rmax is the maximin universe distance
    # x0 = b^n * sqrt(4*focal*rmax)  whith b =  (1 - sqrt(2*a); n number of tangent planes to consider
    #

    center, axis, focal = parabola.params
    nt = 7
    a = 0.22
    rmax = 7.5e5
    b = 1 - math.sqrt(2 * a)
    x0 = b**nt * math.sqrt(4 * focal * rmax)

    axis = axis.normalized()
    x, y = get_orto_axis(axis)
    dphi = twoPi / 4
    phi = 0

    p0 = GPlane.from_values(center, axis)
    cplanes = [p0]
    focal = float(focal)
    xp = x0
    for n in range(nt + 1):
        zi = 0.25 * xp * xp / focal
        for i in range(4):
            rho = x * math.cos(phi) + y * math.sin(phi)
            vec = y * math.cos(phi) - x * math.sin(phi)
            slope = 2 * focal * rho + xp * axis  # slope xp/(2*focal)
            xe = center + xp * rho + zi * axis
            normal = vec.cross(slope)
            normal = normal.normalized()
            pi = GPlane.from_values(xe, normal)
            cplanes.append(pi)
            phi += dphi
        xp = xp / b

    return cplanes


def plane_definition(seq, surf_index, orientation):
    for s, planes in surf_index.items():
        if len(planes) == 0:
            continue
        addpos = False
        if type(planes[0]) is str:
            if planes[0] == "torus":
                extplanes, inplanes = planes[1:3]
                extm = BoolSequence(" ".join((str(p) for p in extplanes)))
                if len(inplanes) > 0:
                    inm = BoolSequence(":".join((str(p) for p in inplanes)))
                    pm = BoolSequence(operator="AND")
                    pm.append(extm, inm)
                else:
                    pm = extm
            elif planes[0] == "dblcone":
                addpos = True
                cplanes = planes[1]
                cone1 = BoolSequence(" ".join((str(p) for p in cplanes)))
                cone2 = BoolSequence(" ".join((str(-p) for p in cplanes)))
                pm = BoolSequence(operator="OR")
                pm.append(cone1, cone2)
            else:
                cplanes = planes[1]
                addpos = True
                pm = BoolSequence(" ".join((str(p) for p in cplanes)))
        else:
            pm = BoolSequence(" ".join((str(p) for p in planes)))
        if type(seq.elements) is bool:
            return seq
        pp = pm.get_complementary()
        if orientation == "Forward":
            change_surf(seq, -s, pm)
            if addpos:
                change_surf(seq, s, pp)
            else:
                change_surf(seq, s, True)
        elif orientation == "Reversed":
            change_surf(seq, s, pp)
            if addpos:
                change_surf(seq, -s, pm)
            else:
                change_surf(seq, -s, False)
        else:
            change_surf(seq, s, pp)
            change_surf(seq, -s, pm)

    seq.join_operators()
    return seq


def change_surf(seq, old, new):
    clean = False
    for i, e in enumerate(seq.elements):
        if type(e) is BoolSequence:
            if abs(old) in e.get_surfaces_numbers():
                change_surf(e, old, new)
                if type(e.elements) is bool:
                    if e.elements == (seq.operator == "OR"):
                        seq.elements = e.elements
                        return
                    else:
                        clean = True
        else:
            if e == old:
                if type(new) is bool:
                    if new == (seq.operator == "OR"):
                        seq.elements = new
                        return
                    else:
                        seq.elements[i] = new
                        clean = True
                else:
                    seq.elements[i] = new
    if clean:
        seq.clean()


def plane_intersect(plane_list, externalBox, cutBoundary):
    """
    `externalBox` is a `GBoundBox`. Real plane-plane and line-plane
    intersections now go through `geo`'s own `GPlane.intersect_plane`/
    `GPlane.intersect_line` (pure GVector math, verified against native
    FreeCAD intersection results when those methods were added) instead
    of native `Part.Plane.intersect()`/`Part.Line.intersect()`.
    """
    point_list = []
    if not cutBoundary:
        for i, p1 in enumerate(plane_list[0:-2]):
            j = i + 1
            for p2 in plane_list[i + 1 : -1]:
                line = p1.intersect_plane(p2)
                if line is None:
                    continue
                for p3 in plane_list[j + 1 :]:
                    p = p3.intersect_line(line)
                    if p is None:
                        continue
                    if externalBox.contains_point(p):
                        point_list.append(p)
                j += 1
    else:
        XYZ = (
            GVector(1, 0, 0),
            GVector(0, 1, 0),
            GVector(0, 0, 1),
        )
        pxm = GPlane.from_values(GVector(externalBox.XMin, 0, 0), XYZ[0])
        pxp = GPlane.from_values(GVector(externalBox.XMax, 0, 0), XYZ[0])
        pym = GPlane.from_values(GVector(0, externalBox.YMin, 0), XYZ[1])
        pyp = GPlane.from_values(GVector(0, externalBox.YMax, 0), XYZ[1])
        pzm = GPlane.from_values(GVector(0, 0, externalBox.ZMin), XYZ[2])
        pzp = GPlane.from_values(GVector(0, 0, externalBox.ZMax), XYZ[2])
        PXYZ = (pxm, pxp, pym, pyp, pzm, pzp)

        for i, p1 in enumerate(plane_list[0:]):
            j = i + 1
            point_list.extend(plane_boundary(p1, externalBox))
            for p2 in plane_list[i + 1 :]:
                line = p1.intersect_plane(p2)
                if line is None:
                    continue
                point_list.extend(line_boundary(line, externalBox, PXYZ))
                for p3 in plane_list[j + 1 :]:
                    p = p3.intersect_line(line)
                    if p is None:
                        continue
                    if externalBox.contains_point(p):
                        point_list.append(p)

        for i in range(8):
            p = externalBox.get_point(i)
            point_list.append(p)

    return point_list


def plane_boundary(plane, externalBox):
    """
    Points where `plane` crosses each of `externalBox`'s 12 edges.
    `GPlane.intersect_line` gives the crossing point of the edge's own
    *infinite* line with the plane; `t` (the fraction along the edge
    segment) is checked explicitly here to keep it bounded to the real
    segment, replacing the original's `Part.LineSegment(...).intersect(plane)`
    (a bounded native segment-vs-plane query) with the same effect.
    """
    point_list = []
    for i in range(12):
        v0, v1 = externalBox.get_edge(i)
        edge_dir = v1 - v0
        line = GLine.from_values(v0, edge_dir)
        p = plane.intersect_line(line)
        if p is None:
            continue
        # recover t along the segment from the returned point (edge_dir may not be unit length)
        denom = edge_dir.dot(edge_dir)
        if denom < 1e-20:
            continue
        t = (p - v0).dot(edge_dir) / denom
        if 0.0 <= t <= 1.0:
            point_list.append(p)
    return point_list


def line_boundary(line, externalBox, PXYZ):
    points = []
    for i, plane in enumerate(PXYZ):
        p = plane.intersect_line(line)
        if p is None:
            continue
        if i < 2:
            if (externalBox.YMin <= p.y <= externalBox.YMax) and (externalBox.ZMin <= p.z <= externalBox.ZMax):
                points.append(p)
        elif i < 4:
            if (externalBox.XMin <= p.x <= externalBox.XMax) and (externalBox.ZMin <= p.z <= externalBox.ZMax):
                points.append(p)
        else:
            if (externalBox.XMin <= p.x <= externalBox.XMax) and (externalBox.YMin <= p.y <= externalBox.YMax):
                points.append(p)
        if len(points) == 2:
            return points
    return points


def sort_point(point_list, axis):
    axis_points = []
    if axis == "x":
        for i, point in enumerate(point_list):
            axis_points.append((point.x, i))
    elif axis == "y":
        for i, point in enumerate(point_list):
            axis_points.append((point.y, i))
    elif axis == "z":
        for i, point in enumerate(point_list):
            axis_points.append((point.z, i))
    else:
        print("bad axis name")

    axis_points.sort()
    sorted_points = list(point_list[x[1]] for x in axis_points)
    removed = remove_close_points(list(sorted_points))
    return removed


def remove_close_points(sorted_list):
    if len(sorted_list) < 2:
        return sorted_list
    new_points = []
    p = sorted_list.pop()
    new_points.append(p)
    while len(sorted_list) > 0:
        nextp = sorted_list.pop()
        dp = p - nextp
        while dp.length < 0.1:
            if len(sorted_list) > 0:
                nextp = sorted_list.pop()
                dp = p - nextp
            else:
                break
        else:
            p = nextp
            new_points.append(p)
    new_points.reverse()
    return new_points


def remove_points(point_list, value, axis, lower, Lmax=None):
    kept_points = []
    if axis == "x":
        if lower:
            for p in point_list[::-1]:
                if p.x < value:
                    break
                kept_points.append(p)
        else:
            for p in point_list[::-1]:
                if p.x > value:
                    break
                kept_points.append(p)
    elif axis == "y":
        if lower:
            for p in point_list[::-1]:
                if p.y < value:
                    break
                kept_points.append(p)
        else:
            for p in point_list[::-1]:
                if p.y > value:
                    break
                kept_points.append(p)
    elif axis == "z":
        if lower:
            for p in point_list[::-1]:
                if p.z < value:
                    break
                kept_points.append(p)
        else:
            for p in point_list[::-1]:
                if p.z > value:
                    break
                kept_points.append(p)
    else:
        print("bad axis name")
    return kept_points


def pointaxis(p, axis):
    if axis == "x":
        return p.x
    elif axis == "y":
        return p.y
    elif axis == "z":
        return p.z


def makePlane(normal, position, Box):
    """
    A single planar `GFace` bounded by `Box`'s own extent, or `None` if
    the (infinite) plane doesn't cross `Box` at all. Ported from the
    original's edge-by-edge parametric scan (unchanged algorithm) --
    `GVector` throughout instead of `FreeCAD.Vector`, `Gmake_polygon_face`
    (already exists in `geo`) instead of the original's own
    `Part.Face(Part.makePolygon(...))` call.
    """
    from .._geo_bridge import Gmake_polygon_face

    p0 = normal.dot(position)

    pointEdge = []
    for i in range(12):
        edge = Box.get_edge(i)
        p1 = normal.dot(edge[0])
        p2 = normal.dot(edge[1])
        d0 = p0 - p1
        d1 = p2 - p1
        if d1 != 0:
            a = d0 / d1
            if a >= 0 and a <= 1:
                pointEdge.append(edge[0] + a * (edge[1] - edge[0]))

    if len(pointEdge) == 0:
        return None  # Plane does not cross box

    s = GVector(0, 0, 0)
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
        phi = numpy.arctan2(v.dot(Y0), v.dot(X0))
        orden.append((phi, i))
    orden.sort()

    return Gmake_polygon_face([pointEdge[p[1]] for p in orden])


def inertia_matrix(points):
    npoints = len(points)
    numpy_points = numpy.ndarray((npoints, 3))
    for i, p in enumerate(points):
        numpy_points[i] = numpy.array((p.x, p.y, p.z))

    x0 = numpy.sum(numpy_points[:, 0]) / npoints
    y0 = numpy.sum(numpy_points[:, 1]) / npoints
    z0 = numpy.sum(numpy_points[:, 2]) / npoints
    Sxx = numpy.sum(numpy_points[:, 0] * numpy_points[:, 0])
    Syy = numpy.sum(numpy_points[:, 1] * numpy_points[:, 1])
    Szz = numpy.sum(numpy_points[:, 2] * numpy_points[:, 2])
    Sxy = numpy.sum(numpy_points[:, 0] * numpy_points[:, 1])
    Sxz = numpy.sum(numpy_points[:, 0] * numpy_points[:, 2])
    Syz = numpy.sum(numpy_points[:, 1] * numpy_points[:, 2])

    Ixx = Sxx / npoints - x0 * x0
    Iyy = Syy / npoints - y0 * y0
    Izz = Szz / npoints - z0 * z0
    Ixy = Sxy / npoints - x0 * y0
    Ixz = Sxz / npoints - x0 * z0
    Iyz = Syz / npoints - y0 * z0

    imatrix = numpy.array(((Ixx, Ixy, Ixz), (Ixy, Iyy, Iyz), (Ixz, Iyz, Izz)))
    eigvalue, vectors = numpy.linalg.eig(imatrix)
    return


def operate_box(definition, boxes):
    fullbox = myBox()
    definition.level_update()
    for e in definition.elements:
        if type(e) is int:
            box = boxes[abs(e)]
            if definition.operator == "AND":
                fullbox.mult(box)
            else:
                fullbox.add(box)
        else:
            box = operate_box(e, boxes)
            if definition.operator == "AND":
                fullbox.mult(box)
            else:
                fullbox.add(box)
    return fullbox
