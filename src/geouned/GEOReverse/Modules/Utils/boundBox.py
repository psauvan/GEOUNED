import math
import numpy
import typing

from ....geo import GVector, GBoundBox, GPlane, GLine, Gmake_polygon_face, myBox
from .booleanFunction import BoolSequence, evaluate_three_valued, signed_surfaces

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


# myBox (Forward/Reversed box arithmetic) moved to `geo.myBox`, 2026-09-17
# -- shared verbatim with GEOUNED's `build_region/`, which turned out to
# have an independent copy of the same class with the exact same
# mixed-orientation bug this one's own docstring documents (see CLAUDE.md's
# "build_region/ vs CAD/buildSolidCell.py+splitFunction.py unification"
# entry). Nothing else in this file changes.


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
                # bool(...): GVector.dot() can return a numpy scalar, so
                # `dot > 0` may be a numpy.bool_ rather than a plain Python
                # bool. GEOUNED's own BoolSequence.substitute() branches on
                # `type(val) is not bool`, which is True for numpy.bool_ --
                # it would take the wrong branch (treating the value as
                # another surface number instead of a true/false
                # substitution) and corrupt the sequence.
                surf_value[p_index] = bool(dot > 0)
        inside = evaluate_three_valued(self.definition, surf_value)

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

    surf_index = signed_surfaces(cellDef)
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
            elif s.type == "hyperboloid":
                if s.params[4]:
                    # one-sheet hourglass: `hyperboloid_to_planes` returns 3
                    # separate groups (2 mirrored asymptotic sheets + a waist
                    # band) that must be OR'd together, not AND'd as a flat
                    # list would imply -- a point only needs to satisfy one
                    # group, never all 3 at once (see that function's own
                    # docstring).
                    groups_idx = []
                    for group in surf_planes:
                        idx = []
                        for p in group:
                            planes[next_index] = p
                            idx.append(next_index)
                            next_index += 1
                        groups_idx.append(idx)
                    surf_planes_dict[s_label] = ("hyp1sheet", groups_idx)
                else:
                    p_index = []
                    for p in surf_planes:
                        planes[next_index] = p
                        p_index.append(next_index)
                        next_index += 1
                    surf_planes_dict[s_label] = ("hyp2sheet", p_index)
            elif s.type == "cylinder_hyperbolic":
                groups_idx = []
                for group in surf_planes:
                    idx = []
                    for p in group:
                        planes[next_index] = p
                        idx.append(next_index)
                        next_index += 1
                    groups_idx.append(idx)
                surf_planes_dict[s_label] = ("cylhyp", groups_idx)
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
    elif s.type == "cylinder_elliptic":
        return elliptic_cylinder_to_planes(s, pos)
    elif s.type == "ellipsoid":
        return ellipsoid_to_planes(s, pos)
    elif s.type == "cone":
        return cone_to_planes(s, pos)
    elif s.type == "cone_elliptic":
        return elliptic_cone_to_planes(s, pos)
    elif s.type == "hyperboloid":
        return hyperboloid_to_planes(s, pos)
    elif s.type == "cylinder_hyperbolic":
        return cylinder_hyperbolic_to_planes(s, pos)
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

    return w, v


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


def elliptic_cylinder_to_planes(cyl, pos):
    """Same idea as `cylinder_to_planes` (a bounding/bounded rectangle of
    4 tangent planes, shrunk by 1/sqrt(2) for a conservative "inside"
    approximation, left full-size for a conservative "outside" one,
    averaged when the sense is undetermined), just with the single
    radius `R` replaced by the ellipse's own two semi-axes `a`
    (`major_radius`, along `major_axis`) and `b` (`minor_radius`, along
    `minor_axis`) -- unlike the circular case, these two directions are
    NOT interchangeable, so they're taken directly from the surface's
    own stored axes instead of `get_orto_axis(axis)`."""
    center, axis, radii, raxes = cyl.params
    minor_radius, major_radius = radii
    minor_axis, major_axis = raxes
    if pos is None:
        minor_radius = minor_radius * 0.8535533906
        major_radius = major_radius * 0.8535533906
    elif pos:
        minor_radius = minor_radius * 0.70710678
        major_radius = major_radius * 0.70710678

    r1 = center + major_axis * major_radius
    r2 = center - major_axis * major_radius
    r3 = center + minor_axis * minor_radius
    r4 = center - minor_axis * minor_radius

    p1 = GPlane.from_values(r1, -major_axis)
    p2 = GPlane.from_values(r2, major_axis)
    p3 = GPlane.from_values(r3, -minor_axis)
    p4 = GPlane.from_values(r4, minor_axis)
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


def elliptic_cone_to_planes(cone, pos):
    """Same idea as `cone_to_planes`, but the cross-section's two principal
    directions (`major_axis`/`minor_axis`) get their own distinct half-angle
    instead of sharing a single, direction-independent one -- same principle
    as `elliptic_cylinder_to_planes` vs. `cylinder_to_planes`: the tangent
    planes are built directly from the surface's own stored major/minor
    axes (not an arbitrary `get_orto_axis` basis), one pair of planes per
    axis instead of `nface` evenly-spaced directions, since the two
    directions are not interchangeable. `ref_radius` is the axial distance
    at which the cross-section ellipse's semi-axes equal `major_radius`/
    `minor_radius` exactly (the MCNP GQ/SQ scale-with-distance convention),
    so each direction's own half-angle is `atan(radius / ref_radius)`."""
    apex, axis, ref_radius, radii, raxes, dblsht = cone.params
    minor_radius, major_radius = radii
    minor_axis, major_axis = raxes

    t_major = major_radius / ref_radius
    t_minor = minor_radius / ref_radius
    if pos is None:
        t_major = t_major * 0.8535533906
        t_minor = t_minor * 0.8535533906
    elif pos:
        t_major = t_major * 0.70710678
        t_minor = t_minor * 0.70710678

    sa_major = math.atan(t_major)
    sa_minor = math.atan(t_minor)

    directions = (
        (major_axis, sa_major),
        (-major_axis, sa_major),
        (minor_axis, sa_minor),
        (-minor_axis, sa_minor),
    )
    cplanes = []
    for rho, sa in directions:
        ni = -axis * math.sin(sa) + rho * math.cos(sa)
        cplanes.append(GPlane.from_values(apex, -ni))

    cplanes.append(GPlane.from_values(apex, axis))
    return tuple(cplanes)


def cylinder_hyperbolic_to_planes(cyl, pos):
    """Bounding-plane approximation for the "cylinder_hyperbolic" GQ
    stype -- the real MCNP flat hyperbolic prism (a genuinely zero
    eigenvalue along the extrusion axis, from `get_cylinder_parameters`;
    NOT the same surface as "hyperboloid"'s `onesht=True` revolve-based
    hourglass -- see the history log for the full distinction). Per
    direct user instruction, 2026-09-14: same ring/vertex technique as
    `_hyperboloid_two_sheets_planes` (the 2-sheet hyperboloid's own
    single-branch model), called TWICE -- once for the surface's own
    `major_axis`, once rotated 180 degrees around `axis` (the true
    extrusion axis, perpendicular to the hyperbola's own plane -- NOT
    simply negating `major_axis` alone, which would mirror rather than
    rotate the construction and leave the resulting planes' normals
    pointing the wrong way for the second branch) -- since a real flat
    hyperbolic prism genuinely has BOTH branches present as bounding
    walls (unlike the 2-sheet hyperboloid, where only one branch is
    ever real material). The azimuthal sweep is restricted to `np=2`
    (phi=0/pi) with `x` set to the surface's own `minor_axis` exactly
    (not an arbitrary perpendicular from `get_orto_axis`) -- this is a
    flat 2D curve extruded along `axis`, not a surface of revolution,
    so there is no azimuthal direction to sweep at all; `y` (=`axis`
    itself here) is never actually used since sin(0)=sin(pi)=0 exactly,
    but still needed to complete the orthonormal basis
    `_hyperboloid_two_sheets_planes` expects.

    `_hyperboloid_two_sheets_planes`'s own planes point OUTWARD (away
    from `center`, beyond the vertex) -- correct for its own use (a
    2-sheet hyperboloid's real material sits BEYOND its one real
    vertex). Here the real material is the OPPOSITE: the channel
    BETWEEN the two branches, i.e. BEFORE each branch's own vertex --
    so every returned plane's normal is negated relative to the reused
    function's own output."""
    center, axis, radii, rAxes = cyl.params
    a_len, b_len = radii[1], radii[0]
    x, y = get_orto_axis(axis)
    nt = 2
    rmax = 7.5e5

    p0 = b_len / a_len
    cplanes = [[], []]

    for n in range(nt):
        if n == 0:
            xn = math.sqrt((rmax * rmax + b_len * b_len) / (1 + p0 * p0))
            yn = math.sqrt(p0 * p0 * xn * xn - b_len * b_len)
            slope = p0
        else:
            pn = (n + 1) * p0
            n2 = 1 / ((n + 1) * (n + 1))
            xn = a_len / math.sqrt(1 - n2)  # (p0/pn)^2 == 1/(n+1)^2
            yn = xn * p0 / (n + 1)
            slope = pn

        for i in range(2):
            slope_1 = -slope * x + y
            slope_2 = -slope * x - y
            xe_1 = center + yn * y + xn * x
            xe_2 = center - yn * y + xn * x
            normal_1 = slope_1.normalized()
            normal_2 = slope_2.normalized()
            cplanes[i].append(GPlane.from_values(xe_1, normal_1))
            cplanes[i].append(GPlane.from_values(xe_2, normal_2))
            x = -x
            y = -y

    xe_1 = center + a_len * x
    xe_2 = center - a_len * x
    cplanes[0].append(GPlane.from_values(xe_1, -x))
    cplanes[1].append(GPlane.from_values(xe_2, x))

    return cplanes


def _hyperboloid_two_sheets_planes(center, major_axis, x, y, a_len, b_len, np, nt, rmax):
    """One vertex-on-`axis` hyperboloid sheet's own tangent-plane ring
    sequence -- the shared core of the "2-sheet" technique in
    `hyperboloid_to_planes` below (`parabola_to_planes`'s own faceted,
    converging-ring model, just built from the hyperbola's own curve
    instead of the parabola's, with the outermost ring built from the
    exact asymptote instead of an approximation -- see that function's
    own docstring for the full derivation). `a_len` (semi-transverse,
    along `axis`, the vertex offset from `center`) and `b_len` (semi-
    conjugate, radial) are the hyperbola's own two defining lengths;
    `x`/`y` is the orthonormal basis perpendicular to `axis`."""
    dphi = twoPi / np
    vertex = center + major_axis * b_len
    # x0 chosen so the outermost ring (n == nt) lands at the radial
    # coordinate whose asymptote-axial-position is ~rmax (mirrors
    # parabola_to_planes's own x0, just solved through the hyperbola's
    # own asymptote z ~ (a/b)*xp instead of the parabola's z=xp^2/(4f)).

    p0 = b_len / a_len

    cplanes = [GPlane.from_values(vertex, major_axis)]
    for n in range(nt):
        if n == 0:
            xn = math.sqrt((rmax * rmax + b_len * b_len) / (1 + p0 * p0))
            z_local = math.sqrt(p0 * p0 * xn * xn - b_len * b_len)
            slope_rho_coeff = b_len
            slope_axis_coeff = a_len
        else:
            pn = p0 / (n + 1)
            n2 = (n + 1) * (n + 1)
            nsq = math.sqrt(n + 1)
            xn = a_len / math.sqrt(1 + n2)  # (p0/pn)^2 == (n+1)^2
            yn = xn * pn * n2
            z_local = yn
            slope_rho_coeff = b_len / nsq
            slope_axis_coeff = a_len * nsq

        for i in range(np):
            phi = i * dphi
            rho = x * math.cos(phi) + y * math.sin(phi)
            slope = -slope_rho_coeff * rho + slope_axis_coeff * major_axis
            xe = center + xn * rho + z_local * major_axis
            normal = slope.normalized()
            cplanes.append(GPlane.from_values(xe, normal))

    return cplanes


def _hyperboloid_one_sheet_planes(center, major_axis, x, y, a_len, b_len, np, nt, rmax):
    """One vertex-on-`axis` hyperboloid sheet's own tangent-plane ring
    sequence -- the shared core of the "2-sheet" technique in
    `hyperboloid_to_planes` below (`parabola_to_planes`'s own faceted,
    converging-ring model, just built from the hyperbola's own curve
    instead of the parabola's, with the outermost ring built from the
    exact asymptote instead of an approximation -- see that function's
    own docstring for the full derivation). `a_len` (semi-transverse,
    along `axis`, the vertex offset from `center`) and `b_len` (semi-
    conjugate, radial) are the hyperbola's own two defining lengths;
    `x`/`y` is the orthonormal basis perpendicular to `axis`."""
    dphi = twoPi / np

    p0 = a_len / b_len
    cplanes = [[] for _ in range(np)]

    for n in range(nt):
        if n == 0:
            xn = math.sqrt((rmax * rmax + a_len * a_len) / (1 + p0 * p0))
            z_local = math.sqrt(p0 * p0 * xn * xn - a_len * a_len)
            slope_rho_coeff = a_len
            slope_axis_coeff = b_len
        else:
            pn = (n + 1) * p0
            n2 = 1 / ((n + 1) * (n + 1))
            nsq = math.sqrt(n + 1)
            xn = b_len / math.sqrt(1 - n2)  # (p0/pn)^2 == 1/(n+1)^2
            yn = xn * pn * n2
            z_local = yn
            slope_rho_coeff = a_len * nsq
            slope_axis_coeff = b_len / nsq

        for i in range(np):
            phi = i * dphi
            rho = x * math.cos(phi) + y * math.sin(phi)
            slope_1 = -slope_rho_coeff * rho + slope_axis_coeff * major_axis
            slope_2 = -slope_rho_coeff * rho - slope_axis_coeff * major_axis
            xe_1 = center + xn * rho + z_local * major_axis
            xe_2 = center + xn * rho - z_local * major_axis
            normal_1 = slope_1.normalized()
            normal_2 = slope_2.normalized()
            cplanes[i].append(GPlane.from_values(xe_1, normal_1))
            cplanes[i].append(GPlane.from_values(xe_2, normal_2))

    for i in range(np):
        phi = i * dphi
        rho = x * math.cos(phi) + y * math.sin(phi)
        xe = center + b_len * rho
        cplanes[i].append(GPlane.from_values(xe, -rho))

    return cplanes


def hyperboloid_to_planes(hyp, pos):
    """Bounding-plane approximation for the "hyperboloid" GQ stype, built
    differently depending on `onesht` (per direct user instruction,
    2026-09-14 -- the two surfaces are NOT bounded the same way):

    `onesht=False` (standard 2-sheet, only the +axis branch is ever
    built, matching `Objects.py::Hyperboloid.buildShape`'s own
    convention): a single `_hyperboloid_sheet_planes` call -- same model
    as `parabola_to_planes`.

    `onesht=True` (the connected one-sheet "hourglass", revolved around
    the true conjugate axis -- `axis`/`rAxes[1]` here): built from TWO
    `_hyperboloid_sheet_planes` calls, mirrored on `+major_axis` and
    `-major_axis` (an outer, cone-like envelope sharing the surface's own
    real asymptotic slope, even though `major_axis` isn't its true
    revolution axis), PLUS 4 simple tangent planes wrapping `minor_axis`
    at the waist radius (`cylinder_to_planes`'s own technique, using
    `minor_axis` as the cylinder axis) to close off the near-waist region
    where the mirrored sheets are a poor local bound (the true surface
    has no vertex there at all -- the waist already has nonzero radius).

    Returned as 3 SEPARATE plane groups (a point only needs to satisfy
    ALL of one group, not all 3 groups at once -- the two mirrored sheets
    are mutually exclusive almost everywhere, so requiring both at the
    same time, as a flat plane list normally implies, would leave almost
    no point "inside" at all). `quadric_to_plane`/`plane_definition` know
    to OR the 3 groups together for this specific return shape (`"cone"`/
    `"torus"` already get equivalent special-cased treatment there for
    the same reason)."""
    center, axis, radii, rAxes, onesht = hyp.params
    axis = axis.normalized()
    np = 4
    nt = 2
    rmax = 7.5e5

    a_len, b_len = radii[1], radii[0]  # a=semi-transverse(axial), b=semi-conjugate(radial)
    x, y = get_orto_axis(axis)
    if onesht:
        return _hyperboloid_one_sheet_planes(center, axis, x, y, a_len, b_len, np, nt, rmax)
    else:
        return _hyperboloid_two_sheets_planes(center, axis, x, y, b_len, a_len, np, nt, rmax)


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


def ellipsoid_to_planes(ellip, pos):
    """Mix of `sphere_to_planes` and `cylinder_to_planes` (per direct
    user instruction): 4 planes surround the equator, all at the SAME
    distance from `center` (the equatorial cross-section of a spheroid
    of revolution is always a circle) -- that shared distance is
    `perp_radius`, whichever of the ellipsoid's own two radii is
    perpendicular to `axis` (`major_radius` if oblate, `minor_radius` if
    prolate, matching `_make_ellipsoid_native`'s own prolate/oblate
    branch). 2 more planes close it off at each pole, at `rev_radius`
    (the other radius, the one along `axis` itself)."""
    center, axis, radii, raxes = ellip.params
    minor_radius, major_radius = radii
    minor_axis, major_axis = raxes

    if (axis - minor_axis).length < 1e-5:
        rev_radius, perp_radius = minor_radius, major_radius
    else:
        rev_radius, perp_radius = major_radius, minor_radius

    if pos is None:
        rev_radius = rev_radius * 0.8535533906
        perp_radius = perp_radius * 0.8535533906
    elif pos:
        rev_radius = rev_radius * 0.70710678
        perp_radius = perp_radius * 0.70710678

    x, y = get_orto_axis(axis)

    r1 = center + x * perp_radius
    r2 = center - x * perp_radius
    r3 = center + y * perp_radius
    r4 = center - y * perp_radius
    r5 = center + axis * rev_radius
    r6 = center - axis * rev_radius

    p1 = GPlane.from_values(r1, -x)
    p2 = GPlane.from_values(r2, x)
    p3 = GPlane.from_values(r3, -y)
    p4 = GPlane.from_values(r4, y)
    p5 = GPlane.from_values(r5, -axis)
    p6 = GPlane.from_values(r6, axis)
    return (p1, p2, p3, p4, p5, p6)


def torus_to_planes(torus, pos):
    center, axis, majorRadius, minorR, minorA, degenerated = torus.params

    x = GVector(1, 0, 0)
    y = GVector(0, 1, 0)
    z = GVector(0, 0, 1)

    if degenerated == 0:
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
    else:
        outer = degenerated > 0
        if minorA > minorR :
            if outer:
                h = minorR 
            else:    
                df = math.sqrt(minorA*minorA-minorR*minorR)
                h2 = minorA**2 - df**2 - majorRadius**2 *(1-(df/minorA)**2)
                h = math.sqrt(h2) 
        else:
            if outer:
                h = minorR
            else:    
                df = math.sqrt(minorR*minorR-minorA*minorA)
                h2 = (minorR**2 - majorRadius**2 - df**2)/(1-(df/minorR)**2) 
                h = math.sqrt(h2)

        if pos is None:
            h *= 0.8535533906
            if outer:
                dist = (majorRadius + minorA) * 0.8535533906
            else:
                dist = (minorA - majorRadius) * 0.8535533906
        elif pos:
            h *= 0.70710678
            if outer:
                dist = (majorRadius + minorA) * 0.70710678
            else:
                dist = (minorA - majorRadius) * 0.70710678
        else:
            if outer:
                dist = majorRadius + minorA
            else:
                dist = minorA - majorRadius    

        if abs(abs(axis.dot(x)) - 1) < 1e-5:
            r1 = center + x * h
            r2 = center - x * h
            r3 = center + y * dist
            r4 = center - y * dist
            r5 = center + z * dist
            r6 = center - z * dist
        elif abs(abs(axis.dot(y)) - 1) < 1e-5:
            r1 = center + x * dist
            r2 = center - x * dist
            r3 = center + y * h
            r4 = center - y * h
            r5 = center + z * dist
            r6 = center - z * dist
        elif abs(abs(axis.dot(z)) - 1) < 1e-5:
            r1 = center + x * dist
            r2 = center - x * dist
            r3 = center + y * dist
            r4 = center - y * dist
            r5 = center + z * h
            r6 = center - z * h
 
        p1 = GPlane.from_values(r1, -x)
        p2 = GPlane.from_values(r2, x)
        p3 = GPlane.from_values(r3, -y)
        p4 = GPlane.from_values(r4, y)
        p5 = GPlane.from_values(r5, -z)
        p6 = GPlane.from_values(r6, z)
        external_planes = (p1, p2, p3, p4, p5, p6)
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
    np = 4
    a = 0.22
    rmax = 7.5e5
    b = 1 - math.sqrt(2 * a)
    x0 = b**nt * math.sqrt(4 * focal * rmax)

    axis = axis.normalized()
    x, y = get_orto_axis(axis)
    dphi = twoPi / np

    p0 = GPlane.from_values(center, axis)
    cplanes = [p0]
    focal = float(focal)
    xp = x0
    for n in range(nt + 1):
        zi = 0.25 * xp * xp / focal
        phi = 0
        for i in range(np):
            rho = x * math.cos(phi) + y * math.sin(phi)
            vec = y * math.cos(phi) - x * math.sin(phi)
            slope = 2 * focal * rho + xp * axis  # slope xp/(2*focal)
            xe = center + xp * rho + zi * axis
            normal = vec.cross(slope)
            normal = normal.normalized()
            pi = GPlane.from_values(xe, -normal)
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
                cone1 = BoolSequence(" ".join((str(p) for p in planes[1])))
                cone2 = BoolSequence(" ".join((str(-p) for p in planes[1])))
                pm = BoolSequence(operator="OR")
                pm.append(cone1, cone2)
            elif planes[0] == "hyp1sheet":
                addpos = True
                pm = BoolSequence(operator="AND")
                for side in planes[1]:
                    groups = BoolSequence(":".join((str(p) for p in side)))
                    pm.append(groups)
            elif planes[0] == "hyp2sheet":
                addpos = True
                s = -s  # hyperboloid 2 sheet has inverted orientation sign
                pm = BoolSequence(" ".join((str(p) for p in planes[1])))
            elif planes[0] == "cylhyp":
                addpos = True
                pm = BoolSequence(operator="AND")
                side1 = BoolSequence(":".join((str(p) for p in planes[1][0])))
                side2 = BoolSequence(":".join((str(p) for p in planes[1][1])))
                pm.append(side1, side2)
            else:
                addpos = True
                pm = BoolSequence(" ".join((str(p) for p in planes[1])))
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
    # A plain literal element that resolves to the operator's identity
    # value (case below) is dropped from the list outright rather than
    # index-assigned as a bare True/False -- GEOUNED's own BoolSequence
    # (the canonical class since the 2026-09-12 BoolSequence unification)
    # only expects `.elements` list entries to be int literals or
    # BoolSequence instances; a bare bool sitting nested inside the list
    # crashes `.copy()` (`AttributeError: 'bool' object has no attribute
    # 'copy'`) and is silently ignored (never actually removed) by
    # `.clean()`. Collecting indices and deleting them below never
    # creates that shape in the first place.
    clean = False
    to_remove = []
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
                        to_remove.append(i)
                        clean = True
                else:
                    seq.elements[i] = new
    for i in sorted(to_remove, reverse=True):
        del seq.elements[i]
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
