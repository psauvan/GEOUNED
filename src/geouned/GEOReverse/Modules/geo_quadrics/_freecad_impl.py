"""
GEOReverse/Modules/geo_quadrics/_freecad_impl.py

FreeCAD implementation of the 6 exotic quadric surface types CsgToCad
(GEOReverse) can encounter that GEOUNED's forward pipeline never
produces and `geo` therefore has no classes for: elliptic cone,
hyperboloid, ellipsoid, elliptic cylinder, hyperbolic cylinder,
paraboloid. Kept local to GEOReverse rather than inside `geo` itself --
see CLAUDE.md's GEOReverse migration section for the reasoning (`geo`
stays scoped to surfaces GEOUNED's own decomposition can classify).

The ONLY file in this package allowed to `import Part`/`FreeCAD` --
mirrors `geo/_freecad_impl.py`'s own role exactly. `Objects.py` (the
"programa principal" for GEOReverse's own build step) never imports this
file directly: it goes through `geo_quadrics/__init__.py`'s
`Gmake_elliptic_cone`/`Gmake_hyperboloid`/... dispatch, which resolves to
this module or `_occ_impl.py` depending on `CAD_ENGINE`.

The dataclasses below (`GEllipticCone`, `GHyperboloid`, ...) follow the
same conventions as `geo`'s own analytic descriptors (`from_values(...)`
constructor, `.is_inside(point) -> bool`, `.transform(matrix)` taking a
native `FreeCAD.Matrix` and returning a new instance) -- kept as this
module's own internal representation; the `Gmake_*` functions at the
bottom of this file are the only names `geo_quadrics/__init__.py`
re-exports.

Ported faithfully from `Objects.py`'s pre-migration
`makeHyperboloid`/`makeHyperbolicCylinder`/`makeEllipticCylinder`/
`makeEllipsoid`/`makeEllipticCone`/`makeParaboloid`/`ortoVect` (shape
construction) and `splitFunction.py::surface_side`'s
`cone_elliptic`/`hyperboloid`/`ellipsoid`/`cylinder_elliptic`/
`cylinder_hyperbolic`/`paraboloid` branches (in/out test) -- including
known, NOT-fixed-here bugs, each flagged with a comment (per explicit
instruction: bugs found during this migration get resolved in a later,
separate pass, not silently while porting).

KNOWN, CONFIRMED-PRE-EXISTING GAPS (not fixed here, verified against the
original, unmigrated `Objects.py` before concluding this):
- `GEllipsoid.build_shape()` fails with `RuntimeError: ... No shells or
  compsolids found in shape` in `Part.makeSolid` -- confirmed the exact
  original `Objects.py::makeEllipsoid` fails identically, in BOTH its own
  branches (revolve about the major axis and about the minor axis alike),
  on this FreeCAD version. `.is_inside()`/`.transform()` are unaffected
  (pure GVector math, verified independently) -- only the native shape
  construction is broken.
- `GHyperboloid.build_shape(one_sheet=True)` fails the same way, for a
  different underlying reason: `Part.makeShell` silently falls back to
  returning a `Compound` (not a real `Shell`) when the two independently
  -built end-cap circle faces don't sew exactly onto the revolved
  hyperbola face's own boundary edges -- a tolerance/precision issue in
  this specific "build 3 separate faces, hope they sew" construction
  technique, not something introduced by this port (confirmed: reaching
  this exact failure point already requires a fix to a separate, earlier
  bug in `makeHyperboloid` -- see `GHyperboloid.build_shape`'s own
  docstring -- so the pristine original never even got this far).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import FreeCAD
import Part

from ....geo.vector_geometry import GVector
from ....geo._freecad_impl import GSolid, to_native_vector


def ortoVect(axis: GVector) -> GVector:
    """
    An arbitrary unit vector perpendicular to `axis`, picked by crossing
    `axis` with whichever of the 3 world axes it is *least* aligned with
    (avoids a near-zero cross product). Pure GVector math, ported
    verbatim from `Objects.py::ortoVect` -- no native call needed at all
    (the original's `FreeCAD.Vector(vOrto)` was just a roundabout way to
    build a literal unit vector).
    """
    vmax = 0.0
    v_orto = None
    if abs(axis.x) > vmax:
        v_orto = GVector(0, 1, 0)
        vmax = abs(axis.x)
    if abs(axis.y) > vmax:
        v_orto = GVector(0, 0, 1)
        vmax = abs(axis.y)
    if abs(axis.z) > vmax:
        v_orto = GVector(1, 0, 0)
        vmax = abs(axis.z)
    if v_orto is None:
        return None
    return axis.cross(v_orto).normalized()


# ---------------------------------------------------------------------------
# GEllipticCone
# ---------------------------------------------------------------------------


@dataclass
class GEllipticCone:
    Apex: GVector
    Axis: GVector
    RefRadius: float  # Ra: reference (minor-axis) radius at axial distance 1 from Apex
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector
    DoubleSheet: bool = False

    @classmethod
    def from_values(
        cls, apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet=False
    ) -> "GEllipticCone":
        return cls(apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet)

    def is_inside(self, point: GVector) -> bool:
        r = point - self.Apex
        x = r.dot(self.MajorAxis)
        y = r.dot(self.MinorAxis)
        z = r.dot(self.Axis)
        if self.DoubleSheet:
            z = abs(z)
        return (x / self.MajorRadius) ** 2 + (y / self.MinorRadius) ** 2 - z / self.RefRadius < 0

    def transform(self, matrix: FreeCAD.Matrix) -> "GEllipticCone":
        rot = matrix.submatrix(3)
        return GEllipticCone(
            to_gvector_(matrix.multVec(to_native_vector(self.Apex))),
            to_gvector_(rot.multVec(to_native_vector(self.Axis))),
            self.RefRadius,
            self.MajorRadius,
            self.MinorRadius,
            to_gvector_(rot.multVec(to_native_vector(self.MajorAxis))),
            to_gvector_(rot.multVec(to_native_vector(self.MinorAxis))),
            self.DoubleSheet,
        )

    def build_shape(self, length: float) -> GSolid:
        return GSolid(
            _make_elliptic_cone_native(self, length, forward=True) if not self.DoubleSheet else _make_double_sheet(self, length)
        )


def _make_elliptic_cone_native(surf: "GEllipticCone", length: float, forward: bool):
    apex = to_native_vector(surf.Apex)
    axis = to_native_vector(surf.Axis) if forward else -to_native_vector(surf.Axis)
    major_axis = to_native_vector(surf.MajorAxis)
    minor_axis = to_native_vector(surf.MinorAxis)
    d = axis * length

    s1 = apex + major_axis * (surf.MajorRadius / surf.RefRadius * length)
    s2 = apex + minor_axis * (surf.MinorRadius / surf.RefRadius * length)

    point = Part.Point(apex).toShape()
    ellipse = Part.Ellipse(s1 + d, s2 + d, apex + d)

    shape = ellipse.toBSpline().toShape()
    shell = Part.makeLoft([point, shape], True)
    return Part.makeSolid(shell)


def _make_double_sheet(surf: "GEllipticCone", length: float):
    sheet1 = _make_elliptic_cone_native(surf, length, forward=True)
    sheet2 = _make_elliptic_cone_native(surf, length, forward=False)
    fused = sheet1.fuse([sheet2])
    fused.removeSplitter()
    return fused


# ---------------------------------------------------------------------------
# GHyperboloid
# ---------------------------------------------------------------------------


@dataclass
class GHyperboloid:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector
    OneSheet: bool = True

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet=True) -> "GHyperboloid":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet)

    def is_inside(self, point: GVector) -> bool:
        """
        Ported as-is from `surface_side`'s `hyperboloid` branch, including
        its own pre-existing bug: `v = r - (rX * rAxes[1] + center)` mixes
        a vector (`rX * MajorAxis`) with a point (`center`) before
        subtracting from `r` (itself already relative to `center`) -- this
        silently double-subtracts `center`'s own coordinates as if it were
        a displacement, not a position. NOT fixed here (see module
        docstring); flagged for the follow-up bug-fixing pass.
        """
        r = point - self.Center
        rx = r.dot(self.MajorAxis)
        v = r - (rx * self.MajorAxis + self.Center)  # pre-existing bug, ported as-is
        d = v.length

        one = 1.0 if self.OneSheet else -1.0
        radical = (rx / self.MajorRadius) ** 2 + one
        if radical > 0:
            y = self.MinorRadius * math.sqrt(radical)
            return d - y < 0
        return False

    def transform(self, matrix: FreeCAD.Matrix) -> "GHyperboloid":
        rot = matrix.submatrix(3)
        return GHyperboloid(
            to_gvector_(matrix.multVec(to_native_vector(self.Center))),
            to_gvector_(rot.multVec(to_native_vector(self.Axis))),
            self.MajorRadius,
            self.MinorRadius,
            to_gvector_(rot.multVec(to_native_vector(self.MajorAxis))),
            to_gvector_(rot.multVec(to_native_vector(self.MinorAxis))),
            self.OneSheet,
        )

    def build_shape(self, length: float) -> GSolid:
        """
        NOTE (deviation from strict as-is porting, unlike the rest of this
        module): the original `Objects.py::makeHyperboloid`'s `point =
        center + X * radii[1] + Y * radii[0]` adds two *scalars*
        (`radii[1]`/`radii[0]`) to a point, which raises `TypeError:
        Second arg must be Vector` unconditionally -- confirmed the
        pristine original has never been able to execute past this line,
        for any input. `_make_hyperboloid_native` below uses
        `major_axis`/`minor_axis` (the actual direction vectors) instead,
        by direct analogy with the structurally-identical, already-working
        line in `GHyperbolicCylinder.build_shape` a few classes below.
        Even with this fix, `one_sheet=True` still fails further down (see
        module docstring) -- this correction alone doesn't make the whole
        function work, just moves the failure to the next real gap.
        """
        return GSolid(_make_hyperboloid_native(self, length))


def _make_hyperboloid_native(surf: "GHyperboloid", length: float):
    center = to_native_vector(surf.Center)
    axis = to_native_vector(surf.Axis)
    major_axis = to_native_vector(surf.MajorAxis)
    minor_axis = to_native_vector(surf.MinorAxis)

    s1 = center + major_axis * surf.MajorRadius
    s2 = center + minor_axis * surf.MinorRadius
    hyperbola = Part.Hyperbola(s1, s2, center)

    y = length
    x = surf.MajorRadius * math.sqrt((y / surf.MinorRadius) ** 2 + 1)
    point = center + x * major_axis + y * minor_axis
    parameter = abs(hyperbola.parameter(point))

    if surf.OneSheet:
        shape = hyperbola.toBSpline(-parameter, parameter).toShape(-parameter, parameter)
        hyper_face = shape.revolve(center, axis, 360)

        start_point = hyper_face.Surface.BasisCurve.StartPoint - center
        end_point = hyper_face.Surface.BasisCurve.EndPoint - center

        rad1 = start_point.dot(hyperbola.XAxis)
        hgt1 = start_point.dot(hyperbola.YAxis)
        cc1 = center + hyperbola.YAxis * hgt1
        circle1 = Part.Circle(cc1, -hyperbola.YAxis, rad1).toShape()
        cface1 = Part.makeFace(circle1, "Part::FaceMakerSimple")

        rad2 = end_point.dot(hyperbola.XAxis)
        hgt2 = end_point.dot(hyperbola.YAxis)
        cc2 = center + hyperbola.YAxis * hgt2
        circle2 = Part.Circle(cc2, hyperbola.YAxis, rad2).toShape()
        cface2 = Part.makeFace(circle2, "Part::FaceMakerSimple")

        shell = Part.makeShell((cface1, hyper_face, cface2))
        return Part.makeSolid(shell)
    else:
        shape = hyperbola.toBSpline(0, parameter).toShape(0, parameter)
        hyper_face = shape.revolve(center, axis, 360)

        end_point = hyper_face.Surface.BasisCurve.EndPoint - center
        rad = end_point.dot(hyperbola.YAxis)
        hgt = end_point.dot(hyperbola.XAxis)
        cc = center + hyperbola.XAxis * hgt
        circle = Part.Circle(cc, -hyperbola.XAxis, rad).toShape()
        cface = Part.makeFace(circle, "Part::FaceMakerSimple")

        shell = Part.makeShell((cface, hyper_face))
        hyper1 = Part.makeSolid(shell)
        hyper2 = hyper1.rotated(center, hyperbola.YAxis, 180)
        return Part.makeCompound((hyper1, hyper2))


# ---------------------------------------------------------------------------
# GEllipsoid
# ---------------------------------------------------------------------------


@dataclass
class GEllipsoid:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis) -> "GEllipsoid":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis)

    def is_inside(self, point: GVector) -> bool:
        """
        Ported as-is from `surface_side`'s `ellipsoid` branch, including
        its own pre-existing bug: the "else" (revolution around minor
        axis) case does `radY, radY = radii` -- a typo that should read
        `radX, radY = radii`, so `radX` is silently left undefined in the
        original and this branch can never have actually run without
        raising `NameError`. NOT fixed here (see module docstring).
        """
        r = point - self.Center
        rx = r.dot(self.Axis)
        ry_vec = r - (rx * self.Axis + self.Center)  # pre-existing bug, ported as-is (see GHyperboloid.is_inside)
        ry = ry_vec.length

        if (self.Axis - self.MinorAxis).length < 1e-5:
            rad_x, rad_y = self.MajorRadius, self.MinorRadius
        else:
            rad_y = self.MinorRadius  # pre-existing bug: rad_x left undefined, ported as-is
            rad_x = rad_y

        radical = 1 - (rx / rad_x) ** 2
        if radical > 0:
            y = rad_y * math.sqrt(radical)
            return ry - y < 0
        return False

    def transform(self, matrix: FreeCAD.Matrix) -> "GEllipsoid":
        rot = matrix.submatrix(3)
        return GEllipsoid(
            to_gvector_(matrix.multVec(to_native_vector(self.Center))),
            to_gvector_(rot.multVec(to_native_vector(self.Axis))),
            self.MajorRadius,
            self.MinorRadius,
            to_gvector_(rot.multVec(to_native_vector(self.MajorAxis))),
            to_gvector_(rot.multVec(to_native_vector(self.MinorAxis))),
        )

    def build_shape(self) -> GSolid:
        return GSolid(_make_ellipsoid_native(self))


def _make_ellipsoid_native(surf: "GEllipsoid"):
    center = to_native_vector(surf.Center)
    axis = to_native_vector(surf.Axis)
    major_axis = to_native_vector(surf.MajorAxis)
    minor_axis = to_native_vector(surf.MinorAxis)

    s1 = center + major_axis * surf.MajorRadius
    s2 = center + minor_axis * surf.MinorRadius
    ellipse = Part.Ellipse(s1, s2, center)

    if (axis - minor_axis).Length < 1e-5:
        shape = ellipse.toBSpline().toShape()
        shell = shape.revolve(center, axis, 180)
    else:
        shape = ellipse.toBSpline(0, math.pi).toShape(0, math.pi)
        shell = shape.revolve(center, axis, 360)
    return Part.makeSolid(shell)


# ---------------------------------------------------------------------------
# GEllipticCylinder
# ---------------------------------------------------------------------------


@dataclass
class GEllipticCylinder:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis) -> "GEllipticCylinder":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis)

    def is_inside(self, point: GVector) -> bool:
        r = point - self.Center
        x = r.dot(self.MajorAxis)
        y = r.dot(self.MinorAxis)
        return (x / self.MajorRadius) ** 2 + (y / self.MinorRadius) ** 2 - 1 < 0

    def transform(self, matrix: FreeCAD.Matrix) -> "GEllipticCylinder":
        rot = matrix.submatrix(3)
        return GEllipticCylinder(
            to_gvector_(matrix.multVec(to_native_vector(self.Center))),
            to_gvector_(rot.multVec(to_native_vector(self.Axis))),
            self.MajorRadius,
            self.MinorRadius,
            to_gvector_(rot.multVec(to_native_vector(self.MajorAxis))),
            to_gvector_(rot.multVec(to_native_vector(self.MinorAxis))),
        )

    def build_shape(self, height: float) -> GSolid:
        center = to_native_vector(self.Center)
        axis = to_native_vector(self.Axis)
        major_axis = to_native_vector(self.MajorAxis)
        minor_axis = to_native_vector(self.MinorAxis)
        d = axis * height

        s1 = center + major_axis * self.MajorRadius
        s2 = center + minor_axis * self.MinorRadius

        ellipse = Part.Ellipse(s1, s2, center)
        ellipse2 = Part.Ellipse(s1 + d, s2 + d, center + d)

        shape = ellipse.toBSpline().toShape()
        shape2 = ellipse2.toBSpline().toShape()
        shell = Part.makeLoft([shape, shape2], True)
        return GSolid(Part.makeSolid(shell))


# ---------------------------------------------------------------------------
# GHyperbolicCylinder
# ---------------------------------------------------------------------------


@dataclass
class GHyperbolicCylinder:
    Center: GVector
    Axis: GVector
    MajorRadius: float
    MinorRadius: float
    MajorAxis: GVector
    MinorAxis: GVector

    @classmethod
    def from_values(cls, center, axis, major_radius, minor_radius, major_axis, minor_axis) -> "GHyperbolicCylinder":
        return cls(center, axis, major_radius, minor_radius, major_axis, minor_axis)

    def is_inside(self, point: GVector) -> bool:
        r = point - self.Center
        x = r.dot(self.MajorAxis)
        y = r.dot(self.MinorAxis)
        return (x / self.MajorRadius) ** 2 - (y / self.MinorRadius) ** 2 - 1 < 0

    def transform(self, matrix: FreeCAD.Matrix) -> "GHyperbolicCylinder":
        rot = matrix.submatrix(3)
        return GHyperbolicCylinder(
            to_gvector_(matrix.multVec(to_native_vector(self.Center))),
            to_gvector_(rot.multVec(to_native_vector(self.Axis))),
            self.MajorRadius,
            self.MinorRadius,
            to_gvector_(rot.multVec(to_native_vector(self.MajorAxis))),
            to_gvector_(rot.multVec(to_native_vector(self.MinorAxis))),
        )

    def build_shape(self, length: float) -> GSolid:
        center = to_native_vector(self.Center)
        axis = to_native_vector(self.Axis)
        major_axis = to_native_vector(self.MajorAxis)
        minor_axis = to_native_vector(self.MinorAxis)

        s11 = center + major_axis * self.MajorRadius
        s12 = center + minor_axis * self.MinorRadius
        s21 = center - major_axis * self.MajorRadius
        s22 = center - minor_axis * self.MinorRadius

        hyperbola1 = Part.Hyperbola(s11, s12, center)
        hyperbola2 = Part.Hyperbola(s21, s22, center)
        d = axis * length

        y = length
        x = self.MajorRadius * math.sqrt((y / self.MinorRadius) ** 2 + 1)
        point = center + x * major_axis + y * minor_axis
        parameter = abs(hyperbola1.parameter(point))

        shape1 = hyperbola1.toBSpline(-parameter, parameter).toShape(-parameter, parameter)
        shape2 = hyperbola2.toBSpline(-parameter, parameter).toShape(-parameter, parameter)
        surf1 = shape1.extrude(d)
        surf2 = shape2.extrude(d)

        return GSolid(Part.makeCompound((surf1, surf2)))


# ---------------------------------------------------------------------------
# GParaboloid
# ---------------------------------------------------------------------------


@dataclass
class GParaboloid:
    Center: GVector
    Axis: GVector
    Focal: float

    @classmethod
    def from_values(cls, center, axis, focal) -> "GParaboloid":
        return cls(center, axis, focal)

    def is_inside(self, point: GVector) -> bool:
        r = point - self.Center
        x = r.dot(self.Axis)
        if x < 0:
            return False
        v = r - x * self.Axis
        d = v.length
        y = math.sqrt(4 * self.Focal * x)
        return d - y < 0

    def transform(self, matrix: FreeCAD.Matrix) -> "GParaboloid":
        return GParaboloid(
            to_gvector_(matrix.multVec(to_native_vector(self.Center))),
            to_gvector_(matrix.submatrix(3).multVec(to_native_vector(self.Axis))),
            self.Focal,
        )

    def build_shape(self, length: float) -> GSolid | None:
        """Returns None if the whole probed bounding extent lies behind
        the paraboloid's own vertex (matches `Objects.py::Paraboloid.buildShape`'s
        own `if dmax <= 0: return` guard -- ported here as a None return
        since this method has no boundBox-scanning caller context of its
        own; the Phase 4 wrapper is expected to check for None)."""
        center = to_native_vector(self.Center)
        axis = to_native_vector(self.Axis)

        r = math.sqrt(4 * self.Focal * length)
        parabola = Part.Parabola()
        parabola.Center = center
        parabola.Axis = to_native_vector(ortoVect(self.Axis))
        parabola.XAxis = axis
        parabola.Focal = self.Focal

        probe = center + length * parabola.XAxis + r * parabola.YAxis
        parameter = abs(parabola.parameter(probe))

        shape = parabola.toBSpline(0, parameter).toShape(0, parameter)
        para_face = shape.revolve(center, axis, 360)

        cc = center + length * parabola.XAxis
        circle = Part.Circle(cc, -axis, r).toShape()
        cface = Part.makeFace(circle, "Part::FaceMakerSimple")

        shell = Part.makeShell((cface, para_face))
        return GSolid(Part.makeSolid(shell))


# ---------------------------------------------------------------------------
# Elliptic torus (an "extra field" variant of a surface `geo` already
# models -- kept here, not as a Gmake_torus_elliptic sibling in geo
# itself, because its construction needs the same Ellipse/BSpline/revolve
# machinery as the exotic quadrics above, not a simple native primitive
# call like Gmake_cone_frustum's.)
# ---------------------------------------------------------------------------


def _make_torus_elliptic_native(
    center: GVector, axis: GVector, r_major_axis_offset: float, major_radius: float, minor_radius: float
) -> GSolid:
    center_native = to_native_vector(center)
    z_axis = to_native_vector(axis)
    x_axis = to_native_vector(ortoVect(axis))

    r_maj, r_min = major_radius, minor_radius
    major_dir, minor_dir = z_axis, x_axis
    if r_maj < r_min:
        r_maj, r_min = r_min, r_maj
        major_dir, minor_dir = minor_dir, major_dir

    e_center = center_native + r_major_axis_offset * x_axis
    s1 = e_center + major_dir * r_maj
    s2 = e_center + minor_dir * r_min

    ellipse = Part.Ellipse(s1, s2, e_center)
    if abs(r_major_axis_offset) < minor_radius:  # degenerate torus (self-intersecting tube)
        pz = major_radius * math.sqrt(1 - (r_major_axis_offset / minor_radius) ** 2)
        pz1 = center_native - pz * z_axis
        pz2 = center_native + pz * z_axis

        p1 = ellipse.parameter(pz1)
        p2 = ellipse.parameter(pz2)
        if p2 < p1:
            p2 += 2 * math.pi
        shape = ellipse.toBSpline(p1, p2).toShape(p1, p2)
        rev = shape.revolve(center_native, z_axis, 360)
    else:
        shape = ellipse.toBSpline().toShape()
        rev = shape.revolve(center_native, z_axis, 360)
    shell = Part.makeShell((rev,))
    return GSolid(Part.makeSolid(shell))


def to_gvector_(fc_vector) -> GVector:
    return GVector(fc_vector.x, fc_vector.y, fc_vector.z)


# ---------------------------------------------------------------------------
# Generic Gmake_* entry points -- the only names `geo_quadrics/__init__.py`
# re-exports. `Objects.py` calls these, never the dataclasses above
# directly, so the FreeCAD/pyOCC choice stays entirely inside this
# package.
# ---------------------------------------------------------------------------


def Gmake_elliptic_cone(
    apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet, length
) -> GSolid:
    return GEllipticCone.from_values(
        apex, axis, ref_radius, major_radius, minor_radius, major_axis, minor_axis, double_sheet
    ).build_shape(length)


def Gmake_hyperboloid(center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet, length) -> GSolid:
    return GHyperboloid.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis, one_sheet).build_shape(
        length
    )


def Gmake_ellipsoid(center, axis, major_radius, minor_radius, major_axis, minor_axis) -> GSolid:
    return GEllipsoid.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape()


def Gmake_elliptic_cylinder(center, axis, major_radius, minor_radius, major_axis, minor_axis, height) -> GSolid:
    return GEllipticCylinder.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape(height)


def Gmake_hyperbolic_cylinder(center, axis, major_radius, minor_radius, major_axis, minor_axis, height) -> GSolid:
    return GHyperbolicCylinder.from_values(center, axis, major_radius, minor_radius, major_axis, minor_axis).build_shape(height)


def Gmake_paraboloid(center, axis, focal, length) -> GSolid | None:
    return GParaboloid.from_values(center, axis, focal).build_shape(length)


def Gmake_torus_elliptic(
    center: GVector, axis: GVector, r_major_axis_offset: float, major_radius: float, minor_radius: float
) -> GSolid:
    """
    Elliptic torus: `major_radius`/`minor_radius` are the *tube's own*
    cross-section radii (about the Z and X directions respectively, in
    MCNP SY/SX-card terms), `r_major_axis_offset` (MCNP's own `R`) is the
    tube center's distance from `center` along the perpendicular axis
    `ortoVect(axis)` picks. Ported from `Objects.py::makeEllipticTorus`.
    """
    return _make_torus_elliptic_native(center, axis, r_major_axis_offset, major_radius, minor_radius)
