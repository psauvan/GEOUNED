"""
geo/surface_geometry.py

Split out of `vector_geometry.py` (2026-08-28) -- this is the file that
was originally meant by "the geometric predicates created at the start
of this migration," grown over time with more analytic-surface
operations added as GEOUNED needed them. Still pure math: no
`Part`/`FreeCAD`/`OCC.Core` import here, only `GVector` arithmetic and
the fields of the analytic surface descriptors (`GPlane`/`GCylinder`/
`GCone`/`GSphere`/`GTorus`) -- the functions below are duck-typed on
those fields (`.Axis`/`.Position`/`.Center`/`.Radius`/`.Apex`/
`.SemiAngle`/...), so they work identically whether called on a real
`geo` descriptor or on GEOUNED's own Tier-1 `*OnlyParams` (`PlaneParams`,
`CylinderOnlyParams`, ...), which store the same fields under the same
names since the Tier-1 GVector-storage migration.

Everything here operates on individual surfaces/points, never on a whole
solid -- see `solid_defects.py` for the load-time, whole-`GSolid` CAD-
defect detection this file does not do.
"""

from __future__ import annotations

import math

from .constants import (
    NUMERIC_DOUBLE_TOL,
    REL_TOL_E2,
    REL_TOL_E3,
    RELATIVE_TOL_ABS_FLOOR,
)
from .vector_geometry import GVector


def relative_tolerance(base: float, scale: float) -> float:
    """`base * scale`, never below `RELATIVE_TOL_ABS_FLOOR` (see that constant: `scale` is 0 for a surface anchored
    at the origin). Shared by every `is_same_*_surface` predicate's own `relativeTol` handling, and by a caller that
    needs the same effective tolerance for a diagnostic (e.g. the registry's fuzzy-match log)."""
    return max(base * scale, RELATIVE_TOL_ABS_FLOOR)

# ---------------------------------------------------------------------------
# Basic geometric predicates on GVector
# ---------------------------------------------------------------------------


def is_same_value(v1: float, v2: float, tolerance: float = NUMERIC_DOUBLE_TOL) -> bool:
    return abs(v1 - v2) < tolerance


# ---------------------------------------------------------------------------
# "Is this the same underlying analytic surface" predicates
# ---------------------------------------------------------------------------

def axes_parallel(axis_1: GVector, axis_2: GVector, angle_tol: float) -> bool:
    """True if two vectors lie along the same line, either direction (angle 0 or pi), to within `angle_tol` radians."""
    angle = axis_1.angle_to(axis_2)
    return min(angle, math.pi - angle) <= angle_tol


def axes_same_direction(axis_1: GVector, axis_2: GVector, angle_tol: float) -> bool:
    """True if two vectors point the same way (angle 0), to within `angle_tol` radians."""
    return axis_1.angle_to(axis_2) <= angle_tol


def axes_perpendicular(axis_1: GVector, axis_2: GVector, angle_tol: float) -> bool:
    """True if two vectors are perpendicular, to within `angle_tol` radians -- the same tolerance as `axes_parallel`.

    Perpendicular means alpha = +-pi/2 (both give cos(alpha) = 0), hence `abs(pi/2 - abs(alpha))`. `angle_to` is
    unsigned, in [0, pi], so both orientations arrive as +pi/2 and the inner `abs` only matters for an oriented angle."""
    alpha = axis_1.angle_to(axis_2)
    return abs(math.pi / 2.0 - abs(alpha)) <= angle_tol


def opposite_sense(axis_1: GVector, axis_2: GVector) -> bool:
    """True if two axes that are ALREADY known to lie along the same line point in opposite senses.

    A sign, not a tolerance: once two surfaces are recognised as the same one, whether their axes point the same way
    or the opposite way is the sign of the dot product. Deciding it with an angle tolerance can disagree with the
    tolerance that recognised them as parallel (a plane matched with `add_pln_angle`, 1e-2, whose sense is then
    checked with `pln_angle`, 1e-4, could come out with the wrong sign)."""
    return axis_1.dot(axis_2) < 0.0


_CARTESIAN_AXES = (GVector(1, 0, 0), GVector(0, 1, 0), GVector(0, 0, 1))


def axis_alignment(direction: GVector, angle_tol: float, same_direction: bool = False) -> "int | None":
    """Index (0=X, 1=Y, 2=Z) of the cartesian axis `direction` is aligned to within `angle_tol`, or `None` if it
    isn't aligned with any of them -- the one classification every surface writer (mcnp/openmc/serpent/phits) makes
    before choosing an axis-specific surface card (`PX`/`CX`/`KX`/`TX`, ...) over the general quadric/`GQ` fallback,
    unified 2026-09-22 (they used to each run the same 3-way `if/elif/elif` themselves).

    `same_direction=True` (Plane only: which way the normal points decides which card a plane is classified into,
    not just its formatted text) requires the SAME sense (`axes_same_direction`); the default (Cylinder/Cone/Torus,
    where only the axis LINE matters, not which way a vector happens to point along it) accepts either sense
    (`axes_parallel`). Confirmed consistent across all 4 writers before unifying: every one of them already used
    `axes_same_direction` for Plane and `axes_parallel` for Cylinder/Cone/Torus."""
    test = axes_same_direction if same_direction else axes_parallel
    for index, axis in enumerate(_CARTESIAN_AXES):
        if test(direction, axis, angle_tol):
            return index
    return None


def plane_offset(plane_1, plane_2) -> float:
    """Distance `is_same_plane_surface` compares -- see that function's docstring for why it is measured from a
    point of the plane rather than from the coordinate origin. The LARGER of the two planes' own axis: measured
    against BOTH `plane_1.Axis` and `plane_2.Axis` (not just `plane_1`'s), so the result does not depend on which
    argument is which -- swapping `plane_1`/`plane_2` gives the identical value. Meaningless (but still computable)
    when the two axes are not close to parallel; callers that need the decision use
    `is_same_plane_surface`/`plane_within`, not this value alone."""
    r12 = plane_2.Position - plane_1.Position
    d1 = abs(plane_1.Axis.dot(r12))
    d2 = abs(plane_2.Axis.dot(r12))
    return max(d1, d2)


def plane_within(plane_1, plane_2, angle_tol: float, distance_tol: float, relative_tol: bool = False) -> bool:
    """`is_same_plane_surface`, taking the two tolerances directly instead of reading `.pln_angle`/`.pln_distance`
    off a `Tolerances` object -- lets a caller apply a different pair (e.g. the registry's own `add_pln_angle`/
    `add_pln_distance`, for a non-real/auxiliary plane) without a tolerances-shaped proxy object."""
    if not axes_parallel(plane_1.Axis, plane_2.Axis, angle_tol):
        return False
    tol = distance_tol
    if relative_tol:
        scale = max(abs(plane_1.Axis.dot(plane_1.Position)), abs(plane_2.Axis.dot(plane_2.Position)))
        tol = relative_tolerance(distance_tol, scale)
    return plane_offset(plane_1, plane_2) <= tol


def is_same_plane_surface(plane_1, plane_2, tolerances) -> bool:
    """
    True if two planes are the same infinite analytic plane: axes parallel -- either way -- within `tolerances.pln_angle`,
    and the `Position` of `plane_2` no farther than `tolerances.pln_distance` from `plane_1`.

    The distance is measured from a POINT OF THE PLANE, not from the coordinate origin. A plane's `Position` lies in
    the region where its face is defined, so this compares the two planes where they are actually used. The offset
    from the origin (`Axis.dot(Position)`) would add `angle * distance-to-the-origin` to what is really a local
    difference (1e-4 rad at 1 m from the origin is 0.1 mm), and says nothing about where each face is.

    The distance is the LARGER of `|plane_1.Axis . (plane_2.Position - plane_1.Position)|` and
    `|plane_2.Axis . (plane_2.Position - plane_1.Position)|` (see `plane_offset`) -- symmetric in `plane_1`/`plane_2`
    (swapping the two arguments gives the identical value), and it does not depend on the senses of the axes (the
    same plane reached through two Face Orientations has antiparallel axes). It is also what makes two genuinely
    different parallel planes 3.5 units apart NOT match whatever their senses (the `d1 == d2` bug found on
    `Solidos/test_models/RoundCorners/rc9.stp`, 2026-08-23, which merged the two bounding planes of a round corner).

    With `tolerances.relativeTol` set (GEOUNED-only; absent/False for every other caller, including GEOReverse's bare
    `GeoTolerances`), the distance tolerance scales by the planes' own offset from the ORIGIN, not from each other --
    kept exactly as `basic_functions_part2.is_same_plane` computed it before this unification (2026-09-21: an
    unrelated scale reference to the new point-based distance, same class of open question as `is_same_cylinder_surface`'s
    own `relativeTol` docstring note; left as-is, not redesigned here).

    `tolerances` is a `GeoTolerances` (the user's own values): the decomposition and the output stage share one
    notion of "same surface", instead of this check keeping a fixed, private threshold. This is the ONE
    implementation: `GEOUNED.utils.basic_functions_part2.is_same_plane` (the CSG-registry entry point, which also
    needs `add_pln_*` for non-real planes and an optional near-miss diagnostic log) calls `plane_within` with the
    tolerances it selects, rather than keeping its own copy of this decision (unified 2026-09-22 after the registry's
    old, origin-based copy was found to disagree with this one -- see CLAUDE.md).
    """
    return plane_within(plane_1, plane_2, tolerances.pln_angle, tolerances.pln_distance, getattr(tolerances, "relativeTol", False))


def is_same_oriented_plane_surface(plane_1, plane_2, tolerances) -> bool:
    """True if two planes are the same infinite plane AND face the same way (`opposite_sense` is False).

    What `PlaneParams.__eq__` used to mean (removed: it had no access to the run's tolerances and was used implicitly by
    `in`, `.index`, `!=`), now explicit and with the user's `pln_distance` / `pln_angle`."""
    return is_same_plane_surface(plane_1, plane_2, tolerances) and not opposite_sense(plane_1.Axis, plane_2.Axis)


def is_parallel_plane_surface(plane_1, plane_2, tolerances) -> bool:
    """True if two planes have the same normal line (either way), to within `tolerances.pln_angle`."""
    return axes_parallel(plane_1.Axis, plane_2.Axis, tolerances.pln_angle)


def is_same_cylinder_surface(cylinder_1, cylinder_2, tolerances) -> bool:
    """True if two cylinders are the same infinite analytic cylinder (same
    radius, same axis line -- direction either way).

    A cylinder's `Center` is an *arbitrary point on its axis*: two
    representations of the same infinite cylinder can carry `Center`
    points anywhere along that shared axis. So the axis-line coincidence
    must be tested by the distance between the two *axis lines* -- the
    component of `(Center_1 - Center_2)` **perpendicular to the axis** --
    not the raw `|Center_1 - Center_2|`, which grows without bound as one
    `Center` slides along the axis. Confirmed as a real miss (2026-08-29,
    an `L4_body.stp` decomposition fragment): two faces of the identical
    R=7 cylinder whose `Center`s were 29.97 units apart *along the axis*
    (perpendicular distance ~2e-12) were wrongly judged different
    surfaces, so `Gsliver_heal` could not recognise the malformed
    duplicate cylinder face it needed to drop.

    Radius and axis-line distance are compared with `tolerances.cyl_distance`,
    the axis direction with `tolerances.cyl_angle`. With `tolerances.relativeTol` set (GEOUNED-only; see
    `is_same_plane_surface`'s own note), each scales by the LARGER of the two cylinders' own `Radius`/`Center.length`
    -- `Center` is an arbitrary point of the axis, so that second scale is itself arbitrary; kept as-is, not
    redesigned here (2026-09-21 user decision, same open question as `is_same_plane_surface`'s)."""
    relative_tol = getattr(tolerances, "relativeTol", False)
    if not cylinder_radius_within(cylinder_1, cylinder_2, tolerances.cyl_distance, relative_tol):
        return False
    if not axes_parallel(cylinder_1.Axis, cylinder_2.Axis, tolerances.cyl_angle):
        return False
    return cylinder_axis_within(cylinder_1, cylinder_2, tolerances.cyl_distance, relative_tol)


def cylinder_radius_diff(cylinder_1, cylinder_2) -> float:
    """`cylinder_2.Radius - cylinder_1.Radius` (signed): the quantity `is_same_cylinder_surface` compares."""
    return cylinder_2.Radius - cylinder_1.Radius


def cylinder_radius_within(cylinder_1, cylinder_2, distance_tol: float, relative_tol: bool = False) -> bool:
    """The radius half of `is_same_cylinder_surface`, taking the tolerance directly -- see `plane_within`."""
    tol = distance_tol
    if relative_tol:
        tol = relative_tolerance(distance_tol, max(cylinder_1.Radius, cylinder_2.Radius))
    return abs(cylinder_radius_diff(cylinder_1, cylinder_2)) <= tol


def cylinder_axis_offset(cylinder_1, cylinder_2) -> float:
    """Perpendicular distance between the two cylinders' axis LINES (see `is_same_cylinder_surface`'s own docstring
    for why not the raw `Center` distance) -- the quantity `is_same_cylinder_surface` compares once the axes are
    already known to be parallel."""
    offset = cylinder_1.Center - cylinder_2.Center
    perpendicular = offset - cylinder_1.Axis * offset.dot(cylinder_1.Axis)
    return perpendicular.length


def cylinder_axis_within(cylinder_1, cylinder_2, distance_tol: float, relative_tol: bool = False) -> bool:
    """The axis-line half of `is_same_cylinder_surface`, taking the tolerance directly -- see `plane_within`."""
    tol = distance_tol
    if relative_tol:
        tol = relative_tolerance(distance_tol, max(cylinder_1.Center.length, cylinder_2.Center.length))
    return cylinder_axis_offset(cylinder_1, cylinder_2) <= tol


def is_same_cone_surface(cone_1, cone_2, tolerances) -> bool:
    """True if two cones are the same infinite cone: `SemiAngle` and axis line within `tolerances.kne_angle`,
    apex within `tolerances.kne_distance`. With `tolerances.relativeTol` set (GEOUNED-only; see
    `is_same_plane_surface`'s own note), the apex tolerance scales by the larger of the two apexes' own distance
    from the origin -- kept as-is, not redesigned here."""
    if abs(cone_1.SemiAngle - cone_2.SemiAngle) > tolerances.kne_angle:
        return False
    apex_tol = tolerances.kne_distance
    if getattr(tolerances, "relativeTol", False):
        apex_tol = relative_tolerance(tolerances.kne_distance, max(cone_1.Apex.length, cone_2.Apex.length))
    if (cone_1.Apex - cone_2.Apex).length > apex_tol:
        return False
    return axes_parallel(cone_1.Axis, cone_2.Axis, tolerances.kne_angle)


def is_coaxial_cone_pair(cone_1, cone_2, tolerances) -> bool:
    """True when two cones share the same axis *line* (their axis vectors
    may be parallel or anti-parallel -- direction is not constrained) and
    the same `SemiAngle`, but sit at *different* apexes along that line.

    This is the exact "two coaxial cones pointing toward or away from each
    other" configuration whose analytic intersection degenerates into a
    single circle instead of a generic space curve, which is what makes it
    a hard case for a general quadric-quadric BOP solver (see `Gsplit`'s
    coaxial-cone fallback in `_occ_impl.py`/`_ocp_impl.py`).

    The complement of `is_same_cone_surface` (which additionally requires
    a matching apex *and* matching axis direction, i.e. literally the same
    infinite cone): this predicate explicitly requires the apex to differ
    and tolerates either axis direction along the shared line.
    """
    if abs(cone_1.SemiAngle - cone_2.SemiAngle) > tolerances.kne_angle:
        return False
    if not axes_parallel(cone_1.Axis, cone_2.Axis, tolerances.kne_angle):
        return False
    apex_offset = cone_2.Apex - cone_1.Apex
    if apex_offset.length < tolerances.kne_distance:
        return False  # same apex -> the same cone entirely, not a pair
    along = apex_offset.dot(cone_1.Axis)
    radial = (apex_offset - cone_1.Axis * along).length
    return radial < tolerances.kne_distance


def is_coaxial_cone_cylinder_pair(cone, cylinder, tolerances, semiangle_min: float = NUMERIC_DOUBLE_TOL) -> bool:
    """True when `cone` and `cylinder` share the same axis *line* (either
    axis direction) and `cone`'s SemiAngle is non-degenerate (not exactly
    0 or 90 degrees), so the cone genuinely reaches `cylinder.Radius` at
    exactly one height along its own axis -- an analytic certainty for
    any coaxial cone/cylinder pair, not a coincidence.

    This is the cone/cylinder counterpart of `is_coaxial_cone_pair`: when
    the real solid's own boundary is also *tangent* along that one circle
    (e.g. a cone built to blend smoothly into a cylinder of the same
    radius), the circle is the same kind of degenerate quadric-quadric
    intersection a general BOP solver struggles with -- confirmed live on
    a real fixture where a cutting cone, a cylinder, and a second cone all
    shared one exact circle simultaneously (`Gsplit`'s coaxial-cone
    fallback in `_occ_impl.py`/`_ocp_impl.py` only searched for a second
    *cone*, missing this case entirely).
    """
    if abs(math.tan(cone.SemiAngle)) < semiangle_min:
        return False
    if not axes_parallel(cone.Axis, cylinder.Axis, tolerances.kne_angle):
        return False
    offset = cylinder.Center - cone.Apex
    along = offset.dot(cone.Axis)
    radial = (offset - cone.Axis * along).length
    return radial < tolerances.kne_distance


def is_same_sphere_surface(sphere_1, sphere_2, tolerances) -> bool:
    """True if two spheres coincide: radius and centre within `tolerances.sph_distance`. With
    `tolerances.relativeTol` set (GEOUNED-only; see `is_same_plane_surface`'s own note), both scale by the larger of
    the two spheres' own radius/centre-distance-from-the-origin -- kept as-is, not redesigned here."""
    relative_tol = getattr(tolerances, "relativeTol", False)
    radius_tol = tolerances.sph_distance
    if relative_tol:
        radius_tol = relative_tolerance(tolerances.sph_distance, max(sphere_1.Radius, sphere_2.Radius))
    if abs(sphere_1.Radius - sphere_2.Radius) > radius_tol:
        return False
    centre_tol = tolerances.sph_distance
    if relative_tol:
        centre_tol = relative_tolerance(tolerances.sph_distance, max(sphere_1.Center.length, sphere_2.Center.length))
    return (sphere_1.Center - sphere_2.Center).length <= centre_tol


def is_same_torus_surface(torus_1, torus_2, tolerances, check_a_sign: bool = False) -> bool:
    """True if two tori coincide: both radii and the centre within `tolerances.tor_distance`, the axis LINE (either
    direction -- two antiparallel-axis tori are the same torus, 2026-09-21 user decision) within `tolerances.tor_angle`.

    `check_a_sign` opts into one extra, narrower requirement: the two tori's own `a_sign` (which sheet of a
    self-intersecting/degenerate torus they represent, see `write/functions.py`) must also match. Two genuinely
    distinct uses of "same torus" need different answers here: grouping same-analytic-surface face fragments during
    decomposition (`SolidGu.same_torus_surf`, `check_a_sign=True`) must NOT merge a self-intersecting torus's outer
    and inner sheets, since they are geometrically distinct faces; CSG-surface registration
    (`MetaSurfacesDict`/`SurfacesDict`'s `add_torus`/`get_id`, the default `check_a_sign=False`) deliberately does
    NOT -- both sheets of one degenerate torus are written as a single MCNP/OpenMC/etc surface (the sign is encoded
    into the written major radius instead), so they must compare equal there. With `tolerances.relativeTol` set
    (GEOUNED-only; see `is_same_plane_surface`'s own note), the radii and centre tolerances scale by the larger of
    the two tori's own major/minor radius or centre-distance-from-the-origin -- kept as-is, not redesigned here."""
    if not axes_parallel(torus_1.Axis, torus_2.Axis, tolerances.tor_angle):
        return False
    if check_a_sign and getattr(torus_1, "a_sign", 1) != getattr(torus_2, "a_sign", 1):
        return False
    relative_tol = getattr(tolerances, "relativeTol", False)
    major_tol = minor_tol = tolerances.tor_distance
    if relative_tol:
        major_tol = relative_tolerance(tolerances.tor_distance, max(torus_1.MajorRadius, torus_2.MajorRadius))
        minor_tol = relative_tolerance(tolerances.tor_distance, max(torus_1.MinorRadius, torus_2.MinorRadius))
    if abs(torus_1.MajorRadius - torus_2.MajorRadius) > major_tol:
        return False
    if abs(torus_1.MinorRadius - torus_2.MinorRadius) > minor_tol:
        return False
    centre_tol = tolerances.tor_distance
    if relative_tol:
        centre_tol = relative_tolerance(tolerances.tor_distance, max(torus_1.Center.length, torus_2.Center.length))
    return (torus_1.Center - torus_2.Center).length <= centre_tol


# ---------------------------------------------------------------------------
# Point-classification predicates ("is `point` outside this analytic
# surface"). Free functions, duck-typed on .Axis/.Position/.Center/etc,
# so they work identically whether called on a `geo` descriptor (GPlane,
# GCylinder, ...) or on GEOUNED's own Tier-1 *OnlyParams (PlaneParams,
# CylinderOnlyParams, ...) -- both store the same fields under the same
# names since the Tier-1 GVector-storage migration. This is what lets
# `boolean_solids.check_sign_primitive` (a decomposition-time hot path,
# operating on *OnlyParams) share these formulas with `GPlane.is_inside`
# etc instead of duplicating them, with no transient object construction
# on either side.
# ---------------------------------------------------------------------------


def is_inside_plane(point: GVector, plane) -> bool:
    return plane.Axis.dot(point - plane.Position) > 0


def is_inside_cylinder(point: GVector, cylinder) -> bool:
    r = point - cylinder.Center
    z = cylinder.Axis.dot(r)
    return (r.length * r.length - z * z) > cylinder.Radius * cylinder.Radius


def is_inside_cone(point: GVector, cone) -> bool:
    r = (point - cone.Apex).normalized()
    # rounded to 15 decimals before acos: guards against a just-past-1.0
    # float rounding error (acos would otherwise raise on a point exactly
    # on the axis).
    z = round(cone.Axis.dot(r), 15)
    alpha = math.acos(z)
    return alpha > cone.SemiAngle


def is_inside_sphere(point: GVector, sphere) -> bool:
    return (point - sphere.Center).length > sphere.Radius


def is_inside_torus(point: GVector, torus) -> bool:
    r = point - torus.Center
    h = r.dot(torus.Axis)
    rho = r - h * torus.Axis
    rp = math.sqrt((rho.length - torus.MajorRadius) ** 2 + h**2)
    return rp > torus.MinorRadius


def torus_sheet_sign(vertex: GVector, torus) -> int:
    """For a self-intersecting (degenerate: MinorRadius > MajorRadius)
    torus, +1 if `vertex` lies on the ordinary outer sheet, -1 if on the
    pinched, self-intersecting inner sheet -- these are two genuinely
    distinct subsets of the point set satisfying the torus's own implicit
    equation (folded through the axis where the naive tube radius would
    go negative), not just two ways of naming the same surface. Always +1
    for a non-degenerate torus, where the two sheets coincide.

    Derivation: `radial` is the true (always non-negative) radial
    direction of `vertex` itself, read directly off its real 3D position
    -- not derived from the torus's own (u, v) parametrization, which is
    exactly what folds over and becomes ambiguous on the inner sheet.
    `radial * MajorRadius` is therefore the point on the torus's own
    major (central) circle at `vertex`'s true azimuth; the true tube
    radius at that azimuth is the distance from there to `vertex`. On the
    outer sheet this distance is always MinorRadius (matching the
    standard parametrization); on the inner sheet it isn't, since the
    inner sheet's parametrization has its major-circle offset applied in
    the *opposite* azimuthal direction from where the point actually
    sits.
    """
    r = vertex - torus.Center
    outer = r.length > math.sqrt(torus.MinorRadius**2 - torus.MajorRadius**2)
    return 1 if outer else -1


# ---------------------------------------------------------------------------
# Face parametrization (value/tangent at (u, v)), matching FreeCAD/OCCT's
# own analytic conventions exactly
# ---------------------------------------------------------------------------


def _require_x_dir(surface) -> GVector:
    if surface.XDir is None:
        raise ValueError(
            f"{type(surface).__name__}.XDir is required to evaluate value_at/tangent_at analytically "
            "(this instance was reconstructed without a native face, so its (u, v) reference "
            "direction is unknown)"
        )
    return surface.XDir


def plane_value_at(plane, u: float, v: float) -> GVector:
    """
    Point on the infinite plane at parametric coordinates (u, v), matching
    FreeCAD/OCCT's own Plane parametrization exactly (Position + u*XDir +
    v*YDir, YDir = Axis x XDir) -- not just a geometrically-equivalent
    plane with an arbitrary (u, v) origin/orientation.
    """
    x_dir = _require_x_dir(plane)
    y_dir = plane.Axis.cross(x_dir)
    return plane.Position + x_dir * u + y_dir * v


def plane_tangent_at(plane, u: float, v: float) -> tuple[GVector, GVector]:
    """Unit tangent directions (d/du, d/dv) on the plane -- constant everywhere, same convention as `plane_value_at`."""
    x_dir = _require_x_dir(plane)
    y_dir = plane.Axis.cross(x_dir)
    return x_dir, y_dir


def cylinder_value_at(cylinder, u: float, v: float) -> GVector:
    """
    Point on the infinite cylinder at parametric coordinates (u, v),
    matching FreeCAD/OCCT's own Cylinder parametrization exactly (u is the
    angle from XDir toward YDir = Axis x XDir, v is the distance along
    Axis) -- not just a geometrically-equivalent cylinder with an
    arbitrary angular origin.
    """
    x_dir = _require_x_dir(cylinder)
    y_dir = cylinder.Axis.cross(x_dir)
    radial = x_dir * math.cos(u) + y_dir * math.sin(u)
    return cylinder.Center + radial * cylinder.Radius + cylinder.Axis * v


def cylinder_tangent_at(cylinder, u: float, v: float) -> tuple[GVector, GVector]:
    """Unit tangent directions (d/du, d/dv) on the cylinder, same convention as `cylinder_value_at`."""
    x_dir = _require_x_dir(cylinder)
    y_dir = cylinder.Axis.cross(x_dir)
    tangent_u = y_dir * math.cos(u) - x_dir * math.sin(u)
    return tangent_u, cylinder.Axis


# ---------------------------------------------------------------------------
# Can/RoundCorner closing-plane derivation (analytic, closed-form)
# ---------------------------------------------------------------------------


def _solve_quadratic(a: float, b: float, c: float) -> tuple[float, float] | None:
    """Real roots of a*t^2 + b*t + c = 0, ordered (smaller, larger).
    None if there are 0 real roots, or if the equation degenerates to
    non-quadratic (a ~ 0) -- the caller needs a genuine entry/exit pair,
    not a single crossing."""
    if abs(a) < NUMERIC_DOUBLE_TOL:
        return None
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    r1 = (-b - sq) / (2.0 * a)
    r2 = (-b + sq) / (2.0 * a)
    return (r1, r2) if r1 <= r2 else (r2, r1)


def find_can_plane(
    main_center: GVector,
    main_axis: GVector,
    main_radius: float,
    kind: str,
    secondary,
    n_angles: int = 720,
    narrow_wide_threshold: float | None = None,
) -> tuple[GVector, GVector] | None:
    """Compute the disambiguating plane for a Can/RoundCorner-style
    composite surface: a main cylinder split into two disjoint pieces by
    a secondary surface (`kind` one of "cylinder"/"cone"/"sphere",
    `secondary` a GCylinder/GCone/GSphere-like descriptor).

    The two (infinite) analytic surfaces meet along two closed tangency
    contours on the main cylinder -- for each angle phi around the main
    cylinder's own circumference, the line along its axis crosses the
    secondary surface's boundary at up to two points (a quadratic in the
    axial parameter t, for all three secondary types). The near contour
    is where each such line *enters* the secondary surface, the far
    contour where it *exits*; the free zone -- where a plane can sit
    without cutting either the "outside secondary" or "inside secondary"
    real material -- lies strictly between the near contour's own
    furthest-advanced point and the far contour's own furthest-back
    point, projected onto the plane's normal.

    Returns None if either contour fails to close over the full angular
    range (no real roots, or -- for a cone secondary -- a root on the
    wrong nappe -- at some phi): the main cylinder isn't actually split
    into two disjoint pieces by this secondary surface, so the premise
    for a Can doesn't hold at all.

    The plane's normal is not always main_axis: for a Cylinder/Cone
    secondary (which has its own axis), it's the direction perpendicular
    to the secondary's axis within the plane containing both axes --
    the natural cross-cutting direction where the two axes are close to
    parallel, not a cut transverse to the main cylinder's own length.
    Only a Sphere secondary (no axis of its own) uses main_axis directly.

    Once the free zone is found, its position is either the midpoint (if
    narrow) or a small step past the near contour capped at
    narrow_wide_threshold (if wide, so the plane never drifts far from
    the real local feature just because the analytic zone is huge) --
    see the docstring-adjacent discussion in functions.py::build_can_params
    for why both regimes are needed.
    """
    A = main_axis.normalized()
    e2 = A.cross(GVector(1.0, 0.0, 0.0))
    if e2.length < NUMERIC_DOUBLE_TOL:
        e2 = A.cross(GVector(0.0, 1.0, 0.0))
    e2 = e2.normalized()
    e1 = A.cross(e2).normalized()

    if kind == "cylinder":
        secondary_axis = secondary.Axis.normalized()
        a_dot_as = A.dot(secondary_axis)
        radius2 = secondary.Radius * secondary.Radius
        reference = secondary.Center

        def coeffs(u: GVector) -> tuple[float, float, float]:
            a = 1.0 - a_dot_as * a_dot_as
            u_a = u.dot(A)
            u_as = u.dot(secondary_axis)
            b = 2.0 * (u_a - u_as * a_dot_as)
            c = u.dot(u) - u_as * u_as - radius2
            return a, b, c

        def root_ok(u: GVector, t: float) -> bool:
            return True

    elif kind == "cone":
        secondary_axis = secondary.Axis.normalized()
        a_dot_ac = A.dot(secondary_axis)
        cos2 = math.cos(secondary.SemiAngle) ** 2
        reference = secondary.Apex

        def coeffs(u: GVector) -> tuple[float, float, float]:
            p_ = u.dot(secondary_axis)
            a = a_dot_ac * a_dot_ac - cos2
            b = 2.0 * (p_ * a_dot_ac - cos2 * u.dot(A))
            c = p_ * p_ - cos2 * u.dot(u)
            return a, b, c

        def root_ok(u: GVector, t: float) -> bool:
            # single-nappe cone: only the side u.dot(axis) + t*(A.dot(axis)) >= 0
            # (matching is_inside_cone's own convention) is physically real.
            return (u.dot(secondary_axis) + t * a_dot_ac) >= 0.0

    elif kind == "sphere":
        secondary_axis = None
        radius2 = secondary.Radius * secondary.Radius
        reference = secondary.Center

        def coeffs(u: GVector) -> tuple[float, float, float]:
            a = 1.0
            b = 2.0 * u.dot(A)
            c = u.dot(u) - radius2
            return a, b, c

        def root_ok(u: GVector, t: float) -> bool:
            return True

    else:
        raise ValueError(f"find_can_plane: unknown secondary kind {kind!r}")

    if secondary_axis is None:
        normal = A
    else:
        n_common = A.cross(secondary_axis)
        if n_common.length < NUMERIC_DOUBLE_TOL:
            normal = A  # axes (near-)parallel -- common plane undefined
        else:
            normal = secondary_axis.cross(n_common.normalized()).normalized()

    best_near = (-math.inf, None)  # (projection onto normal, point)
    best_far = (math.inf, None)
    for i in range(n_angles):
        phi = 2.0 * math.pi * i / n_angles
        offset = e1 * (main_radius * math.cos(phi)) + e2 * (main_radius * math.sin(phi))
        u = (main_center - reference) + offset

        roots = _solve_quadratic(*coeffs(u))
        if roots is None:
            return None
        t_near, t_far = roots
        if not (root_ok(u, t_near) and root_ok(u, t_far)):
            return None

        p_near = main_center + A * t_near + offset
        p_far = main_center + A * t_far + offset
        proj_near = p_near.dot(normal)
        proj_far = p_far.dot(normal)
        if proj_near > best_near[0]:
            best_near = (proj_near, p_near)
        if proj_far < best_far[0]:
            best_far = (proj_far, p_far)

    proj_near, point_near = best_near
    proj_far, point_far = best_far
    if proj_far <= proj_near:
        return None

    if narrow_wide_threshold is None:
        narrow_wide_threshold = REL_TOL_E2 * main_radius

    half_width = 0.5 * (proj_far - proj_near)
    if half_width <= narrow_wide_threshold:
        offset_amount = half_width
    else:
        offset_amount = min(REL_TOL_E3 * half_width, narrow_wide_threshold)

    position = point_near + normal * offset_amount
    return position, normal


def convex_planes(plane_list, zaxis, closed) -> tuple[bool, str]:
    center = plane_list[0].Surf.Position
    for p in plane_list[1:]:
        center = center + p.Surf.Position
    center = center / len(plane_list)

    ref = plane_list[0].Surf.Position - center
    ref = (ref - zaxis * ref.dot(zaxis)).normalized()
    orientation = "Forward" if plane_list[0].Surf.Axis.dot(ref) < 0 else "Reversed"

    if len(plane_list) < 3:
        return True, orientation

    angles = []
    for i, p in enumerate(plane_list):
        rp = p.Surf.Position - center
        rp = (rp - zaxis * rp.dot(zaxis)).normalized()
        cosa = ref.dot(rp)
        cross = ref.cross(rp)
        sina = cross.length
        if cross.dot(zaxis) < 0:
            sina = -sina
        angle = math.atan2(sina, cosa)
        while angle < 0:
            angle += 2 * math.pi
        angles.append((angle, i))

    angles.sort()

    if closed:
        p0 = plane_list[angles[-1][1]].Surf.Axis
        p1 = plane_list[angles[0][1]].Surf.Axis
        start = 1
    else:
        p0 = plane_list[angles[0][1]].Surf.Axis
        p1 = plane_list[angles[1][1]].Surf.Axis
        start = 2

    signref = zaxis.dot(p0.cross(p1))

    p0 = p1
    convex = True
    for a, i in angles[start:]:
        p1 = plane_list[i].Surf.Axis
        sign = zaxis.dot(p0.cross(p1))
        if signref * sign < 0:
            convex = False
            break
        p0 = p1

    # if set is open the ref point may be the final
    # point of the plane sequence, so move to the nextpoint
    # to check whether convex is false because end point was selected
    if not convex and not closed:
        angles.append(angles[0])
        del angles[0]
        p0 = plane_list[angles[0][1]].Surf.Axis
        p1 = plane_list[angles[1][1]].Surf.Axis
        signref = zaxis.dot(p0.cross(p1))

        p0 = p1
        convex = True
        for a, i in angles[2:]:
            p1 = plane_list[i].Surf.Axis
            sign = zaxis.dot(p0.cross(p1))
            if signref * sign < 0:
                convex = False
                break
            p0 = p1

    return convex, orientation
