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

from .vector_geometry import GVector

# ---------------------------------------------------------------------------
# Basic geometric predicates on GVector
# ---------------------------------------------------------------------------


def is_same_value(v1: float, v2: float, tolerance: float = 1e-6) -> bool:
    return abs(v1 - v2) < tolerance


def is_opposite(vector_1: GVector, vector_2: GVector, tolerance: float = 1e-3) -> bool:
    return vector_1.angle_to(-vector_2) < tolerance


def is_parallel(vector_1: GVector, vector_2: GVector, tolerance: float = 1e-3) -> bool:
    angle = vector_1.angle_to(vector_2)
    return angle < tolerance or is_same_value(angle, math.pi, tolerance)


def is_in_line(point: GVector, direction: GVector, point_on_line: GVector, tolerance: float = 1e-6) -> bool:
    to_point = point - point_on_line
    return is_parallel(direction, to_point) or to_point.length < tolerance


def is_in_plane(point: GVector, plane, tolerance: float = 1e-7) -> bool:
    return abs(plane.Axis.dot(point - plane.Position)) < tolerance


def sign_plane(point: GVector, plane) -> int:
    return 1 if plane.Axis.dot(point - plane.Position) >= 0.0 else -1


# ---------------------------------------------------------------------------
# "Is this the same underlying analytic surface" predicates
# ---------------------------------------------------------------------------


def is_same_plane_surface(plane_1, plane_2) -> bool:
    """
    True if two planes are the same infinite analytic plane (same axis
    direction -- either way -- and same offset from the origin). Direct
    port of the former `PlaneGu.isSameSurface`, kept at the same fixed
    tolerances: this is a decomposition-time "is this literally the same
    underlying surface" check, distinct from `is_same_plane` in
    `basic_functions_part2.py`, which compares already-built output
    surfaces with user-configurable tolerances for a different purpose
    (surface-list deduplication).

    Each plane's own offset (`Axis.dot(Position)`) is measured along its
    *own* axis -- when the two axes are antiparallel (opposite direction,
    still the same infinite plane, e.g. the same real plane reached via
    two different Face Orientations), those two offsets are measured in
    opposite directions and must be compared via `d1 == -d2`, not
    `d1 == d2` (confirmed as a real bug via `Solidos/test_models/
    RoundCorners/rc9.stp`, 2026-08-23: two genuinely different, parallel
    planes 3.5 units apart with antiparallel axes have equal-magnitude
    offsets of the same sign, which `d1 == d2` alone wrongly matched --
    `MetaSurfacesDict`/`build_roundC_params` then treated one cylinder's
    own 2 distinct bounding planes as a single coincident one, losing its
    real additional/closing plane entirely).
    """
    axis_dot = plane_1.Axis.dot(plane_2.Axis)
    if abs(axis_dot) < 0.99999:
        return False
    d1 = plane_1.Axis.dot(plane_1.Position)
    d2 = plane_2.Axis.dot(plane_2.Position)
    if axis_dot > 0:
        return abs(d1 - d2) <= 1e-5
    else:
        return abs(d1 + d2) <= 1e-5


def is_parallel_plane_surface(plane_1, plane_2) -> bool:
    """Direct port of the former `PlaneGu.isParallel`, same fixed tolerance."""
    return abs(plane_1.Axis.dot(plane_2.Axis)) > 0.99999


def is_same_cylinder_surface(cylinder_1, cylinder_2) -> bool:
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
    duplicate cylinder face it needed to drop. Was a direct port of the
    former `CylinderGu.isSameSurface`; same fixed 1e-5 tolerances."""
    if abs(cylinder_1.Radius - cylinder_2.Radius) > 1e-5:
        return False
    if abs(cylinder_1.Axis.dot(cylinder_2.Axis)) < 0.99999:
        return False
    offset = cylinder_1.Center - cylinder_2.Center
    perpendicular = offset - cylinder_1.Axis * offset.dot(cylinder_1.Axis)
    return perpendicular.length <= 1e-5


def is_same_cone_surface(cone_1, cone_2) -> bool:
    """Direct port of the former `ConeGu.isSameSurface`, same fixed tolerances."""
    if abs(cone_1.SemiAngle - cone_2.SemiAngle) > 1e-5:
        return False
    if (cone_1.Apex - cone_2.Apex).length > 1e-5:
        return False
    return abs(cone_1.Axis.dot(cone_2.Axis)) >= 0.99999


def is_coaxial_cone_pair(
    cone_1, cone_2, angle_tol: float = 1e-6, axis_dot_tol: float = 1e-5, apex_line_tol: float = 1e-5
) -> bool:
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
    if abs(cone_1.SemiAngle - cone_2.SemiAngle) > angle_tol:
        return False
    if abs(cone_1.Axis.dot(cone_2.Axis)) < 1.0 - axis_dot_tol:
        return False
    apex_offset = cone_2.Apex - cone_1.Apex
    if apex_offset.length < apex_line_tol:
        return False  # same apex -> the same cone entirely, not a pair
    along = apex_offset.dot(cone_1.Axis)
    radial = (apex_offset - cone_1.Axis * along).length
    return radial < apex_line_tol


def is_coaxial_cone_cylinder_pair(cone, cylinder, radial_tol: float = 1e-5, semiangle_min: float = 1e-6) -> bool:
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
    if abs(cone.Axis.dot(cylinder.Axis)) < 1.0 - 1e-5:
        return False
    offset = cylinder.Center - cone.Apex
    along = offset.dot(cone.Axis)
    radial = (offset - cone.Axis * along).length
    return radial < radial_tol


def is_same_sphere_surface(sphere_1, sphere_2) -> bool:
    """Direct port of the former `SphereGu.isSameSurface`, same fixed tolerance."""
    if abs(sphere_1.Radius - sphere_2.Radius) > 1e-5:
        return False
    return (sphere_1.Center - sphere_2.Center).length <= 1e-5


def is_same_torus_surface(torus_1, torus_2) -> bool:
    """Direct port of the former `TorusGu.isSameSurface`, same fixed tolerances."""
    if abs(torus_1.MajorRadius - torus_2.MajorRadius) > 1e-5:
        return False
    if abs(torus_1.MinorRadius - torus_2.MinorRadius) > 1e-5:
        return False
    if (torus_1.Center - torus_2.Center).length > 1e-5:
        return False
    return abs(torus_1.Axis.dot(torus_2.Axis)) >= 0.99999


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


def torus_sheet_sign(vertex: GVector, torus, tol: float = 1e-8) -> int:
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
    if abs(a) < 1e-9:
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
    if e2.length < 1e-6:
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
        if n_common.length < 1e-9:
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
        narrow_wide_threshold = 0.01 * main_radius

    half_width = 0.5 * (proj_far - proj_near)
    if half_width <= narrow_wide_threshold:
        offset_amount = half_width
    else:
        offset_amount = min(0.001 * half_width, narrow_wide_threshold)

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
        rp = (p.Surf.Position - center)
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
        p0 = plane_list[angles[-1][1] ].Surf.Axis
        p1 = plane_list[angles[0][1] ].Surf.Axis
        start = 1
    else:
        p0 = plane_list[angles[0][1] ].Surf.Axis
        p1 = plane_list[angles[1][1] ].Surf.Axis
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

    return convex, orientation

