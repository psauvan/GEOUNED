#
# Set of useful functions used in different parts of the code
#
import logging


from .basic_functions_part1 import is_in_tolerance
from ...geo.surface_geometry import (
    axes_angle,
    axes_parallel,
    cone_apex_offset,
    cone_semiangle_diff,
    cylinder_axis_offset,
    cylinder_radius_diff,
    is_same_cone_surface,
    is_same_cylinder_surface,
    is_same_elliptic_cylinder_surface,
    is_same_sphere_surface,
    plane_offset,
    plane_within,
    relative_tolerance,
    sphere_center_offset,
    sphere_radius_diff,
)


def _require(tolerances, caller):
    if tolerances is None:
        raise TypeError(f"{caller}() needs the run's Tolerances (tolerances=...): a default instance would silently ignore the user's values")
    return tolerances


_logged_fuzzy = set()


def reset_fuzzy_log():
    """Forget which near-misses have already been written: call once per run, when the fuzzy log file is (re)opened."""
    _logged_fuzzy.clear()


def Fuzzy(index, dtype, val, tol):

    fuzzy_logger = logging.getLogger("fuzzy_logger")

    same = val <= tol

    # The same pair of surfaces is compared again every time another face of the same surface is registered, and
    # each comparison lands on the same near-miss (only the last digits of `val` change, from float noise in the
    # faces' own positions), so one entry per (stored surface, quantity, outcome, val/tol to 3 decimals) is enough.
    key = (index, dtype, same, round(val / tol, 3) if tol else val)
    if key in _logged_fuzzy:
        return
    _logged_fuzzy.add(key)

    if dtype == "plane":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Plane distance / Tolerance : {val} {tol}")

    elif dtype == "cylRad":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Cylinder Rad_diff / Tolerance: {val} {tol}")

    elif dtype == "cylAxs":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Cylinder Axis Dist_diff / Tolerance: {val} {tol}")

    elif dtype == "cylAng":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Cylinder Axis Angle_diff / Tolerance: {val} {tol}")

    elif dtype == "sphRad":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Sphere Rad_diff / Tolerance: {val} {tol}")

    elif dtype == "sphCen":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Sphere Center Dist_diff / Tolerance: {val} {tol}")

    elif dtype == "coneSAng":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Cone SemiAngle_diff / Tolerance: {val} {tol}")

    elif dtype == "coneApx":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Cone Apex Dist_diff / Tolerance: {val} {tol}")

    elif dtype == "coneAng":
        fuzzy_logger.info(f"Dict surface index / Same Surf: {index} {same}")
        fuzzy_logger.info(f"Cone Axis Angle_diff / Tolerance: {val} {tol}")


def is_same_plane(
    p1, p2, tolerances=None, fuzzy=(False, 0), stdtol=True
):
    """The CSG-registry entry point for plane identity: same decision as `geo.surface_geometry.is_same_plane_surface`
    (that is the ONE implementation, unified 2026-09-22 -- see its own docstring), plus what only the registry needs:
    `add_pln_angle`/`add_pln_distance` for a non-real (`stdtol=False`) plane, and an opt-in near-miss diagnostic log
    (`fuzzy`, written to the `fuzzy_logger`, never affects the returned decision)."""
    tolerances = _require(tolerances, "is_same_plane")
    angle_tol = tolerances.pln_angle if stdtol else tolerances.add_pln_angle
    distance_tol = tolerances.pln_distance if stdtol else tolerances.add_pln_distance
    same = plane_within(p1, p2, angle_tol, distance_tol, tolerances.relativeTol)

    if fuzzy[0] and axes_parallel(p1.Axis, p2.Axis, angle_tol):
        tol = distance_tol
        if tolerances.relativeTol:
            scale = max(abs(p1.Axis.dot(p1.Position)), abs(p2.Axis.dot(p2.Position)))
            tol = relative_tolerance(distance_tol, scale)
        d = plane_offset(p1, p2)
        _, is_fuzzy = is_in_tolerance(d, tol, 0.5 * tol, 2 * tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "plane", d, tol)

    return same


def is_same_cylinder(
    cyl1,
    cyl2,
    tolerances=None,
    fuzzy=(False, 0),
):
    """The CSG-registry entry point for cylinder identity: same decision as
    `geo.surface_geometry.is_same_cylinder_surface` (that is the ONE implementation -- unified 2026-09-22), plus the
    registry's own opt-in near-miss diagnostic log (see `is_same_plane`'s docstring -- same contract). The radius
    and axis-angle fuzzy-log checks run independently of the axis/centre one (matching this function's own
    historical control flow): a radius near-miss is still worth logging even when the axes turn out not to be
    parallel at all, and the angle check (deviation from parallel against `cyl_angle`, absolute -- `relativeTol`
    only scales distances) is by definition about axes that are NOT yet parallel within tolerance."""
    tolerances = _require(tolerances, "is_same_cylinder")
    same = is_same_cylinder_surface(cyl1, cyl2, tolerances)

    if fuzzy[0]:
        relative_tol = tolerances.relativeTol

        radius_tol = tolerances.cyl_distance
        if relative_tol:
            radius_tol = relative_tolerance(tolerances.cyl_distance, max(cyl2.Radius, cyl1.Radius))
        radius_diff = cylinder_radius_diff(cyl1, cyl2)
        _, is_fuzzy = is_in_tolerance(radius_diff, radius_tol, 0.5 * radius_tol, 2 * radius_tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "cylRad", abs(radius_diff), radius_tol)

        angle_tol = tolerances.cyl_angle
        angle = axes_angle(cyl1.Axis, cyl2.Axis)
        _, is_fuzzy = is_in_tolerance(angle, angle_tol, 0.5 * angle_tol, 2 * angle_tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "cylAng", angle, angle_tol)

        if axes_parallel(cyl1.Axis, cyl2.Axis, tolerances.cyl_angle):
            axis_tol = tolerances.cyl_distance
            if relative_tol:
                axis_tol = relative_tolerance(tolerances.cyl_distance, max(cyl1.Center.length, cyl2.Center.length))
            d = cylinder_axis_offset(cyl1, cyl2)
            _, is_fuzzy = is_in_tolerance(d, axis_tol, 0.5 * axis_tol, 2 * axis_tol)
            if is_fuzzy:
                Fuzzy(fuzzy[1], "cylAxs", d, axis_tol)

    return same


def is_same_elliptic_cylinder(
    cyl1,
    cyl2,
    tolerances=None,
    fuzzy=(False, 0),
):
    """The CSG-registry entry point for elliptic-cylinder identity (added 2026-09-18, see CLAUDE.md's
    "Spline-vs-quadric identification" entry; reconciled 2026-09-27 with the `tolerances`-threaded unification of
    every other `is_same_*` registry entry point): same decision as
    `geo.surface_geometry.is_same_elliptic_cylinder_surface` (the ONE implementation, reusing `cyl_distance`/
    `cyl_angle` -- no dedicated tolerance fields for this base-surface-only first cut). No near-miss diagnostic log
    yet (`fuzzy` accepted for call-site symmetry with `is_same_cylinder`/`is_same_sphere`/`is_same_cone`, but
    unused -- see CLAUDE.md's TODO list, this base surface hasn't gone through the fuzzy-log extension the other
    4 got)."""
    tolerances = _require(tolerances, "is_same_elliptic_cylinder")
    return is_same_elliptic_cylinder_surface(cyl1, cyl2, tolerances)


def is_same_sphere(sph1, sph2, tolerances=None, fuzzy=(False, 0)):
    """The CSG-registry entry point for sphere identity: same decision as `geo.surface_geometry.is_same_sphere_surface`,
    plus the registry's own opt-in near-miss diagnostic log (see `is_same_plane`'s docstring -- same contract). Radius
    and centre are logged independently of each other, both against `sph_distance`."""
    tolerances = _require(tolerances, "is_same_sphere")
    same = is_same_sphere_surface(sph1, sph2, tolerances)

    if fuzzy[0]:
        relative_tol = tolerances.relativeTol

        radius_tol = tolerances.sph_distance
        if relative_tol:
            radius_tol = relative_tolerance(tolerances.sph_distance, max(sph1.Radius, sph2.Radius))
        radius_diff = abs(sphere_radius_diff(sph1, sph2))
        _, is_fuzzy = is_in_tolerance(radius_diff, radius_tol, 0.5 * radius_tol, 2 * radius_tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "sphRad", radius_diff, radius_tol)

        centre_tol = tolerances.sph_distance
        if relative_tol:
            centre_tol = relative_tolerance(tolerances.sph_distance, max(sph1.Center.length, sph2.Center.length))
        d = sphere_center_offset(sph1, sph2)
        _, is_fuzzy = is_in_tolerance(d, centre_tol, 0.5 * centre_tol, 2 * centre_tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "sphCen", d, centre_tol)

    return same


def is_same_cone(cone1, cone2, tolerances=None, fuzzy=(False, 0)):
    """The CSG-registry entry point for cone identity: same decision as `geo.surface_geometry.is_same_cone_surface`,
    plus the registry's own opt-in near-miss diagnostic log (see `is_same_plane`'s docstring -- same contract).
    Semi-angle, apex distance and axis angle are logged independently of each other (the angles against `kne_angle`,
    absolute; the apex against `kne_distance`, scaled by the apexes' own distance from the origin under
    `relativeTol`)."""
    tolerances = _require(tolerances, "is_same_cone")
    same = is_same_cone_surface(cone1, cone2, tolerances)

    if fuzzy[0]:
        angle_tol = tolerances.kne_angle
        semiangle_diff = abs(cone_semiangle_diff(cone1, cone2))
        _, is_fuzzy = is_in_tolerance(semiangle_diff, angle_tol, 0.5 * angle_tol, 2 * angle_tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "coneSAng", semiangle_diff, angle_tol)

        apex_tol = tolerances.kne_distance
        if tolerances.relativeTol:
            apex_tol = relative_tolerance(tolerances.kne_distance, max(cone1.Apex.length, cone2.Apex.length))
        d = cone_apex_offset(cone1, cone2)
        _, is_fuzzy = is_in_tolerance(d, apex_tol, 0.5 * apex_tol, 2 * apex_tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "coneApx", d, apex_tol)

        axis_angle = axes_angle(cone1.Axis, cone2.Axis)
        _, is_fuzzy = is_in_tolerance(axis_angle, angle_tol, 0.5 * angle_tol, 2 * angle_tol)
        if is_fuzzy:
            Fuzzy(fuzzy[1], "coneAng", axis_angle, angle_tol)

    return same
