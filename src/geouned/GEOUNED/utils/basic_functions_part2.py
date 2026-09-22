#
# Set of useful functions used in different parts of the code
#
import logging
import math


from .data_classes import Options, NumericFormat
from .basic_functions_part1 import is_in_tolerance
from ..write.functions import mcnp_surface
from ...geo.constants import PARAM_ANGLE_TOL_E5
from ...geo.surface_geometry import (
    axes_parallel,
    cylinder_axis_offset,
    cylinder_radius_diff,
    is_same_cylinder_surface,
    plane_offset,
    plane_within,
    relative_tolerance,
)


def _require(tolerances, caller):
    if tolerances is None:
        raise TypeError(f"{caller}() needs the run's Tolerances (tolerances=...): a default instance would silently ignore the user's values")
    return tolerances


def Fuzzy(index, dtype, surf1, surf2, val, tol, options, tolerances, numeric_format):

    fuzzy_logger = logging.getLogger("fuzzy_logger")

    same = val <= tol

    if dtype == "plane":
        p1str = mcnp_surface(index, "Plane", surf1, options, tolerances, numeric_format)
        p2str = mcnp_surface(0, "Plane", surf2, options, tolerances, numeric_format)
        fuzzy_logger.info(f"Same surface : {same}")
        fuzzy_logger.info(f"Plane distance / Tolerance : {val} {tol}\n {p1str}\n {p2str}\n\n")

    elif dtype == "cylRad":
        cyl1str = mcnp_surface(index, "Cylinder", surf1, options, tolerances, numeric_format)
        cyl2str = mcnp_surface(0, "Cylinder", surf2, options, tolerances, numeric_format)
        fuzzy_logger.info(f"Same surface : {same}")
        fuzzy_logger.info(f"Diff Radius / Tolerance: {val} {tol}")
        fuzzy_logger.info(f"{cyl1str}\n {cyl2str}\n\n")

    elif dtype == "cylAxs":
        cyl1str = mcnp_surface(index, "Cylinder", surf1, options, tolerances, numeric_format)
        cyl2str = mcnp_surface(0, "Cylinder", surf2, options, tolerances, numeric_format)
        fuzzy_logger.info(f"Same surface : {same}")
        fuzzy_logger.info(f"Dist Axis / Tolerance: {val} {tol}")
        fuzzy_logger.info(f"{cyl1str} {cyl2str}")


def is_same_plane(
    p1, p2, options=Options(), tolerances=None, numeric_format=NumericFormat(), fuzzy=(False, 0), stdtol=True
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
            Fuzzy(fuzzy[1], "plane", p2, p1, d, tol, options, tolerances, numeric_format)

    return same


def is_same_cylinder(
    cyl1,
    cyl2,
    options=Options(),
    tolerances=None,
    numeric_format=NumericFormat(),
    fuzzy=(False, 0),
):
    """The CSG-registry entry point for cylinder identity: same decision as
    `geo.surface_geometry.is_same_cylinder_surface` (that is the ONE implementation -- unified 2026-09-22), plus the
    registry's own opt-in near-miss diagnostic log (see `is_same_plane`'s docstring -- same contract). The radius
    fuzzy-log check runs independently of the axis/centre one (matching this function's own historical control
    flow): a radius near-miss is still worth logging even when the axes turn out not to be parallel at all."""
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
            Fuzzy(fuzzy[1], "cylRad", cyl2, cyl1, abs(radius_diff), radius_tol, options, tolerances, numeric_format)

        if axes_parallel(cyl1.Axis, cyl2.Axis, tolerances.cyl_angle):
            axis_tol = tolerances.cyl_distance
            if relative_tol:
                axis_tol = relative_tolerance(tolerances.cyl_distance, max(cyl1.Center.length, cyl2.Center.length))
            d = cylinder_axis_offset(cyl1, cyl2)
            _, is_fuzzy = is_in_tolerance(d, axis_tol, 0.5 * axis_tol, 2 * axis_tol)
            if is_fuzzy:
                Fuzzy(fuzzy[1], "cylAxs", cyl1, cyl2, d, axis_tol, options, tolerances, numeric_format)

    return same


def is_duplicate_in_list(num_str1, i, lista):
    for j, elem2 in enumerate(lista):
        if i == j:
            continue
        num_str2 = f"{elem2:11.4E}"
        num_str3 = f"{elem2 + 2.0 * math.pi:11.4E}"
        num_str4 = f"{elem2 - 2.0 * math.pi:11.4E}"

        if abs(float(num_str2)) < PARAM_ANGLE_TOL_E5:
            num_str2 = "%11.4E" % 0.0

        if abs(float(num_str3)) < PARAM_ANGLE_TOL_E5:
            num_str3 = "%11.4E" % 0.0

        if abs(float(num_str4)) < PARAM_ANGLE_TOL_E5:
            num_str4 = "%11.4E" % 0.0

        if num_str1 == num_str2 or num_str1 == num_str3 or num_str1 == num_str4:
            return True

    return False
