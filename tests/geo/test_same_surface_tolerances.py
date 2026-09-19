import math

import pytest

from geouned.geo import GeoTolerances, GVector
from geouned.geo.constants import NUMERIC_TOL
from geouned.geo import surface_geometry as sg
from geouned.geo.surface_geometry import axes_parallel, axes_perpendicular, axes_same_direction, opposite_sense
from geouned.GEOUNED.utils.basic_functions_part2 import (
    is_same_cone,
    is_same_cylinder,
    is_same_plane,
    is_same_sphere,
    is_same_torus,
)
from geouned.GEOUNED.utils.data_classes import Tolerances


class _Surf:
    """Duck-typed stand-in for a surface descriptor (same field names)."""

    def __init__(self, **fields):
        self.__dict__.update(fields)


def _tilted_axis(angle):
    return GVector(math.sin(angle), 0.0, math.cos(angle))


Z = GVector(0, 0, 1)
ORIGIN = GVector(0, 0, 0)
TOL = GeoTolerances()


# ---------------------------------------------------------------------------
# geo-side identity predicates read the USER's tolerances (GeoTolerances), so the decomposition and the output stage
# agree on what "the same surface" means.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("factor, expected", [(0.9, True), (1.1, False)])
def test_same_plane_angle_and_offset_boundaries(factor, expected):
    tilted = _Surf(Axis=_tilted_axis(factor * TOL.pln_angle), Position=ORIGIN)
    shifted = _Surf(Axis=Z, Position=GVector(0, 0, factor * TOL.pln_distance))
    plane = _Surf(Axis=Z, Position=ORIGIN)
    assert sg.is_same_plane_surface(plane, tilted, TOL) is expected
    assert sg.is_same_plane_surface(tilted, plane, TOL) is expected
    assert sg.is_same_plane_surface(plane, shifted, TOL) is expected


def test_same_plane_follows_the_tolerances_given():
    plane = _Surf(Axis=Z, Position=ORIGIN)
    tilted = _Surf(Axis=_tilted_axis(5e-3), Position=ORIGIN)  # 50x the default pln_angle
    assert not sg.is_same_plane_surface(plane, tilted, TOL)
    assert sg.is_same_plane_surface(plane, tilted, GeoTolerances(pln_angle=1e-2))
    apart = _Surf(Axis=Z, Position=GVector(0, 0, 5e-4))
    assert not sg.is_same_plane_surface(plane, apart, TOL)
    assert sg.is_same_plane_surface(plane, apart, GeoTolerances(pln_distance=1e-3))


@pytest.mark.parametrize("factor, expected", [(0.9, True), (1.1, False)])
def test_axis_angle_uses_each_surface_own_tolerance(factor, expected):
    cyl = (_Surf(Radius=5.0, Axis=Z, Center=ORIGIN), _Surf(Radius=5.0, Axis=_tilted_axis(factor * TOL.cyl_angle), Center=ORIGIN))
    cone = (_Surf(SemiAngle=0.3, Axis=Z, Apex=ORIGIN), _Surf(SemiAngle=0.3, Axis=_tilted_axis(factor * TOL.kne_angle), Apex=ORIGIN))
    tor = (
        _Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=Z, Center=ORIGIN),
        _Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=_tilted_axis(factor * TOL.tor_angle), Center=ORIGIN),
    )
    assert sg.is_same_cylinder_surface(*cyl, TOL) is expected
    assert sg.is_same_cone_surface(*cone, TOL) is expected
    assert sg.is_same_torus_surface(*tor, TOL) is expected


def test_distances_use_each_surface_own_tolerance():
    d = 0.5 * TOL.cyl_distance
    far = 2.0 * TOL.cyl_distance
    assert sg.is_same_cylinder_surface(_Surf(Radius=5.0, Axis=Z, Center=ORIGIN), _Surf(Radius=5.0 + d, Axis=Z, Center=ORIGIN), TOL)
    assert not sg.is_same_cylinder_surface(_Surf(Radius=5.0, Axis=Z, Center=ORIGIN), _Surf(Radius=5.0 + far, Axis=Z, Center=ORIGIN), TOL)
    assert sg.is_same_sphere_surface(_Surf(Radius=5.0, Center=ORIGIN), _Surf(Radius=5.0, Center=GVector(d, 0, 0)), TOL)
    assert not sg.is_same_sphere_surface(_Surf(Radius=5.0, Center=ORIGIN), _Surf(Radius=5.0, Center=GVector(far, 0, 0)), TOL)
    # a cylinder's Center is an arbitrary point on its axis: sliding it along the axis must not matter
    assert sg.is_same_cylinder_surface(_Surf(Radius=5.0, Axis=Z, Center=ORIGIN), _Surf(Radius=5.0, Axis=Z, Center=GVector(0, 0, 1e4)), TOL)


def test_parallel_plane_surface_uses_pln_angle():
    a = _Surf(Axis=Z)
    assert sg.is_parallel_plane_surface(a, _Surf(Axis=_tilted_axis(0.9 * TOL.pln_angle)), TOL)
    assert not sg.is_parallel_plane_surface(a, _Surf(Axis=_tilted_axis(1.1 * TOL.pln_angle)), TOL)


def test_same_plane_antiparallel_axes_compare_offsets_with_opposite_sign():
    a = _Surf(Axis=Z, Position=GVector(0, 0, 3.5))
    same = _Surf(Axis=-Z, Position=GVector(0, 0, 3.5))
    other = _Surf(Axis=-Z, Position=GVector(0, 0, -3.5))
    assert sg.is_same_plane_surface(a, same, TOL)
    assert not sg.is_same_plane_surface(a, other, TOL)


def test_predicates_require_tolerances():
    with pytest.raises(TypeError):
        sg.is_same_plane_surface(_Surf(Axis=Z, Position=ORIGIN), _Surf(Axis=Z, Position=ORIGIN))


def test_tiny_axis_angles_are_resolved():
    # an atan2-based angle, not acos(dot): still exact for angles far below sqrt(machine epsilon)
    plane = _Surf(Axis=Z, Position=ORIGIN)
    assert sg.is_same_plane_surface(plane, _Surf(Axis=_tilted_axis(1e-9), Position=ORIGIN), TOL)
    assert not sg.is_same_plane_surface(plane, _Surf(Axis=_tilted_axis(2e-4), Position=ORIGIN), TOL)


# ---------------------------------------------------------------------------
# Tolerances-based predicates with relativeTol=True: identical surfaces
# must always compare equal, including the ones sitting exactly at the
# origin (a relative tolerance of `rel * 0` used to be 0, and
# `is_in_tolerance(0, 0, ...)` answers "not same").
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("relative", [False, True])
def test_identical_surfaces_at_origin_are_same(relative):
    tol = Tolerances(relativeTol=relative)
    plane = _Surf(Axis=Z, Position=ORIGIN, real=True)
    cylinder = _Surf(Radius=10.0, Axis=Z, Center=ORIGIN)
    sphere = _Surf(Radius=10.0, Center=ORIGIN)
    cone = _Surf(SemiAngle=0.3, Axis=Z, Apex=ORIGIN)
    torus = _Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=Z, Center=ORIGIN)

    assert is_same_plane(plane, plane, tolerances=tol)
    assert is_same_cylinder(cylinder, cylinder, tolerances=tol)
    assert is_same_sphere(sphere, sphere, tol)
    assert is_same_cone(cone, cone, tol)
    assert is_same_torus(torus, torus, tol)


@pytest.mark.parametrize("relative", [False, True])
def test_float_noise_at_origin_does_not_split_a_surface(relative):
    # 1e-12 mm of representation noise on a surface at the origin.
    tol = Tolerances(relativeTol=relative)
    noise = GVector(1e-12, -1e-12, 1e-12)
    plane_a = _Surf(Axis=Z, Position=ORIGIN, real=True)
    plane_b = _Surf(Axis=Z, Position=noise, real=True)
    sphere_a = _Surf(Radius=10.0, Center=ORIGIN)
    sphere_b = _Surf(Radius=10.0, Center=noise)

    assert is_same_plane(plane_a, plane_b, tolerances=tol)
    assert is_same_sphere(sphere_a, sphere_b, tol)


def test_relative_tolerance_still_distinguishes_genuinely_different_surfaces():
    tol = Tolerances(relativeTol=True)
    plane_a = _Surf(Axis=Z, Position=ORIGIN, real=True)
    plane_b = _Surf(Axis=Z, Position=GVector(0, 0, 0.5), real=True)
    sphere_a = _Surf(Radius=10.0, Center=ORIGIN)
    sphere_b = _Surf(Radius=10.0, Center=GVector(0.5, 0, 0))

    assert not is_same_plane(plane_a, plane_b, tolerances=tol)
    assert not is_same_sphere(sphere_a, sphere_b, tol)


# ---------------------------------------------------------------------------
# angle helpers: every "same axis / perpendicular" test is an angle compared with an angle tolerance
# ---------------------------------------------------------------------------

X = GVector(1, 0, 0)
Y = GVector(0, 1, 0)


@pytest.mark.parametrize("factor, expected", [(0.9, True), (1.1, False)])
def test_axes_parallel_either_direction(factor, expected):
    tol = 1e-4
    tilted = GVector(math.cos(factor * tol), math.sin(factor * tol), 0.0)
    assert axes_parallel(X, tilted, tol) is expected
    assert axes_parallel(X, -tilted, tol) is expected  # antiparallel is the same line


def test_axes_same_direction_rejects_antiparallel():
    assert axes_same_direction(X, X * 3.0, 1e-4)
    assert not axes_same_direction(X, -X, 1e-4)


@pytest.mark.parametrize("sign", [+1, -1])
@pytest.mark.parametrize("factor, expected", [(0.9, True), (1.1, False)])
def test_axes_perpendicular_accepts_plus_and_minus_90_degrees(sign, factor, expected):
    # alpha = +pi/2 and alpha = -pi/2 both give cos(alpha) = 0: both must count as perpendicular
    tol = 1e-4
    other = GVector(math.sin(factor * tol), sign * math.cos(factor * tol), 0.0)
    assert axes_perpendicular(X, other, tol) is expected
    assert axes_perpendicular(other, X, tol) is expected


def test_perpendicular_uses_the_same_tolerance_as_parallel():
    tol = 2e-4
    just_in = GVector(math.sin(0.9 * tol), math.cos(0.9 * tol), 0.0)
    just_out = GVector(math.sin(1.1 * tol), math.cos(1.1 * tol), 0.0)
    assert axes_perpendicular(X, just_in, tol) and not axes_perpendicular(X, just_out, tol)
    tilt_in = GVector(math.cos(0.9 * tol), math.sin(0.9 * tol), 0.0)
    tilt_out = GVector(math.cos(1.1 * tol), math.sin(1.1 * tol), 0.0)
    assert axes_parallel(X, tilt_in, tol) and not axes_parallel(X, tilt_out, tol)


def test_perpendicular_is_scale_invariant_and_rejects_parallel():
    assert axes_perpendicular(X * 1e6, Y * 1e-6, 1e-4)  # displacements, not unit vectors
    assert not axes_perpendicular(X, X, 1e-4)
    assert not axes_perpendicular(X, -X, 1e-4)


def test_coaxial_cone_pair_uses_kne_tolerances():
    cone_a = _Surf(SemiAngle=0.3, Axis=Z, Apex=ORIGIN)
    cone_b = _Surf(SemiAngle=0.3, Axis=Z, Apex=GVector(0, 0, 50.0))
    assert sg.is_coaxial_cone_pair(cone_a, cone_b, TOL)
    tilted = _Surf(SemiAngle=0.3, Axis=_tilted_axis(1.5 * TOL.kne_angle), Apex=GVector(0, 0, 50.0))
    assert not sg.is_coaxial_cone_pair(cone_a, tilted, TOL)
    assert sg.is_coaxial_cone_pair(cone_a, tilted, GeoTolerances(kne_angle=1e-3))


# ---------------------------------------------------------------------------
# is_same_cone / is_same_sphere / is_same_torus take the run's Tolerances, like plane and cylinder: no private defaults
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("func", [is_same_cone, is_same_sphere, is_same_torus])
def test_registry_predicates_require_the_tolerances(func):
    a = _Surf(SemiAngle=0.3, Axis=Z, Apex=ORIGIN, Radius=5.0, Center=ORIGIN, MajorRadius=20.0, MinorRadius=3.0)
    with pytest.raises(TypeError):
        func(a, a)
    with pytest.raises(TypeError):
        func(a, a, None)


def test_registry_predicates_read_each_surfaces_own_tolerance():
    sph = (_Surf(Radius=5.0, Center=ORIGIN), _Surf(Radius=5.0 + 5e-5, Center=ORIGIN))
    assert is_same_sphere(*sph, Tolerances(sph_distance=1e-4))
    assert not is_same_sphere(*sph, Tolerances(sph_distance=1e-5))
    cone = (_Surf(SemiAngle=0.3, Axis=Z, Apex=ORIGIN), _Surf(SemiAngle=0.3 + 5e-5, Axis=Z, Apex=ORIGIN))
    assert is_same_cone(*cone, Tolerances(kne_angle=1e-4))
    assert not is_same_cone(*cone, Tolerances(kne_angle=1e-5))
    tor = (_Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=Z, Center=ORIGIN), _Surf(MajorRadius=20.0 + 5e-5, MinorRadius=3.0, Axis=Z, Center=ORIGIN))
    assert is_same_torus(*tor, Tolerances(tor_distance=1e-4))
    assert not is_same_torus(*tor, Tolerances(tor_distance=1e-5))


def test_torus_with_reversed_axis_is_kept_apart():
    a = _Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=Z, Center=ORIGIN)
    b = _Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=GVector(0, 0, -1), Center=ORIGIN)
    assert not is_same_torus(a, b, Tolerances())  # unchanged behaviour: opposite axes are kept apart


# ---------------------------------------------------------------------------
# the sense of an axis is a sign, not an angle tolerance
# ---------------------------------------------------------------------------


def test_opposite_sense_is_the_sign_of_the_dot_product():
    assert opposite_sense(Z, GVector(0, 0, -1))
    assert not opposite_sense(Z, Z)


def test_opposite_sense_does_not_depend_on_any_tolerance():
    # a plane matched with add_pln_angle (1e-2) and tilted 5e-3 rad, antiparallel: still recognised as reversed,
    # which is_opposite(., ., pln_angle=1e-4) used to miss
    tilted_reversed = GVector(math.sin(5e-3), 0.0, -math.cos(5e-3))
    assert opposite_sense(Z, tilted_reversed)
    assert not sg.is_opposite(Z, tilted_reversed, 1e-4)


# ---------------------------------------------------------------------------
# "same oriented plane" (what PlaneParams.__eq__ meant), explicit and with the user's tolerances
# ---------------------------------------------------------------------------


def test_same_oriented_plane_needs_same_plane_and_same_sense():
    plane = _Surf(Axis=Z, Position=ORIGIN)
    assert sg.is_same_oriented_plane_surface(plane, _Surf(Axis=Z, Position=GVector(3, 0, 1e-6)), TOL)
    assert not sg.is_same_oriented_plane_surface(plane, _Surf(Axis=GVector(0, 0, -1), Position=ORIGIN), TOL)  # same plane, reversed
    assert not sg.is_same_oriented_plane_surface(plane, _Surf(Axis=Z, Position=GVector(0, 0, 1.0)), TOL)  # another plane
    assert sg.is_same_plane_surface(plane, _Surf(Axis=GVector(0, 0, -1), Position=ORIGIN), TOL)  # ... which IS the same infinite plane


def test_same_oriented_plane_uses_the_users_tolerances():
    plane = _Surf(Axis=Z, Position=ORIGIN)
    off = _Surf(Axis=Z, Position=GVector(0, 0, 5e-5))
    assert sg.is_same_oriented_plane_surface(plane, off, GeoTolerances(pln_distance=1e-4))
    assert not sg.is_same_oriented_plane_surface(plane, off, GeoTolerances(pln_distance=1e-5))
    tilted = _Surf(Axis=_tilted_axis(5e-5), Position=ORIGIN)
    assert sg.is_same_oriented_plane_surface(plane, tilted, GeoTolerances(pln_angle=1e-4))
    assert not sg.is_same_oriented_plane_surface(plane, tilted, GeoTolerances(pln_angle=1e-5))


def test_surface_params_define_no_equality():
    # PlaneParams / MultiPlanesParams / GeounedSurface used to define __eq__ with constants and no access to the run's
    # tolerances, and `in` / `.index` / `!=` called it implicitly. Comparisons are explicit now
    # (surface_geometry.is_same_oriented_plane_surface with the user's tolerances); nothing may reintroduce a hidden one.
    from geouned.GEOUNED.utils.basic_functions_part1 import MultiPlanesParams, PlaneParams
    from geouned.GEOUNED.utils.geouned_classes import GeounedSurface

    for cls in (PlaneParams, MultiPlanesParams, GeounedSurface):
        assert "__eq__" not in vars(cls), cls.__name__
