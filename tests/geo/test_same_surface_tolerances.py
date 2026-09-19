import math

import pytest

from geouned.geo import GeoTolerances, GVector
from geouned.geo import surface_geometry as sg
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
    assert is_same_sphere(sphere, sphere, tol.sph_distance, rel_tol=relative)
    assert is_same_cone(cone, cone, dtol=tol.kne_distance, atol=tol.kne_angle, rel_tol=relative)
    assert is_same_torus(torus, torus, dtol=tol.tor_distance, atol=tol.tor_angle, rel_tol=relative)


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
    assert is_same_sphere(sphere_a, sphere_b, tol.sph_distance, rel_tol=relative)


def test_relative_tolerance_still_distinguishes_genuinely_different_surfaces():
    tol = Tolerances(relativeTol=True)
    plane_a = _Surf(Axis=Z, Position=ORIGIN, real=True)
    plane_b = _Surf(Axis=Z, Position=GVector(0, 0, 0.5), real=True)
    sphere_a = _Surf(Radius=10.0, Center=ORIGIN)
    sphere_b = _Surf(Radius=10.0, Center=GVector(0.5, 0, 0))

    assert not is_same_plane(plane_a, plane_b, tolerances=tol)
    assert not is_same_sphere(sphere_a, sphere_b, tol.sph_distance, rel_tol=True)
