import math

import pytest

from geouned.geo import GVector
from geouned.geo import surface_geometry as sg
from geouned.geo.constants import SAME_SURFACE_AXIS_ANGLE_TOL
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


# ---------------------------------------------------------------------------
# geo-side predicates: the axis test is an explicit angle, not a bare dot
# ---------------------------------------------------------------------------


def test_axis_angle_tol_matches_the_historical_dot_threshold():
    # SAME_SURFACE_AXIS_ANGLE_TOL replaced a literal `dot >= 0.99999`; the
    # refactor must not move the threshold.
    assert math.cos(SAME_SURFACE_AXIS_ANGLE_TOL) == pytest.approx(0.99999, abs=1e-12)


@pytest.mark.parametrize("factor, expected", [(0.9, True), (1.1, False)])
def test_same_plane_axis_angle_boundary(factor, expected):
    a = _Surf(Axis=Z, Position=ORIGIN)
    b = _Surf(Axis=_tilted_axis(factor * SAME_SURFACE_AXIS_ANGLE_TOL), Position=ORIGIN)
    assert sg.is_same_plane_surface(a, b) is expected
    assert sg.is_same_plane_surface(b, a) is expected


@pytest.mark.parametrize("factor, expected", [(0.9, True), (1.1, False)])
def test_same_cylinder_cone_torus_axis_angle_boundary(factor, expected):
    axis = _tilted_axis(factor * SAME_SURFACE_AXIS_ANGLE_TOL)
    cyl = (_Surf(Radius=5.0, Axis=Z, Center=ORIGIN), _Surf(Radius=5.0, Axis=axis, Center=ORIGIN))
    cone = (_Surf(SemiAngle=0.3, Axis=Z, Apex=ORIGIN), _Surf(SemiAngle=0.3, Axis=axis, Apex=ORIGIN))
    tor = (
        _Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=Z, Center=ORIGIN),
        _Surf(MajorRadius=20.0, MinorRadius=3.0, Axis=axis, Center=ORIGIN),
    )
    assert sg.is_same_cylinder_surface(*cyl) is expected
    assert sg.is_same_cone_surface(*cone) is expected
    assert sg.is_same_torus_surface(*tor) is expected


def test_parallel_plane_surface_uses_same_angle():
    a = _Surf(Axis=Z)
    assert sg.is_parallel_plane_surface(a, _Surf(Axis=_tilted_axis(0.9 * SAME_SURFACE_AXIS_ANGLE_TOL)))
    assert not sg.is_parallel_plane_surface(a, _Surf(Axis=_tilted_axis(1.1 * SAME_SURFACE_AXIS_ANGLE_TOL)))


def test_same_plane_antiparallel_axes_compare_offsets_with_opposite_sign():
    a = _Surf(Axis=Z, Position=GVector(0, 0, 3.5))
    same = _Surf(Axis=-Z, Position=GVector(0, 0, 3.5))
    other = _Surf(Axis=-Z, Position=GVector(0, 0, -3.5))
    assert sg.is_same_plane_surface(a, same)
    assert not sg.is_same_plane_surface(a, other)


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
