import math

import pytest

from geouned.geometry_backend.geometry_backend_interface import GVector


def test_add_sub():
    a = GVector(1, 2, 3)
    b = GVector(4, 5, 6)
    assert a + b == GVector(5, 7, 9)
    assert b - a == GVector(3, 3, 3)


def test_scalar_mul():
    a = GVector(1, 2, 3)
    assert a * 2 == GVector(2, 4, 6)
    assert 2 * a == GVector(2, 4, 6)


def test_neg():
    assert -GVector(1, -2, 3) == GVector(-1, 2, -3)


def test_dot():
    assert GVector(1, 0, 0).dot(GVector(0, 1, 0)) == 0
    assert GVector(1, 2, 3).dot(GVector(1, 2, 3)) == 14


def test_cross():
    assert GVector(1, 0, 0).cross(GVector(0, 1, 0)) == GVector(0, 0, 1)


def test_length():
    assert GVector(3, 4, 0).length == pytest.approx(5.0)


def test_normalized():
    n = GVector(0, 5, 0).normalized()
    assert n == GVector(0, 1, 0)
    assert n.length == pytest.approx(1.0)


def test_is_equal():
    assert GVector(1, 1, 1).is_equal(GVector(1, 1 + 1e-9, 1))
    assert not GVector(1, 1, 1).is_equal(GVector(1, 2, 1))


def test_angle_to_perpendicular():
    angle = GVector(1, 0, 0).angle_to(GVector(0, 1, 0))
    assert angle == pytest.approx(math.pi / 2)


def test_angle_to_parallel():
    assert GVector(2, 0, 0).angle_to(GVector(5, 0, 0)) == pytest.approx(0.0)


def test_angle_to_opposite():
    assert GVector(1, 0, 0).angle_to(GVector(-1, 0, 0)) == pytest.approx(math.pi)


def test_iter():
    assert tuple(GVector(1, 2, 3)) == (1, 2, 3)
