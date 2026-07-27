import math

from geouned.geometry_backend.geometry_backend_interface import GVector, PlaneParams
from geouned.geometry_backend.vector_geometry import (
    is_in_line,
    is_in_plane,
    is_opposite,
    is_parallel,
    is_same_value,
    sign_plane,
)


def test_is_same_value():
    assert is_same_value(1.0, 1.0 + 1e-9)
    assert not is_same_value(1.0, 1.1)


def test_is_parallel_same_direction():
    assert is_parallel(GVector(1, 0, 0), GVector(2, 0, 0))


def test_is_parallel_opposite_direction():
    assert is_parallel(GVector(1, 0, 0), GVector(-3, 0, 0))


def test_is_parallel_perpendicular_is_false():
    assert not is_parallel(GVector(1, 0, 0), GVector(0, 1, 0))


def test_is_opposite():
    assert is_opposite(GVector(1, 0, 0), GVector(-1, 0, 0))
    assert not is_opposite(GVector(1, 0, 0), GVector(1, 0, 0))


def test_is_in_line_point_on_line():
    assert is_in_line(GVector(5, 0, 0), GVector(1, 0, 0), GVector(0, 0, 0))


def test_is_in_line_point_off_line():
    assert not is_in_line(GVector(5, 1, 0), GVector(1, 0, 0), GVector(0, 0, 0))


def test_is_in_plane():
    plane = PlaneParams(point=GVector(0, 0, 0), normal=GVector(0, 0, 1))
    assert is_in_plane(GVector(3, 4, 0), plane)
    assert not is_in_plane(GVector(3, 4, 1), plane)


def test_sign_plane():
    plane = PlaneParams(point=GVector(0, 0, 0), normal=GVector(0, 0, 1))
    assert sign_plane(GVector(0, 0, 1), plane) == 1
    assert sign_plane(GVector(0, 0, -1), plane) == -1
