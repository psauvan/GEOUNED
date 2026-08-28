import math

import pytest

from geouned.geo import GBoundBox, GCylinder, GPlane, GVector
from geouned.geo.vector_geometry import to_gboundbox
from geouned.geo.surface_geometry import (
    cylinder_tangent_at,
    cylinder_value_at,
    is_in_line,
    is_in_plane,
    is_opposite,
    is_parallel,
    is_same_value,
    plane_tangent_at,
    plane_value_at,
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
    plane = GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1))
    assert is_in_plane(GVector(3, 4, 0), plane)
    assert not is_in_plane(GVector(3, 4, 1), plane)


def test_sign_plane():
    plane = GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1))
    assert sign_plane(GVector(0, 0, 1), plane) == 1
    assert sign_plane(GVector(0, 0, -1), plane) == -1


def test_plane_value_at():
    plane = GPlane.from_values(GVector(1, 2, 3), GVector(0, 0, 1), xdir=GVector(1, 0, 0))
    assert plane_value_at(plane, 2, 5).is_equal(GVector(3, 7, 3), 1e-9)


def test_plane_tangent_at():
    plane = GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1), xdir=GVector(1, 0, 0))
    tangent_u, tangent_v = plane_tangent_at(plane, 0.5, -1.2)
    assert tangent_u.is_equal(GVector(1, 0, 0), 1e-9)
    assert tangent_v.is_equal(GVector(0, 1, 0), 1e-9)


def test_plane_value_at_without_x_dir_raises():
    plane = GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1))
    with pytest.raises(ValueError):
        plane_value_at(plane, 0, 0)


def test_cylinder_value_at_on_axis():
    cylinder = GCylinder.from_values(GVector(0, 0, 0), GVector(0, 0, 1), 2.0, xdir=GVector(1, 0, 0))
    assert cylinder_value_at(cylinder, 0.0, 3.0).is_equal(GVector(2, 0, 3), 1e-9)
    assert cylinder_value_at(cylinder, math.pi / 2, 0.0).is_equal(GVector(0, 2, 0), 1e-9)


def test_cylinder_tangent_at_on_axis():
    cylinder = GCylinder.from_values(GVector(0, 0, 0), GVector(0, 0, 1), 2.0, xdir=GVector(1, 0, 0))
    tangent_u, tangent_v = cylinder_tangent_at(cylinder, 0.0, 0.0)
    assert tangent_u.is_equal(GVector(0, 1, 0), 1e-9)
    assert tangent_v.is_equal(GVector(0, 0, 1), 1e-9)


def test_cylinder_value_at_without_x_dir_raises():
    cylinder = GCylinder.from_values(GVector(0, 0, 0), GVector(0, 0, 1), 2.0)
    with pytest.raises(ValueError):
        cylinder_value_at(cylinder, 0.0, 0.0)


def test_gboundbox_derived_properties():
    box = GBoundBox(0, 0, 0, 10, 20, 30)
    assert (box.XLength, box.YLength, box.ZLength) == (10, 20, 30)
    assert box.Center.is_equal(GVector(5, 10, 15), 1e-9)
    assert box.DiagonalLength == pytest.approx(math.sqrt(10**2 + 20**2 + 30**2))


def test_gboundbox_is_valid():
    assert GBoundBox(0, 0, 0, 10, 10, 10).is_valid()
    assert not GBoundBox(0, 0, 0, -1, 10, 10).is_valid()


def test_gboundbox_intersects():
    box = GBoundBox(0, 0, 0, 10, 10, 10)
    assert box.intersects(GBoundBox(5, 5, 5, 15, 15, 15))
    assert box.intersects(GBoundBox(10, 0, 0, 20, 10, 10))  # touching at the boundary
    assert not box.intersects(GBoundBox(10.0001, 0, 0, 20, 10, 10))


def test_gboundbox_enlarged():
    box = GBoundBox(0, 0, 0, 10, 10, 10)
    enlarged = box.enlarged(2)
    assert (enlarged.XMin, enlarged.YMin, enlarged.ZMin, enlarged.XMax, enlarged.YMax, enlarged.ZMax) == (
        -2,
        -2,
        -2,
        12,
        12,
        12,
    )


def test_gboundbox_union():
    a = GBoundBox(0, 0, 0, 10, 10, 10)
    b = GBoundBox(20, 20, 20, 30, 30, 30)  # disjoint
    u = a.union(b)
    assert (u.XMin, u.YMin, u.ZMin, u.XMax, u.YMax, u.ZMax) == (0, 0, 0, 30, 30, 30)


def test_gboundbox_intersected_overlapping():
    a = GBoundBox(0, 0, 0, 10, 10, 10)
    b = GBoundBox(5, 5, 5, 20, 20, 20)
    i = a.intersected(b)
    assert i.is_valid()
    assert (i.XMin, i.YMin, i.ZMin, i.XMax, i.YMax, i.ZMax) == (5, 5, 5, 10, 10, 10)


def test_gboundbox_intersected_disjoint_is_invalid():
    a = GBoundBox(0, 0, 0, 10, 10, 10)
    b = GBoundBox(20, 20, 20, 30, 30, 30)
    assert not a.intersected(b).is_valid()


def test_gboundbox_get_point_matches_freecad_numbering():
    # values captured from a real FreeCAD.BoundBox(0, 0, 0, 1, 2, 3)
    # probe -- get_point/get_edge must match its corner/edge numbering
    box = GBoundBox(0, 0, 0, 1, 2, 3)
    expected = [
        (0, 0, 3),
        (1, 0, 3),
        (1, 2, 3),
        (0, 2, 3),
        (0, 0, 0),
        (1, 0, 0),
        (1, 2, 0),
        (0, 2, 0),
    ]
    for i, xyz in enumerate(expected):
        assert tuple(box.get_point(i)) == pytest.approx(xyz)


def test_gboundbox_get_edge_matches_freecad_numbering():
    box = GBoundBox(0, 0, 0, 1, 2, 3)
    expected = [
        ((0, 0, 3), (1, 0, 3)),
        ((1, 0, 3), (1, 2, 3)),
        ((1, 2, 3), (0, 2, 3)),
        ((0, 2, 3), (0, 0, 3)),
        ((0, 0, 0), (1, 0, 0)),
        ((1, 0, 0), (1, 2, 0)),
        ((1, 2, 0), (0, 2, 0)),
        ((0, 2, 0), (0, 0, 0)),
        ((0, 0, 3), (0, 0, 0)),
        ((1, 0, 3), (1, 0, 0)),
        ((1, 2, 3), (1, 2, 0)),
        ((0, 2, 3), (0, 2, 0)),
    ]
    for i, (p1, p2) in enumerate(expected):
        e1, e2 = box.get_edge(i)
        assert tuple(e1) == pytest.approx(p1)
        assert tuple(e2) == pytest.approx(p2)


def test_to_gboundbox():
    class FakeBox:
        XMin, YMin, ZMin, XMax, YMax, ZMax = 0, 1, 2, 10, 11, 12

    box = to_gboundbox(FakeBox())
    assert (box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax) == (0, 1, 2, 10, 11, 12)
