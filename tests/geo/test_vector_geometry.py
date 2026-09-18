import math
import random

import pytest

from geouned.geo import GBoundBox, GCylinder, GPlane, GVector
from geouned.geo.vector_geometry import arbitrary_perpendicular, arc_extent, to_gboundbox
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


@pytest.mark.parametrize(
    "axis",
    [
        GVector(1, 0, 0),
        GVector(0, 1, 0),
        GVector(0, 0, 1),
        GVector(1, 1, 1).normalized(),
        GVector(0.3, -0.7, 0.2).normalized(),
    ],
)
def test_arbitrary_perpendicular(axis):
    perp = arbitrary_perpendicular(axis)
    assert abs(perp.dot(axis)) < 1e-9
    assert abs(perp.length - 1.0) < 1e-9


# ---------------------------------------------------------------------------
# arc_extent
# ---------------------------------------------------------------------------

_TWO_PI = 2.0 * math.pi


def _ang_diff(a, b):
    d = (a - b) % _TWO_PI
    return min(d, _TWO_PI - d)


def test_arc_extent_single_pair():
    assert arc_extent([(1.0, 2.5)]) == (1.0, 0, 2.5, 0)


def test_arc_extent_chained_overlap():
    umin, imin, umax, imax = arc_extent([(2.0, 3.0), (0.5, 2.2)])
    assert (umin, imin, umax, imax) == (0.5, 1, 3.0, 0)


def test_arc_extent_stitches_across_the_boundary():
    # one arc from 6.0 rad through 0/2*pi to 0.3 rad, in two pieces
    umin, imin, umax, imax = arc_extent([(6.0, _TWO_PI), (0.0, 0.3)])
    assert (umin, imin) == (6.0, 0)
    assert (umax, imax) == (0.3, 1)


def test_arc_extent_real_gap_still_raises():
    with pytest.raises(ValueError):
        arc_extent([(0.5, 1.0), (2.0, 2.5)])
    with pytest.raises(ValueError):
        arc_extent([(0.5, 1.0), (2.0, 2.5), (4.0, 4.5)])


def test_arc_extent_nested_under_wrapped_tail_ends_at_the_tail():
    # The wrapping pair (6.0 -> 6.5 == 0.2168 after the wrap) covers the
    # nested pair (0.05, 0.2); the arc really ends at the wrapped tail, not
    # at the nested pair's end (the previous algorithm returned 0.2 here,
    # silently truncating the arc).
    umin, imin, umax, imax = arc_extent([(6.0, 6.5), (0.05, 0.2)])
    assert (umin, imin) == (6.0, 0)
    assert (umax, imax) == (6.5, 0)


def test_arc_extent_sleeve_cylinder_seven_faces():
    # Real data: the 7 faces of one R=115 cylinder in Mixed/sleeve.stp (two
    # axial bands whose U intervals nest under a face that starts exactly on
    # the 0/2*pi cut). The previous algorithm split them into 3 "disconnected"
    # groups and raised ValueError, aborting the decomposition of the solid.
    pairs = [
        (2.0173770178394963, 2.171413186947318),
        (2.0943951023934235, 3.085498235515562),
        (6.283185307179581, 8.377580409573),
        (0.991103133121801, 1.1032919692720564),
        (6.283185307179581, 6.360203391733817),
        (6.279270721607444, 6.283185307179587),
        (6.279270721607446, 6.283185307179604),
    ]
    assert arc_extent(pairs) == (6.279270721607444, 5, 3.085498235515562, 1)


def _random_single_arc(rng):
    """A random set of pairs whose union is exactly one arc [s, s+L), L < 2*pi:
    contiguous or chain-overlapping tiles, extra pairs nested inside the arc,
    and sometimes a pair starting exactly on a multiple of 2*pi."""
    s = rng.uniform(-4 * math.pi, 4 * math.pi)
    length = rng.uniform(0.05, 5.9)
    cuts = sorted(rng.uniform(0, length) for _ in range(rng.randint(0, 4)))
    edges = [0.0] + cuts + [length]
    pairs = []
    for a, b in zip(edges[:-1], edges[1:]):
        if b - a < 1e-6:
            continue
        pad = rng.choice([0.0, 0.0, rng.uniform(0, 0.3)])
        pairs.append((s + max(0.0, a - pad), s + min(length, b + pad)))
    if not pairs:
        pairs = [(s, s + length)]
    for _ in range(rng.randint(0, 4)):
        a = rng.uniform(0, length)
        b = rng.uniform(a, length)
        if b - a > 1e-6:
            pairs.append((s + a, s + b))
    if rng.random() < 0.4:
        cut = math.ceil(s / _TWO_PI) * _TWO_PI - s
        if 0 < cut < length:
            eps = rng.choice([0.0, -5e-15, 5e-15])
            pairs.append((s + cut + eps, min(s + length, s + cut + rng.uniform(0.01, length))))
    pairs = [(a + _TWO_PI * m, b + _TWO_PI * m) for (a, b) in pairs for m in [rng.choice([0, 0, 0, -1, 1, 2])]]
    rng.shuffle(pairs)
    return s, length, pairs


def test_arc_extent_random_single_arcs_match_ground_truth():
    # The endpoints must match the true arc to within arc_extent's own 1e-5
    # tolerance for every configuration -- including arcs crossing 0/2*pi with
    # pairs nested under a wrapped tail (about 1 in 8 of these configurations
    # made the previous algorithm raise, and about 1 in 20 return a silently
    # wrong endpoint).
    rng = random.Random(20260918)
    for _ in range(20000):
        s, length, pairs = _random_single_arc(rng)
        umin, imin, umax, imax = arc_extent(pairs)
        assert _ang_diff(umin, s) < 1e-5
        assert _ang_diff(umax, s + length) < 1e-5
        assert umin == pairs[imin][0] and umax == pairs[imax][1]
