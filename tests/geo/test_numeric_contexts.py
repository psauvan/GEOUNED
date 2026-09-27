"""Decisions where a comparison is made with the intrinsic NUMERIC_TOL instead of a user tolerance:
contact between points / edges / faces of one solid, and the full turn of a periodic parameter.

Surface identity is NOT here: it keeps each surface's own user tolerance (tests/geo/test_same_surface_tolerances.py). The
approximation of a surface by an axis-aligned one (PX/PY/PZ, C/X, K/X, TX...) is the same decision as "same surface" and
uses the same tolerance -- a section below pins that.
"""

import math
from types import SimpleNamespace

import pytest

from geouned.geo import GVector
from geouned.geo.constants import NUMERIC_TOL
from geouned.GEOUNED.utils import meta_surfaces_utils as msu
from geouned.GEOUNED.utils.data_classes import NumericFormat, Tolerances
from geouned.GEOUNED.utils.geometry_gu import merge_periodic_uv

Z = GVector(0, 0, 1)
X = GVector(1, 0, 0)


def _tilted(angle):
    return GVector(math.sin(angle), 0.0, math.cos(angle))


# ---------------------------------------------------------------------------
# contact: exactly-touching things sit at distance 0 (measured on the corpus: nothing between 1e-9 and 1e-2)
# ---------------------------------------------------------------------------


class _Edge:
    def __init__(self, distance):
        self.distance = distance

    def my_distToshape(self, other):
        return self.distance


class _Face:
    def __init__(self, edges=(), distance=0.0):
        self.Edges = list(edges)
        self.OuterWire = SimpleNamespace(Edges=list(edges))
        self.distance = distance

    def distToShape(self, other):
        return (self.distance,)


def test_contiguous_face_is_decided_with_numeric_tol_not_the_user_distance():
    assert msu.contiguous_face(_Face([_Edge(0.0)]), _Face([_Edge(0.0)]))
    assert msu.contiguous_face(_Face([_Edge(0.5 * NUMERIC_TOL)]), _Face([_Edge(0.0)]))
    # 1e-5 mm was "touching" with the user's distance (1e-4); it is a real gap now
    assert not msu.contiguous_face(_Face([_Edge(1e-5)]), _Face([_Edge(0.0)]))


def test_common_edge_face_treats_float_noise_as_touching():
    assert msu.commonEdgeFace(_Face(distance=1e-12), _Face()) == []  # `> 0` used to reject this
    assert msu.commonEdgeFace(_Face(distance=0.5 * NUMERIC_TOL), _Face()) == []
    assert msu.commonEdgeFace(_Face(distance=2 * NUMERIC_TOL), _Face()) is None


def test_common_vertex_asks_for_contact_with_numeric_tol(monkeypatch):
    seen = []

    def fake_in_contact(shape_1, shape_2, tolerance=None):
        seen.append(tolerance)
        return False

    monkeypatch.setattr(msu, "shapes_in_contact", fake_in_contact)
    edge = SimpleNamespace()
    setattr(edge, "__native__", object())
    assert msu.commonVertex(edge, edge) == []
    assert seen == [NUMERIC_TOL]


# ---------------------------------------------------------------------------
# a periodic parameter that adds up to a full turn (one function, was two with 1e-6 and 1e-5)
# ---------------------------------------------------------------------------


class _Piece:
    def __init__(self, u0, u1):
        self.ParameterRange = (u0, u1, 0.0, 1.0)


def test_full_turn_within_numeric_tol():
    half = math.pi
    closed, (v_min, v_max) = merge_periodic_uv("U", [_Piece(0.0, half), _Piece(half, 2 * half - 0.5 * NUMERIC_TOL)])
    assert closed and v_max == pytest.approx(v_min + 2 * math.pi)


def test_a_gap_larger_than_numeric_tol_is_not_a_full_turn():
    closed, _ = merge_periodic_uv("U", [_Piece(0.0, math.pi), _Piece(math.pi, 2 * math.pi - 1e-5)])
    assert not closed  # 1e-5 rad used to pass as a full turn (PARAM_ANGLE_TOL_E5 in one copy, 1e-6 in the other)


def test_merge_periodic_uv_rejects_an_unknown_parameter():
    with pytest.raises(ValueError):
        merge_periodic_uv("W", [_Piece(0.0, 1.0)])


def test_relative_precision_and_value_are_kept_but_the_periodic_merge_does_not_read_them():
    import inspect

    tol = Tolerances(relativePrecision=2.0e-6, value=3.0e-6)
    assert (tol.relativePrecision, tol.value) == (2.0e-6, 3.0e-6)  # still public, still validated, kept by scaled()
    assert (tol.scaled(1.0e6).relativePrecision, tol.scaled(1.0e6).value) == (2.0e-6, 3.0e-6)
    assert list(inspect.signature(merge_periodic_uv).parameters) == ["parameter", "faces"]  # no tolerance to read


# ---------------------------------------------------------------------------
# the approximation "this surface is axis-aligned" (PX/PY/PZ, C/X.., K/X.., TX..) is the SAME decision as "same
# surface": it must flip at exactly the tilt where the identity flips, so it uses the identity's own tolerance
# ---------------------------------------------------------------------------


def _write(surface_type, surface, tolerances):
    from geouned.GEOUNED.write.functions import mcnp_surface

    return mcnp_surface(1, surface_type, surface, SimpleNamespace(prnt3PPlane=False), tolerances, NumericFormat())


TILT = 5.0e-5  # between 1e-5 and 1e-4


@pytest.mark.parametrize(
    "field, tolerance, same",
    [("cyl_angle", 1.0e-4, True), ("cyl_angle", 1.0e-5, False)],
)
def test_cylinder_is_written_axis_aligned_exactly_when_it_is_the_same_cylinder(field, tolerance, same):
    from geouned.GEOUNED.utils.basic_functions_part2 import is_same_cylinder

    tol = Tolerances(**{field: tolerance})
    aligned = SimpleNamespace(Axis=Z, Center=GVector(0, 0, 0), Radius=10.0)
    tilted = SimpleNamespace(Axis=_tilted(TILT), Center=GVector(0, 0, 0), Radius=10.0)
    assert is_same_cylinder(aligned, tilted, tolerances=tol) is same
    assert ("CZ" in _write("CylinderOnly", tilted, tol)) is same


@pytest.mark.parametrize("tolerance, same", [(1.0e-4, True), (1.0e-5, False)])
def test_plane_is_written_axis_aligned_exactly_when_it_is_the_same_plane(tolerance, same):
    from geouned.GEOUNED.utils.basic_functions_part2 import is_same_plane

    tol = Tolerances(pln_angle=tolerance)
    aligned = SimpleNamespace(Axis=X, Position=GVector(5, 0, 0), pointDef=False, real=True)
    tilted = SimpleNamespace(Axis=GVector(math.cos(TILT), 0.0, math.sin(TILT)), Position=GVector(5, 0, 0), pointDef=False, real=True)
    assert is_same_plane(aligned, tilted, tolerances=tol) is same
    assert ("PX" in _write("Plane", tilted, tol)) is same


@pytest.mark.parametrize("tolerance, same", [(1.0e-4, True), (1.0e-5, False)])
def test_cone_is_written_axis_aligned_exactly_when_it_is_the_same_cone(tolerance, same):
    from geouned.geo.surface_geometry import is_same_cone_surface

    tol = Tolerances(kne_angle=tolerance)
    aligned = SimpleNamespace(Apex=GVector(0, 0, 0), Axis=Z, SemiAngle=0.3)
    tilted = SimpleNamespace(Apex=GVector(0, 0, 0), Axis=_tilted(TILT), SemiAngle=0.3)
    assert is_same_cone_surface(aligned, tilted, tol) is same
    assert ("KZ" in _write("ConeOnly", tilted, tol)) is same


@pytest.mark.parametrize("tolerance, same", [(1.0e-4, True), (1.0e-5, False)])
def test_torus_is_written_axis_aligned_exactly_when_it_is_the_same_torus(tolerance, same):
    from geouned.geo.surface_geometry import is_same_torus_surface

    tol = Tolerances(tor_angle=tolerance)
    fields = dict(Center=GVector(0, 0, 0), MajorRadius=20.0, MinorRadius=3.0, Degenerated=False, a_sign=1)
    aligned = SimpleNamespace(Axis=Z, **fields)
    tilted = SimpleNamespace(Axis=_tilted(TILT), **fields)
    assert is_same_torus_surface(aligned, tilted, tol) is same
    assert ("TZ" in _write("TorusOnly", tilted, tol)) is same


def test_each_writer_branch_reads_its_own_surface_tolerance_not_a_neighbours():
    # a loose cyl_angle must not make a plane or a cone axis-aligned, and vice versa
    tol = Tolerances(cyl_angle=1.0e-3, pln_angle=1.0e-6, kne_angle=1.0e-6, tor_angle=1.0e-6)
    tilted_plane = SimpleNamespace(Axis=GVector(math.cos(TILT), 0.0, math.sin(TILT)), Position=GVector(5, 0, 0), pointDef=False)
    tilted_cyl = SimpleNamespace(Axis=_tilted(TILT), Center=GVector(0, 0, 0), Radius=10.0)
    assert "PX" not in _write("Plane", tilted_plane, tol)
    assert "CZ" in _write("CylinderOnly", tilted_cyl, tol)


def test_cone_apex_plane_uses_the_cone_tolerance_and_gen_torus_the_torus_one():
    from geouned.GEOUNED.conversion.cell_definition_functions import cone_apex_plane, gen_torus

    cone = lambda axis: SimpleNamespace(Surface=SimpleNamespace(Axis=axis, Apex=GVector(0, 0, 0)))
    assert cone_apex_plane(cone(_tilted(TILT)), Tolerances(kne_angle=1.0e-4)) is None  # snapped: no apex plane needed
    assert cone_apex_plane(cone(_tilted(TILT)), Tolerances(kne_angle=1.0e-5)) is not None

    torus = lambda axis: SimpleNamespace(Surface=SimpleNamespace(Axis=axis, Center=GVector(0, 0, 0), MajorRadius=20.0, MinorRadius=3.0, a_sign=1))
    assert gen_torus(torus(_tilted(TILT)), Tolerances(tor_angle=1.0e-4)) is not None  # snapped to TZ, keeps the torus
    assert gen_torus(torus(_tilted(TILT)), Tolerances(tor_angle=1.0e-5)) is None


def test_remove_box_faces_matches_a_box_face_within_the_plane_tolerance():
    from geouned.GEOUNED.utils.build_shape_functions import remove_box_faces

    face = lambda axis: SimpleNamespace(Surface=SimpleNamespace(Axis=axis, Position=GVector(1.0, 0, 0)))
    boxlim = [1.0, 0.0, 0.0, 9.0, 0.0, 0.0]
    tilted = GVector(math.cos(TILT), 0.0, math.sin(TILT))
    assert remove_box_faces(["cap"], [face(tilted)], boxlim, tolerances=Tolerances(pln_angle=1.0e-4)) == []
    assert remove_box_faces(["cap"], [face(tilted)], boxlim, tolerances=Tolerances(pln_angle=1.0e-5)) == ["cap"]


# ---------------------------------------------------------------------------
# D9 -- an accidental `from turtle import distance` made this module need tkinter
# ---------------------------------------------------------------------------


def test_basic_functions_do_not_import_turtle():
    from geouned.GEOUNED.utils import basic_functions_part1

    assert "turtle" not in open(basic_functions_part1.__file__, encoding="utf-8").read()


# ---------------------------------------------------------------------------
# spline_2D: is a BSpline edge planar? -- an intrinsic constant (how the CAD stores the curve), no user tolerance
# ---------------------------------------------------------------------------


class _SplineEdge:
    """A circle of radius 1 sampled at 3 knots; the plane of the LAST knot is tilted by `tilt` radians about X."""

    def __init__(self, tilt):
        self.tilt = tilt
        self.thetas = [0.0, 1.0, 2.0]

    def knots(self):
        return list(range(3))

    def curvature(self, k):
        return 1.0

    def _tilt(self, k):
        return self.tilt if k == 2 else 0.0

    def _rot(self, v, k):
        a = self._tilt(k)  # rotation about X
        return GVector(v.x, v.y * math.cos(a) - v.z * math.sin(a), v.y * math.sin(a) + v.z * math.cos(a))

    def derivative1_at(self, k):
        t = self.thetas[k]
        return self._rot(GVector(-math.sin(t), math.cos(t), 0.0), k)

    def normal_at(self, k):
        t = self.thetas[k]
        return self._rot(GVector(-math.cos(t), -math.sin(t), 0.0), k)


def test_spline_2d_uses_the_intrinsic_planarity_constant():
    from geouned.geo.constants import SPLINE_PLANARITY_ANGLE

    assert SPLINE_PLANARITY_ANGLE == 1.0e-3
    assert msu.spline_2D(_SplineEdge(0.0))  # exactly planar
    assert msu.spline_2D(_SplineEdge(0.5 * SPLINE_PLANARITY_ANGLE))  # fitting noise inside the constant
    assert not msu.spline_2D(_SplineEdge(2.0 * SPLINE_PLANARITY_ANGLE))  # a genuinely 3D curve


def test_spline_2d_takes_no_tolerance_argument():
    import inspect

    assert list(inspect.signature(msu.spline_2D).parameters) == ["edge"]
