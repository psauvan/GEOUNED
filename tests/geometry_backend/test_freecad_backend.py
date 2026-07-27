import math

import pytest

pytest.importorskip("FreeCAD", reason="FreeCAD not available on this machine")
pytest.importorskip("Part", reason="Part (FreeCAD lib) not available on this machine")
pytest.importorskip("BOPTools.SplitAPI", reason="BOPTools not available on this machine")

from geouned.geometry_backend.freecad_backend import FreeCADBackend
from geouned.geometry_backend.geometry_backend_interface import (
    CurveType,
    GVector,
    PlaneParams,
    SurfaceType,
)


@pytest.fixture
def backend():
    return FreeCADBackend()


@pytest.fixture
def unit_box(backend):
    return backend.make_box(-5, -5, -5, 5, 5, 5)


# -- Primitive construction -------------------------------------------------

def test_make_box(backend):
    box = backend.make_box(0, 0, 0, 10, 10, 10)
    assert backend.volume(box) == pytest.approx(1000.0)
    assert backend.bounding_box(box) == pytest.approx((0, 0, 0, 10, 10, 10))


def test_make_cylinder(backend):
    cylinder = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 2.0, 5.0)
    assert backend.volume(cylinder) == pytest.approx(math.pi * 2.0**2 * 5.0)


def test_make_cone(backend):
    apex = GVector(0, 0, 10)
    cone = backend.make_cone(apex, GVector(0, 0, -1), math.atan(2.0 / 10.0), 10.0)
    assert backend.volume(cone) == pytest.approx((1 / 3) * math.pi * 2.0**2 * 10.0, rel=1e-4)


def test_make_sphere(backend):
    sphere = backend.make_sphere(GVector(0, 0, 0), 3.0)
    assert backend.volume(sphere) == pytest.approx((4 / 3) * math.pi * 3.0**3, rel=1e-4)


def test_make_torus(backend):
    torus = backend.make_torus(GVector(0, 0, 0), GVector(0, 0, 1), 10.0, 2.0)
    assert backend.volume(torus) == pytest.approx(2 * math.pi**2 * 10.0 * 2.0**2, rel=1e-4)


def test_make_half_space_cuts_box_in_half(backend, unit_box):
    half_space = backend.make_half_space(PlaneParams(GVector(0, 0, 0), GVector(0, 0, 1)))
    remaining = backend.cut(unit_box, [half_space])
    assert len(remaining) == 1
    assert backend.volume(remaining[0]) == pytest.approx(500.0)


def test_make_wire_round_trips_outer_wire_edges(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    outer = backend.get_outer_wire(face)
    edges = backend.get_wire_edges(outer)
    assert len(edges) == 4

    rebuilt = backend.make_wire(edges)
    assert len(backend.get_wire_edges(rebuilt)) == 4


# -- Boolean operations -------------------------------------------------------

def test_cut_full_containment_returns_no_solids(backend, unit_box):
    containing_tool = backend.make_half_space(PlaneParams(GVector(0, 0, 5), GVector(0, 0, 1)))
    result = backend.cut(unit_box, [containing_tool])
    assert result == []


def test_fuse(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(10, 0, 0, 20, 10, 10)
    fused = backend.fuse([box_a, box_b])
    assert backend.volume(fused) == pytest.approx(2000.0)


def test_split_interior_plane_gives_two_solids(backend, unit_box):
    interior_plane = backend.make_half_space(PlaneParams(GVector(0, 0, 0), GVector(0, 0, 1)))
    result = backend.split(unit_box, interior_plane, 1e-6)
    assert len(result.solids) == 2


def test_split_plane_coincident_with_existing_face_is_a_known_limitation(backend, unit_box):
    """
    Known limitation documented in freecad_backend.py's module docstring:
    when the split tool's boundary coincides with an existing face of the
    solid (here, the tool's plane sits exactly on the box's own z=5 face),
    BOPTools.SplitAPI.slice silently returns the solid unchanged instead of
    raising or flagging the degenerate case. This is the motivating
    tangency bug for the whole pyOCC migration -- FreeCADBackend does NOT
    fix it, only OCCBackend is expected to. This test documents today's
    actual (buggy) behavior so a regression is visible, not to assert it
    is correct.
    """
    coincident_tool = backend.make_half_space(PlaneParams(GVector(0, 0, 5), GVector(0, 0, 1)))
    result = backend.split(unit_box, coincident_tool, 1e-6)
    assert len(result.solids) == 1
    assert result.degenerate_case_handled is False


# -- Topological traversal --------------------------------------------------

def test_get_faces_edges_vertices(backend, unit_box):
    faces = backend.get_faces(unit_box)
    assert len(faces) == 6
    edges = backend.get_edges(faces[0])
    assert len(edges) == 4
    vertices = backend.get_vertices(edges[0])
    assert len(vertices) == 2
    point = backend.get_vertex_point(vertices[0])
    assert isinstance(point, GVector)


def test_faces_sharing_edge(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = backend.get_edges(face)[0]
    sharing = backend.faces_sharing_edge(unit_box, edge)
    assert len(sharing) == 2


# -- Surface and curve classification ----------------------------------------

def test_classify_surface_plane(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    geometry = backend.classify_surface(face)
    assert geometry.surface_type is SurfaceType.PLANE


def test_classify_surface_cylinder(backend):
    cylinder = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(
        f for f in backend.get_faces(cylinder)
        if backend.classify_surface(f).surface_type is SurfaceType.CYLINDER
    )
    geometry = backend.classify_surface(side_face)
    assert geometry.surface_type is SurfaceType.CYLINDER
    assert geometry.params.radius == pytest.approx(3.0)


def test_classify_edge_line(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = backend.get_edges(face)[0]
    geometry = backend.classify_edge(edge)
    assert geometry.curve_type is CurveType.LINE


def test_classify_edge_circle(backend):
    cylinder = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(
        f for f in backend.get_faces(cylinder)
        if backend.classify_surface(f).surface_type is SurfaceType.CYLINDER
    )
    circle_edge = next(
        e for e in backend.get_edges(side_face)
        if backend.classify_edge(e).curve_type is CurveType.CIRCLE
    )
    geometry = backend.classify_edge(circle_edge)
    assert geometry.curve_type is CurveType.CIRCLE
    assert geometry.params.radius == pytest.approx(3.0)


def test_face_orientation_outward_is_true_for_all_box_faces(backend, unit_box):
    for face in backend.get_faces(unit_box):
        assert backend.face_orientation_outward(unit_box, face) is True


# -- Face and edge parametric queries -----------------------------------------

def test_parameter_range_and_face_value_at(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    u_min, u_max, v_min, v_max = backend.parameter_range(face)
    mid_point = backend.face_value_at(face, (u_min + u_max) / 2, (v_min + v_max) / 2)
    normal = backend.face_normal_at(face, (u_min + u_max) / 2, (v_min + v_max) / 2)
    assert isinstance(mid_point, GVector)
    assert normal.length == pytest.approx(1.0)


def test_edge_parameter_range_and_value_at(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = backend.get_edges(face)[0]
    u_min, u_max = backend.edge_parameter_range(edge)
    mid_point = backend.edge_value_at(edge, (u_min + u_max) / 2)
    assert isinstance(mid_point, GVector)


def test_tessellate(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    points = backend.tessellate(face, 0.5)
    assert len(points) > 0
    assert all(isinstance(p, GVector) for p in points)


# -- Spatial queries -----------------------------------------------------------

def test_is_inside(backend, unit_box):
    assert backend.is_inside(unit_box, GVector(0, 0, 0), 1e-7) is True
    assert backend.is_inside(unit_box, GVector(100, 100, 100), 1e-7) is False


def test_in_contact_touching_solids(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(10, 0, 0, 20, 10, 10)
    assert backend.in_contact(box_a, box_b, 1e-6) is True


def test_in_contact_far_apart_solids(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(100, 100, 100, 110, 110, 110)
    assert backend.in_contact(box_a, box_b, 1e-6) is False


# -- Validation / diagnostics --------------------------------------------------

def test_is_valid(backend, unit_box):
    assert backend.is_valid(unit_box) is True


def test_fix_shape_preserves_volume_of_a_valid_solid(backend, unit_box):
    fixed = backend.fix_shape(unit_box, 1e-6)
    assert backend.volume(fixed) == pytest.approx(backend.volume(unit_box))


# -- Transformations ------------------------------------------------------------

def test_translate(backend):
    box = backend.make_box(0, 0, 0, 10, 10, 10)
    translated = backend.translate(box, GVector(5, 0, 0))
    assert backend.bounding_box(translated) == pytest.approx((5, 0, 0, 15, 10, 10))


def test_rotate_quarter_turn_around_z(backend, unit_box):
    rotated = backend.rotate(unit_box, GVector(0, 0, 0), GVector(0, 0, 1), math.pi / 2)
    assert backend.bounding_box(rotated) == pytest.approx(backend.bounding_box(unit_box))


# -- I/O ------------------------------------------------------------------------

def test_step_export_load_roundtrip(backend, unit_box, tmp_path):
    step_file = tmp_path / "roundtrip.step"
    backend.export_step([unit_box], str(step_file))
    loaded = backend.load_step(str(step_file))
    assert len(loaded) == 1
    assert backend.volume(loaded[0]) == pytest.approx(backend.volume(unit_box))
