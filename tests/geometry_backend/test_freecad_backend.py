import math
from pathlib import Path

import pytest

pytest.importorskip("FreeCAD", reason="FreeCAD not available on this machine")
pytest.importorskip("Part", reason="Part (FreeCAD lib) not available on this machine")
pytest.importorskip("BOPTools.SplitAPI", reason="BOPTools not available on this machine")

import FreeCAD
import Part

from geouned.geometry_backend.freecad_backend import FreeCADBackend
from geouned.geometry_backend.geometry_backend_interface import (
    GBoundBox,
    GCircle,
    GCylinder,
    GLine,
    GPlane,
    GShell,
    GVector,
)
from geouned.geometry_backend import vector_geometry


@pytest.fixture
def backend():
    return FreeCADBackend()


@pytest.fixture
def unit_box(backend):
    return backend.make_box(-5, -5, -5, 5, 5, 5)


def _bbox_tuple(bbox):
    return (bbox.XMin, bbox.YMin, bbox.ZMin, bbox.XMax, bbox.YMax, bbox.ZMax)


# -- Primitive construction -------------------------------------------------

def test_make_box(backend):
    box = backend.make_box(0, 0, 0, 10, 10, 10)
    assert backend.volume(box) == pytest.approx(1000.0)
    bbox = backend.bounding_box(box)
    assert type(bbox) is GBoundBox
    assert _bbox_tuple(bbox) == pytest.approx((0, 0, 0, 10, 10, 10))


def test_gsolid_boundbox_is_populated_eagerly(backend):
    box = backend.make_box(0, 0, 0, 10, 10, 10)
    assert type(box.BoundBox) is GBoundBox
    assert _bbox_tuple(box.BoundBox) == pytest.approx((0, 0, 0, 10, 10, 10))


def test_gsolid_export_step_convenience_method(backend, unit_box, tmp_path):
    step_file = tmp_path / "solid.step"
    unit_box.export_step(str(step_file))
    loaded = backend.load_step(str(step_file))
    assert len(loaded) == 1
    assert backend.volume(loaded[0]) == pytest.approx(backend.volume(unit_box))


def test_gface_export_step_convenience_method(backend, unit_box, tmp_path):
    step_file = tmp_path / "face.step"
    face = backend.get_faces(unit_box)[0]
    face.export_step(str(step_file))
    loaded = backend.load_step(str(step_file))
    assert loaded == []  # a bare face has no Solids, but the file must still be valid/loadable


def test_optimal_bounding_box_matches_bounding_box_for_a_box(backend):
    box = backend.make_box(0, 0, 0, 10, 10, 10)
    assert _bbox_tuple(backend.optimal_bounding_box(box)) == pytest.approx(_bbox_tuple(backend.bounding_box(box)))


def test_get_solid_vertices(backend, unit_box):
    vertices = backend.get_solid_vertices(unit_box)
    assert len(vertices) == 8


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
    half_space = backend.make_half_space(GPlane(GVector(0, 0, 0), GVector(0, 0, 1)))
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


def test_make_polygon_face(backend):
    points = [GVector(0, 0, 0), GVector(10, 0, 0), GVector(10, 10, 0), GVector(0, 10, 0)]
    face = backend.make_polygon_face(points)
    assert type(face.Surface) is GPlane
    assert backend.area(face) == pytest.approx(100.0)


def test_make_shell_from_box_faces(backend, unit_box):
    faces = backend.get_faces(unit_box)
    shell = backend.make_shell(faces)
    assert type(shell) is GShell
    assert len(shell.Faces) == 6
    assert shell.Orientation in ("Forward", "Reversed")


def test_make_shell_supports_in_contact_and_distance(backend, unit_box):
    faces = backend.get_faces(unit_box)
    shell = backend.make_shell(faces[:1])
    other_box = backend.make_box(100, 100, 100, 110, 110, 110)
    assert backend.in_contact(shell, unit_box, 1e-6) is True
    assert backend.in_contact(shell, other_box, 1e-6) is False
    assert backend.distance(shell, other_box) > 0


# -- Boolean operations -------------------------------------------------------

def test_cut_full_containment_returns_no_solids(backend, unit_box):
    containing_tool = backend.make_half_space(GPlane(GVector(0, 0, 5), GVector(0, 0, 1)))
    result = backend.cut(unit_box, [containing_tool])
    assert result == []


def test_fuse(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(10, 0, 0, 20, 10, 10)
    fused = backend.fuse([box_a, box_b])
    assert backend.volume(fused) == pytest.approx(2000.0)


def test_make_compound(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(100, 100, 100, 110, 110, 110)
    compound = backend.make_compound([box_a, box_b])
    assert backend.volume(compound) == pytest.approx(2000.0)


def test_reverse_flips_volume_sign(backend, unit_box):
    reversed_box = backend.reverse(unit_box)
    assert backend.volume(reversed_box) == pytest.approx(-backend.volume(unit_box))
    # original must be untouched
    assert backend.volume(unit_box) > 0


def test_refine_keeps_volume_of_coplanar_fuse(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(10, 0, 0, 20, 10, 10)
    fused = backend.fuse([box_a, box_b])
    refined = backend.refine(fused)
    assert backend.volume(refined) == pytest.approx(backend.volume(fused))


def test_split_interior_plane_gives_two_solids(backend, unit_box):
    interior_plane = backend.make_half_space(GPlane(GVector(0, 0, 0), GVector(0, 0, 1)))
    result = backend.split(unit_box, interior_plane, 1e-6)
    assert len(result.solids) == 2


def test_split_scale_up_floor_bootstraps_from_tiny_tolerance(backend, unit_box):
    # mirrors GEOUNED's public Options.scaleUp / Options.splitTolerance:
    # starting tolerance of 0.0 with scale_up_floor set must not just
    # attempt the tiny tolerance as-is -- it climbs back up from the floor.
    interior_plane = backend.make_half_space(GPlane(GVector(0, 0, 0), GVector(0, 0, 1)))
    result = backend.split(unit_box, interior_plane, 0.0, scale_up_floor=0.0)
    assert len(result.solids) == 2


def test_split_tool_not_intersecting_solid_returns_solid_unchanged(backend, unit_box):
    far_face = backend.make_polygon_face([
        GVector(1000, -500, -500), GVector(1000, 500, -500),
        GVector(1000, 500, 500), GVector(1000, -500, 500),
    ])
    result = backend.split(unit_box, far_face, 1e-6)
    assert len(result.solids) == 1
    assert result.degenerate_case_handled is True
    assert backend.volume(result.solids[0]) == pytest.approx(backend.volume(unit_box))


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
    coincident_tool = backend.make_half_space(GPlane(GVector(0, 0, 5), GVector(0, 0, 1)))
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
    assert isinstance(vertices[0].Point, GVector)


def test_get_faces_are_eagerly_enriched(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    assert type(face.Surface) is GPlane
    assert len(face.Edges) == 4
    assert all(type(e.Curve) is GLine for e in face.Edges)
    assert face.OuterWire is not None
    assert face.ParameterRange == pytest.approx(backend.parameter_range(face))
    assert face.index in range(6)
    assert face.Orientation in ("Forward", "Reversed")


def test_get_edges_are_eagerly_enriched(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = face.Edges[0]
    assert type(edge.Curve) is GLine
    assert len(edge.Vertexes) == 2
    assert edge.ParameterRange == pytest.approx(backend.edge_parameter_range(edge))
    assert edge.Orientation in ("Forward", "Reversed")


def test_get_solid_orientation(backend, unit_box):
    assert unit_box.Orientation in ("Forward", "Reversed")


def test_outer_wire_single_wire_face_matches_only_wire(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    outer_edges = backend.get_wire_edges(face.OuterWire)
    assert len(outer_edges) == 4


def test_outer_wire_picks_outer_boundary_not_a_hole(backend):
    """
    A face with a through-hole has two wires: the outer boundary and the
    hole's inner wire. GEOUNED's own heuristic (largest mean
    vertex-to-centroid distance) must pick the outer one -- this is the
    exact case where FreeCAD's native Face.OuterWire was found to pick
    the wrong wire, which is why get_outer_wire() doesn't use it.
    """
    box = Part.makeBox(20, 20, 20)
    hole = Part.makeCylinder(3, 22, FreeCAD.Vector(10, 10, -1), FreeCAD.Vector(0, 0, 1))
    drilled = box.cut(hole)
    solid = backend._wrap_solid(drilled)

    top_face = next(
        f for f in backend.get_faces(solid)
        if type(f.Surface) is GPlane and f.Surface.Position.z == pytest.approx(20)
    )
    assert len(top_face.native.Wires) == 2
    assert len(top_face.Edges) == 5  # 4 outer + 1 circular hole

    outer_edges = backend.get_wire_edges(top_face.OuterWire)
    assert len(outer_edges) == 4


def test_faces_sharing_edge(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = backend.get_edges(face)[0]
    sharing = backend.faces_sharing_edge(unit_box, edge)
    assert len(sharing) == 2


def test_is_same_edge(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edges = backend.get_edges(face)
    assert backend.is_same_edge(edges[0], edges[0]) is True
    assert backend.is_same_edge(edges[0], edges[1]) is False


def test_is_same_vertex(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = backend.get_edges(face)[0]
    vertices = backend.get_vertices(edge)
    assert backend.is_same_vertex(vertices[0], vertices[0]) is True
    assert backend.is_same_vertex(vertices[0], vertices[1]) is False


# -- Surface and curve classification ----------------------------------------

def test_classify_surface_plane(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    geometry = backend.classify_surface(face)
    assert type(geometry) is GPlane


def test_classify_surface_cylinder(backend):
    cylinder = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(
        f for f in backend.get_faces(cylinder)
        if type(f.Surface) is GCylinder
    )
    geometry = backend.classify_surface(side_face)
    assert type(geometry) is GCylinder
    assert geometry.Radius == pytest.approx(3.0)


def test_classify_surface_populates_x_dir_for_plane_and_cylinder(backend, unit_box):
    plane_face = backend.get_faces(unit_box)[0]
    plane = backend.classify_surface(plane_face)
    assert plane.XDir is not None
    assert plane.XDir.length == pytest.approx(1.0)
    assert abs(plane.XDir.dot(plane.Axis)) < 1e-9

    cylinder_solid = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in backend.get_faces(cylinder_solid) if type(f.Surface) is GCylinder)
    cylinder = backend.classify_surface(side_face)
    assert cylinder.XDir is not None
    assert cylinder.XDir.length == pytest.approx(1.0)
    assert abs(cylinder.XDir.dot(cylinder.Axis)) < 1e-9


def test_plane_value_and_tangent_at_match_native(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    plane = backend.classify_surface(face)
    u, v = 1.3, -0.7
    expected_point = backend.face_value_at(face, u, v)
    expected_tangent_u, expected_tangent_v = backend.face_tangent_at(face, u, v)

    point = vector_geometry.plane_value_at(plane, u, v)
    tangent_u, tangent_v = vector_geometry.plane_tangent_at(plane, u, v)

    assert point.is_equal(expected_point, 1e-6)
    assert tangent_u.is_equal(expected_tangent_u, 1e-6)
    assert tangent_v.is_equal(expected_tangent_v, 1e-6)


def test_cylinder_value_and_tangent_at_match_native_off_axis(backend):
    # Deliberately not aligned to any global axis: a bug that only shows up
    # for an arbitrary XDir/YDir frame would pass undetected on a Z-aligned
    # cylinder.
    base = GVector(3, -1, 7)
    axis_dir = GVector(1, 1, 1).normalized()
    cylinder_solid = backend.make_cylinder(base, axis_dir, 4.0, 15.0)
    side_face = next(f for f in backend.get_faces(cylinder_solid) if type(f.Surface) is GCylinder)
    cylinder = backend.classify_surface(side_face)

    u, v = 0.9, 3.3
    expected_point = backend.face_value_at(side_face, u, v)
    expected_tangent_u, expected_tangent_v = backend.face_tangent_at(side_face, u, v)

    point = vector_geometry.cylinder_value_at(cylinder, u, v)
    tangent_u, tangent_v = vector_geometry.cylinder_tangent_at(cylinder, u, v)

    assert point.is_equal(expected_point, 1e-6)
    assert tangent_u.is_equal(expected_tangent_u, 1e-6)
    assert tangent_v.is_equal(expected_tangent_v, 1e-6)


def test_classify_edge_line(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = backend.get_edges(face)[0]
    geometry = backend.classify_edge(edge)
    assert type(geometry) is GLine


def test_classify_edge_circle(backend):
    cylinder = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(
        f for f in backend.get_faces(cylinder)
        if type(f.Surface) is GCylinder
    )
    circle_edge = next(e for e in side_face.Edges if type(e.Curve) is GCircle)
    geometry = backend.classify_edge(circle_edge)
    assert type(geometry) is GCircle
    assert geometry.Radius == pytest.approx(3.0)


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


def test_face_parameter_at_is_inverse_of_face_value_at(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    u_min, u_max, v_min, v_max = backend.parameter_range(face)
    u, v = (u_min + u_max) / 2, (v_min + v_max) / 2
    point = backend.face_value_at(face, u, v)
    round_tripped_u, round_tripped_v = backend.face_parameter_at(face, point)
    assert round_tripped_u == pytest.approx(u)
    assert round_tripped_v == pytest.approx(v)


def test_face_tangent_at_returns_unit_orthogonal_vectors(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    u_min, u_max, v_min, v_max = backend.parameter_range(face)
    u, v = (u_min + u_max) / 2, (v_min + v_max) / 2
    tangent_u, tangent_v = backend.face_tangent_at(face, u, v)
    normal = backend.face_normal_at(face, u, v)
    assert tangent_u.length == pytest.approx(1.0)
    assert tangent_v.length == pytest.approx(1.0)
    assert abs(tangent_u.dot(tangent_v)) < 1e-9
    assert tangent_u.cross(tangent_v).is_equal(normal, 1e-6) or tangent_u.cross(tangent_v).is_equal(-normal, 1e-6)


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


def test_face_get_uv_nodes_matches_tessellate_point_count(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    points = backend.tessellate(face, 0.5)
    uv_nodes = backend.face_get_uv_nodes(face, 0.5)
    assert len(uv_nodes) == len(points)
    for u, v in uv_nodes:
        assert isinstance(u, float)
        assert isinstance(v, float)


def test_edge_length_of_unit_box_edge(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    edge = backend.get_edges(face)[0]
    assert backend.edge_length(edge) == pytest.approx(10.0)


def test_edge_derivative1_at_is_not_unit_normalized_on_a_circle(backend):
    cylinder = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in backend.get_faces(cylinder) if type(f.Surface) is GCylinder)
    circle_edge = next(e for e in side_face.Edges if type(e.Curve) is GCircle)
    u_min, u_max = backend.edge_parameter_range(circle_edge)
    derivative = backend.edge_derivative1_at(circle_edge, (u_min + u_max) / 2)
    assert derivative.length == pytest.approx(3.0)


def test_edge_normal_at_on_a_circle(backend):
    cylinder = backend.make_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in backend.get_faces(cylinder) if type(f.Surface) is GCylinder)
    circle_edge = next(e for e in side_face.Edges if type(e.Curve) is GCircle)
    u_min, u_max = backend.edge_parameter_range(circle_edge)
    normal = backend.edge_normal_at(circle_edge, (u_min + u_max) / 2)
    assert normal.length == pytest.approx(1.0)


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


def test_distance_touching_solids_is_zero(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(10, 0, 0, 20, 10, 10)
    assert backend.distance(box_a, box_b) == pytest.approx(0.0)


def test_distance_far_apart_solids(backend):
    box_a = backend.make_box(0, 0, 0, 10, 10, 10)
    box_b = backend.make_box(20, 0, 0, 30, 10, 10)
    assert backend.distance(box_a, box_b) == pytest.approx(10.0)


def test_find_interior_point_trivial_box(backend, unit_box):
    point = backend.find_interior_point(unit_box)
    assert point is not None
    assert backend.is_inside(unit_box, point, 0.0) is True


def test_find_interior_point_torus_center_of_mass_outside(backend):
    # a torus's center of mass sits in the hole, not inside the solid
    torus = backend.make_torus(GVector(0, 0, 0), GVector(0, 0, 1), 20.0, 5.0)
    point = backend.find_interior_point(torus)
    assert point is not None
    assert backend.is_inside(torus, point, 0.0) is True


def test_find_interior_point_thin_torus(backend):
    # regression: a thin ring is exactly the shape a RoundCorner/TCone
    # fillet produces. Alternative strategies (vertex-to-vertex segment
    # sampling, bounding-box octree subdivision) either take orders of
    # magnitude longer or fail outright here within their fixed
    # subdivision depth -- face-normal probing does not have that
    # weakness (see benchmark in the migration notes).
    torus = backend.make_torus(GVector(0, 0, 0), GVector(0, 0, 1), 200.0, 0.2)
    point = backend.find_interior_point(torus)
    assert point is not None
    assert backend.is_inside(torus, point, 0.0) is True


def test_is_part_of_domain_at_face_center(backend, unit_box):
    face = backend.get_faces(unit_box)[0]
    u_min, u_max, v_min, v_max = backend.parameter_range(face)
    u = 0.5 * (u_min + u_max)
    v = 0.5 * (v_min + v_max)
    assert backend.is_part_of_domain(face, u, v) is True


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
    assert _bbox_tuple(backend.bounding_box(translated)) == pytest.approx((5, 0, 0, 15, 10, 10))


def test_rotate_quarter_turn_around_z(backend, unit_box):
    rotated = backend.rotate(unit_box, GVector(0, 0, 0), GVector(0, 0, 1), math.pi / 2)
    assert _bbox_tuple(backend.bounding_box(rotated)) == pytest.approx(_bbox_tuple(backend.bounding_box(unit_box)))


# -- I/O ------------------------------------------------------------------------

def test_kernel_version_returns_a_dotted_string(backend):
    version = backend.kernel_version()
    assert isinstance(version, str)
    assert version.count(".") == 2


def test_step_export_load_roundtrip(backend, unit_box, tmp_path):
    step_file = tmp_path / "roundtrip.step"
    backend.export_step([unit_box], str(step_file))
    loaded = backend.load_step(str(step_file))
    assert len(loaded) == 1
    assert backend.volume(loaded[0]) == pytest.approx(backend.volume(unit_box))


def test_load_step_labels_matches_load_step_solid_count(backend, unit_box, tmp_path):
    step_file = tmp_path / "labels.step"
    backend.export_step([unit_box], str(step_file))
    solids = backend.load_step(str(step_file))
    nodes = backend.load_step_labels(str(step_file))
    assert sum(n.n_solids for n in nodes) == len(solids)


def test_load_step_labels_against_real_hierarchical_file(backend):
    step_file = Path(__file__).resolve().parents[2] / "testing" / "inputSTEP" / "Misc" / "rails.stp"
    solids = backend.load_step(str(step_file))
    nodes = backend.load_step_labels(str(step_file))

    assert sum(n.n_solids for n in nodes) == len(solids)

    enclosure_nodes = [n for n in nodes if n.parent is not None and n.parent.label.startswith("enclosure")]
    assert len(enclosure_nodes) > 0
    for n in enclosure_nodes:
        # walking .parent must reach all the way up to a root (no parent)
        ancestor = n
        depth = 0
        while ancestor.parent is not None:
            ancestor = ancestor.parent
            depth += 1
            assert depth < 100  # guards against an accidental cycle
        assert depth > 0
