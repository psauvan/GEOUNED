import math
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("FreeCAD", reason="FreeCAD not available on this machine")
pytest.importorskip("Part", reason="Part (FreeCAD lib) not available on this machine")
pytest.importorskip("BOPTools.SplitAPI", reason="BOPTools not available on this machine")

import FreeCAD
import Part

from geouned.geo import (
    GBoundBox,
    GCircle,
    GCylinder,
    GLine,
    GPlane,
    GShell,
    GSolid,
    GVector,
    Gclassify_surface,
    Gcommon,
    Gcut,
    Gdistance,
    Gexport_step,
    Gfuse,
    Gin_contact,
    Gload_step,
    Gload_step_labels,
    Gmake_box,
    Gmake_compound,
    Gmake_cone,
    Gmake_cylinder,
    Gmake_half_space,
    Gmake_polygon_face,
    Gmake_shell,
    Gmake_sphere,
    Gmake_torus,
    Gmake_wire,
    Gsplit,
    kernel_version,
)
from geouned.geo import surface_geometry


@pytest.fixture
def unit_box():
    return Gmake_box(-5, -5, -5, 5, 5, 5)


def _bbox_tuple(bbox):
    return (bbox.XMin, bbox.YMin, bbox.ZMin, bbox.XMax, bbox.YMax, bbox.ZMax)


def _tolerances(split_tolerance=1e-6, scale_up_floor=None, scale=0.1):
    # Gsplit's own `tolerances` param is duck-typed (not literally
    # GEOUNED.utils.data_classes.Tolerances -- geo must not depend on
    # GEOUNED) -- a SimpleNamespace carrying exactly the fields Gsplit reads
    # keeps this test file's own established independence from GEOUNED.
    return SimpleNamespace(split_tolerance=split_tolerance, scale_up_floor=scale_up_floor, scale=scale)


# -- Primitive construction -------------------------------------------------


def test_make_box():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    assert box.Volume == pytest.approx(1000.0)
    assert type(box.BoundBox) is GBoundBox
    assert _bbox_tuple(box.BoundBox) == pytest.approx((0, 0, 0, 10, 10, 10))


def test_gsolid_export_step_convenience_method(unit_box, tmp_path):
    step_file = tmp_path / "solid.step"
    unit_box.export_step(str(step_file))
    loaded = Gload_step(str(step_file))
    assert len(loaded) == 1
    assert loaded[0].Volume == pytest.approx(unit_box.Volume)


def test_gface_export_step_convenience_method(unit_box, tmp_path):
    step_file = tmp_path / "face.step"
    face = unit_box.Faces[0]
    face.export_step(str(step_file))
    loaded = Gload_step(str(step_file))
    assert loaded == []  # a bare face has no Solids, but the file must still be valid/loadable


def test_optimal_bounding_box_matches_bounding_box_for_a_box():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    assert _bbox_tuple(box.optimal_bounding_box()) == pytest.approx(_bbox_tuple(box.BoundBox))


def test_solid_vertexes_are_plain_gvectors(unit_box):
    assert len(unit_box.Vertexes) == 8
    assert all(isinstance(v, GVector) for v in unit_box.Vertexes)


def test_make_cylinder():
    cylinder = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 2.0, 5.0)
    assert cylinder.Volume == pytest.approx(math.pi * 2.0**2 * 5.0)


def test_make_cone():
    apex = GVector(0, 0, 10)
    cone = Gmake_cone(apex, GVector(0, 0, -1), math.atan(2.0 / 10.0), 10.0)
    assert cone.Volume == pytest.approx((1 / 3) * math.pi * 2.0**2 * 10.0, rel=1e-4)


def test_make_sphere():
    sphere = Gmake_sphere(GVector(0, 0, 0), 3.0)
    assert sphere.Volume == pytest.approx((4 / 3) * math.pi * 3.0**3, rel=1e-4)


def test_make_torus():
    torus = Gmake_torus(GVector(0, 0, 0), GVector(0, 0, 1), 10.0, 2.0)
    assert torus.Volume == pytest.approx(2 * math.pi**2 * 10.0 * 2.0**2, rel=1e-4)


def test_make_half_space_cuts_box_in_half(unit_box):
    half_space = Gmake_half_space(GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1)))
    remaining = Gcut(unit_box, [half_space])
    assert len(remaining) == 1
    assert remaining[0].Volume == pytest.approx(500.0)


def test_make_wire_round_trips_outer_wire_edges(unit_box):
    face = unit_box.Faces[0]
    outer = face.outer_wire()
    assert len(outer.Edges) == 4

    rebuilt = Gmake_wire(outer.Edges)
    assert len(rebuilt.Edges) == 4


def test_make_polygon_face():
    points = [GVector(0, 0, 0), GVector(10, 0, 0), GVector(10, 10, 0), GVector(0, 10, 0)]
    face = Gmake_polygon_face(points)
    assert type(face.Surface) is GPlane
    assert face.Area == pytest.approx(100.0)


def test_make_shell_from_box_faces(unit_box):
    shell = Gmake_shell(unit_box.Faces)
    assert type(shell) is GShell
    assert len(shell.Faces) == 6
    assert shell.Orientation in ("Forward", "Reversed")


def test_make_shell_supports_in_contact_and_distance(unit_box):
    shell = Gmake_shell(unit_box.Faces[:1])
    other_box = Gmake_box(100, 100, 100, 110, 110, 110)
    assert Gin_contact(shell, unit_box, 1e-6) is True
    assert Gin_contact(shell, other_box, 1e-6) is False
    assert Gdistance(shell, other_box) > 0


# -- Boolean operations -------------------------------------------------------


def test_cut_full_containment_returns_no_solids(unit_box):
    containing_tool = Gmake_half_space(GPlane.from_values(GVector(0, 0, 5), GVector(0, 0, 1)))
    result = Gcut(unit_box, [containing_tool])
    assert result == []


def test_common():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(5, 0, 0, 15, 10, 10)
    result = Gcommon(box_a, [box_b])
    assert len(result) == 1
    assert result[0].Volume == pytest.approx(500.0)


def test_fuse():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(10, 0, 0, 20, 10, 10)
    fused = Gfuse([box_a, box_b])
    assert fused.Volume == pytest.approx(2000.0)


def test_make_compound():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(100, 100, 100, 110, 110, 110)
    compound = Gmake_compound([box_a, box_b])
    assert compound.Volume == pytest.approx(2000.0)


def test_reverse_flips_volume_sign(unit_box):
    reversed_box = unit_box.reverse()
    assert reversed_box.Volume == pytest.approx(-unit_box.Volume)
    # original must be untouched
    assert unit_box.Volume > 0


def test_refine_keeps_volume_of_coplanar_fuse():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(10, 0, 0, 20, 10, 10)
    fused = Gfuse([box_a, box_b])
    refined = fused.refine()
    assert refined.Volume == pytest.approx(fused.Volume)


def test_split_interior_plane_gives_two_solids(unit_box):
    interior_plane = Gmake_half_space(GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1)))
    result = Gsplit(unit_box, interior_plane, _tolerances())
    assert len(result.solids) == 2


def test_split_scale_up_floor_bootstraps_from_tiny_tolerance(unit_box):
    # mirrors GEOUNED's public Options.scaleUp / Options.splitTolerance:
    # starting tolerance of 0.0 with scale_up_floor set must not just
    # attempt the tiny tolerance as-is -- it climbs back up from the floor.
    interior_plane = Gmake_half_space(GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1)))
    result = Gsplit(unit_box, interior_plane, _tolerances(split_tolerance=0.0, scale_up_floor=0.0))
    assert len(result.solids) == 2


def test_split_tool_not_intersecting_solid_returns_solid_unchanged(unit_box):
    far_face = Gmake_polygon_face(
        [
            GVector(1000, -500, -500),
            GVector(1000, 500, -500),
            GVector(1000, 500, 500),
            GVector(1000, -500, 500),
        ]
    )
    result = Gsplit(unit_box, far_face, _tolerances())
    assert len(result.solids) == 1
    assert result.degenerate_case_handled is True
    assert result.solids[0].Volume == pytest.approx(unit_box.Volume)


def test_split_plane_coincident_with_existing_face_is_a_known_limitation(unit_box):
    """
    Known limitation documented in _freecad_impl.py's module docstring:
    when the split tool's boundary coincides with an existing face of the
    solid (here, the tool's plane sits exactly on the box's own z=5 face),
    BOPTools.SplitAPI.slice silently returns the solid unchanged instead of
    raising or flagging the degenerate case. This is the motivating
    tangency bug for the whole pyOCC migration -- the FreeCAD
    implementation does NOT fix it, only a future OCC one is expected to.
    This test documents today's actual (buggy) behavior so a regression is
    visible, not to assert it is correct.
    """
    coincident_tool = Gmake_half_space(GPlane.from_values(GVector(0, 0, 5), GVector(0, 0, 1)))
    result = Gsplit(unit_box, coincident_tool, _tolerances())
    assert len(result.solids) == 1
    assert result.degenerate_case_handled is False


# -- Topological traversal --------------------------------------------------


def test_get_faces_edges_vertices(unit_box):
    faces = unit_box.Faces
    assert len(faces) == 6
    edges = faces[0].Edges
    assert len(edges) == 4
    vertices = edges[0].Vertexes
    assert len(vertices) == 2
    assert isinstance(vertices[0], GVector)


def test_faces_are_eagerly_enriched(unit_box):
    face = unit_box.Faces[0]
    assert type(face.Surface) is GPlane
    assert len(face.Edges) == 4
    assert all(type(e.Curve) is GLine for e in face.Edges)
    assert face.index in range(6)
    assert face.Orientation in ("Forward", "Reversed")


def test_face_outer_wire_is_lazy_and_cached(unit_box):
    face = unit_box.Faces[0]
    assert face.outer_wire() is not None
    assert face.outer_wire() is face.outer_wire()  # cached, not recomputed
    assert face.wires() is face.wires()


def test_edges_are_eagerly_enriched(unit_box):
    face = unit_box.Faces[0]
    edge = face.Edges[0]
    assert type(edge.Curve) is GLine
    assert len(edge.Vertexes) == 2
    assert edge.Orientation in ("Forward", "Reversed")


def test_get_solid_orientation(unit_box):
    assert unit_box.Orientation in ("Forward", "Reversed")


def test_outer_wire_single_wire_face_matches_only_wire(unit_box):
    face = unit_box.Faces[0]
    assert len(face.outer_wire().Edges) == 4


def test_outer_wire_picks_outer_boundary_not_a_hole():
    """
    A face with a through-hole has two wires: the outer boundary and the
    hole's inner wire. GEOUNED's own heuristic (largest mean
    vertex-to-centroid distance) must pick the outer one -- this is the
    exact case where FreeCAD's native Face.OuterWire was found to pick
    the wrong wire, which is why _pick_outer_wire doesn't use it.
    """
    box = Part.makeBox(20, 20, 20)
    hole = Part.makeCylinder(3, 22, FreeCAD.Vector(10, 10, -1), FreeCAD.Vector(0, 0, 1))
    drilled = box.cut(hole)
    solid = GSolid(drilled)

    top_face = next(f for f in solid.Faces if type(f.Surface) is GPlane and f.Surface.Position.z == pytest.approx(20))
    assert len(top_face.__native__.Wires) == 2
    assert len(top_face.Edges) == 5  # 4 outer + 1 circular hole
    assert len(top_face.outer_wire().Edges) == 4


def test_faces_sharing_edge(unit_box):
    face = unit_box.Faces[0]
    edge = face.Edges[0]
    sharing = unit_box.faces_sharing_edge(edge)
    assert len(sharing) == 2


def test_edge_is_same(unit_box):
    face = unit_box.Faces[0]
    edges = face.Edges
    assert edges[0].is_same(edges[0]) is True
    assert edges[0].is_same(edges[1]) is False


# -- Surface and curve classification (happens eagerly at construction) ------


def test_face_surface_is_classified_as_plane(unit_box):
    assert type(unit_box.Faces[0].Surface) is GPlane


def test_face_surface_is_classified_as_cylinder():
    cylinder = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in cylinder.Faces if type(f.Surface) is GCylinder)
    assert side_face.Surface.Radius == pytest.approx(3.0)


def test_gsolid_does_not_crash_on_a_genuine_bspline_surface():
    """
    Regression: GSolid/GFace build eagerly, so a solid with a real (not
    just mislabeled-plane) BSplineSurface face -- e.g. a loft between two
    non-planar profiles, which STEP files with swept/freeform geometry
    can legitimately contain -- must not raise just from being wrapped.
    Gclassify_surface returns None for that face instead of raising;
    `loadfile.load_functions.spline()` relies on exactly this to detect
    such solids gracefully rather than crashing on load.
    """

    def wavy_wire(z):
        n = 12
        pts = [
            FreeCAD.Vector(
                (5 + (0.8 * math.sin(3 * 2 * math.pi * i / n) if z > 0 else 0)) * math.cos(2 * math.pi * i / n),
                (5 + (0.8 * math.sin(3 * 2 * math.pi * i / n) if z > 0 else 0)) * math.sin(2 * math.pi * i / n),
                z,
            )
            for i in range(n)
        ]
        pts.append(pts[0])
        return Part.makePolygon(pts)

    loft = Part.makeLoft([wavy_wire(0.0), wavy_wire(10.0)], True, True)
    bspline_face = next(f for f in loft.Faces if type(f.Surface) is Part.BSplineSurface)
    assert Gclassify_surface(bspline_face) is None

    solid = GSolid(loft)  # must not raise
    assert any(f.Surface is None for f in solid.Faces)


def test_classify_surface_populates_x_dir_for_plane_and_cylinder(unit_box):
    plane = unit_box.Faces[0].Surface
    assert plane.XDir is not None
    assert plane.XDir.length == pytest.approx(1.0)
    assert abs(plane.XDir.dot(plane.Axis)) < 1e-9

    cylinder_solid = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in cylinder_solid.Faces if type(f.Surface) is GCylinder)
    cylinder = side_face.Surface
    assert cylinder.XDir is not None
    assert cylinder.XDir.length == pytest.approx(1.0)
    assert abs(cylinder.XDir.dot(cylinder.Axis)) < 1e-9


def test_plane_value_and_tangent_at_match_native(unit_box):
    face = unit_box.Faces[0]
    plane = face.Surface
    u, v = 1.3, -0.7
    expected_point = face.value_at(u, v)
    expected_tangent_u, expected_tangent_v = face.tangent_at(u, v)

    point = surface_geometry.plane_value_at(plane, u, v)
    tangent_u, tangent_v = surface_geometry.plane_tangent_at(plane, u, v)

    assert point.is_equal(expected_point, 1e-6)
    assert tangent_u.is_equal(expected_tangent_u, 1e-6)
    assert tangent_v.is_equal(expected_tangent_v, 1e-6)


def test_cylinder_value_and_tangent_at_match_native_off_axis():
    # Deliberately not aligned to any global axis: a bug that only shows up
    # for an arbitrary XDir/YDir frame would pass undetected on a Z-aligned
    # cylinder.
    base = GVector(3, -1, 7)
    axis_dir = GVector(1, 1, 1).normalized()
    cylinder_solid = Gmake_cylinder(base, axis_dir, 4.0, 15.0)
    side_face = next(f for f in cylinder_solid.Faces if type(f.Surface) is GCylinder)
    cylinder = side_face.Surface

    u, v = 0.9, 3.3
    expected_point = side_face.value_at(u, v)
    expected_tangent_u, expected_tangent_v = side_face.tangent_at(u, v)

    point = surface_geometry.cylinder_value_at(cylinder, u, v)
    tangent_u, tangent_v = surface_geometry.cylinder_tangent_at(cylinder, u, v)

    assert point.is_equal(expected_point, 1e-6)
    assert tangent_u.is_equal(expected_tangent_u, 1e-6)
    assert tangent_v.is_equal(expected_tangent_v, 1e-6)


def test_edge_curve_is_classified_as_line(unit_box):
    assert type(unit_box.Faces[0].Edges[0].Curve) is GLine


def test_edge_curve_is_classified_as_circle():
    cylinder = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in cylinder.Faces if type(f.Surface) is GCylinder)
    circle_edge = next(e for e in side_face.Edges if type(e.Curve) is GCircle)
    assert circle_edge.Curve.Radius == pytest.approx(3.0)


def test_face_orientation_outward_is_true_for_all_box_faces(unit_box):
    for face in unit_box.Faces:
        assert face.orientation_outward(unit_box) is True


# -- Face and edge parametric queries -----------------------------------------


def test_parameter_range_and_face_value_at(unit_box):
    face = unit_box.Faces[0]
    u_min, u_max, v_min, v_max = face.ParameterRange
    mid_point = face.value_at((u_min + u_max) / 2, (v_min + v_max) / 2)
    normal = face.normal_at((u_min + u_max) / 2, (v_min + v_max) / 2)
    assert isinstance(mid_point, GVector)
    assert normal.length == pytest.approx(1.0)


def test_face_parameter_is_inverse_of_face_value_at(unit_box):
    face = unit_box.Faces[0]
    u_min, u_max, v_min, v_max = face.ParameterRange
    u, v = (u_min + u_max) / 2, (v_min + v_max) / 2
    point = face.value_at(u, v)
    round_tripped_u, round_tripped_v = face.parameter(point)
    assert round_tripped_u == pytest.approx(u)
    assert round_tripped_v == pytest.approx(v)


def test_face_tangent_at_returns_unit_orthogonal_vectors(unit_box):
    face = unit_box.Faces[0]
    u_min, u_max, v_min, v_max = face.ParameterRange
    u, v = (u_min + u_max) / 2, (v_min + v_max) / 2
    tangent_u, tangent_v = face.tangent_at(u, v)
    normal = face.normal_at(u, v)
    assert tangent_u.length == pytest.approx(1.0)
    assert tangent_v.length == pytest.approx(1.0)
    assert abs(tangent_u.dot(tangent_v)) < 1e-9
    assert tangent_u.cross(tangent_v).is_equal(normal, 1e-6) or tangent_u.cross(tangent_v).is_equal(-normal, 1e-6)


def test_edge_parameter_range_and_value_at(unit_box):
    edge = unit_box.Faces[0].Edges[0]
    u_min, u_max = edge.ParameterRange
    mid_point = edge.value_at((u_min + u_max) / 2)
    assert isinstance(mid_point, GVector)


def test_tessellate(unit_box):
    face = unit_box.Faces[0]
    points = face.tessellate(0.5)
    assert len(points) > 0
    assert all(isinstance(p, GVector) for p in points)


def test_edge_length_of_unit_box_edge(unit_box):
    assert unit_box.Faces[0].Edges[0].Length == pytest.approx(10.0)


def test_edge_derivative1_at_is_not_unit_normalized_on_a_circle():
    cylinder = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in cylinder.Faces if type(f.Surface) is GCylinder)
    circle_edge = next(e for e in side_face.Edges if type(e.Curve) is GCircle)
    u_min, u_max = circle_edge.ParameterRange
    derivative = circle_edge.derivative1_at((u_min + u_max) / 2)
    assert derivative.length == pytest.approx(3.0)


def test_edge_normal_at_on_a_circle():
    cylinder = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 3.0, 5.0)
    side_face = next(f for f in cylinder.Faces if type(f.Surface) is GCylinder)
    circle_edge = next(e for e in side_face.Edges if type(e.Curve) is GCircle)
    u_min, u_max = circle_edge.ParameterRange
    normal = circle_edge.normal_at((u_min + u_max) / 2)
    assert normal.length == pytest.approx(1.0)


# -- Spatial queries -----------------------------------------------------------


def test_is_inside(unit_box):
    assert unit_box.is_inside(GVector(0, 0, 0), 1e-7) is True
    assert unit_box.is_inside(GVector(100, 100, 100), 1e-7) is False


def test_in_contact_touching_solids():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(10, 0, 0, 20, 10, 10)
    assert Gin_contact(box_a, box_b, 1e-6) is True


def test_in_contact_far_apart_solids():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(100, 100, 100, 110, 110, 110)
    assert Gin_contact(box_a, box_b, 1e-6) is False


def test_distance_touching_solids_is_zero():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(10, 0, 0, 20, 10, 10)
    assert Gdistance(box_a, box_b) == pytest.approx(0.0)


def test_distance_far_apart_solids():
    box_a = Gmake_box(0, 0, 0, 10, 10, 10)
    box_b = Gmake_box(20, 0, 0, 30, 10, 10)
    assert Gdistance(box_a, box_b) == pytest.approx(10.0)


def test_find_interior_point_trivial_box(unit_box):
    point = unit_box.find_interior_point()
    assert point is not None
    assert unit_box.is_inside(point, 0.0) is True


def test_find_interior_point_torus_center_of_mass_outside():
    # a torus's center of mass sits in the hole, not inside the solid
    torus = Gmake_torus(GVector(0, 0, 0), GVector(0, 0, 1), 20.0, 5.0)
    point = torus.find_interior_point()
    assert point is not None
    assert torus.is_inside(point, 0.0) is True


def test_find_interior_point_thin_torus():
    # regression: a thin ring is exactly the shape a RoundCorner/TCone
    # fillet produces. Alternative strategies (vertex-to-vertex segment
    # sampling, bounding-box octree subdivision) either take orders of
    # magnitude longer or fail outright here within their fixed
    # subdivision depth -- face-normal probing does not have that
    # weakness (see benchmark in the migration notes).
    torus = Gmake_torus(GVector(0, 0, 0), GVector(0, 0, 1), 200.0, 0.2)
    point = torus.find_interior_point()
    assert point is not None
    assert torus.is_inside(point, 0.0) is True


def test_is_part_of_domain_at_face_center(unit_box):
    face = unit_box.Faces[0]
    u_min, u_max, v_min, v_max = face.ParameterRange
    u = 0.5 * (u_min + u_max)
    v = 0.5 * (v_min + v_max)
    assert face.is_part_of_domain(u, v) is True


# -- Validation / diagnostics --------------------------------------------------


def test_is_valid(unit_box):
    assert unit_box.is_valid() is True


def test_fix_preserves_volume_of_a_valid_solid(unit_box):
    fixed = unit_box.fix(1e-6)
    assert fixed.Volume == pytest.approx(unit_box.Volume)


# -- Transformations ------------------------------------------------------------


def test_translate():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    translated = box.translate(GVector(5, 0, 0))
    assert _bbox_tuple(translated.BoundBox) == pytest.approx((5, 0, 0, 15, 10, 10))


def test_rotate_quarter_turn_around_z(unit_box):
    rotated = unit_box.rotate(GVector(0, 0, 0), GVector(0, 0, 1), math.pi / 2)
    assert _bbox_tuple(rotated.BoundBox) == pytest.approx(_bbox_tuple(unit_box.BoundBox))


# -- I/O ------------------------------------------------------------------------


def test_kernel_version_returns_a_dotted_string():
    version = kernel_version()
    assert isinstance(version, str)
    assert version.count(".") == 2


def test_step_export_load_roundtrip(unit_box, tmp_path):
    step_file = tmp_path / "roundtrip.step"
    Gexport_step([unit_box], str(step_file))
    loaded = Gload_step(str(step_file))
    assert len(loaded) == 1
    assert loaded[0].Volume == pytest.approx(unit_box.Volume)


def test_load_step_labels_matches_load_step_solid_count(unit_box, tmp_path):
    step_file = tmp_path / "labels.step"
    Gexport_step([unit_box], str(step_file))
    solids = Gload_step(str(step_file))
    nodes = Gload_step_labels(str(step_file))
    assert sum(n.n_solids for n in nodes) == len(solids)


def test_load_step_labels_against_real_hierarchical_file():
    step_file = Path(__file__).resolve().parents[2] / "testing" / "inputSTEP" / "Misc" / "rails.stp"
    solids = Gload_step(str(step_file))
    nodes = Gload_step_labels(str(step_file))

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
