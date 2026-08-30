import math
import os
from types import SimpleNamespace

import pytest

os.environ.setdefault("GEOUNED_CAD_ENGINE", "ocp")

pytest.importorskip("OCP.BRepPrimAPI", reason="OCP not available on this machine")

if os.environ.get("GEOUNED_CAD_ENGINE", "freecad").strip().lower() != "ocp":
    pytest.skip(
        "GEOUNED_CAD_ENGINE is not 'ocp' -- geouned.geo already loaded a different backend in this process",
        allow_module_level=True,
    )

from geouned.geo import (
    GCircle,
    GCylinder,
    GLine,
    GPlane,
    GVector,
    Gclassify_surface,
    Gcommon,
    Gcut,
    Gdistance,
    Gexport_step,
    Gfuse,
    Gin_contact,
    Gload_step,
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
)

# ---------------------------------------------------------------------------
# Primitives
# ---------------------------------------------------------------------------


def test_box_volume_and_topology():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    assert box.Volume == pytest.approx(1000.0, abs=1e-6)
    assert len(box.Faces) == 6
    assert box.BoundBox.XMax == pytest.approx(10.0, abs=1e-6)
    assert box.BoundBox.XMin == pytest.approx(0.0, abs=1e-6)
    assert box.is_valid()


def test_box_is_inside():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    assert box.is_inside(GVector(5, 5, 5))
    assert not box.is_inside(GVector(-5, 5, 5))


def test_cylinder_volume():
    cyl = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 5, 20)
    assert cyl.Volume == pytest.approx(math.pi * 25 * 20, rel=1e-6)


def test_cone_volume_positive():
    cone = Gmake_cone(GVector(0, 0, 0), GVector(0, 0, 1), math.atan(0.5), 20)
    assert cone.Volume > 0


def test_sphere_volume():
    sph = Gmake_sphere(GVector(0, 0, 0), 5)
    assert sph.Volume == pytest.approx(4 / 3 * math.pi * 125, rel=1e-4)


def test_torus_volume():
    tor = Gmake_torus(GVector(0, 0, 0), GVector(0, 0, 1), 20, 5)
    assert tor.Volume == pytest.approx(2 * math.pi**2 * 20 * 25, rel=1e-3)


# ---------------------------------------------------------------------------
# Surface classification
# ---------------------------------------------------------------------------


def test_box_faces_all_planes():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    assert all(type(f.Surface).__name__ == "GPlane" for f in box.Faces)
    assert box.Faces[0].Surface.Axis.length == pytest.approx(1.0, abs=1e-9)


def test_cylinder_face_classification():
    cyl = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 5, 20)
    cyl_face = next(f for f in cyl.Faces if type(f.Surface).__name__ == "GCylinder")
    assert cyl_face.Surface.Radius == pytest.approx(5.0, abs=1e-6)
    assert abs(cyl_face.Surface.Axis.z) == pytest.approx(1.0, abs=1e-6)


def test_cone_face_classification():
    half_angle = math.atan(0.5)
    cone = Gmake_cone(GVector(0, 0, 0), GVector(0, 0, 1), half_angle, 20)
    cone_face = next(f for f in cone.Faces if type(f.Surface).__name__ == "GCone")
    assert abs(cone_face.Surface.SemiAngle) == pytest.approx(half_angle, abs=1e-6)


def test_sphere_face_classification():
    sph = Gmake_sphere(GVector(0, 0, 0), 5)
    face = sph.Faces[0]
    assert type(face.Surface).__name__ == "GSphere"
    assert face.Surface.Radius == pytest.approx(5.0, abs=1e-6)


def test_torus_face_classification():
    tor = Gmake_torus(GVector(0, 0, 0), GVector(0, 0, 1), 20, 5)
    face = tor.Faces[0]
    assert type(face.Surface).__name__ == "GTorus"
    assert face.Surface.MajorRadius == pytest.approx(20, abs=1e-6)
    assert face.Surface.MinorRadius == pytest.approx(5, abs=1e-6)


# ---------------------------------------------------------------------------
# GPlane
# ---------------------------------------------------------------------------


def test_gplane_is_inside_and_value_at():
    p = GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1), GVector(1, 0, 0))
    assert p.is_inside(GVector(0, 0, 5))
    assert not p.is_inside(GVector(0, 0, -5))
    v = p.value_at(2, 3)
    assert v.x == pytest.approx(2, abs=1e-9)
    assert v.y == pytest.approx(3, abs=1e-9)
    assert v.z == pytest.approx(0, abs=1e-9)


def test_gplane_parameter_on_real_face():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    face = box.Faces[0]
    umin, umax, vmin, vmax = face.ParameterRange
    mid = face.value_at((umin + umax) / 2, (vmin + vmax) / 2)
    u, v = face.Surface.parameter(mid)
    assert isinstance(u, float) and isinstance(v, float)


def test_gplane_intersect_plane():
    p1 = GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1))
    p2 = GPlane.from_values(GVector(0, 0, 0), GVector(1, 0, 0))
    line = p1.intersect_plane(p2)
    assert line is not None
    assert abs(line.Direction.y) == pytest.approx(1.0, abs=1e-6)


def test_gplane_intersect_plane_parallel_returns_none():
    p1 = GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1))
    p3 = GPlane.from_values(GVector(0, 0, 5), GVector(0, 0, 1))
    assert p1.intersect_plane(p3) is None


# ---------------------------------------------------------------------------
# GLine
# ---------------------------------------------------------------------------


def test_gline_intersect_line():
    l1 = GLine.from_values(GVector(0, 0, 0), GVector(1, 0, 0))
    l2 = GLine.from_values(GVector(5, -5, 0), GVector(0, 1, 0))
    pt = l1.intersect_line(l2)
    assert pt is not None
    assert pt.x == pytest.approx(5, abs=1e-6)
    assert pt.y == pytest.approx(0, abs=1e-6)


def test_gline_intersect_line_parallel_returns_none():
    l1 = GLine.from_values(GVector(0, 0, 0), GVector(1, 0, 0))
    l3 = GLine.from_values(GVector(0, 5, 0), GVector(1, 0, 0))
    assert l1.intersect_line(l3) is None


def test_gline_intersect_line_skew_returns_none():
    l1 = GLine.from_values(GVector(0, 0, 0), GVector(1, 0, 0))
    l4 = GLine.from_values(GVector(0, 0, 5), GVector(0, 1, 1))
    assert l1.intersect_line(l4) is None


# ---------------------------------------------------------------------------
# Curve classification
# ---------------------------------------------------------------------------


def test_circle_edge_classification():
    cyl = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 5, 20)
    circle_edges = [e for f in cyl.Faces for e in f.Edges if type(e.Curve).__name__ == "GCircle"]
    assert circle_edges
    c = circle_edges[0].Curve
    assert c.Radius == pytest.approx(5.0, abs=1e-6)


def test_edge_matrix_of_inertia_populated():
    cyl = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 5, 20)
    edge = cyl.Faces[0].Edges[0]
    mat = edge.MatrixOfInertia
    assert any(getattr(mat, f"A{i}{j}") != 0.0 for i in (1, 2, 3) for j in (1, 2, 3))


# ---------------------------------------------------------------------------
# GFace / GWire / GEdge topology
# ---------------------------------------------------------------------------


def test_face_edges_and_wire():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    face = box.Faces[0]
    assert len(face.Edges) == 4
    assert type(face.Edges[0].Curve).__name__ == "GLine"

    wires = face.wires()
    assert len(wires) == 1
    outer = face.outer_wire()
    assert len(outer.Edges) == 4


def test_edge_is_inside():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    edge = box.Faces[0].Edges[0]
    midpoint = edge.value_at(sum(edge.ParameterRange) / 2)
    assert edge.is_inside(midpoint, 1e-6)


# ---------------------------------------------------------------------------
# Half space
# ---------------------------------------------------------------------------


def test_half_space_is_inside():
    hs = Gmake_half_space(GPlane.from_values(GVector(0, 0, 0), GVector(0, 0, 1)))
    assert hs.is_inside(GVector(0, 0, 100))
    assert not hs.is_inside(GVector(0, 0, -100))


# ---------------------------------------------------------------------------
# Boolean operations
# ---------------------------------------------------------------------------


def test_gcut():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    box2 = Gmake_box(5, 5, 5, 15, 15, 15)
    cut = Gcut(box, [box2])
    assert len(cut) == 1
    assert cut[0].Volume == pytest.approx(875.0, abs=1e-3)


def test_gcommon():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    box2 = Gmake_box(5, 5, 5, 15, 15, 15)
    common = Gcommon(box, [box2])
    assert common[0].Volume == pytest.approx(125.0, abs=1e-3)


def test_gfuse():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    box2 = Gmake_box(5, 5, 5, 15, 15, 15)
    fused = Gfuse([box, box2])
    assert fused.Volume == pytest.approx(1875.0, abs=1e-3)


def test_gsplit_box_by_half_space():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    tool = Gmake_half_space(GPlane.from_values(GVector(5, 0, 0), GVector(-1, 0, 0)))
    # Gsplit's own `tolerances` param is duck-typed (not literally
    # GEOUNED.utils.data_classes.Tolerances -- geo must not depend on
    # GEOUNED) -- a SimpleNamespace carrying exactly the fields Gsplit/
    # _raw_bop_split/_repair_non_manifold_solid read keeps this test
    # file's own established independence from the GEOUNED subpackage.
    tolerances = SimpleNamespace(
        split_tolerance=1e-6,
        scale_up_floor=None,
        min_solid_volume=1e-6,
        min_face_width=0.1,
        fix_tolerance=1e-6,
        volume_tolerance=1e-6,
    )
    result = Gsplit(box, tool, tolerances)
    assert len(result.solids) == 2
    total = sum(s.Volume for s in result.solids)
    assert total == pytest.approx(box.Volume, abs=1e-3)


# ---------------------------------------------------------------------------
# Spatial queries
# ---------------------------------------------------------------------------


def test_gdistance_and_gin_contact():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    far = Gmake_box(20, 0, 0, 30, 10, 10)
    touching = Gmake_box(10, 0, 0, 20, 10, 10)
    assert Gdistance(box, far) == pytest.approx(10.0, abs=1e-6)
    assert not Gin_contact(box, far, 1e-6)
    assert Gin_contact(box, touching, 1e-6)


# ---------------------------------------------------------------------------
# find_interior_point / fix / refine / reverse / translate / rotate
# ---------------------------------------------------------------------------


def test_find_interior_point():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    interior = box.find_interior_point()
    assert interior is not None
    assert box.is_inside(interior)


def test_refine_preserves_volume():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    refined = box.refine()
    assert refined.Volume == pytest.approx(box.Volume, abs=1e-3)


def test_reverse_changes_orientation():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    reversed_box = box.reverse()
    assert reversed_box.Orientation != box.Orientation


def test_translate():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    translated = box.translate(GVector(100, 0, 0))
    assert translated.BoundBox.XMin == pytest.approx(100, abs=1e-6)


def test_rotate():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    rotated = box.rotate(GVector(0, 0, 0), GVector(0, 0, 1), math.pi / 2)
    assert rotated.BoundBox.XMin == pytest.approx(-10, abs=1e-3)


# ---------------------------------------------------------------------------
# Construction helpers
# ---------------------------------------------------------------------------


def test_gmake_polygon_face_area():
    poly_face = Gmake_polygon_face([GVector(0, 0, 0), GVector(10, 0, 0), GVector(10, 10, 0), GVector(0, 10, 0)])
    assert poly_face.Area == pytest.approx(100.0, abs=1e-3)


def test_gmake_wire():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    edge = box.Faces[0].Edges[0]
    wire = Gmake_wire([edge])
    assert len(wire.Edges) == 1


def test_gmake_shell():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    shell = Gmake_shell(box.Faces)
    assert len(shell.Faces) == 6


def test_gmake_compound():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    box2 = Gmake_box(20, 0, 0, 30, 10, 10)
    compound = Gmake_compound([box, box2])
    assert len(compound.Solids) == 2


# ---------------------------------------------------------------------------
# STEP round trip
# ---------------------------------------------------------------------------


def test_step_roundtrip(tmp_path):
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    step_path = str(tmp_path / "box.stp")
    box.export_step(step_path)
    reloaded = Gload_step(step_path)
    assert len(reloaded) == 1
    assert reloaded[0].Volume == pytest.approx(box.Volume, abs=1e-3)


def test_gexport_step_multi_shape(tmp_path):
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    box2 = Gmake_box(20, 0, 0, 30, 10, 10)
    step_path = str(tmp_path / "two_boxes.stp")
    Gexport_step([box, box2], step_path)
    reloaded = Gload_step(step_path)
    assert len(reloaded) == 2
