"""Prototype validation (branch `sewing-composite-surface`) for
`Gmake_sewn_multi_round_corner` -- the MultiRoundCorner case (N chained
RoundCorners sharing a junction plane between each pair of neighbors).
Generalizes `Gmake_sewn_round_corner_cylinder` (see
test_sewn_composite_round_corner_ocp.py) from 1 corner to N, reusing the
exact same per-face recipe. See
geo/ocp/sewn_composite.py::Gmake_sewn_multi_round_corner's own docstring
for the design.
"""

import math
import os
from types import SimpleNamespace

import pytest

os.environ.setdefault("GEOUNED_CAD_ENGINE", "ocp")

pytest.importorskip("OCP.BRepPrimAPI", reason="OCP not available on this machine")

if os.environ.get("GEOUNED_CAD_ENGINE", "occ").strip().lower() != "ocp":
    pytest.skip(
        "GEOUNED_CAD_ENGINE is not 'ocp' -- geouned.geo already loaded a different backend in this process",
        allow_module_level=True,
    )

from OCP.BRep import BRep_Tool
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from geouned.geo import GCylinder, GPlane, GSolid, GVector, Gmake_box, Gsplit
from geouned.geo.ocp.sewn_composite import Gmake_sewn_multi_round_corner

_TOLERANCES = SimpleNamespace(
    split_tolerance=1e-6,
    scale_up_floor=None,
    min_solid_volume=1e-6,
    min_face_width=0.1,
    fix_tolerance=1e-6,
    volume_tolerance=1e-6,
)

_FILLET_RADIUS = 2.0


def _vertical_edge_at(box, x: float, y: float):
    eexp = TopExp_Explorer(box.__native__, TopAbs_EDGE)
    while eexp.More():
        native_edge = TopoDS.Edge(eexp.Current())
        vexp = TopExp_Explorer(native_edge, TopAbs_VERTEX)
        pts = []
        while vexp.More():
            pts.append(BRep_Tool.Pnt_s(TopoDS.Vertex(vexp.Current())))
            vexp.Next()
        if len(pts) == 2 and all(abs(p.X() - x) < 1e-9 and abs(p.Y() - y) < 1e-9 for p in pts):
            return native_edge
        eexp.Next()
    return None


def _build_double_filleted_box():
    """A real, robustly-built (via BRepFilletAPI) box with BOTH vertical
    edges of its x=10 face (at y=0 and y=10) replaced by tangent
    cylindrical fillets of radius `_FILLET_RADIUS` -- a genuine open,
    2-corner MultiRoundCorner chain: plane_y0 -- corner1(cyl) --
    plane_x10 (SHARED by both corners) -- corner2(cyl) -- plane_y10."""
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    edge_a = _vertical_edge_at(box, 10.0, 0.0)
    edge_b = _vertical_edge_at(box, 10.0, 10.0)
    assert edge_a is not None and edge_b is not None

    fillet_maker = BRepFilletAPI_MakeFillet(box.__native__)
    fillet_maker.Add(_FILLET_RADIUS, edge_a)
    fillet_maker.Add(_FILLET_RADIUS, edge_b)
    return GSolid(fillet_maker.Shape())


def _find_plane_face(solid, outward_axis: GVector, position_coord: str, position_value: float):
    for f in solid.Faces:
        if not isinstance(f.Surface, GPlane) or abs(f.Surface.Axis.dot(outward_axis)) <= 0.9:
            continue
        if abs(getattr(f.Surface.Position, position_coord) - position_value) < 1e-6:
            return f
    return None


def _find_cylinder_faces(solid, radius):
    return [f for f in solid.Faces if isinstance(f.Surface, GCylinder) and abs(f.Surface.Radius - radius) < 1e-6]


def _shared_edge(face1, face2):
    for e1 in face1.Edges:
        for e2 in face2.Edges:
            if e1.is_same(e2):
                return e1
    return None


def _build_tool(solid):
    """Build the 2-corner sewn tool from the real double-filleted solid,
    grouping the shared x=10 plane the same way
    `build_shape_functions.py::makeMultiRoundCorner` does (by geometric
    identity, not assumed list order)."""
    plane_y0 = _find_plane_face(solid, GVector(0, -1, 0), "y", 0.0)
    plane_y10 = _find_plane_face(solid, GVector(0, 1, 0), "y", 10.0)
    plane_x10 = _find_plane_face(solid, GVector(1, 0, 0), "x", 10.0)
    cyl_a, cyl_b = _find_cylinder_faces(solid, _FILLET_RADIUS)
    assert plane_y0 is not None and plane_y10 is not None and plane_x10 is not None

    # Identify which cylinder borders plane_y0 (corner1) vs plane_y10 (corner2).
    edge_a_y0 = _shared_edge(plane_y0, cyl_a)
    if edge_a_y0 is not None:
        corner1_cyl, corner2_cyl = cyl_a, cyl_b
    else:
        corner1_cyl, corner2_cyl = cyl_b, cyl_a

    edge1_open = _shared_edge(plane_y0, corner1_cyl)
    edge1_shared = _shared_edge(plane_x10, corner1_cyl)
    edge2_shared = _shared_edge(plane_x10, corner2_cyl)
    edge2_open = _shared_edge(plane_y10, corner2_cyl)
    assert all(e is not None for e in (edge1_open, edge1_shared, edge2_shared, edge2_open))

    cylinder1 = GCylinder.from_values(corner1_cyl.Surface.Center, corner1_cyl.Surface.Axis, corner1_cyl.Surface.Radius)
    cylinder2 = GCylinder.from_values(corner2_cyl.Surface.Center, corner2_cyl.Surface.Axis, corner2_cyl.Surface.Radius)
    plane_x10_geo = GPlane.from_values(plane_x10.Surface.Position, plane_x10.Surface.Axis)

    cylinders = [
        (cylinder1, edge1_open, edge1_shared, corner1_cyl.CenterOfMass),
        (cylinder2, edge2_shared, edge2_open, corner2_cyl.CenterOfMass),
    ]
    shared_planes = [(plane_x10_geo, edge1_shared, edge2_shared)]
    open_planes = [
        (GPlane.from_values(plane_y0.Surface.Position, plane_y0.Surface.Axis), edge1_open, plane_y0.CenterOfMass),
        (GPlane.from_values(plane_y10.Surface.Position, plane_y10.Surface.Axis), edge2_open, plane_y10.CenterOfMass),
    ]
    return Gmake_sewn_multi_round_corner(cylinders, shared_planes, open_planes, 15.0)


def test_gmake_sewn_multi_round_corner_builds_a_watertight_shell():
    solid = _build_double_filleted_box()
    tool = _build_tool(solid)

    assert len(tool.Faces) == 5
    plane_faces = [f for f in tool.Faces if isinstance(f.Surface, GPlane)]
    cyl_faces = [f for f in tool.Faces if isinstance(f.Surface, GCylinder)]
    assert len(plane_faces) == 3 and len(cyl_faces) == 2
    for cf in cyl_faces:
        shared = [e for e in cf.Edges for pf in plane_faces for e2 in pf.Edges if e.is_same(e2)]
        assert len(shared) == 2


def test_sewn_multi_round_corner_separates_both_corner_wedges():
    """Use the sewn 5-face shell as a Gsplit tool against the ORIGINAL,
    sharp-cornered box. Ground truth: the tool exactly separates the
    sharp box into the double-filleted body and the 2 independent
    corner wedges each fillet replaces (far enough apart along y not to
    interact)."""
    solid = _build_double_filleted_box()
    tool = _build_tool(solid)

    sharp_box = Gmake_box(0, 0, 0, 10, 10, 10)
    result = Gsplit(sharp_box, tool, _TOLERANCES)

    wedge_volume = (_FILLET_RADIUS**2 - math.pi * _FILLET_RADIUS**2 / 4.0) * 10.0
    expected = sorted([1000.0 - 2.0 * wedge_volume, wedge_volume, wedge_volume])
    volumes = sorted(round(s.Volume, 3) for s in result.solids)
    assert volumes == pytest.approx(expected, abs=1e-2)
