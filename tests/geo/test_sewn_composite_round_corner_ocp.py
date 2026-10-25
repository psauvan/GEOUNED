"""Prototype validation (branch `sewing-composite-surface`) for
`Gmake_sewn_round_corner_cylinder` -- the RoundCorner case (2 planes + 1
cylinder), the first composite type that genuinely still pays for
`build_complex_shape`'s own `BuildDepth`/`Gfuse_solids` boolean cascade
today (unlike MultiPlane's own `makeMultiPlanes`, which never used it --
see test_sewn_composite_ocp.py). See geo/ocp/sewn_composite.py's own
module docstring for the design.
"""

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

from OCP.BRep import BRep_Tool
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from geouned.geo import GCylinder, GPlane, GSolid, GVector, Gmake_box, Gsplit
from geouned.geo.ocp.sewn_composite import Gmake_sewn_round_corner_cylinder

_TOLERANCES = SimpleNamespace(
    split_tolerance=1e-6,
    scale_up_floor=None,
    min_solid_volume=1e-6,
    min_face_width=0.1,
    fix_tolerance=1e-6,
    volume_tolerance=1e-6,
)

_FILLET_RADIUS = 2.0


def _build_filleted_box():
    """A real, robustly-built (via BRepFilletAPI, the standard OCCT way --
    not a hand-rolled boolean notch) box with its vertical edge at
    x=10,y=10 replaced by a tangent cylindrical fillet of radius
    `_FILLET_RADIUS` -- a genuine RoundCorner fixture: 2 real tangent
    generatrix edges, no extra bounding plane needed (the fillet is
    tangent to both original faces)."""
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    target_edge = None
    eexp = TopExp_Explorer(box.__native__, TopAbs_EDGE)
    while eexp.More():
        native_edge = TopoDS.Edge(eexp.Current())
        vexp = TopExp_Explorer(native_edge, TopAbs_VERTEX)
        pts = []
        while vexp.More():
            pts.append(BRep_Tool.Pnt_s(TopoDS.Vertex(vexp.Current())))
            vexp.Next()
        if len(pts) == 2 and all(abs(p.X() - 10.0) < 1e-9 and abs(p.Y() - 10.0) < 1e-9 for p in pts):
            target_edge = native_edge
            break
        eexp.Next()
    assert target_edge is not None

    fillet_maker = BRepFilletAPI_MakeFillet(box.__native__)
    fillet_maker.Add(_FILLET_RADIUS, target_edge)
    return GSolid(fillet_maker.Shape())


def _find_plane_face(solid, outward_axis: GVector, position_coord: str, position_value: float):
    for f in solid.Faces:
        if not isinstance(f.Surface, GPlane) or f.Surface.Axis.dot(outward_axis) <= 0.9:
            continue
        if abs(getattr(f.Surface.Position, position_coord) - position_value) < 1e-6:
            return f
    return None


def _find_cylinder_face(solid, radius):
    for f in solid.Faces:
        if isinstance(f.Surface, GCylinder) and abs(f.Surface.Radius - radius) < 1e-6:
            return f
    return None


def _shared_edge(face1, face2):
    for e1 in face1.Edges:
        for e2 in face2.Edges:
            if e1.is_same(e2):
                return e1
    return None


def test_gmake_sewn_round_corner_builds_two_shared_seams():
    """The 3-face shell must share exactly one real edge between the
    cylinder face and each plane -- reusing the real tangent edges found
    on the filleted solid itself (design point 3), not a recomputed
    tangency line."""
    filleted = _build_filleted_box()
    face_x = _find_plane_face(filleted, GVector(1, 0, 0), "x", 10.0)
    face_y = _find_plane_face(filleted, GVector(0, 1, 0), "y", 10.0)
    cyl_face = _find_cylinder_face(filleted, _FILLET_RADIUS)
    assert face_x is not None and face_y is not None and cyl_face is not None

    edge1 = _shared_edge(face_x, cyl_face)
    edge2 = _shared_edge(face_y, cyl_face)
    assert edge1 is not None and edge2 is not None

    tool = Gmake_sewn_round_corner_cylinder(
        face_x.Surface,
        face_x.CenterOfMass,
        edge1,
        cyl_face.Surface,
        cyl_face.CenterOfMass,
        edge2,
        face_y.Surface,
        face_y.CenterOfMass,
        15.0,
    )

    assert len(tool.Faces) == 3
    plane_faces = [f for f in tool.Faces if isinstance(f.Surface, GPlane)]
    cyl_faces = [f for f in tool.Faces if isinstance(f.Surface, GCylinder)]
    assert len(plane_faces) == 2 and len(cyl_faces) == 1
    for pf in plane_faces:
        shared = [e for e in pf.Edges if any(e.is_same(e2) for e2 in cyl_faces[0].Edges)]
        assert len(shared) == 1


def test_sewn_round_corner_separates_the_corner_wedge():
    """Use the sewn 3-face shell as a Gsplit tool against the ORIGINAL,
    sharp-cornered box -- with no BOPAlgo_Splitter call made while
    building the tool itself. Ground truth: the tool exactly separates
    the sharp box into the filleted body (1000 - (4*R^2 - pi*R^2)) and
    the small corner wedge the fillet replaces (4*R^2 - pi*R^2)."""
    filleted = _build_filleted_box()
    face_x = _find_plane_face(filleted, GVector(1, 0, 0), "x", 10.0)
    face_y = _find_plane_face(filleted, GVector(0, 1, 0), "y", 10.0)
    cyl_face = _find_cylinder_face(filleted, _FILLET_RADIUS)
    edge1 = _shared_edge(face_x, cyl_face)
    edge2 = _shared_edge(face_y, cyl_face)

    tool = Gmake_sewn_round_corner_cylinder(
        face_x.Surface,
        face_x.CenterOfMass,
        edge1,
        cyl_face.Surface,
        cyl_face.CenterOfMass,
        edge2,
        face_y.Surface,
        face_y.CenterOfMass,
        15.0,
    )

    sharp_box = Gmake_box(0, 0, 0, 10, 10, 10)
    result = Gsplit(sharp_box, tool, _TOLERANCES)

    wedge_volume = (_FILLET_RADIUS**2 - math.pi * _FILLET_RADIUS**2 / 4.0) * 10.0
    expected = sorted([1000.0 - wedge_volume, wedge_volume])
    volumes = sorted(round(s.Volume, 3) for s in result.solids)
    assert volumes == pytest.approx(expected, abs=1e-2)
