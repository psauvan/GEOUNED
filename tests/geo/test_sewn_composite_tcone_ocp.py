"""Prototype validation (branch `sewing-composite-surface`) for
`Gmake_sewn_tcone` -- the TCone case (1 cone, closed 360 degrees, + 2
real planar closures), genuinely different from RoundCorner/MultiPlane's
own "oversized free sides" design: a TCone is already a topologically
closed shell, so no oversizing of any kind is needed. The cone's own
face is built via a direct parametric trim (confirmed empirically more
robust than reusing the real closing edges as a wire for this one,
periodic-surface case -- see geo/ocp/sewn_composite.py's own module
docstring), so its boundary is geometrically, not topologically,
identical to the planes' own real closing curves -- `Gmake_shell`'s own
tolerance-based sewing stitches them together, rather than sharing the
exact same edge object the way every other composite in this module
does.
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

from OCP.BRepCheck import BRepCheck_Analyzer

from geouned.geo import GCone, GPlane, GVector, Gmake_box, Gmake_cone_frustum, Gsplit
from geouned.geo.ocp.sewn_composite import Gmake_sewn_tcone

_TOLERANCES = SimpleNamespace(
    split_tolerance=1e-6,
    scale_up_floor=None,
    min_solid_volume=1e-6,
    min_face_width=0.1,
    fix_tolerance=1e-6,
    volume_tolerance=1e-6,
    sliver_edge_rel_tol=1e-4,
)

_RADIUS_BOTTOM = 5.0
_RADIUS_TOP = 3.0
_HEIGHT = 10.0
_APEX_POINT = GVector(0, 0, 0)
_AXIS = GVector(0, 0, 1)


def _build_frustum_solid():
    """A real, robustly-built (BRepPrimAPI_MakeCone) truncated cone --
    exactly 3 real faces (1 conical lateral face, 2 flat circular caps),
    each cap sharing a real, closed circular edge with the cone -- the
    simplest genuine TCone fixture."""
    return Gmake_cone_frustum(_APEX_POINT, _AXIS, _RADIUS_BOTTOM, _RADIUS_TOP, _HEIGHT)


def _find_cap(solid, z_value):
    for f in solid.Faces:
        if isinstance(f.Surface, GPlane) and abs(f.Surface.Position.z - z_value) < 1e-6:
            return f
    return None


def _find_cone_face(solid):
    for f in solid.Faces:
        if isinstance(f.Surface, GCone):
            return f
    return None


def _shared_edges(face1, face2):
    return [e1 for e1 in face1.Edges for e2 in face2.Edges if e1.is_same(e2)]


def test_gmake_sewn_tcone_builds_a_watertight_shell():
    """The 3-face shell's cone face and the 2 real closing planes must
    sew into a genuinely closed (no free edges) shell, even though the
    cone's own boundary is only geometrically -- not topologically --
    identical to each plane's real closing curve (`Gmake_shell`'s own
    tolerance-based sewing, not a shared edge object)."""
    frustum = _build_frustum_solid()
    cone_face = _find_cone_face(frustum)
    cap_bottom = _find_cap(frustum, 0.0)
    cap_top = _find_cap(frustum, _HEIGHT)
    assert cone_face is not None and cap_bottom is not None and cap_top is not None

    edges_bottom = _shared_edges(cone_face, cap_bottom)
    edges_top = _shared_edges(cone_face, cap_top)
    assert len(edges_bottom) >= 1 and len(edges_top) >= 1

    cone = GCone.from_values(cone_face.Surface.Apex, cone_face.Surface.Axis, cone_face.Surface.SemiAngle)

    tool = Gmake_sewn_tcone(cone, edges_bottom, cap_bottom.Surface, edges_top, cap_top.Surface)

    assert len(tool.Faces) == 3
    plane_faces = [f for f in tool.Faces if isinstance(f.Surface, GPlane)]
    cone_faces = [f for f in tool.Faces if isinstance(f.Surface, GCone)]
    assert len(plane_faces) == 2 and len(cone_faces) == 1
    assert BRepCheck_Analyzer(cone_faces[0].__native__).IsValid()

    # watertight: every edge of the shell must be shared by exactly 2 faces
    native_shell = tool.__native__
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp, TopExp_Explorer
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape

    edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(native_shell, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
    free_edges = sum(1 for i in range(1, edge_face_map.Extent() + 1) if edge_face_map.FindFromIndex(i).Size() < 2)
    assert free_edges == 0


def test_sewn_tcone_separates_the_frustum_from_a_bigger_box():
    """Use the sewn 3-face shell as a Gsplit tool against a box that
    fully contains the frustum -- with no BOPAlgo_Splitter call made
    while building the tool itself. Ground truth: the frustum's own
    analytic volume, pi*h*(r1^2 + r1*r2 + r2^2)/3."""
    frustum = _build_frustum_solid()
    cone_face = _find_cone_face(frustum)
    cap_bottom = _find_cap(frustum, 0.0)
    cap_top = _find_cap(frustum, _HEIGHT)
    edges_bottom = _shared_edges(cone_face, cap_bottom)
    edges_top = _shared_edges(cone_face, cap_top)

    cone = GCone.from_values(cone_face.Surface.Apex, cone_face.Surface.Axis, cone_face.Surface.SemiAngle)
    tool = Gmake_sewn_tcone(cone, edges_bottom, cap_bottom.Surface, edges_top, cap_top.Surface)

    big_box = Gmake_box(-20, -20, -5, 20, 20, 20)
    result = Gsplit(big_box, tool, _TOLERANCES)

    frustum_volume = math.pi * _HEIGHT * (_RADIUS_BOTTOM**2 + _RADIUS_BOTTOM * _RADIUS_TOP + _RADIUS_TOP**2) / 3.0
    box_volume = 40.0 * 40.0 * 25.0
    expected = sorted([frustum_volume, box_volume - frustum_volume])
    volumes = sorted(round(s.Volume, 2) for s in result.solids)
    assert volumes == pytest.approx(expected, abs=1e-1)
