"""Prototype validation (branch `sewing-composite-surface`) for
`Gmake_sewn_can` -- the Can case (1 cylinder, closed 360 degrees, + 2
real closures, each a plane, sphere, cone, or cylinder). Every face of
a Can is built the same wire-based way (`_sign_corrected_face`),
regardless of each closing surface's own type or whether it's coaxial
with the main cylinder -- see geo/ocp/sewn_composite.py's own module
docstring for the full rationale and the real fixture
(`Cans/barrel_left.stp`, plane/cylinder only) this was validated
against end to end.

A sphere or cone cap reuses the real closing face's own COMPLETE
boundary (`face_edges`, not just the tangency curve shared with the
cylinder) -- which, for a dome/nozzle that genuinely narrows to a
single point, already carries one real OCCT *degenerate* edge at that
pole/apex (both endpoints identical, zero length) built by whatever
operation created the solid, not reconstructed here. This is the
standard, expected topology for a full 360-degree surface trimmed up
to its own singular point, not a watertightness defect -- so the
watertightness check below excludes degenerate edges explicitly (a
plain free-edge count would wrongly flag this one edge even though the
shell is otherwise fully sewn)."""

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
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCP.TopoDS import TopoDS

from geouned.geo import (
    GCone,
    GCylinder,
    GPlane,
    GSphere,
    GVector,
    Gcut,
    Gfuse,
    Gmake_box,
    Gmake_cone,
    Gmake_cylinder,
    Gmake_sphere,
    Gsplit,
)
from geouned.geo.ocp.sewn_composite import Gmake_sewn_can

_TOLERANCES = SimpleNamespace(
    split_tolerance=1e-6,
    scale_up_floor=None,
    min_solid_volume=1e-6,
    min_face_width=0.1,
    fix_tolerance=1e-6,
    volume_tolerance=1e-6,
    sliver_edge_rel_tol=1e-4,
)

_R_CYL = 5.0
_HEIGHT = 10.0


def _shared_edges(face1, face2):
    return [e1 for e1 in face1.Edges for e2 in face2.Edges if e1.is_same(e2)]


def _free_non_degenerate_edges(tool):
    native_shell = tool.__native__
    edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(native_shell, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
    count = 0
    for i in range(1, edge_face_map.Extent() + 1):
        if edge_face_map.FindFromIndex(i).Size() >= 2:
            continue
        edge = TopoDS.Edge(edge_face_map.FindKey(i))
        if BRep_Tool.Degenerated_s(edge):
            continue
        count += 1
    return count


def _build_domed_can():
    """A cylinder (flat bottom) closed at the top by a spherical dome,
    tangent-free (a real, visible shoulder at the junction -- no G1
    continuity needed, only a shared real edge): sphere radius R > r,
    centered on the cylinder's own axis so the tangency circle (radius
    r) sits at a single constant latitude -- the coaxial case
    `Gmake_sewn_can`'s own sphere cap assumes."""
    r_sph = 7.0
    d = math.sqrt(r_sph**2 - _R_CYL**2)
    cyl = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), _R_CYL, _HEIGHT)
    sphere_solid = Gmake_sphere(GVector(0, 0, _HEIGHT - d), r_sph)
    dome = Gcut(sphere_solid, [Gmake_box(-20, -20, -20, 20, 20, _HEIGHT)])[0]
    cap_h = r_sph - d
    dome_volume = math.pi * cap_h**2 * (3 * r_sph - cap_h) / 3.0
    cyl_volume = math.pi * _R_CYL**2 * _HEIGHT
    return Gfuse([cyl, dome]), cyl_volume + dome_volume


def _build_nozzle_can():
    """A cylinder (flat bottom) closed at the top by a coaxial conical
    nozzle narrowing to a real apex."""
    length = 6.0
    half_angle = math.atan2(_R_CYL, length)
    apex = GVector(0, 0, _HEIGHT + length)
    cone_solid = Gmake_cone(apex, GVector(0, 0, -1), half_angle, length)
    cyl = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), _R_CYL, _HEIGHT)
    cone_volume = math.pi / 3.0 * _R_CYL**2 * length
    cyl_volume = math.pi * _R_CYL**2 * _HEIGHT
    return Gfuse([cyl, cone_solid]), cyl_volume + cone_volume


def _faces_of(solid):
    cyl_face = next(f for f in solid.Faces if isinstance(f.Surface, GCylinder))
    plane_face = next(f for f in solid.Faces if isinstance(f.Surface, GPlane))
    other_face = next(f for f in solid.Faces if not isinstance(f.Surface, (GCylinder, GPlane)))
    return cyl_face, plane_face, other_face


@pytest.mark.parametrize("build_can", [_build_domed_can, _build_nozzle_can], ids=["sphere_cap", "cone_cap"])
def test_gmake_sewn_can_builds_a_watertight_shell(build_can):
    can_solid, _ = build_can()
    cyl_face, plane_face, other_face = _faces_of(can_solid)

    edges_plane = _shared_edges(cyl_face, plane_face)
    edges_other = _shared_edges(cyl_face, other_face)
    assert len(edges_plane) >= 1 and len(edges_other) >= 1

    cylinder = GCylinder.from_values(cyl_face.Surface.Center, cyl_face.Surface.Axis, cyl_face.Surface.Radius)
    tool = Gmake_sewn_can(
        cylinder, edges_plane, plane_face.Surface, edges_other, other_face.Surface, plane_face.Edges, other_face.Edges
    )

    assert tool is not None
    assert len(tool.Faces) == 3
    assert _free_non_degenerate_edges(tool) == 0


@pytest.mark.parametrize("build_can", [_build_domed_can, _build_nozzle_can], ids=["sphere_cap", "cone_cap"])
def test_sewn_can_separates_the_can_from_a_bigger_box(build_can):
    can_solid, expected_volume = build_can()
    cyl_face, plane_face, other_face = _faces_of(can_solid)
    edges_plane = _shared_edges(cyl_face, plane_face)
    edges_other = _shared_edges(cyl_face, other_face)

    cylinder = GCylinder.from_values(cyl_face.Surface.Center, cyl_face.Surface.Axis, cyl_face.Surface.Radius)
    tool = Gmake_sewn_can(
        cylinder, edges_plane, plane_face.Surface, edges_other, other_face.Surface, plane_face.Edges, other_face.Edges
    )

    big_box = Gmake_box(-30, -30, -30, 30, 30, 30)
    result = Gsplit(big_box, tool, _TOLERANCES)

    box_volume = 60.0**3
    expected = sorted([expected_volume, box_volume - expected_volume])
    volumes = sorted(round(s.Volume, 2) for s in result.solids)
    assert volumes == pytest.approx(expected, abs=1e-1)
