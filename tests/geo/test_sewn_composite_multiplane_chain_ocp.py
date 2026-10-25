"""Prototype validation (branch `sewing-composite-surface`) for
`Gmake_sewn_multiplane_chain` -- the N>2 generalization of
`Gmake_sewn_plane_pair`, for a MultiPlane composite whose component
planes form a simple open chain (a zigzag/staircase of 3+ panels, each
touching at most 2 neighbors). See geo/ocp/sewn_composite.py's own
module docstring for the design.
"""

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

from geouned.geo import GPlane, GVector, Gcut, Gmake_box, Gsplit
from geouned.geo.ocp.sewn_composite import Gmake_sewn_multiplane_chain

_TOLERANCES = SimpleNamespace(
    split_tolerance=1e-6,
    scale_up_floor=None,
    min_solid_volume=1e-6,
    min_face_width=0.1,
    fix_tolerance=1e-6,
    volume_tolerance=1e-6,
)


def _find_plane_face(solid, outward_axis: GVector, position_coord: str, position_value: float):
    for f in solid.Faces:
        if not isinstance(f.Surface, GPlane) or f.Surface.Axis.dot(outward_axis) <= 0.9:
            continue
        if abs(getattr(f.Surface.Position, position_coord) - position_value) < 1e-6:
            return f
    return None


def _shared_edge(face1, face2):
    for e1 in face1.Edges:
        for e2 in face2.Edges:
            if e1.is_same(e2):
                return e1
    return None


def _build_staircase_solid():
    """A box with its (x=10, y=10) corner replaced by a real 2-step
    staircase notch, cut with 2 overlapping boxes -- the resulting solid
    carries a genuine 4-plane open chain (x=7 -> y=8 -> x=9 -> y=7, each
    pair of neighbors sharing one real edge): the box's own x=10/y=10
    faces each connect to one end of the chain through a FREE boundary
    (not a 5th shared edge), so this fixture exercises both the chain's
    2 endpoint links (1 real edge, `_oversized_plane_face`) and its 2
    interior links (2 real edges, `_oversized_plane_face_two_edges`)."""
    big = Gmake_box(0, 0, 0, 10, 10, 10)
    cut_a = Gmake_box(7, 8, -1, 11, 11, 11)
    cut_b = Gmake_box(9, 7, -1, 11, 11, 11)
    return Gcut(big, [cut_a, cut_b])[0]


def _chain_faces(staircase):
    plane1 = _find_plane_face(staircase, GVector(1, 0, 0), "x", 7.0)
    plane2 = _find_plane_face(staircase, GVector(0, 1, 0), "y", 8.0)
    plane3 = _find_plane_face(staircase, GVector(1, 0, 0), "x", 9.0)
    plane4 = _find_plane_face(staircase, GVector(0, 1, 0), "y", 7.0)
    assert None not in (plane1, plane2, plane3, plane4)
    return plane1, plane2, plane3, plane4


def test_gmake_sewn_multiplane_chain_shares_three_real_seams():
    """The 4-face shell must reuse the 3 real junction edges exactly --
    one seam per consecutive pair -- not a numerically re-derived
    coincidence, and the 3 seams must be genuinely distinct edges."""
    staircase = _build_staircase_solid()
    plane1, plane2, plane3, plane4 = _chain_faces(staircase)

    edge_12 = _shared_edge(plane1, plane2)
    edge_23 = _shared_edge(plane2, plane3)
    edge_34 = _shared_edge(plane3, plane4)
    assert None not in (edge_12, edge_23, edge_34)

    tool = Gmake_sewn_multiplane_chain(
        [plane1.Surface, plane2.Surface, plane3.Surface, plane4.Surface],
        [plane1.CenterOfMass, plane2.CenterOfMass, plane3.CenterOfMass, plane4.CenterOfMass],
        [edge_12, edge_23, edge_34],
        20.0,
    )

    assert len(tool.Faces) == 4
    f1, f2, f3, f4 = tool.Faces
    seam_12 = [e for e in f1.Edges if any(e.is_same(e2) for e2 in f2.Edges)]
    seam_23 = [e for e in f2.Edges if any(e.is_same(e2) for e2 in f3.Edges)]
    seam_34 = [e for e in f3.Edges if any(e.is_same(e2) for e2 in f4.Edges)]
    assert len(seam_12) == 1 and len(seam_23) == 1 and len(seam_34) == 1
    seams = [seam_12[0], seam_23[0], seam_34[0]]
    for i in range(3):
        for j in range(i + 1, 3):
            assert not seams[i].is_same(seams[j])


def test_sewn_multiplane_chain_separates_the_staircase_notch():
    """Use the sewn 4-face shell as a Gsplit tool against the ORIGINAL,
    sharp-cornered box -- with no BOPAlgo_Splitter call made while
    building the tool itself. Ground truth: box volume 1000, 2-step
    notch volume 70 (union of the 2 cutting boxes, clipped to the box's
    own extent), so the tool must exactly separate [70, 930]."""
    staircase = _build_staircase_solid()
    plane1, plane2, plane3, plane4 = _chain_faces(staircase)
    edge_12 = _shared_edge(plane1, plane2)
    edge_23 = _shared_edge(plane2, plane3)
    edge_34 = _shared_edge(plane3, plane4)

    tool = Gmake_sewn_multiplane_chain(
        [plane1.Surface, plane2.Surface, plane3.Surface, plane4.Surface],
        [plane1.CenterOfMass, plane2.CenterOfMass, plane3.CenterOfMass, plane4.CenterOfMass],
        [edge_12, edge_23, edge_34],
        20.0,
    )

    sharp_box = Gmake_box(0, 0, 0, 10, 10, 10)
    result = Gsplit(sharp_box, tool, _TOLERANCES)

    volumes = sorted(round(s.Volume, 3) for s in result.solids)
    assert volumes == pytest.approx([70.0, 930.0], abs=1e-2)


def test_gmake_sewn_multiplane_chain_subsumes_the_2_plane_case():
    """`len(planes) == 2` must behave exactly like `Gmake_sewn_plane_pair`
    (its own single-edge test fixture, reused here) -- the chain
    function is a strict generalization, not a separate code path for
    the simple case."""
    from geouned.geo.ocp.sewn_composite import Gmake_sewn_plane_pair

    notched = Gcut(Gmake_box(0, 0, 0, 10, 10, 10), [Gmake_box(5, 5, -1, 11, 11, 11)])[0]
    face_x = _find_plane_face(notched, GVector(1, 0, 0), "x", 5.0)
    face_y = _find_plane_face(notched, GVector(0, 1, 0), "y", 5.0)
    shared = _shared_edge(face_x, face_y)

    pair_tool = Gmake_sewn_plane_pair(face_x.Surface, face_x.CenterOfMass, face_y.Surface, face_y.CenterOfMass, shared, 20.0)
    chain_tool = Gmake_sewn_multiplane_chain(
        [face_x.Surface, face_y.Surface], [face_x.CenterOfMass, face_y.CenterOfMass], [shared], 20.0
    )

    big = Gmake_box(0, 0, 0, 10, 10, 10)
    pair_result = Gsplit(big, pair_tool, _TOLERANCES)
    chain_result = Gsplit(big, chain_tool, _TOLERANCES)

    pair_volumes = sorted(round(s.Volume, 3) for s in pair_result.solids)
    chain_volumes = sorted(round(s.Volume, 3) for s in chain_result.solids)
    assert chain_volumes == pytest.approx(pair_volumes, abs=1e-3)
