"""Prototype validation (branch `sewing-composite-surface`) -- occ mirror
of test_sewn_composite_ocp.py. See that file's and
geo/occ/sewn_composite.py's own module docstrings for the design
rationale; freecad is out of scope for this prototype.
"""

import os
from types import SimpleNamespace

import pytest

os.environ.setdefault("GEOUNED_CAD_ENGINE", "occ")

pytest.importorskip("OCC.Core.BRepPrimAPI", reason="pythonocc-core not available on this machine")

if os.environ.get("GEOUNED_CAD_ENGINE", "ocp").strip().lower() != "occ":
    pytest.skip(
        "GEOUNED_CAD_ENGINE is not 'occ' -- geouned.geo already loaded a different backend in this process",
        allow_module_level=True,
    )

from geouned.geo import GPlane, GVector, Gcut, Gmake_box, Gsplit
from geouned.geo.occ.sewn_composite import Gmake_sewn_plane_pair

_TOLERANCES = SimpleNamespace(
    split_tolerance=1e-6,
    scale_up_floor=None,
    min_solid_volume=1e-6,
    min_face_width=0.1,
    fix_tolerance=1e-6,
    volume_tolerance=1e-6,
)


def _find_plane_face(solid, outward_axis: GVector, position_coord: str, position_value: float):
    # GPlane.Axis reports the underlying surface's own intrinsic normal,
    # not adjusted for the TopoDS_Face's own Forward/Reversed orientation
    # -- a plain axis-direction filter alone can match a same-normal face
    # at a completely different position (e.g. the box's original x=0
    # face shares Axis=(1,0,0) with the notch's own new x=5 face), so the
    # position must be checked too.
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


def test_gmake_sewn_plane_pair_builds_a_single_shared_seam():
    """The two oversized faces must share the exact same native edge along
    their junction -- the whole point of reusing the real edge's own
    curve (design point 3) instead of letting Sewing discover a
    coincidence numerically."""
    notched = Gcut(Gmake_box(0, 0, 0, 10, 10, 10), [Gmake_box(5, 5, -1, 11, 11, 11)])[0]
    face_x = _find_plane_face(notched, GVector(1, 0, 0), "x", 5.0)
    face_y = _find_plane_face(notched, GVector(0, 1, 0), "y", 5.0)
    shared = _shared_edge(face_x, face_y)
    assert shared is not None

    tool = Gmake_sewn_plane_pair(face_x.Surface, face_x.CenterOfMass, face_y.Surface, face_y.CenterOfMass, shared, 20.0)

    assert len(tool.Faces) == 2
    f1, f2 = tool.Faces
    junction1 = [e for e in f1.Edges if any(e.is_same(e2) for e2 in f2.Edges)]
    assert len(junction1) == 1


def test_sewn_plane_pair_separates_a_notch_like_two_full_cuts():
    """Use the sewn 2-face shell -- built from a genuinely real junction
    edge harvested from an already-notched solid's own topology, exactly
    how a real MultiPlane's own detection finds it -- directly as a
    Gsplit cutting tool against the UN-notched box, with no
    BOPAlgo_Splitter call made while building the tool itself (point 1 +
    point 3). Ground truth: the tool's own "L" wall exactly separates the
    original box into the notched L-shaped body (volume 750) and the
    corner cube that would have been removed (volume 250) -- point 2:
    nothing here needed to know in advance which side of the composite
    the material was on, that still falls out of Gsplit's own, unchanged,
    point-classification downstream."""
    notch_tool = Gmake_box(5, 5, -1, 11, 11, 11)
    big = Gmake_box(0, 0, 0, 10, 10, 10)
    notched = Gcut(big, [notch_tool])[0]

    face_x = _find_plane_face(notched, GVector(1, 0, 0), "x", 5.0)
    face_y = _find_plane_face(notched, GVector(0, 1, 0), "y", 5.0)
    shared = _shared_edge(face_x, face_y)
    assert shared is not None

    tool = Gmake_sewn_plane_pair(face_x.Surface, face_x.CenterOfMass, face_y.Surface, face_y.CenterOfMass, shared, 20.0)

    result = Gsplit(big, tool, _TOLERANCES)

    volumes = sorted(round(s.Volume, 3) for s in result.solids)
    assert volumes == pytest.approx([250.0, 750.0], abs=1e-3)
