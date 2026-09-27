"""
Tests for `GEOUNED/utils/meta_surfaces_utils.py::region_sign`.

Regression test for a real finding (2026-09-27, verified against a real
fixture -- `debug/origSolid_0.stp`, generated via `Settings.debug=True` on
a real workshop model): `region_sign` used to force an "AND"/"OR" answer
even when the two faces it compares are genuinely the SAME analytic
surface (e.g. a residual same-plane sliver left by a boolean cut,
sharing an edge with the real face it split from). There is no real
dihedral angle in that case (it's exactly 0), so intersecting or
unioning one half-space with itself gives that same half-space either
way -- the question is ill-posed, not just hard to decide. Point-sampling
the real solid's own material even suggested a specific answer ("AND"),
but that ground truth turned out not to be meaningful either (the sample
point never leaves the shared plane, so it's a boundary-membership test,
not a genuine interior/exterior one) -- per direct user instruction, the
fix is to skip the test entirely (return `None`) rather than force
either answer.
"""

import geouned
from geouned.GEOUNED.utils.geometry_gu import SolidGu
from geouned.GEOUNED.utils.meta_surfaces_utils import region_sign
from geouned.geo import GVector, Gmake_box


def _box_solid_gu():
    box = Gmake_box(0, 0, 0, 10, 10, 10)
    return SolidGu(box, tolerances=geouned.Tolerances())


def test_region_sign_returns_none_for_the_same_surface():
    """The exact same face compared against itself is the simplest,
    fully deterministic reproduction of the "same analytic surface"
    case the real fixture surfaced -- `is_same_surface` must gate this
    before any AND/OR logic runs."""
    solid_gu = _box_solid_gu()
    face = solid_gu.Faces[0]

    assert region_sign(face, face, tolerances=solid_gu.tolerances) is None

    sign, angle = region_sign(face, face, outAngle=True, tolerances=solid_gu.tolerances)
    assert sign is None
    assert angle is None


def test_region_sign_still_resolves_a_genuine_convex_corner():
    """A plain box's own two adjacent faces meet at a real 90-degree
    convex corner (never the same surface) -- confirms the new
    same-surface gate only suppresses the genuinely degenerate case
    above, not real, everyday adjacency."""
    solid_gu = _box_solid_gu()
    face = solid_gu.Faces[0]

    found_real_pair = False
    for other in solid_gu.Faces[1:]:
        sign = region_sign(face, other, tolerances=solid_gu.tolerances)
        if sign is not None:
            found_real_pair = True
            assert sign == "AND"  # a box is convex everywhere

    assert found_real_pair
