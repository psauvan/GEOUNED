"""Tests for the GQ/SQ -> surface-type classifier
(MCNP_parser/MCNPinput.py::gq2params/getGQAxis and its own callers).

Engine-agnostic (this code is pure numpy math, no native CAD kernel
calls) -- runs under whatever GEOUNED_CAD_ENGINE the interpreter has,
same as test_csgtocad.py.

Context (2026-09-14 investigation, see CLAUDE.md/history log): the
classifier used fixed *absolute* tolerances (and, in getGQAxis, outright
exact `== 0` comparisons) to decide whether two eigenvalues of the
quadric's matrix are equal or zero. Since a GQ card is only defined up
to an arbitrary nonzero overall scale (multiplying all 10 coefficients
by any constant doesn't change the surface, but scales every eigenvalue
and the reduced constant `k` by that same constant), a fixed absolute
tolerance cannot work across differently-normalized cards representing
the same surface -- confirmed live: the one real GQ fixture in this
repo (a clean circular cylinder) had its "zero" eigenvalue land at
-5.55e-17, which the old `e0 == 0` test missed entirely, misrouting
classification to "hyperboloid" (it only produced the right answer by
accident, via an unrelated large-radius-ratio fallback). Fixed by
replacing every such comparison with a *relative* one, scaled against
the eigenvalues' own magnitude (see data_class.py::Tolerances).

These tests build synthetic GQ coefficients for a *rotated* (not
axis-aligned) cylinder -- axis-aligned cylinders have zero off-diagonal
coefficients to begin with and can't exercise this rounding-sensitivity
at all -- then round them to a realistic hand-typed-MCNP-card precision
(6 significant figures) before classifying, to confirm the fix actually
forgives that rounding rather than just the one already-clean fixture.
"""

import math

import numpy as np

import geouned  # must be imported before Part -- sets up FreeCAD's sys.path
from geouned.GEOReverse.Modules.MCNP_parser.MCNPinput import gq2params


def _round_sig(x: float, sig: int) -> float:
    if x == 0:
        return 0.0
    d = sig - int(math.floor(math.log10(abs(x)))) - 1
    return round(x, d)


def _cylinder_gq_coeffs(center, axis, radius_u, radius_v, sig_figs=None):
    """Builds the 10 raw GQ coefficients (A,B,C,D,E,F,G,H,J,K, matching
    MCNPinput.py::gq2params's own unpacking convention) for a cylinder
    -- circular if radius_u == radius_v, elliptic otherwise -- centered
    at `center`, axis `axis` (need not be unit or axis-aligned), cross-
    section semi-axes `radius_u`/`radius_v` along two directions
    orthogonal to `axis` (picked arbitrarily, the cylinder doesn't care
    which for the circular case). Rounds every coefficient to
    `sig_figs` significant figures if given, simulating a real,
    hand-typed MCNP card's own limited precision."""
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    # an arbitrary vector not parallel to axis, to build an orthonormal frame
    seed = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, seed)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    center = np.array(center, dtype=float)
    M = np.outer(u, u) / radius_u**2 + np.outer(v, v) / radius_v**2
    lin = -2.0 * (M @ center)
    const = center @ M @ center - 1.0

    coeffs = [
        M[0, 0],
        M[1, 1],
        M[2, 2],
        2 * M[0, 1],
        2 * M[1, 2],
        2 * M[0, 2],
        lin[0],
        lin[1],
        lin[2],
        const,
    ]
    if sig_figs is not None:
        coeffs = [_round_sig(c, sig_figs) for c in coeffs]
    return tuple(coeffs)


AXIS = (0.55, 0.30, 0.78)  # deliberately not axis-aligned
CENTER = (12.3, -5.4, 7.8)


def test_real_fixture_rotated_cylinder_classifies_as_cylinder():
    """The exact coefficients from tests/csg_files/cylinder_box.mcnp
    surface 14 -- a real, clean circular cylinder whose own eigenvalue
    computation lands at -5.55e-17 rather than exactly 0.0, and used to
    misroute to "hyperboloid" under the old exact-equality test."""
    coeffs = (
        0.103855177195422,
        0.999999999999999,
        0.896144822804578,
        -0.000000018142699,
        -0.000000006176279,
        -0.610145160974434,
        -17.298344774043834,
        34.000069238333026,
        50.813553217939962,
        1006.753657691386934,
    )
    stype, params = gq2params(coeffs)
    assert stype == "cylinder"
    assert params is not None
    _, _, radius = params
    assert abs(radius - 1.6) < 1e-2


def test_rotated_circular_cylinder_survives_6sigfig_rounding():
    """A circular cylinder, rotated so its raw GQ coefficients are all
    nonzero, rounded to 6 significant figures (a realistic hand-typed
    MCNP card) -- must still classify as a plain circular "cylinder",
    not "cylinder_elliptic"/"cylinder_hyperbolic"/an unrelated type."""
    coeffs = _cylinder_gq_coeffs(CENTER, AXIS, radius_u=25.0, radius_v=25.0, sig_figs=6)
    stype, params = gq2params(coeffs)
    assert stype == "cylinder"
    assert params is not None
    _, _, radius = params
    assert abs(radius - 25.0) < 25.0 * 1e-3


def test_rotated_circular_cylinder_survives_8sigfig_rounding():
    """Same as above at a tighter (more typical of a script-generated
    deck) 8 significant figures -- should round-trip even more cleanly."""
    coeffs = _cylinder_gq_coeffs(CENTER, AXIS, radius_u=25.0, radius_v=25.0, sig_figs=8)
    stype, params = gq2params(coeffs)
    assert stype == "cylinder"
    assert params is not None
    _, _, radius = params
    assert abs(radius - 25.0) < 25.0 * 1e-5


def test_genuinely_elliptic_cylinder_not_rounded_to_circular():
    """A real, 20%-eccentric elliptic cylinder must NOT be swallowed by
    the same tolerance that forgives rounding noise -- the gap between
    "rounding" and "real ellipticity" is the whole point of using a
    tight, principled relative tolerance instead of a loose one."""
    coeffs = _cylinder_gq_coeffs(CENTER, AXIS, radius_u=25.0, radius_v=30.0, sig_figs=6)
    stype, params = gq2params(coeffs)
    assert stype == "cylinder_elliptic"
    assert params is not None
    _, _, radii, _ = params
    minor, major = sorted(radii)
    assert abs(minor - 25.0) < 25.0 * 1e-3
    assert abs(major - 30.0) < 30.0 * 1e-3


def test_axis_aligned_circular_cylinder_still_exact():
    """Control case: an axis-aligned circular cylinder has zero
    off-diagonal/cross terms from the start (no rotation to round away
    precision on), so it classifies correctly regardless of this fix --
    confirms the fix didn't regress the easy case."""
    coeffs = _cylinder_gq_coeffs((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), radius_u=10.0, radius_v=10.0)
    stype, params = gq2params(coeffs)
    assert stype == "cylinder"
    _, _, radius = params
    assert abs(radius - 10.0) < 1e-6
