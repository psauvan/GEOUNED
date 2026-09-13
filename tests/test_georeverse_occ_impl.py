import math
import os

import pytest

os.environ.setdefault("GEOUNED_CAD_ENGINE", "occ")

pytest.importorskip("OCC.Core.BRepPrimAPI", reason="pythonocc-core not available on this machine")

if os.environ.get("GEOUNED_CAD_ENGINE", "freecad").strip().lower() != "occ":
    pytest.skip(
        "GEOUNED_CAD_ENGINE is not 'occ' -- geouned.geo already loaded a different backend in this process",
        allow_module_level=True,
    )

from OCC.Core.BRepCheck import BRepCheck_Analyzer

from geouned.geo import GVector
from geouned.GEOReverse.Modules.engine_dependency._occ_impl import (
    Gmake_ellipsoid,
    Gmake_elliptic_cylinder,
    Gmake_torus_elliptic,
)

MAJOR_AXIS = GVector(1, 0, 0)
MINOR_AXIS = GVector(0, 1, 0)
CENTER = GVector(5, -3, 2)


def _spheroid_volume(rev_radius, perp_radius):
    return (4.0 / 3.0) * math.pi * perp_radius * perp_radius * rev_radius


def test_gmake_ellipsoid_prolate_axis_is_major():
    """Revolving around the ellipse's own major axis: axis radius (30) >
    perpendicular radius (10) -- a prolate (elongated) spheroid."""
    shape = Gmake_ellipsoid(CENTER, MAJOR_AXIS, 30.0, 10.0, MAJOR_AXIS, MINOR_AXIS)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _spheroid_volume(30.0, 10.0)) < 1e-3


def test_gmake_ellipsoid_oblate_axis_is_minor():
    """Revolving around the ellipse's own minor axis: axis radius (10) <
    perpendicular radius (30) -- an oblate (flattened) spheroid."""
    shape = Gmake_ellipsoid(CENTER, MINOR_AXIS, 30.0, 10.0, MAJOR_AXIS, MINOR_AXIS)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _spheroid_volume(10.0, 30.0)) < 1e-3


def test_gmake_ellipsoid_degenerate_sphere():
    """Equal radii: the spheroid degenerates to a plain sphere."""
    shape = Gmake_ellipsoid(CENTER, MAJOR_AXIS, 15.0, 15.0, MAJOR_AXIS, MINOR_AXIS)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - (4.0 / 3.0) * math.pi * 15.0**3) < 1e-3


def test_gmake_ellipsoid_arbitrary_axis_orientation():
    """The construction must not depend on the axis being aligned with a
    global X/Y/Z direction."""
    axis = GVector(0.3, -0.5, 0.8).normalized()
    perp = axis.cross(GVector(1, 0, 0)).normalized()
    shape = Gmake_ellipsoid(CENTER, axis, 22.0, 8.0, axis, perp)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _spheroid_volume(22.0, 8.0)) < 1e-2


def test_gmake_elliptic_cylinder_volume_and_validity():
    shape = Gmake_elliptic_cylinder(CENTER, GVector(0, 0, 1), 20.0, 8.0, MAJOR_AXIS, MINOR_AXIS, 50.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - math.pi * 20.0 * 8.0 * 50.0) < 1e-3


def test_gmake_elliptic_cylinder_center_is_extrusion_start_point():
    """`center` is the extrusion's own start point (matching
    `_freecad_impl.py`'s own convention, confirmed with the user before
    implementing) -- the built solid spans from `center` to
    `center + axis * height`, not symmetrically around `center`."""
    axis = GVector(0, 0, 1)
    height = 50.0
    shape = Gmake_elliptic_cylinder(CENTER, axis, 20.0, 8.0, MAJOR_AXIS, MINOR_AXIS, height)
    bbox = shape.BoundBox
    assert abs(bbox.ZMin - CENTER.z) < 1e-3
    assert abs(bbox.ZMax - (CENTER.z + height)) < 1e-3


def test_gmake_elliptic_cylinder_arbitrary_axis_orientation():
    axis = GVector(0.3, -0.5, 0.8).normalized()
    major = axis.cross(GVector(1, 0, 0)).normalized()
    minor = axis.cross(major).normalized()
    shape = Gmake_elliptic_cylinder(CENTER, axis, 14.0, 6.0, major, minor, 33.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - math.pi * 14.0 * 6.0 * 33.0) < 1e-2


def _elliptic_torus_volume(major_radius, minor_radius_a, minor_radius_b):
    return 2.0 * math.pi**2 * major_radius * minor_radius_a * minor_radius_b


def test_gmake_torus_elliptic_elongated_along_axis():
    """minor_radius_b (12, axis-paired) > minor_radius_a (6, radial-paired,
    same direction as major_radius): the torus is elongated along its own
    axis, without needing the internal Geom_Ellipse major/minor swap."""
    shape = Gmake_torus_elliptic(CENTER, GVector(0, 0, 1), 40.0, 6.0, 12.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _elliptic_torus_volume(40.0, 6.0, 12.0)) < 1e-2


def test_gmake_torus_elliptic_flattened():
    """minor_radius_a (10, radial-paired) > minor_radius_b (5, axis-paired):
    a flattened ("oblate") torus -- forces the internal Geom_Ellipse
    MajorRadius >= MinorRadius swap."""
    shape = Gmake_torus_elliptic(CENTER, GVector(0, 0, 1), 40.0, 10.0, 5.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _elliptic_torus_volume(40.0, 10.0, 5.0)) < 1e-2


def test_gmake_torus_elliptic_arbitrary_axis_orientation():
    axis = GVector(0.3, -0.5, 0.8).normalized()
    shape = Gmake_torus_elliptic(CENTER, axis, 25.0, 4.0, 9.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _elliptic_torus_volume(25.0, 4.0, 9.0)) < 1e-2


# Degenerate torus (tube self-intersects the axis, abs(major_radius) <
# minor_radius_a): expected volumes cross-validated independently via a
# Pappus numerical integration over the kept arc (not the closed-form
# circular/elliptic torus formula above, which only applies to the
# non-degenerate case) -- see the history log for the verification script.
# Circular case uses minor_radius_a == minor_radius_b (a circle is just a
# degenerate ellipse).
AXIS_Z = GVector(0, 0, 1)


def test_gmake_torus_elliptic_degenerate_circular_outer():
    shape = Gmake_torus_elliptic(CENTER, AXIS_Z, 5.0, 8.0, 8.0, outer=True)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - 6516.8684) < 1e-2


def test_gmake_torus_elliptic_degenerate_circular_inner():
    shape = Gmake_torus_elliptic(CENTER, AXIS_Z, 5.0, 8.0, 8.0, outer=False)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - 200.3215) < 1e-2


def test_gmake_torus_elliptic_degenerate_outer_bigger_than_inner():
    outer_shape = Gmake_torus_elliptic(CENTER, AXIS_Z, 5.0, 8.0, 8.0, outer=True)
    inner_shape = Gmake_torus_elliptic(CENTER, AXIS_Z, 5.0, 8.0, 8.0, outer=False)
    assert outer_shape.Volume > inner_shape.Volume


def test_gmake_torus_elliptic_degenerate_elongated_along_axis():
    """minor_radius_b (10, axis-paired) > minor_radius_a (6, radial-paired):
    no internal Geom_Ellipse swap needed."""
    shape = Gmake_torus_elliptic(CENTER, AXIS_Z, 4.0, 6.0, 10.0, outer=True)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - 4842.8500) < 1e-2

    shape_inner = Gmake_torus_elliptic(CENTER, AXIS_Z, 4.0, 6.0, 10.0, outer=False)
    assert BRepCheck_Analyzer(shape_inner.__native__).IsValid()
    assert abs(shape_inner.Volume - 105.4399) < 1e-2


def test_gmake_torus_elliptic_degenerate_flattened():
    """minor_radius_a (9, radial-paired) > minor_radius_b (4, axis-paired):
    forces the internal Geom_Ellipse major/minor swap."""
    shape = Gmake_torus_elliptic(CENTER, AXIS_Z, 3.0, 9.0, 4.0, outer=True)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - 2647.1616) < 1e-2

    shape_inner = Gmake_torus_elliptic(CENTER, AXIS_Z, 3.0, 9.0, 4.0, outer=False)
    assert BRepCheck_Analyzer(shape_inner.__native__).IsValid()
    assert abs(shape_inner.Volume - 515.3270) < 1e-2


# `outer` defaults to None, meaning "derive from the sign of
# major_radius" -- matching the round-trip convention already established
# via GTorus.a_sign/torus_sheet_sign on the forward side
# (GEOUNED/write/functions.py negates the major radius for the inner
# sheet when writing a degenerate torus back to MCNP/OpenMC/etc), so a
# real signed `Ra` read back from such a file needs no extra logic from
# the caller.
def test_gmake_torus_elliptic_degenerate_positive_r_defaults_to_outer():
    shape = Gmake_torus_elliptic(CENTER, AXIS_Z, 5.0, 8.0, 8.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - 6516.8684) < 1e-2


def test_gmake_torus_elliptic_degenerate_negative_r_defaults_to_inner():
    shape = Gmake_torus_elliptic(CENTER, AXIS_Z, -5.0, 8.0, 8.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - 200.3215) < 1e-2


def test_gmake_torus_elliptic_degenerate_explicit_outer_overrides_sign():
    shape = Gmake_torus_elliptic(CENTER, AXIS_Z, -5.0, 8.0, 8.0, outer=True)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - 6516.8684) < 1e-2


def test_gmake_torus_elliptic_non_degenerate_ignores_r_sign():
    """`outer` (whether explicit or sign-derived) only matters in the
    degenerate case -- the non-degenerate shape's magnitude and validity
    must not depend on major_radius's own sign."""
    shape_pos = Gmake_torus_elliptic(CENTER, AXIS_Z, 40.0, 6.0, 12.0)
    shape_neg = Gmake_torus_elliptic(CENTER, AXIS_Z, -40.0, 6.0, 12.0)
    assert BRepCheck_Analyzer(shape_pos.__native__).IsValid()
    assert BRepCheck_Analyzer(shape_neg.__native__).IsValid()
    assert abs(shape_pos.Volume - shape_neg.Volume) < 1e-2
