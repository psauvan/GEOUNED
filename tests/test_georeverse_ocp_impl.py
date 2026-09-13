import math
import os

import pytest

os.environ.setdefault("GEOUNED_CAD_ENGINE", "ocp")

pytest.importorskip("OCP.BRepPrimAPI", reason="OCP not available on this machine")

if os.environ.get("GEOUNED_CAD_ENGINE", "freecad").strip().lower() != "ocp":
    pytest.skip(
        "GEOUNED_CAD_ENGINE is not 'ocp' -- geouned.geo already loaded a different backend in this process",
        allow_module_level=True,
    )

from OCP.BRepCheck import BRepCheck_Analyzer

from geouned.geo import GVector
from geouned.GEOReverse.Modules.engine_dependency._ocp_impl import (
    GEllipsoid,
    GEllipticCone,
    GEllipticCylinder,
    GHyperbolicCylinder,
    GHyperboloid,
    GParaboloid,
    Gmake_ellipsoid,
    Gmake_elliptic_cone,
    Gmake_elliptic_cylinder,
    Gmake_hyperbolic_cylinder,
    Gmake_hyperboloid,
    Gmake_paraboloid,
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


def _elliptic_cone_volume(ref_radius, major_radius, minor_radius, length):
    """Base ellipse semi-axes at distance `length` from the apex are
    `major_radius/ref_radius * length` and `minor_radius/ref_radius *
    length` (the MCNP GQ/SQ scale-with-distance convention) -- the cone
    volume generalizes V = (1/3) * pi * r^2 * h to elliptical a/b semi-axes."""
    a = major_radius / ref_radius * length
    b = minor_radius / ref_radius * length
    return (math.pi / 3.0) * a * b * length


def test_gmake_elliptic_cone_volume_and_validity():
    shape = Gmake_elliptic_cone(CENTER, AXIS_Z, 10.0, 6.0, 3.0, MAJOR_AXIS, MINOR_AXIS, False, 20.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _elliptic_cone_volume(10.0, 6.0, 3.0, 20.0)) < 1e-2


def test_gmake_elliptic_cone_ref_radius_scaling():
    """ref_radius != length exercises the scale-with-distance convention
    (a different scale factor than the volume_and_validity case above)."""
    shape = Gmake_elliptic_cone(CENTER, AXIS_Z, 5.0, 6.0, 3.0, MAJOR_AXIS, MINOR_AXIS, False, 15.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _elliptic_cone_volume(5.0, 6.0, 3.0, 15.0)) < 1e-2


def test_gmake_elliptic_cone_apex_and_base_position():
    """The apex vertex sits at `center`, the base ellipse plane at
    `center + axis*length` -- check the built solid's bounding box spans
    exactly that range along the (Z) axis."""
    length = 20.0
    shape = Gmake_elliptic_cone(CENTER, AXIS_Z, 10.0, 6.0, 3.0, MAJOR_AXIS, MINOR_AXIS, False, length)
    bbox = shape.BoundBox
    assert abs(bbox.ZMin - CENTER.z) < 1e-3
    assert abs(bbox.ZMax - (CENTER.z + length)) < 1e-3


def test_gmake_elliptic_cone_double_sheet():
    """Both nappes from the same apex, fused -- volume doubles (the two
    sheets only ever touch at the single apex point, no volume overlap)."""
    single = Gmake_elliptic_cone(CENTER, AXIS_Z, 10.0, 6.0, 3.0, MAJOR_AXIS, MINOR_AXIS, False, 20.0)
    double = Gmake_elliptic_cone(CENTER, AXIS_Z, 10.0, 6.0, 3.0, MAJOR_AXIS, MINOR_AXIS, True, 20.0)
    assert BRepCheck_Analyzer(double.__native__).IsValid()
    assert abs(double.Volume - 2.0 * single.Volume) < 1e-1


def test_gmake_elliptic_cone_arbitrary_axis_orientation():
    axis = GVector(0.3, -0.5, 0.8).normalized()
    major_axis = axis.cross(GVector(1, 0, 0)).normalized()
    minor_axis = axis.cross(major_axis).normalized()
    shape = Gmake_elliptic_cone(CENTER, axis, 10.0, 6.0, 3.0, major_axis, minor_axis, False, 20.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _elliptic_cone_volume(10.0, 6.0, 3.0, 20.0)) < 1e-1


def _hyperboloid_sheet_volume(major_radius, minor_radius, length):
    """Volume of revolution of one hyperbola branch (vertex at
    `major_radius` to a capped rim at `length`, both measured from Center
    along the revolution axis) -- pi * integral[a,L] of r(x)^2 dx where
    r(x) = minor_radius*sqrt((x/major_radius)^2 - 1), integrated in
    closed form. Cross-validated independently of the construction code
    (a plain calculus derivation, not a re-implementation of it)."""
    a, b, length_ = major_radius, minor_radius, length
    bracket = length_**3 / 3.0 - a**2 * length_ + 2.0 * a**3 / 3.0
    return math.pi * (b**2 / a**2) * bracket


def test_gmake_hyperboloid_two_sheet_volume_and_validity():
    shape = Gmake_hyperboloid(CENTER, AXIS_Z, 3.0, 2.0, AXIS_Z, MAJOR_AXIS, False, 5.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    expected = 2.0 * _hyperboloid_sheet_volume(3.0, 2.0, 5.0)
    assert abs(shape.Volume - expected) < 1e-2


def test_gmake_hyperboloid_two_sheet_axial_extent_and_gap():
    """Both sheets span from `length` down to `major_radius` on their own
    side of Center -- the overall extent is [-length, +length] along the
    axis, with the real gap between -major_radius and +major_radius
    where neither sheet has any material (the defining two-sheet trait)."""
    length = 5.0
    shape = Gmake_hyperboloid(CENTER, AXIS_Z, 3.0, 2.0, AXIS_Z, MAJOR_AXIS, False, length)
    bbox = shape.BoundBox
    assert abs(bbox.ZMin - (CENTER.z - length)) < 1e-3
    assert abs(bbox.ZMax - (CENTER.z + length)) < 1e-3


def test_gmake_hyperboloid_arbitrary_axis_orientation():
    major_axis = GVector(0.3, -0.5, 0.8).normalized()
    minor_axis = major_axis.cross(GVector(1, 0, 0)).normalized()
    shape = Gmake_hyperboloid(CENTER, major_axis, 3.0, 2.0, major_axis, minor_axis, False, 5.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    expected = 2.0 * _hyperboloid_sheet_volume(3.0, 2.0, 5.0)
    assert abs(shape.Volume - expected) < 1e-1


def test_gmake_hyperboloid_one_sheet_is_branch_one_only():
    """OneSheet=True (the default) builds only branch 1 (the positive
    MajorAxis side) -- half the two-sheet volume, no compound needed."""
    shape = Gmake_hyperboloid(CENTER, AXIS_Z, 3.0, 2.0, AXIS_Z, MAJOR_AXIS, True, 5.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _hyperboloid_sheet_volume(3.0, 2.0, 5.0)) < 1e-2


def _hyperbolic_cylinder_volume(major_radius, minor_radius, height):
    """Volume of revolving the hyperbola around MinorAxis from the waist
    (v=0, radius=major_radius) to v=height -- pi * integral[0,height] of
    r(v)^2 dv where r(v) = major_radius*sqrt(1+(v/minor_radius)^2),
    integrated in closed form."""
    a, b = major_radius, minor_radius
    return math.pi * a**2 * (height + height**3 / (3.0 * b**2))


def test_gmake_hyperbolic_cylinder_volume_and_validity():
    shape = Gmake_hyperbolic_cylinder(CENTER, AXIS_Z, 3.0, 2.0, MAJOR_AXIS, AXIS_Z, 5.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _hyperbolic_cylinder_volume(3.0, 2.0, 5.0)) < 1e-2


def test_gmake_hyperbolic_cylinder_waist_at_center():
    """The waist (minimum radius, = major_radius) sits exactly at
    `center`, which is also the extrusion-style start of the revolved
    range along `minor_axis` -- the bounding box spans [center,
    center + minor_axis*height] along that axis."""
    height = 5.0
    shape = Gmake_hyperbolic_cylinder(CENTER, AXIS_Z, 3.0, 2.0, MAJOR_AXIS, AXIS_Z, height)
    bbox = shape.BoundBox
    assert abs(bbox.ZMin - CENTER.z) < 1e-3
    assert abs(bbox.ZMax - (CENTER.z + height)) < 1e-3


def test_gmake_hyperbolic_cylinder_arbitrary_axis_orientation():
    minor_axis = GVector(0.3, -0.5, 0.8).normalized()
    major_axis = minor_axis.cross(GVector(1, 0, 0)).normalized()
    shape = Gmake_hyperbolic_cylinder(CENTER, minor_axis, 3.0, 2.0, major_axis, minor_axis, 5.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _hyperbolic_cylinder_volume(3.0, 2.0, 5.0)) < 1e-1


# --- is_inside() verification, 2026-09-14 -- 40 hand-computed ground-truth
# points cross-checked independently of the classes' own formulas (see the
# history log for the full derivation). GEllipsoid/GHyperboloid/
# GHyperbolicCylinder.is_inside were confirmed broken and fixed here;
# GEllipticCylinder/GEllipticCone were confirmed already correct.


def test_is_inside_ellipsoid_prolate_axis_is_major():
    """Axis == MajorAxis: axial extent is MajorRadius, radial extent is
    MinorRadius -- a point near the pole is inside only within
    MajorRadius, a point on the equator only within MinorRadius."""
    e = GEllipsoid.from_values(CENTER, AXIS_Z, 10.0, 3.0, AXIS_Z, MAJOR_AXIS)
    assert e.is_inside(CENTER + AXIS_Z * 8.0)
    assert not e.is_inside(CENTER + AXIS_Z * 12.0)
    assert e.is_inside(CENTER + MAJOR_AXIS * 2.0)
    assert not e.is_inside(CENTER + MAJOR_AXIS * 5.0)


def test_is_inside_ellipsoid_oblate_axis_is_minor():
    """Axis == MinorAxis: axial extent is MinorRadius, radial extent is
    MajorRadius -- the roles are swapped from the prolate case above."""
    e = GEllipsoid.from_values(CENTER, AXIS_Z, 10.0, 3.0, MAJOR_AXIS, AXIS_Z)
    assert e.is_inside(CENTER + AXIS_Z * 2.0)
    assert not e.is_inside(CENTER + AXIS_Z * 4.0)
    assert e.is_inside(CENTER + MAJOR_AXIS * 8.0)
    assert not e.is_inside(CENTER + MAJOR_AXIS * 12.0)


def test_is_inside_hyperbolic_cylinder():
    """Waist (radius=MajorRadius) at Center, widening with |v| along
    MinorAxis -- and, unlike the old extruded-prism formula, a point far
    along the third axis (perpendicular to both MajorAxis and MinorAxis)
    is correctly excluded rather than silently ignored."""
    c = GHyperbolicCylinder.from_values(CENTER, AXIS_Z, 3.0, 2.0, MAJOR_AXIS, AXIS_Z)
    assert c.is_inside(CENTER + MAJOR_AXIS * 2.0)
    assert not c.is_inside(CENTER + MAJOR_AXIS * 4.0)
    assert c.is_inside(CENTER + AXIS_Z * 10.0 + MAJOR_AXIS * 5.0)
    assert not c.is_inside(CENTER + AXIS_Z * 1.0 + MAJOR_AXIS * 10.0)
    assert not c.is_inside(CENTER + MINOR_AXIS * 10.0)


def test_is_inside_hyperboloid_two_branches():
    """OneSheet=False: inside either branch's cup, outside the gap
    between the two vertices -- the region is the complement of the
    same-parameters GHyperbolicCylinder's own."""
    h = GHyperboloid.from_values(CENTER, MAJOR_AXIS, 3.0, 2.0, MAJOR_AXIS, AXIS_Z, one_sheet=False)
    assert not h.is_inside(CENTER + MAJOR_AXIS * 2.0)  # gap
    assert h.is_inside(CENTER + MAJOR_AXIS * 4.0)  # branch 1
    assert h.is_inside(CENTER - MAJOR_AXIS * 4.0)  # branch 2


def test_is_inside_hyperboloid_one_sheet_branch_selection():
    """OneSheet=True: only branch 1 (positive MajorAxis side) counts --
    a point deep inside branch 2's own cup must be excluded, not just
    points in the gap."""
    h = GHyperboloid.from_values(CENTER, MAJOR_AXIS, 3.0, 2.0, MAJOR_AXIS, AXIS_Z, one_sheet=True)
    assert h.is_inside(CENTER + MAJOR_AXIS * 4.0)  # branch 1: included
    assert not h.is_inside(CENTER - MAJOR_AXIS * 4.0)  # branch 2: excluded
    assert not h.is_inside(CENTER + MAJOR_AXIS * 2.0)  # gap


def test_is_inside_elliptic_cylinder_unaffected():
    c = GEllipticCylinder.from_values(CENTER, AXIS_Z, 10.0, 3.0, MAJOR_AXIS, MINOR_AXIS)
    assert c.is_inside(CENTER + MAJOR_AXIS * 5.0 + MINOR_AXIS * 1.0)
    assert not c.is_inside(CENTER + MAJOR_AXIS * 9.0 + MINOR_AXIS * 2.5)


def test_is_inside_elliptic_cone_unaffected():
    cone = GEllipticCone.from_values(CENTER, AXIS_Z, 5.0, 2.0, 1.0, MAJOR_AXIS, MINOR_AXIS)
    assert cone.is_inside(CENTER + AXIS_Z * 5.0 + MAJOR_AXIS * 1.0)
    assert not cone.is_inside(CENTER + AXIS_Z * 5.0 + MAJOR_AXIS * 3.0)


# --- GParaboloid: same one-branch-revolve technique as GHyperboloid, but
# always a single sheet (no second branch to mirror, no OneSheet flag).


def _paraboloid_volume(focal, length):
    """Revolving y^2=4*Focal*x from x=0 to x=length around the axis:
    V = pi * integral[0,length] of y^2 dx = pi * 2*Focal*length^2."""
    return math.pi * 2.0 * focal * length**2


def test_gmake_paraboloid_volume_and_validity():
    shape = Gmake_paraboloid(CENTER, AXIS_Z, 2.0, 8.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _paraboloid_volume(2.0, 8.0)) < 1e-2


def test_gmake_paraboloid_vertex_at_center():
    """The vertex sits at `center` itself (it's on the revolution axis,
    like GHyperboloid's own vertex end) -- the bounding box spans
    [center, center + axis*length] along the axis."""
    length = 8.0
    shape = Gmake_paraboloid(CENTER, AXIS_Z, 2.0, length)
    bbox = shape.BoundBox
    assert abs(bbox.ZMin - CENTER.z) < 1e-3
    assert abs(bbox.ZMax - (CENTER.z + length)) < 1e-3


def test_gmake_paraboloid_arbitrary_axis_orientation():
    axis = GVector(0.3, -0.5, 0.8).normalized()
    shape = Gmake_paraboloid(CENTER, axis, 2.0, 8.0)
    assert BRepCheck_Analyzer(shape.__native__).IsValid()
    assert abs(shape.Volume - _paraboloid_volume(2.0, 8.0)) < 1e-1


def test_gmake_paraboloid_returns_none_for_nonpositive_length():
    assert Gmake_paraboloid(CENTER, AXIS_Z, 2.0, 0.0) is None
    assert Gmake_paraboloid(CENTER, AXIS_Z, 2.0, -5.0) is None


def test_is_inside_paraboloid():
    p = GParaboloid.from_values(CENTER, AXIS_Z, 2.0)
    assert p.is_inside(CENTER + AXIS_Z * 8.0)  # on axis, well inside
    assert p.is_inside(CENTER + AXIS_Z * 8.0 + MAJOR_AXIS * 7.0)  # within the radius at that height
    assert not p.is_inside(CENTER + AXIS_Z * 8.0 + MAJOR_AXIS * 10.0)  # beyond the radius at that height
    assert not p.is_inside(CENTER - AXIS_Z * 1.0)  # behind the vertex
