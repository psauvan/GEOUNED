import math
import logging

from geouned.GEOUNED.utils.geometry_gu import ShellFaceGu

from ..utils.basic_functions_part1 import (
    twoPimod,
    is_parallel,
    is_same_value,
)
from ..utils.basic_functions_part2 import is_same_plane
from ..utils.build_region.Objects import plane_polygon_from_box
from ..utils.geouned_classes import GeounedSurface
from ...geo import GPlane, GSphere, GBoundBox, GVector, Gmake_wire

logger = logging.getLogger("general_logger")


def gen_plane(face):
    normal = face.Surface.Axis
    if face.Orientation == "Forward":
        normal = -normal
    pos = face.CenterOfMass
    return GeounedSurface(("Plane", (pos, normal, 1, 1)))


def gen_cylinder(face):
    Axis = face.Surface.Axis
    Center = face.Surface.Center
    Radius = face.Surface.Radius
    return GeounedSurface(("CylinderOnly", (Center, Axis, Radius, 1)))


def gen_cone(face):
    Axis = face.Surface.Axis
    Apex = face.Surface.Apex
    SemiAngle = face.Surface.SemiAngle
    return GeounedSurface(("ConeOnly", (Apex, Axis, SemiAngle, 1, 1)))


def gen_sphere(face):
    Center = face.Surface.Center
    Radius = face.Surface.Radius
    return GeounedSurface(("SphereOnly", (Center, Radius)))


def gen_torus(face, tolerances):
    Center = face.Surface.Center
    Axis = face.Surface.Axis
    MajorRadius = face.Surface.MajorRadius
    MinorRadius = face.Surface.MinorRadius
    if (
        is_parallel(Axis, GVector(1, 0, 0), tolerances.angle)
        or is_parallel(Axis, GVector(0, 1, 0), tolerances.angle)
        or is_parallel(Axis, GVector(0, 0, 1), tolerances.angle)
    ):
        return GeounedSurface(("TorusOnly", (Center, Axis, MajorRadius, MinorRadius, face.Surface.a_sign)))
    else:
        return None


def cone_apex_plane(cone, tolerances):
    if (
        is_parallel(cone.Surface.Axis, GVector(1, 0, 0), tolerances.angle)
        or is_parallel(cone.Surface.Axis, GVector(0, 1, 0), tolerances.angle)
        or is_parallel(cone.Surface.Axis, GVector(0, 0, 1), tolerances.angle)
    ):
        return None

    return GeounedSurface(("Plane", (cone.Surface.Apex, cone.Surface.Axis, 1, 1)))


def check_torus_bounds(shell):

    def merge_periodic_uv(parameter, faceList):
        two_pi = 2.0 * math.pi
        if parameter == "U":
            i1 = 0
            i2 = 2
        elif parameter == "V":
            i1 = 2
            i2 = 4

        params = []
        arcLength = 0.0
        for face in faceList:
            V0, V1 = face.ParameterRange[i1:i2]
            arcLength += V1 - V0
            params.append((V0, V1))

        params.sort()
        V0 = params[0][0]
        V1 = params[-1][1]
        if arcLength >= two_pi * (1.0 - 1e-5):
            mergedParams = (True, (V0, V0 + two_pi))
        else:
            if is_same_value(V0, 0.0, 1e-5) and is_same_value(V1, two_pi, 1e-5):
                for i in range(len(params) - 1):
                    if not is_same_value(
                        params[i][1],
                        params[i + 1][0],
                        1e-5,
                    ):
                        break
                v_min = params[i + 1][0] - two_pi
                v_max = params[i][1]
            else:
                # params is sorted by V0 ascending, so params[0][0] is always
                # the true minimum V0 -- but sorting by V0 does not imply
                # sorted V1, so params[-1][1] is only the true maximum V1
                # when the pieces form a simple, non-nested chain. When one
                # piece's own range is fully nested inside another's (e.g. a
                # tiny residual sliver piece sitting within a larger piece's
                # own V-span), params[-1][1] can under-report the real
                # merged extent -- take the max explicitly instead.
                v_min = params[0][0]
                v_max = max(v1 for _, v1 in params)
            mergedParams = (False, (v_min, v_max))

        return mergedParams

    if type(shell) is ShellFaceGu:
        tFaces = shell.Faces
    else:
        tFaces = [shell]

    URange = merge_periodic_uv("U", tFaces)
    VRange = merge_periodic_uv("V", tFaces)

    return URange, VRange


def V_torus_surface(face, v_params, Surfaces):
    if is_parallel(face.Surface.Axis, GVector(1, 0, 0), Surfaces.tolerances.tor_angle):
        axis = GVector(1, 0, 0)
    elif is_parallel(face.Surface.Axis, GVector(0, 1, 0), Surfaces.tolerances.tor_angle):
        axis = GVector(0, 1, 0)
    elif is_parallel(face.Surface.Axis, GVector(0, 0, 1), Surfaces.tolerances.tor_angle):
        axis = GVector(0, 0, 1)

    torus_center = face.Surface.Center
    vmin, vmax = v_params
    if type(face) is ShellFaceGu:
        tface = face.Faces[0]
    else:
        tface = face

    p1 = tface.value_at(0.0, vmin) - torus_center
    p2 = tface.value_at(0.0, vmax) - torus_center

    z1 = p1.dot(axis)
    d1 = p1.cross(axis).length

    z2 = p2.dot(axis)
    d2 = p2.cross(axis).length

    if is_same_value(z1, z2, Surfaces.tolerances.distance):
        center = torus_center + z1 * axis
        v_mid = (v_params[0] + v_params[1]) * 0.5
        p_mid = face.value_at(0, v_mid) - torus_center
        if p_mid.dot(axis) < z1:
            axis = -axis
        return GeounedSurface(("Plane", (center, axis, 1, 1))), None

    elif is_same_value(d1, d2, Surfaces.tolerances.distance):
        radius = min(d1, d2)
        center = torus_center
        if is_same_value(d1, tface.Surface.MajorRadius, Surfaces.tolerances.distance):
            v_mid = (vmin + vmax) * 0.5

            p_mid = tface.value_at(0, v_mid) - center
            if p_mid.cross(axis).length < tface.Surface.MajorRadius:
                in_surf = True
            v_mid = (vmin + vmax) * 0.5
            p_mid = tface.value_at(0, v_mid) - center
            if p_mid.cross(axis).length < tface.Surface.MajorRadius:
                in_surf = True
                radius = max(d1, d2)
            else:
                in_surf = False

            if in_surf:
                orientation = "Forward"
            else:
                orientation = "Reversed"
        else:
            if d1 < tface.Surface.MajorRadius:
                orientation = "Forward"
                radius = max(d1, d2)
            else:
                orientation = "Reversed"
        return GeounedSurface(("CylinderOnly", (center, axis, radius, 1))), orientation
    else:
        za = (z2 * d1 - z1 * d2) / (d1 - d2)
        apex = torus_center + za * axis
        semi_angle = abs(math.atan(d1 / (z1 - za)))

        cone_axis = axis if (z1 - za) > 0.0 else -axis
        cone = GeounedSurface(("ConeOnly", (apex, cone_axis, semi_angle, 1, 1)))

        v_mid = (vmin + vmax) * 0.5
        p_mid = tface.value_at(0, v_mid) - torus_center
        z_mid = p_mid.dot(axis)
        d_mid = p_mid.cross(axis).length

        d_cone = d1 * (z_mid - za) / (z1 - za)
        in_surf = True if d_mid < d_cone else False

        if in_surf:
            orientation = "Forward"
        else:
            orientation = "Reversed"

        # apex plane not produced because torus axis along x,y,z
        return cone, orientation


def U_torus_planes(face, UParams, Surfaces):
    if is_parallel(face.Surface.Axis, GVector(1, 0, 0), Surfaces.tolerances.tor_angle):
        axis = GVector(1, 0, 0)
    elif is_parallel(face.Surface.Axis, GVector(0, 1, 0), Surfaces.tolerances.tor_angle):
        axis = GVector(0, 1, 0)
    elif is_parallel(face.Surface.Axis, GVector(0, 0, 1), Surfaces.tolerances.tor_angle):
        axis = GVector(0, 0, 1)

    umin, umax = UParams

    if type(face) is ShellFaceGu:
        p1 = face.Faces[0].value_at(umin, 0.0)
        p2 = face.Faces[0].value_at(umax, 0.0)
        pmid = face.Faces[0].value_at(0.5 * (umin + umax), 0.0)
    else:
        p1 = face.value_at(umin, 0.0)
        p2 = face.value_at(umax, 0.0)
        pmid = face.value_at(0.5 * (umin + umax), 0.0)

    center = face.Surface.Center

    angle = twoPimod(abs(umax - umin))
    if angle < math.pi + Surfaces.tolerances.value:
        d = axis.cross(p2 - p1).normalized()
        if d.dot(pmid - center) < 0:
            d = -d
        return (GeounedSurface(("Plane", (center, d, 1, 1))),)
    else:
        d1 = axis.cross(p1 - center)
        d1 = d1.normalized()
        if d1.dot(pmid - center) < 0:
            d1 = -d1

        d2 = axis.cross(p2 - center)
        d2 = d2.normalized()
        if d2.dot(pmid - center) < 0:
            d2 = -d2

        plane1 = GeounedSurface(("Plane", (center, d1, 1, 1)))
        plane2 = GeounedSurface(("Plane", (center, d2, 1, 1)))
        return (plane1, plane2)


def _unwrap_near(u, reference):
    """The representative of u's own periodic equivalence class
    (u + k*2*pi, any integer k) that lands closest to `reference` --
    avoids an artificial 0/2*pi discontinuity when sampling near the
    surface's own periodic U seam: confirmed live, 2026-08-26,
    codo.stp -- Umax sits almost exactly at 2*pi, and twoPimod's own
    unconditional wrap into [0, 2*pi) was reducing that side's own
    sampled U values down to ~0, misclassifying it as the Umin side."""
    two_pi = 2 * math.pi
    return u + two_pi * round((reference - u) / two_pi)


def _edge_u_extent(face, edge, reference, n_samples=9):
    """The U extent [u_lo, u_hi] this edge's own geometry actually
    reaches on `face`, sampled along its full length -- not just its 2
    endpoints. A torus face bounded by a plane that does NOT pass
    through the torus axis produces a boundary curve where U genuinely
    varies along the edge, including for a topologically "closed" edge
    (same start/end vertex): confirmed live, 2026-08-26,
    U_open_Fwd_3.stp -- one such edge's own endpoints sit at U=3.66, but
    it dips to U=3.41 (matching Umin exactly) at its own midpoint;
    checking only the 2 endpoints would have missed this and wrongly
    concluded the edge doesn't belong to either side.

    Each sample is unwrapped near `reference` (see _unwrap_near) instead
    of naively reduced via twoPimod, so a seam sitting inside or right
    at the edge of [Umin, Umax] doesn't fracture the edge's own extent
    across the 0/2*pi cut."""
    t0, t1 = edge.ParameterRange
    us = [
        _unwrap_near(face.parameter(edge.value_at(t0 + (t1 - t0) * k / (n_samples - 1)))[0], reference)
        for k in range(n_samples)
    ]
    return min(us), max(us)


def _classify_edge_u_side(u_lo, u_hi, Umin, Umax, region_frac=0.25):
    """ "min" if the edge's own U extent [u_lo, u_hi] reaches the Umin
    region but not the Umax region, "max" if the reverse, or None if it
    reaches BOTH regions (a genuine connector edge, bridging the two
    sides -- see below) or NEITHER (an edge of an unrelated wire, e.g.
    an interior hole, not anchored to either side at all).

    A first version compared against the midpoint of [Umin, Umax] --
    wrong per the user's own counter-example (2026-08-26,
    U_open_Rev_2.stp): the two sides' own boundary edges are not
    symmetric in how far they dip toward the middle (one reaching only
    to ~10% of the full span, the other to ~65%), so a fixed midpoint
    split misclassified the deeper-dipping real side edge as a
    connector.

    Corrected per the user's own direct description of how a connector
    edge is actually identified: a connector edge is the one whose own
    U extent runs from the Umin region all the way to the Umax region
    (touches both), which is a real, qualitatively different signature
    from a side edge (which only ever touches ONE of the two regions,
    however deep it dips toward the middle). "Region" here is the
    [Umin, Umin + region_frac*span] / [Umax - region_frac*span, Umax]
    neighborhood of each boundary -- confirmed against both known
    non-connector cases (U_open_Fwd_3.stp, codo.stp) and the
    counter-example above with region_frac=0.25."""
    span = Umax - Umin
    tol = region_frac * span
    reaches_min = (u_lo - Umin) < tol
    reaches_max = (Umax - u_hi) < tol
    if reaches_min and reaches_max:
        return None  # connector: reaches both regions
    if reaches_min:
        return "min"
    if reaches_max:
        return "max"
    return None  # reaches neither -- unrelated


def torus_u_side_edges(shell, Uparams):
    """Split a torus face/shell's own boundary into the two U-side edge
    sets (Umin-side edges, Umax-side edges) -- only meaningful when U is
    open (Uclosed=False).

    Gathers the shell's own boundary edges (real face.wires() for a
    single face; _boundary_edges_of_merged_faces for a merged
    ShellFaceGu, since an internal seam between merged pieces must not
    be treated as a real boundary) and classifies each edge
    independently by its own sampled U extent (_classify_edge_u_side) --
    no wire-level grouping needed, since an edge's own extent already
    anchors it to one side or marks it as a connector on its own."""
    Umin, Umax = Uparams

    if type(shell) is ShellFaceGu:
        # Local import: meta_surfaces_utils.py imports FROM this module
        # at module level (gen_cone/gen_cylinder/cone_apex_plane), so
        # importing from it here at module level would be circular.
        from ..utils.meta_surfaces_utils import _boundary_edges_of_merged_faces

        edges = _boundary_edges_of_merged_faces(shell.Faces)
        # any merged face works for .parameter() -- they all share the
        # same underlying analytic torus surface (that's what merging
        # means), so U/V parametrization is consistent across them
        param_face = shell.Faces[0]
    else:
        edges = [e for w in shell.wires() for e in w.Edges]
        param_face = shell

    reference = 0.5 * (Umin + Umax)
    min_edges, max_edges = [], []
    for e in edges:
        u_lo, u_hi = _edge_u_extent(param_face, e, reference)
        side = _classify_edge_u_side(u_lo, u_hi, Umin, Umax)
        if side == "min":
            min_edges.append(e)
        elif side == "max":
            max_edges.append(e)
        # else: connector edge, or an unrelated wire's edge -- ignore

    return min_edges, max_edges


def _u_sides_planar(shell, Uclosed, Vclosed, Uparams):
    """(UminSide_planar, UmaxSide_planar): whether each U-boundary
    side's own boundary edges are all INDIVIDUALLY planar (each edge on
    its own, not necessarily all sharing one common plane -- an off-axis
    plane cut can leave a side made of several distinct planar pieces
    at different orientations, see edges_individually_planar's own
    docstring), so no additional U-bounding plane needs constructing
    for it (U_torus_planes could skip that side).

    Only computed for the Uopen+Vclosed case for now (per explicit user
    scoping, 2026-08-26: Uopen+Vopen is a harder case, tackled later) --
    returns (None, None) otherwise, including for a closed-U face (the
    question doesn't apply there: no U-bounding plane is ever needed)."""
    if Uclosed or not Vclosed:
        return None, None

    # Local import: meta_surfaces_utils.py imports FROM this module at
    # module level (gen_cone/gen_cylinder/cone_apex_plane), so importing
    # from it here at module level would be circular.
    from ..utils.meta_surfaces_utils import edges_individually_planar

    min_edges, max_edges = torus_u_side_edges(shell, Uparams)
    return edges_individually_planar(min_edges), edges_individually_planar(max_edges)


def torus_face_configuration(shell, Urange, Vrange):
    """Group every classification characteristic of a torus face/shell
    into one configuration tuple -- the key used to select the correct
    boolean-expression construction for its Torus branch.

    `Urange`/`Vrange` are check_torus_bounds(shell)'s own return values
    (each a (closed, (min, max)) pair). Returns:
    (orientation, Uclosed, Vclosed, u_over_180, v_side_value,
     v_arc_class, degenerated, UminSide_planar, UmaxSide_planar)
    - orientation: "Forward" or "Reversed" (shell.Orientation)
    - Uclosed / Vclosed: whether the U / V range spans a full 2*pi loop
    - u_over_180: whether the U range spans more than pi (180 degrees)
    - v_side_value: v_side(*Vparams) -- +1 outer/convex, -1 inner/
      concave, 0 straddles neither band cleanly (see v_side's own
      docstring)
    - v_arc_class: the V arc length (Vmax - Vmin) bucketed into 3
      cases -- 1 if v_arc <= pi/2, 2 if pi/2 < v_arc <= pi, 3 if
      v_arc > pi
    - degenerated: whether the underlying torus is self-intersecting
      (MinorRadius > MajorRadius)
    - UminSide_planar / UmaxSide_planar: whether the boundary at
      U=Umin / U=Umax is already delimited exclusively by a (real)
      plane -- only computed for Uopen+Vclosed, None otherwise -- see
      _u_sides_planar's own docstring"""
    Uclosed, Uparams = Urange
    Vclosed, Vparams = Vrange
    Umin, Umax = Uparams
    Vmin, Vmax = Vparams
    u_over_180 = (Umax - Umin) > math.pi
    v_side_value = v_side(*Vparams)
    v_arc = Vmax - Vmin
    if v_arc <= math.pi / 2:
        v_arc_class = 1
    elif v_arc <= math.pi:
        v_arc_class = 2
    else:
        v_arc_class = 3
    degenerated = shell.Surface.Degenerated
    UminSide_planar, UmaxSide_planar = _u_sides_planar(shell, Uclosed, Vclosed, Uparams)
    return (
        shell.Orientation,
        Uclosed,
        Vclosed,
        u_over_180,
        v_side_value,
        v_arc_class,
        degenerated,
        UminSide_planar,
        UmaxSide_planar,
    )


def v_side(vmin, vmax):
    """Classify a torus V range by which side of the tube it lies on.

    V=0 is the tube's outer equator (farthest from the torus axis,
    rho(v) = MajorRadius + MinorRadius*cos(v) maximal); V=pi is the
    inner equator (closest to the axis, rho minimal) -- see the torus
    parametrization P(u,v) = Center + rho(v)*radial(u) + MinorRadius*
    sin(v)*Axis.

    Returns +1 if the whole [vmin, vmax] range lies within [-pi/2, pi/2]
    (mod 2*pi) -- the outer/convex side, cos(v) > 0 throughout -- -1 if
    it lies entirely within [pi/2, 3*pi/2] -- the inner/concave side,
    cos(v) < 0 throughout -- or 0 if the range straddles a boundary and
    doesn't fit cleanly in either band."""
    v0 = twoPimod(vmin)
    v1 = v0 + (vmax - vmin)  # shift by the same amount as v0, so the
    # true span (vmax - vmin) is preserved instead of being corrupted by
    # wrapping each endpoint independently

    # The outer band ([-pi/2, pi/2] mod 2*pi) straddles the v0=0 wrap
    # point once v0 is reduced into [0, 2*pi) -- check both periodic
    # copies of the band that can overlap that window.
    for offset in (0.0, 2 * math.pi):
        lo, hi = -math.pi / 2 + offset, math.pi / 2 + offset
        if lo <= v0 and v1 <= hi:
            return 1

    # The inner band ([pi/2, 3*pi/2]) never straddles the v0=0 wrap
    # point, so a single check in [0, 2*pi) is sufficient.
    if math.pi / 2 <= v0 and v1 <= 3 * math.pi / 2:
        return -1

    return 0


def one_degenerated_torus_plane(shell):
    """Build the single closing plane for a DEGENERATE (self-intersecting,
    MinorRadius > MajorRadius) torus face whose U and/or V range is
    open -- per the user's own direct confirmation, 2026-08-25: for a
    degenerate torus, whenever U and V aren't BOTH already closed,
    exactly one plane always suffices (unlike the ordinary case, which
    needs oneplane_surface's own existence check first).

    General by construction: works for any combination of open U/V,
    unlike one_torus_plane's own chord-based construction (through the
    2 U-extreme points at a single, analytically-chosen v), which
    breaks whenever U alone is already fully closed, since its own 2
    endpoints then collapse to the same physical point (Umin and
    Umax=Umin+2*pi are the same point on a closed torus) -- confirmed
    live on Solidos/../tank.stp's own degenerate torus, which hits
    exactly this Uclosed=True case.

    Per the user's own direct instruction: the plane's own normal is
    the face's own boundary contour's principal inertia axis (same
    machinery already used for a similar purpose in
    decom_utils_generator.py::spline_wires -- Gmake_wire + get_axis_inertia,
    falling back to summing each boundary edge's own individual axis if
    the boundary edges don't form one single connected wire, e.g. a
    merged shell whose own outer boundary is more than one loop). Its
    position is the contour's own extreme point in the direction AWAY
    from the face's own material -- i.e. the point a plane sweeping in
    from infinity, moving toward the face (normal pointing at the
    face's own material), would touch first. This guarantees the whole
    contour -- and, since the face's own surface is bounded by that
    contour, the whole face -- ends up on one side.
    """
    if type(shell) is ShellFaceGu:
        # deferred import -- meta_surfaces_utils.py itself imports from
        # this file at module level (gen_cone/gen_cylinder/cone_apex_plane),
        # so importing it back at module level here would be circular;
        # safe as a local import since neither module needs the other's
        # names until a real call happens, well after both are loaded.
        from ..utils.meta_surfaces_utils import _boundary_edges_of_merged_faces

        boundary_edges = _boundary_edges_of_merged_faces(shell.Faces)
    else:
        boundary_edges = shell.outer_wire().Edges

    # deferred import -- same circular-import reason as
    # _boundary_edges_of_merged_faces above (decom_utils_generator.py
    # itself imports from meta_surfaces_utils.py at module level, which
    # in turn imports from this file).
    from ..decompose.decom_utils_generator import get_axis_inertia

    try:
        wire = Gmake_wire(list(boundary_edges))
        axis = get_axis_inertia(wire.MatrixOfInertia)
    except Exception:
        axis = GVector(0, 0, 0)
        for e in boundary_edges:
            axis = axis + get_axis_inertia(e.MatrixOfInertia)
        axis = axis.normalized()

    contour_points = [v for e in boundary_edges for v in e.Vertexes]
    material = shell.CenterOfMass

    # orient axis toward the face's own material: the material's own
    # centroid must project further along axis than every contour point
    # does, i.e. the contour's own extreme (the plane's position, found
    # below) sits on the far/outside end, material on the near/inside end.
    if material.dot(axis) < max(p.dot(axis) for p in contour_points):
        axis = -axis

    position = min(contour_points, key=lambda p: p.dot(axis))

    return (GeounedSurface(("Plane", (position, axis, 1, 1))),)


def gen_plane_sphere(shell):

    if type(shell.Surface) is not GSphere:
        return None

    center = shell.Surface.Center
    if type(shell) is ShellFaceGu:
        normal = GVector(0, 0, 0)
        for f in shell.Faces:
            normal = normal + f.Area * (f.CenterOfMass - center)
    else:
        normal = shell.CenterOfMass - center

    normal = normal.normalized()
    radius = shell.Surface.Radius

    # A plane clipped to a box centered on the sphere (side = 2*radius, +1%
    # margin) is enough: same_faces are all faces of this exact sphere, so
    # every point we measure distToShape against is within `radius` of
    # sphere_center in every direction -- well inside the box. No need for
    # a true infinite Part.Plane.
    half_side = radius * 1.01
    box = GBoundBox(
        center.x - half_side,
        center.y - half_side,
        center.z - half_side,
        center.x + half_side,
        center.y + half_side,
        center.z + half_side,
    )
    tmp_plane = plane_polygon_from_box(normal, normal.dot(center), box)

    dmin = 2 * radius
    if type(shell) is ShellFaceGu:
        for f in shell.Faces:
            dist = tmp_plane.distance_to(f)
            dmin = min(dmin, dist)
    else:
        dmin = tmp_plane.distance_to(shell)

    if dmin > 1e-6:
        new_center = center + 0.95 * dmin * normal
        plane = GeounedSurface(("Plane", (new_center, normal, 1, 1)))
        return plane
    else:
        return None


def omit_multiplane_repeated_planes(mp_region, Surfaces, Faces):
    repeated_planes = set()
    planes = mp_region.region.get_surfaces_numbers()
    for p in planes:
        pg = Surfaces.primitive_surfaces.get_surface(p)
        for face in Faces:
            if not isinstance(face, GPlane):
                continue
            if is_same_plane(face.Surface, pg.Surf, Surfaces.options, Surfaces.tolerances, Surfaces.numeric_format):
                repeated_planes.add(face.Index)
    return repeated_planes
