"""
geo/occ/sewn_composite.py

Prototype (branch `sewing-composite-surface`): build a composite
meta-surface's own cutting tool by sewing its real, oversized component
faces together, instead of via the recursive boolean split cascade
(geo/solid_ops.py's BuildDepth/BuildSolidParts, used by
build_shape_functions.py::build_complex_shape for RoundCorner/Can/TCone/
MultiRoundCorner).

See geo/ocp/sewn_composite.py's own module docstring for the full design
rationale (the three simplifications this relies on, confirmed directly
by the user 2026-10-24) -- this is the same technique, ported to
pythonocc-core. Binding differences from the ocp version:
- pythonocc-core's `BRep_Tool.Curve(edge)` already returns
  `(curve, first, last)` directly, unlike OCP's `BRep_Tool.Curve_s`
  (needs two dummy float arguments and a separate `Range_s` call) -- no
  dedicated helper is needed here.
- `topods.Vertex(...)` is a plain function in pythonocc-core, not a
  static method on a `TopoDS` class (`TopoDS.Vertex(...)` under OCP).

First case: two adjacent planes of a MultiPlane composite. Second case:
RoundCorner (`Gmake_sewn_round_corner_cylinder`) -- 2 planes + 1 cylinder
face, the first composite that genuinely still pays for
`build_complex_shape`'s own `BuildDepth`/`Gfuse_solids` cascade today.

Third case: TCone (`Gmake_sewn_tcone`) -- 1 cone, closed 360 degrees, + 2
real planar closures (GEOUNED only ever detects a TCone from a cone face
with 2 real closing PLANES, never a RoundCorner variant). Genuinely
different design from RoundCorner/MultiPlane -- a TCone is already a
topologically CLOSED shell, so no oversizing of any kind is needed: the
cone's own face is trimmed precisely between the two real closing
curves' own heights, and each plane's own face is trimmed to precisely
the finite-area region its own real closing curve (reused exactly)
bounds.

TCone's own cone face is deliberately NOT built from the usual "reuse
the real edge as the boundary wire" technique, unlike every other
function here -- confirmed empirically (2026-10-25) that reusing the 2
real closing edges as one wire (closed loop + a synthetic seam, the
standard OCCT idiom for a periodic surface trimmed between 2 full
iso-curves) is fragile in a way this project's own established
`_substitute_face_surface` fix doesn't resolve. A direct parametric trim
(`BRepBuilderAPI_MakeFace(surface, 0, 2*pi, vmin, vmax)` -- confirmed
empirically to always come back `BRepCheck_Analyzer`-valid) is used
instead; `vmin`/`vmax` are found by projecting one real point of each
closing curve onto the cone's own native V parameter
(`ShapeAnalysis_Surface.ValueOfUV`) -- NOT the axial height from the
apex, since `Geom_ConicalSurface`'s own V is the SLANT distance. The
cone face's own new boundary edges are then geometrically, but not
topologically, identical to the planes' own real, reused closing curves
-- `Gmake_shell`'s own `BRepBuilderAPI_Sewing(SEW_TOLERANCE)` stitches
them together by coincidence. See geo/ocp/sewn_composite.py's own
module docstring for the full rationale.

Fourth case: Can (`Gmake_sewn_can`) -- same idea as TCone, a cylinder
instead of a cone, but a Can's own 2 closing surfaces can each be a
plane, a sphere, or a cone (a torus is rejected outright, and a
differently-radiused or non-coaxial closing cylinder falls back to
`build_complex_shape`). A sphere/cone closing surface, ASSUMED coaxial
with the main cylinder, has its real tangency circle at a single
constant latitude/slant distance, so each cap is built the same direct
parametric way as TCone's own cone body -- see geo/ocp/sewn_composite.py's
own module docstring for the full rationale.
"""

from __future__ import annotations

import math

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
from OCC.Core.Geom import Geom_Circle, Geom_ConicalSurface, Geom_CylindricalSurface, Geom_SphericalSurface
from OCC.Core.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Pln
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
from OCC.Core.TopAbs import TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods

from ..constants import UV_PROJECTION_TOL
from ..vector_geometry import GVector, arbitrary_perpendicular
from ._native_utils import to_native_vector
from .primitives import Gmake_shell
from .repair import _order_wire_edges
from .spline_quadrics import _substitute_face_surface
from .topology import GCone, GCylinder, GEdge, GFace, GPlane, GShell, GSphere


def _extended_edge_on_curve(junction_edge: GEdge, span: float) -> GEdge:
    """Rebuild an edge on `junction_edge`'s own supporting curve (point 3 of
    the design: "usa la curva que lo soporta"), re-centered on the original
    edge's own midpoint and widened to span `2 * span` -- the real edge
    found during composite detection is usually only as long as the local
    composite feature itself; the cutting tool needs its shared seam to
    reach well past the solid being cut (point 1: "mas grande que el
    solido que corta")."""
    curve, first, last = BRep_Tool.Curve(junction_edge.__native__)
    mid = (first + last) / 2.0
    new_native = BRepBuilderAPI_MakeEdge(curve, mid - span, mid + span).Edge()
    return GEdge(new_native)


def _oversized_plane_face(plane: GPlane, shared_edge: GEdge, span: float, material_hint: GVector) -> GFace:
    """Build one oversized planar face of the composite, with `shared_edge`
    (reused as-is -- the identical native edge object, not a copy) as one
    side of its boundary wire, extended `span` away from it on whichever
    side `material_hint` (any point already known to lie on this plane's
    own real, un-oversized face -- e.g. its CenterOfMass) sits. This local
    choice of side is purely a construction convenience (point 2: nothing
    here needs to know which side of the *composite* the solid being cut
    is on) -- it only decides how to extend one single face of the tool,
    using information already available from the real face that was
    already found during detection.
    """
    p_a, p_b = shared_edge.Vertexes[0], shared_edge.Vertexes[1]
    edge_dir = (p_b - p_a).normalized()
    perp = plane.Axis.cross(edge_dir).normalized()
    mid = (p_a + p_b) * 0.5
    if perp.dot(material_hint - mid) < 0:
        perp = perp * -1.0

    far_a = p_a + perp * span
    far_b = p_b + perp * span

    e_far_side = GEdge(BRepBuilderAPI_MakeEdge(to_native_vector(p_b), to_native_vector(far_b)).Edge())
    e_outer = GEdge(BRepBuilderAPI_MakeEdge(to_native_vector(far_b), to_native_vector(far_a)).Edge())
    e_back = GEdge(BRepBuilderAPI_MakeEdge(to_native_vector(far_a), to_native_vector(p_a)).Edge())

    # Edges must be added in connectivity order -- BRepBuilderAPI_MakeWire
    # only ever connects a new edge to the wire's current open end, it does
    # not defer/retry an edge added out of order (see geo's own
    # open_solid_repair.py docstring for this exact, previously-found
    # pitfall).
    wire_maker = BRepBuilderAPI_MakeWire()
    for edge in (shared_edge, e_far_side, e_outer, e_back):
        wire_maker.Add(edge.__native__)
    wire = wire_maker.Wire()

    ax3 = gp_Ax3(to_native_vector(plane.Position), gp_Dir(plane.Axis.x, plane.Axis.y, plane.Axis.z))
    native_face = BRepBuilderAPI_MakeFace(gp_Pln(ax3), wire).Face()
    return GFace(native_face)


def Gmake_sewn_plane_pair(
    plane1: GPlane,
    material_hint_1: GVector,
    plane2: GPlane,
    material_hint_2: GVector,
    junction_edge: GEdge,
    span: float,
) -> GShell:
    """Build the open, two-face cutting-tool shell for a 2-plane composite
    (the simplest MultiPlane case): both faces are oversized (bigger than
    `span`, point 1) and share one real seam built from `junction_edge`'s
    own curve (point 3), reused identically on both faces so the sewing
    step joins them exactly -- no BOPAlgo_Splitter call is made while
    building the tool itself.

    `plane1`/`plane2` are the composite's own two real planes (same
    Axis/Position as the original solid's own faces); `junction_edge` is
    the real edge already found connecting those two original faces (e.g.
    one entry of `MultiPlaneParams.Edges`); `material_hint_1`/`_2` are any
    point already known to lie on each plane's own original face (its
    CenterOfMass is enough). `span` should be at least the diagonal of the
    solid this tool will cut, so the tool is safely "bigger than the
    solid" on every side.
    """
    shared_edge = _extended_edge_on_curve(junction_edge, span)
    face1 = _oversized_plane_face(plane1, shared_edge, span, material_hint_1)
    face2 = _oversized_plane_face(plane2, shared_edge, span, material_hint_2)
    return Gmake_shell([face1, face2])


def _oversized_plane_face_two_edges(plane: GPlane, edge_prev: GEdge, edge_next: GEdge) -> GFace:
    """Build one oversized planar face of an INTERIOR link of a MultiPlane
    chain of 3+ planes -- unlike `_oversized_plane_face` (exactly 1 real
    shared edge, 3 free sides built from scratch via a material-hint-
    chosen perpendicular offset), this plane already has 2 real shared
    edges (one per neighbor, both already extended -- `_extended_edge_on_
    curve`, point 1) and needs no material hint at all: both boundary
    sides are already real and fixed, so there is no "which side is
    material" ambiguity left to resolve locally (point 2).

    The quad is closed with 2 new straight connectors, one between each
    pair of the 2 edges' own endpoints -- the pairing (which endpoint of
    `edge_prev` connects to which of `edge_next`) is whichever of the 2
    possible pairings has the smaller total connector length. This is
    correct for both real topologies a chain link's own 2 junction edges
    can have: the common case (2 roughly-parallel generatrix lines,
    offset by this plane's own real width -- its 2 "opposite sides") and
    the degenerate case (the 2 edges genuinely meet at a shared vertex,
    e.g. both planes converge to a point at one end) -- that pairing then
    gives one connector of near-zero length, collapsing the quad into a
    triangle on its own, with no special-casing needed.

    Reuses `edge_prev`/`edge_next`'s own real native vertices for the 2
    connectors (point 3's "exact shared seam" extended to vertices, not
    just edges -- the same "BRepBuilderAPI_MakeWire only bridges exact
    vertex identity, not near-coincident coordinates" pitfall
    `_top_bottom`'s own docstring already documents)."""
    a0, a1 = edge_prev.Vertexes[0], edge_prev.Vertexes[1]
    b0, b1 = edge_next.Vertexes[0], edge_next.Vertexes[1]
    a_native = _native_vertices(edge_prev)
    b_native = _native_vertices(edge_next)

    d_direct = (a0 - b0).length + (a1 - b1).length
    d_cross = (a0 - b1).length + (a1 - b0).length
    if d_direct <= d_cross:
        b_for_a1, b_for_a0 = b_native[1], b_native[0]
    else:
        b_for_a1, b_for_a0 = b_native[0], b_native[1]

    e_conn_1 = GEdge(BRepBuilderAPI_MakeEdge(a_native[1], b_for_a1).Edge())
    e_conn_2 = GEdge(BRepBuilderAPI_MakeEdge(b_for_a0, a_native[0]).Edge())

    wire_maker = BRepBuilderAPI_MakeWire()
    for edge in (edge_prev, e_conn_1, edge_next, e_conn_2):
        wire_maker.Add(edge.__native__)
    wire = wire_maker.Wire()

    ax3 = gp_Ax3(to_native_vector(plane.Position), gp_Dir(plane.Axis.x, plane.Axis.y, plane.Axis.z))
    native_face = BRepBuilderAPI_MakeFace(gp_Pln(ax3), wire).Face()
    return GFace(native_face)


def Gmake_sewn_multiplane_chain(
    planes: list[GPlane],
    material_hints: list[GVector],
    junction_edges: list[GEdge],
    span: float,
) -> GShell:
    """Build the open, N-face cutting-tool shell for a MultiPlane composite
    whose component planes form a simple OPEN CHAIN (`planes[i]` borders
    `planes[i+1]` via the real edge `junction_edges[i]`, for every
    `i` -- the by far most common MultiPlane topology: a zigzag/staircase
    of panels, each touching at most 2 neighbors). Generalizes
    `Gmake_sewn_plane_pair` (which this subsumes for `len(planes) == 2`)
    to any chain length: the 2 endpoint planes still use
    `_oversized_plane_face` (1 real edge, 3 free sides); every interior
    plane uses `_oversized_plane_face_two_edges` (2 real edges, 2 free
    sides) instead.

    Each junction edge is extended ONCE (`_extended_edge_on_curve`) and
    the SAME extended edge object is reused by both of the 2 planes it
    borders, so every seam in the sewn shell -- not just the chain's own
    endpoints -- is exact (point 3), not merely numerically coincident.

    A plane touched by more than 2 real junction edges (a branching/star
    topology) or a chain that loops back on itself (a closed ring) is
    NOT a simple chain and is not handled here -- `len(junction_edges)`
    must be exactly `len(planes) - 1`, the caller is responsible for
    only calling this on a confirmed simple open chain."""
    n = len(planes)
    if len(material_hints) != n or len(junction_edges) != n - 1:
        raise ValueError("Gmake_sewn_multiplane_chain: planes/material_hints/junction_edges size mismatch")

    extended_edges = [_extended_edge_on_curve(e, span) for e in junction_edges]

    faces = []
    for i in range(n):
        if i == 0:
            faces.append(_oversized_plane_face(planes[0], extended_edges[0], span, material_hints[0]))
        elif i == n - 1:
            faces.append(_oversized_plane_face(planes[-1], extended_edges[-1], span, material_hints[-1]))
        else:
            faces.append(_oversized_plane_face_two_edges(planes[i], extended_edges[i - 1], extended_edges[i]))
    return Gmake_shell(faces)


def _axial_height(point: GVector, axis_point: GVector, axis: GVector) -> float:
    return (point - axis_point).dot(axis)


def _native_vertices(edge: GEdge) -> list:
    """The edge's own native vertices, in the same traversal order as
    `GEdge.Vertexes` (mirrors `GEdge.__init__`'s own `TopExp_Explorer`
    loop exactly), so `_native_vertices(e)[i]` is the real vertex behind
    `e.Vertexes[i]`."""
    vs = []
    vexp = TopExp_Explorer(edge.__native__, TopAbs_VERTEX)
    while vexp.More():
        vs.append(topods.Vertex(vexp.Current()))
        vexp.Next()
    return vs


def _rebuild_edge_to_axial_range(edge: GEdge, axis_point: GVector, axis: GVector, v_bottom: float, v_top: float) -> GEdge:
    """Rebuild `edge` (one of a RoundCorner's own 2 real tangent
    generatrices) as a fresh straight edge spanning exactly `[v_bottom,
    v_top]` along `axis`, anchored at `edge`'s own (x, y) position --
    the per-edge piece of `_matched_axial_extension`'s own fix, kept as
    a standalone, module-level function so both call sites it needs
    (one per edge) stay plain, explicit calls."""
    p0 = edge.Vertexes[0]
    anchor = p0 - axis * _axial_height(p0, axis_point, axis)
    bottom_pt = anchor + axis * v_bottom
    top_pt = anchor + axis * v_top
    native = BRepBuilderAPI_MakeEdge(to_native_vector(bottom_pt), to_native_vector(top_pt)).Edge()
    return GEdge(native)


def _matched_axial_extension(edge1: GEdge, edge2: GEdge, axis_point: GVector, axis: GVector, span: float) -> tuple[GEdge, GEdge]:
    """Rebuild `edge1`/`edge2` (the RoundCorner's own 2 real tangent
    generatrices) so BOTH share the exact same extended axial (V) range,
    instead of each being independently re-centered on its own original
    midpoint (`_extended_edge_on_curve`'s own behaviour, correct for
    MultiPlane's single shared edge but not here): a real corner's 2
    tangent lines need not span the same real height (confirmed live,
    2026-10-25 -- a genuine 60mm/20mm tangent-edge pair), but the
    cylinder's own 2 cap arcs (built from each pair's "top"/"bottom"
    endpoints, see `_oversized_cylinder_face`) need both edges at the
    SAME top height and the SAME bottom height to form a geometrically
    consistent circle through both ends -- otherwise the native wire/
    face construction can fail outright (`StdFail_NotDone: BRep_API:
    command not done`, confirmed live on this exact mismatched-length
    case)."""
    heights = [_axial_height(p, axis_point, axis) for e in (edge1, edge2) for p in e.Vertexes]
    v_bottom = min(heights) - span
    v_top = max(heights) + span
    return (
        _rebuild_edge_to_axial_range(edge1, axis_point, axis, v_bottom, v_top),
        _rebuild_edge_to_axial_range(edge2, axis_point, axis, v_bottom, v_top),
    )


def _top_bottom(edge: GEdge, axis_point: GVector, axis: GVector):
    """`(top, bottom)`, each a `(point, native_vertex)` pair -- the edge's
    own 2 vertices sorted by their position along `axis` (a generatrix's 2
    extended endpoints, point 1), carrying the REAL native vertex along
    with each point so the cap arcs built from them (below) can reuse it
    directly instead of creating a new, merely-coincident one (the same
    "BRepBuilderAPI_MakeWire only bridges exact vertex identity, not
    near-coincident coordinates" pitfall already documented in this
    project's own open_solid_repair.py -- confirmed live here too: without
    this, one of the two generatrix edges came back silently REBUILT by
    the wire/face construction, defeating design point 3's own "exact
    shared seam" purpose even though the geometry still split correctly)."""
    points = edge.Vertexes
    natives = _native_vertices(edge)
    h0 = _axial_height(points[0], axis_point, axis)
    h1 = _axial_height(points[1], axis_point, axis)
    if h0 >= h1:
        return (points[0], natives[0]), (points[1], natives[1])
    return (points[1], natives[1]), (points[0], natives[0])


def _angle_around_axis(point: GVector, center: GVector, axis: GVector, e1: GVector, e2: GVector) -> float:
    """`point`'s own azimuthal angle (in `[0, 2*pi)`) around `axis`
    through `center`, measured from `e1` toward `e2` -- `e1`/`e2` are an
    orthonormal in-plane basis (perpendicular to `axis`), built once by
    `_cylinder_cap_arc` and reused for both of its own angle queries."""
    rel = point - center
    rel = rel - axis * rel.dot(axis)
    return math.atan2(rel.dot(e2), rel.dot(e1)) % (2.0 * math.pi)


def _cylinder_cap_arc(
    center: GVector,
    axis: GVector,
    radius: float,
    p1: GVector,
    v1_native,
    p2: GVector,
    v2_native,
    material_hint: GVector,
) -> GEdge:
    """One of the cylinder's own 2 "cap" edges (point 1's axial boundary,
    taking the place of a plane's own 3 "open" sides): the circular arc of
    `radius` around `axis`, through `center`, from `p1` to `p2` -- built
    directly on `v1_native`/`v2_native` (the generatrix edges' own real
    vertices, point 3's own "reuse what's already there" spirit extended
    to vertices, not just the edges themselves) -- choosing whichever of
    the circle's 2 complementary arcs passes through the angular position
    of `material_hint` -- the same local, construction-only side choice
    `_oversized_plane_face` already makes (point 2)."""
    e1 = (p1 - center)
    e1 = (e1 - axis * e1.dot(axis)).normalized()
    e2 = axis.cross(e1).normalized()

    angle_p2 = _angle_around_axis(p2, center, axis, e1, e2)
    angle_hint = _angle_around_axis(material_hint, center, axis, e1, e2)

    ax2 = gp_Ax2(to_native_vector(center), gp_Dir(axis.x, axis.y, axis.z), gp_Dir(e1.x, e1.y, e1.z))
    circle = Geom_Circle(ax2, radius)
    if 0.0 < angle_hint < angle_p2:
        native_edge = BRepBuilderAPI_MakeEdge(circle, v1_native, v2_native, 0.0, angle_p2).Edge()
    else:
        native_edge = BRepBuilderAPI_MakeEdge(circle, v2_native, v1_native, angle_p2, 2.0 * math.pi).Edge()
    return GEdge(native_edge)


def _oversized_cylinder_face(
    cylinder: GCylinder, edge1_ext: GEdge, edge2_ext: GEdge, material_hint: GVector
) -> GFace:
    """Build the composite's own oversized cylindrical face, trimmed in the
    angular (U) direction by the 2 real (already-extended) tangent edges
    -- reused as-is, same native edge objects the 2 plane faces also use,
    point 3 -- and in the axial (V) direction by however far those edges
    were already extended (point 1)."""
    (top1, top1_v), (bottom1, bottom1_v) = _top_bottom(edge1_ext, cylinder.Center, cylinder.Axis)
    (top2, top2_v), (bottom2, bottom2_v) = _top_bottom(edge2_ext, cylinder.Center, cylinder.Axis)

    top_center = cylinder.Center + cylinder.Axis * _axial_height(top1, cylinder.Center, cylinder.Axis)
    bottom_center = cylinder.Center + cylinder.Axis * _axial_height(bottom1, cylinder.Center, cylinder.Axis)

    arc_top = _cylinder_cap_arc(top_center, cylinder.Axis, cylinder.Radius, top1, top1_v, top2, top2_v, material_hint)
    arc_bottom = _cylinder_cap_arc(
        bottom_center, cylinder.Axis, cylinder.Radius, bottom1, bottom1_v, bottom2, bottom2_v, material_hint
    )

    wire_maker = BRepBuilderAPI_MakeWire()
    for edge in (edge1_ext, arc_top, edge2_ext, arc_bottom):
        wire_maker.Add(edge.__native__)
    wire = wire_maker.Wire()

    ax3 = gp_Ax3(to_native_vector(cylinder.Center), gp_Dir(cylinder.Axis.x, cylinder.Axis.y, cylinder.Axis.z))
    surface = Geom_CylindricalSurface(ax3, cylinder.Radius)
    native_face = BRepBuilderAPI_MakeFace(surface, wire).Face()
    # BRepBuilderAPI_MakeFace(surface, wire) alone comes back
    # BRepCheck-invalid (zero area) -- see geo/ocp/sewn_composite.py's own
    # comment at this exact point for the full diagnosis (the auto pcurve
    # computation fails for this mix of a reused straight generatrix edge
    # and a freshly-built arc edge; a bare `ShapeFix_Face` pass recovers a
    # valid face but silently rebuilds one of the 2 reused generatrix
    # edges, defeating design point 3). `_substitute_face_surface` is this
    # project's own already-validated recipe for exactly this situation.
    _substitute_face_surface(native_face, surface)
    return GFace(native_face)


def Gmake_sewn_round_corner_cylinder(
    plane1: GPlane,
    material_hint_1: GVector,
    edge1: GEdge,
    cylinder: GCylinder,
    cylinder_material_hint: GVector,
    edge2: GEdge,
    plane2: GPlane,
    material_hint_2: GVector,
    span: float,
) -> GShell:
    """Build the open, three-face cutting-tool shell for a RoundCorner
    composite (2 planes + 1 cylinder): no BOPAlgo_Splitter call is made
    while building the tool itself, in place of
    `build_complex_shape`'s own `BuildDepth`/`Gfuse_solids` cascade.

    `edge1`/`edge2` are the real tangent generatrix edges already found
    during RoundCorner detection (`get_adjacent_cylplane`'s own
    `touching_edge`, one per plane); `material_hint_1`/`_2`/
    `cylinder_material_hint` are any point already known to lie on each
    original, un-oversized face. `span` should be at least the diagonal
    of the solid this tool will cut.
    """
    e1_ext, e2_ext = _matched_axial_extension(edge1, edge2, cylinder.Center, cylinder.Axis, span)
    face1 = _oversized_plane_face(plane1, e1_ext, span, material_hint_1)
    face2 = _oversized_plane_face(plane2, e2_ext, span, material_hint_2)
    cyl_face = _oversized_cylinder_face(cylinder, e1_ext, e2_ext, cylinder_material_hint)
    return Gmake_shell([face1, cyl_face, face2])


def _wire_from_edges(edges: list[GEdge]):
    """A native wire built from `edges`, reused exactly (point 3) -- the
    real closed boundary between a TCone/Can's own main surface and one
    of its closing surfaces (a circle, or an ellipse for an oblique
    closing plane), possibly represented as more than one edge segment.
    `BRepBuilderAPI_MakeWire.Add()` only ever connects a new edge to the
    wire's own CURRENT open end -- for the common case (a single real
    edge already forming the whole closed loop) that's a one-element,
    already-trivially-ordered call; for 2+ segments, `_order_wire_edges`
    (already validated for exactly this failure mode elsewhere in this
    cascade, see its own docstring) reorders them into a connectivity-
    respecting sequence first.

    Returns `None` (never raises) when `edges` genuinely don't form one
    simple closed loop at all -- confirmed live, 2026-10-25,
    Cans/pipe.stp: a real candidate's own "shared edges" came back as 2
    (or 3) segments with NO matching endpoints between them whatsoever
    (a genuine detection/geometry defect on this specific candidate, not
    an ordering problem `_order_wire_edges` could ever fix) -- forcing a
    wire through disconnected edges raises a native `StdFail_NotDone`
    from `.Wire()` instead of this function's own `IsDone()` check
    catching it first. The caller (`_tcone_cap_face`/`_can_cap_face`)
    propagates `None` up to `Gmake_sewn_tcone`/`Gmake_sewn_can`, whose
    own caller (`makeTCone`/`makeCan`) falls back to
    `build_complex_shape` for this one candidate -- the same
    "sewing prerequisites not met" fallback already used for a missing
    Edge1/Edge2 or an unsupported closing-surface type, not a crash."""
    natives = [e.__native__ for e in edges]
    if len(natives) > 1:
        ordered = _order_wire_edges(natives)
        if ordered is None:
            return None
        natives = ordered
    wire_maker = BRepBuilderAPI_MakeWire()
    for native_edge in natives:
        wire_maker.Add(native_edge)
    if not wire_maker.IsDone():
        return None
    try:
        return wire_maker.Wire()
    except Exception:
        # IsDone() alone doesn't catch every disconnected-edge case --
        # confirmed live, Cans/pipe.stp: it still reported True right
        # before .Wire() itself raised StdFail_NotDone.
        return None


def _tcone_cap_face(plane: GPlane, wire) -> "GFace | None":
    """One of a TCone/Can's own planar end caps: `plane`'s own real
    (Position, Axis) analytic plane, trimmed to exactly the finite-area
    region `wire` bounds -- `BRepBuilderAPI_MakeFace(surface, wire)`'s
    own default meaning for a single closed planar wire, no material
    hint or side choice needed (unlike `_oversized_plane_face`, nothing
    here is ambiguous: there is only one bounded side to a closed curve
    on a plane). `None` if `wire` itself is `None` (see
    `_wire_from_edges`'s own docstring for why that can happen) --
    never raises."""
    if wire is None:
        return None
    xdir = plane.XDir if plane.XDir is not None else arbitrary_perpendicular(plane.Axis)
    ax3 = gp_Ax3(to_native_vector(plane.Position), gp_Dir(plane.Axis.x, plane.Axis.y, plane.Axis.z), gp_Dir(xdir.x, xdir.y, xdir.z))
    native_face = BRepBuilderAPI_MakeFace(gp_Pln(ax3), wire).Face()
    return GFace(native_face)


def _cone_native_v(cone_surface, point: GVector) -> float:
    """`point`'s own V parameter on `cone_surface` -- NOT the axial
    height from the apex, `Geom_ConicalSurface`'s own V is the SLANT
    distance (confirmed empirically, 2026-10-25: `Value(0, v)` at the
    real apex gives a point at axial height `v*cos(SemiAngle)`, not
    `v`); projecting the real point directly sidesteps ever needing that
    conversion formula."""
    uv = ShapeAnalysis_Surface(cone_surface).ValueOfUV(to_native_vector(point), 1e-6)
    return uv.Y()


def Gmake_sewn_tcone(
    cone: GCone,
    edges1: list[GEdge],
    plane1: GPlane,
    edges2: list[GEdge],
    plane2: GPlane,
) -> "GShell | None":
    """Build the open, three-face cutting-tool shell for a TCone
    composite (1 cone, closed 360 degrees, + 2 real planar closures): no
    BOPAlgo_Splitter call is made while building the tool itself, in
    place of `build_complex_shape`'s own `BuildDepth`/`Gfuse_solids`
    cascade. See this module's own docstring for why no oversizing of
    any kind is needed here, unlike every other function in this file,
    and for why the cone's own face is built via a direct parametric
    trim rather than a reused-edge wire.

    `edges1`/`edges2` are the real edges already found during TCone
    detection between the cone and each closing plane
    (`functions.py::build_tcone_params`'s own `_common_edges` call,
    stored as `TConeParams.Edge1`/`.Edge2`) -- each a closed, planar
    curve (usually a single edge; occasionally more than one segment),
    reused exactly (point 3) as each closing plane's own boundary.
    `plane1`/`plane2` are the composite's own two real closing planes.

    Returns `None` (never raises) when either closing curve's own real
    edges don't form one simple closed loop -- see `_wire_from_edges`'s
    own docstring; the caller (`build_shape_functions.py::makeTCone`)
    falls back to `build_complex_shape` for this one candidate."""
    wire1 = _wire_from_edges(edges1)
    wire2 = _wire_from_edges(edges2)
    if wire1 is None or wire2 is None:
        return None

    apex = to_native_vector(cone.Apex)
    cone_xdir = arbitrary_perpendicular(cone.Axis)
    cone_ax3 = gp_Ax3(apex, gp_Dir(cone.Axis.x, cone.Axis.y, cone.Axis.z), gp_Dir(cone_xdir.x, cone_xdir.y, cone_xdir.z))
    cone_surface = Geom_ConicalSurface(cone_ax3, cone.SemiAngle, 0.0)

    v1 = _cone_native_v(cone_surface, edges1[0].Vertexes[0])
    v2 = _cone_native_v(cone_surface, edges2[0].Vertexes[0])
    v_min, v_max = (v1, v2) if v1 <= v2 else (v2, v1)
    cone_native = BRepBuilderAPI_MakeFace(cone_surface, 0.0, 2.0 * math.pi, v_min, v_max, UV_PROJECTION_TOL).Face()
    cone_face = GFace(cone_native)

    plane1_face = _tcone_cap_face(plane1, wire1)
    plane2_face = _tcone_cap_face(plane2, wire2)

    # `cone_face`'s own 2 rim edges are geometrically, but not
    # topologically, identical to wire1/wire2 -- Gmake_shell's own
    # BRepBuilderAPI_Sewing(SEW_TOLERANCE) stitches them by coincidence.
    return Gmake_shell([plane1_face, cone_face, plane2_face])


def _cylinder_native_v(cyl_surface, point: GVector) -> float:
    """`point`'s own V (axial height) parameter on `cyl_surface` --
    unlike `Geom_ConicalSurface`, `Geom_CylindricalSurface`'s own V
    already IS the axial height directly, but `point` is still
    projected (never computed from a formula) for the same robustness
    `_cone_native_v` already established."""
    uv = ShapeAnalysis_Surface(cyl_surface).ValueOfUV(to_native_vector(point), 1e-6)
    return uv.Y()


def _sphere_cap_face(sphere: GSphere, axis: GVector, edge_point: GVector) -> GFace:
    """One of a Can's own end caps when the closing surface is a
    sphere, assumed coaxial with the main cylinder (the sphere's own
    center lies on the cylinder's axis -- the common domed-end-cap
    case): a direct parametric trim from the real tangency circle's
    own latitude (found by projecting one real point of the detected
    edge, same technique as `_cone_native_v`) to whichever pole sits on
    the same side -- mirrors `Gmake_sewn_tcone`'s own cone-face
    technique, generalized to a surface with 2 poles instead of 1 apex.

    `axis` is the main cylinder's own axis direction, reused directly
    as the sphere's own pole axis under the coaxial assumption; its
    sign does not matter here -- the near pole is picked from
    `edge_point`'s own projected sign, not assumed from `axis`'s
    orientation."""
    xdir = arbitrary_perpendicular(axis)
    ax3 = gp_Ax3(to_native_vector(sphere.Center), gp_Dir(axis.x, axis.y, axis.z), gp_Dir(xdir.x, xdir.y, xdir.z))
    sphere_surface = Geom_SphericalSurface(ax3, sphere.Radius)
    v_edge = ShapeAnalysis_Surface(sphere_surface).ValueOfUV(to_native_vector(edge_point), 1e-6).Y()
    v_min, v_max = (v_edge, math.pi / 2.0) if v_edge >= 0.0 else (-math.pi / 2.0, v_edge)
    native_face = BRepBuilderAPI_MakeFace(sphere_surface, 0.0, 2.0 * math.pi, v_min, v_max, UV_PROJECTION_TOL).Face()
    return GFace(native_face)


def _cone_cap_face(cone: GCone, edge_point: GVector) -> GFace:
    """One of a Can's own end caps when the closing surface is a cone,
    assumed coaxial with the main cylinder: the cap is the cone's own
    surface from its real apex (V=0 by construction, see
    `_cone_native_v`'s own docstring for why V is slant distance, not
    axial height) to the real tangency circle's own projected V.
    `v_edge`'s sign depends only on which way `cone.Axis` happens to
    point relative to "from apex toward the real tangency circle" (the
    real face's own analytic convention, not chosen here) -- 0 (the
    apex) is always one of the two trim bounds regardless."""
    apex = to_native_vector(cone.Apex)
    cone_xdir = arbitrary_perpendicular(cone.Axis)
    cone_ax3 = gp_Ax3(apex, gp_Dir(cone.Axis.x, cone.Axis.y, cone.Axis.z), gp_Dir(cone_xdir.x, cone_xdir.y, cone_xdir.z))
    cone_surface = Geom_ConicalSurface(cone_ax3, cone.SemiAngle, 0.0)
    v_edge = ShapeAnalysis_Surface(cone_surface).ValueOfUV(to_native_vector(edge_point), 1e-6).Y()
    v_min, v_max = (0.0, v_edge) if v_edge >= 0.0 else (v_edge, 0.0)
    native_face = BRepBuilderAPI_MakeFace(cone_surface, 0.0, 2.0 * math.pi, v_min, v_max, UV_PROJECTION_TOL).Face()
    return GFace(native_face)


def _can_cap_face(surf, axis: GVector, edges: list[GEdge]) -> "GFace | None":
    """Dispatch one of a Can's own 2 end caps by the closing surface's
    real analytic type -- a plane (reuses `_tcone_cap_face`/the real
    edges exactly, point 3), a sphere, or a cone (both via the direct
    parametric trim above, assumed coaxial with the main cylinder, no
    edge reuse needed at all -- same reasoning as the main cylindrical
    wall below). `Gmake_sewn_can`'s own caller (`build_shape_functions.
    py::makeCan`) only calls this when both of a Can's own closing
    surfaces are one of these 3 types -- a 4th (a differently-radiused
    or non-coaxial closing cylinder, whose real tangency curve is in
    general not planar/not at constant height) falls back to
    `build_complex_shape` instead, same precondition-gated pattern
    `makeRoundCorner`/`makeMultiPlanes`/`makeTCone` already use. Returns
    `None` (propagated from `_tcone_cap_face`/`_wire_from_edges`) for a
    plane cap whose own real edges don't form one simple closed loop --
    see `_wire_from_edges`'s own docstring; the sphere/cone branches
    never fail this way since they don't build a wire at all."""
    if isinstance(surf, GPlane):
        return _tcone_cap_face(surf, _wire_from_edges(edges))
    if isinstance(surf, GSphere):
        return _sphere_cap_face(surf, axis, edges[0].Vertexes[0])
    if isinstance(surf, GCone):
        return _cone_cap_face(surf, edges[0].Vertexes[0])
    raise ValueError(f"Gmake_sewn_can: unsupported closing surface type {type(surf).__name__}")


def Gmake_sewn_can(
    cylinder: GCylinder,
    edges1: list[GEdge],
    surf1,
    edges2: list[GEdge],
    surf2,
) -> "GShell | None":
    """Build the open, three-face cutting-tool shell for a Can
    composite (1 cylinder, closed 360 degrees, + 2 real closures, each
    a plane, sphere, or cone): no `BOPAlgo_Splitter` call is made while
    building the tool itself, in place of `build_complex_shape`'s own
    `BuildDepth`/`Gfuse_solids` cascade. Mirrors `Gmake_sewn_tcone`
    exactly -- see this module's own docstring for the full "why no
    oversizing, why a direct parametric trim instead of the usual
    reused-edge wire" reasoning, which applies here identically (a Can
    is already a topologically closed shell, same as a TCone).

    `edges1`/`edges2` are the real edges already found during Can
    detection between the cylinder and each closing surface
    (`functions.py::build_can_params`'s own `_common_edges` call,
    stored as `CanParams.Edge1`/`.Edge2`); `surf1`/`surf2` are the
    composite's own 2 real closing surfaces, already resolved to a
    `geo` analytic type (`GPlane`/`GSphere`/`GCone`) by the caller.

    Returns `None` (never raises) when a plane closing surface's own
    real edges don't form one simple closed loop (see
    `_wire_from_edges`'s own docstring) -- the caller
    (`build_shape_functions.py::makeCan`) falls back to
    `build_complex_shape` for this one candidate."""
    ax3 = gp_Ax3(to_native_vector(cylinder.Center), gp_Dir(cylinder.Axis.x, cylinder.Axis.y, cylinder.Axis.z))
    cyl_surface = Geom_CylindricalSurface(ax3, cylinder.Radius)

    v1 = _cylinder_native_v(cyl_surface, edges1[0].Vertexes[0])
    v2 = _cylinder_native_v(cyl_surface, edges2[0].Vertexes[0])
    v_min, v_max = (v1, v2) if v1 <= v2 else (v2, v1)
    cyl_native = BRepBuilderAPI_MakeFace(cyl_surface, 0.0, 2.0 * math.pi, v_min, v_max, UV_PROJECTION_TOL).Face()
    cyl_face = GFace(cyl_native)

    cap1 = _can_cap_face(surf1, cylinder.Axis, edges1)
    cap2 = _can_cap_face(surf2, cylinder.Axis, edges2)
    if cap1 is None or cap2 is None:
        return None

    # Same coincidence-only stitching as Gmake_sewn_tcone's own cone/
    # plane seam -- none of cyl_face/cap1/cap2's own rim edges are the
    # same native object as any other face's, for any closing surface
    # type (unlike the plane-pair/RoundCorner cases, where the shared
    # edge IS reused topologically).
    return Gmake_shell([cap1, cyl_face, cap2])
