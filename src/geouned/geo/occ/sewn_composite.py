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
There is no cone-based RoundCorner to handle here -- GEOUNED only ever
detects a RoundCorner from a cylinder face (`GEOUNED/utils/
functions.py::get_roundCorner`); a cone with 2 adjacent planes is its
own, separate composite type, TCone, not a RoundCorner variant. See
geo/ocp/sewn_composite.py's own module docstring for the full note on
what a future TCone version of this technique would need.
"""

from __future__ import annotations

import math

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
from OCC.Core.Geom import Geom_Circle, Geom_CylindricalSurface
from OCC.Core.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Pln
from OCC.Core.TopAbs import TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods

from ..vector_geometry import GVector
from ._native_utils import to_native_vector
from .primitives import Gmake_shell
from .spline_quadrics import _substitute_face_surface
from .topology import GCylinder, GEdge, GFace, GPlane, GShell


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
