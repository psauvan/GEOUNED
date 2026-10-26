"""
geo/ocp/sewn_composite.py

Prototype (branch `sewing-composite-surface`): build a composite
meta-surface's own cutting tool by sewing its real, oversized component
faces together, instead of via the recursive boolean split cascade
(geo/solid_ops.py's BuildDepth/BuildSolidParts, used by
build_shape_functions.py::build_complex_shape for RoundCorner/Can/TCone/
MultiRoundCorner).

Three simplifications make this tractable, confirmed directly by the user
(2026-10-24) and encoded here:

1. The tool does not need to be a closed, valid solid -- an open shell,
   larger than the real solid being cut, is enough (BOPAlgo_Splitter
   already accepts a bare face as a cutting tool for every basic surface
   today; a multi-face shell is a direct generalization of that, not a
   new kind of input).
2. Splitting the real solid with this tool does not need to know which
   side of the composite the solid's own material is on -- that
   classification happens afterward, the same way it already does for
   every other candidate surface in `generic_split`.
3. The specific edge(s) connecting the composite's component faces are
   already found during composite detection (e.g. `MultiPlaneParams.Edges`,
   `get_adjacent_cylplane`'s own `touching_edge`) -- this module reuses
   that edge's own supporting curve to build the shared seam, instead of
   re-deriving a tangency/junction line analytically.

First case implemented: two adjacent planes of a MultiPlane composite --
the simplest one, with no curved-surface tangency math involved. See
CLAUDE.md's "sewing composite surface" study for why MultiPlane was
chosen first even though its own production path (`makeMultiPlanes`)
does not currently use the boolean cascade either -- it validated the
sew-with-a-real-edge mechanism in isolation before risking it on
RoundCorner.

Second case: RoundCorner (`Gmake_sewn_round_corner_cylinder`) -- 2 planes
+ 1 cylinder face, the first composite that genuinely still pays for
`build_complex_shape`'s own `BuildDepth`/`Gfuse_solids` cascade today.
Both of the cylinder's own tangent lines with the 2 planes are straight
generatrices, so `_extended_edge_on_curve`/`_oversized_plane_face` are
reused unchanged for the 2 plane faces; the cylinder's own oversized face
is new -- bounded in the angular (U) direction by the 2 real tangent
edges (extended, point 1), and in the axial (V) direction by `span`, via
2 new circular-arc "cap" edges built on the cylinder's own real
Center/Axis/Radius.

Third case: TCone (`Gmake_sewn_tcone`) -- 1 cone, closed 360 degrees,
+ 2 real planar closures (`GEOUNED/utils/functions.py::get_tcone_surfaces`
always requires both closing surfaces to be `GPlane`; a cone with 2
adjacent planes is its own composite type, never a RoundCorner variant,
since `get_roundCorner` only ever detects one from a `GCylinder` face).
Genuinely different design from RoundCorner/MultiPlane, confirmed
directly by the user (2026-10-25): a TCone is already a topologically
CLOSED shell (a cone, closed 360 degrees, with 2 planar end caps), so
none of points 1/2's own "oversized, bigger-than-the-solid, free sides"
reasoning applies here at all -- the cone's own face is trimmed
precisely between the two real closing curves' own heights, and each
plane's own face is trimmed to precisely the finite-area region its own
real closing curve (reused exactly, point 3) bounds -- both are already
the composite's true, complete boundary. No `span`/material-hint
parameter is needed at all, unlike every other function in this module.

TCone's own cone face is deliberately NOT built from point 3's usual
"reuse the real edge as the boundary wire" technique, unlike every other
function here -- confirmed empirically (2026-10-25) that reusing the 2
real closing edges as ONE wire (closed loop + a synthetic seam, the
standard OCCT idiom for a periodic surface trimmed between 2 full
iso-curves) is fragile in a way this project's own established
`_substitute_face_surface` fix doesn't resolve: the 2 edges' real
relative orientation (as found in an actual solid) is not a fixed
convention, and even once a seam/orientation combination passes
`BRepCheck_Analyzer`, the resulting face can still make BOPAlgo_Splitter
produce a wrong (volume-inconsistent) split. The direct parametric trim
(`BRepBuilderAPI_MakeFace(surface, 0, 2*pi, vmin, vmax)` -- no wire, no
seam, confirmed empirically to always come back `BRepCheck_Analyzer`-
valid) is used instead; `vmin`/`vmax` are found by projecting one real
point of each closing curve onto the cone's own native V parameter
(`ShapeAnalysis_Surface.ValueOfUV`) -- NOT the axial height from the
apex, since `Geom_ConicalSurface`'s own V is the SLANT distance
(confirmed empirically: `Value(0, v)` at the real apex gives a point at
axial height `v*cos(SemiAngle)`, not `v`). The cone face's own new
boundary edges are then geometrically, but not topologically, identical
to `wire1`/`wire2` (the planes' own real, reused closing curves) --
`Gmake_shell`'s own `BRepBuilderAPI_Sewing(SEW_TOLERANCE)` stitches them
together by coincidence, exactly as it would for any other 2 faces built
independently.

Fourth case: Can (`Gmake_sewn_can`) -- the same idea as TCone (a cylinder
instead of a cone, always closed 360 degrees), but a Can's own 2 closing
surfaces can each be a plane, a sphere, a cone, or a cylinder
(`GEOUNED/utils/functions.py::get_can_surfaces` accepts any of these --
a torus is rejected outright). First implemented (2026-10-25) with a
coaxial-only assumption for a sphere/cone closing surface (a direct
parametric `(u, v)` trim, no wire, no `_substitute_face_surface` fix-up
needed) and a separate, `Gcommon`-based technique for a differently-
radiused or non-coaxial closing cylinder/cone -- both superseded
2026-10-26, see below.

**Design (2026-10-26), per direct user instruction to always use this
one technique, for every face of the Can, regardless of each closing
surface's own type or whether it's coaxial with the main cylinder**:
the main cylinder's own boundary is the two tangency curves
(`edges1`/`edges2`, already found during detection, each the real
curve shared between the cylinder and one closing surface) joined into
ONE wire by a synthetic seam (`_join_boundary_wires`) between their own
first vertices, in both directions -- the standard OCCT idiom for a
periodic surface trimmed between 2 full loops (a face/shell needs
exactly one outer wire). Each cap's own boundary is instead its
closing face's own COMPLETE real wire (`face_edges1`/`face_edges2`,
the real closing face's own full edge list, not just the tangency
curve it shares with the cylinder) -- confirmed directly by the user,
2026-10-26: a closing surface's real face in the solid already encodes
its own true extent exactly (a single tangency circle plus a genuine
degenerate pole/apex for a true dome/cone-to-a-point, two real
circular edges for a frustum or a differently-radiused closing
cylinder, or anything else OCCT already built correctly there) --
reusing it directly is simpler and more robust than reconstructing a
pole/apex boundary analytically, which an earlier revision of this
module tried first (`_pole_cap_face`/`_degenerate_pole_wire`, now
removed -- see git history): that technique worked for a sphere but
left a cone's own degenerate apex edge with an unexplained pcurve
period shift that was never isolated (the same `ShapeFix_Face`
state-not-reflected-in-its-own-status mystery this project had already
hit and given up on isolating from Python for a cone surface
specifically, see the "Spline-vs-quadric identification" entry in
CLAUDE.md). Reusing the real face's own boundary sidesteps that
mystery entirely, for sphere AND cone (and any frustum/cylinder
closing surface) uniformly -- no pole/apex reconstruction of any kind
is needed any more.

`_native_surface` builds each closing surface's own real `Geom_*`
surface from its bare analytic parameters; `BRepBuilderAPI_MakeFace(surface, wire)` can pick either
side of a closed wire depending on the wire's own parametric sense --
`_sign_corrected_face` checks the resulting face's own signed area via
`BRepGProp.SurfaceProperties_s` (negative when the wrong side was
picked) and rebuilds with the wire reversed (`.Complemented()` on the
face and negating the surface's own axis were both tried and confirmed
NOT to fix it). Found and validated via an isolated probe script
against the real motivating fixture, `Cans/barrel_left.stp` (a R=395
bore through a R=1936.8/1879.6 barrel wall, axis in the XY plane, NOT
coaxial with either barrel cylinder) -- the exact case neither of the
2026-10-25 techniques could handle at all -- before being ported into
production: a direct joined-wire attempt on the main cylinder first
failed with 2 separate closed loops added to one `MakeFace` call (area
0 before substitution, wrong/incomplete area after -- the same "2 full
loops" fragility TCone's own docstring already documents for its cone
body); the synthetic-seam join fixed that. The real orientation bug
(`BRepBuilderAPI_MakeFace` silently choosing the wrong side for a
single-edge cap wire) was then found and fixed via direct, repeated
empirical testing, not derived analytically -- confirmed live that
`wire.Reversed()` under OCP returns a bare `TopoDS_Shape`, not
`TopoDS_Wire`, so it must be explicitly downcast
(`TopoDS.Wire(wire.Reversed())`) before `BRepBuilderAPI_MakeFace` will
accept it; the un-downcast call raises a plain Python `TypeError`,
easily mistaken for a native crash if swallowed by an outer
`try`/`except` (see `geo.STRICT_EXCEPTIONS`, `STRICT_LEVEL_CANDIDATE`,
for exactly this situation).

Re-verified end to end against the real fixture itself (not just the
synthetic cylinder-only test above), 2026-10-26: `Cans/barrel_left.stp`
converts through `decompose_solids()` + `build_solid_definition()`
with BOTH of its Can candidates taking the sewn path (0 fallbacks),
decomposed-piece volume conserved to `4.0e-8` relative against the
true CAD volume -- the plane/cylinder half of this module's own design
(`_sign_corrected_face`, no pole involved at all, since this fixture's
own closing surfaces are both cylinders) is confirmed correct on the
real motivating case, not only on synthetic fixtures.

**Sphere and cone, same recipe as plane/cylinder, generalized
2026-10-26**: a first revision of this module handled a sphere or cone
closing surface by analytically reconstructing a pole/apex boundary
(`_pole_cap_face`/`_degenerate_pole_wire`, since removed) -- this
worked for a sphere (`Gsplit` against a domed-can tool gave the exact
right split) but left a cone's own degenerate apex edge with an
unexplained pcurve period shift that was never isolated, so the cone
case was deliberately left on the `build_complex_shape` fallback.
Reusing the closing face's own real, already-correct boundary wire
(`face_edges1`/`face_edges2`, see `Gmake_sewn_can`'s own docstring)
instead of reconstructing one needs no pole/apex handling at all --
whatever OCCT already built into the real face (a degenerate
pole/apex edge included, when the real face genuinely narrows to a
point) comes along for free -- so sphere and cone are now both built
through the exact same `_sign_corrected_face` call as plane/cylinder,
with no special-casing by closing-surface type anywhere in this
module."""

from __future__ import annotations

import math

from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
)
from OCP.Geom import Geom_Circle, Geom_ConicalSurface, Geom_CylindricalSurface, Geom_Plane, Geom_SphericalSurface
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Pln, gp_Pnt2d, gp_Vec2d
from OCP.ShapeAnalysis import ShapeAnalysis_Surface
from OCP.TopAbs import TopAbs_EDGE, TopAbs_REVERSED, TopAbs_VERTEX, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from ..constants import NUMERIC_TOL, UV_PROJECTION_TOL
from ..vector_geometry import GVector, arbitrary_perpendicular
from ._native_utils import to_native_vector, _edge_curve_and_range, _surface_props
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
    curve, first, last = _edge_curve_and_range(junction_edge.__native__)
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
        vs.append(TopoDS.Vertex(vexp.Current()))
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
    # BRepCheck-invalid (zero area) -- confirmed via direct diagnostic
    # that the computed (u, v) trim itself is exactly right (checked via
    # ShapeAnalysis_Surface.ValueOfUV on all 4 corners), so the auto
    # pcurve computation this constructor does internally is what's
    # failing for this mix of a reused straight generatrix edge and a
    # freshly-built arc edge. A first fix via a bare `ShapeFix_Face` pass
    # DID recover a valid, correct-area face, but it also silently
    # REBUILT one of the 2 reused generatrix edges (confirmed live: the
    # fixed cylinder face no longer passed `is_same` against the plane
    # face's own copy of that edge) -- defeating design point 3's own
    # purpose (an exact shared seam, not a numerically-close one).
    # `_substitute_face_surface` is this project's own already-validated
    # recipe for exactly this situation (reusing an edge with a real 3D
    # curve on a surface it doesn't have a pcurve for yet): attach a
    # correct pcurve to every edge via `_project_curve_2d`, in place,
    # without rebuilding the edges themselves.
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


def Gmake_sewn_multi_round_corner(
    cylinders: "list[tuple[GCylinder, GEdge, GEdge, GVector]]",
    shared_planes: "list[tuple[GPlane, GEdge, GEdge]]",
    open_planes: "list[tuple[GPlane, GEdge, GVector]]",
    span: float,
) -> GShell:
    """Build the open cutting-tool shell for a MultiRoundCorner
    composite (N chained RoundCorners -- an open chain or a closed
    ring, sharing a junction plane between each pair of neighbors):
    generalizes `Gmake_sewn_round_corner_cylinder` from 1 corner to N,
    reusing the exact same per-face recipe (an oversized plane for
    every real, un-shared end plane; an oversized plane for every
    shared junction, built from BOTH neighbors' own real tangent
    edges; an oversized cylinder per corner) -- no BOPAlgo_Splitter
    call is made while building the tool itself, in place of
    `build_complex_shape`'s own `BuildDepth`/`Gfuse_solids` cascade.

    `cylinders[i]` is `(cylinder, edge1, edge2, cylinder_hint)` for the
    i-th corner, exactly the per-corner inputs
    `Gmake_sewn_round_corner_cylinder` already takes; `edge1`/`edge2`
    are the real tangent generatrices found during RoundCorner
    detection (`RoundCornerParams.Edge1`/`.Edge2`). `shared_planes` is
    `(plane, edge_a, edge_b)` for a plane bordering exactly 2
    neighboring corners -- `edge_a`/`edge_b` must be the SAME edge
    objects passed as one of `cylinders`' own `edge1`/`edge2` entries
    (point 3: this function extends each cylinder's own edges exactly
    once via `_matched_axial_extension`, keyed by object identity, and
    reuses that SAME extended copy wherever the same real edge appears
    again here or in `open_planes`, so every seam is exact, not merely
    numerically coincident). `open_planes` is `(plane, edge,
    material_hint)` for a plane bordering only 1 corner (an open
    chain's own 2 loose ends -- never populated for a closed ring).

    The caller (`build_shape_functions.py::makeMultiRoundCorner`) is
    responsible for grouping each corner's own 2 real planes into
    `shared_planes`/`open_planes` by geometric identity (a plane
    touched by more than 2 corners is not a simple chain/ring and
    should be rejected there, before calling this function at all --
    mirrors `Gmake_sewn_multiplane_chain`'s own identical caveat)."""
    extended: dict[int, GEdge] = {}
    cyl_faces = []
    for cylinder, edge1, edge2, hint in cylinders:
        e1_ext, e2_ext = _matched_axial_extension(edge1, edge2, cylinder.Center, cylinder.Axis, span)
        extended[id(edge1)] = e1_ext
        extended[id(edge2)] = e2_ext
        cyl_faces.append(_oversized_cylinder_face(cylinder, e1_ext, e2_ext, hint))

    plane_faces = []
    for plane, edge_a, edge_b in shared_planes:
        plane_faces.append(_oversized_plane_face_two_edges(plane, extended[id(edge_a)], extended[id(edge_b)]))
    for plane, edge, hint in open_planes:
        plane_faces.append(_oversized_plane_face(plane, extended[id(edge)], span, hint))

    return Gmake_shell(cyl_faces + plane_faces)


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
    trim rather than point 3's usual reused-edge wire.

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


def _first_vertex_of_wire(wire):
    """The first vertex a plain `TopAbs_VERTEX` explorer reaches on
    `wire` -- used as the anchor point for a synthetic seam joining two
    closed boundary loops into one wire (`_join_boundary_wires`); which
    of a closed loop's own vertices counts as "first" is arbitrary, but
    it only has to be used consistently on both sides of the seam,
    which it is (each wire is explored exactly once)."""
    vexp = TopExp_Explorer(wire, TopAbs_VERTEX)
    return TopoDS.Vertex(vexp.Current())


def _edges_of_wire(wire) -> list:
    """The native edges of `wire`, in topological traversal order --
    used by `_join_boundary_wires` to feed `wire_a`/`wire_b` into the
    joined wire one edge at a time, rather than passing the whole
    `TopoDS_Wire` object to a single `BRepBuilderAPI_MakeWire.Add()`
    call. Confirmed empirically, 2026-10-26, a real TCone/Can tangency
    curve split into 2+ edge segments (e.g. a cone's own tangency
    circle with the main cylinder, represented as 2 half-arcs): passing
    the already-closed multi-edge wire directly to `.Add()` silently
    drops every one of its edges from the result (no exception, no
    `IsDone()` failure -- the resulting face just ends up missing that
    whole boundary loop) -- exploding it into its own edges first and
    adding each individually (exactly how `_wire_from_edges` already
    builds `wire_a`/`wire_b` themselves) avoids this entirely."""
    edges = []
    eexp = TopExp_Explorer(wire, TopAbs_EDGE)
    while eexp.More():
        edges.append(TopoDS.Edge(eexp.Current()))
        eexp.Next()
    return edges


def _join_boundary_wires(wire_a, wire_b) -> "object | None":
    """Join two closed boundary loops (`wire_a`/`wire_b`, each a full
    loop already lying exactly on the shell's own periodic main
    surface -- the real curve between that surface and one of its 2
    closing surfaces) into ONE wire, via a synthetic seam between their
    own first vertices, in both directions -- a face/shell needs
    exactly one outer wire, and a periodic surface trimmed between 2
    full loops needs this standard OCCT idiom. The seam is a straight
    edge between the two loops' own corresponding first vertices --
    valid because both loops are boundaries of the SAME periodic
    surface, so the straight segment between them is itself one of
    that surface's own generatrices (confirmed empirically, 2026-10-26,
    Cans/barrel_left.stp -- this technique is used for the main
    cylinder/cone body of EVERY composite built this way now, not only
    the general non-coaxial case it was first found for).

    Returns `None` (never raises) if the wire assembly fails (defensive
    -- a real pair of boundary loops on a shared surface always joins
    cleanly)."""
    va = _first_vertex_of_wire(wire_a)
    vb = _first_vertex_of_wire(wire_b)
    seam_out = BRepBuilderAPI_MakeEdge(va, vb).Edge()
    seam_back = BRepBuilderAPI_MakeEdge(vb, va).Edge()
    wire_maker = BRepBuilderAPI_MakeWire()
    for edge in _edges_of_wire(wire_a):
        wire_maker.Add(edge)
    wire_maker.Add(seam_out)
    for edge in _edges_of_wire(wire_b):
        wire_maker.Add(edge)
    wire_maker.Add(seam_back)
    if not wire_maker.IsDone():
        return None
    try:
        return wire_maker.Wire()
    except Exception:
        return None


def _fix_periodic_pcurve_continuity(face, surface) -> None:
    """After `_substitute_face_surface` rebuilds a pcurve for every edge
    of `face` INDEPENDENTLY, an edge whose own first sampled 3D point
    sits exactly on a periodic surface's own seam (ambiguous between 2
    equally-valid period branches -- e.g. a full tangency CIRCLE edge,
    reused whole as one side of a joined wire, point 3) can land on a
    DIFFERENT principal branch than its neighbors in the same wire --
    `_project_curve_2d`'s own per-sample unwrap only keeps ONE edge's
    own walk internally consistent, nothing enforces agreement BETWEEN
    independently-sourced edges of the same wire.

    Confirmed empirically, 2026-10-26, `Gmake_sewn_can`'s own main
    cylinder/cone body (`_join_boundary_wires`, joining a tangency
    circle with 2 synthetic seams and the OTHER closing surface's own
    tangency curve into one wire): on a real fixture whose cone
    tangency curve is split into 2 half-arcs, the plane's own full
    tangency circle (a single edge, used once, starting exactly at
    u=0=2*pi) came back with its WHOLE pcurve shifted by one full
    U period relative to every other edge of the same wire -- a real
    (u, v) discontinuity of exactly one period at the shared vertex,
    correctly flagged `BRepCheck_Analyzer`-invalid even though the
    face's own overall area/sign (and therefore `_sign_corrected_face`'s
    own check) was already right.

    Walks each wire of `face` in its own native build order (the same
    order `BRepBuilderAPI_MakeWire.Add()` calls inserted the edges,
    which IS the wire's geometric traversal order here), tracking the
    running (u, v) point the chain has reached so far -- using each
    edge's own `Orientation()` to know which end of its pcurve is the
    "entry" point in that order, not a distance-based guess -- and
    shifts any edge's pcurve by a whole multiple of the surface's own
    U/V period whenever its own entry point doesn't already match the
    running point within half a period. A no-op for the overwhelming
    majority of faces, where every edge's own independently-sampled
    pcurve already agrees with its neighbors (confirmed: this function
    never fires on any of the existing, already-validated plane/
    cylinder/sphere fixtures for this module, only on the specific
    real-arc-split case above)."""
    u_period = surface.UPeriod() if surface.IsUPeriodic() else None
    v_period = surface.VPeriod() if surface.IsVPeriodic() else None
    if u_period is None and v_period is None:
        return

    def _shift(period, delta):
        if period is None or abs(delta) <= period / 2.0:
            return 0.0
        return -period * round(delta / period)

    builder = BRep_Builder()
    wexp = TopExp_Explorer(face, TopAbs_WIRE)
    while wexp.More():
        wire = TopoDS.Wire(wexp.Current())
        running_end = None
        eexp = TopExp_Explorer(wire, TopAbs_EDGE)
        while eexp.More():
            edge = TopoDS.Edge(eexp.Current())
            if not BRep_Tool.Degenerated_s(edge):
                first, last = BRep_Tool.Range_s(edge, face)
                pcurve = BRep_Tool.CurveOnSurface_s(edge, face, first, last)
                reversed_ = edge.Orientation() == TopAbs_REVERSED
                p_entry = pcurve.Value(last if reversed_ else first)
                p_exit = pcurve.Value(first if reversed_ else last)
                if running_end is not None:
                    du = _shift(u_period, p_entry.X() - running_end[0])
                    dv = _shift(v_period, p_entry.Y() - running_end[1])
                    if du != 0.0 or dv != 0.0:
                        shifted = pcurve.Translated(gp_Vec2d(du, dv))
                        tol = BRep_Tool.Tolerance_s(edge)
                        builder.UpdateEdge(edge, shifted, surface, face.Location(), tol)
                        p_exit = gp_Pnt2d(p_exit.X() + du, p_exit.Y() + dv)
                running_end = (p_exit.X(), p_exit.Y())
            eexp.Next()
        wexp.Next()


def _sign_corrected_face(surface, wire) -> GFace:
    """Build a face trimmed by `wire` on `surface`, correcting its
    orientation if `BRepBuilderAPI_MakeFace(surface, wire)` picked the
    wrong side of the closed wire -- which side it picks depends on the
    wire's own parametric sense, not controlled by this function's
    callers (the wire's edges are reused from elsewhere, point 3).
    Confirmed empirically (2026-10-26, Cans/barrel_left.stp): checking
    the face's own signed area via `BRepGProp.SurfaceProperties_s`
    (negative when the wrong side was picked) and rebuilding with the
    wire reversed is the only fix that works -- `.Complemented()` on the
    face and negating the surface's own axis were both tried and
    confirmed NOT to fix it. Also fixes any periodic pcurve
    discontinuity left between independently-sourced edges of the same
    wire -- see `_fix_periodic_pcurve_continuity`'s own docstring."""

    def _build(w):
        native_face = BRepBuilderAPI_MakeFace(surface, w).Face()
        _substitute_face_surface(native_face, surface)
        _fix_periodic_pcurve_continuity(native_face, surface)
        return native_face

    native_face = _build(wire)
    if _surface_props(native_face).Mass() < 0.0:
        native_face = _build(TopoDS.Wire(wire.Reversed()))
    return GFace(native_face)


def _native_surface(surf):
    """Build the real `Geom_*` surface for any of the 4 closing-surface
    types a Can can have, from its own bare analytic parameters --
    used uniformly now that every cap is built by the same real-edge
    wire + sign-correction technique (point 3), regardless of the
    closing surface's own type or whether it is coaxial with the main
    cylinder (2026-10-26, supersedes the earlier coaxial-only
    parametric-trim + `Gcommon`-based general-case split). A sphere has
    no intrinsic axis of its own, so any orthonormal frame works --
    its choice has no effect on the sphere's own geometry, only on an
    internal (u, v) parametrization this function's own caller never
    reads directly (trimming is done by the real 3D wire, not a (u, v)
    box)."""
    if isinstance(surf, GPlane):
        xdir = surf.XDir if surf.XDir is not None else arbitrary_perpendicular(surf.Axis)
        ax3 = gp_Ax3(to_native_vector(surf.Position), gp_Dir(surf.Axis.x, surf.Axis.y, surf.Axis.z), gp_Dir(xdir.x, xdir.y, xdir.z))
        return Geom_Plane(ax3)
    if isinstance(surf, GSphere):
        return Geom_SphericalSurface(gp_Ax3(to_native_vector(surf.Center), gp_Dir(0, 0, 1)), surf.Radius)
    if isinstance(surf, GCone):
        cone_xdir = arbitrary_perpendicular(surf.Axis)
        ax3 = gp_Ax3(to_native_vector(surf.Apex), gp_Dir(surf.Axis.x, surf.Axis.y, surf.Axis.z), gp_Dir(cone_xdir.x, cone_xdir.y, cone_xdir.z))
        return Geom_ConicalSurface(ax3, surf.SemiAngle, 0.0)
    if isinstance(surf, GCylinder):
        ax3 = gp_Ax3(to_native_vector(surf.Center), gp_Dir(surf.Axis.x, surf.Axis.y, surf.Axis.z))
        return Geom_CylindricalSurface(ax3, surf.Radius)
    raise ValueError(f"_native_surface: unsupported closing surface type {type(surf).__name__}")


def _v_extent_of_boundary(native_surface, edges: list[GEdge]) -> "tuple[float, float] | None":
    """`(v_min, v_max)` spanned by `edges` on `native_surface`, or
    `None` if `native_surface` isn't periodic in U or `edges` don't
    each sit "flat" at one constant V -- i.e. the boundary decomposes
    entirely into V-slices of a periodic surface: a full circle (or
    several arc pieces of one, e.g. a cone's tangency split into 2
    half-arcs), each at its own fixed V, or a degenerate point (a
    pole/apex, every one of its own sampled points collapsing to the
    same V by construction). This is the general pattern behind BOTH a
    Can's own main cylinder/cone body (2 closing curves) and a sphere/
    cone cap (a tangency circle + a possible degenerate pole) --
    `None` for a genuinely 3D, non-coaxial curve that varies in both U
    and V (e.g. `Cans/barrel_left.stp`'s own non-coaxial closing
    cylinders), or for a non-periodic surface (`GPlane`).

    Confirmed empirically, 2026-10-26, this V-only check is itself
    immune to the "wrong U branch" ambiguity that makes a reused
    self-looping edge's own U value unreliable (see
    `_flat_periodic_trim`'s own docstring for why that ambiguity is
    unavoidable for a wire-based reconstruction): the branch choice
    only ever affects U, never V, so sampling V alone is safe even on
    an edge whose own independently-rebuilt U branch disagrees with
    its neighbors.

    Not every edge of a real closing face needs to be flat itself: a
    true dome/cone-to-a-point's own real topology (confirmed live,
    2026-10-26, a real sphere cap) also carries 2 MERIDIAN edges
    connecting the tangency circle to the pole -- genuinely varying in
    V along their own length, by design, not a defect. These are
    skipped for the purpose of finding `v_min`/`v_max` (only the FLAT
    edges -- circles/arcs and degenerate points -- contribute a V
    value), but their own sampled V range must stay within whatever
    `v_min`/`v_max` the flat edges already establish -- a genuinely
    out-of-range non-flat edge (the real, non-coaxial case) still
    returns `None`."""
    if not native_surface.IsUPeriodic():
        return None
    sa = ShapeAnalysis_Surface(native_surface)
    flat_values = []
    other_ranges = []
    for edge in edges:
        native = edge.__native__
        if BRep_Tool.Degenerated_s(native):
            flat_values.append(sa.ValueOfUV(to_native_vector(edge.Vertexes[0]), UV_PROJECTION_TOL).Y())
            continue
        curve, first, last = _edge_curve_and_range(native)
        samples = [sa.ValueOfUV(curve.Value(first + (last - first) * f), UV_PROJECTION_TOL).Y() for f in (0.0, 0.5, 1.0)]
        if max(samples) - min(samples) <= NUMERIC_TOL:
            flat_values.append(sum(samples) / len(samples))
        else:
            other_ranges.append((min(samples), max(samples)))
    if not flat_values:
        return None
    v_min, v_max = min(flat_values), max(flat_values)
    for lo, hi in other_ranges:
        if lo < v_min - NUMERIC_TOL or hi > v_max + NUMERIC_TOL:
            return None
    return v_min, v_max


def _flat_periodic_trim(native_surface, edges: list[GEdge]) -> "GFace | None":
    """Build a face on `native_surface` by a direct parametric trim
    (`BRepBuilderAPI_MakeFace(surface, 0, 2*pi, v_min, v_max)`, no
    wire, no reused edges at all for this one face) whenever `edges`'
    own real boundary decomposes entirely into V-slices
    (`_v_extent_of_boundary`) -- `None` otherwise (the general,
    non-coaxial case, or a non-periodic surface), for the caller to
    fall back to the wire-based `_wire_from_edges`/`_sign_corrected_
    face` reconstruction instead.

    The exact same technique `Gmake_sewn_tcone`'s own cone body
    already uses (no wire, no seam, see its own docstring), extended
    here to BOTH a Can's main cylinder/cone body (2 closing curves)
    AND its own sphere/cone caps (previously reconstructed via a
    wire). Confirmed empirically, 2026-10-26, this is NECESSARY, not
    just a simplification, for any of these faces whose real boundary
    includes a single full-360-degree edge (whether that's one of the
    2 closing curves on the main body, or a cap's own tangency circle
    alongside a degenerate pole): joining such curves into one wire
    (`_join_boundary_wires`) or trimming a surface by one directly
    (`_sign_corrected_face`) is mathematically unable to represent a
    true 360-degree boundary without a literal one-period
    discontinuity at the seam, since a single self-looping edge
    cannot hold 2 different (u, v) values at its own single vertex to
    satisfy both of its neighbors in the wire at once -- confirmed by
    direct numeric derivation, and that OCCT itself actively prevents
    building 2 distinct vertex objects at one coincident point for a
    full-period edge (`StdFail_NotDone`)."""
    extent = _v_extent_of_boundary(native_surface, edges)
    if extent is None:
        return None
    v_min, v_max = extent
    native_face = BRepBuilderAPI_MakeFace(native_surface, 0.0, 2.0 * math.pi, v_min, v_max, UV_PROJECTION_TOL).Face()
    return GFace(native_face)


def _main_body_face(native_surface, edges1: list[GEdge], edges2: list[GEdge]) -> "GFace | None":
    """Build the Can's own main cylinder/cone body face: a direct
    parametric trim (`_flat_periodic_trim`) whenever both closing
    curves sit flat (the COAXIAL case), else the joined-wire technique
    (`_join_boundary_wires` + `_sign_corrected_face`) -- the general,
    non-coaxial case, confirmed working end to end against the real
    motivating fixture (`Cans/barrel_left.stp`)."""
    flat = _flat_periodic_trim(native_surface, edges1 + edges2)
    if flat is not None:
        return flat

    wire1 = _wire_from_edges(edges1)
    wire2 = _wire_from_edges(edges2)
    if wire1 is None or wire2 is None:
        return None
    main_wire = _join_boundary_wires(wire1, wire2)
    if main_wire is None:
        return None
    return _sign_corrected_face(native_surface, main_wire)


def _cap_face(native_surface, face_edges: list[GEdge]) -> "GFace | None":
    """Build one of a Can's own end caps: a direct parametric trim
    (`_flat_periodic_trim`) whenever `native_surface` is periodic and
    `face_edges` sits flat (any sphere/cone/cylinder cap whose own
    real boundary is a tangency circle plus, for a true dome/cone-to-
    a-point, a degenerate pole/apex), else the wire-based
    `_wire_from_edges`/`_sign_corrected_face` reconstruction -- always
    used for a `GPlane` cap (never periodic) and for a genuinely
    non-flat (non-coaxial) closing surface."""
    flat = _flat_periodic_trim(native_surface, face_edges)
    if flat is not None:
        return flat
    wire = _wire_from_edges(face_edges)
    if wire is None:
        return None
    return _sign_corrected_face(native_surface, wire)


def Gmake_sewn_can(
    cylinder: GCylinder,
    edges1: list[GEdge],
    surf1,
    edges2: list[GEdge],
    surf2,
    face_edges1: list[GEdge],
    face_edges2: list[GEdge],
) -> "GShell | None":
    """Build the open, three-face cutting-tool shell for a Can
    composite (1 cylinder, closed 360 degrees, + 2 real closures, each
    a plane, sphere, cone, or cylinder): no `BOPAlgo_Splitter` call is
    made while building the tool itself, in place of
    `build_complex_shape`'s own `BuildDepth`/`Gfuse_solids` cascade.

    The main cylinder/cone body (`_main_body_face`) and each cap
    (`_cap_face`) are each built by a direct parametric trim whenever
    their own real boundary sits flat on a periodic surface (the
    COAXIAL case -- confirmed, 2026-10-26, this is NECESSARY, not just
    a simplification: see `_flat_periodic_trim`'s own docstring for
    why joining a full-360-degree closing curve into a wire is
    mathematically unable to represent it without a literal one-period
    discontinuity), falling back to the wire-based
    `_wire_from_edges`/`_join_boundary_wires`/`_sign_corrected_face`
    reconstruction for the general, non-coaxial case (confirmed working
    end to end against the real motivating fixture,
    `Cans/barrel_left.stp`) and for any `GPlane` closing surface
    (never periodic).

    `edges1`/`edges2` and `face_edges1`/`face_edges2` are both found
    during Can detection (`functions.py::build_can_params`, stored as
    `CanParams.Edge1`/`.Edge2` and `.FaceEdges1`/`.FaceEdges2`);
    `surf1`/`surf2` are the composite's own 2 real closing surfaces,
    already resolved to a `geo` analytic type
    (`GPlane`/`GSphere`/`GCone`/`GCylinder`) by the caller.

    Returns `None` (never raises) when the main body or either cap
    can't be built this way (see `_main_body_face`/`_cap_face`'s own
    docstrings) -- the caller (`build_shape_functions.py::makeCan`)
    falls back to `build_complex_shape` for this one candidate."""
    ax3 = gp_Ax3(to_native_vector(cylinder.Center), gp_Dir(cylinder.Axis.x, cylinder.Axis.y, cylinder.Axis.z))
    cyl_surface = Geom_CylindricalSurface(ax3, cylinder.Radius)
    cyl_face = _main_body_face(cyl_surface, edges1, edges2)
    if cyl_face is None:
        return None

    native_surf1 = _native_surface(surf1)
    native_surf2 = _native_surface(surf2)
    cap1 = _cap_face(native_surf1, face_edges1)
    cap2 = _cap_face(native_surf2, face_edges2)
    if cap1 is None or cap2 is None:
        return None

    return Gmake_shell([cap1, cyl_face, cap2])
