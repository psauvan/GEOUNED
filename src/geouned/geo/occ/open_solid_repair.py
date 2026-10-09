"""
geo/occ/open_solid_repair.py

Detection + targeted repair of *open* (non-watertight) TopoDS_Solids left
behind by a split. See geo/ocp/open_solid_repair.py for the full account;
this is the byte-parallel pythonocc-core port.

`_diagnose_open_solid` first tells whether the solid is open at all, and
if so classifies *why* against a small registry of known causes;
`_close_open_solid` applies the repair registered for that cause and
returns a watertight, BRepCheck-valid, volume-conserving solid (or None).

Adding a new known cause later = one `(tag, matcher)` entry in
`_KNOWN_OPEN_PROBLEMS` + one `tag -> repair` entry in
`_OPEN_PROBLEM_REPAIRS`; nothing else changes.

Known causes so far:
  - "split_duplicate_seam": BOPAlgo emitted one trimming-surface boundary
    curve twice (once per adjacent face) at a plane-tangent-to-cylinder /
    near-tangent intersection, so the shell isn't sewn shut along that
    line. Signature: every free edge is either a sub-tolerance micro
    connector or has a near-coincident (< OPEN_SEAM_REL_TOL) duplicate
    partner free edge that shares no vertex with it. Repair: re-sew every
    face at a tolerance a few x the measured seam width.
  - "missing_sliver_strip": a sliver face was removed from the shell
    (e.g. by Gsliver_heal) without re-capping the thin slot it left. Same
    shape as split_duplicate_seam but a *visible* (tenths of a mm) gap,
    additionally required to be thin relative to its own length and below
    OPEN_STRIP_GAP_ABS / OPEN_STRIP_GAP_REL*diag. Same re-sew repair.
  - "retrace_spike": a boolean cut landed exactly tangent to a
    pre-existing boundary and left a residual edge that RETRACES part
    of its own neighbor's span instead of cleanly stopping there (two
    free edges sharing one vertex, both extending from it in the SAME
    direction). Repair: trim the longer edge back to the shorter edge's
    far (real) vertex and drop the shorter edge
    (`_rebuild_face_without_spike`, planar faces only, preserving any
    OTHER wire of the face unchanged -- e.g. a real inner hole shared
    with a third face) -- never a re-sew. See geo/ocp/open_solid_repair.py
    for the full account (Big_model_reserved/divertor_cam.stp's own
    STRUCTURAL_PLATE_2#BYZ2RP_31 solid, 2026-10-09).

Accepts a TopoDS_Shell as well as a TopoDS_Solid on input.

`occ`/`ocp` only.
"""

from __future__ import annotations

from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Builder, BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.gp import gp_Dir, gp_Pln, gp_Pnt, gp_Vec
from OCC.Core.ShapeBuild import ShapeBuild_ReShape
from OCC.Core.ShapeFix import ShapeFix_Shape
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID, TopAbs_WIRE
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopoDS import TopoDS_Shell, topods

from ..constants import (
    COLLINEAR_SIN_TOL,
    DEGENERATE_EDGE_LENGTH_FLOOR,
    MAX_REPAIR_VOLUME_REL_CHANGE,
    MIN_SLIVER_EDGE_LENGTH,
    OPEN_SEAM_REL_TOL,
    OPEN_STRIP_GAP_ABS,
    OPEN_STRIP_GAP_REL,
    OPEN_STRIP_THIN_RATIO,
    POINT_POINT_TOL,
    RESEW_CEILING,
)
from ._native_utils import _linear_props, _volume_props
from .split_repair import _edge_face_map
from .topology import GFace, GPlane
from ..volume_utils import volume_within


# --------------------------------------------------------------------------
# generic helpers
# --------------------------------------------------------------------------
def _faces(native_solid) -> list:
    out = []
    explorer = TopExp_Explorer(native_solid, TopAbs_FACE)
    while explorer.More():
        out.append(topods.Face(explorer.Current()))
        explorer.Next()
    return out


def _first_solid(shape):
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    return topods.Solid(explorer.Current()) if explorer.More() else None


def _solid_diagonal(native_solid) -> float:
    box = Bnd_Box()
    brepbndlib.Add(native_solid, box)
    if box.IsVoid():
        return 0.0
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return ((xmax - xmin) ** 2 + (ymax - ymin) ** 2 + (zmax - zmin) ** 2) ** 0.5


def _edge_length(native_edge) -> float:
    return abs(_linear_props(native_edge).Mass())


def _edges_share_vertex(edge_a, edge_b) -> bool:
    va = (topexp.FirstVertex(edge_a), topexp.LastVertex(edge_a))
    vb = (topexp.FirstVertex(edge_b), topexp.LastVertex(edge_b))
    return any(a.IsSame(b) for a in va for b in vb)


def _max_curve_deviation(edge_a, edge_b, samples: int = 7) -> float:
    """Largest distance from an interior point of `edge_a`'s own curve to
    `edge_b` -- how far the two edges bow apart along their span, as
    opposed to `BRepExtrema_DistShapeShape` which returns 0 once they
    touch at a shared endpoint. See geo/ocp/open_solid_repair.py."""
    ca = BRepAdaptor_Curve(edge_a)
    t0, t1 = ca.FirstParameter(), ca.LastParameter()
    worst = 0.0
    for k in range(1, samples):
        p = ca.Value(t0 + (t1 - t0) * k / samples)
        d = BRepExtrema_DistShapeShape(BRepBuilderAPI_MakeVertex(p).Vertex(), edge_b).Value()
        worst = max(worst, d)
    return worst


def _free_edges(native_solid) -> list:
    """Non-degenerate edges used by exactly one face -- the open boundary
    of a shell. A watertight solid has none."""
    edge_map = _edge_face_map(native_solid)
    free = []
    for i in range(1, edge_map.Size() + 1):
        if edge_map.FindFromIndex(i).Size() != 1:
            continue
        edge = topods.Edge(edge_map.FindKey(i))
        if BRep_Tool.Degenerated(edge):
            continue
        if _edge_length(edge) <= DEGENERATE_EDGE_LENGTH_FLOOR:
            continue
        free.append(edge)
    return free


def _seam_tolerance(native_solid) -> float:
    return max(_solid_diagonal(native_solid) * OPEN_SEAM_REL_TOL, MIN_SLIVER_EDGE_LENGTH)


# --------------------------------------------------------------------------
# known cause: "split_duplicate_seam"
# --------------------------------------------------------------------------
def _match_split_duplicate_seam(native_solid, free, seam_tol) -> float | None:
    """Return the measured seam width if `free` is entirely explained by a
    doubled BOPAlgo seam -- micro connector edges plus near-coincident
    duplicate-partner pairs, nothing that looks like a real missing face
    -- else None."""
    if len(free) < 2:
        return None
    max_gap = 0.0
    have_duplicate = False
    for i, edge in enumerate(free):
        if _edge_length(edge) < seam_tol:
            max_gap = max(max_gap, _edge_length(edge))
            continue
        partner_gap = None
        for j, other in enumerate(free):
            if j == i or _edges_share_vertex(edge, other):
                continue
            gap = BRepExtrema_DistShapeShape(edge, other).Value()
            if gap < seam_tol and (partner_gap is None or gap < partner_gap):
                partner_gap = gap
        if partner_gap is None:
            return None  # a free edge with no duplicate -> genuinely missing face
        have_duplicate = True
        max_gap = max(max_gap, partner_gap)
    return max_gap if have_duplicate else None


def _resew_faces_to_solid(native_solid, width, seam_tol):
    """Re-sew every face at a tolerance a few x the crack/slot `width` so
    the two sides weld into one shared edge, then close to a TopoDS_Solid.
    Shared by "split_duplicate_seam" and "missing_sliver_strip"."""
    ceiling = max(_solid_diagonal(native_solid) * RESEW_CEILING, seam_tol, 2.0 * width)
    sew_tol = min(max(3.0 * width, seam_tol), ceiling)

    sewer = BRepBuilderAPI_Sewing(sew_tol, True, True, True, False)
    for face in _faces(native_solid):
        sewer.Add(face)
    sewer.Perform()

    builder = BRep_Builder()
    shell = TopoDS_Shell()
    builder.MakeShell(shell)
    seen = False
    explorer = TopExp_Explorer(sewer.SewedShape(), TopAbs_FACE)
    while explorer.More():
        builder.Add(shell, topods.Face(explorer.Current()))
        seen = True
        explorer.Next()
    if not seen:
        return None

    maker = BRepBuilderAPI_MakeSolid(shell)
    if not maker.IsDone():
        return None
    solid = maker.Solid()

    if _free_edges(solid):
        fixer = ShapeFix_Shape(solid)
        fixer.SetPrecision(sew_tol)
        fixer.SetMaxTolerance(sew_tol)
        fixer.Perform()
        fixed = _first_solid(fixer.Shape())
        if fixed is not None:
            solid = fixed
    return solid


def _close_split_duplicate_seam(native_solid, tolerances):
    seam_tol = _seam_tolerance(native_solid)
    width = _match_split_duplicate_seam(native_solid, _free_edges(native_solid), seam_tol)
    if width is None:
        return None
    return _resew_faces_to_solid(native_solid, width, seam_tol)


# --------------------------------------------------------------------------
# known cause: "missing_sliver_strip"
# --------------------------------------------------------------------------
def _match_missing_sliver_strip(native_solid, free, seam_tol) -> float | None:
    """Return the measured strip width if the free edges are entirely
    explained by narrow uncapped slot(s) -- a thin missing strip face
    left when a sliver face was removed without re-capping. See
    geo/ocp/open_solid_repair.py for the full account."""
    if len(free) < 2:
        return None
    diag = _solid_diagonal(native_solid)
    ceiling = min(OPEN_STRIP_GAP_ABS, diag * OPEN_STRIP_GAP_REL)
    if ceiling <= seam_tol:
        return None

    lengths = [_edge_length(e) for e in free]
    long_idx = [i for i, length in enumerate(lengths) if length > ceiling]
    short_idx = [i for i, length in enumerate(lengths) if length <= ceiling]
    if len(long_idx) < 2:
        return None

    max_gap = 0.0
    for i in long_idx:
        partner_gap = None
        for j in long_idx:
            if j == i:
                continue
            dev = max(_max_curve_deviation(free[i], free[j]),
                      _max_curve_deviation(free[j], free[i]))
            if dev < ceiling and dev < OPEN_STRIP_THIN_RATIO * min(lengths[i], lengths[j]):
                if partner_gap is None or dev < partner_gap:
                    partner_gap = dev
        if partner_gap is None:
            return None  # a long free edge with no thin partner -> real missing face
        max_gap = max(max_gap, partner_gap)

    if max_gap <= 0.0:
        return None
    connector_ceiling = max(2.0 * max_gap, seam_tol)
    for i in short_idx:
        if lengths[i] > connector_ceiling:
            return None
    return max_gap


def _close_missing_sliver_strip(native_solid, tolerances):
    seam_tol = _seam_tolerance(native_solid)
    width = _match_missing_sliver_strip(native_solid, _free_edges(native_solid), seam_tol)
    if width is None:
        return None
    return _resew_faces_to_solid(native_solid, width, seam_tol)


# --------------------------------------------------------------------------
# known cause: "retrace_spike"
# --------------------------------------------------------------------------
def _edge_endpoints_verts(native_edge):
    return topexp.FirstVertex(native_edge), topexp.LastVertex(native_edge)


def _shared_vertex(edge_a, edge_b):
    """The TopoDS_Vertex shared by edge_a and edge_b, or None."""
    for va in _edge_endpoints_verts(edge_a):
        for vb in _edge_endpoints_verts(edge_b):
            if va.IsSame(vb):
                return va
    return None


def _other_endpoint(native_edge, shared_vertex):
    """The endpoint of `native_edge` that is NOT `shared_vertex`."""
    a, b = _edge_endpoints_verts(native_edge)
    return b if a.IsSame(shared_vertex) else a


def _retrace_spike_shared_vertex(edge_a, edge_b):
    """The shared vertex if `edge_a`/`edge_b` meet at exactly one vertex
    AND both extend from it in the SAME direction (collinear, not a
    genuine polygon corner, which would point apart) -- the signature
    of a boolean cut landing exactly tangent to a pre-existing boundary,
    leaving a residual edge that retraces part of its own neighbor's
    span instead of stopping there (confirmed live, ocp engine,
    Big_model_reserved/divertor_cam.stp's own STRUCTURAL_PLATE_2#BYZ2RP_31
    solid, 2026-10-09). None if they don't share a vertex, or point in
    genuinely different directions from it."""
    shared = _shared_vertex(edge_a, edge_b)
    if shared is None:
        return None
    pv = BRep_Tool.Pnt(shared)
    pa = BRep_Tool.Pnt(_other_endpoint(edge_a, shared))
    pb = BRep_Tool.Pnt(_other_endpoint(edge_b, shared))
    va, vb = gp_Vec(pv, pa), gp_Vec(pv, pb)
    la, lb = va.Magnitude(), vb.Magnitude()
    if la <= DEGENERATE_EDGE_LENGTH_FLOOR or lb <= DEGENERATE_EDGE_LENGTH_FLOOR:
        return None
    cos_angle = va.Dot(vb) / (la * lb)
    if cos_angle < 1.0 - COLLINEAR_SIN_TOL:
        return None
    return shared


def _match_retrace_spike(native_solid, free, seam_tol):
    """Return the list of `(short_edge, long_edge, shared_vertex)` spike
    triples if every free edge in `free` is explained as half of a
    retrace-spike pair with another free edge (see
    `_retrace_spike_shared_vertex`) -- else None (some free edge is
    genuinely unexplained, meaning a missing face, not this cause).

    A genuine retrace spike is always two edges of the SAME face's own
    wire -- two edges of DIFFERENT faces can be collinear and point the
    same way from a shared vertex perfectly legitimately (an ordinary
    straight corner), so `_single_host_face` identity is required too,
    not just `_retrace_spike_shared_vertex`'s own geometric test alone
    (see geo/ocp/open_solid_repair.py's own docstring for the real
    fixture that confirmed this)."""
    if not free:
        return None
    hosts = {i: _single_host_face(native_solid, e) for i, e in enumerate(free)}
    spikes = []
    explained = set()
    for i, e_i in enumerate(free):
        if i in explained:
            continue
        for j, e_j in enumerate(free):
            if i == j or hosts[i] is None or hosts[j] is None or not hosts[i].IsSame(hosts[j]):
                continue
            shared = _retrace_spike_shared_vertex(e_i, e_j)
            if shared is None:
                continue
            if _edge_length(e_i) <= _edge_length(e_j):
                spikes.append((e_i, e_j, shared))
                explained.add(i)
                explained.add(j)  # the long half is TRIMMED (consumed), not unexplained either
                break
    # A free edge of a NEIGHBOR face can still be resolved by this
    # repair as a side effect: once a spike's own long edge is trimmed
    # to its short partner's far vertex, the new trimmed edge shares
    # its 2 endpoints EXACTLY (same TopoDS_Vertex objects, never fresh
    # ones) with whatever real boundary edge the neighbor face already
    # has there -- sewing then welds the two into one shared edge,
    # clearing both from the free list.
    trim_endpoints = []
    for short_edge, long_edge, shared_v in spikes:
        trim_endpoints.append((_other_endpoint(long_edge, shared_v), _other_endpoint(short_edge, shared_v)))
    for i, e in enumerate(free):
        if i in explained:
            continue
        v1, v2 = _edge_endpoints_verts(e)
        if any((v1.IsSame(a) and v2.IsSame(b)) or (v1.IsSame(b) and v2.IsSame(a)) for a, b in trim_endpoints):
            explained.add(i)
    if len(explained) != len(free) or not spikes:
        return None  # a genuinely unexplained free edge -> not this cause
    return spikes


def _order_wire_edges_local(edges: list) -> "list | None":
    """Reorder `edges` (assumed to form exactly one simple closed loop)
    into a connectivity-respecting sequence -- `BRepBuilderAPI_MakeWire.
    Add()` only connects a new edge to the CURRENT open end of the wire
    being built so far, it does not defer and retry one that doesn't
    connect yet. A local copy rather than importing `repair.py`'s own
    `_order_wire_edges` directly: `repair.py` itself imports from this
    module at its own bottom, so importing back from it here would be
    circular. `None` if the edges don't actually form a single closed
    loop."""
    remaining = list(edges)
    if not remaining:
        return None

    def endpoints(e):
        return _edge_endpoints_verts(e)

    ordered = [remaining.pop(0)]
    _, end_v = endpoints(ordered[0])
    end_pt = BRep_Tool.Pnt(end_v)
    while remaining:
        for i, e in enumerate(remaining):
            v1, v2 = endpoints(e)
            p1, p2 = BRep_Tool.Pnt(v1), BRep_Tool.Pnt(v2)
            if p1.Distance(end_pt) < POINT_POINT_TOL:
                ordered.append(e)
                end_pt = p2
                remaining.pop(i)
                break
            if p2.Distance(end_pt) < POINT_POINT_TOL:
                ordered.append(e)
                end_pt = p1
                remaining.pop(i)
                break
        else:
            return None  # no remaining edge connects -- not a single closed loop
    return ordered


def _single_host_face(native_solid, edge):
    """The one face bordering `edge` in `native_solid`, or None if it
    borders zero or more than one. `_edge_face_map`'s own ancestor list
    holds bare `TopoDS_Shape` wrappers -- downcast to `TopoDS_Face`
    before returning, since every real caller needs the properly typed
    shape."""
    edge_map = _edge_face_map(native_solid)
    for i in range(1, edge_map.Size() + 1):
        if edge_map.FindKey(i).IsSame(edge):
            faces = list(edge_map.FindFromIndex(i))
            return topods.Face(faces[0]) if len(faces) == 1 else None
    return None


def _rebuild_face_without_spike(face, long_edge, short_edge, shared_vertex):
    """`face`'s own outer wire, with the (long_edge, short_edge) retrace
    pair replaced by a single edge trimmed to stop at `short_edge`'s own
    far endpoint -- the real neighbor's own boundary point, reusing its
    actual `TopoDS_Vertex` directly (never a fresh, numerically-close
    point). Any OTHER wire of `face` (a real inner hole, e.g. a
    circular bore shared with a different neighbor face entirely) is
    preserved unchanged and re-added to the rebuilt face -- see
    geo/ocp/open_solid_repair.py's own docstring for the real regression
    this guards against. Planar faces only -- declines (returns None)
    for anything else."""
    gf = GFace(face)
    if not isinstance(gf.Surface, GPlane):
        return None

    wexp = TopExp_Explorer(face, TopAbs_WIRE)
    all_wires = []
    while wexp.More():
        all_wires.append(topods.Wire(wexp.Current()))
        wexp.Next()
    if not all_wires:
        return None

    def wire_edges(w):
        out = []
        e2 = TopExp_Explorer(w, TopAbs_EDGE)
        while e2.More():
            out.append(topods.Edge(e2.Current()))
            e2.Next()
        return out

    target_idx = next((wi for wi, w in enumerate(all_wires) if any(e.IsSame(long_edge) for e in wire_edges(w))), None)
    if target_idx is None:
        return None
    edges = wire_edges(all_wires[target_idx])
    other_wires = [w for wi, w in enumerate(all_wires) if wi != target_idx]

    long_idx = next((i for i, e in enumerate(edges) if e.IsSame(long_edge)), None)
    short_idx = next((i for i, e in enumerate(edges) if e.IsSame(short_edge)), None)
    if long_idx is None or short_idx is None:
        return None

    far_vertex = _other_endpoint(short_edge, shared_vertex)
    other_vertex = _other_endpoint(long_edge, shared_vertex)
    new_edge = BRepBuilderAPI_MakeEdge(other_vertex, far_vertex).Edge()

    kept_edges = [e for i, e in enumerate(edges) if i not in (long_idx, short_idx)]
    kept_edges.append(new_edge)
    ordered = _order_wire_edges_local(kept_edges)
    if ordered is None:
        return None

    wire_maker = BRepBuilderAPI_MakeWire()
    for e in ordered:
        wire_maker.Add(e)
    if not wire_maker.IsDone():
        return None

    pos, ax = gf.Surface.Position, gf.Surface.Axis
    gpln = gp_Pln(gp_Pnt(pos.x, pos.y, pos.z), gp_Dir(ax.x, ax.y, ax.z))
    face_maker = BRepBuilderAPI_MakeFace(gpln, wire_maker.Wire(), True)
    for w in other_wires:
        face_maker.Add(w)
    if not face_maker.IsDone():
        return None
    new_face = face_maker.Face()
    new_face.Orientation(face.Orientation())
    return new_face


def _close_retrace_spike(native_solid, tolerances):
    free = _free_edges(native_solid)
    seam_tol = _seam_tolerance(native_solid)
    spikes = _match_retrace_spike(native_solid, free, seam_tol)
    if spikes is None:
        return None

    try:
        reshaper = ShapeBuild_ReShape()
        used_hosts = []
        for short_edge, long_edge, shared_vertex in spikes:
            host = _single_host_face(native_solid, short_edge)
            if host is None or not _single_host_face(native_solid, long_edge).IsSame(host):
                return None
            if any(host.IsSame(h) for h in used_hosts):
                return None  # two spikes on the same face -- not a verified case
            new_face = _rebuild_face_without_spike(host, long_edge, short_edge, shared_vertex)
            if new_face is None:
                return None
            reshaper.Replace(host, new_face)
            used_hosts.append(host)

        reduced = reshaper.Apply(native_solid)

        sewer = BRepBuilderAPI_Sewing(seam_tol)
        fexp = TopExp_Explorer(reduced, TopAbs_FACE)
        while fexp.More():
            sewer.Add(fexp.Current())
            fexp.Next()
        sewer.Perform()

        builder = BRep_Builder()
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        seen = False
        sexp = TopExp_Explorer(sewer.SewedShape(), TopAbs_FACE)
        while sexp.More():
            builder.Add(shell, topods.Face(sexp.Current()))
            seen = True
            sexp.Next()
        if not seen:
            return None

        maker = BRepBuilderAPI_MakeSolid(shell)
        if not maker.IsDone():
            return None
        solid = maker.Solid()
        # The raw MakeSolid result is reliably BRepCheck-INVALID even
        # once every free edge is gone (a face-level orientation/
        # parametrization residue from the wire rebuild above, not a
        # topology gap, see geo/ocp/open_solid_repair.py's own
        # docstring) -- a single `ShapeFix_Shape` pass resolves it.
        # Same `_first_solid`-guarded pattern as `_resew_faces_to_solid`'s
        # own tail, in case it degrades to a Shell/Compound instead of
        # fixing in place.
        if not BRepCheck_Analyzer(solid).IsValid():
            fixer = ShapeFix_Shape(solid)
            fixer.Perform()
            fixed = _first_solid(fixer.Shape())
            if fixed is not None:
                solid = fixed
        return solid
    except Exception:
        return None


# --------------------------------------------------------------------------
# registry + public entry points
# --------------------------------------------------------------------------
_KNOWN_OPEN_PROBLEMS = (
    ("split_duplicate_seam", _match_split_duplicate_seam),
    ("missing_sliver_strip", _match_missing_sliver_strip),
    ("retrace_spike", _match_retrace_spike),
)

_OPEN_PROBLEM_REPAIRS = {
    "split_duplicate_seam": _close_split_duplicate_seam,
    "missing_sliver_strip": _close_missing_sliver_strip,
    "retrace_spike": _close_retrace_spike,
}


def _diagnose_open_solid(native_solid, tolerances) -> str | None:
    """Classify an (already post-split) solid's open-ness:
      * None            -- watertight, nothing to do
      * "<known tag>"   -- open, recognised as a known repairable cause
      * "unknown"       -- open, cause not recognised
    """
    free = _free_edges(native_solid)
    if not free:
        return None
    seam_tol = _seam_tolerance(native_solid)
    for tag, matcher in _KNOWN_OPEN_PROBLEMS:
        if matcher(native_solid, free, seam_tol) is not None:
            return tag
    return "unknown"


def _close_open_solid(native_solid, problem: str, tolerances):
    """Apply the repair registered for `problem` (a tag from
    `_diagnose_open_solid`). Returns a watertight, BRepCheck-valid,
    volume-conserving TopoDS_Solid, or None."""
    repair = _OPEN_PROBLEM_REPAIRS.get(problem)
    if repair is None:
        return None
    try:
        result = repair(native_solid, tolerances)
    except Exception:
        return None
    if result is None or _free_edges(result):
        return None
    if not BRepCheck_Analyzer(result).IsValid():
        return None
    v0 = abs(_volume_props(native_solid).Mass())
    v1 = abs(_volume_props(result).Mass())
    if not volume_within(v1, v0, MAX_REPAIR_VOLUME_REL_CHANGE):
        return None
    return result


def _repair_open_solid(native_solid, tolerances):
    """`_diagnose_open_solid` + `_close_open_solid` in one call: returns a
    watertight, valid, volume-conserving TopoDS_Solid when `native_solid`
    is open from a *known* cause, else None (already watertight / unknown
    cause / repair didn't hold)."""
    problem = _diagnose_open_solid(native_solid, tolerances)
    if problem is None or problem == "unknown":
        return None
    return _close_open_solid(native_solid, problem, tolerances)
