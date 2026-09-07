"""
geo/ocp/open_solid_repair.py

Detection + targeted repair of *open* (non-watertight) TopoDS_Solids left
behind by a split.

`_diagnose_open_solid` first tells whether the solid is open at all, and
if so classifies *why* against a small registry of known, recognised
causes; `_close_open_solid` applies the repair registered for that cause
and returns a watertight, BRepCheck-valid, volume-conserving solid (or
None if the cause is unknown / has no repair / the repair didn't hold).

Adding a new known cause later = one `(tag, matcher)` entry in
`_KNOWN_OPEN_PROBLEMS` + one `tag -> repair` entry in
`_OPEN_PROBLEM_REPAIRS`; nothing else changes.

Known causes so far:
  - "split_duplicate_seam": BOPAlgo emitted one trimming-surface boundary
    curve twice (once per adjacent face) at a plane-tangent-to-cylinder /
    near-tangent intersection, so the shell isn't sewn shut along that
    line. Signature: every free edge is either a sub-tolerance micro
    connector or has a near-coincident (< OPEN_SEAM_REL_TOL) duplicate
    partner free edge that shares no vertex with it -- i.e. no free edge
    corresponds to a genuinely missing face. Repair: re-sew every face at
    a tolerance a few x the measured seam width so the doubled seam
    collapses to one shared edge. Confirmed on `Test RoundCorners/
    rc1.stp`'s RoundCorner `surf.shape` wedge (2026-09-04): 21826.54mm^3
    open shell (4 free edges, 4.2e-4mm doubled seam) -> valid closed
    solid, volume unchanged to ~1e-13 relative.
  - "missing_sliver_strip": a sliver face was removed from the shell
    (e.g. by Gsliver_heal) without re-capping the thin slot it left, so
    the shell has a narrow, genuinely-visible-but-thin uncapped strip.
    Same shape as split_duplicate_seam (two near-parallel long free edges
    a small distance apart, bridged by short connectors) but the gap is
    tenths of a mm, not sub-micron -- so it is additionally required to
    be *thin relative to its own length* (< OPEN_STRIP_THIN_RATIO x the
    strip length) and below OPEN_STRIP_GAP_ABS / OPEN_STRIP_GAP_REL*diag,
    to distinguish a slot from a genuinely missing full face. Same
    re-sew repair as split_duplicate_seam. Confirmed on `rev_pipe.stp`'s
    decomposition (2026-09-05): a 0.071mm x 10mm slot beside an R=6 pipe
    -> valid closed solid, volume unchanged.

Accepts a TopoDS_Shell as well as a TopoDS_Solid on input (the shell that
Gsliver_heal can return instead of a solid) -- the re-sew repair produces
a real TopoDS_Solid either way.

`occ`/`ocp` only -- `freecad`'s own Gsplit path (recursive_freecad_Gsplit)
does not go through `_raw_bop_split` and has no equivalent.
"""

from __future__ import annotations

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_MakeVertex, BRepBuilderAPI_Sewing
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.ShapeFix import ShapeFix_Shape
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shell

from ..constants import (
    DEGENERATE_EDGE_LENGTH_FLOOR,
    MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE,
    MIN_SLIVER_EDGE_LENGTH,
    OPEN_SEAM_REL_TOL,
    OPEN_STRIP_GAP_ABS,
    OPEN_STRIP_GAP_REL,
    OPEN_STRIP_THIN_RATIO,
)
from ._native_utils import _linear_props, _volume_props
from .split_repair import _edge_face_map


# --------------------------------------------------------------------------
# generic helpers
# --------------------------------------------------------------------------
def _faces(native_solid) -> list:
    out = []
    explorer = TopExp_Explorer(native_solid, TopAbs_FACE)
    while explorer.More():
        out.append(TopoDS.Face(explorer.Current()))
        explorer.Next()
    return out


def _first_solid(shape):
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    return TopoDS.Solid(explorer.Current()) if explorer.More() else None


def _solid_diagonal(native_solid) -> float:
    box = Bnd_Box()
    BRepBndLib.Add_s(native_solid, box)
    if box.IsVoid():
        return 0.0
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return ((xmax - xmin) ** 2 + (ymax - ymin) ** 2 + (zmax - zmin) ** 2) ** 0.5


def _edge_length(native_edge) -> float:
    return abs(_linear_props(native_edge).Mass())


def _edges_share_vertex(edge_a, edge_b) -> bool:
    va = (TopExp.FirstVertex_s(edge_a), TopExp.LastVertex_s(edge_a))
    vb = (TopExp.FirstVertex_s(edge_b), TopExp.LastVertex_s(edge_b))
    return any(a.IsSame(b) for a in va for b in vb)


def _max_curve_deviation(edge_a, edge_b, samples: int = 7) -> float:
    """Largest distance from an interior point of `edge_a`'s own curve to
    `edge_b` -- how far the two edges bow apart *along their span*, as
    opposed to `BRepExtrema_DistShapeShape(edge_a, edge_b)` which returns
    0 the moment they touch at a shared endpoint. Used to recognise a
    doubled edge that shares its endpoints but diverges in the middle
    (the thin-sliver-strip signature)."""
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
    for i in range(1, edge_map.Extent() + 1):
        if edge_map.FindFromIndex(i).Size() != 1:
            continue
        edge = TopoDS.Edge(edge_map.FindKey(i))
        if BRep_Tool.Degenerated_s(edge):
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
    """Return the measured seam width if `free` (the solid's free edges)
    is entirely explained by a doubled BOPAlgo seam -- micro connector
    edges plus near-coincident duplicate-partner pairs, nothing that
    looks like a real missing face -- else None."""
    if len(free) < 2:
        return None
    max_gap = 0.0
    have_duplicate = False
    for i, edge in enumerate(free):
        if _edge_length(edge) < seam_tol:
            # sub-tolerance connector edge bridging two doubled vertices
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
    """Re-sew every face of `native_solid` at a tolerance a few x the
    measured crack/slot `width` so the two sides of the gap weld into one
    shared edge, then close to a TopoDS_Solid. Shared by both known
    re-sew causes ("split_duplicate_seam", "missing_sliver_strip")."""
    ceiling = max(_solid_diagonal(native_solid) * 1.0e-3, seam_tol, 2.0 * width)
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
        builder.Add(shell, TopoDS.Face(explorer.Current()))
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
    left when a sliver face was removed without re-capping. Unlike
    `_match_split_duplicate_seam` this tolerates a *visible* gap (up to
    OPEN_STRIP_GAP_ABS and OPEN_STRIP_GAP_REL x diag), but only when each
    pair of near-parallel long free edges is thin relative to its own
    length (< OPEN_STRIP_THIN_RATIO); anything wider than the ceiling
    that isn't a thin connector means a genuinely missing full face."""
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
        # nearest other long free edge, measured as how far the two curves
        # bow apart along their span (they may share endpoints -- a doubled
        # edge that diverges in the middle, not two disjoint rails).
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
    # every short free edge must be a connector no longer than ~2x the slot width
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
# registry + public entry points
# --------------------------------------------------------------------------
_KNOWN_OPEN_PROBLEMS = (
    ("split_duplicate_seam", _match_split_duplicate_seam),
    ("missing_sliver_strip", _match_missing_sliver_strip),
)

_OPEN_PROBLEM_REPAIRS = {
    "split_duplicate_seam": _close_split_duplicate_seam,
    "missing_sliver_strip": _close_missing_sliver_strip,
}


def _diagnose_open_solid(native_solid, tolerances) -> str | None:
    """Classify an (already post-split) solid's open-ness:
      * None            -- watertight, nothing to do
      * "<known tag>"   -- open, recognised as a known repairable cause
      * "unknown"       -- open, cause not recognised (leave it to the
                           generic heal/reject path)
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
    volume-conserving TopoDS_Solid, or None (unknown cause / no repair /
    repair didn't produce a sound closed solid)."""
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
    if abs(v1 - v0) > MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE * max(v0, 1.0):
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
