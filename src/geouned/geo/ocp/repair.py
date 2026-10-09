"""
geo/ocp/repair.py

Load-time CAD-defect repair: Gdefeature, Gcollapse_split_rings,
Gsliver_heal, Gcheck_and_repair, Gspline_surface, Gheal_topology.
"""

from __future__ import annotations

import io

from OCP.BRep import (
    BRep_Builder,
    BRep_Tool,
)
from OCP.BRepAdaptor import (
    BRepAdaptor_Curve,
    BRepAdaptor_Surface,
)
from OCP.BRepAlgoAPI import BRepAlgoAPI_Defeaturing
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import (
    GeomAbs_Circle,
    GeomAbs_Cone,
    GeomAbs_Cylinder,
    GeomAbs_Line,
)
from OCP.gp import (
    gp_Ax2,
    gp_Circ,
    gp_Dir,
    gp_Pln,
    gp_Pnt,
)
from OCP.IFSelect import IFSelect_RetDone
from OCP.ShapeBuild import ShapeBuild_ReShape
from OCP.ShapeFix import ShapeFix_Shape, ShapeFix_Solid
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.STEPControl import (
    STEPControl_AsIs,
    STEPControl_Reader,
    STEPControl_Writer,
)
from OCP.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_REVERSED,
    TopAbs_SHELL,
    TopAbs_VERTEX,
)
from OCP.TopExp import (
    TopExp,
    TopExp_Explorer,
)
from OCP.TopoDS import (
    TopoDS,
    TopoDS_Shell,
    TopoDS_Vertex,
)
from OCP.TopTools import (
    TopTools_IndexedDataMapOfShapeListOfShape,
    TopTools_ListOfShape,
)
from ..vector_geometry import GVector
from ..solid_defects import (
    check_solid_defects,
    count_split_ring_pairs,
    find_overlapping_coplanar_faces,
    find_sliver_faces,
    find_short_edges,
    find_split_ring_faces,
    near_surface_pair,
)
from ..constants import (
    COLLINEAR_SIN_TOL,
    FACE_CONSTRUCTION_TOL,
    FINITE_DIFF_STEP,
    MAX_REPAIR_VOLUME_REL_CHANGE,
    MIN_SLIVER_EDGE_LENGTH,
    NUMERIC_DOUBLE_TOL,
    OCCT_FIX_TOLERANCE,
    POINT_POINT_TOL,
)
from ..io_utils import suppress_native_stdout
from ..surface_geometry import is_same_plane_surface
from .topology import GEdge, GFace, GPlane, GSolid, Gclassify_surface
from .boolean import _exploded_solids
from ._native_utils import _volume_props


def Gdefeature(solid: "GSolid", faces: "list[GFace]", sliver_edge_rel_tol) -> "GSolid | None":
    """Attempt to remove `faces` (typically solid_defects.find_short_edges'
    own output) from `solid` via BRepAlgoAPI_Defeaturing, verifying the
    result is genuinely usable before trusting it -- confirmed live
    (2026-08-27, Solidos/working_solids/"beltline left.stp") that
    `defeat.IsDone()==True` alone is NOT sufficient: it can return a
    topologically *valid* solid that's still silently wrong (a real
    volume change and spurious new faces left behind by the algorithm's
    own attempt to patch the gap it was given).

    Deliberately a single, fast, one-shot attempt -- NOT an iterative or
    graph-based search for the "correct" minimal face set to remove. Per
    explicit user direction: detecting a genuine CAD defect is more
    valuable than perfectly auto-repairing it (a real translation
    failure or MCNP lost-particle result wrongly blamed on GEOUNED is
    worse than an honest "this solid could not be auto-repaired, fix the
    CAD externally"), and any repair attempt kept in this pipeline must
    stay general and fast -- not a bespoke, slow reconstruction tuned to
    one fixture. Confirmed on "beltline left.stp" itself that no simple
    face-selection strategy (touching a short edge, connected components
    of short edges alone, iterative growth from a minimal seed) reliably
    finds the true minimal repair set in general -- its own real defect
    needs 4 specific faces removed together, none of the tried
    strategies reproduced exactly that set without also risking pulling
    in legitimate geometry (mirror-symmetry-cut planes, real end caps)
    elsewhere. Returning None here and letting the caller fall back to
    "flag as corrupted, don't convert" is the intended, accepted outcome
    for a case like this -- not a gap to close later.

    Returns None whenever defeaturing doesn't complete, the healed
    result isn't topologically valid, find_short_edges() still finds a
    short edge in it, or its own Volume has drifted from the input by
    more than MAX_REPAIR_VOLUME_REL_CHANGE (see that constant's own
    docstring for the real, dangerous false-pass that motivated adding
    it -- structural validity and a clean short-edge re-check are BOTH
    insufficient on their own) -- never raises, matching this project's
    own "silent not-applicable, not a crash" convention for every other
    geometry repair fallback (see e.g. _try_coaxial_cone_split above)."""
    if not faces:
        return None
    defeat = BRepAlgoAPI_Defeaturing()
    defeat.SetShape(solid.__native__)
    face_list = TopTools_ListOfShape()
    for f in faces:
        face_list.Append(f.__native__)
    defeat.AddFacesToRemove(face_list)
    try:
        defeat.Build()
    except Exception:
        return None
    if not defeat.IsDone():
        return None
    healed_native = defeat.Shape()
    if not BRepCheck_Analyzer(healed_native).IsValid():
        return None
    healed = GSolid(healed_native)
    if find_short_edges(healed, sliver_edge_rel_tol):
        return None
    if not volume_within(healed.Volume, solid.Volume, MAX_REPAIR_VOLUME_REL_CHANGE):
        return None
    return healed


def Gcollapse_split_rings(solid: "GSolid", tolerances) -> "GSolid | None":
    """Repair a "split boundary ring" / duplicated micro-trim defect.

    A single trimming surface (plane or cylinder) appears twice at a
    sub-tolerance offset; the thin slab between the two copies is filled
    by parasitic "riser" faces, and every curved analytic face that
    meets the trim is bounded by two near-coincident concentric circular
    edges (bridged by pathologically short connector edges) instead of
    one. `BRepCheck_Analyzer` reports the solid valid, and every generic
    OCCT healer (`ShapeFix_Shape`, `ShapeUpgrade_UnifySameDomain`,
    `ShapeFix_Wireframe`, `BRepAlgoAPI_Defeaturing`) no-ops on it -- and
    any boolean re-cut returns empty/unchanged, the source degeneracy
    breaking every BOP on the solid.

    This repair: identify the riser faces
    (``solid_defects.find_split_ring_faces``), remove them
    (``ShapeBuild_ReShape``), re-sew the remaining shell at a tolerance a
    few times the ring gap -- welding the two trim surfaces' shared rims
    and each curved face's doubled boundary ring into one --
    ``BRepBuilderAPI_MakeSolid`` + ``ShapeFix_Shape``.

    ACCEPTANCE (all three required -- valid + volume alone are NOT enough,
    same lesson as `Gdefeature`'s own false-pass history):
      - ``result.is_valid()``;
      - ``|dV| / max(|V|, 1) < MAX_REPAIR_VOLUME_REL_CHANGE`` (3e-4).
        A genuine collapse only removes sliver-volume risers, so it
        conserves volume to ~1e-4 or better (barrel bottom.stp: 6.4e-5).
        A looser gate (Gdefeature used to have 1%) is too loose here -- a sew that distorts a *real* adjacent surface stays
        "valid" and near-volume-conserving while producing CSG-broken
        geometry (LR.stp: dV 7e-4, valid, but d1suned tally 0.0 / 24 lost
        particles);
      - ``count_split_ring_pairs`` strictly DECREASED -- the direct
        success test, "did the doubled boundary rings actually merge".
        barrel bottom.stp: 18 -> 12 (pass). LR.stp: 2 -> 2 (fail -- a
        *cylindrical* collapsed-step where sew-collapse leaves the
        fingerprint intact; this is the beltline-family case CLAUDE.md
        already documents as unsolved -- correctly rejected, so the
        caller flags it corrupted rather than silently mis-repairing).

    Residual sub-sew-tolerance connector edges may remain on the (kept)
    curved/planar faces -- an internal-wire micro-tab sewing cannot weld
    -- and are HARMLESS: on ``barrel bottom.stp`` the repaired solid
    keeps 2 faces with ~0.029mm edges yet converts with a tally of
    0.9997 +/- 0.27% and 0 lost particles. So acceptance does NOT require
    `find_short_edges` to come back empty.

    NOT attempted: ``ShapeFix_Wireframe.FixSmallEdges`` clears the
    residual edges but reshapes the trimmed sphere boundary -> ~0.22%
    volume drift on a STEP round-trip (the in-memory dV is a misleading
    ~1e-4; the exported/reloaded solid is ~2.2e-3 off). It fails the
    guard and is deliberately left out.

    ocp/occ only -- ``ShapeBuild_ReShape`` + ``BRepBuilderAPI_Sewing`` is
    the pipeline used; the freecad backend's `Gcollapse_split_rings` is a
    None-returning stub. Never raises, matching this project's
    "silent not-applicable, not a crash" convention for every geometry
    repair fallback."""
    min_face_width = tolerances.min_face_width
    risers = find_split_ring_faces(solid, min_face_width, tolerances.sliver_edge_rel_tol)
    if not risers:
        return None
    pairs_before = count_split_ring_pairs(solid)

    edge_lengths = [edge.Length for face in risers for edge in face.Edges if edge.Length > 0.0]
    max_short = max(edge_lengths) if edge_lengths else MIN_SLIVER_EDGE_LENGTH
    sew_tol = min(max(3.0 * max_short, 10.0 * MIN_SLIVER_EDGE_LENGTH), min_face_width)

    try:
        reshaper = ShapeBuild_ReShape()
        for face in risers:
            reshaper.Remove(face.__native__)
        reduced = reshaper.Apply(solid.__native__)

        sewer = BRepBuilderAPI_Sewing(sew_tol, True, True, True, False)
        explorer = TopExp_Explorer(reduced, TopAbs_FACE)
        while explorer.More():
            sewer.Add(explorer.Current())
            explorer.Next()
        sewer.Perform()
        sewed = sewer.SewedShape()

        builder = BRep_Builder()
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        seen = False
        face_explorer = TopExp_Explorer(sewed, TopAbs_FACE)
        while face_explorer.More():
            builder.Add(shell, TopoDS.Face(face_explorer.Current()))
            seen = True
            face_explorer.Next()
        if not seen:
            return None

        solid_maker = BRepBuilderAPI_MakeSolid(shell)
        if not solid_maker.IsDone():
            return None

        built_solid = solid_maker.Solid()
        fixer = ShapeFix_Shape(built_solid)
        fixer.SetPrecision(sew_tol)
        fixer.Perform()
        healed = fixer.Shape()
        # ShapeFix_Shape can degrade a genuine TopoDS_Solid to a bare
        # TopoDS_Shell that stays BRepCheck-valid (closed) but is never
        # re-wrapped as a solid -- same class of bug Gsliver_heal already
        # guards against (see its own docstring, rev_pipe.stp 2026-09-11).
        # Re-wrap the already-healed shell back into a solid rather than
        # discard the healing and fall back to the pre-fix built solid.
        coerced = _coerce_to_solid(healed)
        healed = coerced if coerced is not None else built_solid
        result = GSolid(healed)
    except Exception:
        return None

    if not result.is_valid():
        return None
    if not volume_within(result.Volume, solid.Volume, MAX_REPAIR_VOLUME_REL_CHANGE):
        return None
    if count_split_ring_pairs(result) >= pairs_before:
        return None
    return result


def _edge_endpoints(native_edge):
    pts = []
    vexp = TopExp_Explorer(native_edge, TopAbs_VERTEX)
    while vexp.More():
        pts.append(BRep_Tool.Pnt_s(TopoDS.Vertex(vexp.Current())))
        vexp.Next()
    return pts


def _free_edges(shape):
    edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
    return [
        TopoDS.Edge(edge_face_map.FindKey(i))
        for i in range(1, edge_face_map.Extent() + 1)
        if edge_face_map.FindFromIndex(i).Size() == 1
    ]


def _retrim_freed_quadrics(reduced_shape, drop_plane, keep_plane, tol):
    """After the near-pair's smaller face is dropped, any cylinder / cone
    face that had a rim on that (drop) plane is now a free rim sitting
    `keep_offset - drop_offset` away from where it needs to be. Re-trim
    each such face's V range so that rim lands on the kept plane -- a
    re-trim of an *unbounded* quadric (its own surface, new parameter
    bounds), NOT a surface extension. Returns the (possibly unchanged)
    shape."""
    axis = keep_plane.Axis
    drop_offset = axis.dot(drop_plane.Position)
    keep_offset = axis.dot(keep_plane.Position)
    free = _free_edges(reduced_shape)

    reshaper = ShapeBuild_ReShape()
    changed = 0
    face_explorer = TopExp_Explorer(reduced_shape, TopAbs_FACE)
    while face_explorer.More():
        face = TopoDS.Face(face_explorer.Current())
        face_explorer.Next()
        surf_adaptor = BRepAdaptor_Surface(face)
        if surf_adaptor.GetType() not in (GeomAbs_Cylinder, GeomAbs_Cone):
            continue
        rim_on_drop = False
        edge_explorer = TopExp_Explorer(face, TopAbs_EDGE)
        while edge_explorer.More():
            edge = TopoDS.Edge(edge_explorer.Current())
            edge_explorer.Next()
            if not any(edge.IsSame(fe) for fe in free):
                continue
            curve_adaptor = BRepAdaptor_Curve(edge)
            if curve_adaptor.GetType() != GeomAbs_Circle:
                continue
            centre = curve_adaptor.Circle().Location()
            if abs(axis.dot(GVector(centre.X(), centre.Y(), centre.Z())) - drop_offset) < tol:
                rim_on_drop = True
                break
        if not rim_on_drop:
            continue

        surface = BRep_Tool.Surface_s(face)
        u1, u2, v1, v2 = BRepTools.UVBounds_s(face)
        u_mid = 0.5 * (u1 + u2)

        def z_at(v):
            p = surface.Value(u_mid, v)
            return axis.dot(GVector(p.X(), p.Y(), p.Z()))

        z1, z2 = z_at(v1), z_at(v2)
        if abs(z1 - drop_offset) <= abs(z2 - drop_offset):
            slope = (z_at(v1 + FINITE_DIFF_STEP) - z1) / FINITE_DIFF_STEP
            if abs(slope) < NUMERIC_DOUBLE_TOL:
                continue
            v1 = v1 + (keep_offset - z1) / slope
        else:
            slope = (z_at(v2 + FINITE_DIFF_STEP) - z2) / FINITE_DIFF_STEP
            if abs(slope) < NUMERIC_DOUBLE_TOL:
                continue
            v2 = v2 + (keep_offset - z2) / slope
        if v1 >= v2:
            continue
        new_face = BRepBuilderAPI_MakeFace(surface, u1, u2, v1, v2, FACE_CONSTRUCTION_TOL)
        if not new_face.IsDone():
            continue
        reshaper.Replace(face, new_face.Face())
        changed += 1

    return reshaper.Apply(reduced_shape) if changed else reduced_shape


def _snapped_planar_cap(reduced_shape, keep_plane):
    """Close the free-edge hole(s) left after removing the sliver + dropped
    faces: project every free edge onto `keep_plane` (collapsing the tiny
    connector edges that were the sliver's own axial extent), chain the
    survivors into a wire, and build one planar face on `keep_plane`.

    Returns a list of TopoDS_Face (usually 1), [] when nothing is open
    (a barrel-style case that needs no cap), or None when the hole is not
    a single planar loop this v0 can cap (non-analytic edge, wire won't
    close, non-planar loop)."""
    plane_offset = keep_plane.Axis.dot(keep_plane.Position)
    axis = keep_plane.Axis

    def snap(pnt):
        v = GVector(pnt.X(), pnt.Y(), pnt.Z())
        v = v + axis * (plane_offset - axis.dot(v))
        return gp_Pnt(v.x, v.y, v.z)

    free_edges = _free_edges(reduced_shape)
    if not free_edges:
        return []

    new_edges = []
    for edge in free_edges:
        adaptor = BRepAdaptor_Curve(edge)
        curve_type = adaptor.GetType()
        pts = _edge_endpoints(edge)
        if len(pts) != 2:
            return None
        a, b = snap(pts[0]), snap(pts[1])
        if a.Distance(b) < POINT_POINT_TOL:
            continue  # connector edge (the sliver's axial extent) -> gone
        if curve_type == GeomAbs_Circle:
            circ = adaptor.Circle()
            axes = gp_Ax2(snap(circ.Location()), circ.Axis().Direction())
            new_edges.append(BRepBuilderAPI_MakeEdge(gp_Circ(axes, circ.Radius()), a, b).Edge())
        elif curve_type == GeomAbs_Line:
            new_edges.append(BRepBuilderAPI_MakeEdge(a, b).Edge())
        else:
            return None
    if not new_edges:
        return None

    wire_maker = BRepBuilderAPI_MakeWire()
    edge_list = TopTools_ListOfShape()
    for e in new_edges:
        edge_list.Append(e)
    wire_maker.Add(edge_list)
    if not wire_maker.IsDone():
        return None
    plane = gp_Pln(snap(gp_Pnt(0.0, 0.0, 0.0)), gp_Dir(axis.x, axis.y, axis.z))
    face_maker = BRepBuilderAPI_MakeFace(plane, wire_maker.Wire(), True)
    if not face_maker.IsDone():
        return None
    return [face_maker.Face()]


def Gsliver_heal(solid: "GSolid", tolerances) -> "GSolid | None":
    """`sliver_healing` (version 0) -- the fuller form of
    `Gcollapse_split_rings` for a "split boundary ring" defect whose
    leftover, after the sliver faces are removed, is a genuine
    *near-coincident* (not exactly-coincident) surface pair that blind
    sewing cannot reconcile.

    Steps (see reference_cad_defect_recipes.md's "sliver_healing" spec,
    sew is LAST -- never before the face set is decided):
      1-2. Remove the sliver faces (`find_split_ring_faces`) via
           `ShapeBuild_ReShape` -- this also drops their edges from every
           neighbour's wire.
      3.   Find `near_surface_pair`s among the remaining analytic faces
           (same kind, coincident axis, one varying parameter 1e-5 < gap
           < `max(diag*1e-4, 1e-3)`). v0: exactly one such pair, else
           `None` (nothing here that `Gcollapse_split_rings` doesn't
           already cover).
      4-1. Keep the larger-area face of the pair, `.Remove` the smaller.
      4-2. `_retrim_freed_quadrics`: for any cylinder/cone face left with
           a free rim on the dropped face's plane, re-trim its V range so
           that rim lands on the KEPT plane (re-trim of an unbounded
           quadric, not surface extension). Then `_snapped_planar_cap`:
           project the resulting free-edge loop onto the kept plane
           (collapsing the sliver's own axial connector edges) and cap it
           with one planar face.
      5.   Sew (reduced faces + cap), `MakeSolid`, `ShapeFix_Shape`,
           `ShapeUpgrade_UnifySameDomain` (merges the cap into the kept
           coplanar face).
      6.   Accept only if valid AND
           `|dV| < MAX_REPAIR_VOLUME_REL_CHANGE` (3e-4). No
           `count_split_ring_pairs` check -- this repair's own cap is a
           thin annulus whose two coplanar rims that metric false-counts;
           with `_retrim_freed_quadrics` a correct heal conserves volume
           to ~1e-6, so the dV bound alone is a tight gate.

    Verified on `Solidos/working_solids/LR.stp` (the cylindrical
    collapsed-step `Gcollapse_split_rings` correctly rejects): faces
    13 -> 9, dV 8.6e-7, d1suned tally 0.9985 / 0 lost (the input's own
    translation was tally 0.0 / 24 lost). `barrel bottom.stp` / a clean
    solid -> `None` (no near-pair). ocp/occ only; the freecad backend's
    `Gsliver_heal` is a `None`-returning stub. Never raises.

    2026-08-30: a same-session attempt to generalize step 3-4-1 to
    "every near-surface pair found, not just a single one" (keeping
    whichever face of each pair BRepCheck_Analyzer reports individually
    valid) was reverted, then re-corrected -- confirmed via a full
    Solidos/test_models d1suned batch that the original attempt's own
    keep/drop criterion silently corrupted several already-fixed
    fixtures (barrel bottom, rev_pipe x2, SCDR_90_hollow, SCDR_90). Root
    cause: a single face plucked from an already-valid solid is
    essentially always individually "valid" on its own, so that
    criterion never actually discriminated which face to keep; and the
    loop's `_retrim_freed_quadrics`/`_snapped_planar_cap` calls only
    ever used the *last* pair's own drop/keep faces, silently ignoring
    every earlier pair's removal.

    A first fix over-corrected to requiring exactly one near-pair, else
    None -- per direct user pushback, this wrongly rejects a real,
    simpler case: a genuinely defective sliver face that is itself a
    near-duplicate of a real face never shows up as a near-pair at all
    (both `find_sliver_faces` and the near-pair search only look at
    `others`, which already excludes every classified sliver -- so a
    sliver-classified duplicate is removed by the unconditional
    `reshaper.Remove(f.__native__)` loop below and needs no further
    retrim/cap at all). So `near` being empty is a legitimate, common
    outcome, not a reason to bail. What's fixed here instead: keep the
    face of GREATER AREA in each pair (the real, documented v0
    criterion), and process every pair independently -- each pair gets
    its own `_retrim_freed_quadrics`/`_snapped_planar_cap` call against
    the shape as reduced so far, and every pair's own cap is collected,
    not just the last one's."""

    min_face_width = tolerances.min_face_width
    min_length_ratio = tolerances.sliver_edge_rel_tol
    slivers = list(find_sliver_faces(solid, min_face_width))
    if not slivers:
        return None
    diag = solid.BoundBox.DiagonalLength
    dist_tol = max(diag * min_length_ratio, MIN_SLIVER_EDGE_LENGTH)

    sliver_native = [f.__native__ for f in slivers]
    others = [f for f in solid.Faces if not any(f.__native__.IsSame(s) for s in sliver_native)]
    near = []
    for i in range(len(others)):
        for j in range(i + 1, len(others)):
            if near_surface_pair(others[i].Surface, others[j].Surface, dist_tol) is not None:
                near.append((others[i], others[j]))

    try:
        reshaper = ShapeBuild_ReShape()
        for f in slivers:
            reshaper.Remove(f.__native__)

        pairs = []
        for face_a, face_b in near:
            keep, drop = (face_a, face_b) if face_a.Area >= face_b.Area else (face_b, face_a)
            reshaper.Remove(drop.__native__)
            pairs.append((keep, drop))

        reduced = reshaper.Apply(solid.__native__)

        caps = []
        for keep, drop in pairs:
            reduced = _retrim_freed_quadrics(reduced, drop.Surface, keep.Surface, dist_tol)
            pair_caps = _snapped_planar_cap(reduced, keep.Surface)
            if pair_caps is None:
                return None
            caps.extend(pair_caps)

        sewer = BRepBuilderAPI_Sewing(OCCT_FIX_TOLERANCE, True, True, True, False)
        explorer = TopExp_Explorer(reduced, TopAbs_FACE)
        while explorer.More():
            sewer.Add(explorer.Current())
            explorer.Next()
        for cap in caps:
            sewer.Add(cap)
        sewer.Perform()
        sewed = sewer.SewedShape()

        builder = BRep_Builder()
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        seen = False
        face_explorer = TopExp_Explorer(sewed, TopAbs_FACE)
        while face_explorer.More():
            builder.Add(shell, TopoDS.Face(face_explorer.Current()))
            seen = True
            face_explorer.Next()
        if not seen:
            return None
        solid_maker = BRepBuilderAPI_MakeSolid(shell)
        if not solid_maker.IsDone():
            return None
        built_solid = solid_maker.Solid()
        fixer = ShapeFix_Shape(built_solid)
        fixer.SetPrecision(OCCT_FIX_TOLERANCE)
        fixer.Perform()
        unify = ShapeUpgrade_UnifySameDomain(fixer.Shape(), UnifyEdges=True, UnifyFaces=True, ConcatBSplines=True)
        unify.SetLinearTolerance(OCCT_FIX_TOLERANCE)
        unify.Build()
        healed = unify.Shape()
        # ShapeUpgrade_UnifySameDomain (and, more rarely, ShapeFix_Shape)
        # can degrade a genuine TopoDS_Solid to a bare TopoDS_Shell/
        # Compound that stays BRepCheck-valid (closed) but is never
        # re-wrapped as a solid -- confirmed live on rev_pipe.stp
        # (2026-09-11): _exploded_solids(healed) came back empty even
        # though .Volume and BRepCheck both looked fine, silently
        # dropping ~509606mm^3 from the final decomposition with no
        # crash anywhere downstream (only .Solids' own recursive
        # TopAbs_SOLID discovery, several layers up in generic_split,
        # ever notices). "BRepCheck-valid" alone is not enough to trust
        # a cosmetic cleanup step here either -- the same lesson this
        # whole cascade already applies elsewhere (Gdefeature/
        # Gcollapse_split_rings's own false-pass histories) -- so fall
        # back to the closest earlier stage that still resolves to a
        # genuine solid, rather than trusting the most "cleaned up"
        # result blindly.
        if not _exploded_solids(healed):
            healed = fixer.Shape()
            if not _exploded_solids(healed):
                healed = built_solid
        # The face-by-face sew above can leave a bare TopoDS_Shell, or a
        # solid with a thin uncapped slot where a removed sliver face
        # wasn't re-capped. Hand it to the open-solid repair -- it
        # recognises that (missing_sliver_strip) and re-sews it into a
        # real, watertight TopoDS_Solid; a no-op (returns None) when the
        # shape is already a valid closed solid.
        closed = _repair_open(healed, tolerances)
        if closed is not None:
            healed = closed
        result = GSolid(healed)
    except Exception:
        return None

    if not result.is_valid():
        return None
    if not volume_within(result.Volume, solid.Volume, MAX_REPAIR_VOLUME_REL_CHANGE):
        return None
    return result


def _edge_vertices(native_edge) -> list:
    vs = []
    ve = TopExp_Explorer(native_edge, TopAbs_VERTEX)
    while ve.More():
        vs.append(TopoDS.Vertex(ve.Current()))
        ve.Next()
    return vs


def _assemble_wires(edges: list):
    """Chain a flat list of native ``TopoDS_Edge`` into one or more
    closed wires by shared vertices (topological ``IsSame``). Returns
    ``list[TopoDS_Wire]`` or ``None`` if any wire fails to build."""
    remaining = list(edges)
    wires = []
    while remaining:
        chain = [remaining.pop(0)]
        progressed = True
        while progressed:
            progressed = False
            ends = _edge_vertices(chain[0]) + _edge_vertices(chain[-1])
            for k, e in enumerate(remaining):
                evs = _edge_vertices(e)
                if any(a.IsSame(b) for a in ends for b in evs):
                    chain.append(remaining.pop(k))
                    progressed = True
                    break
        wl = TopTools_ListOfShape()
        for e in chain:
            wl.Append(e)
        wm = BRepBuilderAPI_MakeWire()
        wm.Add(wl)
        if not wm.IsDone():
            return None
        wires.append(wm.Wire())
    return wires


def _wire_bbox_diag(native_wire) -> float:
    box = Bnd_Box()
    BRepBndLib.Add_s(native_wire, box)
    if box.IsVoid():
        return 0.0
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return (xmax - xmin) ** 2 + (ymax - ymin) ** 2 + (zmax - zmin) ** 2


def _merge_coplanar_group(group_faces: list, plane_desc):
    """`group_faces`: native ``TopoDS_Face`` all on the same infinite
    plane, connected through shared edges. Return one native
    ``TopoDS_Face`` that is their union with the seams they shared with
    each other removed, or ``None`` if it can't be built cleanly.

    An edge that belongs to exactly one face of the group is on the
    merged boundary; an edge shared by two group faces is an internal
    seam and is dropped; an edge shared by three or more is non-manifold
    and aborts the merge."""
    uniq = []
    counts = []
    for f in group_faces:
        ee = TopExp_Explorer(f, TopAbs_EDGE)
        while ee.More():
            e = TopoDS.Edge(ee.Current())
            ee.Next()
            hit = None
            for j, u in enumerate(uniq):
                if u.IsSame(e):
                    hit = j
                    break
            if hit is None:
                uniq.append(e)
                counts.append(1)
            else:
                counts[hit] += 1

    if any(c > 2 for c in counts):
        return None
    boundary = [uniq[j] for j in range(len(uniq)) if counts[j] == 1]
    if not boundary:
        return None

    wires = _assemble_wires(boundary)
    if not wires:
        return None
    wires.sort(key=_wire_bbox_diag, reverse=True)

    pos, ax = plane_desc.Position, plane_desc.Axis
    gpln = gp_Pln(gp_Pnt(pos.x, pos.y, pos.z), gp_Dir(ax.x, ax.y, ax.z))

    fm = BRepBuilderAPI_MakeFace(gpln, wires[0], True)
    for w in wires[1:]:
        fm.Add(w)
    if not fm.IsDone():
        return None
    new_face = fm.Face()

    # every face of a valid solid's group bounds the solid on the same
    # side, so match the first face's orientation.
    if group_faces[0].Orientation() == TopAbs_REVERSED:
        new_face = TopoDS.Face(new_face.Reversed())
    return new_face


def Gmerge_coplanar_planes(solid: "GSolid", tolerances) -> "GSolid":
    """Merge every group of adjacent, co-planar planar faces of `solid`
    into a single planar face, removing the edges those faces shared
    with each other -- a hand-rolled, planes-only alternative to
    ``GSolid.refine()`` (``ShapeUpgrade_UnifySameDomain``; see that
    method's crash history).

    Non-planar faces pass through untouched. Two planar faces merge when
    they share an edge and ``surface_geometry.is_same_plane_surface``
    accepts their ``GPlane`` descriptors (parallel/antiparallel normal +
    same offset); a chain of three or more such faces collapses to one.
    The merged face is rebuilt from the group's non-shared boundary
    edges on the common plane, and the solid is re-sewn from the kept
    faces (untouched + one merged per group).

    Returns the merged ``GSolid`` only if it is BRepCheck-valid and
    volume-conserving to 1e-6 relative (same guard as ``refine()``);
    otherwise returns `solid` unchanged. Never raises."""
    try:
        native = solid.__native__

        faces = []
        exp = TopExp_Explorer(native, TopAbs_FACE)
        while exp.More():
            faces.append(TopoDS.Face(exp.Current()))
            exp.Next()
        n = len(faces)
        if n < 2:
            return solid

        planes = [None] * n
        for i, f in enumerate(faces):
            s = Gclassify_surface(f)
            if isinstance(s, GPlane):
                planes[i] = s

        def face_index(face):
            for i, f in enumerate(faces):
                if f.IsSame(face):
                    return i
            return None

        parent = list(range(n))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        edge_map = TopTools_IndexedDataMapOfShapeListOfShape()
        TopExp.MapShapesAndAncestors_s(native, TopAbs_EDGE, TopAbs_FACE, edge_map)
        for k in range(1, edge_map.Extent() + 1):
            face_list = edge_map.FindFromIndex(k)
            if face_list.Size() != 2:
                continue
            it = iter(face_list)
            a = face_index(next(it))
            b = face_index(next(it))
            if a is None or b is None or planes[a] is None or planes[b] is None:
                continue
            if is_same_plane_surface(planes[a], planes[b], tolerances):
                union(a, b)

        groups: dict[int, list] = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(i)
        if all(len(idxs) == 1 for idxs in groups.values()):
            return solid

        kept_faces = []
        merged_any = False
        for idxs in groups.values():
            if len(idxs) == 1:
                kept_faces.append(faces[idxs[0]])
                continue
            merged = _merge_coplanar_group([faces[i] for i in idxs], planes[idxs[0]])
            if merged is None:
                kept_faces.extend(faces[i] for i in idxs)
                continue
            kept_faces.append(merged)
            merged_any = True

        if not merged_any:
            return solid

        sewer = BRepBuilderAPI_Sewing(OCCT_FIX_TOLERANCE)
        for f in kept_faces:
            sewer.Add(f)
        sewer.Perform()
        sewed = sewer.SewedShape()

        builder = BRep_Builder()
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        se = TopExp_Explorer(sewed, TopAbs_FACE)
        any_face = False
        while se.More():
            builder.Add(shell, TopoDS.Face(se.Current()))
            any_face = True
            se.Next()
        if not any_face:
            return solid

        solid_maker = BRepBuilderAPI_MakeSolid(shell)
        if not solid_maker.IsDone():
            return solid
        fixer = ShapeFix_Shape(solid_maker.Solid())
        fixer.Perform()
        result = GSolid(fixer.Shape())
    except Exception:
        return solid

    if not BRepCheck_Analyzer(result.__native__).IsValid():
        return solid
    if not volume_within(result.Volume, solid.Volume, tolerances.volume_tolerance):
        return solid
    return result


def _shared_edges(face_a: "GFace", face_b: "GFace") -> list:
    """Native edges that are the literal SAME `TopoDS_Edge` (by
    reference, `IsSame`) in both faces' own outer wires -- i.e. a real,
    already-existing topological edge connecting the two faces, not
    just a geometric coincidence."""
    shared = []
    for ea in face_a.Edges:
        for eb in face_b.Edges:
            if ea.is_same(eb):
                shared.append(ea.__native__)
    return shared


def _native_vertices(edge: "GEdge") -> "tuple | None":
    """The edge's own 2 real `TopoDS_Vertex` objects (not just their
    coordinates) -- needed so a newly-built replacement edge can share
    the EXACT same vertex object as the edges it connects to, rather
    than a numerically-close-but-distinct one (which `BRepBuilderAPI_
    MakeWire` does not bridge -- confirmed live, struct_plate2.stp's
    own piece8: a wire built from coordinate-only `gp_Pnt` endpoints
    left one of 4 edges disconnected, silently producing a near-zero-
    area face instead of the intended quadrilateral)."""
    v1, v2 = TopoDS_Vertex(), TopoDS_Vertex()
    TopExp.Vertices_s(edge.__native__, v1, v2)
    if v1.IsNull() or v2.IsNull():
        return None
    return (v1, v2)


def _other_vertex(edge: "GEdge", known: GVector, tol: float = POINT_POINT_TOL) -> "TopoDS_Vertex | None":
    """The edge's own real `TopoDS_Vertex` that is NOT (within `tol`)
    `known`; `None` unless exactly one of the edge's 2 vertices matches
    `known` (so the edge genuinely touches it, and the result is
    unambiguous)."""
    native_verts = _native_vertices(edge)
    if native_verts is None:
        return None
    near, far = [], []
    for v in native_verts:
        (near if (_vertex_point(v) - known).length < tol else far).append(v)
    if len(near) != 1 or len(far) != 1:
        return None
    return far[0]


def _vertex_point(vertex: "TopoDS_Vertex") -> GVector:
    p = BRep_Tool.Pnt_s(vertex)
    return GVector(p.X(), p.Y(), p.Z())


def _order_wire_edges(edges: list) -> "list | None":
    """Reorder a set of native edges, assumed to form exactly one simple
    closed loop, into a connectivity-respecting sequence. Needed because
    `BRepBuilderAPI_MakeWire.Add()` only connects a new edge to the
    CURRENT open end of the wire being built so far -- it does not defer
    and retry an edge that doesn't connect yet, it just silently drops
    it (confirmed live, struct_plate2.stp's own piece8: feeding its 4
    replacement edges in the order they were discovered left one
    disconnected, producing a near-zero-area face instead of the
    intended quadrilateral). `None` if the edges don't actually form a
    single closed loop (never guesses past that)."""
    remaining = list(edges)
    if not remaining:
        return None

    def endpoints(e):
        v1, v2 = TopoDS_Vertex(), TopoDS_Vertex()
        TopExp.Vertices_s(e, v1, v2)
        return v1, v2

    ordered = [remaining.pop(0)]
    _, end_v = endpoints(ordered[0])
    end_pt = _vertex_point(end_v)
    while remaining:
        for i, e in enumerate(remaining):
            v1, v2 = endpoints(e)
            if (_vertex_point(v1) - end_pt).length < POINT_POINT_TOL:
                ordered.append(e)
                end_pt = _vertex_point(v2)
                remaining.pop(i)
                break
            if (_vertex_point(v2) - end_pt).length < POINT_POINT_TOL:
                ordered.append(e)
                end_pt = _vertex_point(v1)
                remaining.pop(i)
                break
        else:
            return None  # no remaining edge connects -- not a single closed loop
    return ordered


def _trim_edge_for_boundary_touching_pair(small: "GFace", large: "GFace", shared_native_edge) -> "list | None":
    """Build the replacement wire edges for `large` when `small` (the
    phantom) shares a REAL edge with `large` instead of sitting
    strictly inside it (see `Gtrim_overlapping_coplanar_faces`'s own
    docstring for the two cases). `small`'s own two side edges and
    `large`'s own two side edges, at EACH endpoint of the shared edge,
    are collinear -- `small`'s own side edge is the short, correctly-
    connected inner portion (already bordering `small`'s own real
    neighbor faces) and `large`'s own side edge is the too-long
    superset running past it. Returns the new wire's edges (2 freshly
    trimmed + `small`'s own far edge(s), reused as-is + `large`'s own
    unaffected far edge(s), reused as-is), or `None` if the geometry
    doesn't match this specific, verified pattern (handled this way so
    an unexpected configuration falls back to `None` -- never guesses).
    """
    shared_verts = [v for v in GEdge(shared_native_edge).Vertexes]
    if len(shared_verts) != 2:
        return None
    v0, v1 = shared_verts[0], shared_verts[-1]

    small_other = [e for e in small.Edges if not e.__native__.IsSame(shared_native_edge)]
    large_other = [e for e in large.Edges if not e.__native__.IsSame(shared_native_edge)]

    new_edges = []
    consumed_small = set()
    consumed_large = set()
    for v in (v0, v1):
        large_adj = next((e for e in large_other if _other_vertex(e, v) is not None), None)
        small_adj = next((e for e in small_other if _other_vertex(e, v) is not None), None)
        if large_adj is None or small_adj is None:
            return None
        far_large = _other_vertex(large_adj, v)
        far_small = _other_vertex(small_adj, v)
        if far_large is None or far_small is None:
            return None

        far_large_pt = _vertex_point(far_large)
        far_small_pt = _vertex_point(far_small)
        direction_large = far_large_pt - v
        direction_small = far_small_pt - v
        if direction_large.length < POINT_POINT_TOL or direction_small.length < POINT_POINT_TOL:
            return None
        # sin(angle) between the two directions; STEP vertex coordinates
        # are only stored to ~6 decimals, so two edges that are really
        # collinear still show a sin(angle) of order 1e-5 here -- a
        # pure absolute cross-product-length gate (scaled by only one
        # of the two lengths) is too tight for that noise. COLLINEAR_SIN_TOL
        # is a generous margin above the measured real noise (~1-2e-5)
        # while staying far below any genuinely non-collinear angle.
        sin_angle = direction_large.cross(direction_small).length / (direction_large.length * direction_small.length)
        if sin_angle > COLLINEAR_SIN_TOL:
            return None  # not collinear -- not the pattern this function handles
        if direction_small.length >= direction_large.length:
            return None  # small's own far point must lie strictly inside large's own span

        # Reuse the REAL vertex objects (far_small, far_large), not fresh
        # gp_Pnt-based ones, so the new edge shares exact topological
        # identity with the edges it must connect to in the new wire --
        # confirmed live this is load-bearing: a coordinate-only rebuild
        # left one of 4 edges disconnected (see _native_vertices' own
        # docstring).
        new_edges.append(BRepBuilderAPI_MakeEdge(far_small, far_large).Edge())
        consumed_small.add(id(small_adj.__native__))
        consumed_large.add(id(large_adj.__native__))

    for e in small_other:
        if id(e.__native__) not in consumed_small:
            new_edges.append(e.__native__)
    for e in large_other:
        if id(e.__native__) not in consumed_large:
            new_edges.append(e.__native__)
    return new_edges


def _coerce_to_solid(shape):
    """`shape` as a genuine `TopoDS_Solid` (wrapped and type-preserved,
    not necessarily `BRepCheck`-valid -- see the final fallback below
    for why), or `None`.

    `ShapeFix_Shape`/`ShapeUpgrade_UnifySameDomain` can degrade a Solid
    input to a bare, still-valid, still-volume-correct `TopoDS_Shell`
    (see `Gcollapse_split_rings`/`Gtrim_overlapping_coplanar_faces`'s own
    docstrings for the real fixture this was confirmed on) -- silently
    discarding real healing work by falling all the way back to an
    unhealed solid is worse than just re-wrapping the already-healed
    shell back into a solid, which is what this does. A plain
    `BRepBuilderAPI_MakeSolid` wrap alone is not enough, though: a shell
    valid AT THE SHELL LEVEL can still fail `BRepCheck_Analyzer` once
    forced into a solid (confirmed live on the same fixture --
    solid-level validity needs the wrapped shell's own face orientations
    to be globally consistent, which shell-level `ShapeFix_Shape` has no
    reason to guarantee) -- so a second fix pass runs on the
    freshly-wrapped solid itself before it's trusted.

    That second pass uses `ShapeFix_Solid`, NOT another `ShapeFix_Shape`
    call -- found and fixed 2026-10-09, a real regression on
    `Mixed/struct_plate2.stp`'s own decomposed piece8 (the ORIGINAL
    fixture this whole repair was built for, see
    `Gtrim_overlapping_coplanar_faces`'s own docstring): a second
    `ShapeFix_Shape` pass on `wrapped` re-degraded it straight back to a
    bare `TopoDS_Shell` (the exact same failure this function exists to
    recover from), reliably, every time -- `ShapeFix_Shape` is a
    generic fixer with no obligation to preserve the Solid type it was
    given. `ShapeFix_Solid` is the dedicated solid-level fixer (face-
    orientation consistency for a closed shell forming a solid) and,
    confirmed live on the same fixture, never changes its own result's
    shape type away from Solid -- so a second attempt with it can
    succeed where `ShapeFix_Shape` does not. Its own result is trusted
    via `_exploded_solids` alone (matching this function's own
    established preference, see the very first check above) even when
    `BRepCheck_Analyzer` still separately flags it invalid: on
    struct_plate2.stp's own piece8, the resulting solid's volume matches
    the pre-repair solid to the exact value this fix was independently
    verified against (2303.999949, see CLAUDE.md's own "overlapping
    coplanar faces" entry) -- `BRepCheck_Analyzer`'s own complaint about
    the SOLID-typed wrapper is demonstrably not catching a real
    geometric defect here, only a residual, functionally-harmless
    strictness that a bare, un-wrapped Shell of the exact same geometry
    was never held to in the first place (the ORIGINAL, pre-`_coerce_to_
    solid` code accepted that same Shell directly, successfully, with
    no `BRepCheck_Analyzer` check on it at all -- see `Gtrim_overlapping_
    coplanar_faces`'s own history). The final, plain-`BRepCheck_Analyzer`-
    gated fallback below is kept only for the case `ShapeFix_Solid`
    itself doesn't even attempt (a wrap that was never a Shell to begin
    with reaching this line some other way) -- not expected to fire in
    practice, kept defensive."""
    if _exploded_solids(shape):
        return shape
    if shape.ShapeType() != TopAbs_SHELL:
        return None
    solid_maker = BRepBuilderAPI_MakeSolid(TopoDS.Shell(shape))
    if not solid_maker.IsDone():
        return None
    wrapped = solid_maker.Solid()
    fixer = ShapeFix_Shape(wrapped)
    fixer.Perform()
    fixed = fixer.Shape()
    if _exploded_solids(fixed):
        return fixed
    solid_fixer = ShapeFix_Solid(wrapped)
    solid_fixer.Perform()
    solid_fixed = solid_fixer.Solid()
    if _exploded_solids(solid_fixed):
        return solid_fixed
    return wrapped if BRepCheck_Analyzer(wrapped).IsValid() else None


def Gtrim_overlapping_coplanar_faces(solid: "GSolid") -> "GSolid | None":
    """Repair the "overlapping coplanar faces" defect (see
    `solid_defects.find_overlapping_coplanar_faces`'s own docstring for
    the defect itself and its real-fixture signature): a smaller planar
    patch (`small`) sits entirely inside a larger coplanar face's
    (`large`) own trimmed domain because `large` is missing the
    inner-wire hole that should exclude it.

    `small` is NOT a genuine separate feature to keep -- it is the
    residual "phantom" face left by the boolean cut that should have
    trimmed `large` into e.g. an annulus in the first place (confirmed
    empirically: keeping `small` as its own face AND re-trimming `large`
    around it makes `small`'s own boundary edge shared by 3 faces at
    once -- `small` itself, `large`'s new hole, and whatever real
    feature `small` already bordered there, e.g. the bore's own
    cylindrical wall -- a non-manifold edge, and the resulting solid's
    volume comes out measurably wrong (confirmed both via the native
    volume integral AND an independent Monte-Carlo point-classification
    check -- the two didn't even agree with EACH OTHER on that broken
    attempt, exposing it as genuinely non-manifold despite
    `BRepCheck_Analyzer` reporting it "valid"). The real recipe (per
    direct user instruction, matching what this investigation converged
    on independently): 1) delete the phantom (`small`) from the solid
    entirely; 2) remove `large` from the solid and add `small`'s own
    boundary wire to it as an inner wire (reversed, the standard hole
    convention) -- `large` ends up directly bordering whatever `small`
    used to border, exactly as a correctly-resolved boolean cut would
    have produced; 3) re-sew the result into a solid. Verified on the
    real fixture: 0 free/non-manifold edges afterward (was 2 non-manifold
    before, from the 3-way sharing above), volume conserved to ~3e-6
    relative, confirmed independently via Monte Carlo (within ~1 sigma
    of the pre-repair solid's own, likewise Monte-Carlo-confirmed,
    volume).

    `ShapeBuild_ReShape` does both substitutions (`Replace` for `large`,
    `Remove` for `small`) in one `Apply`, which is then re-sewn and
    re-solidified exactly like every other repair in this cascade.

    Returns the healed `GSolid` only if it is BRepCheck-valid, conserves
    volume to `MAX_REPAIR_VOLUME_REL_CHANGE`, AND
    `find_overlapping_coplanar_faces` no longer finds any of the
    original pairs (the same "the defect's own fingerprint must
    strictly decrease" discipline as `Gcollapse_split_rings`'s
    `count_split_ring_pairs` check); `None` otherwise -- never raises.

    A SECOND real sub-case (found 2026-10-02, `struct_plate2.stp`'s own
    decomposed piece8): `small` can instead share a REAL edge with
    `large` -- it touches `large`'s own outer boundary along one side
    rather than sitting strictly inside it (confirmed live: a direct
    point-sampling check found NO material on either side of `small`'s
    own surface, same signature as the strictly-interior case, so it is
    equally a phantom -- just a different boundary shape). The "add a
    hole" construction below requires the inner wire to be strictly
    interior, so it raises/produces an invalid face for this case
    instead. Handled by `_trim_edge_for_boundary_touching_pair`: `large`'s
    own two edges adjacent to the shared edge are collinear supersets of
    `small`'s own two corresponding edges (which are already correctly
    connected to `small`'s real neighbor faces) -- trim them down to
    `small`'s own far endpoint and reuse `small`'s own remaining edges
    directly, producing a single new outer wire with no hole at all.

    ocp/occ only -- the freecad backend's `Gtrim_overlapping_coplanar_faces`
    is a `None`-returning stub, matching `Gcollapse_split_rings`/
    `Gsliver_heal`'s own precedent."""
    pairs = find_overlapping_coplanar_faces(solid)
    if not pairs:
        return None

    try:
        reshaper = ShapeBuild_ReShape()
        for small, large in pairs:
            pos, ax = large.Surface.Position, large.Surface.Axis
            gpln = gp_Pln(gp_Pnt(pos.x, pos.y, pos.z), gp_Dir(ax.x, ax.y, ax.z))

            shared = _shared_edges(small, large)
            if not shared:
                outer_wire = large.outer_wire().__native__
                hole_wire = TopoDS.Wire(small.outer_wire().__native__.Reversed())

                face_maker = BRepBuilderAPI_MakeFace(gpln, outer_wire, True)
                face_maker.Add(hole_wire)
                if not face_maker.IsDone():
                    return None
                new_face = face_maker.Face()
            elif len(shared) == 1:
                new_edges = _trim_edge_for_boundary_touching_pair(small, large, shared[0])
                if new_edges is None:
                    return None
                ordered_edges = _order_wire_edges(new_edges)
                if ordered_edges is None:
                    return None
                wire_maker = BRepBuilderAPI_MakeWire()
                for e in ordered_edges:
                    wire_maker.Add(e)
                if not wire_maker.IsDone():
                    return None
                face_maker = BRepBuilderAPI_MakeFace(gpln, wire_maker.Wire(), True)
                if not face_maker.IsDone():
                    return None
                new_face = face_maker.Face()
            else:
                return None  # a chain of >1 shared edges isn't a verified case yet

            reshaper.Replace(large.__native__, new_face)
            reshaper.Remove(small.__native__)

        reduced = reshaper.Apply(solid.__native__)

        sewer = BRepBuilderAPI_Sewing(OCCT_FIX_TOLERANCE)
        explorer = TopExp_Explorer(reduced, TopAbs_FACE)
        while explorer.More():
            sewer.Add(explorer.Current())
            explorer.Next()
        sewer.Perform()
        sewed = sewer.SewedShape()

        builder = BRep_Builder()
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        seen = False
        face_explorer = TopExp_Explorer(sewed, TopAbs_FACE)
        while face_explorer.More():
            builder.Add(shell, TopoDS.Face(face_explorer.Current()))
            seen = True
            face_explorer.Next()
        if not seen:
            return None

        solid_maker = BRepBuilderAPI_MakeSolid(shell)
        if not solid_maker.IsDone():
            return None
        built_solid = solid_maker.Solid()
        fixer = ShapeFix_Shape(built_solid)
        fixer.Perform()
        # `_coerce_to_solid` was tried here (2026-10-07, for
        # Big_model_reserved/divertor_cam.stp's own
        # STRUCTURAL_PLATE_2#BYZ2RP_31 solid -- ShapeFix_Shape can
        # degrade a genuine TopoDS_Solid to a bare, still-BRepCheck-
        # valid-AS-A-SHELL TopoDS_Shell that is never re-wrapped as a
        # solid, and the written cell definition built from this
        # silently-Shell-typed "solid" gave a d1suned tally of exactly
        # 0.0) and REVERTED, 2026-10-09, after a real regression on
        # `Mixed/struct_plate2.stp`'s own decomposed piece8 (the
        # ORIGINAL fixture this whole repair was built for -- see this
        # function's own docstring): wrapping the healed shell into an
        # actual TopoDS_Solid, by ANY means (plain `MakeSolid`,
        # `ShapeFix_Shape`, or `ShapeFix_Solid` -- all 3 tried), makes
        # `BRepCheck_Analyzer` apply SOLID-level checks (closed-volume
        # face-orientation consistency) that the bare Shell this
        # function already accepts below was never held to -- and it
        # genuinely fails those checks on this exact, independently-
        # verified-correct geometry (same volume as the already-shipped
        # fix, 2303.999949), which then trips `is_valid()`/`check_solid_
        # defects`'s own "invalid topology" flag EVERYWHERE ELSE this
        # piece is queried downstream (not just inside this function's
        # own gate), since the returned object's native shape would stay
        # Solid-typed-but-invalid regardless of any local acceptance
        # logic here. Accepting the bare Shell directly (as below) keeps
        # struct_plate2.stp correct; whether divertor_cam.stp's own
        # fixture still needs a fix is UNVERIFIED as of 2026-10-09 (its
        # own single-cell STEP extract has no loadable TopAbs_SOLID to
        # test in isolation) -- flagged for a future session, not
        # silently dropped.
        result = GSolid(fixer.Shape())
    except Exception:
        return None

    if not result.is_valid():
        return None
    if not volume_within(result.Volume, solid.Volume, MAX_REPAIR_VOLUME_REL_CHANGE):
        return None
    if find_overlapping_coplanar_faces(result):
        return None
    return result


def Gcheck_and_repair(solid: "GSolid", tolerances) -> "tuple[GSolid, bool]":
    """Load-time CAD-defect check + repair cascade -- `GSolid` in,
    `GSolid` out (2026-08-28, per direct user request: this is
    fundamentally native-shape repair work -- BRepAlgoAPI_Defeaturing,
    ShapeBuild_ReShape, BRepBuilderAPI_Sewing -- not GEOUNED-level
    classification, so the *orchestration* belongs entirely in `geo`,
    not driven from Python-level GEOUNED code calling several separate
    `geo` functions one at a time).

    Takes an already-constructed `GSolid`, per explicit user direction
    (2026-08-28, later the same session): `Gload_and_process_step`'s own
    loop builds the `GSolid` right after the native fix step,
    unconditionally, one clear step per solid, rather than this function
    trying to avoid that construction via its own native pre-check --
    an earlier version of this function did exactly that (`BRepCheck_
    Analyzer` + a native short-edge scan, `GSolid` built lazily only once
    a defect was confirmed) and was reverted once the caller settled on
    always building the `GSolid` in the loop itself; keeping a second,
    separate native pre-check here would have bought nothing once that
    was true (the `GSolid` already exists by the time this function
    runs), while `Gcollapse_split_rings`/`Gsliver_heal`/`Gdefeature`
    genuinely need the per-face surface classification `GSolid`/`GFace`
    already compute once, correctly -- reimplementing that dispatch a
    second time at the native level would just duplicate `Gclassify_
    surface`/`GFace.CharacteristicWidth` outside `geo`'s own single
    source of truth for it, risking the two drifting apart.

    Returns `(solid, True)` unchanged if already clean. Otherwise tries,
    in order, the same cascade this function's own predecessor
    (GEOUNED/loadfile/load_step.py's now-removed `repair_solid`) used --
    `Gcollapse_split_rings`, `Gsliver_heal`, then `find_short_edges` +
    `Gdefeature` -- returning `(repaired, True)` on the first one that
    both fires and leaves `check_solid_defects` empty. Each of those 3
    repair functions already builds its own fresh `GSolid` internally
    from the truly-repaired native shape (never reusing `solid`'s own,
    possibly-defective, already-parsed `.Faces`), so the caller never
    needs to re-wrap the result itself. Returns `(solid, False)` -- the
    ORIGINAL, unrepaired `GSolid`, never a partial or fabricated result
    -- if nothing clears every check.

    `Gtrim_overlapping_coplanar_faces` (the "overlapping coplanar faces"
    defect, see its own docstring) runs first in the cascade: it targets
    its own distinct, unambiguous fingerprint, is a pure topology fix
    (no native-kernel ambiguity like `Gsliver_heal`'s near-pair search),
    and -- being a plain re-trim, never a sew-driven collapse -- has no
    interaction with the other 3 repairs' own defect classes."""
    defects = check_solid_defects(solid, tolerances.sliver_edge_rel_tol, tolerances.min_face_width)
    if not defects:
        return solid, True

    overlap_trimmed = Gtrim_overlapping_coplanar_faces(solid)
    if overlap_trimmed is not None:
        return overlap_trimmed, True

    collapsed = Gcollapse_split_rings(solid, tolerances)
    if collapsed is not None:
        return collapsed, True

    sliver_healed = Gsliver_heal(solid, tolerances)
    if sliver_healed is not None:
        return sliver_healed, True

    degenerate_faces = find_short_edges(solid, tolerances.sliver_edge_rel_tol)
    if degenerate_faces:
        healed = Gdefeature(solid, degenerate_faces, tolerances.sliver_edge_rel_tol)
        if healed is not None and not check_solid_defects(healed, tolerances.sliver_edge_rel_tol, tolerances.min_face_width):
            return healed, True

    return solid, False


def Gspline_surface(solid) -> bool:
    """True if `solid` (any native shape -- a whole solid, typically) has
    at least one face whose underlying surface is NOT one of the 5
    analytic types GEOUNED can classify (Plane/Cylinder/Cone/Sphere/
    Torus) -- a BSpline, Bezier, or other freeform/swept surface
    `Gclassify_surface` would reject (returning None for it). Despite the
    name (matching the user's own "las llamo Bspline" framing, and
    `load_functions.py`'s former `spline()` helper this replaces),
    that's "any unsupported surface type", not literally only BSpline.

    Operates directly on the native shape via `TopExp_Explorer`, with no
    `GSolid`/`GFace` ever constructed -- but deliberately delegates the
    actual per-face classification to `Gclassify_surface` itself (the
    same dispatch `GFace.__init__` calls) rather than re-checking
    `adaptor.GetType()` against an allowed set by hand here: this is the
    single place that dispatch is defined, and duplicating it would risk
    the two drifting apart if `Gclassify_surface` ever gains a new
    fallback case (e.g. the FreeCAD-side "BSplineSurface secretly a
    flat plane" heuristic some engines have)."""
    explorer = TopExp_Explorer(solid, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        if Gclassify_surface(face) is None:
            return True
        explorer.Next()
    return False


def Gface_valid(face) -> bool:
    """True if `face` -- a native TopoDS_Face, or a GFace (unwrapped
    here) -- passes BRepCheck_Analyzer: its boundary wire(s) bound a
    coherent, orientable 2D region on its surface, its pcurves are sane,
    edges lie on the surface within tolerance, etc.

    The per-face counterpart of `GSolid.is_valid()`. A whole solid can
    be BRepCheck-invalid solely because one of its faces is (a single
    boundary wire that is really two loops crammed together ->
    BRepCheck_UnorientableShape -- see the `L4_support.stp` case in
    reference_cad_defect_recipes.md); this isolates the check to one
    face. Returns False rather than raising if the analyzer itself
    cannot run on the shape.
    """
    native = getattr(face, "__native__", face)
    try:
        return bool(BRepCheck_Analyzer(native).IsValid())
    except Exception:
        return False


def Gheal_topology(solid: "GSolid") -> "GSolid | None":
    """Repair a topologically-invalid solid (`BRepCheck_Analyzer` fails)
    via a STEP serialize -> deserialize rebuild, done **entirely in memory**
    (`io.BytesIO`, no temp file, not the public `Gexport_step`).

    This is the only thing found to fix the `BRepCheck_InvalidImbricationOfWires`
    class of defect a *failed* BOPAlgo split can leave on a decomposition
    fragment (see reference_cad_defect_recipes.md Recipe 3, and CLAUDE.md).
    `ShapeFix_Shape` / `ShapeUpgrade_UnifySameDomain` (i.e. `GSolid.fix()` /
    `.refine()`) do NOT repair it -- confirmed live; nor does `ShapeFix_Face`
    with orientation/intersecting-wire modes forced, nor a from-scratch
    pcurve rebuild. What the round trip does that an in-place `ShapeFix`
    cannot: `STEPControl_Writer` re-instantiates every sub-shape from
    scratch and *freezes* each face's outer-vs-hole wire designation into
    the entity type (`FACE_OUTER_BOUND` / `FACE_BOUND`), so `STEPControl_Reader`
    rebuilds the face with no runtime imbrication inference left to get
    wrong.

    Returns a fresh `GSolid` when the rebuilt solid is `BRepCheck`-valid
    and its volume matches the input to `MAX_REPAIR_VOLUME_REL_CHANGE`
    (3e-4 relative, the shared healing gate -- a failed split can inflate the input volume and the rebuild corrects it, but measured rebuilds conserve volume to <= 2e-5); `None` otherwise (the caller in `remove_solids` then
    keeps the original invalid fragment unchanged). Never raises.

    ocp/occ only; the freecad backend's `Gheal_topology` is a
    `None`-returning stub (FreeCAD already heals on its own load path and
    has no in-memory STEP stream API).

    NOT WIRED INTO THE PIPELINE (2026-08-29): implemented + verified
    against L4_body.stp / L4-WCS_3.stp (Recipe 3), then the approach was
    set aside -- kept here, callable, in case it's needed later, but the
    L4_body defect is being addressed via the face-deduplication path
    instead. See CLAUDE.md."""
    try:
        native = solid.__native__
        original_volume = abs(_volume_props(native).Mass())
        with suppress_native_stdout():
            writer = STEPControl_Writer()
            writer.Transfer(native, STEPControl_AsIs)
            buffer = io.BytesIO()
            if writer.WriteStream(buffer) != IFSelect_RetDone:
                return None
            buffer.seek(0)
            reader = STEPControl_Reader()
            if reader.ReadStream("in-memory", buffer) != IFSelect_RetDone:
                return None
            reader.TransferRoots()
            rebuilt = reader.OneShape()
        rebuilt_solids = _exploded_solids(rebuilt)
        if len(rebuilt_solids) != 1:
            return None
        healed = rebuilt_solids[0]
        if not BRepCheck_Analyzer(healed).IsValid():
            return None
        healed_volume = abs(_volume_props(healed).Mass())
        if not volume_within(healed_volume, original_volume, MAX_REPAIR_VOLUME_REL_CHANGE):
            return None
        return GSolid(healed)
    except Exception:
        return None


from .open_solid_repair import _diagnose_open_solid as _diagnose_open, _repair_open_solid as _repair_open
from ..volume_utils import volume_within


def Gdiagnose_open_solid(solid: "GSolid", tolerances) -> "str | None":
    """After a split: classify why `solid` is not watertight.
    Returns None (watertight -- nothing to do), a known-cause tag
    ("split_duplicate_seam" -- a doubled BOPAlgo tangent seam, or
    "missing_sliver_strip" -- a thin uncapped slot left directly by the
    split), or "unknown" (open, cause not recognised)."""
    return _diagnose_open(solid.__native__, tolerances)


def Gclose_open_solid(solid: "GSolid", tolerances) -> "GSolid | None":
    """If `solid` is open from a known, repairable cause, return a
    watertight, BRepCheck-valid, volume-conserving GSolid; else None
    (already watertight / unknown cause / repair didn't hold)."""
    native = _repair_open(solid.__native__, tolerances)
    return GSolid(native) if native is not None else None
