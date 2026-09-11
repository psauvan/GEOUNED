"""
geo/occ/repair.py

Load-time CAD-defect repair: Gdefeature, Gcollapse_split_rings,
Gsliver_heal, Gcheck_and_repair, Gspline_surface, Gheal_topology.
Assembled from 3 non-contiguous ranges of the original file (Gspline_
surface physically sat in the I/O section, Gheal_topology physically
sat in the boolean/split section) -- grouped here by what they do,
not where they happened to be written.
"""

from __future__ import annotations

from OCC.Core.BRep import (
    BRep_Builder,
    BRep_Tool,
)
from OCC.Core.BRepAdaptor import (
    BRepAdaptor_Curve,
    BRepAdaptor_Surface,
)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Defeaturing
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepTools import breptools
from OCC.Core.GeomAbs import (
    GeomAbs_Circle,
    GeomAbs_Cone,
    GeomAbs_Cylinder,
    GeomAbs_Line,
    GeomAbs_Plane,
    GeomAbs_Sphere,
    GeomAbs_Torus,
)
from OCC.Core.gp import (
    gp_Ax2,
    gp_Circ,
    gp_Dir,
    gp_Pln,
    gp_Pnt,
)
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.ShapeBuild import ShapeBuild_ReShape
from OCC.Core.ShapeFix import ShapeFix_Shape
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.STEPControl import (
    STEPControl_AsIs,
    STEPControl_Reader,
    STEPControl_Writer,
)
from OCC.Core.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_REVERSED,
    TopAbs_VERTEX,
)
from OCC.Core.TopExp import (
    TopExp_Explorer,
    topexp,
)
from OCC.Core.TopoDS import (
    TopoDS_Shell,
    topods,
)
from OCC.Core.TopTools import (
    TopTools_IndexedDataMapOfShapeListOfShape,
    TopTools_ListOfShape,
)
from ..vector_geometry import GVector
from ..solid_defects import (
    check_solid_defects,
    count_split_ring_pairs,
    find_sliver_faces,
    find_short_edges,
    find_split_ring_faces,
    near_surface_pair,
)
from ..constants import (
    MAX_DEFEATURE_VOLUME_REL_CHANGE,
    MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE,
    MAX_SLIVER_HEAL_VOLUME_REL_CHANGE,
    MAX_SPLIT_RING_VOLUME_REL_CHANGE,
    MIN_SLIVER_EDGE_LENGTH,
    OCCT_FIX_TOLERANCE,
)
from ..io_utils import suppress_native_stdout
from ..surface_geometry import is_same_plane_surface
from .topology import GFace, GPlane, GSolid, Gclassify_surface
from .boolean import _exploded_solids
from ._native_utils import _volume_props


def Gdefeature(solid: "GSolid", faces: "list[GFace]", sliver_edge_rel_tol) -> "GSolid | None":
    """Attempt to remove `faces` (typically solid_defects.find_short_edges'
    own output) from `solid` via BRepAlgoAPI_Defeaturing, verifying the
    result is genuinely usable before trusting it -- see _ocp_impl.py's
    own identical docstring for the full story (confirmed live,
    2026-08-27, Solidos/working_solids/"beltline left.stp": IsDone()==True
    alone is not enough, it can return a topologically valid but
    silently wrong solid).

    Deliberately a single, fast, one-shot attempt, not an iterative or
    graph-based search for the "correct" minimal face set -- per
    explicit user direction, detecting a genuine CAD defect matters more
    than perfectly auto-repairing it, and any repair kept here must stay
    general and fast. Returning None and letting the caller fall back to
    "flag as corrupted, don't convert" is the intended outcome when this
    single attempt doesn't cleanly resolve the defect -- not a gap to
    close with a more elaborate search later.

    Returns None whenever defeaturing doesn't complete, the healed
    result isn't topologically valid, find_short_edges() still finds a
    short edge in it, or its own Volume has drifted from the input by
    more than MAX_DEFEATURE_VOLUME_REL_CHANGE -- never raises."""
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
    if abs(healed.Volume - solid.Volume) > MAX_DEFEATURE_VOLUME_REL_CHANGE * max(abs(solid.Volume), 1.0):
        return None
    return healed


def Gcollapse_split_rings(solid: "GSolid", min_face_width: float = 0.1) -> "GSolid | None":
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
      - ``|dV| / max(|V|, 1) < MAX_SPLIT_RING_VOLUME_REL_CHANGE`` (3e-4) --
        a real collapse only removes sliver-volume risers (barrel
        bottom.stp: 6.4e-5); a larger drift means the sew moved a real
        adjacent surface (LR.stp: valid, dV 7e-4, CSG-broken -- d1suned
        tally 0.0 / 24 lost particles);
      - ``count_split_ring_pairs`` strictly DECREASED -- the direct
        success test, "did the doubled boundary rings actually merge".
        barrel bottom.stp: 18 -> 12 (pass). LR.stp: 2 -> 2 (fail -- a
        *cylindrical* collapsed-step where sew-collapse leaves the
        fingerprint intact; the beltline-family case CLAUDE.md documents
        as unsolved -- correctly rejected).

    Residual sub-sew-tolerance connector edges may remain on the (kept)
    faces -- an internal-wire micro-tab sewing cannot weld -- and are
    HARMLESS: on ``barrel bottom.stp`` the repaired solid keeps 2 faces
    with ~0.029mm edges yet converts with a tally of 0.9997 +/- 0.27% and
    0 lost particles. So acceptance does NOT require `find_short_edges`
    to come back empty.

    NOT attempted: ``ShapeFix_Wireframe.FixSmallEdges`` clears the
    residual edges but reshapes the trimmed sphere boundary -> ~0.22%
    volume drift on a STEP round-trip. It fails the guard and is
    deliberately left out.

    ocp/occ only -- ``ShapeBuild_ReShape`` + ``BRepBuilderAPI_Sewing`` is
    the pipeline used; the freecad backend's `Gcollapse_split_rings` is a
    None-returning stub. Never raises."""
    risers = find_split_ring_faces(solid, min_face_width)
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
            builder.Add(shell, topods.Face(face_explorer.Current()))
            seen = True
            face_explorer.Next()
        if not seen:
            return None

        solid_maker = BRepBuilderAPI_MakeSolid(shell)
        if not solid_maker.IsDone():
            return None

        fixer = ShapeFix_Shape(solid_maker.Solid())
        fixer.SetPrecision(sew_tol)
        fixer.Perform()
        result = GSolid(fixer.Shape())
    except Exception:
        return None

    if not result.is_valid():
        return None
    if abs(result.Volume - solid.Volume) > MAX_SPLIT_RING_VOLUME_REL_CHANGE * max(abs(solid.Volume), 1.0):
        return None
    if count_split_ring_pairs(result) >= pairs_before:
        return None
    return result


def _edge_endpoints(native_edge):
    pts = []
    vexp = TopExp_Explorer(native_edge, TopAbs_VERTEX)
    while vexp.More():
        pts.append(BRep_Tool.Pnt(topods.Vertex(vexp.Current())))
        vexp.Next()
    return pts


def _free_edges(shape):
    edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
    return [
        topods.Edge(edge_face_map.FindKey(i))
        for i in range(1, edge_face_map.Size() + 1)
        if edge_face_map.FindFromIndex(i).Size() == 1
    ]


def _retrim_freed_quadrics(reduced_shape, drop_plane, keep_plane, tol):
    """See _ocp_impl.py::_retrim_freed_quadrics -- re-trim the V range of
    any cylinder/cone face whose free rim landed on the dropped plane so
    it lands on the kept plane instead (re-trim of an unbounded quadric,
    not surface extension). Returns the (possibly unchanged) shape."""
    axis = keep_plane.Axis
    drop_offset = axis.dot(drop_plane.Position)
    keep_offset = axis.dot(keep_plane.Position)
    free = _free_edges(reduced_shape)

    reshaper = ShapeBuild_ReShape()
    changed = 0
    face_explorer = TopExp_Explorer(reduced_shape, TopAbs_FACE)
    while face_explorer.More():
        face = topods.Face(face_explorer.Current())
        face_explorer.Next()
        surf_adaptor = BRepAdaptor_Surface(face, True)
        if surf_adaptor.GetType() not in (GeomAbs_Cylinder, GeomAbs_Cone):
            continue
        rim_on_drop = False
        edge_explorer = TopExp_Explorer(face, TopAbs_EDGE)
        while edge_explorer.More():
            edge = topods.Edge(edge_explorer.Current())
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

        surface = BRep_Tool.Surface(face)
        u1, u2, v1, v2 = breptools.UVBounds(face)
        u_mid = 0.5 * (u1 + u2)

        def z_at(v):
            p = surface.Value(u_mid, v)
            return axis.dot(GVector(p.X(), p.Y(), p.Z()))

        z1, z2 = z_at(v1), z_at(v2)
        if abs(z1 - drop_offset) <= abs(z2 - drop_offset):
            slope = (z_at(v1 + 1e-3) - z1) / 1e-3
            if abs(slope) < 1e-6:
                continue
            v1 = v1 + (keep_offset - z1) / slope
        else:
            slope = (z_at(v2 + 1e-3) - z2) / 1e-3
            if abs(slope) < 1e-6:
                continue
            v2 = v2 + (keep_offset - z2) / slope
        if v1 >= v2:
            continue
        new_face = BRepBuilderAPI_MakeFace(surface, u1, u2, v1, v2, tol)
        if not new_face.IsDone():
            continue
        reshaper.Replace(face, new_face.Face())
        changed += 1

    return reshaper.Apply(reduced_shape) if changed else reduced_shape


def _snapped_planar_cap(reduced_shape, keep_plane):
    """See _ocp_impl.py::_snapped_planar_cap. Returns [TopoDS_Face], []
    (nothing open), or None (not a single planar loop v0 can cap)."""
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
        if a.Distance(b) < 1e-7:
            continue
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
    """`sliver_healing` (version 0). See _ocp_impl.py::Gsliver_heal for the
    full algorithm/spec and the verified `LR.stp` fixture. ocp/occ only;
    the freecad backend's `Gsliver_heal` is a `None`-returning stub.

    `tolerances` supplies `min_face_width` (which faces count as slivers)
    and `sliver_edge_rel_tol` (the near-surface-pair distance tolerance,
    scaled by the solid's own BoundBox diagonal) -- previously hardcoded
    (0.1 / 1e-4), now sourced from Tolerances at this function's only
    call site (_raw_bop_split), same default values.

    2026-08-30: a same-session attempt to generalize this to "every
    near-surface pair found, not just a single one" (keeping whichever
    face of each pair BRepCheck_Analyzer reports individually valid) was
    reverted, then re-corrected -- confirmed via a full Solidos/
    test_models d1suned batch that the original attempt's own keep/drop
    criterion silently corrupted several already-fixed fixtures (barrel
    bottom, rev_pipe x2, SCDR_90_hollow, SCDR_90). Root cause: a single
    face plucked from an already-valid solid is essentially always
    individually "valid" on its own, so that criterion never actually
    discriminated which face to keep; and the loop's `_retrim_freed_
    quadrics`/`_snapped_planar_cap` calls only ever used the *last*
    pair's own drop/keep faces, silently ignoring every earlier pair's
    removal.

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
            builder.Add(shell, topods.Face(face_explorer.Current()))
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
        unify = ShapeUpgrade_UnifySameDomain(fixer.Shape(), True, True, True)
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
    if abs(result.Volume - solid.Volume) > MAX_SLIVER_HEAL_VOLUME_REL_CHANGE * max(abs(solid.Volume), 1.0):
        return None
    return result


def _edge_vertices(native_edge) -> list:
    vs = []
    ve = TopExp_Explorer(native_edge, TopAbs_VERTEX)
    while ve.More():
        vs.append(topods.Vertex(ve.Current()))
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
    brepbndlib.Add(native_wire, box)
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
            e = topods.Edge(ee.Current())
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
        new_face = topods.Face(new_face.Reversed())
    return new_face


def Gmerge_coplanar_planes(solid: "GSolid") -> "GSolid":
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
            faces.append(topods.Face(exp.Current()))
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
        topexp.MapShapesAndAncestors(native, TopAbs_EDGE, TopAbs_FACE, edge_map)
        for k in range(1, edge_map.Size() + 1):
            face_list = edge_map.FindFromIndex(k)
            if face_list.Size() != 2:
                continue
            pair = list(face_list)
            a = face_index(pair[0])
            b = face_index(pair[1])
            if a is None or b is None or planes[a] is None or planes[b] is None:
                continue
            if is_same_plane_surface(planes[a], planes[b]):
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
            builder.Add(shell, topods.Face(se.Current()))
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
    if abs(result.Volume - solid.Volume) > 1e-6 * max(abs(solid.Volume), 1.0):
        return solid
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
    -- if nothing clears every check."""
    if not check_solid_defects(solid, tolerances.sliver_edge_rel_tol, tolerances.min_face_width):
        return solid, True

    collapsed = Gcollapse_split_rings(solid, tolerances.min_face_width)
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


_ALLOWED_SURFACE_TYPES = (GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_Torus)


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
    the two drifting apart."""
    explorer = TopExp_Explorer(solid, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
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
    (no temp file, not the public `Gexport_step`).

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
    and its volume matches the input to `MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE`
    (1e-3 relative -- the failed split can inflate the input volume and the
    rebuild corrects it, so this is deliberately looser than the other
    heal gates); `None` otherwise (the caller in `remove_solids` then
    keeps the original invalid fragment unchanged). Never raises.

    ocp/occ only; the freecad backend's `Gheal_topology` is a
    `None`-returning stub (FreeCAD already heals on its own load path and
    has no in-memory STEP stream API). NOTE: pythonocc-core's
    `WriteStream()` takes no argument and returns a `(status, text)`
    tuple (a Python string), where OCP's takes an `io.BytesIO` -- this
    file uses the pythonocc-core form.

    NOT WIRED INTO THE PIPELINE (2026-08-29): this was implemented +
    verified against L4_body.stp / L4-WCS_3.stp (Recipe 3), then the
    approach was set aside -- kept here, callable, in case it's needed
    later, but the L4_body defect is being addressed via the
    face-deduplication path instead. See CLAUDE.md."""
    try:
        native = solid.__native__
        original_volume = abs(_volume_props(native).Mass())
        with suppress_native_stdout():
            writer = STEPControl_Writer()
            writer.Transfer(native, STEPControl_AsIs)
            status, step_text = writer.WriteStream()
            if status != IFSelect_RetDone:
                return None
            reader = STEPControl_Reader()
            if reader.ReadStream("in-memory", step_text) != IFSelect_RetDone:
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
        if abs(healed_volume - original_volume) > MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE * max(original_volume, 1.0):
            return None
        return GSolid(healed)
    except Exception:
        return None


from .open_solid_repair import _diagnose_open_solid as _diagnose_open, _repair_open_solid as _repair_open


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
