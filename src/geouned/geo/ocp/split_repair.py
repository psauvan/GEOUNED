"""
geo/ocp/split_repair.py

Non-manifold-solid repair for a raw BOPAlgo_Splitter result: face-
adjacency-graph reconstruction (_repair_non_manifold_solid) and the
"phantom cut" edge-joined-components separation
(_separate_edge_joined_components). Used only by split.py's own
_raw_bop_split.
"""

from __future__ import annotations

from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_Copy,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
)
from OCP.TopExp import (
    TopExp,
    TopExp_Explorer,
)
from OCP.TopoDS import (
    TopoDS,
    TopoDS_Shell,
)
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from ..constants import MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE
from ._native_utils import _volume_props


def _edge_face_map(native_solid) -> TopTools_IndexedDataMapOfShapeListOfShape:
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(native_solid, TopAbs_EDGE, TopAbs_FACE, m)
    return m


def _repair_non_manifold_solid(native_solid, fix_tolerance: float = 1e-6) -> list:
    """Attempt to split a non-manifold TopoDS_Solid (confirmed invalid
    via BRepCheck_Analyzer) into its real connected components: build a
    face-adjacency graph over the solid's own faces, excluding edges
    shared by != 2 faces, find connected components via union-find, and
    for each component missing a proper boundary at a non-manifold edge,
    duplicate the real face found there (via BRepBuilderAPI_Copy) so
    both sides get their own capping copy. Returns a list of native
    TopoDS_Solid -- may be a single-element list containing the
    original, unrepaired solid if reconstruction doesn't succeed.

    `fix_tolerance` parameterizes the BRepBuilderAPI_Sewing tolerance
    used when re-sewing each component's own faces (2026-08-30 --
    previously a hardcoded 1e-6, now sourced from Tolerances.fix_tolerance
    at this function's own only call site, same default value)."""
    faces = []
    explorer = TopExp_Explorer(native_solid, TopAbs_FACE)
    while explorer.More():
        faces.append(TopoDS.Face(explorer.Current()))
        explorer.Next()
    n = len(faces)

    edge_map = _edge_face_map(native_solid)
    non_manifold_edge_keys = {i for i in range(1, edge_map.Extent() + 1) if edge_map.FindFromIndex(i).Size() != 2}

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

    for i in range(1, edge_map.Extent() + 1):
        if i in non_manifold_edge_keys:
            continue
        face_list = edge_map.FindFromIndex(i)
        if face_list.Size() == 2:
            it = iter(face_list)
            f1 = next(it)
            f2 = next(it)
            i1, i2 = face_index(f1), face_index(f2)
            if i1 is not None and i2 is not None:
                union(i1, i2)

    components: dict[int, list] = {}
    for i in range(n):
        components.setdefault(find(i), []).append(i)

    if len(components) < 2:
        return [native_solid]

    comp_extra_faces: dict[int, list] = {root: [] for root in components}
    for i in non_manifold_edge_keys:
        face_list = edge_map.FindFromIndex(i)
        by_component: dict[int, list] = {}
        for f in face_list:
            idx = face_index(f)
            if idx is None:
                continue
            by_component.setdefault(find(idx), []).append(f)
        for root in components:
            if root not in by_component:
                donor_root, donor_faces = next(iter(by_component.items()))
                comp_extra_faces.setdefault(root, []).append(donor_faces[0])

    results = []
    for root, idxs in components.items():
        comp_faces = [faces[i] for i in idxs]
        for donor_face in comp_extra_faces.get(root, []):
            comp_faces.append(BRepBuilderAPI_Copy(donor_face).Shape())

        sewer = BRepBuilderAPI_Sewing(fix_tolerance)
        for f in comp_faces:
            sewer.Add(f)
        sewer.Perform()
        sewed = sewer.SewedShape()

        builder = BRep_Builder()
        shell_explorer = TopExp_Explorer(sewed, TopAbs_FACE)
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        seen_any = False
        while shell_explorer.More():
            builder.Add(shell, TopoDS.Face(shell_explorer.Current()))
            seen_any = True
            shell_explorer.Next()
        if not seen_any:
            continue

        try:
            solid_maker = BRepBuilderAPI_MakeSolid(shell)
            if solid_maker.IsDone():
                results.append(solid_maker.Solid())
        except Exception:
            pass

    if not results:
        return [native_solid]
    return results


def _separate_edge_joined_components(native_solid) -> "list | None":
    """The "phantom cut" ("corte fantasma") case -- see CLAUDE.md's
    dedicated section. After a BOPAlgo split, two (or more) regions that
    touch **only along edges** (zero-area contact) come back fused into a
    single TopoDS_Solid, with the seam appearing as non-manifold edges
    (each shared by 4 faces -- 2 per side, since BOPAlgo already gave each
    region its own copy of the cut surface). GEOUNED needs them returned
    as separate solids: an edge is not volume.

    Detection is topological, not a heuristic: build the face-adjacency
    graph from **manifold edges only** (shared by exactly 2 faces). If it
    has >= 2 components AND the full graph (all edges) has fewer -- i.e.
    the non-manifold edges are the *only* thing joining the components --
    the solid is really N regions with zero-area contact. A clean solid,
    or a plain solid-with-cavity, has no non-manifold edges at all and is
    rejected immediately.

    Unlike `_repair_non_manifold_solid`, this adds **no donor/capping
    faces** -- a true phantom cut already carries each region's full
    closed boundary. Each manifold-only component's faces are sewn
    (BRepBuilderAPI_Sewing) and made into a solid. Accepts the split only
    if every piece is an individually valid solid and the summed volume
    matches the input to 1e-6 relative (same safety net as
    `_raw_bop_split`'s own repair gate). Returns the list of native
    TopoDS_Solid on success, or None to fall through to the existing
    valid/heal/reject path.

    Verified on `Solidos/working_solids/Piece_1.stp` (cutting plane) +
    `Tool_1.stp` (piece): 2 valid pieces, summed volume matching the
    fused input to ~4e-9 relative (vs. `_repair_non_manifold_solid`'s
    donor-face reconstruction here: +0.5 % volume, one piece still
    invalid).
    """
    faces = []
    explorer = TopExp_Explorer(native_solid, TopAbs_FACE)
    while explorer.More():
        faces.append(TopoDS.Face(explorer.Current()))
        explorer.Next()
    n = len(faces)
    if n < 2:
        return None

    edge_map = _edge_face_map(native_solid)
    non_manifold_edge_keys = {i for i in range(1, edge_map.Extent() + 1) if edge_map.FindFromIndex(i).Size() != 2}
    if not non_manifold_edge_keys:
        return None

    def face_index(face):
        for i, f in enumerate(faces):
            if f.IsSame(face):
                return i
        return None

    def components(include_non_manifold: bool) -> dict:
        parent = list(range(n))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for i in range(1, edge_map.Extent() + 1):
            if not include_non_manifold and i in non_manifold_edge_keys:
                continue
            idxs = [face_index(f) for f in edge_map.FindFromIndex(i)]
            idxs = [x for x in idxs if x is not None]
            for k in range(1, len(idxs)):
                a, b = find(idxs[0]), find(idxs[k])
                if a != b:
                    parent[a] = b

        groups: dict[int, list] = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(i)
        return groups

    manifold_components = components(include_non_manifold=False)
    if len(manifold_components) < 2:
        return None
    if len(manifold_components) <= len(components(include_non_manifold=True)):
        # the non-manifold edges are not the load-bearing connection
        # (e.g. an internal cavity component, or genuinely disjoint shells)
        return None

    pieces = []
    for idxs in manifold_components.values():
        sewer = BRepBuilderAPI_Sewing(1e-6)
        for i in idxs:
            sewer.Add(faces[i])
        sewer.Perform()
        builder = BRep_Builder()
        shell = TopoDS_Shell()
        builder.MakeShell(shell)
        face_explorer = TopExp_Explorer(sewer.SewedShape(), TopAbs_FACE)
        seen = False
        while face_explorer.More():
            builder.Add(shell, TopoDS.Face(face_explorer.Current()))
            seen = True
            face_explorer.Next()
        if not seen:
            return None
        try:
            solid_maker = BRepBuilderAPI_MakeSolid(shell)
        except Exception:
            return None
        if not solid_maker.IsDone():
            return None
        pieces.append(solid_maker.Solid())

    if len(pieces) < 2:
        return None
    if not all(BRepCheck_Analyzer(p).IsValid() for p in pieces):
        return None
    original_volume = abs(_volume_props(native_solid).Mass())
    summed_volume = sum(abs(_volume_props(p).Mass()) for p in pieces)
    if abs(summed_volume - original_volume) > MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE * max(original_volume, 1.0):
        return None
    return pieces
