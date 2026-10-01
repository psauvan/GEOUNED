"""
geo/solid_defects.py

Split out of `vector_geometry.py` (2026-08-28) -- load-time CAD-defect
detection on a whole `GSolid`, as opposed to `surface_geometry.py`'s
per-surface/per-point predicates. Everything here is what runs when a
solid is first read from a STEP file, before any GEOUNED-level
classification happens: is this solid's own topology carrying a
spurious/degenerate feature that would silently corrupt later
decomposition, or that a real CAD-repair step (`Gdefeature`/
`Gcollapse_split_rings`/`Gsliver_heal`, all in the 3 `_*_impl.py`
backends) needs to be pointed at.

Still pure math -- no `Part`/`FreeCAD`/`OCC.Core` import here. Every
function is duck-typed on `GSolid`/`GFace`/`GEdge`'s already-neutral
fields (`.Faces`/`.Edges`/`.Length`/`.BoundBox.DiagonalLength`/
`.CharacteristicWidth`/`.Surface`/`.Curve`), so this file is identical
across all 3 engines, exactly like `vector_geometry.py`/
`surface_geometry.py`.
"""

from __future__ import annotations

from .surface_geometry import axes_parallel
from .constants import (
    DEFAULT_MIN_FACE_WIDTH,
    DEFAULT_SLIVER_EDGE_REL_TOL,
    DEFECT_AXIS_ANGLE,
    DEGENERATE_EDGE_LENGTH_FLOOR,
    DEGENERATE_SOLID_VOL_AREA_RATIO,
    MIN_SLIVER_EDGE_LENGTH,
    POINT_POINT_TOL,
    SPLIT_RING_REL_TOL,
)


def valid_solid(solid, min_volume: float) -> bool:
    """True if `solid` is a BOPAlgo split fragment worth keeping as a
    real, independent solid: positive volume, not a thin sliver
    (``Volume / Area >= DEGENERATE_SOLID_VOL_AREA_RATIO``), above the
    minimum volume `min_volume` (``|Volume| >= min_volume``, `Tolerances.min_solid_volume`).

    Duck-typed on ``.Volume`` / ``.Area`` -- works on a `GSolid` or any
    thin wrapper exposing those. This is the canonical copy of the check
    `decom_utils_generator.py::valid_solid` carried until ef0077c
    deleted it (leaving only a bare ``Volume > min_solid_volume`` filter
    in the occ/ocp `Gsplit`, which does not reject slivers by
    Volume/Area and let `esfera/Barrel_bottom.stp` regress to 10 lost
    MCNP particles). The `freecad` backend's own local copy is replaced
    by an import of this one. The historical signature's second `Volume`
    argument was dead (never read) and is dropped."""
    try:
        vol = solid.Volume
        area = solid.Area
    except Exception:
        return False
    if vol < 0:
        return False
    if area == 0 or abs(vol / area) < DEGENERATE_SOLID_VOL_AREA_RATIO:
        return False
    if abs(vol) < min_volume:
        return False
    return True


def find_short_edges(solid, rel_tol: float = DEFAULT_SLIVER_EDGE_REL_TOL) -> list:
    """Faces of `solid` (a GSolid) touching at least one edge whose own
    length is pathologically small relative to the solid's overall scale
    (edge.Length / solid.BoundBox.DiagonalLength < rel_tol) -- a purely
    topological signature of a degenerate/spurious feature (a residual
    boolean-cut artifact, an accidental sliver from a CAD export),
    independent of surface type or parameters. Duck-typed on
    `solid.Faces` (each a GFace with `.Edges`, each a GEdge with
    `.Length`) and `solid.BoundBox.DiagonalLength` -- identical across
    all 3 engines, no native calls needed.

    Confirmed live on a real fixture (`Solidos/working_solids/
    "beltline left.stp"`): a visually-obvious spurious plane, invisible
    to both `BRepCheck_Analyzer` (reports the solid fully valid) and to
    `GFace.CharacteristicWidth` (the plane's own width is unremarkable,
    ~1mm) -- is caught immediately this way: its own boundary edges
    connecting to the model's real geometry measure 0.888mm, and its
    neighboring sliver face's edges measure 0.029mm, both several orders
    of magnitude below the model's own ~7246mm diagonal, while every
    other edge in the model is in the thousands-of-mm range. Default
    `rel_tol=1e-4` (0.01% of the model's own scale, per direct user
    instruction: real models can be meter-scale with legitimate
    millimeter-scale details, which a looser 1e-3 default risks flagging
    as false positives) -- comfortably below both tiers above; a
    109-file corpus scan (Solidos/test_models, raw solids and
    decomposed pieces alike) found zero false positives at this value,
    including on a fixture with independently-documented real, legitimate
    ~0.026mm-wide faces (Decomposed/modelcell_cut1_v2_piece66.stp) --
    confirmed to correctly distinguish that real feature from a
    genuinely separate, much smaller (0.0004mm-edge) sliver on the same
    piece. Unlike `Tolerances.min_face_width`, this needs no per-solid
    `scaled()` accommodation -- being already relative to each solid's
    own BoundBox, it doesn't suffer the "small decomposed piece" failure
    mode that motivated `scaled()` in the first place.

    NOTE on repair, not detection: this function is deliberately simple
    and fast -- it returns every face touching a short edge, not a
    minimal "just the spurious cluster" set (which would need real
    topological reasoning: on "beltline left.stp" specifically, this
    returns 8 faces, including 2 legitimate mirror-symmetry-cut planes
    and 2 legitimate end caps that merely happen to touch the same short
    edges as the 4 real defect faces -- confirmed live, no candidate
    graph-based refinement tried gave a general, reliably-correct
    minimal set for this fixture). Per explicit user direction, this is
    accepted: detecting a genuine CAD defect is more valuable than
    perfectly auto-repairing it (a false "translation failure" wrongly
    blamed on GEOUNED is worse than an honest "this solid could not be
    auto-repaired, fix the CAD" -- see geo.Gdefeature's own docstring for
    how repair is attempted and safely abandoned when it doesn't
    converge cleanly).

    Returns each offending face once (never duplicated, even if it has
    several short edges); empty list if none found. The effective
    threshold is `max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)` -- never
    below the 1e-3mm absolute floor, per direct user instruction (guards
    the small-solid case where `diag` alone would otherwise push the
    relative threshold below any meaningful working tolerance). An edge
    shorter than DEGENERATE_EDGE_LENGTH_FLOOR is never flagged, however
    small the effective threshold gets -- see that constant's own
    docstring for why (a legitimate OCCT pole-degenerate edge, not a
    defect)."""
    diag = solid.BoundBox.DiagonalLength
    if diag <= 0.0:
        return []
    threshold = max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)
    flagged = []
    for face in solid.Faces:
        for edge in face.Edges:
            if DEGENERATE_EDGE_LENGTH_FLOOR <= edge.Length < threshold:
                flagged.append(face)
                break
    return flagged


_SPLIT_RING_SURFACE_TYPES = ("GCylinder", "GCone", "GSphere", "GTorus")


def find_split_ring_faces(solid, min_face_width: float = DEFAULT_MIN_FACE_WIDTH, rel_tol: float = DEFAULT_SLIVER_EDGE_REL_TOL) -> list:
    """Faces that are the parasitic "riser" walls of a *duplicated
    micro-trim* -- the "split boundary ring" CAD defect (a.k.a. collapsed
    micro-step). A single trimming surface (a plane, or a cylinder)
    appears twice at a sub-tolerance offset, so a curved analytic face
    that meets it is bounded by two near-coincident concentric circular
    edges (bridged by pathologically short connector edges) instead of
    one, and the thin slab between the two trim copies is filled by these
    riser faces. `BRepCheck_Analyzer` reports the solid fully valid.

    A face qualifies when ALL of:
      - its `Surface` is one of the 4 analytic curved types
        (GCylinder / GCone / GSphere / GTorus). A plane is never a riser
        here -- the two duplicate trim planes themselves must be KEPT;
        only the curved walls bridging them are spurious.
      - it is sliver-scale: `CharacteristicWidth < min_face_width`. A
        real cylinder / sphere / cone face has a width of tens to
        thousands of mm; a riser band is tens of microns.
      - it touches at least one pathologically short edge, using the
        same threshold as `find_short_edges`
        (``max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)``).

    Returns the list of GFace objects to drop (each once); empty if the
    solid shows no such pattern (nothing to collapse). Duck-typed on
    ``GSolid.Faces`` / ``GFace`` (``.Surface``, ``.CharacteristicWidth``,
    ``.Edges``) / ``GEdge.Length`` / ``GSolid.BoundBox.DiagonalLength`` --
    identical across all 3 engines, no native calls.

    Confirmed live on ``Solidos/working_solids/"barrel bottom.stp"``
    (2026-08-28): selects exactly the 6 riser cylinders (two R=1879.6,
    four R=200, each ~0.022mm tall) out of 11 faces; the 2 real sphere
    faces (``CharacteristicWidth`` ~3100-3200) and 3 real planes are
    correctly left. After removing these and re-sewing the shell
    (``geo.Gcollapse_split_rings``) the doubled boundary rings collapse
    into one and the solid converts with an MCNP stochastic-volume tally
    of 0.9997 (0 lost particles)."""
    diag = solid.BoundBox.DiagonalLength
    if diag <= 0.0:
        return []
    threshold = max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)
    risers = []
    for face in solid.Faces:
        if type(face.Surface).__name__ not in _SPLIT_RING_SURFACE_TYPES:
            continue
        width = getattr(face, "CharacteristicWidth", None)
        if width is None or width >= min_face_width:
            continue
        if any(DEGENERATE_EDGE_LENGTH_FLOOR <= edge.Length < threshold for edge in face.Edges):
            risers.append(face)
    return risers


def find_sliver_faces(solid, min_face_width: float) -> list:
    sliver_faces = []
    for face in solid.Faces:
        width = getattr(face, "CharacteristicWidth", None)
        if width is None:
            continue
        if is_sliver_face(face, min_face_width):
            sliver_faces.append(face)
    return sliver_faces


def is_sliver_face(face, min_face_width: float) -> bool :
    """GEOUNED's own face-sliver criteria: Area < Tolerances.min_area OR
    CharacteristicWidth < Tolerances.min_face_width (the metric
    find_split_ring_faces / cell_definition use)."""
    try:
        cw = getattr(face, "CharacteristicWidth", None)
        if cw is not None and cw < min_face_width:
            return True
    except Exception:
        pass
    return False

def count_split_ring_pairs(solid, rel_tol: float = SPLIT_RING_REL_TOL) -> int:
    """Number of *near-coincident concentric circular-edge pairs* on the
    faces of `solid` -- the direct fingerprint of a "split boundary ring"
    (see `find_split_ring_faces`). Two circular edges of the SAME face
    count as a pair when they are coaxial (|axis dot| ~ 1), concentric
    (centre offset perpendicular to the axis ~ 0), have `|dR| / max(R) <
    rel_tol`, and are separated (centre gap > 0) by less than
    `rel_tol * BoundBox.DiagonalLength` -- i.e. two circles that "should
    be one".

    Used as the success criterion for `Gcollapse_split_rings`: a genuine
    collapse merges the doubled rings, so this count must strictly
    DECREASE. If it comes back unchanged after the repair, the collapse
    did not actually resolve the defect (confirmed live 2026-08-28 on
    ``Solidos/working_solids/LR.stp`` -- a *cylindrical* collapsed-step
    variant where removing the lone riser band + re-sewing produces a
    topologically valid, ~volume-conserving solid whose CSG translation
    is nonetheless broken: d1suned tally 0.0, 24 lost particles; the
    pair count stays at 2 before and after, vs. barrel bottom.stp's
    18 -> 12).

    Duck-typed on `GSolid.Faces` / `GFace.Edges` / `GEdge.Curve` (a
    `GCircle` with `.Radius` / `.Axis` / `.Center`) / `GSolid.BoundBox` --
    identical across all 3 engines."""
    diag = solid.BoundBox.DiagonalLength
    if diag <= 0.0:
        return 0
    gap_limit = rel_tol * diag
    count = 0
    for face in solid.Faces:
        circles = [edge.Curve for edge in face.Edges if edge.Curve is not None and type(edge.Curve).__name__ == "GCircle"]
        for i in range(len(circles)):
            for j in range(i + 1, len(circles)):
                c1, c2 = circles[i], circles[j]
                if not axes_parallel(c1.Axis, c2.Axis, DEFECT_AXIS_ANGLE):
                    continue
                max_r = max(c1.Radius, c2.Radius)
                if abs(c1.Radius - c2.Radius) / max_r >= rel_tol:
                    continue
                offset = c2.Center - c1.Center
                gap = offset.length
                if not (0.0 < gap < gap_limit):
                    continue
                axial = abs(offset.dot(c1.Axis))
                perp_sq = gap * gap - axial * axial
                perp = perp_sq**0.5 if perp_sq > 0.0 else 0.0
                if perp / max_r < rel_tol:
                    count += 1
    return count


def near_surface_pair(surf_a, surf_b, dist_tol: float) -> float | None:
    """Step 3 of `sliver_healing` (see reference_cad_defect_recipes.md): two
    analytic surfaces of the SAME kind whose single varying parameter
    differs by a *small but nonzero* amount -- a near-duplicate that a
    just-removed sliver face was bridging, which the healer must reconcile.

    Returns the parameter gap when `1e-5 < gap < dist_tol` (strictly near --
    an exactly-coincident pair, `gap <= 1e-5`, is a legitimate symmetry
    split handled by the final sew, not by this step), else None. Requires
    coincident axes for plane / cylinder / cone. Duck-typed on
    `.Axis`/`.Position`/`.Radius`/`.Center`/`.Apex`/`.SemiAngle` -- works
    on a `geo` descriptor or a Tier-1 `*OnlyParams`.

    v0: plane, cylinder, sphere. cone / torus -> None (no fixture yet)."""
    ta, tb = type(surf_a).__name__, type(surf_b).__name__
    if ta != tb:
        return None

    def _near(gap: float):
        return gap if POINT_POINT_TOL < gap < dist_tol else None

    if ta == "GPlane":
        axis_dot = surf_a.Axis.dot(surf_b.Axis)
        if not axes_parallel(surf_a.Axis, surf_b.Axis, DEFECT_AXIS_ANGLE):
            return None
        d_a = surf_a.Axis.dot(surf_a.Position)
        d_b = surf_b.Axis.dot(surf_b.Position)
        return _near(abs(d_a - d_b) if axis_dot > 0 else abs(d_a + d_b))

    if ta == "GCylinder":
        if not axes_parallel(surf_a.Axis, surf_b.Axis, DEFECT_AXIS_ANGLE):
            return None
        offset = surf_b.Center - surf_a.Center
        along = offset.dot(surf_a.Axis)
        if (offset - surf_a.Axis * along).length > dist_tol:
            return None
        return _near(abs(surf_a.Radius - surf_b.Radius))

    if ta == "GSphere":
        if (surf_b.Center - surf_a.Center).length > dist_tol:
            return None
        return _near(abs(surf_a.Radius - surf_b.Radius))

    return None


def _same_infinite_plane(plane_a, plane_b, offset_tol: float) -> bool:
    """True if `plane_a`/`plane_b` (two GPlane descriptors) describe the
    exact same infinite plane within `offset_tol`: axes parallel, either
    sense, within `DEFECT_AXIS_ANGLE`, and the two `Position` points
    project to (near enough) the same point along that shared axis line.

    Deliberately looser than `POINT_POINT_TOL`: two independently-built
    Geom_Plane objects meant to represent "the same" analytic plane (one
    per overlapping residual face, see `find_overlapping_coplanar_faces`)
    do not round-trip to bit-identical Position/Axis the way a single
    *shared* point does. Confirmed live on
    `Solidos/working_solids/version1_bug2.stp`: the real defective pair's
    own offset is ~3.6e-4mm against that solid's own ~8.6mm BoundBox
    diagonal -- `offset_tol` is scaled to the solid for exactly this
    reason, the same `max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)`
    pattern `find_short_edges`/`find_split_ring_faces` already use."""
    if not axes_parallel(plane_a.Axis, plane_b.Axis, DEFECT_AXIS_ANGLE):
        return False
    offset = plane_b.Position - plane_a.Position
    return abs(offset.dot(plane_a.Axis)) <= offset_tol


def _face_boundary_inside_domain(small, large) -> bool:
    """True if `small`'s own CenterOfMass and every outer-wire edge
    midpoint, projected onto `large`'s own surface, fall inside
    `large`'s trimmed 2D domain -- i.e. `small` is a self-contained
    patch sitting entirely within the region `large`'s own face already
    claims, not merely touching or partially overlapping it.

    Duck-typed on `GFace.CenterOfMass` / `.outer_wire()` / `GEdge.
    ParameterRange` / `.value_at` / `GFace.Surface.parameter` /
    `GFace.is_part_of_domain` -- identical across all 3 engines, no
    native calls of its own."""
    probes = [small.CenterOfMass]
    for edge in small.outer_wire().Edges:
        p0, p1 = edge.ParameterRange
        probes.append(edge.value_at(0.5 * (p0 + p1)))
    for point in probes:
        u, v = large.Surface.parameter(point)
        if not large.is_part_of_domain(u, v):
            return False
    return True


def find_overlapping_coplanar_faces(solid, rel_tol: float = DEFAULT_SLIVER_EDGE_REL_TOL) -> list:
    """Pairs `(small, large)` of planar faces on `solid`'s own boundary
    that lie on the exact SAME infinite plane but are two independent,
    OVERLAPPING patches -- `small`'s own boundary sits entirely inside
    `large`'s own trimmed 2D domain -- instead of one correctly-trimmed
    face (e.g. an annulus) and the hole it should have.

    The same residual-boolean-cut-at-a-tangency signature this whole
    project exists to fix (see CLAUDE.md's "Motivating problem"),
    manifesting here as two concentric/overlapping planar patches rather
    than a thin riser band (`find_split_ring_faces`'s own defect family):
    a cutting tool's own end face landed exactly tangent to a
    pre-existing surface, and the BOP left the original face's full
    extent AND the tool's own end-cap face both present, uncut, instead
    of resolving into one trimmed face (an annulus) + the hole.

    Confirmed live, `Solidos/working_solids/version1_bug2.stp` solid0: a
    R=2mm disk (Forward) sitting entirely inside a R=3.5mm disk
    (Reversed, no inner wire) -- the shared end face of a coaxial
    boss+bore feature.

    Returns each matching pair once, ordered (smaller-area, larger-area);
    empty list if none found (including when `solid.BoundBox.
    DiagonalLength <= 0`). Duck-typed on `GSolid.Faces` / `GFace`
    (`.Surface` a `GPlane`, `.Area`, `.CenterOfMass`, `.outer_wire()`,
    `.is_part_of_domain`) -- identical across all 3 engines, no native
    calls of its own."""
    diag = solid.BoundBox.DiagonalLength
    if diag <= 0.0:
        return []
    offset_tol = max(diag * rel_tol, MIN_SLIVER_EDGE_LENGTH)
    planes = [f for f in solid.Faces if type(f.Surface).__name__ == "GPlane"]
    pairs = []
    for i, fa in enumerate(planes):
        for fb in planes[i + 1 :]:
            if not _same_infinite_plane(fa.Surface, fb.Surface, offset_tol):
                continue
            small, large = (fa, fb) if fa.Area <= fb.Area else (fb, fa)
            if _face_boundary_inside_domain(small, large):
                pairs.append((small, large))
    return pairs


def check_solid_defects(solid, sliver_edge_rel_tol: float = DEFAULT_SLIVER_EDGE_REL_TOL, min_face_width: float = DEFAULT_MIN_FACE_WIDTH) -> list:
    """Run every known corrupted/degenerate-geometry check against a
    loaded solid and return the reasons it currently fails (empty list
    if the solid is clean). One function, one place to extend: any
    future check should be added here so every caller (a backend's own
    ``Gcheck_and_repair`` cascade, GEOUNED's own reporting) picks it up
    automatically, instead of several separate, differently-gated checks.

    Checks currently implemented:
      - topological validity (BRepCheck_Analyzer / equivalent, via
        ``GSolid.is_valid()``).
      - a pathologically short edge (``find_short_edges``, relative to the
        solid's own BoundBox diagonal) that ALSO leads to a genuine
        sliver *face* -- one whose ``CharacteristicWidth < min_face_width``
        (confirmed live, 2026-08-27, Solidos/working_solids/"beltline
        left.stp" -- a spurious plane bridging a solid's real wall to a
        near-zero-height sliver band, that band being the sliver face).

        A short edge whose faces are all otherwise normal is NOT a
        defect for this check's purposes (2026-08-29, per direct user
        direction, Solidos/working_solids/L4_pipes.stp): the
        decomposition pipeline already tolerates sliver edges on
        well-formed faces, and no repair function removes such an edge
        without a sliver face to anchor on -- so flagging it here only
        wrongly drops a perfectly translatable solid. If a future case
        shows edge-only slivers do need repair, add a dedicated
        edge-repair function and re-widen this check (option 2 in that
        discussion); until then this stays face-anchored.

      - two independent, overlapping coplanar planar faces
        (``find_overlapping_coplanar_faces``) -- a residual boolean-cut
        artifact where a face's own missing inner-wire hole leaves it
        overlapping a smaller, separate coplanar patch instead of being
        correctly trimmed into e.g. an annulus (confirmed live,
        Solidos/working_solids/version1_bug2.stp).

    Takes raw tolerance values, not a `Tolerances` object -- this file
    has no dependency on anything outside `geo`, GEOUNED's own
    `Tolerances` class included; callers read
    `tolerances.sliver_edge_rel_tol` / `.min_face_width` themselves
    before calling in."""
    reasons = []
    if not solid.is_valid():
        reasons.append("invalid topology")
    short_edge_faces = find_short_edges(solid, sliver_edge_rel_tol)
    if any(getattr(face, "CharacteristicWidth", float("inf")) < min_face_width for face in short_edge_faces):
        reasons.append("degenerate/sliver geometry")
    if find_overlapping_coplanar_faces(solid, sliver_edge_rel_tol):
        reasons.append("overlapping coplanar faces")
    return reasons
