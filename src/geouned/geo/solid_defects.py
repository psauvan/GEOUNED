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

MIN_SLIVER_EDGE_LENGTH = 1.0e-3
"""Absolute floor (mm) for find_short_edges' own threshold -- per direct
user instruction: the effective threshold must never drop below this,
even for a solid whose own BoundBox diagonal is small enough that
`rel_tol * diagonal` alone would push it below the model's own working
geometric tolerance (e.g. a tiny decomposed piece), which would make the
detector unable to catch even a genuinely near-zero-length degenerate
edge there."""

DEGENERATE_EDGE_LENGTH_FLOOR = 1.0e-9
"""Lower floor (mm): an edge shorter than this is treated as a
legitimate OCCT *degenerate* edge (a pole singularity on a closed
sphere/cone, where the surface parametrization collapses to a single
point) rather than a genuine CAD defect -- confirmed live, 2026-08-27,
`testing/inputSTEP/Torus/face2.stp` and `tank.stp`: both real,
long-working fixtures have real sphere faces whose own pole edges
measure ~7.7e-15mm (floating-point noise around a mathematically exact
zero, not a real gap), which find_short_edges' own detection wrongly
flagged as corrupted before this floor was added -- a real false
positive that broke `tests/test_cadtocsg.py` outright (the new
"stop"-by-default behavior halted on 2 previously-clean files). Per
direct user instruction, set well below the smallest genuine defect
found anywhere in this project's own corpus work (~4e-4mm, Decomposed/
modelcell_cut1_v2_piece66.stp's own real sliver) while still staying
comfortably above the ~1e-15 floating-point noise floor -- 1e-9 keeps
6 orders of magnitude of margin on the noise side and 5 on the real-
defect side."""


def find_short_edges(solid, rel_tol: float = 1e-4) -> list:
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


def find_split_ring_faces(solid, min_face_width: float = 0.1, rel_tol: float = 1e-4) -> list:
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


def count_split_ring_pairs(solid, rel_tol: float = 1e-3) -> int:
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
        circles = [
            edge.Curve
            for edge in face.Edges
            if edge.Curve is not None and type(edge.Curve).__name__ == "GCircle"
        ]
        for i in range(len(circles)):
            for j in range(i + 1, len(circles)):
                c1, c2 = circles[i], circles[j]
                if abs(abs(c1.Axis.dot(c2.Axis)) - 1.0) > 1e-4:
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
        return gap if 1e-5 < gap < dist_tol else None

    if ta == "GPlane":
        axis_dot = surf_a.Axis.dot(surf_b.Axis)
        if abs(axis_dot) < 0.99999:
            return None
        d_a = surf_a.Axis.dot(surf_a.Position)
        d_b = surf_b.Axis.dot(surf_b.Position)
        return _near(abs(d_a - d_b) if axis_dot > 0 else abs(d_a + d_b))

    if ta == "GCylinder":
        if abs(surf_a.Axis.dot(surf_b.Axis)) < 0.99999:
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
