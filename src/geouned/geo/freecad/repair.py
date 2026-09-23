"""
geo/freecad/repair.py

Load-time CAD-defect repair -- Gcollapse_split_rings/Gsliver_heal/
Gheal_topology are None-returning stubs (no native tools wired for
FreeCAD, see their own docstrings), and Gcheck_and_repair bypasses the
whole cascade unconditionally (see its own docstring) rather than
orchestrating them. Gdefeature (Part.Shape.defeaturing()) used to be
the one real, non-stub implementation here, but was confirmed 100% dead
code -- Gcheck_and_repair's own unconditional bypass means nothing in
the whole repo ever called it -- and deleted 2026-09-23.
"""

from __future__ import annotations

import FreeCAD
import Part

from .topology import GSolid, Gclassify_surface
from ..constants import DEFAULT_MIN_FACE_WIDTH


def Gcollapse_split_rings(solid: "GSolid", tolerances) -> "GSolid | None":
    """Repair a "split boundary ring" / duplicated micro-trim defect --
    a single trimming surface duplicated at a sub-tolerance offset, with
    parasitic "riser" faces bridging the thin slab and every curved face
    on the trim carrying a doubled boundary ring.

    Implemented for the ocp/occ backends only (a ``ShapeBuild_ReShape`` +
    ``BRepBuilderAPI_Sewing`` + ``ShapeFix_Shape`` pipeline -- see
    ``geo._ocp_impl.Gcollapse_split_rings`` for the full account and the
    verified fixture). No FreeCAD ``Part`` equivalent is wired: this is a
    None-returning stub so the caller (``Gcheck_and_repair``, itself a
    no-op bypass under this engine -- see below) never actually reaches
    it under freecad."""
    return None


def Gsliver_heal(solid: "GSolid", min_face_width: float = DEFAULT_MIN_FACE_WIDTH) -> "GSolid | None":
    """`sliver_healing` (version 0) -- the fuller form of
    `Gcollapse_split_rings` (remove sliver faces, resolve a
    near-coincident surface pair by dropping the smaller face + capping
    the freed hole on the kept plane, sew last).

    ocp/occ only -- see ``geo._ocp_impl.Gsliver_heal`` for the algorithm
    and the verified fixture (``LR.stp``). A ``None``-returning stub here,
    same reasoning as ``Gcollapse_split_rings`` above."""
    return None


def Gheal_topology(solid: "GSolid") -> "GSolid | None":
    """Repair a topologically-invalid decomposition fragment via an
    in-memory STEP serialize -> deserialize rebuild (see
    ``geo._ocp_impl.Gheal_topology`` and reference_cad_defect_recipes.md
    Recipe 3). A ``None``-returning stub under the freecad engine:
    FreeCAD already heals on its own ``Part.Shape().read()`` load path and
    exposes no in-memory STEP stream API, so this repair has no place in
    the freecad pipeline."""
    return None


def Gmerge_coplanar_planes(solid: "GSolid") -> "GSolid":
    """Merge every group of adjacent, co-planar planar faces of `solid`
    into a single planar face -- a hand-rolled, planes-only alternative to
    ``GSolid.refine()``.

    Implemented for the ocp/occ backends only (a ``TopExp`` edge->faces
    adjacency map + union-find over the planar faces + a
    ``BRepBuilderAPI_Sewing``/``ShapeFix_Shape`` re-sew -- see
    ``geo.ocp.repair.Gmerge_coplanar_planes`` for the full account). No
    FreeCAD ``Part`` equivalent is wired: this is an identity stub
    returning `solid` unchanged, matching the real implementations' own
    "returns the input unchanged when it cannot improve on it" contract."""
    return solid


def Gcheck_and_repair(solid: FreeCAD.Solid, tolerances) -> "tuple[FreeCAD.Solid, bool]":
    """FreeCAD has none of the native CAD-defect-repair tools this
    cascade needs (``Gdefeature`` exists here via ``Part.Shape.
    defeaturing()``, but ``Gcollapse_split_rings``/``Gsliver_heal`` are
    both ``None``-returning stubs above -- no ``Part`` pipeline is wired
    for either) -- per direct user instruction ("no tiene las
    herramientas para realizar las operaciones"), bypass the whole
    check+repair process entirely for this engine and return `solid`
    exactly as loaded, `True`, unconditionally. No `check_solid_defects`
    call, no attempt. `GeounedSolid.__init__` already applies its own
    `.refine()` unconditionally downstream regardless of this bypass, so
    a basic level of cleanup still happens -- just not this dedicated
    corrupted-solid detection/repair pass, which needs tools this engine
    doesn't have."""
    return solid, True


def Gspline_surface(solid) -> bool:
    """True if `solid` (any native shape -- a whole solid, typically) has
    at least one face whose underlying surface is NOT one of the 5
    analytic types GEOUNED can classify (Plane/Cylinder/Cone/Sphere/
    Torus) -- a BSpline, Bezier, or other freeform/swept surface
    `Gclassify_surface` would reject (returning None for it). Replaces
    the former `load_functions.py::spline()` helper (which read this off
    an already-built `GSolid.Faces`) -- this version works directly on
    the native shape, no `GSolid`/`GFace` construction needed, matching
    the other 2 engines' own `Gspline_surface`.

    Delegates the actual per-face classification to `Gclassify_surface`
    itself (the same dispatch `GFace.__init__` calls) rather than
    re-checking the surface type against an allowed set by hand here --
    this is the single place that dispatch is defined (including
    FreeCAD's own "BSplineSurface secretly a flat plane" `findPlane()`
    fallback), and duplicating it would risk the two drifting apart."""
    for native_face in solid.Faces:
        if Gclassify_surface(native_face) is None:
            return True
    return False


def Gface_valid(face) -> bool:
    """True if `face` -- a native Part.Face, or a GFace (unwrapped here)
    -- passes Part.Face.isValid(): its boundary wire(s) bound a
    coherent, orientable 2D region on its surface, its pcurves are sane,
    edges lie on the surface within tolerance, etc.

    The per-face counterpart of `GSolid.is_valid()`. A whole solid can
    be invalid solely because one of its faces is (a single boundary
    wire that is really two loops crammed together -- see the
    `L4_support.stp` case in reference_cad_defect_recipes.md); this
    isolates the check to one face. Returns False rather than raising
    if the check itself cannot run on the shape.
    """
    native = getattr(face, "__native__", face)
    try:
        return bool(native.isValid())
    except Exception:
        return False


def Gdiagnose_open_solid(solid, tolerances):
    """No open-solid diagnosis on the FreeCAD engine -- its Gsplit path
    (recursive_freecad_Gsplit) does not go through _raw_bop_split and
    this native BOPAlgo-seam repair has no FreeCAD equivalent. Always
    None (see the occ/ocp engines for the real implementation)."""
    return None


def Gclose_open_solid(solid, tolerances):
    """No open-solid repair on the FreeCAD engine -- see
    Gdiagnose_open_solid. Always None."""
    return None
