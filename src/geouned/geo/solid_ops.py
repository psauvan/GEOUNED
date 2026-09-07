"""
geo/solid_ops.py

Engine-agnostic *policy* helpers that sit one layer above the raw kernel
primitives (`Gfuse`, `Gcut`, `Gsplit`, ...) -- "how to combine solids
robustly, with a safe fallback", not "how to call the boolean kernel".
Shared between GEOUNED's forward pipeline (`build_region/`) and
GEOReverse's reverse pipeline, which previously each carried their own
near-identical copy (`build_region/splitFunction.py::FuseSolid` and
`GEOReverse/Modules/matrix_utils.py::fuse_solids`).

Lives at the `geo/` top level, next to `vector_geometry.py` /
`surface_geometry.py` / `solid_defects.py`, because nothing here touches
a CAD kernel directly -- every name it uses (`Gfuse`, `Gmake_compound`,
`Gclose_open_solid`) is itself already engine-dispatched by
`geo/__init__.py`. The imports are function-local to keep this module
free of an import cycle with `geo/__init__.py`.
"""


def Gfuse_solids(parts, tolerances=None):
    """Boolean-union `parts` (a list of `GSolid`) into one `GSolid`, or
    `None` if `parts` is empty.

    Robust against a failed / invalid fuse: falls back through
    `fix(1e-6)` (ShapeFix_Shape) and finally an unfused `Gmake_compound`
    (which keeps each part's own, possibly-overlapping boundary instead
    of a true union) rather than raising.

    When `tolerances` is given, each part is first passed through
    `Gclose_open_solid`: a split can leave a part as an *open* solid from
    a known cause (a doubled BOPAlgo tangent seam -- one trimming
    surface's boundary curve emitted twice, once per adjacent face, at a
    plane-tangent-to-cylinder intersection). An open part poisons the
    boolean fuse entirely (BRepAlgoAPI_Fuse with an invalid operand
    returns an empty result), so `Gfuse` would give up to an unfused
    compound. Closing the doubled seam first lets the real fuse -- across
    the genuine coplanar shared wall the two parts do share -- succeed.
    Confirmed on `Test RoundCorners/rc1.stp`'s RoundCorner `surf.shape`
    (2026-09-04): open 21826.54mm^3 wedge + 463617.49mm^3 block -> one
    valid 485444mm^3 solid instead of a 2-shell compound.
    `Gclose_open_solid` is a no-op (returns None) for any already-
    watertight part or unrecognised open part, so this never changes a
    case that was already fusing.

    ShapeFix_Shape (`fix()`) doesn't always repair a genuinely invalid
    boolean-fuse result (e.g. a real solid with 4 valid input parts whose
    `Gfuse()` came back topologically invalid per BRepCheck_Analyzer,
    confirmed via a real case on `Solidos/Cans/rev_can_1.stp`) -- but it
    is a stronger repair than `refine()`/removeSplitter and fixed it
    there without changing the volume. Try it before giving up on a real
    fused solid and falling back to an unmerged compound.

    `refine()` (removeSplitter / ShapeUpgrade_UnifySameDomain) is applied
    only as a final tidy step, and only on a single, BRepCheck-valid
    (hence watertight) solid: run against a not-properly-closed fuse
    result (an open shell, or an unfused >1-solid compound) it
    historically errored / mangled the geometry. It is called with
    `rel_tol=1e-4` rather than the strict `1e-6` default: merging the
    redundant tangent-seam faces a boolean fuse leaves (e.g. a
    RoundCorner composite `surf.shape` where a slanted plane is tangent
    to a fillet cylinder) legitimately moves the volume by ~1e-6
    relative -- the merged tangent face sits a hair outside where the two
    split faces met -- and the strict guard would then discard the clean
    result and keep the redundant-edge solid (confirmed on
    `Test RoundCorners/cs_1.stp`: 13 faces / 54 edges -> 7 faces / 30
    edges, dV_rel 1.7e-6, matching FreeCAD's own guard-less "Refine
    shape"). `1e-4` stays far below the ~3.4% corruption signal the
    guard exists to catch. Any residual invalid refine result is
    discarded.
    """
    from . import Gclose_open_solid, Gfuse, Gmake_compound

    if not parts:
        return None

    if tolerances is not None:
        parts = [(Gclose_open_solid(p, tolerances) or p) for p in parts]

    try:
        fused = Gfuse(parts)
    except Exception:
        fused = None

    if fused is not None:
        if fused.is_valid():
            gsolid = fused
        else:
            try:
                fixed = fused.fix(1e-6)
            except Exception:
                fixed = None

            if fixed is not None and fixed.is_valid():
                gsolid = fixed
            else:
                gsolid = Gmake_compound(parts)
    else:
        gsolid = Gmake_compound(parts)

    if len(gsolid.Solids) == 1 and gsolid.is_valid():
        try:
            refined = gsolid.refine(rel_tol=1e-4)
            if refined.is_valid():
                gsolid = refined
        except Exception:
            pass

    if gsolid.Volume < 0:
        gsolid = gsolid.reverse()
    return gsolid
