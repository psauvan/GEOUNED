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


# ---------------------------------------------------------------------------
# Split cascade: BuildDepth / BuildSolidParts / filterparts / SplitSolid /
# SplitBase / joinBase / getPart / space_decomposition
# ---------------------------------------------------------------------------
#
# Shared 2026-09-17-18 between GEOUNED's `build_region/` (constructs the
# small solid a composite meta-surface -- RoundCorner/Can/TCone/
# MultiRoundCorner -- itself represents, from its own 2-4 primitive
# components) and GEOReverse's `CAD/buildSolidCell.py` (reconstructs an
# arbitrary MCNP/OpenMC cell's solid from its boolean surface definition:
# starting from a bounding box, split whatever pieces come out by each of
# the cell's own real surfaces one at a time, and keep/reject/keep-splitting
# each resulting piece per the cell's boolean definition). Despite the
# different end goal, this recursive cascade was found to be essentially
# line-for-line identical in both pipelines -- previously two independently
# hand-kept-in-sync copies. See CLAUDE.md's "build_region/ vs
# CAD/buildSolidCell.py+splitFunction.py unification" entry for the full
# rationale and the two remaining, genuine differences this module
# generalizes over rather than hides:
#
# - `classify(point, surf) -> bool`, a pluggable point-classification
#   callable: GEOUNED passes `lambda p, s: s.is_inside(p)` (its own
#   `CellSurface.is_inside` already delegates to the shared `geo`
#   plane/cylinder/cone/sphere descriptors); GEOReverse passes its own,
#   much richer `CAD/splitFunction.py::surface_side` (~15 surface types,
#   including the exotic quadrics) unchanged. Neither is touched by this
#   move.
# - `hasattr(cell, "build_BoundBox")` / `hasattr(cell, "buildSurfaceShape")`
#   guards: GEOUNED's `CellObj` always has its single, already-known,
#   always-Forward `.boundBox` set once up front (by
#   `build_shape_functions.py::build_complex_shape`, which also pre-builds
#   every surface's shape against that one box before this cascade ever
#   runs) and never recomputes it per subcell -- neither method exists on
#   `CellObj`, so these guards are a no-op for it, exactly matching its
#   previous (commented-out / absent) behavior. GEOReverse's `CadCell` has
#   both: an arbitrary CSG cell's own subcells genuinely need their own,
#   tighter box (`build_BoundBox`, which can come back `Reversed`, falling
#   back to `cell.externalBox`), and each surface's shape is built lazily,
#   sized to whatever box is current at that recursion depth
#   (`buildSurfaceShape`).
#
# Every duck-typed name used here (`.definition`, `.surfaces`, `.boundBox`,
# `.externalBox`, `.getSubCell()`, `.makeBox()`, `.copy()` on the cell;
# `.type`, `.shape`, `.buildShape(boundBox)` on a surface) already existed
# identically on both `CellObj`/`CellSurface` (GEOUNED) and `CadCell`/its
# surface classes (GEOReverse) -- no new interface, no ABC.


class SplitBase:
    def __init__(self, base, knownSurf={}, orientation="Forward"):
        self.base = base  # GSolid
        self.knownSurf = knownSurf
        self.orientation = orientation


def joinBase(baseList, tolerances=None):
    shape = []
    surf = {}
    removedKeys = []
    fwd = True
    for b in baseList:
        if b.orientation == "Reversed":
            fwd = False
        if b.base is not None:
            shape.append(b.base)
        for k, v in b.knownSurf.items():
            if k in removedKeys:
                continue
            if k not in surf.keys():
                surf[k] = v
            else:
                if surf[k] == v:
                    continue
                else:
                    surf[k] = None
                    removedKeys.append(k)

    newbase = Gfuse_solids(shape, tolerances)
    orientation = "Forward" if fwd else "Reversed"
    return SplitBase(newbase, surf, orientation)


def getPart(slist):
    sol = []
    for s in slist:
        if type(s) is list:
            sol.extend(getPart(s))
        else:
            sol.append(s)
    return sol


def space_decomposition(solids, surfaces, classify):
    """Get the position of each subregion of `solids` with respect to all
    of `surfaces`, via the caller-supplied `classify(point, surf) -> bool`
    point-classification callable (see this section's own module-level
    comment for what each pipeline passes)."""
    component = []
    good_solids = []
    for c in solids:
        if c.Volume < 1e-3:
            if abs(c.Volume) < 1e-3:
                continue
            else:
                c = c.reverse()
                print("Negative solid Volume", c.Volume)
        Svalues = {}
        point = c.find_interior_point()
        if point is None:
            continue  # point not found in solid (solid is surface or very thin can be source of lost particules in MCNP)
        for surf in surfaces:
            Svalues[surf.id] = classify(point, surf)

        component.append(Svalues)
        good_solids.append(c)
    return component, good_solids


def SplitSolid(base, surfacesCut, cellObj, tolerances, classify):
    """Split `base` (a `SplitBase`, or list/tuple of them) with `surfacesCut`
    (always exactly one real surface -- the tuple form is legacy, only its
    first element is ever used). `cellObj` is the cell being reconstructed;
    `tolerances` a `GEOUNED.utils.data_classes.Tolerances` instance passed
    straight through to `Gsplit`; `classify` the point-classification
    callable (see this section's own module-level comment).

    Returns `(fullPart, cutPart)`: solids fully enclosed in the cell, and
    solids not fully enclosed (needing more splitting against the cell's
    other surfaces)."""
    from . import GFace, GSolid, Gsplit
    from ..boolean_utils.boolean_function import evaluate_three_valued

    fullPart = []
    cutPart = []

    if type(base) is list or type(base) is tuple:
        for b in base:
            fullList, cutList = SplitSolid(b, surfacesCut, cellObj, tolerances, classify)
            fullPart.extend(fullList)
            cutPart.extend(cutList)
        return fullPart, cutPart

    # resulting cell orientation is "Reversed" only if both
    # cells have reversed orientations
    if cellObj.boundBox.Orientation == base.orientation:
        orientation = cellObj.boundBox.Orientation
    else:
        orientation = "Forward"

    if abs(base.base.Volume / base.base.Area) < 1e-2:
        return fullPart, cutPart

    tool = surfacesCut[0].shape
    if tool is not None:
        # GEOUNED's own CellSurface.buildShape() stores a bare native shape
        # on `.shape`; GEOReverse's surface classes already store a `geo`
        # wrapper (GFace for a plane, GSolid for cylinder/cone/sphere/the
        # exotic quadrics) -- accept either.
        if not isinstance(tool, (GSolid, GFace)):
            tool = GSolid(tool)
        try:
            Solids = [s.__native__ for s in Gsplit(base.base, tool, tolerances).solids]
        except Exception as e:
            # Was a silent `except Exception: Solids = []` in GEOReverse's
            # own copy (GEOUNED's had no try/except at all -- would have
            # crashed outright). Kept as a fallback (continue with the
            # uncut solid) in both, per direct user decision, but now
            # printed rather than swallowed -- same "surface the real
            # error" principle already applied to
            # CAD/buildCAD.py::BuildUniverseCells this same session.
            print(f"SplitSolid: Gsplit failed ({type(e).__name__}: {e}) -- falling back to uncut solid")
            Solids = []
        if not Solids:
            Solids = [base.base.__native__]
        Solids = [GSolid(s) for s in Solids]
    else:
        Solids = [base.base]

    partPositions, partSolids = space_decomposition(Solids, surfacesCut, classify)

    for pos, sol in zip(partPositions, partSolids):
        pos.update(base.knownSurf)
        inSolid = evaluate_three_valued(cellObj.definition, pos)

        if inSolid:
            fullPart.append(SplitBase(sol, pos, orientation))
        elif inSolid is None:
            cutPart.append(SplitBase(sol, pos, orientation))
    return fullPart, cutPart


def filterparts(parts, cell, tolerances, classify):
    from . import myBox

    process_part = []
    keep_part = []
    cellBox = cell.boundBox
    built = False
    if type(parts) is SplitBase:
        parts = (parts,)
    for p in parts:
        if p is None:
            process_part.append(p)
            continue
        cBox = myBox(cellBox.Box, "Forward")
        pbb = p.base.BoundBox

        pBox = myBox(pbb, "Forward")
        cBox.mult(pBox)
        if cBox.Box is None:
            if p.orientation == "Forward":
                if cellBox.Orientation == "Reversed":
                    keep_part.append(p)
            else:
                if cellBox.Orientation == "Reversed":
                    keep_part.append(p)
                    if not built:
                        built = True
                        cellpart = BuildDepth(cell, None, tolerances, classify)
                        keep_part.extend(cellpart)
        else:
            process_part.append(p)
    return process_part, keep_part


def BuildSolidParts(cell, base, tolerances, classify):

    # part if several base in input
    if isinstance(base, (list, tuple)):
        fullPart = []
        cutPart = []

        for b in base:
            fullList, cutList = BuildSolidParts(cell, b, tolerances, classify)
            fullPart.extend(fullList)
            cutPart.extend(cutList)

        return fullPart, cutPart

    if base:
        boundBox = base.base.BoundBox
        if boundBox.XLength < 1e-6 or boundBox.YLength < 1e-6 or boundBox.ZLength < 1e-6:
            return [], []
    else:
        # GEOUNED's CellObj has no build_BoundBox at all (its single,
        # always-Forward boundBox is already set once by the caller) --
        # see this section's own module-level comment.
        if cell.boundBox is None and hasattr(cell, "build_BoundBox"):
            cell.build_BoundBox(cell.externalBox, enlarge=0.2)
        if cell.boundBox.Orientation == "Reversed":
            boundBox = cell.externalBox.Box
        else:
            boundBox = cell.boundBox.Box

    if boundBox is None:
        return [], []

    surfaces = tuple(cell.surfaces.values())
    # GEOUNED pre-builds every surface's shape once, up front, against the
    # cell's own single box (build_shape_functions.py::build_complex_shape)
    # -- CellObj has no buildSurfaceShape method, so this is a no-op there.
    if hasattr(cell, "buildSurfaceShape"):
        cell.buildSurfaceShape(boundBox)

    if not surfaces:
        print("not cutting surfaces")
        return tuple(base.base), tuple()

    if base is None:
        cellBox = cell.makeBox()
        if cellBox is None:
            return [], []
        base = SplitBase(cellBox, orientation=cell.boundBox.Orientation)

    planes = []
    others = []
    for s in surfaces:
        if s.type == "plane":
            planes.append(s)
        else:
            others.append(s)

    cut = base
    full = []
    for p in planes:
        newf, cut = SplitSolid(cut, (p,), cell, tolerances, classify)
        full.extend(newf)
        if len(cut) == 0:
            break

    for surf in others:
        newf, cut = SplitSolid(cut, (surf,), cell, tolerances, classify)
        full.extend(newf)
        if len(cut) == 0:
            break

    if type(cut) is SplitBase:
        cut = [cut]

    return full, cut


def BuildDepth(cell, base, tolerances, classify):
    from ..boolean_utils.boolean_function import BoolSequence

    cell.definition.group_single()
    if cell.definition.level == 0:
        # if base is None build solid from cell boundBox
        # else base is build solid split by cell surfaces
        base, cut = BuildSolidParts(cell, base, tolerances, classify)
        return base

    if type(base) is not list:
        base = [base]
    newBase = []

    for CS in base:
        if type(cell.definition.elements) is not bool:
            if cell.definition.level == 0:
                tmp = BoolSequence(operator=cell.definition.operator)
                tmp.append(cell.definition)
                cell.definition = tmp

            if cell.definition.operator == "AND":
                part = CS
                for e in cell.definition.elements:
                    subcell = cell.getSubCell(e)
                    keep = []
                    if part is not None:
                        if hasattr(subcell, "build_BoundBox"):
                            subcell.build_BoundBox(cell.externalBox, enlarge=10)
                        if subcell.boundBox.Box is None:
                            if subcell.boundBox.Orientation == "Reversed":
                                continue
                            else:
                                part = []
                                break

                        part, keep = filterparts(part, subcell, tolerances, classify)
                        if len(part) == 0:
                            if len(keep) == 0:
                                break
                            else:
                                part = keep
                                continue
                    part = BuildDepth(subcell, part, tolerances, classify)
                    part.extend(keep)
                newBase.extend(part)
            else:
                cellParts = []
                for e in cell.definition.elements:
                    subcell = cell.getSubCell(e)
                    if CS is not None:
                        if hasattr(subcell, "build_BoundBox"):
                            subcell.build_BoundBox(cell.externalBox, enlarge=10)
                        if subcell.boundBox.Box is None:
                            if subcell.boundBox.Orientation == "Reversed":
                                if type(CS) is SplitBase:
                                    cellParts.append(CS)
                                else:
                                    cellParts.extend(CS)
                            continue
                        part, keep = filterparts(CS, subcell, tolerances, classify)
                        cellParts.extend(keep)
                        if len(part) == 0:
                            continue
                    else:
                        part = CS
                    part = BuildDepth(subcell, part, tolerances, classify)
                    cellParts.extend(part)

                JB = joinBase(cellParts, tolerances)
                if JB.base is not None:
                    newBase.append(JB)

        elif cell.definition.elements:
            newBase.append(CS)

    return newBase
