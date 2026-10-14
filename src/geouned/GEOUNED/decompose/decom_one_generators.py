#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging

from .generators import get_surfaces
from ...geo import (
    CAD_ENGINE,
    GCompound,
    GSolid,
    Gheal_topology,
    Gmake_compound,
    Gsolid_max_tolerance,
    Gspline_surface,
    Gsplit,
    Gtrim_overlapping_coplanar_faces,
    is_sliver_solid,
    merge_sliver_pieces,
)

if CAD_ENGINE != "freecad":
    from ...geo import Gsubstitute_spline_quadrics
from ...geo.constants import (
    FACE_PROBE_BISECTION_STEPS,
    MAX_REPAIR_VOLUME_REL_CHANGE,
    NUMERIC_DOUBLE_TOL,
    SPLIT_CANDIDATE_VOLUME_REL_TOL,
    STUCK_FRAGMENT_PROBE_GRID,
    STUCK_FRAGMENT_PROBE_MIN_COUNT,
)
from ...geo.solid_defects import find_overlapping_coplanar_faces
from ...geo.volume_utils import volume_within

logger = logging.getLogger("general_logger")


def split_surfaces(solid, options, tolerances):

    solid_components = generic_split(solid, options, tolerances)

    # `.fix()` can raise a real native crash (Standard_Failure: "Courbes non
    # jointives", the same ShapeUpgrade_UnifySameDomain crash class already
    # documented for Mixed/ConeSphere.stp) on a piece that never needed
    # fixing in the first place -- confirmed live on Complex_cell/SCDR_90.stp.
    # Keep the original, unfixed piece on that failure rather than let one
    # auxiliary repair attempt abort an otherwise-fine decomposition.
    fixed_solids = []
    for s in solid_components:
        try:
            fixed_s = s.fix(tolerances.fix_tolerance)
            # Same corruption class as generic_split's own two .fix()
            # call sites (see _fix_result_is_sane's own docstring) --
            # confirmed live, 2026-10-13, Mixed/base_all8_to_cylinder.
            # stp: generic_split can legitimately return a tiny,
            # inverted-orientation-but-structurally-sane sliver (a prior
            # .fix() call already flipped its sign while keeping it
            # BRepCheck-valid), and re-running .fix() on THAT here, a
            # second time with no sanity check at all, collapsed it to a
            # single 0-wire face with Volume~6.7e166 -- silently
            # accepted (no exception raised) and propagated all the way
            # into set_cad_solid()/optimalBoundingBox(), which then
            # crashed void generation's own Gmake_box with an "Infinite
            # parameter" domain error, far downstream of the real cause.
            if _fix_result_is_sane(fixed_s, s.Volume):
                s = fixed_s
        except Exception:
            pass
        # A boolean cut landing exactly tangent to a pre-existing surface
        # can leave a residual "phantom" face duplicating part of a real
        # neighboring face -- `generic_split`'s own recursive cuts can
        # create this defect fresh, on a piece that never had it at load
        # time (confirmed live, struct_plate2.stp's own piece8: 0 defects
        # on the original, undecomposed solid; the defect only appears
        # after decomposition splits it), so the load-time repair cascade
        # (`Gcheck_and_repair`) never gets a chance to catch it -- it has
        # to be checked again here, on each piece this function itself
        # produces. Same crash-safety discipline as `.fix()` just above:
        # never let an auxiliary repair attempt abort an otherwise-fine
        # decomposition, and keep the original piece if the repair isn't
        # possible (`Gtrim_overlapping_coplanar_faces` already returns
        # `None` rather than raising in that case, but the whole check is
        # still wrapped defensively).
        try:
            if find_overlapping_coplanar_faces(s):
                repaired = Gtrim_overlapping_coplanar_faces(s)
                if repaired is not None:
                    s = repaired
        except Exception:
            pass
        fixed_solids.append(s)

    # A thin sliver that survived Gsplit's own repair cascade (see
    # merge_sliver_pieces's own docstring, geo/solid_ops.py) is its own
    # genuinely separate, valid solid -- Gsliver_heal declines to touch
    # it (not a redundant coplanar face WITHIN one piece), so it is still
    # present here, in the FULL list of this solid's final irreducible
    # pieces. Eliminate it now, by fusing it into whichever real
    # neighbor it is still in contact with, rather than letting it become
    # its own separate cell with a face no MCNP/OpenMC boolean definition
    # can ever meaningfully bound.
    fixed_solids = merge_sliver_pieces(fixed_solids, tolerances)

    comp = GCompound(fixed_solids)

    volratio = (comp.Volume - solid.Volume) / solid.Volume
    if volratio > tolerances.volume_tolerance:
        logger.warning(f"Lost {volratio * 100:6.2f}% of the original volume")

    # A fragment Gsplit accepted as "sane" (BRepCheck-valid + a real
    # volume, via _finalize_split's own filter) can still fail to
    # resolve to a genuine TopoDS_Solid: a repair step inside
    # _raw_bop_split's cascade (confirmed: Gsliver_heal's own
    # ShapeUpgrade_UnifySameDomain step) can leave a fragment as a bare
    # TopoDS_Compound/Shell that is still topologically valid and still
    # reports a correct .Volume (computed shape-type-agnostically) -- but
    # contains zero real solid leaves. Detected here via a one-shot,
    # throwaway native wrap+re-extract of EACH piece ALONE (never several
    # touching pieces together, which is the exact scenario GCompound's
    # own docstring documents as unreliable) -- so this probe only ever
    # asks "does this single fragment's own native shape contain a real
    # TopAbs_SOLID", never anything aggregate, and is never used for the
    # actual returned pieces/volumes (those come from `fixed_solids`/`comp`
    # above) -- a fragment that fails this resolves to zero real solid
    # leaves and will NOT be part of this cell's boolean expression, a
    # real, currently-unrepaired gap (see CLAUDE.md).
    dropped = [f for f in fixed_solids if not Gmake_compound([f]).Solids]
    if dropped:
        dropped_volume = sum(abs(f.Volume) for f in dropped)
        fragment_volumes = ", ".join(f"{abs(f.Volume):.2f}" for f in fixed_solids)
        logger.warning(
            f"generic_split produced {len(fixed_solids)} fragment(s) "
            f"(volumes: {fragment_volumes}) but {len(dropped)} did not resolve to "
            f"a real solid -- {dropped_volume:.2f} of volume silently dropped and "
            "will NOT be considered when building this solid's boolean expression."
        )

    return comp


def _tool_classifier(surf):
    """A `point -> bool` callable for `surf`'s own already-built
    `.shape`, used by `_some_real_candidate_should_have_split` to decide
    which side of this real candidate a sampled point falls on.

    NOT simply `GSolid(surf.shape).is_inside(point)` for every type: a
    "Plane" candidate's own `.shape` (`GeounedSurface.build_surface`,
    `self.shape = makePlane(...)`) is literally just the bare plane FACE
    itself (`self.shell = self.shape`, the same object) -- zero
    enclosed volume, one face. `BRepClass3d_SolidClassifier` on a shape
    like that cannot ever report a point as genuinely "inside" (there is
    no interior to be inside of), so naively reusing `is_inside` here
    silently makes every Plane candidate read as 0 points on one side
    regardless of the real geometry -- confirmed live, 2026-10-09: this
    masked the real anomaly on Big_model_reserved/divertor_cam.stp's own
    EC_SUPPORT#BYZ2TW_41 piece0 (whose own genuine candidate is a
    Plane), and happened to still give the (coincidentally) correct
    "one-sided" answer on testing/inputSTEP/BC.stp's own Plane
    candidates only because that fragment is genuinely irreducible
    anyway. Cylinder/Cone/Sphere/Torus and the composite Can/TCone/
    RoundCorner/MultiRoundCorner/MultiPlane types all build a real,
    closed solid for `.shape` (confirmed by direct inspection of each
    branch in `geouned_classes.py::GeounedSurface.build_surface` and, for
    MultiPlane, `build_shape_functions.py::makeMultiPlanes`'s own
    `Gmake_solid(Gmake_shell(cutfaces))`), so `is_inside` is correct for
    every one of them -- only Plane needs the direct half-space formula
    instead."""
    if surf.Type == "Plane":
        position = surf.Surf.Position
        axis = surf.Surf.Axis
        return lambda p: (p - position).dot(axis) > 0
    tool_solid = GSolid(surf.shape)
    return lambda p: tool_solid.is_inside(p)


def _face_anchored_points(solid):
    """Points genuinely inside `solid`, anchored to `solid`'s own real
    faces -- the same "probe inward from each face along its inward
    normal" strategy `GSolid.find_interior_point` already uses and has
    already benchmarked against sampling/octree alternatives (see that
    method's own docstring), generalized here to collect MANY such
    points (a small grid per face) instead of stopping at the first one.

    For each face, a `STUCK_FRAGMENT_PROBE_GRID` x `STUCK_FRAGMENT_PROBE_GRID`
    grid of (u, v) samples across its own `ParameterRange` gives a
    candidate point (`value_at`) and the face's own normal there
    (`normal_at`); `is_part_of_domain` discards a grid point outside the
    face's real trimmed boundary (`ParameterRange` is only the
    rectangle, not the trim); `orientation_outward` decides which sign
    of the normal points OUT of the material, computed once per face
    (not per grid point -- a face's own orientation is a single,
    solid-wide property). Each kept point is then walked inward along
    that direction via `find_interior_point`'s own bisection (start at
    half the solid's own characteristic length, `0.5 * Volume ** (1/3)`,
    then halve up to `FACE_PROBE_BISECTION_STEPS` times, keeping the
    FIRST, i.e. largest, distance that lands inside) rather than a
    single small fixed offset.

    A single small fixed offset was tried first, 2026-10-09, and
    rejected: it found real material at only ~3% of BC.stp's own
    motivating second fragment's grid points (6-8 of ~130 passing
    `is_part_of_domain`), wrongly suggesting face-anchored sampling was
    fundamentally unreliable for this kind of thin/sparse composite
    shape -- but the actual problem was testing the wrong length scale,
    not the method itself: a probe this close to a real boundary point
    can be exactly at the floating-point edge of a thin wedge or a
    concave corner, landing just outside instead of just inside, while
    a MEANINGFUL distance along the exact same direction (this
    fragment's own characteristic length is ~45mm, not a sub-millimeter
    epsilon) reliably lands in real material -- confirmed directly:
    bisecting from that scale raised the hit rate to ~11% (28 of ~130)
    on the exact same fragment and faces, recovering the correct
    material-on-both-sides signal for its own real Plane candidate.
    Every resulting point is still confirmed via a real `is_inside`
    call at each bisection step, so an occasional genuine miss (all 12
    halvings exhausted without finding material -- a face whose own
    local neighborhood really is too thin or concave for this strategy)
    is simply dropped, never silently trusted."""
    length0 = 0.5 * abs(solid.Volume) ** (1.0 / 3.0)
    points = []
    for face in solid.Faces:
        try:
            outward = face.orientation_outward(solid)
        except Exception:
            continue
        u_min, u_max, v_min, v_max = face.ParameterRange
        for iu in range(STUCK_FRAGMENT_PROBE_GRID):
            u = u_min + (u_max - u_min) * (iu + 0.5) / STUCK_FRAGMENT_PROBE_GRID
            for iv in range(STUCK_FRAGMENT_PROBE_GRID):
                v = v_min + (v_max - v_min) * (iv + 0.5) / STUCK_FRAGMENT_PROBE_GRID
                try:
                    if not face.is_part_of_domain(u, v):
                        continue
                    point = face.value_at(u, v)
                    normal = face.normal_at(u, v)
                except Exception:
                    continue
                inward = -normal if outward else normal
                d = length0
                for _ in range(FACE_PROBE_BISECTION_STEPS):
                    d *= 0.5
                    p = point + inward * d
                    if solid.is_inside(p):
                        points.append(p)
                        break
    return points


def _some_real_candidate_should_have_split(solid, surfaces):
    """Independent, geometric ground truth for "did at least one of the
    REAL candidates `generic_split` just exhausted genuinely divide
    `solid`'s own material, even though BOPAlgo_Splitter reported no
    split for every one of them" -- the signal that decides whether a
    STEP round-trip heal-and-retry is actually warranted for a stuck
    fragment, replacing the former tolerance-inflation-based guess.

    Confirmed by direct measurement, 2026-10-09, that tolerance alone
    cannot answer this: a genuinely stuck-but-CORRECT (nothing to fix)
    fragment of testing/inputSTEP/BC.stp carried a native BRep tolerance
    of 8.67mm -- two orders of magnitude ABOVE the inflated 0.075mm
    tolerance of Big_model_reserved/divertor_cam.stp's own genuinely
    BROKEN EC_SUPPORT#BYZ2TW_41 piece0 -- so no threshold on
    Gsolid_max_tolerance can separate the two cases; nor can the
    SameParameter/SameRange BRep consistency flags (both clean on the
    broken fragment too), nor whether the tolerance is spread uniformly
    across every vertex/edge or only some (a genuinely fine fragment of
    Hollow_plates/placa.stp showed the exact same partial, non-uniform
    pattern), nor a direct geometric check for an actual near-tangent
    OTHER face close to the worst vertex (none found, 34mm away, on the
    broken fragment -- the inflation is not a measurable local
    tangency). Only asking the geometry itself, independently of
    BOPAlgo_Splitter, resolved this: for each real candidate surface
    already tried and rejected, check whether real material sits on
    BOTH sides of that candidate at the face-anchored points
    `_face_anchored_points` collects (via `is_inside`'s own
    BRepClass3d_SolidClassifier, unrelated to the boolean-split
    machinery that might be wrong), classified via `_tool_classifier`
    (see its own docstring for why a Plane candidate needs a direct
    half-space test rather than `is_inside` on its own bare-face
    `.shape`). Verified on cell1182's own broken fragment: all 3 of its
    real candidates (confirmed directly to cleanly split a fresh copy of
    the same solid) show material on both sides; verified on every
    stuck fragment across BC.stp's own WHOLE decomposition (19
    fragments, both under meta_surfaces=True and False): zero show
    material on both sides for any of their own real candidates --
    correctly never triggering this retry for solids that are
    genuinely, correctly irreducible.

    `surfaces` is the plain list of real `GeounedSurface` candidates to
    test against `solid` -- generic_split's own caller rebuilds this
    list fresh from get_surfaces() against whichever exact `solid` is
    being checked (see its own call site for why a fresh rebuild, not
    the original candidate loop's own tools, is required)."""
    if not surfaces:
        return False
    points = _face_anchored_points(solid)
    if not points:
        return False
    for surf in surfaces:
        classify = _tool_classifier(surf)
        in_n = sum(1 for p in points if classify(p))
        out_n = len(points) - in_n
        if in_n >= STUCK_FRAGMENT_PROBE_MIN_COUNT and out_n >= STUCK_FRAGMENT_PROBE_MIN_COUNT:
            return True
    return False


def _fix_result_is_sane(fixed, reference_volume) -> bool:
    """`GSolid.fix()` (`ShapeFix_Shape`/`ShapeUpgrade_UnifySameDomain`) can
    report a repaired solid as `BRepCheck`-valid while actually producing
    nonsense -- confirmed live, 2026-10-13, `Mixed/base_all8_to_cylinder.
    stp`: a tiny, genuinely-invalid 3-face sliver fragment
    (Volume=+0.7495, is_valid=False) that `generic_split`'s own top-of-
    function `.fix()` turned into a topologically-valid but INVERTED-
    orientation solid (Volume=-0.7495, is_valid=True, same magnitude --
    `is_valid()` alone doesn't check orientation sign) -- which then, a
    few lines later, got handed BACK into `.fix()` a second time
    (the "did .fix() change something, try fresh candidates" retry
    below) and came back completely degenerate: 3 faces collapsed into 1
    with 0 wires, Area~1.2e101, Volume~6.7e166 -- `ShapeUpgrade_
    UnifySameDomain` merging coaxial cylindrical faces whose normals are
    all backwards from a prior inversion. Neither corruption was caught
    by `is_valid()` itself, matching this project's own repeatedly-
    documented "BRepCheck_Analyzer reporting valid is a false pass"
    lesson elsewhere in this cascade. Checks the two cheap, specific
    signatures of this corruption class: every face keeps at least one
    real wire, and the repaired solid's own volume MAGNITUDE (not signed
    value -- a correct repair can legitimately flip orientation/sign,
    just never balloon or collapse its magnitude) stays within the same
    relative-change gate every other repair in this cascade already uses
    for "did this repair conserve the real geometry"."""
    if any(len(f.wires()) == 0 for f in fixed.Faces):
        return False
    return volume_within(abs(fixed.Volume), abs(reference_volume), MAX_REPAIR_VOLUME_REL_CHANGE)


def generic_split(solid, options, tolerances, loop=0, healed=False):
    # Heal a BRepCheck-invalid decomposition fragment before splitting it
    # further (or returning it as a final piece). BOPAlgo_Splitter can
    # leave a cut cylinder/cone wedge with one V-boundary edge's pcurve a
    # full 2*pi period off: the face's BRepTools::UVBounds then spans the
    # whole period (IsUClosed()==True) with ~one period of phantom
    # material glued on, so its ParameterRange contradicts its own
    # small-angle arc edges and Can/RoundCorner/RevCC detection treats it
    # as a closed cylinder. Only ShapeFix_Shape (GSolid.fix() on an
    # invalid solid) resolves it -- removeSplitter/UnifySameDomain
    # (.refine(), already applied by GeounedSolid.__init__) does not, and
    # _raw_bop_split's own repair result gets discarded whenever it
    # collapses to a single solid. Confirmed on RevCC_regression/
    # Big_one_cell__modelCell_670000__solid0_piece59: the first decomposed
    # sub-solid's R=6 round-corner cylinder went from a 2*pi U range
    # (piece vol 6849, split total overshooting the input by exactly the
    # 754mm^3 phantom region) to a ~75 deg wedge (vol 6095, split total
    # matching the input). Only fires on a genuinely invalid piece, so
    # the normal path is untouched.
    if not solid.is_valid():
        fixed = solid.fix(tolerances.fix_tolerance)
        if fixed.is_valid() and _fix_result_is_sane(fixed, solid.Volume):
            solid = fixed

    # A fragment can inherit an unclassifiable face from a cutting TOOL
    # used to split it earlier in this same recursion -- confirmed live,
    # 2026-10-12, ITER-TOKAMAK_comp/inout_RC.stp: a degenerate-torus
    # cutting tool built by Gmake_torus_elliptic (GeounedSurface.
    # build_surface's own Torus branch) has a genuine GeomAbs_
    # SurfaceOfRevolution face (an exact Geom_Ellipse/Geom_Circle profile
    # revolved) that Gclassify_surface doesn't recognize; after the
    # boolean cut, the result fragment inherits a face of the same
    # native type (its own basis curve re-approximated to a BSpline by
    # the cut itself), which later made cutting_face_number raise
    # "Spline surface detected" the first time this fragment's own
    # candidate surfaces were searched. Try the substitution once,
    # opportunistically, before this fragment is used as a candidate
    # source at all; keep the original fragment untouched if the attempt
    # doesn't fully resolve it (same all-or-nothing contract as
    # Gsubstitute_spline_quadrics's own load-time use). Four real bugs in
    # Gsubstitute_spline_quadrics itself were found and fixed chasing
    # this exact fixture's own 12 affected fragments (see geo/{occ,ocp}/
    # spline_quadrics.py's own docstrings): a whole-curve pcurve
    # projection silently picking the wrong (u,v) branch on a
    # self-intersecting torus; the generic sample+fit converging to the
    # wrong torus parameters on a face whose own trimmed domain samples
    # too narrow a region of the same degenerate surface (fixed by
    # `_detect_revolution_profile_torus`'s exact, sampling-free
    # recognition for a SurfaceOfRevolution with a circular profile); a
    # periodic-unwrap regression the first fix introduced for a plain,
    # non-degenerate full seam edge; and `_detect_revolution_profile_
    # torus` itself not unwrapping a `Geom_TrimmedCurve` wrapping the
    # real circular profile curve. All 12 of this fixture's own affected
    # fragments now resolve correctly, and the whole file converts end
    # to end (decompose + build_solid_definition + build_void +
    # export_csg) -- a real d1suned run on the result still loses 10
    # particles, a separate, not-yet-investigated boundary/adjacency gap
    # in the written cell definition, unrelated to spline detection
    # itself. freecad has no equivalent (occ/ocp only, matching every
    # other spline-quadric feature).
    if CAD_ENGINE != "freecad" and Gspline_surface(solid.__native__):
        substituted, resolved = Gsubstitute_spline_quadrics(solid, tolerances)
        if resolved:
            solid = substituted

    bbox = solid.BoundBox
    bbox = bbox.enlarged(10)

    comsolid_solids = [solid]
    omitfaces = set()

    # BOPAlgo_Splitter.Perform() can corrupt `solid`'s own native shape as a
    # side effect of merely attempting an intersection, even when the
    # candidate tool ultimately fails to split it at all (still 1 output
    # piece) -- since Gsplit's own `base` argument is passed by reference
    # straight into BOPAlgo_Splitter.AddArgument, this corruption is
    # cumulative and order-dependent across the WHOLE candidate-surface
    # search below, not scoped to one failed attempt. Confirmed live on
    # Mixed/multiplane_add_plane_cyl.stp: an earlier, failed Plane candidate
    # raised the base fragment's own tolerance 5x (1.2e-7 -> 6.0e-7), and a
    # LATER candidate that would have split it correctly (3 pieces summing
    # exactly to the input volume, on a pristine copy) instead
    # under-separated it (2 pieces, losing a whole 276 mm^3 fragment) once
    # tried against the now-degraded shape -- the volume-conservation guard
    # below correctly rejected that wrong split, but no OTHER candidate ever
    # got a chance to try against a clean base, so the fragment was left
    # permanently unsplit. A tolerance reset alone (Gsolid_set_tolerance)
    # was tried and confirmed INSUFFICIENT: instrumented directly, face/
    # edge/vertex counts and every vertex position stayed bit-identical
    # across repeated "failed" attempts even with the tolerance reset
    # applied each time, yet the base's own `is_valid()` (BRepCheck_Analyzer)
    # flipped True -> False -- BOPAlgo_Splitter had corrupted the shape's
    # internal parametrization consistency (pcurve/SameParameter state), not
    # its tolerance or geometry, and that is what made the later candidate
    # misbehave. Fixed by resetting BOTH the tolerance AND re-running the
    # same validity fix already used once at the top of this function,
    # before every new candidate is tried, not just once at the top.
    # freecad has no equivalent tolerance-reset primitive
    # (Gsolid_set_tolerance) yet -- occ/ocp only, matching every other
    # native-tolerance-inspection feature in this cascade; its own `.fix()`
    # re-run still applies there since it's engine-agnostic.
    if CAD_ENGINE != "freecad":
        from ...geo import Gsolid_set_tolerance

        original_tolerance = Gsolid_max_tolerance(solid)
    else:
        original_tolerance = None

    # Candidate-surface detection (cutting planes, MultiPlane/Can/TCone/
    # RoundCorner masters, ...) must judge a face's own min_area/
    # min_face_width against THIS solid's own scale, exactly like
    # cell_definition.py::simple_solid_definition's own scaled_tolerances
    # -- otherwise the same face can be read as a genuine feature at the
    # cell-definition stage (scaled) but as a sliver here (unscaled),
    # confirmed live on Solidos/working_solids/clamp.stp's solid0: two
    # real, 0.081mm-wide corner-forming planes (CharacteristicWidth
    # 0.081115) sit strictly between the default min_face_width=0.1 and
    # this solid's own scaled 0.068 -- eligible_plane/order_plane_face
    # read them as slivers here (unscaled tolerances), while
    # cell_definition.py's own get_multiplanes (scaled) correctly reads
    # them as real geometry. The `Gsplit`/`build_surface`/`solid.fix()`
    # calls below keep the ORIGINAL, unscaled `tolerances` -- those are
    # kernel-operation precision (split/fix tolerance), not surface-size
    # classification, and scaling them down for a small fragment is not
    # the concern this scaling exists for.
    scaled_tolerances = tolerances.scaled(solid.Volume)

    new_split = False
    # The first candidate that successfully splits `solid` is not
    # necessarily the BEST one: a candidate tried early (per the
    # established cylinder/cone-before-plane ordering) can "win" by
    # carving off only sliver-sized debris from what is still, in
    # substance, a single real piece, while a LATER candidate -- tried
    # only if this one is rejected -- can give a genuinely clean,
    # multi-piece split instead. Confirmed live, 2026-10-05, on
    # Solidos/working_solids/clamp.stp's solid0: an R=5.25 cylinder
    # candidate "succeeds" first, splitting off two 0.0837 mm^3 wedges
    # that merge_sliver_pieces (see its own docstring) then fuses right
    # back -- but a real, 0.081 mm-wide corner-forming plane, tried much
    # later (ranked low by cutting_face_number), gives a genuinely clean
    # 3-piece split (188.74 + 62.95 + 62.95 mm^3, none of them slivers)
    # that was never reached because the cylinder candidate already broke
    # this loop. Per direct user instruction, merge_sliver_pieces exists
    # for the case the SOURCE solid itself has a genuine, unavoidable
    # geometric imprecision that forces a sliver fragment no matter which
    # candidate is used (see struct_plate3.stp, CLAUDE.md) -- it is a
    # last resort, not something a candidate should be allowed to trigger
    # just by being tried first. `fallback_solids` remembers the first
    # sliver-producing split found (in case nothing better ever turns
    # up); only a split leaving >=2 genuinely non-sliver pieces is
    # accepted immediately.
    fallback_solids = None
    for surf in get_surfaces(solid, omitfaces, scaled_tolerances, options, meta_surface=options.meta_surfaces):
        if original_tolerance is not None:
            Gsolid_set_tolerance(solid, original_tolerance)
        if not solid.is_valid():
            fixed = solid.fix(tolerances.fix_tolerance)
            if fixed.is_valid():
                solid = fixed
        try:
            # build_surface (Can/RoundCorner/... construction, via
            # get_cell_object) can raise -- e.g. round_corner_region's own
            # "this configuration should not exist" sanity-check assertion
            # on a genuinely degenerate near-tangency configuration -- not
            # just return shape=None. Both outcomes mean the same thing to
            # this loop ("this candidate surface could not be built, try
            # the next one"), so both are handled the same way: log and
            # move on, never let one candidate's failure abort the whole
            # decomposition. Confirmed reachable in practice once
            # large_cell_plane_split started feeding many-face solids
            # through this loop -- their synthetic cutting-plane faces
            # create new corner configurations this classifier was never
            # exercised against before.
            surf.build_surface(bbox, tolerances, forward=True)
            if surf.shape is None:
                logger.info(f"Cannot build {surf.Type} surface.")
                continue
            result = Gsplit(
                solid,
                GSolid(surf.shape),
                tolerances,
            )
            comsolid_solids = result.solids
            if result.dropped_no_solid:
                vols = ", ".join(f"{v:.2f}" for v in result.dropped_no_solid)
                logger.warning(
                    f"Gsplit: {len(result.dropped_no_solid)} candidate fragment(s) (volumes: {vols}) never "
                    "resolved to a real solid and were dropped -- this material will NOT be part of any "
                    "cell's boolean expression."
                )
        except Exception as e:
            comsolid_solids = [solid]
            logger.info(f"Failed to build or split base with {surf.Type} surface: {e}")

        if len(comsolid_solids) > 1:
            # A "successful" split (>=2 sane solids returned) is not
            # necessarily a CORRECT one -- confirmed live, 2026-09-12,
            # Big_complex_cell/modelCell_670000.stp: a MultiPlane candidate
            # against a heavily-recut fragment (many nested large_cell_
            # plane_split cuts deep) returned 2 real, valid, tiny slivers
            # (volumes 1204/1475) while BOPAlgo_Splitter silently merged
            # the other ~3.88 million mm^3 of real material away entirely
            # -- a raw split, not filtered by anything downstream, since
            # both kept pieces are genuinely valid and above every size
            # threshold. Re-running the identical (unhealed) base+tool
            # pair confirmed this deterministically: _raw_bop_split itself
            # only ever finds 2 pieces on the live, tolerance-accumulated
            # geometry, but does find all 4 real pieces (matching the base
            # volume exactly) once the SAME base is healed via a STEP
            # round-trip first -- the exact class of BOPAlgo tolerance-weld
            # this file's own STEP-heal retry below already exists to catch,
            # just never triggered here because a (silently wrong) split
            # DID technically happen. Guard against this directly: verify
            # the pieces actually tile the base's own volume before trusting
            # the split at all.
            piece_sum = sum(abs(p.Volume) for p in comsolid_solids)
            orig_vol = abs(solid.Volume)
            if orig_vol > 0 and not volume_within(piece_sum, orig_vol, SPLIT_CANDIDATE_VOLUME_REL_TOL):
                logger.warning(
                    f"Gsplit with a {surf.Type} surface produced {len(comsolid_solids)} piece(s) summing to "
                    f"{piece_sum:.2f}, but the base fragment's own volume is {orig_vol:.2f} -- a likely "
                    "BOPAlgo under-separation (real material silently merged away) rather than a genuine "
                    "split. Discarding this candidate and trying the next one."
                )
                comsolid_solids = [solid]
                continue

            non_sliver_pieces = [p for p in comsolid_solids if not is_sliver_solid(p)]
            if len(non_sliver_pieces) <= 1:
                # At most one of the returned pieces is genuinely real --
                # the rest are the same class of thin-wedge artifact
                # merge_sliver_pieces exists to fuse back later. Keep this
                # as a fallback (only used if the whole candidate search
                # below never finds anything better), but don't treat it
                # as a win: trying the next candidate instead.
                if fallback_solids is None:
                    fallback_solids = comsolid_solids
                comsolid_solids = [solid]
                continue

            new_split = True
            break

    # No candidate surface separated this fragment -- every real tool was
    # genuinely tried and failed. This alone does NOT mean the fragment
    # needs repair: it is also the ordinary, correct signature of a
    # genuinely irreducible solid, and the two cases are NOT
    # distinguishable from tolerance/validity state alone (see
    # _some_real_candidate_should_have_split's own docstring for the
    # measurements that ruled out tolerance magnitude, SameParameter/
    # SameRange, tolerance-distribution uniformity, and a direct
    # geometric near-tangency search). Only retry with a STEP round-trip
    # heal when an INDEPENDENT geometric check confirms at least one of
    # the real candidates just tried genuinely should have split this
    # fragment's own material -- never pre-emptively, and never on every
    # successfully-split piece either (both tried and rejected: the
    # former crashes OCCT natively on fragments that never needed it --
    # testing/inputSTEP/BC.stp's own stuck-but-correct fragments -- the
    # latter perturbs geometry that was already fine and reactivates an
    # unrelated, already-fixed RevCC closure false positive on
    # Hollow_plates/placa.stp). `.fix()` first, matching split_surfaces's
    # own final pass on every OTHER returned piece: skipping it leaves a
    # fragment at its raw, as-split state, whose own STEP round-trip can
    # drift just outside MAX_REPAIR_VOLUME_REL_CHANGE (measured directly,
    # 4.52e-4 relative) so Gheal_topology silently rejects the rebuild;
    # .fix() alone already reduces it to the same, cleaner form
    # split_surfaces's own pass would produce, whose own round-trip drifts
    # only 3.50e-4 -- comfortably inside the existing gate. Confirmed on
    # RevCC_regression/Big_one_cell__modelCell_670000__solid0_piece52 (the
    # original tolerance-weld fixture this mechanism was first built for)
    # and on Big_model_reserved/divertor_cam.stp's own EC_SUPPORT#BYZ2TW_41
    # piece0 (the fragment with no non-manifold edges that the former,
    # narrower tolerance+non-manifold signature could never reach at all).
    if not new_split and not healed:
        try:
            fixed = solid.fix(tolerances.fix_tolerance)
        except Exception:
            fixed = solid
        # .fix() is a no-op for the large majority of genuinely-stuck
        # fragments (measured directly, 2026-10-09, on
        # testing/inputSTEP/BC.stp's own 15 stuck fragments: 14/15 come
        # back with the exact same face count and volume) -- and when it
        # changes nothing, there is no reason to expect get_surfaces() to
        # offer a different candidate set either, so the expensive part
        # below (rebuilding every real candidate's own native shape AND
        # the point-sampling check) is skipped entirely for those. This
        # is what actually keeps this whole mechanism's average cost low
        # -- confirmed this alone brings testing/inputSTEP/BC.stp's own
        # decomposition time back down from 69.7s (every stuck fragment
        # paying the full rebuild+sample cost) to 23.6s -- a real
        # remaining gap over this file's own pre-this-feature baseline
        # (8.3s), but legitimate: the one fragment .fix() DOES change
        # (29->28 faces) genuinely resolves into further real pieces
        # (11->14 final pieces total) via this exact mechanism, with the
        # file's own true-vs-decomposed volume match IMPROVING (3.19e-3
        # -> 1.44e-3 relative) alongside it, not a sign of wasted work.
        fix_changed_something = (
            len(fixed.Faces) != len(solid.Faces)
            or not volume_within(fixed.Volume, solid.Volume, NUMERIC_DOUBLE_TOL)
        ) and _fix_result_is_sane(fixed, solid.Volume)
        if fix_changed_something:
            # Rebuild the real candidate list against the FIXED geometry,
            # not the stale `tried_tools` collected before .fix() ran --
            # .fix()'s own face-merging (e.g. 11->7 faces on
            # Big_model_reserved/divertor_cam.stp's own
            # EC_SUPPORT#BYZ2TW_41 piece0) can be exactly what exposes the
            # one candidate that should split this fragment:
            # get_surfaces() generates candidates from the CURRENT face
            # structure, so a candidate collected against the unfixed
            # solid is simply not the same tool. Confirmed live: testing
            # `tried_tools` (pre-fix) against `solid` (pre-fix) found no
            # anomaly here even though the real, correct candidate
            # exists -- it only forms once get_surfaces() sees the fixed,
            # merged faces.
            fresh_tools = []
            # A fresh, empty omitfaces: the stale set (populated against
            # the UNFIXED solid's own face indices during the candidate
            # loop above) is meaningless here -- .fix() can renumber/
            # merge faces, so the same integer index refers to a
            # different face (or none) in `fixed`. Confirmed live:
            # reusing the stale set left only 4 fresh candidates instead
            # of the expected full set, silently excluding the genuine
            # one.
            for surf in get_surfaces(fixed, set(), scaled_tolerances, options, meta_surface=options.meta_surfaces):
                try:
                    surf.build_surface(bbox, tolerances, forward=True)
                except Exception:
                    continue
                if surf.shape is not None:
                    fresh_tools.append(surf)
        else:
            fresh_tools = []
        if fresh_tools and _some_real_candidate_should_have_split(fixed, fresh_tools):
            rebuilt = Gheal_topology(fixed)
            if rebuilt is not None:
                try:
                    # The heal is opportunistic: if the rebuilt solid trips
                    # anything downstream, keep the un-healed fragment --
                    # no worse off than not having tried.
                    return generic_split(rebuilt, options, tolerances, loop, healed=True)
                except Exception:
                    logger.info("STEP round-trip heal retry failed; keeping the original fragment")

    if not new_split and fallback_solids is not None:
        # Nothing cleaner ever turned up (and the tolerance-weld heal
        # retry above either didn't apply or didn't help either) -- this
        # solid genuinely needs a sliver-producing split to decompose at
        # all, matching struct_plate3.stp's own real, unavoidable
        # source-CAD imprecision (see merge_sliver_pieces's own docstring).
        # Only now is that candidate accepted.
        comsolid_solids = fallback_solids
        new_split = True

    if new_split:
        components = []
        for part in comsolid_solids:
            subcomp = generic_split(part, options, tolerances, loop + 1)
            components.extend(subcomp)
    else:
        # NOTE: an earlier version of this branch unconditionally re-ran
        # .fix() on every piece here, based on an initial (wrong) diagnosis
        # of the RoundCorners/rrc23.stp volume-corruption bug -- the real
        # cause turned out to be in split_surfaces's own Gmake_compound
        # wrap/re-extract step (see that function's own comment), not here.
        # Removed: confirmed live on Complex_cell/SCDR_90.stp that an
        # unconditional .fix() at every recursive leaf can itself trigger a
        # real native crash (Standard_Failure: "Courbes non jointives",
        # the same ShapeUpgrade_UnifySameDomain crash class already
        # documented for Mixed/ConeSphere.stp) on a piece that never needed
        # fixing in the first place -- strictly more crash-prone than the
        # single, narrowly-scoped fix in split_surfaces, for no benefit.
        components = comsolid_solids
    return components


def main_split(solids, options, tolerances):
    """decompose in basic solids a list of solids from CAD.

    `solids` is a plain list of GSolid (the top-level input solids, usually
    just one) -- never a native compound: see GCompound's own docstring for
    why this whole pipeline avoids native TopoDS_Compound wrap/re-extract
    cycles for anything other than a one-shot STEP export.
    """
    solid_parts = []

    for solid in solids:
        piece = split_surfaces(solid, options, tolerances)
        solid_parts.extend(piece.Solids)

    return GCompound(solid_parts)
