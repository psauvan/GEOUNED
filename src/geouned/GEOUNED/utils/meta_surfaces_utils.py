import math

from collections import OrderedDict

from .geometry_gu import ShellFaceGu, FaceGu, other_face_edge, is_same_surface
from .geouned_classes import GeounedSurface
from .data_classes import Tolerances
from .basic_functions_part1 import twoPimod
from .basic_functions_part2 import is_same_plane
from .data_constants import twoPi, mask
from ..utils.basic_functions_part1 import is_parallel, shapes_in_contact
from ..conversion.cell_definition_functions import gen_cone, gen_cylinder, cone_apex_plane
from ...geo import (
    GEdge,
    GFace,
    GVector,
    GPlane,
    GCylinder,
    GCone,
    GSphere,
    GTorus,
    GLine,
    GCircle,
    GEllipse,
    GBSpline,
    Gclassify_curve,
    surface_geometry,
    vector_geometry,
)
from ...geo.constants import MIN_SLIVER_EDGE_LENGTH


class reversedCCP:
    def __init__(self, surfType, params):
        self.Type = surfType
        self.Surf_index = set()
        self.Params = params
        self.Index = None


def remove_twice_parallel(mplanes):
    plane_list = []
    omit = set()
    for i, p1 in enumerate(mplanes):
        if p1.Index in omit:
            continue
        parallel = []
        for p2 in mplanes[i + 1 :]:
            if p2.Index in omit:
                continue
            if surface_geometry.is_parallel_plane_surface(p1.Surface, p2.Surface):
                parallel.append(p2)
                omit.add(p2.Index)
        if len(parallel) > 1:
            parallel.append(p1)
            omit.add(p1.Index)
            plane_list.append(parallel)

    for parallel in plane_list:
        p0 = parallel[0]
        dmin = 0
        dmax = 0
        pmin = p0
        pmax = p0
        for i, p in enumerate(parallel[1:]):
            d = p0.Surface.Axis.dot(p.Surface.Position - p0.Surface.Position)
            if d > dmax:
                dmax = d
                pmax = p
            elif d < dmin:
                dmin = d
                pmin = p

        if dmax - dmin < 1e-5:
            continue

        for p in reversed(parallel):
            if surface_geometry.is_same_plane_surface(p.Surface, pmin.Surface):
                parallel.remove(p)
            elif surface_geometry.is_same_plane_surface(p.Surface, pmax.Surface):
                parallel.remove(p)

        for p in parallel:
            mplanes.remove(p)


def convex_wire(p):
    Edges = p.OuterWire.Edges
    for e in Edges:
        if type(Gclassify_curve(e)) is not GLine:
            return []

    axis = p.Surface.Axis
    normal = axis
    v0 = Edges[0].Curve.Direction.cross(axis)
    if Edges[0].Orientation == "Forward":
        v0 = -v0

    for e in Edges[1:]:
        v1 = e.Curve.Direction.cross(axis)
        if e.Orientation == "Forward":
            v1 = -v1
        if normal.dot(v0.cross(v1)) < 0:
            return []
        v0 = v1

    return Edges


def get_adjacent_cylplane(cyl, Faces, cornerPlanes=True, axial_bounds=None):
    """Find the planar faces adjacent to a cylinder/cone `cyl` (a `FaceGu`,
    or a `ShellFaceGu` of several contiguous same-surface pieces).

    Two modes, fully independent:

    - `cornerPlanes=True` (RoundCorner corner-plane search): walk the
      *straight* (GLine) boundary edges and return, for each, a
      `(cyl, touching_edge, near_face, otherface)` tuple where `otherface`
      is a plane whose normal is perpendicular to `cyl`'s axis -- a round
      corner's bounding plane. `touching_edge`/`near_face` are where
      `otherface` actually borders the cylinder (possibly a sliver bridging
      them), the real boundary `get_additional_corner_plane` evaluates at;
      the cylinder's own edge `e` is only used locally to reach `otherface`
      and is not carried out.

    - `cornerPlanes=False` (axial end-cap search, e.g. RevCC/MultiPlane):
      walk the *curved* boundary edges and return the bare planar faces
      found across them.

    The `if cornerPlanes:` split is the outer one; the ShellFaceGu handling
    is a sub-case of each, and is pure fan-out: recurse over every piece
    with the same mode, then pool + deduplicate by the found plane's own
    Index (a corner plane may be reachable only from one specific piece's
    edges when a cut left each piece touching a different bounding plane).
    """
    is_shell = type(cyl) is ShellFaceGu

    # ---- cornerPlanes = True : RoundCorner bounding-plane search ----------
    if cornerPlanes:
        if is_shell:
            planes = []
            seen_index = set()
            for f in cyl.Faces:
                for item in get_adjacent_cylplane(f, Faces, True, axial_bounds):
                    if item[3].Index in seen_index:
                        continue
                    seen_index.add(item[3].Index)
                    planes.append(item)
            return planes

        planes = []
        for e in cyl.OuterWire.Edges:
            if type(Gclassify_curve(e)) is not GLine:
                continue
            result = other_face_edge(e, cyl, Faces, outer_only=True, skip_slivers=True)
            if result is None:
                continue
            touching_edge, near_face, otherface = result
            if not isinstance(otherface.Surface, GPlane):
                continue
            if abs(otherface.Surface.Axis.dot(cyl.Surface.Axis)) < 1.0e-4:
                planes.append((cyl, touching_edge, near_face, otherface))

        # If both of the cylinder's own corner edges close against the
        # *same* real face (same Index -- the identical physical plane
        # closing both ends, not just two geometrically-coincident faces on
        # opposite sides), that's not a valid RoundCorner: a real round
        # corner needs two distinct closing planes. Deduplicating here
        # collapses that case down to a single entry, so the caller's own
        # existing `len(adjacent_planes) != 2` check rejects it naturally --
        # no special-case needed downstream. Confirmed concretely
        # (Solidos/working_solids/rc1_decomp.stp): when the same face closes
        # both ends, the "additional plane" pd (through both e1 and e2) *is*
        # that same face, so cyl_plane_region_conf's p1_pd/p2_pd angle test
        # sits at an exact, meaningless 0-degree singularity, producing an
        # unconditional AND_p1_pd/AND_p2_pd disagreement -- not a tangency
        # or numerical-noise issue, a genuinely invalid RoundCorner premise.
        # The legitimate "p1 == p2 geometrically, but from 2 disjoint real
        # faces" case (round_corner_region's own p1id == p2id branch) is
        # unaffected -- those are different Index values here.
        seen_index = set()
        deduped = []
        for item in planes:
            if item[3].Index in seen_index:
                continue
            seen_index.add(item[3].Index)
            deduped.append(item)
        return deduped

    # ---- cornerPlanes = False : axial end-cap plane search ---------------
    if is_shell:
        planes = []
        seen_index = set()
        for f in cyl.Faces:
            for p in get_adjacent_cylplane(f, Faces, False, axial_bounds):
                if p.Index in seen_index:
                    continue
                seen_index.add(p.Index)
                planes.append(p)
        return planes

    planes = []
    for e in cyl.OuterWire.Edges:
        if type(Gclassify_curve(e)) is GLine:
            continue
        if axial_bounds is not None:
            # Not every curved boundary edge of cyl's own OuterWire is a
            # real axial end cap -- a hole/notch/step cut through the middle
            # of the cylinder/cone also leaves a curved edge, and a real
            # plane found across *that* one is not a legitimate closing
            # plane for the surface as a whole. Confirmed live
            # (Big_model_reserved/TVA_final_allencl.stp solid8 piece0): the
            # RevCC's own cylinder segment spans Z=[-1000, 6500], but this
            # unfiltered walk picked up an unrelated real plane at Z=4000 --
            # restricting to edges actually sitting at the surface's own
            # axial extreme (Vmin/Vmax) excludes it.
            pnt = 0.5 * (e.Vertexes[0] + e.Vertexes[-1])
            _, v = cyl.parameter(pnt)
            vmin, vmax = axial_bounds
            tol = 1e-3 * max(abs(vmax - vmin), 1.0)
            if abs(v - vmin) > tol and abs(v - vmax) > tol:
                continue
        result = other_face_edge(e, cyl, Faces, outer_only=False, skip_slivers=True)
        if result is None:
            continue
        _, _, otherface = result
        if isinstance(otherface.Surface, GPlane):
            planes.append(otherface)

    delindex = set()
    for i, p1 in enumerate(planes):
        for j, p2 in enumerate(planes[i + 1 :]):
            if p1.isSame(p2):
                delindex.add(j + i + 1)
    for i in sorted(delindex, reverse=True):
        del planes[i]

    return planes


def get_adjacent_cylknesurf(cylkne, Faces):
    if type(cylkne) is ShellFaceGu:
        adjacent = []
        adjIndexes = set()
        surface = cylkne.Faces[0].Surface
        for f in cylkne.Faces:
            adj = get_adjacent_cylknesurfFace(f, Faces)
            for af in adj:
                if is_same_surface(af.Surface, surface):
                    continue
                if af.Index not in adjIndexes:
                    adjIndexes.add(af.Index)
                    adjacent.append(af)

        if len(adjacent) > 2:
            adjacent, dummy = most_outer_faces(cylkne, adjacent)
        return adjacent

    else:
        return get_adjacent_cylknesurfFace(cylkne, Faces)


def get_adjacent_cylknesurfFace(cylkne, Faces):
    adjfaces = []

    other_index = set()
    for e in cylkne.OuterWire.Edges:
        if e.Length < 1e-6:
            continue
        if type(Gclassify_curve(e)) is GLine:
            continue
        result = other_face_edge(e, cylkne, Faces, outer_only=False, skip_slivers=True)
        if result is None:
            continue
        _, _, otherface = result
        if otherface.Index in other_index:
            continue

        other_index.add(otherface.Index)
        adjfaces.append(otherface)

    delindex = set()
    for i, s1 in enumerate(adjfaces):
        for j, s2 in enumerate(adjfaces[i + 1 :]):
            if s1.isSame(s2):
                delindex.add(j + i + 1)

    delindex = list(delindex)
    delindex.sort()
    delindex.reverse()
    for i in delindex:
        del adjfaces[i]

    return adjfaces


_WINDING_N_SAMPLES = 16
_WINDING_ANGLE_TOL = 2e-3  # rad


def _oriented_angle_sweep(angle_of, edge, v_from, v_to):
    """Cumulative unwrapped angle swept by `edge`, sampled and oriented so
    it runs from `v_from` to `v_to` (the wire's own traversal direction)."""
    p0, p1 = edge.ParameterRange
    e0 = edge.value_at(p0)
    e1 = edge.value_at(p1)
    forward = (e0 - v_from).length + (e1 - v_to).length <= (e0 - v_to).length + (e1 - v_from).length
    params = [p0 + (p1 - p0) * k / _WINDING_N_SAMPLES for k in range(_WINDING_N_SAMPLES + 1)]
    if not forward:
        params.reverse()
    angles = [angle_of(edge.value_at(p)) for p in params]
    cumulative = 0.0
    for a0, a1 in zip(angles, angles[1:]):
        d = a1 - a0
        while d > math.pi:
            d -= twoPi
        while d < -math.pi:
            d += twoPi
        cumulative += d
    return cumulative


def _closes_full_turn(total_angle):
    k = round(total_angle / twoPi)
    return k != 0 and abs(total_angle - k * twoPi) <= _WINDING_ANGLE_TOL


def _perpendicular_axis(axis):
    """An arbitrary, stable unit vector perpendicular to `axis`. Only
    relative angle *changes* around `axis` are ever measured (never an
    absolute reference angle), so any fixed choice works -- this avoids
    depending on a surface's own XDir, which GCone doesn't carry at all
    and GCylinder only has when built from a real native face."""
    ref = GVector(1, 0, 0) if abs(axis.dot(GVector(1, 0, 0))) < 0.9 else GVector(0, 1, 0)
    return axis.cross(ref).normalized()


def _surface_axis_origin_e1(surf):
    """(axis, a point on that axis, a reference direction perpendicular to
    axis) for the two surface types this closure check supports, or None
    for anything else. GCylinder uses its own real XDir when it has one
    (matching the exact frame this check was validated against -- the
    winding math is rotation-invariant in theory, but the discretized,
    tolerance-bounded implementation isn't perfectly so in practice, so
    the validated frame is kept rather than swapped for a generic one);
    GCone has no XDir at all, so it always gets an arbitrary (but stable)
    perpendicular vector instead."""
    if type(surf) is GCylinder:
        e1 = surf.XDir if surf.XDir is not None else _perpendicular_axis(surf.Axis)
        return surf.Axis, surf.Center, e1
    if type(surf) is GCone:
        return surf.Axis, surf.Apex, _perpendicular_axis(surf.Axis)
    return None


def _angle_function(axis, origin, e1):
    e2 = axis.cross(e1)

    def angle_of(point):
        rel = point - origin
        rel = rel - rel.dot(axis) * axis
        return math.atan2(rel.dot(e2), rel.dot(e1))

    return angle_of


def _wire_oriented_edges(wire):
    """(edge, v_from, v_to) triples for every edge of `wire`, in traversal order."""
    verts = wire.OrderedVertexes
    return [(e, verts[i], verts[(i + 1) % len(verts)]) for i, e in enumerate(wire.Edges)]


def _boundary_edges_of_merged_faces(faces):
    """The edges that bound the *union* of several faces known to lie on
    the same analytic surface (a ShellGu's merged pieces): an edge shared
    between two of these faces is an internal seam where they join, not
    part of the union's outer boundary, and is excluded. Detected purely
    by how many times each edge occurs across all the faces' own wires --
    once (odd) means it's on the boundary, twice (even) means it's an
    internal join."""
    all_edges = [e for face in faces for wire in face.wires() for e in wire.Edges]
    counts = [1] * len(all_edges)
    for i in range(len(all_edges)):
        for j in range(i + 1, len(all_edges)):
            if all_edges[i].is_same(all_edges[j]):
                counts[i] += 1
                counts[j] += 1
    return [e for e, c in zip(all_edges, counts) if c % 2 == 1]


def _assemble_boundary_loops(edges):
    """Group an unordered set of boundary edges into one or more ordered
    closed loops by chaining shared endpoints, each as a list of
    (edge, v_from, v_to) triples -- the same shape `_wire_oriented_edges`
    produces for a real wire, so both can feed `_loop_closes_full_turn`."""
    remaining = list(edges)
    loops = []
    while remaining:
        e0 = remaining.pop(0)
        v_start = e0.Vertexes[0]
        current = e0.Vertexes[-1]
        loop = [(e0, v_start, current)]
        while (current - v_start).length > 1e-6 and remaining:
            for i, e in enumerate(remaining):
                v0, v1 = e.Vertexes[0], e.Vertexes[-1]
                if (v0 - current).length < 1e-6:
                    loop.append((e, v0, v1))
                    current = v1
                    remaining.pop(i)
                    break
                elif (v1 - current).length < 1e-6:
                    loop.append((e, v1, v0))
                    current = v0
                    remaining.pop(i)
                    break
            else:
                break  # dangling/malformed boundary -- stop this loop here
        loops.append(loop)
    return loops


def _loop_sample_points(oriented_edges):
    """Flatten a loop's (edge, v_from, v_to) triples into one ordered list
    of sampled 3D points around the whole loop -- fine per-edge sampling,
    not just each edge's own net sweep, so a direction reversal is caught
    even when it happens partway through a single curved (e.g. BSpline)
    edge rather than only at edge boundaries."""
    pts = []
    for edge, v_from, v_to in oriented_edges:
        p0, p1 = edge.ParameterRange
        e0 = edge.value_at(p0)
        e1 = edge.value_at(p1)
        forward = (e0 - v_from).length + (e1 - v_to).length <= (e0 - v_to).length + (e1 - v_from).length
        params = [p0 + (p1 - p0) * k / _WINDING_N_SAMPLES for k in range(_WINDING_N_SAMPLES + 1)]
        if not forward:
            params.reverse()
        pts.extend(edge.value_at(p) for p in params[:-1])  # drop last -- it's the next edge's first sample
    return pts


def _loop_closes_full_turn(angle_of, oriented_edges):
    """True if walking `oriented_edges` (a closed loop, as (edge, v_from,
    v_to) triples in traversal order) genuinely wraps the axis behind
    `angle_of` a full 360deg -- unlike checking a face's raw UV bounding
    box (which a jagged, irregular trim can satisfy without actually
    being closed, e.g. a bad cut whose boundary drifts to a different
    height instead of closing cleanly), this requires that every maximal
    run of consistent angular direction close to an exact multiple of
    2*pi before it's allowed to reverse sense. A genuine annulus-shaped
    boundary (top rim + bottom rim + seam) legitimately reverses sense
    between the two rims -- that alone is not a sign of a broken face --
    but each rim must complete a full lap, not a partial arc. Since the
    loop is a cycle, a run can straddle the arbitrary start/end of the
    sample sequence (e.g. a rim split into two arcs that happen to be the
    list's first and last entries) -- handled by rotating to start right
    after a genuine direction change before partitioning into runs. Works
    on individually sampled points around the whole loop, not per-edge
    net sweeps, so a reversal partway through one curved edge is caught
    too, not just reversals that happen to fall on an edge boundary."""
    # fast path: a single edge that already closes on itself (start ==
    # end vertex) and genuinely sweeps a full turn on its own proves the
    # loop wraps the axis completely, regardless of anything else in it.
    if any(
        (e.Vertexes[0] - e.Vertexes[-1]).length < 1e-6
        and e.Length > 1e-3
        and _closes_full_turn(_oriented_angle_sweep(angle_of, e, v_from, v_to))
        for e, v_from, v_to in oriented_edges
    ):
        return True

    points = _loop_sample_points(oriented_edges)
    if len(points) < 3:
        return False
    angles = [angle_of(p) for p in points]

    n = len(angles)
    deltas = []
    for i in range(n):
        d = angles[(i + 1) % n] - angles[i]
        while d > math.pi:
            d -= twoPi
        while d < -math.pi:
            d += twoPi
        deltas.append(d)

    nonzero = [d for d in deltas if abs(d) >= 1e-4]
    if not nonzero:
        return False

    split_idx = next(
        (i for i in range(len(nonzero)) if (nonzero[i] > 0) != (nonzero[i - 1] > 0)),
        None,
    )
    rotated = nonzero if split_idx is None else nonzero[split_idx:] + nonzero[:split_idx]

    run_total = 0.0
    run_dir = 0
    for d in rotated:
        this_dir = 1 if d > 0 else -1
        if run_dir == 0:
            run_dir = this_dir
            run_total = d
        elif this_dir == run_dir:
            run_total += d
        else:
            if not _closes_full_turn(run_total):
                return False
            run_dir = this_dir
            run_total = d
    return _closes_full_turn(run_total)


def _is_closed_by_winding(shape):
    """True/False if `shape` (a single face, or a ShellGu of several faces
    known to share the same analytic surface) genuinely closes a full
    360deg around its cylinder/cone's own axis; None if the surface type
    isn't one this check supports (caller should fall back)."""
    if type(shape) is ShellFaceGu:
        params = _surface_axis_origin_e1(shape.Faces[0].Surface)
        if params is None:
            return None
        angle_of = _angle_function(*params)
        boundary = _boundary_edges_of_merged_faces(shape.Faces)
        return any(_loop_closes_full_turn(angle_of, loop) for loop in _assemble_boundary_loops(boundary))

    params = _surface_axis_origin_e1(shape.Surface)
    if params is None:
        return None
    angle_of = _angle_function(*params)
    return any(_loop_closes_full_turn(angle_of, _wire_oriented_edges(wire)) for wire in shape.wires())


def is_closed_cylinder_cone(shape):
    result = _is_closed_by_winding(shape)
    if result is not None:
        return result

    if type(shape) is not ShellFaceGu:
        umin, umax, vmin, vmax = shape.ParameterRange
        return umax - umin > twoPi - 1e-5

    Urange = []
    for f in shape.Faces:
        umin, umax, vmin, vmax = f.ParameterRange
        umin, umax = twoPimod(umin), twoPimod(umax)
        if umin > umax:
            Urange.append((umin - twoPi, umax))
        else:
            Urange.append((umin, umax))

    Urange.sort()
    Umin, Umax = Urange[0]
    if Umin < 0:
        Umin0 = Umin + twoPi
    else:
        Umin0 = Umin

    angle = Umax - Umin
    for umin, umax in Urange[1:]:
        if umax <= Umax:
            continue
        elif umin - Umax < 1e-5:
            angle += umax - Umax
            Umax = umax
            if angle > twoPi - 1e-5:
                return True
        else:
            return False
    return angle > twoPi - 1e-5


def get_side_edges(cylinder_faces):

    origin = cylinder_faces[0].Surface.Center
    axis = cylinder_faces[0].Surface.Axis
    sideLow = (1e15, None)
    sideHigh = (-1e15, None)

    for ic, cyl in enumerate(cylinder_faces):
        for ie, e in enumerate(cyl.OuterWire.Edges):
            if type(Gclassify_curve(e)) is GLine:
                continue
            D = axis.dot(e.CenterOfMass - origin)
            if D < sideLow[0]:
                sideLow = (D, (ic, ie))
            if D > sideHigh[0]:
                sideHigh = (D, (ic, ie))

    facelow = cylinder_faces[sideLow[1][0]]
    facehigh = cylinder_faces[sideHigh[1][0]]
    edgelow = facelow.OuterWire.Edges[sideLow[1][1]]
    edgehigh = facehigh.OuterWire.Edges[sideHigh[1][1]]

    return (facelow, edgelow, axis), (facehigh, edgehigh, axis)


def face_in_cylinder(edge, face):
    axis = face.Surface.Axis
    if type(Gclassify_curve(edge)) is GBSpline:
        return edge.Curve.getD0(0).dot(axis) < 0
    else:
        return edge.Curve.Axis.dot(axis) < 0


def convex_face_cyl(cyl, edge, otherface):
    v1 = cyl.CenterOfMass - edge.CenterOfMass
    v2 = otherface.CenterOfMass - edge.CenterOfMass
    return v1.dot(v2) < 0


def _find_adjacent_multiplane_planes(shell_or_face, GUFaces, multiplanes, tolerances):
    """Real MultiPlane component planes physically adjacent to this RevCC
    segment's own cylinder/cone (`face`, possibly merged into a ShellGu of
    several same-analytic-surface pieces via merge_same_surface_faces --
    a boolean cut can split what's really one cylinder/cone into several
    contiguous fragments).

    Needed because a MultiPlane can make the irreducible solid non-convex,
    which is exactly the configuration where the RevCC's own additional
    plane p -- correct only locally, near its own cylinder/cone -- must
    not be applied as an unrestricted global cut; identifying which real
    faces actually border this segment is the first step toward limiting
    it there.

    Search direction is inverted from an earlier version of this function
    (per direct user instruction, 2026-08-23): rather than first guessing
    *which* boundary edges could plausibly border a closing plane (a
    curved-edge-only, axial-bounds-with-a-fixed-tolerance heuristic borrowed
    from get_adjacent_cylplane) and only then checking whether what's found
    happens to be a known MultiPlane component, this walks *every* edge of
    the segment's own shell and checks directly whether its real neighbor
    (via other_face_edge, tolerant of residual slivers) is already known to
    be one of `multiplanes`' own component planes -- if so, it's added,
    full stop. This sidesteps the earlier heuristic's real, confirmed
    failure mode: a cylinder/cone cut by a non-perpendicular plane has an
    *elliptical* rim edge whose own midpoint does not sit at the face's own
    true V-extreme the way a perpendicular cut's circular rim does (off by
    ~0.05-0.06 against a tolerance of ~0.003 -- confirmed live,
    Solidos/test_models/Mixed/multiplane_add_plane_cyl.stp -- so the old
    heuristic silently found nothing at all for a real, adjacent MultiPlane
    plane). Checking every edge against the already-known candidate set is
    both more direct (no shape/position heuristic to get subtly wrong) and
    strictly no more expensive than it looks: `multiplanes` is normally a
    handful of planes at most, and a MultiPlane-adjacent RevCC segment is
    already a narrow, uncommon case."""

    candidates = multiplanes.Surf.Planes

    pieces = shell_or_face.Faces if type(shell_or_face) is ShellFaceGu else [shell_or_face]

    found = []
    seen_planes = set()
    for piece in pieces:
        for e in piece.OuterWire.Edges:
            result = other_face_edge(
                e,
                piece,
                GUFaces,
                outer_only=False,
                skip_slivers=True,
                _min_area=tolerances.min_area,
                _min_face_width=tolerances.min_face_width,
            )
            if result is None:
                continue
            _, _, otherface = result
            if not isinstance(otherface.Surface, GPlane):
                continue
            for mpp in candidates:
                if id(mpp) in seen_planes:
                    continue
                if is_same_plane(otherface.Surface, mpp.Surf, tolerances=tolerances):
                    found.append(mpp)
                    seen_planes.add(id(mpp))
                    break
    return found


def _valid_chain_junction(shared_edge, faceA, faceB, tol=1e-6):
    """True if `shared_edge` (already known to be the boundary between
    faceA and faceB) represents a genuine near-parallel RevCC chain
    junction rather than a spurious/incidental edge-sharing between two
    unrelated cylinder/cone faces.

    This replaces an earlier attempt at this same test based on the
    curve shape of `shared_edge` (straight vs curved) -- confirmed wrong
    empirically: for two near-parallel (not coaxial) cylinder/cone faces,
    the real tangency/intersection curve is *never* straight (a genuine
    space curve -- an exact ellipse when the two axes happen to be
    coplanar, a GBSpline in the general skew-axis case), so requiring
    "straight" rejected every legitimate chain junction it was tested
    against (confirmed live on Solidos/Reversed_Cyl_Cones/cyl_cone.stp,
    whose own real chain regressed to fragments once this shape-based
    test was added). Several other shape/position-based alternatives were
    also tried and confirmed NOT to discriminate the real bad case
    (Big_model_reserved/hylife-v06.stp solid358, a R=250 cylinder wrongly
    chained to a R=5170 one with a *perpendicular* axis) from cyl_cone.stp's
    real one: exact convergence of the two faces' own "other" edges to a
    common vertex (fails on cyl_cone.stp too -- real trim boundaries don't
    coincide even for a genuine chain member), and "do the two faces' own
    other-edges lead to the same neighboring face" (matches on both the
    good and the bad case alike).

    The test that does discriminate them, found empirically by directly
    comparing both cases' real topology: a genuine chain junction's shared
    edge has exactly 2 vertices, and on EACH of the two faces, the edge
    touching the first vertex and the edge touching the second vertex are
    two *distinct* edges (an ordinary quad-like face boundary). The real
    hylife-v06 false match instead has, on the R=5170 face specifically,
    only 2 edges total (a degenerate "bigon": the shared edge plus a
    single other edge that touches *both* endpoints of the shared edge) --
    confirmed by direct inspection, and absent on cyl_cone.stp's own real
    chain (every face there has 2 genuinely distinct "other" edges)."""
    verts = shared_edge.Vertexes
    if len(verts) != 2:
        return True
    V0, V1 = verts[0], verts[-1]

    for face in (faceA, faceB):
        e_at_V0 = [e for e in face.Edges if not e.is_same(shared_edge) and any((v - V0).length < tol for v in e.Vertexes)]
        e_at_V1 = [e for e in face.Edges if not e.is_same(shared_edge) and any((v - V1).length < tol for v in e.Vertexes)]
        if len(e_at_V0) == 1 and len(e_at_V1) == 1 and e_at_V0[0].is_same(e_at_V1[0]):
            return False
    return True


def get_join_cone_cyl(face_or_shell, GUFaces, multiplanes, omitFaces, tolerances, root=True):

    face_index = list(face_or_shell.Indexes) if type(face_or_shell) is ShellFaceGu else [face_or_shell.Index]
    omitFaces.update(face_index)
    joined_faces = []
    arc_angle = 0.0
    if type(face_or_shell) is ShellFaceGu:
        Umin, Umax, ifacemin, ifacemax = face_or_shell.U_parameter_range
        if twoPimod(Umax - Umin) == 0:
            return ([], False) if root else ([], 0.0)
        emin = extreme_edge(Umin, face_or_shell.Faces[ifacemin])
        emax = extreme_edge(Umax, face_or_shell.Faces[ifacemax])
        facemin = face_or_shell.Faces[ifacemin]
        facemax = face_or_shell.Faces[ifacemax]
    else:
        Umin, Umax, _, _ = face_or_shell.ParameterRange
        if twoPimod(Umax - Umin) == 0:
            return ([], False) if root else ([], 0.0)
        facemin = face_or_shell
        facemax = face_or_shell
        emin = extreme_edge(Umin, face_or_shell)
        emax = extreme_edge(Umax, face_or_shell)
    arc_angle = Umax - Umin if Umax > Umin else Umax - Umin + twoPi
    if arc_angle > math.pi:
        arc_angle = twoPi - arc_angle

    # skip_slivers=True: a residual near-zero-area sliver face bridging the
    # cylinder/cone's own Umin/Umax boundary to its real neighboring plane
    # (confirmed live, Solidos/Big_one_cell/modelcell_cut1.stp piece 66) must
    # not be treated as the real adjacent plane itself -- same class of fix
    # already applied to multiplane()/eligible_plane().
    result1 = other_face_edge(emin, facemin, GUFaces, skip_slivers=True)
    result2 = other_face_edge(emax, facemax, GUFaces, skip_slivers=True)
    adjacent1 = result1[2] if result1 is not None else None
    adjacent2 = result2[2] if result2 is not None else None

    new_adjacent1 = []
    new_adjacent2 = []
    arc1 = arc2 = 0

    if adjacent1 is not None:
        if isinstance(adjacent1.Surface, (GCone, GCylinder)):
            if (
                adjacent1.Index not in omitFaces
                and adjacent1.Orientation == "Reversed"
                # near-parallel means near-parallel: the two axes must not
                # be too close to perpendicular. The topological
                # _valid_chain_junction test alone isn't sufficient --
                # confirmed live on Solidos/lost_particles/
                # modelcell_cut1_piece51_lost_particles.stp, where two
                # genuinely perpendicular-axis cylinders passed the
                # topological test but still aren't a real RevCC chain
                # member. 0.1 is a permissive floor (rejects only the
                # ~last 6deg approaching exactly perpendicular), not a
                # tight "must be small angle" bound.
                and abs(face_or_shell.Surface.Axis.dot(adjacent1.Surface.Axis)) > 0.1
                and _valid_chain_junction(result1[0], result1[1], adjacent1)
            ):
                adjacent1_shell = merge_same_surface_faces(adjacent1, GUFaces)
                new_adjacent1, arc1 = get_join_cone_cyl(adjacent1_shell, GUFaces, multiplanes, omitFaces, tolerances, False)

    if adjacent2 is not None:
        if isinstance(adjacent2.Surface, (GCone, GCylinder)):
            if (
                adjacent2.Index not in omitFaces
                and adjacent2.Orientation == "Reversed"
                and abs(face_or_shell.Surface.Axis.dot(adjacent2.Surface.Axis)) > 0.1
                and _valid_chain_junction(result2[0], result2[1], adjacent2)
            ):
                adjacent2_shell = merge_same_surface_faces(adjacent2, GUFaces)
                new_adjacent2, arc2 = get_join_cone_cyl(adjacent2_shell, GUFaces, multiplanes, omitFaces, tolerances, False)

    mp_list = []
    for mp in multiplanes:
        mp_planes = _find_adjacent_multiplane_planes(face_or_shell, GUFaces, mp, tolerances)
        if mp_planes:
            mp_list.append(mp_planes)

    if type(face_or_shell.Surface) is GCylinder:
        cylOnly = gen_cylinder(face_or_shell)
        cylcone_plane = gen_plane_cylinder(face_or_shell)

        facein = reversedCCP("Cylinder", (cylOnly, cylcone_plane, mp_list))
        facein.Surf_index.update(face_index)
        facein.Index = face_index[0]

    else:
        coneOnly = gen_cone(face_or_shell)
        apexPlane = cone_apex_plane(face_or_shell, Tolerances())
        cylcone_plane = gen_plane_cone(face_or_shell)

        facein = reversedCCP("Cone", (coneOnly, apexPlane, cylcone_plane, mp_list))
        facein.Surf_index.update(face_index)
        facein.Index = face_index[0]

    joined_faces.extend(new_adjacent1)
    joined_faces.extend(new_adjacent2)
    joined_faces.append(facein)
    arc_angle += arc1 + arc2
    if not root:
        return joined_faces, arc_angle
    else:
        closed_set = twoPimod(arc_angle) == 0.0
        return joined_faces, closed_set


# Tolerance in this function are not the general once
# function should be reviewed
def gen_plane_cylinder(face_or_shell):

    if type(face_or_shell) is ShellFaceGu:
        Umin, Umax, ifacemin, ifacemax = face_or_shell.U_parameter_range
        Faces = face_or_shell.Faces
    else:
        Umin, Umax, _, _ = face_or_shell.ParameterRange
        ifacemin = 0
        ifacemax = 0
        Faces = [face_or_shell]

    UVNode_min, UVNode_max = get_shell_UV_nodes(face_or_shell)

    Uminr = twoPimod(Umin)
    Umaxr = twoPimod(Umax)
    # min()-based search, not a hand-rolled "if d < best" loop: UVNode_min/
    # UVNode_max are now guaranteed non-empty (see the fallback above), but
    # a strict-less-than loop starting from a fixed twoPi bound can still
    # leave indmin/indmax undefined if no candidate distance ever comes out
    # below that bound -- min() always returns *some* index (the true
    # closest one), with the same first-occurrence-wins tie-break a strict
    # "<" loop has, so this is behavior-preserving for every case that
    # already worked and merely removes the possibility of a crash.
    indmin = min(range(len(UVNode_min)), key=lambda i: abs(twoPimod(UVNode_min[i][0]) - Uminr))
    indmax = min(range(len(UVNode_max)), key=lambda i: abs(twoPimod(UVNode_max[i][0]) - Umaxr))

    V1 = Faces[ifacemin].value_at(UVNode_min[indmin][0], UVNode_min[indmin][1])
    V2 = Faces[ifacemax].value_at(UVNode_max[indmax][0], UVNode_max[indmax][1])

    axis = Faces[ifacemin].Surface.Axis
    cross = (V2 - V1).cross(axis)
    vmid = (V1 + V2) * 0.5
    if cross.length < 1e-9:
        # V1 == V2 (or V2-V1 happens to lie exactly along axis) -- the
        # closest-UV-node search picked the same point for both ends, a
        # real degenerate case confirmed live (2026-08-23,
        # Solidos/test_models/Big_complex_cell/modelcell_cut1.stp) rather
        # than guessed. No well-defined bounding-plane normal exists here;
        # falling back to an arbitrary perpendicular to axis is safer than
        # crashing (this function's own header comment already flags it
        # as a simplified approximation, not exact geometry).
        normal = _perpendicular_axis(axis)
    else:
        normal = cross.normalized()

    plane = GeounedSurface(("Plane", (vmid, normal, 1, 1)))

    return plane


# Tolerance in this function are not the general once
# function should be reviewed
def gen_plane_cone(face_or_shell):

    if type(face_or_shell) is ShellFaceGu:
        Umin, Umax, ifacemin, ifacemax = face_or_shell.U_parameter_range
        Faces = face_or_shell.Faces
    else:
        Umin, Umax, _, _ = face_or_shell.ParameterRange
        ifacemin = 0
        ifacemax = 0
        Faces = [face_or_shell]

    UVNode_min, UVNode_max = get_shell_UV_nodes(face_or_shell)

    # min()-based search, not a hand-rolled "if d < best" loop -- see the
    # identical comment in gen_plane_cylinder just above this function for
    # why (UVNode_min/UVNode_max are guaranteed non-empty, but a
    # strict-less-than loop from a fixed twoPi bound can still leave
    # indmin/indmax undefined; min() always returns the true closest index,
    # same tie-break, so this is crash-safe).
    #
    # Umin/Umax ARE wrapped via twoPimod before comparison (confirmed a
    # real, previously-dormant bug via live reproduction, 2026-08-16,
    # placa2.stp under the occ engine: a face whose native U range spans
    # past 2*pi -- e.g. ~7.12 to ~11.72 rad -- combined with comparing that
    # unwrapped Umin/Umax against an always-wrapped `nd` made both the
    # indmin and indmax searches independently converge on the *same*
    # index, giving V1 == V2 exactly and a degenerate (zero-length) cross
    # product a few lines below. gen_plane_cylinder already wraps both
    # sides of this exact comparison (Uminr/Umaxr) -- this was a plain
    # omission here, not an intentional difference; verified fixed against
    # the real reproduction, not just pattern-matched from the sibling.
    Uminr = twoPimod(Umin)
    Umaxr = twoPimod(Umax)
    indmin = min(range(len(UVNode_min)), key=lambda i: abs(twoPimod(UVNode_min[i][0]) - Uminr))
    indmax = min(range(len(UVNode_max)), key=lambda i: abs(twoPimod(UVNode_max[i][0]) - Umaxr))

    V1 = Faces[ifacemin].value_at(UVNode_min[indmin][0], UVNode_min[indmin][1])
    V2 = Faces[ifacemax].value_at(UVNode_max[indmax][0], UVNode_max[indmax][1])

    apex = Faces[ifacemin].Surface.Apex

    # A cone face's own V=0 boundary is the apex itself -- for a face
    # whose real ParameterRange reaches all the way to V=0 (confirmed live,
    # Solidos/test_models/Mixed/multiplane_add_plane_cone.stp: Vmin=0.0),
    # the tessellation-based UV-node search above picks the node closest
    # in U alone, with no regard for V, and can land exactly on the apex
    # node -- (V1 - apex) then has zero length, a real ZeroDivisionError,
    # not a tangency/numerical-noise case. A cone's own generatrix (fixed
    # U, increasing V) is a straight line through the apex, so any V > 0
    # at the *same* U gives a direction identical (up to normalization) to
    # whatever a non-degenerate node at that U would have given -- fixed
    # by re-evaluating at a small positive V nudge instead of the
    # apex-coincident one, rather than trusting the tessellated V as-is.
    if (V1 - apex).length < 1e-7:
        _, _, _, vmax1 = Faces[ifacemin].ParameterRange
        V1 = Faces[ifacemin].value_at(UVNode_min[indmin][0], 1e-3 * vmax1)
    if (V2 - apex).length < 1e-7:
        _, _, _, vmax2 = Faces[ifacemax].ParameterRange
        V2 = Faces[ifacemax].value_at(UVNode_max[indmax][0], 1e-3 * vmax2)

    dir1 = (V1 - apex).normalized()
    dir2 = (V2 - apex).normalized()
    normal = dir2.cross(dir1).normalized()

    plane = GeounedSurface(("Plane", (apex, normal, 1, 1)))

    return plane


def extreme_edge(U, face):
    du = twoPi
    Umod = twoPimod(U)
    for e in face.OuterWire.Edges:
        pnt = 0.5 * (e.Vertexes[0] + e.Vertexes[-1])
        u, v = face.parameter(pnt)
        u = twoPimod(u)  # u itself can exceed 2*pi (e.g. a face's own
        # ParameterRange spanning past a full turn) -- reduce it first, or
        # the "d, twoPi - d" wraparound correction below can go negative
        # and win the "closest edge" comparison outright regardless of the
        # real angular distance (confirmed live on Reversed_Cyl_Cones/
        # cyl_cone.stp: this silently picked a real but unrelated plane
        # instead of the true adjacent cylinder, breaking the RevCC chain).
        d = abs(Umod - u)
        d = min(d, twoPi - d)  # wraparound-aware: Umod==0 must also match u near twoPi
        if d < du:
            du = d
            edge = e
    return edge


#   Check if to faces are joint
def contiguous_face(face1, face2, tolerances):
    """True if face1 and face2 touch, tested via their boundary edges
    rather than a full face-to-face distance/boolean query.

    Both of same_faces()'s own callers (this function's only caller)
    already restrict `Faces` to fragments of one single analytic surface
    (is_same_surface-filtered) before ever reaching here -- non-overlapping
    fragments of the same surface can only touch along their shared
    boundary, never via interior tangency, so an edge-vs-edge test is a
    sound characterization of "contiguous" for this specific use, even
    though it would NOT be a valid general "do these two faces touch
    anywhere" test.

    Edge-to-edge distance (GEdge.my_distToshape, a BoundBox-prefiltered
    curve-curve extrema query) replaces the previous whole-face
    distToShape/boolean-common call -- found, via py-spy profiling on
    hylife-v06.stp, to be the dominant cost of same_faces()'s O(n^2)
    face-pair walk on real, large split-face groups (BRepAlgoAPI_Common
    between two general trimmed surfaces is far more expensive than a
    curve-curve query between their -- typically few -- boundary edges)."""
    for e1 in face1.Edges:
        for e2 in face2.Edges:
            if e1.my_distToshape(e2) < tolerances.distance:
                return True
    return False


def same_faces(Faces, tolerances):
    """Every face index connected, directly or through a chain of other
    contiguous faces, to Faces[0] -- 0 itself is never included in the
    result (the caller, merge_same_surface_faces, re-inserts it).

    Connection[i] only ever lists forward edges (i, j) with j > i (the
    O(n^2) pairwise walk below never re-tests a pair the other way
    round), so this is an undirected graph stored as an upper-triangular
    adjacency list -- finding everything reachable from node 0 needs a
    real graph traversal over both edge directions, not a single linear
    pass over the keys in insertion order (a face whose only recorded
    edge points to a *later* key that hasn't been shown to connect to 0
    yet, at the time that key is visited, was silently dropped by an
    earlier single-pass version of this function -- confirmed live,
    Connection={0:[3], 1:[2], 2:[3]}: face 1 only connects via face 2,
    which itself only reaches 0 via face 3, so a pass that visits key 1
    before key 2 has been resolved misses it entirely)."""
    Connection = OrderedDict()
    for i, face1 in enumerate(Faces):
        Couples = []
        for j, face2 in enumerate(Faces[i + 1 :], start=i + 1):
            if contiguous_face(face1, face2, tolerances):
                Couples.append(j)
        Connection[i] = Couples

    adjacency = {i: set(neighbors) for i, neighbors in Connection.items()}
    for i, neighbors in Connection.items():
        for j in neighbors:
            adjacency.setdefault(j, set()).add(i)

    visited = {0}
    queue = [0]
    while queue:
        node = queue.pop()
        for neighbor in adjacency.get(node, ()):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)

    visited.discard(0)
    return list(visited)


def closed_circle_edge(planes):
    angle = 0
    for p in planes:
        umin, umax = p.edge.ParameterRange
        angle += umax - umin
    return abs(angle - 2 * math.pi) < 1e-5


def most_outer_faces(cyl, faces):

    if type(cyl) is ShellFaceGu:
        cylSurf = cyl.Faces[0].Surface
    else:
        cylSurf = cyl.Surface

    surfPos = []
    for i, f in enumerate(faces):
        d = cylSurf.Axis.dot(f.CenterOfMass)
        surfPos.append((d, i))
    surfPos.sort()
    face1 = faces[surfPos[0][1]]
    face2 = faces[surfPos[-1][1]]
    remove_surf = set()

    for f in faces:
        if not is_same_surface(f.Surface, face1.Surface) and not is_same_surface(f.Surface, face2.Surface):
            remove_surf.add(f.Index)

    return (face1, face2), remove_surf


def eligible_plane(plane, tolerances=None):
    """An eligible master plane is a plane where the adjacent concave planes make a convex shape"""
    # `tolerances=None` (rather than a `Tolerances()` default argument)
    # deliberately keeps every existing call site working unchanged, but
    # note that a bare default previously meant this ALWAYS silently used
    # a fresh Tolerances() instance -- ignoring whatever min_area the
    # caller had actually configured via CadToCsg(tolerances=...). Fixed,
    # 2026-08-23: callers now thread their own real tolerances object
    # through; only truly tolerances-agnostic call sites (if any remain)
    # fall back to the class default here.
    if tolerances is None:
        tolerances = Tolerances()
    if plane.Area < tolerances.min_area:
        # A residual near-zero-area sliver face (left over from a boolean
        # cut that grazed tangentially, same class of artifact
        # other_face_edge's skip_slivers mode already treats as
        # transparent elsewhere) is never a real plane of the solid --
        # confirmed live on Solidos/Big_one_cell/modelcell_cut1.stp's
        # piece 36, whose real 5-plane boundary was being read as 6
        # planes because a 0.0021-area sliver kept qualifying as its own
        # multiplane() master, producing a spurious MultiPlane grouping.
        return False
    if getattr(plane, "CharacteristicWidth", float("inf")) < tolerances.min_face_width:
        # min_area alone doesn't catch a real, large-area but genuinely
        # thin sliver (see Tolerances.min_face_width's own docstring) --
        # same reasoning as order_plane_face's identical check.
        return False

    Edges = plane.OuterWire.Edges

    # A residual sliver edge on the boundary -- confirmed live
    # (Solidos/working_solids/null.stp): a tiny circular-arc fragment
    # left by a boolean cut that grazed the plane's real straight
    # boundary. The walk below treats every edge as a straight segment
    # between its own two endpoints regardless of curve type (there is
    # no GLine-only requirement, deliberately -- see the walk itself);
    # a genuine sliver arc's own endpoints sit close to, but not exactly
    # on, the real corner the two flanking real edges actually meet at,
    # introducing a spurious extra "kink" that the convexity test below
    # then reads as a real non-convex turn -- wrongly rejecting an
    # otherwise-convex, otherwise-eligible plane.
    #
    # Filtered by length alone (not curve type -- a genuinely short
    # straight edge is the same class of artifact). The floor is
    # `tolerances.min_face_width` (already established, a few lines up,
    # as this codebase's own "is this a sliver" width convention) --
    # confirmed live on Solidos/working_solids/null.stp that a real
    # sliver-arc pair (0.071mm) sits well below min_face_width's 0.1mm
    # default but roughly 4-6x ABOVE `diag * sliver_edge_rel_tol`
    # (~0.012-0.018mm for this file's ~120-160mm-diagonal faces), so the
    # relative term alone never catches it; the smallest genuine
    # (non-sliver) boundary edge found on this same file is 2.8mm, ~28x
    # above min_face_width, so the wider floor still leaves a comfortable
    # margin against filtering real geometry. MIN_SLIVER_EDGE_LENGTH
    # (geo.constants -- geo.solid_defects.find_short_edges' own absolute
    # floor) is the last-resort fallback, only relevant if min_face_width
    # were ever configured below it. Once a sliver is dropped, the two
    # flanking real edges no longer share an exact vertex -- widen the
    # vertex-matching tolerance in the ordered walk to bridge that gap
    # (bounded by the removed sliver's own chord, itself <= its own
    # length, so this can never wrongly merge two genuinely distinct
    # corners). If filtering would leave too few edges to form a polygon
    # at all (a genuinely tiny plane, e.g. a real triangle), fall back
    # to the unfiltered edge list and the original tight tolerance
    # rather than risk making things worse.
    diag = plane.BoundBox.DiagonalLength
    sliver_floor = max(tolerances.min_face_width, MIN_SLIVER_EDGE_LENGTH)
    sliver_length = max(diag * tolerances.sliver_edge_rel_tol, sliver_floor)
    real_edges = [e for e in Edges if e.Length >= sliver_length]
    if len(real_edges) >= 3:
        Edges = real_edges
        vertex_tol = max(1e-6, sliver_length)
    else:
        vertex_tol = 1e-6

    Vertexes = []
    for e in Edges:
        if len(e.Vertexes) < 2:
            # a degenerate (zero-length) edge -- not a real, usable boundary segment
            return False
        Vertexes.append((e.Vertexes[0], e.Vertexes[1]))

    if len(Vertexes) == 0:
        return False

    ei = Vertexes[0][1]
    Ordered = [Vertexes[0]]
    del Vertexes[0]

    while len(Vertexes) > 0:
        for i, e12 in enumerate(Vertexes):
            e1, e2 = e12
            found = False
            if (e1 - ei).length < vertex_tol:
                ei = e2
                Ordered.append((e1, e2))
                del Vertexes[i]
                found = True
                break
            elif (e2 - ei).length < vertex_tol:
                ei = e1
                Ordered.append((e2, e1))
                del Vertexes[i]
                found = True
                break
        if not found:
            break

    # Merge consecutive edges of Ordered that share the exact same
    # direction -- a straight boundary the CAD kernel split into several
    # collinear segments (not a sliver, but the same class of spurious-
    # kink risk: two genuinely collinear segments have a cross product
    # of exactly zero, and floating-point noise on that near-zero value
    # can wobble it slightly negative, wrongly flipping the convexity
    # test below). A run of 3+ collinear edges collapses the same way,
    # one pass merging left to right.
    if len(Ordered) >= 2:
        merged = [Ordered[0]]
        for e1, e2 in Ordered[1:]:
            prev_start, prev_end = merged[-1]
            prev_dir = (prev_end - prev_start).normalized()
            cur_dir = (e2 - e1).normalized()
            if abs(prev_dir.dot(cur_dir) - 1.0) < 1e-6:
                merged[-1] = (prev_start, e2)
            else:
                merged.append((e1, e2))
        Ordered = merged

    v0 = -(Ordered[-1][1] - Ordered[-1][0])
    v1 = Ordered[0][1] - Ordered[0][0]

    convex = True
    axis = v0.cross(v1)
    for e1, e2 in Ordered[1:]:
        v0 = -v1
        v1 = e2 - e1
        if axis.dot(v0.cross(v1)) < 0:
            convex = False
            break
    return convex


def no_convex(mplane_list):
    """keep part of no complex plane set"""
    planes = mplane_list[:]
    while len(planes) > 1:
        p = planes.pop()
        Edges = p.OuterWire.Edges
        for e in Edges:
            if type(Gclassify_curve(e)) is not GLine:
                continue
            adjacent_plane = other_face_edge(e, p, planes, outer_only=True)
            if adjacent_plane is not None:
                sign = region_sign(p, adjacent_plane)
                if sign == "AND":
                    return False
    return True


def commonVertex(e1, e2):
    """Returns the GVector point(s) (not native Vertex objects -- nothing
    downstream needs vertex identity, only the coordinate) shared by e1
    and e2."""
    if not shapes_in_contact(e1.__native__, e2.__native__):
        return []

    common = []
    if e1.Vertexes[0] == e2.Vertexes[0]:
        common.append(e1.Vertexes[0])
    elif e1.Vertexes[0] == e2.Vertexes[1]:
        common.append(e1.Vertexes[0])

    if e1.Vertexes[1] == e2.Vertexes[0]:
        common.append(e1.Vertexes[1])
    elif e1.Vertexes[1] == e2.Vertexes[1]:
        common.append(e1.Vertexes[1])

    return common


def commonEdge(face1, face2, outer1_only=True, outer2_only=True):
    if type(face1) is ShellFaceGu:
        for face in face1.Faces:
            edges = commonEdgeFace(face, face2, outer1_only, outer2_only)
            if edges is not None:
                if len(edges) > 0:
                    return edges, face
        return None, None
    else:
        return commonEdgeFace(face1, face2, outer1_only, outer2_only)


def commonEdgeFace(face1, face2, outer1_only=True, outer2_only=True):
    if face1.distToShape(face2)[0] > 0:
        return None

    edges = []
    Edges1 = face1.OuterWire.Edges if outer1_only else face1.Edges
    Edges2 = face2.OuterWire.Edges if outer2_only else face2.Edges
    for e1 in Edges1:
        for e2 in Edges2:
            if e1.is_same(e2):
                edges.append(e1)
    return edges


def _and_or_by_material_sampling(
    center, axis, radius, along, pos_this, axis_this, pos_other, axis_other, solid, n_samples=600, margin_factor=2.0
):
    """Robust fallback for `cyl_plane_region_conf`'s AND_p1_cyl/AND_p2_cyl
    sign test, used only when the cylinder and the corner plane are so
    close to exactly tangent that the analytic cross-product sign is pure
    numerical noise (confirmed live on
    Solidos/Big_one_cell/modelCell_670000.stp piece 59, where 3
    structurally-identical R=6 round corners have this exact degeneracy on
    2 of their 3 corner planes).

    Samples random points in a local square around the cylinder's own
    axis, keeping only those that are (a) outside the cylinder radius and
    (b) on the material side of the *other* corner plane (via its own
    already-reliable resolved normal `axis_other`/`pos_other` -- this
    doesn't depend on any cross-product sign, so it stays trustworthy even
    when the plane being tested is degenerate). Among those, compares real
    solid membership (`solid.is_inside`) conditioned on whether each point
    is also on the material side of the plane being tested
    (`axis_this`/`pos_this`):
      - AND signature: material only appears when `axis_this`'s condition
        also holds (material fraction collapses when it doesn't).
      - OR signature: material appears comparably either way (the
        cylinder/other-plane already account for it on their own).

    Returns True (AND), False (OR), or None if too few samples landed on
    one side to decide reliably -- the caller falls back to the analytic
    sign test in that case, so this can only ever improve or match
    today's behavior, never make it worse.

    `center`/`axis`/`radius`/`along` describe the cylinder's own real
    geometry (`along` is the offset from `center` *along `axis`*, at
    which the two corner planes actually meet the cylinder -- the plane
    perpendicular to `axis` to sample in, not a fixed world-Z height).
    An earlier version hardcoded sampling in the world X/Y plane at a
    fixed world-Z, silently assuming every round-corner cylinder is
    Z-axis-aligned -- confirmed live, 2026-08-23,
    Solidos/test_models/Mixed/SCDR_90_hollow.stp's own decomposed piece
    4: a real R=37mm round-corner cylinder with axis (0,1,0) (Y-aligned)
    hit this exact fallback (cross1.length=6.8e-07, well inside the
    degenerate-tangency range) and returned a wrong answer (False) where
    the already-correct naive sign test said True, because "outside the
    cylinder radius" was being decided from X/Y distance alone -- which
    has no relationship to the real cylinder surface when the axis isn't
    Z. Fixed to sample in the plane actually perpendicular to `axis`.
    """
    import random

    u = _perpendicular_axis(axis)
    v = axis.cross(u).normalized()
    axis_n = axis.normalized()
    margin = margin_factor * radius
    rng = random.Random(0)
    true_total = true_material = 0
    false_total = false_material = 0
    for _ in range(n_samples):
        du = rng.uniform(-margin, margin)
        dv = rng.uniform(-margin, margin)
        if (du * du + dv * dv) ** 0.5 <= radius:
            continue
        pt = center + axis_n * along + u * du + v * dv
        if axis_other.dot(pt - pos_other) <= 0:
            continue
        real = solid.is_inside(pt)
        if axis_this.dot(pt - pos_this) > 0:
            true_total += 1
            true_material += real
        else:
            false_total += 1
            false_material += real

    if true_total < 15 or false_total < 15:
        return None

    frac_true = true_material / true_total
    frac_false = false_material / false_total
    return (frac_true - frac_false) > 0.15


def cyl_plane_region_conf(cylinder, ep1, ep2, solid=None):

    # ep1[0]/ep2[0] (cyl1/cyl2) are each end's own real adjacent cylinder
    # piece -- normally the same object as `cylinder` (the seed), but when
    # `cylinder` is one contiguous piece of a same-surface cylinder split
    # into several (see merge_same_surface_faces -- get_adjacent_cylplane's
    # ShellGu branch searches every piece and tags each found plane with
    # the specific piece that actually touches it), p1 and p2 can each
    # border a *different* piece. r1/nc1/nt1 must come from p1's own piece
    # and r2/nc2 from p2's own piece -- using the single seed `cylinder`
    # for both (the original bug) evaluates the "far" end at that seed's
    # own trim boundary instead of the real adjacent piece's, which can
    # land far from the true corner geometry (confirmed concretely: for
    # L1_S23.stp solid 174, whose round-corner cylinder is split into 2
    # 90-degree pieces meeting at a shared seam, p2's real adjacent piece
    # is tangent to p2's own plane at its own far boundary, while the
    # seed piece's own far boundary -- what the old code evaluated r2/nc2
    # at -- is just the seam between the two pieces, unrelated to p2).
    cyl1, _, _, p1 = ep1
    cyl2, _, _, p2 = ep2

    if type(cylinder) is ShellFaceGu:
        umin, umax = cylinder.U_parameter_range[:2]
        vmin, vmax = cylinder.Faces[0].ParameterRange[2:]
        cyl_face = cylinder.Faces[0]
    else:
        umin, umax, vmin, vmax = cylinder.ParameterRange
        cyl_face = cylinder

    v_value = 0.5 * (vmin + vmax)
    cyl_tg1, z1 = cyl_face.tangent_at(umin, v_value)
    cyl_tg2, z2 = cyl_face.tangent_at(umax, v_value)
    cyl_r1 = cyl_face.value_at(umin, v_value)
    cyl_r2 = cyl_face.value_at(umax, v_value)

    ed11 = abs(p1.Surface.Axis.dot(p1.Surface.Position - cyl_r1))
    ed12 = abs(p1.Surface.Axis.dot(p1.Surface.Position - cyl_r2))
    if ed12 < ed11:  # plane 1/2 are not asociated to edge 1/2. -> switch planess
        p1, p2 = p2, p1
        switched = True
    else:
        switched = False

    p1_axis = p1.Surface.Axis if p1.Orientation == "Reversed" else -p1.Surface.Axis
    p2_axis = p2.Surface.Axis if p2.Orientation == "Reversed" else -p2.Surface.Axis

    cyl_normal1 = cyl_tg1.cross(z1).normalized()
    cyl_normal2 = cyl_tg2.cross(z2).normalized()

    cross1 = cyl_normal1.cross(p1_axis)
    cross2 = cyl_normal2.cross(p2_axis)

    fwd_cyl = cylinder.Orientation == "Forward"
    if cross1.length < 1e-3:
        upmin, upmax, vpmin, vpmax = p1.ParameterRange
        up = 0.5 * (upmin + upmax)
        vp = 0.5 * (vpmin + vpmax)
        inp1 = p1.value_at(up, vp)
        ref1 = (inp1 - cyl_r1).normalized()
        along1 = p1_axis.cross(z1)
        if ref1.dot(along1) < 0:
            along1 = -along1
        base = cyl_tg1.dot(along1) < 0
        base = base if fwd_cyl else not base
    else:
        base = z1.dot(cross1) > 0
    AND_p1_cyl = base

    if cross2.length < 1e-3:
        upmin, upmax, vpmin, vpmax = p2.ParameterRange
        up = 0.5 * (upmin + upmax)
        vp = 0.5 * (vpmin + vpmax)
        inp2 = p2.value_at(up, vp)
        ref2 = (inp2 - cyl_r2).normalized()
        along2 = p2_axis.cross(z2)
        if ref2.dot(along2) < 0:
            along2 = -along2
        base = cyl_tg2.dot(along2) > 0
        base = base if fwd_cyl else not base
    else:
        base = z1.dot(cross2) < 0
    AND_p2_cyl = base

    v1 = z1.cross(cyl_r1 - cyl_r2).normalized()  # v1 fixed vector oriented toward cylinder arc, z1 ref rotacion axis

    normal_cyl_plane = -v1 if fwd_cyl else v1

    def signed_angle(v3, npd, n_plane):
        # Angle from npd to n_plane, measured around v3, standard
        # atan2(sin, cos) form -- 0 exactly when n_plane == npd.
        s = v3.dot(npd.cross(n_plane))
        c = npd.dot(n_plane)
        return math.atan2(s, c)

    a1_max = signed_angle(z1, -v1, cyl_normal1)
    a2_max = signed_angle(-z1, -v1, cyl_normal2)
    a1 = signed_angle(z1, normal_cyl_plane, p1_axis)
    a2 = signed_angle(-z1, normal_cyl_plane, p2_axis)

    if a1 > a1_max or a2 > a2_max:
        # planes cannot go beyong pd plane
        return None, switched

    # allow crossing planes
    # if a1+a2 < -math.pi:
    #    # planes cannot cross
    #    return None, switched

    base = a1 < 0
    AND_p1_pd = base if fwd_cyl else not base
    base = a2 < 0
    AND_p2_pd = base if fwd_cyl else not base

    OR_p12_bracket = z1.dot(p1_axis.cross(p2_axis)) < 0  # si no funciona asi es que es el valor negativo
    same_p1_pd = (p1_axis.dot(normal_cyl_plane)) > 0.999999
    same_p2_pd = (p2_axis.dot(normal_cyl_plane)) > 0.999999

    configuration = fwd_cyl * mask.fwd_cyl
    configuration += AND_p1_cyl * mask.p1_cyl
    configuration += AND_p2_cyl * mask.p2_cyl
    configuration += AND_p1_pd * mask.p1_pd
    configuration += AND_p2_pd * mask.p2_pd
    configuration += OR_p12_bracket * mask.p1_p2
    configuration += same_p1_pd * mask.same_p1_pd
    configuration += same_p2_pd * mask.same_p2_pd

    return configuration, switched


def material_direction(pos: GVector, face: GFace | FaceGu, edge: GEdge):

    pe = edge.parameter(pos)
    dir = edge.derivative1_at(pe).normalized()
    if edge.Orientation == "Reversed":
        dir = -dir
    u, v = face.Surface.parameter(pos)
    normalf = face.normal_at(u, v).normalized()
    matvec = normalf.cross(dir)

    return matvec, normalf


def region_sign(s1_in, s2, outAngle=False):
    if type(s1_in) is ShellFaceGu:
        Edges, s1 = commonEdge(s1_in, s2, outer1_only=False, outer2_only=False)
    else:
        Edges = commonEdge(s1_in, s2, outer1_only=False, outer2_only=False)
        s1 = s1_in

    if not Edges:
        # s1/s2 were only speculatively adjacent (a caller walking every
        # face pair, not one that already confirmed a shared edge) --
        # my_distToshape's BoundBox-fallback branch can also report a
        # real, touching pair (native distToShape == 0) as far apart when
        # one shape is flat/degenerate along an axis the other doesn't
        # span, making commonEdge legitimately find nothing here. No
        # sign to report either way.
        return (None, None) if outAngle else None

    e1 = Edges[0]
    p0, p1 = e1.ParameterRange
    pe = 0.5 * (p1 + p0)
    # e1.value_at (curve-agnostic) instead of e1.Curve.value: e1.Curve can
    # be None for an edge whose curve type Gclassify_curve doesn't model
    # (e.g. Hyperbola/Parabola) -- material_direction below only needs the
    # point, not the classified curve.
    pos = e1.value_at(pe)

    vect, normal1 = material_direction(pos, s1, e1)

    u, v = s2.parameter(pos)
    normal2 = s2.normal_at(u, v)

    if type(Gclassify_curve(e1)) is GLine and not isinstance(s2.Surface, GPlane):
        umin, umax, vmin, vmax = s2.ParameterRange
        arc = abs(umax - umin)
    else:
        arc = 0

    dprod = vect.dot(normal2)

    if abs(dprod) < 1e-4:
        if type(s2.Surface) is GSphere:
            operator = "AND" if s2.Orientation == "Forward" else "OR"
        elif type(s1.Surface) is GSphere:
            operator = "AND" if s1.Orientation == "Forward" else "OR"
        else:
            if type(s2.Surface) is GCylinder:
                fwd = s2.Orientation == "Forward"
            elif type(s1.Surface) is GCylinder:
                fwd = s1.Orientation == "Forward"
            else:
                fwd = True
            dotpos = normal2.dot(normal1) > 0

            if abs(dprod) < arc:
                operator = "AND" if fwd == dotpos else "OR"
            else:
                operator = "OR" if fwd == dotpos else "AND"
        if outAngle:
            vect2, _ = material_direction(pos, s2, e1)
            return operator, angle(vect, -vect2, operator)
        else:
            return operator

    else:
        operator = "OR" if dprod > 0 else "AND"
        if outAngle:
            vect2, _ = material_direction(pos, s2, e1)
            # oposite of vect2 because evaluated with e1 and not the edge corresponding to surface2
            return operator, angle(vect, -vect2, operator)
        else:
            return operator


def angle(v1, v2, operator):
    d = v1.dot(v2) / (v1.length * v2.length)
    a = math.acos(max(-1, min(1, d)))
    if operator == "AND":
        return a
    else:
        return twoPi - a


def merge_same_surface_faces(face_in, solidFaces):
    """A boolean cut that splits a single analytic cylinder/cone into
    several contiguous face pieces (e.g. a residual-cut artifact, or a
    genuine multi-piece split) shouldn't be treated as several unrelated
    features. Groups `cylkne` with every other face in `solidFaces` that
    is both the same underlying surface (is_same_surface) and physically
    connected to it, directly or via a chain of other same-surface pieces
    (same_faces), and returns a ShellGu of the merged group -- or `cylkne`
    itself, unchanged, if no such piece exists. Shared by closed_cylinder_cone
    (Can/TCone closure) and get_roundcorner_surfaces (RoundCorner corner-plane
    search).

    Residual sliver faces (area below Tolerances().min_area -- the same
    degenerate, near-zero-area boolean-cut artifact `other_face_edge`'s
    `skip_slivers` mode treats as transparent noise, not a real feature)
    are excluded from the same-surface group entirely before the O(n^2)
    same_faces adjacency walk: on a solid with many such slivers on one
    analytic surface, comparing them pairwise via native distToShape can
    be catastrophically slow or, on select near-degenerate pairs, hang
    outright in pyOCC's BRepAlgoAPI_Common/BRepExtrema_DistShapeShape
    (confirmed live, hylife-v06.stp solid 17, 2026-08-16 -- FreeCAD's own
    OCCT build handles the identical face pairs quickly). A sliver's
    negligible area means dropping it from the merged group doesn't
    change the group's real geometry (closure angle, corner-plane
    adjacency) in any way that matters."""
    min_area = Tolerances().min_area
    same_surface = [face_in]
    for current_face in solidFaces:
        if current_face.Index == face_in.Index:
            continue
        if current_face.Area < min_area:
            continue
        if is_same_surface(face_in.Surface, current_face.Surface):
            same_surface.append(current_face)

    if len(same_surface) > 1:
        sameIndex = same_faces(
            same_surface, Tolerances()
        )  # return all face connected (direct or indirectly ) to first face (cylinder)
        sameIndex.insert(0, 0)
        connected_faces = [same_surface[i] for i in sameIndex]
        if len(connected_faces) > 1:
            return ShellFaceGu(connected_faces)

    return face_in


def closed_cylinder_cone(cylkne, solidFaces):
    ck_shell = merge_same_surface_faces(cylkne, solidFaces)
    ck_index = ck_shell.Indexes if type(ck_shell) is ShellFaceGu else {cylkne.Index}
    return ck_shell, ck_index, is_closed_cylinder_cone(ck_shell)


def _edge_is_planar(edge):
    """Whether ONE edge, on its own, lies within a plane -- unlike
    planar_edges (which additionally requires several edges to share
    ONE common plane), this makes no claim about any other edge. A
    straight line or a circle/ellipse is always planar by construction;
    a BSpline is planar only if spline_2D confirms it (or its own
    tangent is degenerate, in which case it's effectively a straight
    segment); any other/unsupported curve type (e.g. Hyperbola/
    Parabola) is treated as not confirmed planar."""
    if edge.Length < 1e-5:
        return False
    curve = edge.Curve
    if type(curve) is GBSpline:
        d0 = edge.derivative1_at(0)
        if d0.length < 1e-5:
            return True
        return spline_2D(edge)
    if type(curve) in (GCircle, GEllipse):
        return True
    if curve is None:
        return False
    return True  # GLine


def edges_individually_planar(edges):
    """True if EVERY edge in `edges` is individually planar
    (_edge_is_planar), regardless of whether they all share one common
    plane -- unlike planar_edges. Used where a boundary side can
    legitimately be made of several distinct planar pieces at different
    orientations (e.g. the torus U-side check), not one single flat
    boundary."""
    if len(edges) == 0:
        return False
    return all(_edge_is_planar(e) for e in edges)


def planar_edges(edges):
    if len(edges) == 0:
        return False
    e0 = edges[0]
    if e0.Length < 1e-5:
        return False
    # e0.Curve is already the classified curve (GLine/GCircle/GEllipse/
    # GBSpline/None), set once by GEdge.__init__ -- re-running
    # Gclassify_curve(e0) here would misclassify everything as None
    # (it expects a native edge, and GEdge.Curve is not one).
    curve0 = e0.Curve
    if type(curve0) is GBSpline:
        d0 = e0.derivative1_at(0)
        if d0.length < 1e-5:
            dir0 = (e0.Vertexes[1] - e0.Vertexes[0]).normalized()
            center0 = 0.5 * (e0.Vertexes[1] + e0.Vertexes[0])
        elif spline_2D(e0):
            dir0 = e0.derivative1_at(0).cross(e0.normal_at(0)).normalized()
            center0 = 0.5 * (e0.Vertexes[1] + e0.Vertexes[0])
        else:
            return False
    elif type(curve0) in (GCircle, GEllipse):
        dir0 = curve0.Axis
        center0 = curve0.Center
    elif curve0 is None:  # unsupported curve type (e.g. Hyperbola/Parabola)
        return False
    else:  # should be a line
        dir0 = curve0.Direction
        center0 = curve0.Position

    if len(edges) == 1:
        if edge_1D(edges[0]):
            return False
        else:
            return True

    oneD = edge_1D(edges[0])

    for ei in edges[1:]:
        curve_i = ei.Curve
        if type(curve_i) is GBSpline:
            di = ei.derivative1_at(0)
            if di.length < 1e-5:
                dir = (ei.Vertexes[1] - ei.Vertexes[0]).normalized()
                center = 0.5 * (ei.Vertexes[1] + ei.Vertexes[0])
            elif spline_2D(ei):
                dir = ei.derivative1_at(0).cross(ei.normal_at(0)).normalized()
                center = 0.5 * (ei.Vertexes[1] + ei.Vertexes[0])
            else:
                return False
        elif type(curve_i) in (GCircle, GEllipse):
            dir = curve_i.Axis
            center = curve_i.Center
        elif curve_i is None:  # unsupported curve type (e.g. Hyperbola/Parabola)
            return False
        else:  # should be a line
            dir = curve_i.Direction
            center = curve_i.Position

        if not is_parallel(dir0, dir, Tolerances().angle):
            return False
        if abs(dir0.dot(center - center0)) > 1e-5:
            return False

        if not edge_1D(ei):
            oneD = False

    if oneD:
        return False
    else:
        return True


def same_curve(edges):
    """
    True if every edge in `edges` lies on the SAME single underlying
    curve -- unlike planar_edges, this does not require the curve to be
    planar, since a legitimate boundary between two surfaces can be a
    genuinely non-planar curve (e.g. the intersection of two
    perpendicular cylinders). Used to tell a clean single-surface
    boundary from a jumble of edges left by an irregular/messy cut.
    """
    if len(edges) == 0:
        return False
    e0 = edges[0]
    if e0.Length < 1e-5:
        return False
    curve0 = e0.Curve
    if curve0 is None:  # unsupported curve type (e.g. Hyperbola/Parabola)
        return False
    if len(edges) == 1:
        return True

    if type(curve0) is GLine:
        for ei in edges[1:]:
            curve_i = ei.Curve
            if type(curve_i) is not GLine:
                return False
            if not is_parallel(curve0.Direction, curve_i.Direction, Tolerances().angle):
                return False
            if curve0.Direction.cross(curve_i.Position - curve0.Position).length > 1e-5:
                return False
        return True

    if type(curve0) in (GCircle, GEllipse):
        for ei in edges[1:]:
            curve_i = ei.Curve
            if type(curve_i) is not type(curve0):
                return False
            if not is_parallel(curve0.Axis, curve_i.Axis, Tolerances().angle):
                return False
            if (curve_i.Center - curve0.Center).length > 1e-5:
                return False
            if type(curve0) is GCircle:
                if abs(curve_i.Radius - curve0.Radius) > 1e-5:
                    return False
            else:
                if abs(curve_i.MajorRadius - curve0.MajorRadius) > 1e-5:
                    return False
                if abs(curve_i.MinorRadius - curve0.MinorRadius) > 1e-5:
                    return False
        return True

    if type(curve0) is GBSpline:
        # No cheap analytic identity test for a BSpline curve -- fall
        # back to a weaker but meaningful check instead: the edges must
        # chain into a single connected loop (each shares a vertex with
        # the next) with a continuous tangent direction at every shared
        # vertex. A single intersection curve split into several edges
        # by the CAD kernel always has this property; an arbitrary
        # jumble of unrelated edges from a messy cut does not.
        remaining = list(edges)
        chain = [remaining.pop(0)]
        while remaining:
            tail = chain[-1].Vertexes[-1]
            for i, ei in enumerate(remaining):
                if (ei.Vertexes[0] - tail).length < 1e-5 or (ei.Vertexes[-1] - tail).length < 1e-5:
                    chain.append(remaining.pop(i))
                    break
            else:
                return False  # no remaining edge continues the chain

        for i in range(len(chain) - 1):
            t1 = chain[i].derivative1_at(chain[i].ParameterRange[1])
            t2 = chain[i + 1].derivative1_at(chain[i + 1].ParameterRange[0])
            if t1.length < 1e-5 or t2.length < 1e-5:
                continue
            if not is_parallel(t1.normalized(), t2.normalized(), Tolerances().angle):
                return False
        return True

    return False  # unsupported curve type


def edge_1D(edge):
    if edge.Length < 1e-5:
        return False
    p0, p1 = edge.ParameterRange
    pe = 0.5 * (p1 + p0)
    return edge.curvature(pe) < 1e-6


def spline_2D(edge):
    knots = edge.knots()

    if edge.curvature(knots[0]) < 1e-6:
        return False  # straight line

    d0 = edge.derivative1_at(knots[0])
    if d0.length < 1e-5:
        return False

    norm_0 = d0.cross(edge.normal_at(knots[0])).normalized()

    for k in knots[1:]:
        # check if derivative orthogonal to curve normal vector
        dk = edge.derivative1_at(k)
        normal_k = dk.cross(edge.normal_at(k)).normalized()
        if abs(1.0 - abs(normal_k.dot(norm_0))) > Tolerances().value:
            return False
    return True


def get_shell_UV_nodes(face_or_shell):

    if type(face_or_shell) is ShellFaceGu:
        _, _, ifacemin, ifacemax = face_or_shell.U_parameter_range
        Faces = face_or_shell.Faces
    else:
        ifacemin = 0
        ifacemax = 0
        Faces = [face_or_shell]

    if ifacemin == ifacemax:
        face = Faces[ifacemin]
        UVNode_min = tessellate_face(face)
        if not UVNode_min:
            # tessellate() can succeed (no RuntimeError) yet still return
            # zero UV nodes on some healed/degenerate faces -- treat that
            # the same as the tessellation-failed case rather than leaving
            # the min/max search loop below with nothing to iterate.
            PR = face.ParameterRange
            UVNode1 = (PR[0], PR[2])
            UVNode2 = (PR[1], PR[3])
            UVNode_min = (UVNode1, UVNode2)
        UVNode_max = UVNode_min
    else:
        face_min = Faces[ifacemin]
        UVNode_min = tessellate_face(face_min)
        if not UVNode_min:
            PR = face_min.ParameterRange
            UVNode_min = ((PR[0], PR[2]),)

        face_max = Faces[ifacemax]
        UVNode_max = tessellate_face(face_max)
        if not UVNode_max:
            PR = face_max.ParameterRange
            UVNode_max = ((PR[1], PR[3]),)

    return UVNode_min, UVNode_max


def tessellate_face(face):
    try:
        face.tessellate(0.1)
        UVNode = face.getUVNodes()
    except RuntimeError:
        UVNode = ()
    return UVNode


def get_additional_corner_plane(cylinder):

    if type(cylinder) is ShellFaceGu:
        umin, umax = cylinder.U_parameter_range[:2]
        vmin, vmax = cylinder.Faces[0].ParameterRange[2:]
        face = cylinder.Faces[0]
    else:
        umin, umax, vmin, vmax = cylinder.ParameterRange
        face = cylinder
    v_value = 0.5 * (vmin + vmax)

    pos1 = face.value_at(umin, v_value)
    pos2 = face.value_at(umax, v_value)
    t, z = face.tangent_at(umin, v_value)

    point = 0.5 * (pos1 + pos2)
    r12 = pos1 - pos2
    paxis = z.cross(r12).normalized()  # v1 fixed vector oriented toward cylinder arc, z ref rotation axis

    return GeounedSurface(("Plane", (point, paxis, 1.0, 1.0, False)))
