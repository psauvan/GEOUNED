"""
geo/ocp/split_coaxial_cone.py

_try_coaxial_cone_split and its own private helpers -- the coaxial-
cone/cylinder degeneracy fallback Gsplit checks before the generic
BOP split.

Imports split.py's _raw_bop_split lazily (function-local, not at
module top) -- split.py's own Gsplit needs this module's
_try_coaxial_cone_split, so a top-level import here would be
circular.
"""

from __future__ import annotations

import math

from OCP.BOPAlgo import BOPAlgo_Splitter
from OCP.BRep import BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Splitter
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.BRepLib import BRepLib
from OCP.Geom2d import (
    Geom2d_Line,
    Geom2d_TrimmedCurve,
)
from OCP.gp import (
    gp_Dir2d,
    gp_Pnt2d,
)
from OCP.ShapeAnalysis import ShapeAnalysis_Surface
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_ListOfShape
from ..vector_geometry import GVector
from ..surface_geometry import (
    is_coaxial_cone_cylinder_pair,
    is_coaxial_cone_pair,
)
from .topology import GCone, GCylinder, GFace, GSolid
from ._native_utils import to_native_vector
from .primitives import Gmake_shell, Gmake_solid


def _find_cone_face(shape) -> "GFace | None":
    """First face of `shape` (anything with a `.Faces` list of GFace, e.g.
    a GSolid) whose analytic surface is a GCone, or None if it has none."""
    for f in shape.Faces:
        if isinstance(f.Surface, GCone):
            return f
    return None


def _group_coaxial_cone_faces(base_faces: "list[GFace]", tool_cone: "GCone") -> "list[list[GFace]]":
    """Groups of `base_faces` whose own cone surface is coaxial with, and
    shares the same |SemiAngle| as, `tool_cone` (see
    surface_geometry.is_coaxial_cone_pair) -- each group sharing one exact
    (Apex, Axis, SemiAngle) among its own members, i.e. real fragments of
    the *same* second cone (a solid can have that cone split into several
    faces by an earlier cut)."""
    groups: list[list[GFace]] = []
    for f in base_faces:
        s = f.Surface
        if not isinstance(s, GCone):
            continue
        if not is_coaxial_cone_pair(tool_cone, s):
            continue
        for group in groups:
            gs = group[0].Surface
            if (
                abs(gs.SemiAngle - s.SemiAngle) < 1e-6
                and abs(gs.Axis.dot(s.Axis)) > 1.0 - 1e-5
                and (gs.Apex - s.Apex).length < 1e-5
            ):
                group.append(f)
                break
        else:
            groups.append([f])
    return groups


def _group_coaxial_cylinder_faces(base_faces: "list[GFace]", tool_cone: "GCone") -> "list[list[GFace]]":
    """Cylinder counterpart of `_group_coaxial_cone_faces`: groups of
    `base_faces` whose own cylinder surface is coaxial with `tool_cone`
    (see surface_geometry.is_coaxial_cone_cylinder_pair) -- each group
    sharing one exact (Center-on-axis ignored, Axis, Radius) among its
    own members, i.e. real fragments of the *same* cylinder."""
    groups: list[list[GFace]] = []
    for f in base_faces:
        s = f.Surface
        if not isinstance(s, GCylinder):
            continue
        if not is_coaxial_cone_cylinder_pair(tool_cone, s):
            continue
        for group in groups:
            gs = group[0].Surface
            if abs(gs.Radius - s.Radius) < 1e-5 and abs(gs.Axis.dot(s.Axis)) > 1.0 - 1e-5:
                group.append(f)
                break
        else:
            groups.append([f])
    return groups


def _cone_v_value(point: GVector, native_cone_surf) -> float:
    return ShapeAnalysis_Surface(native_cone_surf).ValueOfUV(to_native_vector(point), 1e-6).Y()


def _find_v_crossings(face: "GFace", native_cone_surf, v0: float, samples: int = 64) -> "list[GVector]":
    """Points where `face`'s own outer-wire boundary crosses the constant
    V=v0 line on `native_cone_surf` (the surface `face` itself lies on) --
    i.e. where an analytically-known circle at that fixed V (see
    `_try_coaxial_cone_split`) crosses the face's *real* trimmed boundary.
    Samples each boundary edge and bisects across any sign change of
    (V - v0); works for any edge curve type (line, circle, BSpline...) and
    makes no assumption about how many boundary edges the face has."""
    crossings = []
    for edge in face.outer_wire().Edges:
        umin, umax = edge.ParameterRange
        prev_t = umin
        prev_v = _cone_v_value(edge.value_at(prev_t), native_cone_surf)
        for i in range(1, samples + 1):
            t = umin + (umax - umin) * i / samples
            v = _cone_v_value(edge.value_at(t), native_cone_surf)
            if (prev_v - v0) * (v - v0) < 0:
                lo, hi, lo_v = prev_t, t, prev_v
                for _ in range(60):
                    mid = (lo + hi) / 2.0
                    mid_v = _cone_v_value(edge.value_at(mid), native_cone_surf)
                    if (lo_v - v0) * (mid_v - v0) <= 0:
                        hi = mid
                    else:
                        lo, lo_v = mid, mid_v
                crossings.append(edge.value_at((lo + hi) / 2.0))
            prev_t, prev_v = t, v
    return crossings


def _split_face_at_v_line(native_face, native_cone_surf, point_a: GVector, point_b: GVector) -> list:
    """Split `native_face` (lying on `native_cone_surf`) at the constant-V
    line between `point_a`/`point_b` (both already confirmed to sit on
    that surface). The edge is built directly in the surface's own (U,V)
    space and needs BRepLib.BuildCurve3d_s before use as a splitting tool
    -- otherwise BRepAlgoAPI_Splitter crashes the process natively rather
    than raising (confirmed 2026-08-18). Returns the resulting native
    faces (a 1-element list if the split didn't actually separate anything)."""
    sas = ShapeAnalysis_Surface(native_cone_surf)
    uv_a = sas.ValueOfUV(to_native_vector(point_a), 1e-6)
    uv_b = sas.ValueOfUV(to_native_vector(point_b), 1e-6)
    v_common = (uv_a.Y() + uv_b.Y()) / 2.0
    line2d = Geom2d_Line(gp_Pnt2d(0.0, v_common), gp_Dir2d(1.0, 0.0))
    u_lo, u_hi = sorted([uv_a.X(), uv_b.X()])
    if u_hi - u_lo < 1e-9:
        # point_a/point_b project to (numerically) the same U on this
        # surface -- e.g. a periodic (cylinder/cone) surface where the two
        # candidate crossings differ by a full 2*pi wrap and so coincide
        # once reduced -- Geom2d_TrimmedCurve requires U1 != U2 and raises
        # Standard_ConstructionError otherwise (confirmed live, 2026-08-23,
        # on Cans/fwd_can_0.stp and 3 siblings once the cone/cylinder
        # candidate search started reaching this surface). Not a real arc
        # to split at; treat it the same as any other candidate that
        # doesn't pan out.
        return [native_face]
    edge = BRepBuilderAPI_MakeEdge(Geom2d_TrimmedCurve(line2d, u_lo, u_hi), native_cone_surf).Edge()
    BRepLib.BuildCurve3d_s(edge)

    splitter = BRepAlgoAPI_Splitter()
    args = TopTools_ListOfShape()
    args.Append(native_face)
    tools = TopTools_ListOfShape()
    tools.Append(edge)
    splitter.SetArguments(args)
    splitter.SetTools(tools)
    splitter.Build()
    if not splitter.IsDone():
        return [native_face]
    pieces = []
    exp = TopExp_Explorer(splitter.Shape(), TopAbs_FACE)
    while exp.More():
        pieces.append(TopoDS.Face(exp.Current()))
        exp.Next()
    return pieces if pieces else [native_face]


def _try_coaxial_cone_split(base: "GSolid", tool: "GSolid", tolerance_floor: float, tolerances) -> "list[GSolid] | None":
    """Checked *before* the generic split is even attempted, whenever
    `tool` is a cone -- avoids wastefully running BOPAlgo_Splitter once on
    geometry already known to defeat it, then again after the presplit
    fix (see Gsplit). Targets a specific, real degeneracy: `tool`'s own
    cutting surface is a cone that is coaxial
    with, and shares the same semi-angle as, a *different* cone already on
    `base`'s own boundary (see surface_geometry.is_coaxial_cone_pair). Two
    coaxial cones with equal semi-angle intersect in an exact circle,
    which is a genuinely degenerate case for OCCT's own quadric-quadric
    solver (confirmed 2026-08-18 against a real fixture,
    Solidos/BadCAD_decomposition/SCDR_90_piece0_badvolume.stp:
    BOPAlgo_Splitter silently returns the unsplit solid at every tolerance
    from 0.1 to 1e-22; GeomAPI_IntSS "succeeds" but returns a wrong curve,
    confined to one meridian plane, oscillating between the two apexes,
    rather than the real circle). No parameter exposed by OCP or
    pythonocc-core resolves this; patching OCCT's own C++ solver was
    explicitly ruled out (would mean maintaining a permanent OCCT fork).

    Rather than reconstructing the whole cut face by hand, this resolves
    only the one genuinely degenerate piece -- the circular arc where the
    tool's cone crosses the other cone -- in closed form (center = midpoint
    of the two apexes, radius = half their distance), splits just that one
    real face of `base` at the arc (an ordinary, well-conditioned
    face-level operation, not a 3D solid-level one), and retries the
    *normal* BOP split on the resulting solid: once the arc already exists
    as real topology, the tool no longer needs to discover it via the
    degenerate solver, and the ordinary 3D split succeeds on its own.

    Finding a coaxial-cone pair on `base` is a candidate, not a guarantee
    -- the pair may be unrelated to this particular cut (a real
    counterexample, given directly by the user: the first cut attempted
    while decomposing the un-decomposed
    Solidos/BadCAD_decomposition/SCDR_90.stp, which this fixture was itself
    cut from, has this kind of coincidental match elsewhere in the model
    and the *generic* split already works correctly there). Returns None
    whenever the candidate doesn't pan out at any step (not exactly 2 arc
    crossings found, the face doesn't actually split, the presplit solid
    doesn't rebuild validly, or the retried split still doesn't produce a
    volume-conserving multi-solid result) -- every one of these is an
    ordinary, silent "not applicable here" outcome, never an error; the
    caller falls through to today's existing unchanged-solid behavior.

    Also tries the cone/cylinder counterpart of the same degeneracy (see
    surface_geometry.is_coaxial_cone_cylinder_pair): `tool`'s cone reaching
    a coaxial cylinder's own radius at one exact height on `base`'s
    boundary. Confirmed live (2026-08-23) on a real fixture where this
    was the *actual* blocking degeneracy and the cone-cone search alone
    was actively misleading: it found an unrelated, coincidental 2-crossing
    arc on a nearby cone fragment (at the wrong height along the axis),
    "successfully" presplit and rebuilt a topologically valid solid there,
    and only the cylinder candidate at the *true* shared circle -- where a
    cutting cone, a cylinder, and a second cone all meet at once --
    actually let the retry split separate the solid. Trying every
    candidate in turn and keeping only the first whose retry genuinely
    succeeds (already the existing discipline) means a cone-cone false
    positive can never be silently preferred over a real cone-cylinder fix
    -- the false positive's own retry simply fails its safety net and the
    search moves on.
    """
    tool_cone_face = _find_cone_face(tool)
    if tool_cone_face is None:
        return None
    tool_cone = tool_cone_face.Surface

    candidates: list[tuple["GFace", GVector, float]] = []
    for group in _group_coaxial_cone_faces(base.Faces, tool_cone):
        for other_face in group:
            other_cone = other_face.Surface
            mid_point = (tool_cone.Apex + other_cone.Apex) * 0.5
            radius = (tool_cone.Apex - other_cone.Apex).length / 2.0
            candidates.append((other_face, mid_point, radius))
    for group in _group_coaxial_cylinder_faces(base.Faces, tool_cone):
        for other_face in group:
            cylinder = other_face.Surface
            tan_semi = math.tan(tool_cone.SemiAngle)
            if abs(tan_semi) < 1e-9:
                continue
            axis = tool_cone.Axis.normalized()
            t = cylinder.Radius / abs(tan_semi)
            # both nappes are tried -- whichever doesn't correspond to the
            # real, physical circle simply fails the crossings/retry
            # checks below and is silently skipped, same as any other
            # candidate that doesn't pan out.
            candidates.append((other_face, tool_cone.Apex + axis * t, cylinder.Radius))
            candidates.append((other_face, tool_cone.Apex - axis * t, cylinder.Radius))

    axis = tool_cone.Axis.normalized()
    for other_face, mid_point, radius in candidates:
        ref = GVector(1, 0, 0)
        if abs(ref.dot(axis)) > 0.9:
            ref = GVector(0, 1, 0)
        u_dir = (ref - axis * ref.dot(axis)).normalized()
        probe_point = mid_point + u_dir * radius

        native_other_surf = BRep_Tool.Surface_s(other_face.__native__)
        v0 = _cone_v_value(probe_point, native_other_surf)

        crossings = _find_v_crossings(other_face, native_other_surf, v0)
        if len(crossings) != 2:
            continue

        split_pieces = _split_face_at_v_line(other_face.__native__, native_other_surf, crossings[0], crossings[1])
        if len(split_pieces) < 2:
            continue

        new_faces = [f for f in base.Faces if f is not other_face]
        new_faces += [GFace(p) for p in split_pieces]

        try:
            presplit = Gmake_solid(Gmake_shell(new_faces))
        except Exception:
            presplit = None
        if presplit is None:
            continue

        # The presplit's own new edge is only ever geometrically exact to
        # floating-point precision, not identical to the tool's own BRep
        # representation of the same curve -- retrying at the caller's own
        # (often zero/near-zero) `tolerance` can still leave the two
        # topologies just barely too far apart for BOPAlgo_Splitter to
        # recognize them as coincident. Escalating through a fixed
        # tolerance ladder before giving up is the same "loosen fuzzy
        # value until it works" technique this project's own d1suned/OCCT
        # investigations have used throughout -- confirmed live
        # (2026-08-23) on a real fixture where tolerance=0 (and every
        # value up to 0.05) left the presplit solid unseparated, but 0.1
        # split it cleanly into the expected 2 pieces. A looser retry
        # tolerance also means the resulting fragments' own volumes carry
        # more numerical slack than an exact (tolerance=0) split would --
        # the volume-conservation check below is scaled accordingly
        # (still far tighter than the retry tolerance itself: confirmed
        # live that the real deviation at tolerance=0.1 is ~2.3e-4
        # relative, comfortably inside the 1e-3 bound used here).
        from .split import _raw_bop_split  # lazy: split.py imports this module too

        for retry_tolerance in (tolerance_floor, 1e-6, 1e-4, 1e-2, 0.1, 0.5, 1.0):
            if retry_tolerance < tolerance_floor:
                continue
            retry_native_solids, _ = _raw_bop_split(presplit.__native__, tool.__native__, retry_tolerance, tolerances)
            if len(retry_native_solids) < 2:
                continue
            retry_solids = [GSolid(s) for s in retry_native_solids]
            total_volume = sum(s.Volume for s in retry_solids)
            volume_rel_tol = 1e-6 if retry_tolerance == 0.0 else 1e-3
            if abs(total_volume - base.Volume) > volume_rel_tol * max(abs(base.Volume), 1.0):
                continue
            if not all(s.is_valid() for s in retry_solids):
                continue
            if len(retry_solids) > 1:
                return retry_solids
            else:
                return [base]

    return None
