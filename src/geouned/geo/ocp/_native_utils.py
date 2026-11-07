"""
geo/ocp/_native_utils.py

Native-conversion helpers and kernel provenance -- split out of the
former monolithic _ocp_impl.py (2026-08-30), grouped by function
alongside its freecad/occ siblings of the same name.
"""

from __future__ import annotations

import OCP

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_Torus
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.GProp import GProp_GProps
from OCP.gp import (
    gp_Dir,
    gp_Pnt,
    gp_Trsf,
    gp_Vec,
)
from OCP.ShapeFix import ShapeFix_Shape
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from ..constants import NUMERIC_TOL, POINT_POINT_TOL
from ..vector_geometry import (
    GBoundBox,
    GMatrix,
    GVector,
    to_gvector,
)


def _unimplemented(name):
    def _raise(*args, **kwargs):
        raise NotImplementedError(f"geo.ocp.{name} is not implemented yet.")

    _raise.__name__ = name
    return _raise


def kernel_version() -> str:
    return OCP.__version__


def to_native_vector(vector: GVector) -> gp_Pnt:
    """Name kept for cross-backend API compatibility -- despite the
    name, this returns an OCP gp_Pnt, not a FreeCAD.Vector."""
    return gp_Pnt(vector.x, vector.y, vector.z)


def to_native_matrix(matrix: GMatrix) -> gp_Trsf:
    """Name kept for cross-backend API compatibility with
    _freecad_impl.py's own to_native_matrix -- despite the name, this returns
    an OCP gp_Trsf, not a FreeCAD.Matrix. Assumes `matrix` represents a
    rigid transform (rotation + translation, no scale/shear): true for
    every matrix GEOReverse ever builds (MCNP TRn cards), the only real
    consumer of this function today."""
    trsf = gp_Trsf()
    trsf.SetValues(
        matrix.A11,
        matrix.A12,
        matrix.A13,
        matrix.A14,
        matrix.A21,
        matrix.A22,
        matrix.A23,
        matrix.A24,
        matrix.A31,
        matrix.A32,
        matrix.A33,
        matrix.A34,
    )
    return trsf


def _to_gvector(pnt) -> GVector:
    """Internal: native gp_Pnt/gp_Dir/gp_Vec -> GVector. Not exported --
    the public `to_gvector` (vector_geometry.py) duck-types on .x/.y/.z
    attributes, which these OCP types don't have (they use .X()/.Y()/.Z()
    methods instead)."""
    return GVector(pnt.X(), pnt.Y(), pnt.Z())


def _orientation_str(shape) -> str:
    """TopAbs_Orientation (FORWARD=0/REVERSED=1/...) -> the "Forward"/
    "Reversed" strings GEOUNED compares against everywhere, matching
    FreeCAD's own native .Orientation string attribute."""
    return "Reversed" if shape.Orientation() == 1 else "Forward"


def _to_gmatrix_3x3(mat) -> GMatrix:
    """A gp_Mat (3x3, .Value(i,j) 1-indexed) -> GMatrix. Only A11..A33
    are ever meaningful for GEdge/GWire.MatrixOfInertia (see GMatrix's
    own docstring) -- the rest of the 4x4 is filled with the
    translation-free identity convention (A44=1, all else 0)."""
    return GMatrix(
        mat.Value(1, 1),
        mat.Value(1, 2),
        mat.Value(1, 3),
        0.0,
        mat.Value(2, 1),
        mat.Value(2, 2),
        mat.Value(2, 3),
        0.0,
        mat.Value(3, 1),
        mat.Value(3, 2),
        mat.Value(3, 3),
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )


def _linear_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    BRepGProp.LinearProperties_s(shape, props)
    return props


def _surface_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, props)
    return props


def _volume_props(shape) -> GProp_GProps:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props


def _edge_curve_and_range(native_edge):
    """Restores pythonocc-core's BRep_Tool.Curve(edge) -> (curve, first,
    last) convenience (or None if the edge is degenerate/has no 3D
    curve) -- OCP's BRep_Tool.Curve_s needs dummy first/last args (their
    values are discarded, verified live) and BRep_Tool.Range_s is a
    separate call that returns the real (first, last) tuple on its own."""
    curve = BRep_Tool.Curve_s(native_edge, 0.0, 0.0)
    if curve is None:
        return None
    first, last = BRep_Tool.Range_s(native_edge)
    return curve, first, last


def _project_point_on_surface(point: GVector, geom_surface) -> tuple[float, float]:
    """(u, v) of the nearest point on `geom_surface` to `point`. The
    default GeomAPI_ProjectPointOnSurf algorithm (Extrema_ExtAlgo_Grad,
    gradient-based) occasionally fails to converge (StdFail_NotDone) on
    a real, well-formed surface/point pair -- confirmed on a real case
    in PiezaDavid.stp -- so this retries with the more exhaustive
    Extrema_ExtAlgo_Tree before giving up. Confirmed via that same case:
    even Extrema_ExtAlgo_Tree can genuinely fail for a point sitting
    exactly on a cylinder's own axis (r=0) -- there the "nearest point"
    isn't mathematically unique (every point on the circle at that
    height is equidistant), not a numerical fluke, so no algorithm can
    do better; falls back to (0.0, 0.0), an arbitrary-but-deterministic
    answer for what is an inherently undefined query at that exact point."""
    from OCP.Extrema import Extrema_ExtAlgo_Tree

    proj = GeomAPI_ProjectPointOnSurf(to_native_vector(point), geom_surface)
    if proj.IsDone():
        return proj.LowerDistanceParameters()
    proj = GeomAPI_ProjectPointOnSurf(to_native_vector(point), geom_surface, Extrema_ExtAlgo_Tree)
    if proj.IsDone():
        return proj.LowerDistanceParameters()
    return (0.0, 0.0)


def _bnd_box(shape) -> GBoundBox:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return GBoundBox(xmin, ymin, zmin, xmax, ymax, zmax)


def _degenerate_pole_surface_signature(face):
    """`(kind, *params)` for `face`'s own analytic surface if it is a
    sphere/cone/torus (the 3 types that can carry a native OCCT
    *degenerate* edge -- a pole/apex, see `find_degenerate_plane_faces`'s
    own docstring for why a plane never has one), else `None`. Pure
    native OCCT/OCP (`BRepAdaptor_Surface`/`gp_Sphere`/`gp_Cone`/
    `gp_Torus`), no `GSolid`/`GFace` dependency -- this helper must run
    on a bare native shape before any `GSolid` wrapper can exist (this
    module's own `_native_fix` is called from `Gload_and_process_step`
    before `GSolid.__init__` ever runs)."""
    adaptor = BRepAdaptor_Surface(face, True)
    kind = adaptor.GetType()
    if kind == GeomAbs_Sphere:
        s = adaptor.Sphere()
        return ("sphere", s.Location(), s.Radius())
    if kind == GeomAbs_Cone:
        c = adaptor.Cone()
        return ("cone", c.Apex(), c.Position().Direction(), c.SemiAngle())
    if kind == GeomAbs_Torus:
        t = adaptor.Torus()
        return ("torus", t.Location(), t.Position().Direction(), t.MajorRadius(), t.MinorRadius())
    return None


def _same_degenerate_pole_surface(sig_a, sig_b) -> bool:
    """Loose, intrinsic-tolerance equality between two
    `_degenerate_pole_surface_signature` results -- deliberately generous
    (a false positive here only costs a little lost simplification, via
    `KeepShape` forbidding a merge that would have been safe anyway; a
    false negative risks the real, confirmed crash this whole mechanism
    exists to avoid), so plain `POINT_POINT_TOL`/`NUMERIC_TOL` suffice --
    no need for the full per-surface `Tolerances` machinery
    `geo.surface_geometry.is_same_sphere_surface`/`is_same_cone_surface`/
    `is_same_torus_surface` use, which needs an already-built `GFace`
    this module's own callers don't have yet."""
    if sig_a is None or sig_b is None or sig_a[0] != sig_b[0]:
        return False
    kind = sig_a[0]
    if kind == "sphere":
        _, center_a, radius_a = sig_a
        _, center_b, radius_b = sig_b
        return center_a.Distance(center_b) < POINT_POINT_TOL and abs(radius_a - radius_b) < POINT_POINT_TOL
    if kind == "cone":
        _, apex_a, axis_a, angle_a = sig_a
        _, apex_b, axis_b, angle_b = sig_b
        return (
            apex_a.Distance(apex_b) < POINT_POINT_TOL
            and abs(abs(axis_a.Dot(axis_b)) - 1.0) < NUMERIC_TOL
            and abs(angle_a - angle_b) < NUMERIC_TOL
        )
    # torus
    _, center_a, axis_a, major_a, minor_a = sig_a
    _, center_b, axis_b, major_b, minor_b = sig_b
    return (
        center_a.Distance(center_b) < POINT_POINT_TOL
        and abs(abs(axis_a.Dot(axis_b)) - 1.0) < NUMERIC_TOL
        and abs(major_a - major_b) < POINT_POINT_TOL
        and abs(minor_a - minor_b) < POINT_POINT_TOL
    )


def _has_degenerate_edge(face) -> bool:
    eexp = TopExp_Explorer(face, TopAbs_EDGE)
    while eexp.More():
        if BRep_Tool.Degenerated_s(TopoDS.Edge(eexp.Current())):
            return True
        eexp.Next()
    return False


def _degenerate_pole_keep_edges(shape) -> list:
    """Every native edge that must be passed to
    `ShapeUpgrade_UnifySameDomain.KeepShape(...)` before `Build()`, to
    avoid a confirmed native access-violation crash in OCCT 7.9.3's own
    `UnifyFaces()` internals (`Mixed/ConeSphere.stp`, 2026-11-07,
    reproduced identically under both `occ` and `ocp`): merging a face
    with NO degenerate edge into an ADJACENT face on the SAME analytic
    sphere/cone/torus surface that DOES carry one (a pole/apex)
    reliably crashes -- confirmed by direct bisection down to a minimal
    2-face reproducer (`ConeSphere.stp`'s own face#1, a full sphere
    wedge with no degenerate edge, merging into face#4/face#5, each
    carrying one). A pair where BOTH or NEITHER face has a degenerate
    edge merges safely (confirmed: the same file's face#2/face#3 pair,
    both degenerate, and face#4/face#5 pair, both degenerate, each
    merge without incident) and is never protected here -- so this
    still lets `UnifyFaces=True` fully simplify everything that IS safe
    to merge, rather than disabling it for the whole shape.

    `KeepShape` on an edge "forbids merging of connected faces" per its
    own documented contract -- passing the pair's own SHARED edge is
    exactly the surgical exclusion needed, confirmed live to let the
    rest of the shape's own safe merges (e.g. that same face#2/face#3
    and face#4/face#5 pair) proceed normally.

    "Same surface" is checked loosely (`_same_degenerate_pole_surface`,
    plain intrinsic tolerances) -- a false positive here only costs a
    little unnecessary protection (safe), never risks missing a real
    one. No `GSolid`/`GFace` needed (see `_degenerate_pole_surface_
    signature`'s own docstring); works directly on a bare `TopoDS_Shape`
    via `TopExp_Explorer`, so this can run before `_native_fix` builds
    anything, or be reused by any other `ShapeUpgrade_UnifySameDomain`
    call site in this package (`repair.py`'s own `Gsliver_heal`,
    `GSolid.refine()` in `topology.py`)."""
    faces = []
    fexp = TopExp_Explorer(shape, TopAbs_FACE)
    while fexp.More():
        faces.append(TopoDS.Face(fexp.Current()))
        fexp.Next()

    signatures = [_degenerate_pole_surface_signature(f) for f in faces]
    degenerate_flags = [_has_degenerate_edge(f) for f in faces]

    keep_edges = []
    for i in range(len(faces)):
        if signatures[i] is None:
            continue
        for j in range(i + 1, len(faces)):
            if signatures[j] is None or degenerate_flags[i] == degenerate_flags[j]:
                continue
            if not _same_degenerate_pole_surface(signatures[i], signatures[j]):
                continue
            eexp_i = TopExp_Explorer(faces[i], TopAbs_EDGE)
            edges_i = []
            while eexp_i.More():
                edges_i.append(TopoDS.Edge(eexp_i.Current()))
                eexp_i.Next()
            eexp_j = TopExp_Explorer(faces[j], TopAbs_EDGE)
            while eexp_j.More():
                edge_j = TopoDS.Edge(eexp_j.Current())
                if any(edge_j.IsSame(e) for e in edges_i):
                    keep_edges.append(edge_j)
                eexp_j.Next()
    return keep_edges


def _unify_same_domain_safe(
    shape,
    unify_edges: bool,
    unify_faces: bool,
    concat_bsplines: bool,
    linear_tolerance: float | None = None,
):
    """`ShapeUpgrade_UnifySameDomain(shape, unify_edges, unify_faces,
    concat_bsplines)`, pre-protected via `_degenerate_pole_keep_edges`
    against the confirmed `UnifyFaces=True` crash (see that function's
    own docstring) -- the single place every `ShapeUpgrade_
    UnifySameDomain` call in this package should go through, so a
    "valid, healed solid" means the same thing regardless of which call
    site produced it, instead of each one carrying its own ad hoc
    crash-avoidance (or none at all). `linear_tolerance`, when given,
    is set via `SetLinearTolerance` before `Build()` -- the one real
    per-call-site customization the 3 existing call sites need (`repair.
    py::Gsliver_heal` sets it to `OCCT_FIX_TOLERANCE`; the other 2 keep
    the native default). Returns the resulting `TopoDS_Shape`
    (``.Shape()`` already called) -- never raises on its own account;
    the native `Build()` call itself can still raise or (in principle,
    for an unrelated signature) crash, exactly as before."""
    unify = ShapeUpgrade_UnifySameDomain(shape, unify_edges, unify_faces, concat_bsplines)
    if linear_tolerance is not None:
        unify.SetLinearTolerance(linear_tolerance)
    if unify_faces:
        for edge in _degenerate_pole_keep_edges(shape):
            unify.KeepShape(edge)
    unify.Build()
    return unify.Shape()


def _native_fix(native, tolerance: float):
    """Native-in/native-out extraction of `GSolid.fix()`'s own body --
    see that method's docstring for the full, hard-won history of why
    every step here is exactly what it is (UnifyEdges-only-when-already-
    valid, the fallback repair running on the original `native` rather
    than UnifyEdges' own possibly-corrupted output). `GSolid.fix()`
    itself is a thin wrapper around this (`GSolid(_native_fix(self.
    __native__, tolerance))`) -- single source of this fragile logic,
    not two copies to keep in sync, while still letting
    `Gload_and_process_step` call it directly on the raw loaded shape,
    before that loop builds its own `GSolid`.

    `UnifyFaces=True`'s own confirmed native-crash risk (`Mixed/
    ConeSphere.stp`) is no longer just "kept on unconditionally per
    explicit user decision" -- `_unify_same_domain_safe` resolves the
    real mechanism directly (see its own docstring), so this call is
    both safe AND still gets the full simplification benefit."""
    if BRepCheck_Analyzer(native).IsValid():
        unified = _unify_same_domain_safe(native, True, True, True)
        # ShapeUpgrade_UnifySameDomain can degrade a genuine TopoDS_Solid
        # to a bare TopoDS_Shell that stays BRepCheck-valid (closed) but
        # is never re-wrapped as a solid -- the same class of bug
        # Gsliver_heal/Gcollapse_split_rings/Gtrim_overlapping_coplanar_faces
        # guard against in repair.py (confirmed live on
        # Big_model_reserved/divertor_cam.stp's own
        # STRUCTURAL_PLATE_2#BYZ2RP_31 solid, 2026-10-07). Reject the
        # unify result rather than silently return a Shell as "the
        # native solid" -- fall through to the untouched original.
        if BRepCheck_Analyzer(unified).IsValid() and TopExp_Explorer(unified, TopAbs_SOLID).More():
            return unified
    fixer = ShapeFix_Shape(BRepBuilderAPI_Copy(native).Shape())
    fixer.SetPrecision(tolerance)
    fixer.Perform()
    fixed = fixer.Shape()
    if TopExp_Explorer(fixed, TopAbs_SOLID).More():
        return fixed
    return native
