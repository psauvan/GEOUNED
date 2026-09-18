"""
geo/occ/spline_quadrics.py

Detect a BSplineSurface face that is secretly a cylinder, sphere, or
torus (within tolerance) and substitute the exact analytic surface for
it in the native solid -- so `Gclassify_surface` processes it like any
other analytic face instead of `Gspline_surface` flagging the whole
solid as unsupported ("has a spline") and having it removed/halted at
load time.

Investigation and full methodology: CLAUDE.md's "Known open items" ->
"Shared / cross-cutting" -> "Spline-vs-quadric identification" entry
(opened 2026-09-18, branch `spline-quadric-detection`). Summary of what
is implemented here:

- Detection: sample a 15x15 grid of (point, normal) pairs over the
  face's own (u,v) domain (`GeomLProp_SLProps`), fit each candidate type
  (sphere: linear algebraic fit; cylinder: axis via SVD of mean-centered
  normals + algebraic circle fit; torus: hand-rolled Levenberg-Marquardt
  -- no scipy dependency, `ocpenv`/`pyoccenv` don't ship it) and accept
  the first, in sphere -> cylinder -> torus priority, whose residual
  falls under `tolerances.spline_quadric_fit_rel_tol` x the face's own
  bounding-box diagonal. The priority order matters: a sphere is a
  degenerate torus (major radius 0), so the torus fit alone would also
  match a genuine sphere face almost perfectly -- sphere must be tried
  first, or a real sphere would get needlessly reconstructed as a torus.
  Confirmed empirically during the investigation: a true positive (a
  real quadric round-tripped through BSpline conversion, even via a real
  STEP write/read) has a residual ~1e-12 to 1e-13 x face size, vs. ~1 for
  a genuine freeform surface -- about 9 orders of magnitude of margin.
- Substitution: `BRep_Builder.UpdateFace` alone leaves the solid
  `BRepCheck`-invalid (the face's edges still carry pcurve
  representations for the OLD surface) -- every edge needs a rebuilt
  pcurve too (`GeomProjLib.Curve2d`, or a hand-built `Geom2d_Line` for a
  degenerate pole/apex edge, whose 3D curve is null and can't be
  projected), followed by `BRepLib.SameParameter` + `ShapeFix_Face`.
- **Cone is deliberately NOT attempted here.** The exact same recipe
  leaves a cone face `BRepCheck_UnorientableShape` for reasons not yet
  root-caused (see the CLAUDE.md entry for the full investigation, both
  the topology-patch and an alternative boolean-based approach that were
  tried) -- wiring cone in before that is solved would silently accept
  broken topology. A cone-shaped spline face still falls through to the
  existing spline-solid policy unchanged.

`Gsubstitute_spline_quadrics` is all-or-nothing per solid, operating on
a native-shape copy: every unsupported face in the solid must resolve to
a valid substitution (fit succeeds AND the final whole-solid
`BRepCheck_Analyzer`/volume-conservation checks pass) for the copy to be
adopted; otherwise the original, untouched `GSolid` is returned
unchanged and the caller (`Gload_and_process_step`) falls back to its
existing spline-solid policy (remove the solid / stop execution,
depending on `corrupted_solids`), exactly as it did before this existed.
"""

from __future__ import annotations

import numpy as np

from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Builder, BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Copy
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepLib import breplib
from OCC.Core.Geom import (
    Geom_CylindricalSurface,
    Geom_SphericalSurface,
    Geom_ToroidalSurface,
)
from OCC.Core.Geom2d import Geom2d_Curve, Geom2d_Line
from OCC.Core.GeomAbs import GeomAbs_BSplineSurface
from OCC.Core.GeomLProp import GeomLProp_SLProps
from OCC.Core.GeomProjLib import geomprojlib
from OCC.Core.gp import gp_Ax3, gp_Dir, gp_Dir2d, gp_Pnt, gp_Pnt2d, gp_Vec2d
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
from OCC.Core.ShapeFix import ShapeFix_Face
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopoDS import topods
from ._native_utils import _volume_props
from .topology import GSolid, Gclassify_surface


def _faces_of(shape) -> list:
    faces = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        faces.append(topods.Face(exp.Current()))
        exp.Next()
    return faces


def _edges_of(face) -> list:
    edges = []
    exp = TopExp_Explorer(face, TopAbs_EDGE)
    while exp.More():
        edges.append(topods.Edge(exp.Current()))
        exp.Next()
    return edges


def _face_diagonal(face) -> float:
    box = Bnd_Box()
    brepbndlib.Add(face, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return float(((xmax - xmin) ** 2 + (ymax - ymin) ** 2 + (zmax - zmin) ** 2) ** 0.5)


def _sample_face(face, n: int = 15):
    """Grid-sample `n`x`n` (point, normal) pairs over the face's own
    (u,v) domain. Skips grid cells where the normal isn't defined (a
    degenerate/singular point) rather than raising."""
    surf = BRep_Tool.Surface(face)
    u1, u2, v1, v2 = surf.Bounds()
    pts, nrms = [], []
    for i in range(n):
        u = u1 + (u2 - u1) * (i + 0.5) / n
        for j in range(n):
            v = v1 + (v2 - v1) * (j + 0.5) / n
            props = GeomLProp_SLProps(surf, u, v, 1, 1e-9)
            if not props.IsNormalDefined():
                continue
            p = props.Value()
            nrm = props.Normal()
            pts.append([p.X(), p.Y(), p.Z()])
            nrms.append([nrm.X(), nrm.Y(), nrm.Z()])
    return np.array(pts), np.array(nrms)


def _fit_sphere(pts: np.ndarray):
    """Closed-form algebraic fit: x^2+y^2+z^2 + Dx+Ey+Fz+G = 0."""
    A = np.column_stack([pts, np.ones(len(pts))])
    b = -(pts**2).sum(axis=1)
    D, E, F, G = np.linalg.lstsq(A, b, rcond=None)[0]
    center = np.array([-D / 2, -E / 2, -F / 2])
    radius = float(np.sqrt(max(center @ center - G, 0.0)))
    residual = float(np.abs(np.linalg.norm(pts - center, axis=1) - radius).max())
    return {"center": center, "radius": radius}, residual


def _fit_cylinder(pts: np.ndarray, nrms: np.ndarray):
    """Axis via SVD of the mean-centered normals (they all lie
    perpendicular to the true axis; centering removes the constant-zero
    axial component so the smallest singular vector is the axis
    direction), then an algebraic circle fit in the plane perpendicular
    to it."""
    centered = nrms - nrms.mean(axis=0)
    _, _, vt = np.linalg.svd(centered)
    axis = vt[-1]
    axis = axis / np.linalg.norm(axis)
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, ref)
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    origin = pts.mean(axis=0)
    local = pts - origin
    x, y = local @ e1, local @ e2
    A = np.column_stack([x, y, np.ones_like(x)])
    b = -(x**2 + y**2)
    dcoef, ecoef, fcoef = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = -dcoef / 2, -ecoef / 2
    radius = float(np.sqrt(max(cx**2 + cy**2 - fcoef, 0.0)))
    center = origin + cx * e1 + cy * e2
    dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    residual = float(np.abs(dist - radius).max())
    return {"center": center, "axis": axis, "radius": radius}, residual


def _levenberg_marquardt(residual_fn, x0: np.ndarray, iters: int = 80, lam0: float = 1e-3):
    """Dependency-free Levenberg-Marquardt (numerical Jacobian) -- only
    used for the torus fit, which has no closed-form solution. Neither
    `ocpenv` nor `pyoccenv` ships scipy."""
    x = np.array(x0, dtype=float)
    lam = lam0
    r = residual_fn(x)
    cost = r @ r
    n = len(x)
    for _ in range(iters):
        jac = np.zeros((len(r), n))
        for k in range(n):
            dx = np.zeros(n)
            step = 1e-6 * max(abs(x[k]), 1.0)
            dx[k] = step
            jac[:, k] = (residual_fn(x + dx) - r) / step
        jtj = jac.T @ jac
        jtr = jac.T @ r
        for _ in range(20):
            try:
                delta = np.linalg.solve(jtj + lam * np.eye(n), -jtr)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            x_new = x + delta
            r_new = residual_fn(x_new)
            cost_new = r_new @ r_new
            if cost_new < cost:
                x, r, cost = x_new, r_new, cost_new
                lam = max(lam / 3, 1e-12)
                break
            lam *= 5
        else:
            break
    return x, r


def _fit_torus(pts: np.ndarray):
    """Nonlinear fit against the exact torus implicit distance,
    `sqrt((rho-R)^2 + s^2) - r`, initialized from a PCA-based axis guess
    and the point cloud's own radial spread."""
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid)
    axis0 = vt[-1]
    axis0 = axis0 / np.linalg.norm(axis0)
    d = pts - centroid
    s = d @ axis0
    radial = d - np.outer(s, axis0)
    rho = np.linalg.norm(radial, axis=1)
    major0 = rho.mean()
    minor0 = max(float(np.sqrt(((rho - major0) ** 2 + s**2)).mean()), 1e-3)

    def residuals(x):
        center = x[0:3]
        axis = x[3:6]
        axis = axis / np.linalg.norm(axis)
        major, minor = x[6], x[7]
        d = pts - center
        s = d @ axis
        radial = d - np.outer(s, axis)
        rho = np.linalg.norm(radial, axis=1)
        return np.sqrt((rho - major) ** 2 + s**2) - minor

    x0 = np.concatenate([centroid, axis0, [major0, minor0]])
    x, r = _levenberg_marquardt(residuals, x0)
    center, axis = x[0:3], x[3:6] / np.linalg.norm(x[3:6])
    major_radius, minor_radius = float(x[6]), float(x[7])
    residual = float(np.abs(r).max())
    return {"center": center, "axis": axis, "major_radius": major_radius, "minor_radius": minor_radius}, residual


def _detect_quadric(face, fit_tol: float):
    """Returns `(kind, params)` for the first of sphere/cylinder/torus
    (in that priority -- see module docstring) whose fit residual is
    below `fit_tol`, or `None` if none of the 3 match well enough."""
    pts, nrms = _sample_face(face)
    if len(pts) < 10:
        return None

    params, residual = _fit_sphere(pts)
    if residual < fit_tol:
        return "sphere", params

    params, residual = _fit_cylinder(pts, nrms)
    if residual < fit_tol:
        return "cylinder", params

    params, residual = _fit_torus(pts)
    if residual < fit_tol and 0.0 < params["minor_radius"] < params["major_radius"]:
        return "torus", params

    return None


def _build_native_surface(kind: str, params: dict):
    if kind == "sphere":
        return Geom_SphericalSurface(gp_Ax3(gp_Pnt(*params["center"]), gp_Dir(0, 0, 1)), params["radius"])
    if kind == "cylinder":
        return Geom_CylindricalSurface(gp_Ax3(gp_Pnt(*params["center"]), gp_Dir(*params["axis"])), params["radius"])
    if kind == "torus":
        return Geom_ToroidalSurface(
            gp_Ax3(gp_Pnt(*params["center"]), gp_Dir(*params["axis"])),
            params["major_radius"],
            params["minor_radius"],
        )
    raise ValueError(kind)


def _substitute_face_surface(face, new_surf) -> None:
    """The validated recipe (see module docstring): swap the surface,
    rebuild a pcurve for every one of the face's own edges on it
    (degenerate/pole edges get a hand-built `Geom2d_Line` -- their null
    3D curve can't be projected -- found via `ShapeAnalysis_Surface.
    ValueOfUV` point inversion of the vertex, spanning the new surface's
    own full U period), then `SameParameter` + `ShapeFix_Face`. Mutates
    `face` in place; never raises and never checks its own success --
    the caller validates the whole solid afterward
    (`Gsubstitute_spline_quadrics`).

    Note (shared seam edge): a face on a FULLY periodic surface (a whole
    cylinder/sphere/torus, not a partial/trimmed one) has its own seam
    edge appear TWICE in the wire -- same underlying `TopoDS_Edge`
    (`IsSame`), once FORWARD and once REVERSED -- needing TWO distinct
    pcurve representations on the SAME (surface, location), one at u=u1
    and one at u=u1+period. Calling the single-pcurve `UpdateEdge`
    overload once per occurrence does NOT do this -- the second call just
    overwrites the first, leaving only one side of the seam. This worked
    by accident for an exactly-`GeomConvert`-converted face (`ShapeFix_
    Face`'s own `FixMissingSeam` silently reconstructed the missing side)
    but broke for a face that had gone through a real STEP write/read
    round-trip first (the realistic case for an actually-loaded model) --
    confirmed live, 2026-09-18. Fixed by detecting the duplicate (`IsSame`)
    and using the two-pcurve `UpdateEdge(E, C1, C2, S, L, Tol)` overload
    directly: `C2` is `C1` translated by exactly the surface's own U
    period (both are then guaranteed geometrically consistent, since a
    surface's own periodicity means any pcurve shifted by its period
    still maps to the identical 3D curve). Note also that a solid's own
    `_native_fix`/`ShapeFix_Shape` healing pass (already applied before
    this function ever runs, at load time) sometimes splits a shared seam
    edge into two independent (non-`IsSame`) copies on its own -- in that
    case each copy legitimately needs only its own single pcurve, and
    this function's duplicate detection correctly falls back to that."""
    builder = BRep_Builder()
    face_tol = BRep_Tool.Tolerance(face)
    builder.UpdateFace(face, new_surf, face.Location(), face_tol)
    u1, u2, _, _ = new_surf.Bounds()
    period = u2 - u1
    surface_analysis = ShapeAnalysis_Surface(new_surf)

    all_edges = _edges_of(face)
    handled = []
    for edge in all_edges:
        edge_tol = BRep_Tool.Tolerance(edge)
        if BRep_Tool.Degenerated(edge):
            vertex = topexp.FirstVertex(edge)
            point = BRep_Tool.Pnt(vertex)
            uv = surface_analysis.ValueOfUV(point, 1e-6)
            pcurve = Geom2d_Line(gp_Pnt2d(u1, uv.Y()), gp_Dir2d(1, 0))
            builder.UpdateEdge(edge, pcurve, new_surf, face.Location(), edge_tol)
            builder.Degenerated(edge, True)
            continue
        if any(e.IsSame(edge) for e in handled):
            continue
        handled.append(edge)
        first, last = BRep_Tool.Range(edge)
        curve, _, _ = BRep_Tool.Curve(edge)
        pcurve = geomprojlib.Curve2d(curve, first, last, new_surf)
        duplicate = any(e.IsSame(edge) and e is not edge for e in all_edges)
        if duplicate:
            # pythonocc-core's own Geom2d_Curve.Translated() is bound at
            # the Geom2d_Geometry (base-class) level, returning that
            # wider handle type even though the object is still
            # concretely a curve -- DownCast back, or UpdateEdge's own
            # SWIG overload resolution rejects it outright (confirmed
            # live, 2026-09-18: TypeError, "wrong number or type of
            # arguments"). OCP does not have this issue.
            pcurve2 = Geom2d_Curve.DownCast(pcurve.Translated(gp_Vec2d(period, 0)))
            builder.UpdateEdge(edge, pcurve, pcurve2, new_surf, face.Location(), edge_tol)
        else:
            builder.UpdateEdge(edge, pcurve, new_surf, face.Location(), edge_tol)
    breplib.SameParameter(face, face_tol, True)
    ShapeFix_Face(face).Perform()


def Gsubstitute_spline_quadrics(solid: "GSolid", tolerances) -> "tuple[GSolid, bool]":
    """See module docstring. Returns `(solid, False)` unchanged unless
    EVERY unsupported face in it is a BSplineSurface that resolves to a
    valid cylinder/sphere/torus substitution -- never a partially-
    substituted result. Only called when `Gspline_surface(solid.
    __native__)` is already known True."""
    native_copy = BRepBuilderAPI_Copy(solid.__native__).Shape()
    original_volume = abs(_volume_props(solid.__native__).Mass())

    fit_rel_tol = tolerances.spline_quadric_fit_rel_tol
    volume_rel_tol = tolerances.spline_quadric_volume_rel_tol

    any_substituted = False
    for face in _faces_of(native_copy):
        if Gclassify_surface(face) is not None:
            continue
        if BRepAdaptor_Surface(face, True).GetType() != GeomAbs_BSplineSurface:
            # some other unsupported type (Bezier, revolution, ...) --
            # not attempted, falls through to the existing policy.
            return solid, False
        fit_tol = fit_rel_tol * _face_diagonal(face)
        detected = _detect_quadric(face, fit_tol)
        if detected is None:
            return solid, False
        kind, params = detected
        new_surf = _build_native_surface(kind, params)
        _substitute_face_surface(face, new_surf)
        any_substituted = True

    if not any_substituted:
        return solid, False

    if not BRepCheck_Analyzer(native_copy).IsValid():
        return solid, False

    new_volume = abs(_volume_props(native_copy).Mass())
    if original_volume > 0 and abs(new_volume - original_volume) > volume_rel_tol * original_volume:
        return solid, False

    return GSolid(native_copy), True
