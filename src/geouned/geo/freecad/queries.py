"""
geo/freecad/queries.py

Spatial queries between two independent shapes (Gin_contact/Gdistance).
"""

from __future__ import annotations

from .topology import GShape


# ---------------------------------------------------------------------------
# Spatial queries between two independent shapes
# ---------------------------------------------------------------------------


def Gsolid_max_tolerance(solid: GShape) -> float:
    """Largest BRep tolerance carried by any edge or vertex of `solid`.

    A clean solid sits around 1e-6..1e-5 mm; a value orders of magnitude
    above that (and above the fuzzy `split_tolerance` a cut would use)
    means the kernel papered a near-tangent junction over with inflated
    tolerance instead of separating it -- the signature `generic_split`
    uses to decide a STEP round-trip (Gheal_topology) is worth trying on
    an otherwise-stuck fragment. (FreeCAD `Gheal_topology` is a stub, so
    this only ever gates a no-op there, but the value is still real.)
    """
    try:
        return float(solid.__native__.getTolerance(1))
    except Exception:
        return 0.0


def Gsolid_nonmanifold_edge_count(solid: GShape) -> int:
    """Number of edges of `solid` shared by other than exactly 2 faces.

    A watertight closed solid has every edge on exactly 2 faces; a nonzero
    count means a boolean op left a non-manifold junction it papered over.
    Combined with an inflated `Gsolid_max_tolerance`, this is the
    signature of a near-tangent kernel weld -- see `generic_split`.
    """
    native = solid.__native__
    try:
        incidence = {}
        for face in native.Faces:
            for edge in face.Edges:
                key = edge.hashCode()
                incidence[key] = incidence.get(key, 0) + 1
        return sum(1 for v in incidence.values() if v != 2)
    except Exception:
        return 0


def Gin_contact(shape_a: GShape, shape_b: GShape, tolerance: float) -> bool:
    """
    True if shape_a and shape_b share at least one point in the
    volumetric sense (touching or overlapping within tolerance; e.g. two
    concentric spherical shells count as in contact). Accepts any
    combination of GSolid/GFace/GEdge/GShell.

    GEOUNED never needs the actual distance value, only this boolean --
    so this function owns all robustness workarounds internally
    (bounding-box pre-filtering, boolean-common fallback, degenerate/
    slow-kernel cases) rather than each call site reimplementing them.
    """
    native_a = shape_a.__native__
    native_b = shape_b.__native__

    box_intersection = native_a.BoundBox.intersected(native_b.BoundBox)
    if not (
        box_intersection.XLength > -tolerance
        and box_intersection.YLength > -tolerance
        and box_intersection.ZLength > -tolerance
    ):
        return False

    try:
        return native_a.distToShape(native_b)[0] < tolerance
    except Exception:
        pass

    if hasattr(native_a, "Volume") and hasattr(native_b, "Volume"):
        common = native_a.common(native_b)
        return abs(common.Volume) > 1e-8 or bool(common.Solids) or bool(common.Faces) or bool(common.Edges)
    return False


def Gdistance(shape_a: GShape, shape_b: GShape) -> float:
    """
    Minimum distance between shape_a and shape_b (0.0 if touching or
    overlapping). Unlike `Gin_contact`, this is a thin wrapper with no
    extra robustness layer -- use `Gin_contact` instead wherever only the
    boolean is actually needed.
    """
    return shape_a.__native__.distToShape(shape_b.__native__)[0]
