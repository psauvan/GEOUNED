"""
geo/freecad/queries.py

Spatial queries between two independent shapes (Gin_contact/Gdistance).
"""

from __future__ import annotations

from .topology import GShape


# ---------------------------------------------------------------------------
# Spatial queries between two independent shapes
# ---------------------------------------------------------------------------


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
