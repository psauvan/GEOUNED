"""
geo/ocp/queries.py

Spatial queries between two independent shapes (Gin_contact/Gdistance).
"""

from __future__ import annotations

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
)
from OCP.TopExp import TopExp_Explorer
from .topology import GShape
from ._native_utils import _bnd_box, _volume_props


def Gin_contact(shape_a: GShape, shape_b: GShape, tolerance: float) -> bool:
    native_a = shape_a.__native__
    native_b = shape_b.__native__

    box_a = _bnd_box(native_a)
    box_b = _bnd_box(native_b)
    if not (
        min(box_a.XMax, box_b.XMax) - max(box_a.XMin, box_b.XMin) > -tolerance
        and min(box_a.YMax, box_b.YMax) - max(box_a.YMin, box_b.YMin) > -tolerance
        and min(box_a.ZMax, box_b.ZMax) - max(box_a.ZMin, box_b.ZMin) > -tolerance
    ):
        return False

    try:
        return BRepExtrema_DistShapeShape(native_a, native_b).Value() < tolerance
    except Exception:
        pass

    try:
        common = BRepAlgoAPI_Common(native_a, native_b).Shape()
        if common.IsNull():
            return False
        props = _volume_props(common)
        if abs(props.Mass()) > 1e-8:
            return True
        for kind in (TopAbs_FACE, TopAbs_EDGE):
            if TopExp_Explorer(common, kind).More():
                return True
        return False
    except Exception:
        return False


def Gdistance(shape_a: GShape, shape_b: GShape) -> float:
    return BRepExtrema_DistShapeShape(shape_a.__native__, shape_b.__native__).Value()
