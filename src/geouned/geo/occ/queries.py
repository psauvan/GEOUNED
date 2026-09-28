"""
geo/occ/queries.py

Spatial queries between two independent shapes (Gin_contact/Gdistance).
"""

from __future__ import annotations

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.ShapeFix import ShapeFix_ShapeTolerance
from OCC.Core.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_SHAPE,
    TopAbs_VERTEX,
)
from OCC.Core.TopExp import topexp, TopExp_Explorer
from OCC.Core.TopoDS import topods
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from .topology import GShape
from ._native_utils import _bnd_box, _volume_props
from ..constants import VOLUME_MIN_E8


# ---------------------------------------------------------------------------
# Spatial queries between two independent shapes
# ---------------------------------------------------------------------------


def Gsolid_max_tolerance(solid: GShape) -> float:
    """Largest BRep tolerance carried by any edge or vertex of `solid`.

    A clean solid sits around 1e-6..1e-5 mm; a value orders of magnitude
    above that (and above the fuzzy `split_tolerance` a cut would use)
    means BOPAlgo papered a near-tangent junction over with inflated
    tolerance instead of separating it -- the signature `generic_split`
    uses to decide a STEP round-trip (Gheal_topology) is worth trying on
    an otherwise-stuck fragment.
    """
    native = solid.__native__
    worst = 0.0
    exp = TopExp_Explorer(native, TopAbs_EDGE)
    while exp.More():
        worst = max(worst, BRep_Tool.Tolerance(topods.Edge(exp.Current())))
        exp.Next()
    exp = TopExp_Explorer(native, TopAbs_VERTEX)
    while exp.More():
        worst = max(worst, BRep_Tool.Tolerance(topods.Vertex(exp.Current())))
        exp.Next()
    return worst


def Gsolid_set_tolerance(solid: GShape, value: float) -> None:
    """Uniformly set every vertex/edge/face tolerance of `solid` to
    `value`, mutating its own native shape in place (not a copy -- every
    other reference to the same underlying TopoDS data sees the change
    too, which is exactly what undoes the mutation this function exists
    to counter).

    `BOPAlgo_Splitter.Perform()` can raise a shape's own BRep tolerances
    in place as a side effect of merely attempting an intersection, even
    when the candidate tool ultimately fails to split it at all (still 1
    output piece) -- confirmed live on `Mixed/multiplane_add_plane_cyl.stp`:
    an earlier, failed Plane candidate raised the base fragment's own
    tolerance 5x (1.2e-7 -> 6.0e-7), and a LATER candidate that would have
    split it correctly (3 pieces summing exactly to the input volume, on a
    pristine copy) instead under-separated it (2 pieces, losing a whole
    276 mm^3 fragment) once tried against the now-degraded shape. Since
    `Gsplit`'s own `base` argument is passed by reference straight into
    `BOPAlgo_Splitter.AddArgument`, this degradation is cumulative and
    order-dependent across the WHOLE candidate-surface search in
    `generic_split`, not scoped to one failed attempt -- resetting the
    base solid's own tolerance back to its own pre-search baseline before
    every new candidate is what actually undoes it.
    """
    ShapeFix_ShapeTolerance().SetTolerance(solid.__native__, value, TopAbs_SHAPE)


def Gsolid_nonmanifold_edge_count(solid: GShape) -> int:
    """Number of edges of `solid` shared by other than exactly 2 faces.

    A watertight closed solid has every edge on exactly 2 faces; a nonzero
    count on a `BRepCheck`-valid solid means a boolean op left a
    non-manifold junction it papered over rather than resolved. Combined
    with an inflated `Gsolid_max_tolerance`, this is the signature of a
    near-tangent BOPAlgo weld -- see `generic_split`.
    """
    edge_faces = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(solid.__native__, TopAbs_EDGE, TopAbs_FACE, edge_faces)
    return sum(
        1
        for i in range(1, edge_faces.Size() + 1)
        if edge_faces.FindFromIndex(i).Size() != 2
    )


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
        if abs(props.Mass()) > VOLUME_MIN_E8:
            return True
        for kind in (TopAbs_FACE, TopAbs_EDGE):
            if TopExp_Explorer(common, kind).More():
                return True
        return False
    except Exception:
        return False


def Gdistance(shape_a: GShape, shape_b: GShape) -> float:
    return BRepExtrema_DistShapeShape(shape_a.__native__, shape_b.__native__).Value()
