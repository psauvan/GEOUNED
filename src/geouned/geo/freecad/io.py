"""
geo/freecad/io.py

STEP load/export and the assembly-label walk (Gload_step_labels, via
FreeCAD's own Import.insert/document tree).
"""

from __future__ import annotations

import uuid

import FreeCAD
import Part
from FreeCAD import Import

from .topology import GEdge, GFace, GShape, GShell, GSolid
from .repair import Gspline_surface
from ..io_utils import GLabelNode, suppress_native_stdout


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------


def Gfirst_shell(native_shape):
    """
    The first shell of a native solid shape. Native-in/native-out (used
    at the handful of GEOUNED call sites -- geouned_classes.py::build_surface,
    build_shape_functions.py::build_complex_shape -- that deliberately
    stay in native space at this boundary, per those files' own
    comments, rather than round-tripping through GSolid).
    """
    return native_shape.Shells[0]


def Gload_step(filename: str) -> list[GSolid]:
    """
    Load a STEP file and return the list of top-level solids, with every
    transformation from the file's assembly/placement hierarchy already
    applied (baked into each solid's own geometry) -- callers never need
    to apply a separate placement themselves.

    This is the plain loading primitive -- no defect check/repair, no
    spline detection -- kept exactly as-is for every caller that just
    wants solids back (tests, GEOReverse's own round-trip checks). See
    `Gload_and_process_step` for GEOUNED's own richer load-time pass.
    """
    shape = Part.Shape()
    shape.read(filename)
    return [GSolid(solid) for solid in shape.Solids]


def Gload_and_process_step(filename: str, tolerances) -> "tuple[list[GSolid], list[int], list[int]]":
    """GEOUNED's own load-time pass: load every solid and run
    `Gspline_surface` on it. `Gcheck_and_repair` is deliberately never
    called here -- under this engine it's an unconditional `(solid,
    True)` no-op (see its own docstring: no native CAD-repair tools are
    wired for FreeCAD), so calling it would only add overhead for zero
    effect; `corrupted_indices` is therefore always empty.

    Returns `(gsolids, corrupted_indices, spline_indices)`, positionally
    aligned with the original solid order (matching `Gload_step_labels`'
    own node/solid-count alignment contract) -- see `_ocp_impl.py`'s own
    `Gload_and_process_step` docstring for the full contract, including
    why a spline-bearing solid still gets its real `GSolid` here rather
    than `None` (GEOUNED/loadfile/load_step.py::load_cad is the one that
    knows `spline_surfaces`' own stop/remove/ignore mode and decides
    whether to null it out)."""
    shape = Part.Shape()
    shape.read(filename)
    gsolids = []
    spline_indices = []
    for index, native_solid in enumerate(shape.Solids):
        if Gspline_surface(native_solid):
            spline_indices.append(index)
        gsolids.append(GSolid(native_solid))
    return gsolids, [], spline_indices


def Gload_step_labels(filename: str) -> list[GLabelNode]:
    """
    Parse the same STEP file's assembly tree and return one `GLabelNode`
    per solid-bearing leaf, in the same order as `Gload_step`'s solids
    (see `GLabelNode`'s docstring for exactly how they line up). This is
    a separate read from `Gload_step`, not a by-product of it: extracting
    labels/hierarchy needs the file's assembly-tree structure (via
    `Import.insert`, which builds FreeCAD's own Part::Feature/Label/InList
    document tree), a different concern from -- and a different reader
    than -- resolving each solid's final, transformed geometry
    (`Part.Shape.read`, used by `Gload_step`, returns geometry only, no
    labels at all).
    """
    doc = FreeCAD.newDocument(uuid.uuid4().hex)
    try:
        Import.insert(filename, doc.Name)

        nodes: dict[str, GLabelNode] = {}

        def build_node(elem) -> GLabelNode:
            if elem.Name in nodes:
                return nodes[elem.Name]
            parent = build_node(elem.InList[0]) if elem.InList else None
            n_solids = 0
            if elem.TypeId == "Part::Feature" and elem.Shape.Solids:
                n_solids = len(elem.Shape.Solids)
            node = GLabelNode(label=elem.Label, parent=parent, n_solids=n_solids)
            nodes[elem.Name] = node
            return node

        return [build_node(elem) for elem in doc.Objects if elem.TypeId == "Part::Feature" and elem.Shape.Solids]
    finally:
        FreeCAD.closeDocument(doc.Name)


def Gexport_step(shapes: list[GShape], filename: str) -> None:
    """Export a list of shapes (any mix of GSolid/GFace/GEdge/GShell) to a single STEP file."""
    compound = Part.makeCompound([shape.__native__ for shape in shapes])
    with suppress_native_stdout():
        compound.exportStep(filename)
