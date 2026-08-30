"""
geo/occ/io.py

STEP load/export and the assembly-label walk (Gload_step_labels, via
OCC's own XCAF document tree).
"""

from __future__ import annotations

import OCC

from OCC.Core.BOPAlgo import BOPAlgo_Splitter
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.ShapeFix import ShapeFix_Shape
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.STEPControl import (
    STEPControl_AsIs,
    STEPControl_Reader,
    STEPControl_Writer,
)
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from ..io_utils import (
    GLabelNode,
    suppress_native_stdout,
)
from .topology import GEdge, GFace, GShape, GShell, GSolid
from .repair import Gcheck_and_repair, Gspline_surface
from ._native_utils import _native_fix


def _export_shapes_step(native_shapes: list, filename: str) -> None:
    # STEPControl_Writer prints its own "Statistics on Transfer (Write)"
    # banner directly via std::cout, unconditionally, split across TWO
    # calls (confirmed live, 2026-08-27, by isolating each call with its
    # own flush-marked print: the first half prints during Transfer(),
    # the second half during Write() -- wrapping only Write() leaves the
    # first half visible). No Interface_Static parameter or WorkSession
    # trace-level setter exists to silence it either way; see
    # io_utils.py's suppress_native_stdout docstring.
    with suppress_native_stdout():
        writer = STEPControl_Writer()
        for shape in native_shapes:
            writer.Transfer(shape, STEPControl_AsIs)
        status = writer.Write(filename)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP export failed for {filename} (status={status})")


def Gfirst_shell(native_shape):
    """The first shell of a native solid shape. See _freecad_impl.py's
    Gfirst_shell docstring for why this exists and where it's called
    from."""
    from OCC.Core.TopAbs import TopAbs_SHELL

    explorer = TopExp_Explorer(native_shape, TopAbs_SHELL)
    return topods.Shell(explorer.Current())


def Gload_step(filename: str) -> list[GSolid]:
    """Loads a STEP file's solids and heals each one (GSolid.fix(1e-6),
    ShapeFix_Shape) before returning it.

    FreeCAD's own STEP importer (Part.Shape.read, _freecad_impl.py's
    Gload_step) does this kind of cleanup implicitly as part of its
    translation pipeline; pyOCC's raw STEPControl_Reader does not -- a
    solid loaded this way can carry small tolerance/topology issues
    invisible to BRepCheck_Analyzer.IsValid() but severe enough to make
    later native BOP calls (BOPAlgo_Splitter, BRepAlgoAPI_Common, even
    the "reliable" BRepExtrema_DistShapeShape fallback) pathologically
    slow or hang outright on otherwise simple, valid-looking geometry.
    Confirmed live (2026-08-16, hylife-v06.stp solid 17 -- a simple
    solid FreeCAD converts without any issue): unhealed, decompose_solids()
    never returned (traced via py-spy to 3 different native hangs across
    repeated attempts); healed via .fix(1e-6) right after load (volume
    shift ~0.003%, real cleanup not corruption), the exact same solid
    decomposes cleanly in ~23s. GeounedSolid.__init__ already calls
    .refine() (ShapeUpgrade_UnifySameDomain) on every loaded solid, but
    that alone was not sufficient here -- refine() has no ShapeFix_Shape
    step of its own; only .fix() does, which is why the healing has to
    happen here, not left to rely on that later call."""
    reader = STEPControl_Reader()
    status = reader.ReadFile(filename)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed for {filename} (status={status})")
    reader.TransferRoots()
    shape = reader.OneShape()
    solids = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        solids.append(GSolid(topods.Solid(explorer.Current())).fix(1e-6))
        explorer.Next()
    return solids


def Gload_and_process_step(filename: str, tolerances) -> "tuple":
    """GEOUNED's own load-time pass: load every solid, natively fix it
    (`_native_fix`, the same healing `Gload_step` applies, required
    regardless of defect detection -- see `Gload_step`'s own docstring),
    build its `GSolid` right there in the loop, then run `Gcheck_and_
    repair` and `Gspline_surface` on it -- one pass over the solids,
    per explicit user direction (2026-08-28): read and process each
    solid in the same loop, with the `GSolid` built as one clear,
    unconditional step rather than deferred/optimized away.

    Returns `(gsolids, corrupted_indices, spline_indices)`:
      - `gsolids[i]` is the (possibly repaired) `GSolid` for solid `i`,
        positionally aligned with the original solid order (never
        dropped -- matching `Gload_step_labels`' own node/solid-count
        alignment contract), or `None` only when `i` is
        corrupted-and-unrepairable. A solid with an unsupported
        ("spline") surface still gets its real `GSolid` here --
        `Gload_and_process_step` itself has no concept of
        `spline_surfaces`' 3-way stop/remove/ignore mode
        (`corrupted_solids` only has 2 modes, stop/remove, with no
        "keep it anyway" option, so `None` is always correct there
        instead) -- `load_cad` is the one that knows the mode and
        decides whether to null this entry out for "remove"/"stop", or
        genuinely attempt translation on the as-loaded spline geometry
        for "ignore".
      - `corrupted_indices`/`spline_indices` are plain solid indices
        (0-based, matching `gsolids`' own positions) -- NOT `GSolid`
        objects -- since `GEOUNED/loadfile/load_step.py::load_cad` uses
        them directly for `i in removed_indexes` membership tests and to
        look up each one's own label/comment for reporting.

    If `Gcheck_and_repair` actually repairs the solid, the `GSolid` it
    returns is already a fresh one built from the truly-repaired native
    shape (each repair function -- `Gcollapse_split_rings`/
    `Gsliver_heal`/`Gdefeature` -- constructs its own `GSolid` internally
    from its own repaired result) -- so if the original solid had a
    sliver face, that sliver is gone from the `GSolid` this loop keeps,
    not just from some intermediate native shape never actually used.

    Checked in the same order `load_cad`'s own predecessor loop used:
    repair first (a solid that gets successfully repaired is still
    eligible for the spline check on its own, possibly-different,
    repaired geometry), corrupted-and-unrepaired short-circuits to a
    placeholder without ever reaching the spline check (matching the old
    loop's own `continue` there -- safe regardless of `corrupted_solids`
    mode, since "stop" mode exits before `gsolids` is ever used further
    anyway)."""
    reader = STEPControl_Reader()
    status = reader.ReadFile(filename)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed for {filename} (status={status})")
    reader.TransferRoots()
    shape = reader.OneShape()
    gsolids = []
    corrupted_indices = []
    spline_indices = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    index = 0
    while explorer.More():
        native_solid = _native_fix(topods.Solid(explorer.Current()), 1e-6)
        gsolid = GSolid(native_solid)
        gsolid, ok = Gcheck_and_repair(gsolid, tolerances)
        if not ok:
            corrupted_indices.append(index)
            gsolids.append(None)
        else:
            if Gspline_surface(gsolid.__native__):
                spline_indices.append(index)
            gsolids.append(gsolid)
        index += 1
        explorer.Next()

    return gsolids, corrupted_indices, spline_indices


def Gload_step_labels(filename: str) -> list[GLabelNode]:
    """
    Parse the STEP file's assembly tree via XCAF and return one
    `GLabelNode` per solid-bearing leaf, in the same order as
    `Gload_step`'s solids -- see `GLabelNode`'s own docstring for the
    positional-alignment contract this must satisfy. Only XCAF "simple
    shape" leaf labels are ever appended to the returned list (an
    assembly label's own conceptual shape is just a grouping container,
    matching FreeCAD's `Import.insert()`-based version, which only ever
    sees `Part::Feature` leaf objects, never the group/assembly
    containers STEP's importer builds around them) -- but assembly
    labels are still walked and given their own `GLabelNode`, used as
    the `parent` of their children, exactly like the FreeCAD version's
    non-solid-bearing ancestor nodes.
    """
    from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
    from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.XCAFApp import XCAFApp_Application
    from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

    app = XCAFApp_Application.GetApplication()
    doc = TDocStd_Document("XmlXCAF")
    app.NewDocument("XmlXCAF", doc)

    reader = STEPCAFControl_Reader()
    reader.SetColorMode(False)
    reader.SetNameMode(True)
    reader.SetLayerMode(False)
    reader.SetMatMode(False)
    status = reader.ReadFile(filename)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed for {filename} (status={status})")
    reader.Transfer(doc)

    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    nodes: list[GLabelNode] = []

    def count_solids(shape) -> int:
        n = 0
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        while exp.More():
            n += 1
            exp.Next()
        return n

    def walk(label, parent_node):
        name = label.GetLabelName()
        if shape_tool.IsReference(label):
            referred = TDF_Label()
            shape_tool.GetReferredShape(label, referred)
            walk(referred, parent_node)
            return
        if shape_tool.IsAssembly(label):
            node = GLabelNode(label=name, parent=parent_node, n_solids=0)
            comps = TDF_LabelSequence()
            shape_tool.GetComponents(label, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), node)
        elif shape_tool.IsSimpleShape(label):
            n_solids = count_solids(shape_tool.GetShape(label))
            if n_solids == 0:
                return
            if n_solids == 1:
                nodes.append(GLabelNode(label=name, parent=parent_node, n_solids=1))
            else:
                # A single XCAF "simple shape" label whose own geometry is a
                # compound of several solids -- unlike FreeCAD's Import.insert(),
                # which splits this into several separate Part::Feature leaf
                # objects (with auto-suffixed names), XCAF keeps it as one
                # label. Split into one node per solid here too, to preserve
                # the positional-alignment contract with Gload_step's own
                # per-solid TopExp_Explorer walk. The suffix below does NOT
                # attempt to replicate FreeCAD's own internal auto-suffix
                # convention (an undocumented Document-level object-naming
                # counter, not reliably derivable from XCAF data alone) --
                # it exists only so the N solids' own written comments
                # (load_step.py's `comment + "/" + label`) stay distinct
                # from each other. Confirmed live, 2026-08-24, without this:
                # every one of the N solids got the exact same label text,
                # not just a differently-suffixed one -- a real regression
                # from FreeCAD's own distinguishable-per-solid comments, not
                # merely a cosmetic mismatch.
                for i in range(n_solids):
                    nodes.append(GLabelNode(label=f"{name}_{i + 1}", parent=parent_node, n_solids=1))

    free_labels = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_labels)
    for i in range(1, free_labels.Length() + 1):
        walk(free_labels.Value(i), None)

    return nodes


def Gexport_step(shapes: list[GShape], filename: str) -> None:
    _export_shapes_step([shape.__native__ for shape in shapes], filename)
