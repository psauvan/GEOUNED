"""
geo/io_utils.py

Split out of `vector_geometry.py` (2026-08-28). Not geometry math at all
-- process-level/loading utilities needed by more than one `_*_impl.py`
backend, kept in their own tiny file rather than mixed into a
geometry-math module.
"""

from __future__ import annotations

import contextlib
import os
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class GLabelNode:
    """
    One node of a STEP file's assembly/label tree, as read by
    `Gload_step_labels`. Only solid-bearing leaves are returned in that
    list, but `parent` still walks up through every ancestor (including
    non-solid-bearing group/assembly nodes) so callers can reconstruct a
    full label path, exactly as GEOUNED's own comment/material/dilution/
    enclosure parsing needs.

    Positionally aligned with `Gload_step`: node[i]'s `n_solids` solids
    are `Gload_step(...)`'s next `n_solids` entries, in order, once all
    nodes up to `i` have been consumed.
    """

    label: str
    parent: "GLabelNode | None"
    n_solids: int


@contextlib.contextmanager
def suppress_native_stdout():
    """Silences C/C++-level writes to stdout for the duration of the block
    -- e.g. OCCT's STEPControl_Writer (used by every export_step()/
    Gexport_step() in all 3 backends, including FreeCAD's own
    Part.Shape.exportStep(), which wraps the identical OCCT writer), which
    prints its own "Statistics on Transfer (Write)" banner directly via
    std::cout, unconditionally, with no verbosity/quiet flag exposed
    anywhere -- confirmed live (2026-08-27): none of Interface_Static's
    known parameter names ("write.step.verbosity", "write.verbosity",
    "write.step.trace", ...) exist, so there is no OCCT-side switch to
    flip instead.

    contextlib.redirect_stdout has no effect on this kind of write --
    it only reroutes Python's own sys.stdout object, not the OS file
    descriptor a native library's std::cout is bound to. This redirects
    the real file descriptor (fd 1) instead, so it silences a native
    library's own direct writes too, not just Python's print(). Restores
    the original fd unconditionally, even if the block raises.
    """
    sys.stdout.flush()
    saved_fd = os.dup(1)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull_fd, 1)
        yield
    finally:
        sys.stdout.flush()
        os.dup2(saved_fd, 1)
        os.close(devnull_fd)
        os.close(saved_fd)
