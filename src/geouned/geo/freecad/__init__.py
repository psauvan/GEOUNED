"""
geo/freecad/__init__.py

FreeCAD/Part implementation of the `geo` package -- the ONLY subpackage
in GEOUNED allowed to `import Part`/`FreeCAD`/`BOPTools` (spread across
its own sibling modules, grouped by function: `_native_utils.py`,
`topology.py`, `repair.py`, `io.py`, `primitives.py`, `boolean.py`,
`split.py`, `queries.py`). Everything else imports the GSolid/GFace/
.../Gmake_*/Gsplit/... names from `geo` (re-exported by `geo/__init__.py`
via this module), never Part/FreeCAD directly.

Split out of the former monolithic `_freecad_impl.py` (2026-08-30),
mirroring the earlier `vector_geometry.py` split -- see CLAUDE.md for the
full account, including why `topology.py` stays one file (the analytic
surface/curve descriptors and the neutral topology classes are genuinely
mutually recursive) while everything else split cleanly (one-directional
consumers of `topology.py`, never referencing each other).

Known limitation: `Gsplit()` here only ports GEOUNED's existing
tolerance-scaling retry, which handles the kernel raising an exception at
very small tolerances. It does NOT solve the silent-uncut-solid tangency
bug described in the project's motivating problem (a plane's intersection
with a solid coinciding with a pre-existing tangency line) -- today
GEOUNED only works around that case via a STEP export/import round-trip
elsewhere in the pipeline. The proper fix (face-adjacency graph excluding
non-manifold edges, reconstructing solids per connected component) is
solved by the `occ`/`ocp` engines instead.
"""

from __future__ import annotations

from ._native_utils import kernel_version, to_native_matrix, to_native_vector
from .topology import (
    GBSpline,
    GCircle,
    GCone,
    GCylinder,
    GEdge,
    GEllipse,
    GFace,
    GLine,
    GPlane,
    GShape,
    GShell,
    GSolid,
    GSphere,
    GTorus,
    GWire,
    Gclassify_curve,
    Gclassify_surface,
    pick_outer_wire,
)
from .repair import (
    Gcheck_and_repair,
    Gclose_open_solid,
    Gcollapse_split_rings,
    Gdefeature,
    Gdiagnose_open_solid,
    Gface_valid,
    Gheal_topology,
    Gmerge_coplanar_planes,
    Gsliver_heal,
    Gspline_surface,
)
from .io import Gexport_step, Gfirst_shell, Gload_and_process_step, Gload_step, Gload_step_labels
from .primitives import (
    Gmake_box,
    Gmake_compound,
    Gmake_cone,
    Gmake_cone_double_sheet,
    Gmake_cone_frustum,
    Gmake_cylinder,
    Gmake_half_space,
    Gmake_polygon_face,
    Gmake_shell,
    Gmake_solid,
    Gmake_sphere,
    Gmake_torus,
    Gmake_wire,
)
from .boolean import Gcommon, Gcut, Gfuse
from .split import SplitResult, Gsplit
from .queries import Gdistance, Gin_contact, Gsolid_max_tolerance, Gsolid_nonmanifold_edge_count
