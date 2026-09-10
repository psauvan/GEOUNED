"""
geo/occ/__init__.py

pythonocc-core (pyOCC) implementation of the `geo` package -- a sibling
to `geo/freecad/`, selected instead of it by `geo/__init__.py` when
GEOUNED_CAD_ENGINE=occ. Mirrors `geo/freecad/`'s class/function names
and field names (matching FreeCAD's own attribute names, e.g.
GPlane.Position/.Axis/.XDir) so the rest of GEOUNED does not change
regardless of which backend is selected.

Split out of the former monolithic `_occ_impl.py` (2026-08-30), mirroring
the `geo/freecad/` split -- see CLAUDE.md for the full account.
`split.py`'s own orchestration (`Gsplit`/`_raw_bop_split`) is further
split from its own supporting cascades: `split_repair.py` (non-manifold/
phantom-cut repair of a raw BOPAlgo_Splitter result) and
`split_coaxial_cone.py` (the coaxial-cone/cylinder degeneracy fallback) --
both purely internal, not re-exported here. `split.py` and
`split_coaxial_cone.py` reference each other (Gsplit needs
`_try_coaxial_cone_split`; the coaxial-cone retry loop needs
`_raw_bop_split`) -- broken via one function-local (lazy) import inside
the retry loop, same technique `topology.py`'s own `.export_step()`
methods use for their circular need of `io.py`'s `_export_shapes_step`.

`GEOReverse` (CsgToCad) is out of scope for this whole package entirely
-- it keeps importing FreeCAD/Part directly, unconditionally, regardless
of GEOUNED_CAD_ENGINE (see geouned/__init__.py).

Known gaps (see CLAUDE.md's pyOCC migration section for the full
history): `Gclassify_surface`'s BSplineSurface-secretly-a-plane fallback
(FreeCAD's face.findPlane()) has no pyOCC port, so a mislabeled flat face
returns None here instead of a GPlane.
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
