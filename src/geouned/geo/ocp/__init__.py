"""
geo/ocp/__init__.py

OCP (pybind11-based, the binding CadQuery/build123d use) implementation
of the `geo` package -- the default engine (see `geo/__init__.py`), a
sibling to `geo/occ/` (pythonocc-core/SWIG) and `geo/freecad/`. Mirrors
their class/function names and field names (matching FreeCAD's own
attribute names, e.g. GPlane.Position/.Axis/.XDir) so the rest of
GEOUNED does not change regardless of which backend is selected.

Split out of the former monolithic `_ocp_impl.py` (2026-08-30), mirroring
the `geo/occ/` split -- see CLAUDE.md for the full account, including the
2 genuine circular dependencies (split.py<->split_coaxial_cone.py,
topology.py<->io.py) broken with function-local (lazy) imports.

`GEOReverse` (CsgToCad) is out of scope for this whole package entirely
-- it keeps importing FreeCAD/Part directly, unconditionally, regardless
of GEOUNED_CAD_ENGINE (see geouned/__init__.py).

`spline_quadrics.py`'s `Gsubstitute_spline_quadrics` (2026-09-18) lets a
BSplineSurface face that's secretly a cylinder/sphere/torus be recovered
instead of dropping the whole solid -- called from `Gload_and_process_
step`, not part of `Gclassify_surface` itself. Cone is deliberately
excluded (see that module's own docstring and CLAUDE.md's "Spline-vs-
quadric identification" entry for why).

`GEllipticCylinder` (2026-09-18, same entry) is a 6th BASE analytic
surface type, recognized directly by `Gclassify_surface` from an exact
(never fitted) `GeomAbs_SurfaceOfExtrusion` of a `Geom_Ellipse` -- the
first of GEOReverse's own "exotic quadric" surfaces GEOUNED's forward
pipeline now supports too, per direct user instruction (deliberately
NOT combined into composite meta-surfaces like RoundCorner/Can yet).
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
    GEllipticCylinder,
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
from .io import Gexport_binary, Gexport_step, Gfirst_shell, Gload_and_process_step, Gload_binary, Gload_step, Gload_step_labels
from .spline_quadrics import Gsubstitute_spline_quadrics
from .primitives import (
    Gmake_box,
    Gmake_compound,
    Gmake_cone,
    Gmake_cone_double_sheet,
    Gmake_cone_frustum,
    Gmake_cylinder,
    Gmake_elliptic_cylinder,
    Gmake_half_space,
    Gmake_polygon_face,
    Gmake_shell,
    Gmake_solid,
    Gmake_sphere,
    Gmake_torus,
    Gmake_torus_elliptic,
    Gmake_wire,
)
from .boolean import Gcommon, Gcut, Gfuse
from .split import SplitResult, Gsplit
from .queries import Gdistance, Gin_contact, Gsolid_max_tolerance, Gsolid_nonmanifold_edge_count
