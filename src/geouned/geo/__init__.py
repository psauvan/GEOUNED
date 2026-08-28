"""
geo/__init__.py

Single point of import for the rest of GEOUNED: `from ...geo import
GSolid, Gmake_cylinder, ...`. Never `import Part`/`FreeCAD`/`BOPTools`
outside this package (and, symmetrically, never `import OCC`/`OCP`
outside this package either).

The geometry engine is chosen once, here, via the GEOUNED_CAD_ENGINE
environment variable ("ocp", the default -- pybind11-based, the binding
CadQuery/build123d use; "occ" for pythonocc-core, SWIG-based; "freecad"
for the original Part-API backend) -- read at import time because this
module is reached well before any Settings/CadToCsg object could exist
(see CLAUDE.md's pyOCC migration section for the full import-chain
trace). All three are complete, production implementations, developed
**in parallel** -- "occ"/"freecad" are not legacy/deprecated relative to
"ocp": all three are actively maintained, a fix found in one backend
should be ported to the others too. The default moved from "freecad" to
"ocp" once real-world use confirmed cases where FreeCAD's own bundled
OCCT (7.8.1) silently fails to split a solid (the project's original
motivating tangency bug) where OCP's newer OCCT (7.9.3) succeeds --
"freecad" remains a fully supported, explicit opt-in for anyone who
needs it (e.g. GEOReverse's own .FCStd export).
"""

import os

_engine = os.environ.get("GEOUNED_CAD_ENGINE", "ocp").strip().lower()

CAD_ENGINE = _engine
"""Public name for the resolved engine ("freecad", "occ", or "ocp") --
read by GEOReverse's own dispatch so its export follows the same
`GEOUNED_CAD_ENGINE` choice as GEOUNED's own forward pipeline."""

from .vector_geometry import (
    GBoundBox,
    GMatrix,
    GVector,
    to_gboundbox,
    to_gmatrix,
    to_gvector,
)
from .io_utils import GLabelNode
from .surface_geometry import (
    cylinder_tangent_at,
    cylinder_value_at,
    is_in_line,
    is_in_plane,
    is_opposite,
    is_parallel,
    is_parallel_plane_surface,
    is_same_cone_surface,
    is_same_cylinder_surface,
    is_same_plane_surface,
    is_same_sphere_surface,
    is_same_torus_surface,
    is_same_value,
    plane_tangent_at,
    plane_value_at,
    sign_plane,
)
from .solid_defects import (
    count_split_ring_pairs,
    find_short_edges,
    find_split_ring_faces,
    near_surface_pair,
)

if _engine == "occ":
    from ._occ_impl import (
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
        SplitResult,
        Gcheck_and_repair,
        Gclassify_curve,
        Gclassify_surface,
        Gcollapse_split_rings,
        Gcommon,
        Gcut,
        Gdefeature,
        Gdistance,
        Gexport_step,
        Gfirst_shell,
        Gfuse,
        Gin_contact,
        Gload_and_process_step,
        Gload_step,
        Gload_step_labels,
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
        Gsliver_heal,
        Gspline_surface,
        Gsplit,
        kernel_version,
        pick_outer_wire,
        to_native_matrix,
        to_native_vector,
    )
elif _engine == "ocp":
    from ._ocp_impl import (
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
        SplitResult,
        Gcheck_and_repair,
        Gclassify_curve,
        Gclassify_surface,
        Gcollapse_split_rings,
        Gcommon,
        Gcut,
        Gdefeature,
        Gdistance,
        Gexport_step,
        Gfirst_shell,
        Gfuse,
        Gin_contact,
        Gload_and_process_step,
        Gload_step,
        Gload_step_labels,
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
        Gsliver_heal,
        Gspline_surface,
        Gsplit,
        kernel_version,
        pick_outer_wire,
        to_native_matrix,
        to_native_vector,
    )
else:
    from ._freecad_impl import (
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
        SplitResult,
        Gcheck_and_repair,
        Gclassify_curve,
        Gclassify_surface,
        Gcollapse_split_rings,
        Gcommon,
        Gcut,
        Gdefeature,
        Gdistance,
        Gexport_step,
        Gfirst_shell,
        Gfuse,
        Gin_contact,
        Gload_and_process_step,
        Gload_step,
        Gload_step_labels,
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
        Gsliver_heal,
        Gspline_surface,
        Gsplit,
        kernel_version,
        pick_outer_wire,
        to_native_matrix,
        to_native_vector,
    )
