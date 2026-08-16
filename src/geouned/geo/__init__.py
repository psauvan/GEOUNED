"""
geo/__init__.py

Single point of import for the rest of GEOUNED: `from ...geo import
GSolid, Gmake_cylinder, ...`. Never `import Part`/`FreeCAD`/`BOPTools`
outside this package (and, symmetrically, never `import OCC`/`OCP`
outside this package either).

The geometry engine is chosen once, here, via the GEOUNED_CAD_ENGINE
environment variable ("freecad", the default; "occ" for pythonocc-core;
"ocp" for OCP) -- read at import time because this module is reached
well before any Settings/CadToCsg object could exist (see CLAUDE.md's
pyOCC migration section for the full import-chain trace). All three are
complete, production implementations. "occ" (pythonocc-core, SWIG-based)
is kept for backward compatibility; "ocp" (pybind11-based, the binding
CadQuery/build123d use) is the recommended choice for new pyOCC-backed
work -- a live benchmark found pythonocc-core's SWIG bindings cost
measurably more per native call than OCP's (see CLAUDE.md's OCP
evaluation section).
"""

import os

_engine = os.environ.get("GEOUNED_CAD_ENGINE", "freecad").strip().lower()

CAD_ENGINE = _engine
"""Public name for the resolved engine ("freecad", "occ", or "ocp") --
read by GEOReverse's own dispatch so its export follows the same
`GEOUNED_CAD_ENGINE` choice as GEOUNED's own forward pipeline."""

from .vector_geometry import (
    GBoundBox,
    GLabelNode,
    GMatrix,
    GVector,
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
    to_gboundbox,
    to_gmatrix,
    to_gvector,
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
        Gclassify_curve,
        Gclassify_surface,
        Gcommon,
        Gcut,
        Gdistance,
        Gexport_step,
        Gfirst_shell,
        Gfuse,
        Gin_contact,
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
        Gmake_sphere,
        Gmake_torus,
        Gmake_wire,
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
        Gclassify_curve,
        Gclassify_surface,
        Gcommon,
        Gcut,
        Gdistance,
        Gexport_step,
        Gfirst_shell,
        Gfuse,
        Gin_contact,
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
        Gmake_sphere,
        Gmake_torus,
        Gmake_wire,
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
        Gclassify_curve,
        Gclassify_surface,
        Gcommon,
        Gcut,
        Gdistance,
        Gexport_step,
        Gfirst_shell,
        Gfuse,
        Gin_contact,
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
        Gmake_sphere,
        Gmake_torus,
        Gmake_wire,
        Gsplit,
        kernel_version,
        pick_outer_wire,
        to_native_matrix,
        to_native_vector,
    )
