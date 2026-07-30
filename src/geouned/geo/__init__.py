"""
geo/__init__.py

Single point of import for the rest of GEOUNED: `from ...geo import
GSolid, Gmake_cylinder, ...`. Never `import Part`/`FreeCAD`/`BOPTools`
outside this package.

Today this re-exports `_freecad_impl`. Swapping geometry engines means
adding a sibling `_occ_impl.py` with the same names and changing the
import below -- the rest of GEOUNED does not change.
"""

from .vector_geometry import (
    GBoundBox,
    GLabelNode,
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
    to_gvector,
)
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
    Gfuse,
    Gin_contact,
    Gload_step,
    Gload_step_labels,
    Gmake_box,
    Gmake_compound,
    Gmake_cone,
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
    to_fc_vector,
)
