"""
geo/__init__.py

Single point of import for the rest of GEOUNED: `from ...geo import
GSolid, Gmake_cylinder, ...`. Never `import OCC`/`OCP` outside this
package.

The geometry engine is chosen once, here, via the GEOUNED_CAD_ENGINE
environment variable ("ocp", the default -- pybind11-based, the binding
CadQuery/build123d use; "occ" for pythonocc-core, SWIG-based) -- read
at import time because this module is reached well before any
Settings/CadToCsg object could exist (see CLAUDE.md's pyOCC migration
section for the full import-chain trace). Both are complete,
production implementations, developed **in parallel** -- "occ" is not
legacy/deprecated relative to "ocp": both are actively maintained, a
fix found in one backend should be ported to the other too.

The original FreeCAD/`Part`-API backend ("freecad") was removed once
the "sewing composite surface" work made it clear there was no longer
any reason to keep a third engine around: FreeCAD's own bundled OCCT
(7.8.1) silently fails to split a solid in cases OCP's newer OCCT
(7.9.3)/pythonocc-core succeed (the project's original motivating
tangency bug), and every feature added after the migration (exotic
quadrics, spline-quadric substitution, sewn composite surfaces, ...)
had no FreeCAD equivalent to begin with.
"""

import os

_engine = os.environ.get("GEOUNED_CAD_ENGINE", "ocp").strip().lower()

if _engine not in ("occ", "ocp"):
    raise ValueError(f"GEOUNED_CAD_ENGINE must be 'occ' or 'ocp', got {_engine!r}")

CAD_ENGINE = _engine
"""Public name for the resolved engine ("occ" or "ocp") -- read by
GEOReverse's own dispatch so its export follows the same
`GEOUNED_CAD_ENGINE` choice as GEOUNED's own forward pipeline."""

from .vector_geometry import (
    GBoundBox,
    GMatrix,
    GVector,
    arbitrary_perpendicular,
    arc_extent,
    myBox,
    to_gboundbox,
    to_gmatrix,
    to_gvector,
)
from .io_utils import GLabelNode
from .surface_geometry import (
    cylinder_tangent_at,
    cylinder_value_at,
    is_parallel_plane_surface,
    is_same_cone_surface,
    is_same_cylinder_surface,
    is_same_plane_surface,
    is_same_sphere_surface,
    is_same_torus_surface,
    is_same_value,
    plane_tangent_at,
    plane_value_at,
    convex_planes,
)
from .solid_defects import (
    count_split_ring_pairs,
    find_overlapping_coplanar_faces,
    find_short_edges,
    find_split_ring_faces,
    near_surface_pair,
)
from .tolerances import GeoTolerances
from .volume_utils import volume_within
from .solid_ops import (
    BuildDepth,
    BuildSolidParts,
    GCompound,
    Gfuse_solids,
    SplitBase,
    SplitSolid,
    filterparts,
    getPart,
    is_sliver_solid,
    joinBase,
    merge_sliver_pieces,
    space_decomposition,
)

if _engine == "occ":
    from .occ import (
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
        SplitResult,
        Gcheck_and_repair,
        Gclassify_curve,
        Gclassify_surface,
        Gclose_open_solid,
        Gcollapse_split_rings,
        Gcommon,
        Gcut,
        Gdefeature,
        Gdiagnose_open_solid,
        Gdistance,
        Gexport_binary,
        Gexport_step,
        Gface_valid,
        Gfirst_shell,
        Gfuse,
        Gheal_topology,
        Gin_contact,
        Gload_and_process_step,
        Gload_binary,
        Gload_step,
        Gload_step_labels,
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
        Gmake_sewn_multiplane_chain,
        Gmake_sewn_plane_pair,
        Gmake_sewn_round_corner_cylinder,
        Gmake_sewn_tcone,
        Gmake_sphere,
        Gmake_torus,
        Gmake_torus_elliptic,
        Gmake_wire,
        Gmerge_coplanar_planes,
        Gsliver_heal,
        Gsolid_max_tolerance,
        Gsolid_nonmanifold_edge_count,
        Gsolid_set_tolerance,
        Gspline_surface,
        Gsplit,
        Gsubstitute_spline_quadrics,
        Gtrim_overlapping_coplanar_faces,
        kernel_version,
        pick_outer_wire,
        to_native_matrix,
        to_native_vector,
    )
else:
    from .ocp import (
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
        SplitResult,
        Gcheck_and_repair,
        Gclassify_curve,
        Gclassify_surface,
        Gclose_open_solid,
        Gcollapse_split_rings,
        Gcommon,
        Gcut,
        Gdefeature,
        Gdiagnose_open_solid,
        Gdistance,
        Gexport_binary,
        Gexport_step,
        Gface_valid,
        Gfirst_shell,
        Gfuse,
        Gheal_topology,
        Gin_contact,
        Gload_and_process_step,
        Gload_binary,
        Gload_step,
        Gload_step_labels,
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
        Gmake_sewn_multiplane_chain,
        Gmake_sewn_plane_pair,
        Gmake_sewn_round_corner_cylinder,
        Gmake_sewn_tcone,
        Gmake_sphere,
        Gmake_torus,
        Gmake_torus_elliptic,
        Gmake_wire,
        Gmerge_coplanar_planes,
        Gsliver_heal,
        Gsolid_max_tolerance,
        Gsolid_nonmanifold_edge_count,
        Gsolid_set_tolerance,
        Gspline_surface,
        Gsplit,
        Gsubstitute_spline_quadrics,
        Gtrim_overlapping_coplanar_faces,
        kernel_version,
        pick_outer_wire,
        to_native_matrix,
        to_native_vector,
    )
