"""
GEOReverse/Modules/_geo_bridge.py

The single point of import for the rest of GEOReverse: `from ._geo_bridge
import GVector, GSolid, Gmake_cylinder, ...`. Never `import Part`/
`FreeCAD`/`BOPTools` outside this module -- mirrors the role
`geo/__init__.py` plays for GEOUNED, with one deliberate difference (see
below).

GEOReverse always needs the full FreeCAD implementation of `geo` and
never the pyOCC one -- CsgToCad has a hard dependency on FreeCAD's own
document format for `.FCStd` export (`core.py::export_cad`), so it can
never be pyOCC-only, and there is currently no plan to give it a pyOCC
backend at all. Importing through `geo/__init__.py` would risk silently
resolving against `geo._occ_impl`'s stubs whenever a process also has
`GEOUNED_CAD_ENGINE=occ` set (e.g. for GEOUNED's own forward pipeline,
in the same process). This module therefore imports directly from
`geo._freecad_impl`, bypassing `geo/__init__.py`'s engine switch
entirely -- a deliberate, narrow exception to "always go through the
package's single import point", justified because that rule exists to
keep GEOUNED's engine swappable, and GEOReverse is explicitly not.

GEOReverse represents its own affine transforms (MCNP TRn cards,
universe nesting) as plain numpy 4x4 arrays, not `geo`'s `GMatrix`
(which stays a passive data container, used only for GEOUNED's own
`MatrixOfInertia` bookkeeping) -- `to_fc_matrix`/`to_np_matrix` below
are the bridge between that representation and the native
`FreeCAD.Matrix` that `GSolid.transform_geometry`/`GPlane.transform`/
`GCylinder.transform`/etc. and `GBoundBox.transformed` all expect.
"""

import numpy as np
import FreeCAD
import Part

from ...geo._freecad_impl import (
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
    to_fc_vector,
)
from ...geo.vector_geometry import GBoundBox, GVector, to_gboundbox, to_gvector


def to_fc_matrix(matrix: np.ndarray) -> FreeCAD.Matrix:
    """4x4 row-major numpy array (GEOReverse's own transform
    representation) -> native FreeCAD.Matrix, for the handful of `geo`
    calls that need one (`GSolid.transform_geometry`, `GPlane.transform`
    and friends, `GBoundBox.transformed`)."""
    return FreeCAD.Matrix(*matrix.flatten().tolist())


def to_np_matrix(matrix: FreeCAD.Matrix) -> np.ndarray:
    """Native FreeCAD.Matrix -> 4x4 row-major numpy array."""
    return np.array(
        [
            [matrix.A11, matrix.A12, matrix.A13, matrix.A14],
            [matrix.A21, matrix.A22, matrix.A23, matrix.A24],
            [matrix.A31, matrix.A32, matrix.A33, matrix.A34],
            [matrix.A41, matrix.A42, matrix.A43, matrix.A44],
        ]
    )
