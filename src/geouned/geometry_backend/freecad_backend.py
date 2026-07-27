"""
geometry_backend/freecad_backend.py

FreeCAD/Part implementation of the `GeometryBackend` interface. This is
the first concrete backend, used to validate that `GeometryBackend`
actually covers what GEOUNED's decomposition/meta-surface code needs
before `OCCBackend` is written.

Known limitation: `split()` here only ports GEOUNED's existing
tolerance-scaling retry (`utils/split_function.py`), which handles the
kernel raising an exception at very small tolerances. It does NOT solve
the silent-uncut-solid tangency bug described in the project's
motivating problem (a plane's intersection with a solid coinciding with
a pre-existing tangency line) -- today GEOUNED only works around that
case via a STEP export/import round-trip elsewhere in the pipeline. The
proper fix (face-adjacency graph excluding non-manifold edges,
reconstructing solids per connected component) is reserved for
`OCCBackend`, per the migration plan.
"""

from __future__ import annotations

import math
from typing import Sequence

import BOPTools.SplitAPI
import FreeCAD
import Part

from .geometry_backend_interface import (
    BSplineParams,
    CircleParams,
    ConeParams,
    CurveType,
    CylinderParams,
    EdgeGeometry,
    EllipseParams,
    GEdge,
    GeometryBackend,
    GFace,
    GShape,
    GSolid,
    GVector,
    GVertex,
    GWire,
    LineParams,
    PlaneParams,
    SphereParams,
    SplitResult,
    SurfaceGeometry,
    SurfaceType,
    TorusParams,
)


def _to_gvector(vector: FreeCAD.Vector) -> GVector:
    return GVector(vector.x, vector.y, vector.z)


def _to_fc_vector(vector: GVector) -> FreeCAD.Vector:
    return FreeCAD.Vector(vector.x, vector.y, vector.z)


class FreeCADBackend(GeometryBackend):
    """GeometryBackend implementation on top of FreeCAD's Part API."""

    def _wrap_solid(self, shape: Part.Shape) -> GSolid:
        return GSolid(native=shape, backend=self)

    def _wrap_face(self, face: Part.Face) -> GFace:
        return GFace(native=face, backend=self)

    def _wrap_wire(self, wire: Part.Wire) -> GWire:
        return GWire(native=wire, backend=self)

    def _wrap_edge(self, edge: Part.Edge) -> GEdge:
        return GEdge(native=edge, backend=self)

    def _wrap_vertex(self, vertex: Part.Vertex) -> GVertex:
        return GVertex(native=vertex, backend=self)

    # -- I/O ------------------------------------------------------------

    def load_step(self, filename: str) -> list[GSolid]:
        shape = Part.Shape()
        shape.read(filename)
        return [self._wrap_solid(solid) for solid in shape.Solids]

    def export_step(self, solids: Sequence[GSolid], filename: str) -> None:
        compound = Part.makeCompound([solid.native for solid in solids])
        compound.exportStep(filename)

    # -- Primitive construction -------------------------------------------

    def make_box(
        self, xmin: float, ymin: float, zmin: float,
        xmax: float, ymax: float, zmax: float,
    ) -> GSolid:
        box = Part.makeBox(
            xmax - xmin, ymax - ymin, zmax - zmin,
            FreeCAD.Vector(xmin, ymin, zmin),
        )
        return self._wrap_solid(box)

    def make_cylinder(
        self, base_point: GVector, axis_dir: GVector,
        radius: float, height: float,
    ) -> GSolid:
        cylinder = Part.makeCylinder(
            radius, height, _to_fc_vector(base_point), _to_fc_vector(axis_dir),
        )
        return self._wrap_solid(cylinder)

    def make_cone(
        self, apex: GVector, axis_dir: GVector,
        half_angle: float, height: float,
    ) -> GSolid:
        base_radius = height * math.tan(abs(half_angle))
        cone = Part.makeCone(
            0.0, base_radius, height, _to_fc_vector(apex), _to_fc_vector(axis_dir),
        )
        return self._wrap_solid(cone)

    def make_sphere(self, center: GVector, radius: float) -> GSolid:
        sphere = Part.makeSphere(radius, _to_fc_vector(center))
        return self._wrap_solid(sphere)

    def make_torus(
        self, center: GVector, axis_dir: GVector,
        major_radius: float, minor_radius: float,
    ) -> GSolid:
        torus = Part.makeTorus(
            major_radius, minor_radius, _to_fc_vector(center), _to_fc_vector(axis_dir),
        )
        return self._wrap_solid(torus)

    def make_half_space(self, plane: PlaneParams) -> GSolid:
        extent = 1.0e6
        box = Part.makeBox(
            extent, extent, extent,
            FreeCAD.Vector(-extent / 2.0, -extent / 2.0, -extent),
        )
        normal = _to_fc_vector(plane.normal.normalized())
        box.Placement = FreeCAD.Placement(
            _to_fc_vector(plane.point),
            FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), normal),
        )
        return self._wrap_solid(box)

    def make_wire(self, edges: Sequence[GEdge]) -> GWire:
        wire = Part.Wire([edge.native for edge in edges])
        return self._wrap_wire(wire)

    # -- Boolean operations -------------------------------------------------

    def cut(self, solid: GSolid, tools: Sequence[GSolid]) -> list[GSolid]:
        result = solid.native.cut([tool.native for tool in tools])
        return [self._wrap_solid(s) for s in result.Solids]

    def common(self, solid: GSolid, tools: Sequence[GSolid]) -> list[GSolid]:
        result = solid.native.common([tool.native for tool in tools])
        return [self._wrap_solid(s) for s in result.Solids]

    def fuse(self, solids: Sequence[GSolid]) -> GSolid:
        shapes = [solid.native for solid in solids]
        fused = shapes[0].fuse(shapes[1:]) if len(shapes) > 1 else shapes[0]
        return self._wrap_solid(fused)

    def split(
        self, solid: GSolid, tool: GFace | GSolid, tolerance: float, scale: float = 0.1,
    ) -> SplitResult:
        tools = [tool.native]
        try:
            compound = BOPTools.SplitAPI.slice(solid.native, tools, "Split", tolerance=tolerance)
        except Exception:
            if tolerance < 1e-12:
                raise
            retried = self.split(solid, tool, tolerance * scale, scale)
            return SplitResult(
                solids=retried.solids,
                degenerate_case_handled=True,
                notes=f"retried at tolerance={tolerance * scale}",
            )
        return SplitResult(solids=[self._wrap_solid(s) for s in compound.Solids])

    # -- Topological traversal --------------------------------------------

    def get_faces(self, solid: GSolid) -> list[GFace]:
        return [self._wrap_face(f) for f in solid.native.Faces]

    def get_edges(self, face: GFace) -> list[GEdge]:
        return [self._wrap_edge(e) for e in face.native.Edges]

    def get_outer_wire(self, face: GFace) -> GWire:
        return self._wrap_wire(face.native.OuterWire)

    def get_wire_edges(self, wire: GWire) -> list[GEdge]:
        return [self._wrap_edge(e) for e in wire.native.OrderedEdges]

    def get_vertices(self, edge: GEdge) -> list[GVertex]:
        return [self._wrap_vertex(v) for v in edge.native.Vertexes]

    def get_vertex_point(self, vertex: GVertex) -> GVector:
        return _to_gvector(vertex.native.Point)

    def faces_sharing_edge(self, solid: GSolid, edge: GEdge) -> list[GFace]:
        matches = []
        for face in solid.native.Faces:
            for candidate in face.Edges:
                if candidate.isSame(edge.native):
                    matches.append(face)
                    break
        return [self._wrap_face(f) for f in matches]

    # -- Surface and curve classification -----------------------------

    def classify_surface(self, face: GFace) -> SurfaceGeometry:
        surface = face.native.Surface
        kind = type(surface)
        if kind is Part.Plane:
            return SurfaceGeometry(
                SurfaceType.PLANE,
                PlaneParams(_to_gvector(surface.Position), _to_gvector(surface.Axis)),
            )
        if kind is Part.Cylinder:
            return SurfaceGeometry(
                SurfaceType.CYLINDER,
                CylinderParams(_to_gvector(surface.Center), _to_gvector(surface.Axis), surface.Radius),
            )
        if kind is Part.Cone:
            return SurfaceGeometry(
                SurfaceType.CONE,
                ConeParams(_to_gvector(surface.Apex), _to_gvector(surface.Axis), surface.SemiAngle),
            )
        if kind is Part.Sphere:
            return SurfaceGeometry(
                SurfaceType.SPHERE,
                SphereParams(_to_gvector(surface.Center), surface.Radius),
            )
        if kind is Part.Toroid:
            return SurfaceGeometry(
                SurfaceType.TORUS,
                TorusParams(
                    _to_gvector(surface.Center), _to_gvector(surface.Axis),
                    surface.MajorRadius, surface.MinorRadius,
                ),
            )
        return SurfaceGeometry(SurfaceType.UNKNOWN, None)

    def classify_edge(self, edge: GEdge) -> EdgeGeometry:
        curve = edge.native.Curve
        kind = type(curve)
        if kind is Part.Line:
            return EdgeGeometry(
                CurveType.LINE,
                LineParams(_to_gvector(curve.Location), _to_gvector(curve.Direction)),
            )
        if kind is Part.Circle:
            return EdgeGeometry(
                CurveType.CIRCLE,
                CircleParams(_to_gvector(curve.Center), _to_gvector(curve.Axis), curve.Radius),
            )
        if kind is Part.Ellipse:
            return EdgeGeometry(
                CurveType.ELLIPSE,
                EllipseParams(
                    _to_gvector(curve.Center), _to_gvector(curve.Axis), _to_gvector(curve.XAxis),
                    curve.MajorRadius, curve.MinorRadius,
                ),
            )
        if kind is Part.BSplineCurve:
            return EdgeGeometry(
                CurveType.BSPLINE,
                BSplineParams([_to_gvector(pole) for pole in curve.getPoles()]),
            )
        return EdgeGeometry(CurveType.UNKNOWN, None)

    def face_orientation_outward(self, solid: GSolid, face: GFace) -> bool:
        u_min, u_max, v_min, v_max = face.native.ParameterRange
        u = (u_min + u_max) / 2.0
        v = (v_min + v_max) / 2.0
        point = face.native.valueAt(u, v)
        normal = face.native.normalAt(u, v)
        probe = point + normal * 1e-6
        return not solid.native.isInside(probe, 1e-7, False)

    # -- Face parametric queries ---------------------------------------------

    def parameter_range(self, face: GFace) -> tuple[float, float, float, float]:
        return face.native.ParameterRange

    def face_value_at(self, face: GFace, u: float, v: float) -> GVector:
        return _to_gvector(face.native.valueAt(u, v))

    def face_normal_at(self, face: GFace, u: float, v: float) -> GVector:
        return _to_gvector(face.native.normalAt(u, v))

    def tessellate(self, face: GFace, tolerance: float) -> list[GVector]:
        vertices, _facets = face.native.tessellate(tolerance)
        return [_to_gvector(v) for v in vertices]

    # -- Edge parametric queries -----------------------------------------

    def edge_parameter_range(self, edge: GEdge) -> tuple[float, float]:
        return edge.native.ParameterRange

    def edge_value_at(self, edge: GEdge, u: float) -> GVector:
        return _to_gvector(edge.native.valueAt(u))

    # -- Geometric properties -------------------------------------------------

    def volume(self, solid: GSolid) -> float:
        return solid.native.Volume

    def area(self, face: GFace) -> float:
        return face.native.Area

    def bounding_box(
        self, solid: GSolid,
    ) -> tuple[float, float, float, float, float, float]:
        box = solid.native.BoundBox
        return (box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)

    def center_of_mass(self, solid: GSolid) -> GVector:
        return _to_gvector(solid.native.CenterOfMass)

    # -- Spatial queries -------------------------------------------------------

    def is_inside(self, solid: GSolid, point: GVector, tolerance: float) -> bool:
        return solid.native.isInside(_to_fc_vector(point), tolerance, False)

    def in_contact(self, shape_a: GShape, shape_b: GShape, tolerance: float) -> bool:
        native_a = shape_a.native
        native_b = shape_b.native

        box_intersection = native_a.BoundBox.intersected(native_b.BoundBox)
        if not (
            box_intersection.XLength > -tolerance
            and box_intersection.YLength > -tolerance
            and box_intersection.ZLength > -tolerance
        ):
            return False

        try:
            return native_a.distToShape(native_b)[0] < tolerance
        except Exception:
            pass

        if hasattr(native_a, "Volume") and hasattr(native_b, "Volume"):
            common = native_a.common(native_b)
            return abs(common.Volume) > 1e-8 or bool(common.Solids) or bool(common.Faces) or bool(common.Edges)
        return False

    # -- Validation / diagnostics --------------------------------------------

    def is_valid(self, solid: GSolid) -> bool:
        return solid.native.isValid()

    def fix_shape(self, solid: GSolid, tolerance: float) -> GSolid:
        shape = solid.native.removeSplitter()
        if not shape.isValid():
            shape = shape.copy()
            shape.fix(tolerance, tolerance, tolerance)
        return self._wrap_solid(shape)

    # -- Transformations -------------------------------------------------------

    def translate(self, solid: GSolid, vector: GVector) -> GSolid:
        shape = solid.native.copy()
        shape.translate(_to_fc_vector(vector))
        return self._wrap_solid(shape)

    def rotate(
        self, solid: GSolid, axis_point: GVector, axis_dir: GVector,
        angle_rad: float,
    ) -> GSolid:
        shape = solid.native.copy()
        shape.rotate(_to_fc_vector(axis_point), _to_fc_vector(axis_dir), math.degrees(angle_rad))
        return self._wrap_solid(shape)
