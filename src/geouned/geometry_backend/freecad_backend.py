"""
geometry_backend/freecad_backend.py

FreeCAD/Part implementation of the `GeometryBackend` interface. This is
the first concrete backend, used to validate that `GeometryBackend`
actually covers what GEOUNED's decomposition/meta-surface code needs
before `OCCBackend` is written.

Known limitation: `split()` here only ports GEOUNED's existing
tolerance-scaling retry (formerly `utils/split_function.py` and
`build_region/splitFunction.py::SplitSolid`, both now folded in here),
which handles the kernel raising an exception at very small tolerances.
It does NOT solve the silent-uncut-solid tangency bug described in the project's
motivating problem (a plane's intersection with a solid coinciding with
a pre-existing tangency line) -- today GEOUNED only works around that
case via a STEP export/import round-trip elsewhere in the pipeline. The
proper fix (face-adjacency graph excluding non-manifold edges,
reconstructing solids per connected component) is reserved for
`OCCBackend`, per the migration plan.
"""

from __future__ import annotations

import math
import uuid
from typing import Sequence

import BOPTools.SplitAPI
import FreeCAD
import Part
from FreeCAD import Import

from .vector_geometry import to_gboundbox, to_gvector
from .geometry_backend_interface import (
    GBoundBox,
    GBSpline,
    GCircle,
    GCone,
    GCurve,
    GCylinder,
    GEdge,
    GeometryBackend,
    GEllipse,
    GFace,
    GLabelNode,
    GLine,
    GPlane,
    GShape,
    GShell,
    GSolid,
    GSphere,
    GSurface,
    GTorus,
    GVector,
    GVertex,
    GWire,
    SplitResult,
)


def to_fc_vector(vector: GVector) -> FreeCAD.Vector:
    """Write-side half of the transitional pair with `vector_geometry.to_gvector` --
    materializes a neutral GVector back into a native FreeCAD.Vector, needed only
    where GEOUNED still calls a native Part/FreeCAD function directly."""
    return FreeCAD.Vector(vector.x, vector.y, vector.z)


class FreeCADBackend(GeometryBackend):
    """GeometryBackend implementation on top of FreeCAD's Part API."""

    def _wrap_solid(self, shape: Part.Shape) -> GSolid:
        return GSolid(native=shape, backend=self, Orientation=shape.Orientation, BoundBox=to_gboundbox(shape.BoundBox))

    def _wrap_wire(self, wire: Part.Wire) -> GWire:
        return GWire(native=wire, backend=self)

    def _build_gvertex(self, vertex: Part.Vertex) -> GVertex:
        return GVertex(native=vertex, backend=self, Point=to_gvector(vertex.Point))

    def _build_gedge(self, edge: Part.Edge) -> GEdge:
        return GEdge(
            native=edge,
            backend=self,
            Curve=self._classify_native_curve(edge),
            Vertexes=tuple(self._build_gvertex(v) for v in edge.Vertexes),
            ParameterRange=edge.ParameterRange,
            Orientation=edge.Orientation,
        )

    def _build_gface(self, face: Part.Face) -> GFace:
        return GFace(
            native=face,
            backend=self,
            Surface=self._classify_native_surface(face),
            Edges=tuple(self._build_gedge(e) for e in face.Edges),
            OuterWire=self._wrap_wire(self._native_outer_wire(face)),
            ParameterRange=face.ParameterRange,
            Orientation=face.Orientation,
        )

    def _native_outer_wire(self, face: Part.Face) -> Part.Wire:
        """
        GEOUNED's own heuristic (largest mean vertex-to-centroid distance
        among the face's wires), not FreeCAD's native `Face.OuterWire` --
        the native attribute picks the wrong wire for some faces.
        """
        wires = face.Wires
        if len(wires) == 1:
            return wires[0]
        best_wire = None
        best_extension = 0.0
        for wire in wires:
            vertices = wire.OrderedVertexes
            center = wire.CenterOfMass
            extension = sum((v.Point - center).Length for v in vertices) / len(vertices)
            if extension > best_extension:
                best_extension = extension
                best_wire = wire
        return best_wire

    def _classify_native_surface(self, face: Part.Face) -> GSurface:
        surface = face.Surface
        kind = type(surface)
        if kind is Part.Plane:
            x_dir = to_gvector(surface.Rotation.multVec(FreeCAD.Vector(1, 0, 0)))
            return GPlane(to_gvector(surface.Position), to_gvector(surface.Axis), x_dir)
        if kind is Part.Cylinder:
            x_dir = to_gvector(surface.Rotation.multVec(FreeCAD.Vector(1, 0, 0)))
            return GCylinder(to_gvector(surface.Center), to_gvector(surface.Axis), surface.Radius, x_dir)
        if kind is Part.Cone:
            return GCone(to_gvector(surface.Apex), to_gvector(surface.Axis), surface.SemiAngle, surface.Radius)
        if kind is Part.Sphere:
            return GSphere(to_gvector(surface.Center), surface.Radius)
        if kind is Part.Toroid:
            return GTorus(
                to_gvector(surface.Center), to_gvector(surface.Axis),
                surface.MajorRadius, surface.MinorRadius,
            )
        if kind is Part.BSplineSurface:
            # Transport codes (MCNP/OpenMC/...) don't support BSpline surfaces
            # at all -- the only acceptable case is a BSplineSurface that is
            # geometrically just a mislabeled plane (some CAD exports do
            # this for flat faces). If it isn't even that, this must be a
            # hard failure, not a silently-skipped face.
            plane = face.findPlane()
            if plane is not None:
                x_dir = to_gvector(plane.Rotation.multVec(FreeCAD.Vector(1, 0, 0)))
                return GPlane(to_gvector(plane.Position), to_gvector(plane.Axis), x_dir)
            raise ValueError("BSplineSurface is not planar -- unsupported surface type for CSG conversion")
        raise ValueError(f"Unsupported surface type: {kind}")

    def _classify_native_curve(self, edge: Part.Edge) -> GCurve:
        curve = edge.Curve
        kind = type(curve)
        if kind is Part.Line:
            return GLine(to_gvector(curve.Location), to_gvector(curve.Direction))
        if kind is Part.Circle:
            return GCircle(to_gvector(curve.Center), to_gvector(curve.Axis), curve.Radius)
        if kind is Part.Ellipse:
            return GEllipse(
                to_gvector(curve.Center), to_gvector(curve.Axis), to_gvector(curve.XAxis),
                curve.MajorRadius, curve.MinorRadius,
            )
        if kind is Part.BSplineCurve:
            return GBSpline([to_gvector(pole) for pole in curve.getPoles()])
        raise ValueError(f"Unsupported curve type: {kind}")

    # -- Metadata ---------------------------------------------------------

    def kernel_version(self) -> str:
        return "{V[0]}.{V[1]}.{V[2]}".format(V=FreeCAD.Version())

    # -- I/O ------------------------------------------------------------

    def load_step(self, filename: str) -> list[GSolid]:
        shape = Part.Shape()
        shape.read(filename)
        return [self._wrap_solid(solid) for solid in shape.Solids]

    def load_step_labels(self, filename: str) -> list[GLabelNode]:
        # a throwaway document, not `load_step`'s own read: Import.insert
        # builds FreeCAD's Part::Feature/Label/InList tree, which is what
        # carries the assembly labels -- Part.Shape.read (used by
        # load_step) only returns geometry, already placed, with no label
        # information at all.
        doc = FreeCAD.newDocument(uuid.uuid4().hex)
        try:
            Import.insert(filename, doc.Name)

            nodes: dict[str, GLabelNode] = {}

            def build_node(elem) -> GLabelNode:
                if elem.Name in nodes:
                    return nodes[elem.Name]
                parent = build_node(elem.InList[0]) if elem.InList else None
                n_solids = 0
                if elem.TypeId == "Part::Feature" and elem.Shape.Solids:
                    n_solids = len(elem.Shape.Solids)
                node = GLabelNode(label=elem.Label, parent=parent, n_solids=n_solids)
                nodes[elem.Name] = node
                return node

            return [
                build_node(elem) for elem in doc.Objects
                if elem.TypeId == "Part::Feature" and elem.Shape.Solids
            ]
        finally:
            FreeCAD.closeDocument(doc.Name)

    def export_step(self, shapes: Sequence[GShape], filename: str) -> None:
        compound = Part.makeCompound([shape.native for shape in shapes])
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
            radius, height, to_fc_vector(base_point), to_fc_vector(axis_dir),
        )
        return self._wrap_solid(cylinder)

    def make_cone(
        self, apex: GVector, axis_dir: GVector,
        half_angle: float, height: float,
    ) -> GSolid:
        base_radius = height * math.tan(abs(half_angle))
        cone = Part.makeCone(
            0.0, base_radius, height, to_fc_vector(apex), to_fc_vector(axis_dir),
        )
        return self._wrap_solid(cone)

    def make_sphere(self, center: GVector, radius: float) -> GSolid:
        sphere = Part.makeSphere(radius, to_fc_vector(center))
        return self._wrap_solid(sphere)

    def make_torus(
        self, center: GVector, axis_dir: GVector,
        major_radius: float, minor_radius: float,
    ) -> GSolid:
        torus = Part.makeTorus(
            major_radius, minor_radius, to_fc_vector(center), to_fc_vector(axis_dir),
        )
        return self._wrap_solid(torus)

    def make_half_space(self, plane: GPlane) -> GSolid:
        extent = 1.0e6
        box = Part.makeBox(
            extent, extent, extent,
            FreeCAD.Vector(-extent / 2.0, -extent / 2.0, -extent),
        )
        normal = to_fc_vector(plane.Axis.normalized())
        box.Placement = FreeCAD.Placement(
            to_fc_vector(plane.Position),
            FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), normal),
        )
        return self._wrap_solid(box)

    def make_wire(self, edges: Sequence[GEdge]) -> GWire:
        wire = Part.Wire([edge.native for edge in edges])
        return self._wrap_wire(wire)

    def make_polygon_face(self, points: Sequence[GVector]) -> GFace:
        face = Part.Face(Part.makePolygon([to_fc_vector(p) for p in points], True))
        return self._build_gface(face)

    def make_shell(self, faces: Sequence[GFace]) -> GShell:
        shell = Part.makeShell([face.native for face in faces])
        return GShell(native=shell, backend=self, Faces=tuple(faces), Orientation=shell.Orientation)

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

    def make_compound(self, shapes: Sequence[GSolid]) -> GSolid:
        return self._wrap_solid(Part.makeCompound([s.native for s in shapes]))

    def reverse(self, solid: GSolid) -> GSolid:
        reversed_shape = solid.native.copy()
        reversed_shape.reverse()
        return self._wrap_solid(reversed_shape)

    def refine(self, solid: GSolid) -> GSolid:
        return self._wrap_solid(solid.native.removeSplitter())

    def split(
        self, solid: GSolid, tool: GFace | GSolid, tolerance: float,
        scale: float = 0.1, scale_up_floor: float | None = None,
    ) -> SplitResult:
        """
        `scale_up_floor` is a FreeCAD-backend-specific extension (not part
        of the abstract signature), mirroring GEOUNED's public
        `Options.scaleUp`/`Options.splitTolerance`: when `tolerance` drops
        below 1e-12 and `scale_up_floor` is given, retry upward starting
        from that floor instead of just attempting the tiny tolerance
        as-is. Below 1e-12 with no floor, and at `tolerance >= 0.1`, there
        is no retry at all -- those are the two cases where shrinking
        further is not expected to help.
        """
        tools = [tool.native]

        if tolerance >= 0.1:
            compound = BOPTools.SplitAPI.slice(solid.native, tools, "Split", tolerance=tolerance)
        elif tolerance < 1e-12:
            if scale_up_floor is not None:
                floor = 1e-13 if scale_up_floor == 0 else scale_up_floor
                return self.split(solid, tool, floor / scale, scale=1.0 / scale, scale_up_floor=scale_up_floor)
            compound = BOPTools.SplitAPI.slice(solid.native, tools, "Split", tolerance=tolerance)
        else:
            try:
                compound = BOPTools.SplitAPI.slice(solid.native, tools, "Split", tolerance=tolerance)
            except Exception:
                retried = self.split(solid, tool, tolerance * scale, scale, scale_up_floor)
                return SplitResult(
                    solids=retried.solids,
                    degenerate_case_handled=True,
                    notes=f"retried at tolerance={tolerance * scale}",
                )

        if not compound.Solids:
            # tool doesn't intersect solid at all (e.g. a cutting plane
            # entirely outside the solid's extent) -- slice() reports this
            # as an empty compound rather than raising. Not a fragmentation,
            # so fall back to the solid unchanged instead of reporting "no
            # solids", which the ABC forbids.
            return SplitResult(
                solids=[solid], degenerate_case_handled=True,
                notes="tool did not intersect solid; returning it unchanged",
            )
        return SplitResult(solids=[self._wrap_solid(s) for s in compound.Solids])

    # -- Topological traversal --------------------------------------------

    def get_faces(self, solid: GSolid) -> list[GFace]:
        faces = [self._build_gface(f) for f in solid.native.Faces]
        for index, face in enumerate(faces):
            face.index = index
        return faces

    def get_edges(self, face: GFace) -> list[GEdge]:
        return [self._build_gedge(e) for e in face.native.Edges]

    def get_outer_wire(self, face: GFace) -> GWire:
        return self._wrap_wire(self._native_outer_wire(face.native))

    def get_wire_edges(self, wire: GWire) -> list[GEdge]:
        return [self._build_gedge(e) for e in wire.native.OrderedEdges]

    def get_vertices(self, edge: GEdge) -> list[GVertex]:
        return [self._build_gvertex(v) for v in edge.native.Vertexes]

    def get_solid_vertices(self, solid: GSolid) -> list[GVertex]:
        return [self._build_gvertex(v) for v in solid.native.Vertexes]

    def faces_sharing_edge(self, solid: GSolid, edge: GEdge) -> list[GFace]:
        matches = []
        for face in solid.native.Faces:
            for candidate in face.Edges:
                if candidate.isSame(edge.native):
                    matches.append(face)
                    break
        return [self._build_gface(f) for f in matches]

    def is_same_edge(self, edge_1: GEdge, edge_2: GEdge) -> bool:
        return edge_1.native.isSame(edge_2.native)

    def is_same_vertex(self, vertex_1: GVertex, vertex_2: GVertex) -> bool:
        return vertex_1.native.isSame(vertex_2.native)

    # -- Surface and curve classification -----------------------------

    def classify_surface(self, face: GFace) -> GSurface:
        return self._classify_native_surface(face.native)

    def classify_edge(self, edge: GEdge) -> GCurve:
        return self._classify_native_curve(edge.native)

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

    def is_part_of_domain(self, face: GFace, u: float, v: float) -> bool:
        return face.native.isPartOfDomain(u, v)

    def face_value_at(self, face: GFace, u: float, v: float) -> GVector:
        return to_gvector(face.native.valueAt(u, v))

    def face_parameter_at(self, face: GFace, point: GVector) -> tuple[float, float]:
        return face.native.Surface.parameter(to_fc_vector(point))

    def face_normal_at(self, face: GFace, u: float, v: float) -> GVector:
        return to_gvector(face.native.normalAt(u, v))

    def face_tangent_at(self, face: GFace, u: float, v: float) -> tuple[GVector, GVector]:
        d_u, d_v = face.native.tangentAt(u, v)
        return to_gvector(d_u), to_gvector(d_v)

    def tessellate(self, face: GFace, tolerance: float) -> list[GVector]:
        vertices, _facets = face.native.tessellate(tolerance)
        return [to_gvector(v) for v in vertices]

    def face_get_uv_nodes(self, face: GFace, tolerance: float) -> list[tuple[float, float]]:
        face.native.tessellate(tolerance)
        return face.native.getUVNodes()

    # -- Edge parametric queries -----------------------------------------

    def edge_parameter_range(self, edge: GEdge) -> tuple[float, float]:
        return edge.native.ParameterRange

    def edge_value_at(self, edge: GEdge, u: float) -> GVector:
        return to_gvector(edge.native.valueAt(u))

    def edge_derivative1_at(self, edge: GEdge, u: float) -> GVector:
        return to_gvector(edge.native.derivative1At(u))

    def edge_normal_at(self, edge: GEdge, u: float) -> GVector:
        return to_gvector(edge.native.normalAt(u))

    def edge_length(self, edge: GEdge) -> float:
        return edge.native.Length

    # -- Geometric properties -------------------------------------------------

    def volume(self, solid: GSolid) -> float:
        return solid.native.Volume

    def area(self, face: GFace) -> float:
        return face.native.Area

    def bounding_box(self, solid: GSolid) -> GBoundBox:
        return to_gboundbox(solid.native.BoundBox)

    def optimal_bounding_box(self, solid: GSolid, use_triangulation: bool = True) -> GBoundBox:
        return to_gboundbox(solid.native.optimalBoundingBox(use_triangulation))

    def center_of_mass(self, solid: GSolid) -> GVector:
        return to_gvector(solid.native.CenterOfMass)

    # -- Spatial queries -------------------------------------------------------

    def is_inside(self, solid: GSolid, point: GVector, tolerance: float) -> bool:
        return solid.native.isInside(to_fc_vector(point), tolerance, False)

    def find_interior_point(self, solid: GSolid) -> GVector | None:
        native = solid.native
        point = native.Solids[0].CenterOfMass
        if native.isInside(point, 0.0, False):
            return to_gvector(point)

        length = 0.5 * abs(native.Volume) ** 0.33333
        for face in native.Faces:
            u_min, u_max, v_min, v_max = face.ParameterRange
            u = 0.5 * (u_min + u_max)
            v = 0.5 * (v_min + v_max)
            if face.isPartOfDomain(u, v):
                normal = -face.normalAt(u, v)
                pos = face.valueAt(u, v)
                d = length
                for _ in range(12):
                    d = d * 0.5
                    point = pos + d * normal
                    if native.isInside(point, 0.0, False):
                        return to_gvector(point)
        return None

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

    def distance(self, shape_a: GShape, shape_b: GShape) -> float:
        return shape_a.native.distToShape(shape_b.native)[0]

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
        shape.translate(to_fc_vector(vector))
        return self._wrap_solid(shape)

    def rotate(
        self, solid: GSolid, axis_point: GVector, axis_dir: GVector,
        angle_rad: float,
    ) -> GSolid:
        shape = solid.native.copy()
        shape.rotate(to_fc_vector(axis_point), to_fc_vector(axis_dir), math.degrees(angle_rad))
        return self._wrap_solid(shape)
