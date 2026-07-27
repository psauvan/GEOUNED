"""
geometry_backend/vector_geometry.py

Geometric predicates built purely on `GVector` arithmetic. Backend-
agnostic: these never touch a `GeometryBackend`, `Part`, `FreeCAD`, or
`OCC.Core` object. This is the layer above the adapter interface where
GEOUNED's own reasoning about parallelism, colinearity, and plane
membership lives (previously scattered across `basic_functions_part1.py`
as free functions taking FreeCAD.Vector directly).
"""

from __future__ import annotations

import math

from .geometry_backend_interface import GVector, PlaneParams


def is_same_value(v1: float, v2: float, tolerance: float = 1e-6) -> bool:
    return abs(v1 - v2) < tolerance


def is_opposite(vector_1: GVector, vector_2: GVector, tolerance: float = 1e-3) -> bool:
    return vector_1.angle_to(-vector_2) < tolerance


def is_parallel(vector_1: GVector, vector_2: GVector, tolerance: float = 1e-3) -> bool:
    angle = vector_1.angle_to(vector_2)
    return angle < tolerance or is_same_value(angle, math.pi, tolerance)


def is_in_line(point: GVector, direction: GVector, point_on_line: GVector, tolerance: float = 1e-6) -> bool:
    to_point = point - point_on_line
    return is_parallel(direction, to_point) or to_point.length < tolerance


def is_in_plane(point: GVector, plane: PlaneParams, tolerance: float = 1e-7) -> bool:
    return abs(plane.normal.dot(point - plane.point)) < tolerance


def sign_plane(point: GVector, plane: PlaneParams) -> int:
    return 1 if plane.normal.dot(point - plane.point) >= 0.0 else -1
