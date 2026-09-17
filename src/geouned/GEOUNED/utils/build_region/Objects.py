import math

import numpy as np

from ....geo import (
    GBoundBox,
    GCone,
    GCylinder,
    GPlane,
    GSphere,
    GVector,
    Gmake_box,
    Gmake_cone,
    Gmake_cylinder,
    Gmake_polygon_face,
    Gmake_sphere,
    to_gboundbox,
)


class CellObj:
    def __init__(self):
        self.boundBox = None
        self.surfaces = {}
        self.definition = None

    def copy(self):
        cpCell = CellObj()
        cpCell.boundBox = self.boundBox
        cpCell.surfaces = {}
        for name, s in self.surfaces.items():
            cpCell.surfaces[name] = s.copy()

        cpCell.definition = self.definition.copy()
        return cpCell

    def makeBox(self):
        boundBox = self.boundBox.Box
        if boundBox.XLength < 1e-6 or boundBox.YLength < 1e-6 or boundBox.ZLength < 1e-6:
            return None
        else:
            return Gmake_box(
                boundBox.XMin,
                boundBox.YMin,
                boundBox.ZMin,
                boundBox.XMax,
                boundBox.YMax,
                boundBox.ZMax,
            )

    def getSubCell(self, seq):

        subCell = self.copy()
        subCell.definition = seq.copy()
        subCell.boundBox = self.boundBox

        surfaceList = subCell.definition.get_surfaces_numbers()
        for s in tuple(subCell.surfaces.keys()):
            if s not in surfaceList:
                del subCell.surfaces[s]

        return subCell


# myBox (Forward/Reversed box arithmetic), box_intersect, plane_region, and
# operate_box used to live here as GEOUNED's own copy of GEOReverse's
# `Utils/boundBox.py` machinery -- confirmed dead code (operate_box has zero
# callers anywhere in the repo besides its own recursion) carrying the exact
# same mixed-orientation bug GEOReverse's copy had (see `geo.myBox`'s own
# docstring for the fix). Moved to `geo.myBox`, 2026-09-17 -- see CLAUDE.md's
# "build_region/ vs CAD/buildSolidCell.py+splitFunction.py unification"
# entry. `sameBox`/`mult`'s Forward/Forward branch (the only one this
# pipeline's single live call site, `filterparts` below, ever exercises)
# behave identically to before.


def plane_polygon_from_box(normal: GVector, offset: float, box: GBoundBox):
    """
    Build the (possibly non-rectangular) polygon face where the infinite
    plane {p : normal.dot(p) == offset} crosses `box`, by intersecting the
    plane with each of the box's 12 edges and ordering the resulting
    points into a convex polygon. Returns None if the plane doesn't cross
    the box at all. `box` should already include whatever margin the
    caller wants (this function does not enlarge it).
    """
    pointEdge = []
    for i in range(12):
        edge = box.get_edge(i)
        p1 = normal.dot(edge[0])
        p2 = normal.dot(edge[1])
        d0 = offset - p1
        d1 = p2 - p1
        if d1 != 0:
            a = d0 / d1
            if 0 <= a <= 1:
                pointEdge.append(edge[0] + a * (edge[1] - edge[0]))

    if len(pointEdge) == 0:
        return None

    s = GVector(0, 0, 0)
    for v in pointEdge:
        s = s + v
    s = s / len(pointEdge)

    X0 = pointEdge[0] - s
    Y0 = normal.cross(X0)

    orden = []
    for i, v in enumerate(pointEdge):
        vv = v - s
        phi = np.arctan2(vv.dot(Y0), vv.dot(X0))
        orden.append((phi, i))
    orden.sort()

    return Gmake_polygon_face([pointEdge[p[1]] for p in orden])


def cylinder_from_box(center: GVector, axis: GVector, radius: float, box: GBoundBox):
    """
    Build a cylinder solid long enough to fully cover `box` along its
    axis, with a 10% margin on each end. Verified empirically (splitting a
    real solid against the resulting tool) that the exact margin size
    doesn't change the result as long as it's nonzero -- only that the
    tool is big enough to clear the box.
    """
    dmin = axis.dot(box.get_point(0) - center)
    dmax = dmin
    for i in range(1, 8):
        d = axis.dot(box.get_point(i) - center)
        dmin = min(d, dmin)
        dmax = max(d, dmax)

    height = dmax - dmin
    dmin -= 0.1 * height
    dmax += 0.1 * height
    height = dmax - dmin

    point = center + dmin * axis
    return Gmake_cylinder(point, axis, radius, height)


def cone_from_box(apex: GVector, axis: GVector, tan: float, box: GBoundBox):
    """
    Build a cone solid extending from `apex` along `axis` (or `-axis`, if
    `tan` is negative). OCC's native Cone.SemiAngle -- and therefore `tan`,
    computed from it upstream -- is signed exactly to record which
    direction the real material lies in (verified empirically: a frustum
    whose wide end is on the -axis side of the apex reports a negative
    SemiAngle, matching a sample point actually on that face). This must
    be trusted, never re-derived by guessing from which side of the apex
    the box happens to sit -- scanning the box in the *wrong* direction
    can also come out positive, silently building a cone that misses the
    box entirely instead of failing loudly. Returns None if the box
    doesn't extend at all in the (correctly signed) direction the cone
    opens in -- i.e. the cone genuinely doesn't intersect the box.
    """
    build_axis = axis if tan >= 0 else -axis
    half_angle = math.atan(abs(tan))

    dmax = build_axis.dot(box.get_point(0) - apex)
    for i in range(1, 8):
        d = build_axis.dot(box.get_point(i) - apex)
        dmax = max(d, dmax)

    if dmax <= 0:
        return None

    return Gmake_cone(apex, build_axis, half_angle, dmax * 1.1)


class CellSurface:
    """A CSG-cell surface used by BuildDepth/SplitSolid to reconstruct a
    composite (Can/TCone/RoundCorner/MultiRoundCorner) meta-surface's CAD
    shape from its primitive components. Wraps one `geo` analytic
    descriptor (GPlane/GCylinder/GCone/GSphere) with the id/label CSG
    numbering and a CAD shape that gets rebuilt per call against whatever
    (shrinking, as BuildDepth recurses) box it's asked for -- this is why
    it can't just reuse GeounedSurface.shape (built once, against the
    top-level box).

    Point classification (`is_inside`) and affine transform are delegated
    straight to the wrapped descriptor instead of reimplementing them
    natively a third time -- this replaces the old separate Plane/
    Cylinder/Cone/Sphere classes here, whose `.transform()` (native
    FreeCAD.Matrix) was confirmed dead in CadToCsg (get_surface() never
    passed a `tr`) and whose point-classification (splitFunction.py's
    surface_side) duplicated boolean_solids.check_sign_primitive's
    formulas natively instead of reusing them. `.transform()` is kept
    (not deleted outright) because CsgToCad (GEOReverse) does need to
    move surfaces around -- see GPlane.transform et al.

    The old classes' `truncated` flag (an explicit-end-planes cylinder/
    cone variant) and Cone's double-sheet branch are not carried over:
    both were already confirmed dead in the code they came from
    (get_surface() never sets `truncated=True` or builds a double-sheet
    Cone), and neither has an equivalent in `geo`'s descriptors.
    """

    _TYPE_NAMES = {GPlane: "plane", GCylinder: "cylinder", GCone: "cone", GSphere: "sphere"}

    def __init__(self, label, Id, descriptor, tr=None):
        self.label = label
        self.id = Id
        self.shape = None
        self.descriptor = descriptor if tr is None else descriptor.transform(tr)
        self.type = self._TYPE_NAMES[type(self.descriptor)]
        if self.type in ("sphere", "cylinder") and self.descriptor.Radius <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {self.descriptor.Radius}")

    def copy(self):
        s = CellSurface(self.label, self.id, self.descriptor)
        s.shape = self.shape
        return s

    def is_inside(self, point):
        return self.descriptor.is_inside(point)

    def transform(self, matrix):
        self.descriptor = self.descriptor.transform(matrix)

    def buildShape(self, boundBox):
        box = to_gboundbox(boundBox).enlarged(10)
        d = self.descriptor

        if self.type == "plane":
            face = plane_polygon_from_box(d.Axis, d.Axis.dot(d.Position), box)
            self.shape = face.__native__ if face is not None else None

        elif self.type == "cylinder":
            self.shape = cylinder_from_box(d.Center, d.Axis, d.Radius, box).__native__

        elif self.type == "cone":
            gsolid = cone_from_box(d.Apex, d.Axis, math.tan(d.SemiAngle), box)
            self.shape = gsolid.__native__ if gsolid is not None else None

        elif self.type == "sphere":
            self.shape = Gmake_sphere(d.Center, d.Radius).__native__


class Undefined:
    def __init__(self, Id):
        self.type = "Undefined"
        self.id = Id
        self.shape = None
        self.params = None

    def copy(self):
        return Undefined(self.id)

    def buildShape(self, boundBox):
        return

    def transform(self, matrix):
        return
