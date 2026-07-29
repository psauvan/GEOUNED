import math

import numpy as np

from ....geo import (
    GBoundBox,
    GCone,
    GCylinder,
    GSolid,
    GVector,
    Gfuse,
    Gmake_box,
    Gmake_compound,
    Gmake_cone,
    Gmake_cylinder,
    Gmake_polygon_face,
    Gmake_shell,
    Gmake_sphere,
    to_gboundbox,
    to_gvector,
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
                boundBox.XMin, boundBox.YMin, boundBox.ZMin,
                boundBox.XMax, boundBox.YMax, boundBox.ZMax,
            ).__native__

    def getSubCell(self, seq):

        subCell = self.copy()
        subCell.definition = seq.copy()
        subCell.boundBox = self.boundBox

        surfaceList = subCell.definition.get_surfaces_numbers()
        for s in tuple(subCell.surfaces.keys()):
            if s not in surfaceList:
                del subCell.surfaces[s]

        return subCell


class myBox:
    def __init__(self, boundBox=None, orientation=None):
        self.Volume = 0
        if boundBox is not None:
            boundBox = to_gboundbox(boundBox)
            if boundBox.XLength <= 1e-12:
                self.Box = None
            elif boundBox.YLength <= 1e-12:
                self.Box = None
            elif boundBox.ZLength <= 1e-12:
                self.Box = None
            else:
                self.Box = boundBox
                self.Volume = boundBox.XLength * boundBox.YLength * boundBox.ZLength
        else:
            self.Box = None
        self.Orientation = orientation

    def add(self, box):
        if self.Orientation is None:
            self.Box = box.Box
            self.Orientation = box.Orientation
        elif self.Box is None:
            if self.Orientation == "Forward":
                self.Box = box.Box
                self.Orientation = box.Orientation
        elif box.Box is None:
            if box.Orientation == "Reversed":
                self.Box = None
                self.Orientation = "Reversed"
        elif self.Orientation == box.Orientation:
            self.Box = self.Box.union(box.Box)
        else:
            # -A OR B == -(A AND -B)
            if self.Orientation == "Forward":
                Rbox, Fbox = self, box
            else:
                Rbox, Fbox = box, self
            self.Box = box_intersect(Fbox, Rbox)
            self.Orientation = "Reversed"

    def mult(self, box):
        if self.Orientation is None:
            self.Box = box.Box
            self.Orientation = box.Orientation
        elif self.Box is None:
            if self.Orientation == "Reversed":
                self.Box = box.Box
                self.Orientation = box.Orientation
        elif box.Box is None:
            if box.Orientation == "Forward":
                self.Box = None
                self.Orientation = "Forward"
        elif self.Orientation == box.Orientation:
            inter = self.Box.intersected(box.Box)
            if inter.is_valid():
                self.Box = inter
            else:
                self.Box = None
        else:
            if self.Orientation == "Forward":
                Fbox, Rbox = self, box
            else:
                Fbox, Rbox = box, self
            self.Box = box_intersect(Fbox, Rbox)
            self.Orientation = "Forward"

    def sameBox(self, box):
        if self.Box is None or box.Box is None:
            if self.Box is None and box.Box is None:
                return self.Orientation == box.Orientation
            else:
                return False

        for i in range(6):
            p1 = self.Box.get_point(i)
            p2 = box.Box.get_point(i)
            if (p1 - p2).length > 1e-6:
                return False
        return True


def box_intersect(Fbox, Rbox):
    PX1 = (Fbox.Box.XMin, Fbox.Box.XMax)
    PX2 = (Rbox.Box.XMin, Rbox.Box.XMax)
    PY1 = (Fbox.Box.YMin, Fbox.Box.YMax)
    PY2 = (Rbox.Box.YMin, Rbox.Box.YMax)
    PZ1 = (Fbox.Box.ZMin, Fbox.Box.ZMax)
    PZ2 = (Rbox.Box.ZMin, Rbox.Box.ZMax)

    orientation = Fbox.Orientation
    bXmin, bXmax = Fbox.Box.XMin, Fbox.Box.XMax
    bYmin, bYmax = Fbox.Box.YMin, Fbox.Box.YMax
    bZmin, bZmax = Fbox.Box.ZMin, Fbox.Box.ZMax

    xmin, xmax = plane_region(PX1, PX2, orientation)
    boxes = []
    if xmin is not None:
        box = GBoundBox(xmin, bYmin, bZmin, xmax, bYmax, bZmax)
        boxes.append(box)

    ymin, ymax = plane_region(PY1, PY2, orientation)
    if ymin is not None:
        box = GBoundBox(bXmin, ymin, bZmin, bXmax, ymax, bZmax)
        boxes.append(box)

    zmin, zmax = plane_region(PZ1, PZ2, orientation)
    if zmin is not None:
        box = GBoundBox(bXmin, bYmin, zmin, bXmax, bYmax, zmax)
        boxes.append(box)

    if len(boxes) > 0:
        box = boxes[0]
        for b in boxes[1:]:
            box = box.union(b)
        return box
    else:
        return None


def plane_region(P1, P2, orient1):
    p11, p12 = P1
    p21, p22 = P2

    if p11 >= p22:
        return (p11, p12) if orient1 == "Forward" else (p21, p22)
    elif p12 <= p21:
        return (p11, p12) if orient1 == "Forward" else (p21, p22)
    else:
        if p11 < p21:
            if p12 < p22:
                return (p11, p21) if orient1 == "Forward" else (p12, p22)
            else:
                return (p11, p12) if orient1 == "Forward" else (None, None)
        elif p11 > p21:
            if p12 <= p22:
                return (None, None) if orient1 == "Forward" else (p21, p22)  # OK
            else:
                return (p22, p12) if orient1 == "Forward" else (p21, p11)
        else:
            if p12 < p22:
                return (None, None) if orient1 == "Forward" else (p12, p22)
            elif p12 > p22:
                return (p22, p12) if orient1 == "Forward" else (None, None)
            else:
                return (None, None)


def operate_box(definition, boxes):
    fullbox = myBox()
    definition.level_update()
    for e in definition.elements:
        if type(e) is int:
            box = boxes[abs(e)]
            if definition.operator == "AND":
                fullbox.mult(box)
            else:
                fullbox.add(box)
        else:
            box = operate_box(e, boxes)
            if definition.operator == "AND":
                fullbox.mult(box)
            else:
                fullbox.add(box)
    return fullbox


class Plane:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "plane"
        self.id = Id
        self.shape = None
        self.params = params
        if tr:
            self.transform(tr)

    def __str__(self):
        return f"plane : {self.id}\nParameters : {self.params}"

    def copy(self):
        plane = Plane(self.label, self.id, self.params)
        plane.shape = self.shape
        return plane

    def transform(self, matrix):
        v, d = self.params
        p = d * v  # vector p is d*plane normal
        v = matrix.submatrix(3).multVec(v)
        v.normalize()
        d = matrix.multVec(p) * v
        self.params = (v, d)

    def buildShape(self, boundBox):
        normal, p0 = to_gvector(self.params[0]), self.params[1]
        Box = to_gboundbox(boundBox).enlarged(10)

        pointEdge = []
        for i in range(12):
            edge = Box.get_edge(i)
            p1 = normal.dot(edge[0])
            p2 = normal.dot(edge[1])
            d0 = p0 - p1
            d1 = p2 - p1
            if d1 != 0:
                a = d0 / d1
                if a >= 0 and a <= 1:
                    pointEdge.append(edge[0] + a * (edge[1] - edge[0]))

        if len(pointEdge) == 0:
            self.shape = None
            return
        s = GVector(0, 0, 0)
        for v in pointEdge:
            s = s + v
        s = s / len(pointEdge)

        vtxvec = []
        for v in pointEdge:
            vtxvec.append(v - s)

        X0 = vtxvec[0]
        Y0 = normal.cross(X0)

        orden = []
        for i, v in enumerate(vtxvec):
            phi = np.arctan2(v.dot(Y0), v.dot(X0))
            orden.append((phi, i))
        orden.sort()

        self.shape = Gmake_polygon_face([pointEdge[p[1]] for p in orden]).__native__
        self.shell = self.shape


class Sphere:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "sphere"
        self.id = Id
        self.shape = None
        self.params = params
        if params[1] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[1]}")
        if tr:
            self.transform(tr)

    def copy(self):
        sphere = Sphere(self.label, self.id, self.params)
        sphere.shape = self.shape
        return sphere

    def transform(self, matrix):
        p, R = self.params
        p = matrix.multVec(p)
        self.params = (p, R)

    def buildShape(self, boundBox):
        origin, R = self.params
        self.shape = Gmake_sphere(to_gvector(origin), R).__native__
        self.shell = self.shape.Faces[0]


class Cylinder:
    def __init__(self, label, Id, params, tr=None, truncated=False):
        self.label = label
        self.type = "cylinder"
        self.id = Id
        self.shape = None
        self.params = params
        self.truncated = truncated
        if params[2] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2]}")
        if tr:
            self.transform(tr)

    def copy(self):
        cylinder = Cylinder(self.label, self.id, self.params, truncated=self.truncated)
        cylinder.shape = self.shape
        return cylinder

    def transform(self, matrix):
        p, v, R = self.params
        v = matrix.submatrix(3).multVec(v)
        p = matrix.multVec(p)
        self.params = (p, v, R)

    def buildShape(self, boundBox):

        p, vec, r = self.params

        if not self.truncated:
            dmin = vec.dot(boundBox.getPoint(0) - p)
            dmax = dmin
            for i in range(1, 8):
                d = vec.dot(boundBox.getPoint(i) - p)
                dmin = min(d, dmin)
                dmax = max(d, dmax)

            height = dmax - dmin
            dmin -= 0.1 * height
            dmax += 0.1 * height
            height = dmax - dmin

            point = p + dmin * vec
            gsolid = Gmake_cylinder(to_gvector(point), to_gvector(vec), r, height)
        else:
            gsolid = Gmake_cylinder(to_gvector(p), to_gvector(vec), r, vec.Length)

        self.shape = gsolid.__native__
        for f in gsolid.Faces:
            if type(f.Surface) is GCylinder:
                self.shell = f.__native__
        return


class Cone:
    def __init__(self, label, Id, params, tr=None, truncated=False):
        self.label = label
        self.type = "cone"
        self.id = Id
        self.shape = None
        self.params = params
        self.truncated = truncated
        # if params[2] <= 0:
        #    print(f"{self.type} surface {label} has a zero semi-angle value.")
        if tr:
            self.transform(tr)

    def copy(self):
        cone = Cone(self.label, self.id, self.params, truncated=self.truncated)
        cone.shape = self.shape
        return cone

    def transform(self, matrix):
        if not self.truncated:
            p, v, t, dbl = self.params
            v = matrix.submatrix(3).multVec(v)
            p = matrix.multVec(p)
            self.params = (p, v, t, dbl)
        else:
            p, v, r1, r2 = self.params
            v = matrix.submatrix(3).multVec(v)
            p = matrix.multVec(p)
            self.params = (p, v, r1, r2)

    def buildShape(self, boundBox):
        if not self.truncated:
            apex, axis, t, dblsht = self.params

            dmin = axis.dot(boundBox.getPoint(0) - apex)
            dmax = dmin
            for i in range(1, 8):
                d = axis.dot(boundBox.getPoint(i) - apex)
                dmin = min(d, dmin)
                dmax = max(d, dmax)

            length = max(abs(dmin), abs(dmax))
            half_angle = math.atan(t)
            one_sheet = Gmake_cone(to_gvector(apex), to_gvector(axis), half_angle, length)
            oneface = next(f for f in one_sheet.Faces if type(f.Surface) is GCone)

            if not dblsht:
                self.shape = one_sheet.__native__
                self.shell = oneface.__native__
            else:
                other_sheet = Gmake_cone(to_gvector(apex), to_gvector(-axis), half_angle, length)
                otherface = next(f for f in other_sheet.Faces if type(f.Surface) is GCone)
                double_sheet = Gfuse([one_sheet, other_sheet])
                self.shape = double_sheet.__native__
                self.shell = Gmake_shell((oneface, otherface)).__native__
        # truncated (frustum, two explicit radii) Cone is never constructed
        # with truncated=True anywhere in build_region -- no backend
        # primitive covers that case, so it is intentionally left
        # unimplemented rather than ported speculatively.


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


def FuseSolid(parts):
    if (len(parts)) <= 1:
        if parts:
            solid = parts[0]
        else:
            return None
    else:
        gparts = [GSolid(p) for p in parts]
        try:
            fused = Gfuse(gparts)
        except Exception:
            fused = None

        if fused is not None:
            try:
                refined = fused.refine()
            except Exception:
                refined = fused

            if refined.is_valid():
                gsolid = refined
            elif fused.is_valid():
                gsolid = fused
            else:
                gsolid = Gmake_compound(gparts)
        else:
            gsolid = Gmake_compound(gparts)
        solid = gsolid.__native__

    if solid.Volume < 0:
        solid = GSolid(solid).reverse().__native__
    return solid
