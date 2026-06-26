import math

import FreeCAD
import numpy as np
import Part


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
        box_origin = FreeCAD.Vector(boundBox.XMin, boundBox.YMin, boundBox.ZMin)
        if boundBox.XLength < 1e-6 or boundBox.YLength < 1e-6 or boundBox.ZLength < 1e-6:
            return None
        else:
            return Part.makeBox(boundBox.XLength, boundBox.YLength, boundBox.ZLength, box_origin)

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
            self.Box.add(box.Box)
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
            if inter.isValid():
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
            p1 = self.Box.getPoint(i)
            p2 = box.Box.getPoint(i)
            if (p1 - p2).Length > 1e-6:
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
        box = FreeCAD.BoundBox(xmin, bYmin, bZmin, xmax, bYmax, bZmax)
        boxes.append(box)

    ymin, ymax = plane_region(PY1, PY2, orientation)
    if ymin is not None:
        box = FreeCAD.BoundBox(bXmin, ymin, bZmin, bXmax, ymax, bZmax)
        boxes.append(box)

    zmin, zmax = plane_region(PZ1, PZ2, orientation)
    if zmin is not None:
        box = FreeCAD.BoundBox(bXmin, bYmin, zmin, bXmax, bYmax, zmax)
        boxes.append(box)

    if len(boxes) > 0:
        box = boxes[0]
        for b in boxes[1:]:
            box.add(b)
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
        normal, p0 = self.params
        Box = FreeCAD.BoundBox(boundBox)
        Box.enlarge(10)

        pointEdge = []
        for i in range(12):
            edge = Box.getEdge(i)
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
        s = FreeCAD.Vector((0, 0, 0))
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

        self.shape = Part.Face(Part.makePolygon([pointEdge[p[1]] for p in orden], True))
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
        self.shape = Part.makeSphere(R, origin)
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
            self.shape = Part.makeCylinder(r, height, point, vec, 360)
            # self.shape = makeCylinder2( r,height,point,vec)
        else:
            self.shape = Part.makeCylinder(r, vec.Length, p, vec, 360)
            # self.shape = Part.makeCylinder2( r,vec.Length,p,vec)

        for f in self.shape.Faces:
            if type(f.Surface) is Part.Cylinder:
                self.shell = f
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
            R = length * t
            OneSheetCone = Part.makeCone(0, R, length, apex, axis, 360)
            for f in OneSheetCone.Faces:
                if type(f.Surface) is Part.Cone:
                    oneface = f
            if not dblsht:
                self.shape = OneSheetCone
                self.shell = oneface
            else:
                OtherSheet = Part.makeCone(0, R, length, apex, -axis, 360)
                DoubleSheetCone = OneSheetCone.fuse([OtherSheet])
                DoubleSheetCone.removeSplitter()
                self.shape = DoubleSheetCone
                for f in OtherSheet.Faces:
                    if type(f.Surface) is Part.Cone:
                        otherface = f
                self.shell = Part.makeShell((oneface, otherface))
        else:
            center, axis, r1, r2 = self.params
            self.shape = Part.makeCone(r1, r2, axis.Length, center, axis, 360)
            for f in self.shape.Faces:
                if type(f.Surface) is Part.Cone:
                    self.shell = f


class Torus:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "torus"
        self.id = Id
        self.shape = None
        self.params = params
        if params[2] < 0:
            print(f"{self.type} surface {label} has a negative major radius: {params[2]}")
        if params[3] <= 0:
            print(f"{self.type} surface {label} has a bad minor radius a value: {params[3]}")
        if params[4] <= 0:
            print(f"{self.type} surface {label} has a bad minor radius b value: {params[4]}")

        if tr:
            self.transform(tr)

    def copy(self):
        torus = Torus(self.label, self.id, self.params)
        torus.shape = self.shape
        return torus

    def transform(self, matrix):
        p, v, Ra, Rb, Rc = self.params
        v = matrix.submatrix(3).multVec(v)
        p = matrix.multVec(p)
        self.params = (p, v, Ra, Rb, Rc)

    def buildShape(self, boundBox):
        center, axis, Ra, Rb, Rc = self.params  # Ra distance from torus axis; R radius of toroidal-cylinder
        if (abs(Rb - Rc) < 1e-5) and Ra > 0:
            self.shape = Part.makeTorus(Ra, Rb, center, axis)  # FreeCAD circular Torus
            self.shell = self.shape.Shells[0]
        else:
            self.shape, self.shell = makeEllipticTorus(Ra, Rb, Rc, center, axis)  # Home made elliptic Torus


class Box:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "box"
        self.id = Id
        self.shape = None
        self.params = params
        if params[1].Length <= 0:
            print(f"{self.type} surface {label} has a bad X dimension: {params[1]}")
        if params[2].Length <= 0:
            print(f"{self.type} surface {label} has a bad Y dimension: {params[2]}")
        if params[3].Length <= 0:
            print(f"{self.type} surface {label} has a bad Z dimension: {params[3]}")

        if tr:
            self.transform(tr)

    def copy(self):
        box = Box(self.label, self.id, self.params)
        box.shape = self.shape
        return box

    def transform(self, matrix):
        p, v1, v2, v3 = self.params
        p = matrix.multVec(p)
        v1 = matrix.multVec(v1)
        v2 = matrix.multVec(v2)
        v3 = matrix.multVec(v3)
        self.params = (p, v1, v2, v3)

    def buildShape(self, boundBox):
        p, v1, v2, v3 = self.params
        a1 = FreeCAD.Vector(v1)
        a2 = FreeCAD.Vector(v2)
        a3 = FreeCAD.Vector(v3)
        a1.normalize()
        a2.normalize()
        a3.normalize()

        m = FreeCAD.Matrix(
            a1.x,
            a2.x,
            a3.x,
            p.x,
            a1.y,
            a2.y,
            a3.y,
            p.y,
            a1.z,
            a2.z,
            a3.z,
            p.z,
            0,
            0,
            0,
            1,
        )
        box = Part.makeBox(v1.Length, v2.Length, v3.Length)
        self.shape = box.transformGeometry(m)
        trsfBox = box.transformGeometry(m)
        self.shell = trsfBox.Shells[0]


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
        try:
            fused = parts[0].fuse(parts[1:])
        except:
            fused = None

        if fused is not None:
            try:
                refinedfused = fused.removeSplitter()
            except:
                refinedfused = fused

            if refinedfused.isValid():
                solid = refinedfused
            else:
                if fused.isValid():
                    solid = fused
                else:
                    solid = Part.makeCompound(parts)
        else:
            solid = Part.makeCompound(parts)

    if solid.Volume < 0:
        solid.reverse()
    return solid


def makeEllipticTorus(R, RZ, RX, center, ZAxis):

    rMaj = RZ
    rMin = RX
    XAxis = ortoVect(ZAxis)

    majorAxis = ZAxis
    minorAxis = XAxis
    if rMaj < rMin:
        rMaj, rMin = rMin, rMaj
        majorAxis, minorAxis = minorAxis, majorAxis

    eCenter = center + R * XAxis
    S1 = eCenter + majorAxis * rMaj  # major axis
    S2 = eCenter + minorAxis * rMin  # minor axis

    ellipse = Part.Ellipse(S1, S2, eCenter)
    if abs(R) < RX:  # degenerated Torus
        pz = RZ * math.sqrt(1 - (R / RX) ** 2)
        pz1 = center - pz * ZAxis
        pz2 = center + pz * ZAxis

        p1 = ellipse.parameter(pz1)
        p2 = ellipse.parameter(pz2)
        if p2 < p1:
            p2 += 2 * math.pi
        shape = ellipse.toBSpline(p1, p2).toShape(p1, p2)  # revolution around Major axis
        rev = shape.revolve(center, ZAxis, 360)
    else:
        shape = ellipse.toBSpline().toShape()  # revolution around Minor axis
        rev = shape.revolve(center, ZAxis, 360)
    shell = Part.makeShell((rev,))
    return (Part.makeSolid(shell), shell)


def ortoVect(v):
    vmax = 0
    vOrto = None
    if abs(v.x) > vmax:
        vOrto = (0, 1, 0)
        vmax = abs(v.x)
    if abs(v.y) > vmax:
        vOrto = (0, 0, 1)
        vmax = abs(v.y)
    if abs(v.z) > vmax:
        vOrto = (1, 0, 0)
        vmax = abs(v.z)

    if vOrto is None:
        return None

    vOrto = v.cross(FreeCAD.Vector(vOrto))
    vOrto.normalize()
    return vOrto
