import math

import numpy as np

from .buildSolidCell import BuildSolid
from .remh import Cline
from .Utils.booleanFunction import BoolSequence, outer_terms
from .Utils.boundBox import solid_plane_box, myBox, BoxSettings, makePlane
from .geo_quadrics import (
    Gmake_elliptic_cone,
    Gmake_elliptic_cylinder,
    Gmake_ellipsoid,
    Gmake_hyperbolic_cylinder,
    Gmake_hyperboloid,
    Gmake_paraboloid,
    Gmake_torus_elliptic,
)
from ._geo_bridge import (
    GBoundBox,
    Gmake_box,
    Gmake_cone,
    Gmake_cone_double_sheet,
    Gmake_cone_frustum,
    Gmake_cylinder,
    Gmake_sphere,
    Gmake_torus,
    fuse_solids,
    matrix_multVec,
    matrix_rotate_vec,
    transform_solid,
)


class CadCell:
    def __init__(self, stringCell: str = None, settings: BoxSettings = BoxSettings()):

        self.settings = settings
        self.boundBox = None

        if not stringCell:
            self.surfaces = {}
            self.surfaceList = []
            self.shape = None
            # self.likeCell = None
            self.definition = None
            self.name = 0
            # self.TRCL     = None  # cell transformacion "like-but" cells
            self.TRFL = None  # Universe transformation in fill Universe
            self.U = -1  # Cell Universe number
            self.FILL = 0  # Fill Universe number
            self.MAT = 0  # material number
            self.CurrentTR = None
            self.level = None
            self.__defTerms__ = None
            self.__operator__ = None
            self.externalBox = None
            self.solid_plane = None
            self.boundBox = None

        else:
            self.surfaces = None
            self.shape = None
            self.name = stringCell.name
            self.TRFL = stringCell.TR  # Universe transformation in fill Universe
            self.U = stringCell.U  # Cell Universe number
            self.FILL = stringCell.FILL  # Fill Universe number
            self.MAT = stringCell.MAT  # material number
            self.CurrentTR = self.TRFL
            self.level = None

            self.__defTerms__ = None
            self.__operator__ = None
            self.__setDefinition__(stringCell)
            self.surfaceList = self.definition.get_surfaces_numbers()
            self.externalBox = None
            self.solid_plane = None
            self.boundBox = None

    def copy(self):
        cpCell = CadCell(settings=self.settings)
        cpCell.solid_plane = self.solid_plane.copy()
        cpCell.surfaceList = self.surfaceList[:]
        cpCell.externalBox = self.externalBox
        cpCell.boundBox = self.boundBox
        cpCell.surfaces = {}
        for name, s in self.surfaces.items():
            cpCell.surfaces[name] = s.copy()

        if type(self.definition) is Cline:
            cpCell.definition = Cline(self.definition.str)

        elif type(self.definition) is BoolSequence:
            cpCell.definition = self.definition.copy()

        cpCell.name = self.name
        cpCell.TRFL = self.TRFL
        cpCell.U = self.U
        cpCell.FILL = self.FILL
        cpCell.MAT = self.MAT
        cpCell.level = self.level

        if self.CurrentTR is not None:
            cpCell.CurrentTR = self.CurrentTR.copy()

        if self.shape is not None:
            cpCell.shape = self.shape.copy()

        return cpCell

    def getSubCell(self, seq):

        subCell = self.copy()
        subCell.definition = seq.copy()
        subCell.shape = None
        subCell.boundBox = None

        subCell.surfaceList = subCell.definition.get_surfaces_numbers()
        for s in tuple(subCell.surfaces.keys()):
            if s not in subCell.surfaceList:
                del subCell.surfaces[s]

        return subCell

    def getOuterTerms(self):
        if not self.__defTerms__:
            self.__defTerms__, self.__operator__ = outer_terms(self.definition.str)
        return self.__defTerms__, self.__operator__

    def makeBox(self):
        if self.boundBox.Orientation == "Forward":
            if self.boundBox.Box is None:
                boundBox = self.externalBox.Box
            else:
                boundBox = self.boundBox.Box
        else:
            boundBox = self.externalBox.Box
        if boundBox.XLength < 1e-6 or boundBox.YLength < 1e-6 or boundBox.ZLength < 1e-6:
            return None
        else:
            return Gmake_box(boundBox.XMin, boundBox.YMin, boundBox.ZMin, boundBox.XMax, boundBox.YMax, boundBox.ZMax)

    def build_BoundBox(self, externalBox=None, enlarge=0):

        if externalBox:
            outBox = externalBox
            self.externalBox = externalBox
        elif self.externalBox:
            outBox = self.externalBox
        else:
            r = self.settings.universe_radius
            outBox = myBox(GBoundBox(-r, -r, -r, r, r, r), "Forward")
            self.externalBox = outBox

        if outBox.Box is None:
            self.boundBox = outBox
        else:
            if self.solid_plane is None:
                self.solid_plane = solid_plane_box(self, outbox=outBox)
            elif not self.solid_plane.outBox.sameBox(outBox):
                self.solid_plane = solid_plane_box(self, outbox=outBox)
            self.boundBox = self.solid_plane.get_boundBox(enlarge=enlarge)

    def buildShape(self, force=False, surfTR=None, simplify=False, fuse=False):

        if self.shape is not None and not force:
            return
        if surfTR is not None:
            self.transformSurfaces(surfTR)

        cutShape = BuildSolid(self)
        self.shape = fuse_solids(cutShape)

    def buildSurfaceShape(self, boundBox):
        for s in self.surfaces.values():
            s.buildShape(boundBox)

    def transformSolid(self, matrix, reverse=False):
        if not self.shape:
            return
        m = np.linalg.inv(matrix) if reverse else matrix
        self.shape = transform_solid(self.shape, m)

    def transformSurfaces(self, matrix):
        for s in self.surfaces.values():
            s.transform(matrix)

    def setSurfaces(self, Surfaces):
        if self.surfaces is not None:
            return
        self.surfaces = {}
        for s in self.surfaceList:
            self.surfaces[s] = Surfaces[s]

    def cleanUndefined(self):
        undefined = []
        for s in self.definition.get_surfaces_numbers():
            if self.surfaces[s].params is None:
                undefined.append(s)
        if undefined:
            self.definition.removeSurface(undefined)

        for s in undefined:
            del self.surfaces[s]

    def __setDefinition__(self, stringCell):

        self.definition = stringCell.geom
        self.definition.remove_comments(full=True)
        self.definition.remove_cr()
        self.definition.remove_multispace()
        self.definition.remove_redundant()


class Plane:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "plane"
        self.id = Id
        self.shape = None
        self.params = params
        if tr is not None:
            self.transform(tr)

    def __str__(self):
        return f"plane : {self.id}\nParameters : {self.params}"

    def copy(self):
        return Plane(self.label, self.id, self.params)

    def transform(self, matrix):
        v, d = self.params
        p = d * v  # vector p is d*plane normal
        v = matrix_rotate_vec(matrix, v).normalized()
        d = matrix_multVec(matrix, p).dot(v)
        self.params = (v, d)

    def buildShape(self, boundBox):
        normal, d = self.params
        position = normal * d
        self.shape = makePlane(normal, position, boundBox.enlarged(10))


class Sphere:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "sphere"
        self.id = Id
        self.shape = None
        self.params = params
        if params[1] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[1]}")
        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Sphere(self.label, self.id, self.params)

    def transform(self, matrix):
        p, R = self.params
        p = matrix_multVec(matrix, p)
        self.params = (p, R)

    def buildShape(self, boundBox):
        origin, R = self.params
        self.shape = Gmake_sphere(origin, R)


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
        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Cylinder(self.label, self.id, self.params, truncated=self.truncated)

    def transform(self, matrix):
        p, v, R = self.params
        v = matrix_rotate_vec(matrix, v)
        p = matrix_multVec(matrix, p)
        self.params = (p, v, R)

    def buildShape(self, boundBox):

        p, vec, r = self.params

        if not self.truncated:
            dmin = vec.dot(boundBox.get_point(0) - p)
            dmax = dmin
            for i in range(1, 8):
                d = vec.dot(boundBox.get_point(i) - p)
                dmin = min(d, dmin)
                dmax = max(d, dmax)

            height = dmax - dmin
            dmin -= 0.1 * height
            dmax += 0.1 * height
            height = dmax - dmin

            point = p + dmin * vec
            self.shape = Gmake_cylinder(point, vec, r, height)
        else:
            self.shape = Gmake_cylinder(p, vec, r, vec.length)

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
        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Cone(self.label, self.id, self.params, truncated=self.truncated)

    def transform(self, matrix):
        if not self.truncated:
            p, v, t, dbl = self.params
            v = matrix_rotate_vec(matrix, v)
            p = matrix_multVec(matrix, p)
            self.params = (p, v, t, dbl)
        else:
            p, v, r1, r2 = self.params
            v = matrix_rotate_vec(matrix, v)
            p = matrix_multVec(matrix, p)
            self.params = (p, v, r1, r2)

    def buildShape(self, boundBox):
        if not self.truncated:
            apex, axis, t, dblsht = self.params

            dmin = axis.dot(boundBox.get_point(0) - apex)
            dmax = dmin
            for i in range(1, 8):
                d = axis.dot(boundBox.get_point(i) - apex)
                dmin = min(d, dmin)
                dmax = max(d, dmax)

            length = max(abs(dmin), abs(dmax))
            half_angle = math.atan(t)
            if not dblsht:
                self.shape = Gmake_cone(apex, axis, half_angle, length)
            else:
                self.shape = Gmake_cone_double_sheet(apex, axis, half_angle, length)
        else:
            center, axis, r1, r2 = self.params
            self.shape = Gmake_cone_frustum(center, axis, r1, r2, axis.length)


class EllipticCone:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "cone_elliptic"
        self.id = Id
        self.shape = None
        self.params = params
        if params[3][0] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[3][0]}")
        if params[3][1] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[3][1]}")
        if tr is not None:
            self.transform(tr)

    def copy(self):
        return EllipticCone(self.label, self.id, self.params)

    def transform(self, matrix):
        p, v, ra, radii, raxes, dbl = self.params
        v = matrix_rotate_vec(matrix, v)
        raxes = [matrix_rotate_vec(matrix, raxes[0]), matrix_rotate_vec(matrix, raxes[1])]
        p = matrix_multVec(matrix, p)
        self.params = (p, v, ra, radii, raxes, dbl)

    def buildShape(self, boundBox):
        apex, axis, ra, radii, raxes, dblsht = self.params

        dmin = axis.dot(boundBox.get_point(0) - apex)
        dmax = dmin
        for i in range(1, 8):
            d = axis.dot(boundBox.get_point(i) - apex)
            dmin = min(d, dmin)
            dmax = max(d, dmax)

        length = max(abs(dmin), abs(dmax))
        self.shape = Gmake_elliptic_cone(apex, axis, ra, radii[1], radii[0], raxes[1], raxes[0], dblsht, length)


class Hyperboloid:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "hyperboloid"
        self.id = Id
        self.shape = None
        self.params = params
        if params[2][0] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][0]}")
        if params[2][1] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][1]}")
        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Hyperboloid(self.label, self.id, self.params)

    def transform(self, matrix):
        p, v, radii, raxes, onesht = self.params
        v = matrix_rotate_vec(matrix, v)
        raxes = [matrix_rotate_vec(matrix, raxes[0]), matrix_rotate_vec(matrix, raxes[1])]
        p = matrix_multVec(matrix, p)
        self.params = (p, v, radii, raxes, onesht)

    def buildShape(self, boundBox):
        center, axis, radii, rAxes, onesht = self.params

        dmin = axis.dot(boundBox.get_point(0) - center)
        dmax = dmin
        for i in range(1, 8):
            d = axis.dot(boundBox.get_point(i) - center)
            dmin = min(d, dmin)
            dmax = max(d, dmax)

        length = max(abs(dmin), abs(dmax))
        self.shape = Gmake_hyperboloid(center, axis, radii[1], radii[0], rAxes[1], rAxes[0], onesht, length)


class Ellipsoid:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "ellipsoid"
        self.id = Id
        self.shape = None
        self.params = params
        if params[2][0] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][0]}")
        if params[2][1] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][1]}")
        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Ellipsoid(self.label, self.id, self.params)

    def transform(self, matrix):
        p, v, radii, raxes = self.params
        v = matrix_rotate_vec(matrix, v)
        raxes = [matrix_rotate_vec(matrix, raxes[0]), matrix_rotate_vec(matrix, raxes[1])]
        p = matrix_multVec(matrix, p)
        self.params = (p, v, radii, raxes)

    def buildShape(self, boundBox):
        center, axis, radii, rAxes = self.params
        self.shape = Gmake_ellipsoid(center, axis, radii[1], radii[0], rAxes[1], rAxes[0])


class EllipticCylinder:
    def __init__(self, label, Id, params, tr=None, truncated=False):
        self.label = label
        self.type = "cylinder_elliptic"
        self.id = Id
        self.shape = None
        self.params = params
        self.truncated = truncated
        if params[2][0] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][0]}")
        if params[2][1] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][1]}")
        if tr is not None:
            self.transform(tr)

    def copy(self):
        return EllipticCylinder(self.label, self.id, self.params, truncated=self.truncated)

    def transform(self, matrix):
        p, v, radii, raxes = self.params
        v = matrix_rotate_vec(matrix, v)
        raxes = [matrix_rotate_vec(matrix, raxes[0]), matrix_rotate_vec(matrix, raxes[1])]
        p = matrix_multVec(matrix, p)
        self.params = (p, v, radii, raxes)

    def buildShape(self, boundBox):
        center, axis, radii, rAxes = self.params
        if not self.truncated:
            dmin = axis.dot(boundBox.get_point(0) - center)
            dmax = dmin
            for i in range(1, 8):
                d = axis.dot(boundBox.get_point(i) - center)
                dmin = min(d, dmin)
                dmax = max(d, dmax)

            height = dmax - dmin
            dmin -= 0.1 * height
            dmax += 0.1 * height
            height = dmax - dmin
            point = center + dmin * axis

            self.shape = Gmake_elliptic_cylinder(point, axis, radii[1], radii[0], rAxes[1], rAxes[0], height)
        else:
            height = axis.length
            self.shape = Gmake_elliptic_cylinder(center, axis / height, radii[1], radii[0], rAxes[1], rAxes[0], height)


class HyperbolicCylinder:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "cylinder_hyperbolic"
        self.id = Id
        self.shape = None
        self.params = params
        if params[2][0] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][0]}")
        if params[2][1] <= 0:
            print(f"{self.type} surface {label} has a bad radius value: {params[2][1]}")

        if tr is not None:
            self.transform(tr)

    def copy(self):
        return HyperbolicCylinder(self.label, self.id, self.params)

    def transform(self, matrix):
        p, v, radii, raxes = self.params
        v = matrix_rotate_vec(matrix, v)
        raxes = [matrix_rotate_vec(matrix, raxes[0]), matrix_rotate_vec(matrix, raxes[1])]
        p = matrix_multVec(matrix, p)
        self.params = (p, v, radii, raxes)

    def buildShape(self, boundBox):
        center, axis, radii, rAxes = self.params

        dmin = axis.dot(boundBox.get_point(0) - center)
        dmax = dmin
        for i in range(1, 8):
            d = axis.dot(boundBox.get_point(i) - center)
            dmin = min(d, dmin)
            dmax = max(d, dmax)

        height = dmax - dmin
        dmin -= 0.1 * height
        dmax += 0.1 * height
        height = dmax - dmin
        point = center + dmin * axis

        self.shape = Gmake_hyperbolic_cylinder(point, axis, radii[1], radii[0], rAxes[1], rAxes[0], height)


class Paraboloid:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "paraboloid"
        self.id = Id
        self.shape = None
        self.params = params
        if params[2] == 0:
            print(f"{self.type} surface {label} has a zero focal")

        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Paraboloid(self.label, self.id, self.params)

    def transform(self, matrix):
        p, v, focal = self.params
        v = matrix_rotate_vec(matrix, v)
        p = matrix_multVec(matrix, p)
        self.params = (p, v, focal)

    def buildShape(self, boundBox):
        center, axis, focal = self.params
        axis = axis.normalized()
        self.params = (center, axis, focal)

        dist = []
        for i in range(8):
            d = axis.dot(boundBox.get_point(i) - center)
            dist.append(d)
        dist.sort()
        dmin, dmax = dist[0], dist[-1]
        if dmax <= 0:
            return
        if dmin < 0:
            dmin = 0

        rmin = math.sqrt(4 * focal * dmin)
        rmax = math.sqrt(4 * focal * dmax)

        if (rmax - rmin) / rmax < 0.01:
            r = 0.5 * (rmin + rmax)
            self.shape = Gmake_cylinder(center, axis, r, dmax)
        else:
            self.shape = Gmake_paraboloid(center, axis, focal, dmax)


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

        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Torus(self.label, self.id, self.params)

    def transform(self, matrix):
        p, v, Ra, Rb, Rc = self.params
        v = matrix_rotate_vec(matrix, v)
        p = matrix_multVec(matrix, p)
        self.params = (p, v, Ra, Rb, Rc)

    def buildShape(self, boundBox):
        center, axis, Ra, Rb, Rc = self.params  # Ra distance from torus axis; R radius of toroidal-cylinder
        if (abs(Rb - Rc) < 1e-5) and Ra > 0:
            self.shape = Gmake_torus(center, axis, Ra, Rb)  # circular Torus
        else:
            self.shape = Gmake_torus_elliptic(center, axis, Ra, Rb, Rc)  # elliptic Torus


class Box:
    def __init__(self, label, Id, params, tr=None):
        self.label = label
        self.type = "box"
        self.id = Id
        self.shape = None
        self.params = params
        if params[1].length <= 0:
            print(f"{self.type} surface {label} has a bad X dimension: {params[1]}")
        if params[2].length <= 0:
            print(f"{self.type} surface {label} has a bad Y dimension: {params[2]}")
        if params[3].length <= 0:
            print(f"{self.type} surface {label} has a bad Z dimension: {params[3]}")

        if tr is not None:
            self.transform(tr)

    def copy(self):
        return Box(self.label, self.id, self.params)

    def transform(self, matrix):
        # NOTE: uses the full affine matrix_multVec (translation included)
        # for v1/v2/v3, exactly matching the pre-migration original's own
        # `matrix.multVec(v1)` (not `.submatrix(3).multVec(v1)`) -- almost
        # certainly a pre-existing bug (these are edge *direction* vectors,
        # which should only rotate, not translate) but ported as-is per
        # this migration's "bugs get fixed in a later pass" rule, not
        # silently corrected here.
        p, v1, v2, v3 = self.params
        p = matrix_multVec(matrix, p)
        v1 = matrix_multVec(matrix, v1)
        v2 = matrix_multVec(matrix, v2)
        v3 = matrix_multVec(matrix, v3)
        self.params = (p, v1, v2, v3)

    def buildShape(self, boundBox):
        p, v1, v2, v3 = self.params
        a1 = v1.normalized()
        a2 = v2.normalized()
        a3 = v3.normalized()

        m = np.array(
            [
                [a1.x, a2.x, a3.x, p.x],
                [a1.y, a2.y, a3.y, p.y],
                [a1.z, a2.z, a3.z, p.z],
                [0, 0, 0, 1],
            ],
            dtype=float,
        )
        box = Gmake_box(0, 0, 0, v1.length, v2.length, v3.length)
        self.shape = transform_solid(box, m)


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
