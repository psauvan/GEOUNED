import math

import numpy as np

from .CAD.buildSolidCell import BuildSolid
from .MCNP_parser.remh import Cline
from .Utils.booleanFunction import BoolSequence, outer_terms, remove_surf
from .Utils.boundBox import solid_plane_box, myBox, BoxSettings, makePlane
from . import (
    Gmake_elliptic_cone,
    Gmake_elliptic_cylinder,
    Gmake_ellipsoid,
    Gmake_hyperbolic_cylinder,
    Gmake_hyperbolic_prism,
    Gmake_hyperboloid,
    Gmake_paraboloid,
    Gmake_torus_elliptic,
)
from ...geo import (
    GBoundBox,
    Gfuse_solids,
    Gmake_box,
    Gmake_cone,
    Gmake_cone_double_sheet,
    Gmake_cone_frustum,
    Gmake_cylinder,
    Gmake_sphere,
    Gmake_torus,
)
from .Utils.matrix_utils import matrix_multVec, matrix_rotate_vec, transform_solid


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
        # self.surfaceList may be a tuple (Cline.get_surfaces_numbers, before
        # the definition is converted to BoolSequence) or a set (BoolSequence
        # .get_surfaces_numbers, after) -- neither the old [:] slice (sets
        # don't support it) nor a plain .copy() (tuples don't have one) works
        # for both, so rebuild the same container type explicitly.
        cpCell.surfaceList = type(self.surfaceList)(self.surfaceList)
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
        self.shape = Gfuse_solids(cutShape)

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
        for s in undefined:
            remove_surf(self.definition, s)

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

        if onesht:
            # Fixed 2026-09-14: `axis` here is `get_hyperboloid_parameters`'s
            # own "majorAxis" (the algebraically distinct/odd-sign-out
            # eigenvector) -- for the SAME hyperbola, revolving around this
            # axis gives a 2-sheet hyperboloid when `onesht` is False, but
            # gives the connected one-sheet "hourglass" when `onesht` is
            # True (both come from the same curve; only the axis role
            # differs, per direct user design -- see the history log for
            # the empirical derivation with known analytic examples).
            # `Gmake_hyperboloid`'s own construction technique (a vertex
            # ON the revolution axis, single branch capped at one end)
            # is only valid for the 2-sheet/transverse-axis case; the
            # one-sheet/conjugate-axis case has no on-axis vertex at all
            # (the waist, at v=0, already has nonzero radius) and needs
            # `Gmake_hyperbolic_cylinder`'s own waist-based technique
            # instead. In this branch, `radii`/`rAxes` (classifier fields
            # still named "minor"/"major" from the generic iaxis
            # convention) are exactly swapped relative to their usual
            # roles: `radii[1]`/`rAxes[1]` (classifier's "majorAxis") is
            # the true CONJUGATE axis here (the one to revolve around),
            # and `radii[0]`/`rAxes[0]` (classifier's "minorAxis") is the
            # true TRANSVERSE axis (the waist-radius direction) --
            # confirmed empirically against known analytic one-sheet
            # examples (see the history log). `dmin`/`dmax` above are
            # already projected onto `axis` (== the true revolution axis
            # here), matching `Gmake_hyperbolic_cylinder`'s own v_min/
            # height convention directly, letting one continuous revolve
            # cover both sides of the true waist at `center`. `boundBox`
            # itself only ever approximates the cell's real extent (via
            # `hyperboloid_to_planes`'s own faceted plane set), so unlike
            # a real boolean cut against the cell's true bounding
            # surfaces, `dmin`/`dmax` here become the SOLID's own real
            # geometric extent directly, with nothing downstream to
            # correct an under-sized approximation -- same 10% margin
            # `HyperbolicCylinder.buildShape` already applies for the
            # same reason (both consume an approximate `boundBox`).
            span = dmax - dmin
            dmin -= 0.1 * span
            dmax += 0.1 * span
            self.shape = Gmake_hyperbolic_cylinder(center, axis, radii[0], radii[1], rAxes[0], rAxes[1], dmax, v_min=dmin)
        else:
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
        minor_axis = rAxes[0]

        # `axis` (the true flat/zero-eigenvalue extrusion direction) and
        # `minor_axis` (the hyperbola profile's own conjugate direction)
        # are independent extents -- fixed 2026-09-15: `dmin`/`dmax` size
        # the EXTRUSION (along `axis`); `ymin`/`ymax`, projected onto
        # `minor_axis` instead, size how far the profile curve itself
        # must reach to cover the real cell (previously reused `height`
        # for both, which is wrong whenever the two extents genuinely
        # differ -- confirmed via a real fixture where the extrusion was
        # only 40cm but the profile needed to reach ~500cm radially).
        dmin = axis.dot(boundBox.get_point(0) - center)
        dmax = dmin
        ymin = minor_axis.dot(boundBox.get_point(0) - center)
        ymax = ymin
        for i in range(1, 8):
            p = boundBox.get_point(i) - center
            d = axis.dot(p)
            dmin = min(d, dmin)
            dmax = max(d, dmax)
            y = minor_axis.dot(p)
            ymin = min(y, ymin)
            ymax = max(y, ymax)

        height = dmax - dmin
        dmin -= 0.1 * height
        dmax += 0.1 * height
        height = dmax - dmin
        point = center + dmin * axis

        y_reach = max(abs(ymin), abs(ymax), radii[0])
        y_reach *= 1.1

        self.shape = Gmake_hyperbolic_prism(point, axis, radii[1], radii[0], rAxes[1], rAxes[0], height, y_reach)


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
        if params[2] < 0 and abs(params[2]) >= params[4]:
            # A negative major radius only has meaning for a degenerate
            # (self-intersecting) torus, where its sign selects the inner
            # sheet -- GEOUNED's own round-trip encoding convention (no
            # MCNP/OpenMC/Serpent/PHITS format has a real field for this;
            # see `geo/*/primitives.py::Gmake_torus_elliptic`'s docstring
            # and `GEOUNED/write/functions.py`'s `radMaj *= surf.a_sign`),
            # not a general negative-value error. Outside the degenerate
            # case (abs(Ra) >= minor_radius_a, i.e. params[4]) the sign is
            # meaningless, so a negative value there really is bad.
            print(f"{self.type} surface {label} has a negative major radius: {params[2]}")
        if params[3] <= 0:
            print(f"{self.type} surface {label} has a bad minor radius b value: {params[3]}")
        if params[4] <= 0:
            print(f"{self.type} surface {label} has a bad minor radius a value: {params[4]}")

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
        circular = abs(Rb - Rc) < 1e-5
        degenerate = abs(Ra) < max(Rb, Rc)
        if circular and Ra > 0 and not degenerate:
            self.shape = Gmake_torus(center, axis, Ra, Rb)  # circular, non-degenerate torus
        else:
            # Gmake_torus_elliptic(center, axis, major_radius, minor_radius_a,
            # minor_radius_b): minor_radius_a pairs with the same (radial)
            # direction as major_radius/Ra itself -- that's Rc here, not Rb
            # (see Gmake_torus_elliptic's own docstring in geo/*/primitives.py).
            #
            # Fixed 2026-09-16: a DEGENERATE circular torus (abs(Ra) <
            # Rb == Rc) used to also take the `Gmake_torus` branch above
            # (only `Ra > 0` was checked, not degeneracy) -- but
            # `Gmake_torus`'s own plain, non-degenerate-aware
            # `BRepPrimAPI_MakeTorus` builds the FULL, self-intersecting
            # double-sheet torus (both inner and outer lobes merged into
            # one ambiguous solid), not the correct single sheet
            # `Gmake_torus_elliptic` already builds for the elliptic
            # degenerate case via its own arc-splitting technique.
            # Confirmed via a real fixture (a=b=50cm degenerate circular
            # torus, R=30cm, "outer"): `Gsplit`'s own boolean cut against
            # this self-intersecting solid picked an interior point AT
            # the origin for what should have been the "outside" piece
            # (the self-intersection makes the naive solid's own "hole"
            # ambiguous), so `surface_side` (correctly testing membership
            # against the *intended* single-sheet surface) misclassified
            # it as also "inside", and the two pieces got fused back into
            # the uncut box.
            self.shape = Gmake_torus_elliptic(center, axis, Ra, Rc, Rb)  # elliptic OR degenerate circular torus


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
