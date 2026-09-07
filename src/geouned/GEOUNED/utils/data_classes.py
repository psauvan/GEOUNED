import typing
from numbers import Real

from ...geo import CAD_ENGINE


class Options:
    """A class for containing conversion options

    Args:
        forceCylinder (bool, optional): Use cylinder (instead of cones) as
            ancillary surface where unclosed torus surfaces are involved in
            the solid definition. Defaults to False.
        newSplitPlane (bool, optional): New method to consider plane as
            cutting surface during the decomposition process. Former method
            split first planes perpendicular to X,Y,Z axis and then the
            other planes involved in the solid definition. New method group
            all parallel planes independently whether their normal are
            along X,Y,Z axes, and start the decomposition process cutting
            first with the group having the highest number of parallel
            planes. Defaults to True.
        delLastNumber (bool, optional): Deleting the last word in the
            comment if it is a number. Defaults to False.
        enlargeBox (Real, optional): Enlarge box boundary when evaluating
            the constraint table during the simplification of the void cell
            definition. (unit is millimeter). Defaults to 2.
        nPlaneReverse (int, optional): Threshold value to determine whether
            cut with parallel planes should be carried out first. Defaults
            to 0.
        splitTolerance (Real, optional): Fuzzy tolerance value used in the
            solid-splitting call during decomposition (FreeCAD's
            “BOPTools.SplitAPI.slice”, or the pyOCC/OCP “BOPAlgo_Splitter”
            equivalent). Defaults to `None`, which resolves per engine:
            `0.0` under FreeCAD, `1e-4` under `occ`/`ocp`. The pyOCC-family
            default is not `0.0` like FreeCAD's because OCCT 7.9.x's own
            internal fuzzy tolerance (used when none is set explicitly) is
            measurably more sensitive to spurious near-tangencies than
            FreeCAD's bundled OCCT 7.8.1 -- confirmed on a real case
            (hylife-v06.stp solid 17, 2026-08-16): a candidate cut that
            FreeCAD recognizes as a no-op in ~1s took ~18s under `occ`/`ocp`
            at the implicit zero tolerance (computing, then discarding, 13
            genuinely near-zero-volume spurious fragments), and dropped to
            ~0.2s -- faster than FreeCAD -- once an explicit `1e-4`
            tolerance was set; verified against the full 50-file
            `tests/test_cadtocsg.py` corpus with zero regressions.
        scaleUp (bool, optional): Scale up Fuzzy tolerance once get below
            1e-12. Defaults to True.
        quadricPY (bool, optional): In openMC python script format, the
            cones or cylinders no aligned with the X,Y, or Z axis can be
            defined using the openmc.Cone or open.Cylinder methods but can
            also be defined with their quadric parameter. If “quadricPY” is
            11 True then all cones and cylinders will be defined in the
            openMC python script format under their quadric form. Defaults
            to False.
        forceNoOverlap (bool, optional): force no overlaping cell
            definition. Adjacent cell definition are rested from current
            cell definition. Defaults to False.
        n_thread (int, optional): number of threads used for decomposition
            and cell definition building. Experimental doesn't give signicant
            improvements until freecad is base on python version > 3.12.
            Defaults to 1.
    """

    def __init__(
        self,
        forceCylinder: bool = False,
        newSplitPlane: bool = True,
        delLastNumber: bool = False,
        enlargeBox: Real = 2.0,
        nPlaneReverse: int = 0,
        splitTolerance: typing.Optional[Real] = None,
        scaleUp: bool = True,
        quadricPY: bool = False,
        Facets: bool = False,
        prnt3PPlane: bool = False,
        forceNoOverlap: bool = False,
        n_thread: int = 1,
    ):

        self.forceCylinder = forceCylinder
        self.newSplitPlane = newSplitPlane
        self.delLastNumber = delLastNumber
        self.enlargeBox = enlargeBox
        self.nPlaneReverse = nPlaneReverse
        if splitTolerance is None:
            splitTolerance = 1e-4 if CAD_ENGINE in ("occ", "ocp") else 0.0
        self.splitTolerance = splitTolerance
        self.scaleUp = scaleUp
        self.quadricPY = quadricPY
        self.Facets = Facets
        self.prnt3PPlane = prnt3PPlane
        self.forceNoOverlap = forceNoOverlap
        self.n_thread = n_thread

    @property
    def forceCylinder(self):
        return self._forceCylinder

    @forceCylinder.setter
    def forceCylinder(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.forceCylinder should be a bool, not a {type(value)}")
        self._forceCylinder = value

    @property
    def newSplitPlane(self):
        return self._newSplitPlane

    @newSplitPlane.setter
    def newSplitPlane(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.newSplitPlane should be a bool, not a {type(value)}")
        self._newSplitPlane = value

    @property
    def delLastNumber(self):
        return self._delLastNumber

    @delLastNumber.setter
    def delLastNumber(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.delLastNumber should be a bool, not a {type(value)}")
        self._delLastNumber = value

    @property
    def enlargeBox(self):
        return self._enlargeBox

    @enlargeBox.setter
    def enlargeBox(self, value: Real):
        if not isinstance(value, Real):
            raise TypeError(f"geouned.Options.enlargeBox should be a Real, not a {type(value)}")
        if value < 0:
            raise ValueError(f"geouned.Options.enlargeBox should be above 0, not {value}")
        self._enlargeBox = value

    @property
    def nPlaneReverse(self):
        return self._nPlaneReverse

    @nPlaneReverse.setter
    def nPlaneReverse(self, value: int):
        if not isinstance(value, int):
            raise TypeError(f"geouned.Options.nPlaneReverse should be a int, not a {type(value)}")
        self._nPlaneReverse = value

    @property
    def splitTolerance(self):
        return self._splitTolerance

    @splitTolerance.setter
    def splitTolerance(self, value: Real):
        if not isinstance(value, Real):
            raise TypeError(f"geouned.Options.splitTolerance should be a Real, not a {type(value)}")
        if value < 0:
            raise ValueError(f"geouned.Options.splitTolerance should be above 0, not {value}")
        self._splitTolerance = value

    @property
    def scaleUp(self):
        return self._scaleUp

    @scaleUp.setter
    def scaleUp(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.scaleUp should be a bool, not a {type(value)}")
        self._scaleUp = value

    @property
    def quadricPY(self):
        return self._quadricPY

    @quadricPY.setter
    def quadricPY(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.quadricPY should be a bool, not a {type(value)}")
        self._quadricPY = value

    @property
    def Facets(self):
        return self._Facets

    @Facets.setter
    def Facets(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.Facets should be a bool, not a {type(value)}")
        self._Facets = value

    @property
    def prnt3PPlane(self):
        return self._prnt3PPlane

    @prnt3PPlane.setter
    def prnt3PPlane(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.prnt3PPlane should be a bool, not a {type(value)}")
        self._prnt3PPlane = value

    @property
    def forceNoOverlap(self):
        return self._forceNoOverlap

    @forceNoOverlap.setter
    def forceNoOverlap(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Options.forceNoOverlap should be a bool, not a {type(value)}")
        self._forceNoOverlap = value

    @property
    def n_thread(self):
        return self._n_thread

    @n_thread.setter
    def n_thread(self, value: int):
        if not isinstance(value, int):
            raise TypeError(f"geouned.Options.n_thread should be a int, not a {type(value)}")
        self._n_thread = value


class Tolerances:
    """A class for containing tolerances values

    Args:
        relativeTol (bool, optional): _description_. Defaults to False.
        relativePrecision (float, optional): relative precision. Defaults to 1.0e-6.
        value (float, optional): Tolerance in single value comparison. Defaults to 1.0e-6.
        distance (float, optional): General Distance Tolerance. Defaults to 1.0e-4.
        angle (float, optional): General Angle Tolerance. Defaults to 1.0e-4.
        pln_distance (float, optional): distance between planes equal planes if distance between parallel planes < 1e-4 cm. Defaults to 1.0e-4.
        pln_angle (float, optional): angle between axis. 1e-4 : planes separate each other 0.1mm each 1m. Defaults to 1.0e-4.
        cyl_distance (float, optional): distance between radius/center. Defaults to 1.0e-4.
        cyl_angle (float, optional): angle between axis. Defaults to 1.0e-4.
        sph_distance (float, optional): distance between radius/center. Defaults to 1.0e-4.
        kne_distance (float, optional): distance between apex. Defaults to 1.0e-4.
        kne_angle (float, optional): angle between semiangles/axis. Defaults to 1.0e-4.
        tor_distance (float, optional): distance between Major/Minor radii/center. Defaults to 1.0e-4.
        tor_angle (float, optional): angle between axis. Defaults to 1.0e-4.
        min_area (float, optional): minimum face area to consider in cell definition. Defaults to 1.0e-2.
        min_face_width (float, optional): minimum characteristic width (the face's own true short physical
            dimension, sqrt(Area*Compactness/12) -- correctly recovers this for both straight and curved
            slivers, unlike Area alone, which a thin-but-long real feature can also satisfy by coincidence)
            below which a face is treated as a residual boolean-cut sliver rather than a real feature,
            regardless of its raw Area. Defaults to 0.1 (mm) -- verified, 2026-08-23, against a 109-file/
            1733-face corpus: every known real face's own characteristic width is >=0.19mm and every known
            sliver's is <=0.055mm, a clean >0.5-decade gap with 0.1 sitting in the middle.
        sliver_edge_rel_tol (float, optional): threshold, as a fraction of a solid's own BoundBox
            diagonal, below which an edge length is treated as a purely topological signature of a
            corrupted/spurious CAD feature (see geo.find_short_edges) -- unlike min_area/min_face_width,
            this is already relative to each solid's own scale, so it needs no scaled() accommodation.
            Defaults to 1.0e-4 (0.01% of the model's own diagonal, per direct user instruction: real
            models can be meter-scale with legitimate millimeter-scale details) -- verified, 2026-08-27,
            with zero false positives across a 109-file test_models corpus scan (raw solids and decomposed
            pieces alike), including on a fixture with independently-documented real ~0.026mm-wide faces
            that this value correctly leaves alone (see geo.find_short_edges' own docstring for the full
            story). geo.find_short_edges also enforces an absolute floor, MIN_SLIVER_EDGE_LENGTH=1e-3mm,
            so this relative value alone never needs to account for very small solids either.
        split_tolerance (float, optional): geo.Gsplit's own BOPAlgo_Splitter fuzzy tolerance --
            previously sourced ad hoc from Options.splitTolerance at each Gsplit call site (a plain
            float, not carried on Tolerances at all). Defaults to None, resolved the same way
            Options.splitTolerance's own default is: 1.0e-4 under the occ/ocp engines, 0.0 under freecad
            (see CAD_ENGINE, already imported in this module for exactly this) -- PROVISIONAL: this field
            is not yet wired to Options.splitTolerance itself at any call site (2026-08-30, added as part
            of Gsplit's own tolerances-object refactor; deferred question, not yet resolved, whether/where
            a Tolerances instance should pick up the user's own Options.splitTolerance value instead of
            resolving its own default independently).
        scale_up_floor (float | None, optional): geo.Gsplit's own scale-up-floor retry bound (see
            Options.scaleUp's own docstring for the mechanism) -- same PROVISIONAL status as
            split_tolerance above. Defaults to None (no floor), matching the prior default.
        scale (float, optional): geo.Gsplit's own per-retry tolerance scale-up factor. Defaults to 0.1,
            matching the prior hardcoded default on Gsplit's own (now-retired) `scale` parameter.
        min_solid_volume (float, optional): geo.Gsplit's own minimum-volume filter -- a split-result
            fragment whose |Volume| falls at or below this is discarded rather than returned as a real
            solid. A NEW filter (no prior equivalent existed before this same 2026-08-30 refactor) --
            PROVISIONAL default of 1.0e-6 (mm^3), picked only to filter genuinely near-zero-volume
            numerical-noise fragments, not real slivers (see min_face_width/sliver_edge_rel_tol above for
            those) -- not yet independently verified against a real corpus the way this file's other
            defaults are.
        fix_tolerance (float, optional): geo._repair_non_manifold_solid's own BRepBuilderAPI_Sewing
            tolerance when reconstructing a non-manifold solid's real connected components. Defaults to
            1.0e-6, matching that function's own prior hardcoded value exactly (this is a pure
            parameterization, not a new default -- see that function's own docstring).
        volume_tolerance (float, optional): relative volume-conservation tolerance used by geo.Gsplit's
            own non-manifold-repair/sliver-heal acceptance checks (a repaired result is only trusted if
            its own summed volume matches the pre-repair input to within this fraction). Defaults to
            1.0e-6, matching the prior hardcoded value in geo._raw_bop_split (see that function's own
            docstring, and _try_coaxial_cone_split's identical, separately-hardcoded 1e-6 -- both trace to
            the same "never trust a topology repair blindly" discipline documented throughout this
            project's history).
    """

    # Reference length (mm, GEOUNED's own internal unit) used by scaled()
    # below the min_area/min_face_width defaults are considered correctly
    # calibrated -- 1cm, MCNP's own natural length unit, per direct user
    # choice. Deliberately a class constant, not a constructor parameter:
    # unlike min_area/min_face_width themselves (real per-run tuning
    # knobs), this is calibration machinery for scaled() -- not something
    # a user is expected to tune per run.
    SCALE_REFERENCE_LENGTH = 10.0

    def __init__(
        self,
        relativeTol: bool = False,
        relativePrecision: float = 1.0e-6,
        value: float = 1.0e-6,
        distance: float = 1.0e-4,
        angle: float = 1.0e-4,
        pln_distance: float = 1.0e-4,
        pln_angle: float = 1.0e-4,
        cyl_distance: float = 1.0e-4,
        cyl_angle: float = 1.0e-4,
        sph_distance: float = 1.0e-4,
        kne_distance: float = 1.0e-4,
        kne_angle: float = 1.0e-4,
        tor_distance: float = 1.0e-4,
        tor_angle: float = 1.0e-4,
        min_area: float = 1.0e-2,
        add_pln_distance: float = 1.0e-2,
        add_pln_angle: float = 1.0e-2,
        min_face_width: float = 0.1,
        sliver_edge_rel_tol: float = 1.0e-4,
        split_tolerance: typing.Optional[float] = 1.e-6,
        scale_up_floor: typing.Optional[float] = 1e-12,
        scale: float = 0.1,
        min_solid_volume: float = 1.0e-3,
        fix_tolerance: float = 1.0e-6,
        volume_tolerance: float = 1.0e-6,
    ):

        self.relativeTol = relativeTol
        self.relativePrecision = relativePrecision
        self.value = value
        self.distance = distance
        self.angle = angle
        self.pln_distance = pln_distance
        self.pln_angle = pln_angle
        self.cyl_distance = cyl_distance
        self.cyl_angle = cyl_angle
        self.sph_distance = sph_distance
        self.kne_distance = kne_distance
        self.kne_angle = kne_angle
        self.tor_distance = tor_distance
        self.tor_angle = tor_angle
        self.min_area = min_area
        self.add_pln_distance = add_pln_distance
        self.add_pln_angle = add_pln_angle
        self.min_face_width = min_face_width
        self.sliver_edge_rel_tol = sliver_edge_rel_tol
        if split_tolerance is None:
            split_tolerance = 1.0e-4 if CAD_ENGINE in ("occ", "ocp") else 0.0
        self.split_tolerance = split_tolerance
        self.scale_up_floor = scale_up_floor
        self.scale = scale
        self.min_solid_volume = min_solid_volume
        self.fix_tolerance = fix_tolerance
        self.volume_tolerance = volume_tolerance

    @property
    def relativeTol(self):
        return self._relativeTol

    @relativeTol.setter
    def relativeTol(self, value: bool):
        if not isinstance(value, bool):
            raise TypeError(f"geouned.Tolerances.relativeTol should be a bool, not a {type(value)}")
        self._relativeTol = value

    @property
    def relativePrecision(self):
        return self._relativePrecision

    @relativePrecision.setter
    def relativePrecision(self, value: float):
        if not isinstance(value, float):
            raise TypeError(f"geouned.Tolerances.relativePrecision should be a float, not a {type(value)}")
        self._relativePrecision = value

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value: float):
        if not isinstance(value, float):
            raise TypeError(f"geouned.Tolerances.value should be a float, not a {type(value)}")
        self._value = value

    @property
    def distance(self):
        return self._distance

    @distance.setter
    def distance(self, distance: float):
        if not isinstance(distance, float):
            raise TypeError(f"geouned.Tolerances.distance should be a float, not a {type(distance)}")
        self._distance = distance

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle: float):
        if not isinstance(angle, float):
            raise TypeError(f"geouned.Tolerances.angle should be a float, not a {type(angle)}")
        self._angle = angle

    @property
    def pln_distance(self):
        return self._pln_distance

    @pln_distance.setter
    def pln_distance(self, pln_distance: float):
        if not isinstance(pln_distance, float):
            raise TypeError(f"geouned.Tolerances.pln_distance should be a float, not a {type(pln_distance)}")
        self._pln_distance = pln_distance

    @property
    def cyl_distance(self):
        return self._cyl_distance

    @cyl_distance.setter
    def cyl_distance(self, cyl_distance: float):
        if not isinstance(cyl_distance, float):
            raise TypeError(f"geouned.Tolerances.cyl_distance should be a float, not a {type(cyl_distance)}")
        self._cyl_distance = cyl_distance

    @property
    def cyl_angle(self):
        return self._cyl_angle

    @cyl_angle.setter
    def cyl_angle(self, cyl_angle: float):
        if not isinstance(cyl_angle, float):
            raise TypeError(f"geouned.Tolerances.cyl_angle should be a float, not a {type(cyl_angle)}")
        self._cyl_angle = cyl_angle

    @property
    def sph_distance(self):
        return self._sph_distance

    @sph_distance.setter
    def sph_distance(self, sph_distance: float):
        if not isinstance(sph_distance, float):
            raise TypeError(f"geouned.Tolerances.sph_distance should be a float, not a {type(sph_distance)}")
        self._sph_distance = sph_distance

    @property
    def pln_angle(self):
        return self._pln_angle

    @pln_angle.setter
    def pln_angle(self, pln_angle: float):
        if not isinstance(pln_angle, float):
            raise TypeError(f"geouned.Tolerances.pln_angle should be a float, not a {type(pln_angle)}")
        self._pln_angle = pln_angle

    @property
    def kne_distance(self):
        return self._kne_distance

    @kne_distance.setter
    def kne_distance(self, kne_distance: float):
        if not isinstance(kne_distance, float):
            raise TypeError(f"geouned.Tolerances.kne_distance should be a float, not a {type(kne_distance)}")
        self._kne_distance = kne_distance

    @property
    def kne_angle(self):
        return self._kne_angle

    @kne_angle.setter
    def kne_angle(self, kne_angle: float):
        if not isinstance(kne_angle, float):
            raise TypeError(f"geouned.Tolerances.kne_angle should be a float, not a {type(kne_angle)}")
        self._kne_angle = kne_angle

    @property
    def tor_distance(self):
        return self._tor_distance

    @tor_distance.setter
    def tor_distance(self, tor_distance: float):
        if not isinstance(tor_distance, float):
            raise TypeError(f"geouned.Tolerances.tor_distance should be a float, not a {type(tor_distance)}")
        self._tor_distance = tor_distance

    @property
    def tor_angle(self):
        return self._tor_angle

    @tor_angle.setter
    def tor_angle(self, tor_angle: float):
        if not isinstance(tor_angle, float):
            raise TypeError(f"geouned.Tolerances.tor_angle should be a float, not a {type(tor_angle)}")
        self._tor_angle = tor_angle

    @property
    def add_pln_distance(self):
        return self._add_pln_distance

    @add_pln_distance.setter
    def add_pln_distance(self, add_pln_distance: float):
        if not isinstance(add_pln_distance, float):
            raise TypeError(f"geouned.Tolerances.add_pln_distance should be a float, not a {type(add_pln_distance)}")
        self._add_pln_distance = add_pln_distance

    @property
    def add_pln_angle(self):
        return self._add_pln_angle

    @add_pln_angle.setter
    def add_pln_angle(self, add_pln_angle: float):
        if not isinstance(add_pln_angle, float):
            raise TypeError(f"geouned.Tolerances.add_pln_angle should be a float, not a {type(add_pln_angle)}")
        self._add_pln_angle = add_pln_angle

    @property
    def min_area(self):
        return self._min_area

    @min_area.setter
    def min_area(self, min_area: float):
        if not isinstance(min_area, float):
            raise TypeError(f"geouned.Tolerances.min_area should be a float, not a {type(min_area)}")
        self._min_area = min_area

    @property
    def min_face_width(self):
        return self._min_face_width

    @min_face_width.setter
    def min_face_width(self, min_face_width: float):
        if not isinstance(min_face_width, float):
            raise TypeError(f"geouned.Tolerances.min_face_width should be a float, not a {type(min_face_width)}")
        self._min_face_width = min_face_width

    @property
    def sliver_edge_rel_tol(self):
        return self._sliver_edge_rel_tol

    @sliver_edge_rel_tol.setter
    def sliver_edge_rel_tol(self, sliver_edge_rel_tol: float):
        if not isinstance(sliver_edge_rel_tol, float):
            raise TypeError(f"geouned.Tolerances.sliver_edge_rel_tol should be a float, not a {type(sliver_edge_rel_tol)}")
        self._sliver_edge_rel_tol = sliver_edge_rel_tol

    @property
    def split_tolerance(self):
        return self._split_tolerance

    @split_tolerance.setter
    def split_tolerance(self, split_tolerance: float):
        if not isinstance(split_tolerance, float):
            raise TypeError(f"geouned.Tolerances.split_tolerance should be a float, not a {type(split_tolerance)}")
        self._split_tolerance = split_tolerance

    @property
    def scale_up_floor(self):
        return self._scale_up_floor

    @scale_up_floor.setter
    def scale_up_floor(self, scale_up_floor: typing.Optional[float]):
        if scale_up_floor is not None and not isinstance(scale_up_floor, float):
            raise TypeError(f"geouned.Tolerances.scale_up_floor should be a float or None, not a {type(scale_up_floor)}")
        self._scale_up_floor = scale_up_floor

    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, scale: float):
        if not isinstance(scale, float):
            raise TypeError(f"geouned.Tolerances.scale should be a float, not a {type(scale)}")
        self._scale = scale

    @property
    def min_solid_volume(self):
        return self._min_solid_volume

    @min_solid_volume.setter
    def min_solid_volume(self, min_solid_volume: float):
        if not isinstance(min_solid_volume, float):
            raise TypeError(f"geouned.Tolerances.min_solid_volume should be a float, not a {type(min_solid_volume)}")
        self._min_solid_volume = min_solid_volume

    @property
    def fix_tolerance(self):
        return self._fix_tolerance

    @fix_tolerance.setter
    def fix_tolerance(self, fix_tolerance: float):
        if not isinstance(fix_tolerance, float):
            raise TypeError(f"geouned.Tolerances.fix_tolerance should be a float, not a {type(fix_tolerance)}")
        self._fix_tolerance = fix_tolerance

    @property
    def volume_tolerance(self):
        return self._volume_tolerance

    @volume_tolerance.setter
    def volume_tolerance(self, volume_tolerance: float):
        if not isinstance(volume_tolerance, float):
            raise TypeError(f"geouned.Tolerances.volume_tolerance should be a float, not a {type(volume_tolerance)}")
        self._volume_tolerance = volume_tolerance

    def scaled(self, volume: float) -> "Tolerances":
        """Returns a copy of this Tolerances with min_area/min_face_width
        scaled down for a solid whose own Volume is small relative to
        GEOUNED's internal mm units -- never scaled up. min_area/
        min_face_width are absolute thresholds calibrated for "normal"
        solids; a genuinely small decomposed piece (e.g. Volume < 1 mm^3)
        can have real, legitimate faces whose own area/width falls below
        these defaults, causing them to be wrongly dropped as slivers
        (confirmed live, 2026-08-24,
        Solidos/test_models/Decomposed/modelcell_cut1_v2_piece66.stp:
        Volume=0.072mm^3, 2 real lateral faces with CharacteristicWidth
        =0.026mm, below the default min_face_width=0.1mm -- dropping
        them left a real gap in the cell definition, causing MCNP lost
        particles).

        Reference length SCALE_REFERENCE_LENGTH=10mm (1cm, GEOUNED's own
        internal unit is mm, and 1cm is the natural MCNP length unit the
        user chose as the scale at which the absolute defaults are
        considered correctly calibrated -- a class constant, not a
        constructor parameter, since it's calibration machinery for this
        method rather than a per-run tuning knob like min_area/
        min_face_width themselves). L = Volume**(1/3), a pure volumetric
        length scale --
        deliberately NOT combined with the solid's own BoundBox extent
        (its "elongation"): the failure mode being guarded against is
        real face *area* falling below an absolute threshold, which
        tracks the solid's own volume (Volume ~= typical face area x
        typical thickness), not its spatial reach. A highly elongated
        but small-volume sliver (exactly the piece66 case: BoundBox
        diagonal ~5mm despite Volume^(1/3) ~0.4mm) would fail to trigger
        scaling at all if diagonal were used instead or combined as an
        additional gate -- confirmed by direct calculation on piece66's
        own numbers before choosing this formula.

        min_face_width scales linearly with L (both are lengths);
        min_area scales with L^2 (an area) -- dimensionally consistent,
        derived from the same single length scale. Both are `min()`-ed
        against the original value, so a solid whose own L >=
        SCALE_REFERENCE_LENGTH gets back the unmodified original
        tolerances -- zero behavior change for every solid at or above
        "normal" scale.

        `volume` is the caller's raw `GSolid.Volume`, which is *signed*
        (negative for a reversed-orientation solid) -- take its magnitude
        before the cube root, or `(-x) ** (1/3)` returns a Python
        `complex` and the `min()` below raises `TypeError`."""
        length_scale = min(1.0, abs(volume) ** (1.0 / 3.0) / self.SCALE_REFERENCE_LENGTH)
        scaled_min_area = min(self.min_area, self.min_area * length_scale**2)
        scaled_min_face_width = min(self.min_face_width, self.min_face_width * length_scale)
        return Tolerances(
            relativeTol=self.relativeTol,
            relativePrecision=self.relativePrecision,
            value=self.value,
            distance=self.distance,
            angle=self.angle,
            pln_distance=self.pln_distance,
            pln_angle=self.pln_angle,
            cyl_distance=self.cyl_distance,
            cyl_angle=self.cyl_angle,
            sph_distance=self.sph_distance,
            kne_distance=self.kne_distance,
            kne_angle=self.kne_angle,
            tor_distance=self.tor_distance,
            tor_angle=self.tor_angle,
            min_area=scaled_min_area,
            add_pln_distance=self.add_pln_distance,
            add_pln_angle=self.add_pln_angle,
            min_face_width=scaled_min_face_width,
            sliver_edge_rel_tol=self.sliver_edge_rel_tol,
            split_tolerance=self.split_tolerance,
            scale_up_floor=self.scale_up_floor,
            scale=self.scale,
            min_solid_volume=self.min_solid_volume,
            fix_tolerance=self.fix_tolerance,
            volume_tolerance=self.volume_tolerance,
        )


class NumericFormat:
    """Numerical format options for each of the surface types.

    Args:
        P_abc (str, optional): Plane general a,b,c params. Defaults to "14.7e".
        P_d (str, optional): Plane general d params. Defaults to "14.7e".
        P_xyz (str, optional): PX/PY/PZ params. Defaults to "14.7e".
        S_r (str, optional): SO/SX/SY/SZ/S radius. Defaults to "14.7e".
        S_xyz (str, optional): SO/SX/SY/SZ/S center. Defaults to "14.7e".
        C_r (str, optional): Cylinder radius. Defaults to "12f".
        C_xyz (str, optional): Cylinder center. Defaults to "12f".
        K_xyz (str, optional): Cone apex. Defaults to "13.6e".
        K_tan2 (str, optional): Cone tan^2 value. Defaults to "13.6e".
        T_r (str, optional): Torus radii. Defaults to "14.7e".
        T_xyz (str, optional): Torus center. Defaults to "14.7e".
        GQ_1to6 (str, optional): GQ 1 to 6 coefficients (order 2 x2,y2,z2,xy,...). Defaults to "18.15f".
        GQ_7to9 (str, optional): GQ 7 to 9 coefficients (order 1 x,y,z). Defaults to "18.15f".
        GQ_10 (str, optional): GQ 10 coefficient. Defaults to "18.15f".
    """

    def __init__(
        self,
        P_abc: str = "14.7e",
        P_d: str = "14.7e",
        P_xyz: str = "14.7e",
        S_r: str = "14.7e",
        S_xyz: str = "14.7e",
        C_r: str = "12f",
        C_xyz: str = "12f",
        K_xyz: str = "13.6e",
        K_tan2: str = "13.6e",
        T_r: str = "14.7e",
        T_xyz: str = "14.7e",
        GQ_1to6: str = "18.15f",
        GQ_7to9: str = "18.15f",
        GQ_10: str = "18.15f",
    ):

        self.P_abc = P_abc
        self.P_d = P_d
        self.P_xyz = P_xyz
        self.S_r = S_r
        self.S_xyz = S_xyz
        self.C_r = C_r
        self.C_xyz = C_xyz
        self.K_xyz = K_xyz
        self.K_tan2 = K_tan2
        self.T_r = T_r
        self.T_xyz = T_xyz
        self.GQ_1to6 = GQ_1to6
        self.GQ_7to9 = GQ_7to9
        self.GQ_10 = GQ_10

    @property
    def P_abc(self):
        return self._P_abc

    @P_abc.setter
    def P_abc(self, P_abc: str):
        if not isinstance(P_abc, str):
            raise TypeError(f"geouned.Tolerances.P_abc should be a str, not a {type(P_abc)}")
        self._P_abc = P_abc

    @property
    def P_d(self):
        return self._P_d

    @P_d.setter
    def P_d(self, P_d: str):
        if not isinstance(P_d, str):
            raise TypeError(f"geouned.Tolerances.P_d should be a str, not a {type(P_d)}")
        self._P_d = P_d

    @property
    def P_xyz(self):
        return self._P_xyz

    @P_xyz.setter
    def P_xyz(self, P_xyz: str):
        if not isinstance(P_xyz, str):
            raise TypeError(f"geouned.Tolerances.P_xyz should be a str, not a {type(P_xyz)}")
        self._P_xyz = P_xyz

    @property
    def S_r(self):
        return self._S_r

    @S_r.setter
    def S_r(self, S_r: str):
        if not isinstance(S_r, str):
            raise TypeError(f"geouned.Tolerances.S_r should be a str, not a {type(S_r)}")
        self._S_r = S_r

    @property
    def S_xyz(self):
        return self._S_xyz

    @S_xyz.setter
    def S_xyz(self, S_xyz: str):
        if not isinstance(S_xyz, str):
            raise TypeError(f"geouned.Tolerances.S_xyz should be a str, not a {type(S_xyz)}")
        self._S_xyz = S_xyz

    @property
    def C_r(self):
        return self._C_r

    @C_r.setter
    def C_r(self, C_r: str):
        if not isinstance(C_r, str):
            raise TypeError(f"geouned.Tolerances.C_r should be a str, not a {type(C_r)}")
        self._C_r = C_r

    @property
    def C_xyz(self):
        return self._C_xyz

    @C_xyz.setter
    def C_xyz(self, C_xyz: str):
        if not isinstance(C_xyz, str):
            raise TypeError(f"geouned.Tolerances.C_xyz should be a str, not a {type(C_xyz)}")
        self._C_xyz = C_xyz

    @property
    def K_xyz(self):
        return self._K_xyz

    @K_xyz.setter
    def K_xyz(self, K_xyz: str):
        if not isinstance(K_xyz, str):
            raise TypeError(f"geouned.Tolerances.K_xyz should be a str, not a {type(K_xyz)}")
        self._K_xyz = K_xyz

    @property
    def K_tan2(self):
        return self._K_tan2

    @K_tan2.setter
    def K_tan2(self, K_tan2: str):
        if not isinstance(K_tan2, str):
            raise TypeError(f"geouned.Tolerances.K_tan2 should be a str, not a {type(K_tan2)}")
        self._K_tan2 = K_tan2

    @property
    def T_r(self):
        return self._T_r

    @T_r.setter
    def T_r(self, T_r: str):
        if not isinstance(T_r, str):
            raise TypeError(f"geouned.Tolerances.T_r should be a str, not a {type(T_r)}")
        self._T_r = T_r

    @property
    def T_xyz(self):
        return self._T_xyz

    @T_xyz.setter
    def T_xyz(self, T_xyz: str):
        if not isinstance(T_xyz, str):
            raise TypeError(f"geouned.Tolerances.T_xyz should be a str, not a {type(T_xyz)}")
        self._T_xyz = T_xyz

    @property
    def GQ_1to6(self):
        return self._GQ_1to6

    @GQ_1to6.setter
    def GQ_1to6(self, GQ_1to6: str):
        if not isinstance(GQ_1to6, str):
            raise TypeError(f"geouned.Tolerances.GQ_1to6 should be a str, not a {type(GQ_1to6)}")
        self._GQ_1to6 = GQ_1to6

    @property
    def GQ_7to9(self):
        return self._GQ_7to9

    @GQ_7to9.setter
    def GQ_7to9(self, GQ_7to9: str):
        if not isinstance(GQ_7to9, str):
            raise TypeError(f"geouned.Tolerances.GQ_7to9 should be a str, not a {type(GQ_7to9)}")
        self._GQ_7to9 = GQ_7to9

    @property
    def GQ_10(self):
        return self._GQ_10

    @GQ_10.setter
    def GQ_10(self, GQ_10: str):
        if not isinstance(GQ_10, str):
            raise TypeError(f"geouned.Tolerances.GQ_10 should be a str, not a {type(GQ_10)}")
        self._GQ_10 = GQ_10


class Settings:
    """Settings for changing the way the CAD to CSG conversion is done

    Args:
        outPath (str, optional): Path to folder where output file will be
            written. If folder doesn't exist it will be created. Defaults
            to ".".
        matFile (str, optional): Name of the file in which the material
            information is provided. Defaults to "".
        voidGen (bool, optional): Generate voids of the geometry. Defaults
            to True.
        debug (bool, optional): Write step files of original and decomposed
            solids, for each solid in the STEP file. Defaults to False.
        compSolids (bool, optional): Join subsolids of STEP file as a single
            compound solid. Step files generated with SpaceClaim have not
            exactly the same level of solids as FreeCAD. It may a happened
            that solids defined has separated solids are read by FreeCAD
            as a single compound solid (and will produce only one MCNP
            cell). In this case compSolids should be set to False. Defaults
            to False.
        simplify (str, optional): Simplify the cell definition considering
            relative surfaces position and using Boolean logics. Available
            options are: "no" no optimization, "void" only void cells are
            simplified. Algorithm is faster but the simplification is not
            optimal. "voidfull" : only void cells are simplified with the
            most optimal algorithm. The time of the conversion can be
            multiplied by 5 or more. "full" : all the cells (solids and
            voids) are simplified. Defaults to "No".
        exportSolids (str, optional): Export CAD solid after reading.
            The execution is stopped after export, the translation is not
            carried out. Defaults to "".
        minVoidSize (float, optional): Minimum size of the edges of the
            void cell. Units are in mm. Defaults to 200.0.
        maxSurf (int, optional): #TODO
        maxBracket (int, optional): Maximum number of brackets (solid
            complementary) allowed in void cell definition. Defaults to 30.
        voidMat (list, optional): Assign a material defined by the user
            instead of void for cells without material definition and the
            cells generated in the automatic void generation. The format
            is a 3 valued tuple (mat_label, mat_density, mat_description).
            Example (100,1e-3,'Air assigned to Void'). Defaults to [].
        voidExclude (list, optional): #TODO see issue 87. Defaults to [].
        startCell (int, optional): Starting cell numbering label. Defaults to 1.
        startSurf (int, optional): Starting surface numbering label. Defaults to 1.
        sort_enclosure (bool, optional): If enclosures are defined in the
            CAD models, the voids cells of the enclosure will be located in
            the output file in the same location where the enclosure solid
            is located in the CAD solid tree.. Defaults to False.
    """

    def __init__(
        self,
        outPath: str = ".",
        matFile: str = "",
        voidGen: bool = True,
        debug: bool = False,
        compSolids: bool = False,
        simplify: str = "no",
        exportSolids: typing.Optional[str] = None,
        minVoidSize: float = 200.0,  # units mm
        maxSurf: int = 50,
        maxBracket: int = 30,
        voidMat: list = [],
        voidExclude: list = [],
        startCell: int = 1,
        startSurf: int = 1,
        sort_enclosure: bool = False,
    ):

        self.outPath = outPath
        self.matFile = matFile
        self.voidGen = voidGen
        self.debug = debug
        self.compSolids = compSolids
        self.simplify = simplify
        self.exportSolids = exportSolids
        self.minVoidSize = minVoidSize
        self.maxSurf = maxSurf
        self.maxBracket = maxBracket
        self.voidMat = voidMat
        self.voidExclude = voidExclude
        self.startCell = startCell
        self.startSurf = startSurf
        self.sort_enclosure = sort_enclosure

    @property
    def outPath(self):
        return self._outPath

    @outPath.setter
    def outPath(self, outPath: str):
        if not isinstance(outPath, str):
            raise TypeError(f"geouned.Settings.outPath should be a str, not a {type(outPath)}")
        self._outPath = outPath

    @property
    def matFile(self):
        return self._matFile

    @matFile.setter
    def matFile(self, matFile: str):
        if not isinstance(matFile, str):
            raise TypeError(f"geouned.Settings.matFile should be a str, not a {type(matFile)}")
        self._matFile = matFile

    @property
    def voidGen(self):
        return self._voidGen

    @voidGen.setter
    def voidGen(self, voidGen: bool):
        if not isinstance(voidGen, bool):
            raise TypeError(f"geouned.Settings.voidGen should be a bool, not a {type(voidGen)}")
        self._voidGen = voidGen

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, debug: bool):
        if not isinstance(debug, bool):
            raise TypeError(f"geouned.Settings.debug should be a bool, not a {type(debug)}")
        self._debug = debug

    @property
    def compSolids(self):
        return self._compSolids

    @compSolids.setter
    def compSolids(self, compSolids: bool):
        if not isinstance(compSolids, bool):
            raise TypeError(f"geouned.Settings.compSolids should be a bool, not a {type(compSolids)}")
        self._compSolids = compSolids

    @property
    def simplify(self):
        return self._simplify

    @simplify.setter
    def simplify(self, simplify: str):
        if not isinstance(simplify, str):
            raise TypeError(f"geouned.Settings.simplify should be a str, not a {type(simplify)}")
        self._simplify = simplify

    @property
    def exportSolids(self):
        return self._exportSolids

    @exportSolids.setter
    def exportSolids(self, exportSolids: str):
        if exportSolids == None:
            pass
        elif not isinstance(exportSolids, str):
            raise TypeError(f"geouned.Settings.exportSolids should be a str, not a {type(exportSolids)}")
        self._exportSolids = exportSolids

    @property
    def minVoidSize(self):
        return self._minVoidSize

    @minVoidSize.setter
    def minVoidSize(self, minVoidSize: float):
        if not isinstance(minVoidSize, float):
            raise TypeError(f"geouned.Settings.minVoidSize should be a float, not a {type(minVoidSize)}")
        self._minVoidSize = minVoidSize

    @property
    def maxSurf(self):
        return self._maxSurf

    @maxSurf.setter
    def maxSurf(self, maxSurf: int):
        if not isinstance(maxSurf, int):
            raise TypeError(f"geouned.Settings.maxSurf should be a int, not a {type(maxSurf)}")
        self._maxSurf = maxSurf

    @property
    def maxBracket(self):
        return self._maxBracket

    @maxBracket.setter
    def maxBracket(self, maxBracket: int):
        if not isinstance(maxBracket, int):
            raise TypeError(f"geouned.Settings.maxBracket should be a int, not a {type(maxBracket)}")
        self._maxBracket = maxBracket

    @property
    def voidMat(self):
        return self._voidMat

    @voidMat.setter
    def voidMat(self, voidMat: list):
        if not isinstance(voidMat, list):
            raise TypeError(f"geouned.Settings.voidMat should be a list, not a {type(voidMat)}")
        if len(voidMat) == 3:
            entry0, entry1, entry2 = voidMat
            if not isinstance(entry0, int):
                raise TypeError(f"first entry of geouned.Settings.voidMat should be an int, not a {type(entry0)}")
            if not isinstance(entry1, int):
                if not isinstance(entry1, float):
                    raise TypeError(f"second entry of geouned.Settings.voidMat should be an int or float, not a {type(entry1)}")
            if not isinstance(entry2, str):
                raise TypeError(f"third entry of geouned.Settings.voidMat should be a str, not a {type(entry2)}")
        elif len(voidMat) > 0:
            raise TypeError(f"geouned.Settings.voidMat should be a list with 3 elements or void list")
        self._voidMat = voidMat

    @property
    def voidExclude(self):
        return self._voidExclude

    @voidExclude.setter
    def voidExclude(self, voidExclude: list):
        if not isinstance(voidExclude, list):
            raise TypeError(f"geouned.Settings.voidExclude should be a list, not a {type(voidExclude)}")
        for entry in voidExclude:
            if not isinstance(entry, int):
                raise TypeError(f"geouned.Settings.voidExclude should be a list of ints, not a {type(entry)}")
        self._voidExclude = voidExclude

    @property
    def startCell(self):
        return self._startCell

    @startCell.setter
    def startCell(self, startCell: int):
        if not isinstance(startCell, int):
            raise TypeError(f"geouned.Settings.startCell should be a int, not a {type(startCell)}")
        self._startCell = startCell

    @property
    def startSurf(self):
        return self._startSurf

    @startSurf.setter
    def startSurf(self, startSurf: int):
        if not isinstance(startSurf, int):
            raise TypeError(f"geouned.Settings.startSurf should be a int, not a {type(startSurf)}")
        self._startSurf = startSurf

    @property
    def sort_enclosure(self):
        return self._sort_enclosure

    @sort_enclosure.setter
    def sort_enclosure(self, sort_enclosure: bool):
        if not isinstance(sort_enclosure, bool):
            raise TypeError(f"geouned.Settings.sort_enclosure should be a bool, not a {type(sort_enclosure)}")
        self._sort_enclosure = sort_enclosure
