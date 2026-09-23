"""
geo/tolerances.py

`GeoTolerances`: the tolerances that `geo` itself reads, shared by every pipeline built on it (CadToCsg and CsgToCad).
Pure Python, no native import.

`geouned.Tolerances` (GEOUNED/utils/data_classes.py) extends this class with what only CadToCsg uses, so a user
always builds ONE tolerances object; `geo` accepts any subclass and only reads the fields defined here.

What lives here vs. elsewhere (rule agreed with the maintainers):
  - a value the USER may legitimately want to change (it depends on their model or on the output they want) is a
    field of Tolerances / GeoTolerances;
  - a value intrinsic to the code (floating-point floors, kernel query tolerances, algorithm thresholds) is a
    constant in `constants.py` and is never exposed.

Per-field documentation: see `geouned.Tolerances`.
"""

from __future__ import annotations

import typing

from . import CAD_ENGINE
from .constants import (
    DEFAULT_FIX_TOLERANCE,
    DEFAULT_MIN_FACE_WIDTH,
    DEFAULT_MIN_SOLID_VOLUME,
    DEFAULT_SLIVER_EDGE_REL_TOL,
    DEFAULT_SPLIT_SCALE,
)


class GeoTolerances:
    """Tolerances read by `geo` (surface identity, sliver detection, split/repair behaviour).

    Args:
        pln_distance
        pln_angle
        cyl_distance
        cyl_angle
        sph_distance
        kne_distance
        kne_angle
        tor_distance
        tor_angle
        min_face_width
        sliver_edge_rel_tol
        split_tolerance
        scale_up_floor
        scale
        min_solid_volume
        fix_tolerance
        volume_tolerance

    See `geouned.Tolerances` for what each one means and its default.
    """

    def __init__(
        self,
        pln_distance: float = 1.0e-4,
        pln_angle: float = 1.0e-4,
        cyl_distance: float = 1.0e-4,
        cyl_angle: float = 1.0e-4,
        sph_distance: float = 1.0e-4,
        kne_distance: float = 1.0e-4,
        kne_angle: float = 1.0e-4,
        tor_distance: float = 1.0e-4,
        tor_angle: float = 1.0e-4,
        min_face_width: float = DEFAULT_MIN_FACE_WIDTH,
        sliver_edge_rel_tol: float = DEFAULT_SLIVER_EDGE_REL_TOL,
        split_tolerance: typing.Optional[float] = 1.0e-6,
        scale_up_floor: typing.Optional[float] = 1e-12,
        scale: float = DEFAULT_SPLIT_SCALE,
        min_solid_volume: float = DEFAULT_MIN_SOLID_VOLUME,
        fix_tolerance: float = DEFAULT_FIX_TOLERANCE,
        volume_tolerance: float = 1.0e-4,
    ):
        self.pln_distance = pln_distance
        self.pln_angle = pln_angle
        self.cyl_distance = cyl_distance
        self.cyl_angle = cyl_angle
        self.sph_distance = sph_distance
        self.kne_distance = kne_distance
        self.kne_angle = kne_angle
        self.tor_distance = tor_distance
        self.tor_angle = tor_angle
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
    def pln_distance(self):
        return self._pln_distance

    @pln_distance.setter
    def pln_distance(self, pln_distance: float):
        if not isinstance(pln_distance, float):
            raise TypeError(f"geouned.Tolerances.pln_distance should be a float, not a {type(pln_distance)}")
        self._pln_distance = pln_distance

    @property
    def pln_angle(self):
        return self._pln_angle

    @pln_angle.setter
    def pln_angle(self, pln_angle: float):
        if not isinstance(pln_angle, float):
            raise TypeError(f"geouned.Tolerances.pln_angle should be a float, not a {type(pln_angle)}")
        self._pln_angle = pln_angle

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
