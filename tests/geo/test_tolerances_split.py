import inspect

import pytest

import geouned
from geouned.geo import GeoTolerances
from geouned.geo.tolerances import GeoTolerances as GeoTolerancesDirect
from geouned.GEOUNED.utils.data_classes import Tolerances


def _defaults(cls):
    return {name: p.default for name, p in inspect.signature(cls.__init__).parameters.items() if name != "self"}


def test_public_tolerances_is_a_geotolerances_and_keeps_its_name():
    assert geouned.Tolerances is Tolerances
    assert issubclass(Tolerances, GeoTolerances)
    assert GeoTolerances is GeoTolerancesDirect


def test_shared_fields_have_a_single_default():
    # Tolerances re-declares the shared fields in its signature (so users keep one flat constructor); the values must
    # never drift from GeoTolerances', which is what geo and GEOReverse use on their own.
    geo_defaults = _defaults(GeoTolerances)
    tol_defaults = _defaults(Tolerances)
    assert set(geo_defaults) <= set(tol_defaults)
    for name, default in geo_defaults.items():
        assert tol_defaults[name] == default, name


def test_geotolerances_holds_only_what_geo_reads():
    # CadToCsg-only fields must not leak into the base that GEOReverse uses
    for name in ("min_area", "relativeTol", "relativePrecision", "value", "distance", "angle", "add_pln_distance", "add_pln_angle"):
        assert not hasattr(GeoTolerances(), name), name
        assert hasattr(Tolerances(), name), name


def test_flat_constructor_and_shared_validation():
    t = Tolerances(pln_distance=2.0e-4, min_area=5.0e-2, relativeTol=True)
    assert (t.pln_distance, t.min_area, t.relativeTol) == (2.0e-4, 5.0e-2, True)
    with pytest.raises(TypeError, match="pln_distance should be a float"):
        Tolerances(pln_distance=1)
    with pytest.raises(TypeError, match="pln_distance should be a float"):
        GeoTolerances(pln_distance=1)


def test_scaled_keeps_shared_fields():
    t = Tolerances(pln_distance=3.0e-4, fix_tolerance=2.0e-6)
    s = t.scaled(1.0)
    assert isinstance(s, Tolerances)
    assert (s.pln_distance, s.fix_tolerance) == (3.0e-4, 2.0e-6)
    assert s.min_area < t.min_area and s.min_face_width < t.min_face_width
