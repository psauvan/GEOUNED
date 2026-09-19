import pytest

from geouned.geo import volume_within
from geouned.geo.constants import VOLUME_REF


def test_relative_for_large_solids():
    assert volume_within(1000.0 * (1 + 2.9e-4), 1000.0, 3e-4)
    assert not volume_within(1000.0 * (1 + 3.1e-4), 1000.0, 3e-4)


def test_absolute_below_the_reference_volume():
    # a 0.01 mm^3 solid: the tolerance is tol * VOLUME_REF, not tol * 0.01
    tol = 3e-4
    assert volume_within(0.01 + 0.9 * tol * VOLUME_REF, 0.01, tol)
    assert not volume_within(0.01 + 1.1 * tol * VOLUME_REF, 0.01, tol)


def test_reference_defaults_to_the_expected_volume_and_can_be_given():
    assert volume_within(1.0, 1.0 + 0.5e-6, 1e-6) is True
    # explicit reference: the change is measured against a much bigger volume
    assert volume_within(100.0, 100.5, 1e-3, reference=1e4)  # 1e-3 * 1e4 = 10 >= 0.5
    assert not volume_within(100.0, 100.5, 1e-3)  # 1e-3 * 100.5 = 0.1005 < 0.5


def test_negative_volumes_use_the_magnitude():
    assert volume_within(-1000.0, -1000.0 * (1 + 1e-6), 3e-4)


def test_reference_volume_is_a_scale_not_the_minimum_piece():
    from geouned.GEOUNED.utils.data_classes import Tolerances

    # the reference scale is intrinsic; the minimum piece is the user's field, and it is far smaller
    assert Tolerances().min_solid_volume < VOLUME_REF


@pytest.mark.parametrize("engine_free_name", ["volume_within"])
def test_exported_from_geo(engine_free_name):
    import geouned.geo as geo

    assert hasattr(geo, engine_free_name)


# ---------------------------------------------------------------------------
# one minimum volume for every place that discards a piece
# ---------------------------------------------------------------------------


class _Piece:
    def __init__(self, volume, area):
        self.Volume = volume
        self.Area = area


def test_valid_solid_uses_the_given_minimum_volume():
    from geouned.geo.solid_defects import valid_solid

    piece = _Piece(volume=5e-3, area=1.0)  # V/A = 5e-3: not a sliver
    assert not valid_solid(piece, 1e-2)
    assert valid_solid(piece, 1e-3)  # the user lowered the minimum: the same piece is now real


def test_default_minimum_volume_is_one_value_and_below_the_smallest_known_real_piece():
    from geouned.geo.constants import DEFAULT_MIN_SOLID_VOLUME
    from geouned.GEOUNED.utils.data_classes import Tolerances

    assert Tolerances().min_solid_volume == DEFAULT_MIN_SOLID_VOLUME == 1e-2
    assert DEFAULT_MIN_SOLID_VOLUME < 0.072  # Decomposed/modelcell_cut1_v2_piece66.stp, a legitimate piece


def test_removed_minimum_volume_constants_are_gone():
    import geouned.geo.constants as constants

    for old in ("DEGENERATE_SOLID_VOLUME_FLOOR", "VOLUME_MIN_E3"):
        assert not hasattr(constants, old), old
