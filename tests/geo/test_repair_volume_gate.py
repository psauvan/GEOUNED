import pathlib
import re

import geouned.geo.constants as constants

SRC = pathlib.Path(__file__).resolve().parents[2] / "src" / "geouned"


def test_every_healing_repair_shares_one_volume_gate():
    assert constants.MAX_REPAIR_VOLUME_REL_CHANGE == 3.0e-4
    # the four per-repair constants (1e-2, 3e-4, 5e-4, 1e-3) are gone
    for old in ("MAX_DEFEATURE_VOLUME_REL_CHANGE", "MAX_SPLIT_RING_VOLUME_REL_CHANGE", "MAX_SLIVER_HEAL_VOLUME_REL_CHANGE", "MAX_HEAL_TOPOLOGY_VOLUME_REL_CHANGE"):
        assert not hasattr(constants, old), old


def test_no_source_file_still_names_a_removed_gate():
    pattern = re.compile(r"MAX_(DEFEATURE|SPLIT_RING|SLIVER_HEAL|HEAL_TOPOLOGY)_VOLUME_REL_CHANGE")
    stale = [p.relative_to(SRC).as_posix() for p in SRC.rglob("*.py") if pattern.search(p.read_text(encoding="utf-8-sig"))]
    assert stale == []


def test_exact_operations_keep_their_own_tight_gate():
    # Gmerge_coplanar_planes and Gsplit's volume_tolerance confirm an (almost) exact operation: not part of the
    # shared gate. Gmerge_coplanar_planes itself now uses tolerances.volume_tolerance directly (2026-09-23,
    # replacing its own former REL_TOL_E6 literal); GSolid.refine()'s own kernel self-check keeps its own,
    # separately-named NATIVE_VOL_RATIO_TOL (same historical value, split out of REL_TOL_E6 the same day).
    from geouned.GEOUNED.utils.data_classes import Tolerances

    assert Tolerances().volume_tolerance < constants.MAX_REPAIR_VOLUME_REL_CHANGE
    assert constants.NATIVE_VOL_RATIO_TOL < constants.MAX_REPAIR_VOLUME_REL_CHANGE
