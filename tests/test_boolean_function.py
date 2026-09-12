import geouned  # must be imported before Part -- sets up FreeCAD's sys.path
from geouned.boolean_utils.boolean_function import BoolSequence as GeounedBoolSequence
from geouned.GEOReverse.Modules.Objects import CadCell
from geouned.GEOReverse.Modules.Utils.booleanFunction import (
    BoolSequence,
    evaluate_three_valued,
    remove_surf,
    signed_surfaces,
)


def test_georeverse_boolsequence_is_geouned_boolsequence():
    """Per the 2026-09-12 BoolSequence unification decision, GEOReverse no
    longer carries its own BoolSequence class -- it depends directly on
    GEOUNED's canonical one instead of a second, divergent copy."""
    assert BoolSequence is GeounedBoolSequence


def test_remove_surf_and_positive():
    seq = BoolSequence("1 2 3 4")
    remove_surf(seq, 3)
    assert seq.get_surfaces_numbers() == {1, 2, 4}


def test_remove_surf_and_negative():
    """Regression for a real bug (2026-09-12): the old removeSurf's `if
    e == name:` check only ever matched the positive literal, so a
    negative reference (-name) to the same surface was silently left in
    place."""
    seq = BoolSequence("1 2 -3 4")
    remove_surf(seq, 3)
    assert seq.get_surfaces_numbers() == {1, 2, 4}


def test_remove_surf_or_positive_collapses_to_true():
    seq = BoolSequence("1:2:3:4")
    remove_surf(seq, 3)
    assert seq.elements is True


def test_remove_surf_or_negative_collapses_to_true():
    seq = BoolSequence("1:2:-3:4")
    remove_surf(seq, 3)
    assert seq.elements is True


def test_remove_surf_nested_negative():
    seq = BoolSequence("(1 2):(-3 4)")
    remove_surf(seq, 3)
    assert seq.get_surfaces_numbers() == {1, 2, 4}


def test_clean_undefined_uses_real_remove_surf_function():
    """Regression for a real bug (2026-09-12): CadCell.cleanUndefined
    used to call self.definition.removeSurface(undefined) -- a method
    that never existed anywhere in GEOReverse. Would raise AttributeError
    the moment a cell referenced an undefined surface."""

    class FakeSurf:
        def __init__(self, params):
            self.params = params

    cell = CadCell()
    cell.definition = BoolSequence("1 2 -3 4")
    cell.surfaces = {1: FakeSurf(object()), 2: FakeSurf(object()), 3: FakeSurf(None), 4: FakeSurf(object())}

    cell.cleanUndefined()

    assert cell.definition.get_surfaces_numbers() == {1, 2, 4}
    assert 3 not in cell.surfaces


def test_signed_surfaces_flat():
    seq = BoolSequence("1 -2 3")
    assert signed_surfaces(seq) == {1, -2, 3}


def test_signed_surfaces_nested():
    seq = BoolSequence("(1 -2):(3 -4)")
    assert signed_surfaces(seq) == {1, -2, 3, -4}


def test_evaluate_three_valued_and_true():
    seq = BoolSequence("1 2")
    assert evaluate_three_valued(seq, {1: True, 2: True}) is True


def test_evaluate_three_valued_and_false_short_circuits():
    seq = BoolSequence("1 2")
    assert evaluate_three_valued(seq, {1: False, 2: None}) is False


def test_evaluate_three_valued_undetermined_is_none_not_a_sequence():
    """evaluate_three_valued is a thin wrapper around BoolSequence.evaluate
    -- an undetermined result must come back as the plain value `None`,
    not the residual BoolSequence object BoolSequence.evaluate itself
    returns in that case (which would be truthy and misread by callers
    like `if inSolid: ... elif inSolid is None: ...`)."""
    seq = BoolSequence("1 2")
    result = evaluate_three_valued(seq, {1: True, 2: None})
    assert result is None


def test_evaluate_three_valued_resolves_structural_contradiction():
    """`2 AND (1 OR -2) AND -1`, with only surface 1 known False: a naive
    top-down walk can't resolve this (surface 2's own value is unknown),
    but substituting 1=False first collapses the OR down to just `-2`,
    leaving `2 AND -2` at the AND level -- an outright contradiction
    regardless of what surface 2 actually is. evaluate_three_valued
    reuses BoolSequence.evaluate's substitution-based resolution, so it
    catches this and returns False rather than None."""
    seq = BoolSequence("2 (1:-2) -1")
    assert evaluate_three_valued(seq, {1: False}) is False


def test_georeverse_simplify_dead_caller_uses_geouned_simplify_directly():
    """remh.py's hash_sequence (confirmed dead code, never actually
    called by the real pipeline) used to call its own removed
    simplify()/factorize() free functions; it now calls
    BoolSequence.simplify() (GEOUNED's own method, no CT needed) instead.
    This locks in that GEOUNED's own simplify does the same job for the
    one case that mattered: 1 AND (1 OR 2) reduces to just surface 1."""
    seq = BoolSequence(operator="AND")
    seq.append(1, BoolSequence("1:2"))
    seq.join_operators()
    seq.simplify()
    assert seq.get_surfaces_numbers() == {1}


def test_evaluate_three_valued_or_true_short_circuits():
    seq = BoolSequence("1:2")
    assert evaluate_three_valued(seq, {1: True, 2: None}) is True


def test_evaluate_three_valued_negative_literal():
    seq = BoolSequence("-1 2")
    assert evaluate_three_valued(seq, {1: False, 2: True}) is True
