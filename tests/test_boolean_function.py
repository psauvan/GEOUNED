import geouned  # must be imported before Part -- sets up FreeCAD's sys.path
from geouned.GEOReverse.Modules.Objects import CadCell
from geouned.GEOReverse.Modules.Utils.booleanFunction import BoolSequence


def test_remove_surf_and_positive():
    seq = BoolSequence("1 2 3 4")
    seq.removeSurf(3)
    assert seq.get_surfaces_numbers() == (1, 2, 4)


def test_remove_surf_and_negative():
    """Regression for a real bug (2026-09-12): removeSurf's own `if e ==
    name:` check only ever matched the positive literal, so a negative
    reference (-name) to the same surface was silently left in place."""
    seq = BoolSequence("1 2 -3 4")
    seq.removeSurf(3)
    assert seq.get_surfaces_numbers() == (1, 2, 4)


def test_remove_surf_or_positive_collapses_to_true():
    seq = BoolSequence("1:2:3:4")
    seq.removeSurf(3)
    assert seq.elements is True


def test_remove_surf_or_negative_collapses_to_true():
    seq = BoolSequence("1:2:-3:4")
    seq.removeSurf(3)
    assert seq.elements is True


def test_remove_surf_nested_negative():
    seq = BoolSequence("(1 2):(-3 4)")
    seq.removeSurf(3)
    assert seq.get_surfaces_numbers() == (1, 2, 4)


def test_clean_undefined_uses_real_remove_surf_method():
    """Regression for a real bug (2026-09-12): CadCell.cleanUndefined
    used to call self.definition.removeSurface(undefined) -- a method
    that never existed anywhere in GEOReverse (BoolSequence only has
    removeSurf, singular, one surface at a time). Would raise
    AttributeError the moment a cell referenced an undefined surface."""

    class FakeSurf:
        def __init__(self, params):
            self.params = params

    cell = CadCell()
    cell.definition = BoolSequence("1 2 -3 4")
    cell.surfaces = {1: FakeSurf(object()), 2: FakeSurf(object()), 3: FakeSurf(None), 4: FakeSurf(object())}

    cell.cleanUndefined()

    assert cell.definition.get_surfaces_numbers() == (1, 2, 4)
    assert 3 not in cell.surfaces
