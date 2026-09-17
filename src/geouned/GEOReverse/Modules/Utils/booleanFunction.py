"""GEOReverse no longer carries its own BoolSequence class. Per the
2026-09-12 BoolSequence unification decision, GEOUNED's own class
(`GEOUNED/utils/boolean_function.py`) is canonical and must not be
touched from this side (it is historically fragile and load-bearing
there) -- GEOReverse now depends on it unmodified instead of carrying
a second, divergent copy.

Two of GEOReverse's own algorithms have no equivalent at all on
GEOUNED's class, so they are kept here as free functions operating on
a BoolSequence instance's public attributes/methods rather than as
methods on the (untouched) class itself: `remove_surf` and
`signed_surfaces` (see the 2026-09-12 `cleanUndefined` bug-fix history
entry).

`evaluate_three_valued` moved to `boolean_utils/boolean_function.py`
2026-09-17 (re-exported below for this module's existing callers --
`Utils/boundBox.py`'s `isInside`, `splitFunction.py`'s `SplitSolid`):
it's a thin adapter, not a reimplementation, with zero dependency beyond
`BoolSequence.evaluate()` itself, and GEOUNED's own
`build_region/splitFunction.py::SplitSolid` used to hand-duplicate the
exact same logic inline -- see its own docstring for the full history
(confirmed by random testing, 3000 generated expressions, to be at
least as resolving as GEOReverse's old hand-rolled three-valued walk).

GEOReverse's own `simplify`/`factorize` (Shannon-expansion redundancy
removal, used only by `remh.py`'s `hash_sequence` -- confirmed dead
code, never actually called anywhere) are not ported at all: GEOUNED's
own `BoolSequence.simplify()` (no `CT` argument needed, defaults to the
same trivial per-surface expansion `hash_sequence`'s own default did)
does the same job directly.
"""

from ....boolean_utils.boolean_function import BoolSequence, evaluate_three_valued
from ....boolean_utils.boolean_expression_parser import is_integer, outer_terms, redundant


def remove_surf(seq, name):
    """Remove every occurrence of surface `name` from `seq`, regardless
    of sign -- an undefined surface can't be evaluated either way, so a
    bare `+name`/`-name` reference to it must be dropped (AND) or the
    whole clause trivialized to True (OR) identically for both."""
    if type(seq.elements) is bool:
        return
    for e in reversed(seq.elements):
        if isinstance(e, int):
            if abs(e) == name:
                if seq.operator == "AND":
                    seq.elements.remove(e)
                elif seq.operator == "OR":
                    seq.elements = True
                    seq.level = 0
                    return
        else:
            remove_surf(e, name)

    seq.clean()
    seq.level_update()


def signed_surfaces(seq):
    """Every signed surface literal (+n or -n) appearing anywhere in `seq`."""
    signed = set()
    for e in seq.elements:
        if isinstance(e, BoolSequence):
            signed.update(signed_surfaces(e))
        else:
            signed.add(e)
    return signed
