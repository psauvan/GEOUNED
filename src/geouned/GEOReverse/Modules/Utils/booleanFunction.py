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

`evaluate_three_valued` is a thin adapter, not a reimplementation:
GEOUNED's own `BoolSequence.evaluate()` already substitutes known
values and returns either a resolved bool or the residual (partially
substituted) BoolSequence when it can't fully resolve -- every real
caller here (`Utils/boundBox.py`'s `isInside`, `splitFunction.py`'s
`SplitSolid`) needs a plain `None` for "undetermined" instead of a
residual object (which would be truthy and misread as "resolved"), so
this just downgrades a non-bool result to `None`. Confirmed by random
testing (3000 generated expressions) to be at least as resolving as
GEOReverse's own old hand-rolled three-valued walk -- and strictly more
so in some cases, since `.evaluate()`'s use of `.substitute()` catches
structural contradictions (e.g. a nested OR collapsing until an outer
AND is left with both `+n` and `-n` of the same surface) that a single
top-down walk over the original tree does not.

GEOReverse's own `simplify`/`factorize` (Shannon-expansion redundancy
removal, used only by `remh.py`'s `hash_sequence` -- confirmed dead
code, never actually called anywhere) are not ported at all: GEOUNED's
own `BoolSequence.simplify()` (no `CT` argument needed, defaults to the
same trivial per-surface expansion `hash_sequence`'s own default did)
does the same job directly.
"""

from ....boolean_utils.boolean_function import BoolSequence
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


def evaluate_three_valued(seq, value_set):
    """Evaluate `seq` against `value_set` (surface number -> True/False/
    None), returning True, False, or None (undetermined) -- never the
    residual BoolSequence that `BoolSequence.evaluate()` itself returns
    for an unresolved result."""
    result = seq.evaluate(value_set)
    return result if isinstance(result, bool) else None
