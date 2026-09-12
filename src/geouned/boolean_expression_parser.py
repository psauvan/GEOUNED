"""
geouned/boolean_expression_parser.py

Pure-Python parser for MCNP-syntax boolean cell-definition expressions
(the `-1 2 (3:-4)`-style text a cell's geometry line is written in) --
zero dependency on anything else in this package, not even `geo`. Sits
at the top level, alongside `geo`, because it is shared by both
pipelines' own `BoolSequence` classes:
- `GEOUNED/utils/boolean_function.py::BoolSequence.set_def`
- `GEOReverse/Modules/Utils/booleanFunction.py::BoolSequence.set_def`

Extracted 2026-09-12 once confirmed the two files' own copies of
`outer_terms`/`redundant`/`is_integer` (plus the 3 regexes below) were
functionally identical -- differing only in a handful of cosmetic
variable renames (`left_ok`/`right_ok` vs `leftOK`/`rightOK`, `new_pos`
vs `newpos`) and docstrings. GEOReverse's own copy additionally carried
4 confirmed-dead regexes (`number`, `PValue`, `NValue`, `conversion` --
none referenced anywhere in that file's real code, and `outer_terms`'s
own `value != "number"` branch that would use `TFX`/`conversion` is
never reached by any real caller either) -- dropped here rather than
carried forward, since this is the first time the two copies are
compared side by side rather than each evolving independently.

This is deliberately scoped to *only* this shared parsing logic --
`BoolSequence` itself is NOT unified between the two pipelines (see
`docs/history/geouned_migration_log.md`'s "BoolSequence unification
analysis" entry for the full reasoning): GEOUNED's own class carries a
`BoolVariable`/`BoolSurface` tier built up over many sessions of
carefully-verified sign-convention fixes that GEOReverse has no use
for, and GEOReverse's own class is intentionally the simpler,
plain-integer-only one. Only the text -> term-list parsing step, which
neither side's own boolean-algebra logic touches at all, is shared.
"""

import re

mostinner = re.compile(r"\([^\(^\)]*\)")  # identify most inner parentheses
mix = re.compile(r"(?P<value>([-+]?\d+|\[0+\]))")  # identify signed integer or [000...] pattern. Record the value.
TFX = re.compile(r"(?P<value>[FTXo]+)")  # identify pattern including F,T,X, or o sequence ( in any order).


def outer_terms(expression, value="number"):
    """Return the list and the boolean operator of the outer terms of the expression."""
    if value == "number":
        reValue = mix
        nullVal = "0"
    else:
        reValue = TFX
        nullVal = "o"

    expr = expression

    # Loop until no redundant parentheses are found
    cont = True

    while cont:
        # Loop over most inner parentheses
        pos = 0
        cont = False
        while True:
            m = mostinner.search(expr, pos)
            if not m:
                break
            cont = True
            if redundant(m, expr):
                # remove redundant parentheses
                expr = expr[: m.start()] + " " + expr[m.start() + 1 : m.end() - 1] + " " + expr[m.end() :]
            else:
                # replace no redundant parentheses by 0 and : by ;
                zeros = "[" + nullVal * (m.end() - m.start() - 2) + "]"
                expr = expr[: m.start()] + zeros + expr[m.end() :]

            pos = m.end()

    if ":" in expr:
        terms = []
        pos = 0
        while True:
            new_pos = expr.find(":", pos)
            if new_pos == -1:
                terms.append(expression[pos:].strip())
                break
            terms.append(expression[pos:new_pos].strip())
            pos = new_pos + 1
        return (terms, "OR")
    else:
        terms = []
        pos = 0
        while True:
            m = reValue.search(expr, pos)
            if not m:
                break
            terms.append(expression[m.start() : m.end()])
            pos = m.end()
        return (terms, "AND")


def redundant(m, geom):
    """Check if the inner parentheses are redundant."""
    term = m.group()

    # Find first valid character at the left of the  parenthese
    left_ok = True
    left = m.start() - 1
    while left > -1:
        if geom[left] in ("\n", "C", "$", " "):
            left -= 1
        else:
            if geom[left] not in ("(", ":"):
                left_ok = False
            break

    # check if no ':' (or) are inside the parenthese
    # if not, parentheses are redundants
    if term.find(":") == -1:
        return True

    # Find first valid character at the right of the  parenthese
    right_ok = True
    right = m.end()
    while right < len(geom):
        if geom[right] in ("\n", "C", "$", " "):
            right += 1
        else:
            if geom[right] not in (")", ":"):
                right_ok = False
            break

    # if parentheses are like:
    # {( or : } ( ....... ) {) or :}
    # parentheses are redundants

    if left_ok and right_ok:
        return True
    else:
        return False


def is_integer(x):
    try:
        int(x.strip("(").strip(")"))
        return True
    except:
        return False
