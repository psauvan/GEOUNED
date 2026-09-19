"""A `Tolerances()` built inside the pipeline silently ignores whatever the user configured.

The tolerances object must be threaded from `CadToCsg(tolerances=...)` down to where it is read. The only places allowed to
create one are the ones that stand in for "the caller did not give any".
"""
import ast
import pathlib

import pytest

SRC = pathlib.Path(__file__).resolve().parents[2] / "src" / "geouned"

# file (relative to src/geouned) -> why creating a default instance there is legitimate
ALLOWED = {
    "GEOUNED/core.py": "public CadToCsg constructor: no tolerances given by the user",
    "GEOUNED/loadfile/load_step.py": "load_cad() used standalone without tolerances",
    "GEOUNED/utils/data_classes.py": "the class itself",
}


def _fresh_instances():
    found = []
    for path in sorted((SRC / "GEOUNED").rglob("*.py")):
        rel = path.relative_to(SRC).as_posix()
        if rel in ALLOWED:
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8-sig"))):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id in ("Tolerances", "GeoTolerances")
                and not node.args
                and not node.keywords
            ):
                found.append(f"{rel}:{node.lineno}")
    return found


def test_no_default_tolerances_built_inside_the_pipeline():
    assert _fresh_instances() == []


def test_identity_predicates_refuse_to_guess_the_tolerances():
    from geouned.GEOUNED.utils.basic_functions_part2 import is_same_cylinder, is_same_plane

    with pytest.raises(TypeError, match="needs the run's Tolerances"):
        is_same_plane(object(), object())
    with pytest.raises(TypeError, match="needs the run's Tolerances"):
        is_same_cylinder(object(), object())


def test_sliver_walk_refuses_to_guess_the_tolerances():
    from geouned.GEOUNED.utils.geometry_gu import other_face_edge

    class _Edge:
        def is_same(self, other):
            return True

    class _Face:
        Index = 1
        Edges = [_Edge()]
        OuterWire = type("W", (), {"Edges": [_Edge()]})()
        Area = 1.0

    class _Current:
        Index = 0

    with pytest.raises(ValueError, match="needs the run's tolerances"):
        other_face_edge(_Edge(), _Current(), [_Face()], skip_slivers=True)
