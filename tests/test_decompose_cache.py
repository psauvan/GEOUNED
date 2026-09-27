"""
Tests for the decomposition cache (`GEOUNED/decompose/decompose_cache.py`),
keyed by a solid's own STEP label (`GeounedSolid.StepLabel`) and driven by
the user-added `__modified__` label marker -- see `Settings.
load_from_cache`'s own docstring and CLAUDE.md's "Decomposition rerun
cache" entry for the full design.

Two-tier storage (see `decompose_cache.py`'s own module docstring):
`decompose_cache/{solids,enclosures}.bin` + `manifest.json` (the last
FULLY successful run's consolidated result -- written unconditionally on
success, regardless of `Settings.load_from_cache`) and
`decompose_cache/tmp/` (per-solid staging written as a run progresses,
gated by the internal `TMP_CACHE` constant, `geo/constants.py`, default
True -- what survives if a run is interrupted, merged on top of the last
good consolidated result by the next `load_from_cache=True` run).

`Gload_step_labels` is monkeypatched to return controlled, synthetic
`GLabelNode`s (one per solid, `n_solids=1`, no parent) instead of relying
on a real XCAF-authored labeled STEP file -- `Gexport_step` itself has no
label-assignment API, so this isolates "does the label-driven caching
logic behave correctly" from "can a STEP writer embed a given name",
which is not part of this feature. Note the cache itself stores each
solid's pieces via `Gexport_binary`/`Gload_binary` (OCCT's/FreeCAD's own
native binary shape format), not STEP -- only the INPUT model fixtures
built by `_make_step` are real STEP files, simulating what a user's own
CAD tool would export.

Every fixture solid is a plain, unsplittable box (6 planar faces, no
candidate cutting surface) so `main_split` is fast and a no-op on the
geometry itself -- these tests are only about whether it gets CALLED for
the right labels, not about decomposition correctness (already covered
by the wider `tests/test_cadtocsg.py` corpus).
"""

import json
from unittest.mock import patch

import pytest

import geouned
from geouned.GEOUNED import core as core_module
from geouned.GEOUNED.decompose.decom_one_generators import main_split as real_main_split
from geouned.GEOUNED.loadfile import load_step as load_step_module
from geouned.geo import (
    GLabelNode,
    GVector,
    Gexport_binary,
    Gexport_step,
    Gload_binary,
    Gmake_box,
    Gmake_cone,
    Gmake_cylinder,
    Gmake_sphere,
    Gmake_torus,
)


def _make_step(path, n_boxes):
    """One 10x10x10 box solid per entry, spaced apart so they never touch."""
    shapes = [Gmake_box(i * 20, 0, 0, i * 20 + 10, 10, 10) for i in range(n_boxes)]
    Gexport_step(shapes, str(path))
    return shapes


def _fake_nodes(labels):
    return [GLabelNode(label=label, parent=None, n_solids=1) for label in labels]


def _read_manifest(tmp_path):
    return json.loads((tmp_path / "decompose_cache" / "manifest.json").read_text(encoding="utf-8"))


def _load_and_decompose(tmp_path, step_path, labels, load_from_cache, tolerances=None):
    settings = geouned.Settings(outPath=str(tmp_path), voidGen=False, load_from_cache=load_from_cache)
    c = geouned.CadToCsg(
        options=geouned.Options(),
        tolerances=tolerances if tolerances is not None else geouned.Tolerances(),
        settings=settings,
    )
    with patch.object(load_step_module, "Gload_step_labels", return_value=_fake_nodes(labels)):
        c.load_step_file(str(step_path))
    with patch.object(core_module, "main_split", wraps=core_module.main_split) as spy:
        c.decompose_solids()
    return c, spy


def test_first_run_populates_cache_and_decomposes_everything(tmp_path):
    step_path = tmp_path / "model.stp"
    _make_step(step_path, 3)

    c, spy = _load_and_decompose(tmp_path, step_path, ["A", "B", "C"], load_from_cache=True)

    assert spy.call_count == 3
    manifest = _read_manifest(tmp_path)
    assert set(manifest["solids"]) == {"A", "B", "C"}


def test_second_run_unchanged_labels_skips_decomposition_entirely(tmp_path):
    step_path = tmp_path / "model.stp"
    _make_step(step_path, 3)

    _load_and_decompose(tmp_path, step_path, ["A", "B", "C"], load_from_cache=True)
    c2, spy2 = _load_and_decompose(tmp_path, step_path, ["A", "B", "C"], load_from_cache=True)

    assert spy2.call_count == 0
    volumes = sorted(round(m.Volume) for m in c2.meta_list)
    assert volumes == [1000, 1000, 1000]


def test_modified_marker_forces_redecompose_of_only_that_solid(tmp_path):
    step_path = tmp_path / "model.stp"
    _make_step(step_path, 3)

    _load_and_decompose(tmp_path, step_path, ["A", "B", "C"], load_from_cache=True)
    c2, spy2 = _load_and_decompose(tmp_path, step_path, ["A__modified__", "B", "C"], load_from_cache=True)

    assert spy2.call_count == 1
    # the marker must be stripped before being used as the identity key
    labels = {m.StepLabel for m in c2.meta_list}
    assert labels == {"A", "B", "C"}
    assert not any(m.StepLabel == "A" and m.Modified is False for m in c2.meta_list if m.StepLabel != "A")


def test_new_label_is_decomposed_and_added_others_reused(tmp_path):
    step_path1 = tmp_path / "model1.stp"
    _make_step(step_path1, 2)
    _load_and_decompose(tmp_path, step_path1, ["A", "B"], load_from_cache=True)

    step_path2 = tmp_path / "model2.stp"
    _make_step(step_path2, 3)
    c2, spy2 = _load_and_decompose(tmp_path, step_path2, ["A", "B", "D"], load_from_cache=True)

    assert spy2.call_count == 1  # only the new label "D"
    manifest = _read_manifest(tmp_path)
    assert "D" in manifest["solids"]


def test_removed_label_is_dropped_from_cache(tmp_path):
    step_path1 = tmp_path / "model1.stp"
    _make_step(step_path1, 3)
    _load_and_decompose(tmp_path, step_path1, ["A", "B", "C"], load_from_cache=True)

    assert set(_read_manifest(tmp_path)["solids"]) == {"A", "B", "C"}

    step_path2 = tmp_path / "model2.stp"
    _make_step(step_path2, 2)
    _load_and_decompose(tmp_path, step_path2, ["A", "B"], load_from_cache=True)  # "C" removed

    assert set(_read_manifest(tmp_path)["solids"]) == {"A", "B"}


def test_duplicate_label_always_redecomposes_both(tmp_path):
    step_path = tmp_path / "model.stp"
    _make_step(step_path, 2)

    _load_and_decompose(tmp_path, step_path, ["A", "A"], load_from_cache=True)
    c2, spy2 = _load_and_decompose(tmp_path, step_path, ["A", "A"], load_from_cache=True)

    # never trusted as a cache identity: always redecomposed, both runs
    assert spy2.call_count == 2
    assert "A" not in _read_manifest(tmp_path)["solids"]


def test_global_key_change_invalidates_whole_cache(tmp_path):
    step_path = tmp_path / "model.stp"
    _make_step(step_path, 2)

    _load_and_decompose(tmp_path, step_path, ["A", "B"], load_from_cache=True)
    changed_tolerances = geouned.Tolerances(volume_tolerance=1.0e-3)
    _, spy2 = _load_and_decompose(tmp_path, step_path, ["A", "B"], load_from_cache=True, tolerances=changed_tolerances)

    assert spy2.call_count == 2


def test_load_from_cache_false_still_writes_but_never_reads(tmp_path):
    """The consolidated cache is written unconditionally on a fully
    successful run, regardless of `load_from_cache` -- but a run with
    `load_from_cache=False` must never READ it back, even if one already
    exists from a previous run."""
    step_path = tmp_path / "model.stp"
    _make_step(step_path, 2)

    _, spy1 = _load_and_decompose(tmp_path, step_path, ["A", "B"], load_from_cache=False)
    assert spy1.call_count == 2
    assert (tmp_path / "decompose_cache" / "manifest.json").exists()
    assert set(_read_manifest(tmp_path)["solids"]) == {"A", "B"}

    # Run again, still disabled: must fully redecompose again, ignoring
    # the cache this same run just wrote.
    _, spy2 = _load_and_decompose(tmp_path, step_path, ["A", "B"], load_from_cache=False)
    assert spy2.call_count == 2

    # Only once load_from_cache is actually True does the (now twice-
    # written, still valid) cache get consulted.
    _, spy3 = _load_and_decompose(tmp_path, step_path, ["A", "B"], load_from_cache=True)
    assert spy3.call_count == 0


def test_interrupted_run_resumes_from_tmp_and_prior_cache(tmp_path):
    """Simulates a crash partway through a decomposition run: the two
    solids that finished before the crash must be recoverable from tmp/
    staging on the next run, and the one solid never reached this run
    must still fall back correctly to the previous run's own good
    consolidated cache -- neither should need to be recomputed."""
    step_path = tmp_path / "model.stp"
    _make_step(step_path, 3)

    # Baseline: a fully successful run establishes a good consolidated cache.
    _load_and_decompose(tmp_path, step_path, ["A", "B", "C"], load_from_cache=True)

    # Force all 3 to redecompose this run (every label marked modified),
    # but simulate a crash right after the 2nd one completes.
    calls = []

    def flaky_main_split(solid_shape, options, tolerances):
        calls.append(1)
        if len(calls) == 3:
            raise RuntimeError("simulated crash mid-decomposition")
        return real_main_split(solid_shape, options, tolerances)

    settings = geouned.Settings(outPath=str(tmp_path), voidGen=False, load_from_cache=True)
    c = geouned.CadToCsg(options=geouned.Options(), tolerances=geouned.Tolerances(), settings=settings)
    modified_labels = ["A__modified__", "B__modified__", "C__modified__"]
    with patch.object(load_step_module, "Gload_step_labels", return_value=_fake_nodes(modified_labels)):
        c.load_step_file(str(step_path))
    with patch.object(core_module, "main_split", side_effect=flaky_main_split):
        with pytest.raises(RuntimeError, match="simulated crash"):
            c.decompose_solids()

    # The baseline run's own consolidated cache must be completely
    # untouched (finalize() was never reached this run).
    assert set(_read_manifest(tmp_path)["solids"]) == {"A", "B", "C"}

    tmp_dir = tmp_path / "decompose_cache" / "tmp"
    assert tmp_dir.exists()
    tmp_manifest = json.loads((tmp_dir / "manifest.json").read_text(encoding="utf-8"))
    assert set(tmp_manifest["solids"]) == {"A", "B"}  # exactly the 2 that completed before the crash

    # A fresh, un-flagged run should now recover fully: A/B served from
    # tmp staging, C served from the untouched baseline cache -- nothing
    # recomputed.
    c2, spy2 = _load_and_decompose(tmp_path, step_path, ["A", "B", "C"], load_from_cache=True)
    assert spy2.call_count == 0
    assert not (tmp_path / "decompose_cache" / "tmp").exists()  # folded into the fresh consolidated cache


def test_cache_round_trip_survives_native_healing_on_curved_solids(tmp_path):
    """Regression test for a real bug (2026-09-27, found via a real
    31-solid model, `Solidos/test_models/Big_model_reserved/
    shed_shutter.stp`): `Gload_binary` used to apply the same defensive
    native healing `Gload_step` needs, which crashed OCCT's own
    `ShapeUpgrade_UnifySameDomain` (`Courbes non jointives`) on a solid
    that never needed any healing at all -- a binary round-trip has no
    reader-reconstruction step to compensate for, unlike STEP. Every
    other fixture in this file is a plain box (6 planar faces), which
    never exercised this path at all -- this one uses a real cylinder
    solid (2 planar caps + 1 curved side) instead."""
    from geouned.geo import Gmake_cylinder

    step_path = tmp_path / "cyl_model.stp"
    cyl = Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 5.0, 10.0)
    Gexport_step([cyl], str(step_path))

    _load_and_decompose(tmp_path, step_path, ["Cyl"], load_from_cache=True)
    c2, spy2 = _load_and_decompose(tmp_path, step_path, ["Cyl"], load_from_cache=True)

    assert spy2.call_count == 0  # served from cache -- this is what used to crash
    assert round(c2.meta_list[0].Volume, 3) == round(cyl.Volume, 3)


def test_cache_export_preserves_analytic_quadric_surfaces(tmp_path):
    """The binary export/import the cache actually uses (Gexport_binary/
    Gload_binary) must never substitute an analytic quadric surface
    (plane/cylinder/cone/sphere/torus/...) for a generic spline/
    revolution/extrusion representation -- losing that would defeat the
    point of caching: a reloaded piece has to reclassify exactly as it
    did before being cached, per direct user instruction. It's fine if
    the reloaded file doesn't render nicely in a viewer (it isn't meant
    to be opened by anything but GEOUNED itself); only GEOUNED's own
    Gclassify_surface reading it back matters here."""

    solids = [
        Gmake_cylinder(GVector(0, 0, 0), GVector(0, 0, 1), 5.0, 10.0),
        Gmake_cone(GVector(0, 0, 30), GVector(0, 0, 1), 0.4, 10.0),
        Gmake_sphere(GVector(0, 0, 60), 5.0),
        Gmake_torus(GVector(0, 0, 90), GVector(0, 0, 1), 20.0, 5.0),
    ]

    def curved_surface_types(shapes):
        types = set()
        for shape in shapes:
            for face in shape.Faces:
                if face.Surface is not None:
                    types.add(type(face.Surface).__name__)
        return types

    before = curved_surface_types(solids)
    assert {"GCylinder", "GCone", "GSphere", "GTorus"} <= before

    bin_path = tmp_path / "quadrics.bin"
    Gexport_binary(solids, str(bin_path))
    reloaded = Gload_binary(str(bin_path))

    after = curved_surface_types(reloaded)
    # exactly the same analytic surface types survive the round trip --
    # none silently downgraded to an unclassified/BSpline representation
    assert after == before
