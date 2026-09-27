"""
GEOUNED/decompose/decompose_cache.py

Persists each solid's decomposed convex pieces (`main_split`'s own
output, the CPU-expensive phase of the pipeline) across runs, keyed by
the solid's own STEP label (`GeounedSolid.StepLabel`) -- see
`Settings.load_from_cache`'s own docstring for the full user-facing
contract.

Pieces are stored via `geo`'s own `Gexport_binary`/`Gload_binary`
(OCCT's/FreeCAD's native binary shape format, `BinTools`/
`exportBinary`/`importBinary`), never via STEP: this is a purely
internal, GEOUNED-to-GEOUNED round-trip, never meant to be opened by any
other tool, and going through STEP would mean paying its write/read cost
for nothing (measured, 2026-09-27: the binary format is ~3x smaller,
~7x faster to write, ~50x faster to read for a small multi-quadric
compound) while also carrying STEP's own, unrelated, already-documented
risk of an analytic quadric surface being downgraded to a generic
spline/revolution/extrusion representation on some rarer surface types
(see GEOReverse's own "STEP round-trip for Geom_Hyperbola-based..."
entry in CLAUDE.md) -- the binary format serializes the exact in-memory
geometry directly, with no format translation at all, so that risk
cannot arise here regardless of what surface types a cached piece has.

Deliberately holds NO geometric fingerprint/comparison: change detection
is user-driven (the `__modified__` label marker parsed in
`loadfile/load_step.py::load_cad`), not automatic -- a solid edited
without the marker silently reuses its stale cached decomposition, an
explicit, accepted trade-off (see CLAUDE.md's "Decomposition rerun cache"
entry and the `project_decomposition_rerun_cache_design` memory for the
full design rationale).

Only phase 1 (decomposition into convex pieces) is ever cached here --
phase 2 (`conversion/cell_definition.py::build_definition`, which
registers surfaces into the model-wide, deduplicated `Surfaces` registry
and assigns final numbering) always reruns identically for every solid,
so this cache can only ever change how long a run takes, never its
written output.

Two-tier storage, per direct user design (2026-09-27):
- `decompose_cache/{solids,enclosures}.bin` + `manifest.json`: the
  consolidated result of the last FULLY successful decomposition run --
  every processed solid's pieces packed into ONE compound per namespace
  (native shape order is preserved exactly through a binary round-trip,
  verified on all 3 engines), with the manifest recording each label's
  own `{start, count}` slice into that flat piece list. Written
  unconditionally at the end of any run that completes both the
  `solids` and `enclosures` passes without raising -- regardless of
  `Settings.load_from_cache` -- so even a deliberate "ignore the cache,
  redo everything" run leaves behind a fresh, trustworthy cache for the
  next one.
- `decompose_cache/tmp/`: one small `.bin` file per (freshly decomposed
  or reused) solid, written incrementally as the run progresses, plus
  its own manifest kept up to date atomically after every solid --
  purely a crash-recovery staging area, gated by the internal
  `TMP_CACHE` constant (`utils/data_constants.py`, default True). If a
  run is interrupted (raises) before reaching the final consolidation
  step, this is what survives on disk; the NEXT `load_from_cache=True`
  run overlays it on top of the last good consolidated cache (tmp's own
  entries win, being the freshest), so an interrupted run can be resumed
  from exactly where it left off. Cleared out once a run's own
  consolidation succeeds -- its data is folded into the fresh
  `{solids,enclosures}.bin` at that point, so it's no longer needed.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import shutil
from importlib.metadata import version
from pathlib import Path

from ...geo import CAD_ENGINE, GSolid, Gexport_binary, Gload_binary, kernel_version
from ...geo.constants import TMP_CACHE

logger = logging.getLogger("general_logger")

_MANIFEST_NAME = "manifest.json"
_NAMESPACES = ("solids", "enclosures")
_TMP_NAMESPACE_DIRS = {"solids": "pieces", "enclosures": "enclosures"}

# Every Tolerances field the decomposition path (main_split/generic_split/
# Gsplit's own retry cascade) actually reads -- confirmed from
# decompose/decom_one_generators.py. A change to any of these can produce
# a different decomposition for geometry that itself didn't change, so it
# must invalidate the whole cache, not just be ignored.
_DECOMPOSITION_TOLERANCE_FIELDS = (
    "split_tolerance",
    "fix_tolerance",
    "volume_tolerance",
    "scale",
    "scale_up_floor",
    "min_solid_volume",
)


def label_key(label: str) -> str:
    """A filesystem-safe cache filename for a raw STEP label -- the label
    itself stays the manifest's own (human-readable) JSON key."""
    return hashlib.sha1(label.encode("utf-8")).hexdigest()


def compute_global_key(options, tolerances) -> dict:
    """Everything that invalidates the WHOLE cache if it differs from a
    prior run's: GEOUNED version, CAD engine/kernel, and every
    Tolerances/Options field the decomposition path itself reads."""
    return {
        "geouned_version": version("geouned"),
        "cad_engine": CAD_ENGINE,
        "kernel_version": kernel_version(),
        "decomposition_params": {
            **{field: getattr(tolerances, field) for field in _DECOMPOSITION_TOLERANCE_FIELDS},
            "cut_large_cell": options.cut_large_cell,
        },
    }


def _atomic_write_json(path: Path, data) -> None:
    """Writes `data` to `path` via a temp file + `os.replace` -- the file
    at `path` is always either its previous, fully-valid content or the
    new, fully-valid content, never a partially-written state, no matter
    when a crash happens."""
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
    os.replace(tmp_path, path)


class DecomposeCache:
    """Wraps a `<Settings.outPath>/decompose_cache/` directory. See
    `Settings.load_from_cache`'s own docstring for the user-facing
    contract, and this module's own docstring for the two-tier storage
    design."""

    def __init__(self, cache_dir, options, tolerances, enabled: bool):
        self.cache_dir = Path(cache_dir)
        self.tmp_dir = self.cache_dir / "tmp"
        self.enabled = enabled
        self._global_key = compute_global_key(options, tolerances)
        self._manifest = {ns: {} for ns in _NAMESPACES}
        self._flat_pieces = {}
        self._tmp_overrides = {ns: {} for ns in _NAMESPACES}
        self._tmp_manifest = {"global_key": self._global_key, **{ns: {} for ns in _NAMESPACES}}
        # Every solid processed this run (cache hit or freshly
        # decomposed), tracked so the final consolidation never needs to
        # re-read anything from disk.
        self._all_pieces = {ns: {} for ns in _NAMESPACES}

    def load(self) -> None:
        """Loads the last good consolidated cache (if `enabled` and one
        exists with a matching `global_key`) into memory once, and
        overlays any leftover `tmp/` staging from an interrupted previous
        run on top of it. A no-op entirely when `enabled` is False --
        every subsequent `lookup()` then simply finds nothing, so the run
        decomposes every solid as if there were no cache at all."""
        if not self.enabled:
            return

        manifest_path = self.cache_dir / _MANIFEST_NAME
        if manifest_path.exists():
            with open(manifest_path, "r", encoding="utf-8") as f:
                manifest = json.load(f)
            if manifest.get("global_key") == self._global_key:
                self._manifest = manifest
                for namespace in _NAMESPACES:
                    bin_path = self.cache_dir / f"{namespace}.bin"
                    if manifest.get(namespace) and bin_path.exists():
                        self._flat_pieces[namespace] = Gload_binary(str(bin_path))
            else:
                logger.info(
                    "decompose cache: stored parameters (GEOUNED version/engine/kernel/decomposition "
                    "tolerances) differ from this run's own -- ignoring the cached result entirely"
                )
        else:
            logger.info("decompose cache: no cached result found under %s", self.cache_dir)

        tmp_manifest_path = self.tmp_dir / _MANIFEST_NAME
        if tmp_manifest_path.exists():
            with open(tmp_manifest_path, "r", encoding="utf-8") as f:
                tmp_manifest = json.load(f)
            if tmp_manifest.get("global_key") == self._global_key:
                for namespace in _NAMESPACES:
                    self._tmp_overrides[namespace] = tmp_manifest.get(namespace, {})
                logger.info(
                    "decompose cache: found tmp staging from an interrupted previous run -- "
                    "resuming from it (its own entries take precedence)"
                )

    def lookup(self, namespace: str, label: str) -> "list[GSolid] | None":
        """Cached pieces for `label`, or None if there is no usable cache
        entry. Checks the freshest source first (an interrupted previous
        run's own `tmp/` staging, if any), then the last good consolidated
        cache. Always None when `enabled` is False. Does not check label
        uniqueness for the current run -- the caller (core.py, which
        already computed the current run's own label counts) is expected
        to only call this for a label it has already confirmed is unique
        this run."""
        if not self.enabled:
            return None

        override = self._tmp_overrides[namespace].get(label)
        if override is not None:
            piece_file = self.tmp_dir / _TMP_NAMESPACE_DIRS[namespace] / override["file"]
            if piece_file.exists():
                return Gload_binary(str(piece_file))

        entry = self._manifest.get(namespace, {}).get(label)
        if entry is None:
            return None
        flat = self._flat_pieces.get(namespace)
        if flat is None:
            return None
        start, count = entry["start"], entry["count"]
        return flat[start : start + count]

    def record(self, namespace: str, label: str, pieces) -> None:
        """Tracks `pieces` in memory for the final consolidation, without
        touching `tmp/` -- used for a cache HIT (nothing changed, nothing
        new to persist)."""
        self._all_pieces[namespace][label] = pieces

    def store(self, namespace: str, label: str, pieces) -> None:
        """A freshly (re)decomposed solid: records it (see `record`) and,
        if `TMP_CACHE`, immediately persists it to `tmp/` for crash
        safety -- the `tmp/` manifest is rewritten atomically right away,
        so it's never more than one solid behind the real progress of
        this run."""
        self.record(namespace, label, pieces)
        if not TMP_CACHE:
            return
        namespace_dir = self.tmp_dir / _TMP_NAMESPACE_DIRS[namespace]
        namespace_dir.mkdir(parents=True, exist_ok=True)
        filename = f"{label_key(label)}.bin"
        Gexport_binary(pieces, str(namespace_dir / filename))
        self._tmp_manifest[namespace][label] = {"file": filename}
        _atomic_write_json(self.tmp_dir / _MANIFEST_NAME, self._tmp_manifest)

    def finalize(self, namespaces, usable_labels: dict) -> None:
        """Called ONCE, only after a run's decomposition pass(es)
        complete without raising -- builds a fresh consolidated
        `<namespace>.bin` + its slice of `manifest.json` for every
        namespace in `namespaces` (the ones this run actually attempted
        -- e.g. `enclosures` is left out entirely when
        `Settings.voidGen` is False or there are no enclosures this run,
        so a namespace that wasn't touched keeps whatever was already
        cached for it, untouched, rather than being wiped), from every
        solid tracked this run (`record`/`store`, which only ever
        happens for a usable -- unique, trustworthy -- label; a label no
        longer present this run simply never entered `_all_pieces` and
        is therefore silently absent from the new cache, no explicit
        reconciliation needed). Discards `tmp/` once done (its data is
        now folded into the fresh consolidated files, for every
        namespace actually finalized).

        Never reached if the run raises partway through -- the previous
        run's own consolidated cache is left completely untouched in
        that case, and whatever `tmp/` staging exists (if `TMP_CACHE`)
        still reflects this run's own partial progress for the next
        `load_from_cache=True` run's `load()` to pick up."""
        manifest = {"global_key": self._global_key}
        for namespace in _NAMESPACES:
            if namespace not in namespaces:
                # Not attempted this run -- keep whatever was loaded at
                # the start of this run (or nothing, if there was none)
                # completely untouched.
                manifest[namespace] = self._manifest.get(namespace, {})
                continue
            labels = sorted(label for label in self._all_pieces[namespace] if label in usable_labels[namespace])
            flat_native = []
            offsets = {}
            for label in labels:
                pieces = self._all_pieces[namespace][label]
                offsets[label] = {"start": len(flat_native), "count": len(pieces)}
                flat_native.extend(pieces)
            target = self.cache_dir / f"{namespace}.bin"
            if flat_native:
                self.cache_dir.mkdir(parents=True, exist_ok=True)
                Gexport_binary(flat_native, str(target))
            elif target.exists():
                target.unlink()
            manifest[namespace] = offsets

        self.cache_dir.mkdir(parents=True, exist_ok=True)
        _atomic_write_json(self.cache_dir / _MANIFEST_NAME, manifest)

        if self.tmp_dir.exists():
            shutil.rmtree(self.tmp_dir)
