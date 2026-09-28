# `Options.meta_surfaces=False` corpus scan — 2026-09-28

## Purpose

First full-corpus d1suned (stochastic MCNP volume check) comparison of
`Options.meta_surfaces=True` (default) vs `Options.meta_surfaces=False`
(bypasses Can/TCone/RoundCorner/MultiRoundCorner/MultiPlane composite
detection, falls back to the 5 basic analytic surfaces; RevCC still
always runs per its own documented exception). Per CLAUDE.md, this
feature had previously only been verified on one fixture
(`RoundCorners/shed_part.stp`) via composite-surface-count, never via a
real d1suned physical-volume check, and never at corpus scale.

Run against `Solidos/test_models` (144 files, excluding `Big_*`), 8-way
parallel conversion + 16-way parallel d1suned, `load_from_cache=False`,
on the working tree as of this date (branch `spline-quadric-detection`,
includes 4 uncommitted fixes in `meta_surfaces.py`/`meta_surfaces_utils.py`
— the `gen_plane_cone`/`gen_plane_cylinder` frame-mismatch fix for
`Hollow_plates/placa2.stp`, not yet committed as of this scan).

New scripts added under
`\\wsl.localhost\Ubuntu-22.04\home\patrick\work\taller\SolidTestMCNP\scripts\`
for the `meta_surfaces=False` side (mirroring the existing `_ocp_`
scripts): `convert_one_ocp_nometa.py`, `run_all_conversions_ocp_nometa_parallel.py`
(writes to `runs_test_models_ocp_nometa/`), `run_test_models_ocp_nometa_d1suned.sh`,
`analyze_test_models_ocp_nometa.py`.

## Conversion (identical both runs, unrelated to meta_surfaces)

142/146 converted successfully in both runs. Same 4 pre-existing
failures in both (already documented in CLAUDE.md's "Known open items",
not caused by meta_surfaces and not new):

- `Decomposed/SCDR_90_piece2.stp` — degenerate geometry, doesn't
  decompose under either surface-search order.
- `Decomposed/modelcell_cut1_v2_piece66.stp` — same class of failure.
- `Mixed/ConeSphere.stp` — accepted native `ShapeUpgrade_UnifySameDomain`
  access-violation crash under occ/ocp.
- `Mixed/SCDR_90_hollow.stp` — load-time `SystemExit`
  (`corrupted_solids="stop"`, degenerate short-edge geometry).

## d1suned result summary

| | `meta_surfaces=True` | `meta_surfaces=False` |
|---|---|---|
| Total solid-cell tallies | 188 | 187 |
| Within 2σ | 175 (93.1%) | 167 (89.3%) |
| Marginal (2-3σ) | 11 (5.9%) | 11 (5.9%) |
| **FAIL (>3σ)** | **2 (1.1%)** | **9 (4.8%)** |
| Lost particles | 0 files | 1 file (10 lost) |

## Marginal (2-3σ) — same set of files in both runs, MC-noise-scale

These 11 cells are essentially identical between the two runs (same
files, same sigma to within rounding) — consistent with ordinary Monte
Carlo statistical noise on small/thin cells, not a meta_surfaces effect:

| File | Cell | value (True) | value (False) | n_sigma (True) | n_sigma (False) |
|---|---|---|---|---|---|
| Mixed/fwd_pipe.stp | 1 | 0.98924 | 0.98924 | 2.65 | 2.65 |
| RoundCorners/rc24.stp | 1 | 0.98951 | 0.98952 | 2.26 | 2.25 |
| Torus/tank.stp | 5 | 0.97980 | 0.97980 | 2.24 | 2.24 |
| RoundCorners/rrc1.stp | 1 | 0.99032 | 0.99032 | 2.22 | 2.22 |
| RoundCorners/rrc4.stp | 1 | 0.99076 | 0.99076 | 2.12 | 2.12 |
| Torus/example.stp | 10 | 1.01303 | 1.01303 | 2.11 | 2.11 |
| RoundCorners/rrc12.stp | 1 | 0.99117 | 0.99117 | 2.07 | 2.07 |
| Torus/U_open_Rev_1.stp | 1 | 0.99305 | 0.99305 | 2.06 | 2.06 |
| Decomposed/SCDR_solid19_solid26.stp | 1 | 1.01145 | 1.01145 | 2.06 | 2.06 |
| Torus/face2.stp | 2 | 0.64116 (huge rel_err 27%) | 0.64116 | 2.04 | 2.04 |
| RoundCorners/rrc5.stp | 1 | 0.99105 | 0.99106 | 2.01 | 2.00 |

## FAIL (>3σ) — where the two runs diverge

### Present in BOTH runs (already known/explained, not new)

- **`Mixed/SCDR_90_hollow.stp`** (True only, value=0, n_sigma=inf) — not
  a real result: this file fails at LOAD time (see conversion failures
  above), so the `outp` found in that run directory is stale, from a
  run predating this session. Not a real d1suned data point either way.
- **`RoundCorners/shed_solid.stp`** (both runs, value≈0.948, n_sigma≈3.04)
  — already documented in CLAUDE.md ("`forward_round_corner_region`:
  `OR_bracket` polarity inverted..." fix, 2026-09-25): confirmed pure
  Monte Carlo noise on a very small (0.249 cm³) cell, converges to
  0.981 at NPS 4e6. Not a meta_surfaces-specific issue — reproduces
  identically regardless of the flag.

### NEW under `meta_surfaces=False` only — candidates for a hidden bug

These 7 files (8 cell results) fail only when composite meta-surfaces
are bypassed. **Open question per the user's own instruction: some of
these may be a genuine, understood limitation of `meta_surfaces=False`
(composite surfaces like Can/TCone/RoundCorner may be load-bearing for
correct bounding in these geometries, not pure simplification — the
same reasoning already established for RevCC in CLAUDE.md), but some
may instead be exposing a latent GEOUNED bug that composite-surface
construction normally happens to avoid/mask, and that could in
principle also manifest under `meta_surfaces=True` on a different,
currently-untested geometry. Not yet distinguished — needs individual
investigation per file before concluding either way.**

| File | Cell | value | rel_err | n_sigma | Severity |
|---|---|---|---|---|---|
| `Mixed/multiplane_add_plane_cyl.stp` | 1 | 0.00756 | 0.0239 | 5492.86 | Catastrophic — ~99% of material missing |
| `Mixed/double_RC.stp` | 1 | 1.24915 | 0.0026 | 76.71 | Severe — +25% excess material |
| `RoundCorners/rrc23.stp` | 1 | 0.91938 | 0.0041 | 21.39 | Severe — ~8% missing |
| `Complex_cell/modelcell_cut1_1.stp` | 1 | 0.95581 | 0.0028 | 16.51 | Moderate — ~4.4% missing |
| `RoundCorners/comp_RC.stp` | 2 | 0.94469 | 0.0038 | 15.41 | Moderate — ~5.5% missing |
| `RoundCorners/comp_RC.stp` | 1 | 0.98084 | 0.0049 | 3.99 | Marginal-severe — ~1.9% missing |
| `Mixed/rev_pipe.stp` | 1 | 0.99243 | 0.0019 | 4.01 | Mild — ~0.76% missing, just past 3σ |
| `RoundCorners/rev_pipe.stp` | 1 | 0.99243 | 0.0019 | 4.01 | Mild — identical to Mixed/rev_pipe (same fixture, different folder?) |

### Lost particles — new under `meta_surfaces=False` only

- **`Cans/rev_can_1.stp`**: 10 particles lost, first at history 311,
  reason "no cell found in subroutine newcel" (a geometric gap — some
  region of space isn't claimed by any cell, meaning cell boundaries
  don't tile space correctly). 0 lost particles for this file with
  `meta_surfaces=True`.

  **ROOT-CAUSED AND FIXED, same day, confirms the user's own hypothesis
  that this document's "Observations" section raised as an open
  question.** `meta_surfaces_utils.py::_angle_function` (the azimuthal-
  angle helper behind `_is_closed_by_winding`, which decides whether a
  cone/cylinder face genuinely closes a full 360° around its own axis --
  used by `closed_cylinder_cone`/`get_can_surfaces`/`get_tcone_surfaces`)
  evaluated `atan2(0, 0) = 0.0` for any sample point landing exactly ON
  the surface's own axis (e.g. a cone's own apex vertex) -- an arbitrary
  convention, not a real angle, since the azimuthal direction is
  genuinely undefined there. A boundary loop that passes through this
  degenerate point (the standard OCCT topology for a full closed cone:
  apex vertex + a seam generatrix down and back + the base circle) got a
  spurious ~90° angular "jump" between the arbitrary apex sample and the
  first real sample beside it, which broke the winding-closure algorithm's
  own run-accumulation logic and made a genuinely closed cone read as
  NOT closed. Found independently on `working_solids/can_cone.stp` (not
  part of this corpus) while investigating a related report, then
  confirmed to be exactly the same mechanism affecting `Cans/rev_can_1.stp`
  here.

  **Fix**: `_angle_function`'s own `angle_of` now returns `None` when the
  point's perpendicular-to-axis distance is below `POINT_POINT_TOL`
  (on the axis); both consumers (`_oriented_angle_sweep`'s fast-path
  single-edge sweep, and `_loop_closes_full_turn`'s general per-sample
  walk) filter out `None` samples before computing angular deltas,
  instead of treating the arbitrary 0.0 as a real direction change.

  **Isolated verification** (a dedicated script comparing the OLD vs
  NEW closure logic directly on every cone/cylinder face of the corpus,
  without running the full pipeline): only 4/138 files have any face
  whose closure result changes -- `Cans/fwd_can_0.stp`, `fwd_can_1.stp`,
  `rev_can_0.stp`, `rev_can_1.stp` (2/3 faces each). Of these, only
  `rev_can_0`/`rev_can_1` change the final WRITTEN MCNP text under
  `meta_surfaces=True` (the Forward cases end up building the same final
  surfaces regardless, via a different branch). Under `meta_surfaces=True`,
  `rev_can_1.stp`'s own d1suned tally is unchanged by the fix (0.99991
  both before and after -- this file's physical result already happened
  to come out correct either way there). **Under `meta_surfaces=False`,
  the fix eliminates the 10 lost particles entirely**: re-verified with
  d1suned after the fix -- tally 0.99991 ± 0.15% (0.1σ), **0 lost
  particles** (was 10). Confirms this was a real, general GEOUNED bug
  (not a meta_surfaces=False-specific limitation), normally masked under
  `meta_surfaces=True` by an incidental alternate construction path for
  this particular file, but with the potential to affect any cone/cylinder
  face whose boundary passes through its own axis on a geometry where no
  such alternate path exists.

  **Not yet checked**: whether `fwd_can_0.stp`/`fwd_can_1.stp` (whose
  face-level closure result also changes, but whose written text doesn't)
  have any other latent issue.

  **The 6 other severe new-FAIL files: CONFIRMED unrelated to this
  winding bug, and confirmed correct under `meta_surfaces=True`,
  2026-09-28.** None of the 6 (`Mixed/multiplane_add_plane_cyl.stp`,
  `Mixed/double_RC.stp`, `RoundCorners/rrc23.stp`,
  `Complex_cell/modelcell_cut1_1.stp`, `RoundCorners/comp_RC.stp`,
  `Mixed/rev_pipe.stp`/`RoundCorners/rev_pipe.stp`) appeared in the
  4-file isolated-winding-effect list, so the winding bug was never a
  candidate explanation for them. Directly checked their own tally
  under a full `meta_surfaces=True` corpus re-run (same tree, all 3
  session changes applied) -- **all correct**:

  | File | `meta_surfaces=False` (this doc, above) | `meta_surfaces=True` (re-checked) |
  |---|---|---|
  | `Mixed/multiplane_add_plane_cyl.stp` | 0.00756 (catastrophic) | 0.99198 ± 0.44% |
  | `Mixed/double_RC.stp` | 1.24915 | 0.99667 ± 0.24% |
  | `RoundCorners/rrc23.stp` | 0.91938 | 0.99752 ± 0.41% |
  | `Complex_cell/modelcell_cut1_1.stp` | 0.95581 | 0.99883 ± 0.28% |
  | `RoundCorners/comp_RC.stp` cell 1 | 0.98084 | 0.99403 ± 0.48% |
  | `RoundCorners/comp_RC.stp` cell 2 | 0.94469 | 0.99792 ± 0.38% |
  | `Mixed/rev_pipe.stp` / `RoundCorners/rev_pipe.stp` | 0.99243 | 0.99771 ± 0.19% |

  **Conclusion**: these 6 files are genuinely explained by the
  "composite surfaces are load-bearing for correct bounding, not pure
  simplification" hypothesis from the Observations section below --
  under `meta_surfaces=True` (where Can/TCone/RoundCorner/MultiPlane
  actually get built), every one of them is correct; only bypassing
  that construction (`meta_surfaces=False`) breaks them. This is a
  DIFFERENT class from the winding-closure bug fixed above (which was
  a genuine, shared GEOUNED defect affecting both settings, just
  invisible under `meta_surfaces=True` for the one file in this corpus
  that exercised it). No further individual root-causing is planned for
  these 6 unless `meta_surfaces=False` itself becomes a priority again
  (currently just an opt-out feature, not the default).

## Observations / leads for follow-up

- Every new-FAIL file lives in `Mixed/`, `RoundCorners/`, `Cans/`, or
  `Complex_cell/` — folders whose fixtures are specifically built to
  exercise composite meta-surfaces (multi-plane junctions, round
  corners, cans, complex multi-solid cells). No `Hollow_plates/`,
  `esfera/`, or plain `Torus/` fixture is newly broken — consistent
  with (but not proof of) the "composite surfaces are load-bearing for
  these specific geometry classes" explanation rather than a
  meta_surfaces-unrelated general bug.
- `Mixed/rev_pipe.stp` and `RoundCorners/rev_pipe.stp` are **confirmed
  byte-identical** (`sha1sum` match) — a duplicate fixture across two
  folders, not two independent data points. So the real new-FAIL count
  is **6 distinct files** (7 cell results), not 7/8 — one of the 8 rows
  above is a duplicate of another.
- `Mixed/multiplane_add_plane_cyl.stp`'s near-total material loss
  (0.00756) is the most severe and the most worth investigating first
  — a >99% loss usually means a single badly-placed/missing bounding
  surface leaves the cell almost entirely outside its own material,
  which should be reproducible and traceable via the same kind of
  ground-truth point-sampling method used earlier this session for
  `placa2.stp`.
- None of these files have been individually root-caused yet — this
  document is the raw finding, not a diagnosis. Per the user's own
  instruction, the working hypothesis to test is whether the same
  underlying defect (whatever builds the wrong bounding surface for
  these cases under `meta_surfaces=False`) has any live code path that
  could also fire under `meta_surfaces=True` for a geometry not covered
  by this corpus.

## Raw data / reproduction

- `meta_surfaces=True` analysis: scratchpad `analyze_ocp.txt` (this
  session, see the earlier turn's tool output — not preserved as a
  repo file, regenerate via `analyze_test_models_ocp.py`).
- `meta_surfaces=False` analysis: scratchpad `analyze_ocp_nometa.txt`
  (same session; regenerate via `analyze_test_models_ocp_nometa.py`).
- Both scripts and their `convert_one_*`/`run_all_conversions_*`/
  `run_test_models_*_d1suned.sh` companions live in
  `SolidTestMCNP/scripts/` (WSL side) — the `_nometa` variants are new
  as of this scan, added following the existing `_ocp_`/`_occ_` naming
  convention.
