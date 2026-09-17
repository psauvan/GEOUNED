from pathlib import Path
import pytest
import geouned  # must be imported before Part -- sets up FreeCAD's sys.path
from geouned.geo import CAD_ENGINE, Gload_step

# Baseline solid volumes captured from the current (pre-FreeCAD-removal-migration)
# FreeCAD output of tests/csg_files/cylinder_box.{mcnp,xml} -- the two CSG input
# formats don't decompose into the same number/shape of cells (mcnp's own
# graveyard cell has no openmc_xml equivalent), so each format gets its own
# expected volume set. This turns test_cylbox_convertion's assertions from
# "output files exist" into a real geometric regression check. Reused as-is
# for the pyOCC engine too -- cross-kernel (FreeCAD's OCCT build vs
# pythonocc-core's own) differences are ~1e-8 relative, confirmed via a real
# pyoccenv run (2026-08-16), comfortably inside the existing 1e-6 tolerance.
#
# `openmc_xml`'s own baseline was corrected 2026-09-12: the old 5-value
# baseline (102747.4353, 1254566.101, 228095.8178 in place of a single
# 3864483.442) reflected the pre-migration FreeCAD build's own bug --
# `mcnp`'s and `openmc_xml`'s cell 2 (the auto-generated void cell) have the
# identical region definition, and once GEOReverse's real Gsplit-tolerance
# bug was fixed (see the history log's "Gsplit tolerance argument mismatch"
# entry), both formats independently reconstruct it as the exact same
# single solid (~3864483.45), cross-validating each other. `openmc_xml` has
# no cell 4 (Graveyard) equivalent -- surface 23 carries `boundary="vacuum"`
# directly instead -- so its own list has no huge 7.99e18 entry either.
_EXPECTED_VOLUMES = {
    "mcnp": [1520814.9834, 3864483.442, 20092792.2374, 7.999999999974521e18],
    "openmc_xml": [1520814.9834, 3864483.442, 20092792.2374],
}


@pytest.mark.parametrize("csg_format", ["mcnp", "openmc_xml"])
def test_cylbox_convertion(csg_format):

    if csg_format == "openmc_xml":
        suffix = ".xml"
    elif csg_format == "mcnp":
        suffix = ".mcnp"

    geo = geouned.CsgToCad()

    geo.read_csg_file(
        # csg file was made from testing/inputSTEP/cylBox.stp
        input_filename=f"tests/csg_files/cylinder_box{suffix}",
        csg_format=csg_format,
    )

    geo.build_universe()
    # "fcstd" has no pyOCC equivalent (see Modules/_occ_impl.py) -- only
    # requested under the engine that actually supports it.
    formats = ["stp", "fcstd"] if CAD_ENGINE == "freecad" else ["stp"]
    geo.export_cad(output_filename=f"tests_outputs/csgtocad/{csg_format}", format=formats)

    stp_path = Path(f"tests_outputs/csgtocad/{csg_format}.stp")
    assert stp_path.exists()
    if CAD_ENGINE == "freecad":
        assert Path(f"tests_outputs/csgtocad/{csg_format}.FCStd").exists()

    # Gload_step is engine-agnostic (geo.CAD_ENGINE-dispatched) -- works
    # identically whether this process loaded the FreeCAD or pyOCC backend.
    solids = Gload_step(str(stp_path))
    volumes = sorted(s.Volume for s in solids)
    expected = sorted(_EXPECTED_VOLUMES[csg_format])
    assert len(volumes) == len(expected)
    for v, e in zip(volumes, expected):
        assert abs(v - e) < 1e-6 * max(abs(e), 1.0)


# The 7 "exotic quadric" surfaces' own end-to-end MCNP fixtures
# (tests/csg_files/*.mcnp, see CLAUDE.md's "Known open items" -> GEOReverse
# for how each surface is implemented). `_EXOTIC_VOLUMES` is each fixture's
# own real (non-complement) solid volume(s), mm^3, computed independently
# from each fixture's own MCNP card parameters via its closed-form analytic
# formula (ellipsoid/cylinder/cone/paraboloid volumes, the one-sheet
# hyperboloid's `2*pi*a^2*(L+L^3/(3*b^2))`, the two-sheet hyperboloid's own
# integral from its vertex, the hyperbolic cylinder's and cooling tower's
# own numerically-integrated cross-sections, and the torus's closed form
# for the non-degenerate case / a Pappus integral over the kept arc for the
# degenerate ones) -- not just copied from a single run's own output.
# Writing these assertions is what first exercised 2 of these 14 fixtures
# end to end and surfaced 2 real, independent, previously-undiscovered bugs
# (both now fixed, see CLAUDE.md's own 2026-09-17 entry for the full
# derivation of each):
# 1. `Utils/boundBox.py::parabola_to_planes` built every one of its own
#    tangent-plane approximations with the normal pointing away from
#    material instead of toward it -- `paraboloid.mcnp`'s own cell 1 never
#    got a boundBox at all, so only its complement (the universe box) ever
#    appeared.
# 2. `CAD/splitFunction.py::surface_side`'s `"torus"` branch used the same
#    tube-offset sign for a degenerate torus's inner sheet as for its outer
#    one -- correct for the outer sheet (nested *around* the inner one) but
#    not the inner (a point genuinely outside the small inner-lobe solid,
#    yet still within the outer lobe's own much larger radius from the tube
#    center, was misread as "inside"), corrupting the split.
_EXOTIC_VOLUMES = {
    "ellipsoid": [837758040.9571629],
    "ellipse_cyl": [942478584.5594437],
    "elliptic_cone": [1256638322.8083227],
    "paraboloid": [157079692133.82733],
    "hyperboloid_one_sheet": [351858458176.44977],
    "hyperboloid_two_sheet_one_branch": [322152573550950.4],
    "hyperboloid_two_sheet_outside": [1562803004843846.2],
    "hyperbolic_cylinder_test": [19201456312.790123],
    "cooling_tower": [6597344551645.707],
    "torus_elliptic_nondegenerate": [1184352528.137808],
    "torus_circular_degenerate_outer": [1537740327.6399193],
    "torus_circular_degenerate_inner": [57299667.47721507],
    "torus_elliptic_degenerate_outer": [1230192262.111913],
    "torus_elliptic_degenerate_inner": [45839733.98176985],
}

# These fixtures have a real complement cell too, but it's the raw
# (or near-raw) universe box -- genuinely unbounded given no other
# constraining surface in these single-surface fixtures, and of no real
# MCNP/OpenMC modeling interest of its own (see CLAUDE.md's own
# "Complement-cell / degenerate-torus boundBox, closed out 2026-09-17"
# entry) -- so its own presence (exactly one extra solid) is checked, but
# not its precise value, which is neither meaningful nor stable across a
# BoxSettings.universe_radius change. `ellipsoid` and the 4 degenerate
# torus fixtures have no such entry: a single, fully closed surface
# (ellipsoid) or a torus whose own complement additionally never resolves
# to a real solid at all in the degenerate case (see that same entry).
_HAS_UNIVERSE_COMPLEMENT = {
    "ellipse_cyl",
    "elliptic_cone",
    "paraboloid",
    "hyperboloid_one_sheet",
    "hyperboloid_two_sheet_one_branch",
    "hyperboloid_two_sheet_outside",
    "hyperbolic_cylinder_test",
    "cooling_tower",
    "torus_elliptic_nondegenerate",
}

_UNIVERSE_LIKE_VOLUME = 1e17  # well above any real solid here, comfortably below the ~8e18 universe box


@pytest.mark.skipif(
    CAD_ENGINE == "freecad",
    reason=(
        "freecad's own exotic-quadric implementations (_freecad_impl.py) are deliberately "
        "untouched, separate constructions from occ/ocp's -- like tests/test_georeverse_occ_impl.py/"
        "_ocp_impl.py, this end-to-end regression is occ/ocp-only; most of these fixtures don't even "
        "convert under freecad yet (a real, separate gap, not caused by either fix this test's own "
        "docstring describes -- flagged, not chased down here)"
    ),
)
@pytest.mark.parametrize("name", sorted(_EXOTIC_VOLUMES))
def test_exotic_quadric_convertion(name):
    geo = geouned.CsgToCad()
    geo.read_csg_file(input_filename=f"tests/csg_files/{name}.mcnp", csg_format="mcnp")
    geo.build_universe()
    geo.export_cad(output_filename=f"tests_outputs/csgtocad/{name}", format=["stp"])

    stp_path = Path(f"tests_outputs/csgtocad/{name}.stp")
    assert stp_path.exists()

    solids = Gload_step(str(stp_path))
    volumes = [s.Volume for s in solids]

    real_volumes = sorted(v for v in volumes if v < _UNIVERSE_LIKE_VOLUME)
    complement_volumes = [v for v in volumes if v >= _UNIVERSE_LIKE_VOLUME]

    expected = sorted(_EXOTIC_VOLUMES[name])
    assert len(real_volumes) == len(expected)
    for v, e in zip(real_volumes, expected):
        assert abs(v - e) < 1e-4 * max(abs(e), 1.0)

    assert len(complement_volumes) == (1 if name in _HAS_UNIVERSE_COMPLEMENT else 0)
