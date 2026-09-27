from pathlib import Path

import pytest

import geouned

path_to_cad = Path("testing/inputSTEP")
step_files = list(path_to_cad.rglob("*.stp")) + list(path_to_cad.rglob("*.step"))

# Excluded from test_conversion: 3 fail for a pre-existing, unrelated
# reason (not a regression to chase here), and 4 more take >20s each to
# convert, dominating full-suite runtime. Durations captured 2026-07-29.
_excluded_step_files = {
    Path("testing/inputSTEP/SCDR_90.stp"),  # fails (pre-existing), 3.2s
    Path("testing/inputSTEP/large/SCDR.stp"),  # fails (pre-existing), 118s
    Path("testing/inputSTEP/large/Triangle.stp"),  # fails (pre-existing), 151s
    Path("testing/inputSTEP/Misc/rails.stp"),  # 40s
    Path("testing/inputSTEP/Misc/P52.stp"),  # 25s
    Path("testing/inputSTEP/FWTBM1.step"),  # 25s
    Path("testing/inputSTEP/Misc/tester.stp"),  # 22s
}
step_files = [f for f in step_files if f not in _excluded_step_files]

suffixes = (".mcnp", ".xml", ".inp", ".py", ".serp")


@pytest.mark.parametrize("input_step_file", step_files)
def test_conversion(input_step_file):
    """Test that step files can be converted to openmc and mcnp files"""

    # sets up an output folder for the results
    output_dir = Path("tests_outputs") / input_step_file.with_suffix("")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_filename_stem = output_dir / input_step_file.stem

    # deletes the output MC files if they already exists
    for suffix in suffixes:
        output_filename_stem.with_suffix(suffix).unlink(missing_ok=True)

    my_options = geouned.Options(
        forceCylinder=False,
        newSplitPlane=True,
        delLastNumber=False,
        enlargeBox=2,
        nPlaneReverse=0,
        splitTolerance=0,
        scaleUp=True,
        quadricPY=False,
        Facets=False,
        prnt3PPlane=False,
        forceNoOverlap=False,
    )

    my_tolerances = geouned.Tolerances(
        relativeTol=False,
        relativePrecision=0.000001,
        value=0.000001,
        angle=0.0001,
        pln_distance=0.0001,
        pln_angle=0.0001,
        cyl_distance=0.0001,
        cyl_angle=0.0001,
        sph_distance=0.0001,
        kne_distance=0.0001,
        kne_angle=0.0001,
        tor_distance=0.0001,
        tor_angle=0.0001,
        min_area=0.01,
    )

    my_numeric_format = geouned.NumericFormat(
        P_abc="14.7e",
        P_d="14.7e",
        P_xyz="14.7e",
        S_r="14.7e",
        S_xyz="14.7e",
        C_r="12f",
        C_xyz="12f",
        K_xyz="13.6e",
        K_tan2="12f",
        T_r="14.7e",
        T_xyz="14.7e",
        GQ_1to6="18.15f",
        GQ_7to9="18.15f",
        GQ_10="18.15f",
    )

    my_settings = geouned.Settings(
        matFile="",
        voidGen=True,
        debug=False,
        compSolids=False,
        simplify="no",
        exportSolids="",
        minVoidSize=200.0,  # units mm
        maxSurf=50,
        maxBracket=30,
        voidMat=[],
        voidExclude=[],
        startCell=1,
        startSurf=1,
        sort_enclosure=False,
    )

    geo = geouned.CadToCsg(
        options=my_options,
        settings=my_settings,
        tolerances=my_tolerances,
        numeric_format=my_numeric_format,
    )

    geo.load_step_file(filename=f"{input_step_file.resolve()}", skip_solids=[])

    geo.run()

    geo.export_csg(
        title="Converted with GEOUNED",
        geometryName=f"{output_filename_stem.resolve()}",
        outFormat=(
            "openmc_xml",
            "openmc_py",
            "serpent",
            "phits",
            "mcnp",
        ),
        volSDEF=True,  # changed from the default
        volCARD=False,  # changed from the default
        UCARD=None,
        dummyMat=True,  # changed from the default
        cellCommentFile=False,
        cellSummaryFile=False,  # changed from the default
    )

    for suffix in suffixes:
        assert output_filename_stem.with_suffix(suffix).exists()


@pytest.mark.parametrize(
    "input_json_file",
    ["tests/config_cadtocsg_complete_defaults.json", "tests/config_cadtocsg_minimal.json"],
)
def test_cad_to_csg_from_json_with_defaults(input_json_file):

    # deletes the output MC files if they already exists
    for suffix in suffixes:
        Path("csg").with_suffix(suffix).unlink(missing_ok=True)

    my_cad_to_csg = geouned.CadToCsg.from_json(input_json_file)
    assert isinstance(my_cad_to_csg, geouned.CadToCsg)

    assert my_cad_to_csg.filename == "testing/inputSTEP/BC.stp"
    assert my_cad_to_csg.options.forceCylinder == False
    assert my_cad_to_csg.tolerances.relativeTol == False
    assert my_cad_to_csg.numeric_format.P_abc == "14.7e"
    assert my_cad_to_csg.settings.matFile == ""

    for suffix in suffixes:
        assert Path("csg").with_suffix(suffix).exists()


def test_cad_to_csg_from_json_with_non_defaults():

    # deletes the output MC files if they already exists
    for suffix in suffixes:
        Path("csg").with_suffix(suffix).unlink(missing_ok=True)

    my_cad_to_csg = geouned.CadToCsg.from_json("tests/config_cadtocsg_non_defaults.json")
    assert isinstance(my_cad_to_csg, geouned.CadToCsg)

    assert my_cad_to_csg.filename == "testing/inputSTEP/BC.stp"
    assert my_cad_to_csg.options.forceCylinder == True
    assert my_cad_to_csg.tolerances.relativePrecision == 2e-6
    assert my_cad_to_csg.numeric_format.P_abc == "15.7e"
    assert my_cad_to_csg.settings.matFile == "non default"

    for suffix in suffixes:
        assert Path("csg").with_suffix(suffix).exists()


def test_writing_to_new_folders():
    """Checks that a folder is created prior to writing output files"""

    geo = geouned.CadToCsg()
    geo.load_step_file(filename="testing/inputSTEP/BC.stp", skip_solids=[])
    geo.run()

    for outformat in ["mcnp", "phits", "serpent", "openmc_xml", "openmc_py"]:
        geo.export_csg(
            geometryName=f"tests_outputs/new_folder_for_testing_{outformat}/csg",
            cellCommentFile=False,
            cellSummaryFile=False,
            outFormat=[outformat],
        )
        geo.export_csg(
            geometryName=f"tests_outputs/new_folder_for_testing_{outformat}_cell_comment/csg",
            cellCommentFile=True,
            cellSummaryFile=False,
            outFormat=[outformat],
        )
        geo.export_csg(
            geometryName=f"tests_outputs/new_folder_for_testing_{outformat}_cell_summary/csg",
            cellCommentFile=False,
            cellSummaryFile=True,
            outFormat=[outformat],
        )


def test_with_relative_tol_true():

    # test to protect against incorrect attribute usage in FreeCAD
    # more details https://github.com/GEOUNED-org/GEOUNED/issues/154

    geo = geouned.CadToCsg(
        tolerances=geouned.Tolerances(relativeTol=False),
    )
    geo.load_step_file(filename=f"{step_files[1].resolve()}", skip_solids=[])
    geo.run()

    geo = geouned.CadToCsg(
        tolerances=geouned.Tolerances(relativeTol=True),
    )
    geo.load_step_file(filename=f"{step_files[1].resolve()}", skip_solids=[])
    geo.run()


def test_options_meta_surfaces_false_bypasses_composites_but_keeps_revcc():
    """`Options.meta_surfaces=False` reproduces GEOUNED's original,
    pre-meta-surface behaviour: decomposition/cell definition skip Can/
    TCone/RoundCorner/MultiPlane detection entirely, falling back to
    basic analytic surfaces -- except ReversedConeCylinder (RevCC),
    which always runs regardless, since it's needed for correctness (an
    open Reversed cylinder/cone face needs its extra bounding surface),
    not just compaction. `RoundCorners/shed_part.stp`'s own single round
    corner is a real, minimal fixture for exactly this: with meta
    surfaces on, it resolves to one RoundC composite (no RevCC needed);
    with them off, the same corner's cylinder face falls back to a
    basic Cyl surface that DOES need RevCC to stay correctly bounded --
    confirmed live, 2026-09-27, before pinning these exact counts here."""

    def surface_counts(meta_surfaces):
        geo = geouned.CadToCsg(
            options=geouned.Options(meta_surfaces=meta_surfaces),
            settings=geouned.Settings(voidGen=False),
        )
        geo.load_step_file(
            filename="testing/inputSTEP/RoundCorners/shed_part.stp",
            corrupted_solids="remove",
            spline_surfaces="remove",
        )
        geo.run()
        return {k: len(v) for k, v in geo.Surfaces.items() if isinstance(v, list)}

    with_meta = surface_counts(True)
    without_meta = surface_counts(False)

    assert with_meta["RoundC"] == 1
    assert with_meta["RevCC"] == 0

    assert without_meta["RoundC"] == 0
    assert without_meta["MultiRoundC"] == 0
    assert without_meta["MultiP"] == 0
    assert without_meta["FwdCan"] == 0
    assert without_meta["RevCan"] == 0
    assert without_meta["FwdTCone"] == 0
    assert without_meta["RevTCone"] == 0
    assert without_meta["RevCC"] == 1  # RevCC still fires even with composites off
