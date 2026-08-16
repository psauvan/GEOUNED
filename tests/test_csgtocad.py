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
_EXPECTED_VOLUMES = {
    "mcnp": [1520814.9834, 3864483.442, 20092792.2374, 7.999999999974521e18],
    "openmc_xml": [1520814.9834, 102747.4353, 1254566.101, 228095.8178, 20092792.2374],
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
