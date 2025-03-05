import typing
import FreeCAD
import Import

from pathlib import Path

from .Modules.buildCAD import buildCAD, makeTree
from .Modules.MCNPinput import McnpInput
from .Modules.XMLinput import XmlInput


class CsgToCad:
    """Base class for the conversion of CSG to CAD models"""

    def __init__(self, settings):
        self.settings = settings
        self.cell_range_type = "all"
        self.cell_range = None
        self.mat_range_type = "all"
        self.mat_range = None
        self.build_universe = dict()

    def read_csg_file(self, input_filename: str, csg_format: str):
        """get geometry definition from OpenMC XML or MCNP input.
        Args:
            input_filename (str): The filename and path of the input CSG text file.
            csg_format (str): The format of the CSG input file, options are 'openmc_xml' or 'mcnp'"""

        if csg_format == "mcnp":
            self.geometry = McnpInput(input_filename)
        elif csg_format == "openmc_xml":
            self.geometry = XmlInput(input_filename)
        else:
            msg = f"input format type {csg_format} is not supported. Supported options are 'openmc_xml' or 'mcnp'"
            raise ValueError(msg)

    def select_materials_cells(self, materials=None, cells=None):
        """select material and/or cell to build export the CSG geometry in OpenMC or MCNP format to a CAD model."""

        if cells is not None:
            if type(cells) is str:
                cells = (cells,)

            if cells[0] in ("exclude", "include", "all"):
                self.cell_range_type = cells[0]
            else:
                print("bad cells range type. Ignored")

            if len(cells) > 1:
                self.cell_range = cells[1]
            else:
                self.cell_range = None
            if self.cell_range is None:
                if self.cell_range_type in ("exclude", "include"):
                    self.cell_range_type = "all"

        if materials is not None:
            if type(materials) is str:
                materials = (materials,)

            if materials[0] in ("exclude", "include", "all"):
                self.mat_range_type = materials[0]
            else:
                print("bad materials range type. Ignored")

            if len(materials) > 1:
                self.mat_range = materials[1]
            else:
                self.mat_range = None
            if self.mat_range is None:
                if self.mat_range_type in ("exclude", "include"):
                    self.mat_range_type = "all"

    def build_universe(self, U=0, depth=-1, in_container=True):

        UniverseCut = True
        UnivCell = CadCell(self.settings)
        UnivCell.name = U
        UnivCell.Fill = U

        # read all surfaces definition
        modelSurfaces = self.geometry.GetSurfaces()  # scale units change are carried out in GetSurfaces method

        # read Cells and group into universes
        matcel_list = {
            "mat": (self.mat_range_type, self.mat_range),
            "cell": (self.mat_range_type, self.mat_range),
        }
        levels, UniverseCells, modelSurfaces = self.geometry.GetFilteredCells(modelSurfaces, matcel_list)

        # assign to each cell the surfaces belonging to the cell
        AssignSurfaceToCell(UniverseCells, modelSurfaces)

        UnivCell.level = None
        levelMax = depth
        Ustart = U
        if levelMax == -1:
            levelMax = len(levels)

        for lev, Univ in levels.items():
            if Ustart in Univ:
                UnivCell.level = lev - 1
                break
        startInfo = (Ustart, levelMax)

        CADCells, fails = BuildUniverse(startInfo, UnivCell, UniverseCells, universeCut=UniverseCut)

        self.build_universe[U] = CADCells
        if fails:
            print("failed in conversion", fails)

    def export_cad(self, output_filename: str = "cad_from_csg"):
        """export the CSG geometry in OpenMC or MCNP format to a CAD model.

        Args:
            output_filename (str, optional): The filename stem and path of the output file created.
                Two files will be created with the '.step' suffix and one with the 'FCStd' suffix.
                Defaults to 'cad_from_csg'.

        Raises:
            ValueError: If the csg_format is not 'openmc_xml' or 'mcnp' then a ValueError is raised.
        """

        # TODO check file extensions are correct
        # if Path(output_filename).suffix not in ['.stp', '.step']:
        #     raise ValueError(f"output file must have a .stp or .step extension, not {universe_start.suffix}")

        Path(output_filename).parent.mkdir(parents=True, exist_ok=True)

        # TODO don't return fails variable, just fail in the method and raise the error there

        CADdoc = FreeCAD.newDocument("converted_with_geouned")

        makeTree(CADdoc, self.CADCells)
        Import.export(CADdoc.Objects[0:1], f"{output_filename}.step")
        CADdoc.saveAs(f"{output_filename}.FCStd")
