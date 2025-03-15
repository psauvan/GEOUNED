import FreeCAD
import Import

from pathlib import Path

from .Modules.buildCAD import makeTree, AssignSurfaceToCell, BuildUniverseCells
from .Modules.Utils.booleanFunction import BoolSequence
from .Modules.Utils.boundBox import solid_plane_box
from .Modules.data_class import BoxSettings
from .Modules.Objects import CadCell
from .Modules.MCNPinput import McnpInput
from .Modules.XMLinput import XmlInput


class CsgToCad:
    """Base class for the conversion of CSG to CAD models"""

    def __init__(self, settings: BoxSettings):
        self.settings = settings
        self.cell_range_type = "all"
        self.cell_range = None
        self.mat_range_type = "all"
        self.mat_range = None
        self.buildCAD_list = []

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
            # read all surfaces definition

        self.geometry.GetSurfaces()  # scale units change are carried out in GetSurfaces method
        self.geometry.GetLevelStructure()

    def cell_filter(self, type="all", cells=None):
        """select cell to build export the CSG geometry in OpenMC or MCNP format to a CAD model."""

        if type in ("exclude", "include", "all"):
            self.cell_range_type = type
        else:
            self.cell_range_type = "all"
            print("bad cells range type. Ignored")

        if cells is not None:
            self.cell_range = cells[:]
        else:
            if self.cell_range_type == "exclude":
                self.cell_range_type = "all"

    def material_filter(self, type="all", materials=None):
        """select material to build export the CSG geometry in OpenMC or MCNP format to a CAD model."""

        if type in ("exclude", "include", "all"):
            self.mat_range_type = type
        else:
            self.mat_range_type = "all"
            print("bad cells range type. Ignored")

        if materials is not None:
            self.mat_range = materials[:]
        else:
            if self.mat_range_type == "exclude":
                self.mat_range_type = "all"

    def build_container(self, cell_name, depth=-1):

        # get and build container universe
        UnivCell = self.geometry.GetCell(cell_name, self.settings)
        UnivCell.definition = BoolSequence(UnivCell.definition.str)

        solid_box = solid_plane_box(UnivCell)
        bBox = solid_box.get_boundBox(hashBox=True, enlarge=0.1)
        if bBox.XLength < 1e-6 or bBox.YLength < 1e-6 or bBox.ZLength < 1e-6:
            UnivCell.shape = None
            print(f"Cell {UnivCell.name} BoundBox is null")

        debug = False
        if debug:
            UnivCell.buildShape(bBox, hashbox=True, simplify=False)
        else:
            try:
                UnivCell.buildShape(bBox, hashbox=True, simplify=False)
            except:
                print(f"fail converting cell {UnivCell.name}")

        # generate universe cells inside container
        matcel_list = {
            "mat": (self.mat_range_type, self.mat_range),
            "cell": (self.cell_range_type, self.cell_range),
        }

        UniverseCells, modelSurfaces = self.geometry.GetFilteredCells(UnivCell.FILL, depth, matcel_list, self.settings)
        AssignSurfaceToCell(UniverseCells, modelSurfaces)

        UnivCell.level = None
        levelMax = depth
        Ustart = UnivCell.FILL
        if levelMax == -1:
            levelMax = len(self.geometry.levels)

        for lev, Univ in self.geometry.levels.items():
            if Ustart in Univ:
                UnivCell.level = lev - 1
                break
        startInfo = (Ustart, levelMax)
        CADCells, fails = BuildUniverseCells(startInfo, UnivCell, UniverseCells, universeCut=True)
        if fails:
            print("failed cell conversion:", fails)
        self.buildCAD_list.append(CADCells)

    def build_universe(self, U=0, depth=-1):

        UniverseCut = True
        UnivCell = CadCell(settings=self.settings)
        UnivCell.name = 0
        UnivCell.Fill = U
        UnivCell.MAT = 0

        # read Cells and group into universes
        matcel_list = {
            "mat": (self.mat_range_type, self.mat_range),
            "cell": (self.cell_range_type, self.cell_range),
        }
        UniverseCells, modelSurfaces = self.geometry.GetFilteredCells(U, depth, matcel_list, self.settings)

        # assign to each cell the surfaces belonging to the cell
        AssignSurfaceToCell(UniverseCells, modelSurfaces)

        UnivCell.level = None
        levelMax = depth
        Ustart = U
        if levelMax == -1:
            levelMax = len(self.geometry.levels)

        for lev, Univ in self.geometry.levels.items():
            if Ustart in Univ:
                UnivCell.level = lev - 1
                break
        startInfo = (Ustart, levelMax)

        CADCells, fails = BuildUniverseCells(startInfo, UnivCell, UniverseCells, universeCut=UniverseCut)
        self.buildCAD_list.append(CADCells)
        if fails:
            print("failed cell conversion:", fails)

    def export_cad(self, output_filename: str = "cad_from_csg"):
        """export the CSG geometry in OpenMC or MCNP format to a CAD model.

        Args:
            output_filename (str, optional): The filename stem and path of the output file created.
                Two files will be created with the '.step' suffix and one with the 'FCStd' suffix.
                Defaults to 'cad_from_csg'.

        Raises:
            ValueError: If the csg_format is not 'openmc_xml' or 'mcnp' then a ValueError is raised.
        """

        Path(output_filename).parent.mkdir(parents=True, exist_ok=True)

        fullname = Path(output_filename).name
        suffix = Path(output_filename).suffix
        if suffix != "":
            barename = fullname[0 : -len(suffix)]
            output_filename = output_filename[0 : -len(suffix)]
        else:
            barename = fullname

        if suffix not in (".stp", ".step"):
            suffix = ".stp"

        CADdoc = FreeCAD.newDocument("converted_with_geouned")

        CADobj = CADdoc.addObject("App::Part", "Universes")
        CADobj.Label = barename

        for CAD in self.buildCAD_list:
            CADobj.addObject(makeTree(CADdoc, CAD))

        Import.export(CADdoc.Objects[0:1], output_filename + suffix)
        CADdoc.saveAs(f"{output_filename}.FCStd")
