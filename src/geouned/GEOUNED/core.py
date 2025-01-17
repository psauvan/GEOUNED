import json
import logging
import typing
from datetime import datetime
from pathlib import Path
from typing import get_type_hints
from importlib.metadata import version
import time

import FreeCAD
import Part
from tqdm import tqdm

from .code_version import *
from .conversion import cell_definition as Conv
from .cuboid.translate import translate

# from .decompose.decom_one import main_split
from .decompose.decom_one_generators import main_split
from .loadfile import load_step as Load
from .utils.geouned_classes import GeounedSolid, SurfacesDict, MetaSurfacesDict
from .utils.functions import get_box
from .utils.boolean_solids import build_c_table_from_solids
from .utils.data_classes import NumericFormat, Options, Settings, Tolerances
from .threading.geouned_threads import ThreadPoolExecutor
from .void import void as void
from .write.functions import write_mcnp_cell_def
from .write.write_files import write_geometry

logger = logging.getLogger("general_logger")
# logger.info(f"GEOUNED version {version('geouned')}")
logger.info(f"FreeCAD version {'.'.join(FreeCAD.Version()[:3])}")


class CadToCsg:
    """Base class for the conversion of CAD to CSG models

    Args:
        options (geouned.Options, optional): An instance of a geouned.Options
            class with the attributes set for the desired conversion. Defaults
            to a geouned.Options with default attributes values.
        tolerances (geouned.Tolerances, optional): An instance of a
            geouned.Tolerances class with the attributes set for the desired
            conversion. Defaults to a geouned.Tolerances with default
            attributes values.
        numeric_format (geouned.NumericFormat, optional): An instance of a
            geouned.NumericFormat class with the attributes set for the desired
            conversion. Defaults to a geouned.NumericFormat with default
            attributes values.
        settings (geouned.Settings, optional): An instance of a
            geouned.Settings class with the attributes set for the desired
            conversion. Defaults to a geouned.Settings with default
            attributes values.
    """

    def __init__(
        self,
        options: Options = Options(),
        tolerances: Tolerances = Tolerances(),
        numeric_format: NumericFormat = NumericFormat(),
        settings: Settings = Settings(),
    ):

        self.options = options
        self.tolerances = tolerances
        self.numeric_format = numeric_format
        self.settings = settings

        # define later when running the code
        self.geometry_bounding_box = None
        self.meta_list = []
        self.filename = None
        self.skip_solids = []

    @property
    def options(self):
        return self._options

    @options.setter
    def options(self, value: Options):
        if not isinstance(value, Options):
            raise TypeError(f"geouned.CadToCsg.options should be an instance of geouned.Options, not a {type(value)}")
        self._options = value

    @property
    def tolerances(self):
        return self._tolerances

    @tolerances.setter
    def tolerances(self, value: tolerances):
        if not isinstance(value, Tolerances):
            raise TypeError(f"geouned.CadToCsg.tolerances should be an instance of geouned.Tolerances, not a {type(value)}")
        self._tolerances = value

    @property
    def numeric_format(self):
        return self._numeric_format

    @numeric_format.setter
    def numeric_format(self, value: numeric_format):
        if not isinstance(value, NumericFormat):
            raise TypeError(
                f"geouned.CadToCsg.numeric_format should be an instance of geouned.NumericFormat, not a {type(value)}"
            )
        self._numeric_format = value

    @property
    def settings(self):
        return self._settings

    @settings.setter
    def settings(self, value: settings):
        if not isinstance(value, Settings):
            raise TypeError(f"geouned.CadToCsg.settings should be an instance of geouned.Settings, not a {type(value)}")
        self._settings = value

    def export_csg(
        self,
        title: str = "Converted with GEOUNED",
        geometryName: str = "csg",
        outFormat: typing.Tuple[str] = (
            "openmc_xml",
            "openmc_py",
            "serpent",
            "phits",
            "mcnp",
        ),
        volSDEF: bool = False,
        volCARD: bool = True,
        UCARD: typing.Union[int, None] = None,
        dummyMat: bool = False,
        cellCommentFile: bool = False,
        cellSummaryFile: bool = True,
    ):
        """Writes out a CSG file in the requested Monte Carlo code format.

        Args:
            title (str, optional): Title of the model written at the top of the
                output file. Defaults to "Geouned conversion".
            geometryName (str, optional): the file stem of the output file(s).
                Defaults to "converted_with_geouned".
            outFormat (typing.Tuple[str], optional): Format for the output
                geometry. Available format are: "mcnp", "openmc_xml",
                "openmc_py", "phits" and "serpent". Several output format can
                be written in the same method call. Defaults to output all codes.
            volSDEF (bool, optional):  Write SDEF definition and tally of solid
                cell for stochastic volume checking. Defaults to False.
            volCARD (bool, optional): Write the CAD calculated volume in the
                cell definition using the VOL card. Defaults to True.
            UCARD (int, optional): Write universe card in the cell definition
                with the specified universe number (if value = 0 Universe card
                is not written). Defaults to None.
            dummyMat (bool, optional): Write dummy material definition card in
               the MCNP output file for all material labels present in the
               model. Dummy material definition is "MX 1001 1". Defaults to False.
            cellCommentFile (bool, optional): Write an additional file with
               comment associated to each CAD cell in the MCNP output file.
               Defaults to False.
            cellSummaryFile (bool, optional): Write an additional file with
               information on the CAD cell translated. Defaults to True.
        """

        if not isinstance(UCARD, int) and not isinstance(UCARD, type(None)):
            raise TypeError(f"UCARD should be of type int or None not {type(UCARD)}")
        if isinstance(UCARD, int):
            if UCARD < 0:
                raise ValueError("UCARD should be a 0 or a positive integer ")

        for arg, arg_str in (
            (volSDEF, "volSDEF"),
            (volCARD, "volCARD"),
            (dummyMat, "dummyMat"),
            (cellCommentFile, "cellCommentFile"),
            (cellSummaryFile, "cellSummaryFile"),
        ):
            if not isinstance(arg, bool):
                raise TypeError(f"{arg} should be of type bool not {type(arg_str)}")

        for arg, arg_str in ((title, "title"), (geometryName, "geometryName")):
            if not isinstance(arg, str):
                raise TypeError(f"{arg} should be of type str not {type(arg_str)}")

        # if the geometry_bounding_box has not previuosly been calculated, then make a default one
        if self.geometry_bounding_box is None:
            self._set_geometry_bounding_box()

        write_geometry(
            UniverseBox=self.geometry_bounding_box,
            MetaList=self.meta_list,
            Surfaces=self.Surfaces,
            settings=self.settings,
            options=self.options,
            tolerances=self.tolerances,
            numeric_format=self.numeric_format,
            geometryName=geometryName,
            outFormat=outFormat,
            cellCommentFile=cellCommentFile,
            cellSummaryFile=cellSummaryFile,
            title=title,
            volSDEF=volSDEF,
            volCARD=volCARD,
            UCARD=UCARD,
            dummyMat=dummyMat,
            step_filename=self.filename,
        )

        logger.info("End of Monte Carlo code translation phase")

    @classmethod
    def from_json(cls, filename: str):
        """Creates a CadToCsg instance, runs CadToCsg.load_Step_file(), runs
        CadToCsg.start() and returns the instance. Populating the arguments for
        the methods that are run by looking for keys with the same name as the
        method in the JSON file. For example CadToCsg.start() accepts arguments
        for Options, Tolerance, Settings and NumericFormat and can be populated
        from matching key names in the JSON. If export_to_csg key is present
        then this method also runs CadToCsg.export_to_csg() on the instance.

        Args:
            filename str: The filename of the config file.

        Raises:
            FileNotFoundError: If the config file is not found
            ValueError: If the config JSON file is found to contain an invalid key

        Returns:
            geouned.CadToCsg: returns a geouned CadToCsg class instance.
        """

        if not Path(filename).exists():
            raise FileNotFoundError(f"config file {filename} not found")

        with open(filename) as f:
            config = json.load(f)

        cad_to_csg = cls()

        for key in config.keys():

            if key in ["load_step_file", "export_csg"]:
                pass  # these two keys are used before or after this loop

            elif key == "Tolerances":
                cad_to_csg.tolerances = Tolerances(**config["Tolerances"])

            elif key == "Options":
                cad_to_csg.options = Options(**config["Options"])

            elif key == "NumericFormat":
                cad_to_csg.numeric_format = NumericFormat(**config["NumericFormat"])

            elif key == "Settings":
                cad_to_csg.settings = Settings(**config["Settings"])

            else:
                raise ValueError(
                    f"Invalid key '{key}' found in config file {filename}. Acceptable key names are 'load_step_file', 'export_csg', 'Settings', 'Parameters', 'Tolerances' and 'NumericFormat'"
                )

        cad_to_csg.load_step_file(**config["load_step_file"])
        cad_to_csg.run()
        if "export_csg" in config.keys():
            cad_to_csg.export_csg(**config["export_csg"])
        else:
            cad_to_csg.export_csg()
        return cad_to_csg

    def load_step_file(
        self,
        filename: typing.Union[str, typing.Sequence[str]],
        skip_solids: typing.Sequence[int] = [],
    ):
        """
        Load STEP file(s) and extract solid volumes and enclosure volumes.

        Args:
            filename (str): The path to the STEP file or a list of paths to multiple STEP files.
            skip_solids (Sequence[int], optional): A sequence (list or tuple) of indexes of solids to not load for conversion.

        Returns:
            tuple: A tuple containing the solid volumes list and enclosure volumes list extracted from the STEP files.
        """
        logger.info("Start of step file loading phase")

        if not isinstance(skip_solids, (list, tuple)):
            raise TypeError(f"skip_solids should be a list, tuple of ints, not a {type(skip_solids)}")
        for entry in skip_solids:
            if not isinstance(entry, int):
                raise TypeError(f"skip_solids should contain only ints, not a {type(entry)}")

        if not isinstance(filename, (str, list, tuple)):
            raise TypeError(f"filename should be a str or a sequence of str, not a {type(filename)}")
        if isinstance(filename, (list, tuple)):
            for entry in filename:
                if not isinstance(entry, str):
                    raise TypeError(f"filename should contain only str, not a {type(entry)}")

        self.filename = filename
        self.skip_solids = skip_solids

        if isinstance(filename, (list, tuple)):
            step_files = filename
        else:
            step_files = [filename]

        for step_file in step_files:
            if not Path(step_file).is_file():
                raise FileNotFoundError(f"Step file {step_file} not found.")

        MetaChunk = []
        EnclosureChunk = []
        for step_file in tqdm(step_files, desc="Loading CAD files"):
            logger.info(f"read step file : {step_file}")
            Meta, Enclosure = Load.load_cad(step_file, self.settings, self.options)
            MetaChunk.append(Meta)
            EnclosureChunk.append(Enclosure)
        self.meta_list = join_meta_lists(MetaChunk)
        self.enclosure_list = join_meta_lists(EnclosureChunk)

        # deleting the solid index in reverse order so the indexes don't change for subsequent deletions
        for solid_index in sorted(skip_solids, reverse=True):
            logger.info(f"Removing solid index: {solid_index} from list of {len(self.meta_list)} solids")
            del self.meta_list[solid_index]

        logger.info("End of step file loading phase")

        if self.settings.exportSolids:
            self._export_solids(filename=self.settings.exportSolids)
        logger.info("End of loading phase")

        return self.meta_list, self.enclosure_list

    def run(self):
        t0 = time.time()
        self.decompose_solids()
        t1 = time.time()
        self.build_solid_definition()
        t2 = time.time()
        self.build_void()
        t3 = time.time()
        print(f'decomposition : {t1-t0}s\nbuild : {t2-t1}s\nvoid : {t3-t2}s')

    def decompose_solids(self):

        # decompose all solids in elementary solids (convex ones)
        self._decompose_solids(meta=True)

        # decompose Enclosure solids
        if self.settings.voidGen and self.enclosure_list:
            self._decompose_solids(meta=False)

    def build_solid_definition(self):
        # start Building CGS cells phase
        self.Surfaces = MetaSurfacesDict(options=self.options, tolerances=self.tolerances, numeric_format=self.numeric_format)

        if self.options.n_thread > 1:
            onlysolidList = []
            for m in self.meta_list:
                if m.IsEnclosure:
                    continue
                onlysolidList.append(m)

            ThreadPoolExecutor(
                Conv.build_definition,
                onlysolidList,
                target_options=(self.Surfaces,),
                max_thread=self.options.n_thread,
                progress_bar_label="Translating solid cells",
            )
            if self.settings.voidGen and self.enclosure_list:
                ThreadPoolExecutor(
                    Conv.build_definition,
                    self.enclosure_list,
                    target_options=(self.Surfaces,),
                    max_thread=self.options.n_thread,
                    progress_bar_label="Translating Encloures",
                )
        else:
            #  building solids
            for m in tqdm(self.meta_list, desc="Translating solid cells"):
                if m.IsEnclosure:
                    continue
                Conv.build_definition(m, self.Surfaces)

            #  building enclosure regions
            if self.settings.voidGen and self.enclosure_list:
                for m in tqdm(self.enclosure_list, desc="Translating Enclosures"):
                    Conv.build_definition(m, self.Surfaces)

    def build_void(self):
       # sets self.geometry_bounding_box with default padding
        self._set_geometry_bounding_box()

        # void generation phase
        meta_void = []
        if self.settings.voidGen:
            logger.info("Build Void")
            logger.info(self.settings.voidExclude)
            if not self.settings.voidExclude:
                meta_reduced = self.meta_list
            else:
                meta_reduced = exclude_cells(self.meta_list, self.settings.voidExclude)

            if self.meta_list:
                init = self.meta_list[-1].__id__ - len(self.enclosure_list)
            else:
                init = 0

            meta_void = void.void_generation(
                meta_reduced,
                self.enclosure_list,
                self.Surfaces,
                self.geometry_bounding_box,
                self.settings,
                init,
                self.options,
                self.tolerances,
                self.numeric_format,
            )

        # if self.settings.simplify == 'full' and not self.options.forceNoOverlap:
        if self.settings.simplify == "full":
            Surfs = {}
            for lst in self.Surfaces.values():
                for s in lst:
                    Surfs[s.Index] = s

            for c in tqdm(self.meta_list, desc="Simplifying"):
                if c.Definition.level == 0 or c.IsEnclosure:
                    continue
                logger.info(f"simplify cell {c.__id__}")
                Box = get_box(c, self.options.enlargeBox)
                CT = build_c_table_from_solids(Box, (c.Surfaces, Surfs), "full", options=self.options)
                c.Definition.simplify(CT)
                c.Definition.clean()
                if type(c.Definition.elements) is bool:
                    logger.info(f"unexpected constant cell {c.__id__} :{c.Definition.elements}")

        cellOffSet = self.settings.startCell - 1
        if self.enclosure_list and self.settings.sort_enclosure:
            # sort group solid cell / void cell sequence in each for each enclosure
            # if a solid belong to several enclosure, its definition will be written
            # for the highest enclosure level or if same enclosure level in the first
            # enclosure found
            self.meta_list = sort_enclosure(self.meta_list, meta_void, cellOffSet)
        else:
            # remove Null Cell and apply cell numbering offset
            deleted = []
            idLabel = {0: 0}
            icount = cellOffSet
            for i, m in enumerate(self.meta_list):
                if m.NullCell or m.IsEnclosure:
                    deleted.append(i)
                    continue

                icount += 1
                m.label = icount
                idLabel[m.__id__] = m.label

            for i in reversed(deleted):
                del self.meta_list[i]

            lineComment = """\
##########################################################
             VOID CELLS
##########################################################"""
            mc = GeounedSolid(None)
            mc.Comments = lineComment
            self.meta_list.append(mc)

            deleted = []
            for i, m in enumerate(meta_void):
                if m.NullCell:
                    deleted.append(i)
                    continue
                icount += 1
                m.label = icount
                update_comment(m, idLabel)
            for i in reversed(deleted):
                del meta_void[i]

            self.meta_list.extend(meta_void)

    def _export_solids(self, filename: str):
        """Export all the solid volumes from the loaded geometry to a STEP file.

        Args:
            filename (str): filepath of the output STEP file.
        """
        # export in STEP format solids read from input file
        if self.meta_list == []:
            raise ValueError(
                "No solids in CadToCsg.meta_list to export. Try loading the STEP file first with CadToCsg.load_step_file"
            )
        solids = []
        for m in self.meta_list:
            if m.IsEnclosure:
                continue
            solids.extend(m.Solids)
        Part.makeCompound(solids).exportStep(filename)

    def _set_geometry_bounding_box(self, padding: float = 10.0):
        """
        Get the bounding box of the geometry.

        Args:
            padding (float): The padding value to add to the bounding box dimensions.

        Returns:
            FreeCAD.BoundBox: The universe bounding box.
        """
        # set up Universe
        meta_list = self.meta_list

        Box = meta_list[0].optimalBoundingBox()
        xmin = Box.XMin
        xmax = Box.XMax
        ymin = Box.YMin
        ymax = Box.YMax
        zmin = Box.ZMin
        zmax = Box.ZMax
        for m in meta_list[1:]:
            # MIO. This was removed since in HELIAS the enclosure cell is the biggest one
            # if m.IsEnclosure: continue
            optBox = m.optimalBoundingBox()
            xmin = min(optBox.XMin, xmin)
            xmax = max(optBox.XMax, xmax)
            ymin = min(optBox.YMin, ymin)
            ymax = max(optBox.YMax, ymax)
            zmin = min(optBox.ZMin, zmin)
            zmax = max(optBox.ZMax, zmax)

        self.geometry_bounding_box = FreeCAD.BoundBox(
            FreeCAD.Vector(xmin - padding, ymin - padding, zmin - padding),
            FreeCAD.Vector(xmax + padding, ymax + padding, zmax + padding),
        )

    def _decompose_solids(self, meta: bool):

        if meta:
            meta_list = self.meta_list
            description = "Decomposing solids"
        else:
            meta_list = self.enclosure_list
            description = "Decomposing enclosure solids"

        if self.settings.debug:
            self.debug_output_folder = Path("debug")
            self.debug_output_folder.mkdir(parents=True, exist_ok=True)

        if self.options.n_thread > 1:
            ThreadPoolExecutor(
                self._decompose_target,
                meta_list,
                target_options=[meta],
                max_thread=self.options.n_thread,
                progress_bar_label=description,
                add_index=True,
            )
        else:
            for i, m in enumerate(tqdm(meta_list, desc=description)):
                self._decompose_target(m, i, meta)

    def _decompose_target(self, m, i, meta):

        if meta and m.IsEnclosure:
            return

        if self.settings.debug:
            if m.IsEnclosure:
                m.Solids[0].exportStep(str(self.debug_output_folder / f"origEnclosure_{i}.stp"))
            else:
                m.Solids[0].exportStep(str(self.debug_output_folder / f"origSolid_{i}.stp"))

        comsolid = main_split(
            Part.makeCompound(m.Solids),
            self.options,
            self.tolerances,
        )

        if False:  # todo decomposition error information
            sus_output_folder = Path("suspicious_solids")
            sus_output_folder.mkdir(parents=True, exist_ok=True)
            if m.IsEnclosure:
                Part.CompSolid(m.Solids).exportStep(str(sus_output_folder / f"Enclosure_original_{i}.stp"))
                comsolid.exportStep(str(sus_output_folder / f"Enclosure_split_{i}.stp"))
            else:
                Part.CompSolid(m.Solids).exportStep(str(sus_output_folder / f"Solid_original_{i}.stp"))
                comsolid.exportStep(str(sus_output_folder / f"Solid_split_{i}.stp"))

            warningSolids.append(i)

        if self.settings.debug:
            if m.IsEnclosure:
                comsolid.exportStep(str(self.debug_output_folder / f"/compEnclosure_{i}.stp"))
            else:
                comsolid.exportStep(str(self.debug_output_folder / f"compSolid_{i}.stp"))

        m.set_cad_solid()
        m.update_solids(comsolid.Solids)


def update_comment(meta, idLabel):
    if meta.__commentInfo__ is None:
        return
    if meta.__commentInfo__[1] is None:
        return
    newLabel = (idLabel[i] for i in meta.__commentInfo__[1])
    meta.set_comments(void.void_comment_line((meta.__commentInfo__[0], newLabel)))


def print_warning_solids(warnSolids, warnEnclosures):

    solids_logger = logging.getLogger("solids_logger")

    if warnSolids or warnEnclosures:
        pass
    else:
        return

    if warnSolids:
        lines = "Solids :\n"
        for sol in warnSolids:
            lines += "\n"
            lines += f"{sol.label}\n"
            lines += f"{sol.Comments}\n"
            lines += f"{write_mcnp_cell_def(sol.Definition)}\n"
        solids_logger.info(lines)

    if warnEnclosures:
        lines = "Enclosures :\n"
        for sol in warnEnclosures:
            lines += "\n"
            lines += f"{sol.label}\n"
            lines += f"{sol.Comments}\n"
            lines += f"{write_mcnp_cell_def(sol.Definition)}\n"

        solids_logger.info(lines)


def join_meta_lists(MList) -> typing.List[GeounedSolid]:

    newMetaList = MList[0]
    if MList[0]:
        for M in MList[1:]:
            lastID = newMetaList[-1].__id__ + 1
            for i, meta in enumerate(M):
                meta.__id__ = lastID + i
                newMetaList.append(meta)

    return newMetaList


def exclude_cells(MetaList, labelList):
    voidMeta = []
    for m in MetaList:
        if m.IsEnclosure:
            continue
        found = False
        for label in labelList:
            if label in m.Comments:
                found = True
                break
        if not found:
            voidMeta.append(m)

    return voidMeta


def sort_enclosure(MetaList, meta_void, offSet=0):

    newList = {}
    for m in meta_void:
        if m.EnclosureID in newList.keys():
            newList[m.EnclosureID].append(m)
        else:
            newList[m.EnclosureID] = [m]

    icount = offSet
    idLabel = {0: 0}
    newMeta = []
    for m in MetaList:
        if m.NullCell:
            continue
        if m.IsEnclosure:
            lineComment = f"""##########################################################
             ENCLOSURE {m.EnclosureID}
##########################################################"""
            mc = GeounedSolid(None)
            mc.Comments = lineComment
            newMeta.append(mc)
            for e in newList[m.EnclosureID]:
                if e.NullCell:
                    continue
                icount += 1
                e.label = icount
                idLabel[e.__id__] = e.label
                newMeta.append(e)
            lineComment = f"""##########################################################
            END  ENCLOSURE {m.EnclosureID}
##########################################################"""
            mc = GeounedSolid(None)
            mc.Comments = lineComment
            newMeta.append(mc)

        else:
            icount += 1
            m.label = icount
            idLabel[m.__id__] = m.label
            newMeta.append(m)

    lineComment = """\
##########################################################
             VOID CELLS 
##########################################################"""
    mc = GeounedSolid(None)
    mc.Comments = lineComment
    newMeta.append(mc)

    for v in newList[0]:
        if v.NullCell:
            continue
        icount += 1
        v.label = icount
        idLabel[v.__id__] = v.label
        newMeta.append(v)

    for m in newMeta:
        if not m.Void:
            continue
        if m.IsEnclosure:
            continue
        update_comment(m, idLabel)

    return newMeta
