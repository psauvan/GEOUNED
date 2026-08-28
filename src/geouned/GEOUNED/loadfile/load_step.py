#
# Module to load a STEP file
#
import logging
import os
import re

from ..utils.data_classes import Tolerances
from ...geo import Gload_and_process_step, Gload_step_labels
from . import load_functions as LF

logger = logging.getLogger("general_logger")


# Paco mod
def extract_materials(filename):
    rho_real = []
    m_dict = {}  # _ Material dictionary
    with open(filename, "rt") as file:
        for line in file:
            vals = line.split()
            if vals[0].startswith("#"):
                continue
            mat_label = int(vals[0])
            rho_real = -float(vals[1])
            matname = " ".join(vals[2:])
            m_dict[mat_label] = (rho_real, matname)
    return m_dict


def load_cad(filename, spline_surf, settings, options, corrupted_solids="stop", tolerances=None):

    if tolerances is None:
        tolerances = Tolerances()

    if settings.matFile != "":
        if os.path.exists(settings.matFile):
            m_dict = extract_materials(settings.matFile)
        else:
            logger.info(f"Material definition file {settings.matFile} does not exist.")
            m_dict = {}
    else:
        m_dict = {}

    gsolid_list, corrupted_solids_list, spline_solids = Gload_and_process_step(
        filename, tolerances.sliver_edge_rel_tol, tolerances.min_face_width
    )

    # Gload_and_process_step always keeps a spline-bearing solid's real
    # geometry (it has no notion of spline_surfaces' own 3-way stop/
    # remove/ignore mode -- see its own docstring) -- only "stop"/"remove"
    # null it out here; "ignore" genuinely attempts translation on the
    # as-loaded geometry, exactly as it did before this loop moved into
    # geo. corrupted_solids has no such 3rd mode (stop/remove only, no
    # "keep it anyway"), so Gload_and_process_step already returns None
    # for those unconditionally -- nothing to gate here.
    loop_spline = spline_surf.lower() in ("remove", "stop")
    meta_list = []
    for i, s in enumerate(gsolid_list):
        if i in spline_solids and loop_spline:
            s = None
        meta_list.append(LF.GeounedSolid(i + 1, s))

    i_solid = 0
    missing_mat = set()

    nodes = Gload_step_labels(filename)
    removed_labels = dict()
    removed_indexes = corrupted_solids_list + spline_solids
    
    if removed_indexes:
        stop_process = (spline_surf.lower() == "stop") or (corrupted_solids.lower() == "stop")
    else:
        stop_process = False    

    for i,node in enumerate(nodes):
        comment = LF.getCommentTree(node, options)
        tempre_mat = None
        tempre_dil = None

        # MIO: lightly modification of label if required
        label = LF.get_label(node.label, options)
        comment = comment + "/" + label
        if i in removed_indexes :
            removed_labels[i] = comment
        if node.parent is not None:
            # MIO: lightly modification of label if required
            label_in_list = LF.get_label(node.parent.label, options)
            encl_label = re.search("enclosure(?P<encl>[0-9]+)_(?P<parent>[0-9]+)_", label_in_list)
            if not encl_label:
                encl_label = re.search("enclosure(?P<encl>[0-9]+)_(?P<parent>[0-9]+)_", label)

            envel_label = re.search("envelope(?P<env>[0-9]+)_(?P<parent>[0-9]+)_", label_in_list)
            if not envel_label:
                envel_label = re.search("envelope(?P<env>[0-9]+)_(?P<parent>[0-9]+)_", label)

            # Paco modifications
            # Search for material definition in tree
            current = node
            while current is not None and not tempre_mat:
                # MIO: Modification of label if required
                temp_label = LF.get_label(current.label, options)
                tempre_mat = re.search("_m(?P<mat>\d+)_", "_" + temp_label)
                current = current.parent

            # Search for dilution definition in tree
            current = node
            while current is not None and not tempre_dil:
                # MIO: Modification of label if required
                temp_label = LF.get_label(current.label, options)
                tempre_dil = re.search("_d(?P<dil>\d*\.\d*)_", temp_label)
                current = current.parent
            # Paco end
        else:
            encl_label = None
            envel_label = None

        # compSolid Diferent solid of the same cell are stored in the same metaObject (compSolid)
        # enclosures and envelopes are always stored as compound
        if settings.compSolids or encl_label or envel_label:

            init = i_solid
            end = i_solid + node.n_solids
            LF.fuse_meta_obj(meta_list, init, end)
            n_solids = 1
        else:
            n_solids = node.n_solids

        for i in range(n_solids):
            meta_list[i_solid].set_comments(f"{comment}{i + 1}")
            meta_list[i_solid].set_cad_solid()

            if tempre_mat:
                mat_label = int(tempre_mat.group("mat"))
                if mat_label in m_dict.keys():
                    meta_list[i_solid].set_material(mat_label, m_dict[mat_label][0], m_dict[mat_label][1])
                else:
                    if mat_label == 0:
                        meta_list[i_solid].set_material(mat_label, 0, 0)
                    else:
                        meta_list[i_solid].set_material(
                            mat_label,
                            -100,
                            "Missing material density information",
                        )
                        missing_mat.add(mat_label)
            else:
                # logger.warning('No material label associated to solid {}.\nDefault material used instead.'.format(comment))
                if settings.voidMat:
                    meta_list[i_solid].set_material(*settings.voidMat)
            if tempre_dil:
                meta_list[i_solid].set_dilution(float(tempre_dil.group("dil")))

            if encl_label is not None:
                meta_list[i_solid].EnclosureID = int(encl_label.group("encl"))
                meta_list[i_solid].ParentEnclosureID = int(encl_label.group("parent"))
                meta_list[i_solid].IsEnclosure = True
                meta_list[i_solid].CellType = "void"

            if envel_label is not None:
                meta_list[i_solid].EnclosureID = int(envel_label.group("env"))
                meta_list[i_solid].ParentEnclosureID = int(envel_label.group("parent"))
                meta_list[i_solid].IsEnclosure = True
                meta_list[i_solid].CellType = "envelope"
            i_solid += 1

    LF.display_removed_solids(corrupted_solids_list, spline_solids, removed_labels )
    if stop_process: 
        print("Corrupted solids or solids with splines found. More information in log file.")
        print("Exit process")
        exit()

    LF.joinEnvelopes(meta_list)
    if missing_mat:
        logger.warning("At least one material in the CAD model is not present in the material file")
        logger.info(f"List of not present materials: {missing_mat}")

    enclosure_list = LF.set_enclosure_solid_list(meta_list)
    if enclosure_list:
        LF.check_enclosure(enclosure_list)
        # LF.remove_enclosure(meta_list)
        return meta_list, enclosure_list
    else:
        return meta_list, []
