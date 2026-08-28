#
# Module to load a STEP file
#
import logging
import os
import re

from ..utils.geouned_classes import GeounedSolid
from ..utils.data_classes import Tolerances
from ...geo import Gload_step, Gload_step_labels, Gdefeature, Gcollapse_split_rings, find_short_edges
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


def check_solid_defects(solid, tolerances):
    """
    Run every known corrupted/degenerate-geometry check against a loaded
    solid and return the reasons it currently fails (empty list if the
    solid is clean). One function, one place to extend: any future check
    should be added here so both the repair attempt and the reporting in
    load_cad automatically pick it up, instead of the two separate,
    parallel checks this replaced (one for topological validity, one for
    slivers -- the user never cared which specific check fired, only
    whether the solid is usable as-is).

    Checks currently implemented:
      - topological validity (BRepCheck_Analyzer / equivalent, via
        GSolid.is_valid()).
      - pathologically short edges relative to the solid's own BoundBox
        diagonal (geo.find_short_edges) -- a real, generalizable signature
        of a spurious/degenerate CAD feature invisible to is_valid() alone
        (confirmed live, 2026-08-27, Solidos/working_solids/"beltline
        left.stp" -- a spurious plane bridging a solid's real wall to a
        near-zero-height sliver).
    """
    reasons = []
    if not solid.is_valid():
        reasons.append("invalid topology")
    if find_short_edges(solid, tolerances.sliver_edge_rel_tol):
        reasons.append("degenerate/sliver geometry")
    return reasons


def repair_solid(solid, tolerances):
    """
    Attempt to repair a solid flagged by check_solid_defects, trying the
    fastest and most generally-applicable method first and only falling
    back to a more targeted (and more expensive) one if the cheap attempt
    doesn't fully clear every check. Returns the repaired solid once
    check_solid_defects(result) comes back empty (or, for the split-ring
    branch, once geo.Gcollapse_split_rings' own valid + volume-conserved
    check passes -- see below), or None if nothing tried leaves the solid
    usable.
    """
    # fix() is cheap, general-purpose, and already the standard repair
    # for invalid topology; it can also incidentally clear some sliver
    # cases (face/edge unification), so it's always worth trying first.
    repaired = solid.fix(1e-6)
    if not check_solid_defects(repaired, tolerances):
        return repaired

    # Targeted: "split boundary ring" / duplicated micro-trim -- a single
    # trimming surface duplicated at a sub-tolerance offset, with parasitic
    # curved "riser" faces bridging the thin slab and every curved face on
    # the trim carrying a doubled boundary ring. Every generic OCCT healer
    # (fix/refine/UnifySameDomain/Defeaturing) and any boolean re-cut
    # no-ops or fails on this. Gcollapse_split_rings removes the riser
    # faces and re-sews; it validates valid + |dV| < 1% internally and
    # returns None when it doesn't apply or doesn't converge. Its result
    # may keep residual sub-0.05mm connector edges (an internal-wire
    # micro-tab sewing can't weld) -- HARMLESS, verified via an MCNP
    # stochastic-volume check (barrel bottom.stp: tally 0.9997, 0 lost
    # particles) -- so accept it on its own return, NOT on a clean
    # check_solid_defects re-check (which would reject it for those edges).
    collapsed = Gcollapse_split_rings(repaired, tolerances.min_face_width)
    if collapsed is not None:
        return collapsed

    # Still failing (typically: a sliver fix() doesn't touch) -- try the
    # more targeted, more expensive BRepAlgoAPI_Defeaturing pass, seeded
    # from whichever short edges remain after fix().
    degenerate_faces = find_short_edges(repaired, tolerances.sliver_edge_rel_tol)
    if degenerate_faces:
        healed = Gdefeature(repaired, degenerate_faces)
        if healed is not None and not check_solid_defects(healed, tolerances):
            return healed

    return None


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

    Solids = Gload_step(filename)
    meta_list = []
    spline_solids = []
    corrupted_solids_list = []
    loop_spline = spline_surf.lower() in ("remove", "stop")
    loop_corrupted = corrupted_solids.lower() == "remove"
    for i, s in enumerate(Solids):
        # One unified check (check_solid_defects: topological validity +
        # sliver/degenerate-edge detection today, any future check added
        # there automatically) instead of two separate, differently-gated
        # checks -- the user doesn't care which specific check fired, only
        # whether the solid is usable. A repair is always attempted
        # (repair_solid: fix() first, since it's cheap/general-purpose and
        # already the standard repair for invalid topology; falls back to
        # the more targeted Gdefeature only if fix() alone doesn't clear
        # every check) -- the corrupted_solids mode is only consulted for
        # what to do once repair has genuinely failed, not to gate whether
        # repair is attempted at all.
        if check_solid_defects(s, tolerances):
            healed = repair_solid(s, tolerances)
            if healed is not None:
                s = healed
            else:
                corrupted_solids_list.append(i)
                if loop_corrupted:
                    meta_list.append(LF.GeounedSolid(i + 1))
                    continue
        if LF.spline(s):
            spline_solids.append(i)
            if loop_spline:
                meta_list.append(LF.GeounedSolid(i + 1))
                continue
        meta_list.append(GeounedSolid(i + 1, s))   

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
