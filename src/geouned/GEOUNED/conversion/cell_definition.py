############################
# Module for Cell definiton #
#############################
import logging

from ..utils import geometry_gu as GU
from ..utils.geouned_classes import GeounedSurface
from ..utils.boolean_solids import build_c_table_from_solids, remove_extra_surfaces
from ..utils.functions import (
    get_multiplanes,
    get_roundCorner,
    get_Can,
    get_TCone,
    get_reversed_cone_cylinder,
    shapes_in_contact,
    get_box,
)
from ..utils.boolean_function import BoolSequence
from ..utils.meta_surfaces_utils import merge_same_surface_faces
from ..decompose.decom_utils_generator import omit_isolated_planes
from .cell_definition_functions import (
    gen_plane,
    gen_cylinder,
    gen_cone,
    gen_sphere,
    gen_torus,
    cone_apex_plane,
    check_torus_bounds,
    V_torus_surface,
    U_torus_planes,
    oneplane_surface,
    one_torus_plane,
    one_degenerated_torus_plane,
    gen_plane_sphere,
    omit_multiplane_repeated_planes,
    torus_face_configuration,
)

logger = logging.getLogger("general_logger")


def build_definition(meta_obj, Surfaces, simplifyComp=True):

    solid_definition = BoolSequence(operator="OR")
    for basic_solid in meta_obj.Solids:
        comp = simple_solid_definition(basic_solid, Surfaces)
        # if simplifyComp:
        # comp.expand_regions_to_boolVar()
        # comp.simplify()
        solid_definition.append(comp)
    meta_obj.set_definition(solid_definition)


def simple_solid_definition(solid, Surfaces, meta_surfaces=True):
    component_definition = BoolSequence(operator="AND")

    solid_gu = GU.SolidGu(solid.Solids[0], tolerances=Surfaces.tolerances)
    # A genuinely small decomposed piece (e.g. residual sliver-adjacent
    # fragment) can have real, legitimate faces whose own area/width falls
    # below the absolute min_area/min_face_width defaults -- scaled once
    # here, per solid, rather than changing the defaults globally. See
    # Tolerances.scaled()'s own docstring for the full rationale.
    scaled_tolerances = Surfaces.tolerances.scaled(solid_gu.Volume)
    if meta_surfaces:
        RFCan, omitFaces = get_Can(solid_gu.Faces)
        for cs in RFCan:
            if cs.Orientation == "Reversed":
                cs_region = Surfaces.add_reverseCan(cs)
            else:
                cs_region = Surfaces.add_forwardCan(cs)
            component_definition.append(cs_region)
        omit_isolated_planes(solid_gu.Faces, omitFaces)

        RFTCone = get_TCone(solid_gu.Faces, omitFaces)
        for cs in RFTCone:
            if cs.Orientation == "Reversed":
                cs_region = Surfaces.add_reverseTCone(cs)
            else:
                cs_region = Surfaces.add_forwardTCone(cs)
            component_definition.append(cs_region)
        omit_isolated_planes(solid_gu.Faces, omitFaces)

        roundCorner = get_roundCorner(solid_gu.Faces, omitFaces, solid=solid_gu)
        for rc in roundCorner:
            if rc.Type == "MultiRoundCorner":
                rc_region = Surfaces.add_multiRoundCorner(rc)
            else:
                rc_region = Surfaces.add_roundCorner(rc)
            component_definition.append(rc_region)

        # multiplanes,pindex = get_multiplanes(solid_gu,solid.BoundBox) #pindex are all faces index used to produced multiplanes, do not count as standard planes
        # pindex are all faces index used to produced multiplanes, do not count as standard planes
        multiplanes = get_multiplanes(solid_gu.Faces, omitFaces, scaled_tolerances)
        for mp in multiplanes:
            mp_region = Surfaces.add_multiPlane(mp)
            component_definition.append(mp_region)
            planeset = omit_multiplane_repeated_planes(mp_region, Surfaces, solid_gu.Faces)
            omitFaces.update(planeset)

        # `multiplanes` is threaded through as the real MultiPlane list, not
        # just a boolean gate -- get_join_cone_cyl needs it to identify
        # *which* of a RevCC's own 2 chain ends (if any) borders one of
        # these, not just whether any exist in the solid.
        reversedCC = get_reversed_cone_cylinder(solid_gu.Faces, multiplanes, scaled_tolerances, omitFaces)
        for cs in reversedCC:
            cc_region = Surfaces.add_reversedCC(cs)
            component_definition.append(cc_region)

    else:
        omitFaces = set()
        omit_isolated_planes(solid_gu.Faces, omitFaces)

    last_torus = -1
    for iface, face in enumerate(solid_gu.Faces):
        if iface in omitFaces:
            continue
        if abs(face.Area) < scaled_tolerances.min_area:
            logger.warning(
                f"{str(face.Surface)} surface removed from cell definition. Face area < Min area ({face.Area} < {scaled_tolerances.min_area})"
            )
            continue
        if getattr(face, "CharacteristicWidth", float("inf")) < scaled_tolerances.min_face_width:
            # min_area alone doesn't catch a real, large-area but genuinely
            # thin sliver face of ANY surface type (see
            # Tolerances.min_face_width's own docstring, and
            # order_plane_face's identical check on the decomposition
            # side) -- confirmed live, 2026-08-23,
            # Solidos/test_models/Decomposed/SCDR_90_piece2.stp: a
            # 1.9262mm^2 sliver plane (area comfortably above the default
            # min_area=0.01) still leaked into the written cell definition
            # as a spurious extra bounding plane via this exact loop. Not
            # restricted to GPlane -- a thin sliver Cylinder/Cone/Sphere/
            # Torus patch is exactly the same class of artifact and
            # CharacteristicWidth is already computed generically for
            # every surface type.
            logger.warning(
                f"{str(face.Surface)} surface removed from cell definition. Face characteristic width < min_face_width "
                f"({face.CharacteristicWidth} < {scaled_tolerances.min_face_width})"
            )
            continue
        if face.Area < 0:
            logger.warning("Negative surface Area")
        if face.Orientation not in ("Forward", "Reversed"):
            continue

        shell = merge_same_surface_faces(face, solid_gu.Faces)
        omitFaces.update(shell.Indexes if isinstance(shell, GU.ShellFaceGu) else {face.Index})

        if isinstance(face.Surface, GU.GPlane):
            plane = gen_plane(face)
            plane_region = Surfaces.add_plane(plane, True)
            component_definition.append(plane_region)

        elif isinstance(face.Surface, GU.GCylinder):
            # this branch doesn't need additional plane for
            # reversed orientation, because open reversed orientation
            # is handled by RevCC, and closed reversed orientation
            # doesn't need additional plane.
            cylinderOnly = gen_cylinder(face)
            cylinder = GeounedSurface(("Cylinder", (cylinderOnly, None), face.Orientation))
            cylinder_region = Surfaces.add_cylinder(cylinder)
            component_definition.append(cylinder_region)

        elif isinstance(face.Surface, GU.GCone):
            # this branch doesn't need additional plane for
            # reversed orientation, because open reversed orientation
            # is handled by RevCC, and closed reversed orientation
            # doesn't need additional plane.

            coneOnly = gen_cone(face)
            apexPlane = cone_apex_plane(face, Surfaces.tolerances)

            cone = GeounedSurface(("Cone", (coneOnly, apexPlane, None), face.Orientation))
            cone_region = Surfaces.add_cone(cone)
            component_definition.append(cone_region)

        elif isinstance(face.Surface, GU.GSphere):
            sphereOnly = gen_sphere(face)
            if face.Orientation == "Reversed":
                plane = gen_plane_sphere(shell)
            else:
                plane = None

            sphere = GeounedSurface(("Sphere", (sphereOnly, plane), face.Orientation))
            sphere_region = Surfaces.add_sphere(sphere)
            component_definition.append(sphere_region)

        elif isinstance(face.Surface, GU.GTorus):
            torusOnly = gen_torus(face, Surfaces.tolerances)
            if torusOnly is not None:
                Urange, Vrange = check_torus_bounds(shell)
                Uclosed, Uparams = Urange
                Vclosed, Vparams = Vrange

                Uplanes = []
                Vsurface = []
                Vconfig = None

                if type(shell) is GU.ShellFaceGu:
                    degenerated = shell.Faces[0].Surface.Degenerated
                else:
                    degenerated = shell.Surface.Degenerated

                face_orientation = face.Orientation
                if degenerated:
                    if face.Surface.a_sign < 0:
                        # A self-intersecting torus's two sheets are written
                        # as a single, signed-major-radius surface card (see
                        # write/functions.py) -- which physical side of that
                        # one card counts as "inside" for this cell flips
                        # depending on which sheet the real face belongs to.
                        face_orientation = "Reversed" if face.Orientation == "Forward" else "Forward"
                    if not (Uclosed and Vclosed) and face_orientation == "Reversed":
                        Uplanes = one_degenerated_torus_plane(shell, Surfaces)
                else:
                    if Uclosed and not Vclosed:
                        if shell.Orientation == "Reversed":
                            Vsurface, Vconfig = V_torus_surface(shell, Vparams, Surfaces)
                    elif not Uclosed and Vclosed:
                        Uplanes = U_torus_planes(shell, Uparams, Surfaces)
                    elif not Uclosed and not Vclosed:
                        radius_ratio = shell.Surface.MajorRadius / shell.Surface.MinorRadius
                        if oneplane_surface(Uparams, Vparams, radius_ratio) and False:
                            Uplanes = one_torus_plane(shell, Uparams, Vparams, Surfaces)
                        else:
                            Uplanes = U_torus_planes(shell, Uparams, Surfaces)
                            if shell.Orientation == "Reversed":
                                Vsurface, Vconfig = V_torus_surface(shell, Vparams, Surfaces)

                torus = GeounedSurface(("Torus", (torusOnly, Uplanes, Vsurface, Vconfig), face_orientation, degenerated))
                torus_region = Surfaces.add_torus(torus)
                component_definition.append(torus_region)
            else:
                logger.info("Only Torus with axis along X, Y, Z axis can be reproduced")

    return component_definition


def noOverlapCell(m, i, meta_list, surfaces, options):
    complementary_cells = []
    for other_cell in meta_list[0:i]:
        if other_cell.CellType != "solid" or other_cell.NullCell:
            continue
        if shapes_in_contact(m.CADSolid, other_cell.CADSolid):
            complementary_cells.append(other_cell)

    if complementary_cells:
        process_overlap(m, complementary_cells, surfaces, options)


def process_overlap(m, complementary_cells, surfaces, options):

    new_def = BoolSequence(operator="AND")
    new_def.append(m.Definition.copy())

    cell_def = new_def.copy()
    newcomp = False
    for comp in complementary_cells:
        Seq = cell_def.copy()
        compDef = comp.Definition.get_complementary()
        Seq.append(compDef)
        Seq.simplify()
        if type(Seq.elements) is list:
            new_def.append(compDef)
            newcomp = True

    if not newcomp:
        return

    new_def.simplify()
    if new_def.level == 0:
        return

    cell_def.simplify()
    if new_def == cell_def:
        return

    box = get_box(m, options.enlargeBox)

    # evaluate only diagonal elements of the Constraint Table (fastest) and remove surface not
    # crossing in the solid boundBox
    CT = build_c_table_from_solids(
        box,
        (tuple(new_def.get_surfaces_numbers()), surfaces),
        "diag",
        options=options,
    )

    new_def = remove_extra_surfaces(new_def, CT)

    # evaluate full constraint Table with less surfaces involved
    CT = build_c_table_from_solids(
        box,
        (tuple(new_def.get_surfaces_numbers()), surfaces),
        "full",
        options=options,
    )

    new_def.simplify(CT)
    # new_def.simplify_sequence(CT, surfaces=cell_def.get_surfaces_numbers())
    new_def.clean()
    new_def.join_operators()
    new_def.same_level()

    m.set_definition(new_def)
