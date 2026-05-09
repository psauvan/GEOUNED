############################
# Module for Cell definiton #
#############################
import logging

from ..utils import geometry_gu as GU
from ..utils.geouned_classes import GeounedSurface
from ..utils.boolean_solids import build_c_table_from_solids, remove_extra_surfaces
from ..utils.functions import get_multiplanes, get_roundCorner, get_reversed_cone_cylinder, get_Can, my_dist_to_shape, get_box
from ..utils.boolean_function import BoolSequence
from ..decompose.decom_utils_generator import omit_isolated_planes
from .cell_definition_functions import (
    gen_plane,
    gen_cylinder,
    gen_cone,
    gen_sphere,
    gen_torus,
    cone_apex_plane,
    V_torus_surfaces,
    U_torus_planes,
    gen_plane_sphere,
    gen_plane_cylinder,
    gen_plane_cone,
    omit_multiplane_repeated_planes,
)

logger = logging.getLogger("general_logger")


def build_definition(meta_obj, Surfaces, simplifyComp=True):

    solid_definition = BoolSequence(operator="OR")
    for basic_solid in meta_obj.Solids:
        comp = simple_solid_definition(basic_solid, Surfaces)
        if simplifyComp:
            comp.expand_regions_to_boolVar()
            comp.simplify()
        solid_definition.append(comp)
    meta_obj.set_definition(solid_definition)


def simple_solid_definition(solid, Surfaces, meta_surfaces=True):
    component_definition = BoolSequence(operator="AND")

    solid_gu = GU.SolidGu(solid.Solids[0], tolerances=Surfaces.tolerances)
    multiplane_surface = False
    if meta_surfaces:
        roundCorner, omitFaces = get_roundCorner(solid_gu.Faces)
        for rc in roundCorner:
            if rc.Type == "MultiRoundCorner":
                rc_region = Surfaces.add_multiRoundCorner(rc)
            else:
                rc_region = Surfaces.add_roundCorner(rc)
            component_definition.append(rc_region)

        # multiplanes,pindex = get_multiplanes(solid_gu,solid.BoundBox) #pindex are all faces index used to produced multiplanes, do not count as standard planes
        # pindex are all faces index used to produced multiplanes, do not count as standard planes
        multiplanes = get_multiplanes(solid_gu.Faces, omitFaces)
        for mp in multiplanes:
            mp_region = Surfaces.add_multiPlane(mp)
            component_definition.append(mp_region)
            planeset = omit_multiplane_repeated_planes(mp_region, Surfaces, solid_gu.Faces)
            omitFaces.update(planeset)
            multiplane_surface = True

        RFCan = get_Can(solid_gu.Faces, omitFaces)
        for cs in RFCan:
            if cs.Orientation == "Reversed":
                cs_region = Surfaces.add_reverseCan(cs)
            else:
                cs_region = Surfaces.add_forwardCan(cs)
            component_definition.append(cs_region)
        omit_isolated_planes(solid_gu.Faces, omitFaces)

        reversedCC = get_reversed_cone_cylinder(solid_gu.Faces, multiplane_surface, omitFaces)
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
        if abs(face.Area) < Surfaces.tolerances.min_area:
            logger.warning(
                f"{str(face.Surface)} surface removed from cell definition. Face area < Min area ({face.Area} < {Surfaces.tolerances.min_area})"
            )
            continue
        if face.Area < 0:
            logger.warning("Negative surface Area")
        if face.Orientation not in ("Forward", "Reversed"):
            continue

        if isinstance(face.Surface, GU.PlaneGu):
            plane = gen_plane(face)
            plane_region = Surfaces.add_plane(plane, True)
            component_definition.append(plane_region)

        elif isinstance(face.Surface, GU.CylinderGu):
            cylinderOnly = gen_cylinder(face)
            if face.Orientation == "Reversed":
                plane = gen_plane_cylinder(
                    face, solid_gu.Faces, Surfaces.tolerances
                )  # plane must be correctly oriented toward materials
            else:
                plane = None

            cylinder = GeounedSurface(("Cylinder", (cylinderOnly, plane), face.Orientation))
            cylinder_region = Surfaces.add_cylinder(cylinder)
            component_definition.append(cylinder_region)

        elif isinstance(face.Surface, GU.ConeGu):
            coneOnly = gen_cone(face)
            apexPlane = cone_apex_plane(face, Surfaces.tolerances)
            if face.Orientation == "Reversed":
                plane = gen_plane_cone(
                    face, solid_gu.Faces, Surfaces.tolerances
                )  # plane must be correctly oriented toward materials
            else:
                plane = None

            cone = GeounedSurface(("Cone", (coneOnly, apexPlane, plane), face.Orientation))
            cone_region = Surfaces.add_cone(cone)
            component_definition.append(cone_region)

        elif isinstance(face.Surface, GU.SphereGu):
            sphereOnly = gen_sphere(face)
            plane = None
            if face.Orientation == "Reversed":
                plane = gen_plane_sphere(face, solid_gu.Faces)
            else:
                plane = None

            sphere = GeounedSurface(("Sphere", (sphereOnly, plane), face.Orientation))
            sphere_region = Surfaces.add_sphere(sphere)
            component_definition.append(sphere_region)

        elif isinstance(face.Surface, GU.TorusGu):
            torusOnly = gen_torus(face, Surfaces.tolerances)
            if torusOnly is not None:
                index, u_params = solid_gu.TorusUParams[iface]
                if index == last_torus:
                    continue
                last_torus = index
                # add if necesary additional planes following U variable
                u_closed, u_minMax = u_params

                if not u_closed:
                    UPlanes = U_torus_planes(face, u_minMax, Surfaces)
                else:
                    UPlanes = []

                VSurface, surf_orientation = None, None
                if face.Orientation == "Reversed":
                    index, Vparams = solid_gu.TorusVParams[iface]
                    v_closed, VminMax = Vparams
                    if not v_closed:
                        VSurface, surf_orientation = V_torus_surfaces(face, VminMax, Surfaces)

                torus = GeounedSurface(("Torus", (torusOnly, UPlanes, VSurface, surf_orientation), face.Orientation))
                torus_region = Surfaces.add_torus(torus)
                component_definition.append(torus_region)
            else:
                logger.info("Only Torus with axis along X, Y, Z axis can be reproduced")

    # solid.exportStep('solid.stp')
    # for k in Surfaces.keys():
    #    for i,m in enumerate(Surfaces[k]):
    #        m.build_surface(solid.BoundBox)
    #        m.shape.exportStep(f'{k}_{i}.stp')
    return component_definition


def noOverlapCell(m, i, meta_list, surfaces, options):
    complementary_cells = []
    for other_cell in meta_list[0:i]:
        if other_cell.CellType != "solid" or other_cell.NullCell:
            continue
        if my_dist_to_shape(m.CADSolid, other_cell.CADSolid) < 1e-6:
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
