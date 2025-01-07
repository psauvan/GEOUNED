############################
# Module for Cell definiton #
#############################
import logging

from ..utils import geometry_gu as GU
from ..utils.functions import get_multiplanes, get_reverseCan, get_roundCorner
from ..utils.boolean_function import BoolSequence, BoolRegion
from .cell_definition_functions import (
    gen_plane,
    gen_cylinder,
    gen_cone,
    gen_sphere,
    gen_torus,
    auxillary_plane,
    cone_apex_plane,
    V_torus_surfaces,
    U_torus_planes,
    gen_plane_sphere,
    gen_plane_cylinder,
    gen_plane_cone,
)

logger = logging.getLogger("general_logger")


def build_definition(meta_obj, Surfaces):
    solid_definition = BoolSequence(operator="OR")
    for basic_solid in meta_obj.Solids:
        comp = simple_solid_definition(basic_solid, Surfaces)
        solid_definition.append(comp)
    meta_obj.set_definition(solid_definition)


def simple_solid_definition(solid, Surfaces):
    component_definition = BoolSequence(operator="AND")

    solid_gu = GU.SolidGu(solid.Solids[0], tolerances=Surfaces.tolerances)

    roundCorner, omitFaces = get_roundCorner(solid_gu)
    for rc in roundCorner:
        rc_region = Surfaces.add_roundCorner(rc)
        component_definition.append(rc_region)

    # multiplanes,pindex = get_multiplanes(solid_gu,solid.BoundBox) #pindex are all faces index used to produced multiplanes, do not count as standard planes
    # pindex are all faces index used to produced multiplanes, do not count as standard planes
    multiplanes = get_multiplanes(solid_gu, omitFaces)
    for mp in multiplanes:
        mp_region = Surfaces.add_multiPlane(mp)
        component_definition.append(mp_region)

    revereCan = get_reverseCan(solid_gu, omitFaces)
    for cs in revereCan:
        cs_region = Surfaces.add_reverseCan(cs)
        component_definition.append(cs_region)

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
        if solid_gu.inverted:
            orient = "Reversed" if face.Orientation == "Forward" else "Forward"
        else:
            orient = face.Orientation

        if isinstance(face.Surface, GU.PlaneGu):
            plane = gen_plane(face, orient)
            plane_region = Surfaces.add_plane(plane, True)
            component_definition.append(plane_region)

        elif isinstance(face.Surface, GU.CylinderGu):
            cylinder = gen_cylinder(face)
            cylinder_region = Surfaces.add_cylinder(cylinder, orient)

            if orient == "Reversed":
                plane = gen_plane_cylinder(
                    face, solid_gu.Faces, Surfaces.tolerances
                )  # plane must be correctly oriented toward materials
                if plane is not None:
                    cylinder_region = BoolRegion.mult(cylinder_region, auxillary_plane(plane, Surfaces), label=cylinder_region)

            component_definition.append(cylinder_region)

        elif isinstance(face.Surface, GU.ConeGu):
            cone = gen_cone(face, orient)
            cone_region = Surfaces.add_cone(cone, orient)

            apex_plane = cone_apex_plane(face, orient)
            if apex_plane is not None:
                if orient == "Forward":
                    cone_region = BoolRegion.mult(cone_region, auxillary_plane(apex_plane, Surfaces), label=cone_region)
                else:
                    cone_region = BoolRegion.add(cone_region, auxillary_plane(apex_plane, Surfaces), label=cone_region)

            if orient == "Reversed":
                plane = gen_plane_cone(
                    face, solid_gu.Faces, Surfaces.tolerances
                )  # plane must be correctly oriented toward materials
                if plane is not None:
                    cone_region = BoolRegion.mult(cone_region, auxillary_plane(plane, Surfaces), label=cone_region)
            component_definition.append(cone_region)

        elif isinstance(face.Surface, GU.SphereGu):
            sphere = gen_sphere(face)
            sphere_region = Surfaces.add_sphere(sphere, orient)
            if orient == "Reversed":
                plane = gen_plane_sphere(face, solid_gu.Faces)
                if plane is not None:
                    sphere_region = BoolRegion.mult(sphere_region, auxillary_plane(plane, Surfaces), label=sphere_region)
            component_definition.append(sphere_region)

        elif isinstance(face.Surface, GU.TorusGu):
            torus = gen_torus(face, Surfaces.tolerances)
            if torus is not None:
                index, u_params = solid_gu.TorusUParams[iface]
                if index == last_torus:
                    continue
                last_torus = index
                # add if necesary additional planes following U variable
                u_closed, u_minMax = u_params

                torus_region = Surfaces.add_torus(torus, orient)
                if not u_closed:
                    U_plane_region = U_torus_planes(face, u_minMax, Surfaces)
                    torus_region = BoolRegion.mult(torus_region, U_plane_region, label=torus_region)

                if orient == "Reversed":
                    index, Vparams = solid_gu.TorusVParams[iface]
                    v_closed, VminMax = Vparams
                    if not v_closed:
                        V_torus_surface_region = V_torus_surfaces(face, VminMax, Surfaces)
                        torus_region = BoolRegion.mult(torus_region, V_torus_surface_region, label=torus_region)
                component_definition.append(torus_region)
            else:
                logger.info("Only Torus with axis along X, Y, Z axis can be reproduced")
    return component_definition
