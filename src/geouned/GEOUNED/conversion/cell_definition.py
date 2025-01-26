############################
# Module for Cell definiton #
#############################
import logging

from ..utils import geometry_gu as GU
from ..utils.geouned_classes import GeounedSurface
from ..utils.functions import get_multiplanes, get_reverseCan, get_roundCorner, get_reversed_cone_cylinder
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


def build_definition(meta_obj, Surfaces):

    solid_definition = BoolSequence(operator="OR")
    for i, basic_solid in enumerate(meta_obj.Solids):
        comp = simple_solid_definition(basic_solid, Surfaces)
        solid_definition.append(comp)
    meta_obj.set_definition(solid_definition)


def simple_solid_definition(solid, Surfaces, meta_surfaces=True):
    component_definition = BoolSequence(operator="AND")

    solid_gu = GU.SolidGu(solid.Solids[0], tolerances=Surfaces.tolerances)
    multiplane_surface = False
    if meta_surfaces:
        roundCorner, omitFaces = get_roundCorner(solid_gu.Faces)
        for rc in roundCorner:
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

        reverseCan = get_reverseCan(solid_gu.Faces, omitFaces)
        for cs in reverseCan:
            cs_region = Surfaces.add_reverseCan(cs)
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
        if solid_gu.inverted:
            orient = "Reversed" if face.Orientation == "Forward" else "Forward"
        else:
            orient = face.Orientation

        if isinstance(face.Surface, GU.PlaneGu):
            plane = gen_plane(face, orient)
            plane_region = Surfaces.add_plane(plane, True)
            component_definition.append(plane_region)

        elif isinstance(face.Surface, GU.CylinderGu):
            cylinderOnly = gen_cylinder(face)
            if orient == "Reversed":
                plane = gen_plane_cylinder(
                    face, solid_gu.Faces, Surfaces.tolerances
                )  # plane must be correctly oriented toward materials
            else:
                plane = None

            cylinder = GeounedSurface(("Cylinder", (cylinderOnly, plane, [], orient)))
            cylinder_region = Surfaces.add_cylinder(cylinder)
            component_definition.append(cylinder_region)

        elif isinstance(face.Surface, GU.ConeGu):
            coneOnly = gen_cone(face)
            apexPlane = cone_apex_plane(face, orient, Surfaces.tolerances)
            if orient == "Reversed":
                plane = gen_plane_cone(
                    face, solid_gu.Faces, Surfaces.tolerances
                )  # plane must be correctly oriented toward materials
            else:
                plane = None

            cone = GeounedSurface(("Cone", (coneOnly, apexPlane, plane, orient)))
            cone_region = Surfaces.add_cone(cone)
            component_definition.append(cone_region)

        elif isinstance(face.Surface, GU.SphereGu):
            sphereOnly = gen_sphere(face)
            plane = None
            if orient == "Reversed":
                plane = gen_plane_sphere(face, solid_gu.Faces)
            else:
                plane = None

            sphere = GeounedSurface(("Sphere", (sphereOnly, plane, orient)))
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
                if orient == "Reversed":
                    index, Vparams = solid_gu.TorusVParams[iface]
                    v_closed, VminMax = Vparams
                    if not v_closed:
                        VSurface, surf_orientation = V_torus_surfaces(face, VminMax, Surfaces)

                torus = GeounedSurface(("Torus", (torusOnly, UPlanes, VSurface, orient, surf_orientation)))
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
