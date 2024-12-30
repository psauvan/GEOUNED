############################
# Module for Cell definiton #
#############################
import logging

from ..utils import geometry_gu as GU
from ..utils.functions import get_multiplanes
from ..utils.boolean_function import BoolSequence,BoolRegion
from .cell_definition_functions import (
    is_inverted,
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
    gen_plane_cone
)

logger = logging.getLogger("general_logger")
  
def decompose(solid,Surfaces):
    solid_definition = BoolSequence(operator='OR')
    for basic_solid in solid:
        comp = simple_solid_definition(basic_solid,Surfaces)
        solid_definition.append(comp)
    return solid_definition    

def simple_solid_definition(solid,Surfaces):
    component_definition = BoolSequence(operator="AND")

    flag_inv = is_inverted(solid.Solids[0])
    solid_gu = GU.SolidGu(solid.Solids[0], tolerances=Surfaces.tolerances)

    #multiplanes,pindex = get_multiplanes(solid_gu,solid.BoundBox) #pindex are all faces index used to produced multiplanes, do not count as standard planes
    multiplanes,pindex = get_multiplanes(solid_gu,None) #pindex are all faces index used to produced multiplanes, do not count as standard planes

    for mp in multiplanes:
        mp_region = Surfaces.add_multiPlane(mp)
        component_definition.append(mp_region)

    for iface, face in enumerate(solid_gu.Faces):

        if abs(face.Area) < Surfaces.tolerances.min_area:
            logger.warning(
                f"{str(face.Surface)} surface removed from cell definition. Face area < Min area ({face.Area} < {Surfaces.tolerances.min_area})"
            )
            continue
        if face.Area < 0:
            logger.warning("Negative surface Area")
        if face.Orientation not in ("Forward", "Reversed"):
            continue
        if flag_inv:
            orient = "Reversed" if face.Orientation == "Forward" else "Forward"
        else:
            orient = face.Orientation

        if isinstance(face.Surface, GU.PlaneGu):
            if iface in pindex : continue # the face in includes in multiplane
            plane = gen_plane(face,orient)
            plane_region = Surfaces.add_plane(plane,True)
            component_definition.append(plane_region) 
        
        elif isinstance(face.Surface, GU.CylinderGu):
            cylinder = gen_cylinder(face)
            cylinder_region = Surfaces.add_cylinder(cylinder,orient)  
            
            if orient == "Reversed":
                plane = gen_plane_cylinder(face, solid, Surfaces.tolerances)  # plane must be correctly oriented toward materials
                if plane is not None:           
                    cylinder_region = BoolRegion.mult(cylinder_region, auxillary_plane(plane,Surfaces), label=cylinder_region)
            component_definition.append(cylinder_region)
                 
        elif isinstance(face.Surface, GU.ConeGu):
            cone = gen_cone(face,orient)
            cone_region = Surfaces.add_cone(cone,orient)
            
            apex_plane = cone_apex_plane(face, orient)
            if apex_plane is not None:
                if orient == "Forward":
                    cone_region = BoolRegion.mult(cone_region, auxillary_plane(apex_plane,Surfaces), label=cone_region)
                else:    
                    cone_region = BoolRegion.add(cone_region, auxillary_plane(apex_plane,Surfaces), label=cone_region)                                

            if orient == "Reversed":
                plane = gen_plane_cone(face, solid, Surfaces.tolerances)  # plane must be correctly oriented toward materials
                if plane is not None:
                    cone_region = BoolRegion.mult(cone_region, auxillary_plane(plane,Surfaces), label=cone_region)
            component_definition.append(cone_region)         
        
        elif isinstance(face.Surface, GU.SphereGu):
            sphere = gen_sphere(face)
            sphere_region = Surfaces.add_sphere(sphere,orient)  
            
            if orient == "Reversed":
                plane_region = gen_plane_sphere(face, solid, Surfaces) 
                if plane_region is not None:
                    sphere_region = BoolRegion.mult(sphere_region, plane_region, label=sphere_region)
            component_definition.append(sphere_region)    

        elif isinstance(face.Surface, GU.TorusGu):
            torus = gen_torus(face,Surfaces.tolerances)
            if torus is not None:
                torus_region = Surfaces.add_torus(torus,orient)  
                if not u_closed:
                    U_plane_region = U_torus_planes(face, u_minMax, Surfaces)
                    torus_region = BoolRegion.mult(torus_region,U_plane_region,label=torus_region)
                
                if orient == "Reversed":
                    if not v_closed:
                        V_torus_surface_region = V_torus_surfaces(face, VminMax, Surfaces)
                        torus_region = BoolRegion.mult(torus_region,V_torus_surface_region,label=torus_region)
                component_definition.append(torus_region)                   
            else:
                logger.info("Only Torus with axis along X, Y, Z axis can be reproduced")
    return component_definition

