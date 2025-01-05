from ..utils.functions_generator import next_multiplanes, next_reverseCan
from ..utils.geouned_classes import GeounedSurface
from ..utils.geometry_gu import SolidGu
from .decom_utils_generator import cyl_bound_planes, torus_bound_planes, exclude_no_cutting_planes

def get_surfaces(solid, tolerances, plane3pts = False):
    
    solid_GU =   SolidGu(solid, tolerances=tolerances, plane3Pts=plane3pts)   
    omitfaces = exclude_no_cutting_planes(solid_GU.Faces)

    for can in next_reverseCan(solid_GU,omitfaces):
        yield can

    for multiplane in next_multiplanes(solid_GU,omitfaces): 
        yield multiplane

    for surface in  plane_generator(solid_GU.Faces,omitfaces,tolerances,plane3pts):
        yield surface

    for surface in  cylinder_generator(solid_GU.Faces,omitfaces):
        yield surface

    for surface in  cone_generator(solid_GU.Faces):
        yield surface

    for surface in  sphere_generator(solid_GU.Faces):
        yield surface

    for surface in  torus_generator(solid_GU.Faces):
        yield surface        


def plane_generator(GUFaces,omitfaces,tolerances, plane3Pts=False):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        surf = str(face.Surface)

        if surf == "<Plane object>" and not plane3Pts :
            normal = face.Surface.Axis
            pos = face.CenterOfMass
            dim1 = face.ParameterRange[1] - face.ParameterRange[0]
            dim2 = face.ParameterRange[3] - face.ParameterRange[2]
            plane =  GeounedSurface(("Plane", (pos, normal, dim1, dim2)))
            yield plane

        elif surf == "<Cylinder object>":
            for p in cyl_bound_planes(GUFaces, face):
                yield p

        elif surf == "<Cone object>":
            for p in cyl_bound_planes(GUFaces, face):
                yield p

        elif surf[0:6] == "Sphere":
            for p in cyl_bound_planes(GUFaces, face):
                yield plane

        elif surf == "<Toroid object>":
            for p in torus_bound_planes(GUFaces, face, tolerances):
                yield p

        elif surf == "<Plane object>" and plane3Pts:
            pos = face.CenterOfMass
            normal = face.Surface.Axis
            dim1 = face.ParameterRange[1] - face.ParameterRange[0]
            dim2 = face.ParameterRange[3] - face.ParameterRange[2]
            points = tuple(v.Point for v in face.Vertexes)

            plane =  GeounedSurface(("Plane3Pts", (pos, normal, dim1, dim2, points)))
            yield p

def cylinder_generator(GUFaces,omitfaces):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        surf = str(face.Surface)
        if surf != "<Cylinder object>":
            continue

        dir = face.Surface.Axis
        orig = face.Surface.Center
        rad = face.Surface.Radius
        dim_l = face.ParameterRange[3] - face.ParameterRange[2]
        cylinder =  GeounedSurface(("Cylinder", (orig, dir, rad, dim_l)))
        yield cylinder

def cone_generator(GUFaces):
    for face in GUFaces:
        surf = str(face.Surface)
        if surf != "<Cone object>":
            continue
        dir = face.Surface.Axis
        apex = face.Surface.Apex
        half_angle = face.Surface.SemiAngle
        dim_l = face.ParameterRange[3] - face.ParameterRange[2]
        dimR = face.Surface.Radius
        cone =  GeounedSurface(("Cone", (apex, dir, half_angle, dim_l, dimR)))
        yield cone

def sphere_generator(GUFaces):
    for face in GUFaces:
        surf = str(face.Surface)
        if surf[0:6] != "Sphere":
            continue

        rad = face.Surface.Radius
        pnt = face.Surface.Center
        sphere =  GeounedSurface(("Sphere", (pnt, rad)))
        yield sphere

def torus_generator(GUFaces):
    for face in GUFaces:
        surf = str(face.Surface)
        if surf != "<Toroid object>":
            continue

        radMaj = face.Surface.MajorRadius
        radMin = face.Surface.MinorRadius
        center = face.Surface.Center
        dir = face.Surface.Axis
        torus =  GeounedSurface(("Torus", (center, dir, radMaj, radMin)))
        yield torus

