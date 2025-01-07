from ..utils.geouned_classes import GeounedSurface
from ..utils.meta_surfaces import multiplane_loop, no_convex
from ..utils.functions import (
    build_multip_params,
    build_revcan_params,
    build_roundC_params,
    get_revcan_surfaces,
    get_roundcorner_surfaces,
)
from ..utils.geometry_gu import SolidGu, PlaneGu, CylinderGu
from .decom_utils_generator import cyl_bound_planes, torus_bound_planes, exclude_no_cutting_planes, order_plane_face


def get_surfaces(solid, tolerances, plane3pts=False):

    solid_GU = SolidGu(solid, tolerances=tolerances, plane3Pts=plane3pts)
    omitfaces = set()

    for rdc in next_roundCorner(solid_GU, omitfaces):
        yield rdc

    for can in next_reverseCan(solid_GU, omitfaces):
        yield can

    exclude_no_cutting_planes(solid_GU.Faces, omitfaces)

    for multiplane in next_multiplanes(solid_GU, omitfaces):
        yield multiplane

    for surface in plane_generator(solid_GU.Faces, omitfaces, tolerances, plane3pts):
        yield surface

    for surface in cylinder_generator(solid_GU.Faces, omitfaces):
        yield surface

    for surface in cone_generator(solid_GU.Faces):
        yield surface

    for surface in sphere_generator(solid_GU.Faces):
        yield surface

    for surface in torus_generator(solid_GU.Faces):
        yield surface


def plane_generator(GUFaces, omitfaces, tolerances, plane3Pts=False):
    cutting_plane_face = order_plane_face(GUFaces, omitfaces)
    for p in cutting_plane_face:
        normal = p.Surface.Axis
        pos = p.CenterOfMass
        dim1 = p.ParameterRange[1] - p.ParameterRange[0]
        dim2 = p.ParameterRange[3] - p.ParameterRange[2]
        plane = GeounedSurface(("Plane", (pos, normal, dim1, dim2)))
        yield plane

    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        surf = str(face.Surface)

        #        if surf == "<Plane object>" and not plane3Pts:
        #            normal = face.Surface.Axis
        #            pos = face.CenterOfMass
        #            dim1 = face.ParameterRange[1] - face.ParameterRange[0]
        #            dim2 = face.ParameterRange[3] - face.ParameterRange[2]
        #            plane = GeounedSurface(("Plane", (pos, normal, dim1, dim2)))
        #            yield plane

        #        elif surf == "<Cylinder object>":
        if surf == "<Cylinder object>":
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

            plane = GeounedSurface(("Plane3Pts", (pos, normal, dim1, dim2, points)))
            yield p


def cylinder_generator(GUFaces, omitfaces):
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
        cylinder = GeounedSurface(("Cylinder", (orig, dir, rad, dim_l)))
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
        cone = GeounedSurface(("Cone", (apex, dir, half_angle, dim_l, dimR)))
        yield cone


def sphere_generator(GUFaces):
    for face in GUFaces:
        surf = str(face.Surface)
        if surf[0:6] != "Sphere":
            continue

        rad = face.Surface.Radius
        pnt = face.Surface.Center
        sphere = GeounedSurface(("Sphere", (pnt, rad)))
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
        torus = GeounedSurface(("Torus", (center, dir, radMaj, radMin)))
        yield torus


def next_multiplanes(solid, plane_index_set):
    """identify and return all multiplanes in the solid."""
    planes = []
    for f in solid.Faces:
        if f.Index in plane_index_set:
            continue
        if isinstance(f.Surface, PlaneGu):
            planes.append(f)

    multiplane_list = []
    for p in planes:
        loop = False
        for mp in multiplane_list:
            if p in mp:
                loop = True
                break
        if loop:
            continue
        mplanes = [p]
        multiplane_loop([p], mplanes, planes)
        if len(mplanes) != 1:
            if no_convex(mplanes):
                mp_params = build_multip_params(mplanes)
                mp = GeounedSurface(("MultiPlane", mp_params))
                for pp in mplanes:
                    plane_index_set.add(pp.Index)
                multiplane_list.append(mplanes)
                yield mp


def next_reverseCan(solid, canface_index):
    """identify and return all can type in the solid."""

    for f in solid.Faces:
        if isinstance(f.Surface, CylinderGu):
            if f.Index in canface_index:
                continue
            if f.Orientation == "Reversed":
                cs, surfindex = get_revcan_surfaces(f, solid)
                if cs is not None:
                    gc = GeounedSurface(("ReverseCan", build_revcan_params(cs)))
                    canface_index.update(surfindex)
                    yield gc

    return None


def next_roundCorner(solid, cornerface_index):
    """identify and return all roundcorner type in the solid."""

    for f in solid.Faces:
        if isinstance(f.Surface, CylinderGu):
            if f.Index in cornerface_index:
                continue
            if f.Orientation == "Forward":
                rc, surfindex = get_roundcorner_surfaces(f, solid)
                if rc is not None:
                    gc = GeounedSurface(("RoundCorner", build_roundC_params(rc)))
                    cornerface_index.update(surfindex)
                    yield gc

    return None
