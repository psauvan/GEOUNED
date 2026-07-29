from ..utils.geouned_classes import GeounedSurface
from ..utils.meta_surfaces import multiplane, get_can_surfaces, get_tcone_surfaces, get_roundcorner_surfaces
from ..utils.meta_surfaces_utils import no_convex, remove_twice_parallel, eligible_plane

from ..utils.functions import (
    build_multip_params,
    build_can_params,
    build_tcone_params,
    build_roundC_params,
)
from ..utils.geometry_gu import SolidGu
from ...geo import GPlane, GCylinder, GCone, GSphere, GTorus
from .decom_utils_generator import (
    cks_bound_planes,
    torus_bound_planes,
    exclude_no_cutting_planes,
    order_plane_face,
    omit_isolated_planes,
)


def get_surfaces(solid, omitfaces, tolerances, meta_surface=True):

    solid_GU = SolidGu(solid, tolerances=tolerances)

    if meta_surface:

        for can in next_Can(solid_GU, omitfaces):
            yield can

        for tcone in next_truncCone(solid_GU, omitfaces):
            yield tcone

        for rdc in next_roundCorner(solid_GU, omitfaces):
            yield rdc

        extPlanes = exclude_no_cutting_planes(solid_GU.Faces)
        omitfaces.update(extPlanes)

        for multiplane in next_multiplanes(solid_GU.Faces, omitfaces):
            yield multiplane
    else:
        extPlanes = exclude_no_cutting_planes(solid_GU.Faces)
        omitfaces.update(extPlanes)

    for surface in plane_generator(solid_GU.Faces, omitfaces, tolerances):
        yield surface

    for surface in cylinder_generator(solid_GU.Faces, omitfaces):
        yield surface

    for surface in cone_generator(solid_GU.Faces, omitfaces):
        yield surface

    for surface in sphere_generator(solid_GU.Faces, omitfaces):
        yield surface

    for surface in torus_generator(solid_GU.Faces):
        yield surface

    omitfaces = omitfaces - extPlanes
    for surface in plane_generator(solid_GU.Faces, omitfaces, tolerances, True):
        yield surface


def plane_generator(GUFaces, omitfaces, tolerances, externalPlanes=False):
    omit_isolated_planes(GUFaces, omitfaces)
    cutting_plane_face = order_plane_face(GUFaces, omitfaces)
    for p in cutting_plane_face:
        omitfaces.add(p.Index)
        normal = p.Surface.Axis
        pos = p.CenterOfMass
        dim1 = p.ParameterRange[1] - p.ParameterRange[0]
        dim2 = p.ParameterRange[3] - p.ParameterRange[2]
        plane = GeounedSurface(("Plane", (pos, normal, dim1, dim2)))
        yield plane

    if externalPlanes:
        return
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        surf_type = type(face.Surface)

        if surf_type is GCylinder:
            for p in cks_bound_planes(GUFaces, face, omitfaces):
                yield p

        elif surf_type is GCone:
            for p in cks_bound_planes(GUFaces, face, omitfaces):
                yield p

        elif surf_type is GSphere:
            for p in cks_bound_planes(GUFaces, face, omitfaces):
                yield p

        elif surf_type is GTorus:
            for p in torus_bound_planes(GUFaces, face, tolerances):
                yield p


def cylinder_generator(GUFaces, omitfaces):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        if type(face.Surface) is not GCylinder:
            continue

        dir = face.Surface.Axis
        orig = face.Surface.Center
        rad = face.Surface.Radius
        dim_l = face.ParameterRange[3] - face.ParameterRange[2]
        cylinder = GeounedSurface(("CylinderOnly", (orig, dir, rad, dim_l)))
        yield cylinder


def cone_generator(GUFaces, omitfaces):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        if type(face.Surface) is not GCone:
            continue
        dir = face.Surface.Axis
        apex = face.Surface.Apex
        half_angle = face.Surface.SemiAngle
        dim_l = face.ParameterRange[3] - face.ParameterRange[2]
        dimR = face.Surface.Radius
        cone = GeounedSurface(("ConeOnly", (apex, dir, half_angle, dim_l, dimR)))
        yield cone


def sphere_generator(GUFaces, omitfaces):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        if type(face.Surface) is not GSphere:
            continue

        rad = face.Surface.Radius
        pnt = face.Surface.Center
        sphere = GeounedSurface(("SphereOnly", (pnt, rad)))
        yield sphere


def torus_generator(GUFaces):
    for face in GUFaces:
        if type(face.Surface) is not GTorus:
            continue

        radMaj = face.Surface.MajorRadius
        radMin = face.Surface.MinorRadius
        center = face.Surface.Center
        dir = face.Surface.Axis
        torus = GeounedSurface(("TorusOnly", (center, dir, radMaj, radMin)))
        yield torus


def next_multiplanes(solidFaces, plane_index_set):
    """identify and return all multiplanes in the solid."""
    planes = []
    for f in solidFaces:
        if f.Index in plane_index_set:
            continue
        if isinstance(f.Surface, GPlane):
            planes.append(f)

    used_plane = set()
    for p in planes:
        if p.Index in used_plane:
            continue
        if not eligible_plane(p):
            continue
        mp_plane_index = set()
        mplanes = multiplane(p, planes, mp_plane_index)
        used_plane.update(mp_plane_index)
        if len(mplanes) != 1:
            if no_convex(mplanes):
                remove_twice_parallel(mplanes)
                mp_params = build_multip_params(mplanes)
                mp = GeounedSurface(("MultiPlane", mp_params))
                if mp.Surf.PlaneNumber < 2:
                    continue
                for pp in mplanes:
                    plane_index_set.add(pp.Index)
                yield mp


def next_Can(solid, canface_index):
    """identify and return all can type in the solid."""

    solidFaces = solid.Faces

    for f in solidFaces:
        if isinstance(f.Surface, GCylinder):
            if f.Index in canface_index:
                continue

            cs, surfindex = get_can_surfaces(f, solidFaces)
            if cs is not None:
                gc = GeounedSurface(("Can", build_can_params(cs)))
                canface_index.update(surfindex)
                yield gc

    return None


def next_truncCone(solid, tconeface_index):
    """identify and return all truncated cone type in the solid."""

    solidFaces = solid.Faces

    for f in solidFaces:
        if isinstance(f.Surface, GCone):
            if f.Index in tconeface_index:
                continue

            cs, surfindex = get_tcone_surfaces(f, solidFaces)
            if cs is not None:
                gc = GeounedSurface(("TCone", build_tcone_params(cs)))
                tconeface_index.update(surfindex)
                yield gc

    return None


def next_roundCorner(solid, cornerface_index):
    """identify and return all roundcorner type in the solid."""
    solidFaces = solid.Faces
    for f in solidFaces:
        if isinstance(f.Surface, GCylinder):
            if f.Index in cornerface_index:
                continue
            rc, surfindex = get_roundcorner_surfaces(f, solidFaces, {f.Index})
            if rc is not None:
                cornerface_index.update(surfindex)
                rc_list, plane_list, multi_round, orientation = build_roundC_params(rc)
                if not multi_round:
                    for gc in rc_list:
                        yield gc
                else:
                    gc = GeounedSurface(("MultiRoundCorner", (rc_list, plane_list, orientation)))
                    yield gc

    return None
