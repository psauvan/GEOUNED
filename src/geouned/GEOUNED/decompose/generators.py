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
from ...geo import GPlane, GCylinder, GCone, GSphere, GTorus, GVector
from .decom_utils_generator import (
    cks_bound_planes,
    torus_bound_planes,
    exclude_no_cutting_planes,
    order_plane_face,
    omit_isolated_planes,
    tangent_cut_plane,
    torus_tangent_cut_surface,
)


def get_surfaces(solid, omitfaces, tolerances, options, meta_surface=True):

    solid_GU = SolidGu(solid, tolerances=tolerances)

    if len(solid_GU.Faces) > options.cut_large_cell:
        yield large_cell_plane_split(solid_GU)

    if meta_surface:

        for can in next_Can(solid_GU, omitfaces, tolerances=tolerances):
            yield can

        for tcone in next_truncCone(solid_GU, omitfaces, tolerances=tolerances):
            yield tcone

        for rdc in next_roundCorner(solid_GU, omitfaces, tolerances=tolerances):
            yield rdc

        extPlanes = exclude_no_cutting_planes(solid_GU.Faces, tolerances=tolerances)
        omitfaces.update(extPlanes)

        for multiplane in next_multiplanes(solid_GU.Faces, omitfaces, tolerances):
            yield multiplane
    else:
        extPlanes = exclude_no_cutting_planes(solid_GU.Faces, tolerances=tolerances)
        omitfaces.update(extPlanes)

    # Cylinders and cones are tried before planes: on the test_models corpus this
    # gives fewer irreducible solids (346 -> 325 over 176 original solids, 4
    # solids improve, none gets worse) with identical volume conservation.
    for surface in implicit_bound_planes(solid_GU.Faces, omitfaces, tolerances):
        yield surface

    for surface in cylinder_generator(solid_GU.Faces, omitfaces, tolerances):
        yield surface

    for surface in cone_generator(solid_GU.Faces, omitfaces, tolerances):
        yield surface

    for surface in plane_generator(solid_GU.Faces, omitfaces, tolerances):
        yield surface

    for surface in sphere_generator(solid_GU.Faces, omitfaces, tolerances):
        yield surface

    for surface in torus_generator(solid_GU.Faces, tolerances):
        yield surface

    omitfaces = omitfaces - extPlanes
    for surface in plane_generator(solid_GU.Faces, omitfaces, tolerances):
        yield surface


def plane_generator(GUFaces, omitfaces, tolerances):
    omit_isolated_planes(GUFaces, omitfaces, tolerances=tolerances)
    cutting_plane_face = order_plane_face(GUFaces, omitfaces, tolerances.min_area, tolerances.min_face_width, tolerances=tolerances)
    for p in cutting_plane_face:
        omitfaces.add(p.Index)
        normal = p.Surface.Axis
        pos = p.CenterOfMass
        dim1 = p.ParameterRange[1] - p.ParameterRange[0]
        dim2 = p.ParameterRange[3] - p.ParameterRange[2]
        plane = GeounedSurface(("Plane", (pos, normal, dim1, dim2)))
        yield plane


def implicit_bound_planes(GUFaces, omitfaces, tolerances):
    # A cutting plane that isn't a real plane FACE of the solid at all --
    # derived from where a curved face (cylinder/cone/sphere: cks_bound_planes;
    # torus: torus_bound_planes) ends, the same way plane_generator's own real
    # cutting planes are derived from an actual Plane face. Split out of
    # plane_generator, 2026-10-28, so it can be tried in its own position in
    # get_surfaces's own candidate order independently of the real-plane-face
    # search.
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        surf_type = type(face.Surface)

        if surf_type is GCylinder:
            for p in cks_bound_planes(GUFaces, face, omitfaces, tolerances=tolerances):
                yield p

        elif surf_type is GCone:
            for p in cks_bound_planes(GUFaces, face, omitfaces, tolerances=tolerances):
                yield p

        elif surf_type is GSphere:
            for p in cks_bound_planes(GUFaces, face, omitfaces, tolerances=tolerances):
                yield p

        elif surf_type is GTorus:
            for p in torus_bound_planes(GUFaces, face, tolerances=tolerances):
                yield p


def cylinder_generator(GUFaces, omitfaces, tolerances=None):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        if type(face.Surface) is not GCylinder:
            continue
        if tolerances is not None and getattr(face, "CharacteristicWidth", float("inf")) < tolerances.min_face_width:
            # a genuine sliver isn't limited to plane faces -- a thin
            # residual Cylinder patch is exactly the same class of
            # boolean-cut artifact (see Tolerances.min_face_width's own
            # docstring); same check as plane_generator/order_plane_face.
            continue

        if tolerances is not None:
            tangent_plane = tangent_cut_plane(face, GUFaces, tolerances=tolerances)
            if tangent_plane is not None:
                yield tangent_plane
                continue

        dir = face.Surface.Axis
        orig = face.Surface.Center
        rad = face.Surface.Radius
        dim_l = face.ParameterRange[3] - face.ParameterRange[2]
        cylinder = GeounedSurface(("CylinderOnly", (orig, dir, rad, dim_l)))
        yield cylinder


def cone_generator(GUFaces, omitfaces, tolerances=None):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        if type(face.Surface) is not GCone:
            continue
        if tolerances is not None and getattr(face, "CharacteristicWidth", float("inf")) < tolerances.min_face_width:
            continue

        if tolerances is not None:
            tangent_plane = tangent_cut_plane(face, GUFaces, tolerances=tolerances)
            if tangent_plane is not None:
                yield tangent_plane
                continue

        dir = face.Surface.Axis
        apex = face.Surface.Apex
        half_angle = face.Surface.SemiAngle
        dim_l = face.ParameterRange[3] - face.ParameterRange[2]
        dimR = face.Surface.Radius
        cone = GeounedSurface(("ConeOnly", (apex, dir, half_angle, dim_l, dimR)))
        yield cone


def sphere_generator(GUFaces, omitfaces, tolerances=None):
    for face in GUFaces:
        if face.Index in omitfaces:
            continue
        if type(face.Surface) is not GSphere:
            continue
        if tolerances is not None and getattr(face, "CharacteristicWidth", float("inf")) < tolerances.min_face_width:
            continue

        if tolerances is not None:
            tangent_plane = tangent_cut_plane(face, GUFaces, tolerances=tolerances)
            if tangent_plane is not None:
                yield tangent_plane
                continue

        rad = face.Surface.Radius
        pnt = face.Surface.Center
        sphere = GeounedSurface(("SphereOnly", (pnt, rad)))
        yield sphere


def torus_generator(GUFaces, tolerances=None):
    for face in GUFaces:
        if type(face.Surface) is not GTorus:
            continue
        if tolerances is not None and getattr(face, "CharacteristicWidth", float("inf")) < tolerances.min_face_width:
            continue

        if tolerances is not None:
            tangent_surface = torus_tangent_cut_surface(face, GUFaces, tolerances=tolerances)
            if tangent_surface is not None:
                yield tangent_surface
                continue

        radMaj = face.Surface.MajorRadius
        radMin = face.Surface.MinorRadius
        center = face.Surface.Center
        dir = face.Surface.Axis
        torus = GeounedSurface(("TorusOnly", (center, dir, radMaj, radMin, face.Surface.a_sign)))
        yield torus


def next_multiplanes(solidFaces, plane_index_set, tolerances=None):
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
        if not eligible_plane(p, tolerances):
            continue
        mp_plane_index = set()
        mplanes = multiplane(p, planes, mp_plane_index, tolerances)
        used_plane.update(mp_plane_index)
        if len(mplanes) != 1:
            if no_convex(mplanes, tolerances=tolerances):
                remove_twice_parallel(mplanes, tolerances=tolerances)
                mp_params = build_multip_params(mplanes, tolerances=tolerances)
                mp = GeounedSurface(("MultiPlane", mp_params))
                if mp.Surf.PlaneNumber < 2:
                    continue
                for pp in mplanes:
                    plane_index_set.add(pp.Index)
                yield mp


def next_Can(solid, canface_index, *, tolerances):
    """identify and return all can type in the solid."""

    solidFaces = solid.Faces

    for f in solidFaces:
        if isinstance(f.Surface, GCylinder):
            if f.Index in canface_index:
                continue

            cs, surfindex = get_can_surfaces(f, solidFaces, tolerances=tolerances)
            if cs is not None:
                params = build_can_params(cs, tolerances=tolerances)
                if params is not None:
                    # Full params (not a params[0:3] slice -- a pre-existing
                    # truncation that used to only drop cyl.Orientation,
                    # itself still read explicitly below, but would also
                    # silently drop CanParams.Edge1/.Edge2 (params[4:6]),
                    # added for the "sewing composite surface" branch's own
                    # construction -- see CanParams/build_can_params's own
                    # docstrings) is passed through, matching next_truncCone's
                    # own identical, unsliced build_tcone_params(cs) call.
                    gc = GeounedSurface(("Can", params, params[3]))
                    canface_index.update(surfindex)
                    yield gc

    return None


def next_truncCone(solid, tconeface_index, *, tolerances):
    """identify and return all truncated cone type in the solid."""

    solidFaces = solid.Faces

    for f in solidFaces:
        if isinstance(f.Surface, GCone):
            if f.Index in tconeface_index:
                continue

            cs, surfindex = get_tcone_surfaces(f, solidFaces, tolerances=tolerances)
            if cs is not None:
                gc = GeounedSurface(("TCone", build_tcone_params(cs)))
                tconeface_index.update(surfindex)
                yield gc

    return None


def next_roundCorner(solid, cornerface_index, *, tolerances):
    """identify and return all roundcorner type in the solid."""
    solidFaces = solid.Faces
    for f in solidFaces:
        if isinstance(f.Surface, GCylinder):
            if f.Index in cornerface_index:
                continue
            rc, surfindex = get_roundcorner_surfaces(f, solidFaces, {f.Index}, solid=solid, tolerances=tolerances)
            if rc is not None:
                cornerface_index.update(surfindex)
                rc_list, plane_list, multi_round, orientation, closed_set = build_roundC_params(rc, tolerances=tolerances)
                if not multi_round:
                    # `rc` (the real cylinder faces, one tuple per corner)
                    # and `rc_list` (the already-built RoundCorner
                    # GeounedSurface candidates) are in the exact same
                    # order, one entry each per corner -- zip them to
                    # check each corner's own REAL cylinder face for a
                    # near-tangent neighbor before yielding it as the
                    # cylinder itself. A RoundCorner's own cylinder is
                    # already explicitly bounded by its own 2 corner
                    # planes (p1/p2, part of what made it a RoundCorner
                    # in the first place) -- a near-tangent neighbor
                    # found here is some OTHER, different face (e.g. a
                    # tangent torus, see Mixed/base_all8_to_cylinder.
                    # stp), never p1/p2 themselves.
                    for (cyl, p1, p2, gpa, corner_points, config_orientation), gc in zip(rc, rc_list):
                        tangent_plane = tangent_cut_plane(cyl, solidFaces, tolerances=tolerances, roundCorner=True)
                        yield tangent_plane if tangent_plane is not None else gc
                else:
                    # Same check, but across every real cylinder face
                    # feeding this one MultiRoundCorner group -- if ANY of
                    # them has a near-tangent neighbor, that candidate's
                    # own cutting plane is returned instead of ever
                    # building the MultiRoundCorner at all (the first one
                    # found, not every one -- a single real cut is enough
                    # to let generic_split make progress on this fragment).
                    tangent_plane = None
                    for cyl, p1, p2, gpa, corner_points, config_orientation in rc:
                        tangent_plane = tangent_cut_plane(cyl, solidFaces, tolerances=tolerances, roundCorner=True)
                        if tangent_plane is not None:
                            break
                    if tangent_plane is not None:
                        yield tangent_plane
                    else:
                        gc = GeounedSurface(("MultiRoundCorner", (rc_list, plane_list, orientation, closed_set)))
                        yield gc
    return None


_XYZ = (GVector(1, 0, 0), GVector(0, 1, 0), GVector(0, 0, 1))


def large_cell_plane_split(solid):
    bbox = solid.BoundBox
    lengths = [(bbox.XLength, 0), (bbox.YLength, 1), (bbox.ZLength, 2)]
    lengths.sort()

    axis = _XYZ[lengths[-1][1]]
    dot_prod = []
    for i, iaxis in enumerate(solid.InertiaAxes):
        dot_prod.append((abs(axis.dot(iaxis)), i))
    dot_prod.sort()

    normal = solid.InertiaAxes[dot_prod[-1][1]]
    position = solid.CenterOfMass
    return GeounedSurface(("Plane", (position, normal, 1.0, 1.0)))
