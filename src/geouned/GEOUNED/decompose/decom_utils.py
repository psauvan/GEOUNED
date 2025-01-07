#   Conversion to MCNP v0.0
#   Only one solid and planar surfaces
#

import logging
import math
import numpy
from collections import OrderedDict

import FreeCAD
import Part

from ..utils.geouned_classes import GeounedSurface, MetaSurfacesDict
from ..utils.functions import get_multiplanes, get_reverseCan
from ..utils.geometry_gu import SolidGu, PlaneGu, TorusGu
from ..utils.basic_functions_part1 import (
    is_in_line,
    is_parallel,
    is_same_value,
)
from ..utils.basic_functions_part2 import is_duplicate_in_list
from ..utils.meta_surfaces import other_face_edge, region_sign

logger = logging.getLogger("general_logger")
twoPi = math.pi * 2


def get_surfaces(solid, options, tolerances, numeric_format):
    surfaceDict = extract_surfaces(solid, "All", options, tolerances, numeric_format)
    kind_order = ("RoundC", "MultiP", "RevCan", "Planes", "Cyl", "Cone", "Sph", "Tor")

    cut_surfaces = []
    for kind in kind_order:
        if kind == "Planes":
            PX = []
            PY = []
            PZ = []
            P = []
            for p in surfaceDict["Planes"]:
                if is_parallel(p.Surf.Axis, FreeCAD.Vector(1, 0, 0), tolerances.pln_angle):
                    PX.append(p)
                elif is_parallel(p.Surf.Axis, FreeCAD.Vector(0, 1, 0), tolerances.pln_angle):
                    PY.append(p)
                elif is_parallel(p.Surf.Axis, FreeCAD.Vector(0, 0, 1), tolerances.pln_angle):
                    PZ.append(p)
                else:
                    P.append(p)
            surfaces = PZ + PX + PY + P
            cut_surfaces.extend(surfaces)
        else:
            cut_surfaces.extend(surfaceDict[kind])
    return cut_surfaces


def gen_plane(pos, normal, diag):
    plane = Part.makePlane(diag, diag, pos, normal)
    vec_on_plane = plane.Vertexes[3].Point.sub(plane.Vertexes[0].Point)
    new_pos = plane.Vertexes[0].Point.sub(vec_on_plane)
    plane_center = Part.makePlane(2.0 * diag, 2.0 * diag, new_pos, normal)
    return plane_center


def cyl_bound_planes(solid, face):
    Edges = face.OuterWire.Edges
    planes = []
    for e in Edges:
        try:
            curve = str(e.Curve)
        except:
            curve = "none"

        adjacent_face = other_face_edge(e, face, solid.Faces)
        if adjacent_face is not None:
            if type(adjacent_face.Surface) is TorusGu:
                continue  # doesn't create plane if other face is a torus
            if type(adjacent_face.Surface) is PlaneGu:
                continue  # doesn't create plane if other face is a Plane
            if face.Surface.isSameSurface(adjacent_face.Surface):
                continue  # doesn't create plane if other face has same surface

        if curve[0:6] == "Circle":
            dir = e.Curve.Axis
            center = e.Curve.Center
            dim1 = e.Curve.Radius
            dim2 = e.Curve.Radius
            plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
            planes.append(plane)

        elif curve == "<Ellipse object>":
            dir = e.Curve.Axis
            center = e.Curve.Center
            dim1 = e.Curve.MinorRadius
            dim2 = e.Curve.MajorRadius
            plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
            planes.append(plane)

    return planes


def torus_bound_planes(solid, face, tolerances):
    params = face.ParameterRange
    planes = []
    if is_same_value(params[1] - params[0], twoPi, tolerances.value):
        return planes

    Edges = face.OuterWire.Edges

    for e in Edges:
        try:
            curve = str(e.Curve)
        except:
            curve = "none"

        adjacent_face = other_face_edge(e, face, solid.Faces)
        if adjacent_face is not None:
            if face.Surface.isSameSurface(adjacent_face.Surface):
                continue  # doesn't create plane if other face has same surface

        if curve[0:6] == "Circle":
            dir = e.Curve.Axis
            if not is_parallel(dir, face.Surface.Axis, tolerances.angle):
                center = e.Curve.Center
                dim1 = e.Curve.Radius
                dim2 = e.Curve.Radius
                plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
                planes.append(plane)

        elif curve == "<Ellipse object>":
            dir = e.Curve.Axis
            center = e.Curve.Center
            dim1 = e.Curve.MinorRadius
            dim2 = e.Curve.MajorRadius
            plane = GeounedSurface(("Plane", (center, dir, dim1, dim2)))
            planes.append(plane)

        elif curve == "<BSplineCurve object>":
            planeParams = plane_spline_curve(e, tolerances)
            if planeParams is not None:
                plane = GeounedSurface(("Plane", planeParams))
                planes.append(plane)

    return planes


def plane_spline_curve(edge, tolerances):

    normal = edge.derivative1At(0).cross(edge.derivative1At(0.5))
    normal.normalize()
    curve_2d = True
    for p in (0.25, 0.75, 1):
        # check if derivative orthogonal to curve normal vector
        if abs(normal.dot(edge.derivative1At(p))) > tolerances.value:
            curve_2d = False
            break

    r = edge.valueAt(0.25) - edge.valueAt(0.75)
    if curve_2d:
        return (edge.valueAt(0), normal, r.Length, r.Length)
    else:
        return None


def extract_surfaces(solid, kind, options, tolerances, numeric_format):
    fuzzy = False
    if kind == "All":
        solid_GU = SolidGu(solid, tolerances=tolerances)
    else:
        if kind == "Plane3Pts":
            P3P = True
        else:
            P3P = False
        solid_GU = SolidGu(solid, tolerances=tolerances, plane3Pts=P3P)

    Surfaces = MetaSurfacesDict(options=options, tolerances=tolerances, numeric_format=numeric_format)

    can_list, can_faces = get_reverseCan(solid_GU)
    for cs in can_list:
        Surfaces.add_reverseCan(cs)

    multiplanes_list, multiplane_faces = get_multiplanes(solid_GU)
    for mp in multiplanes_list:
        Surfaces.add_multiPlane(mp)

    # Get the parameter of all faces of the solid
    # Add auxillary planes for Cylinders and Cones
    omitfaces = can_faces.union(multiplane_faces)
    for iface, face in enumerate(solid.Faces):
        surf = str(face.Surface)
        if surf == "<Plane object>" and kind in ["Planes", "All"]:
            if iface in omitfaces:
                continue
            normal = face.Surface.Axis
            pos = face.CenterOfMass
            dim1 = face.ParameterRange[1] - face.ParameterRange[0]
            dim2 = face.ParameterRange[3] - face.ParameterRange[2]
            plane = GeounedSurface(("Plane", (pos, normal, dim1, dim2)))
            Surfaces.add_plane(plane, fuzzy)

        elif surf == "<Cylinder object>":
            if iface in omitfaces:
                continue
            dir = face.Surface.Axis
            orig = face.Surface.Center
            rad = face.Surface.Radius
            dim_l = face.ParameterRange[3] - face.ParameterRange[2]
            if kind in ["Cyl", "All"]:
                cylinder = GeounedSurface(("Cylinder", (orig, dir, rad, dim_l)))
                Surfaces.add_cylinder(cylinder, "Forward", fuzzy)

            if kind in ["Planes", "All"]:
                # add planes if cylinder axis is cut by a plane (plane quasi perpedicular to axis)
                for p in cyl_bound_planes(solid_GU, face):
                    Surfaces.add_plane(p, False)

        elif surf == "<Cone object>":
            dir = face.Surface.Axis
            apex = face.Surface.Apex
            half_angle = face.Surface.SemiAngle
            dim_l = face.ParameterRange[3] - face.ParameterRange[2]
            dimR = face.Surface.Radius
            if kind in ["Cone", "All"]:
                cone = GeounedSurface(("Cone", (apex, dir, half_angle, dim_l, dimR)))
                Surfaces.add_cone(cone, "Forward")

            if kind in ["Planes", "All"]:
                for p in cyl_bound_planes(solid_GU, face):
                    Surfaces.add_plane(p, False)

        elif surf[0:6] == "Sphere" and kind in ["Sph", "All"]:
            rad = face.Surface.Radius
            pnt = face.Surface.Center
            sphere = GeounedSurface(("Sphere", (pnt, rad)))
            Surfaces.add_sphere(sphere, "Forward")

            if kind in ["Planes", "All"]:
                for p in cyl_bound_planes(solid_GU, face):
                    Surfaces.add_plane(p, False)

        elif surf == "<Toroid object>":
            if kind in ["Tor", "All"]:
                radMaj = face.Surface.MajorRadius
                radMin = face.Surface.MinorRadius
                center = face.Surface.Center
                dir = face.Surface.Axis
                torus = GeounedSurface(("Torus", (center, dir, radMaj, radMin)))
                Surfaces.add_torus(torus, "Forward")

            if kind in ["Planes", "All"]:
                for p in torus_bound_planes(solid_GU, face, tolerances):
                    Surfaces.add_plane(p, False)

        elif surf == "<Plane object>" and kind == "Plane3Pts":
            pos = face.CenterOfMass
            normal = face.Surface.Axis
            dim1 = face.ParameterRange[1] - face.ParameterRange[0]
            dim2 = face.ParameterRange[3] - face.ParameterRange[2]
            points = tuple(v.Point for v in face.Vertexes)

            plane = GeounedSurface(("Plane3Pts", (pos, normal, dim1, dim2, points)))
            Surfaces.add_plane(plane, fuzzy)

    return Surfaces


#   Check if to faces are joint
def contiguous_face(face1, face2, tolerances):
    return face1.distToShape(face2)[0] < tolerances.distance


def same_faces(Faces, tolerances):
    Connection = OrderedDict()
    if len(Faces) == 1:
        return []

    for i, face1 in enumerate(Faces):
        Couples = []
        if not Faces[i + 1 :]:
            continue
        for j, face2 in enumerate(Faces[i + 1 :]):
            if contiguous_face(face1, face2, tolerances):

                Couples.append(i + 1 + j)

        Connection[i] = Couples

    lista = Connection[0]
    Connection.popitem(0)

    if len(Connection) == 0:  # solo los elementos de la lista de la superficie 0
        return lista

    if not lista:  # ninguna face está conecta conecta con la superficie 0
        return lista

    for elem in Connection:
        if elem in lista:  # que la key esta en lista implica que sus dependencias estan
            lista.extend(Connection[elem])
        else:
            for elem2 in Connection[elem]:
                if elem2 in lista:  # si una de sus dependencias esta en lista lo esta la clave
                    lista.append(elem)

    return list(set(lista))


# Tolerance in this function are not the general once
# function should be reviewed
def gen_plane_cylinder(face, solidFaces, tolerances):
    Surf = face.Surface
    rad = Surf.Radius
    if face.Area < 1e-2:
        return None

    UVNodes = []
    face_index_0 = [solidFaces.index(face)]

    try:
        face.tessellate(0.1)
        UVNodes.append(face.getUVNodes())
    except RuntimeError:
        PR = face.ParameterRange
        UVNode1 = (PR[0], PR[2])
        UVNode2 = (PR[1], PR[3])
        UVNodes.append([UVNode1, UVNode2])

    for i, face2 in enumerate(solidFaces):

        if str(face2.Surface) == "<Cylinder object>" and not (face2.isEqual(face)):
            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and face2.Surface.Radius == rad
                and is_in_line(face2.Surface.Center, face.Surface.Axis, face.Surface.Center)
            ):
                face_index_0.append(i)

    # prueba same_faces, parece ok
    Faces_p = []
    for ind in face_index_0:
        Faces_p.append(solidFaces[ind])

    face_index = [face_index_0[0]]  # la face de entrada

    for k in same_faces(Faces_p, tolerances):
        face_index.append(face_index_0[k])

    # commented to let plane cut rounded corner
    # if len(face_index_0)==len(face_index): #Esto evita un corte creo!!! no debería ser así en conversion
    #    return None

    for ind in reversed(face_index[1:]):
        if solidFaces[ind].Area <= 1e-3:
            face_index.remove(ind)

        else:
            face2 = solidFaces[ind]
            try:
                face2.tessellate(0.1)
                UVNodes.append(face2.getUVNodes())
            except RuntimeError:
                PR = face2.ParameterRange
                UVNode1 = (PR[0], PR[2])
                UVNode2 = (PR[1], PR[3])
                UVNodes.append([UVNode1, UVNode2])

    AngleRange = 0.0

    Uval = []
    for index in face_index:
        Range = solidFaces[index].ParameterRange
        AngleRange = AngleRange + abs(Range[1] - Range[0])
        # if not(Range[0] in Uval) and not(Range[1] in Uval):
        Uval.append(Range[0])
        Uval.append(Range[1])

    if twoPi - AngleRange < 1.0e-2 or AngleRange < 1.0e-2:
        return None

    Uval_str_cl = []

    for i, elem1 in enumerate(Uval):
        num_str1 = f"{elem1:11.4E}"
        if abs(elem1) < 1.0e-5:
            num_str1 = "%11.4E" % 0.0

        if not (is_duplicate_in_list(num_str1, i, Uval)):
            Uval_str_cl.append(num_str1)

    if len(Uval_str_cl) < 2:
        logger.info("gen_plane_cylinder : Uval_str_cl should no be void")
        return None

    face_index_2 = [face_index[0], face_index[0]]

    Node_min = UVNodes[0][0]
    Node_max = UVNodes[0][1]

    dif1_0 = abs(float(Uval_str_cl[0]) - Node_min[0])
    dif2_0 = abs(float(Uval_str_cl[1]) - Node_max[0])

    # searching for minimum and maximum angle points

    for j, Nodes in enumerate(UVNodes):
        for elem in Nodes:
            dif1 = abs(float(Uval_str_cl[0]) - elem[0])
            dif2 = abs(float(Uval_str_cl[1]) - elem[0])

            if dif1 < dif1_0:
                Node_min = elem
                face_index_2[0] = face_index[j]
                dif1_0 = dif1
            if dif2 < dif2_0:
                Node_max = elem
                face_index_2[1] = face_index[j]
                dif2_0 = dif2

    V1 = solidFaces[face_index_2[0]].valueAt(Node_min[0], Node_min[1])
    V2 = solidFaces[face_index_2[1]].valueAt(Node_max[0], Node_max[1])

    if V1.isEqual(V2, 1e-5):
        logger.error("in the additional plane definition")
        return None

    normal = V2.sub(V1).cross(face.Surface.Axis)

    plane = Part.Plane(V1, normal).toShape()

    return plane


# Tolerance in this function are not the general once
# function should be reviewed
def gen_plane_cone(face, solid, tolerances):

    if face.Area < 1e-2:
        return None

    UVNodes = []
    face_index_0 = [solid.Faces.index(face)]

    try:
        face.tessellate(0.1)
        UVNodes.append(face.getUVNodes())
    except RuntimeError:
        PR = face.ParameterRange
        UVNode1 = (PR[0], PR[2])
        UVNode2 = (PR[1], PR[3])
        UVNodes.append([UVNode1, UVNode2])

    for i, face2 in enumerate(solid.Faces):

        if str(face2.Surface) == "<Cone object>" and not (face2.isEqual(face)):

            if (
                face2.Surface.Axis.isEqual(face.Surface.Axis, 1e-5)
                and face2.Surface.Apex.isEqual(face.Surface.Apex, 1e-5)
                and (face2.Surface.SemiAngle - face.Surface.SemiAngle) < 1e-6
            ):

                face_index_0.append(i)

    Faces_p = []
    for ind in face_index_0:
        Faces_p.append(solid.Faces[ind])

    face_index = [face_index_0[0]]  # la face de entrada

    for k in same_faces(Faces_p, tolerances):
        face_index.append(face_index_0[k])

    # same as cylinder commennt
    # if len(face_index_0)==len(face_index):
    #    return None

    for ind in reversed(face_index[1:]):
        if solid.Faces[ind].Area <= 1e-3:
            face_index.remove(ind)
        else:
            face2 = solid.Faces[ind]
            try:
                face2.tessellate(0.1)
                UVNodes.append(face2.getUVNodes())
            except RuntimeError:
                PR = face2.ParameterRange
                UVNode1 = (PR[0], PR[2])
                UVNode2 = (PR[1], PR[3])
                UVNodes.append([UVNode1, UVNode2])

    AngleRange = 0.0

    Uval = []

    for index in face_index:
        Range = solid.Faces[index].ParameterRange
        AngleRange = AngleRange + abs(Range[1] - Range[0])
        Uval.append(Range[0])
        Uval.append(Range[1])
    if twoPi - AngleRange < 1.0e-2 or AngleRange < 1e-2:
        return None

    Uval_str_cl = []

    for i, elem1 in enumerate(Uval):
        num_str1 = f"{elem1:11.4E}"
        if abs(elem1) < 1.0e-5:
            num_str1 = "%11.4E" % 0.0
        if not (is_duplicate_in_list(num_str1, i, Uval)):
            Uval_str_cl.append(num_str1)

    if len(Uval_str_cl) < 2:
        logger.info("gen_plane_cone : Uval_str_cl should no be void")
        return None

    face_index_2 = [face_index[0], face_index[0]]

    Node_min = UVNodes[0][0]
    Node_max = UVNodes[0][1]
    dif1_0 = abs(float(Uval_str_cl[0]) - Node_min[0])
    dif2_0 = abs(float(Uval_str_cl[1]) - Node_max[0])

    # searching for minimum and maximum angle points
    for j, Nodes in enumerate(UVNodes):
        for elem in Nodes:
            dif1 = abs(float(Uval_str_cl[0]) - elem[0])
            dif2 = abs(float(Uval_str_cl[1]) - elem[0])

            if dif1 < dif1_0:
                Node_min = elem
                face_index_2[0] = face_index[j]
                dif1_0 = dif1
            if dif2 < dif2_0:
                Node_max = elem
                face_index_2[1] = face_index[j]
                dif2_0 = dif2

    V1 = solid.Faces[face_index_2[0]].valueAt(Node_min[0], Node_min[1])
    V2 = solid.Faces[face_index_2[1]].valueAt(Node_max[0], Node_max[1])

    if V1.isEqual(V2, 1e-5):
        logger.error("in the additional plane definition")
        return None

    # normal=V2.sub(V1).cross(face.Surface.Axis)

    # plane=Part.Plane(V1,normal).toShape()
    plane = Part.Plane(V1, V2, face.Surface.Apex).toShape()

    return plane


def get_solid_dimensions(solid):
    # not used but may be necessary in the future to check solid validity
    """evaluate characteristic length in the 3 main axis directions of the solid"""
    mat = solid.MatrixOfInertia
    inertialMat = numpy.array(((mat.A11, mat.A12, mat.A13), (mat.A21, mat.A22, mat.A23), (mat.A31, mat.A32, mat.A33)))
    eigval = numpy.linalg.eigvals(inertialMat)

    L0 = math.sqrt(abs(eigval[2] + eigval[1] - eigval[0]) / 2)
    L1 = math.sqrt(abs(eigval[0] + eigval[2] - eigval[1]) / 2)
    L2 = math.sqrt(abs(eigval[1] + eigval[0] - eigval[2]) / 2)
    dim = [L0, L1, L2]
    dim.sort()
    return (dim, solid.Volume, solid.Area)


def valid_solid(solid):
    Vol_tol = 1e-2
    Vol_area_ratio = 1e-3
    if abs(solid.Volume / solid.Area) < Vol_area_ratio:
        return False
    if abs(solid.Volume) < Vol_tol:
        return False
    return True


def remove_solids(original_solid, Solids):

    if len(Solids) == 1:
        return Solids

    Solids_Clean = []
    for solid in Solids:
        if not valid_solid(solid):
            logger.warning(f"remove_solids degenerated solids are produced bad dimensions")
            continue
        Solids_Clean.append(solid)

    return Solids_Clean


def external_plane(plane, Faces):
    Edges = plane.OuterWire.Edges
    for e in Edges:
        adjacent_face = other_face_edge(e, plane, Faces)
        if region_sign(plane, adjacent_face) == "OR":
            return False
    return True


def exclude_no_cutting_planes(Faces):
    omit = set()
    for f in Faces:
        if isinstance(f.Surface, PlaneGu):
            if external_plane(f, Faces):
                omit.add(f.Index)
    return omit
