#
# Script to obtain the Qform of a Cylinder
#
#
#

from ...geometry_backend.geometry_backend_interface import GVector
from ...geometry_backend.vector_geometry import to_gvector


class _RotationMatrix:
    """Minimal 3x3 matrix, just enough for rotation_matrix()'s callers."""

    def __init__(self, a11, a12, a13, a21, a22, a23, a31, a32, a33):
        self.A11, self.A12, self.A13 = a11, a12, a13
        self.A21, self.A22, self.A23 = a21, a22, a23
        self.A31, self.A32, self.A33 = a31, a32, a33

    def transpose(self):
        self.A12, self.A21 = self.A21, self.A12
        self.A13, self.A31 = self.A31, self.A13
        self.A23, self.A32 = self.A32, self.A23

    def multiply(self, v):
        return GVector(
            self.A11 * v.x + self.A12 * v.y + self.A13 * v.z,
            self.A21 * v.x + self.A22 * v.y + self.A23 * v.z,
            self.A31 * v.x + self.A32 * v.y + self.A33 * v.z,
        )


def rotation_matrix(u, v):
    """Definition of the rotation matrix for two vectors"""

    u = to_gvector(u)
    v = to_gvector(v)

    # defintion of the axis of rotation
    Axis = u.cross(v).normalized()

    u = u.normalized()
    v = v.normalized()

    cose = u.dot(v)
    seno = u.cross(v).length

    onecos = 1.0 - cose

    return _RotationMatrix(
        # 1st row
        cose + Axis.x**2 * onecos,
        Axis.x * Axis.y * onecos - Axis.z * seno,
        Axis.x * Axis.z * onecos + Axis.y * seno,
        # 2nd row
        Axis.x * Axis.y * onecos + Axis.z * seno,
        cose + Axis.y**2 * onecos,
        Axis.y * Axis.z * onecos - Axis.x * seno,
        # 3rd row
        Axis.z * Axis.x * onecos - Axis.y * seno,
        Axis.z * Axis.y * onecos + Axis.x * seno,
        cose + Axis.z**2 * onecos,
    )


def q_form_cyl(Axis, Pos, rad):

    R = rotation_matrix(GVector(1, 0, 0), Axis)
    R.transpose()
    Pos2 = -R.multiply(to_gvector(Pos))

    A = R.A21**2 + R.A31**2
    B = R.A22**2 + R.A32**2
    C = R.A23**2 + R.A33**2

    D = 2.0 * (R.A21 * R.A22 + R.A31 * R.A32)
    E = 2.0 * (R.A22 * R.A23 + R.A32 * R.A33)
    F = 2.0 * (R.A23 * R.A21 + R.A33 * R.A31)

    G = 2.0 * (Pos2.y * R.A21 + Pos2.z * R.A31)
    H = 2.0 * (Pos2.y * R.A22 + Pos2.z * R.A32)
    J = 2.0 * (Pos2.y * R.A23 + Pos2.z * R.A33)

    K = Pos2.y**2 + Pos2.z**2 - rad**2

    return (A, B, C, D, E, F, G, H, J, K)


def q_form_cone(Axis, Pos, tan):

    R = rotation_matrix(GVector(1, 0, 0), Axis)
    R.transpose()
    Pos2 = -R.multiply(to_gvector(Pos))

    A = R.A21**2 + R.A31**2 - (tan * R.A11) ** 2
    B = R.A22**2 + R.A32**2 - (tan * R.A12) ** 2
    C = R.A23**2 + R.A33**2 - (tan * R.A13) ** 2

    D = 2.0 * (R.A21 * R.A22 + R.A31 * R.A32 - tan**2 * R.A11 * R.A12)
    E = 2.0 * (R.A22 * R.A23 + R.A32 * R.A33 - tan**2 * R.A12 * R.A13)
    F = 2.0 * (R.A23 * R.A21 + R.A33 * R.A31 - tan**2 * R.A13 * R.A11)

    G = 2.0 * (Pos2.y * R.A21 + Pos2.z * R.A31 - tan**2 * Pos2.x * R.A11)
    H = 2.0 * (Pos2.y * R.A22 + Pos2.z * R.A32 - tan**2 * Pos2.x * R.A12)
    J = 2.0 * (Pos2.y * R.A23 + Pos2.z * R.A33 - tan**2 * Pos2.x * R.A13)

    K = Pos2.y**2 + Pos2.z**2 - (tan * Pos2.x) ** 2

    return (A, B, C, D, E, F, G, H, J, K)
